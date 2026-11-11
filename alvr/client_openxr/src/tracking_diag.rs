// TRACKING DIAGNOSTICS (test builds, always on, log only): what the runtime tells this OpenXR app
// about its tracking reference, appended to screenvr_tracking_diag.log in the app's external files
// dir (the headset's logcat buffer holds only about a minute). Purpose: find a trustworthy signal
// for "the tracking space was re-established / is stale / the floor changed".
//
// Passive: no session, pass or provider is started; nothing here changes what the app does.
// - Render thread: every polled OpenXR event is logged (poll_event below replaces the openxr
//   crate's poll, which silently drops event types it does not know), plus a few cheap stores
//   into shared state. Lines are formatted there only on change and handed to a channel.
// - Sampler thread (TrackingDiag): its own VIEW/LOCAL/STAGE/LOCAL_FLOOR spaces, located at the
//   runtime's current time ~50 times a second while the session runs (2 Hz otherwise); logs
//   changes, jumps and a 10 s heartbeat; polls XR_PICO_virtual_boundary every 3 s when enabled.
// - Writer thread: appends each line with one write (survives a kill), rotates at MAX_BYTES to
//   <file>.1, and polls a few Android system properties every 5 s.
// Line format: `<local wall clock> mono=<s> boot=<s> <text>` (mono stops in suspend, boot does not).

use alvr_common::{
    Pose,
    glam::{EulerRot, Quat},
    info, warn,
};
use openxr::{self as xr, sys};
use std::{
    ffi::{CString, c_void},
    fs::{self, File, OpenOptions},
    io::Write,
    mem::{self, MaybeUninit},
    ptr,
    sync::{
        Arc, Mutex, OnceLock,
        atomic::{AtomicBool, AtomicI64, Ordering},
        mpsc,
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

const DIR: &str = "/sdcard/Android/data/alvr.client.dev/files";
const FILE_NAME: &str = "screenvr_tracking_diag.log";
const MAX_BYTES: u64 = 4 * 1024 * 1024;
const PROP_INTERVAL: Duration = Duration::from_secs(5);
const PROPS: [&str; 4] = [
    "sys.pxr.vrboundary.status",
    "sys.guardian.service.status",
    "sys.pvr.floorheightinvst",
    "persist.pxr.marker_reloc",
];

const SAMPLE_INTERVAL_RUNNING: Duration = Duration::from_millis(20);
const SAMPLE_INTERVAL_IDLE: Duration = Duration::from_millis(500);
const HEARTBEAT_INTERVAL: Duration = Duration::from_secs(10);
const BOUNDS_INTERVAL: Duration = Duration::from_secs(5);
const BOUNDARY_INTERVAL: Duration = Duration::from_secs(3);
// World-locked relations (STAGE/LOCAL_FLOOR in LOCAL, anchor) logged when they change this much
const CHANGE_METERS: f32 = 0.002;
const CHANGE_DEGREES: f32 = 0.1;
// Head pose jump: deviation from the constant-velocity prediction between consecutive samples
const JUMP_METERS: f32 = 0.05;
const JUMP_DEGREES: f32 = 3.0;
const JUMP_MAX_GAP_NANOS: i64 = 100_000_000;
const JUMP_LOG_MIN_INTERVAL: Duration = Duration::from_millis(500);
// "Origin reset" look of the first head pose after a start: x = z = 0, yaw 0
const ORIGIN_RESET_METERS: f32 = 0.001;
const ORIGIN_RESET_DEGREES: f32 = 0.1;

pub const VIRTUAL_BOUNDARY_EXTENSION_NAME: &str = "XR_PICO_virtual_boundary";
// PICO OpenXR SDK 3.0 openxr.h:715-716 (XrStructureType), 8693-8703 (enums), 8711-8744 (structs),
// 8756-8760 (function pointer types)
const TYPE_VIRTUAL_BOUNDARY_INFO_PICO: i32 = 1010001001;
const TYPE_VIRTUAL_BOUNDARY_STATUS_PICO: i32 = 1010001002;
const BOUNDARY_EDGE_OUTER_CONTOUR: i32 = 0;
const BOUNDARY_EDGE_INSCRIBED_RECTANGLE: i32 = 1;
const BOUNDARY_GEOMETRY_MAX_POINTS: u32 = 4096;

// Set by events (session state, reference space change, presence) and stream start/stop: the
// sampler logs a full snapshot (reference spaces, bounds, all relations) and waits for the first
// head pose again
static RESNAPSHOT: AtomicBool = AtomicBool::new(true);
static SENDER: OnceLock<mpsc::Sender<String>> = OnceLock::new();

// ---------------------------------------------------------------------------------------------
// Line output

#[cfg(target_os = "android")]
fn clock_seconds(clock: libc::clockid_t) -> f64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    if unsafe { libc::clock_gettime(clock, &raw mut ts) } != 0 {
        return 0.0;
    }
    ts.tv_sec as f64 + ts.tv_nsec as f64 * 1e-9
}

#[cfg(target_os = "android")]
fn clock_ns(clock: libc::clockid_t) -> i64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    if unsafe { libc::clock_gettime(clock, &raw mut ts) } != 0 {
        return 0;
    }
    ts.tv_sec as i64 * 1_000_000_000 + ts.tv_nsec as i64
}

/// CLOCK_BOOTTIME (counts while the device is suspended), nanoseconds
#[cfg(target_os = "android")]
pub fn boot_ns() -> i64 {
    clock_ns(libc::CLOCK_BOOTTIME)
}

/// CLOCK_MONOTONIC (stops while the device is suspended), nanoseconds
#[cfg(target_os = "android")]
pub fn mono_ns() -> i64 {
    clock_ns(libc::CLOCK_MONOTONIC)
}

#[cfg(not(target_os = "android"))]
fn process_clock_ns() -> i64 {
    static START: OnceLock<Instant> = OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_nanos() as i64
}

#[cfg(not(target_os = "android"))]
pub fn boot_ns() -> i64 {
    process_clock_ns()
}

#[cfg(not(target_os = "android"))]
pub fn mono_ns() -> i64 {
    process_clock_ns()
}

#[cfg(target_os = "android")]
fn timestamp() -> String {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    let seconds = now.as_secs() as libc::time_t;
    let mut tm: libc::tm = unsafe { mem::zeroed() };
    let wall = if unsafe { libc::localtime_r(&raw const seconds, &raw mut tm) }.is_null() {
        format!("epoch={}.{:03}", now.as_secs(), now.subsec_millis())
    } else {
        let offset_minutes = tm.tm_gmtoff / 60;
        format!(
            "{:04}-{:02}-{:02} {:02}:{:02}:{:02}.{:03}{}{:02}:{:02}",
            tm.tm_year + 1900,
            tm.tm_mon + 1,
            tm.tm_mday,
            tm.tm_hour,
            tm.tm_min,
            tm.tm_sec,
            now.subsec_millis(),
            if offset_minutes < 0 { '-' } else { '+' },
            offset_minutes.abs() / 60,
            offset_minutes.abs() % 60
        )
    };
    format!(
        "{wall} mono={:.3} boot={:.3}",
        clock_seconds(libc::CLOCK_MONOTONIC),
        clock_seconds(libc::CLOCK_BOOTTIME)
    )
}

#[cfg(not(target_os = "android"))]
fn timestamp() -> String {
    static START: OnceLock<Instant> = OnceLock::new();
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    format!(
        "epoch={}.{:03} mono={:.3}",
        now.as_secs(),
        now.subsec_millis(),
        START.get_or_init(Instant::now).elapsed().as_secs_f64()
    )
}

/// One diagnostic line (timestamped now). Never blocks, never fails.
pub fn log(text: impl AsRef<str>) {
    let line = format!("{} {}\n", timestamp(), text.as_ref());
    SENDER
        .get_or_init(|| {
            let (sender, receiver) = mpsc::channel::<String>();
            let spawned = thread::Builder::new()
                .name("tracking diag writer".into())
                .spawn(move || writer_loop(receiver));
            if let Err(e) = spawned {
                warn!("Tracking diag: cannot start the writer thread: {e}");
            } else {
                info!("Tracking diag: writing {DIR}/{FILE_NAME}");
            }
            sender
        })
        .send(line)
        .ok();
}

/// Logs a marker and asks the sampler for a full snapshot (stream start/stop and the like)
pub fn mark(text: &str) {
    log(format!("MARK {text}"));
    RESNAPSHOT.store(true, Ordering::Relaxed);
}

struct Writer {
    path: String,
    file: Option<File>,
    size: u64,
    error_logged: bool,
}

impl Writer {
    fn open(&mut self) {
        fs::create_dir_all(DIR).ok();
        match OpenOptions::new().create(true).append(true).open(&self.path) {
            Ok(file) => {
                self.size = file.metadata().map(|m| m.len()).unwrap_or(0);
                self.file = Some(file);
            }
            Err(e) => {
                if !self.error_logged {
                    warn!("Tracking diag: cannot open {}: {e}", self.path);
                    self.error_logged = true;
                }
            }
        }
    }

    fn write(&mut self, line: &str) {
        if self.file.is_none() {
            self.open();
        }
        if self.size + line.len() as u64 > MAX_BYTES && self.file.is_some() {
            self.file = None;
            fs::rename(&self.path, format!("{}.1", self.path)).ok();
            self.open();
        }
        let Some(file) = &mut self.file else {
            return;
        };
        // One write per line: in the kernel once it returns, so a killed app loses nothing
        match file.write_all(line.as_bytes()) {
            Ok(()) => self.size += line.len() as u64,
            Err(e) => {
                if !self.error_logged {
                    warn!("Tracking diag: write failed: {e}");
                    self.error_logged = true;
                }
                self.file = None;
            }
        }
    }
}

fn writer_loop(receiver: mpsc::Receiver<String>) {
    let mut writer = Writer {
        path: format!("{DIR}/{FILE_NAME}"),
        file: None,
        size: 0,
        error_logged: false,
    };
    writer.write(&format!(
        "{} ===== tracking diagnostics start, pid {} =====\n",
        timestamp(),
        std::process::id()
    ));
    let mut props = vec![None::<String>; PROPS.len()];
    let mut next_props = Instant::now();
    loop {
        let timeout = next_props.saturating_duration_since(Instant::now());
        match receiver.recv_timeout(timeout) {
            Ok(line) => writer.write(&line),
            Err(mpsc::RecvTimeoutError::Timeout) => (),
            Err(mpsc::RecvTimeoutError::Disconnected) => return,
        }
        if Instant::now() >= next_props {
            next_props = Instant::now() + PROP_INTERVAL;
            for (name, last) in PROPS.iter().zip(props.iter_mut()) {
                let value = system_property(name);
                if last.as_deref() != Some(value.as_str()) {
                    writer.write(&format!(
                        "{} PROP {name}=\"{value}\"{}\n",
                        timestamp(),
                        if last.is_none() { " (start)" } else { " (changed)" }
                    ));
                    *last = Some(value);
                }
            }
        }
    }
}

#[cfg(target_os = "android")]
fn system_property(name: &str) -> String {
    let Ok(name) = CString::new(name) else {
        return String::new();
    };
    // PROP_VALUE_MAX
    let mut value = [0 as libc::c_char; 92];
    let length = unsafe { libc::__system_property_get(name.as_ptr(), value.as_mut_ptr()) };
    if length <= 0 {
        return String::new();
    }
    let bytes: Vec<u8> = value
        .iter()
        .take_while(|c| **c != 0)
        .map(|c| *c as u8)
        .collect();
    String::from_utf8_lossy(&bytes).into_owned()
}

#[cfg(not(target_os = "android"))]
fn system_property(_name: &str) -> String {
    String::new()
}

// ---------------------------------------------------------------------------------------------
// Formatting

fn pose_text(pose: &Pose) -> String {
    let (yaw, pitch, roll) = pose.orientation.to_euler(EulerRot::YXZ);
    let p = pose.position;
    let q = pose.orientation;
    format!(
        "p=({:.4},{:.4},{:.4}) ypr=({:.2},{:.2},{:.2}) q=({:.5},{:.5},{:.5},{:.5})",
        p.x,
        p.y,
        p.z,
        yaw.to_degrees(),
        pitch.to_degrees(),
        roll.to_degrees(),
        q.x,
        q.y,
        q.z,
        q.w
    )
}

fn difference(a: &Pose, b: &Pose) -> (f32, f32) {
    (
        a.position.distance(b.position),
        a.orientation.angle_between(b.orientation).to_degrees(),
    )
}

fn changed(a: &Pose, b: &Pose) -> bool {
    let (meters, degrees) = difference(a, b);
    meters > CHANGE_METERS || degrees > CHANGE_DEGREES
}

fn anchor_state_name(state: u32) -> &'static str {
    match state {
        0 => "none",
        1 => "creating",
        2 => "located",
        3 => "lost",
        4 => "unsupported",
        5 => "error",
        _ => "?",
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

// ---------------------------------------------------------------------------------------------
// Events (render thread)

fn event_text(raw: &sys::EventDataBuffer) -> String {
    let ty = raw.ty;
    let base = ptr::from_ref(raw);
    // SAFETY: the runtime wrote a structure of type `ty` into the buffer, which is larger and at
    // least as aligned as any event structure
    unsafe {
        match ty {
            sys::StructureType::EVENT_DATA_SESSION_STATE_CHANGED => {
                let e = &*base.cast::<sys::EventDataSessionStateChanged>();
                format!("state={:?} time={}", e.state, e.time.as_nanos())
            }
            sys::StructureType::EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING => {
                let e = &*base.cast::<sys::EventDataReferenceSpaceChangePending>();
                format!(
                    "space={:?} changeTime={} poseValid={} poseInPreviousSpace: {}",
                    e.reference_space_type,
                    e.change_time.as_nanos(),
                    bool::from(e.pose_valid),
                    pose_text(&crate::from_xr_pose(e.pose_in_previous_space))
                )
            }
            sys::StructureType::EVENT_DATA_INSTANCE_LOSS_PENDING => {
                let e = &*base.cast::<sys::EventDataInstanceLossPending>();
                format!("lossTime={}", e.loss_time.as_nanos())
            }
            sys::StructureType::EVENT_DATA_EVENTS_LOST => {
                let e = &*base.cast::<sys::EventDataEventsLost>();
                format!("lost={}", e.lost_event_count)
            }
            sys::StructureType::EVENT_DATA_USER_PRESENCE_CHANGED_EXT => {
                let e = &*base.cast::<sys::EventDataUserPresenceChangedEXT>();
                format!("present={}", bool::from(e.is_user_present))
            }
            sys::StructureType::EVENT_DATA_PERF_SETTINGS_EXT => {
                let e = &*base.cast::<sys::EventDataPerfSettingsEXT>();
                format!(
                    "domain={:?} subDomain={:?} {:?} -> {:?}",
                    e.domain, e.sub_domain, e.from_level, e.to_level
                )
            }
            sys::StructureType::EVENT_DATA_SENSE_DATA_PROVIDER_STATE_CHANGED_BD => {
                let e = &*base.cast::<sys::EventDataSenseDataProviderStateChangedBD>();
                format!("newState={:?}", e.new_state)
            }
            sys::StructureType::EVENT_DATA_INTERACTION_PROFILE_CHANGED => String::new(),
            // Anything else (PICO/BD vendor events): the first bytes after type and next
            _ => format!("raw={}", hex(&raw.varying[..64])),
        }
    }
}

fn log_event(raw: &sys::EventDataBuffer) {
    let ty = raw.ty;
    log(format!(
        "EVENT {} {:?} {}",
        ty.into_raw(),
        ty,
        event_text(raw)
    ));
    if matches!(
        ty,
        sys::StructureType::EVENT_DATA_SESSION_STATE_CHANGED
            | sys::StructureType::EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING
            | sys::StructureType::EVENT_DATA_USER_PRESENCE_CHANGED_EXT
    ) {
        RESNAPSHOT.store(true, Ordering::Relaxed);
    }

    // Tracking continuity (tracking_state.rs): queued, never blocks
    let base = ptr::from_ref(raw);
    match ty {
        sys::StructureType::EVENT_DATA_SESSION_STATE_CHANGED => {
            // SAFETY: see event_text()
            let e = unsafe { &*base.cast::<sys::EventDataSessionStateChanged>() };
            if e.state == sys::SessionState::READY {
                crate::tracking_state::on_session_ready();
            }
        }
        sys::StructureType::EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING => {
            // SAFETY: see event_text()
            let e = unsafe { &*base.cast::<sys::EventDataReferenceSpaceChangePending>() };
            let kind = match e.reference_space_type {
                sys::ReferenceSpaceType::STAGE => crate::tracking_state::SpaceKind::Stage,
                sys::ReferenceSpaceType::LOCAL => crate::tracking_state::SpaceKind::Local,
                _ => crate::tracking_state::SpaceKind::Other,
            };
            crate::tracking_state::on_reference_space_change(kind, e.change_time.as_nanos());
        }
        _ if ty.into_raw() == crate::tracking_state::PICO_TRACKING_EVENT_TYPE => {
            let value = u32::from_le_bytes([
                raw.varying[8],
                raw.varying[9],
                raw.varying[10],
                raw.varying[11],
            ]);
            crate::tracking_state::on_vendor_tracking_event(value);
        }
        _ => (),
    }
}

/// xr::Instance::poll_event, plus a diagnostic line for every event the runtime delivers,
/// including the types the openxr crate does not know (which it skips, as before)
pub fn poll_event<'a>(
    instance: &xr::Instance,
    storage: &'a mut MaybeUninit<sys::EventDataBuffer>,
) -> xr::Result<Option<xr::Event<'a>>> {
    let storage: *mut MaybeUninit<sys::EventDataBuffer> = storage;
    loop {
        unsafe {
            (*storage)
                .as_mut_ptr()
                .cast::<sys::BaseInStructure>()
                .write(sys::BaseInStructure {
                    ty: sys::EventDataBuffer::TYPE,
                    next: ptr::null(),
                });
            let result = (instance.fp().poll_event)(instance.as_raw(), (*storage).as_mut_ptr());
            if result.into_raw() < 0 {
                return Err(result);
            }
            if result == sys::Result::EVENT_UNAVAILABLE {
                return Ok(None);
            }
            log_event((*storage).assume_init_ref());
            if let Some(event) = xr::Event::from_raw(&*storage) {
                return Ok(Some(event));
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// XR_PICO_virtual_boundary (raw function pointers; layouts from the PICO OpenXR SDK 3.0 openxr.h)

#[repr(C)]
struct VirtualBoundaryInfoPico {
    ty: sys::StructureType,
    next: *const c_void,
    base_space: sys::Space,
    edge_type: i32,
}

#[repr(C)]
struct VirtualBoundaryStatusPico {
    ty: sys::StructureType,
    next: *mut c_void,
    is_ready: sys::Bool32,
    is_enabled: sys::Bool32,
    is_visible: sys::Bool32,
}

type GetBoundaryModeFn = unsafe extern "system" fn(sys::Session, *mut i32) -> sys::Result;
type GetBoundaryStatusFn =
    unsafe extern "system" fn(sys::Session, *mut VirtualBoundaryStatusPico) -> sys::Result;
type GetBoundaryGeometryFn = unsafe extern "system" fn(
    sys::Session,
    *const VirtualBoundaryInfoPico,
    u32,
    *mut u32,
    *mut sys::Vector3f,
) -> sys::Result;

#[derive(Clone, Copy)]
struct BoundaryFns {
    get_mode: GetBoundaryModeFn,
    get_status: GetBoundaryStatusFn,
    get_geometry: GetBoundaryGeometryFn,
}

fn instance_proc<F>(instance: &xr::Instance, name: &str) -> Option<F> {
    let name = CString::new(name).ok()?;
    let mut function = None;
    let result = unsafe {
        (instance.fp().get_instance_proc_addr)(instance.as_raw(), name.as_ptr(), &mut function)
    };
    if result != sys::Result::SUCCESS {
        return None;
    }
    // SAFETY: F is the matching extern "system" fn pointer type (same size as the void pointer)
    function.map(|f| unsafe { mem::transmute_copy(&f) })
}

impl BoundaryFns {
    fn load(instance: &xr::Instance) -> Option<Self> {
        Some(Self {
            get_mode: instance_proc(instance, "xrGetVirtualBoundaryModePICO")?,
            get_status: instance_proc(instance, "xrGetVirtualBoundaryStatusPICO")?,
            get_geometry: instance_proc(instance, "xrGetVirtualBoundaryGeometryPICO")?,
        })
    }

    fn mode_text(&self, session: sys::Session) -> String {
        let mut mode = -1;
        let result = unsafe { (self.get_mode)(session, &raw mut mode) };
        if result != sys::Result::SUCCESS {
            return format!("mode=err:{result:?}");
        }
        match mode {
            0 => "mode=STATIONARY".into(),
            1 => "mode=CUSTOM".into(),
            other => format!("mode={other}"),
        }
    }

    fn status_text(&self, session: sys::Session) -> String {
        let mut status = VirtualBoundaryStatusPico {
            ty: sys::StructureType::from_raw(TYPE_VIRTUAL_BOUNDARY_STATUS_PICO),
            next: ptr::null_mut(),
            is_ready: sys::FALSE,
            is_enabled: sys::FALSE,
            is_visible: sys::FALSE,
        };
        let result = unsafe { (self.get_status)(session, &raw mut status) };
        if result != sys::Result::SUCCESS {
            return format!("status=err:{result:?}");
        }
        format!(
            "ready={} enabled={} visible={}",
            bool::from(status.is_ready),
            bool::from(status.is_enabled),
            bool::from(status.is_visible)
        )
    }

    // Point count, bounding box (mm) and centroid in the given space
    fn geometry_text(&self, session: sys::Session, space: sys::Space, edge_type: i32) -> String {
        let info = VirtualBoundaryInfoPico {
            ty: sys::StructureType::from_raw(TYPE_VIRTUAL_BOUNDARY_INFO_PICO),
            next: ptr::null(),
            base_space: space,
            edge_type,
        };
        let mut count = 0u32;
        let result =
            unsafe { (self.get_geometry)(session, &raw const info, 0, &raw mut count, ptr::null_mut()) };
        if result != sys::Result::SUCCESS {
            return format!("err:{result:?}");
        }
        if count == 0 {
            return "n=0".into();
        }
        let capacity = count.min(BOUNDARY_GEOMETRY_MAX_POINTS);
        let mut points = vec![
            sys::Vector3f {
                x: 0.0,
                y: 0.0,
                z: 0.0
            };
            capacity as usize
        ];
        let mut written = 0u32;
        let result = unsafe {
            (self.get_geometry)(
                session,
                &raw const info,
                capacity,
                &raw mut written,
                points.as_mut_ptr(),
            )
        };
        if result != sys::Result::SUCCESS {
            return format!("n={count} err:{result:?}");
        }
        let points = &points[..(written.min(capacity) as usize)];
        if points.is_empty() {
            return format!("n={count} written=0");
        }
        let mut min = [f32::MAX; 3];
        let mut max = [f32::MIN; 3];
        let mut sum = [0.0f64; 3];
        for p in points {
            for (i, v) in [p.x, p.y, p.z].into_iter().enumerate() {
                min[i] = min[i].min(v);
                max[i] = max[i].max(v);
                sum[i] += f64::from(v);
            }
        }
        let n = points.len() as f64;
        let mm = |v: f32| (v * 1000.0).round() as i64;
        format!(
            "n={count} x[{},{}] y[{},{}] z[{},{}] mm centroid=({},{},{}) mm",
            mm(min[0]),
            mm(max[0]),
            mm(min[1]),
            mm(max[1]),
            mm(min[2]),
            mm(max[2]),
            ((sum[0] / n) * 1000.0).round() as i64,
            ((sum[1] / n) * 1000.0).round() as i64,
            ((sum[2] / n) * 1000.0).round() as i64
        )
    }
}

// ---------------------------------------------------------------------------------------------
// Sampler

#[derive(Default)]
struct Shared {
    stop: AtomicBool,
    running: AtomicBool,
    // Render thread's last predicted display time (clock fallback without XR_KHR_convert_timespec_time)
    frame_time: AtomicI64,
    // Persistent anchor: VrAnchorState value and pose in STAGE
    anchor: Mutex<Option<(u32, Option<Pose>)>>,
}

/// Owned by the render loop for one session. Dropping it stops and joins the sampler thread
/// (at most one sample interval), so the session is never kept alive by it.
pub struct TrackingDiag {
    shared: Arc<Shared>,
    thread: Option<JoinHandle<()>>,
}

impl TrackingDiag {
    pub fn new(session: &xr::Session<xr::OpenGlEs>, boundary_enabled: bool) -> Self {
        let shared = Arc::new(Shared::default());
        let instance = session.instance();
        let boundary = if boundary_enabled {
            let fns = BoundaryFns::load(instance);
            log(format!(
                "{VIRTUAL_BOUNDARY_EXTENSION_NAME} enabled, functions {}",
                if fns.is_some() { "loaded" } else { "NOT found" }
            ));
            fns
        } else {
            log(format!(
                "{VIRTUAL_BOUNDARY_EXTENSION_NAME} not offered/enabled: no boundary polling"
            ));
            None
        };
        if let Ok(properties) = instance.properties() {
            log(format!(
                "SESSION created, runtime {} {}",
                properties.runtime_name, properties.runtime_version
            ));
        }
        RESNAPSHOT.store(true, Ordering::Relaxed);
        crate::tracking_state::on_session_created();

        let thread = {
            let shared = Arc::clone(&shared);
            let session = session.clone();
            thread::Builder::new()
                .name("tracking diag".into())
                .spawn(move || sampler_loop(&session, &shared, boundary))
                .map_err(|e| warn!("Tracking diag: cannot start the sampler thread: {e}"))
                .ok()
        };
        Self { shared, thread }
    }

    pub fn set_running(&self, running: bool) {
        self.shared.running.store(running, Ordering::Relaxed);
    }

    pub fn set_frame_time(&self, time: xr::Time) {
        self.shared
            .frame_time
            .store(time.as_nanos(), Ordering::Relaxed);
    }

    /// Never blocks: skipped while the sampler reads it
    pub fn set_anchor(&self, anchor: Option<(u32, Option<Pose>)>) {
        if let Ok(mut slot) = self.shared.anchor.try_lock() {
            *slot = anchor;
        }
    }
}

impl Drop for TrackingDiag {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            thread.join().ok();
        }
    }
}

#[derive(Clone, Copy)]
struct Located {
    flags: xr::SpaceLocationFlags,
    pose: Option<Pose>,
}

fn locate(space: Option<&xr::Space>, base: Option<&xr::Space>, time: xr::Time) -> Option<Located> {
    let (space, base) = (space?, base?);
    match space.locate(base, time) {
        Ok(location) => Some(Located {
            flags: location.location_flags,
            pose: location
                .location_flags
                .contains(
                    xr::SpaceLocationFlags::POSITION_VALID
                        | xr::SpaceLocationFlags::ORIENTATION_VALID,
                )
                .then(|| crate::from_xr_pose(location.pose)),
        }),
        Err(_) => Some(Located {
            flags: xr::SpaceLocationFlags::EMPTY,
            pose: None,
        }),
    }
}

fn flags_text(flags: xr::SpaceLocationFlags) -> String {
    let names = [
        (xr::SpaceLocationFlags::POSITION_VALID, "PV"),
        (xr::SpaceLocationFlags::POSITION_TRACKED, "PT"),
        (xr::SpaceLocationFlags::ORIENTATION_VALID, "OV"),
        (xr::SpaceLocationFlags::ORIENTATION_TRACKED, "OT"),
    ];
    let present: Vec<&str> = names
        .iter()
        .filter(|(flag, _)| flags.contains(*flag))
        .map(|(_, name)| *name)
        .collect();
    if present.is_empty() {
        "none".into()
    } else {
        present.join("|")
    }
}

fn located_text(located: Option<Located>) -> String {
    match located {
        None => "n/a".into(),
        Some(Located { flags, pose }) => match pose {
            Some(pose) => format!("[{}] {}", flags_text(flags), pose_text(&pose)),
            None => format!("[{}]", flags_text(flags)),
        },
    }
}

// A world-locked relation logged on flag changes and on pose changes beyond CHANGE_*
#[derive(Default)]
struct Relation {
    name: &'static str,
    flags: Option<xr::SpaceLocationFlags>,
    logged: Option<Pose>,
}

impl Relation {
    fn new(name: &'static str) -> Self {
        Self {
            name,
            ..Default::default()
        }
    }

    fn update(&mut self, located: Option<Located>, extra: impl Fn(&Pose) -> String) {
        let Some(located) = located else {
            return;
        };
        if self.flags != Some(located.flags) {
            log(format!(
                "{} flags {} -> {}",
                self.name,
                self.flags.map_or("start".into(), flags_text),
                flags_text(located.flags)
            ));
            self.flags = Some(located.flags);
        }
        let Some(pose) = located.pose else {
            return;
        };
        match self.logged {
            Some(logged) if !changed(&pose, &logged) => (),
            logged => {
                let delta = logged.map_or(String::new(), |logged| {
                    let (meters, degrees) = difference(&pose, &logged);
                    format!(" (moved {:.1} mm, {degrees:.2} deg)", meters * 1000.0)
                });
                log(format!(
                    "{} {}{delta}{}",
                    self.name,
                    pose_text(&pose),
                    extra(&pose)
                ));
                self.logged = Some(pose);
            }
        }
    }
}

struct Spaces {
    view: Option<xr::Space>,
    local: Option<xr::Space>,
    stage: Option<xr::Space>,
    local_floor: Option<xr::Space>,
    types: Vec<xr::ReferenceSpaceType>,
}

struct Sampler {
    stage_in_local: Relation,
    floor_in_local: Relation,
    view_flags: Option<xr::SpaceLocationFlags>,
    first_view_pending: bool,
    // Last two head samples (xr time in ns, pose in STAGE) for the jump detector
    previous_views: [Option<(i64, Pose)>; 2],
    last_jump_log: Option<Instant>,
    suppressed_jumps: u32,
    last_heartbeat: Option<Instant>,
    last_bounds: Option<Instant>,
    bounds_text: String,
    last_boundary: Option<Instant>,
    boundary_text: String,
    anchor_state: Option<u32>,
    anchor_logged: Option<Pose>,
    current_stage_in_local: Option<Pose>,
}

fn bounds_text(session: &xr::Session<xr::OpenGlEs>, types: &[xr::ReferenceSpaceType]) -> String {
    types
        .iter()
        .filter(|ty| **ty != xr::ReferenceSpaceType::VIEW)
        .map(|ty| {
            let bounds = match session.reference_space_bounds_rect(*ty) {
                Ok(Some(extent)) => format!("{:.3}x{:.3}", extent.width, extent.height),
                Ok(None) => "unavailable".into(),
                Err(e) => format!("err:{e:?}"),
            };
            format!("{ty:?}={bounds}")
        })
        .collect::<Vec<_>>()
        .join(" ")
}

impl Sampler {
    fn snapshot(&mut self, session: &xr::Session<xr::OpenGlEs>, spaces: &Spaces) {
        let types = session.enumerate_reference_spaces();
        log(format!("SNAPSHOT reference spaces: {types:?}"));
        self.bounds_text = bounds_text(session, &spaces.types);
        log(format!("SNAPSHOT bounds: {}", self.bounds_text));
        self.last_bounds = Some(Instant::now());
        // Log every relation again at its next valid sample
        self.stage_in_local.logged = None;
        self.floor_in_local.logged = None;
        self.anchor_logged = None;
        self.first_view_pending = true;
        self.previous_views = [None, None];
        self.last_heartbeat = None;
        self.last_boundary = None;
        self.boundary_text.clear();
    }

    fn sample(
        &mut self,
        session: &xr::Session<xr::OpenGlEs>,
        spaces: &Spaces,
        boundary: Option<&BoundaryFns>,
        shared: &Shared,
        time: xr::Time,
    ) {
        if RESNAPSHOT.swap(false, Ordering::Relaxed) {
            self.snapshot(session, spaces);
        }

        let view_stage = locate(spaces.view.as_ref(), spaces.stage.as_ref(), time);
        let view_local = locate(spaces.view.as_ref(), spaces.local.as_ref(), time);
        let stage_local = locate(spaces.stage.as_ref(), spaces.local.as_ref(), time);
        let floor_local = locate(spaces.local_floor.as_ref(), spaces.local.as_ref(), time);

        // STAGE origin and PICO's floor relative to LOCAL (the session-start head pose)
        self.stage_in_local.update(stage_local, |_| String::new());
        self.floor_in_local.update(floor_local, |pose| {
            format!(" floor_below_local_origin={:.4}", -pose.position.y)
        });
        self.current_stage_in_local = stage_local.and_then(|l| l.pose);

        // Tracking continuity (tracking_state.rs), from the same locates
        if let Some(view) = view_stage {
            crate::tracking_state::on_sample(crate::tracking_state::Sample {
                boot_ns: boot_ns(),
                mono_ns: mono_ns(),
                xr_ns: time.as_nanos(),
                position_tracked: view
                    .flags
                    .contains(xr::SpaceLocationFlags::POSITION_TRACKED),
                orientation_tracked: view
                    .flags
                    .contains(xr::SpaceLocationFlags::ORIENTATION_TRACKED),
                view_in_stage: view.pose,
                stage_in_local: self.current_stage_in_local,
            });
        }

        self.update_view(view_stage, time);

        if self
            .last_heartbeat
            .is_none_or(|t| t.elapsed() >= HEARTBEAT_INTERVAL)
        {
            self.last_heartbeat = Some(Instant::now());
            let head_above_local_floor = view_local
                .and_then(|l| l.pose)
                .zip(floor_local.and_then(|l| l.pose))
                .map_or("n/a".into(), |(view, floor)| {
                    format!("{:.4}", (floor.inverse() * view).position.y)
                });
            log(format!(
                "HB head_in_STAGE {} | head_above_STAGE_floor={} head_above_LOCAL_FLOOR={} | \
                 head_in_LOCAL {} | STAGE_in_LOCAL {} | LOCAL_FLOOR_in_LOCAL {} | running={}",
                located_text(view_stage),
                view_stage
                    .and_then(|l| l.pose)
                    .map_or("n/a".into(), |p| format!("{:.4}", p.position.y)),
                head_above_local_floor,
                located_text(view_local),
                located_text(stage_local),
                located_text(floor_local),
                shared.running.load(Ordering::Relaxed)
            ));
        }

        if self
            .last_bounds
            .is_none_or(|t| t.elapsed() >= BOUNDS_INTERVAL)
        {
            self.last_bounds = Some(Instant::now());
            let text = bounds_text(session, &spaces.types);
            if text != self.bounds_text {
                log(format!("BOUNDS changed: {} -> {text}", self.bounds_text));
                self.bounds_text = text;
            }
        }

        self.update_anchor(shared);

        if let Some(boundary) = boundary
            && self
                .last_boundary
                .is_none_or(|t| t.elapsed() >= BOUNDARY_INTERVAL)
        {
            self.last_boundary = Some(Instant::now());
            let handle = session.as_raw();
            let mut text = format!(
                "{} {}",
                boundary.mode_text(handle),
                boundary.status_text(handle)
            );
            if let Some(stage) = &spaces.stage {
                text += &format!(
                    " | in STAGE: outer {} | rect {}",
                    boundary.geometry_text(handle, stage.as_raw(), BOUNDARY_EDGE_OUTER_CONTOUR),
                    boundary.geometry_text(
                        handle,
                        stage.as_raw(),
                        BOUNDARY_EDGE_INSCRIBED_RECTANGLE
                    )
                );
            }
            if text != self.boundary_text {
                log(format!("BOUNDARY {text}"));
                self.boundary_text = text;
            }
        }
    }

    fn update_view(&mut self, view_stage: Option<Located>, time: xr::Time) {
        let Some(located) = view_stage else {
            return;
        };
        if self.view_flags != Some(located.flags) {
            log(format!(
                "VIEW_in_STAGE flags {} -> {}",
                self.view_flags.map_or("start".into(), flags_text),
                flags_text(located.flags)
            ));
            self.view_flags = Some(located.flags);
        }
        let Some(pose) = located.pose else {
            self.previous_views = [None, None];
            return;
        };

        if self.first_view_pending {
            self.first_view_pending = false;
            let (yaw, _, _) = pose.orientation.to_euler(EulerRot::YXZ);
            let reset_look = pose.position.x.abs() <= ORIGIN_RESET_METERS
                && pose.position.z.abs() <= ORIGIN_RESET_METERS
                && yaw.to_degrees().abs() <= ORIGIN_RESET_DEGREES;
            log(format!(
                "FIRST VIEW_in_STAGE [{}] {} head_above_STAGE_floor={:.4}{}",
                flags_text(located.flags),
                pose_text(&pose),
                pose.position.y,
                if reset_look {
                    " ORIGIN-RESET LOOK (x=z=0, yaw 0)"
                } else {
                    ""
                }
            ));
        }

        let now = time.as_nanos();
        if let [Some((t1, p1)), Some((t0, p0))] = self.previous_views
            && now > t1
            && t1 > t0
            && now - t1 <= JUMP_MAX_GAP_NANOS
            && t1 - t0 <= JUMP_MAX_GAP_NANOS
        {
            // Constant-velocity prediction from the two previous samples
            let ratio = (now - t1) as f32 / (t1 - t0) as f32;
            let predicted_position = p1.position + (p1.position - p0.position) * ratio;
            let rotation_step = (p1.orientation * p0.orientation.inverse()).to_scaled_axis();
            let predicted_orientation =
                (Quat::from_scaled_axis(rotation_step * ratio) * p1.orientation).normalize();
            let position_error = pose.position.distance(predicted_position);
            let angle_error = pose
                .orientation
                .angle_between(predicted_orientation)
                .to_degrees();
            if position_error > JUMP_METERS || angle_error > JUMP_DEGREES {
                if self
                    .last_jump_log
                    .is_none_or(|t| t.elapsed() >= JUMP_LOG_MIN_INTERVAL)
                {
                    let (step_meters, step_degrees) = difference(&pose, &p1);
                    log(format!(
                        "JUMP VIEW_in_STAGE off prediction {:.1} mm {angle_error:.2} deg, step \
                         {:.1} mm {step_degrees:.2} deg in {:.1} ms (suppressed since last: {}) \
                         before {} after {}",
                        position_error * 1000.0,
                        step_meters * 1000.0,
                        (now - t1) as f64 / 1e6,
                        self.suppressed_jumps,
                        pose_text(&p1),
                        pose_text(&pose)
                    ));
                    self.last_jump_log = Some(Instant::now());
                    self.suppressed_jumps = 0;
                } else {
                    self.suppressed_jumps += 1;
                }
            }
        }
        if self.previous_views[0].is_none_or(|(t, _)| now > t) {
            self.previous_views = [Some((now, pose)), self.previous_views[0]];
        }
    }

    fn update_anchor(&mut self, shared: &Shared) {
        let Ok(anchor) = shared.anchor.lock().map(|slot| *slot) else {
            return;
        };
        let Some((state, pose)) = anchor else {
            return;
        };
        if self.anchor_state != Some(state) {
            log(format!(
                "ANCHOR state {} -> {}",
                self.anchor_state.map_or("start", anchor_state_name),
                anchor_state_name(state)
            ));
            self.anchor_state = Some(state);
        }
        let Some(pose) = pose else {
            return;
        };
        if self.anchor_logged.is_none_or(|logged| changed(&pose, &logged)) {
            let delta = self.anchor_logged.map_or(String::new(), |logged| {
                let (meters, degrees) = difference(&pose, &logged);
                format!(" (moved {:.1} mm, {degrees:.2} deg)", meters * 1000.0)
            });
            log(format!(
                "ANCHOR ({}) in STAGE {}{delta} | in LOCAL {}",
                anchor_state_name(state),
                pose_text(&pose),
                self.current_stage_in_local
                    .map_or("n/a".into(), |stage| pose_text(&(stage * pose)))
            ));
            self.anchor_logged = Some(pose);
        }
    }
}

fn sampler_loop(
    session: &xr::Session<xr::OpenGlEs>,
    shared: &Shared,
    boundary: Option<BoundaryFns>,
) {
    let instance = session.instance().clone();
    let can_now = instance.exts().khr_convert_timespec_time.is_some();
    let types = session.enumerate_reference_spaces().unwrap_or_default();
    let create = |ty: xr::ReferenceSpaceType| {
        (ty == xr::ReferenceSpaceType::VIEW || types.contains(&ty))
            .then(|| session.create_reference_space(ty, xr::Posef::IDENTITY).ok())
            .flatten()
    };
    let spaces = Spaces {
        view: create(xr::ReferenceSpaceType::VIEW),
        local: create(xr::ReferenceSpaceType::LOCAL),
        stage: create(xr::ReferenceSpaceType::STAGE),
        local_floor: create(xr::ReferenceSpaceType::LOCAL_FLOOR_EXT),
        types: types.clone(),
    };
    log(format!(
        "SAMPLER start: spaces view={} local={} stage={} local_floor={}, clock {}",
        spaces.view.is_some(),
        spaces.local.is_some(),
        spaces.stage.is_some(),
        spaces.local_floor.is_some(),
        if can_now {
            "xrConvertTimespecTimeToTimeKHR"
        } else {
            "render frame time"
        }
    ));

    let mut sampler = Sampler {
        stage_in_local: Relation::new("STAGE_in_LOCAL"),
        floor_in_local: Relation::new("LOCAL_FLOOR_in_LOCAL"),
        view_flags: None,
        first_view_pending: true,
        previous_views: [None, None],
        last_jump_log: None,
        suppressed_jumps: 0,
        last_heartbeat: None,
        last_bounds: None,
        bounds_text: String::new(),
        last_boundary: None,
        boundary_text: String::new(),
        anchor_state: None,
        anchor_logged: None,
        current_stage_in_local: None,
    };

    while !shared.stop.load(Ordering::Relaxed) {
        let time = if can_now {
            instance.now().ok().filter(|t| t.as_nanos() > 0)
        } else {
            None
        }
        .or_else(|| {
            let nanos = shared.frame_time.load(Ordering::Relaxed);
            (nanos > 0).then(|| xr::Time::from_nanos(nanos))
        });
        if let Some(time) = time {
            sampler.sample(session, &spaces, boundary.as_ref(), shared, time);
        }

        let interval = if shared.running.load(Ordering::Relaxed) {
            SAMPLE_INTERVAL_RUNNING
        } else {
            SAMPLE_INTERVAL_IDLE
        };
        let until = Instant::now() + interval;
        while !shared.stop.load(Ordering::Relaxed) && Instant::now() < until {
            thread::sleep(Duration::from_millis(20).min(interval));
        }
    }
    log("SAMPLER stop");
    // The spaces are destroyed here, before the session clone
    drop(spaces);
}
