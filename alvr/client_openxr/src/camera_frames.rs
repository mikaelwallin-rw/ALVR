// Streams headset camera frames to the server while it requests them (overlay calibration, e.g.
// ChArUco boards on the physical monitors). Pico only: frames come from SecureMR, see
// extra_extensions/secure_mr_camera.rs.

use crate::extra_extensions::{CameraCapture, SecureMrCamera};
use alvr_client_core::ClientCoreContext;
use alvr_common::{
    Pose, RelaxedAtomic,
    glam::{Mat3, Quat, Vec3},
    info, warn,
};
use alvr_packets::{CameraEye, CameraFrame, CameraFramesConfig};
use openxr as xr;
use std::{
    f32::consts::PI,
    sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

// The first frames after the camera starts may carry a different clock and unsettled exposure
const WARMUP: Duration = Duration::from_millis(500);
// A capture timestamp further than this from "now" is not on the runtime clock
const MAX_CAPTURE_AGE: Duration = Duration::from_secs(1);
const JPEG_QUALITY: u8 = 95;
const ERROR_RETRY_INTERVAL: Duration = Duration::from_secs(1);
// Start-up frames with a timestamp on another clock are retried at once within this time
const STARTUP_CLOCK_GRACE: Duration = Duration::from_secs(2);
// Frames per run whose raw poses are logged (CAMPOSE_DIAG) to validate the camera extrinsics
const POSE_DIAG_FRAMES: u64 = 10;
const TIMING_LOG_FRAMES: u32 = 20;

// Rectified camera poses relative to the head (VIEW space), OpenCV camera axes, per headset
// model (Android product name). The camera pose reported by SecureMR is relative to SecureMR's
// own LOCAL space, which does not always match the app's LOCAL space: on a PICO 4 Ultra Enterprise
// both were identical, and after the runtime re-established its spaces SecureMR's was 8.66 cm
// away (same rotation). The camera pose is therefore derived from the head pose with these
// extrinsics. The default set is CAMERA_EXTRINSICS_PICO below.
// LEGACY SET, KNOWN WRONG, selected only by switch value "fit" for A/B measurement, to be removed in
// the cleanup: measured by hand-eye calibration of CAMPOSE_DIAG data (head rotation up to 75
// degrees, residual below 1 mm), it reproduces SecureMR's reported camera pose, which is ~40 mm too
// high and ~34 mm too far back with the wrong roll sign. Both cameras share one rotation; baseline
// 64.1 mm.
const CAMERA_EXTRINSICS: &[(&str, CameraExtrinsics)] = &[(
    "PICO 4 Ultra Enterprise",
    CameraExtrinsics {
        left: Pose {
            orientation: Quat::from_xyzw(-0.999_959, 0.006_915, 0.001_195, 0.005_793),
            position: Vec3::new(-0.032_15, 0.039_04, -0.034_95),
        },
        right: Pose {
            orientation: Quat::from_xyzw(-0.999_959, 0.006_835, 0.001_114, 0.005_780),
            position: Vec3::new(0.031_97, 0.038_16, -0.035_14),
        },
    },
)];

// DEFAULT SET (debug switch screenvr_camera_extrinsics.txt absent or "pico"; confirmed correct in
// the headset 2026-11-11): PICO's own calibration, read on
// a PICO 4 Ultra Enterprise (PICO OS 5.15.7, 2026-11-11) with the enterprise capture library:
// com.pxr.capturelib.PXRCapture.getCameraExtrinsics (libpvrcapturelib.so ->
// /system_ext/lib64/libpxrcapturendk.so -> PXRCapture service). Raw row-major 4x4 per camera, metres,
// camera (OpenCV axes) -> PICO body frame (x up, y left, z back):
//   left  [ 0.0136635401 -0.999839348  0.0116010717 -0.0201851018
//          -0.999903774  -0.0136904047 -0.00223944977 0.0827398289
//           0.00239791337 -0.0115693566 -0.999930198 -0.0169578897 ]
//   right  same rotation, translation (-0.0193091163, 0.0186348305, -0.0168041567)
// PICO's Unreal plugin (PICOEnterprise libCameraRenderingPlugin.so, getCameraParametersNew) turns
// them into head-relative camera poses as A * E with the constant (.rodata, float, row-major)
//   A = [ 0 -1  0  0.05057 ;  1  0  0  0.01874 ;  0  0  1 -0.05194 ;  0 0 0 1 ]
// i.e. body axes -> (right, up, back) plus the body origin in the head frame. These are A * E for
// both cameras. Compared with the fitted set: same lateral position (0.03 mm), but 40.5/38.7 mm
// lower and 34.0/33.6 mm further forward, and rolled +1.57 deg (PICO +0.78, fit -0.79 deg) - the
// same roll as ScreenVR's empirical cameraRollCorrectionDeg -1.6 (so use 0 with this set).
const CAMERA_EXTRINSICS_PICO: &[(&str, CameraExtrinsics)] = &[(
    "PICO 4 Ultra Enterprise",
    CameraExtrinsics {
        left: Pose {
            orientation: Quat::from_xyzw(-0.999_959, -0.006_839, -0.001_159, 0.005_793),
            position: Vec3::new(-0.032_170, -0.001_445, -0.068_898),
        },
        right: Pose {
            orientation: Quat::from_xyzw(-0.999_959, -0.006_839, -0.001_159, 0.005_793),
            position: Vec3::new(0.031_935, -0.000_569, -0.068_744),
        },
    },
)];

/// Which extrinsics set the camera frames use (debug switch screenvr_camera_extrinsics.txt)
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum ExtrinsicsSet {
    // LEGACY, KNOWN WRONG (~40 mm too high, ~34 mm too far back, wrong roll sign): hand-eye fit of
    // SecureMR camera poses (CAMERA_EXTRINSICS). Only for A/B measurement via the switch value
    // "fit"; to be removed in the cleanup.
    Fit,
    // PICO's capture library calibration (CAMERA_EXTRINSICS_PICO), the default
    Pico,
}

impl ExtrinsicsSet {
    // "fit" selects the legacy set; anything else (absent, "pico", unknown text) the default
    fn from_switch(value: &str) -> Self {
        if value.eq_ignore_ascii_case("fit") {
            Self::Fit
        } else {
            Self::Pico
        }
    }

    fn table(self) -> &'static [(&'static str, CameraExtrinsics)] {
        match self {
            Self::Fit => CAMERA_EXTRINSICS,
            Self::Pico => CAMERA_EXTRINSICS_PICO,
        }
    }

    fn name(self) -> &'static str {
        match self {
            Self::Fit => "fit",
            Self::Pico => "pico",
        }
    }
}

// Roll, pitch, yaw (degrees) of a VIEW-from-camera rotation (OpenCV camera axes) in VIEW axes:
// roll = camera +x (image right) tipped up (counter-clockwise seen from behind the headset), pitch =
// optical axis tipped up, yaw = optical axis turned right
fn roll_pitch_yaw_deg(view_from_camera: Quat) -> (f32, f32, f32) {
    let x = view_from_camera * Vec3::X;
    let z = view_from_camera * Vec3::Z;
    (
        x.y.atan2(x.x).to_degrees(),
        z.y.clamp(-1.0, 1.0).asin().to_degrees(),
        z.x.atan2(-z.z).to_degrees(),
    )
}

fn extrinsics_text(extrinsics: &CameraExtrinsics) -> String {
    let side = |pose: Pose| {
        let p = pose.position * 1000.0;
        let (roll, pitch, yaw) = roll_pitch_yaw_deg(pose.orientation.normalize());
        format!(
            "({:.2}, {:.2}, {:.2}) mm roll {roll:.2} pitch {pitch:.2} yaw {yaw:.2} deg",
            p.x, p.y, p.z
        )
    };
    format!(
        "left {} | right {} (VIEW right/up/back)",
        side(extrinsics.left),
        side(extrinsics.right)
    )
}

#[derive(Clone, Copy)]
struct CameraExtrinsics {
    left: Pose,
    right: Pose,
}

impl CameraExtrinsics {
    fn view_from_camera(&self, eye: CameraEye) -> Pose {
        let pose = match eye {
            CameraEye::Left => self.left,
            CameraEye::Right => self.right,
        };

        Pose {
            orientation: pose.orientation.normalize(),
            position: pose.position,
        }
    }
}

#[cfg(target_os = "android")]
fn product_name() -> String {
    alvr_system_info::android::product_name()
}

#[cfg(not(target_os = "android"))]
fn product_name() -> String {
    String::new()
}

// For the unknown-model warning
#[cfg(target_os = "android")]
fn model_name() -> String {
    alvr_system_info::android::model_name()
}

#[cfg(not(target_os = "android"))]
fn model_name() -> String {
    String::new()
}

/// For the camera probe's comparison (extra_extensions/camera_image_probe.rs): product name and
/// VIEW-from-camera poses (OpenCV camera axes) of the left and right camera, if known
pub(crate) fn reference_extrinsics() -> (String, Option<(Pose, Pose)>) {
    let product = product_name();
    let extrinsics = CAMERA_EXTRINSICS
        .iter()
        .find(|(name, _)| *name == product)
        .map(|(_, extrinsics)| {
            (
                extrinsics.view_from_camera(CameraEye::Left),
                extrinsics.view_from_camera(CameraEye::Right),
            )
        });

    (product, extrinsics)
}

// Camera frame sources alive (normally 0 or 1), for the camera probe
static ACTIVE_SOURCES: AtomicUsize = AtomicUsize::new(0);

/// Whether the SecureMR camera pass is running (the camera probe avoids running alongside it)
pub(crate) fn camera_pass_active() -> bool {
    ACTIVE_SOURCES.load(Ordering::Relaxed) > 0
}

fn find_extrinsics(set: ExtrinsicsSet, product: &str) -> Option<CameraExtrinsics> {
    set.table()
        .iter()
        .find(|(name, _)| *name == product)
        .map(|(_, extrinsics)| *extrinsics)
}

// The extrinsics of the requested set for this model and the set requested. Unknown model: None
// (the camera pose reported by SecureMR is used; no extrinsics are invented) and a loud warning.
// Called once at the start of a camera pass (and again when the switch changes).
fn camera_extrinsics(requested: ExtrinsicsSet) -> (Option<CameraExtrinsics>, ExtrinsicsSet) {
    let product = product_name();
    let extrinsics = find_extrinsics(requested, &product);
    if extrinsics.is_none() {
        warn!(
            "CAMERA_EXTRINSICS UNKNOWN MODEL: no {:?} camera extrinsics for product/model {product:?} \
             (model {:?}); falling back to SecureMR's own camera pose. Monitor poses will be offset \
             by several cm (about 40 mm too high and 34 mm too near on the PICO 4 Ultra Enterprise) \
             until extrinsics for this model are added to camera_frames.rs",
            requested.name(),
            model_name()
        );
    }

    (extrinsics, requested)
}

fn requested_extrinsics_set() -> ExtrinsicsSet {
    ExtrinsicsSet::from_switch(&crate::debug_switches::get().camera_extrinsics)
}

// One line per camera pass start and per switch change: the set in use and its values
fn log_extrinsics_in_use(extrinsics: Option<&CameraExtrinsics>, set: ExtrinsicsSet) {
    match extrinsics {
        Some(extrinsics) => info!(
            "CAMERA_EXTRINSICS set={} {}",
            set.name(),
            extrinsics_text(extrinsics)
        ),
        None => info!("CAMERA_EXTRINSICS set=none (camera pose from SecureMR)"),
    }
}

struct ReferenceSpaces<'a> {
    local: &'a xr::Space,
    view: &'a xr::Space,
    stage: &'a xr::Space,
}

#[derive(Clone, Copy)]
struct PoseDiagnostics {
    // Raw poses (CAMPOSE_DIAG)
    log_poses: bool,
    // Difference between the SecureMR and the extrinsics camera pose (CAMPOSE_DIFF)
    log_path_difference: bool,
}

pub struct CameraFrameSource {
    running: Arc<RelaxedAtomic>,
    thread: Option<JoinHandle<()>>,
}

impl CameraFrameSource {
    pub fn new(
        core_context: Arc<ClientCoreContext>,
        xr_session: xr::Session<xr::OpenGlEs>,
        stage_reference_space: Arc<xr::Space>,
        config: CameraFramesConfig,
    ) -> Self {
        #[cfg(target_os = "android")]
        alvr_system_info::try_get_permission("android.permission.CAMERA");

        let running = Arc::new(RelaxedAtomic::new(true));
        ACTIVE_SOURCES.fetch_add(1, Ordering::Relaxed);

        let thread = thread::Builder::new()
            .name("camera_frames".into())
            .spawn({
                let running = Arc::clone(&running);
                move || {
                    capture_loop(
                        &core_context,
                        &xr_session,
                        &stage_reference_space,
                        config,
                        &running,
                    );
                }
            })
            .ok();

        Self { running, thread }
    }
}

impl Drop for CameraFrameSource {
    fn drop(&mut self) {
        self.running.set(false);
        if let Some(thread) = self.thread.take() {
            thread.join().ok();
        }
        ACTIVE_SOURCES.fetch_sub(1, Ordering::Relaxed);
    }
}

fn capture_loop(
    core_context: &ClientCoreContext,
    xr_session: &xr::Session<xr::OpenGlEs>,
    stage_reference_space: &xr::Space,
    config: CameraFramesConfig,
    running: &RelaxedAtomic,
) {
    let camera = match SecureMrCamera::new(xr_session, config.image_size, config.eye) {
        Ok(camera) => camera,
        Err(e) => {
            warn!("Camera frames: SecureMR camera unavailable: {e}");
            return;
        }
    };
    let local_reference_space = match xr_session
        .create_reference_space(xr::ReferenceSpaceType::LOCAL, xr::Posef::IDENTITY)
    {
        Ok(space) => space,
        Err(e) => {
            warn!("Camera frames: cannot create the LOCAL space: {e}");
            return;
        }
    };
    let view_reference_space = match xr_session
        .create_reference_space(xr::ReferenceSpaceType::VIEW, xr::Posef::IDENTITY)
    {
        Ok(space) => space,
        Err(e) => {
            warn!("Camera frames: cannot create the VIEW space: {e}");
            return;
        }
    };
    let mut requested_set = requested_extrinsics_set();
    let (mut extrinsics, mut extrinsics_set) = camera_extrinsics(requested_set);
    log_extrinsics_in_use(extrinsics.as_ref(), extrinsics_set);
    // Logs CAMPOSE_DIFF again for the first frame after a switch of the extrinsics set
    let mut log_difference_pending = false;
    info!(
        "Camera frames: started, {0}x{0} every {1:?}, camera pose from {2}",
        config.image_size,
        config.interval,
        if extrinsics.is_some() {
            "the head pose and the model's extrinsics"
        } else {
            "SecureMR"
        }
    );

    let started = Instant::now();
    let mut last_error = None;
    let mut sent_count = 0_u64;
    // Timing per stage, logged every TIMING_LOG_FRAMES frames: (capture, encode, send) seconds
    let mut timing = (0.0_f32, 0.0_f32, 0.0_f32);
    let mut timing_frames = 0_u32;
    let mut timing_window = Instant::now();
    while running.value() {
        let deadline = Instant::now() + config.interval;

        // Applied live: the switch is polled by the debug switches thread
        let requested = requested_extrinsics_set();
        if requested != requested_set {
            requested_set = requested;
            (extrinsics, extrinsics_set) = camera_extrinsics(requested);
            log_extrinsics_in_use(extrinsics.as_ref(), extrinsics_set);
            log_difference_pending = true;
        }
        let log_path_difference = started.elapsed() > WARMUP
            && (sent_count == 0 || log_difference_pending);

        let capture_start = Instant::now();
        let result = camera.capture().map_err(|e| format!("capture failed: {e}"));
        let encode_start = Instant::now();
        let frame = result.and_then(|capture| {
            make_frame(
                xr_session.instance(),
                &ReferenceSpaces {
                    local: &local_reference_space,
                    view: &view_reference_space,
                    stage: stage_reference_space,
                },
                capture,
                config.eye,
                extrinsics.map(|extrinsics| (extrinsics, extrinsics_set)),
                PoseDiagnostics {
                    log_poses: sent_count < POSE_DIAG_FRAMES,
                    // The first frame after the warm-up, and after a switch of the set
                    log_path_difference,
                },
            )
        });
        if log_path_difference && frame.is_ok() {
            log_difference_pending = false;
        }

        let send_start = Instant::now();
        match frame {
            Ok(frame) if started.elapsed() > WARMUP => {
                core_context.send_camera_frame(frame);
                sent_count += 1;
                if sent_count == 1 {
                    info!(
                        "Camera frames: first frame sent {} ms after the start",
                        started.elapsed().as_millis()
                    );
                }
                last_error = None;

                let done = Instant::now();
                timing.0 += (encode_start - capture_start).as_secs_f32();
                timing.1 += (send_start - encode_start).as_secs_f32();
                timing.2 += (done - send_start).as_secs_f32();
                timing_frames += 1;
                if timing_frames == TIMING_LOG_FRAMES {
                    let n = timing_frames as f32;
                    info!(
                        "Camera frames: {} frames at {:.1} fps; per frame capture {:.0} ms, pose+JPEG {:.0}                          ms, send {:.0} ms (interval {:?})",
                        timing_frames,
                        n / timing_window.elapsed().as_secs_f32(),
                        timing.0 / n * 1000.0,
                        timing.1 / n * 1000.0,
                        timing.2 / n * 1000.0,
                        config.interval
                    );
                    timing = (0.0, 0.0, 0.0);
                    timing_frames = 0;
                    timing_window = Instant::now();
                }
            }
            Ok(_) => (),
            Err(e) => {
                // The first capture after a start is stamped with another clock: not an error
                // worth backing off for (it cost a whole second before the first frame)
                let startup_frame = e.contains("not on the runtime clock")
                    && started.elapsed() < STARTUP_CLOCK_GRACE;
                if last_error.as_ref() != Some(&e) {
                    warn!("Camera frames: frame dropped: {e}");
                    last_error = Some(e);
                }
                if !startup_frame {
                    thread::sleep(ERROR_RETRY_INTERVAL);
                }
            }
        }

        if let Some(remaining) = deadline.checked_duration_since(Instant::now()) {
            thread::sleep(remaining);
        }
    }

    info!("Camera frames: stopped after {sent_count} frames");
}

// Headset CLOCK_MONOTONIC to XrTime
#[cfg(target_os = "android")]
fn to_runtime_time(instance: &xr::Instance, monotonic: Duration) -> Result<xr::Time, String> {
    let convert = instance
        .exts()
        .khr_convert_timespec_time
        .ok_or("XR_KHR_convert_timespec_time not enabled")?
        .convert_timespec_time_to_time;

    let timespec = libc::timespec {
        tv_sec: monotonic.as_secs() as _,
        tv_nsec: monotonic.subsec_nanos() as _,
    };
    let mut time = std::mem::MaybeUninit::uninit();
    let result = unsafe { convert(instance.as_raw(), &timespec, time.as_mut_ptr()) };
    if result != xr::sys::Result::SUCCESS {
        return Err(format!("time conversion failed: {result:?}"));
    }

    Ok(unsafe { time.assume_init() })
}

#[cfg(not(target_os = "android"))]
fn to_runtime_time(_instance: &xr::Instance, _monotonic: Duration) -> Result<xr::Time, String> {
    Err("camera frames are only supported on Android".into())
}

fn pose_text(pose: Pose) -> String {
    let p = pose.position;
    let q = pose.orientation;
    format!(
        "p=({:.4},{:.4},{:.4}) q=({:.5},{:.5},{:.5},{:.5})",
        p.x, p.y, p.z, q.x, q.y, q.z, q.w
    )
}

fn make_frame(
    instance: &xr::Instance,
    spaces: &ReferenceSpaces,
    capture: CameraCapture,
    eye: CameraEye,
    extrinsics: Option<(CameraExtrinsics, ExtrinsicsSet)>,
    diagnostics: PoseDiagnostics,
) -> Result<CameraFrame, String> {
    let capture_time = to_runtime_time(instance, capture.timestamp)?;

    // Rejects start-up frames stamped with another clock
    let now = instance.now().map_err(|e| format!("{e}"))?;
    let age_nanos = now.as_nanos().abs_diff(capture_time.as_nanos());
    if Duration::from_nanos(age_nanos) > MAX_CAPTURE_AGE {
        return Err(format!(
            "capture time {:?} is not on the runtime clock",
            capture.timestamp
        ));
    }

    let stage_from_local = spaces
        .local
        .locate(spaces.stage, capture_time)
        .map_err(|e| format!("cannot locate LOCAL in STAGE: {e}"))?;
    if !stage_from_local.location_flags.contains(
        xr::SpaceLocationFlags::POSITION_VALID | xr::SpaceLocationFlags::ORIENTATION_VALID,
    ) {
        return Err("LOCAL is not located in STAGE".into());
    }
    let stage_from_local = crate::from_xr_pose(stage_from_local.pose);

    // Row-major 4x4, OpenXR camera axes -> OpenCV camera axes (flip Y and Z)
    let m = capture.local_from_camera;
    let rotation = Mat3::from_cols(
        Vec3::new(m[0], m[4], m[8]),
        Vec3::new(m[1], m[5], m[9]),
        Vec3::new(m[2], m[6], m[10]),
    );
    let local_from_camera = Pose {
        orientation: Quat::from_mat3(&rotation).normalize() * Quat::from_rotation_x(PI),
        position: Vec3::new(m[3], m[7], m[11]),
    };
    if !local_from_camera.orientation.is_finite() || !local_from_camera.position.is_finite() {
        return Err("invalid camera pose".into());
    }

    // Head pose and velocity at capture time: angular speed indicates motion blur
    let (view_location, view_velocity) = spaces
        .view
        .relate(spaces.stage, capture_time)
        .map_err(|e| format!("cannot locate VIEW in STAGE: {e}"))?;
    let head_angular_speed = if view_velocity
        .velocity_flags
        .contains(xr::SpaceVelocityFlags::ANGULAR_VALID)
    {
        crate::from_xr_vec3(view_velocity.angular_velocity).length()
    } else {
        0.0
    };

    let stage_from_view = crate::from_xr_pose(view_location.pose);
    let securemr_stage_from_camera = stage_from_local * local_from_camera;
    let stage_from_camera = if let Some((extrinsics, _)) = extrinsics {
        if !view_location.location_flags.contains(
            xr::SpaceLocationFlags::POSITION_VALID | xr::SpaceLocationFlags::ORIENTATION_VALID,
        ) {
            return Err("VIEW is not located in STAGE".into());
        }

        stage_from_view * extrinsics.view_from_camera(eye)
    } else {
        securemr_stage_from_camera
    };

    if diagnostics.log_poses {
        info!(
            "CAMPOSE_DIAG eye={eye:?} t={} stage_from_view {} | stage_from_local {} | securemr_local_from_cam_cv {} | raw_m {:?}",
            capture_time.as_nanos(),
            pose_text(stage_from_view),
            pose_text(stage_from_local),
            pose_text(local_from_camera),
            m
        );
    }
    if let (true, Some((_, set))) = (diagnostics.log_path_difference, extrinsics) {
        // Nonzero when SecureMR's LOCAL space differs from the app's (or the extrinsics are off)
        let difference = securemr_stage_from_camera.position - stage_from_camera.position;
        let angle = securemr_stage_from_camera
            .orientation
            .angle_between(stage_from_camera.orientation);
        info!(
            "CAMPOSE_DIFF eye={eye:?} securemr - extrinsics(set={}): position ({:.1}, {:.1}, {:.1}) \
            cm (|{:.1}| cm), rotation {:.2} deg",
            set.name(),
            difference.x * 100.0,
            difference.y * 100.0,
            difference.z * 100.0,
            difference.length() * 100.0,
            angle.to_degrees()
        );
    }

    let gray = capture
        .rgb
        .chunks_exact(3)
        .map(|rgb| {
            // BT.601 luma, fixed point
            // # Safety: chunks_exact(3) yields 3-byte slices
            ((77 * u32::from(rgb[0]) + 150 * u32::from(rgb[1]) + 29 * u32::from(rgb[2])) >> 8) as u8
        })
        .collect::<Vec<_>>();

    let side = u16::try_from(capture.size).map_err(|_| "image too large")?;
    let mut jpeg = Vec::new();
    jpeg_encoder::Encoder::new(&mut jpeg, JPEG_QUALITY)
        .encode(&gray, side, side, jpeg_encoder::ColorType::Luma)
        .map_err(|e| format!("JPEG encoding failed: {e}"))?;

    let k = capture.camera_matrix;
    Ok(CameraFrame {
        capture_time: crate::from_xr_time(capture_time),
        eye,
        width: capture.size,
        height: capture.size,
        fx: k[0],
        fy: k[4],
        cx: k[2],
        cy: k[5],
        stage_from_camera,
        head_angular_speed,
        jpeg,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const MODEL: &str = "PICO 4 Ultra Enterprise";

    fn set(set: ExtrinsicsSet) -> CameraExtrinsics {
        find_extrinsics(set, MODEL).unwrap()
    }

    #[test]
    fn switch_values() {
        assert_eq!(ExtrinsicsSet::from_switch(""), ExtrinsicsSet::Pico);
        assert_eq!(ExtrinsicsSet::from_switch("pico"), ExtrinsicsSet::Pico);
        assert_eq!(ExtrinsicsSet::from_switch("PICO"), ExtrinsicsSet::Pico);
        assert_eq!(ExtrinsicsSet::from_switch("garbage"), ExtrinsicsSet::Pico);
        assert_eq!(ExtrinsicsSet::from_switch("fit"), ExtrinsicsSet::Fit);
        assert_eq!(ExtrinsicsSet::from_switch("FIT"), ExtrinsicsSet::Fit);
    }

    #[test]
    fn baseline_and_rotation() {
        for s in [ExtrinsicsSet::Fit, ExtrinsicsSet::Pico] {
            let e = set(s);
            let baseline = (e.right.position - e.left.position).length() * 1000.0;
            assert!((baseline - 64.1).abs() < 0.2, "{s:?} baseline {baseline} mm");
            for pose in [e.left, e.right] {
                assert!((pose.orientation.length() - 1.0).abs() < 1e-4, "{s:?} not unit");
                // Looks forward (-Z), image down is VIEW down
                let m = Mat3::from_quat(pose.orientation.normalize());
                assert!((m.transpose() * m - Mat3::IDENTITY).abs_diff_eq(Mat3::ZERO, 1e-5));
                assert!(m.z_axis.z < -0.99 && m.y_axis.y < -0.99 && m.x_axis.x > 0.99);
            }
        }
    }

    #[test]
    fn pico_minus_fit() {
        let (fit, pico) = (set(ExtrinsicsSet::Fit), set(ExtrinsicsSet::Pico));
        for (f, p) in [(fit.left, pico.left), (fit.right, pico.right)] {
            let d = (p.position - f.position) * 1000.0;
            // VIEW right/up/back, mm: same lateral position, ~40 mm lower, ~34 mm further forward
            assert!(d.x.abs() < 0.5, "right {}", d.x);
            assert!((d.y + 40.0).abs() < 2.0, "up {}", d.y);
            assert!((d.z + 34.0).abs() < 1.0, "back {}", d.z);
            let (fit_roll, fit_pitch, _) = roll_pitch_yaw_deg(f.orientation.normalize());
            let (pico_roll, pico_pitch, _) = roll_pitch_yaw_deg(p.orientation.normalize());
            assert!((fit_roll + 0.79).abs() < 0.05, "fit roll {fit_roll}");
            assert!((pico_roll - 0.78).abs() < 0.05, "pico roll {pico_roll}");
            assert!((fit_pitch - pico_pitch).abs() < 0.05);
        }
    }

    // PICO's set equals A * E of the raw capture library matrices (see CAMERA_EXTRINSICS_PICO)
    #[test]
    fn pico_set_from_raw() {
        let rotation = Mat3::from_cols(
            Vec3::new(0.013_663_54, -0.999_903_8, 0.002_397_913),
            Vec3::new(-0.999_839_35, -0.013_690_405, -0.011_569_357),
            Vec3::new(0.011_601_072, -0.002_239_45, -0.999_930_2),
        );
        let body_to_view = Mat3::from_cols(Vec3::Y, -Vec3::X, Vec3::Z);
        let origin = Vec3::new(0.050_57, 0.018_74, -0.051_94);
        let raw_left = Vec3::new(-0.020_185_1, 0.082_739_83, -0.016_957_89);
        let raw_right = Vec3::new(-0.019_309_116, 0.018_634_83, -0.016_804_157);
        let pico = set(ExtrinsicsSet::Pico);
        let expected = Quat::from_mat3(&(body_to_view * rotation));
        for (pose, raw) in [(pico.left, raw_left), (pico.right, raw_right)] {
            assert!(pose.orientation.normalize().angle_between(expected) < 1e-3);
            let position = body_to_view * raw + origin;
            assert!((pose.position - position).length() < 2e-6, "{position:?}");
        }
    }
}
