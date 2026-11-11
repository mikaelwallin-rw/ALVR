// CAMERA PROBE (TEST ONLY, debug switch screenvr_camera_probe.txt, see debug_switches.rs): asks
// PICO's camera API (XR_PICO_camera_image) once per app start what it reports as the passthrough
// cameras' extrinsics and intrinsics, and logs them (prefix CAMERA_PROBE) next to our
// hand-eye-fitted CAMERA_EXTRINSICS (camera_frames.rs) and the xrLocateViews eye poses. Log only:
// nothing uses the values.
//
// The extension is requested at instance creation only when the switch file exists at start-up
// (lib.rs), so that the instance is unchanged without it. Everything else runs on its own thread,
// never on the render thread; every failure is logged and ends the probe, the app goes on.
//
// The SecureMR camera pass (camera_frames.rs) uses the same physical cameras through another
// extension (XR_PICO_secure_mixed_reality). Opening a camera device here while it runs might
// conflict, so the probe waits for the pass to stop (up to CAMERA_PASS_WAIT) and logs if it did not.
//
// The openxr crate has no bindings for this PICO extension: the types below mirror PICO's openxr.h
// (SecureMR-Samples, external/openxr/include/openxr/openxr.h, section XR_PICO_camera_image).

use alvr_common::{
    Pose,
    glam::{EulerRot, Quat},
    info, warn,
};
use openxr::{self as xr, sys};
use std::{
    ffi::{CStr, c_void},
    f32::consts::PI,
    ptr, thread,
    time::{Duration, Instant},
};

pub const CAMERA_IMAGE_EXTENSION_NAME: &str = "XR_PICO_camera_image";

// XrStructureType values (openxr.h, XR_TYPE_..._PICO = 1010033000..1010033028)
const TYPE_AVAILABLE_CAMERAS_ENUMERATE_INFO: i32 = 1_010_033_000;
const TYPE_CAMERA_PROPERTIES_GET_INFO: i32 = 1_010_033_001;
const TYPE_CAMERA_PROPERTIES: i32 = 1_010_033_002;
const TYPE_CAMERA_PROPERTY_FACING: i32 = 1_010_033_003;
const TYPE_CAMERA_PROPERTY_POSITION: i32 = 1_010_033_004;
const TYPE_CAMERA_PROPERTY_CAMERA_TYPE: i32 = 1_010_033_005;
const TYPE_CAMERA_SUPPORTED_CAPABILITIES_GET_INFO: i32 = 1_010_033_006;
const TYPE_CAMERA_SUPPORTED_CAPABILITIES: i32 = 1_010_033_007;
const TYPE_CAMERA_SUPPORTED_CAPABILITY_IMAGE_RESOLUTION: i32 = 1_010_033_008;
const TYPE_CAMERA_CAPABILITY_IMAGE_RESOLUTION: i32 = 1_010_033_009;
const TYPE_CAMERA_SUPPORTED_CAPABILITY_DATA_TRANSFER_TYPE: i32 = 1_010_033_010;
const TYPE_CAMERA_CAPABILITY_DATA_TRANSFER_TYPE: i32 = 1_010_033_011;
const TYPE_CAMERA_SUPPORTED_CAPABILITY_IMAGE_FORMAT: i32 = 1_010_033_012;
const TYPE_CAMERA_CAPABILITY_IMAGE_FORMAT: i32 = 1_010_033_013;
const TYPE_CAMERA_SUPPORTED_CAPABILITY_CAMERA_MODEL: i32 = 1_010_033_014;
const TYPE_CAMERA_CAPABILITY_CAMERA_MODEL: i32 = 1_010_033_015;
const TYPE_CAMERA_SUPPORTED_CAPABILITY_IMAGE_FPS: i32 = 1_010_033_016;
const TYPE_CAMERA_CAPABILITY_IMAGE_FPS: i32 = 1_010_033_017;
const TYPE_CAMERA_DEVICE_CREATE_INFO: i32 = 1_010_033_018;
const TYPE_CREATE_CAMERA_DEVICE_COMPLETION: i32 = 1_010_033_019;
const TYPE_CAMERA_CAPTURE_SESSION_CREATE_INFO: i32 = 1_010_033_020;
const TYPE_CREATE_CAMERA_CAPTURE_SESSION_COMPLETION: i32 = 1_010_033_021;
const TYPE_CAMERA_INTRINSICS: i32 = 1_010_033_022;
const TYPE_CAMERA_EXTRINSICS: i32 = 1_010_033_023;
const TYPE_CAMERA_CAPTURE_BEGIN_INFO: i32 = 1_010_033_024;
const TYPE_CAMERA_CAPABILITIES: i32 = 1_010_033_028;

// XrCameraPropertyTypePICO
const PROPERTY_FACING: i32 = 1;
const PROPERTY_POSITION: i32 = 2;
const PROPERTY_CAMERA_TYPE: i32 = 3;
// XrCameraPositionPICO
const POSITION_LEFT: i32 = 2;
const POSITION_RIGHT: i32 = 3;
// XrCameraCapabilityTypePICO
const CAPABILITY_IMAGE_RESOLUTION: i32 = 1;
const CAPABILITY_IMAGE_FORMAT: i32 = 2;
const CAPABILITY_DATA_TRANSFER_TYPE: i32 = 3;
const CAPABILITY_CAMERA_MODEL: i32 = 4;
const CAPABILITY_IMAGE_FPS: i32 = 5;

// Lets the session settle (and the debug switch poll run) before the cameras are touched
const START_DELAY: Duration = Duration::from_secs(3);
const CAMERA_PASS_WAIT: Duration = Duration::from_secs(30);
const FUTURE_TIMEOUT: Duration = Duration::from_secs(5);
const FUTURE_POLL_INTERVAL: Duration = Duration::from_millis(10);

type Handle = u64;
type CameraId = u64;

// XrCameraPropertyBaseHeaderPICO, XrCameraCapabilityBaseHeaderPICO,
// XrCameraSupportedCapabilityBaseHeaderPICO (identical layouts)
#[repr(C)]
struct BaseHeader {
    ty: sys::StructureType,
    next: *const c_void,
}

// XrCameraPropertiesPICO, XrCameraCapabilitiesPICO, XrCameraSupportedCapabilitiesPICO
// (identical layouts: type, next, count, array of pointers to base headers)
#[repr(C)]
struct HeaderList {
    ty: sys::StructureType,
    next: *const c_void,
    count: u32,
    items: *mut *mut BaseHeader,
}

#[repr(C)]
struct AvailableCamerasEnumerateInfo {
    ty: sys::StructureType,
    next: *const c_void,
    properties: *const HeaderList,
    capabilities: *const HeaderList,
}

// XrCameraPropertiesGetInfoPICO, XrCameraSupportedCapabilitiesGetInfoPICO,
// XrCameraDeviceCreateInfoPICO (identical layouts: type, next, camera id)
#[repr(C)]
struct CameraIdInfo {
    ty: sys::StructureType,
    next: *const c_void,
    camera_id: CameraId,
}

// XrCameraPropertyFacingPICO, XrCameraPropertyPositionPICO, XrCameraPropertyCameraTypePICO,
// XrCameraCapability{DataTransferType,ImageFormat,CameraModel,ImageFps}PICO (type, next, enum)
#[repr(C)]
struct EnumValue {
    ty: sys::StructureType,
    next: *const c_void,
    value: i32,
}

// XrCameraSupportedCapability{DataTransferType,ImageFormat,CameraModel,ImageFps}PICO
#[repr(C)]
struct SupportedEnumList {
    ty: sys::StructureType,
    next: *const c_void,
    capacity_input: u32,
    count_output: u32,
    values: *mut i32,
}

#[repr(C)]
struct SupportedResolutions {
    ty: sys::StructureType,
    next: *const c_void,
    capacity_input: u32,
    count_output: u32,
    resolutions: *mut sys::Extent2Di,
}

#[repr(C)]
struct CapabilityResolution {
    ty: sys::StructureType,
    next: *const c_void,
    resolution: sys::Extent2Di,
}

#[repr(C)]
struct CreateHandleCompletion {
    ty: sys::StructureType,
    next: *mut c_void,
    future_result: sys::Result,
    handle: Handle,
}

#[repr(C)]
struct CaptureSessionCreateInfo {
    ty: sys::StructureType,
    next: *const c_void,
    camera: Handle,
    config_count: u32,
    configs: *const *const BaseHeader,
}

#[repr(C)]
struct CameraIntrinsics {
    ty: sys::StructureType,
    next: *const c_void,
    focal_length: sys::Vector2f,
    principal_point: sys::Vector2f,
    fov: sys::Vector2f,
}

#[repr(C)]
struct CameraExtrinsics {
    ty: sys::StructureType,
    next: *const c_void,
    pose: sys::Posef,
}

type EnumerateCamerasFn = unsafe extern "system" fn(
    sys::Instance,
    *const AvailableCamerasEnumerateInfo,
    u32,
    *mut u32,
    *mut CameraId,
) -> sys::Result;
type EnumerateTypesFn =
    unsafe extern "system" fn(sys::Instance, CameraId, u32, *mut u32, *mut i32) -> sys::Result;
type GetHeaderListFn =
    unsafe extern "system" fn(sys::Instance, *const CameraIdInfo, *mut HeaderList) -> sys::Result;
type CreateDeviceAsyncFn =
    unsafe extern "system" fn(sys::Instance, *const CameraIdInfo, *mut sys::FutureEXT) -> sys::Result;
type CreateDeviceCompleteFn = unsafe extern "system" fn(
    sys::Instance,
    sys::FutureEXT,
    *mut CreateHandleCompletion,
) -> sys::Result;
type CreateCaptureSessionAsyncFn = unsafe extern "system" fn(
    sys::Session,
    *const CaptureSessionCreateInfo,
    *mut sys::FutureEXT,
) -> sys::Result;
type CreateCaptureSessionCompleteFn = unsafe extern "system" fn(
    sys::Session,
    sys::FutureEXT,
    *mut CreateHandleCompletion,
) -> sys::Result;
type HandleFn = unsafe extern "system" fn(Handle) -> sys::Result;
type GetIntrinsicsFn = unsafe extern "system" fn(Handle, *mut CameraIntrinsics) -> sys::Result;
type GetExtrinsicsFn = unsafe extern "system" fn(Handle, *mut CameraExtrinsics) -> sys::Result;
type BeginCaptureFn = unsafe extern "system" fn(Handle, *mut BaseHeader) -> sys::Result;

struct Functions {
    enumerate_cameras: EnumerateCamerasFn,
    get_properties: GetHeaderListFn,
    create_device_async: CreateDeviceAsyncFn,
    create_device_complete: CreateDeviceCompleteFn,
    destroy_device: HandleFn,
    create_session_async: CreateCaptureSessionAsyncFn,
    create_session_complete: CreateCaptureSessionCompleteFn,
    destroy_session: HandleFn,
    get_intrinsics: GetIntrinsicsFn,
    get_extrinsics: GetExtrinsicsFn,
    // Optional: only used for logging or as a fallback
    enumerate_property_types: Option<EnumerateTypesFn>,
    enumerate_capability_types: Option<EnumerateTypesFn>,
    get_supported_capabilities: Option<GetHeaderListFn>,
    begin_capture: Option<BeginCaptureFn>,
    end_capture: Option<HandleFn>,
}

fn load<G, T>(session: &xr::Session<G>, name: &str) -> Option<T> {
    match super::get_instance_proc(session, name) {
        Ok(function) => Some(function),
        Err(e) => {
            warn!(
                "CAMERA_PROBE xrGetInstanceProcAddr({name}): {}",
                result_text(e)
            );
            None
        }
    }
}

impl Functions {
    fn load_all<G>(session: &xr::Session<G>) -> Option<Self> {
        Some(Self {
            enumerate_cameras: load(session, "xrEnumerateAvailableCamerasPICO")?,
            get_properties: load(session, "xrGetCameraPropertiesPICO")?,
            create_device_async: load(session, "xrCreateCameraDeviceAsyncPICO")?,
            create_device_complete: load(session, "xrCreateCameraDeviceCompletePICO")?,
            destroy_device: load(session, "xrDestroyCameraDevicePICO")?,
            create_session_async: load(session, "xrCreateCameraCaptureSessionAsyncPICO")?,
            create_session_complete: load(session, "xrCreateCameraCaptureSessionCompletePICO")?,
            destroy_session: load(session, "xrDestroyCameraCaptureSessionPICO")?,
            get_intrinsics: load(session, "xrGetCameraIntrinsicsPICO")?,
            get_extrinsics: load(session, "xrGetCameraExtrinsicsPICO")?,
            enumerate_property_types: load(session, "xrEnumerateCameraPropertyTypesPICO"),
            enumerate_capability_types: load(session, "xrEnumerateCameraCapabilityTypesPICO"),
            get_supported_capabilities: load(session, "xrGetCameraSupportedCapabilitiesPICO"),
            begin_capture: load(session, "xrBeginCameraCapturePICO"),
            end_capture: load(session, "xrEndCameraCapturePICO"),
        })
    }
}

/// XR_PICO_camera_image at instance creation (lib.rs)
#[derive(Clone, Copy, Default, Debug)]
pub struct CameraImageExtensionState {
    /// screenvr_camera_probe.txt was set at start-up: the extension was requested
    pub requested: bool,
    /// Spec version, if the runtime lists the extension
    pub offered_version: Option<u32>,
    /// Enabled on the instance
    pub enabled: bool,
    /// Instance creation with the extension failed (it was created again without it)
    pub instance_error: Option<sys::Result>,
}

pub fn is_camera_image_extension(nul_terminated_name: &[u8]) -> bool {
    CStr::from_bytes_with_nul(nul_terminated_name)
        .is_ok_and(|name| name.to_bytes() == CAMERA_IMAGE_EXTENSION_NAME.as_bytes())
}

/// The runtime's spec version of XR_PICO_camera_image, None if it is not listed
pub fn camera_image_extension_version(entry: &xr::Entry) -> Option<u32> {
    let enumerate = entry.fp().enumerate_instance_extension_properties;

    let mut count = 0_u32;
    let result = unsafe { enumerate(ptr::null(), 0, &raw mut count, ptr::null_mut()) };
    if result.into_raw() < 0 {
        return None;
    }
    let mut properties = (0..count)
        .map(|_| sys::ExtensionProperties {
            ty: sys::ExtensionProperties::TYPE,
            next: ptr::null_mut(),
            extension_name: [0; sys::MAX_EXTENSION_NAME_SIZE],
            extension_version: 0,
        })
        .collect::<Vec<_>>();
    let result = unsafe { enumerate(ptr::null(), count, &raw mut count, properties.as_mut_ptr()) };
    if result.into_raw() < 0 {
        return None;
    }
    properties.truncate(count as usize);

    properties
        .iter()
        .find(|p| {
            // # Safety: zero-initialized array the runtime fills with a nul-terminated name
            let name = unsafe { CStr::from_ptr(p.extension_name.as_ptr()) };
            name.to_bytes() == CAMERA_IMAGE_EXTENSION_NAME.as_bytes()
        })
        .map(|p| p.extension_version)
}

/// After session creation: on Pico, log lines from before the session do not reach logcat
pub fn log_camera_image_extension(state: &CameraImageExtensionState) {
    match state.offered_version {
        Some(version) => info!(
            "CAMERA_PROBE runtime offers {CAMERA_IMAGE_EXTENSION_NAME}, spec version {version} \
            (header: 1)"
        ),
        None => warn!("CAMERA_PROBE runtime does NOT offer {CAMERA_IMAGE_EXTENSION_NAME}"),
    }
    if let Some(e) = state.instance_error {
        warn!(
            "CAMERA_PROBE xrCreateInstance with {CAMERA_IMAGE_EXTENSION_NAME} failed: {}; the \
            instance was created without it",
            result_text(e)
        );
    }
    info!(
        "CAMERA_PROBE {CAMERA_IMAGE_EXTENSION_NAME} enabled on the instance: {}",
        state.enabled
    );
}

/// One probe run on its own thread (never blocks the caller)
pub fn spawn_camera_probe(session: xr::Session<xr::OpenGlEs>, state: CameraImageExtensionState) {
    let spawned = thread::Builder::new()
        .name("camera_probe".into())
        .spawn(move || run(&session, state));
    if let Err(e) = spawned {
        warn!("CAMERA_PROBE cannot start the probe thread: {e}");
    }
}

fn result_text(result: sys::Result) -> String {
    format!("{result:?} ({})", result.into_raw())
}

fn succeeded(result: sys::Result) -> bool {
    result.into_raw() >= 0
}

// Logs every call's result; true on success
fn logged(call: &str, result: sys::Result) -> bool {
    if succeeded(result) {
        info!("CAMERA_PROBE {call}: {}", result_text(result));
    } else {
        warn!("CAMERA_PROBE {call}: {}", result_text(result));
    }

    succeeded(result)
}

fn st(raw: i32) -> sys::StructureType {
    sys::StructureType::from_raw(raw)
}

fn enum_name(property: i32, value: i32) -> &'static str {
    match (property, value) {
        (PROPERTY_FACING, 1) => "WORLD",
        (PROPERTY_POSITION, 1) => "UNSPECIFIED",
        (PROPERTY_POSITION, POSITION_LEFT) => "LEFT",
        (PROPERTY_POSITION, POSITION_RIGHT) => "RIGHT",
        (PROPERTY_CAMERA_TYPE, 1) => "PASSTHROUGH_COLOR",
        _ => "?",
    }
}

fn capability_name(capability: i32) -> &'static str {
    match capability {
        CAPABILITY_IMAGE_RESOLUTION => "IMAGE_RESOLUTION",
        CAPABILITY_IMAGE_FORMAT => "IMAGE_FORMAT (1 = RGBA_8888)",
        CAPABILITY_DATA_TRANSFER_TYPE => "DATA_TRANSFER_TYPE (1 = RAW_BUFFER)",
        CAPABILITY_CAMERA_MODEL => "CAMERA_MODEL (1 = PINHOLE)",
        CAPABILITY_IMAGE_FPS => "IMAGE_FPS (1 = 30, 2 = 60)",
        _ => "?",
    }
}

fn pose_text(pose: Pose) -> String {
    let p = pose.position * 1000.0;
    let q = pose.orientation;
    // Intrinsic Y-X-Z: yaw about +Y (up), pitch about +X (right), roll about +Z (back)
    let (yaw, pitch, roll) = q.to_euler(EulerRot::YXZ);
    format!(
        "pos=({:.2}, {:.2}, {:.2}) mm quat_xyzw=({:.6}, {:.6}, {:.6}, {:.6}) \
        roll/pitch/yaw=({:.2}, {:.2}, {:.2}) deg",
        p.x,
        p.y,
        p.z,
        q.x,
        q.y,
        q.z,
        q.w,
        roll.to_degrees(),
        pitch.to_degrees(),
        yaw.to_degrees()
    )
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Side {
    Left,
    Right,
}

// What PICO's numbers are compared with, taken once before the cameras are opened
struct Reference {
    // VIEW from camera, OpenCV camera axes (camera_frames.rs CAMERA_EXTRINSICS)
    ours: Option<(Pose, Pose)>,
    // VIEW from eye (xrLocateViews in VIEW space)
    eyes: Option<(Pose, Pose)>,
}

impl Reference {
    fn capture(session: &xr::Session<xr::OpenGlEs>) -> Self {
        let (product, ours) = crate::camera_frames::reference_extrinsics();
        match ours {
            Some((left, right)) => {
                info!(
                    "CAMERA_PROBE ours ({product:?}, VIEW from camera, OpenCV axes +X right +Y \
                    down +Z forward) LEFT {}",
                    pose_text(left)
                );
                info!("CAMERA_PROBE ours RIGHT {}", pose_text(right));
            }
            None => warn!("CAMERA_PROBE ours: no CAMERA_EXTRINSICS entry for {product:?}"),
        }

        let eyes = Self::locate_eyes(session);
        if let Some((left, right)) = eyes {
            info!(
                "CAMERA_PROBE eyes (xrLocateViews, VIEW from eye) LEFT {}",
                pose_text(left)
            );
            info!("CAMERA_PROBE eyes RIGHT {}", pose_text(right));
            info!(
                "CAMERA_PROBE eyes IPD {:.2} mm",
                (right.position - left.position).length() * 1000.0
            );
        }

        Self { ours, eyes }
    }

    fn locate_eyes(session: &xr::Session<xr::OpenGlEs>) -> Option<(Pose, Pose)> {
        let view_space = session
            .create_reference_space(xr::ReferenceSpaceType::VIEW, xr::Posef::IDENTITY)
            .map_err(|e| warn!("CAMERA_PROBE cannot create the VIEW space: {}", result_text(e)))
            .ok()?;
        let time = session
            .instance()
            .now()
            .map_err(|e| warn!("CAMERA_PROBE cannot get the time: {}", result_text(e)))
            .ok()?;
        let (flags, views) = session
            .locate_views(xr::ViewConfigurationType::PRIMARY_STEREO, time, &view_space)
            .map_err(|e| warn!("CAMERA_PROBE xrLocateViews: {}", result_text(e)))
            .ok()?;
        if !flags.contains(xr::ViewStateFlags::POSITION_VALID | xr::ViewStateFlags::ORIENTATION_VALID)
        {
            warn!("CAMERA_PROBE xrLocateViews: views not valid ({flags:?})");
            return None;
        }
        if views.len() < 2 {
            warn!("CAMERA_PROBE xrLocateViews: {} views", views.len());
            return None;
        }

        Some((
            crate::from_xr_pose(views[0].pose),
            crate::from_xr_pose(views[1].pose),
        ))
    }

    fn log_comparison(&self, side: Side, pico: Pose) {
        let pick = |(left, right): (Pose, Pose)| if side == Side::Left { left } else { right };
        let pico = Pose {
            orientation: pico.orientation.normalize(),
            position: pico.position,
        };
        let ours = self.ours.map(pick);
        let eye = self.eyes.map(pick);

        // PICO's frame is undocumented: both the pose as given and its inverse
        for (label, candidate) in [("as-is", pico), ("inverted", pico.inverse())] {
            info!("CAMERA_PROBE {side:?} PICO {label}: {}", pose_text(candidate));
            if let Some(ours) = ours {
                let d = (candidate.position - ours.position) * 1000.0;
                // Ours with OpenXR camera axes (+Y up, -Z forward), as PICO's poses probably are
                let ours_openxr = ours.orientation * Quat::from_rotation_x(PI);
                info!(
                    "CAMERA_PROBE {side:?} PICO {label} - ours: d=({:.2}, {:.2}, {:.2}) mm |{:.2}| \
                    mm; rotation to ours {:.2} deg (ours in OpenXR camera axes), {:.2} deg (ours in \
                    OpenCV axes)",
                    d.x,
                    d.y,
                    d.z,
                    d.length(),
                    candidate.orientation.angle_between(ours_openxr).to_degrees(),
                    candidate.orientation.angle_between(ours.orientation).to_degrees(),
                );
            }
            if let Some(eye) = eye {
                let d = (candidate.position - eye.position) * 1000.0;
                info!(
                    "CAMERA_PROBE {side:?} PICO {label} - eye: d=({:.2}, {:.2}, {:.2}) mm |{:.2}| mm",
                    d.x,
                    d.y,
                    d.z,
                    d.length()
                );
            }
        }
        if let (Some(ours), Some(eye)) = (ours, eye) {
            let d = (ours.position - eye.position) * 1000.0;
            info!(
                "CAMERA_PROBE {side:?} ours - eye: d=({:.2}, {:.2}, {:.2}) mm",
                d.x, d.y, d.z
            );
        }
    }
}

// Waits for a future; false (logged, future cancelled) on error or timeout
fn wait_future(instance: &xr::Instance, future: sys::FutureEXT, what: &str) -> bool {
    let start = Instant::now();
    loop {
        match super::check_future(instance, future) {
            Ok(true) => {
                info!(
                    "CAMERA_PROBE {what}: future ready after {} ms",
                    start.elapsed().as_millis()
                );
                return true;
            }
            Ok(false) => (),
            Err(e) => {
                warn!("CAMERA_PROBE {what}: xrPollFutureEXT: {}", result_text(e));
                return false;
            }
        }
        if start.elapsed() > FUTURE_TIMEOUT {
            warn!("CAMERA_PROBE {what}: future not ready after {FUTURE_TIMEOUT:?}, cancelling");
            if let Some(future_ext) = instance.exts().ext_future {
                let cancel_info = sys::FutureCancelInfoEXT {
                    ty: xr::StructureType::FUTURE_CANCEL_INFO_EXT,
                    next: ptr::null(),
                    future,
                };
                let result =
                    unsafe { (future_ext.cancel_future)(instance.as_raw(), &raw const cancel_info) };
                logged(&format!("{what}: xrCancelFutureEXT"), result);
            }
            return false;
        }
        thread::sleep(FUTURE_POLL_INTERVAL);
    }
}

fn enumerate_cameras(f: &Functions, instance: &xr::Instance) -> Vec<CameraId> {
    let empty_properties = HeaderList {
        ty: st(TYPE_CAMERA_PROPERTIES),
        next: ptr::null(),
        count: 0,
        items: ptr::null_mut(),
    };
    let empty_capabilities = HeaderList {
        ty: st(TYPE_CAMERA_CAPABILITIES),
        next: ptr::null(),
        count: 0,
        items: ptr::null_mut(),
    };
    // Whether the filters may be null is undocumented: null first, then empty filters
    let attempts = [
        (
            "no filter (null)",
            AvailableCamerasEnumerateInfo {
                ty: st(TYPE_AVAILABLE_CAMERAS_ENUMERATE_INFO),
                next: ptr::null(),
                properties: ptr::null(),
                capabilities: ptr::null(),
            },
        ),
        (
            "empty filters",
            AvailableCamerasEnumerateInfo {
                ty: st(TYPE_AVAILABLE_CAMERAS_ENUMERATE_INFO),
                next: ptr::null(),
                properties: &raw const empty_properties,
                capabilities: &raw const empty_capabilities,
            },
        ),
    ];

    for (label, info) in &attempts {
        let mut count = 0_u32;
        let result = unsafe {
            (f.enumerate_cameras)(
                instance.as_raw(),
                info,
                0,
                &raw mut count,
                ptr::null_mut(),
            )
        };
        if !logged(
            &format!("xrEnumerateAvailableCamerasPICO {label}, count -> {count}"),
            result,
        ) {
            continue;
        }

        let mut ids = vec![0 as CameraId; count as usize];
        let result = unsafe {
            (f.enumerate_cameras)(
                instance.as_raw(),
                info,
                count,
                &raw mut count,
                ids.as_mut_ptr(),
            )
        };
        if !logged(&format!("xrEnumerateAvailableCamerasPICO {label}, ids"), result) {
            continue;
        }
        ids.truncate(count as usize);
        info!("CAMERA_PROBE cameras: {ids:?}");

        return ids;
    }

    Vec::new()
}

// Two-call idiom for xrEnumerateCamera{Property,Capability}TypesPICO
fn enumerate_types(
    function: EnumerateTypesFn,
    name: &str,
    instance: &xr::Instance,
    camera: CameraId,
) -> Option<Vec<i32>> {
    let mut count = 0_u32;
    let result =
        unsafe { function(instance.as_raw(), camera, 0, &raw mut count, ptr::null_mut()) };
    if !logged(&format!("camera {camera}: {name} count -> {count}"), result) {
        return None;
    }
    let mut types = vec![0_i32; count as usize];
    let result =
        unsafe { function(instance.as_raw(), camera, count, &raw mut count, types.as_mut_ptr()) };
    if !logged(&format!("camera {camera}: {name}"), result) {
        return None;
    }
    types.truncate(count as usize);

    Some(types)
}

#[derive(Default, Debug)]
struct CameraProperties {
    facing: Option<i32>,
    position: Option<i32>,
    camera_type: Option<i32>,
}

fn camera_properties(f: &Functions, instance: &xr::Instance, camera: CameraId) -> CameraProperties {
    let property_types = f
        .enumerate_property_types
        .and_then(|function| {
            enumerate_types(
                function,
                "xrEnumerateCameraPropertyTypesPICO",
                instance,
                camera,
            )
        })
        .unwrap_or_else(|| vec![PROPERTY_FACING, PROPERTY_POSITION, PROPERTY_CAMERA_TYPE]);
    info!("CAMERA_PROBE camera {camera}: property types {property_types:?} (1 facing, 2 position, 3 type)");

    let mut values = property_types
        .iter()
        .filter_map(|&property| {
            let ty = match property {
                PROPERTY_FACING => TYPE_CAMERA_PROPERTY_FACING,
                PROPERTY_POSITION => TYPE_CAMERA_PROPERTY_POSITION,
                PROPERTY_CAMERA_TYPE => TYPE_CAMERA_PROPERTY_CAMERA_TYPE,
                _ => return None,
            };
            Some((
                property,
                EnumValue {
                    ty: st(ty),
                    next: ptr::null(),
                    value: 0,
                },
            ))
        })
        .collect::<Vec<_>>();
    let mut pointers = values
        .iter_mut()
        .map(|(_, value)| ptr::from_mut(value).cast::<BaseHeader>())
        .collect::<Vec<_>>();
    let get_info = CameraIdInfo {
        ty: st(TYPE_CAMERA_PROPERTIES_GET_INFO),
        next: ptr::null(),
        camera_id: camera,
    };
    let mut list = HeaderList {
        ty: st(TYPE_CAMERA_PROPERTIES),
        next: ptr::null(),
        count: pointers.len() as u32,
        items: pointers.as_mut_ptr(),
    };
    let result =
        unsafe { (f.get_properties)(instance.as_raw(), &raw const get_info, &raw mut list) };
    let mut properties = CameraProperties::default();
    if logged(&format!("camera {camera}: xrGetCameraPropertiesPICO"), result) {
        for (property, value) in &values {
            let value = value.value;
            info!(
                "CAMERA_PROBE camera {camera}: property {property} = {value} ({})",
                enum_name(*property, value)
            );
            match *property {
                PROPERTY_FACING => properties.facing = Some(value),
                PROPERTY_POSITION => properties.position = Some(value),
                PROPERTY_CAMERA_TYPE => properties.camera_type = Some(value),
                _ => (),
            }
        }
    }

    properties
}

#[derive(Default)]
struct SupportedCapabilities {
    resolutions: Vec<sys::Extent2Di>,
    // (capability type, supported values)
    lists: Vec<(i32, Vec<i32>)>,
}

fn supported_capabilities(
    f: &Functions,
    instance: &xr::Instance,
    camera: CameraId,
) -> SupportedCapabilities {
    let mut supported = SupportedCapabilities::default();
    let (Some(enumerate_capability_types), Some(get_supported)) =
        (f.enumerate_capability_types, f.get_supported_capabilities)
    else {
        return supported;
    };
    let Some(capability_types) = enumerate_types(
        enumerate_capability_types,
        "xrEnumerateCameraCapabilityTypesPICO",
        instance,
        camera,
    ) else {
        return supported;
    };
    info!("CAMERA_PROBE camera {camera}: capability types {capability_types:?}");

    let mut resolutions = capability_types
        .contains(&CAPABILITY_IMAGE_RESOLUTION)
        .then(|| SupportedResolutions {
            ty: st(TYPE_CAMERA_SUPPORTED_CAPABILITY_IMAGE_RESOLUTION),
            next: ptr::null(),
            capacity_input: 0,
            count_output: 0,
            resolutions: ptr::null_mut(),
        });
    let mut lists = capability_types
        .iter()
        .filter_map(|&capability| {
            let ty = match capability {
                CAPABILITY_IMAGE_FORMAT => TYPE_CAMERA_SUPPORTED_CAPABILITY_IMAGE_FORMAT,
                CAPABILITY_DATA_TRANSFER_TYPE => TYPE_CAMERA_SUPPORTED_CAPABILITY_DATA_TRANSFER_TYPE,
                CAPABILITY_CAMERA_MODEL => TYPE_CAMERA_SUPPORTED_CAPABILITY_CAMERA_MODEL,
                CAPABILITY_IMAGE_FPS => TYPE_CAMERA_SUPPORTED_CAPABILITY_IMAGE_FPS,
                _ => return None,
            };
            Some((
                capability,
                SupportedEnumList {
                    ty: st(ty),
                    next: ptr::null(),
                    capacity_input: 0,
                    count_output: 0,
                    values: ptr::null_mut(),
                },
            ))
        })
        .collect::<Vec<_>>();

    let get_info = CameraIdInfo {
        ty: st(TYPE_CAMERA_SUPPORTED_CAPABILITIES_GET_INFO),
        next: ptr::null(),
        camera_id: camera,
    };
    // Two-call idiom: counts first, then the values
    let mut resolution_buffer = Vec::<sys::Extent2Di>::new();
    let mut list_buffers = Vec::<Vec<i32>>::new();
    for pass in ["counts", "values"] {
        if pass == "values" {
            if let Some(resolutions) = &mut resolutions {
                resolution_buffer =
                    vec![sys::Extent2Di::default(); resolutions.count_output as usize];
                resolutions.capacity_input = resolutions.count_output;
                resolutions.resolutions = resolution_buffer.as_mut_ptr();
            }
            list_buffers = lists
                .iter()
                .map(|(_, list)| vec![0_i32; list.count_output as usize])
                .collect();
            for ((_, list), buffer) in lists.iter_mut().zip(&mut list_buffers) {
                list.capacity_input = list.count_output;
                list.values = buffer.as_mut_ptr();
            }
        }

        let mut pointers = Vec::<*mut BaseHeader>::new();
        if let Some(resolutions) = &mut resolutions {
            pointers.push(ptr::from_mut(resolutions).cast());
        }
        for (_, list) in &mut lists {
            pointers.push(ptr::from_mut(list).cast());
        }
        let mut header_list = HeaderList {
            ty: st(TYPE_CAMERA_SUPPORTED_CAPABILITIES),
            next: ptr::null(),
            count: pointers.len() as u32,
            items: pointers.as_mut_ptr(),
        };
        let result =
            unsafe { get_supported(instance.as_raw(), &raw const get_info, &raw mut header_list) };
        if !logged(
            &format!("camera {camera}: xrGetCameraSupportedCapabilitiesPICO ({pass})"),
            result,
        ) {
            return supported;
        }
    }

    if let Some(resolutions) = &resolutions {
        resolution_buffer.truncate(resolutions.count_output as usize);
        supported.resolutions = resolution_buffer;
        let text = supported
            .resolutions
            .iter()
            .map(|r| format!("{}x{}", r.width, r.height))
            .collect::<Vec<_>>();
        info!("CAMERA_PROBE camera {camera}: supported resolutions {text:?}");
    }
    for ((capability, list), mut buffer) in lists.iter().zip(list_buffers) {
        buffer.truncate(list.count_output as usize);
        info!(
            "CAMERA_PROBE camera {camera}: supported {} {buffer:?}",
            capability_name(*capability)
        );
        supported.lists.push((*capability, buffer));
    }

    supported
}

// Async create + future + complete for a camera device or a capture session
fn complete_handle(
    instance: &xr::Instance,
    what: &str,
    async_result: sys::Result,
    future: sys::FutureEXT,
    complete: impl FnOnce(*mut CreateHandleCompletion) -> sys::Result,
    completion_type: i32,
    destroy: HandleFn,
) -> Option<Handle> {
    if !logged(&format!("{what}: async call"), async_result) {
        return None;
    }
    if !wait_future(instance, future, what) {
        return None;
    }
    let mut completion = CreateHandleCompletion {
        ty: st(completion_type),
        next: ptr::null_mut(),
        future_result: sys::Result::SUCCESS,
        handle: 0,
    };
    let result = complete(&raw mut completion);
    let called = logged(&format!("{what}: complete call"), result);
    let completed = called
        && logged(
            &format!("{what}: futureResult (handle {:#x})", completion.handle),
            completion.future_result,
        );
    if completed && completion.handle != 0 {
        Some(completion.handle)
    } else {
        if completion.handle != 0 {
            let result = unsafe { destroy(completion.handle) };
            logged(&format!("{what}: destroy after failure"), result);
        }
        None
    }
}

fn create_device(f: &Functions, instance: &xr::Instance, camera: CameraId) -> Option<Handle> {
    let create_info = CameraIdInfo {
        ty: st(TYPE_CAMERA_DEVICE_CREATE_INFO),
        next: ptr::null(),
        camera_id: camera,
    };
    let mut future = sys::FutureEXT::from_raw(0);
    let async_result = unsafe {
        (f.create_device_async)(instance.as_raw(), &raw const create_info, &raw mut future)
    };

    complete_handle(
        instance,
        &format!("camera {camera}: xrCreateCameraDevice{{Async,Complete}}PICO"),
        async_result,
        future,
        |completion| unsafe { (f.create_device_complete)(instance.as_raw(), future, completion) },
        TYPE_CREATE_CAMERA_DEVICE_COMPLETION,
        f.destroy_device,
    )
}

fn create_capture_session(
    f: &Functions,
    session: &xr::Session<xr::OpenGlEs>,
    camera: CameraId,
    device: Handle,
    configs: &[*const BaseHeader],
    label: &str,
) -> Option<Handle> {
    let create_info = CaptureSessionCreateInfo {
        ty: st(TYPE_CAMERA_CAPTURE_SESSION_CREATE_INFO),
        next: ptr::null(),
        camera: device,
        config_count: configs.len() as u32,
        configs: if configs.is_empty() {
            ptr::null()
        } else {
            configs.as_ptr()
        },
    };
    let mut future = sys::FutureEXT::from_raw(0);
    let async_result = unsafe {
        (f.create_session_async)(session.as_raw(), &raw const create_info, &raw mut future)
    };

    complete_handle(
        session.instance(),
        &format!("camera {camera}: xrCreateCameraCaptureSession{{Async,Complete}}PICO ({label})"),
        async_result,
        future,
        |completion| unsafe { (f.create_session_complete)(session.as_raw(), future, completion) },
        TYPE_CREATE_CAMERA_CAPTURE_SESSION_COMPLETION,
        f.destroy_session,
    )
}

// Capture session with no configuration first; if refused, the cheapest supported one
fn create_capture_session_cheapest(
    f: &Functions,
    session: &xr::Session<xr::OpenGlEs>,
    camera: CameraId,
    device: Handle,
    supported: &SupportedCapabilities,
) -> Option<Handle> {
    if let Some(capture_session) =
        create_capture_session(f, session, camera, device, &[], "no configs")
    {
        return Some(capture_session);
    }

    let resolution = supported
        .resolutions
        .iter()
        .min_by_key(|r| i64::from(r.width) * i64::from(r.height))
        .map(|&resolution| CapabilityResolution {
            ty: st(TYPE_CAMERA_CAPABILITY_IMAGE_RESOLUTION),
            next: ptr::null(),
            resolution,
        });
    let mut values = supported
        .lists
        .iter()
        .filter_map(|(capability, list)| {
            // Lowest value: 30 fps; otherwise the only value the header defines
            let value = *list.iter().min()?;
            let ty = match *capability {
                CAPABILITY_IMAGE_FORMAT => TYPE_CAMERA_CAPABILITY_IMAGE_FORMAT,
                CAPABILITY_DATA_TRANSFER_TYPE => TYPE_CAMERA_CAPABILITY_DATA_TRANSFER_TYPE,
                CAPABILITY_CAMERA_MODEL => TYPE_CAMERA_CAPABILITY_CAMERA_MODEL,
                CAPABILITY_IMAGE_FPS => TYPE_CAMERA_CAPABILITY_IMAGE_FPS,
                _ => return None,
            };
            Some(EnumValue {
                ty: st(ty),
                next: ptr::null(),
                value,
            })
        })
        .collect::<Vec<_>>();
    if resolution.is_none() && values.is_empty() {
        warn!("CAMERA_PROBE camera {camera}: no supported capabilities to configure a session with");
        return None;
    }

    let mut configs = Vec::<*const BaseHeader>::new();
    if let Some(resolution) = &resolution {
        info!(
            "CAMERA_PROBE camera {camera}: session config resolution {}x{}",
            resolution.resolution.width, resolution.resolution.height
        );
        configs.push(ptr::from_ref(resolution).cast());
    }
    for value in &mut values {
        info!(
            "CAMERA_PROBE camera {camera}: session config type {} value {}",
            value.ty.into_raw(),
            value.value
        );
        configs.push(ptr::from_mut(value).cast_const().cast());
    }

    create_capture_session(f, session, camera, device, &configs, "cheapest config")
}

// Extrinsics and intrinsics of an existing capture session
fn read_calibration(
    f: &Functions,
    camera: CameraId,
    capture_session: Handle,
    label: &str,
) -> (Option<Pose>, bool) {
    let mut extrinsics = CameraExtrinsics {
        ty: st(TYPE_CAMERA_EXTRINSICS),
        next: ptr::null(),
        pose: sys::Posef::default(),
    };
    let result = unsafe { (f.get_extrinsics)(capture_session, &raw mut extrinsics) };
    let pose = logged(
        &format!("camera {camera}: xrGetCameraExtrinsicsPICO ({label})"),
        result,
    )
    .then(|| {
        let p = extrinsics.pose.position;
        let q = extrinsics.pose.orientation;
        info!(
            "CAMERA_PROBE camera {camera}: raw extrinsics pose position=({}, {}, {}) m \
            orientation_xyzw=({}, {}, {}, {})",
            p.x, p.y, p.z, q.x, q.y, q.z, q.w
        );
        crate::from_xr_pose(extrinsics.pose)
    });

    let mut intrinsics = CameraIntrinsics {
        ty: st(TYPE_CAMERA_INTRINSICS),
        next: ptr::null(),
        focal_length: sys::Vector2f::default(),
        principal_point: sys::Vector2f::default(),
        fov: sys::Vector2f::default(),
    };
    let result = unsafe { (f.get_intrinsics)(capture_session, &raw mut intrinsics) };
    let intrinsics_ok = logged(
        &format!("camera {camera}: xrGetCameraIntrinsicsPICO ({label})"),
        result,
    );
    if intrinsics_ok {
        let (fl, pp, fov) = (
            intrinsics.focal_length,
            intrinsics.principal_point,
            intrinsics.fov,
        );
        info!(
            "CAMERA_PROBE camera {camera}: intrinsics focal=({}, {}) principal=({}, {}) \
            fov=({}, {}) (fov as degrees if radians: ({:.2}, {:.2}))",
            fl.x,
            fl.y,
            pp.x,
            pp.y,
            fov.x,
            fov.y,
            fov.x.to_degrees(),
            fov.y.to_degrees()
        );
    }

    (pose, intrinsics_ok)
}

fn probe_camera(
    f: &Functions,
    session: &xr::Session<xr::OpenGlEs>,
    camera: CameraId,
    properties: &CameraProperties,
    reference: &Reference,
) {
    let instance = session.instance();
    let supported = supported_capabilities(f, instance, camera);

    let Some(device) = create_device(f, instance, camera) else {
        return;
    };
    if let Some(capture_session) =
        create_capture_session_cheapest(f, session, camera, device, &supported)
    {
        let (mut pose, mut intrinsics_ok) =
            read_calibration(f, camera, capture_session, "session, not capturing");

        // Fallback: the getters may need a running capture
        if pose.is_none()
            && !intrinsics_ok
            && let (Some(begin), Some(end)) = (f.begin_capture, f.end_capture)
        {
            let mut begin_info = BaseHeader {
                ty: st(TYPE_CAMERA_CAPTURE_BEGIN_INFO),
                next: ptr::null(),
            };
            let result = unsafe { begin(capture_session, &raw mut begin_info) };
            if logged(&format!("camera {camera}: xrBeginCameraCapturePICO"), result) {
                (pose, intrinsics_ok) = read_calibration(f, camera, capture_session, "capturing");
                let result = unsafe { end(capture_session) };
                logged(&format!("camera {camera}: xrEndCameraCapturePICO"), result);
            }
        }
        let _ = intrinsics_ok;

        if let Some(pose) = pose {
            match properties.position {
                Some(POSITION_LEFT) => reference.log_comparison(Side::Left, pose),
                Some(POSITION_RIGHT) => reference.log_comparison(Side::Right, pose),
                position => {
                    info!(
                        "CAMERA_PROBE camera {camera}: position {position:?}, no comparison; \
                        PICO as-is: {}",
                        pose_text(pose)
                    );
                }
            }
        }

        let result = unsafe { (f.destroy_session)(capture_session) };
        logged(
            &format!("camera {camera}: xrDestroyCameraCaptureSessionPICO"),
            result,
        );
    }
    let result = unsafe { (f.destroy_device)(device) };
    logged(&format!("camera {camera}: xrDestroyCameraDevicePICO"), result);
}

fn run(session: &xr::Session<xr::OpenGlEs>, state: CameraImageExtensionState) {
    info!("CAMERA_PROBE starting (extension state {state:?})");
    if !state.enabled {
        warn!("CAMERA_PROBE {CAMERA_IMAGE_EXTENSION_NAME} is not enabled, nothing to probe");
        return;
    }
    thread::sleep(START_DELAY);

    // Same physical cameras as the SecureMR camera pass: prefer to run while it is off
    if crate::camera_frames::camera_pass_active() {
        info!(
            "CAMERA_PROBE the SecureMR camera pass is active; waiting up to {CAMERA_PASS_WAIT:?} \
            for it to stop"
        );
        let start = Instant::now();
        while crate::camera_frames::camera_pass_active() && start.elapsed() < CAMERA_PASS_WAIT {
            thread::sleep(Duration::from_millis(500));
        }
        if crate::camera_frames::camera_pass_active() {
            warn!(
                "CAMERA_PROBE the SecureMR camera pass is still active: probing anyway (results \
                or the camera pass may be affected)"
            );
        }
    } else {
        info!("CAMERA_PROBE no SecureMR camera pass active");
    }

    #[cfg(target_os = "android")]
    alvr_system_info::try_get_permission("android.permission.CAMERA");

    let Some(functions) = Functions::load_all(session) else {
        warn!("CAMERA_PROBE a required function is missing, giving up");
        return;
    };

    let reference = Reference::capture(session);

    let cameras = enumerate_cameras(&functions, session.instance());
    let properties = cameras
        .iter()
        .map(|&camera| camera_properties(&functions, session.instance(), camera))
        .collect::<Vec<_>>();
    for (camera, properties) in cameras.iter().zip(&properties) {
        info!("CAMERA_PROBE camera {camera}: {properties:?}");
    }
    // Passthrough colour cameras first (one at a time), then any others
    let mut order = cameras.iter().zip(&properties).collect::<Vec<_>>();
    order.sort_by_key(|(_, properties)| properties.camera_type != Some(1));
    for (&camera, properties) in order {
        probe_camera(&functions, session, camera, properties, &reference);
    }

    info!("CAMERA_PROBE done");
}
