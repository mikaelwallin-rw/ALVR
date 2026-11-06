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
    sync::Arc,
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

// The first frames after the camera starts may carry a different clock and unsettled exposure
const WARMUP: Duration = Duration::from_millis(500);
// A capture timestamp further than this from "now" is not on the runtime clock
const MAX_CAPTURE_AGE: Duration = Duration::from_secs(1);
const JPEG_QUALITY: u8 = 95;
const ERROR_RETRY_INTERVAL: Duration = Duration::from_secs(1);
// Frames per run whose raw poses are logged (CAMPOSE_DIAG) to validate the camera extrinsics
const POSE_DIAG_FRAMES: u64 = 10;

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
    info!(
        "Camera frames: started, {0}x{0} every {1:?}",
        config.image_size, config.interval
    );

    let started = Instant::now();
    let mut last_error = None;
    let mut sent_count = 0_u64;
    while running.value() {
        let deadline = Instant::now() + config.interval;

        let result = camera.capture().map_err(|e| format!("capture failed: {e}"));
        let frame = result.and_then(|capture| {
            make_frame(
                xr_session.instance(),
                &local_reference_space,
                &view_reference_space,
                stage_reference_space,
                capture,
                config.eye,
                sent_count < POSE_DIAG_FRAMES,
            )
        });

        match frame {
            Ok(frame) if started.elapsed() > WARMUP => {
                core_context.send_camera_frame(frame);
                sent_count += 1;
                if sent_count == 1 {
                    info!("Camera frames: first frame sent");
                }
                last_error = None;
            }
            Ok(_) => (),
            Err(e) => {
                if last_error.as_ref() != Some(&e) {
                    warn!("Camera frames: frame dropped: {e}");
                    last_error = Some(e);
                }
                thread::sleep(ERROR_RETRY_INTERVAL);
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
    local_reference_space: &xr::Space,
    view_reference_space: &xr::Space,
    stage_reference_space: &xr::Space,
    capture: CameraCapture,
    eye: CameraEye,
    log_poses: bool,
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

    let stage_from_local = local_reference_space
        .locate(stage_reference_space, capture_time)
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
    let (view_location, view_velocity) = view_reference_space
        .relate(stage_reference_space, capture_time)
        .map_err(|e| format!("cannot locate VIEW in STAGE: {e}"))?;
    let head_angular_speed = if view_velocity
        .velocity_flags
        .contains(xr::SpaceVelocityFlags::ANGULAR_VALID)
    {
        crate::from_xr_vec3(view_velocity.angular_velocity).length()
    } else {
        0.0
    };

    if log_poses {
        let stage_from_view = crate::from_xr_pose(view_location.pose);
        info!(
            "CAMPOSE_DIAG eye={eye:?} t={} stage_from_view {} | stage_from_local {} | securemr_local_from_cam_cv {} | raw_m {:?}",
            capture_time.as_nanos(),
            pose_text(stage_from_view),
            pose_text(stage_from_local),
            pose_text(local_from_camera),
            m
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
        stage_from_camera: stage_from_local * local_from_camera,
        head_angular_speed,
        jpeg,
    })
}
