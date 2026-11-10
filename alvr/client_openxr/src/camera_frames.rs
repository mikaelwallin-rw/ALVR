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
// extrinsics. Measured by hand-eye calibration of CAMPOSE_DIAG data (head rotation up to 75
// degrees, residual below 1 mm, equal in both space regimes to 0.1 mm). Both cameras share one
// rotation; baseline 64.1 mm.
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

fn camera_extrinsics() -> Option<CameraExtrinsics> {
    let product = product_name();
    let extrinsics = CAMERA_EXTRINSICS
        .iter()
        .find(|(name, _)| *name == product)
        .map(|(_, extrinsics)| *extrinsics);
    if extrinsics.is_none() {
        warn!(
            "Camera frames: no camera extrinsics for {product:?}, using the SecureMR camera pose \
            (may be offset if SecureMR's LOCAL space differs from the app's)"
        );
    }

    extrinsics
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
    let extrinsics = camera_extrinsics();
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
                extrinsics,
                PoseDiagnostics {
                    log_poses: sent_count < POSE_DIAG_FRAMES,
                    // Once per run: the first frame after the warm-up
                    log_path_difference: sent_count == 0 && started.elapsed() > WARMUP,
                },
            )
        });

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
    extrinsics: Option<CameraExtrinsics>,
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
    let stage_from_camera = if let Some(extrinsics) = extrinsics {
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
    if diagnostics.log_path_difference && extrinsics.is_some() {
        // Nonzero when SecureMR's LOCAL space differs from the app's (or the extrinsics are off)
        let difference = securemr_stage_from_camera.position - stage_from_camera.position;
        let angle = securemr_stage_from_camera
            .orientation
            .angle_between(stage_from_camera.orientation);
        info!(
            "CAMPOSE_DIFF eye={eye:?} securemr - extrinsics: position ({:.1}, {:.1}, {:.1}) cm \
            (|{:.1}| cm), rotation {:.2} deg",
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
