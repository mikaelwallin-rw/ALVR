// Writer for tracking data that SteamVR does not expose, published to overlay apps (ScreenVR)
// through shared memory: the stage -> driver transform and the index fingertips. Also reads the
// overlay's request block (marker detection on/off). Markers are not supported yet: the headset
// runtime does not offer marker tracking, so a marker request is answered with `Unsupported`.
//
// Contract (source of truth: ScreenVR `src/VrTrackingExtras.h`, copied here, no dependency):
// - `Local\VRTrackingExtras_v1`: `VrTrackingExtrasBlock`, pack(4), written by ALVR.
// - `Local\VRTrackingRequest_v1`: `VrTrackingRequestBlock`, pack(4), written by the overlay.
// - Seqlock: odd `sequence` = write in progress. Readers accept a copy only if `sequence` was even
//   and unchanged before and after the copy.
// - `heartbeat_ms` (GetTickCount64) is refreshed at least every 100 ms. Older than 1000 ms = absent.
// - Driver space = TrackingUniverseRawAndUncalibrated = recenter(client STAGE).

use alvr_common::{Pose, glam::Mat3};
use std::{
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    thread::{self, JoinHandle},
    time::Duration,
};

const HEARTBEAT_INTERVAL: Duration = Duration::from_millis(50);

// Pose as an OpenVR HmdMatrix34_t: row-major 3x4, rotation | translation
pub(crate) fn to_matrix(pose: Pose) -> [[f32; 4]; 3] {
    let rotation = Mat3::from_quat(pose.orientation);
    let p = pose.position;
    let row = |r: usize| {
        [
            rotation.col(0)[r],
            rotation.col(1)[r],
            rotation.col(2)[r],
            p[r],
        ]
    };

    [row(0), row(1), row(2)]
}

#[cfg(windows)]
pub use windows_impl::{
    TRACKING_EXTRAS_WRITER, bump_stage_epoch, camera_frames_request, recenter_counter,
    report_tracking,
};

#[cfg(not(windows))]
pub fn report_tracking(_driver_from_stage: Pose, _fingertips: [Option<Pose>; 2]) {}

// Camera frames requested by the overlay: Some(use the right camera)
#[cfg(not(windows))]
pub fn camera_frames_request() -> Option<bool> {
    None
}

#[cfg(not(windows))]
pub fn recenter_counter() -> u32 {
    0
}

#[cfg(not(windows))]
pub fn bump_stage_epoch() {}

// Refreshes the heartbeat (and request-derived state) while no tracking packets arrive, e.g.
// when no headset is connected. Joined on drop, so that the thread never outlives the driver.
pub struct TrackingExtrasHeartbeat {
    running: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl TrackingExtrasHeartbeat {
    pub fn start() -> Self {
        let running = Arc::new(AtomicBool::new(true));

        let thread = thread::Builder::new()
            .name("tracking_extras".into())
            .spawn({
                let running = Arc::clone(&running);
                move || {
                    while running.load(Ordering::Relaxed) {
                        #[cfg(windows)]
                        TRACKING_EXTRAS_WRITER.lock().refresh();
                        crate::camera_frames::refresh_heartbeat();

                        thread::sleep(HEARTBEAT_INTERVAL);
                    }
                }
            })
            .ok();

        Self { running, thread }
    }
}

impl Drop for TrackingExtrasHeartbeat {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            thread.join().ok();
        }
    }
}

#[cfg(windows)]
mod windows_impl {
    use super::to_matrix;
    use alvr_common::{Pose, info, parking_lot::Mutex, warn};
    use std::{
        mem, ptr,
        sync::atomic::{self, AtomicU32, Ordering},
        time::{Duration, Instant},
    };
    use windows::{
        Win32::{
            Foundation::{CloseHandle, HANDLE, INVALID_HANDLE_VALUE},
            System::{
                Memory::{
                    CreateFileMappingW, FILE_MAP_READ, FILE_MAP_WRITE, MEMORY_MAPPED_VIEW_ADDRESS,
                    MapViewOfFile, OpenFileMappingW, PAGE_READWRITE, UnmapViewOfFile,
                },
                SystemInformation::GetTickCount64,
            },
        },
        core::w,
    };

    const EXTRAS_MAGIC: u32 = 0x584B_5254; // 'TRKX' little-endian
    const REQUEST_MAGIC: u32 = 0x514B_5254; // 'TRKQ' little-endian
    // Version 2 appends stage_epoch: version 1 readers only read the first 2256 bytes
    const VERSION: u32 = 2;
    // The request block layout is unchanged
    const REQUEST_VERSION: u32 = 1;
    const MAX_MARKERS: usize = 32;
    const STALE_MS: u64 = 1000;
    // Fingertips not refreshed by a tracking packet for this long are published as invalid
    const FINGERTIP_STALE_MS: u64 = 250;
    // Live hand tracking jitters: a tip pose that stays bit-identical this long is a runtime
    // fallback repeating the last known pose, not a tracked finger
    const FINGERTIP_FROZEN_MS: u64 = 100;
    const OPEN_RETRY_INTERVAL: Duration = Duration::from_secs(1);
    const MAX_READ_ATTEMPTS: usize = 4;
    const SEQUENCE_OFFSET: usize = 8;

    const MARKER_STATUS_OFF: u32 = 0;
    const MARKER_STATUS_UNSUPPORTED: u32 = 3;
    // VrTrackingRequestBlock::requestMarkers is a bit set
    const REQUEST_MARKERS_BIT: u32 = 1;
    const REQUEST_FRAMES_BIT: u32 = 2;
    // With REQUEST_FRAMES_BIT: frames from the right camera instead of the left one
    const REQUEST_RIGHT_CAMERA_BIT: u32 = 4;

    #[repr(C, packed(4))]
    #[derive(Clone, Copy, Default)]
    struct RawFingertip {
        pose: [[f32; 4]; 3], // HmdMatrix34_t, row-major
        valid: u32,
        reserved: u32,
        sample_ms: u64,
    }

    #[repr(C, packed(4))]
    #[derive(Clone, Copy, Default)]
    struct RawMarker {
        pose: [[f32; 4]; 3],
        id: u32,
        dictionary: u32,
        last_seen_ms: u64,
    }

    #[repr(C, packed(4))]
    #[derive(Clone, Copy)]
    struct RawExtrasBlock {
        magic: u32,
        version: u32,
        sequence: u32,
        recenter_counter: u32,
        heartbeat_ms: u64,
        driver_from_stage: [[f32; 4]; 3],
        fingertips: [RawFingertip; 2],
        marker_status: u32,
        marker_count: u32,
        markers: [RawMarker; MAX_MARKERS],
        stage_epoch: u32,
        reserved: u32,
    }

    #[repr(C, packed(4))]
    #[derive(Clone, Copy)]
    struct RawRequestBlock {
        magic: u32,
        version: u32,
        sequence: u32,
        request_markers: u32,
        heartbeat_ms: u64,
        dictionary: u32,
        marker_side_meters: f32,
    }

    const _: () = assert!(mem::size_of::<RawFingertip>() == 64);
    const _: () = assert!(mem::size_of::<RawMarker>() == 64);
    const _: () =
        assert!(mem::size_of::<RawExtrasBlock>() == 24 + 48 + 2 * 64 + 8 + 64 * MAX_MARKERS + 8);
    const _: () = assert!(mem::size_of::<RawRequestBlock>() == 32);

    pub static TRACKING_EXTRAS_WRITER: Mutex<TrackingExtrasWriter> =
        Mutex::new(TrackingExtrasWriter::new());

    // Camera frames requested by the overlay: Some(use the right camera)
    pub fn camera_frames_request() -> Option<bool> {
        let writer = TRACKING_EXTRAS_WRITER.lock();
        writer
            .frames_requested
            .then_some(writer.frames_right_camera)
    }

    // Incremented whenever the stage -> driver transform changes
    pub fn recenter_counter() -> u32 {
        TRACKING_EXTRAS_WRITER.lock().recenter_counter
    }

    // The client (re)established its STAGE space (new playspace, reference space change or
    // reconnection): poses stored in stage or driver space from before are no longer valid
    pub fn bump_stage_epoch() {
        let mut writer = TRACKING_EXTRAS_WRITER.lock();
        writer.stage_epoch = writer.stage_epoch.wrapping_add(1);
        info!("Tracking extras: stage epoch {}", writer.stage_epoch);
    }

    // Called for every tracking packet, with poses already in driver space
    pub fn report_tracking(driver_from_stage: Pose, fingertips: [Option<Pose>; 2]) {
        TRACKING_EXTRAS_WRITER
            .lock()
            .report_tracking(driver_from_stage, fingertips);
    }

    fn now_ms() -> u64 {
        unsafe { GetTickCount64() }
    }

    struct Mapping {
        handle: HANDLE,
        view: MEMORY_MAPPED_VIEW_ADDRESS,
    }

    // The mapping is only accessed from behind a Mutex
    unsafe impl Send for Mapping {}

    impl Drop for Mapping {
        fn drop(&mut self) {
            unsafe {
                UnmapViewOfFile(self.view).ok();
                CloseHandle(self.handle).ok();
            }
        }
    }

    impl Mapping {
        fn sequence(&self) -> &AtomicU32 {
            // # Safety: the view is at least one block long and `sequence` is 4-byte aligned
            // (pack(4), offset 8, page-aligned view)
            unsafe {
                &*self
                    .view
                    .Value
                    .cast::<u8>()
                    .add(SEQUENCE_OFFSET)
                    .cast::<AtomicU32>()
            }
        }
    }

    #[derive(Clone, Copy, Default)]
    struct FingertipState {
        pose: Pose,
        tracked: bool,
        sample_ms: u64,
        // Last time the pose changed (bitwise)
        changed_ms: u64,
    }

    impl FingertipState {
        fn is_valid(&self, now_ms: u64) -> bool {
            self.tracked
                && now_ms.saturating_sub(self.sample_ms) <= FINGERTIP_STALE_MS
                && now_ms.saturating_sub(self.changed_ms) <= FINGERTIP_FROZEN_MS
        }
    }

    fn same_bits(a: Pose, b: Pose) -> bool {
        let bits = |pose: Pose| {
            let p = pose.position;
            let o = pose.orientation;
            [p.x, p.y, p.z, o.x, o.y, o.z, o.w].map(f32::to_bits)
        };

        bits(a) == bits(b)
    }

    pub struct TrackingExtrasWriter {
        extras_mapping: Option<Mapping>,
        last_create_attempt: Option<Instant>,
        request_mapping: Option<Mapping>,
        last_request_open_attempt: Option<Instant>,
        driver_from_stage: Option<Pose>,
        recenter_counter: u32,
        fingertips: [FingertipState; 2],
        marker_status: u32,
        frames_requested: bool,
        frames_right_camera: bool,
        stage_epoch: u32,
    }

    impl TrackingExtrasWriter {
        pub const fn new() -> Self {
            Self {
                extras_mapping: None,
                last_create_attempt: None,
                request_mapping: None,
                last_request_open_attempt: None,
                driver_from_stage: None,
                recenter_counter: 0,
                fingertips: [FingertipState {
                    pose: Pose::IDENTITY,
                    tracked: false,
                    sample_ms: 0,
                    changed_ms: 0,
                }; 2],
                marker_status: MARKER_STATUS_OFF,
                frames_requested: false,
                frames_right_camera: false,
                stage_epoch: 0,
            }
        }

        fn report_tracking(&mut self, driver_from_stage: Pose, fingertips: [Option<Pose>; 2]) {
            let changed = self.driver_from_stage.is_none_or(|last| {
                last.position != driver_from_stage.position
                    || last.orientation != driver_from_stage.orientation
            });
            if changed {
                if self.driver_from_stage.is_some() {
                    self.recenter_counter = self.recenter_counter.wrapping_add(1);
                }
                self.driver_from_stage = Some(driver_from_stage);
            }

            let now = now_ms();
            for (state, fingertip) in self.fingertips.iter_mut().zip(fingertips) {
                if let Some(pose) = fingertip {
                    let changed_ms = if state.tracked && same_bits(state.pose, pose) {
                        state.changed_ms
                    } else {
                        now
                    };
                    *state = FingertipState {
                        pose,
                        tracked: true,
                        sample_ms: now,
                        changed_ms,
                    };
                } else {
                    state.tracked = false;
                }
            }

            self.publish();
        }

        // Heartbeat: re-read the request and republish
        pub fn refresh(&mut self) {
            let request_flags = self.read_request().map_or(0, |r| r.request_markers);

            let frames_requested = request_flags & REQUEST_FRAMES_BIT != 0;
            let frames_right_camera = request_flags & REQUEST_RIGHT_CAMERA_BIT != 0;
            if (frames_requested, frames_right_camera)
                != (self.frames_requested, self.frames_right_camera)
            {
                info!(
                    "Tracking extras: camera frames requested: {frames_requested} (right camera: {frames_right_camera})"
                );
                self.frames_requested = frames_requested;
                self.frames_right_camera = frames_right_camera;
            }

            let marker_status = if request_flags & REQUEST_MARKERS_BIT != 0 {
                MARKER_STATUS_UNSUPPORTED
            } else {
                MARKER_STATUS_OFF
            };
            if marker_status != self.marker_status {
                info!("Tracking extras: marker status {marker_status} (no headset marker support)");
                self.marker_status = marker_status;
            }

            self.publish();
        }

        fn ensure_extras_mapping(&mut self) -> Option<&Mapping> {
            if self.extras_mapping.is_none()
                && self
                    .last_create_attempt
                    .is_none_or(|instant| instant.elapsed() >= OPEN_RETRY_INTERVAL)
            {
                self.last_create_attempt = Some(Instant::now());

                let size = mem::size_of::<RawExtrasBlock>() as u32;
                // Opens the existing mapping if there is one
                match unsafe {
                    CreateFileMappingW(
                        INVALID_HANDLE_VALUE,
                        None,
                        PAGE_READWRITE,
                        0,
                        size,
                        w!("Local\\VRTrackingExtras_v1"),
                    )
                } {
                    Ok(handle) => {
                        let view = unsafe {
                            MapViewOfFile(
                                handle,
                                FILE_MAP_WRITE,
                                0,
                                0,
                                mem::size_of::<RawExtrasBlock>(),
                            )
                        };
                        if view.Value.is_null() {
                            warn!("Tracking extras: MapViewOfFile failed");
                            unsafe { CloseHandle(handle).ok() };
                        } else {
                            info!(
                                "Tracking extras: shared memory Local\\VRTrackingExtras_v1 created"
                            );
                            self.extras_mapping = Some(Mapping { handle, view });
                        }
                    }
                    Err(e) => warn!("Tracking extras: CreateFileMappingW failed: {e}"),
                }
            }

            self.extras_mapping.as_ref()
        }

        fn publish(&mut self) {
            let now = now_ms();

            let fingertips = self.fingertips.map(|state| RawFingertip {
                pose: to_matrix(state.pose),
                valid: u32::from(state.is_valid(now)),
                reserved: 0,
                sample_ms: state.sample_ms,
            });

            let block = RawExtrasBlock {
                magic: EXTRAS_MAGIC,
                version: VERSION,
                sequence: 0, // not copied, see below
                recenter_counter: self.recenter_counter,
                heartbeat_ms: now,
                driver_from_stage: to_matrix(self.driver_from_stage.unwrap_or(Pose::IDENTITY)),
                fingertips,
                marker_status: self.marker_status,
                marker_count: 0,
                markers: [RawMarker::default(); MAX_MARKERS],
                stage_epoch: self.stage_epoch,
                reserved: 0,
            };

            let Some(mapping) = self.ensure_extras_mapping() else {
                return;
            };

            let sequence = mapping.sequence();
            let before = sequence.load(Ordering::Relaxed);
            // An odd value left by a writer that died mid-write is skipped over
            let odd = if before % 2 == 0 {
                before.wrapping_add(1)
            } else {
                before.wrapping_add(2)
            };
            sequence.store(odd, Ordering::Relaxed);
            atomic::fence(Ordering::Release);

            let source = ptr::from_ref(&block).cast::<u8>();
            let destination = mapping.view.Value.cast::<u8>();
            // # Safety: both buffers are size_of::<RawExtrasBlock>() bytes; the sequence field
            // (bytes 8..12) is skipped so that it is only written atomically
            unsafe {
                ptr::copy_nonoverlapping(source, destination, SEQUENCE_OFFSET);
                ptr::copy_nonoverlapping(
                    source.add(SEQUENCE_OFFSET + 4),
                    destination.add(SEQUENCE_OFFSET + 4),
                    mem::size_of::<RawExtrasBlock>() - SEQUENCE_OFFSET - 4,
                );
            }

            sequence.store(odd.wrapping_add(1), Ordering::Release);
        }

        // Fresh request block, if the overlay is publishing one
        fn read_request(&mut self) -> Option<RawRequestBlock> {
            if self.request_mapping.is_none()
                && self
                    .last_request_open_attempt
                    .is_none_or(|instant| instant.elapsed() >= OPEN_RETRY_INTERVAL)
            {
                self.last_request_open_attempt = Some(Instant::now());

                if let Ok(handle) = unsafe {
                    OpenFileMappingW(FILE_MAP_READ.0, false, w!("Local\\VRTrackingRequest_v1"))
                } {
                    let view = unsafe {
                        MapViewOfFile(
                            handle,
                            FILE_MAP_READ,
                            0,
                            0,
                            mem::size_of::<RawRequestBlock>(),
                        )
                    };
                    if view.Value.is_null() {
                        unsafe { CloseHandle(handle).ok() };
                    } else {
                        info!(
                            "Tracking extras: request mapping Local\\VRTrackingRequest_v1 opened"
                        );
                        self.request_mapping = Some(Mapping { handle, view });
                    }
                }
            }

            let mapping = self.request_mapping.as_ref()?;
            let sequence = mapping.sequence();

            let mut block = None;
            for _ in 0..MAX_READ_ATTEMPTS {
                let before = sequence.load(Ordering::Acquire);
                if before % 2 != 0 {
                    std::hint::spin_loop();
                    continue;
                }

                let copy =
                    unsafe { ptr::read_volatile(mapping.view.Value.cast::<RawRequestBlock>()) };

                atomic::fence(Ordering::Acquire);
                if sequence.load(Ordering::Relaxed) == before {
                    block = Some(copy);
                    break;
                }
            }
            let block = block?;

            // The mapping is kept open even while stale: as long as this handle exists, a restarted
            // overlay opens the same named section again
            let (magic, version, heartbeat_ms) = (block.magic, block.version, block.heartbeat_ms);
            let fresh = now_ms().saturating_sub(heartbeat_ms) <= STALE_MS;

            (magic == REQUEST_MAGIC && version >= REQUEST_VERSION && fresh).then_some(block)
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        use alvr_common::glam::{Quat, Vec3};

        #[test]
        fn matrix_is_row_major_rotation_and_translation() {
            let pose = Pose {
                orientation: Quat::from_rotation_y(std::f32::consts::FRAC_PI_2),
                position: Vec3::new(1.0, 2.0, 3.0),
            };
            let m = to_matrix(pose);

            // +X rotated 90 degrees about Y points to -Z: first column is (0, 0, -1)
            assert!((m[0][0]).abs() < 1e-6 && (m[2][0] + 1.0).abs() < 1e-6);
            assert_eq!([m[0][3], m[1][3], m[2][3]], [1.0, 2.0, 3.0]);
        }

        #[test]
        fn frozen_or_stale_fingertip_is_invalid() {
            let state = FingertipState {
                pose: Pose::IDENTITY,
                tracked: true,
                sample_ms: 1000,
                changed_ms: 1000,
            };
            assert!(state.is_valid(1050));
            // Same pose resent for longer than FINGERTIP_FROZEN_MS
            assert!(
                !FingertipState {
                    sample_ms: 1200,
                    ..state
                }
                .is_valid(1200)
            );
            // No packet for longer than FINGERTIP_STALE_MS
            assert!(!state.is_valid(1000 + FINGERTIP_STALE_MS + 1));
            assert!(
                !FingertipState {
                    tracked: false,
                    ..state
                }
                .is_valid(1000)
            );

            let moved = Pose {
                position: Vec3::new(0.0, 0.0, f32::EPSILON),
                ..Pose::IDENTITY
            };
            assert!(same_bits(Pose::IDENTITY, Pose::IDENTITY));
            assert!(!same_bits(Pose::IDENTITY, moved));
        }
    }
}
