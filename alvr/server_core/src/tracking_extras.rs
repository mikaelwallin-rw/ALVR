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
// - Version 4: hands[2] (raw tracked joints, driver space) and how the client draws them; request
//   version 3: automatic hand correction, forwarded to the client as HandCorrection; request
//   version 4: hand display transform, forwarded as HandDisplay (inactive while the request block
//   is stale or older).
// - Version 3: STAGE identity (changes only when the physical STAGE moves), the persistent anchor
//   and the headset model, from the client's StageInfo reports. Request version 2: anchor commands,
//   forwarded to the client as AnchorRequest.

use alvr_common::{Pose, glam::Mat3};
#[cfg(not(windows))]
use alvr_packets::{AnchorRequest, HandCorrection, HandDisplay, HandShapeInfo, StageInfo};
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
    TRACKING_EXTRAS_WRITER, anchor_request, bump_stage_epoch, camera_frames_request,
    hand_correction, hand_display, recenter_counter, report_hand_shape, report_stage_info,
    report_tracking, stage_epoch,
};

/// Joints of a hand skeleton (XR_EXT_hand_tracking order, 26), driver space
pub type HandJoints = [Pose; 26];

#[cfg(not(windows))]
pub fn report_stage_info(_info: StageInfo) {}

// The overlay's latest anchor command (sequence != 0)
#[cfg(not(windows))]
pub fn anchor_request() -> Option<AnchorRequest> {
    None
}

#[cfg(not(windows))]
pub fn report_tracking(
    _driver_from_stage: Pose,
    _fingertips: [Option<Pose>; 2],
    _hands: [Option<HandJoints>; 2],
) {
}

#[cfg(not(windows))]
pub fn report_hand_shape(_info: HandShapeInfo) {}

// The overlay's latest hand correction (sequence != 0)
#[cfg(not(windows))]
pub fn hand_correction() -> Option<HandCorrection> {
    None
}

// The overlay's hand display transform (inactive when stale)
#[cfg(not(windows))]
pub fn hand_display() -> Option<HandDisplay> {
    None
}

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

#[cfg(not(windows))]
pub fn stage_epoch() -> u32 {
    0
}

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
    use super::{HandJoints, to_matrix};
    use alvr_common::{
        Pose,
        glam::{Mat3, Quat, Vec3},
        info,
        parking_lot::Mutex,
        warn,
    };
    use alvr_packets::{
        AnchorRequest, AnchorRequestKind, HandCorrection, HandDisplay, HandShapeInfo, StageInfo,
    };
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
    // Version 2 appends stage_epoch, version 3 the stage identity, the anchor and the product
    // name: older readers only read the first 2256 / 2264 bytes
    const VERSION: u32 = 3;
    // Oldest request block accepted; version 2 appends the anchor request
    const REQUEST_VERSION: u32 = 1;
    const REQUEST_VERSION_ANCHOR: u32 = 2;
    const REQUEST_VERSION_HAND_CORRECTION: u32 = 3;
    const REQUEST_VERSION_HAND_DISPLAY: u32 = 4;
    const HAND_DISPLAY_ACTIVE_BIT: u32 = 1;
    // XR_EXT_hand_tracking joints published per hand: wrist, palm, index proximal..tip, thumb tip,
    // middle tip (RawHand order)
    const PUBLISHED_HAND_JOINTS: [usize; 8] = [1, 0, 7, 8, 9, 10, 5, 15];
    const ANCHOR_REQUEST_CREATE: u32 = 1;
    const ANCHOR_REQUEST_FORGET: u32 = 2;
    const ANCHOR_STATE_LOCATED: u32 = 2;
    const ANCHOR_STATE_LOST: u32 = 3;
    // The client reports every 500 ms
    const ANCHOR_LOCATED_STALE_MS: u64 = 1000;
    const PRODUCT_NAME_BYTES: usize = 32;
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
        // Version 3
        stage_identity: u64,
        previous_stage_identity: u64,
        stage_change_sequence: u32,
        stage_from_previous_valid: u32,
        stage_from_previous: [[f32; 4]; 3],
        anchor_state: u32,
        anchor_request_served: u32,
        anchor_uuid: [u8; 16],
        anchor_pose: [[f32; 4]; 3],
        anchor_located_ms: u64,
        anchor_generation: u32,
        reserved_v3: u32,
        product_name: [u8; PRODUCT_NAME_BYTES],
        // Version 4
        hands: [RawHand; 2],
    }

    #[repr(C, packed(4))]
    #[derive(Clone, Copy)]
    struct RawHand {
        valid: u32,
        reserved: u32,
        sample_ms: u64,
        // PUBLISHED_HAND_JOINTS
        joints: [[[f32; 4]; 3]; 8],
        draw_thickness: f32,
        draw_scale: f32,
        draw_fingers: f32,
        draw_finger_correction: f32,
        draw_correction_sequence: u32,
        reserved2: u32,
    }

    impl Default for RawHand {
        fn default() -> Self {
            Self {
                valid: 0,
                reserved: 0,
                sample_ms: 0,
                joints: [to_matrix(Pose::IDENTITY); 8],
                draw_thickness: 0.0,
                draw_scale: 0.0,
                draw_fingers: 0.0,
                draw_finger_correction: 0.0,
                draw_correction_sequence: 0,
                reserved2: 0,
            }
        }
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
        // Request version 2 (zeroed for a version 1 block, see read_request())
        anchor_request: u32,
        anchor_request_sequence: u32,
        anchor_request_pose: [[f32; 4]; 3],
        // Request version 3 (zeroed for older blocks, see read_request())
        hand_finger_scale: [f32; 2],
        hand_correction_sequence: u32,
        hand_near_meters: f32,
        hand_correction_plane: [[f32; 4]; 3],
        // Request version 4 (zeroed for older blocks, see read_request())
        hand_display_sequence: u32,
        hand_display_flags: u32,
        hand_display_transform: [[f32; 4]; 3],
        hand_display_scale: f32,
        hand_display_quad_pose: [[f32; 4]; 3],
        hand_display_quad_size: [f32; 2],
        hand_display_full_meters: f32,
        hand_display_falloff_meters: f32,
    }

    const _: () = assert!(mem::size_of::<RawFingertip>() == 64);
    const _: () = assert!(mem::size_of::<RawMarker>() == 64);
    const _: () = assert!(
        mem::size_of::<RawExtrasBlock>()
            == 24 + 48 + 2 * 64 + 8 + 64 * MAX_MARKERS + 8 + 192 + 2 * 424
    );
    const _: () = assert!(mem::size_of::<RawHand>() == 424);
    const _: () = assert!(mem::offset_of!(RawExtrasBlock, hands) == 2456);
    const _: () = assert!(mem::size_of::<RawExtrasBlock>() == 3304);
    const _: () = assert!(mem::offset_of!(RawExtrasBlock, stage_identity) == 2264);
    const _: () = assert!(mem::offset_of!(RawRequestBlock, anchor_request) == 32);
    const _: () = assert!(mem::offset_of!(RawRequestBlock, hand_finger_scale) == 88);
    const _: () = assert!(mem::offset_of!(RawRequestBlock, hand_display_sequence) == 152);
    const _: () = assert!(mem::size_of::<RawRequestBlock>() == 276);

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

    // Current STAGE epoch, see bump_stage_epoch()
    pub fn stage_epoch() -> u32 {
        TRACKING_EXTRAS_WRITER.lock().stage_epoch
    }

    // The client (re)established its STAGE space (new playspace, reference space change or
    // reconnection): poses stored in stage or driver space from before are no longer valid
    pub fn bump_stage_epoch() {
        let mut writer = TRACKING_EXTRAS_WRITER.lock();
        writer.stage_epoch = writer.stage_epoch.wrapping_add(1);
        info!("Tracking extras: stage epoch {}", writer.stage_epoch);
    }

    // The client's STAGE identity and anchor report (sent on change and every 2 s)
    pub fn report_stage_info(info: StageInfo) {
        TRACKING_EXTRAS_WRITER.lock().report_stage_info(info);
    }

    // The overlay's latest anchor command (sequence != 0)
    pub fn anchor_request() -> Option<AnchorRequest> {
        TRACKING_EXTRAS_WRITER.lock().anchor_request
    }

    // Inverse of to_matrix()
    fn from_matrix(m: [[f32; 4]; 3]) -> Pose {
        let rotation = Mat3::from_cols(
            Vec3::new(m[0][0], m[1][0], m[2][0]),
            Vec3::new(m[0][1], m[1][1], m[2][1]),
            Vec3::new(m[0][2], m[1][2], m[2][2]),
        );
        Pose {
            orientation: Quat::from_mat3(&rotation).normalize(),
            position: Vec3::new(m[0][3], m[1][3], m[2][3]),
        }
    }

    // Called for every tracking packet, with poses already in driver space. `hands`: the raw
    // tracked skeletons, under the same validity rule as the fingertips.
    pub fn report_tracking(
        driver_from_stage: Pose,
        fingertips: [Option<Pose>; 2],
        hands: [Option<HandJoints>; 2],
    ) {
        TRACKING_EXTRAS_WRITER
            .lock()
            .report_tracking(driver_from_stage, fingertips, hands);
    }

    // What the client applies when drawing the hands
    pub fn report_hand_shape(info: HandShapeInfo) {
        let mut writer = TRACKING_EXTRAS_WRITER.lock();
        writer.hand_shape = Some(info);
        writer.publish();
    }

    // The overlay's latest hand correction (sequence != 0)
    pub fn hand_correction() -> Option<HandCorrection> {
        TRACKING_EXTRAS_WRITER.lock().hand_correction
    }

    // The overlay's hand display transform: Some once a version 4 request block was seen; inactive
    // while the block is stale or missing
    pub fn hand_display() -> Option<HandDisplay> {
        TRACKING_EXTRAS_WRITER.lock().hand_display
    }

    // NUL-terminated UTF-8: at most 31 bytes, cut at a character boundary
    fn product_name_bytes(name: &str) -> [u8; PRODUCT_NAME_BYTES] {
        let mut bytes = [0; PRODUCT_NAME_BYTES];
        let mut length = name.len().min(PRODUCT_NAME_BYTES - 1);
        while !name.is_char_boundary(length) {
            length -= 1;
        }
        bytes[..length].copy_from_slice(&name.as_bytes()[..length]);
        bytes
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
        stage_info: Option<StageInfo>,
        anchor_located_ms: u64,
        anchor_request: Option<AnchorRequest>,
        // Per hand: receive time and the raw skeleton (driver space)
        hands: [Option<(u64, HandJoints)>; 2],
        hand_shape: Option<HandShapeInfo>,
        hand_correction: Option<HandCorrection>,
        hand_display: Option<HandDisplay>,
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
                stage_info: None,
                anchor_located_ms: 0,
                anchor_request: None,
                hands: [None, None],
                hand_shape: None,
                hand_correction: None,
                hand_display: None,
            }
        }

        fn report_stage_info(&mut self, info: StageInfo) {
            if info.anchor_state == ANCHOR_STATE_LOCATED && info.anchor_pose.is_some() {
                self.anchor_located_ms = now_ms();
            }
            if self.stage_info.as_ref().is_none_or(|previous| {
                previous.stage_identity != info.stage_identity
                    || previous.anchor_state != info.anchor_state
                    || previous.anchor_generation != info.anchor_generation
                    || previous.anchor_request_served != info.anchor_request_served
            }) {
                info!(
                    "Tracking extras: stage {:016x} (previous {:016x}, change {}, transform {}), \
                     anchor state {} generation {} served {}, product {:?}",
                    info.stage_identity,
                    info.previous_stage_identity,
                    info.stage_change_sequence,
                    info.stage_from_previous.is_some(),
                    info.anchor_state,
                    info.anchor_generation,
                    info.anchor_request_served,
                    info.product_name
                );
            }
            self.stage_info = Some(info);
            self.publish();
        }

        fn report_tracking(
            &mut self,
            driver_from_stage: Pose,
            fingertips: [Option<Pose>; 2],
            hands: [Option<HandJoints>; 2],
        ) {
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
            for (state, hand) in self.hands.iter_mut().zip(hands) {
                *state = hand.map(|joints| (now, joints));
            }

            self.publish();
        }

        // Heartbeat: re-read the request and republish
        pub fn refresh(&mut self) {
            let request = self.read_request();
            let request_flags = request.map_or(0, |r| r.request_markers);

            if let Some(request) = request
                && request.hand_correction_sequence != 0
                && self
                    .hand_correction
                    .is_none_or(|last| last.sequence != request.hand_correction_sequence)
            {
                let correction = HandCorrection {
                    sequence: request.hand_correction_sequence,
                    finger_scale: request.hand_finger_scale,
                    near_meters: request.hand_near_meters,
                    plane: from_matrix(request.hand_correction_plane),
                };
                info!("Tracking extras: hand correction {correction:?}");
                self.hand_correction = Some(correction);
            }

            // Hand display transform: as requested, or inactive while the block is stale/older
            match request.filter(|r| r.version >= REQUEST_VERSION_HAND_DISPLAY) {
                Some(request) => {
                    let display = HandDisplay {
                        sequence: request.hand_display_sequence,
                        active: request.hand_display_flags & HAND_DISPLAY_ACTIVE_BIT != 0,
                        transform: from_matrix(request.hand_display_transform),
                        scale: request.hand_display_scale,
                        quad_pose: from_matrix(request.hand_display_quad_pose),
                        quad_size: request.hand_display_quad_size,
                        full_meters: request.hand_display_full_meters,
                        falloff_meters: request.hand_display_falloff_meters,
                    };
                    if self.hand_display.is_none_or(|last| {
                        last.sequence != display.sequence || last.active != display.active
                    }) {
                        if self
                            .hand_display
                            .is_none_or(|last| last.active != display.active)
                        {
                            info!("Tracking extras: hand display transform {display:?}");
                        }
                        self.hand_display = Some(display);
                    }
                }
                None => {
                    if let Some(display) = &mut self.hand_display
                        && display.active
                    {
                        info!("Tracking extras: hand display transform inactive (request stale)");
                        display.active = false;
                    }
                }
            }

            // Kept until replaced: a stale request block does not cancel a pending anchor request
            if let Some(request) = request
                && request.anchor_request_sequence != 0
                && self
                    .anchor_request
                    .is_none_or(|last| last.sequence != request.anchor_request_sequence)
            {
                let kind = match request.anchor_request {
                    ANCHOR_REQUEST_CREATE => Some(AnchorRequestKind::Create(from_matrix(
                        request.anchor_request_pose,
                    ))),
                    ANCHOR_REQUEST_FORGET => Some(AnchorRequestKind::Forget),
                    _ => None,
                };
                if let Some(kind) = kind {
                    let sequence = request.anchor_request_sequence;
                    info!("Tracking extras: anchor request {sequence}: {kind:?}");
                    self.anchor_request = Some(AnchorRequest { sequence, kind });
                }
            }

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
                        // The whole section: one left by an older peer may be smaller than this
                        // block, but views are page-granular, so the block is always addressable
                        let view = unsafe { MapViewOfFile(handle, FILE_MAP_WRITE, 0, 0, 0) };
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
                stage_identity: 0,
                previous_stage_identity: 0,
                stage_change_sequence: 0,
                stage_from_previous_valid: 0,
                stage_from_previous: to_matrix(Pose::IDENTITY),
                anchor_state: 0,
                anchor_request_served: 0,
                anchor_uuid: [0; 16],
                anchor_pose: to_matrix(Pose::IDENTITY),
                anchor_located_ms: 0,
                anchor_generation: 0,
                reserved_v3: 0,
                product_name: [0; PRODUCT_NAME_BYTES],
                hands: [RawHand::default(); 2],
            };
            let shape = self.hand_shape;
            let hands = std::array::from_fn(|idx| {
                let mut hand = RawHand::default();
                if let Some((sample_ms, joints)) = self.hands[idx] {
                    hand.valid = u32::from(now.saturating_sub(sample_ms) <= FINGERTIP_STALE_MS);
                    hand.sample_ms = sample_ms;
                    hand.joints = PUBLISHED_HAND_JOINTS.map(|joint| to_matrix(joints[joint]));
                }
                if let Some(shape) = shape {
                    hand.draw_thickness = shape.thickness;
                    hand.draw_scale = shape.scale;
                    hand.draw_fingers = shape.fingers;
                    hand.draw_finger_correction = shape.finger_correction[idx];
                    hand.draw_correction_sequence = shape.correction_sequence;
                }
                hand
            });
            let block = RawExtrasBlock { hands, ..block };
            let block = match &self.stage_info {
                Some(info) => RawExtrasBlock {
                    stage_identity: info.stage_identity,
                    previous_stage_identity: info.previous_stage_identity,
                    stage_change_sequence: info.stage_change_sequence,
                    stage_from_previous_valid: u32::from(info.stage_from_previous.is_some()),
                    stage_from_previous: to_matrix(
                        info.stage_from_previous.unwrap_or(Pose::IDENTITY),
                    ),
                    // No fresh report (client asleep or disconnected): the last pose is only the
                    // last good one
                    anchor_state: if info.anchor_state == ANCHOR_STATE_LOCATED
                        && now.saturating_sub(self.anchor_located_ms) > ANCHOR_LOCATED_STALE_MS
                    {
                        ANCHOR_STATE_LOST
                    } else {
                        info.anchor_state
                    },
                    anchor_request_served: info.anchor_request_served,
                    anchor_uuid: info.anchor_uuid,
                    anchor_pose: to_matrix(info.anchor_pose.unwrap_or(Pose::IDENTITY)),
                    anchor_located_ms: self.anchor_located_ms,
                    anchor_generation: info.anchor_generation,
                    product_name: product_name_bytes(&info.product_name),
                    ..block
                },
                None => block,
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
                    // The whole section: a version 1 overlay's is 32 bytes (page-granular view)
                    let view = unsafe { MapViewOfFile(handle, FILE_MAP_READ, 0, 0, 0) };
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
            let mut block = block?;
            // A version 1 overlay's bytes after the first 32 are not part of its block
            if block.version < REQUEST_VERSION_ANCHOR {
                block.anchor_request = 0;
                block.anchor_request_sequence = 0;
            }
            if block.version < REQUEST_VERSION_HAND_DISPLAY {
                block.hand_display_sequence = 0;
                block.hand_display_flags = 0;
            }
            if block.version < REQUEST_VERSION_HAND_CORRECTION {
                block.hand_finger_scale = [1.0; 2];
                block.hand_correction_sequence = 0;
                block.hand_near_meters = 0.0;
                block.hand_correction_plane = to_matrix(Pose::IDENTITY);
            }

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
        fn matrix_round_trip_and_product_name() {
            let pose = Pose {
                orientation: Quat::from_euler(alvr_common::glam::EulerRot::YXZ, 0.3, -0.2, 0.1),
                position: Vec3::new(-1.0, 0.5, 2.0),
            };
            let back = from_matrix(to_matrix(pose));
            assert!(back.position.distance(pose.position) < 1e-6);
            assert!(back.orientation.angle_between(pose.orientation) < 1e-5);

            assert_eq!(&product_name_bytes("A9210")[..6], b"A9210\0");
            let long = "\u{e9}".repeat(20); // 40 bytes, 2 per character
            let bytes = product_name_bytes(&long);
            assert_eq!(bytes[30], 0); // 15 whole characters, then NUL
            assert_eq!(bytes[31], 0);
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
