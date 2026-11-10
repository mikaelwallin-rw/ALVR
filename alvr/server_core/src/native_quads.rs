// Reader for debug quads that an overlay app (ScreenVR) asks the client to draw natively, as
// OpenXR quad layers composited by the headset runtime. Used as a reference to compare the
// streamed overlays with: native quads at the true calibrated monitor poses, drawn without the
// streaming path.
//
// Contract (source of truth: ScreenVR `src/VrNativeQuads.h`, copied here, no dependency):
// - Named file mapping `Local\VRNativeQuads_v1`, `VrNativeQuadsBlock` layout, pack(4), written by
//   the overlay app.
// - Seqlock: odd `sequence` = write in progress. A copy is accepted only if `sequence` was even and
//   unchanged before and after the copy.
// - `heartbeat_ms` (GetTickCount64) is refreshed at least every 100 ms. Older than 1000 ms = no
//   quads.
// - Poses are in the client STAGE space of `stage_epoch` (see tracking_extras.rs). Quads of
//   another epoch are not drawn.
// - Version 1: 80-byte quads. Version 2: 96-byte quads (+ videoStreamId, reserved[3]) and the
//   block's reserved field becomes `flags` (scene mode). Both are accepted, by the version field.

#[cfg(windows)]
pub use windows_impl::NativeQuadsReader;

#[cfg(not(windows))]
use alvr_packets::NativeQuad;

#[cfg(not(windows))]
pub struct NativeQuadsReader;

#[cfg(not(windows))]
impl NativeQuadsReader {
    pub const fn new() -> Self {
        Self
    }

    pub fn read(&mut self, _current_stage_epoch: u32) -> Vec<NativeQuad> {
        Vec::new()
    }

    pub fn scene_mode(&self) -> bool {
        false
    }

    pub fn stream_unused(&self) -> bool {
        false
    }

    pub fn scene_loading(&self) -> bool {
        false
    }
}

#[cfg(windows)]
mod windows_impl {
    use alvr_common::{
        Pose,
        glam::{Mat3, Quat, Vec3},
        info, warn,
    };
    use alvr_packets::NativeQuad;
    use std::{
        mem, ptr,
        sync::atomic::{self, AtomicU32, Ordering},
        time::{Duration, Instant},
    };
    use windows::{
        Win32::{
            Foundation::{CloseHandle, HANDLE},
            System::{
                Memory::{
                    FILE_MAP_READ, MEMORY_MAPPED_VIEW_ADDRESS, MapViewOfFile, OpenFileMappingW,
                    UnmapViewOfFile,
                },
                SystemInformation::GetTickCount64,
            },
        },
        core::w,
    };

    const MAGIC: u32 = 0x4441_514E; // 'NQAD' little-endian
    const VERSION_1: u32 = 1;
    const VERSION_2: u32 = 2;
    const MAX_QUADS: usize = 16;
    const MAX_SQUARES: u32 = 256;
    const STALE_MS: u64 = 1000;
    const OPEN_RETRY_INTERVAL: Duration = Duration::from_secs(1);
    const MAX_READ_ATTEMPTS: usize = 4;
    const SEQUENCE_OFFSET: usize = 8;
    const VERSION_OFFSET: usize = 4;
    // v2 block flag: a scene app (V3, full VR) is presenting
    const FLAG_SCENE_MODE: u32 = 1;
    // v2 block flag: nothing of the overlay app's is in the stream (all screens are native quads, no
    // overlay, dashboard or scene app): the client may skip rendering the stream
    const FLAG_STREAM_UNUSED: u32 = 2;
    // v2 block flag (kVrNativeQuadsFlagSceneLoading): a scene app is starting (until it presents) or
    // shutting down (until the front end is restored): the client shows its loading screen
    const FLAG_SCENE_LOADING: u32 = 4;

    #[repr(C, packed(4))]
    #[derive(Clone, Copy)]
    struct RawQuad {
        stage_from_quad: [[f32; 4]; 3], // HmdMatrix34_t, row-major
        width_meters: f32,
        height_meters: f32,
        columns: u32,
        rows: u32,
        dark_rgba: u32,
        light_rgba: u32,
        outline_rgba: u32,
        outline_pixels: u32,
    }

    #[repr(C, packed(4))]
    #[derive(Clone, Copy)]
    struct RawQuadV2 {
        v1: RawQuad,
        video_stream_id: u32,
        reserved: [u32; 3],
    }

    #[repr(C, packed(4))]
    #[derive(Clone, Copy)]
    struct RawHeader {
        magic: u32,
        version: u32,
        sequence: u32,
        count: u32,
        heartbeat_ms: u64,
        stage_epoch: u32,
        // v1: reserved
        flags: u32,
    }

    #[repr(C, packed(4))]
    #[derive(Clone, Copy)]
    struct RawBlockV1 {
        header: RawHeader,
        quads: [RawQuad; MAX_QUADS],
    }

    #[repr(C, packed(4))]
    #[derive(Clone, Copy)]
    struct RawBlockV2 {
        header: RawHeader,
        quads: [RawQuadV2; MAX_QUADS],
    }

    const _: () = assert!(mem::size_of::<RawQuad>() == 80);
    const _: () = assert!(mem::size_of::<RawQuadV2>() == 96);
    const _: () = assert!(mem::size_of::<RawHeader>() == 32);
    const _: () = assert!(mem::size_of::<RawBlockV1>() == 32 + 80 * MAX_QUADS);
    const _: () = assert!(mem::size_of::<RawBlockV2>() == 32 + 96 * MAX_QUADS);

    // A block of either version, with the quads as (v1 fields, video stream id)
    struct Block {
        header: RawHeader,
        quads: Vec<(RawQuad, u32)>,
    }

    // Reader state, logged on change
    #[derive(Clone, Copy, PartialEq, Debug)]
    enum Status {
        NotOpen,
        InvalidHeader {
            magic: u32,
            version: u32,
        },
        Stale,
        OtherStageEpoch {
            block: u32,
            current: u32,
        },
        Active {
            version: u32,
            count: usize,
            scene_mode: bool,
        },
    }

    struct Mapping {
        handle: HANDLE,
        view: MEMORY_MAPPED_VIEW_ADDRESS,
    }

    // The mapping is only accessed from behind a Mutex or by a single thread
    unsafe impl Send for Mapping {}

    impl Drop for Mapping {
        fn drop(&mut self) {
            unsafe {
                UnmapViewOfFile(self.view).ok();
                CloseHandle(self.handle).ok();
            }
        }
    }

    fn color(rgba: u32) -> [u8; 4] {
        // Bytes in memory R, G, B, A (little-endian u32)
        rgba.to_le_bytes()
    }

    fn parse_quad(raw: &RawQuad, video_stream_id: u32) -> Option<NativeQuad> {
        let m = raw.stage_from_quad;
        let (width, height, columns, rows) =
            (raw.width_meters, raw.height_meters, raw.columns, raw.rows);
        let values_ok = m.iter().flatten().all(|v| v.is_finite())
            && width.is_finite()
            && height.is_finite()
            && width > 0.0
            && height > 0.0
            && (1..=MAX_SQUARES).contains(&columns)
            && (1..=MAX_SQUARES).contains(&rows);
        if !values_ok {
            return None;
        }

        let rotation = Mat3::from_cols(
            Vec3::new(m[0][0], m[1][0], m[2][0]),
            Vec3::new(m[0][1], m[1][1], m[2][1]),
            Vec3::new(m[0][2], m[1][2], m[2][2]),
        );
        let orientation = Quat::from_mat3(&rotation).normalize();
        if !orientation.is_finite() {
            return None;
        }

        Some(NativeQuad {
            pose: Pose {
                orientation,
                position: Vec3::new(m[0][3], m[1][3], m[2][3]),
            },
            width,
            height,
            columns,
            rows,
            dark_color: color(raw.dark_rgba),
            light_color: color(raw.light_rgba),
            outline_color: color(raw.outline_rgba),
            outline_pixels: raw.outline_pixels,
            video_stream_id,
        })
    }

    pub struct NativeQuadsReader {
        mapping: Option<Mapping>,
        last_open_attempt: Option<Instant>,
        status: Status,
        // Result of the last successful read, kept when the writer is busy during a read
        last_quads: Vec<NativeQuad>,
        // The writer's block is valid and fresh and has the scene mode flag (any STAGE epoch)
        scene_mode: bool,
        // The writer's block is valid, fresh, of the current STAGE epoch and has the stream unused
        // flag
        stream_unused: bool,
        // The writer's block is valid and fresh and has the scene loading flag (any STAGE epoch)
        scene_loading: bool,
    }

    impl NativeQuadsReader {
        pub const fn new() -> Self {
            Self {
                mapping: None,
                last_open_attempt: None,
                status: Status::NotOpen,
                last_quads: Vec::new(),
                scene_mode: false,
                stream_unused: false,
                scene_loading: false,
            }
        }

        /// A scene app is starting or shutting down: the client shows its loading screen
        pub fn scene_loading(&self) -> bool {
            self.scene_loading
        }

        /// Nothing of the overlay app's is in the stream: the client may skip rendering it
        pub fn stream_unused(&self) -> bool {
            self.stream_unused
        }

        /// A scene app (V3, full VR) is presenting: passthrough and chroma key must be off
        pub fn scene_mode(&self) -> bool {
            self.scene_mode
        }

        fn set_status(&mut self, status: Status) {
            if status != self.status {
                info!("Native quads: {status:?} (was {:?})", self.status);
                self.status = status;
            }
        }

        fn try_open(&mut self) {
            if self
                .last_open_attempt
                .is_some_and(|instant| instant.elapsed() < OPEN_RETRY_INTERVAL)
            {
                return;
            }
            self.last_open_attempt = Some(Instant::now());

            // Not found = the overlay app does not publish quads (the normal case)
            let Ok(handle) = (unsafe {
                OpenFileMappingW(FILE_MAP_READ.0, false, w!("Local\\VRNativeQuads_v1"))
            }) else {
                return;
            };

            // The whole section: its size depends on the writer's version
            let view = unsafe { MapViewOfFile(handle, FILE_MAP_READ, 0, 0, 0) };
            if view.Value.is_null() {
                warn!("Native quads: MapViewOfFile failed");
                unsafe { CloseHandle(handle).ok() };
                return;
            }

            // The mapping is kept open from now on: as long as this handle exists, a restarted
            // writer opens the same named section again
            info!("Native quads: shared memory Local\\VRNativeQuads_v1 opened");
            self.mapping = Some(Mapping { handle, view });
        }

        // Seqlock read. Returns None if the writer kept writing during all attempts. For an
        // unknown version, only the header is read (for the status).
        fn read_block(mapping: &Mapping) -> Option<Block> {
            let base = mapping.view.Value.cast::<u8>();
            // # Safety: the view covers the writer's whole section, which the writer creates with
            // the size of its version, and `sequence`/`version` are 4-byte aligned (pack(4),
            // page-aligned view)
            let sequence = unsafe { &*base.add(SEQUENCE_OFFSET).cast::<AtomicU32>() };
            let version = unsafe { &*base.add(VERSION_OFFSET).cast::<AtomicU32>() };

            for _ in 0..MAX_READ_ATTEMPTS {
                let before = sequence.load(Ordering::Acquire);
                if before % 2 != 0 {
                    std::hint::spin_loop();
                    continue;
                }

                let block = match version.load(Ordering::Acquire) {
                    VERSION_1 => {
                        let raw = unsafe { ptr::read_volatile(base.cast::<RawBlockV1>()) };
                        Block {
                            header: raw.header,
                            quads: raw.quads.iter().map(|quad| (*quad, 0)).collect(),
                        }
                    }
                    VERSION_2 => {
                        let raw = unsafe { ptr::read_volatile(base.cast::<RawBlockV2>()) };
                        Block {
                            header: raw.header,
                            quads: raw
                                .quads
                                .iter()
                                .map(|quad| (quad.v1, quad.video_stream_id))
                                .collect(),
                        }
                    }
                    _ => Block {
                        header: unsafe { ptr::read_volatile(base.cast::<RawHeader>()) },
                        quads: Vec::new(),
                    },
                };

                atomic::fence(Ordering::Acquire);
                if sequence.load(Ordering::Relaxed) == before {
                    return Some(block);
                }
            }

            None
        }

        /// Quads to draw, in client STAGE space. Empty if the writer is missing, stale or wrote
        /// for another STAGE epoch. Also updates scene_mode().
        pub fn read(&mut self, current_stage_epoch: u32) -> Vec<NativeQuad> {
            if self.mapping.is_none() {
                self.try_open();
            }
            let Some(mapping) = &self.mapping else {
                self.set_status(Status::NotOpen);
                self.scene_mode = false;
                self.stream_unused = false;
                self.scene_loading = false;
                return Vec::new();
            };
            let Some(block) = Self::read_block(mapping) else {
                // The writer kept writing during all attempts: nothing changed for the client
                return self.last_quads.clone();
            };

            let quads = self.parse_block(&block, current_stage_epoch);
            self.last_quads.clone_from(&quads);

            quads
        }

        fn parse_block(&mut self, block: &Block, current_stage_epoch: u32) -> Vec<NativeQuad> {
            self.scene_mode = false;
            self.stream_unused = false;
            self.scene_loading = false;

            let header = block.header;
            let (magic, version, count, heartbeat_ms, stage_epoch, flags) = (
                header.magic,
                header.version,
                header.count,
                header.heartbeat_ms,
                header.stage_epoch,
                header.flags,
            );
            if magic != MAGIC || !matches!(version, VERSION_1 | VERSION_2) {
                self.set_status(Status::InvalidHeader { magic, version });
                return Vec::new();
            }
            if unsafe { GetTickCount64() }.saturating_sub(heartbeat_ms) > STALE_MS {
                self.set_status(Status::Stale);
                return Vec::new();
            }
            // Independent of the STAGE epoch: the scene app presents whether or not the quads
            // are calibrated
            self.scene_mode = version >= VERSION_2 && flags & FLAG_SCENE_MODE != 0;
            self.scene_loading = version >= VERSION_2 && flags & FLAG_SCENE_LOADING != 0;
            if stage_epoch != current_stage_epoch {
                self.set_status(Status::OtherStageEpoch {
                    block: stage_epoch,
                    current: current_stage_epoch,
                });
                return Vec::new();
            }

            // Only for the current epoch: the quads that replace the stream are drawn
            self.stream_unused = version >= VERSION_2 && flags & FLAG_STREAM_UNUSED != 0;

            let quads = block
                .quads
                .iter()
                .take((count as usize).min(MAX_QUADS))
                .filter_map(|(quad, video_stream_id)| parse_quad(quad, *video_stream_id))
                .collect::<Vec<_>>();
            self.set_status(Status::Active {
                version,
                count: quads.len(),
                scene_mode: self.scene_mode,
            });

            quads
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn parses_valid_quad_and_rejects_bad_ones() {
            let raw = RawQuad {
                stage_from_quad: [
                    [1.0, 0.0, 0.0, 0.5],
                    [0.0, 1.0, 0.0, 1.2],
                    [0.0, 0.0, 1.0, -0.8],
                ],
                width_meters: 0.216,
                height_meters: 0.136,
                columns: 16,
                rows: 10,
                dark_rgba: 0xFF00_0000,
                light_rgba: 0,
                outline_rgba: 0xFF00_FF00,
                outline_pixels: 2,
            };
            let quad = parse_quad(&raw, 7).unwrap();
            assert_eq!(quad.pose.position, Vec3::new(0.5, 1.2, -0.8));
            assert_eq!(quad.dark_color, [0, 0, 0, 255]);
            assert_eq!(quad.outline_color, [0, 255, 0, 255]);
            assert_eq!(quad.video_stream_id, 7);

            assert!(parse_quad(&RawQuad { columns: 0, ..raw }, 0).is_none());
            assert!(
                parse_quad(
                    &RawQuad {
                        width_meters: f32::NAN,
                        ..raw
                    },
                    0
                )
                .is_none()
            );
        }

        fn block(version: u32, flags: u32, age_ms: u64, stage_epoch: u32) -> Block {
            Block {
                header: RawHeader {
                    magic: MAGIC,
                    version,
                    sequence: 2,
                    count: 0,
                    heartbeat_ms: unsafe { GetTickCount64() }.saturating_sub(age_ms),
                    stage_epoch,
                    flags,
                },
                quads: Vec::new(),
            }
        }

        #[test]
        fn scene_loading_needs_a_fresh_v2_block_of_any_epoch() {
            let mut reader = NativeQuadsReader::new();
            let mut scene_loading = |block: Block| {
                reader.parse_block(&block, 3);
                reader.scene_loading()
            };

            assert!(scene_loading(block(VERSION_2, FLAG_SCENE_LOADING, 0, 3)));
            // Starting before the quads are calibrated for this epoch
            assert!(scene_loading(block(VERSION_2, FLAG_SCENE_LOADING, 0, 9)));
            assert!(scene_loading(block(
                VERSION_2,
                FLAG_SCENE_LOADING | FLAG_SCENE_MODE,
                0,
                3
            )));
            assert!(!scene_loading(block(VERSION_2, FLAG_SCENE_MODE, 0, 3)));
            assert!(!scene_loading(block(VERSION_1, FLAG_SCENE_LOADING, 0, 3)));
            assert!(!scene_loading(block(
                VERSION_2,
                FLAG_SCENE_LOADING,
                STALE_MS + 500,
                3
            )));
        }

        #[test]
        fn stream_unused_needs_a_fresh_v2_block_of_the_current_epoch() {
            let mut reader = NativeQuadsReader::new();
            let mut stream_unused = |block: Block| {
                reader.parse_block(&block, 3);
                reader.stream_unused()
            };

            assert!(stream_unused(block(VERSION_2, FLAG_STREAM_UNUSED, 0, 3)));
            assert!(stream_unused(block(
                VERSION_2,
                FLAG_STREAM_UNUSED | 0x100,
                0,
                3
            )));
            assert!(!stream_unused(block(VERSION_2, FLAG_STREAM_UNUSED, 0, 9)));
            assert!(!stream_unused(block(VERSION_2, FLAG_SCENE_MODE, 0, 3)));
            assert!(!stream_unused(block(VERSION_1, FLAG_STREAM_UNUSED, 0, 3)));
            assert!(!stream_unused(block(
                VERSION_2,
                FLAG_STREAM_UNUSED,
                STALE_MS + 500,
                3
            )));
        }

        #[test]
        fn scene_mode_needs_a_fresh_v2_block_with_the_flag() {
            let mut reader = NativeQuadsReader::new();
            let mut scene_mode = |block: Block| {
                reader.parse_block(&block, 3);
                reader.scene_mode()
            };

            assert!(scene_mode(block(VERSION_2, FLAG_SCENE_MODE, 0, 3)));
            // Any STAGE epoch: the scene app presents whether or not the quads are calibrated
            assert!(scene_mode(block(VERSION_2, FLAG_SCENE_MODE, 0, 9)));
            assert!(!scene_mode(block(VERSION_2, 0, 0, 3)));
            // v1: the field is reserved
            assert!(!scene_mode(block(VERSION_1, FLAG_SCENE_MODE, 0, 3)));
            // The writer crashed or hangs: not scene mode
            assert!(!scene_mode(block(
                VERSION_2,
                FLAG_SCENE_MODE,
                STALE_MS + 500,
                3
            )));
            assert!(!scene_mode(Block {
                header: RawHeader {
                    magic: 0,
                    ..block(VERSION_2, FLAG_SCENE_MODE, 0, 3).header
                },
                quads: Vec::new(),
            }));
        }
    }
}
