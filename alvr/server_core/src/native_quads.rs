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
    const VERSION: u32 = 1;
    const MAX_QUADS: usize = 16;
    const MAX_SQUARES: u32 = 256;
    const STALE_MS: u64 = 1000;
    const OPEN_RETRY_INTERVAL: Duration = Duration::from_secs(1);
    const MAX_READ_ATTEMPTS: usize = 4;
    const SEQUENCE_OFFSET: usize = 8;

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
    struct RawBlock {
        magic: u32,
        version: u32,
        sequence: u32,
        count: u32,
        heartbeat_ms: u64,
        stage_epoch: u32,
        reserved: u32,
        quads: [RawQuad; MAX_QUADS],
    }

    const _: () = assert!(mem::size_of::<RawQuad>() == 80);
    const _: () = assert!(mem::size_of::<RawBlock>() == 32 + 80 * MAX_QUADS);

    // Reader state, logged on change
    #[derive(Clone, Copy, PartialEq, Debug)]
    enum Status {
        NotOpen,
        InvalidHeader { magic: u32, version: u32 },
        Stale,
        OtherStageEpoch { block: u32, current: u32 },
        Active { count: usize },
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

    fn parse_quad(raw: &RawQuad) -> Option<NativeQuad> {
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
        })
    }

    pub struct NativeQuadsReader {
        mapping: Option<Mapping>,
        last_open_attempt: Option<Instant>,
        status: Status,
    }

    impl NativeQuadsReader {
        pub const fn new() -> Self {
            Self {
                mapping: None,
                last_open_attempt: None,
                status: Status::NotOpen,
            }
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

            let view =
                unsafe { MapViewOfFile(handle, FILE_MAP_READ, 0, 0, mem::size_of::<RawBlock>()) };
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

        // Seqlock read. Returns None if the writer kept writing during all attempts.
        fn read_block(mapping: &Mapping) -> Option<RawBlock> {
            let base = mapping.view.Value.cast::<u8>();
            // # Safety: the view is at least size_of::<RawBlock>() bytes and `sequence` is 4-byte
            // aligned (pack(4), offset 8, page-aligned view)
            let sequence = unsafe { &*base.add(SEQUENCE_OFFSET).cast::<AtomicU32>() };

            for _ in 0..MAX_READ_ATTEMPTS {
                let before = sequence.load(Ordering::Acquire);
                if before % 2 != 0 {
                    std::hint::spin_loop();
                    continue;
                }

                let block = unsafe { ptr::read_volatile(base.cast::<RawBlock>()) };

                atomic::fence(Ordering::Acquire);
                if sequence.load(Ordering::Relaxed) == before {
                    return Some(block);
                }
            }

            None
        }

        /// Quads to draw, in client STAGE space. Empty if the writer is missing, stale or wrote
        /// for another STAGE epoch.
        pub fn read(&mut self, current_stage_epoch: u32) -> Vec<NativeQuad> {
            if self.mapping.is_none() {
                self.try_open();
            }
            let Some(block) = self.mapping.as_ref().and_then(Self::read_block) else {
                if self.mapping.is_none() {
                    self.set_status(Status::NotOpen);
                }
                return Vec::new();
            };

            let (magic, version, count, heartbeat_ms, stage_epoch) = (
                block.magic,
                block.version,
                block.count,
                block.heartbeat_ms,
                block.stage_epoch,
            );
            if magic != MAGIC || version != VERSION {
                self.set_status(Status::InvalidHeader { magic, version });
                return Vec::new();
            }
            if unsafe { GetTickCount64() }.saturating_sub(heartbeat_ms) > STALE_MS {
                self.set_status(Status::Stale);
                return Vec::new();
            }
            if stage_epoch != current_stage_epoch {
                self.set_status(Status::OtherStageEpoch {
                    block: stage_epoch,
                    current: current_stage_epoch,
                });
                return Vec::new();
            }

            let raw_quads = block.quads;
            let quads = raw_quads
                .iter()
                .take((count as usize).min(MAX_QUADS))
                .filter_map(parse_quad)
                .collect::<Vec<_>>();
            self.set_status(Status::Active { count: quads.len() });

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
            let quad = parse_quad(&raw).unwrap();
            assert_eq!(quad.pose.position, Vec3::new(0.5, 1.2, -0.8));
            assert_eq!(quad.dark_color, [0, 0, 0, 255]);
            assert_eq!(quad.outline_color, [0, 255, 0, 255]);

            assert!(parse_quad(&RawQuad { columns: 0, ..raw }).is_none());
            assert!(
                parse_quad(&RawQuad {
                    width_meters: f32::NAN,
                    ..raw
                })
                .is_none()
            );
        }
    }
}
