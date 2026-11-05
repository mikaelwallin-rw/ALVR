// Reader for the "keep these regions opaque" hints published by an overlay app (ScreenVR) through
// shared memory. The regions are forwarded to the client with each video frame, so that chroma
// keying never removes them.
//
// Contract (source of truth: ScreenVR `src/VrOpaqueRegions.h`, copied here, no dependency):
// - Named file mapping `Local\VROpaqueRegions_v1`, `VrOpaqueRegionsBlock` layout, pack(4).
// - Seqlock: odd `sequence` = write in progress. A copy is accepted only if `sequence` was even and
//   unchanged before and after the copy.
// - `heartbeat_ms` (GetTickCount64) is refreshed at least every 100 ms. Older than 1000 ms = no
//   regions.
// - Poses are in TrackingUniverseRawAndUncalibrated space, which is the OpenVR driver pose space
//   (the HMD pose is submitted with an identity world-from-driver transform).

#[cfg(windows)]
pub use windows_impl::OpaqueRegionsReader;

#[cfg(not(windows))]
use alvr_packets::OpaqueRegion;

#[cfg(not(windows))]
pub struct OpaqueRegionsReader;

#[cfg(not(windows))]
impl OpaqueRegionsReader {
    pub const fn new() -> Self {
        Self
    }

    pub fn read(&mut self) -> Vec<OpaqueRegion> {
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
    use alvr_packets::OpaqueRegion;
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

    const MAGIC: u32 = 0x5251_504F; // 'OPQR' little-endian
    const VERSION: u32 = 1;
    const MAX_REGIONS: usize = 32;
    const STALE_MS: u64 = 1000;
    const SPACE_RAW_AND_UNCALIBRATED: u32 = 0;
    const OPEN_RETRY_INTERVAL: Duration = Duration::from_secs(1);
    const MAX_READ_ATTEMPTS: usize = 4;
    // An invalid or stale block for this long drops the mapping, so that a restarted writer is
    // picked up again
    const REOPEN_AFTER_INVALID: Duration = Duration::from_secs(3);

    // Reader state, logged on change
    #[derive(Clone, Copy, PartialEq, Debug)]
    enum Status {
        NotOpen,
        InvalidHeader {
            magic: u32,
            version: u32,
            space: u32,
        },
        Stale {
            age_ms: u64,
        },
        Active {
            raw_count: u32,
            valid_count: usize,
        },
    }

    impl Status {
        // Stale ages change every read: compare kinds only
        fn same_kind(self, other: Self) -> bool {
            match (self, other) {
                (Self::Stale { .. }, Self::Stale { .. }) => true,
                _ => self == other,
            }
        }
    }

    #[repr(C, packed(4))]
    #[derive(Clone, Copy)]
    struct RawRegion {
        pose: [[f32; 4]; 3], // HmdMatrix34_t, row-major
        width_meters: f32,
        height_meters: f32,
        curvature: f32,
        inset_meters: f32,
    }

    #[repr(C, packed(4))]
    #[derive(Clone, Copy)]
    struct RawBlock {
        magic: u32,
        version: u32,
        sequence: u32,
        count: u32,
        heartbeat_ms: u64,
        space: u32,
        reserved: u32,
        regions: [RawRegion; MAX_REGIONS],
    }

    const _: () = assert!(mem::size_of::<RawRegion>() == 64);
    const _: () = assert!(mem::size_of::<RawBlock>() == 32 + 64 * MAX_REGIONS);
    const SEQUENCE_OFFSET: usize = 8;

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

    pub struct OpaqueRegionsReader {
        mapping: Option<Mapping>,
        last_open_attempt: Option<Instant>,
        last_open_error: Option<i32>,
        last_regions: Vec<OpaqueRegion>,
        status: Status,
        invalid_since: Option<Instant>,
    }

    impl OpaqueRegionsReader {
        pub const fn new() -> Self {
            Self {
                mapping: None,
                last_open_attempt: None,
                last_open_error: None,
                last_regions: Vec::new(),
                status: Status::NotOpen,
                invalid_since: None,
            }
        }

        fn set_status(&mut self, status: Status) {
            if !status.same_kind(self.status) {
                info!("Opaque regions: {:?} (was {:?})", status, self.status);
            }
            self.status = status;
        }

        fn try_open(&mut self) {
            if self
                .last_open_attempt
                .is_some_and(|instant| instant.elapsed() < OPEN_RETRY_INTERVAL)
            {
                return;
            }
            self.last_open_attempt = Some(Instant::now());

            let handle = match unsafe {
                OpenFileMappingW(FILE_MAP_READ.0, false, w!("Local\\VROpaqueRegions_v1"))
            } {
                Ok(handle) => handle,
                Err(e) => {
                    // Logged once per distinct error (not found = writer not running yet)
                    let code = e.code().0;
                    if self.last_open_error != Some(code) {
                        info!("Opaque regions: OpenFileMappingW failed ({code:#010x}): {e}");
                        self.last_open_error = Some(code);
                    }
                    return;
                }
            };
            self.last_open_error = None;

            let view =
                unsafe { MapViewOfFile(handle, FILE_MAP_READ, 0, 0, mem::size_of::<RawBlock>()) };
            if view.Value.is_null() {
                warn!("Opaque regions: MapViewOfFile failed");
                unsafe { CloseHandle(handle).ok() };
                return;
            }

            info!("Opaque regions: shared memory Local\\VROpaqueRegions_v1 opened");
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

        fn parse_block(block: &RawBlock) -> Option<Vec<OpaqueRegion>> {
            let (magic, version, space, count, heartbeat_ms) = (
                block.magic,
                block.version,
                block.space,
                block.count,
                block.heartbeat_ms,
            );
            if magic != MAGIC || version != VERSION || space != SPACE_RAW_AND_UNCALIBRATED {
                return None;
            }

            let now_ms = unsafe { GetTickCount64() };
            if now_ms.saturating_sub(heartbeat_ms) > STALE_MS {
                return Some(Vec::new());
            }

            let regions = block.regions;
            let regions = regions
                .iter()
                .take((count as usize).min(MAX_REGIONS))
                .filter_map(|region| {
                    let m = region.pose;
                    let (width, height, curvature, inset) = (
                        region.width_meters,
                        region.height_meters,
                        region.curvature,
                        region.inset_meters,
                    );
                    let values_ok = m.iter().flatten().all(|v| v.is_finite())
                        && width.is_finite()
                        && height.is_finite()
                        && curvature.is_finite()
                        && inset.is_finite()
                        && width > 0.0
                        && height > 0.0
                        && curvature >= 0.0;
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

                    Some(OpaqueRegion {
                        pose: Pose {
                            orientation,
                            position: Vec3::new(m[0][3], m[1][3], m[2][3]),
                        },
                        width,
                        height,
                        curvature,
                        inset: inset.max(0.0),
                    })
                })
                .collect();

            Some(regions)
        }

        /// Current regions in OpenVR driver space. Empty if the writer is missing or stale.
        pub fn read(&mut self) -> Vec<OpaqueRegion> {
            if self.mapping.is_none() {
                self.try_open();
            }
            let Some(mapping) = &self.mapping else {
                self.set_status(Status::NotOpen);
                self.last_regions.clear();
                return Vec::new();
            };

            if let Some(block) = Self::read_block(mapping) {
                let (magic, version, space, raw_count, heartbeat_ms) = (
                    block.magic,
                    block.version,
                    block.space,
                    block.count,
                    block.heartbeat_ms,
                );
                // An invalid header (e.g. writer not initialized yet) means no regions
                self.last_regions = Self::parse_block(&block).unwrap_or_default();

                let age_ms = unsafe { GetTickCount64() }.saturating_sub(heartbeat_ms);
                let status = if magic != MAGIC
                    || version != VERSION
                    || space != SPACE_RAW_AND_UNCALIBRATED
                {
                    Status::InvalidHeader {
                        magic,
                        version,
                        space,
                    }
                } else if age_ms > STALE_MS {
                    Status::Stale { age_ms }
                } else {
                    Status::Active {
                        raw_count,
                        valid_count: self.last_regions.len(),
                    }
                };
                self.set_status(status);

                if matches!(status, Status::Active { .. }) {
                    self.invalid_since = None;
                } else {
                    let since = *self.invalid_since.get_or_insert_with(Instant::now);
                    if since.elapsed() > REOPEN_AFTER_INVALID {
                        info!(
                            "Opaque regions: no valid data for {REOPEN_AFTER_INVALID:?}, reopening the mapping"
                        );
                        self.mapping = None;
                        self.invalid_since = None;
                    }
                }
            }
            // else: writer busy during all attempts, keep the previous copy

            self.last_regions.clone()
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        fn block_with(regions: &[RawRegion], heartbeat_ms: u64) -> RawBlock {
            let mut block = RawBlock {
                magic: MAGIC,
                version: VERSION,
                sequence: 2,
                count: regions.len() as u32,
                heartbeat_ms,
                space: SPACE_RAW_AND_UNCALIBRATED,
                reserved: 0,
                regions: [RawRegion {
                    pose: [[0.0; 4]; 3],
                    width_meters: 0.0,
                    height_meters: 0.0,
                    curvature: 0.0,
                    inset_meters: 0.0,
                }; MAX_REGIONS],
            };
            block.regions[..regions.len()].copy_from_slice(regions);
            block
        }

        #[test]
        fn parses_pose_and_rejects_stale_or_invalid() {
            // 90 deg rotation around Y: local +Z maps to world +X
            let region = RawRegion {
                pose: [
                    [0.0, 0.0, 1.0, 1.0],
                    [0.0, 1.0, 0.0, 1.44],
                    [-1.0, 0.0, 0.0, -1.25],
                ],
                width_meters: 1.344,
                height_meters: 0.84,
                curvature: 0.0494,
                inset_meters: 0.01,
            };
            let bad = RawRegion {
                width_meters: f32::NAN,
                ..region
            };
            let now = unsafe { GetTickCount64() };

            let regions = OpaqueRegionsReader::parse_block(&block_with(&[region, bad], now))
                .expect("valid header");
            assert_eq!(regions.len(), 1);
            let parsed = regions[0];
            assert!((parsed.pose.position - Vec3::new(1.0, 1.44, -1.25)).length() < 1e-6);
            assert!((parsed.pose.orientation * Vec3::Z - Vec3::X).length() < 1e-5);
            assert!((parsed.width - 1.344).abs() < 1e-6);

            let stale = block_with(&[region], now.saturating_sub(STALE_MS + 500));
            assert!(OpaqueRegionsReader::parse_block(&stale).unwrap().is_empty());

            let mut wrong_magic = block_with(&[region], now);
            wrong_magic.magic = 0;
            assert!(OpaqueRegionsReader::parse_block(&wrong_magic).is_none());
        }

        // Reads the real mapping published by ScreenVR, read-only. Run with:
        // cargo test -p alvr_server_core read_live_mapping -- --ignored --nocapture
        #[test]
        #[ignore]
        fn read_live_mapping() {
            let mut reader = OpaqueRegionsReader::new();
            let regions = reader.read();
            println!("mapping open: {}", reader.mapping.is_some());
            if let Some(block) = reader
                .mapping
                .as_ref()
                .and_then(OpaqueRegionsReader::read_block)
            {
                let (magic, version, sequence, count, heartbeat_ms, space) = (
                    block.magic,
                    block.version,
                    block.sequence,
                    block.count,
                    block.heartbeat_ms,
                    block.space,
                );
                let age_ms = unsafe { GetTickCount64() }.saturating_sub(heartbeat_ms);
                println!(
                    "magic {magic:#x} version {version} sequence {sequence} count {count} \
                     heartbeat age {age_ms} ms space {space}"
                );

                // Layout check on the local copy only, ignoring staleness
                let mut fresh = block;
                fresh.heartbeat_ms = unsafe { GetTickCount64() };
                println!(
                    "regions ignoring staleness: {:#?}",
                    OpaqueRegionsReader::parse_block(&fresh)
                );
            }
            println!("live opaque regions: {regions:#?}");
        }
    }
}
