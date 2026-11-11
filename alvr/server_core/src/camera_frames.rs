// Writer for headset camera frames, published to overlay apps (ScreenVR) through shared memory,
// e.g. for locating physical monitors with ChArUco boards. The client only sends frames while
// the overlay requests them (see `tracking_extras::camera_frames_request`).
//
// Contract (source of truth: ScreenVR `src/VrCameraFrame.h`, copied here, no dependency):
// - Named file mapping `Local\VRCameraFrame_v1`: a 152-byte header, pack(4), then up to 2048x2048
//   GRAY8 pixels, top-down rows.
// - Seqlock on the header `sequence` (odd = write in progress), covering header and pixels.
// - `heartbeat_ms` (GetTickCount64) is refreshed at least every 100 ms. Older than 1000 ms = no
//   frames.
// - `stage_from_camera` is the camera pose in the client STAGE space (row-major 3x4,
//   p_stage = M * p_cam) with OpenCV camera axes. `recenter_counter` pairs it with the
//   stage -> driver transform of `Local\VRTrackingExtras_v1`.

// - Tracking stamp (2026-11-11, no version change): a 40-byte VrCameraFrameTrackingStamp right
//   after the pixel area (offset 152 + 2048 * 2048), written under the same seqlock. It holds the
//   head tracking state at the capture time (from the client's CameraTrackingStamp sent just before
//   the frame). Readers that map only header + pixels never see it; readers must map the whole
//   section and accept the stamp only when its magic, frameIndex and captureNs match the header.

use alvr_common::anyhow::Result;
use alvr_packets::{CameraFrame, CameraTrackingStamp};
use std::collections::VecDeque;

#[cfg(windows)]
pub use windows_impl::{publish_frame, refresh_heartbeat};

// The client sends a frame's stamp right before the frame; a few are kept in case of reordering
const STAMPS_KEPT: usize = 8;
static STAMPS: alvr_common::parking_lot::Mutex<VecDeque<CameraTrackingStamp>> =
    alvr_common::parking_lot::Mutex::new(VecDeque::new());

/// ClientControlPacket::ReservedBuffer with a CameraTrackingStamp
pub fn report_stamp(stamp: CameraTrackingStamp) {
    let mut stamps = STAMPS.lock();
    stamps.push_back(stamp);
    while stamps.len() > STAMPS_KEPT {
        stamps.pop_front();
    }
}

// The stamp of the frame captured at `capture_ns`, if the client sent one
#[cfg_attr(not(windows), allow(dead_code))]
fn take_stamp(capture_ns: u64) -> Option<CameraTrackingStamp> {
    let mut stamps = STAMPS.lock();
    let index = stamps.iter().position(|s| s.capture_ns == capture_ns)?;
    stamps.remove(index)
}

#[cfg(not(windows))]
pub fn publish_frame(_frame: &CameraFrame) -> Result<()> {
    Ok(())
}

#[cfg(not(windows))]
pub fn refresh_heartbeat() {}

// Decodes the frame's JPEG into GRAY8 pixels (width * height bytes)
fn decode_gray(frame: &CameraFrame) -> Result<Vec<u8>> {
    use alvr_common::anyhow::bail;
    use std::io::Cursor;
    use zune_jpeg::{
        JpegDecoder,
        zune_core::{colorspace::ColorSpace, options::DecoderOptions},
    };

    let options = DecoderOptions::default().jpeg_set_out_colorspace(ColorSpace::Luma);
    let mut decoder = JpegDecoder::new_with_options(Cursor::new(&frame.jpeg), options);
    let pixels = decoder.decode()?;

    let Some((width, height)) = decoder.dimensions() else {
        bail!("Camera frame: JPEG without dimensions");
    };
    if (width as u32, height as u32) != (frame.width, frame.height)
        || pixels.len() != width * height
    {
        bail!(
            "Camera frame: JPEG is {width}x{height} ({} bytes), header says {}x{}",
            pixels.len(),
            frame.width,
            frame.height
        );
    }

    Ok(pixels)
}

#[cfg(windows)]
mod windows_impl {
    use super::{decode_gray, take_stamp};
    use crate::tracking_extras::{recenter_counter, to_matrix, tracking_epoch_for};
    use alvr_common::{anyhow::Result, info, parking_lot::Mutex, warn};
    use alvr_packets::{CameraEye, CameraFrame};
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
                    CreateFileMappingW, FILE_MAP_WRITE, MEMORY_BASIC_INFORMATION,
                    MEMORY_MAPPED_VIEW_ADDRESS, MapViewOfFile, PAGE_READWRITE, UnmapViewOfFile,
                    VirtualQuery,
                },
                SystemInformation::GetTickCount64,
            },
        },
        core::w,
    };

    const MAGIC: u32 = 0x464D_4143; // 'CAMF' little-endian
    const VERSION: u32 = 1;
    const MAX_WIDTH: u32 = 2048;
    const MAX_HEIGHT: u32 = 2048;
    const FORMAT_GRAY8: u32 = 0;
    const CREATE_RETRY_INTERVAL: Duration = Duration::from_secs(1);
    const SEQUENCE_OFFSET: usize = 8;
    const HEARTBEAT_OFFSET: usize = 16;
    // The heartbeat stops this long after the last frame, so that readers see the frames end
    const HEARTBEAT_AFTER_LAST_FRAME: Duration = Duration::from_secs(3);

    #[repr(C, packed(4))]
    #[derive(Clone, Copy)]
    struct RawHeader {
        magic: u32,
        version: u32,
        sequence: u32,
        frame_index: u32,
        heartbeat_ms: u64,
        capture_ns: u64,
        server_receive_ms: u64,
        width: u32,
        height: u32,
        stride: u32,
        format: u32,
        eye: u32,
        recenter_counter: u32,
        fx: f32,
        fy: f32,
        cx: f32,
        cy: f32,
        k1: f32,
        k2: f32,
        p1: f32,
        p2: f32,
        k3: f32,
        // Head angular speed at capture, degrees per second (VrCameraFrame.h: reserved)
        head_angular_speed_deg_per_sec: f32,
        stage_from_camera: [[f32; 4]; 3],
    }

    const _: () = assert!(mem::size_of::<RawHeader>() == 152);
    const HEADER_SIZE: usize = mem::size_of::<RawHeader>();
    const BLOCK_SIZE: usize = HEADER_SIZE + (MAX_WIDTH * MAX_HEIGHT) as usize;

    // VrCameraFrame.h VrCameraFrameTrackingStamp, at BLOCK_SIZE
    const STAMP_MAGIC: u32 = 0x5354_4643; // 'CFTS' little-endian
    // headTrackingFlags bit 3 in the stamp: the client sent a stamp for this frame
    const STAMP_KNOWN_BIT: u32 = 1 << 3;

    #[repr(C, packed(4))]
    #[derive(Clone, Copy)]
    struct RawTrackingStamp {
        magic: u32,
        size: u32,
        frame_index: u32,
        head_tracking_flags: u32,
        capture_ns: u64,
        tracking_epoch: u32,
        client_tracking_epoch: u32,
        client_instance_id: u64,
    }

    const _: () = assert!(mem::size_of::<RawTrackingStamp>() == 40);
    const STAMP_SIZE: usize = mem::size_of::<RawTrackingStamp>();
    const SECTION_SIZE: usize = BLOCK_SIZE + STAMP_SIZE;

    struct Mapping {
        handle: HANDLE,
        view: MEMORY_MAPPED_VIEW_ADDRESS,
        // Bytes addressable in the view (a section left by an older writer may be smaller than
        // SECTION_SIZE; views are page-granular)
        view_size: usize,
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
        fn base(&self) -> *mut u8 {
            self.view.Value.cast::<u8>()
        }

        fn sequence(&self) -> &AtomicU32 {
            // # Safety: the view is BLOCK_SIZE bytes and `sequence` is 4-byte aligned (pack(4),
            // offset 8, page-aligned view)
            unsafe { &*self.base().add(SEQUENCE_OFFSET).cast::<AtomicU32>() }
        }

        // Seqlock write: `write` must not touch the sequence field
        fn locked_write(&self, write: impl FnOnce(*mut u8)) {
            let sequence = self.sequence();
            let before = sequence.load(Ordering::Relaxed);
            // An odd value left by a writer that died mid-write is skipped over
            let odd = if before.is_multiple_of(2) {
                before.wrapping_add(1)
            } else {
                before.wrapping_add(2)
            };
            sequence.store(odd, Ordering::Relaxed);
            atomic::fence(Ordering::Release);

            write(self.base());

            sequence.store(odd.wrapping_add(1), Ordering::Release);
        }
    }

    struct CameraFrameWriter {
        mapping: Option<Mapping>,
        last_create_attempt: Option<Instant>,
        frame_index: u32,
        last_frame: Option<Instant>,
    }

    static WRITER: Mutex<CameraFrameWriter> = Mutex::new(CameraFrameWriter {
        mapping: None,
        last_create_attempt: None,
        frame_index: 0,
        last_frame: None,
    });

    fn now_ms() -> u64 {
        unsafe { GetTickCount64() }
    }

    impl CameraFrameWriter {
        fn ensure_mapping(&mut self) -> Option<&Mapping> {
            if self.mapping.is_none()
                && self
                    .last_create_attempt
                    .is_none_or(|instant| instant.elapsed() >= CREATE_RETRY_INTERVAL)
            {
                self.last_create_attempt = Some(Instant::now());

                // Opens the existing mapping if there is one
                match unsafe {
                    CreateFileMappingW(
                        INVALID_HANDLE_VALUE,
                        None,
                        PAGE_READWRITE,
                        0,
                        SECTION_SIZE as u32,
                        w!("Local\\VRCameraFrame_v1"),
                    )
                } {
                    Ok(handle) => {
                        // The whole section: one created by an older writer (still held open by
                        // a reader) is smaller
                        let view = unsafe { MapViewOfFile(handle, FILE_MAP_WRITE, 0, 0, 0) };
                        let mut info = MEMORY_BASIC_INFORMATION::default();
                        let queried = !view.Value.is_null()
                            && unsafe {
                                VirtualQuery(
                                    Some(view.Value),
                                    &mut info,
                                    mem::size_of::<MEMORY_BASIC_INFORMATION>(),
                                )
                            } != 0;
                        if view.Value.is_null() || !queried || info.RegionSize < BLOCK_SIZE {
                            warn!("Camera frames: MapViewOfFile failed or section too small");
                            unsafe {
                                if !view.Value.is_null() {
                                    UnmapViewOfFile(view).ok();
                                }
                                CloseHandle(handle).ok();
                            }
                        } else {
                            info!(
                                "Camera frames: shared memory Local\\VRCameraFrame_v1 created \
                                 ({} bytes mapped, tracking stamp {})",
                                info.RegionSize,
                                if info.RegionSize >= SECTION_SIZE {
                                    "written"
                                } else {
                                    "not written (older section)"
                                }
                            );
                            self.mapping = Some(Mapping {
                                handle,
                                view,
                                view_size: info.RegionSize,
                            });
                        }
                    }
                    Err(e) => warn!("Camera frames: CreateFileMappingW failed: {e}"),
                }
            }

            self.mapping.as_ref()
        }
    }

    // Decodes the frame and publishes it with its metadata
    pub fn publish_frame(frame: &CameraFrame) -> Result<()> {
        use alvr_common::anyhow::bail;

        if frame.width > MAX_WIDTH || frame.height > MAX_HEIGHT {
            bail!(
                "Camera frame: {}x{} exceeds {MAX_WIDTH}x{MAX_HEIGHT}",
                frame.width,
                frame.height
            );
        }

        let pixels = decode_gray(frame)?;
        let recenter_counter = recenter_counter();
        let capture_ns = frame.capture_time.as_nanos() as u64;
        // Before the frame writer's lock (tracking_epoch_for locks the extras writer)
        let stamp = take_stamp(capture_ns).map(|stamp| {
            (
                stamp,
                tracking_epoch_for(stamp.client_instance, stamp.client_epoch),
            )
        });

        let mut writer = WRITER.lock();
        writer.last_frame = Some(Instant::now());
        writer.frame_index = writer.frame_index.wrapping_add(1).max(1);
        let frame_index = writer.frame_index;
        let Some(mapping) = writer.ensure_mapping() else {
            return Ok(());
        };

        let now = now_ms();
        let header = RawHeader {
            magic: MAGIC,
            version: VERSION,
            sequence: 0, // not copied, see below
            frame_index,
            heartbeat_ms: now,
            capture_ns: frame.capture_time.as_nanos() as u64,
            server_receive_ms: now,
            width: frame.width,
            height: frame.height,
            stride: frame.width,
            format: FORMAT_GRAY8,
            eye: match frame.eye {
                CameraEye::Left => 0,
                CameraEye::Right => 1,
            },
            recenter_counter,
            fx: frame.fx,
            fy: frame.fy,
            cx: frame.cx,
            cy: frame.cy,
            k1: 0.0,
            k2: 0.0,
            p1: 0.0,
            p2: 0.0,
            k3: 0.0,
            head_angular_speed_deg_per_sec: frame.head_angular_speed.to_degrees(),
            stage_from_camera: to_matrix(frame.stage_from_camera),
        };

        let tracking_stamp = RawTrackingStamp {
            magic: STAMP_MAGIC,
            size: STAMP_SIZE as u32,
            frame_index,
            // Bits 0-2 as the client sent them; bit 3 = known. No stamp: all 0 (unknown)
            head_tracking_flags: stamp.map_or(0, |(s, _)| (s.flags & 0b111) | STAMP_KNOWN_BIT),
            capture_ns,
            tracking_epoch: stamp.map_or(0, |(_, epoch)| epoch),
            client_tracking_epoch: stamp.map_or(0, |(s, _)| s.client_epoch),
            client_instance_id: stamp.map_or(0, |(s, _)| s.client_instance),
        };
        let write_stamp = mapping.view_size >= SECTION_SIZE;

        mapping.locked_write(|base| {
            let source = ptr::from_ref(&header).cast::<u8>();
            // # Safety: the view is BLOCK_SIZE bytes, the header is HEADER_SIZE bytes and the
            // pixels at most MAX_WIDTH * MAX_HEIGHT (checked above). The sequence field
            // (bytes 8..12) is skipped so that it is only written atomically.
            unsafe {
                ptr::copy_nonoverlapping(source, base, SEQUENCE_OFFSET);
                ptr::copy_nonoverlapping(
                    source.add(SEQUENCE_OFFSET + 4),
                    base.add(SEQUENCE_OFFSET + 4),
                    HEADER_SIZE - SEQUENCE_OFFSET - 4,
                );
                ptr::copy_nonoverlapping(pixels.as_ptr(), base.add(HEADER_SIZE), pixels.len());
                // # Safety: only when the view holds SECTION_SIZE bytes
                if write_stamp {
                    ptr::copy_nonoverlapping(
                        ptr::from_ref(&tracking_stamp).cast::<u8>(),
                        base.add(BLOCK_SIZE),
                        STAMP_SIZE,
                    );
                }
            }
        });

        Ok(())
    }

    // Keeps the last frame alive between frames. Stops shortly after the frames stop, so that
    // the block goes stale; does nothing before the first frame.
    pub fn refresh_heartbeat() {
        let writer = WRITER.lock();
        let Some(mapping) = &writer.mapping else {
            return;
        };
        if writer
            .last_frame
            .is_none_or(|instant| instant.elapsed() > HEARTBEAT_AFTER_LAST_FRAME)
        {
            return;
        }

        let now = now_ms();
        mapping.locked_write(|base| {
            // # Safety: heartbeat_ms is at offset 16 within the header, 4-byte aligned
            unsafe { ptr::write_unaligned(base.add(HEARTBEAT_OFFSET).cast::<u64>(), now) };
        });
    }
}

#[cfg(test)]
mod tests {
    use super::decode_gray;
    use alvr_common::Pose;
    use alvr_packets::{CameraEye, CameraFrame};
    use std::time::Duration;

    fn frame(width: u32, height: u32, jpeg: Vec<u8>) -> CameraFrame {
        CameraFrame {
            capture_time: Duration::ZERO,
            eye: CameraEye::Left,
            width,
            height,
            fx: 1.0,
            fy: 1.0,
            cx: 0.0,
            cy: 0.0,
            stage_from_camera: Pose::IDENTITY,
            head_angular_speed: 0.0,
            jpeg,
        }
    }

    #[test]
    fn decodes_gray_jpeg_and_checks_size() {
        let (width, height) = (64_u16, 32_u16);
        let pixels = (0..u32::from(width) * u32::from(height))
            .map(|i| (i % 256) as u8)
            .collect::<Vec<_>>();
        let mut jpeg = Vec::new();
        jpeg_encoder::Encoder::new(&mut jpeg, 95)
            .encode(&pixels, width, height, jpeg_encoder::ColorType::Luma)
            .unwrap();

        let decoded = decode_gray(&frame(width.into(), height.into(), jpeg.clone())).unwrap();
        assert_eq!(decoded.len(), pixels.len());
        let mean_error = decoded
            .iter()
            .zip(&pixels)
            .map(|(a, b)| f64::from(a.abs_diff(*b)))
            .sum::<f64>()
            / pixels.len() as f64;
        assert!(mean_error < 4.0, "mean error {mean_error}");

        // Header dimensions must match the JPEG
        assert!(decode_gray(&frame(32, 32, jpeg)).is_err());
    }
}
