// Reader for screen video streams that an overlay app (ScreenVR) captures and encodes itself. The
// encoded access units are forwarded to the client unchanged (SCREEN_VIDEO stream), where a second
// decoder shows them on the native quad of that screen.
//
// Contract (source of truth: ScreenVR `src/VrScreenVideo.h`, copied here, no dependency):
// - Named file mapping `Local\VRScreenVideo_v1` plus the auto-reset event
//   `Local\VRScreenVideo_v1_Event`, signaled after every frame. pack(4), little-endian.
// - Header (32 bytes), 4 stream descriptors (2112 bytes each, 64 frame slots with a seqlock), then
//   4 byte rings of 8 MiB.
// - Reader, per stream: for every frame index in (last read, latestFrameIndex], read the slot
//   under its seqlock, check its frame index, copy the bytes from the ring, then check that the
//   writer did not overwrite them meanwhile (writeCursor - slot.cursor <= ring size). A gap or an
//   overwritten frame is a loss: frames are dropped until the next keyframe and a keyframe is
//   requested by incrementing idrRequestCounter (the only field written by ALVR).
// - Every keyframe is an IDR with its parameter sets. A new streamId starts a new stream.
// - Heartbeat at least every 100 ms, stale after 1000 ms (= no streams).

use alvr_packets::ScreenVideoPacketHeader;
#[cfg(not(windows))]
use std::time::Duration;

#[cfg(windows)]
pub use windows_impl::ScreenVideoReader;

pub struct ScreenVideoFrame {
    pub header: ScreenVideoPacketHeader,
    pub data: Vec<u8>,
}

#[cfg(not(windows))]
pub struct ScreenVideoReader;

#[cfg(not(windows))]
impl ScreenVideoReader {
    pub const fn new() -> Self {
        Self
    }

    pub fn read(&mut self, timeout: Duration) -> Vec<ScreenVideoFrame> {
        std::thread::sleep(timeout);

        Vec::new()
    }

    pub fn request_idr(&mut self, _stream_id: u32) {}
}

#[cfg(windows)]
mod windows_impl {
    use super::ScreenVideoFrame;
    use alvr_common::{info, warn};
    use alvr_packets::{ScreenVideoCodec, ScreenVideoPacketHeader};
    use std::{
        mem, ptr,
        sync::atomic::{self, AtomicU32, AtomicU64, Ordering},
        thread,
        time::{Duration, Instant},
    };
    use windows::{
        Win32::{
            Foundation::{CloseHandle, HANDLE},
            System::{
                Memory::{
                    FILE_MAP_READ, FILE_MAP_WRITE, MEMORY_MAPPED_VIEW_ADDRESS, MapViewOfFile,
                    OpenFileMappingW, UnmapViewOfFile,
                },
                SystemInformation::GetTickCount64,
                Threading::{OpenEventW, SYNCHRONIZATION_SYNCHRONIZE, WaitForSingleObject},
            },
        },
        core::w,
    };

    const MAGIC: u32 = 0x4449_5653; // 'SVID' little-endian
    const VERSION: u32 = 1;
    const MAX_STREAMS: usize = 4;
    const FRAME_SLOTS: u64 = 64;
    const RING_BYTES: u64 = 8 * 1024 * 1024;
    const STALE_MS: u64 = 1000;
    const CODEC_H264: u32 = 1;
    const CODEC_HEVC: u32 = 2;
    const FRAME_FLAG_KEYFRAME: u32 = 1;
    const OPEN_RETRY_INTERVAL: Duration = Duration::from_secs(1);
    const MAX_READ_ATTEMPTS: usize = 4;
    // Keyframe requests to the writer, at most one per stream in this interval
    const IDR_REQUEST_MIN_INTERVAL: Duration = Duration::from_millis(200);

    #[repr(C, packed(4))]
    #[derive(Clone, Copy)]
    struct RawFrame {
        sequence: u32,
        flags: u32,
        frame_index: u64,
        cursor: u64,
        size: u32,
        capture_time_us: u32,
    }

    #[repr(C, packed(4))]
    struct RawStream {
        stream_id: u32,
        codec: u32,
        width: u32,
        height: u32,
        fps_nominal: u32,
        bitrate_kbps: u32,
        latest_frame_index: u64,
        write_cursor: u64,
        idr_request_counter: u32,
        idr_served_counter: u32,
        stable_id: [u8; 16],
        frames: [RawFrame; FRAME_SLOTS as usize],
    }

    #[repr(C, packed(4))]
    #[derive(Clone, Copy)]
    struct RawHeader {
        magic: u32,
        version: u32,
        heartbeat_ms: u64,
        stream_count: u32,
        reserved: [u32; 3],
    }

    const _: () = assert!(mem::size_of::<RawFrame>() == 32);
    const _: () = assert!(mem::size_of::<RawStream>() == 2112);
    const _: () = assert!(mem::size_of::<RawHeader>() == 32);

    const STREAMS_OFFSET: usize = mem::size_of::<RawHeader>();
    const RINGS_OFFSET: usize = STREAMS_OFFSET + MAX_STREAMS * mem::size_of::<RawStream>();
    const MAPPING_BYTES: usize = RINGS_OFFSET + MAX_STREAMS * RING_BYTES as usize;
    const _: () = assert!(RINGS_OFFSET == 8480);

    struct Mapping {
        handle: HANDLE,
        event: Option<HANDLE>,
        view: MEMORY_MAPPED_VIEW_ADDRESS,
    }

    // The mapping is only used by the thread that owns the reader
    unsafe impl Send for Mapping {}

    impl Drop for Mapping {
        fn drop(&mut self) {
            unsafe {
                UnmapViewOfFile(self.view).ok();
                CloseHandle(self.handle).ok();
                if let Some(event) = self.event {
                    CloseHandle(event).ok();
                }
            }
        }
    }

    impl Mapping {
        fn base(&self) -> *mut u8 {
            self.view.Value.cast()
        }

        // # Safety (all accessors): the view covers MAPPING_BYTES; fields are naturally aligned
        // (page-aligned view, stream descriptors at 32 + k * 2112, both multiples of 8, and the
        // 64-bit fields at 8-aligned offsets inside them)
        fn header(&self) -> RawHeader {
            unsafe { ptr::read_volatile(self.base().cast::<RawHeader>()) }
        }

        fn stream(&self, idx: usize) -> *mut RawStream {
            unsafe {
                self.base()
                    .add(STREAMS_OFFSET + idx * mem::size_of::<RawStream>())
                    .cast()
            }
        }

        fn u32_at(&self, stream_idx: usize, field_offset: usize) -> &AtomicU32 {
            unsafe {
                &*self
                    .stream(stream_idx)
                    .cast::<u8>()
                    .add(field_offset)
                    .cast()
            }
        }

        fn u64_at(&self, stream_idx: usize, field_offset: usize) -> &AtomicU64 {
            unsafe {
                &*self
                    .stream(stream_idx)
                    .cast::<u8>()
                    .add(field_offset)
                    .cast()
            }
        }

        fn ring(&self, stream_idx: usize) -> *const u8 {
            unsafe {
                self.base()
                    .add(RINGS_OFFSET + stream_idx * RING_BYTES as usize)
                    .cast_const()
            }
        }
    }

    // Field offsets inside RawStream
    const STREAM_ID_OFFSET: usize = 0;
    const LATEST_FRAME_INDEX_OFFSET: usize = 24;
    const WRITE_CURSOR_OFFSET: usize = 32;
    const IDR_REQUEST_COUNTER_OFFSET: usize = 40;
    const FRAMES_OFFSET: usize = 64;

    struct StreamState {
        stream_id: u32,
        last_read: u64,
        waiting_keyframe: bool,
        last_idr_request: Option<Instant>,
        lost_frames: u64,
    }

    pub struct ScreenVideoReader {
        mapping: Option<Mapping>,
        last_open_attempt: Option<Instant>,
        streams: [Option<StreamState>; MAX_STREAMS],
        stale_logged: bool,
    }

    impl ScreenVideoReader {
        pub const fn new() -> Self {
            Self {
                mapping: None,
                last_open_attempt: None,
                streams: [const { None }; MAX_STREAMS],
                stale_logged: false,
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

            // Not found = the overlay app does not publish screen video (the normal case)
            let access = FILE_MAP_READ.0 | FILE_MAP_WRITE.0;
            let Ok(handle) =
                (unsafe { OpenFileMappingW(access, false, w!("Local\\VRScreenVideo_v1")) })
            else {
                return;
            };

            let view = unsafe {
                MapViewOfFile(handle, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, MAPPING_BYTES)
            };
            if view.Value.is_null() {
                warn!("Screen video: MapViewOfFile failed");
                unsafe { CloseHandle(handle).ok() };
                return;
            }

            let event = unsafe {
                OpenEventW(
                    SYNCHRONIZATION_SYNCHRONIZE,
                    false,
                    w!("Local\\VRScreenVideo_v1_Event"),
                )
            }
            .ok();

            info!(
                "Screen video: shared memory Local\\VRScreenVideo_v1 opened (event: {})",
                event.is_some()
            );
            self.mapping = Some(Mapping {
                handle,
                event,
                view,
            });
        }

        // Increments idrRequestCounter of the stream, rate limited
        fn request_idr_at(&mut self, stream_idx: usize) {
            let (Some(mapping), Some(state)) = (&self.mapping, &mut self.streams[stream_idx])
            else {
                return;
            };
            if state
                .last_idr_request
                .is_some_and(|instant| instant.elapsed() < IDR_REQUEST_MIN_INTERVAL)
            {
                return;
            }
            state.last_idr_request = Some(Instant::now());

            mapping
                .u32_at(stream_idx, IDR_REQUEST_COUNTER_OFFSET)
                .fetch_add(1, Ordering::AcqRel);
        }

        /// Keyframe request from the client (loss or decoder error)
        pub fn request_idr(&mut self, stream_id: u32) {
            let idx = self.streams.iter().position(|state| {
                state
                    .as_ref()
                    .is_some_and(|state| state.stream_id == stream_id)
            });
            if let Some(idx) = idx {
                self.request_idr_at(idx);
            }
        }

        // Copies `size` bytes at the monotonic `cursor` of a ring, wrapping around its end
        fn copy_from_ring(
            mapping: &Mapping,
            stream_idx: usize,
            cursor: u64,
            size: usize,
        ) -> Vec<u8> {
            let ring = mapping.ring(stream_idx);
            let start = (cursor % RING_BYTES) as usize;
            let first = size.min(RING_BYTES as usize - start);

            let mut data = vec![0; size];
            unsafe {
                ptr::copy_nonoverlapping(ring.add(start), data.as_mut_ptr(), first);
                ptr::copy_nonoverlapping(ring, data.as_mut_ptr().add(first), size - first);
            }

            data
        }

        // Reads one frame. None = loss (overwritten, slot reused or torn).
        fn read_frame(
            mapping: &Mapping,
            stream_idx: usize,
            frame_index: u64,
        ) -> Option<(RawFrame, Vec<u8>)> {
            let slot_offset =
                FRAMES_OFFSET + (frame_index % FRAME_SLOTS) as usize * mem::size_of::<RawFrame>();
            let sequence = mapping.u32_at(stream_idx, slot_offset);
            let slot = unsafe {
                mapping
                    .stream(stream_idx)
                    .cast::<u8>()
                    .add(slot_offset)
                    .cast::<RawFrame>()
            };

            for _ in 0..MAX_READ_ATTEMPTS {
                let before = sequence.load(Ordering::Acquire);
                if !before.is_multiple_of(2) {
                    std::hint::spin_loop();
                    continue;
                }

                let frame = unsafe { ptr::read_volatile(slot) };
                atomic::fence(Ordering::Acquire);
                if sequence.load(Ordering::Relaxed) != before {
                    continue;
                }

                let (slot_index, size, cursor) = (frame.frame_index, frame.size, frame.cursor);
                if slot_index != frame_index || u64::from(size) > RING_BYTES {
                    return None;
                }

                let data = Self::copy_from_ring(mapping, stream_idx, cursor, size as usize);

                atomic::fence(Ordering::Acquire);
                let write_cursor = mapping
                    .u64_at(stream_idx, WRITE_CURSOR_OFFSET)
                    .load(Ordering::Acquire);
                if write_cursor.saturating_sub(cursor) > RING_BYTES {
                    return None;
                }

                return Some((frame, data));
            }

            None
        }

        /// Waits for new frames (event or timeout) and returns them, in order per stream
        pub fn read(&mut self, timeout: Duration) -> Vec<ScreenVideoFrame> {
            if self.mapping.is_none() {
                self.try_open();
            }
            let Some(mapping) = &self.mapping else {
                thread::sleep(timeout);
                return Vec::new();
            };

            if let Some(event) = mapping.event {
                unsafe { WaitForSingleObject(event, timeout.as_millis() as u32) };
            } else {
                thread::sleep(timeout);
            }

            let header = mapping.header();
            let (magic, version, heartbeat_ms, stream_count) = (
                header.magic,
                header.version,
                header.heartbeat_ms,
                header.stream_count,
            );
            let stale = unsafe { GetTickCount64() }.saturating_sub(heartbeat_ms) > STALE_MS;
            if magic != MAGIC || version != VERSION || stale {
                if !self.stale_logged {
                    info!(
                        "Screen video: no streams (magic {magic:#x}, version {version}, stale \
                         {stale})"
                    );
                    self.stale_logged = true;
                }
                self.streams = [const { None }; MAX_STREAMS];
                return Vec::new();
            }
            self.stale_logged = false;

            let mut frames = Vec::new();
            for stream_idx in 0..MAX_STREAMS {
                let Some(mapping) = &self.mapping else {
                    break;
                };
                let stream_id = mapping
                    .u32_at(stream_idx, STREAM_ID_OFFSET)
                    .load(Ordering::Acquire);
                if stream_idx >= stream_count as usize || stream_id == 0 {
                    self.streams[stream_idx] = None;
                    continue;
                }

                let latest = mapping
                    .u64_at(stream_idx, LATEST_FRAME_INDEX_OFFSET)
                    .load(Ordering::Acquire);
                // # Safety: plain fields, read once per descriptor change
                let raw = unsafe { &*mapping.stream(stream_idx) };
                let (codec, width, height, fps, bitrate) = (
                    raw.codec,
                    raw.width,
                    raw.height,
                    raw.fps_nominal,
                    raw.bitrate_kbps,
                );
                let codec = match codec {
                    CODEC_H264 => ScreenVideoCodec::H264,
                    CODEC_HEVC => ScreenVideoCodec::Hevc,
                    _ => continue,
                };

                if self.streams[stream_idx]
                    .as_ref()
                    .is_none_or(|state| state.stream_id != stream_id)
                {
                    let stable_id = raw.stable_id;
                    let name_len = stable_id.iter().position(|&b| b == 0).unwrap_or(16);
                    info!(
                        "Screen video: stream {stream_id} ({}) {codec:?} {width}x{height} \
                         {fps} fps {bitrate} kbps, starting at frame {latest}",
                        String::from_utf8_lossy(&stable_id[..name_len])
                    );
                    // Start at the next keyframe, and ask for one now
                    self.streams[stream_idx] = Some(StreamState {
                        stream_id,
                        last_read: latest,
                        waiting_keyframe: true,
                        last_idr_request: None,
                        lost_frames: 0,
                    });
                    self.request_idr_at(stream_idx);
                }

                let mut lost = false;
                let Some(mapping) = &self.mapping else {
                    break;
                };
                let Some(state) = &mut self.streams[stream_idx] else {
                    continue;
                };
                if latest < state.last_read {
                    // The writer restarted the frame count without a new stream id
                    state.last_read = latest;
                    lost = true;
                }
                if latest - state.last_read > FRAME_SLOTS {
                    state.lost_frames += latest - FRAME_SLOTS - state.last_read;
                    state.last_read = latest - FRAME_SLOTS;
                    lost = true;
                }

                for frame_index in state.last_read + 1..=latest {
                    state.last_read = frame_index;

                    let Some((frame, data)) = Self::read_frame(mapping, stream_idx, frame_index)
                    else {
                        state.lost_frames += 1;
                        lost = true;
                        state.waiting_keyframe = true;
                        continue;
                    };
                    let is_keyframe = frame.flags & FRAME_FLAG_KEYFRAME != 0;
                    if lost && !is_keyframe {
                        state.waiting_keyframe = true;
                    }
                    if is_keyframe {
                        state.waiting_keyframe = false;
                    }
                    if state.waiting_keyframe {
                        continue;
                    }

                    frames.push(ScreenVideoFrame {
                        header: ScreenVideoPacketHeader {
                            stream_id,
                            codec,
                            width,
                            height,
                            frame_index,
                            is_keyframe,
                        },
                        data,
                    });
                }

                if lost {
                    warn!(
                        "Screen video: stream {stream_id} lost frames ({} so far), waiting for \
                         a keyframe",
                        state.lost_frames
                    );
                }
                let waiting = state.waiting_keyframe;
                if waiting {
                    self.request_idr_at(stream_idx);
                }
            }

            frames
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        // Live check against a running writer (ScreenVR --encode-test):
        // cargo test -p alvr_server_core screen_video_live -- --ignored --nocapture
        #[test]
        #[ignore]
        fn screen_video_live() {
            let mut reader = ScreenVideoReader::new();
            let start = Instant::now();
            let (mut frames, mut keyframes, mut bytes) = (0, 0, 0);
            let mut last_index = None::<(u32, u64)>;
            let mut gaps = 0;
            while start.elapsed() < Duration::from_secs(8) {
                for frame in reader.read(Duration::from_millis(5)) {
                    let header = frame.header;
                    if let Some((stream_id, index)) = last_index
                        && stream_id == header.stream_id
                        && header.frame_index != index + 1
                    {
                        gaps += 1;
                    }
                    if frames == 0 {
                        println!("first: {header:?}, {} bytes", frame.data.len());
                        assert!(header.is_keyframe);
                        assert!(
                            frame.data.starts_with(&[0, 0, 0, 1])
                                || frame.data.starts_with(&[0, 0, 1])
                        );
                    }
                    last_index = Some((header.stream_id, header.frame_index));
                    frames += 1;
                    keyframes += usize::from(header.is_keyframe);
                    bytes += frame.data.len();
                }
                if start.elapsed() > Duration::from_secs(4)
                    && start.elapsed() < Duration::from_millis(4010)
                {
                    if let Some((stream_id, _)) = last_index {
                        reader.request_idr(stream_id);
                        println!("requested an IDR");
                    }
                }
            }
            println!(
                "frames {frames}, keyframes {keyframes}, bytes {bytes}, gaps {gaps}, last {last_index:?}"
            );
            assert!(frames > 0);
        }
    }
}
