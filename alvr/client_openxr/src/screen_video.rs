// Screen video streams (SCREEN_VIDEO) decoded into the native quads of their screens. Each stream
// gets its own MediaCodec decoder that renders straight into an XR_KHR_android_surface_swapchain
// swapchain, which the runtime composites as the quad's image (zero copy). Everything runs on the
// render thread: received access units are fed to the decoder and decoded frames are released to
// the surface once per rendered frame.

use alvr_client_core::ClientCoreContext;
use alvr_common::glam::UVec2;
use alvr_packets::ScreenVideoCodec;
use openxr as xr;

#[cfg(target_os = "android")]
pub use android_impl::ScreenVideoStreams;

#[cfg_attr(not(target_os = "android"), allow(dead_code))]
// Annex B NAL units of an access unit: (nal type, start of the start code)
fn nal_units(data: &[u8], codec: ScreenVideoCodec) -> Vec<(u8, usize)> {
    let mut units = Vec::new();
    let mut idx = 0;
    while idx + 3 < data.len() {
        let start_code_len = if data[idx..].starts_with(&[0, 0, 1]) {
            3
        } else if data[idx..].starts_with(&[0, 0, 0, 1]) {
            4
        } else {
            idx += 1;
            continue;
        };
        if let Some(&header) = data.get(idx + start_code_len) {
            let nal_type = match codec {
                ScreenVideoCodec::H264 => header & 0x1F,
                ScreenVideoCodec::Hevc => (header >> 1) & 0x3F,
            };
            units.push((nal_type, idx));
        }
        idx += start_code_len;
    }

    units
}

// The parameter sets of a keyframe: everything before its first picture (VCL) NAL unit
#[cfg_attr(not(target_os = "android"), allow(dead_code))]
fn parameter_sets(data: &[u8], codec: ScreenVideoCodec) -> Option<&[u8]> {
    let first_vcl = nal_units(data, codec)
        .into_iter()
        .find(|(nal_type, _)| match codec {
            ScreenVideoCodec::H264 => (1..=5).contains(nal_type),
            ScreenVideoCodec::Hevc => *nal_type < 32,
        })
        .map(|(_, offset)| offset)?;

    (first_vcl > 0).then(|| &data[..first_vcl])
}

#[cfg(not(target_os = "android"))]
#[derive(Default)]
pub struct ScreenVideoStreams;

#[cfg(not(target_os = "android"))]
impl ScreenVideoStreams {
    pub fn update(
        &mut self,
        _xr_session: &xr::Session<xr::OpenGlEs>,
        core_ctx: &ClientCoreContext,
    ) {
        while core_ctx.poll_screen_video_frame().is_some() {}
    }

    pub fn ready_swapchain(
        &self,
        _stream_id: u32,
    ) -> Option<(&xr::Swapchain<xr::OpenGlEs>, UVec2)> {
        None
    }
}

#[cfg(target_os = "android")]
mod android_impl {
    use super::*;
    use alvr_common::{info, warn};
    use alvr_packets::ScreenVideoPacketHeader;
    use ndk::{
        media::media_codec::{
            DequeuedInputBufferResult, DequeuedOutputBufferInfoResult, MediaCodec,
            MediaCodecDirection, MediaFormat,
        },
        native_window::NativeWindow,
    };
    use std::{
        collections::{HashMap, VecDeque},
        ffi::c_void,
        ptr,
        sync::atomic::{AtomicUsize, Ordering},
        time::{Duration, Instant},
    };

    // Access units waiting for a decoder input buffer; beyond this, wait for a keyframe
    const MAX_PENDING_FRAMES: usize = 30;
    // A stream without packets for this long is closed
    const STREAM_IDLE_TIMEOUT: Duration = Duration::from_secs(10);
    // Decoder errors in a row before the decoder is recreated at the next keyframe
    const MAX_DECODER_ERRORS: u32 = 10;
    const IDR_REQUEST_MIN_INTERVAL: Duration = Duration::from_millis(200);
    // Nominal presentation time step (the surface is shown as soon as a frame is released)
    const FRAME_TIME_US: u64 = 16_667;

    struct ScreenVideoStream {
        header: ScreenVideoPacketHeader,
        // Declared before the window and the swapchain: dropped first
        decoder: MediaCodec,
        _window: NativeWindow,
        swapchain: xr::Swapchain<xr::OpenGlEs>,
        pending: VecDeque<Vec<u8>>,
        waiting_keyframe: bool,
        decoded_frames: u64,
        queued_frames: u64,
        decoder_errors: u32,
        last_packet: Instant,
        last_idr_request: Option<Instant>,
    }

    impl Drop for ScreenVideoStream {
        fn drop(&mut self) {
            self.decoder.stop().ok();
        }
    }

    // JNIEnv of the render thread (attached to the JVM in android_main)
    fn jni_env() -> Result<*mut jni_sys::JNIEnv, String> {
        let vm = ndk_context::android_context()
            .vm()
            .cast::<jni_sys::JavaVM>();
        let mut env = ptr::null_mut::<c_void>();
        // # Safety: vm is the process JavaVM from ndk-context
        let attach = unsafe { (**vm).AttachCurrentThread }.ok_or("no AttachCurrentThread")?;
        let res = unsafe { attach(vm, &mut env, ptr::null_mut()) };
        if res != jni_sys::JNI_OK || env.is_null() {
            return Err(format!("AttachCurrentThread failed: {res}"));
        }

        Ok(env.cast())
    }

    // Create infos tried in turn for xrCreateSwapchainAndroidSurfaceKHR. The PICO 4 Ultra runtime
    // accepts only format, usage and counts all 0 (usage SAMPLED|COLOR_ATTACHMENT with counts 1
    // gives ERROR_VALIDATION_FAILURE); the others are for runtimes that validate differently.
    // (format, usage flags, sample/face/array/mip count)
    const SURFACE_SWAPCHAIN_VARIANTS: [(u32, u64, u32); 5] = [
        (0, 0, 0),
        (0, SAMPLED | COLOR_ATTACHMENT, 1),
        (GL_SRGB8_ALPHA8, SAMPLED | COLOR_ATTACHMENT, 1),
        (GL_RGBA8, SAMPLED | COLOR_ATTACHMENT, 1),
        (0, SAMPLED, 1),
    ];
    // XR_SWAPCHAIN_USAGE_SAMPLED_BIT, XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT
    const SAMPLED: u64 = 0x20;
    const COLOR_ATTACHMENT: u64 = 0x1;
    const GL_SRGB8_ALPHA8: u32 = 0x8C43;
    const GL_RGBA8: u32 = 0x8058;
    // Index of the variant that worked, tried first next time
    static WORKING_VARIANT: AtomicUsize = AtomicUsize::new(0);

    fn create_surface_swapchain(
        xr_session: &xr::Session<xr::OpenGlEs>,
        ext: &xr::raw::AndroidSurfaceSwapchainKHR,
        width: u32,
        height: u32,
    ) -> Result<(xr::sys::Swapchain, *mut c_void), String> {
        let first = WORKING_VARIANT.load(Ordering::Relaxed);
        let order = std::iter::once(first)
            .chain((0..SURFACE_SWAPCHAIN_VARIANTS.len()).filter(|&idx| idx != first));

        let mut failures = Vec::new();
        for idx in order {
            let (format, usage, count) = SURFACE_SWAPCHAIN_VARIANTS[idx];
            let create_info = xr::sys::SwapchainCreateInfo {
                ty: xr::sys::SwapchainCreateInfo::TYPE,
                next: ptr::null(),
                create_flags: xr::SwapchainCreateFlags::EMPTY,
                usage_flags: xr::SwapchainUsageFlags::from_raw(usage),
                format: i64::from(format),
                sample_count: count,
                width,
                height,
                face_count: count,
                array_size: count,
                mip_count: count,
            };
            let mut handle = <xr::sys::Swapchain as xr::sys::Handle>::NULL;
            let mut surface = ptr::null_mut();
            // # Safety: valid session, create info and out pointers
            let res = unsafe {
                (ext.create_swapchain_android_surface)(
                    xr_session.as_raw(),
                    &create_info,
                    &mut handle,
                    &mut surface,
                )
            };
            if res == xr::sys::Result::SUCCESS && !surface.is_null() {
                if WORKING_VARIANT.swap(idx, Ordering::Relaxed) != idx || !failures.is_empty() {
                    info!(
                        "Screen video: surface swapchain created with format {format:#x}, usage \
                         {usage:#x}, counts {count} (failed before: {failures:?})"
                    );
                }
                return Ok((handle, surface));
            }
            failures.push(format!("{format:#x}/{usage:#x}/{count}: {res:?}"));
        }

        Err(format!(
            "xrCreateSwapchainAndroidSurfaceKHR failed for all create infos: {failures:?}"
        ))
    }

    impl ScreenVideoStream {
        fn new(
            xr_session: &xr::Session<xr::OpenGlEs>,
            header: &ScreenVideoPacketHeader,
            keyframe: &[u8],
        ) -> Result<Self, String> {
            let ext = xr_session
                .instance()
                .exts()
                .khr_android_surface_swapchain
                .ok_or("XR_KHR_android_surface_swapchain not enabled")?;
            let csd = parameter_sets(keyframe, header.codec).ok_or("no parameter sets")?;

            let (handle, surface) =
                create_surface_swapchain(xr_session, &ext, header.width, header.height)?;
            // # Safety: the handle was just created on this session; destroyed on drop
            let swapchain =
                unsafe { xr::Swapchain::<xr::OpenGlEs>::from_raw(xr_session.clone(), handle) };

            // # Safety: the surface is a valid android.view.Surface for this thread's JNIEnv. The
            // window holds its own reference.
            let window = unsafe { NativeWindow::from_surface(jni_env()?, surface.cast()) }
                .ok_or("ANativeWindow_fromSurface failed")?;

            let mime = match header.codec {
                ScreenVideoCodec::H264 => "video/avc",
                ScreenVideoCodec::Hevc => "video/hevc",
            };
            let mut format = MediaFormat::new();
            format.set_str("mime", mime);
            format.set_i32("width", header.width as i32);
            format.set_i32("height", header.height as i32);
            format.set_buffer("csd-0", csd);
            format.set_i32("low-latency", 1);

            let decoder = MediaCodec::from_decoder_type(mime)
                .ok_or_else(|| format!("no decoder for {mime}"))?;
            decoder
                .configure(&format, Some(&window), MediaCodecDirection::Decoder)
                .map_err(|e| format!("configure: {e}"))?;
            decoder.start().map_err(|e| format!("start: {e}"))?;

            info!(
                "Screen video: stream {} {:?} {}x{} decoding into a surface swapchain",
                header.stream_id, header.codec, header.width, header.height
            );

            Ok(Self {
                header: *header,
                decoder,
                _window: window,
                swapchain,
                pending: VecDeque::new(),
                waiting_keyframe: false,
                decoded_frames: 0,
                queued_frames: 0,
                decoder_errors: 0,
                last_packet: Instant::now(),
                last_idr_request: None,
            })
        }

        fn request_idr(&mut self, core_ctx: &ClientCoreContext) {
            if self
                .last_idr_request
                .is_some_and(|instant| instant.elapsed() < IDR_REQUEST_MIN_INTERVAL)
            {
                return;
            }
            self.last_idr_request = Some(Instant::now());
            core_ctx.request_screen_idr(self.header.stream_id);
        }

        fn push(
            &mut self,
            header: &ScreenVideoPacketHeader,
            data: Vec<u8>,
            core_ctx: &ClientCoreContext,
        ) {
            self.last_packet = Instant::now();
            if header.is_keyframe {
                self.waiting_keyframe = false;
            }
            if self.waiting_keyframe {
                return;
            }
            if self.pending.len() >= MAX_PENDING_FRAMES {
                warn!(
                    "Screen video: stream {} decoder input full, waiting for a keyframe",
                    self.header.stream_id
                );
                self.pending.clear();
                self.waiting_keyframe = true;
                self.request_idr(core_ctx);
                return;
            }
            self.pending.push_back(data);
        }

        // Feeds pending access units and releases decoded frames to the surface
        fn pump(&mut self, core_ctx: &ClientCoreContext) {
            while let Some(data) = self.pending.front() {
                match self.decoder.dequeue_input_buffer(Duration::ZERO) {
                    Ok(DequeuedInputBufferResult::Buffer(mut buffer)) => {
                        let target = buffer.buffer_mut();
                        if data.len() > target.len() {
                            warn!(
                                "Screen video: access unit of {} bytes > input buffer {}",
                                data.len(),
                                target.len()
                            );
                            self.pending.pop_front();
                            self.waiting_keyframe = true;
                            self.request_idr(core_ctx);
                            continue;
                        }
                        // # Safety: the buffer has room for data (checked above)
                        unsafe {
                            ptr::copy_nonoverlapping(
                                data.as_ptr(),
                                target.as_mut_ptr().cast(),
                                data.len(),
                            )
                        };
                        let len = data.len();
                        let time_us = self.queued_frames * FRAME_TIME_US;
                        if let Err(e) = self.decoder.queue_input_buffer(buffer, 0, len, time_us, 0)
                        {
                            warn!("Screen video: queue_input_buffer: {e}");
                            self.decoder_errors += 1;
                        }
                        self.queued_frames += 1;
                        self.pending.pop_front();
                    }
                    Ok(DequeuedInputBufferResult::TryAgainLater) => break,
                    Err(e) => {
                        warn!("Screen video: dequeue_input_buffer: {e}");
                        self.decoder_errors += 1;
                        break;
                    }
                }
            }

            loop {
                match self.decoder.dequeue_output_buffer(Duration::ZERO) {
                    Ok(DequeuedOutputBufferInfoResult::Buffer(buffer)) => {
                        if let Err(e) = self.decoder.release_output_buffer(buffer, true) {
                            warn!("Screen video: release_output_buffer: {e}");
                            self.decoder_errors += 1;
                            continue;
                        }
                        if self.decoded_frames == 0 {
                            info!(
                                "Screen video: stream {} first frame decoded",
                                self.header.stream_id
                            );
                        }
                        self.decoded_frames += 1;
                        self.decoder_errors = 0;
                    }
                    Ok(DequeuedOutputBufferInfoResult::TryAgainLater) => break,
                    Ok(event) => info!("Screen video: decoder event {event:?}"),
                    Err(e) => {
                        warn!("Screen video: dequeue_output_buffer: {e}");
                        self.decoder_errors += 1;
                        break;
                    }
                }
            }

            if self.decoder_errors > 0 {
                self.request_idr(core_ctx);
            }
        }
    }

    #[derive(Default)]
    pub struct ScreenVideoStreams {
        streams: HashMap<u32, ScreenVideoStream>,
        // Streams that could not be created (logged once)
        failed_streams: HashMap<u32, String>,
    }

    impl ScreenVideoStreams {
        // Once per rendered frame
        pub fn update(
            &mut self,
            xr_session: &xr::Session<xr::OpenGlEs>,
            core_ctx: &ClientCoreContext,
        ) {
            while let Some((header, data)) = core_ctx.poll_screen_video_frame() {
                let compatible = self.streams.get(&header.stream_id).is_some_and(|stream| {
                    stream.header.codec == header.codec
                        && stream.header.width == header.width
                        && stream.header.height == header.height
                        && stream.decoder_errors <= MAX_DECODER_ERRORS
                });
                if !compatible {
                    if !header.is_keyframe {
                        // Decoding starts at a keyframe (the server asks for one)
                        continue;
                    }
                    self.streams.remove(&header.stream_id);
                    match ScreenVideoStream::new(xr_session, &header, &data) {
                        Ok(stream) => {
                            self.failed_streams.remove(&header.stream_id);
                            self.streams.insert(header.stream_id, stream);
                        }
                        Err(e) => {
                            if self.failed_streams.get(&header.stream_id) != Some(&e) {
                                warn!(
                                    "Screen video: cannot decode stream {}: {e}",
                                    header.stream_id
                                );
                                self.failed_streams.insert(header.stream_id, e);
                            }
                            continue;
                        }
                    }
                }

                if let Some(stream) = self.streams.get_mut(&header.stream_id) {
                    stream.push(&header, data, core_ctx);
                }
            }

            for stream in self.streams.values_mut() {
                stream.pump(core_ctx);
            }

            self.streams.retain(|stream_id, stream| {
                let active = stream.last_packet.elapsed() < STREAM_IDLE_TIMEOUT;
                if !active {
                    info!(
                        "Screen video: stream {stream_id} closed ({} frames decoded)",
                        stream.decoded_frames
                    );
                }
                active
            });
        }

        /// The swapchain of a stream that has decoded at least one frame, and its size
        pub fn ready_swapchain(
            &self,
            stream_id: u32,
        ) -> Option<(&xr::Swapchain<xr::OpenGlEs>, UVec2)> {
            self.streams
                .get(&stream_id)
                .filter(|stream| stream.decoded_frames > 0)
                .map(|stream| {
                    (
                        &stream.swapchain,
                        UVec2::new(stream.header.width, stream.header.height),
                    )
                })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finds_hevc_parameter_sets() {
        // VPS (32), SPS (33), PPS (34), IDR_W_RADL (19)
        let data = [
            0, 0, 0, 1, 0x40, 1, 0xAA, 0, 0, 0, 1, 0x42, 1, 0xBB, 0, 0, 1, 0x44, 1, 0xCC, 0, 0, 0,
            1, 0x26, 1, 0xDD,
        ];
        let csd = parameter_sets(&data, ScreenVideoCodec::Hevc).unwrap();
        assert_eq!(csd.len(), 20);
    }

    #[test]
    fn finds_h264_parameter_sets() {
        // SPS (7), PPS (8), IDR (5)
        let data = [
            0, 0, 0, 1, 0x67, 0xAA, 0, 0, 0, 1, 0x68, 0xBB, 0, 0, 0, 1, 0x65, 0xCC,
        ];
        let csd = parameter_sets(&data, ScreenVideoCodec::H264).unwrap();
        assert_eq!(csd.len(), 12);
    }
}
