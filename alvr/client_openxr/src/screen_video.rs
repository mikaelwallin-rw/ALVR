// Screen video streams (SCREEN_VIDEO) decoded into the native quads of their screens. Each stream
// gets its own MediaCodec decoder. Everything runs on the render thread: received access units are
// fed to the decoder and decoded frames are released to its output surface once per rendered frame.
// Two output paths, switchable per stream without restarting the decoder (setOutputSurface):
// - Zero copy: straight into an XR_KHR_android_surface_swapchain swapchain, which the runtime
//   composites as the quad's image. No mip chain: large screens alias when minified.
// - Mipmapped: into an AImageReader; each new frame is drawn (external OES texture of its
//   AHardwareBuffer) into a normal sRGB swapchain with a full mip chain, then glGenerateMipmap.
//   Costs a full-resolution draw + mip generation per decoded frame.
// Which path: mipmapped by default (2026-11-10, judged worn: no minification aliasing); DEBUG
// SWITCH file screenvr_video_path.txt in the app's external files dir, polled: "zero-copy",
// "mipmapped" or "auto" (per stream, from the quad's texel density versus
// the display's angular resolution at the current head distance, with hysteresis).

use alvr_client_core::ClientCoreContext;
use alvr_common::glam::{UVec2, Vec3};
use alvr_packets::ScreenVideoCodec;
use openxr as xr;

/// Where a video stream's quad is, for the auto path choice (STAGE space)
#[cfg_attr(
    not(target_os = "android"),
    expect(dead_code, reason = "the path choice runs on Android only")
)]
pub struct VideoQuadGeometry {
    pub stream_id: u32,
    pub position: Vec3,
    pub width: f32,
}

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
pub struct ScreenVideoStreams;

#[cfg(not(target_os = "android"))]
impl ScreenVideoStreams {
    pub fn new(_display_width: u32) -> Self {
        Self
    }

    pub fn update(
        &mut self,
        _xr_session: &xr::Session<xr::OpenGlEs>,
        _gfx_ctx: &std::rc::Rc<alvr_graphics::GraphicsContext>,
        core_ctx: &ClientCoreContext,
        _stage_space: Option<&xr::Space>,
        _display_time: xr::Time,
        _quads: &[VideoQuadGeometry],
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
    use crate::{gl_state::GlStateGuard, graphics};
    use alvr_common::{info, warn};
    use alvr_graphics::GraphicsContext;
    use alvr_packets::ScreenVideoPacketHeader;
    use glow::{self as gl, HasContext};
    use ndk::{
        hardware_buffer::HardwareBufferUsage,
        media::{
            image_reader::{AcquireResult, Image, ImageFormat, ImageReader},
            media_codec::{
                DequeuedInputBufferResult, DequeuedOutputBufferInfoResult, MediaCodec,
                MediaCodecDirection, MediaFormat,
            },
        },
        native_window::NativeWindow,
    };
    use std::{
        collections::{HashMap, VecDeque},
        ffi::c_void,
        ptr,
        rc::Rc,
        sync::atomic::{AtomicUsize, Ordering},
        time::{Duration, Instant},
    };

    const PATH_POLL_INTERVAL: Duration = Duration::from_secs(1);
    // Auto path: mipmapped above this minification (quad texels per display pixel at the current
    // head distance), zero copy again below the lower one. Bilinear sampling without mips stays
    // alias-free up to about 2:1.
    const AUTO_MIPMAPPED_ABOVE: f32 = 2.0;
    const AUTO_ZERO_COPY_BELOW: f32 = 1.5;
    // Decoded frames buffered by the image reader of the mipmapped path (1 held for drawing)
    const MIP_READER_MAX_IMAGES: i32 = 4;
    // Timing of a regenerated frame (synchronous, glFinish around it), at most this often
    const MIP_TIMING_LOG_INTERVAL: Duration = Duration::from_secs(5);
    const GL_TEXTURE_MAX_ANISOTROPY_EXT: u32 = 0x84FE;
    const GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT: u32 = 0x84FF;

    const BLIT_VERTEX_SHADER: &str = r#"#version 300 es
out vec2 uv;
void main() {
    vec2 screen_uv = vec2(gl_VertexID & 1, gl_VertexID >> 1);
    gl_Position = vec4((screen_uv - 0.5) * 2.0, 0.0, 1.0);
    uv = screen_uv;
}
"#;
    // The decoded image's first row is the top of the screen; the swapchain image's first row
    // (GL origin, bottom left) is shown at the bottom of the quad: flip vertically. The external
    // sampler returns the video's gamma-encoded RGB (BT.709 YUV converted by the driver). Writes
    // to an sRGB attachment are encoded, so decode first to store the same values as the zero
    // copy path shows; mip generation then averages in linear light.
    const BLIT_FRAGMENT_SHADER: &str = r#"#version 300 es
#extension GL_OES_EGL_image_external_essl3 : require
precision highp float;
uniform samplerExternalOES tex;
uniform int linearize;
in vec2 uv;
out vec4 out_color;
void main() {
    vec3 c = texture(tex, vec2(uv.x, 1.0 - uv.y)).rgb;
    if (linearize != 0) {
        c = mix(c / 12.92, pow((c + 0.055) / 1.055, vec3(2.4)), step(vec3(0.04045), c));
    }
    out_color = vec4(c, 1.0);
}
"#;

    #[derive(Clone, Copy, PartialEq, Debug)]
    enum PathMode {
        ZeroCopy,
        Mipmapped,
        Auto,
    }

    #[derive(Clone, Copy, PartialEq, Debug)]
    enum VideoPath {
        ZeroCopy,
        Mipmapped,
    }

    struct BlitProgram {
        program: gl::Program,
        // Our own VAO: wgpu's must stay bound outside our blocks (gl_state.rs)
        vertex_array: gl::VertexArray,
        linearize_uloc: Option<gl::UniformLocation>,
        tex_uloc: Option<gl::UniformLocation>,
    }

    fn create_blit_program(gl: &gl::Context) -> Result<BlitProgram, String> {
        unsafe {
            let program = gl.create_program()?;
            let mut shaders = Vec::new();
            for (kind, source) in [
                (gl::VERTEX_SHADER, BLIT_VERTEX_SHADER),
                (gl::FRAGMENT_SHADER, BLIT_FRAGMENT_SHADER),
            ] {
                let shader = gl.create_shader(kind)?;
                gl.shader_source(shader, source);
                gl.compile_shader(shader);
                if !gl.get_shader_compile_status(shader) {
                    return Err(format!("shader: {}", gl.get_shader_info_log(shader)));
                }
                gl.attach_shader(program, shader);
                shaders.push(shader);
            }
            gl.link_program(program);
            for shader in shaders {
                gl.delete_shader(shader);
            }
            if !gl.get_program_link_status(program) {
                return Err(format!("link: {}", gl.get_program_info_log(program)));
            }

            Ok(BlitProgram {
                vertex_array: gl.create_vertex_array()?,
                linearize_uloc: gl.get_uniform_location(program, "linearize"),
                tex_uloc: gl.get_uniform_location(program, "tex"),
                program,
            })
        }
    }

    // Mipmapped output of one stream
    struct MipTarget {
        // Declared first: dropped (returned to the reader) before the reader
        current_image: Option<Image>,
        reader: ImageReader,
        window: NativeWindow,
        context: Rc<GraphicsContext>,
        oes_texture: gl::Texture,
        framebuffer: gl::Framebuffer,
        swapchain: xr::Swapchain<xr::OpenGlEs>,
        images: Vec<u32>,
        size: UVec2,
        mip_levels: u32,
        srgb: bool,
        frames: u64,
        last_timing_log: Option<Instant>,
    }

    impl MipTarget {
        fn new(
            xr_session: &xr::Session<xr::OpenGlEs>,
            context: &Rc<GraphicsContext>,
            size: UVec2,
        ) -> Result<Self, String> {
            let mut reader = ImageReader::new_with_usage(
                size.x as i32,
                size.y as i32,
                ImageFormat::PRIVATE,
                HardwareBufferUsage::GPU_SAMPLED_IMAGE,
                MIP_READER_MAX_IMAGES,
            )
            .map_err(|e| format!("ImageReader: {e}"))?;
            // Documented as needed to dispose acquired buffers properly
            reader
                .set_buffer_removed_listener(Box::new(|_, _| ()))
                .map_err(|e| format!("buffer removed listener: {e}"))?;
            let window = reader.window().map_err(|e| format!("reader window: {e}"))?;

            context.make_current();
            let gl = &context.gl_context;
            let (format, srgb) = graphics::srgb_swapchain_format(xr_session);
            let full_mips = size.max_element().ilog2() + 1;
            let create = |mip_count| {
                xr_session.create_swapchain(&xr::SwapchainCreateInfo {
                    create_flags: xr::SwapchainCreateFlags::EMPTY,
                    usage_flags: xr::SwapchainUsageFlags::COLOR_ATTACHMENT
                        | xr::SwapchainUsageFlags::SAMPLED,
                    format,
                    sample_count: 1,
                    width: size.x,
                    height: size.y,
                    face_count: 1,
                    array_size: 1,
                    mip_count,
                })
            };
            let (swapchain, mip_levels) = match create(full_mips) {
                Ok(swapchain) => (swapchain, full_mips),
                Err(e) => {
                    warn!(
                        "Screen video: the runtime rejects {full_mips} mip levels ({e}), using 1"
                    );
                    (create(1).map_err(|e| format!("create_swapchain: {e}"))?, 1)
                }
            };
            let images = swapchain
                .enumerate_images()
                .map_err(|e| format!("enumerate_images: {e}"))?;

            let anisotropy = gl
                .supported_extensions()
                .contains("GL_EXT_texture_filter_anisotropic")
                .then(|| unsafe { gl.get_parameter_f32(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT) });
            let guard = GlStateGuard::save(gl);
            unsafe {
                for &image in &images {
                    let texture = gl::NativeTexture(
                        std::num::NonZeroU32::new(image).ok_or("invalid swapchain image")?,
                    );
                    gl.bind_texture(gl::TEXTURE_2D, Some(texture));
                    gl.tex_parameter_i32(
                        gl::TEXTURE_2D,
                        gl::TEXTURE_MIN_FILTER,
                        gl::LINEAR_MIPMAP_LINEAR as i32,
                    );
                    gl.tex_parameter_i32(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, gl::LINEAR as i32);
                    if let Some(max) = anisotropy {
                        gl.tex_parameter_f32(gl::TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, max);
                    }
                }
            }
            drop(guard);
            let (oes_texture, framebuffer) = unsafe {
                (
                    gl.create_texture().map_err(|e| format!("texture: {e}"))?,
                    gl.create_framebuffer()
                        .map_err(|e| format!("framebuffer: {e}"))?,
                )
            };

            info!(
                "Screen video: mipmapped target {}x{}, {mip_levels} mip level(s), format {format:#x}, \
                 {} swapchain images, anisotropy {anisotropy:?}",
                size.x,
                size.y,
                images.len()
            );

            Ok(Self {
                current_image: None,
                reader,
                window,
                context: Rc::clone(context),
                oes_texture,
                framebuffer,
                swapchain,
                images,
                size,
                mip_levels,
                srgb,
                frames: 0,
                last_timing_log: None,
            })
        }

        // Draws the newest decoded frame, if any, into the next swapchain image and regenerates
        // its mip chain. Logs a synchronous timing sample at most every MIP_TIMING_LOG_INTERVAL.
        fn latch(&mut self, program: &BlitProgram, stream_id: u32) {
            let image = match self.reader.acquire_latest_image() {
                Ok(AcquireResult::Image(image)) => image,
                Ok(_) => return,
                Err(e) => {
                    warn!("Screen video: stream {stream_id} acquire_latest_image: {e}");
                    return;
                }
            };
            let buffer = match image.hardware_buffer() {
                Ok(buffer) => buffer,
                Err(e) => {
                    warn!("Screen video: stream {stream_id} hardware_buffer: {e}");
                    return;
                }
            };
            let Ok(index) = self.swapchain.acquire_image() else {
                return;
            };
            if self.swapchain.wait_image(xr::Duration::INFINITE).is_err() {
                return;
            }
            let Some(target) = self
                .images
                .get(index as usize)
                .copied()
                .and_then(std::num::NonZeroU32::new)
                .map(gl::NativeTexture)
            else {
                return;
            };

            self.context.make_current();
            let gl = &self.context.gl_context;
            let timing = self
                .last_timing_log
                .is_none_or(|instant| instant.elapsed() >= MIP_TIMING_LOG_INTERVAL);
            let start = timing.then(|| {
                unsafe { gl.finish() };
                Instant::now()
            });

            let guard = GlStateGuard::save(gl);
            unsafe {
                gl.active_texture(gl::TEXTURE0);
                self.context.render_ahardwarebuffer_using_texture(
                    buffer.as_ptr().cast(),
                    self.oes_texture,
                    || {
                        gl.bind_framebuffer(gl::DRAW_FRAMEBUFFER, Some(self.framebuffer));
                        gl.framebuffer_texture_2d(
                            gl::DRAW_FRAMEBUFFER,
                            gl::COLOR_ATTACHMENT0,
                            gl::TEXTURE_2D,
                            Some(target),
                            0,
                        );
                        gl.viewport(0, 0, self.size.x as i32, self.size.y as i32);
                        gl.disable(gl::SCISSOR_TEST);
                        gl.disable(gl::STENCIL_TEST);
                        gl.disable(gl::DEPTH_TEST);
                        gl.disable(gl::BLEND);
                        gl.disable(gl::CULL_FACE);
                        gl.color_mask(true, true, true, true);
                        gl.use_program(Some(program.program));
                        gl.uniform_1_i32(program.tex_uloc.as_ref(), 0);
                        gl.uniform_1_i32(program.linearize_uloc.as_ref(), i32::from(self.srgb));
                        gl.bind_sampler(0, None);
                        gl.bind_vertex_array(Some(program.vertex_array));
                        gl.draw_arrays(gl::TRIANGLE_STRIP, 0, 4);
                    },
                );
                if self.mip_levels > 1 {
                    gl.bind_texture(gl::TEXTURE_2D, Some(target));
                    gl.generate_mipmap(gl::TEXTURE_2D);
                }
                gl.flush();
            }
            // Everything above (VAO, program, framebuffers, texture/sampler bindings, viewport,
            // enables, color mask) back to what wgpu expects
            drop(guard);

            if let Some(start) = start {
                unsafe { gl.finish() };
                let gl_error = unsafe { gl.get_error() };
                info!(
                    "Screen video: stream {stream_id} mipmapped frame {}x{} ({} levels) drawn and \
                     mip-mapped in {:.2} ms (GPU, synchronous sample), {} frames so far, GL error \
                     {gl_error:#x}",
                    self.size.x,
                    self.size.y,
                    self.mip_levels,
                    start.elapsed().as_secs_f32() * 1000.0,
                    self.frames + 1
                );
                self.last_timing_log = Some(Instant::now());
            }

            self.swapchain.release_image().ok();
            // The previous frame's buffer goes back to the decoder now that a newer one is drawn
            self.current_image = Some(image);
            self.frames += 1;
        }
    }

    impl Drop for MipTarget {
        fn drop(&mut self) {
            self.context.make_current();
            let gl = &self.context.gl_context;
            unsafe {
                gl.delete_texture(self.oes_texture);
                gl.delete_framebuffer(self.framebuffer);
            }
        }
    }

    // Access units waiting for a decoder input buffer; beyond this, wait for a keyframe
    const MAX_PENDING_FRAMES: usize = 30;
    // A stream is closed only when no quad has shown it for this long. It is never closed for being
    // idle: the overlay app encodes on change, so a static screen sends nothing between (rare)
    // keyframes, and the decoder must keep showing its last frame. The grace period covers the
    // quads being withdrawn for a while (a calibration pass, a STAGE epoch change).
    const UNREFERENCED_STREAM_TIMEOUT: Duration = Duration::from_secs(30);
    // A quad that references a stream without a decoder (not started, closed, failed, recreated
    // after errors) asks for a keyframe at most this often, instead of waiting for the next
    // periodic one
    const MISSING_STREAM_IDR_INTERVAL: Duration = Duration::from_secs(1);
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
        // Frames released to the zero copy swapchain (it has content once > 0)
        zero_copy_frames: u64,
        queued_frames: u64,
        decoder_errors: u32,
        // Last time a quad showed this stream (creation counts)
        last_referenced: Instant,
        last_idr_request: Option<Instant>,
        path: VideoPath,
        // Created at the first switch to the mipmapped path, kept afterwards
        mip: Option<MipTarget>,
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
                zero_copy_frames: 0,
                queued_frames: 0,
                decoder_errors: 0,
                last_referenced: Instant::now(),
                last_idr_request: None,
                path: VideoPath::ZeroCopy,
                mip: None,
            })
        }

        // Moves the decoder output to the other path without restarting the decoder, and asks
        // for a keyframe so that the new path gets a picture soon (static screens send few frames)
        fn switch_path(
            &mut self,
            path: VideoPath,
            reason: &str,
            xr_session: &xr::Session<xr::OpenGlEs>,
            gfx_ctx: &Rc<GraphicsContext>,
            core_ctx: &ClientCoreContext,
        ) {
            if path == self.path {
                return;
            }
            if path == VideoPath::Mipmapped && self.mip.is_none() {
                let size = UVec2::new(self.header.width, self.header.height);
                match MipTarget::new(xr_session, gfx_ctx, size) {
                    Ok(mip) => self.mip = Some(mip),
                    Err(e) => {
                        warn!(
                            "Screen video: stream {} stays zero copy: mipmapped target failed: {e}",
                            self.header.stream_id
                        );
                        return;
                    }
                }
            }
            let window = match (path, &self.mip) {
                (VideoPath::Mipmapped, Some(mip)) => &mip.window,
                _ => &self._window,
            };
            if let Err(e) = self.decoder.set_output_surface(window) {
                warn!(
                    "Screen video: stream {} setOutputSurface for {path:?} failed: {e}",
                    self.header.stream_id
                );
                return;
            }
            info!(
                "Screen video: stream {} {}x{} path {path:?} ({reason})",
                self.header.stream_id, self.header.width, self.header.height
            );
            self.path = path;
            // A stream that has not decoded anything yet gets its first keyframe on the new path
            // anyway
            if self.decoded_frames > 0 {
                self.last_idr_request = None;
                self.request_idr(core_ctx);
            }
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
                        if self.path == VideoPath::ZeroCopy {
                            self.zero_copy_frames += 1;
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

    pub struct ScreenVideoStreams {
        streams: HashMap<u32, ScreenVideoStream>,
        // Keyframe requests for streams a quad shows but that have no decoder
        missing_stream_idr_requests: HashMap<u32, Instant>,
        // Streams that could not be created (logged once)
        failed_streams: HashMap<u32, String>,
        // Recommended render width per eye: with the view's FoV, the display's angular resolution
        display_width: u32,
        mode: PathMode,
        last_path_poll: Option<Instant>,
        blit_program: Option<BlitProgram>,
        blit_program_failed: bool,
        logged_display_ppd: bool,
    }

    impl ScreenVideoStreams {
        pub fn new(display_width: u32) -> Self {
            Self {
                streams: HashMap::new(),
                missing_stream_idr_requests: HashMap::new(),
                failed_streams: HashMap::new(),
                display_width,
                mode: PathMode::Mipmapped,
                last_path_poll: None,
                blit_program: None,
                blit_program_failed: false,
                logged_display_ppd: false,
            }
        }

        // Once a second: the switch file, and for auto the minification of each stream's quad
        fn choose_paths(
            &mut self,
            xr_session: &xr::Session<xr::OpenGlEs>,
            gfx_ctx: &Rc<GraphicsContext>,
            core_ctx: &ClientCoreContext,
            stage_space: Option<&xr::Space>,
            display_time: xr::Time,
            quads: &[VideoQuadGeometry],
        ) {
            if self
                .last_path_poll
                .is_some_and(|instant| instant.elapsed() < PATH_POLL_INTERVAL)
            {
                return;
            }
            self.last_path_poll = Some(Instant::now());

            let mode = match crate::debug_switches::get().video_path.as_str() {
                "zero-copy" => PathMode::ZeroCopy,
                "auto" => PathMode::Auto,
                // Default (no switch file): mipmapped for every screen
                _ => PathMode::Mipmapped,
            };
            if mode != self.mode {
                info!("Screen video: path mode {mode:?}");
                self.mode = mode;
            }

            // Head position and the display's pixels per degree at the view centre
            let head = stage_space.and_then(|space| {
                let (flags, views) = xr_session
                    .locate_views(
                        xr::ViewConfigurationType::PRIMARY_STEREO,
                        display_time,
                        space,
                    )
                    .ok()?;
                if !flags.contains(xr::ViewStateFlags::POSITION_VALID) || views.len() < 2 {
                    return None;
                }
                let position = (crate::from_xr_vec3(views[0].pose.position)
                    + crate::from_xr_vec3(views[1].pose.position))
                    / 2.0;
                let fov = views[0].fov;
                let ppd = self.display_width as f32
                    / (fov.angle_right.tan() - fov.angle_left.tan())
                    * std::f32::consts::PI
                    / 180.0;
                Some((position, ppd))
            });
            if let Some((_, ppd)) = head
                && !self.logged_display_ppd
            {
                info!("Screen video: display {ppd:.1} pixels per degree at the view centre");
                self.logged_display_ppd = true;
            }

            for (stream_id, stream) in &mut self.streams {
                let minification = head.and_then(|(position, ppd)| {
                    let quad = quads.iter().find(|quad| quad.stream_id == *stream_id)?;
                    let distance = quad.position.distance(position).max(0.05);
                    let angle_deg = 2.0 * (quad.width / 2.0 / distance).atan().to_degrees();
                    Some((stream.header.width as f32 / angle_deg / ppd, distance))
                });
                let (path, reason) = match mode {
                    PathMode::ZeroCopy => (VideoPath::ZeroCopy, "forced".to_owned()),
                    PathMode::Mipmapped => (VideoPath::Mipmapped, "forced".to_owned()),
                    PathMode::Auto => match (stream.path, minification) {
                        (VideoPath::ZeroCopy, Some((m, d))) if m > AUTO_MIPMAPPED_ABOVE => (
                            VideoPath::Mipmapped,
                            format!("auto: {m:.1}x minified at {d:.2} m"),
                        ),
                        (VideoPath::Mipmapped, Some((m, d))) if m < AUTO_ZERO_COPY_BELOW => (
                            VideoPath::ZeroCopy,
                            format!("auto: {m:.1}x minified at {d:.2} m"),
                        ),
                        (path, _) => (path, String::new()),
                    },
                };
                stream.switch_path(path, &reason, xr_session, gfx_ctx, core_ctx);
            }
        }

        // Once per rendered frame
        pub fn update(
            &mut self,
            xr_session: &xr::Session<xr::OpenGlEs>,
            gfx_ctx: &Rc<GraphicsContext>,
            core_ctx: &ClientCoreContext,
            stage_space: Option<&xr::Space>,
            display_time: xr::Time,
            quads: &[VideoQuadGeometry],
        ) {
            while let Some((header, data)) = core_ctx.poll_screen_video_frame() {
                let compatible = self.streams.get(&header.stream_id).is_some_and(|stream| {
                    stream.header.codec == header.codec
                        && stream.header.width == header.width
                        && stream.header.height == header.height
                        && stream.decoder_errors <= MAX_DECODER_ERRORS
                });
                if !compatible {
                    // No decoder for a stream no quad shows (calibration failed, overlay mode,
                    // a superseded stream id): drop its frames, no decoder, no keyframe request.
                    // Once a quad shows it, the quad loop below asks for a keyframe.
                    if !quads.iter().any(|quad| quad.stream_id == header.stream_id) {
                        continue;
                    }
                    if !header.is_keyframe {
                        // Decoding starts at a keyframe: ask for one now
                        self.request_missing_stream_idr(core_ctx, header.stream_id);
                        continue;
                    }
                    self.streams.remove(&header.stream_id);
                    match ScreenVideoStream::new(xr_session, &header, &data) {
                        Ok(stream) => {
                            self.failed_streams.remove(&header.stream_id);
                            let mut stream = stream;
                            // A new stream starts on the default path before its first keyframe is
                            // decoded (no second keyframe needed)
                            if self.mode == PathMode::Mipmapped {
                                stream.switch_path(
                                    VideoPath::Mipmapped,
                                    "default",
                                    xr_session,
                                    gfx_ctx,
                                    core_ctx,
                                );
                            }
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

            self.choose_paths(
                xr_session,
                gfx_ctx,
                core_ctx,
                stage_space,
                display_time,
                quads,
            );

            if self
                .streams
                .values()
                .any(|stream| stream.path == VideoPath::Mipmapped)
                && self.blit_program.is_none()
                && !self.blit_program_failed
            {
                gfx_ctx.make_current();
                match create_blit_program(&gfx_ctx.gl_context) {
                    Ok(program) => self.blit_program = Some(program),
                    Err(e) => {
                        warn!("Screen video: blit program: {e}");
                        self.blit_program_failed = true;
                    }
                }
            }
            if let Some(program) = &self.blit_program {
                for (stream_id, stream) in &mut self.streams {
                    if stream.path == VideoPath::Mipmapped
                        && let Some(mip) = &mut stream.mip
                    {
                        mip.latch(program, *stream_id);
                    }
                }
            }

            // Quads showing a stream that has no decoder: ask for a keyframe right away
            for quad in quads {
                if !self.streams.contains_key(&quad.stream_id)
                    && !self.failed_streams.contains_key(&quad.stream_id)
                {
                    self.request_missing_stream_idr(core_ctx, quad.stream_id);
                }
            }

            self.streams.retain(|stream_id, stream| {
                if quads.iter().any(|quad| quad.stream_id == *stream_id) {
                    stream.last_referenced = Instant::now();
                }
                let keep = stream.last_referenced.elapsed() < UNREFERENCED_STREAM_TIMEOUT;
                if !keep {
                    info!(
                        "Screen video: stream {stream_id} closed, no quad shows it ({} frames \
                         decoded)",
                        stream.decoded_frames
                    );
                }
                keep
            });
            let streams = &self.streams;
            self.missing_stream_idr_requests
                .retain(|stream_id, _| !streams.contains_key(stream_id));
        }

        fn request_missing_stream_idr(&mut self, core_ctx: &ClientCoreContext, stream_id: u32) {
            if self
                .missing_stream_idr_requests
                .get(&stream_id)
                .is_some_and(|instant| instant.elapsed() < MISSING_STREAM_IDR_INTERVAL)
            {
                return;
            }
            self.missing_stream_idr_requests
                .insert(stream_id, Instant::now());
            info!("Screen video: stream {stream_id} has no decoder, requesting a keyframe");
            core_ctx.request_screen_idr(stream_id);
        }

        /// The swapchain of a stream that has decoded at least one frame, and its size
        pub fn ready_swapchain(
            &self,
            stream_id: u32,
        ) -> Option<(&xr::Swapchain<xr::OpenGlEs>, UVec2)> {
            let stream = self.streams.get(&stream_id)?;
            let zero_copy = (stream.zero_copy_frames > 0).then(|| {
                (
                    &stream.swapchain,
                    UVec2::new(stream.header.width, stream.header.height),
                )
            });
            let mipmapped = stream
                .mip
                .as_ref()
                .filter(|mip| mip.frames > 0)
                .map(|mip| (&mip.swapchain, mip.size));

            // The current path once it has a picture, else whichever has one
            match stream.path {
                VideoPath::Mipmapped => mipmapped.or(zero_copy),
                VideoPath::ZeroCopy => zero_copy.or(mipmapped),
            }
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
