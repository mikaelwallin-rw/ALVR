// Debug reference quads requested by an overlay app (ScreenVR), drawn as OpenXR quad layers that
// the headset runtime composites itself, without the streaming path. Each quad shows a
// procedural chessboard with a border, at a pose in the STAGE space, so that it can be compared
// with the real object seen in passthrough (e.g. the same chessboard shown on a monitor).
// A quad with a video stream id shows that screen video stream instead (screen_video.rs), once
// the stream has decoded a frame.
// A kept quad (same index, or same video stream) that moves a lot at once (scene mode switching the
// touchscreen between its passthrough-compensated and true pose, a recalibration) glides there over
// MOVE_DURATION (ease in/out) instead of jumping; small updates (trim, live tracking) are immediate.

use crate::{
    graphics,
    screen_video::{ScreenVideoStreams, VideoQuadGeometry},
};
use alvr_common::{Pose, glam::UVec2, info, warn};
use alvr_graphics::GraphicsContext;
use alvr_packets::NativeQuad;
use glow::{self as gl, HasContext};
use openxr as xr;
use std::{
    fs::File,
    io::BufReader,
    mem,
    num::NonZeroU32,
    path::Path,
    time::{Duration, Instant},
};

// Approximate texture side, along the longer chessboard axis
const TEXTURE_TARGET_SIZE: u32 = 1024;
const MIN_PIXELS_PER_SQUARE: u32 = 8;
const BYTES_PER_PIXEL: usize = 4;
// TEST HOOK (desktop-fidelity test, to be replaced by a live screen transport):
// <dir>/screenvr_quad_<index>.png replaces the chessboard of quad <index>. The app's external
// files dir needs no permission; Download needs READ_MEDIA_IMAGES.
const IMAGE_FNAME_PREFIX: &str = "screenvr_quad_";
const IMAGE_DIRS: [&str; 2] = [
    "/sdcard/Download",
    "/sdcard/Android/data/alvr.client.dev/files",
];

struct QuadLayer {
    // Index of the quad in the overlay app's block (the PNG test hook is keyed by it)
    index: usize,
    quad: NativeQuad,
    swapchain: xr::Swapchain<xr::OpenGlEs>,
    size: UVec2,
    // Gliding from this pose/size to `quad`'s (see MOVE_DURATION)
    moving: Option<Move>,
}

#[derive(Clone, Copy)]
struct Move {
    start: Instant,
    pose: Pose,
    width: f32,
    height: f32,
}

// A move this large is animated (anything smaller is applied at once)
const MOVE_MIN_METERS: f32 = 0.010;
const MOVE_MIN_DEGREES: f32 = 0.5;
const MOVE_MIN_SCALE: f32 = 0.01;
// Same as the loading screen's fades
const MOVE_DURATION: Duration = Duration::from_millis(500);

fn ease_in_out(x: f32) -> f32 {
    let x = x.clamp(0.0, 1.0);
    0.5 - 0.5 * (std::f32::consts::PI * x).cos()
}

impl QuadLayer {
    /// Pose, width and height to draw now
    fn displayed(&self, now: Instant) -> (Pose, f32, f32) {
        let target = (self.quad.pose, self.quad.width, self.quad.height);
        let Some(from) = self.moving else {
            return target;
        };
        let k = ease_in_out(
            now.saturating_duration_since(from.start).as_secs_f32() / MOVE_DURATION.as_secs_f32(),
        );
        (
            Pose {
                orientation: from.pose.orientation.slerp(target.0.orientation, k),
                position: from.pose.position.lerp(target.0.position, k),
            },
            from.width + (target.1 - from.width) * k,
            from.height + (target.2 - from.height) * k,
        )
    }

    /// The quad's target changes to `quad`: glide if it is a large move
    fn retarget(&mut self, quad: NativeQuad, now: Instant) {
        let (pose, width, height) = self.displayed(now);
        let meters = pose.position.distance(quad.pose.position);
        let degrees = pose
            .orientation
            .angle_between(quad.pose.orientation)
            .to_degrees();
        let scale = quad.width / width.max(1e-6);
        let large = meters >= MOVE_MIN_METERS
            || degrees >= MOVE_MIN_DEGREES
            || (scale - 1.0).abs() >= MOVE_MIN_SCALE;
        let animating = self
            .moving
            .is_some_and(|from| now.saturating_duration_since(from.start) < MOVE_DURATION);
        if large {
            info!(
                "Native quads: quad {} (stream {}) glides {:.0} mm, {degrees:.1} deg, scale {scale:.3} \
                 over {} ms",
                self.index,
                quad.video_stream_id,
                meters * 1000.0,
                MOVE_DURATION.as_millis()
            );
            self.moving = Some(Move {
                start: now,
                pose,
                width,
                height,
            });
        } else if !animating {
            self.moving = None;
        }
        // A small update during a glide moves its target; the glide continues
        self.quad = quad;
    }
}

// Pose/size-only updates are frequent (trim, live driverFromStage): logged as a count
const IN_PLACE_LOG_INTERVAL: Duration = Duration::from_secs(5);

#[derive(Default)]
pub struct NativeQuadLayers {
    layers: Vec<QuadLayer>,
    in_place_updates: u32,
    last_in_place_log: Option<Instant>,
}

// Same chessboard texture: only the pose, size or video stream changed
fn same_texture(a: &NativeQuad, b: &NativeQuad) -> bool {
    (a.columns, a.rows, a.outline_pixels) == (b.columns, b.rows, b.outline_pixels)
        && (a.dark_color, a.light_color, a.outline_color)
            == (b.dark_color, b.light_color, b.outline_color)
}

impl NativeQuadLayers {
    // Replaces all quads. A quad whose texture is unchanged at the same index, or with the same
    // video stream, keeps its swapchain (pose, size or video stream changes), also when other quads
    // appear or disappear (scene mode).
    // Other textures are drawn once here. Screen video decoders are independent of the quads
    // (screen_video.rs), so no change here restarts a decoder.
    pub fn set(
        &mut self,
        xr_session: &xr::Session<xr::OpenGlEs>,
        gfx_ctx: &GraphicsContext,
        quads: Vec<NativeQuad>,
    ) {
        let mut old_layers = mem::take(&mut self.layers);
        let mut new_quads = Vec::new();
        for (index, quad) in quads.into_iter().enumerate() {
            // A video quad keeps its layer by stream id: its index changes when scene mode hides
            // the other screens
            if let Some(position) = old_layers.iter().position(|layer| {
                same_texture(&layer.quad, &quad)
                    && (layer.index == index
                        || quad.video_stream_id != 0
                            && layer.quad.video_stream_id == quad.video_stream_id)
            }) {
                let mut layer = old_layers.swap_remove(position);
                layer.index = index;
                layer.retarget(quad, Instant::now());
                self.layers.push(layer);
            } else {
                new_quads.push((index, quad));
            }
        }
        let reused = self.layers.len();
        let removed = old_layers.len();
        drop(old_layers);
        // Layers are submitted in the block's order
        self.layers.sort_by_key(|layer| layer.index);

        if new_quads.is_empty() && removed == 0 {
            self.in_place_updates += 1;
            if self
                .last_in_place_log
                .is_none_or(|instant| instant.elapsed() >= IN_PLACE_LOG_INTERVAL)
            {
                info!(
                    "Native quads: {} pose/size update(s) applied in place to {reused} layer(s), no \
                     swapchain recreated",
                    self.in_place_updates
                );
                self.in_place_updates = 0;
                self.last_in_place_log = Some(Instant::now());
            }
            return;
        }

        if !new_quads.is_empty() {
            gfx_ctx.make_current();
            // Chessboard colors and desktop images are sRGB-encoded
            let (format, srgb) = graphics::srgb_swapchain_format(xr_session);
            info!(
                "Native quads: swapchain formats offered {:#x?}, using {format:#x}{}",
                xr_session.enumerate_swapchain_formats().unwrap_or_default(),
                if srgb {
                    " (sRGB)"
                } else {
                    " (UNORM, linearizing on upload)"
                }
            );

            for (index, quad) in new_quads {
                match create_layer(xr_session, gfx_ctx, quad, index, format, srgb) {
                    Ok(layer) => self.layers.push(layer),
                    Err(e) => warn!("Native quads: cannot create a quad layer: {e}"),
                }
            }
            self.layers.sort_by_key(|layer| layer.index);
        }
        info!(
            "Native quads: {} quad layer(s) ({reused} kept, {removed} removed): {:?}",
            self.layers.len(),
            self.layers
                .iter()
                .map(|layer| (
                    layer.index,
                    layer.quad.width,
                    layer.quad.height,
                    layer.size,
                    layer.quad.video_stream_id
                ))
                .collect::<Vec<_>>()
        );
    }

    /// Position and width of the quads that show a video stream (for the decode path choice)
    pub fn video_quad_geometry(&self) -> Vec<VideoQuadGeometry> {
        self.layers
            .iter()
            .filter(|layer| layer.quad.video_stream_id != 0)
            .map(|layer| VideoQuadGeometry {
                stream_id: layer.quad.video_stream_id,
                position: layer.quad.pose.position,
                width: layer.quad.width,
            })
            .collect()
    }

    pub fn clear(&mut self) {
        if !self.layers.is_empty() {
            info!("Native quads: cleared");
        }
        self.layers.clear();
    }

    pub fn build<'a>(
        &'a self,
        stage_space: &'a xr::Space,
        screen_video: &'a ScreenVideoStreams,
    ) -> Vec<xr::CompositionLayerQuad<'a, xr::OpenGlEs>> {
        let now = Instant::now();
        self.layers
            .iter()
            .map(|layer| {
                let (pose, width, height) = layer.displayed(now);
                // Video (opaque) once the stream has a decoded frame, else the chessboard
                let video = (layer.quad.video_stream_id != 0)
                    .then(|| screen_video.ready_swapchain(layer.quad.video_stream_id))
                    .flatten();
                let (swapchain, size, flags) = match video {
                    Some((swapchain, size)) => (swapchain, size, xr::CompositionLayerFlags::EMPTY),
                    None => (
                        &layer.swapchain,
                        layer.size,
                        xr::CompositionLayerFlags::BLEND_TEXTURE_SOURCE_ALPHA
                            | xr::CompositionLayerFlags::UNPREMULTIPLIED_ALPHA,
                    ),
                };

                xr::CompositionLayerQuad::new()
                    .layer_flags(flags)
                    .space(stage_space)
                    .eye_visibility(xr::EyeVisibility::BOTH)
                    .sub_image(
                        xr::SwapchainSubImage::new()
                            .swapchain(swapchain)
                            .image_array_index(0)
                            .image_rect(xr::Rect2Di {
                                offset: xr::Offset2Di { x: 0, y: 0 },
                                extent: xr::Extent2Di {
                                    width: size.x as _,
                                    height: size.y as _,
                                },
                            }),
                    )
                    .pose(crate::to_xr_pose(pose))
                    .size(xr::Extent2Df { width, height })
            })
            .collect()
    }
}

fn texture_size(quad: &NativeQuad) -> UVec2 {
    let pixels_per_square =
        (TEXTURE_TARGET_SIZE / quad.columns.max(quad.rows)).max(MIN_PIXELS_PER_SQUARE);

    UVec2::new(quad.columns, quad.rows) * pixels_per_square
}

// RGBA8 pixels, rows from the top of the quad down
fn chessboard_pixels(quad: &NativeQuad, size: UVec2) -> Vec<u8> {
    let pixels_per_square = size.x / quad.columns;
    let outline = quad.outline_pixels;

    let mut pixels = Vec::with_capacity(size.x as usize * size.y as usize * BYTES_PER_PIXEL);
    for y in 0..size.y {
        for x in 0..size.x {
            let is_outline =
                x < outline || y < outline || x + outline >= size.x || y + outline >= size.y;
            let color = if is_outline {
                quad.outline_color
            } else if ((x / pixels_per_square) + (y / pixels_per_square)).is_multiple_of(2) {
                quad.dark_color
            } else {
                quad.light_color
            };
            pixels.extend_from_slice(&color);
        }
    }

    pixels
}

struct QuadTexture {
    size: UVec2,
    // RGBA8, rows from the top of the quad down
    pixels: Vec<u8>,
    mip_levels: u32,
    source: String,
}

fn chessboard_texture(quad: &NativeQuad) -> QuadTexture {
    let size = texture_size(quad);

    QuadTexture {
        size,
        pixels: chessboard_pixels(quad, size),
        mip_levels: 1,
        source: "chessboard".into(),
    }
}

// PNG decoded to RGBA8 rows, top down
fn decode_png(path: &Path) -> Result<(UVec2, Vec<u8>), String> {
    let file = File::open(path).map_err(|e| e.to_string())?;
    let mut decoder = png::Decoder::new(BufReader::new(file));
    decoder.set_transformations(png::Transformations::normalize_to_color8());
    let mut reader = decoder.read_info().map_err(|e| e.to_string())?;
    let mut buffer = vec![0; reader.output_buffer_size()];
    let info = reader.next_frame(&mut buffer).map_err(|e| e.to_string())?;
    buffer.truncate(info.buffer_size());

    let pixels = match info.color_type {
        png::ColorType::Rgba => buffer,
        png::ColorType::Rgb => buffer
            .chunks_exact(3)
            .flat_map(|rgb| [rgb[0], rgb[1], rgb[2], u8::MAX])
            .collect(),
        png::ColorType::GrayscaleAlpha => buffer
            .chunks_exact(2)
            .flat_map(|la| [la[0], la[0], la[0], la[1]])
            .collect(),
        png::ColorType::Grayscale => buffer.iter().flat_map(|&l| [l, l, l, u8::MAX]).collect(),
        png::ColorType::Indexed => return Err("palette not expanded".into()),
    };
    let expected_len = info.width as usize * info.height as usize * BYTES_PER_PIXEL;
    if pixels.len() != expected_len {
        return Err(format!("unexpected pixel data size {}", pixels.len()));
    }

    Ok((UVec2::new(info.width, info.height), pixels))
}

// Full resolution image for the quad at `index` in the quads block, when one was pushed to the
// headset (desktop-fidelity test)
fn image_texture(index: usize, max_texture_size: u32) -> Option<QuadTexture> {
    let fname = format!("{IMAGE_FNAME_PREFIX}{index}.png");
    let path = IMAGE_DIRS
        .iter()
        .map(|dir| Path::new(dir).join(&fname))
        .find(|path| path.exists())?;

    let start = Instant::now();
    match decode_png(&path) {
        Ok((size, _)) if size.max_element() > max_texture_size => {
            warn!(
                "Native quads: {} is {}x{}, above the GL limit {max_texture_size}",
                path.display(),
                size.x,
                size.y
            );

            None
        }
        Ok((size, pixels)) => {
            info!(
                "Native quads: quad {index}: {} {}x{} decoded in {} ms",
                path.display(),
                size.x,
                size.y,
                start.elapsed().as_millis()
            );

            Some(QuadTexture {
                size,
                pixels,
                mip_levels: size.max_element().ilog2() + 1,
                source: path.display().to_string(),
            })
        }
        Err(e) => {
            warn!("Native quads: cannot read {}: {e}", path.display());

            None
        }
    }
}

// sRGB-encoded RGBA8 to linear RGBA8 (alpha unchanged), for UNORM swapchains, which the compositor
// treats as linear
fn linearize(pixels: &mut [u8]) {
    let table: [u8; 256] = std::array::from_fn(|value| {
        let encoded = value as f32 / 255.0;
        let linear = if encoded <= 0.04045 {
            encoded / 12.92
        } else {
            ((encoded + 0.055) / 1.055).powf(2.4)
        };

        (linear * 255.0).round() as u8
    });

    for pixel in pixels.chunks_exact_mut(BYTES_PER_PIXEL) {
        for channel in &mut pixel[..3] {
            *channel = table[*channel as usize];
        }
    }
}

fn create_swapchain(
    xr_session: &xr::Session<xr::OpenGlEs>,
    size: UVec2,
    mip_count: u32,
    format: u32,
) -> xr::Result<xr::Swapchain<xr::OpenGlEs>> {
    xr_session.create_swapchain(&xr::SwapchainCreateInfo {
        create_flags: xr::SwapchainCreateFlags::EMPTY,
        usage_flags: xr::SwapchainUsageFlags::COLOR_ATTACHMENT | xr::SwapchainUsageFlags::SAMPLED,
        format,
        sample_count: 1,
        width: size.x,
        height: size.y,
        face_count: 1,
        array_size: 1,
        mip_count,
    })
}

fn create_layer(
    xr_session: &xr::Session<xr::OpenGlEs>,
    gfx_ctx: &GraphicsContext,
    quad: NativeQuad,
    index: usize,
    format: u32,
    srgb: bool,
) -> Result<QuadLayer, String> {
    if quad.columns == 0 || quad.rows == 0 {
        return Err("empty chessboard".into());
    }

    gfx_ctx.make_current();
    let gl_context = &gfx_ctx.gl_context;

    let max_texture_size = unsafe { gl_context.get_parameter_i32(gl::MAX_TEXTURE_SIZE) } as u32;
    let mut texture =
        image_texture(index, max_texture_size).unwrap_or_else(|| chessboard_texture(&quad));
    let size = texture.size;

    let mut swapchain = match create_swapchain(xr_session, size, texture.mip_levels, format) {
        Ok(swapchain) => swapchain,
        Err(e) if texture.mip_levels > 1 => {
            warn!(
                "Native quads: quad {index}: the runtime rejects {} mip levels ({e}), using 1",
                texture.mip_levels
            );
            texture.mip_levels = 1;

            create_swapchain(xr_session, size, 1, format)
                .map_err(|e| format!("create_swapchain: {e}"))?
        }
        Err(e) => return Err(format!("create_swapchain: {e}")),
    };

    let images = swapchain
        .enumerate_images()
        .map_err(|e| format!("enumerate_images: {e}"))?;

    if !srgb {
        linearize(&mut texture.pixels);
    }

    // OpenGL swapchain images have their origin at the bottom left: upload the rows bottom-up
    let row_bytes = size.x as usize * BYTES_PER_PIXEL;
    let flipped = texture
        .pixels
        .chunks_exact(row_bytes)
        .rev()
        .flatten()
        .copied()
        .collect::<Vec<_>>();
    texture.pixels = Vec::new();

    let start = Instant::now();
    // Leave the GL state as wgpu expects it (gl_state.rs)
    let guard = crate::gl_state::GlStateGuard::save(gl_context);
    // Fill every image, so that whichever one the runtime keeps showing is correct
    for _ in 0..images.len() {
        let image_index = swapchain
            .acquire_image()
            .map_err(|e| format!("acquire_image: {e}"))?;
        swapchain
            .wait_image(xr::Duration::INFINITE)
            .map_err(|e| format!("wait_image: {e}"))?;

        let gl_texture = images
            .get(image_index as usize)
            .copied()
            .and_then(NonZeroU32::new)
            .ok_or("invalid swapchain image")?;
        unsafe {
            gl_context.bind_texture(gl::TEXTURE_2D, Some(gl::NativeTexture(gl_texture)));
            gl_context.pixel_store_i32(gl::UNPACK_ALIGNMENT, 1);
            // wgpu's GLES backend leaves UNPACK_ROW_LENGTH set after its own texture uploads (e.g.
            // 1024 from the lobby HUD), which would make this upload read rows with that pitch
            gl_context.pixel_store_i32(gl::UNPACK_ROW_LENGTH, 0);
            gl_context.pixel_store_i32(gl::UNPACK_SKIP_ROWS, 0);
            gl_context.pixel_store_i32(gl::UNPACK_SKIP_PIXELS, 0);
            gl_context.bind_buffer(gl::PIXEL_UNPACK_BUFFER, None);
            gl_context.tex_sub_image_2d(
                gl::TEXTURE_2D,
                0,
                0,
                0,
                size.x as _,
                size.y as _,
                gl::RGBA,
                gl::UNSIGNED_BYTE,
                gl::PixelUnpackData::Slice(Some(&flipped)),
            );
            if texture.mip_levels > 1 {
                gl_context.generate_mipmap(gl::TEXTURE_2D);
            }
            gl_context.bind_texture(gl::TEXTURE_2D, None);
            gl_context.flush();
        }

        swapchain
            .release_image()
            .map_err(|e| format!("release_image: {e}"))?;
    }

    drop(guard);
    let gl_error = unsafe { gl_context.get_error() };
    info!(
        "Native quads: quad {index}: {} {}x{}, {} mip level(s), {} swapchain images filled in {} \
         ms, GL error {gl_error:#x}",
        texture.source,
        size.x,
        size.y,
        texture.mip_levels,
        images.len(),
        start.elapsed().as_millis()
    );

    Ok(QuadLayer {
        index,
        quad,
        swapchain,
        size,
        moving: None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use alvr_common::Pose;

    fn quad(columns: u32, rows: u32) -> NativeQuad {
        NativeQuad {
            pose: Pose::IDENTITY,
            width: 0.216,
            height: 0.136,
            columns,
            rows,
            dark_color: [0, 0, 0, 255],
            light_color: [255, 255, 255, 0],
            outline_color: [0, 255, 0, 255],
            outline_pixels: 2,
            video_stream_id: 0,
        }
    }

    #[test]
    fn pose_and_video_changes_keep_the_texture() {
        let a = quad(16, 10);
        let moved = NativeQuad {
            pose: Pose {
                position: alvr_common::glam::Vec3::X,
                ..Pose::IDENTITY
            },
            width: 0.3,
            video_stream_id: 4,
            ..a
        };
        assert!(same_texture(&a, &moved));
        assert!(!same_texture(&a, &quad(16, 11)));
    }

    #[test]
    fn chessboard_layout() {
        let quad = quad(16, 10);
        let size = texture_size(&quad);
        assert_eq!(size, UVec2::new(1024, 640));

        let pixels = chessboard_pixels(&quad, size);
        let at = |x: u32, y: u32| {
            let i = (y as usize * size.x as usize + x as usize) * BYTES_PER_PIXEL;
            [pixels[i], pixels[i + 1], pixels[i + 2], pixels[i + 3]]
        };
        assert_eq!(at(0, 0), quad.outline_color);
        assert_eq!(at(1, 300), quad.outline_color);
        assert_eq!(at(10, 10), quad.dark_color);
        assert_eq!(at(64 + 10, 10), quad.light_color);
        assert_eq!(at(64 + 10, 64 + 10), quad.dark_color);
        assert_eq!(at(size.x - 2, 300), quad.outline_color);
    }

    #[test]
    fn many_squares_keep_minimum_square_size() {
        assert_eq!(texture_size(&quad(256, 1)), UVec2::new(2048, 8));
    }
}
