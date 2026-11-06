// Debug reference quads requested by an overlay app (ScreenVR), drawn as OpenXR quad layers that
// the headset runtime composites itself, without the streaming path. Each quad shows a
// procedural chessboard with a border, at a pose in the STAGE space, so that it can be compared
// with the real object seen in passthrough (e.g. the same chessboard shown on a monitor).

use alvr_common::{glam::UVec2, info, warn};
use alvr_graphics::{GraphicsContext, SDR_FORMAT_GL};
use alvr_packets::NativeQuad;
use glow::{self as gl, HasContext};
use openxr as xr;
use std::num::NonZeroU32;

// Approximate texture side, along the longer chessboard axis
const TEXTURE_TARGET_SIZE: u32 = 1024;
const MIN_PIXELS_PER_SQUARE: u32 = 8;
const BYTES_PER_PIXEL: usize = 4;

struct QuadLayer {
    quad: NativeQuad,
    swapchain: xr::Swapchain<xr::OpenGlEs>,
    size: UVec2,
}

#[derive(Default)]
pub struct NativeQuadLayers {
    layers: Vec<QuadLayer>,
}

impl NativeQuadLayers {
    // Replaces all quads. Textures are drawn once here; the layers are static afterwards.
    pub fn set(
        &mut self,
        xr_session: &xr::Session<xr::OpenGlEs>,
        gfx_ctx: &GraphicsContext,
        quads: Vec<NativeQuad>,
    ) {
        self.layers.clear();

        for quad in quads {
            match create_layer(xr_session, gfx_ctx, quad) {
                Ok(layer) => self.layers.push(layer),
                Err(e) => warn!("Native quads: cannot create a quad layer: {e}"),
            }
        }

        info!(
            "Native quads: {} quad layer(s): {:?}",
            self.layers.len(),
            self.layers
                .iter()
                .map(|layer| (layer.quad.width, layer.quad.height, layer.size))
                .collect::<Vec<_>>()
        );
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
    ) -> Vec<xr::CompositionLayerQuad<'a, xr::OpenGlEs>> {
        self.layers
            .iter()
            .map(|layer| {
                xr::CompositionLayerQuad::new()
                    .layer_flags(
                        xr::CompositionLayerFlags::BLEND_TEXTURE_SOURCE_ALPHA
                            | xr::CompositionLayerFlags::UNPREMULTIPLIED_ALPHA,
                    )
                    .space(stage_space)
                    .eye_visibility(xr::EyeVisibility::BOTH)
                    .sub_image(
                        xr::SwapchainSubImage::new()
                            .swapchain(&layer.swapchain)
                            .image_array_index(0)
                            .image_rect(xr::Rect2Di {
                                offset: xr::Offset2Di { x: 0, y: 0 },
                                extent: xr::Extent2Di {
                                    width: layer.size.x as _,
                                    height: layer.size.y as _,
                                },
                            }),
                    )
                    .pose(crate::to_xr_pose(layer.quad.pose))
                    .size(xr::Extent2Df {
                        width: layer.quad.width,
                        height: layer.quad.height,
                    })
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

fn create_layer(
    xr_session: &xr::Session<xr::OpenGlEs>,
    gfx_ctx: &GraphicsContext,
    quad: NativeQuad,
) -> Result<QuadLayer, String> {
    if quad.columns == 0 || quad.rows == 0 {
        return Err("empty chessboard".into());
    }
    let size = texture_size(&quad);

    gfx_ctx.make_current();

    let mut swapchain = xr_session
        .create_swapchain(&xr::SwapchainCreateInfo {
            create_flags: xr::SwapchainCreateFlags::EMPTY,
            usage_flags: xr::SwapchainUsageFlags::COLOR_ATTACHMENT
                | xr::SwapchainUsageFlags::SAMPLED,
            format: SDR_FORMAT_GL,
            sample_count: 1,
            width: size.x,
            height: size.y,
            face_count: 1,
            array_size: 1,
            mip_count: 1,
        })
        .map_err(|e| format!("create_swapchain: {e}"))?;

    let images = swapchain
        .enumerate_images()
        .map_err(|e| format!("enumerate_images: {e}"))?;

    // OpenGL swapchain images have their origin at the bottom left: upload the rows bottom-up
    let pixels = chessboard_pixels(&quad, size);
    let row_bytes = size.x as usize * BYTES_PER_PIXEL;
    let flipped = pixels
        .chunks_exact(row_bytes)
        .rev()
        .flatten()
        .copied()
        .collect::<Vec<_>>();

    // Fill every image, so that whichever one the runtime keeps showing is correct
    for _ in 0..images.len() {
        let index = swapchain
            .acquire_image()
            .map_err(|e| format!("acquire_image: {e}"))?;
        swapchain
            .wait_image(xr::Duration::INFINITE)
            .map_err(|e| format!("wait_image: {e}"))?;

        let texture = images
            .get(index as usize)
            .copied()
            .and_then(NonZeroU32::new)
            .ok_or("invalid swapchain image")?;
        let gl_context = &gfx_ctx.gl_context;
        unsafe {
            gl_context.bind_texture(gl::TEXTURE_2D, Some(gl::NativeTexture(texture)));
            gl_context.pixel_store_i32(gl::UNPACK_ALIGNMENT, 1);
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
            gl_context.bind_texture(gl::TEXTURE_2D, None);
            gl_context.flush();
        }

        swapchain
            .release_image()
            .map_err(|e| format!("release_image: {e}"))?;
    }

    Ok(QuadLayer {
        quad,
        swapchain,
        size,
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
        }
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
