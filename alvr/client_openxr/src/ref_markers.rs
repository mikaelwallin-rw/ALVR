// DEBUG SWITCH screenvr_refmarkers.txt: reference markers for the displayed-view calibration test.
// Each line: `<png file name> <size in m> <12 numbers: STAGE pose as a row-major 3x4 matrix>`; the
// PNG (e.g. an ArUco marker with its white border, generated on the PC) is read from the app files
// dir and drawn as a world-locked opaque quad of that size (square), above everything except the
// hands. Detecting these in a screencap fits the pixel -> view ray mapping of each eye without any
// camera model. Removing the file removes the markers.

use crate::{graphics, loading_screen::QuadSwapchain};
use alvr_common::{
    Pose,
    glam::{Mat3, Quat, UVec2, Vec3},
    info, warn,
};
use alvr_graphics::GraphicsContext;
use openxr as xr;
use std::{fs, rc::Rc};

const FILES_DIR: &str = "/sdcard/Android/data/alvr.client.dev/files";

struct Marker {
    // Rectangle of the marker's image in the atlas
    offset: UVec2,
    size: UVec2,
    pose: Pose,
    size_m: f32,
}

pub struct RefMarkers {
    gfx_ctx: Rc<GraphicsContext>,
    text: String,
    // One swapchain for all markers (the runtime allows only a few swapchains)
    atlas: Option<QuadSwapchain>,
    markers: Vec<Marker>,
}

fn parse_line(line: &str) -> Option<(String, f32, Pose)> {
    let mut parts = line.split_whitespace();
    let name = parts.next()?.to_owned();
    let values = parts
        .map(str::parse::<f32>)
        .collect::<Result<Vec<_>, _>>()
        .ok()?;
    let [size, m @ ..] = values.as_slice() else {
        return None;
    };
    if m.len() != 12 || *size <= 0.0 {
        return None;
    }
    let rotation = Mat3::from_cols(
        Vec3::new(m[0], m[4], m[8]),
        Vec3::new(m[1], m[5], m[9]),
        Vec3::new(m[2], m[6], m[10]),
    );
    Some((
        name,
        *size,
        Pose {
            orientation: Quat::from_mat3(&rotation).normalize(),
            position: Vec3::new(m[3], m[7], m[11]),
        },
    ))
}

impl RefMarkers {
    pub fn new(gfx_ctx: &Rc<GraphicsContext>) -> Self {
        Self {
            gfx_ctx: Rc::clone(gfx_ctx),
            text: String::new(),
            atlas: None,
            markers: Vec::new(),
        }
    }

    /// Once per frame: rebuilds the markers when the switch file changed
    pub fn update(&mut self, xr_session: &xr::Session<xr::OpenGlEs>) {
        let text = crate::debug_switches::get().ref_markers;
        if text == self.text {
            return;
        }
        self.text = text;
        self.markers.clear();
        self.atlas = None;

        let mut images = vec![];
        for line in self.text.lines().filter(|line| !line.trim().is_empty()) {
            let Some((name, size_m, pose)) = parse_line(line) else {
                warn!("Reference markers: cannot parse {line:?}");
                continue;
            };
            let Some((size, pixels)) = fs::read(format!("{FILES_DIR}/{name}"))
                .ok()
                .and_then(|bytes| crate::loading_screen::decode_png(&bytes))
            else {
                warn!("Reference markers: cannot read {name}");
                continue;
            };
            images.push((size, pixels, pose, size_m));
        }
        if images.is_empty() {
            return;
        }

        // Atlas: a grid of cells as large as the largest image
        let cell = images
            .iter()
            .fold(UVec2::ZERO, |cell, (size, ..)| cell.max(*size));
        let columns = (images.len() as f32).sqrt().ceil() as u32;
        let rows = (images.len() as u32).div_ceil(columns);
        let atlas_size = UVec2::new(cell.x * columns, cell.y * rows);
        let mut atlas = vec![255_u8; (atlas_size.x * atlas_size.y * 4) as usize];
        let (format, srgb) = graphics::srgb_swapchain_format(xr_session);
        for (index, (size, pixels, pose, size_m)) in images.into_iter().enumerate() {
            let offset = UVec2::new(
                (index as u32 % columns) * cell.x,
                (index as u32 / columns) * cell.y,
            );
            for y in 0..size.y {
                let source = (y * size.x * 4) as usize;
                let target = (((offset.y + y) * atlas_size.x + offset.x) * 4) as usize;
                atlas[target..target + (size.x * 4) as usize]
                    .copy_from_slice(&pixels[source..source + (size.x * 4) as usize]);
            }
            self.markers.push(Marker {
                offset,
                size,
                pose,
                size_m,
            });
        }
        if !srgb {
            for pixel in atlas.chunks_exact_mut(4) {
                for channel in &mut pixel[..3] {
                    *channel = crate::loading_screen::srgb_to_linear_u8(*channel);
                }
            }
        }

        self.gfx_ctx.make_current();
        if let Some(mut swapchain) = QuadSwapchain::new(xr_session, atlas_size, format) {
            swapchain.upload(&self.gfx_ctx.gl_context, &atlas);
            self.atlas = Some(swapchain);
        } else {
            self.markers.clear();
        }
        info!(
            "Reference markers: {} drawn ({}x{} atlas): {:?}",
            self.markers.len(),
            atlas_size.x,
            atlas_size.y,
            self.markers
                .iter()
                .map(|marker| (marker.pose.position, marker.size_m))
                .collect::<Vec<_>>()
        );
    }

    pub fn build<'a>(
        &'a self,
        stage_space: &'a xr::Space,
    ) -> Vec<xr::CompositionLayerQuad<'a, xr::OpenGlEs>> {
        let Some(atlas) = self.atlas.as_ref().filter(|atlas| atlas.filled) else {
            return Vec::new();
        };
        self.markers
            .iter()
            .map(|marker| {
                atlas.layer_rect(
                    stage_space,
                    marker.pose,
                    marker.size_m,
                    marker.offset,
                    marker.size,
                )
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_a_row_major_pose() {
        let (name, size, pose) =
            parse_line("m900.png 0.06  0 -1 0 0.5  1 0 0 1.2  0 0 1 -0.8").unwrap();
        assert_eq!((name.as_str(), size), ("m900.png", 0.06));
        assert!(pose.position.distance(Vec3::new(0.5, 1.2, -0.8)) < 1e-6);
        // Rotation +90 deg about Z: local X -> STAGE Y
        assert!((pose.orientation * Vec3::X).distance(Vec3::Y) < 1e-6);
    }
}
