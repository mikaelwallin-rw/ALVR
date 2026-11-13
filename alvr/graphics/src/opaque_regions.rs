// Opaque region classes, rasterized per view into the stencil buffer of the stream pass. The stream
// quad is then drawn once per class with an equal stencil test, with a pipeline specialized for that
// class: forced opaque pixels skip keying, forced keyed pixels output zero. This replaces a per
// pixel ray test in the stream shader: on mobile GPUs that code made the whole stream shader slow
// (register pressure), even for pixels that never ran it.
//
// Each region is drawn as 4 nested fills, lowest priority first, all regions per level, so an
// opaque area of one region always wins over the keyed strip of an overlapping one:
// - BAND: up to `edge_band` meters outside the edge (forced keyed)
// - STRIP: the edge itself, `edge_key_px` encoded stream pixels inward (forced keyed)
// - DESPILL: `edge_despill_px` encoded pixels further inward (forced opaque, key spill removed)
// - INNER: the rest (forced opaque)
// The pixel margins are measured in the encoded stream image (after foveated encoding) at each edge
// point, so they follow both the viewing angle and the local encoding resolution.

use crate::{FOVEATION_UNIFORM_VEC4_COUNT, GraphicsContext, OpaqueRegionGeometry};
use alvr_common::{
    Fov, Pose,
    glam::{Mat4, UVec2, Vec2, Vec3, Vec4},
};
use std::{mem, ops::Range, rc::Rc};
use wgpu::{
    Buffer, BufferAddress, BufferDescriptor, BufferUsages, ColorTargetState, ColorWrites,
    CompareFunction, DepthBiasState, DepthStencilState, Extent3d, FragmentState,
    PipelineCompilationOptions, PipelineLayoutDescriptor, PrimitiveState, PrimitiveTopology,
    RenderPass, RenderPipeline, RenderPipelineDescriptor, StencilFaceState, StencilOperation,
    StencilState, TextureDescriptor, TextureDimension, TextureFormat, TextureUsages, TextureView,
    VertexAttribute, VertexBufferLayout, VertexFormat, VertexState, VertexStepMode, include_wgsl,
};

pub const MAX_OPAQUE_REGIONS: usize = 32;

// Stencil values. Must match stream.wgsl
pub const REGION_CLASS_NONE: u8 = 0;
pub const REGION_CLASS_BAND: u8 = 1;
pub const REGION_CLASS_STRIP: u8 = 2;
pub const REGION_CLASS_DESPILL: u8 = 3;
pub const REGION_CLASS_INNER: u8 = 4;
pub const REGION_CLASSES: [u8; 4] = [
    REGION_CLASS_BAND,
    REGION_CLASS_STRIP,
    REGION_CLASS_DESPILL,
    REGION_CLASS_INNER,
];

pub const STENCIL_FORMAT: TextureFormat = TextureFormat::Stencil8;
const MIN_CURVATURE: f32 = 1e-3;
// Columns per fill: curved regions follow the arc, flat ones only interpolate the edge margins
const CURVED_COLUMNS: usize = 24;
const FLAT_COLUMNS: usize = 4;
const VERTICES_PER_COLUMN: usize = 6;
const MAX_VERTICES_PER_VIEW: usize =
    MAX_OPAQUE_REGIONS * REGION_CLASSES.len() * CURVED_COLUMNS * VERTICES_PER_COLUMN;
const MIN_PROJECTION_DEPTH: f32 = 1e-3;
// Step along the surface used to measure the projected size of a pixel at an edge
const MARGIN_PROBE_METERS: f32 = 1e-3;
// An edge seen almost edge-on would need a huge margin: cap it to this fraction of the half size
const MAX_MARGIN_FRACTION: f32 = 0.25;
const TRANSFORM_SIZE: u32 = mem::size_of::<Mat4>() as u32;

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClassVertex {
    // Homogeneous stream quad position: (u - 0.5, 0.5 - v, 0, 1) * eye depth
    pub position: [f32; 4],
    pub class: f32,
}
const VERTEX_SIZE: u64 = mem::size_of::<ClassVertex>() as u64;

pub struct RegionMeshParams {
    /// Meters outside the edge
    pub edge_band: f32,
    /// Encoded stream pixels inside the edge
    pub edge_key_px: f32,
    /// Encoded stream pixels further inside the keyed strip
    pub edge_despill_px: f32,
}

// CPU copy of the foveated encoding mapping in stream.wgsl (view uv -> encoded texture uv)
#[derive(Clone, Copy, Debug)]
struct Foveation {
    view_ratio: Vec2,
    edge_ratio: Vec2,
    c1: Vec2,
    c2: Vec2,
    lo_bound: Vec2,
    hi_bound: Vec2,
    a_left: Vec2,
    b_left: Vec2,
    a_right: Vec2,
    b_right: Vec2,
    c_right: Vec2,
}

/// Maps view uv to pixel coordinates in the encoded stream image.
#[derive(Clone, Copy, Debug)]
pub struct EncodedPixelMap {
    // Per view (the foveation centers can differ per eye)
    foveation: Option<[Foveation; 2]>,
    resolution: Vec2,
}

impl EncodedPixelMap {
    pub fn identity(resolution: UVec2) -> Self {
        Self {
            foveation: None,
            resolution: resolution.as_vec2(),
        }
    }

    /// From the foveation uniforms of stream.wgsl (see `foveated_encoding_data`) and the encoded
    /// (staging) resolution.
    pub fn from_foveation_uniforms(
        uniforms: &[Vec4; FOVEATION_UNIFORM_VEC4_COUNT],
        encoded_resolution: UVec2,
    ) -> Self {
        let common = uniforms[0];
        let eye = |view_idx: usize| {
            let [center, bounds, left, right, right_curve] =
                [0, 1, 2, 3, 4].map(|i| uniforms[1 + view_idx * 5 + i]);
            Foveation {
                view_ratio: common.truncate().truncate(),
                edge_ratio: Vec2::new(common.z, common.w),
                c1: Vec2::new(center.x, center.y),
                c2: Vec2::new(center.z, center.w),
                lo_bound: Vec2::new(bounds.x, bounds.y),
                hi_bound: Vec2::new(bounds.z, bounds.w),
                a_left: Vec2::new(left.x, left.y),
                b_left: Vec2::new(left.z, left.w),
                a_right: Vec2::new(right.x, right.y),
                b_right: Vec2::new(right.z, right.w),
                c_right: Vec2::new(right_curve.x, right_curve.y),
            }
        };
        Self {
            foveation: Some([eye(0), eye(1)]),
            resolution: encoded_resolution.as_vec2(),
        }
    }

    pub fn pixel(&self, uv: Vec2, view_idx: usize) -> Vec2 {
        let Some(foveation) = self.foveation else {
            return uv * self.resolution;
        };
        let f = foveation[view_idx];

        let mut uv = uv;
        if view_idx == 1 {
            uv.x = 1.0 - uv.x;
        }
        let center = (uv - f.c1) * f.edge_ratio / f.c2;
        let left = (-f.b_left
            + (f.b_left * f.b_left + 4.0 * f.a_left * uv)
                .max(Vec2::ZERO)
                .sqrt())
            / (2.0 * f.a_left);
        let right = (-f.b_right
            + (f.b_right * f.b_right - 4.0 * (f.c_right - f.a_right * uv))
                .max(Vec2::ZERO)
                .sqrt())
            / (2.0 * f.a_right);
        let select = |i: usize| {
            if uv[i] < f.lo_bound[i] {
                left[i]
            } else if uv[i] > f.hi_bound[i] {
                right[i]
            } else {
                center[i]
            }
        };
        let mut corrected = Vec2::new(select(0), select(1)) * f.view_ratio;
        if view_idx == 1 {
            corrected.x = 1.0 - corrected.x;
        }

        corrected * self.resolution
    }
}

// One region seen from one view
struct RegionProjection<'a> {
    eye_from_local: Pose,
    radius: f32, // 0 = flat
    half_size: Vec2,
    tan_min: Vec2,  // (tan left, tan up)
    tan_size: Vec2, // (tan right - tan left, tan down - tan up)
    pixel_map: &'a EncodedPixelMap,
    view_idx: usize,
}

impl RegionProjection<'_> {
    // (arc length, height) on the region surface to local space
    fn local(&self, s: f32, y: f32) -> Vec3 {
        if self.radius > 0.0 {
            let (sin, cos) = (s / self.radius).sin_cos();
            Vec3::new(self.radius * sin, y, self.radius - self.radius * cos)
        } else {
            Vec3::new(s, y, 0.0)
        }
    }

    fn eye(&self, s: f32, y: f32) -> Vec3 {
        self.eye_from_local.orientation * self.local(s, y) + self.eye_from_local.position
    }

    // Encoded stream image pixel coordinates, None if behind the eye
    fn pixel(&self, s: f32, y: f32) -> Option<Vec2> {
        let eye = self.eye(s, y);
        (eye.z < -MIN_PROJECTION_DEPTH).then(|| {
            let tan = eye.truncate() / -eye.z;
            self.pixel_map
                .pixel((tan - self.tan_min) / self.tan_size, self.view_idx)
        })
    }

    fn homogeneous(&self, s: f32, y: f32) -> [f32; 4] {
        let eye = self.eye(s, y);
        let w = -eye.z;
        let uv_w = (eye.truncate() - self.tan_min * w) / self.tan_size;
        [uv_w.x - 0.5 * w, 0.5 * w - uv_w.y, 0.0, w]
    }

    // Surface length of one encoded pixel at (s, y), measured along `inward` (unit, in (s, y))
    fn meters_per_pixel(&self, s: f32, y: f32, inward: Vec2) -> Option<f32> {
        let step = inward * MARGIN_PROBE_METERS;
        let distance = (self.pixel(s + step.x, y + step.y)? - self.pixel(s, y)?).length();
        (distance > 1e-6).then(|| MARGIN_PROBE_METERS / distance)
    }

    // Outward offset of the boundary of `class` at an edge point. `limit` caps the inward offsets.
    fn edge_offset(
        &self,
        class: u8,
        s: f32,
        y: f32,
        inward: Vec2,
        limit: f32,
        params: &RegionMeshParams,
    ) -> f32 {
        let inward_px = match class {
            REGION_CLASS_BAND => return params.edge_band.max(0.0),
            REGION_CLASS_STRIP => return 0.0,
            REGION_CLASS_DESPILL => params.edge_key_px.max(0.0),
            _ => params.edge_key_px.max(0.0) + params.edge_despill_px.max(0.0),
        };
        let meters = self.meters_per_pixel(s, y, inward).unwrap_or(0.0) * inward_px;
        -meters.min(limit)
    }
}

/// Appends the class fills of all regions as seen from one view (see the module comment), in
/// increasing class order. Returns the vertex range of each class of `REGION_CLASSES`.
pub fn build_class_mesh(
    regions: &[OpaqueRegionGeometry],
    eye_pose: Pose,
    fov: Fov,
    pixel_map: &EncodedPixelMap,
    view_idx: usize,
    params: &RegionMeshParams,
    out: &mut Vec<ClassVertex>,
) -> [Range<u32>; 4] {
    let tan_min = Vec2::new(fov.left.tan(), fov.up.tan());
    let tan_size = Vec2::new(fov.right.tan(), fov.down.tan()) - tan_min;
    let projections = regions
        .iter()
        .take(MAX_OPAQUE_REGIONS)
        .map(|region| RegionProjection {
            eye_from_local: eye_pose.inverse() * region.pose,
            radius: if region.curvature > MIN_CURVATURE {
                region.width / (2.0 * std::f32::consts::PI * region.curvature)
            } else {
                0.0
            },
            half_size: Vec2::new(region.width, region.height) * 0.5,
            tan_min,
            tan_size,
            pixel_map,
            view_idx,
        })
        .collect::<Vec<_>>();

    REGION_CLASSES.map(|class| {
        let start = out.len() as u32;
        for projection in &projections {
            append_fill(projection, class, params, out);
        }
        start..out.len() as u32
    })
}

fn append_fill(
    projection: &RegionProjection,
    class: u8,
    params: &RegionMeshParams,
    out: &mut Vec<ClassVertex>,
) {
    let half = projection.half_size;
    let limit = half * MAX_MARGIN_FRACTION;

    let left = -half.x - projection.edge_offset(class, -half.x, 0.0, Vec2::X, limit.x, params);
    let right = half.x + projection.edge_offset(class, half.x, 0.0, -Vec2::X, limit.x, params);
    if right <= left {
        return;
    }

    let columns = if projection.radius > 0.0 {
        CURVED_COLUMNS
    } else {
        FLAT_COLUMNS
    };
    // Bottom and top of the fill at a column; the margins are measured at the nearest edge point
    let edges_at = |s: f32| {
        let edge_s = s.clamp(-half.x, half.x);
        let bottom =
            -half.y - projection.edge_offset(class, edge_s, -half.y, Vec2::Y, limit.y, params);
        let top = half.y + projection.edge_offset(class, edge_s, half.y, -Vec2::Y, limit.y, params);
        (bottom, top)
    };

    let class_value = f32::from(class);
    let vertex = |s: f32, y: f32| ClassVertex {
        position: projection.homogeneous(s, y),
        class: class_value,
    };

    let mut previous_s = left;
    let mut previous_edges = edges_at(left);
    for column in 1..=columns {
        let s = left + (right - left) * column as f32 / columns as f32;
        let edges = edges_at(s);
        let ((b0, t0), (b1, t1)) = (previous_edges, edges);
        if t0 > b0 && t1 > b1 {
            let (v00, v10, v01, v11) = (
                vertex(previous_s, b0),
                vertex(s, b1),
                vertex(previous_s, t0),
                vertex(s, t1),
            );
            out.extend([v00, v10, v11, v00, v11, v01]);
        }
        previous_s = s;
        previous_edges = edges;
    }
}

/// Stencil state of the stream pipelines: draw only where the stencil equals the reference class.
pub fn stream_stencil_state() -> DepthStencilState {
    let face = StencilFaceState {
        compare: CompareFunction::Equal,
        fail_op: StencilOperation::Keep,
        depth_fail_op: StencilOperation::Keep,
        pass_op: StencilOperation::Keep,
    };
    DepthStencilState {
        format: STENCIL_FORMAT,
        depth_write_enabled: None,
        depth_compare: None,
        stencil: StencilState {
            front: face,
            back: face,
            read_mask: 0xff,
            write_mask: 0,
        },
        bias: DepthBiasState::default(),
    }
}

/// Owns the per view stencil buffers and class meshes, and draws the meshes into the stencil.
pub struct OpaqueRegionsRenderer {
    context: Rc<GraphicsContext>,
    pipeline: RenderPipeline,
    stencil_views: [TextureView; 2],
    vertex_buffers: [Buffer; 2],
    class_ranges: [[Range<u32>; 4]; 2],
    scratch: Vec<ClassVertex>,
}

impl OpaqueRegionsRenderer {
    pub fn new(
        context: Rc<GraphicsContext>,
        target_resolution: UVec2,
        target_format: TextureFormat,
    ) -> Self {
        let device = &context.device;

        // Explicit layout: it cannot be inferred with immediates
        let pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[],
            immediate_size: TRANSFORM_SIZE,
        });
        let shader_module =
            device.create_shader_module(include_wgsl!("../resources/opaque_regions.wgsl"));
        let replace = StencilFaceState {
            compare: CompareFunction::Always,
            fail_op: StencilOperation::Keep,
            depth_fail_op: StencilOperation::Keep,
            pass_op: StencilOperation::Replace,
        };
        let pipeline = device.create_render_pipeline(&RenderPipelineDescriptor {
            label: None,
            layout: Some(&pipeline_layout),
            vertex: VertexState {
                module: &shader_module,
                entry_point: None,
                compilation_options: PipelineCompilationOptions::default(),
                buffers: &[VertexBufferLayout {
                    array_stride: VERTEX_SIZE,
                    step_mode: VertexStepMode::Vertex,
                    attributes: &[
                        VertexAttribute {
                            format: VertexFormat::Float32x4,
                            offset: 0,
                            shader_location: 0,
                        },
                        VertexAttribute {
                            format: VertexFormat::Float32,
                            offset: mem::size_of::<[f32; 4]>() as BufferAddress,
                            shader_location: 1,
                        },
                    ],
                }],
            },
            primitive: PrimitiveState {
                topology: PrimitiveTopology::TriangleList,
                ..Default::default()
            },
            depth_stencil: Some(DepthStencilState {
                format: STENCIL_FORMAT,
                depth_write_enabled: None,
                depth_compare: None,
                stencil: StencilState {
                    front: replace,
                    back: replace,
                    read_mask: 0xff,
                    write_mask: 0xff,
                },
                bias: DepthBiasState::default(),
            }),
            multisample: Default::default(),
            // Stencil only: the colour target is not written
            fragment: Some(FragmentState {
                module: &shader_module,
                entry_point: None,
                compilation_options: PipelineCompilationOptions::default(),
                targets: &[Some(ColorTargetState {
                    format: target_format,
                    blend: None,
                    write_mask: ColorWrites::empty(),
                })],
            }),
            multiview_mask: None,
            cache: None,
        });

        let stencil_views = [(); 2].map(|_| {
            device
                .create_texture(&TextureDescriptor {
                    label: None,
                    size: Extent3d {
                        width: target_resolution.x,
                        height: target_resolution.y,
                        depth_or_array_layers: 1,
                    },
                    mip_level_count: 1,
                    sample_count: 1,
                    dimension: TextureDimension::D2,
                    format: STENCIL_FORMAT,
                    usage: TextureUsages::RENDER_ATTACHMENT,
                    view_formats: &[],
                })
                .create_view(&Default::default())
        });
        let vertex_buffers = [(); 2].map(|_| {
            device.create_buffer(&BufferDescriptor {
                label: None,
                size: MAX_VERTICES_PER_VIEW as u64 * VERTEX_SIZE,
                usage: BufferUsages::VERTEX | BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        });

        Self {
            context,
            pipeline,
            stencil_views,
            vertex_buffers,
            class_ranges: [[0..0, 0..0, 0..0, 0..0], [0..0, 0..0, 0..0, 0..0]],
            scratch: Vec::with_capacity(MAX_VERTICES_PER_VIEW),
        }
    }

    pub fn stencil_view(&self, view_idx: usize) -> &TextureView {
        &self.stencil_views[view_idx]
    }

    pub fn vertex_counts(&self) -> [u32; 2] {
        self.class_ranges.each_ref().map(|ranges| ranges[3].end)
    }

    pub fn update(
        &mut self,
        view_idx: usize,
        regions: &[OpaqueRegionGeometry],
        eye_pose: Pose,
        fov: Fov,
        pixel_map: &EncodedPixelMap,
        params: &RegionMeshParams,
    ) {
        self.scratch.clear();
        let mut ranges = build_class_mesh(
            regions,
            eye_pose,
            fov,
            pixel_map,
            view_idx,
            params,
            &mut self.scratch,
        );
        // # Safety: never more than MAX_VERTICES_PER_VIEW (MAX_OPAQUE_REGIONS fills of at most
        // CURVED_COLUMNS columns per class), clamped anyway
        let max = MAX_VERTICES_PER_VIEW as u32;
        self.scratch.truncate(MAX_VERTICES_PER_VIEW);
        for range in &mut ranges {
            *range = range.start.min(max)..range.end.min(max);
        }

        let bytes = self
            .scratch
            .iter()
            .flat_map(|vertex| vertex.position.into_iter().chain([vertex.class]))
            .flat_map(f32::to_le_bytes)
            .collect::<Vec<u8>>();
        if !bytes.is_empty() {
            self.context
                .queue
                .write_buffer(&self.vertex_buffers[view_idx], 0, &bytes);
        }
        self.class_ranges[view_idx] = ranges;
    }

    /// Draws the view's classes into the stencil of `render_pass` (which must clear it to
    /// REGION_CLASS_NONE), with the stream pass transform.
    pub fn draw_stencil(
        &self,
        render_pass: &mut RenderPass,
        view_idx: usize,
        transform_bytes: &[u8],
    ) {
        let ranges = &self.class_ranges[view_idx];
        if ranges[3].end == 0 {
            return;
        }
        render_pass.set_pipeline(&self.pipeline);
        render_pass.set_immediates(0, transform_bytes);
        render_pass.set_vertex_buffer(
            0,
            self.vertex_buffers[view_idx].slice(..u64::from(ranges[3].end) * VERTEX_SIZE),
        );
        for (class, range) in REGION_CLASSES.iter().zip(ranges) {
            if !range.is_empty() {
                render_pass.set_stencil_reference(u32::from(*class));
                render_pass.draw(range.clone(), 0..1);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alvr_common::glam::Quat;

    const FOV_45: Fov = Fov {
        left: -std::f32::consts::FRAC_PI_4,
        right: std::f32::consts::FRAC_PI_4,
        up: std::f32::consts::FRAC_PI_4,
        down: -std::f32::consts::FRAC_PI_4,
    };
    // tan range 2 over 1000 px: one pixel is 2 mm at 1 m
    const RESOLUTION: UVec2 = UVec2::new(1000, 1000);
    const PARAMS: RegionMeshParams = RegionMeshParams {
        edge_band: 0.015,
        edge_key_px: 2.0,
        edge_despill_px: 2.0,
    };

    fn uv(vertex: &ClassVertex) -> Vec2 {
        let [x, y, _, w] = vertex.position;
        Vec2::new(x / w + 0.5, 0.5 - y / w)
    }

    fn class_uv_range(mesh: &[ClassVertex], class: u8) -> (Vec2, Vec2) {
        mesh.iter()
            .filter(|vertex| vertex.class == f32::from(class))
            .fold((Vec2::MAX, Vec2::MIN), |(min, max), vertex| {
                (min.min(uv(vertex)), max.max(uv(vertex)))
            })
    }

    #[test]
    fn flat_facing_region_margins_in_pixels() {
        let region = OpaqueRegionGeometry {
            pose: Pose {
                orientation: Quat::IDENTITY,
                position: Vec3::new(0.0, 0.0, -1.0),
            },
            width: 1.0,
            height: 0.5,
            curvature: 0.0,
            inset: 0.0,
        };
        let mut mesh = vec![];
        let ranges = build_class_mesh(
            &[region],
            Pose::IDENTITY,
            FOV_45,
            &EncodedPixelMap::identity(RESOLUTION),
            0,
            &PARAMS,
            &mut mesh,
        );
        assert_eq!(mesh.len(), 4 * FLAT_COLUMNS * VERTICES_PER_COLUMN);
        for (class, range) in REGION_CLASSES.iter().zip(&ranges) {
            assert!(
                mesh[range.start as usize..range.end as usize]
                    .iter()
                    .all(|vertex| vertex.class == f32::from(*class))
            );
        }

        // u = (x + 1) / 2 at depth 1; v = (1 - y) / 2
        let check = |class, half_x: f32, half_y: f32| {
            let (min, max) = class_uv_range(&mesh, class);
            let expected_min = Vec2::new((1.0 - half_x) / 2.0, (1.0 - half_y) / 2.0);
            let expected_max = Vec2::new((1.0 + half_x) / 2.0, (1.0 + half_y) / 2.0);
            assert!(
                (min - expected_min).abs().max_element() < 1e-4,
                "{class}: {min} vs {expected_min}"
            );
            assert!(
                (max - expected_max).abs().max_element() < 1e-4,
                "{class}: {max} vs {expected_max}"
            );
        };
        check(REGION_CLASS_BAND, 0.515, 0.265);
        check(REGION_CLASS_STRIP, 0.5, 0.25);
        check(REGION_CLASS_DESPILL, 0.5 - 0.004, 0.25 - 0.004);
        check(REGION_CLASS_INNER, 0.5 - 0.008, 0.25 - 0.008);
    }

    #[test]
    fn grazing_edge_gets_wider_margin_in_meters() {
        // Region tilted 55 deg around X: its top and bottom edges are foreshortened
        let region = OpaqueRegionGeometry {
            pose: Pose {
                orientation: Quat::from_rotation_x(-55_f32.to_radians()),
                position: Vec3::new(0.0, 0.3, -1.0),
            },
            width: 0.4,
            height: 0.7,
            curvature: 0.0494,
            inset: 0.0,
        };
        let pixel_map = EncodedPixelMap::identity(RESOLUTION);
        let projection = RegionProjection {
            eye_from_local: region.pose,
            radius: region.width / (2.0 * std::f32::consts::PI * region.curvature),
            half_size: Vec2::new(0.2, 0.35),
            tan_min: Vec2::new(-1.0, 1.0),
            tan_size: Vec2::new(2.0, -2.0),
            pixel_map: &pixel_map,
            view_idx: 0,
        };
        let side = projection.meters_per_pixel(0.2, 0.0, -Vec2::X).unwrap();
        let top = projection.meters_per_pixel(0.0, 0.35, -Vec2::Y).unwrap();
        assert!(top > 2.0 * side, "top {top} side {side}");

        // The inner fill still projects 4 px inside the strip on the top edge
        let mut mesh = vec![];
        build_class_mesh(
            &[region],
            Pose::IDENTITY,
            FOV_45,
            &pixel_map,
            0,
            &PARAMS,
            &mut mesh,
        );
        assert!(mesh.iter().all(|vertex| vertex.position[3] > 0.0));
        let strip_top = class_uv_range(&mesh, REGION_CLASS_STRIP).0.y;
        let inner_top = class_uv_range(&mesh, REGION_CLASS_INNER).0.y;
        let gap_px = (inner_top - strip_top) * RESOLUTION.y as f32;
        assert!((gap_px - 4.0).abs() < 0.5, "gap {gap_px} px");
    }

    #[test]
    fn foveated_periphery_gets_wider_margin() {
        // Parameters as the server negotiates them for a 2000x2000 view: center size 0.45 x 0.4,
        // shift 0.4 x 0.1, edge ratio 4 x 5 (alignment as in the server's foveation setup)
        let view = Vec2::splat(2000.0);
        let edge_ratio = Vec2::new(4.0, 5.0);
        let edge_size = view - Vec2::new(0.45, 0.4) * view;
        let center_size = 1.0 - (edge_size / (edge_ratio * 2.0)).ceil() * (edge_ratio * 2.0) / view;
        let optimized = (center_size + (1.0 - center_size) / edge_ratio) * view;
        let aligned = (optimized / 32.0).ceil() * 32.0;
        let params = alvr_common::AlvrFoveatedEncodingParams {
            encoded_view_resolution: aligned.as_uvec2().to_array(),
            view_ratio: (optimized / aligned).to_array(),
            center_size: center_size.to_array(),
            center_shifts: [[0.4, 0.1]; 2],
            edge_ratio: edge_ratio.to_array(),
        };
        let encoded_resolution = aligned.as_uvec2();
        let uniforms =
            crate::foveated_encoding_data(&params, params.center_shifts.map(Vec2::from_array));
        let pixel_map = EncodedPixelMap::from_foveation_uniforms(&uniforms, encoded_resolution);

        // The mapping is monotonic and covers the encoded image
        let start = pixel_map.pixel(Vec2::ZERO, 0);
        let end = pixel_map.pixel(Vec2::ONE, 0);
        assert!(start.abs().max_element() < 2.0, "{start}");
        assert!(
            (end - encoded_resolution.as_vec2()).abs().max_element() < 40.0,
            "{end} vs {encoded_resolution}"
        );

        // One encoded pixel spans more view pixels in the periphery than in the centre
        let span = |uv: Vec2| {
            let step = Vec2::new(0.0, 1e-3);
            (pixel_map.pixel(uv + step, 0) - pixel_map.pixel(uv, 0)).length()
        };
        let centre = span(Vec2::new(0.5, 0.5));
        let top = span(Vec2::new(0.5, 0.05));
        assert!(centre > 2.0 * top, "centre {centre} top {top}");
    }
}
