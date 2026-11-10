// Hand-tracked hands drawn natively by the client, as a skinned mesh (Pico's SM_Hand, converted by
// resources/pico_hand_to_skinned_mesh.py) driven by the XR_EXT_hand_tracking joints. The output is
// meant for its own alpha-blended projection layer above the stream, so the hands never inherit
// the stream latency.
//
// Skinning runs on the CPU (about 1600 vertices per hand): GPU skinning through a uniform array
// of joint matrices indexed by an integer vertex attribute dropped every finger vertex on the PICO
// 4 Ultra (only the wrist, palm base and thumb were drawn).
//
// Shape (HandShape): girth = each vertex's offset from its joints' bone axes (joint-local X and Y;
// -Z runs along the bone) scaled per joint, so fingertip and joint POSITIONS stay exactly where
// tracking puts them; scale = uniform about the wrist (moves the fingertips, off by default).
// The mesh's own girth per joint (median distance of the vertices it dominates from the bone
// axis) is measured at load, so the tracked XR_EXT_hand_tracking joint radii can drive it.

use super::{GraphicsContext, MAX_PUSH_CONSTANTS_SIZE};
use alvr_common::{
    Pose, ViewParams,
    glam::{Mat4, UVec2, Vec3, Vec4},
};
use std::{mem, rc::Rc};
use wgpu::{
    BlendState, Buffer, BufferDescriptor, BufferUsages, Color, ColorTargetState, ColorWrites,
    CommandEncoderDescriptor, CompareFunction, DepthStencilState, Device, Extent3d, FragmentState,
    IndexFormat, LoadOp, MultisampleState, Operations, PipelineLayoutDescriptor, PrimitiveState,
    PrimitiveTopology, PushConstantRange, RenderPassColorAttachment,
    RenderPassDepthStencilAttachment, RenderPassDescriptor, RenderPipeline,
    RenderPipelineDescriptor, ShaderStages, StoreOp, TextureDescriptor, TextureDimension,
    TextureFormat, TextureUsages, TextureView, VertexBufferLayout, VertexState, VertexStepMode,
    include_wgsl, vertex_attr_array,
};

pub const HAND_JOINT_COUNT: usize = 26;
// XR_EXT_hand_tracking tip joints: thumb, index, middle, ring, little
pub const HAND_TIP_JOINTS: [usize; 5] = [5, 10, 15, 20, 25];

const MESH_MAGIC: &[u8; 4] = b"AHND";
const MESH_VERSION: u32 = 1;
const MESH_HEADER_SIZE: usize = 16;
const MATRIX_SIZE: usize = mem::size_of::<Mat4>();
// File vertex: position f32x3, normal f32x3, joint indices u8x4, weights unorm8x4
const FILE_VERTEX_SIZE: usize = 32;
const INDEX_SIZE: usize = mem::size_of::<u16>();
const INFLUENCES: usize = 4;
// GPU vertex: skinned position f32x3, normal f32x3
const GPU_VERTEX_SIZE: usize = 24;

const LEFT_HAND_MESH: &[u8] = include_bytes!("../resources/hands/pico_hand_left.bin");
const RIGHT_HAND_MESH: &[u8] = include_bytes!("../resources/hands/pico_hand_right.bin");

// Skinned vertex buffers in flight
const VERTEX_BUFFER_RING: usize = 3;
// Supported MSAA sample counts (1 = render straight into the swapchain image, no resolve)
pub const HANDS_SAMPLE_COUNTS: [u32; 3] = [1, 2, 4];
const DEPTH_FORMAT: TextureFormat = TextureFormat::Depth24Plus;

const VIEW_PROJ_CONST_SIZE: u32 = mem::size_of::<Mat4>() as u32;
const EYE_POSITION_CONST_SIZE: u32 = mem::size_of::<Vec4>() as u32;
const PUSH_CONSTANTS_SIZE: u32 = VIEW_PROJ_CONST_SIZE + EYE_POSITION_CONST_SIZE;
const _: () = assert!(
    PUSH_CONSTANTS_SIZE <= MAX_PUSH_CONSTANTS_SIZE,
    "Push constants size exceeds the maximum size"
);

pub struct HandsViewParams {
    pub swapchain_index: u32,
    pub view_params: ViewParams,
}

/// Skinning sanity check of one hand pose
pub struct HandSkinningCheck {
    /// Largest distance between a tracked fingertip joint and the skinned mesh vertex that is
    /// nearest to that fingertip in the bind pose, in meters
    pub max_tip_error: f32,
    pub non_finite_vertices: usize,
}

struct MeshVertex {
    position: Vec3,
    normal: Vec3,
    joints: [usize; INFLUENCES],
    weights: [f32; INFLUENCES],
}

/// Hand shape adjustments, per hand
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HandShape {
    /// Girth factor per joint (1 = the mesh as modelled), applied perpendicular to the bones
    pub girth: [f32; HAND_JOINT_COUNT],
    /// Length factor per joint along its bone (joint-local Z): match it to how far the joints were
    /// moved apart (finger lengthening, see lengthen_fingers()) so the segments stretch with them
    pub length: [f32; HAND_JOINT_COUNT],
    /// Uniform scale about the wrist (1 = the tracked joint positions)
    pub scale: f32,
}

impl Default for HandShape {
    fn default() -> Self {
        Self {
            girth: [1.0; HAND_JOINT_COUNT],
            length: [1.0; HAND_JOINT_COUNT],
            scale: 1.0,
        }
    }
}

/// XR_EXT_hand_tracking finger chains, each from its knuckle (the base, which stays) to the tip:
/// thumb, index, middle, ring, little
pub const HAND_FINGER_CHAINS: [&[usize]; 5] = [
    &[3, 4, 5],
    &[7, 8, 9, 10],
    &[12, 13, 14, 15],
    &[17, 18, 19, 20],
    &[22, 23, 24, 25],
];

/// Lengthens every finger bone by `factor` (palm, wrist, metacarpals and knuckles stay): each joint
/// after a finger's knuckle moves outward along its finger, orientations unchanged
pub fn lengthen_fingers(
    joints: &[Pose; HAND_JOINT_COUNT],
    factor: f32,
) -> [Pose; HAND_JOINT_COUNT] {
    let mut out = *joints;
    for chain in HAND_FINGER_CHAINS {
        let base = joints[chain[0]].position;
        for &joint in &chain[1..] {
            out[joint].position = base + (joints[joint].position - base) * factor;
        }
    }
    out
}

/// HandShape::length for lengthen_fingers(factor): the finger joints (knuckle to tip) stretch
pub fn finger_length_factors(factor: f32) -> [f32; HAND_JOINT_COUNT] {
    let mut length = [1.0; HAND_JOINT_COUNT];
    for chain in HAND_FINGER_CHAINS {
        for &joint in chain {
            length[joint] = factor;
        }
    }
    length
}

struct HandMesh {
    joint_from_mesh: [Mat4; HAND_JOINT_COUNT],
    // Median perpendicular distance from the bone axis of the vertices each joint dominates
    // (meters; 0 when a joint dominates too few vertices, e.g. the tips)
    joint_radii: [f32; HAND_JOINT_COUNT],
    vertices: Vec<MeshVertex>,
    // Per tip joint (HAND_TIP_JOINTS), the vertex nearest to it in the bind pose
    tip_vertices: [usize; 5],
    // Ring of skinned vertex buffers: a frame never overwrites the buffer an earlier frame's draw
    // may still read (the GL driver would stall on that implicit sync)
    vertex_buffers: Vec<Buffer>,
    index_buffer: Buffer,
    index_count: u32,
}

pub struct HandsRenderer {
    context: Rc<GraphicsContext>,
    pipeline: RenderPipeline,
    meshes: [HandMesh; 2],
    // Multisampled color target, resolved into the swapchain image; None without MSAA
    color_view: Option<TextureView>,
    depth_view: TextureView,
    render_targets: [Vec<TextureView>; 2],
    frame_index: std::cell::Cell<usize>,
}

fn read_u32(bytes: &[u8], offset: usize) -> Result<u32, String> {
    bytes
        .get(offset..offset + 4)
        .and_then(|b| b.try_into().ok())
        .map(u32::from_le_bytes)
        .ok_or_else(|| "hand mesh truncated".into())
}

fn read_f32(bytes: &[u8], offset: usize) -> Result<f32, String> {
    read_u32(bytes, offset).map(f32::from_bits)
}

fn read_vec3(bytes: &[u8], offset: usize) -> Result<Vec3, String> {
    Ok(Vec3::new(
        read_f32(bytes, offset)?,
        read_f32(bytes, offset + 4)?,
        read_f32(bytes, offset + 8)?,
    ))
}

fn read_matrix(bytes: &[u8], offset: usize) -> Result<Mat4, String> {
    let mut values = [0.0_f32; 16];
    for (idx, value) in values.iter_mut().enumerate() {
        *value = read_f32(bytes, offset + idx * 4)?;
    }

    Ok(Mat4::from_cols_array(&values))
}

fn padded(bytes: &[u8]) -> Vec<u8> {
    let mut data = bytes.to_vec();
    data.resize(bytes.len().next_multiple_of(4), 0);

    data
}

fn create_target(
    device: &Device,
    resolution: UVec2,
    format: TextureFormat,
    sample_count: u32,
    label: &str,
) -> TextureView {
    device
        .create_texture(&TextureDescriptor {
            label: Some(label),
            size: Extent3d {
                width: resolution.x,
                height: resolution.y,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count,
            dimension: TextureDimension::D2,
            format,
            usage: TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        })
        .create_view(&Default::default())
}

fn load_mesh(context: &GraphicsContext, label: &str, bytes: &[u8]) -> Result<HandMesh, String> {
    if bytes.get(0..4) != Some(MESH_MAGIC.as_slice()) || read_u32(bytes, 4)? != MESH_VERSION {
        return Err(format!("{label} hand mesh: unknown format"));
    }
    let vertex_count = read_u32(bytes, 8)? as usize;
    let index_count = read_u32(bytes, 12)? as usize;

    let mut joint_from_mesh = [Mat4::IDENTITY; HAND_JOINT_COUNT];
    for (idx, matrix) in joint_from_mesh.iter_mut().enumerate() {
        *matrix = read_matrix(bytes, MESH_HEADER_SIZE + idx * MATRIX_SIZE)?;
    }

    let vertices_start = MESH_HEADER_SIZE + HAND_JOINT_COUNT * MATRIX_SIZE;
    let mut vertices = Vec::with_capacity(vertex_count);
    for idx in 0..vertex_count {
        let offset = vertices_start + idx * FILE_VERTEX_SIZE;
        let influences = bytes
            .get(offset + 24..offset + FILE_VERTEX_SIZE)
            .ok_or_else(|| format!("{label} hand mesh truncated"))?;
        let mut joints = [0; INFLUENCES];
        let mut weights = [0.0; INFLUENCES];
        for slot in 0..INFLUENCES {
            joints[slot] = usize::from(influences[slot]);
            weights[slot] = f32::from(influences[INFLUENCES + slot]) / f32::from(u8::MAX);
            if joints[slot] >= HAND_JOINT_COUNT {
                return Err(format!("{label} hand mesh: invalid joint {}", joints[slot]));
            }
        }

        vertices.push(MeshVertex {
            position: read_vec3(bytes, offset)?,
            normal: read_vec3(bytes, offset + 12)?,
            joints,
            weights,
        });
    }

    let indices_start = vertices_start + vertex_count * FILE_VERTEX_SIZE;
    let index_bytes = bytes
        .get(indices_start..indices_start + index_count * INDEX_SIZE)
        .ok_or_else(|| format!("{label} hand mesh truncated"))?;

    let joint_radii = std::array::from_fn(|joint| {
        let mut distances = vertices
            .iter()
            .filter(|vertex| {
                let dominant = (0..INFLUENCES)
                    .max_by(|&a, &b| vertex.weights[a].total_cmp(&vertex.weights[b]))
                    .unwrap_or(0);
                vertex.joints[dominant] == joint
            })
            .map(|vertex| {
                let local = joint_from_mesh[joint].transform_point3(vertex.position);
                local.x.hypot(local.y)
            })
            .collect::<Vec<_>>();
        if distances.len() < 5 {
            return 0.0;
        }
        distances.sort_by(f32::total_cmp);
        distances[distances.len() / 2]
    });

    let rest_positions = joint_from_mesh.map(|matrix| matrix.inverse().w_axis.truncate());
    let tip_vertices = HAND_TIP_JOINTS.map(|joint| {
        let target = rest_positions[joint];
        vertices
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                a.position
                    .distance_squared(target)
                    .total_cmp(&b.position.distance_squared(target))
            })
            .map(|(idx, _)| idx)
            .unwrap_or_default()
    });

    let device = &context.device;
    let vertex_buffers = (0..VERTEX_BUFFER_RING)
        .map(|_| {
            device.create_buffer(&BufferDescriptor {
                label: Some("hand_vertices"),
                size: (vertex_count * GPU_VERTEX_SIZE) as u64,
                usage: BufferUsages::VERTEX | BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        })
        .collect();
    let index_data = padded(index_bytes);
    let index_buffer = device.create_buffer(&BufferDescriptor {
        label: Some("hand_indices"),
        size: index_data.len() as u64,
        usage: BufferUsages::INDEX | BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    context.queue.write_buffer(&index_buffer, 0, &index_data);

    Ok(HandMesh {
        joint_from_mesh,
        joint_radii,
        vertices,
        tip_vertices,
        vertex_buffers,
        index_buffer,
        index_count: index_count as u32,
    })
}

// Per joint: stage_from_joint * girth (joint-local X and Y) * joint_from_mesh, and its inverse
// transpose for the normals (the girth scale is non-uniform)
struct Skin {
    points: [Mat4; HAND_JOINT_COUNT],
    normals: [Mat4; HAND_JOINT_COUNT],
}

fn skin_matrices(
    joint_from_mesh: &[Mat4; HAND_JOINT_COUNT],
    joints: &[Pose; HAND_JOINT_COUNT],
    shape: &HandShape,
) -> Skin {
    // The scale about the wrist applies to the joint positions (the tips move with it) and to the
    // whole mesh
    let wrist = joints[1].position;
    let points: [Mat4; HAND_JOINT_COUNT] = std::array::from_fn(|idx| {
        let position = wrist + (joints[idx].position - wrist) * shape.scale;
        let girth = shape.girth[idx];
        Mat4::from_rotation_translation(joints[idx].orientation, position)
            * Mat4::from_scale(Vec3::new(girth, girth, shape.length[idx]) * shape.scale)
            * joint_from_mesh[idx]
    });
    let normals = points.map(|matrix| matrix.inverse().transpose());

    Skin { points, normals }
}

fn skin_vertex(vertex: &MeshVertex, skin: &Skin) -> (Vec3, Vec3) {
    let mut position = Vec3::ZERO;
    let mut normal = Vec3::ZERO;
    for slot in 0..INFLUENCES {
        let weight = vertex.weights[slot];
        if weight > 0.0 {
            let joint = vertex.joints[slot];
            position += skin.points[joint].transform_point3(vertex.position) * weight;
            normal += skin.normals[joint].transform_vector3(vertex.normal) * weight;
        }
    }

    (position, normal.normalize_or_zero())
}

impl HandsRenderer {
    pub fn new(
        context: Rc<GraphicsContext>,
        view_resolution: UVec2,
        swapchain_textures: [Vec<u32>; 2],
        // Format of the swapchain textures (Rgba8UnormSrgb preferred)
        format: TextureFormat,
        // One of HANDS_SAMPLE_COUNTS
        sample_count: u32,
    ) -> Result<Self, String> {
        if !HANDS_SAMPLE_COUNTS.contains(&sample_count) {
            return Err(format!("unsupported hands sample count {sample_count}"));
        }
        let device = &context.device;

        let shader_module = device.create_shader_module(include_wgsl!("../resources/hands.wgsl"));
        let pipeline = device.create_render_pipeline(&RenderPipelineDescriptor {
            label: Some("hands"),
            layout: Some(&device.create_pipeline_layout(&PipelineLayoutDescriptor {
                label: Some("hands"),
                bind_group_layouts: &[],
                push_constant_ranges: &[PushConstantRange {
                    stages: ShaderStages::VERTEX_FRAGMENT,
                    range: 0..PUSH_CONSTANTS_SIZE,
                }],
            })),
            vertex: VertexState {
                module: &shader_module,
                entry_point: None,
                compilation_options: Default::default(),
                buffers: &[VertexBufferLayout {
                    array_stride: GPU_VERTEX_SIZE as u64,
                    step_mode: VertexStepMode::Vertex,
                    attributes: &vertex_attr_array![0 => Float32x3, 1 => Float32x3],
                }],
            },
            primitive: PrimitiveState {
                topology: PrimitiveTopology::TriangleList,
                // The projection flips Y for the GL swapchains, which flips the winding too
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: true,
                depth_compare: CompareFunction::Less,
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: MultisampleState {
                count: sample_count,
                ..Default::default()
            },
            fragment: Some(FragmentState {
                module: &shader_module,
                entry_point: None,
                compilation_options: Default::default(),
                targets: &[Some(ColorTargetState {
                    format,
                    blend: Some(BlendState::PREMULTIPLIED_ALPHA_BLENDING),
                    write_mask: ColorWrites::ALL,
                })],
            }),
            multiview: None,
            cache: None,
        });

        let meshes = [
            load_mesh(&context, "left", LEFT_HAND_MESH)?,
            load_mesh(&context, "right", RIGHT_HAND_MESH)?,
        ];

        let color_view = (sample_count > 1)
            .then(|| create_target(device, view_resolution, format, sample_count, "hands_color"));
        let depth_view = create_target(
            device,
            view_resolution,
            DEPTH_FORMAT,
            sample_count,
            "hands_depth",
        );

        let render_targets = [
            super::create_gl_swapchain(device, &swapchain_textures[0], view_resolution, format),
            super::create_gl_swapchain(device, &swapchain_textures[1], view_resolution, format),
        ];

        Ok(Self {
            context,
            pipeline,
            meshes,
            color_view,
            depth_view,
            render_targets,
            frame_index: std::cell::Cell::new(0),
        })
    }

    /// The mesh's own girth per joint (meters, 0 = not measured), see HandMesh::joint_radii
    pub fn mesh_joint_radii(&self, hand_idx: usize) -> [f32; HAND_JOINT_COUNT] {
        self.meshes[hand_idx].joint_radii
    }

    /// Bind pose joint positions of a hand mesh (0 = left, 1 = right), in mesh space
    pub fn mesh_joint_positions(&self, hand_idx: usize) -> [Vec3; HAND_JOINT_COUNT] {
        self.meshes[hand_idx]
            .joint_from_mesh
            .map(|joint_from_mesh| joint_from_mesh.inverse().w_axis.truncate())
    }

    /// Compares the skinned mesh with the tracked fingertips (diagnostics)
    pub fn check_skinning(
        &self,
        hand_idx: usize,
        joints: &[Pose; HAND_JOINT_COUNT],
    ) -> HandSkinningCheck {
        let mesh = &self.meshes[hand_idx];
        let skin = skin_matrices(&mesh.joint_from_mesh, joints, &HandShape::default());

        let max_tip_error = HAND_TIP_JOINTS
            .iter()
            .zip(mesh.tip_vertices)
            .map(|(&joint, vertex)| {
                skin_vertex(&mesh.vertices[vertex], &skin)
                    .0
                    .distance(joints[joint].position)
            })
            .fold(0.0, f32::max);
        let non_finite_vertices = mesh
            .vertices
            .iter()
            .filter(|vertex| !skin_vertex(vertex, &skin).0.is_finite())
            .count();

        HandSkinningCheck {
            max_tip_error,
            non_finite_vertices,
        }
    }

    /// `hands`: the 26 joints of each hand (left, right) in the same space as the view poses, or
    /// None to hide that hand. `shapes`: girth and scale per hand.
    pub fn render(
        &self,
        view_params: [HandsViewParams; 2],
        hands: [Option<[Pose; HAND_JOINT_COUNT]>; 2],
        shapes: [HandShape; 2],
    ) {
        let ring_index = self.frame_index.get() % VERTEX_BUFFER_RING;
        self.frame_index.set(self.frame_index.get().wrapping_add(1));
        for ((mesh, joints), shape) in self.meshes.iter().zip(&hands).zip(&shapes) {
            if let Some(joints) = joints {
                let skin = skin_matrices(&mesh.joint_from_mesh, joints, shape);
                let mut data = Vec::with_capacity(mesh.vertices.len() * GPU_VERTEX_SIZE);
                for vertex in &mesh.vertices {
                    let (position, normal) = skin_vertex(vertex, &skin);
                    for value in position.to_array().into_iter().chain(normal.to_array()) {
                        data.extend_from_slice(&value.to_le_bytes());
                    }
                }
                self.context
                    .queue
                    .write_buffer(&mesh.vertex_buffers[ring_index], 0, &data);
            }
        }

        let mut encoder = self
            .context
            .device
            .create_command_encoder(&CommandEncoderDescriptor {
                label: Some("hands_command_encoder"),
            });

        for (view_idx, view_input) in view_params.iter().enumerate() {
            let pose = view_input.view_params.pose;
            let view = Mat4::from_rotation_translation(pose.orientation, pose.position).inverse();
            let view_proj = super::projection_from_fov(view_input.view_params.fov) * view;

            let swapchain_view =
                &self.render_targets[view_idx][view_input.swapchain_index as usize];
            // With MSAA: draw into the multisampled target and resolve into the swapchain image;
            // without: draw straight into the swapchain image
            let (view, resolve_target, store) = match &self.color_view {
                Some(color_view) => (color_view, Some(swapchain_view), StoreOp::Discard),
                None => (swapchain_view, None, StoreOp::Store),
            };
            let mut pass = encoder.begin_render_pass(&RenderPassDescriptor {
                label: Some(&format!("hands_view_{view_idx}")),
                color_attachments: &[Some(RenderPassColorAttachment {
                    view,
                    resolve_target,
                    ops: Operations {
                        load: LoadOp::Clear(Color::TRANSPARENT),
                        store,
                    },
                })],
                depth_stencil_attachment: Some(RenderPassDepthStencilAttachment {
                    view: &self.depth_view,
                    depth_ops: Some(Operations {
                        load: LoadOp::Clear(1.0),
                        store: StoreOp::Discard,
                    }),
                    stencil_ops: None,
                }),
                ..Default::default()
            });

            let push_constants = view_proj
                .to_cols_array()
                .into_iter()
                .chain(pose.position.extend(1.0).to_array())
                .flat_map(f32::to_le_bytes)
                .collect::<Vec<u8>>();

            pass.set_pipeline(&self.pipeline);
            pass.set_push_constants(ShaderStages::VERTEX_FRAGMENT, 0, &push_constants);
            for (mesh, joints) in self.meshes.iter().zip(&hands) {
                if joints.is_some() {
                    pass.set_vertex_buffer(0, mesh.vertex_buffers[ring_index].slice(..));
                    pass.set_index_buffer(mesh.index_buffer.slice(..), IndexFormat::Uint16);
                    pass.draw_indexed(0..mesh.index_count, 0, 0..1);
                }
            }
        }

        self.context.queue.submit(Some(encoder.finish()));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alvr_common::glam::Quat;

    #[test]
    fn girth_keeps_joint_positions_and_scales_only_across_the_bone() {
        // Bind pose: every joint at the origin of mesh space, identity frames
        let joint_from_mesh = [Mat4::IDENTITY; HAND_JOINT_COUNT];
        let joints = std::array::from_fn(|idx| Pose {
            orientation: Quat::from_rotation_y(0.3),
            position: Vec3::new(0.1, 1.0, -0.4) + Vec3::Z * -(idx as f32) * 0.01,
        });
        let shape = HandShape {
            girth: [1.5; HAND_JOINT_COUNT],
            ..Default::default()
        };
        let skin = skin_matrices(&joint_from_mesh, &joints, &shape);
        let vertex = |position: Vec3, joint: usize| MeshVertex {
            position,
            normal: Vec3::X,
            joints: [joint, 0, 0, 0],
            weights: [1.0, 0.0, 0.0, 0.0],
        };
        // A vertex on the bone axis (joint-local Z) stays where tracking puts it
        let (on_axis, _) = skin_vertex(&vertex(Vec3::new(0.0, 0.0, -0.01), 10), &skin);
        let expected = joints[10].position + joints[10].orientation * Vec3::new(0.0, 0.0, -0.01);
        assert!(on_axis.distance(expected) < 1e-6);
        // A vertex beside the bone moves 1.5 times as far from the axis
        let (beside, normal) = skin_vertex(&vertex(Vec3::new(0.004, 0.0, 0.0), 10), &skin);
        assert!((beside.distance(joints[10].position) - 0.006).abs() < 1e-6);
        assert!((normal.length() - 1.0).abs() < 1e-5);

        // Scale about the wrist moves the joints
        let shape = HandShape {
            scale: 1.1,
            ..Default::default()
        };
        let skin = skin_matrices(&joint_from_mesh, &joints, &shape);
        let (tip, _) = skin_vertex(&vertex(Vec3::ZERO, 10), &skin);
        let wrist = joints[1].position;
        assert!(tip.distance(wrist + (joints[10].position - wrist) * 1.1) < 1e-6);
    }

    // A straight hand along -Z (each joint's -Z towards the next), 2 cm bones
    fn straight_hand() -> [Pose; HAND_JOINT_COUNT] {
        std::array::from_fn(|idx| Pose {
            orientation: Quat::IDENTITY,
            position: Vec3::new(idx as f32 * 0.001, 0.0, -0.02 * idx as f32),
        })
    }

    #[test]
    fn lengthened_fingers_stay_connected_and_the_palm_stays() {
        let joints = straight_hand();
        let longer = lengthen_fingers(&joints, 1.2);
        // Palm, wrist, metacarpals and knuckles unchanged
        for joint in [0, 1, 2, 3, 6, 7, 11, 12, 16, 17, 21, 22] {
            assert_eq!(
                longer[joint].position, joints[joint].position,
                "joint {joint}"
            );
        }
        for chain in HAND_FINGER_CHAINS {
            for pair in chain.windows(2) {
                let before = joints[pair[1]].position - joints[pair[0]].position;
                let after = longer[pair[1]].position - longer[pair[0]].position;
                // Each bone 1.2 times as long, same direction: the chain stays connected
                assert!((after - before * 1.2).length() < 1e-6, "{pair:?}");
            }
        }

        // The mesh stretches with the joints: a vertex at the next joint in a joint's bind frame
        // lands on the moved next joint
        let joint_from_mesh: [Mat4; HAND_JOINT_COUNT] =
            std::array::from_fn(|idx| Mat4::from_translation(-joints[idx].position));
        let shape = HandShape {
            length: finger_length_factors(1.2),
            ..Default::default()
        };
        let skin = skin_matrices(&joint_from_mesh, &longer, &shape);
        for chain in HAND_FINGER_CHAINS {
            for pair in chain.windows(2) {
                let vertex = MeshVertex {
                    position: joints[pair[1]].position,
                    normal: Vec3::Y,
                    joints: [pair[0], 0, 0, 0],
                    weights: [1.0, 0.0, 0.0, 0.0],
                };
                let (position, _) = skin_vertex(&vertex, &skin);
                // Bone direction of the straight hand is not exactly -Z (x offset): compare along
                // the bone only within the girth-free axis
                let expected = longer[pair[0]].position
                    + (joints[pair[1]].position - joints[pair[0]].position)
                        * Vec3::new(1.0, 1.0, 1.2);
                assert!(position.distance(expected) < 1e-6, "{pair:?}");
            }
        }
    }
}
