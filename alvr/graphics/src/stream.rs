use super::{
    GraphicsContext, MAX_IMMEDIATE_SIZE,
    opaque_regions::{
        EncodedPixelMap, MAX_OPAQUE_REGIONS, OpaqueRegionsRenderer, REGION_CLASS_NONE,
        REGION_CLASSES, RegionMeshParams, stream_stencil_state,
    },
    staging::StagingRenderer,
};
use alvr_common::{
    AlvrFoveatedEncodingParams, Pose, ViewParams,
    glam::{Mat4, UVec2, Vec2, Vec3, Vec4},
    info,
};
use alvr_session::{PassthroughMode, UpscalingConfig};
use std::{collections::VecDeque, ffi::c_void, iter, mem, rc::Rc};
use wgpu::{
    BindGroup, BindGroupDescriptor, BindGroupEntry, BindGroupLayoutDescriptor,
    BindGroupLayoutEntry, BindingResource, BindingType, Buffer, BufferBindingType,
    BufferDescriptor, BufferUsages, Color, ColorTargetState, ColorWrites, FragmentState, LoadOp,
    Operations, PipelineCompilationOptions, PipelineLayoutDescriptor, PrimitiveState,
    PrimitiveTopology, RenderPass, RenderPassColorAttachment, RenderPassDepthStencilAttachment,
    RenderPassDescriptor, RenderPipeline, RenderPipelineDescriptor, SamplerBindingType,
    SamplerDescriptor, ShaderStages, StoreOp, TextureSampleType, TextureView,
    TextureViewDescriptor, TextureViewDimension, VertexState, include_wgsl,
};

const FLOAT_SIZE: u32 = mem::size_of::<f32>() as u32;
const U32_SIZE: u32 = mem::size_of::<u32>() as u32;
const VEC4_SIZE: u32 = mem::size_of::<Vec4>() as u32;
const TRANSFORM_SIZE: u32 = mem::size_of::<Mat4>() as u32;

const TRANSFORM_CONST_OFFSET: u32 = 0;
const VIEW_INDEX_CONST_OFFSET: u32 = TRANSFORM_SIZE;
const PASSTHROUGH_MODE_OFFSET: u32 = VIEW_INDEX_CONST_OFFSET + U32_SIZE;
const ALPHA_CONST_OFFSET: u32 = PASSTHROUGH_MODE_OFFSET + U32_SIZE;
const CK_EDGE_CONST_OFFSET: u32 = ALPHA_CONST_OFFSET + FLOAT_SIZE;
const CK_CHANNEL0_CONST_OFFSET: u32 = CK_EDGE_CONST_OFFSET + U32_SIZE;
const CK_CHANNEL1_CONST_OFFSET: u32 = CK_CHANNEL0_CONST_OFFSET + VEC4_SIZE;
const CK_CHANNEL2_CONST_OFFSET: u32 = CK_CHANNEL1_CONST_OFFSET + VEC4_SIZE;
const IMMEDIATE_SIZE: u32 = CK_CHANNEL2_CONST_OFFSET + VEC4_SIZE;
const FOVEATION_EYE_UNIFORM_VEC4_COUNT: usize = 5;
pub(crate) const FOVEATION_UNIFORM_VEC4_COUNT: usize = 1 + FOVEATION_EYE_UNIFORM_VEC4_COUNT * 2;
const FOVEATION_UNIFORM_SIZE: usize = FOVEATION_UNIFORM_VEC4_COUNT * mem::size_of::<Vec4>();

// Chroma key edge cleanup is packed in a single u32 (immediates are at the 128 byte limit):
// bits 0..8 = erosion radius in texels, bits 8..16 = despill strength quantized to 0..=255.
// Must match the unpacking in stream.wgsl.
const CK_EDGE_MAX_EROSION_PX: u8 = 3;
const CK_EDGE_DESPILL_SHIFT: u32 = 8;

// Must match OpaqueRegions in stream.wgsl: flags, padding, key_chroma_dir
const OPAQUE_REGIONS_UNIFORM_FLOATS: usize = 4 + 4;
const OPAQUE_REGIONS_FLAG_DEBUG_TINT: u32 = 1;

// Pipeline variant bits, for logging
const PIPELINE_EDGE_CLEANUP_BIT: usize = 1;
const PIPELINE_OPAQUE_REGIONS_BIT: usize = 2;

const _: () = assert!(
    IMMEDIATE_SIZE <= MAX_IMMEDIATE_SIZE,
    "Immediate size exceeds the maximum size"
);

/// A region of the frame that must never be keyed out (see `alvr_packets::OpaqueRegion`). The quad
/// lies in the local XY plane of `pose` and faces +Z; with `curvature` > 0 it bends around the local
/// Y axis toward +Z with radius `width / (2 * PI * curvature)`.
#[derive(Clone, Copy, Debug)]
pub struct OpaqueRegionGeometry {
    /// Same space as the stream input view params
    pub pose: Pose,
    pub width: f32,
    pub height: f32,
    pub curvature: f32,
    pub inset: f32,
}

pub struct OpaqueRegionsParams<'a> {
    pub regions: &'a [OpaqueRegionGeometry],
    /// Width of the forced-keyed band just outside every region edge, in meters
    pub edge_band: f32,
    /// The forced-keyed band also extends this many stream pixels inside every region edge (the
    /// filtered/compressed edge texel). The region `inset` is not used.
    pub edge_key_px: f32,
    /// Opaque pixels within this many stream pixels further inside the keyed strip get despilled
    pub edge_despill_px: f32,
    /// Debug: tint the region classes instead of keying
    pub debug_tint: bool,
}

pub struct StreamViewParams {
    pub swapchain_index: u32,
    pub input_view_params: ViewParams,
    pub output_view_params: ViewParams,
}

#[derive(Debug)]
struct ViewObjects {
    bind_group: BindGroup,
    render_target: Vec<TextureView>,
}

pub struct StreamRenderer {
    context: Rc<GraphicsContext>,
    staging_renderer: StagingRenderer,
    // Without opaque regions, indexed by edge cleanup
    plain_pipelines: [RenderPipeline; 2],
    // With opaque regions (stencil attachment): pixels outside all regions, indexed by edge cleanup
    region_none_pipelines: [RenderPipeline; 2],
    // With opaque regions: pixels of each class of REGION_CLASSES
    region_class_pipelines: [RenderPipeline; 4],
    encoded_pixel_map: EncodedPixelMap,
    // (pipeline variant, opaque regions, debug tint), logged on change
    last_logged_state: Option<(usize, usize, bool)>,
    views_objects: [ViewObjects; 2],
    opaque_regions_buffer: Buffer,
    opaque_regions_data: Vec<f32>,
    opaque_regions_renderer: OpaqueRegionsRenderer,
    opaque_regions_count: usize,
    foveated_encoding: Option<AlvrFoveatedEncodingParams>,
    foveation_buffer: Buffer,
    last_foveation_center_shifts: Option<[Vec2; 2]>,
    staging_resolution: UVec2,
    // Key colour probes: the key direction of each pending readback (oldest first, parallel to
    // the staging renderer's pending probes), results of probes collected early (all slots
    // busy), and this loop's result when it needed no readback
    probe_key_dirs: VecDeque<Vec3>,
    probe_stash: Vec<f32>,
    probe_immediate: Option<f32>,
}

/// Key colour probe results collected by StreamRenderer::take_key_probes()
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct KeyProbeResults {
    /// Frames of earlier loops (collected a loop late): the largest key fraction among them
    pub earlier: Option<f32>,
    /// This loop's new frame (None: not probed, or still on the GPU and not waited for)
    pub newest: Option<f32>,
    /// This loop's probe is still pending (it is returned as `earlier` next time)
    pub newest_pending: bool,
}

impl StreamRenderer {
    #[expect(clippy::too_many_arguments)]
    #[cfg_attr(any(target_os = "macos", target_os = "ios"), expect(unused))]
    pub fn new(
        context: Rc<GraphicsContext>,
        base_view_resolution: UVec2,
        target_view_resolution: UVec2,
        swapchain_textures: [Vec<u32>; 2],
        target_format: u32,
        foveated_encoding: Option<AlvrFoveatedEncodingParams>,
        enable_srgb_correction: bool,
        fix_limited_range: bool,
        encoding_gamma: f32,
        upscaling: Option<UpscalingConfig>,
    ) -> Self {
        let device = &context.device;

        let target_format = super::gl_format_to_wgpu(target_format);

        let bind_group_layout = device.create_bind_group_layout(&BindGroupLayoutDescriptor {
            label: None,
            entries: &[
                BindGroupLayoutEntry {
                    binding: 0,
                    visibility: ShaderStages::FRAGMENT,
                    ty: BindingType::Texture {
                        sample_type: TextureSampleType::Float { filterable: true },
                        view_dimension: TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                BindGroupLayoutEntry {
                    binding: 1,
                    visibility: ShaderStages::FRAGMENT,
                    ty: BindingType::Sampler(SamplerBindingType::Filtering),
                    count: None,
                },
                // Foveation uniforms
                BindGroupLayoutEntry {
                    binding: 2,
                    visibility: ShaderStages::FRAGMENT,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // Opaque regions uniforms
                BindGroupLayoutEntry {
                    binding: 3,
                    visibility: ShaderStages::FRAGMENT,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        });

        let opaque_regions_data = vec![0.0; OPAQUE_REGIONS_UNIFORM_FLOATS];
        let opaque_regions_buffer = device.create_buffer(&BufferDescriptor {
            label: None,
            size: (OPAQUE_REGIONS_UNIFORM_FLOATS * mem::size_of::<f32>()) as u64,
            usage: BufferUsages::UNIFORM | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let mut constants = vec![];

        constants.extend([
            ("ENABLE_SRGB_CORRECTION", enable_srgb_correction.into()),
            ("ENCODING_GAMMA", encoding_gamma.into()),
        ]);

        let (staging_resolution, initial_foveation_uniforms) =
            if let Some(config) = &foveated_encoding {
                (
                    UVec2::from_array(config.encoded_view_resolution),
                    foveated_encoding_data(config, config.center_shifts.map(Vec2::from_array)),
                )
            } else {
                (
                    base_view_resolution,
                    [Vec4::ZERO; FOVEATION_UNIFORM_VEC4_COUNT],
                )
            };
        let encoded_pixel_map = if foveated_encoding.is_some() {
            EncodedPixelMap::from_foveation_uniforms(
                &initial_foveation_uniforms,
                staging_resolution,
            )
        } else {
            EncodedPixelMap::identity(base_view_resolution)
        };

        constants.push(("ENABLE_FFE", foveated_encoding.is_some().into()));

        let foveation_buffer = device.create_buffer(&BufferDescriptor {
            label: Some("stream foveation uniforms"),
            size: FOVEATION_UNIFORM_SIZE as u64,
            usage: BufferUsages::UNIFORM | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        context.queue.write_buffer(
            &foveation_buffer,
            0,
            &foveation_uniform_bytes(&initial_foveation_uniforms),
        );

        if foveated_encoding.is_none() {
            debug_assert_eq!(staging_resolution, base_view_resolution);
        }

        if let Some(upscaling) = upscaling {
            constants.extend([
                ("ENABLE_UPSCALING", true.into()),
                (
                    "UPSCALE_USE_EDGE_DIRECTION",
                    upscaling.edge_direction.into(),
                ),
                (
                    "UPSCALE_EDGE_THRESHOLD",
                    (upscaling.edge_threshold / 255.0).into(),
                ),
                ("UPSCALE_EDGE_SHARPNESS", upscaling.edge_sharpness.into()),
            ]);
        };

        // Note: Layout cannot be inferred because of a bug with immediates
        let pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[Some(&bind_group_layout)],
            immediate_size: IMMEDIATE_SIZE,
        });

        // All variants are created upfront, so toggling a feature never stalls a frame.
        // Each variant needs its own shader module: the wgpu GLES backend caches linked programs by
        // shader module and entry point only, ignoring pipeline constants, so variants sharing a
        // module would all silently get the first compiled program.
        let create_pipeline = |edge_cleanup: bool, forced_class: u8, with_stencil: bool| {
            let shader_module =
                device.create_shader_module(include_wgsl!("../resources/stream.wgsl"));
            let mut constants = constants.clone();
            constants.extend([
                ("ENABLE_EDGE_CLEANUP", edge_cleanup.into()),
                ("FORCED_REGION_CLASS", f64::from(forced_class)),
            ]);

            device.create_render_pipeline(&RenderPipelineDescriptor {
                label: None,
                layout: Some(&pipeline_layout),
                vertex: VertexState {
                    module: &shader_module,
                    entry_point: None,
                    compilation_options: PipelineCompilationOptions {
                        constants: &constants,
                        zero_initialize_workgroup_memory: false,
                    },
                    buffers: &[],
                },
                primitive: PrimitiveState {
                    topology: PrimitiveTopology::TriangleStrip,
                    ..Default::default()
                },
                depth_stencil: with_stencil.then(stream_stencil_state),
                multisample: Default::default(),
                fragment: Some(FragmentState {
                    module: &shader_module,
                    entry_point: None,
                    compilation_options: PipelineCompilationOptions {
                        constants: &constants,
                        zero_initialize_workgroup_memory: false,
                    },
                    targets: &[Some(ColorTargetState {
                        format: target_format,
                        blend: None,
                        write_mask: ColorWrites::ALL,
                    })],
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        let plain_pipelines = [false, true]
            .map(|edge_cleanup| create_pipeline(edge_cleanup, REGION_CLASS_NONE, false));
        let region_none_pipelines = [false, true]
            .map(|edge_cleanup| create_pipeline(edge_cleanup, REGION_CLASS_NONE, true));
        let region_class_pipelines =
            REGION_CLASSES.map(|class| create_pipeline(false, class, true));

        let sampler = device.create_sampler(&SamplerDescriptor {
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });

        let opaque_regions_renderer =
            OpaqueRegionsRenderer::new(Rc::clone(&context), target_view_resolution, target_format);

        let mut view_objects = vec![];
        let mut staging_textures_gl = vec![];
        for target_swapchain in &swapchain_textures {
            let staging_texture = super::create_texture(device, staging_resolution, target_format);

            let bind_group = device.create_bind_group(&BindGroupDescriptor {
                label: None,
                layout: &bind_group_layout,
                entries: &[
                    BindGroupEntry {
                        binding: 0,
                        resource: BindingResource::TextureView(
                            &staging_texture.create_view(&TextureViewDescriptor::default()),
                        ),
                    },
                    BindGroupEntry {
                        binding: 1,
                        resource: BindingResource::Sampler(&sampler),
                    },
                    BindGroupEntry {
                        binding: 2,
                        resource: foveation_buffer.as_entire_binding(),
                    },
                    BindGroupEntry {
                        binding: 3,
                        resource: opaque_regions_buffer.as_entire_binding(),
                    },
                ],
            });

            let render_target = super::create_gl_swapchain(
                device,
                target_swapchain,
                target_view_resolution,
                target_format,
            );

            view_objects.push(ViewObjects {
                bind_group,
                render_target,
            });

            #[cfg(not(any(target_os = "macos", target_os = "ios")))]
            {
                let staging_texture_gl = unsafe {
                    let tex = staging_texture.as_hal::<wgpu::hal::api::Gles>();
                    let wgpu::hal::gles::TextureInner::Texture { raw, .. } = tex.unwrap().inner
                    else {
                        panic!("invalid texture type");
                    };
                    raw
                };
                staging_textures_gl.push(staging_texture_gl);
            }
        }

        let staging_renderer = StagingRenderer::new(
            Rc::clone(&context),
            staging_textures_gl.try_into().unwrap(),
            staging_resolution,
            fix_limited_range,
        );

        Self {
            context,
            staging_renderer,
            plain_pipelines,
            region_none_pipelines,
            region_class_pipelines,
            encoded_pixel_map,
            last_logged_state: None,
            views_objects: view_objects.try_into().unwrap(),
            opaque_regions_buffer,
            opaque_regions_data,
            opaque_regions_renderer,
            opaque_regions_count: 0,
            foveated_encoding,
            foveation_buffer,
            last_foveation_center_shifts: None,
            staging_resolution,
            probe_key_dirs: VecDeque::new(),
            probe_stash: Vec::new(),
            probe_immediate: None,
        }
    }

    // Builds the per view region class meshes (relative to each view's render pose and FOV) and
    // fills the uniform. Returns the number of regions, and whether the uniform must be uploaded.
    fn update_opaque_regions(
        &mut self,
        view_params: &[StreamViewParams; 2],
        passthrough: Option<&PassthroughMode>,
        opaque_regions: &OpaqueRegionsParams,
    ) -> (usize, bool) {
        let regions = match passthrough {
            Some(PassthroughMode::RgbChromaKey(_) | PassthroughMode::HsvChromaKey(_)) => {
                &opaque_regions.regions[..opaque_regions.regions.len().min(MAX_OPAQUE_REGIONS)]
            }
            _ => &[],
        };
        if regions.is_empty() && self.opaque_regions_count == 0 {
            return (0, false);
        }
        self.opaque_regions_count = regions.len();

        let mesh_params = RegionMeshParams {
            edge_band: opaque_regions.edge_band,
            edge_key_px: opaque_regions.edge_key_px,
            edge_despill_px: opaque_regions.edge_despill_px,
        };
        for (view_idx, view) in view_params.iter().enumerate() {
            self.opaque_regions_renderer.update(
                view_idx,
                regions,
                view.input_view_params.pose,
                view.input_view_params.fov,
                &self.encoded_pixel_map,
                &mesh_params,
            );
        }

        let data = &mut self.opaque_regions_data;
        data.fill(0.0);
        data[0] = f32::from_bits(if opaque_regions.debug_tint {
            OPAQUE_REGIONS_FLAG_DEBUG_TINT
        } else {
            0
        });
        let key_dir = passthrough.map_or(Vec3::ZERO, key_chroma_direction);
        data[4..7].copy_from_slice(&key_dir.to_array());

        (regions.len(), true)
    }

    fn log_opaque_regions_diagnostic(
        &self,
        opaque_regions: &OpaqueRegionsParams,
        pipeline_variant: usize,
    ) {
        info!(
            "Stream pipeline variant {pipeline_variant}: opaque regions {}, class mesh vertices \
             {:?}, debug tint {}, band {:.3} m, key {} px, despill {} px",
            self.opaque_regions_count,
            self.opaque_regions_renderer.vertex_counts(),
            opaque_regions.debug_tint,
            opaque_regions.edge_band,
            opaque_regions.edge_key_px,
            opaque_regions.edge_despill_px,
        );
    }

    // Immediates are set again after every pipeline change: the GLES backend emulates them with
    // per program uniforms
    fn draw_stream_quad(
        &self,
        render_pass: &mut RenderPass,
        pipeline: &RenderPipeline,
        view_idx: usize,
        transform_bytes: &[u8],
        passthrough: Option<&PassthroughMode>,
    ) {
        render_pass.set_pipeline(pipeline);
        render_pass.set_immediates(TRANSFORM_CONST_OFFSET, transform_bytes);
        render_pass.set_immediates(VIEW_INDEX_CONST_OFFSET, &(view_idx as u32).to_le_bytes());
        render_pass.set_bind_group(0, &self.views_objects[view_idx].bind_group, &[]);
        set_passthrough_immediates(render_pass, passthrough);
        render_pass.draw(0..4, 0..1);
    }

    /// Runtime foveation centers must already be encoder-aligned. `None` uses the negotiated centers.
    ///
    /// # Safety
    /// `hardware_buffer` must be a valid pointer to a ANativeWindowBuffer.
    pub fn render(
        &mut self,
        hardware_buffer: *mut c_void,
        view_params: [StreamViewParams; 2],
        passthrough: Option<&PassthroughMode>,
        foveation_center_shifts: Option<[Vec2; 2]>,
        opaque_regions: &OpaqueRegionsParams,
        // Key chroma direction: probe the new frame for the key colour (see take_key_probes())
        key_probe: Option<Vec3>,
    ) -> bool {
        // Before the opaque regions: their meshes use the encoded pixel map of these centers
        if let Some(config) = &self.foveated_encoding
            && self.last_foveation_center_shifts != foveation_center_shifts
        {
            let center_shifts = foveation_center_shifts
                .unwrap_or_else(|| config.center_shifts.map(Vec2::from_array));
            let uniforms = foveated_encoding_data(config, center_shifts);
            self.context.queue.write_buffer(
                &self.foveation_buffer,
                0,
                &foveation_uniform_bytes(&uniforms),
            );
            self.encoded_pixel_map =
                EncodedPixelMap::from_foveation_uniforms(&uniforms, self.staging_resolution);
            self.last_foveation_center_shifts = foveation_center_shifts;
        }

        let (opaque_regions_count, upload) =
            self.update_opaque_regions(&view_params, passthrough, opaque_regions);
        if upload {
            let bytes = self
                .opaque_regions_data
                .iter()
                .flat_map(|v| v.to_le_bytes())
                .collect::<Vec<u8>>();
            self.context
                .queue
                .write_buffer(&self.opaque_regions_buffer, 0, &bytes);
        }

        let edge_cleanup = match passthrough {
            Some(PassthroughMode::RgbChromaKey(config)) => {
                config.edge_erosion_px > 0 || config.edge_despill > 0.0
            }
            Some(PassthroughMode::HsvChromaKey(config)) => {
                config.edge_erosion_px > 0 || config.edge_despill > 0.0
            }
            Some(PassthroughMode::Blend { .. }) | None => false,
        };
        let pipeline_variant = if edge_cleanup {
            PIPELINE_EDGE_CLEANUP_BIT
        } else {
            0
        } | if opaque_regions_count > 0 {
            PIPELINE_OPAQUE_REGIONS_BIT
        } else {
            0
        };
        let log_state = (
            pipeline_variant,
            opaque_regions_count,
            opaque_regions.debug_tint,
        );
        if self.last_logged_state != Some(log_state) {
            self.last_logged_state = Some(log_state);
            self.log_opaque_regions_diagnostic(opaque_regions, pipeline_variant);
        }

        // if hardware_buffer is available copy stream to staging texture
        if !hardware_buffer.is_null() {
            self.staging_renderer.render(hardware_buffer);
        }
        // Right after the staging copy, before the stream pass is queued: an asynchronous
        // readback of the staged frame; its result is collected later in the loop
        // (take_key_probes), so the render thread does not wait for the GPU here
        self.probe_immediate = None;
        let probe_issued = match key_probe.filter(|_| !hardware_buffer.is_null()) {
            Some(key_dir) if key_dir == Vec3::ZERO => {
                self.probe_immediate = Some(0.0);
                true
            }
            Some(key_dir) => {
                if let Some(samples) = self.staging_renderer.issue_probe() {
                    // All slots were busy: the oldest was collected now
                    let dir = self.probe_key_dirs.pop_front().unwrap_or(Vec3::ZERO);
                    self.probe_stash.push(key_fraction(&samples, dir));
                }
                self.probe_key_dirs.push_back(key_dir);
                true
            }
            None => false,
        };

        let mut encoder = self
            .context
            .device
            .create_command_encoder(&Default::default());

        for (view_idx, view_params) in view_params.iter().enumerate() {
            let input_fov = view_params.input_view_params.fov;

            let tanl = f32::tan(input_fov.left);
            let tanr = f32::tan(input_fov.right);
            let tanu = f32::tan(input_fov.up);
            let tand = f32::tan(input_fov.down);

            let width = tanr - tanl;
            let height = tanu - tand;
            let quad_depth = 1000.0;

            let output_mat4 = Mat4::from_translation(view_params.output_view_params.pose.position)
                * Mat4::from_quat(view_params.output_view_params.pose.orientation);
            let input_mat4 = Mat4::from_translation(view_params.input_view_params.pose.position)
                * Mat4::from_quat(view_params.input_view_params.pose.orientation);

            // The image is at z = -1.0, so we use tangents for the size
            let model_mat =
                Mat4::from_translation(Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: -quad_depth * 0.5,
                }) * Mat4::from_scale(Vec3::new(quad_depth, quad_depth, quad_depth * 0.5))
                    * Mat4::from_translation(Vec3::new(
                        width / 2.0 + tanl,
                        height / 2.0 + tand,
                        -1.0,
                    ))
                    * Mat4::from_scale(Vec3::new(width, height, 1.));
            let view_mat = output_mat4.inverse() * input_mat4;
            let proj_mat = super::projection_from_fov(view_params.output_view_params.fov);

            let transform = proj_mat * view_mat * model_mat;

            let with_regions = pipeline_variant & PIPELINE_OPAQUE_REGIONS_BIT != 0;
            let mut render_pass = encoder.begin_render_pass(&RenderPassDescriptor {
                label: None,
                color_attachments: &[Some(RenderPassColorAttachment {
                    view: &self.views_objects[view_idx].render_target
                        [view_params.swapchain_index as usize],
                    depth_slice: None,
                    resolve_target: None,
                    ops: Operations {
                        load: LoadOp::Clear(Color::BLACK),
                        store: StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: with_regions.then(|| RenderPassDepthStencilAttachment {
                    view: self.opaque_regions_renderer.stencil_view(view_idx),
                    depth_ops: None,
                    stencil_ops: Some(Operations {
                        load: LoadOp::Clear(u32::from(REGION_CLASS_NONE)),
                        store: StoreOp::Discard,
                    }),
                }),
                ..Default::default()
            });

            let transform_bytes = transform
                .to_cols_array()
                .iter()
                .flat_map(|v| v.to_le_bytes())
                .collect::<Vec<u8>>();

            let edge_cleanup_idx = usize::from(pipeline_variant & PIPELINE_EDGE_CLEANUP_BIT != 0);
            if with_regions {
                self.opaque_regions_renderer.draw_stencil(
                    &mut render_pass,
                    view_idx,
                    &transform_bytes,
                );

                render_pass.set_stencil_reference(u32::from(REGION_CLASS_NONE));
                self.draw_stream_quad(
                    &mut render_pass,
                    &self.region_none_pipelines[edge_cleanup_idx],
                    view_idx,
                    &transform_bytes,
                    passthrough,
                );
                for (class, pipeline) in REGION_CLASSES.iter().zip(&self.region_class_pipelines) {
                    render_pass.set_stencil_reference(u32::from(*class));
                    self.draw_stream_quad(
                        &mut render_pass,
                        pipeline,
                        view_idx,
                        &transform_bytes,
                        passthrough,
                    );
                }
            } else {
                self.draw_stream_quad(
                    &mut render_pass,
                    &self.plain_pipelines[edge_cleanup_idx],
                    view_idx,
                    &transform_bytes,
                    passthrough,
                );
            }
        }

        self.context.queue.submit(iter::once(encoder.finish()));

        probe_issued
    }

    /// Key colour probe results: the fraction (0..1) of a sparse sample grid of a staged frame
    /// (left view) whose colour is the key colour or a darkened/faded version of it (same chroma
    /// direction, clearly saturated). Probes of earlier loops are collected (waiting if needed:
    /// they are a loop old, so done). This loop's probe (newest_issued: render() returned true
    /// this loop) is collected if the GPU has finished it, or waited for if wait_for_newest;
    /// otherwise it stays pending and is reported as `earlier` next time.
    pub fn take_key_probes(
        &mut self,
        newest_issued: bool,
        wait_for_newest: bool,
    ) -> KeyProbeResults {
        let mut results = KeyProbeResults::default();
        let merge = |slot: &mut Option<f32>, value: f32| {
            *slot = Some(slot.map_or(value, |old: f32| old.max(value)));
        };
        for value in self.probe_stash.drain(..) {
            merge(&mut results.earlier, value);
        }
        let newest_in_flight = newest_issued && self.probe_immediate.is_none();
        let older = self
            .staging_renderer
            .pending_probes()
            .saturating_sub(usize::from(newest_in_flight));
        for _ in 0..older {
            let dir = self.probe_key_dirs.pop_front().unwrap_or(Vec3::ZERO);
            if let Some(samples) = self.staging_renderer.take_oldest_probe(true) {
                merge(&mut results.earlier, key_fraction(&samples, dir));
            }
        }
        if newest_issued {
            if let Some(value) = self.probe_immediate.take() {
                results.newest = Some(value);
            } else if let Some(samples) = self.staging_renderer.take_oldest_probe(wait_for_newest) {
                let dir = self.probe_key_dirs.pop_front().unwrap_or(Vec3::ZERO);
                results.newest = Some(key_fraction(&samples, dir));
            } else {
                results.newest_pending = self.staging_renderer.pending_probes() > 0;
            }
        }
        results
    }
}

/// key_chroma_direction() of magenta (SteamVR's default chroma key background)
pub fn magenta_chroma_direction() -> Vec3 {
    const LUMA: Vec3 = Vec3::new(0.2126, 0.7152, 0.0722);
    let key = Vec3::new(1.0, 0.0, 1.0);
    (key - Vec3::splat(key.dot(LUMA))).normalize()
}

/// Minimum chroma magnitude (0..1 colour units) of a key-like sample: excludes greys and blacks
const KEY_PROBE_MIN_CHROMA: f32 = 0.06;
/// Minimum cosine between a sample's chroma and the key chroma
const KEY_PROBE_MIN_COSINE: f32 = 0.85;

pub fn key_fraction(samples: &[[u8; 4]], key_dir: Vec3) -> f32 {
    const LUMA: Vec3 = Vec3::new(0.2126, 0.7152, 0.0722);
    if samples.is_empty() {
        return 0.0;
    }
    let key_like = samples
        .iter()
        .filter(|sample| {
            let colour = Vec3::new(
                f32::from(sample[0]),
                f32::from(sample[1]),
                f32::from(sample[2]),
            ) / 255.0;
            let chroma = colour - Vec3::splat(colour.dot(LUMA));
            let magnitude = chroma.length();
            magnitude >= KEY_PROBE_MIN_CHROMA
                && chroma.dot(key_dir) >= KEY_PROBE_MIN_COSINE * magnitude
        })
        .count();

    key_like as f32 / samples.len() as f32
}

// Normalized chroma (colour minus luma) of the key colour, in the decoded stream colour space.
// Must match key_color() in stream.wgsl.
pub fn key_chroma_direction(config: &PassthroughMode) -> Vec3 {
    const LUMA: Vec3 = Vec3::new(0.2126, 0.7152, 0.0722);

    let key = match config {
        PassthroughMode::RgbChromaKey(config) => {
            Vec3::new(
                f32::from(config.red),
                f32::from(config.green),
                f32::from(config.blue),
            ) / 255.
        }
        PassthroughMode::HsvChromaKey(config) => {
            // Middle of the fully keyed hue range [start_min, end_min], full saturation and value
            let start_min = config.hue_start_min_deg / 360.;
            let end_min = config.hue_end_min_deg / 360.;
            let hue = (start_min + (end_min - start_min).rem_euclid(1.) * 0.5).rem_euclid(1.);
            let channel =
                |offset: f32| (((hue + offset).rem_euclid(1.) * 6. - 3.).abs() - 1.).clamp(0., 1.);
            Vec3::new(channel(1.), channel(2. / 3.), channel(1. / 3.))
        }
        PassthroughMode::Blend { .. } => return Vec3::ZERO,
    };

    let chroma = key - Vec3::splat(key.dot(LUMA));
    chroma.try_normalize().unwrap_or(Vec3::ZERO)
}

fn set_passthrough_immediates(render_pass: &mut RenderPass, config: Option<&PassthroughMode>) {
    const DEG_TO_NORM: f32 = 1. / 360.;

    fn set_u32(render_pass: &mut RenderPass, offset: u32, value: u32) {
        render_pass.set_immediates(offset, &value.to_le_bytes());
    }

    fn set_float(render_pass: &mut RenderPass, offset: u32, value: f32) {
        render_pass.set_immediates(offset, &value.to_le_bytes());
    }

    fn set_vec4(render_pass: &mut RenderPass, offset: u32, value: Vec4) {
        render_pass.set_immediates(offset, &value.x.to_le_bytes());
        render_pass.set_immediates(offset + FLOAT_SIZE, &value.y.to_le_bytes());
        render_pass.set_immediates(offset + 2 * FLOAT_SIZE, &value.z.to_le_bytes());
        render_pass.set_immediates(offset + 3 * FLOAT_SIZE, &value.w.to_le_bytes());
    }

    fn pack_edge_config(erosion_px: u8, despill: f32) -> u32 {
        let erosion_px = u32::from(erosion_px.min(CK_EDGE_MAX_EROSION_PX));
        let despill = (despill.clamp(0., 1.) * 255.).round() as u32;

        erosion_px | (despill << CK_EDGE_DESPILL_SHIFT)
    }

    match config {
        None => {
            set_u32(render_pass, PASSTHROUGH_MODE_OFFSET, 0);
            set_float(render_pass, ALPHA_CONST_OFFSET, 1.);
            set_u32(render_pass, CK_EDGE_CONST_OFFSET, 0);
        }
        Some(PassthroughMode::Blend { threshold, .. }) => {
            set_u32(render_pass, PASSTHROUGH_MODE_OFFSET, 0);
            set_float(render_pass, ALPHA_CONST_OFFSET, 1. - threshold);
            set_u32(render_pass, CK_EDGE_CONST_OFFSET, 0);
        }
        Some(PassthroughMode::RgbChromaKey(config)) => {
            set_u32(render_pass, PASSTHROUGH_MODE_OFFSET, 1);
            set_u32(
                render_pass,
                CK_EDGE_CONST_OFFSET,
                pack_edge_config(config.edge_erosion_px, config.edge_despill),
            );

            let norm = |v| v as f32 / 255.;

            let red = norm(config.red);
            let green = norm(config.green);
            let blue = norm(config.blue);

            let thresh = norm(config.distance_threshold);

            let up_feather = 1. + config.feathering;
            let down_feather = 1. - config.feathering;

            let range_vec =
                thresh * Vec4::new(-up_feather, -down_feather, down_feather, up_feather);

            set_vec4(render_pass, CK_CHANNEL0_CONST_OFFSET, red + range_vec);
            set_vec4(render_pass, CK_CHANNEL1_CONST_OFFSET, green + range_vec);
            set_vec4(render_pass, CK_CHANNEL2_CONST_OFFSET, blue + range_vec);
        }
        Some(PassthroughMode::HsvChromaKey(config)) => {
            set_u32(render_pass, PASSTHROUGH_MODE_OFFSET, 2);
            set_u32(
                render_pass,
                CK_EDGE_CONST_OFFSET,
                pack_edge_config(config.edge_erosion_px, config.edge_despill),
            );

            set_vec4(
                render_pass,
                CK_CHANNEL0_CONST_OFFSET,
                Vec4::new(
                    config.hue_start_max_deg,
                    config.hue_start_min_deg,
                    config.hue_end_min_deg,
                    config.hue_end_max_deg,
                ) * DEG_TO_NORM,
            );

            set_vec4(
                render_pass,
                CK_CHANNEL1_CONST_OFFSET,
                Vec4::new(
                    config.saturation_start_max,
                    config.saturation_start_min,
                    config.saturation_end_min,
                    config.saturation_end_max,
                ),
            );

            set_vec4(
                render_pass,
                CK_CHANNEL2_CONST_OFFSET,
                Vec4::new(
                    config.value_start_max,
                    config.value_start_min,
                    config.value_end_min,
                    config.value_end_max,
                ),
            );
        }
    }
}

pub(crate) fn foveated_encoding_data(
    config: &AlvrFoveatedEncodingParams,
    center_shifts: [Vec2; 2],
) -> [Vec4; FOVEATION_UNIFORM_VEC4_COUNT] {
    let center_size_aligned = Vec2::from_array(config.center_size);
    let edge_ratio = Vec2::from_array(config.edge_ratio);
    let view_ratio_aligned = Vec2::from_array(config.view_ratio);

    let c0 = (1. - center_size_aligned) * 0.5;
    let c2 = (edge_ratio - 1.) * center_size_aligned + 1.;

    let mut uniforms = [Vec4::ZERO; FOVEATION_UNIFORM_VEC4_COUNT];
    let [common_uniform, eye_uniforms @ ..] = &mut uniforms;
    *common_uniform = Vec4::new(
        view_ratio_aligned.x,
        view_ratio_aligned.y,
        edge_ratio.x,
        edge_ratio.y,
    );

    for (uniforms, center_shift) in eye_uniforms
        .chunks_exact_mut(FOVEATION_EYE_UNIFORM_VEC4_COUNT)
        .zip(center_shifts)
    {
        let c1 = (edge_ratio - 1.) * c0 * (center_shift + 1.) / edge_ratio;
        let lo_bound = c0 * (center_shift + 1.);
        let hi_bound = c0 * (center_shift - 1.) + 1.;
        let lo_bound_c = c0 * (center_shift + 1.) / c2;
        let hi_bound_c = c0 * (center_shift - 1.) / c2 + 1.;

        let a_left = c2 * (1. - edge_ratio) / (edge_ratio * lo_bound_c);
        let b_left = (c1 + c2 * lo_bound_c) / lo_bound_c;

        let a_right = c2 * (edge_ratio - 1.) / (edge_ratio * (1. - hi_bound_c));
        let b_right = (c2 - edge_ratio * c1 - 2. * edge_ratio * c2
            + c2 * edge_ratio * (1. - hi_bound_c)
            + edge_ratio)
            / (edge_ratio * (1. - hi_bound_c));
        let c_right = (c2 * edge_ratio - c2) * (c1 - hi_bound_c + c2 * hi_bound_c)
            / (edge_ratio * (1. - hi_bound_c) * (1. - hi_bound_c));

        let [center, bounds, left, right, right_curve] = uniforms else {
            unreachable!("foveation eye uniform chunk must contain five vectors");
        };
        *center = Vec4::new(c1.x, c1.y, c2.x, c2.y);
        *bounds = Vec4::new(lo_bound.x, lo_bound.y, hi_bound.x, hi_bound.y);
        *left = Vec4::new(a_left.x, a_left.y, b_left.x, b_left.y);
        *right = Vec4::new(a_right.x, a_right.y, b_right.x, b_right.y);
        *right_curve = Vec4::new(c_right.x, c_right.y, 0.0, 0.0);
    }

    uniforms
}

fn foveation_uniform_bytes(
    uniforms: &[Vec4; FOVEATION_UNIFORM_VEC4_COUNT],
) -> [u8; FOVEATION_UNIFORM_SIZE] {
    let mut bytes = [0; FOVEATION_UNIFORM_SIZE];

    for (chunk, value) in bytes
        .chunks_exact_mut(mem::size_of::<f32>())
        .zip(uniforms.iter().flat_map(|value| value.to_array()))
    {
        chunk.copy_from_slice(&value.to_ne_bytes());
    }

    bytes
}

pub fn compute_target_view_resolution(
    resolution: UVec2,
    upscaling: &Option<UpscalingConfig>,
) -> UVec2 {
    let mut target_resolution = resolution.as_vec2();
    if let Some(upscaling) = upscaling {
        target_resolution *= upscaling.upscale_factor;
    }
    target_resolution.as_uvec2()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn magenta_dir() -> Vec3 {
        let key = Vec3::new(1.0, 0.0, 1.0);
        (key - Vec3::splat(key.dot(Vec3::new(0.2126, 0.7152, 0.0722)))).normalize()
    }

    #[test]
    fn key_fraction_counts_full_and_faded_key_but_not_content() {
        let dir = magenta_dir();
        // Measured in the 2026-11-10 13:31 recording: SteamVR magenta, its fade to black, V3's
        // dark dialog, V3's sunny scene, black
        assert_eq!(key_fraction(&[[246, 4, 246, 255]], dir), 1.0);
        assert_eq!(key_fraction(&[[143, 5, 145, 255]], dir), 1.0);
        assert_eq!(key_fraction(&[[40, 4, 40, 255]], dir), 1.0);
        assert_eq!(key_fraction(&[[20, 20, 24, 255]], dir), 0.0);
        assert_eq!(key_fraction(&[[187, 180, 160, 255]], dir), 0.0);
        assert_eq!(key_fraction(&[[3, 3, 4, 255]], dir), 0.0);
        // Content colours that are not magenta: red, blue, Tenstar green
        assert_eq!(key_fraction(&[[200, 30, 30, 255]], dir), 0.0);
        assert_eq!(key_fraction(&[[30, 30, 200, 255]], dir), 0.0);
        assert_eq!(key_fraction(&[[147, 203, 77, 255]], dir), 0.0);
        assert_eq!(
            key_fraction(&[[246, 4, 246, 255], [20, 20, 24, 255]], dir),
            0.5
        );
    }
}
