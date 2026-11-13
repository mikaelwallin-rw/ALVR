use super::{GraphicsContext, ck};
use crate::GL_TEXTURE_EXTERNAL_OES;
use alvr_common::glam::{IVec2, UVec2};
use glow::{self as gl, HasContext};
use std::{collections::VecDeque, ffi::c_void, rc::Rc};

fn create_program(
    gl: &gl::Context,
    vertex_shader_source: &str,
    fragment_shader_source: &str,
) -> gl::Program {
    unsafe {
        let vertex_shader = ck!(gl.create_shader(gl::VERTEX_SHADER).unwrap());
        ck!(gl.shader_source(vertex_shader, vertex_shader_source));
        ck!(gl.compile_shader(vertex_shader));
        if !gl.get_shader_compile_status(vertex_shader) {
            panic!(
                "Failed to compile vertex shader: {}",
                gl.get_shader_info_log(vertex_shader)
            );
        }

        let fragment_shader = ck!(gl.create_shader(gl::FRAGMENT_SHADER).unwrap());
        ck!(gl.shader_source(fragment_shader, fragment_shader_source));
        ck!(gl.compile_shader(fragment_shader));
        if !gl.get_shader_compile_status(fragment_shader) {
            panic!(
                "Failed to compile fragment shader: {}",
                gl.get_shader_info_log(fragment_shader)
            );
        }

        let program = ck!(gl.create_program().unwrap());
        ck!(gl.attach_shader(program, vertex_shader));
        ck!(gl.attach_shader(program, fragment_shader));
        ck!(gl.link_program(program));
        if !gl.get_program_link_status(program) {
            panic!(
                "Failed to link program: {}",
                gl.get_program_info_log(program)
            );
        }

        ck!(gl.delete_shader(vertex_shader));
        ck!(gl.delete_shader(fragment_shader));

        program
    }
}

pub struct StagingRenderer {
    context: Rc<GraphicsContext>,
    program: gl::Program,
    view_idx_uloc: gl::UniformLocation,
    surface_texture: gl::Texture,
    framebuffers: [gl::Framebuffer; 2],
    viewport_size: IVec2,
    // Key colour probe target (renderbuffer, framebuffer), created on first use
    probe_target: Option<(gl::Renderbuffer, gl::Framebuffer)>,
    // Pixel pack buffers of the asynchronous probes (created on first use), the pending probes
    // (slot, fence) oldest first, and the next slot
    probe_buffers: Option<[gl::Buffer; PROBE_SLOTS]>,
    probe_pending: VecDeque<(usize, gl::Fence)>,
    probe_next_slot: usize,
}

/// Side of the sample grid of issue_probe()
pub const PROBE_GRID: usize = 16;
const PROBE_BYTES: usize = PROBE_GRID * PROBE_GRID * 4;
/// Probes that can be pending at once (this frame's and one collected a loop later)
pub const PROBE_SLOTS: usize = 2;

impl StagingRenderer {
    pub fn new(
        context: Rc<GraphicsContext>,
        staging_textures: [gl::Texture; 2],
        view_resolution: UVec2,
        fix_limited_range: bool,
    ) -> Self {
        let gl = &context.gl_context;
        context.make_current();

        // Add #defines into the shader after the first line
        let mut frag_lines: Vec<&str> = include_str!("../resources/staging_fragment.glsl")
            .lines()
            .collect();
        if fix_limited_range {
            frag_lines.insert(1, "#line 0 1\n#define FIX_LIMITED_RANGE");
        }
        let frag_str = frag_lines.join("\n");

        let program = create_program(
            gl,
            include_str!("../resources/staging_vertex.glsl"),
            frag_str.as_str(),
        );

        unsafe {
            // This is an external surface and storage should not be initialized
            let surface_texture = ck!(gl.create_texture().unwrap());

            let mut framebuffers = vec![];
            for tex in staging_textures {
                let framebuffer = ck!(gl.create_framebuffer().unwrap());
                ck!(gl.bind_framebuffer(gl::DRAW_FRAMEBUFFER, Some(framebuffer)));
                ck!(gl.framebuffer_texture_2d(
                    gl::DRAW_FRAMEBUFFER,
                    gl::COLOR_ATTACHMENT0,
                    gl::TEXTURE_2D,
                    Some(tex),
                    0,
                ));

                framebuffers.push(framebuffer);
            }

            ck!(gl.bind_framebuffer(gl::FRAMEBUFFER, None));

            let view_idx_uloc = ck!(gl.get_uniform_location(program, "view_idx")).unwrap();

            Self {
                context,
                program,
                surface_texture,
                view_idx_uloc,
                framebuffers: framebuffers.try_into().unwrap(),
                viewport_size: view_resolution.as_ivec2(),
                probe_target: None,
                probe_buffers: None,
                probe_pending: VecDeque::new(),
                probe_next_slot: 0,
            }
        }
    }

    #[allow(unused_variables)]
    pub fn render(&self, hardware_buffer: *mut c_void) {
        let gl = &self.context.gl_context;
        self.context.make_current();

        unsafe {
            self.context.render_ahardwarebuffer_using_texture(
                hardware_buffer,
                self.surface_texture,
                || {
                    ck!(gl.use_program(Some(self.program)));

                    ck!(gl.viewport(0, 0, self.viewport_size.x, self.viewport_size.y));
                    ck!(gl.disable(gl::SCISSOR_TEST));
                    ck!(gl.disable(gl::STENCIL_TEST));

                    for (i, framebuffer) in self.framebuffers.iter().enumerate() {
                        ck!(gl.bind_framebuffer(gl::DRAW_FRAMEBUFFER, Some(*framebuffer)));

                        ck!(gl.active_texture(gl::TEXTURE0));
                        ck!(gl.bind_texture(GL_TEXTURE_EXTERNAL_OES, Some(self.surface_texture)));
                        ck!(gl.bind_sampler(0, None));
                        ck!(gl.uniform_1_i32(Some(&self.view_idx_uloc), i as i32));
                        ck!(gl.draw_arrays(gl::TRIANGLE_STRIP, 0, 4));
                    }
                },
            )
        };
    }

    /// Starts reading RGBA8 samples of the left view's staged frame (as decoded, encoded
    /// layout) on a PROBE_GRID x PROBE_GRID grid (bilinear samples at the grid cells' centres: a
    /// sparse grid, not an average), rows bottom-up, into a pixel pack buffer, with a fence.
    /// Asynchronous: nothing waits here; take_oldest_probe() collects it. Call right after
    /// render(). At most PROBE_SLOTS probes are pending: with all slots busy the oldest is
    /// collected first (blocking) and its samples are returned. Leaves the read/draw framebuffer
    /// and pixel pack buffer bindings unbound.
    pub fn issue_probe(&mut self) -> Option<Vec<[u8; 4]>> {
        let evicted = if self.probe_pending.len() >= PROBE_SLOTS {
            self.take_oldest_probe(true)
        } else {
            None
        };

        let gl = &self.context.gl_context;
        self.context.make_current();
        let grid = PROBE_GRID as i32;

        unsafe {
            let (_, probe_framebuffer) = *self.probe_target.get_or_insert_with(|| {
                let renderbuffer = ck!(gl.create_renderbuffer().unwrap());
                ck!(gl.bind_renderbuffer(gl::RENDERBUFFER, Some(renderbuffer)));
                ck!(gl.renderbuffer_storage(gl::RENDERBUFFER, gl::RGBA8, grid, grid));
                ck!(gl.bind_renderbuffer(gl::RENDERBUFFER, None));
                let framebuffer = ck!(gl.create_framebuffer().unwrap());
                ck!(gl.bind_framebuffer(gl::DRAW_FRAMEBUFFER, Some(framebuffer)));
                ck!(gl.framebuffer_renderbuffer(
                    gl::DRAW_FRAMEBUFFER,
                    gl::COLOR_ATTACHMENT0,
                    gl::RENDERBUFFER,
                    Some(renderbuffer),
                ));
                (renderbuffer, framebuffer)
            });
            let buffers = *self.probe_buffers.get_or_insert_with(|| {
                [(); PROBE_SLOTS].map(|_| {
                    let buffer = ck!(gl.create_buffer().unwrap());
                    ck!(gl.bind_buffer(gl::PIXEL_PACK_BUFFER, Some(buffer)));
                    ck!(gl.buffer_data_size(
                        gl::PIXEL_PACK_BUFFER,
                        PROBE_BYTES as i32,
                        gl::STREAM_READ
                    ));
                    ck!(gl.bind_buffer(gl::PIXEL_PACK_BUFFER, None));
                    buffer
                })
            });
            let slot = self.probe_next_slot;
            self.probe_next_slot = (slot + 1) % PROBE_SLOTS;

            ck!(gl.disable(gl::SCISSOR_TEST));
            ck!(gl.bind_framebuffer(gl::READ_FRAMEBUFFER, Some(self.framebuffers[0])));
            ck!(gl.bind_framebuffer(gl::DRAW_FRAMEBUFFER, Some(probe_framebuffer)));
            ck!(gl.blit_framebuffer(
                0,
                0,
                self.viewport_size.x,
                self.viewport_size.y,
                0,
                0,
                grid,
                grid,
                gl::COLOR_BUFFER_BIT,
                gl::LINEAR,
            ));

            ck!(gl.bind_framebuffer(gl::READ_FRAMEBUFFER, Some(probe_framebuffer)));
            ck!(gl.bind_buffer(gl::PIXEL_PACK_BUFFER, Some(buffers[slot])));
            ck!(gl.pixel_store_i32(gl::PACK_ALIGNMENT, 1));
            ck!(gl.pixel_store_i32(gl::PACK_ROW_LENGTH, 0));
            ck!(gl.read_pixels(
                0,
                0,
                grid,
                grid,
                gl::RGBA,
                gl::UNSIGNED_BYTE,
                gl::PixelPackData::BufferOffset(0),
            ));
            ck!(gl.bind_buffer(gl::PIXEL_PACK_BUFFER, None));
            ck!(gl.bind_framebuffer(gl::READ_FRAMEBUFFER, None));
            ck!(gl.bind_framebuffer(gl::DRAW_FRAMEBUFFER, None));

            let fence = ck!(gl.fence_sync(gl::SYNC_GPU_COMMANDS_COMPLETE, 0).unwrap());
            // Start the staging copy and the readback on the GPU now (the CPU goes on with the
            // stream pass)
            ck!(gl.flush());
            self.probe_pending.push_back((slot, fence));
        }

        evicted
    }

    /// Probes issued and not collected yet
    pub fn pending_probes(&self) -> usize {
        self.probe_pending.len()
    }

    /// The samples of the oldest pending probe. block = false: None while the GPU has not
    /// finished it (it stays pending); block = true: waits for it (the old synchronous cost).
    /// None when nothing is pending.
    pub fn take_oldest_probe(&mut self, block: bool) -> Option<Vec<[u8; 4]>> {
        let &(slot, fence) = self.probe_pending.front()?;
        let buffer = self.probe_buffers?[slot];
        let gl = &self.context.gl_context;
        self.context.make_current();

        unsafe {
            let signaled = gl.get_sync_status(fence) == gl::SIGNALED;
            if !signaled {
                if !block {
                    return None;
                }
                // Bounded waits; the map below waits anyway if the GPU takes longer
                for _ in 0..10 {
                    let status =
                        gl.client_wait_sync(fence, gl::SYNC_FLUSH_COMMANDS_BIT, 100_000_000);
                    if status == gl::ALREADY_SIGNALED
                        || status == gl::CONDITION_SATISFIED
                        || status == gl::WAIT_FAILED
                    {
                        break;
                    }
                }
            }
            self.probe_pending.pop_front();
            gl.delete_sync(fence);

            ck!(gl.bind_buffer(gl::PIXEL_PACK_BUFFER, Some(buffer)));
            let mapped = gl.map_buffer_range(
                gl::PIXEL_PACK_BUFFER,
                0,
                PROBE_BYTES as i32,
                gl::MAP_READ_BIT,
            );
            let samples = if mapped.is_null() {
                None
            } else {
                let bytes = std::slice::from_raw_parts(mapped, PROBE_BYTES);
                let samples = bytes
                    .chunks_exact(4)
                    .map(|pixel| [pixel[0], pixel[1], pixel[2], pixel[3]])
                    .collect::<Vec<_>>();
                gl.unmap_buffer(gl::PIXEL_PACK_BUFFER);
                Some(samples)
            };
            ck!(gl.bind_buffer(gl::PIXEL_PACK_BUFFER, None));
            samples
        }
    }
}

impl Drop for StagingRenderer {
    fn drop(&mut self) {
        let gl = &self.context.gl_context;
        self.context.make_current();

        unsafe {
            ck!(gl.delete_program(self.program));
            ck!(gl.delete_texture(self.surface_texture));
            for framebuffer in &self.framebuffers {
                ck!(gl.delete_framebuffer(*framebuffer));
            }
            for (_, fence) in self.probe_pending.drain(..) {
                gl.delete_sync(fence);
            }
            if let Some(buffers) = self.probe_buffers.take() {
                for buffer in buffers {
                    ck!(gl.delete_buffer(buffer));
                }
            }
            if let Some((renderbuffer, framebuffer)) = self.probe_target.take() {
                ck!(gl.delete_framebuffer(framebuffer));
                ck!(gl.delete_renderbuffer(renderbuffer));
            }
        }
    }
}
