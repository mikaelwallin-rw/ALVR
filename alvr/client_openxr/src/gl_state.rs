// Raw GL code in the client shares the context with wgpu's GLES backend, which caches GL state and
// assumes nobody else changes it. In particular wgpu-hal binds ONE vertex array object when the
// device opens and never binds it again: unbinding it (or leaving another one bound) breaks later
// wgpu uploads and draws (a SIGSEGV in the driver's memcpy under wgpu's queue writes was seen when
// the mipmapped video path bound VAO 0). Every raw GL block must leave the state as it found it:
// take a GlStateGuard before touching anything; dropping it restores the saved state.

use alvr_graphics::GL_TEXTURE_EXTERNAL_OES;
use glow::{self as gl, HasContext};
use std::num::NonZeroU32;

const GL_TEXTURE_BINDING_EXTERNAL_OES: u32 = 0x8D67;
const CAPS: [u32; 5] = [
    gl::SCISSOR_TEST,
    gl::STENCIL_TEST,
    gl::DEPTH_TEST,
    gl::BLEND,
    gl::CULL_FACE,
];

fn name(value: i32) -> Option<NonZeroU32> {
    NonZeroU32::new(value as u32)
}

pub struct GlStateGuard<'a> {
    gl: &'a gl::Context,
    vertex_array: i32,
    program: i32,
    draw_framebuffer: i32,
    read_framebuffer: i32,
    array_buffer: i32,
    pixel_unpack_buffer: i32,
    active_texture: i32,
    // Bindings of texture unit 0 (the unit raw code uses)
    texture_2d: i32,
    texture_external: i32,
    sampler: i32,
    viewport: [i32; 4],
    color_mask: [i32; 4],
    caps: [bool; CAPS.len()],
    unpack: [(u32, i32); 4],
}

impl<'a> GlStateGuard<'a> {
    pub fn save(gl: &'a gl::Context) -> Self {
        unsafe {
            let active_texture = gl.get_parameter_i32(gl::ACTIVE_TEXTURE);
            gl.active_texture(gl::TEXTURE0);
            let texture_2d = gl.get_parameter_i32(gl::TEXTURE_BINDING_2D);
            let texture_external = gl.get_parameter_i32(GL_TEXTURE_BINDING_EXTERNAL_OES);
            let sampler = gl.get_parameter_i32(gl::SAMPLER_BINDING);
            gl.active_texture(active_texture as u32);

            let mut viewport = [0; 4];
            gl.get_parameter_i32_slice(gl::VIEWPORT, &mut viewport);
            let mut color_mask = [1; 4];
            gl.get_parameter_i32_slice(gl::COLOR_WRITEMASK, &mut color_mask);

            Self {
                gl,
                vertex_array: gl.get_parameter_i32(gl::VERTEX_ARRAY_BINDING),
                program: gl.get_parameter_i32(gl::CURRENT_PROGRAM),
                draw_framebuffer: gl.get_parameter_i32(gl::DRAW_FRAMEBUFFER_BINDING),
                read_framebuffer: gl.get_parameter_i32(gl::READ_FRAMEBUFFER_BINDING),
                array_buffer: gl.get_parameter_i32(gl::ARRAY_BUFFER_BINDING),
                pixel_unpack_buffer: gl.get_parameter_i32(gl::PIXEL_UNPACK_BUFFER_BINDING),
                active_texture,
                texture_2d,
                texture_external,
                sampler,
                viewport,
                color_mask,
                caps: CAPS.map(|cap| gl.is_enabled(cap)),
                unpack: [
                    gl::UNPACK_ALIGNMENT,
                    gl::UNPACK_ROW_LENGTH,
                    gl::UNPACK_SKIP_ROWS,
                    gl::UNPACK_SKIP_PIXELS,
                ]
                .map(|param| (param, gl.get_parameter_i32(param))),
            }
        }
    }
}

impl Drop for GlStateGuard<'_> {
    fn drop(&mut self) {
        let gl = self.gl;
        unsafe {
            gl.bind_vertex_array(name(self.vertex_array).map(gl::NativeVertexArray));
            gl.use_program(name(self.program).map(gl::NativeProgram));
            gl.bind_framebuffer(
                gl::DRAW_FRAMEBUFFER,
                name(self.draw_framebuffer).map(gl::NativeFramebuffer),
            );
            gl.bind_framebuffer(
                gl::READ_FRAMEBUFFER,
                name(self.read_framebuffer).map(gl::NativeFramebuffer),
            );
            gl.bind_buffer(
                gl::ARRAY_BUFFER,
                name(self.array_buffer).map(gl::NativeBuffer),
            );
            gl.bind_buffer(
                gl::PIXEL_UNPACK_BUFFER,
                name(self.pixel_unpack_buffer).map(gl::NativeBuffer),
            );

            gl.active_texture(gl::TEXTURE0);
            gl.bind_texture(gl::TEXTURE_2D, name(self.texture_2d).map(gl::NativeTexture));
            gl.bind_texture(
                GL_TEXTURE_EXTERNAL_OES,
                name(self.texture_external).map(gl::NativeTexture),
            );
            gl.bind_sampler(0, name(self.sampler).map(gl::NativeSampler));
            gl.active_texture(self.active_texture as u32);

            let [x, y, w, h] = self.viewport;
            gl.viewport(x, y, w, h);
            let [r, g, b, a] = self.color_mask.map(|v| v != 0);
            gl.color_mask(r, g, b, a);
            for (cap, enabled) in CAPS.iter().zip(self.caps) {
                if enabled {
                    gl.enable(*cap);
                } else {
                    gl.disable(*cap);
                }
            }
            for (param, value) in self.unpack {
                gl.pixel_store_i32(param, value);
            }
        }
    }
}
