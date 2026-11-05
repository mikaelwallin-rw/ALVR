// Rasterizes the opaque region classes (see opaque_regions.rs) into an R8 texture with the same
// size and transform as the stream pass, which then reads its own pixel. Classes are drawn in
// increasing priority order, later draws overwrite.

struct PushConstant {
    reprojection_transform: mat4x4f,
}
var<push_constant> pc: PushConstant;

struct VertexOutput {
    @builtin(position) position: vec4f,
    @location(0) @interpolate(flat) region_class: f32,
}

@vertex
fn vertex_main(@location(0) position: vec4f, @location(1) region_class: f32) -> VertexOutput {
    var result: VertexOutput;
    // `position` is the homogeneous stream quad position (uv.x - 0.5, 0.5 - uv.y, 0, 1) * w, see
    // vertex_main in stream.wgsl
    result.position = pc.reprojection_transform * position;
    result.region_class = region_class;
    return result;
}

@fragment
fn fragment_main(@location(0) @interpolate(flat) region_class: f32) -> @location(0) vec4f {
    return vec4f(region_class / 255.0, 0.0, 0.0, 0.0);
}
