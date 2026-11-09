// Hand mesh (Pico SM_Hand), skinned on the CPU with the XR_EXT_hand_tracking joints: vertices
// arrive in the STAGE space. Output is premultiplied alpha for an alpha-blended OpenXR projection
// layer.

// Pico's MI_Hand look: dark body, light fresnel rim (linear colors)
const INNER_COLOR = vec3f(0.1311, 0.1543, 0.2448);
const RIM_COLOR = vec3f(0.7101, 0.7946, 1.0);
const RIM_STRENGTH: f32 = 0.85;
const RIM_EXPONENT: f32 = 2.5;
const ALPHA: f32 = 1.0;

struct PushConstant {
    view_proj: mat4x4f,
    eye_position: vec4f,
}
var<push_constant> pc: PushConstant;

struct VertexInput {
    @location(0) stage_position: vec3f,
    @location(1) normal: vec3f,
}

struct VertexOutput {
    @builtin(position) clip_position: vec4f,
    @location(0) stage_position: vec3f,
    @location(1) normal: vec3f,
}

@vertex
fn vertex_main(input: VertexInput) -> VertexOutput {
    var output: VertexOutput;
    output.clip_position = pc.view_proj * vec4f(input.stage_position, 1.0);
    output.stage_position = input.stage_position;
    output.normal = input.normal;
    return output;
}

@fragment
fn fragment_main(input: VertexOutput) -> @location(0) vec4f {
    let normal = normalize(input.normal);
    let to_eye = normalize(pc.eye_position.xyz - input.stage_position);
    let fresnel = pow(1.0 - abs(dot(normal, to_eye)), RIM_EXPONENT);
    // Linear output: the target is either an sRGB swapchain (encoded by the hardware) or a UNORM
    // swapchain, which OpenXR compositors treat as linear
    let color = mix(INNER_COLOR, RIM_COLOR, fresnel * RIM_STRENGTH);

    return vec4f(color * ALPHA, ALPHA);
}
