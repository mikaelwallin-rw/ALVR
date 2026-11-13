struct Immediate {
    reprojection_transform: mat4x4f,
    view_idx: u32,
    passthrough_mode: u32, // 0: Blend, 1: RGB chroma key, 2: HSV chroma key
    blend_alpha: f32,
    // Chroma key edge cleanup: bits 0..8 = erosion radius (texels), bits 8..16 = despill (0..=255)
    ck_edge: u32,
    ck_channel0: vec4f,
    ck_channel1: vec4f,
    ck_channel2: vec4f,
}
var<immediate> imm: Immediate;

const DIV12: f32 = 1.0 / 12.92;
const DIV1: f32 = 1.0 / 1.055;
const THRESHOLD: f32 = 0.04045;
const GAMMA: vec3f = vec3f(2.4);
const LUMA: vec3f = vec3f(0.2126, 0.7152, 0.0722);
const INV_SQRT_2: f32 = 0.70710678;

// Opaque region classes: stencil values written by opaque_regions.rs. The stream quad is drawn
// once per class (equal stencil test), with FORCED_REGION_CLASS set to that class.
const REGION_NONE: u32 = 0u;
const REGION_BAND: u32 = 1u; // outside the quad edge, within the band: forced keyed
const REGION_STRIP: u32 = 2u; // inside the quad edge, within the edge key margin: forced keyed
const REGION_DESPILL: u32 = 3u; // inside the strip, within the despill margin: forced opaque, despilled
const REGION_INNER: u32 = 4u; // the rest of the quad: forced opaque
const OPAQUE_REGIONS_FLAG_DEBUG_TINT: u32 = 1u;
const DEBUG_TINT_AMOUNT: f32 = 0.5;

override ENABLE_SRGB_CORRECTION: bool;
override ENCODING_GAMMA: f32;

// Pipeline variants: when a feature is off its code is compiled out entirely (dead code still costs
// registers/occupancy on mobile GPUs even if the branch is never taken)
override ENABLE_EDGE_CLEANUP: bool = false;
// REGION_NONE: normal keying. Otherwise this pipeline only draws pixels of that class.
override FORCED_REGION_CLASS: u32 = 0u;

override ENABLE_UPSCALING: bool = false;
override UPSCALE_USE_EDGE_DIRECTION: bool = true;
override UPSCALE_EDGE_THRESHOLD: f32 = 4.0/255.0;
override UPSCALE_EDGE_SHARPNESS: f32 = 2.0;

override ENABLE_FFE: bool = false;

struct OpaqueRegions {
    flags: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
    // Normalized chroma of the key colour
    key_chroma_dir: vec4f,
}

@group(0) @binding(0) var stream_texture: texture_2d<f32>;
@group(0) @binding(1) var stream_sampler: sampler;
struct FoveationEyeUniforms {
    c1_c2: vec4f,
    lo_hi_bound: vec4f,
    a_left_b_left: vec4f,
    a_right_b_right: vec4f,
    c_right: vec4f,
}
struct FoveationUniforms {
    view_ratio_edge_ratio: vec4f,
    eyes: array<FoveationEyeUniforms, 2>,
}
@group(0) @binding(2) var<uniform> foveation: FoveationUniforms;
@group(0) @binding(3) var<uniform> opaque: OpaqueRegions;

struct VertexOutput {
    @builtin(position) position: vec4f,
    @location(0) uv: vec2f,
}

@vertex
fn vertex_main(@builtin(vertex_index) vertex_index: u32) -> VertexOutput {
    var result: VertexOutput;

    result.uv = vec2f(f32(vertex_index & 1), f32(vertex_index >> 1));
    result.position = imm.reprojection_transform * vec4f(result.uv.x - 0.5, 0.5 - result.uv.y, 0.0, 1.0);

    return result;
}

@fragment
fn fragment_main(@location(0) uv: vec2f) -> @location(0) vec4f {
    var corrected_uv = uv;
    // tell upscaler to target a lower resolution for the edges
    var upscale_source_resolution = 1.0;
    if ENABLE_FFE {
        let view_size_ratio = foveation.view_ratio_edge_ratio.xy;
        let edge_ratio = foveation.view_ratio_edge_ratio.zw;
        let eye = foveation.eyes[imm.view_idx];

        let c1 = eye.c1_c2.xy;
        let c2 = eye.c1_c2.zw;
        let lo_bound = eye.lo_hi_bound.xy;
        let hi_bound = eye.lo_hi_bound.zw;

        let a_left = eye.a_left_b_left.xy;
        let b_left = eye.a_left_b_left.zw;

        let a_right = eye.a_right_b_right.xy;
        let b_right = eye.a_right_b_right.zw;
        let c_right = eye.c_right.xy;

        if imm.view_idx == 1 {
            corrected_uv.x = 1.0 - corrected_uv.x;
        }

        let center = (corrected_uv - c1) * edge_ratio / c2;
        let left_edge = (-b_left + sqrt(b_left * b_left + 4.0 * a_left * corrected_uv)) / (2.0 * a_left);
        let right_edge = (-b_right + sqrt(b_right * b_right - 4.0 * (c_right - a_right * corrected_uv))) / (2.0 * a_right);

        if corrected_uv.x < lo_bound.x {
            corrected_uv.x = left_edge.x;
            upscale_source_resolution = upscale_source_resolution * edge_ratio.x;
        } else if corrected_uv.x > hi_bound.x {
            corrected_uv.x = right_edge.x;
            upscale_source_resolution = upscale_source_resolution * edge_ratio.x;
        } else {
            corrected_uv.x = center.x;
        }

        if corrected_uv.y < lo_bound.y {
            corrected_uv.y = left_edge.y;
            upscale_source_resolution = upscale_source_resolution * edge_ratio.y;
        } else if corrected_uv.y > hi_bound.y {
            corrected_uv.y = right_edge.y;
            upscale_source_resolution = upscale_source_resolution * edge_ratio.y;
        } else {
            corrected_uv.y = center.y;
        }

        corrected_uv = corrected_uv * view_size_ratio;

        if imm.view_idx == 1 {
            corrected_uv.x = 1.0 - corrected_uv.x;
        }
    }

    var color: vec3f;
    if ENABLE_UPSCALING {
        color = sgsr(vec4f(corrected_uv.x, corrected_uv.y, 0.0, 0.0), upscale_source_resolution).xyz;
    } else {
        color = textureSample(stream_texture, stream_sampler, corrected_uv).rgb;
    }

    color = decode_color(color);

    var alpha = imm.blend_alpha; // Default to Blend passthrough mode
    if imm.passthrough_mode != 0 { // Chroma key
        if FORCED_REGION_CLASS != REGION_NONE {
            return forced_region_output(color);
        }

        var mask = key_mask(color);

        var erosion_px = 0.0;
        var despill = 0.0;
        var edge_cleanup = false;
        if ENABLE_EDGE_CLEANUP {
            erosion_px = f32(imm.ck_edge & 0xffu);
            despill = f32((imm.ck_edge >> 8u) & 0xffu) / 255.0;
            // Fully keyed pixels (most of the background) need no edge cleanup: skip the taps.
            edge_cleanup = mask > 0.0 && (erosion_px > 0.0 || despill > 0.0);
        }

        if ENABLE_EDGE_CLEANUP && edge_cleanup {
            let texel = 1.0 / vec2f(textureDimensions(stream_texture));

            // Erode the opaque area: the keyed region eats the compression fringe ring.
            if erosion_px > 0.0 {
                mask = min(mask, ring_min_mask(corrected_uv, texel, erosion_px, true));
            }

            // Despill only next to keyed (or partially keyed) pixels. Look one texel beyond the
            // erosion ring, so the pixels that survived erosion still see the keyed region.
            // Key-coloured content far from keyed areas is left untouched.
            if despill > 0.0 && mask > 0.0 {
                let near_key = 1.0 - min(mask, ring_min_mask(corrected_uv, texel, erosion_px + 1.0, false));
                color = remove_key_spill(color, despill * near_key);
            }
        }

        // Note: because of this calculation, we require premultiplied alpha option in the XR layer
        color = max(color * mask, vec3f(0.0));
        alpha = mask;
    }

    return vec4f(color, alpha);
}

// Output of a pixel inside an opaque region class (premultiplied alpha). The edge strip and band
// are keyed even for plain opaque pixels, which carry the filtered/compressed key colour.
fn forced_region_output(color: vec3f) -> vec4f {
    if (opaque.flags & OPAQUE_REGIONS_FLAG_DEBUG_TINT) != 0u {
        return vec4f(mix(color, region_debug_tint(), DEBUG_TINT_AMOUNT), 1.0);
    }
    if FORCED_REGION_CLASS == REGION_INNER {
        return vec4f(color, 1.0);
    }
    if FORCED_REGION_CLASS == REGION_DESPILL {
        return vec4f(remove_region_key_spill(color), 1.0);
    }
    return vec4f(0.0); // BAND, STRIP
}

fn region_debug_tint() -> vec3f {
    if FORCED_REGION_CLASS == REGION_INNER {
        return vec3f(0.0, 1.0, 0.0);
    }
    if FORCED_REGION_CLASS == REGION_DESPILL {
        return vec3f(1.0, 1.0, 0.0);
    }
    if FORCED_REGION_CLASS == REGION_STRIP {
        return vec3f(1.0, 0.0, 0.0);
    }
    return vec3f(0.0, 0.0, 1.0);
}

// Remove the chroma component pointing toward the key chroma (precomputed direction). Luma is kept.
fn remove_region_key_spill(color: vec3f) -> vec3f {
    let dir = opaque.key_chroma_dir.xyz;
    let spill = max(dot(color - vec3f(dot(color, LUMA)), dir), 0.0);
    return max(color - dir * spill, vec3f(0.0));
}

fn decode_color(color_in: vec3f) -> vec3f {
    var color = color_in;

    if ENABLE_SRGB_CORRECTION {
        let condition = vec3f(f32(color.r < THRESHOLD), f32(color.g < THRESHOLD), f32(color.b < THRESHOLD));
        let lowValues = color * DIV12;
        let highValues = pow((color + vec3f(0.055)) * DIV1, GAMMA);
        color = condition * lowValues + (1.0 - condition) * highValues;
    }

    if ENCODING_GAMMA != 0.0 {
        let enc_condition = vec3f(f32(color.r < 0.0), f32(color.g < 0.0), f32(color.b < 0.0));
        let enc_lowValues = color;
        let enc_highValues = pow(color, vec3f(ENCODING_GAMMA));
        color = enc_condition * enc_lowValues + (1.0 - enc_condition) * enc_highValues;
    }

    return color;
}

// 1 = opaque, 0 = keyed
fn key_mask(color: vec3f) -> f32 {
    var current = color;
    if imm.passthrough_mode == 2 { // HSV mode
        current = rgb_to_hsv(color);
    }
    return chroma_key_mask(current);
}

fn sample_key_mask(uv: vec2f) -> f32 {
    // Explicit LOD: this is called from non-uniform control flow
    return key_mask(decode_color(textureSampleLevel(stream_texture, stream_sampler, uv, 0.0).rgb));
}

// Minimum mask over a ring of 4 axial taps at `radius` texels, plus optionally 4 diagonal taps
fn ring_min_mask(uv: vec2f, texel: vec2f, radius: f32, with_diagonals: bool) -> f32 {
    let a = texel * radius;
    var result = min(
        min(sample_key_mask(uv + vec2f(a.x, 0.0)), sample_key_mask(uv - vec2f(a.x, 0.0))),
        min(sample_key_mask(uv + vec2f(0.0, a.y)), sample_key_mask(uv - vec2f(0.0, a.y))),
    );

    if with_diagonals {
        let d = texel * max(1.0, round(radius * INV_SQRT_2));
        result = min(result, min(
            min(sample_key_mask(uv + d), sample_key_mask(uv - d)),
            min(sample_key_mask(uv + vec2f(d.x, -d.y)), sample_key_mask(uv + vec2f(-d.x, d.y))),
        ));
    }

    return result;
}

// Representative key colour, in the same space as the decoded stream colour
fn key_color() -> vec3f {
    if imm.passthrough_mode == 2 {
        // Middle of the fully keyed hue range [start_min, end_min], at full saturation and value
        let start_min = imm.ck_channel0.y;
        let end_min = imm.ck_channel0.z;
        let hue = fract(start_min + fract(end_min - start_min) * 0.5);
        return clamp(abs(fract(vec3f(hue) + vec3f(1.0, 2.0 / 3.0, 1.0 / 3.0)) * 6.0 - 3.0) - 1.0, vec3f(0.0), vec3f(1.0));
    }

    // RGB mode: each channel range is symmetric around the key value
    return 0.5 * vec3f(
        imm.ck_channel0.x + imm.ck_channel0.w,
        imm.ck_channel1.x + imm.ck_channel1.w,
        imm.ck_channel2.x + imm.ck_channel2.w,
    );
}

// Remove the component of the pixel chroma that points towards the key chroma. Luma is preserved.
fn remove_key_spill(color: vec3f, amount: f32) -> vec3f {
    let key = key_color();
    let key_chroma = key - vec3f(dot(key, LUMA));
    let key_chroma_len = length(key_chroma);
    if amount <= 0.0 || key_chroma_len < 1e-4 { // grey key: no direction to remove
        return color;
    }

    let dir = key_chroma / key_chroma_len;
    let spill = max(dot(color - vec3f(dot(color, LUMA)), dir), 0.0);

    return max(color - dir * (spill * amount), vec3f(0.0));
}

fn chroma_key_mask(color: vec3f) -> f32 {
    let start_max = vec3f(imm.ck_channel0.x, imm.ck_channel1.x, imm.ck_channel2.x);
    let start_min = vec3f(imm.ck_channel0.y, imm.ck_channel1.y, imm.ck_channel2.y);
    let end_min = vec3f(imm.ck_channel0.z, imm.ck_channel1.z, imm.ck_channel2.z);
    let end_max = vec3f(imm.ck_channel0.w, imm.ck_channel1.w, imm.ck_channel2.w);

    let start_mask = smoothstep(start_min.yz, start_max.yz, color.yz);
    let end_mask = smoothstep(end_min.yz, end_max.yz, color.yz);
    let sv_mask = max(start_mask, end_mask);
    // create t1 < t2 < t3, based on the rotated hue starting with start_max.x representing 0.
    let t3 = end_max.x - start_max.x;
    // check hue range
    if t3 > 1.0 {
        return max(sv_mask.x, sv_mask.y);
    }
    let x = fract(color.x - start_max.x);
    var t1 = fract(start_min.x - start_max.x);
    var t2 = fract(end_min.x - start_max.x);

    return max(max(smoothstep(t1, 0.0, x), smoothstep(t2, t3, x)), max(sv_mask.x, sv_mask.y));
}

fn rgb_to_hsv(rgb: vec3f) -> vec3f {
    let cmax = max(rgb.r, max(rgb.g, rgb.b));
    let cmin = min(rgb.r, min(rgb.g, rgb.b));
    let delta = cmax - cmin;

    var h = 0.0;
    var s = 0.0;
    let v = cmax;

    if cmax > cmin {
        s = delta / cmax;

        if rgb.r == cmax {
            h = (rgb.g - rgb.b) / delta;
        } else if rgb.g == cmax {
            h = 2.0 + (rgb.b - rgb.r) / delta;
        } else {
            h = 4.0 + (rgb.r - rgb.g) / delta;
        }
        h = fract(h / 6.0);
    }

    return vec3f(h, s, v);
}

//============================================================================================================
//
//
//                  Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
//                              SPDX-License-Identifier: BSD-3-Clause
//
//============================================================================================================

fn fastLanczos2(x: f32) -> f32
{
    var wA: f32 = x - 4.0;
    let wB: f32 = x * wA - wA;
    wA *= wA;
    return wB * wA;
}

fn weightY(dx: f32, dy: f32, c: f32, data: vec3f) -> vec2f {
    let stdA: f32 = data.x;
    let dir: vec2f = data.yz;
    let edgeDis: f32 = ((dx * dir.y) + (dy * dir.x));
    let x: f32 = (((dx * dx) + (dy * dy)) + ((edgeDis * edgeDis) * ((clamp(((c * c) * stdA), 0.0, 1.0) * 0.7) + -1.0)));
    let w: f32 = fastLanczos2(x);
    return vec2f(w, w * c);
}

fn weightYned(dx: f32, dy: f32, c: f32, data: f32) -> vec2f {
    let stdA: f32 = data;
    let x: f32 = ((dx * dx) + (dy * dy)) * 0.55 + clamp(abs(c) * stdA, 0.0, 1.0);
    let w: f32 = fastLanczos2(x);
    return vec2f(w, w * c);
}

fn edgeDirection(left: vec4f, right: vec4f) -> vec2f
{
    var dir: vec2f;
    let RxLz: f32 = (right.x + (-left.z));
    let RwLy: f32 = (right.w + (-left.y));
    var delta: vec2f;
    delta.x = (RxLz + RwLy);
    delta.y = (RxLz + (-RwLy));
    let lengthInv: f32 = inverseSqrt((delta.x * delta.x + 3.075740e-05) + (delta.y * delta.y));
    dir.x = (delta.x * lengthInv);
    dir.y = (delta.y * lengthInv);
    return dir;
}

fn sgsr(in_TEXCOORD0: vec4f, source_resolution_multiplier: f32) -> vec4f {
    // https://github.com/SnapdragonStudios/snapdragon-gsr/issues/2
    let dim = vec2f(textureDimensions(stream_texture)) * source_resolution_multiplier;
    let viewport_info = vec4f(1/dim.x, 1/dim.y, dim.x, dim.y);

    var color: vec4f;
    let texSample = textureSampleLevel(stream_texture, stream_sampler, in_TEXCOORD0.xy, 0.0);
    color.x = texSample.x;
    color.y = texSample.y;
    color.z = texSample.z;

    // all of these 1 values are the OperationMode
    // see https://github.com/SnapdragonStudios/snapdragon-gsr/tree/main/sgsr/v1#operation-mode
    let imgCoord: vec2f = (in_TEXCOORD0.xy * viewport_info.zw) + vec2f(-0.5, 0.5);
    let imgCoordPixel: vec2f = floor(imgCoord);
    var coord: vec2f = (imgCoordPixel * viewport_info.xy);
    let pl: vec2f = (imgCoord + (-imgCoordPixel));
    var left: vec4f = textureGather(1, stream_texture, stream_sampler, coord);

    let edgeVote: f32 = abs(left.z - left.y) + abs(color[1] - left.y) + abs(color[1] - left.z);
    if edgeVote > UPSCALE_EDGE_THRESHOLD {
        coord.x += viewport_info.x;

        var right: vec4f = textureGather(1, stream_texture, stream_sampler, coord + vec2f(viewport_info.x, 0.0));
        var upDown: vec4f;
        let texGatherA = textureGather(1, stream_texture, stream_sampler, coord + vec2f(0.0, -viewport_info.y));
        upDown.x = texGatherA.w;
        upDown.y = texGatherA.z;
        let texGatherB = textureGather(1, stream_texture, stream_sampler, coord + vec2f(0.0, viewport_info.y));
        upDown.z = texGatherB.y;
        upDown.w = texGatherB.x;

        let mean: f32 = (left.y + left.z + right.x + right.w) * 0.25;
        left = left - vec4(mean);
        right = right - vec4(mean);
        upDown = upDown - vec4(mean);
        color.w = color[1] - mean;

        let sum: f32 = (((((abs(left.x) + abs(left.y)) + abs(left.z)) + abs(left.w)) + (((abs(right.x) + abs(right.y)) + abs(right.z)) + abs(right.w))) + (((abs(upDown.x) + abs(upDown.y)) + abs(upDown.z)) + abs(upDown.w)));
        let sumMean: f32 = 1.014185e+01 / sum;
        let stdA: f32 = (sumMean * sumMean);

        var aWY: vec2f;
        if UPSCALE_USE_EDGE_DIRECTION {
            let data = vec3f(stdA, edgeDirection(left, right));
            aWY = weightY(pl.x, pl.y + 1.0, upDown.x, data);
            aWY += weightY(pl.x - 1.0, pl.y + 1.0, upDown.y, data);
            aWY += weightY(pl.x - 1.0, pl.y - 2.0, upDown.z, data);
            aWY += weightY(pl.x, pl.y - 2.0, upDown.w, data);
            aWY += weightY(pl.x + 1.0, pl.y - 1.0, left.x, data);
            aWY += weightY(pl.x, pl.y - 1.0, left.y, data);
            aWY += weightY(pl.x, pl.y, left.z, data);
            aWY += weightY(pl.x + 1.0, pl.y, left.w, data);
            aWY += weightY(pl.x - 1.0, pl.y - 1.0, right.x, data);
            aWY += weightY(pl.x - 2.0, pl.y - 1.0, right.y, data);
            aWY += weightY(pl.x - 2.0, pl.y, right.z, data);
            aWY += weightY(pl.x - 1.0, pl.y, right.w, data);
        } else {
            let data: f32 = stdA;
            aWY = weightYned(pl.x, pl.y + 1.0, upDown.x, data);
            aWY += weightYned(pl.x - 1.0, pl.y + 1.0, upDown.y, data);
            aWY += weightYned(pl.x - 1.0, pl.y - 2.0, upDown.z, data);
            aWY += weightYned(pl.x, pl.y - 2.0, upDown.w, data);
            aWY += weightYned(pl.x + 1.0, pl.y - 1.0, left.x, data);
            aWY += weightYned(pl.x, pl.y - 1.0, left.y, data);
            aWY += weightYned(pl.x, pl.y, left.z, data);
            aWY += weightYned(pl.x + 1.0, pl.y, left.w, data);
            aWY += weightYned(pl.x - 1.0, pl.y - 1.0, right.x, data);
            aWY += weightYned(pl.x - 2.0, pl.y - 1.0, right.y, data);
            aWY += weightYned(pl.x - 2.0, pl.y, right.z, data);
            aWY += weightYned(pl.x - 1.0, pl.y, right.w, data);
        }

        let finalY: f32 = aWY.y / aWY.x;
        let maxY: f32 = max(max(left.y, left.z), max(right.x, right.w));
        let minY: f32 = min(min(left.y, left.z), min(right.x, right.w));
        var deltaY: f32 = clamp(UPSCALE_EDGE_SHARPNESS * finalY, minY, maxY) - color.w;

        //smooth high contrast input
        deltaY = clamp(deltaY, -23.0 / 255.0, 23.0 / 255.0);

        color.x = clamp((color.x + deltaY), 0.0, 1.0);
        color.y = clamp((color.y + deltaY), 0.0, 1.0);
        color.z = clamp((color.z + deltaY), 0.0, 1.0);
    }

    color.w = 1.0; //assume alpha channel is not used

    return color;
}
