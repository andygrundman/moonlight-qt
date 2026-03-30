#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct Vertex
{
    float4 position [[ position ]];
    float2 texCoords;
};

struct CscParams
{
    half3x3 matrix;
    half3 offsets;
    half2 chromaOffset;
    half bitnessScaleFactor;
};

constexpr sampler s(coord::normalized, address::clamp_to_edge, filter::linear);

vertex Vertex vs_draw(constant Vertex *vertices [[ buffer(0) ]], uint id [[ vertex_id ]])
{
    return vertices[id];
}

fragment half4 ps_draw_biplanar(Vertex v [[ stage_in ]],
                                constant CscParams &cscParams [[ buffer(0) ]],
                                texture2d<half> luminancePlane [[ texture(0) ]],
                                texture2d<half> chrominancePlane [[ texture(1) ]])
{
    float2 chromaOffset = float2(cscParams.chromaOffset) / float2(luminancePlane.get_width(),
                                                                  luminancePlane.get_height());
    half3 yuv = half3(luminancePlane.sample(s, v.texCoords).r,
                      chrominancePlane.sample(s, v.texCoords + chromaOffset).rg);
    yuv *= cscParams.bitnessScaleFactor;
    yuv -= cscParams.offsets;

    return half4(yuv * cscParams.matrix, 1.0h);
}

fragment half4 ps_draw_triplanar(Vertex v [[ stage_in ]],
                                 constant CscParams &cscParams [[ buffer(0) ]],
                                 texture2d<half> luminancePlane [[ texture(0) ]],
                                 texture2d<half> chrominancePlaneU [[ texture(1) ]],
                                 texture2d<half> chrominancePlaneV [[ texture(2) ]])
{
    float2 chromaOffset = float2(cscParams.chromaOffset) / float2(luminancePlane.get_width(),
                                                                  luminancePlane.get_height());
    half3 yuv = half3(luminancePlane.sample(s, v.texCoords).r,
                      chrominancePlaneU.sample(s, v.texCoords + chromaOffset).r,
                      chrominancePlaneV.sample(s, v.texCoords + chromaOffset).r);
    yuv *= cscParams.bitnessScaleFactor;
    yuv -= cscParams.offsets;

    return half4(yuv * cscParams.matrix, 1.0h);
}

fragment half4 ps_draw_rgb(Vertex v [[ stage_in ]],
                           texture2d<half> rgbTexture [[ texture(0) ]])
{
    return rgbTexture.sample(s, v.texCoords);
}

/// Linear shaders

// PQ (SMPTE ST 2084) constants for inverse EOTF
constant float PQ_C1 = 0.8359375;          // 3424/4096
constant float PQ_C2 = 18.8515625;         // 2413/128
constant float PQ_C3 = 18.6875;            // 299/16
constant float PQ_M = 78.84375;            // 2523/32
constant float PQ_N = 0.1593017578125;     // 1305/8192

// BT.2020 to Rec.709/sRGB color space conversion matrix
constant float3x3 bt2020_to_rec709 = float3x3(
    float3( 1.7166511, -0.3556708, -0.2533663),
    float3(-0.6666844,  1.6164812,  0.0157685),
    float3( 0.0176399, -0.0427706,  0.9421031)
);

// Convert from PQ curve to linear light
float pq_to_linear(float pq) {
    if (pq <= 0.0) return 0.0;

    float pq_pow_inv_m = pow(pq, 1.0f / PQ_M);
    float numerator = max(pq_pow_inv_m - PQ_C1, 0.0f);
    float denominator = PQ_C2 - PQ_C3 * pq_pow_inv_m;

    if (denominator <= 0.0f) return 0.0f;

    return 10000.0f * pow(numerator / denominator, 1.0f / PQ_N);
}

// Apply PQ inverse EOTF to RGB components
float3 pq_to_linear_rgb(float3 pq_rgb) {
    return float3(
        pq_to_linear(pq_rgb.r),
        pq_to_linear(pq_rgb.g),
        pq_to_linear(pq_rgb.b)
    );
}

fragment half4 ps_draw_linear(Vertex v [[ stage_in ]],
                               constant CscParams &cscParams [[ buffer(0) ]],
                               texture2d<half> luminancePlane [[ texture(0) ]],
                               texture2d<half> chrominancePlane [[ texture(1) ]])
{
    float2 chromaOffset = float2(cscParams.chromaOffset) / float2(luminancePlane.get_width(),
                                                                  luminancePlane.get_height());
    half3 yuv = half3(luminancePlane.sample(s, v.texCoords).r,
                      chrominancePlane.sample(s, v.texCoords + chromaOffset).rg);
    yuv *= cscParams.bitnessScaleFactor;
    yuv -= cscParams.offsets;

    half3 rgbHalf = yuv * cscParams.matrix;

    // PQ EOTF should be done in float precision.
    float3 rgb = clamp(float3(rgbHalf), 0.0f, 1.0f);

    // Convert from normalized PQ signal to absolute linear nits.
    float3 linearRGB = pq_to_linear_rgb(rgb);

    // Convert nits to EDR linear values.
    // This should match opticalOutputScale in HDR10MetadataWithDisplayInfo().
    constexpr float referenceWhite = 203.0f;
    linearRGB = linearRGB / referenceWhite;

    return half4(half3(linearRGB), 1.0h);
}
