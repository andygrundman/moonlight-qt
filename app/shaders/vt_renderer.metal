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
    float3x3 matrix;
    float3 offsets;
    float2 chromaOffset;
    float bitnessScaleFactor;
};

constexpr sampler s(coord::normalized, address::clamp_to_edge, filter::linear);

vertex Vertex vs_draw(constant Vertex *vertices [[ buffer(0) ]], uint id [[ vertex_id ]])
{
    return vertices[id];
}

fragment float4 ps_draw_biplanar(Vertex v [[ stage_in ]],
                                constant CscParams &cscParams [[ buffer(0) ]],
                                texture2d<float> luminancePlane [[ texture(0) ]],
                                texture2d<float> chrominancePlane [[ texture(1) ]])
{
    float2 chromaOffset = float2(cscParams.chromaOffset) / float2(luminancePlane.get_width(),
                                                                  luminancePlane.get_height());
    float3 yuv = float3(luminancePlane.sample(s, v.texCoords).r,
                      chrominancePlane.sample(s, v.texCoords + chromaOffset).rg);
    yuv *= cscParams.bitnessScaleFactor;
    yuv -= cscParams.offsets;

    return float4(yuv * cscParams.matrix, 1.0f);
}

fragment float4 ps_draw_triplanar(Vertex v [[ stage_in ]],
                                 constant CscParams &cscParams [[ buffer(0) ]],
                                 texture2d<float> luminancePlane [[ texture(0) ]],
                                 texture2d<float> chrominancePlaneU [[ texture(1) ]],
                                 texture2d<float> chrominancePlaneV [[ texture(2) ]])
{
    float2 chromaOffset = float2(cscParams.chromaOffset) / float2(luminancePlane.get_width(),
                                                                  luminancePlane.get_height());
    float3 yuv = float3(luminancePlane.sample(s, v.texCoords).r,
                      chrominancePlaneU.sample(s, v.texCoords + chromaOffset).r,
                      chrominancePlaneV.sample(s, v.texCoords + chromaOffset).r);
    yuv *= cscParams.bitnessScaleFactor;
    yuv -= cscParams.offsets;

    return float4(yuv * cscParams.matrix, 1.0f);
}

/// Linear shaders

// PQ (SMPTE ST 2084) constants for inverse EOTF
constant float PQ_C1 = 0.8359375;          // 3424/4096
constant float PQ_C2 = 18.8515625;         // 2413/128
constant float PQ_C3 = 18.6875;            // 299/16
constant float PQ_M = 78.84375;            // 2523/32
constant float PQ_N = 0.1593017578125;     // 1305/8192

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

fragment float4 ps_draw_linear(Vertex v [[ stage_in ]],
                               constant CscParams &cscParams [[ buffer(0) ]],
                               constant float &referenceWhite [[ buffer(2) ]],
                               texture2d<float> luminancePlane [[ texture(0) ]],
                               texture2d<float> chrominancePlane [[ texture(1) ]])
{
    float2 chromaOffset = float2(cscParams.chromaOffset) / float2(luminancePlane.get_width(),
                                                                  luminancePlane.get_height());
    float3 yuv = float3(luminancePlane.sample(s, v.texCoords).r,
                      chrominancePlane.sample(s, v.texCoords + chromaOffset).rg);
    yuv *= cscParams.bitnessScaleFactor;
    yuv -= cscParams.offsets;

    float3 rgb = clamp(yuv * cscParams.matrix, 0.0f, 1.0f);

    // Convert from normalized PQ signal to absolute linear nits.
    float3 linearRGB = pq_to_linear_rgb(rgb);

    // referenceWhite should match opticalOutputScale in HDR10MetadataWithDisplayInfo().
    // referenceWhite is usually 203 or 100.
    // Do not compress highlights here: CAEDRMetadata supplies system tone mapping.
    linearRGB /= max(referenceWhite, 1.0f);

    return float4(linearRGB, 1.0f);
}

fragment float4 ps_draw_linear_triplanar(Vertex v [[ stage_in ]],
                               constant CscParams &cscParams [[ buffer(0) ]],
                               constant float &referenceWhite [[ buffer(2) ]],
                               texture2d<float> luminancePlane [[ texture(0) ]],
                               texture2d<float> chrominancePlaneU [[ texture(1) ]],
                               texture2d<float> chrominancePlaneV [[ texture(2) ]])
{
    float2 chromaOffset = float2(cscParams.chromaOffset) / float2(luminancePlane.get_width(),
                                                                  luminancePlane.get_height());
    float3 yuv = float3(luminancePlane.sample(s, v.texCoords).r,
                      chrominancePlaneU.sample(s, v.texCoords + chromaOffset).r,
                      chrominancePlaneV.sample(s, v.texCoords + chromaOffset).r);
    yuv *= cscParams.bitnessScaleFactor;
    yuv -= cscParams.offsets;

    float3 rgb = clamp(yuv * cscParams.matrix, 0.0f, 1.0f);

    // Convert from normalized PQ signal to absolute linear nits.
    float3 linearRGB = pq_to_linear_rgb(rgb);

    // referenceWhite should match opticalOutputScale in HDR10MetadataWithDisplayInfo().
    // referenceWhite is usually 203 or 100.
    // Do not compress highlights here: CAEDRMetadata supplies system tone mapping.
    linearRGB /= max(referenceWhite, 1.0f);

    return float4(linearRGB, 1.0f);
}

struct OverlayParams {
    float3x3 matrix;
    float referenceWhite;
    uint outputTransfer;
};

float linear_to_pq(float nits) {
    float powered = pow(clamp(nits / 10000.0f, 0.0f, 1.0f), PQ_N);
    return pow((PQ_C1 + PQ_C2 * powered) / (1.0f + PQ_C3 * powered), PQ_M);
}

float linear_to_hlg(float linear) {
    // BT.2100 places diffuse white at HLG signal 0.75.
    float scene = max(linear, 0.0f) * 0.26496256f;
    return scene <= 1.0f / 12.0f ? sqrt(3.0f * scene) :
        0.17883277f * log(12.0f * scene - 0.28466892f) + 0.55991073f;
}

fragment float4 ps_draw_rgb(Vertex v [[ stage_in ]],
                            constant OverlayParams &params [[ buffer(4) ]],
                            texture2d<float> rgbTexture [[ texture(0) ]]) {
    float4 sample = rgbTexture.sample(s, v.texCoords);
    if (params.outputTransfer == 0) return sample;
    float3 linear = select(pow((sample.rgb + 0.055f) / 1.055f, float3(2.4f)),
                           sample.rgb / 12.92f, sample.rgb <= 0.04045f);
    linear = linear * params.matrix;
    if (params.outputTransfer == 1) {
        linear = float3(linear_to_pq(linear.r * params.referenceWhite),
                        linear_to_pq(linear.g * params.referenceWhite),
                        linear_to_pq(linear.b * params.referenceWhite));
    }
    else if (params.outputTransfer == 3) {
        linear = float3(linear_to_hlg(linear.r), linear_to_hlg(linear.g), linear_to_hlg(linear.b));
    }
    return float4(linear, sample.a);
}
