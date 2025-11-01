using namespace metal;

struct Vertex
{
    float4 position [[ position ]];
    float2 texCoords;
};

enum TransferFunction : uint {
    TF_NonLinear = 0,
    TF_Linear = 1
};

struct CscParams
{
    float3 matrix[3];
    float3 offsets;
    float bitnessScaleFactor;
    uint transferFunction;
    float opticalOutputScale;
};

constexpr sampler s(coord::normalized, address::clamp_to_edge, filter::linear);

// Transfer Functions

inline float pq_eotf(float N) {
    N = clamp(N, 0.0f, 1.0f);

    const float c1 = 0.8359375f;        // 3424/4096
    const float c2 = 18.8515625f;       // 2413/128
    const float c3 = 18.6875f;          // 2392/128
    const float m1 = 0.1593017578125f;  // 2610/16384
    const float m2 = 78.84375f;         // 2523/32

    float Np = pow(N, 1.0f / m2);
    float num = max(Np - c1, 0.0f);
    float den = max(c2 - c3 * Np, 1e-6f);
    float L  = 10000.0f * pow(num / den, 1.0f / m1);
    return L;
}

// Apply the selected transfer function to non-linear RGB
inline float3 handleTransferFunction(float3 rgbNonLinear, constant CscParams &cscParams) {
    switch ((TransferFunction)cscParams.transferFunction) {
        case TF_Linear: {
            float3 L = float3(pq_eotf(rgbNonLinear.r),
                              pq_eotf(rgbNonLinear.g),
                              pq_eotf(rgbNonLinear.b));
            // Convert absolute nits to EDR linear
            float scale = max(cscParams.opticalOutputScale, 1e-6f);
            return L / scale;
        }

        default:
        case TF_NonLinear:
            return rgbNonLinear;
    }
}

// Vertex

vertex Vertex vs_draw(constant Vertex *vertices [[ buffer(0) ]], uint id [[ vertex_id ]])
{
    return vertices[id];
}

// Fragments

fragment float4 ps_draw_biplanar(Vertex v [[ stage_in ]],
                                 constant CscParams &cscParams [[ buffer(0) ]],
                                 texture2d<float> luminancePlane [[ texture(0) ]],
                                 texture2d<float> chrominancePlane [[ texture(1) ]])
{
    float3 yuv = float3(luminancePlane.sample(s, v.texCoords).r,
                        chrominancePlane.sample(s, v.texCoords).rg);
    yuv *= cscParams.bitnessScaleFactor;
    yuv -= cscParams.offsets;

    float3 rgb;
    rgb.r = dot(yuv, cscParams.matrix[0]);
    rgb.g = dot(yuv, cscParams.matrix[1]);
    rgb.b = dot(yuv, cscParams.matrix[2]);

    float3 rgbOut = handleTransferFunction(rgb, cscParams);
    return float4(rgbOut, 1.0f);
}

fragment float4 ps_draw_triplanar(Vertex v [[ stage_in ]],
                                  constant CscParams &cscParams [[ buffer(0) ]],
                                  texture2d<float> luminancePlane [[ texture(0) ]],
                                  texture2d<float> chrominancePlaneU [[ texture(1) ]],
                                  texture2d<float> chrominancePlaneV [[ texture(2) ]])
{
    float3 yuv = float3(luminancePlane.sample(s, v.texCoords).r,
                        chrominancePlaneU.sample(s, v.texCoords).r,
                        chrominancePlaneV.sample(s, v.texCoords).r);
    yuv *= cscParams.bitnessScaleFactor;
    yuv -= cscParams.offsets;

    float3 rgb;
    rgb.r = dot(yuv, cscParams.matrix[0]);
    rgb.g = dot(yuv, cscParams.matrix[1]);
    rgb.b = dot(yuv, cscParams.matrix[2]);

    float3 rgbOut = handleTransferFunction(rgb, cscParams);
    return float4(rgb, 1.0f);
}

// used for overlay, no tone-mapping
fragment float4 ps_draw_rgb(Vertex v [[ stage_in ]],
                            texture2d<float> rgbTexture [[ texture(0) ]])
{
    return rgbTexture.sample(s, v.texCoords);
}
