#pragma once

#include "../metalframe.h"
#import <simd/simd.h>
extern "C" {
#include <libavutil/pixdesc.h>
}

// These layouts match vt_renderer.metal's vertex and fragment uniforms.
struct CscParams {
    simd_half3x3 matrix;
    simd_half3 offsets;
};

struct ParamBuffer {
    CscParams cscParams;
    simd_half2 chromaOffset;
    simd_half1 bitnessScaleFactor;
};

struct Vertex {
    simd_float4 position;
    simd_float2 texCoord;
};

inline int getMetalTextureSampleScale(const AVFrame* frame)
{
    // Native GPU planes contain normalized samples. CPU YUV10 puts its samples
    // in the low bits of a 16-bit word, so an R16 texture needs scaling by 64.
    if (frame->format == AV_PIX_FMT_VIDEOTOOLBOX || getMetalVideoFrame(frame)) {
        return 1;
    }
    auto format = av_pix_fmt_desc_get((AVPixelFormat)frame->format);
    return format ? 1 << (format->comp[0].step * 8 - format->comp[0].depth) : 1;
}
