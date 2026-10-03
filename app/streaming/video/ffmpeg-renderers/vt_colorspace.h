#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <simd/simd.h>
#include <CoreGraphics/CoreGraphics.h>
#include <CoreVideo/CoreVideo.h>
#import <Metal/Metal.h>
#include <Limelight.h>
extern "C" {
#include <libavutil/frame.h>
#include <libavutil/mastering_display_metadata.h>
}

// Keep this layout identical to CscParams in vt_renderer.metal. PQ amplifies
// errors in YUV conversion, so both the conversion and sampling use float.
struct VTMetalCscParams {
    simd_float3x3 matrix;
    simd_float3 offsets;
    simd_float2 chromaOffset;
    float bitnessScaleFactor;
};

inline float vtUnormScale(int depth, int storageBits, int shift) {
    return float((uint64_t(1) << storageBits) - 1) /
           float(((uint64_t(1) << depth) - 1) << shift);
}

// Matrix coefficients describe YUV encoding, not the RGB gamut or transfer
// function. Honor explicit primaries and transfer tags independently.
inline CFStringRef vtColorSpaceName(const AVFrame* frame, int fallback, bool linearPQ = false) {
    bool bt2020 = frame->color_primaries == AVCOL_PRI_BT2020 ||
        (frame->color_primaries == AVCOL_PRI_UNSPECIFIED && fallback == COLORSPACE_REC_2020);
    bool p3 = frame->color_primaries == AVCOL_PRI_SMPTE432;
    bool bt709 = frame->color_primaries == AVCOL_PRI_BT709 ||
        (frame->color_primaries == AVCOL_PRI_UNSPECIFIED && fallback == COLORSPACE_REC_709);
    if (frame->color_trc == AVCOL_TRC_SMPTE2084) {
        if (linearPQ) return bt2020 ? kCGColorSpaceExtendedLinearITUR_2020 :
            p3 ? kCGColorSpaceExtendedLinearDisplayP3 : kCGColorSpaceExtendedLinearSRGB;
        return bt2020 ? kCGColorSpaceITUR_2100_PQ :
            p3 ? kCGColorSpaceDisplayP3_PQ : kCGColorSpaceITUR_709_PQ;
    }
    if (frame->color_trc == AVCOL_TRC_ARIB_STD_B67) {
        return bt2020 ? kCGColorSpaceITUR_2100_HLG :
            p3 ? kCGColorSpaceDisplayP3_HLG : kCGColorSpaceITUR_709_HLG;
    }
    if (bt2020) return frame->color_trc == AVCOL_TRC_IEC61966_2_1 ?
        kCGColorSpaceITUR_2020_sRGBGamma : kCGColorSpaceITUR_2020;
    if (p3) return kCGColorSpaceDisplayP3;
    return bt709 && frame->color_trc != AVCOL_TRC_IEC61966_2_1 ?
        kCGColorSpaceITUR_709 : kCGColorSpaceSRGB;
}

inline MTLPixelFormat vtMetalPixelFormat(const AVFrame* frame, int depth, bool useEDR) {
    if (useEDR && frame->color_trc == AVCOL_TRC_SMPTE2084) return MTLPixelFormatRGBA16Float;
    return depth > 8 || frame->color_trc == AVCOL_TRC_SMPTE2084 ||
        frame->color_trc == AVCOL_TRC_ARIB_STD_B67 ? MTLPixelFormatBGR10A2Unorm : MTLPixelFormatBGRA8Unorm;
}

struct VTMetalOverlayParams {
    simd_float3x3 matrix;
    float referenceWhite;
    uint32_t outputTransfer; // 0 = SDR, 1 = PQ, 2 = linear EDR, 3 = HLG
};

inline VTMetalOverlayParams vtMetalOverlayParams(const AVFrame* frame, int fallback, bool linearPQ, float white) {
    VTMetalOverlayParams result = {};
    result.referenceWhite = white;
    result.outputTransfer = frame->color_trc == AVCOL_TRC_SMPTE2084 ? (linearPQ ? 2 : 1) :
        frame->color_trc == AVCOL_TRC_ARIB_STD_B67 ? 3 : 0;
    result.matrix = matrix_identity_float3x3;
    // sRGB overlays are Rec.709. Transform into the video's gamut before
    // encoding them into the same HDR render target.
    if (frame->color_primaries == AVCOL_PRI_BT2020 ||
        (frame->color_primaries == AVCOL_PRI_UNSPECIFIED && fallback == COLORSPACE_REC_2020)) {
        result.matrix = simd_matrix(simd_make_float3(0.6274040f, 0.3292820f, 0.0433136f),
                                    simd_make_float3(0.0690970f, 0.9195400f, 0.0113612f),
                                    simd_make_float3(0.0163916f, 0.0880132f, 0.8955950f));
    }
    else if (frame->color_primaries == AVCOL_PRI_SMPTE432) {
        result.matrix = simd_matrix(simd_make_float3(0.822462f, 0.177538f, 0.0f),
                                    simd_make_float3(0.033194f, 0.966806f, 0.0f),
                                    simd_make_float3(0.017083f, 0.072397f, 0.910520f));
    }
    return result;
}

struct VTHdrMetadata {
    std::array<uint8_t, 24> display = {};
    std::array<uint8_t, 4> content = {};
    bool hasDisplay = false;
    bool hasContent = false;
    // A luminance-only fallback when a PQ stream omits mastering metadata.
    float minNits = 0;
    float maxNits = 1000;
    bool operator==(const VTHdrMetadata& other) const {
        return display == other.display && content == other.content &&
            hasDisplay == other.hasDisplay && hasContent == other.hasContent &&
            minNits == other.minNits && maxNits == other.maxNits;
    }
};

inline void vtWriteBigEndian(uint8_t* out, uint32_t value, int bytes) {
    for (int i = 0; i < bytes; ++i) out[i] = uint8_t(value >> (8 * (bytes - i - 1)));
}

inline VTHdrMetadata vtHdrMetadataForFrame(const AVFrame* frame, const SS_HDR_METADATA* host) {
    VTHdrMetadata result;
    // SDR frames must never inherit HDR metadata from an asynchronous callback
    // or a recycled CVPixelBuffer. Ten-bit pixels alone do not imply HDR.
    if (frame->color_trc != AVCOL_TRC_SMPTE2084) return result;

    uint32_t primaries[3][2] = {}, white[2] = {};
    bool hasPrimaries = false, hasLuminance = false;
    auto display = av_frame_get_side_data(frame, AV_FRAME_DATA_MASTERING_DISPLAY_METADATA);
    if (display && display->size >= sizeof(AVMasteringDisplayMetadata)) {
        auto md = reinterpret_cast<const AVMasteringDisplayMetadata*>(display->data);
        double min = av_q2d(md->min_luminance), max = av_q2d(md->max_luminance);
        hasLuminance = md->has_luminance && std::isfinite(min) && std::isfinite(max) &&
            min >= 0 && max > min && max <= 10000;
        if (hasLuminance) { result.minNits = min; result.maxNits = max; }
        hasPrimaries = md->has_primaries;
        for (int i = 0; i < 3; ++i) for (int j = 0; j < 2; ++j) {
            double value = av_q2d(md->display_primaries[i][j]);
            if (!std::isfinite(value) || value < 0 || value > 1) hasPrimaries = false;
            else primaries[i][j] = std::lround(value * 50000);
        }
        for (int j = 0; j < 2; ++j) {
            double value = av_q2d(md->white_point[j]);
            if (!std::isfinite(value) || value <= 0 || value > 1) hasPrimaries = false;
            else white[j] = std::lround(value * 50000);
        }
    }
    else if (host) {
        hasLuminance = host->maxDisplayLuminance > 0 && host->maxDisplayLuminance <= 10000 &&
            host->maxDisplayLuminance > host->minDisplayLuminance / 10000.0f;
        if (hasLuminance) {
            result.minNits = host->minDisplayLuminance / 10000.0f;
            result.maxNits = host->maxDisplayLuminance;
        }
        hasPrimaries = host->displayPrimaries[0].x != 0 && host->whitePoint.x != 0 && host->whitePoint.y != 0;
        for (int i = 0; i < 3; ++i) {
            primaries[i][0] = host->displayPrimaries[i].x;
            primaries[i][1] = host->displayPrimaries[i].y;
            if (primaries[i][0] > 50000 || primaries[i][1] > 50000) hasPrimaries = false;
        }
        white[0] = host->whitePoint.x; white[1] = host->whitePoint.y;
        if (white[0] > 50000 || white[1] > 50000) hasPrimaries = false;
    }
    if (hasPrimaries && hasLuminance) {
        // CoreVideo's 24-byte MDCV is big-endian, GBR, in 0.0001 nit units.
        for (int i = 0; i < 3; ++i) for (int j = 0; j < 2; ++j)
            vtWriteBigEndian(result.display.data() + 4 * i + 2 * j, primaries[(i + 1) % 3][j], 2);
        for (int j = 0; j < 2; ++j) vtWriteBigEndian(result.display.data() + 12 + 2 * j, white[j], 2);
        vtWriteBigEndian(result.display.data() + 16, std::lround(result.maxNits * 10000.0), 4);
        vtWriteBigEndian(result.display.data() + 20, std::lround(result.minNits * 10000.0), 4);
        result.hasDisplay = true;
    }
    uint32_t maxCLL = 0, maxFALL = 0;
    auto content = av_frame_get_side_data(frame, AV_FRAME_DATA_CONTENT_LIGHT_LEVEL);
    if (content && content->size >= sizeof(AVContentLightMetadata)) {
        auto cl = reinterpret_cast<const AVContentLightMetadata*>(content->data);
        maxCLL = cl->MaxCLL; maxFALL = cl->MaxFALL;
    }
    else if (host) {
        maxCLL = host->maxContentLightLevel; maxFALL = host->maxFrameAverageLightLevel;
    }
    if ((maxCLL || maxFALL) && maxCLL <= 65535 && maxFALL <= 65535) {
        vtWriteBigEndian(result.content.data(), maxCLL, 2);
        vtWriteBigEndian(result.content.data() + 2, maxFALL, 2);
        result.hasContent = true;
    }
    return result;
}

inline void vtAttachHdrMetadata(CVPixelBufferRef buffer, CFDataRef display, CFDataRef content) {
    if (display) CVBufferSetAttachment(buffer, kCVImageBufferMasteringDisplayColorVolumeKey,
                                       display, kCVAttachmentMode_ShouldPropagate);
    else CVBufferRemoveAttachment(buffer, kCVImageBufferMasteringDisplayColorVolumeKey);
    if (content) CVBufferSetAttachment(buffer, kCVImageBufferContentLightLevelInfoKey,
                                       content, kCVAttachmentMode_ShouldPropagate);
    else CVBufferRemoveAttachment(buffer, kCVImageBufferContentLightLevelInfoKey);
}
