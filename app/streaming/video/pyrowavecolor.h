#pragma once

#include <Limelight.h>
extern "C" {
#include <libavutil/frame.h>
}

// Bit depth is negotiated for the session, but HDR can change while streaming.
// Apply the decode unit's color state to each frame, including HDR-to-SDR changes.
inline void setPyroWaveFrameColors(AVFrame* frame, const DECODE_UNIT* du, int colorRange)
{
    frame->color_range = colorRange == COLOR_RANGE_FULL ? AVCOL_RANGE_JPEG : AVCOL_RANGE_MPEG;
    int colorspace = du->hdrActive ? COLORSPACE_REC_2020 : du->colorspace;
    frame->colorspace = colorspace == COLORSPACE_REC_601 ? AVCOL_SPC_SMPTE170M :
                        colorspace == COLORSPACE_REC_2020 ? AVCOL_SPC_BT2020_NCL : AVCOL_SPC_BT709;
    frame->color_primaries = colorspace == COLORSPACE_REC_601 ? AVCOL_PRI_SMPTE170M :
                            colorspace == COLORSPACE_REC_2020 ? AVCOL_PRI_BT2020 : AVCOL_PRI_BT709;
    frame->color_trc = du->hdrActive ? AVCOL_TRC_SMPTE2084 : AVCOL_TRC_BT709;
    frame->chroma_location = AVCHROMA_LOC_CENTER;
}
