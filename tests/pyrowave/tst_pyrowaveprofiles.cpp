#include "streaming/session.h"
#include <cstdio>
#include <cstdlib>
static void require(bool ok, const char* message) {
    if (!ok) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
}
int main() {
    SupportedVideoFormatList formats;
    formats << VIDEO_FORMAT_PYROWAVE10_444 << VIDEO_FORMAT_PYROWAVE10_420
            << VIDEO_FORMAT_PYROWAVE_444 << VIDEO_FORMAT_PYROWAVE << VIDEO_FORMAT_H264;
    require(formats.maskByServerCodecModes((SCM_PYROWAVE | SCM_PYROWAVE_444 | SCM_PYROWAVE10_420 | SCM_PYROWAVE10_444)) == VIDEO_FORMAT_MASK_PYROWAVE,
            "host PyroWave capabilities survive format mapping");
    require(formats.maskByServerCodecModes(SCM_MASK_10BIT) ==
            (VIDEO_FORMAT_PYROWAVE10_420 | VIDEO_FORMAT_PYROWAVE10_444), "HDR negotiation retains PyroWave 10-bit");
    require(formats.maskByServerCodecModes(SCM_MASK_YUV444) ==
            (VIDEO_FORMAT_PYROWAVE_444 | VIDEO_FORMAT_PYROWAVE10_444), "4:4:4 negotiation retains PyroWave profiles");
    require(formats.maskByServerCodecModes(SCM_H264) == VIDEO_FORMAT_H264, "unsupported host has H.264 fallback");
    formats.removeByMask(~formats.maskByServerCodecModes(SCM_PYROWAVE10_444));
    require(formats.size() == 1 && formats.front() == VIDEO_FORMAT_PYROWAVE10_444,
            "host capability filtering selects 10-bit 4:4:4");
    std::puts("PASS: PyroWave host profile mapping and HDR/4:4:4 negotiation");
}
