#pragma once

#include <memory>
#include <cstdint>
extern "C" {
#include <libavutil/frame.h>
}

bool pyroWaveMetalSupported();

#ifdef __OBJC__
#import <Metal/Metal.h>
// Owned by AVFrame.buf[0]. The pool retains textures until the last frame
// reference is released; the renderer finishes its GPU reads before release.
struct PyroWaveMetalFrame {
    uint64_t magic = UINT64_C(0x50574d4554414c31);
    id<MTLTexture> textures[3] = {};
    id<MTLCommandBuffer> completion = nil;
    id<MTLSharedEvent> readyEvent = nil;
    uint64_t readyValue = 0;
};
PyroWaveMetalFrame* pyroWaveMetalFrame(const AVFrame* frame);
bool pyroWaveMetalWait(const PyroWaveMetalFrame* frame);
#endif

#ifdef PYROWAVE_METAL_TEST
// Headless calibration uses the stream's Metal shaders and texture formats.
class PyroWaveMetalCalibrationRenderer {
public:
    PyroWaveMetalCalibrationRenderer();
    ~PyroWaveMetalCalibrationRenderer();
    bool create(int width, int height);
    bool prepare(int width, int height, bool chroma444, bool hdr);
    bool present(AVFrame* frame, bool hdr);
    void* device() const;
private:
    struct Impl;
    std::unique_ptr<Impl> m_Impl;
};

#endif
