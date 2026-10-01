#pragma once

#ifdef HAVE_PYROWAVE

#include "decoder.h"
#include <memory>

class PyroWaveVideoDecoder : public IVideoDecoder {
public:
    explicit PyroWaveVideoDecoder(bool testOnly);
    ~PyroWaveVideoDecoder() override;
    bool initialize(PDECODER_PARAMETERS params) override;
    bool isHardwareAccelerated() override { return true; }
    bool isAlwaysFullScreen() override { return false; }
    bool isHdrSupported() override;
    int getDecoderCapabilities() override { return CAPABILITY_PULL_RENDERER; }
    int getDecoderColorspace() override { return COLORSPACE_REC_709; }
    int getDecoderColorRange() override;
    QSize getDecoderMaxResolution() override { return QSize(16384, 16384); }
    int submitDecodeUnit(PDECODE_UNIT du) override;
    void renderFrameOnMainThread() override;
    void setHdrMode(bool enabled) override;
    bool notifyWindowChanged(PWINDOW_STATE_CHANGE_INFO info) override;

private:
    struct Impl;
    std::unique_ptr<Impl> m_Impl;
    static int decoderThread(void* context);
    int dropFrame(const char* reason);
    int decodeFailed(const char* reason);
};

#endif
