#pragma once

#include "renderer.h"

#ifdef __OBJC__
#import <Metal/Metal.h>
#include "vt_colorspace.h"
#include <mutex>
class VTBaseRenderer : public IFFmpegRenderer {
public:
    VTBaseRenderer(IFFmpegRenderer::RendererType type);
    virtual ~VTBaseRenderer();
    bool checkDecoderCapabilities(id<MTLDevice> device, PDECODER_PARAMETERS params);
    void setHdrMode(bool enabled) override;

protected:
    bool isAppleSilicon();
    void updateHdrMetadataForFrame(const AVFrame* frame);

    bool m_HdrMetadataChanged; // Manual reset
    CFDataRef m_MasteringDisplayColorVolume;
    CFDataRef m_ContentLightLevelInfo;
    float m_MinNits;
    float m_MaxNits;
    bool m_OverrideNits;

private:
    // Control callbacks publish a snapshot. Only the render thread owns and
    // replaces the CoreFoundation metadata consumed by the display layers.
    std::mutex m_HostHdrMetadataLock;
    SS_HDR_METADATA m_HostHdrMetadata = {};
    bool m_HostHdrMetadataValid = false;
    VTHdrMetadata m_FrameHdrMetadata;
    bool m_FrameHdrMetadataInitialized = false;
};

#endif // __OBJC__

// A factory is required to avoid pulling in
// incompatible Objective-C headers.

class VTMetalRendererFactory {
public:
    static
    IFFmpegRenderer* createRenderer(bool hwAccel);
};

class VTRendererFactory {
public:
    static
    IFFmpegRenderer* createRenderer();
};
