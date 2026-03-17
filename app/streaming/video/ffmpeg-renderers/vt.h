#pragma once

#include "renderer.h"

#ifdef __OBJC__
#import <Metal/Metal.h>
class VTBaseRenderer : public IFFmpegRenderer {
public:
    VTBaseRenderer(IFFmpegRenderer::RendererType type);
    virtual ~VTBaseRenderer();
    bool checkDecoderCapabilities(id<MTLDevice> device, PDECODER_PARAMETERS params);
    void setHdrMode(bool enabled) override;

protected:
    bool isAppleSilicon();

    bool m_HdrMetadataChanged; // Manual reset
    CFDataRef m_MasteringDisplayColorVolume;
    CFDataRef m_ContentLightLevelInfo;
};

// Frame queue debugging, uncomment FRAME_QUEUE_VERBOSE or FRAME_QUEUE_VERBOSE_LIMITED
// When FQLog is enabled, the log volume can be intense, so LIMITED only logs data for a short time
#if !defined(NDEBUG)
#define FRAME_QUEUE_VERBOSE
//#define FRAME_QUEUE_VERBOSE_LIMITED
#endif

#ifdef FRAME_QUEUE_VERBOSE
	#define FQLog(fmt, ...) \
		NSLog(fmt, ##__VA_ARGS__)
#else
# ifdef FRAME_QUEUE_VERBOSE_LIMITED
	#include <atomic>
	static std::atomic<int> g_fqlog_counter{0};
	#define FQLog(fmt, ...) \
        if (++g_fqlog_counter > 200 && g_fqlog_counter < 1000) \
		    NSLog(fmt, ##__VA_ARGS__)
# else
    #if defined(_MSC_VER)
        #define FQLog(...) __noop
    #else
        #define FQLog(fmt, ...) do {} while(0)
    #endif
# endif
#endif

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
