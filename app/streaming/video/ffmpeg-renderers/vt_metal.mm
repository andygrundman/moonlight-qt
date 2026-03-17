// Avoid conflict between AVFoundation and
// libavutil both defining AVMediaType
#define AVMediaType AVMediaType_FFmpeg
#include "vt.h"
#include "pacer/displaylink_source.h"
#include "pacer/pacer.h"
#undef AVMediaType

#include <algorithm>
#include <SDL_syswm.h>
#include <Limelight.h>
#include "streaming/session.h"
#include "streaming/streamutils.h"
#include "path.h"

#import <Cocoa/Cocoa.h>
#import <VideoToolbox/VideoToolbox.h>
#import <AVFoundation/AVFoundation.h>
#import <dispatch/dispatch.h>
#import <Metal/Metal.h>
#import <MetalKit/MetalKit.h>

extern "C" {
    #include <libavutil/pixdesc.h>
}

struct CscParams
{
    simd_half3x3 matrix;
    simd_half3 offsets;
};

struct ParamBuffer
{
    CscParams cscParams;
    simd_half2 chromaOffset;
    simd_half1 bitnessScaleFactor;
};

struct Vertex
{
    simd_float4 position;
    simd_float2 texCoord;
};

// https://developer.apple.com/library/archive/documentation/3DDrawing/Conceptual/MTLBestPracticesGuide/TripleBuffering.html
// https://developer.apple.com/documentation/metal/synchronizing-cpu-and-gpu-work?language=objc
#define MAX_FRAMES_IN_FLIGHT 3

#define MAX_VIDEO_PLANES 3

class VTMetalRenderer;
@interface VTMetalObserver : NSObject

- (id)initWithRenderer:(VTMetalRenderer *)renderer;
- (void)stop;

@end

class VTMetalRenderer : public VTBaseRenderer
{
public:
    VTMetalRenderer(bool hwAccel)
        : VTBaseRenderer(RendererType::VTMetal),
          m_HwAccel(hwAccel),
          m_Window(nullptr),
          m_HwContext(nullptr),
          m_MetalLayer(nullptr),
          m_VsyncReady(dispatch_semaphore_create(0)),
          m_TextureCache(nullptr),
          m_CscParamsBuffer(nullptr),
          m_VideoVertexBuffer(nullptr),
          m_OverlayTextures{},
          m_OverlayLock(0),
          m_RenderPassDescriptor(nullptr),
          m_VideoPipelineState(nullptr),
          m_OverlayPipelineState(nullptr),
          m_ShaderLibrary(nullptr),
          m_CommandQueue(nullptr),
          m_CurrentBuffer(0),
          m_SwMappingTextures{},
          m_MetalView(nullptr),
          m_LastFrameWidth(-1),
          m_LastFrameHeight(-1),
          m_LastDrawableWidth(-1),
          m_LastDrawableHeight(-1),
          m_lastPts(0),
          m_IsFullScreen(false),
          m_MinRefreshInterval(1.0f / 60),
          m_MaxRefreshInterval(1.0f / 60),
          m_LastPresented{0.0},
          m_LastGPUStartTime{0.0},
          m_UsingVRR{false},
          m_IsPaused{false},
          m_Observer(nil)
    {
        StreamingPreferences *prefs = StreamingPreferences::get();
        m_MaxFramesInFlight = SDL_min(prefs->vtMetalFramesInFlight, MAX_FRAMES_IN_FLIGHT);
        m_FrameReady = dispatch_semaphore_create(m_MaxFramesInFlight);
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "Metal renderer using MaxFramesInFlight=%d", m_MaxFramesInFlight);
   }

    virtual ~VTMetalRenderer() override
    { @autoreleasepool {
        // hide Metal HUD so it doesn't appear over the Qt UI
        showMetalHud(false);

        if (m_HwContext != nullptr) {
            av_buffer_unref(&m_HwContext);
        }

        if (m_Observer != nil) {
            [m_Observer stop];
            m_Observer = nil;
        }

        if (m_CscParamsBuffer != nullptr) {
            [m_CscParamsBuffer release];
        }

        if (m_VideoVertexBuffer != nullptr) {
            [m_VideoVertexBuffer release];
        }

        if (m_RenderPassDescriptor != nullptr) {
            [m_RenderPassDescriptor release];
        }

        if (m_VideoPipelineState != nullptr) {
            [m_VideoPipelineState release];
        }

        for (int i = 0; i < Overlay::OverlayMax; i++) {
            if (m_OverlayTextures[i] != nullptr) {
                [m_OverlayTextures[i] release];
            }
        }

        for (int i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
            for (int j = 0; j < MAX_VIDEO_PLANES; j++) {
                if (m_SwMappingTextures[i][j] != nullptr) {
                    [m_SwMappingTextures[i][j] release];
                }
            }
        }

        if (m_OverlayPipelineState != nullptr) {
            [m_OverlayPipelineState release];
        }

        if (m_ShaderLibrary != nullptr) {
            [m_ShaderLibrary release];
        }

        if (m_CommandQueue != nullptr) {
            [m_CommandQueue release];
        }

        if (m_TextureCache != nullptr) {
            CFRelease(m_TextureCache);
        }

        if (m_MetalView != nullptr) {
            SDL_Metal_DestroyView(m_MetalView);
        }
    }}

    void showMetalHud(bool visible)
    {
        if (@available(macOS 13.0, *)) {
            if (visible) {
                m_MetalLayer.developerHUDProperties = @{
                    @"MTL_HUD_OPACITY": @0.8,
                    @"MTL_HUD_DISABLE_MENU_BAR": @0,
                    @"MTL_HUD_ALIGNMENT": @"topright",
                    @"MTL_HUD_ELEMENTS": @"device,rosetta,layersize,layerscale,memory,fps,frameinterval,frameintervalgraph,presentdelay,gputime,thermal,refreshrate,gamemode,client",
                    @"MTL_HUD_SHOW_METRICS_RANGE": @1,
                };
            }
            else {
                m_MetalLayer.developerHUDProperties = @{
                    @"MTL_HUD_OPACITY": @0.0,
                    @"MTL_HUD_DISABLE_MENU_BAR": @1,
                };
            }
        }
    }

    bool updateVideoRegionSizeForFrame(AVFrame* frame)
    {
        int drawableWidth, drawableHeight;
        SDL_Metal_GetDrawableSize(m_Window, &drawableWidth, &drawableHeight);

        // Check if anything has changed since the last vertex buffer upload
        if (m_VideoVertexBuffer &&
                frame->width == m_LastFrameWidth && frame->height == m_LastFrameHeight &&
                drawableWidth == m_LastDrawableWidth && drawableHeight == m_LastDrawableHeight) {
            // Nothing to do
            return true;
        }

        // Determine the correct scaled size for the video region
        SDL_Rect src, dst;
        src.x = src.y = 0;
        src.w = frame->width;
        src.h = frame->height;
        dst.x = dst.y = 0;
        dst.w = drawableWidth;
        dst.h = drawableHeight;
        StreamUtils::scaleSourceToDestinationSurface(&src, &dst);

        // Convert screen space to normalized device coordinates
        SDL_FRect renderRect;
        StreamUtils::screenSpaceToNormalizedDeviceCoords(&dst, &renderRect, drawableWidth, drawableHeight);

        Vertex verts[] =
        {
            { { renderRect.x, renderRect.y, 0.0f, 1.0f }, { 0.0f, 1.0f } },
            { { renderRect.x, renderRect.y+renderRect.h, 0.0f, 1.0f }, { 0.0f, 0} },
            { { renderRect.x+renderRect.w, renderRect.y, 0.0f, 1.0f }, { 1.0f, 1.0f} },
            { { renderRect.x+renderRect.w, renderRect.y+renderRect.h, 0.0f, 1.0f }, { 1.0f, 0} },
        };

        [m_VideoVertexBuffer release];
        auto bufferOptions = MTLCPUCacheModeWriteCombined | (m_MetalLayer.device.hasUnifiedMemory ? MTLResourceStorageModeShared : MTLResourceStorageModeManaged);
        m_VideoVertexBuffer = [m_MetalLayer.device newBufferWithBytes:verts length:sizeof(verts) options:bufferOptions];
        if (!m_VideoVertexBuffer) {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                         "Failed to create video vertex buffer");
            return false;
        }

        m_LastFrameWidth = frame->width;
        m_LastFrameHeight = frame->height;
        m_LastDrawableWidth = drawableWidth;
        m_LastDrawableHeight = drawableHeight;

        return true;
    }

    int getFramePlaneCount(AVFrame* frame)
    {
        if (frame->format == AV_PIX_FMT_VIDEOTOOLBOX) {
            return CVPixelBufferGetPlaneCount((CVPixelBufferRef)frame->data[3]);
        }
        else {
            return av_pix_fmt_count_planes((AVPixelFormat)frame->format);
        }
    }

    int getBitnessScaleFactor(AVFrame* frame)
    {
        if (frame->format == AV_PIX_FMT_VIDEOTOOLBOX) {
            // VideoToolbox frames never require scaling
            return 1;
        }
        else {
            const AVPixFmtDescriptor* formatDesc = av_pix_fmt_desc_get((AVPixelFormat)frame->format);
            if (!formatDesc) {
                // This shouldn't be possible but handle it anyway
                SDL_assert(formatDesc);
                return 1;
            }

            // This assumes plane 0 is exclusively the Y component
            SDL_assert(formatDesc->comp[0].step == 1 || formatDesc->comp[0].step == 2);
            return pow(2, (formatDesc->comp[0].step * 8) - formatDesc->comp[0].depth);
        }
    }

    bool updateColorSpaceForFrame(AVFrame* frame)
    {
        if (!hasFrameFormatChanged(frame) && !m_HdrMetadataChanged) {
            return true;
        }

        int colorspace = getFrameColorspace(frame);
        CGColorSpaceRef newColorSpace;
        ParamBuffer paramBuffer;

        switch (colorspace) {
        case COLORSPACE_REC_709:
            m_MetalLayer.colorspace = newColorSpace = CGColorSpaceCreateWithName(kCGColorSpaceITUR_709);
            m_MetalLayer.pixelFormat = MTLPixelFormatBGRA8Unorm;
            break;
        case COLORSPACE_REC_2020:
            m_MetalLayer.pixelFormat = MTLPixelFormatBGR10A2Unorm;
            if (frame->color_trc == AVCOL_TRC_SMPTE2084) {
                // https://developer.apple.com/documentation/metal/hdr_content/using_color_spaces_to_display_hdr_content
                m_MetalLayer.colorspace = newColorSpace = CGColorSpaceCreateWithName(kCGColorSpaceITUR_2100_PQ);
            }
            else {
                m_MetalLayer.colorspace = newColorSpace = CGColorSpaceCreateWithName(kCGColorSpaceITUR_2020);
            }
            break;
        default:
        case COLORSPACE_REC_601:
            m_MetalLayer.colorspace = newColorSpace = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
            m_MetalLayer.pixelFormat = MTLPixelFormatBGRA8Unorm;
            break;
        }

        std::array<float, 9> cscMatrix;
        std::array<float, 3> yuvOffsets;
        std::array<float, 2> chromaOffset;
        getFramePremultipliedCscConstants(frame, cscMatrix, yuvOffsets);
        getFrameChromaCositingOffsets(frame, chromaOffset);

        paramBuffer.cscParams.matrix = simd_matrix(simd_make_half3(cscMatrix[0], cscMatrix[3], cscMatrix[6]),
                                                   simd_make_half3(cscMatrix[1], cscMatrix[4], cscMatrix[7]),
                                                   simd_make_half3(cscMatrix[2], cscMatrix[5], cscMatrix[8]));
        paramBuffer.cscParams.offsets = simd_make_half3(yuvOffsets[0],
                                                        yuvOffsets[1],
                                                        yuvOffsets[2]);
        paramBuffer.chromaOffset = simd_make_half2(chromaOffset[0],
                                                   chromaOffset[1]);

        // Set the EDR metadata for HDR10 to enable OS tonemapping
        if (frame->color_trc == AVCOL_TRC_SMPTE2084 && m_MasteringDisplayColorVolume != nullptr) {
            m_MetalLayer.EDRMetadata = [CAEDRMetadata HDR10MetadataWithDisplayInfo:(__bridge NSData*)m_MasteringDisplayColorVolume
                                                                       contentInfo:(__bridge NSData*)m_ContentLightLevelInfo
                                                                opticalOutputScale:203.0];
        }
        else {
            m_MetalLayer.EDRMetadata = nullptr;
        }

        paramBuffer.bitnessScaleFactor = getBitnessScaleFactor(frame);

        // The CAMetalLayer retains the CGColorSpace
        CGColorSpaceRelease(newColorSpace);

        // Create the new colorspace parameter buffer for our fragment shader
        [m_CscParamsBuffer release];
        auto bufferOptions = MTLCPUCacheModeWriteCombined | (m_MetalLayer.device.hasUnifiedMemory ? MTLResourceStorageModeShared : MTLResourceStorageModeManaged);
        m_CscParamsBuffer = [m_MetalLayer.device newBufferWithBytes:(void*)&paramBuffer length:sizeof(paramBuffer) options:bufferOptions];
        if (!m_CscParamsBuffer) {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                         "Failed to create CSC parameters buffer");
            return false;
        }

        int planes = getFramePlaneCount(frame);
        SDL_assert(planes == 2 || planes == 3);

        MTLRenderPipelineDescriptor *pipelineDesc = [[MTLRenderPipelineDescriptor new] autorelease];
        pipelineDesc.vertexFunction = [[m_ShaderLibrary newFunctionWithName:@"vs_draw"] autorelease];
        pipelineDesc.fragmentFunction = [[m_ShaderLibrary newFunctionWithName:planes == 2 ? @"ps_draw_biplanar" : @"ps_draw_triplanar"] autorelease];
        pipelineDesc.colorAttachments[0].pixelFormat = m_MetalLayer.pixelFormat;
        [m_VideoPipelineState release];
        m_VideoPipelineState = [m_MetalLayer.device newRenderPipelineStateWithDescriptor:pipelineDesc error:nullptr];
        if (!m_VideoPipelineState) {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                         "Failed to create video pipeline state");
            return false;
        }

        pipelineDesc = [[MTLRenderPipelineDescriptor new] autorelease];
        pipelineDesc.vertexFunction = [[m_ShaderLibrary newFunctionWithName:@"vs_draw"] autorelease];
        pipelineDesc.fragmentFunction = [[m_ShaderLibrary newFunctionWithName:@"ps_draw_rgb"] autorelease];
        pipelineDesc.colorAttachments[0].pixelFormat = m_MetalLayer.pixelFormat;
        pipelineDesc.colorAttachments[0].blendingEnabled = YES;
        pipelineDesc.colorAttachments[0].rgbBlendOperation = MTLBlendOperationAdd;
        pipelineDesc.colorAttachments[0].alphaBlendOperation = MTLBlendOperationAdd;
        pipelineDesc.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
        pipelineDesc.colorAttachments[0].sourceAlphaBlendFactor = MTLBlendFactorSourceAlpha;
        pipelineDesc.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
        pipelineDesc.colorAttachments[0].destinationAlphaBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
        [m_OverlayPipelineState release];
        m_OverlayPipelineState = [m_MetalLayer.device newRenderPipelineStateWithDescriptor:pipelineDesc error:nullptr];
        if (!m_OverlayPipelineState) {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                         "Failed to create overlay pipeline state");
            return false;
        }

        m_HdrMetadataChanged = false;
        return true;
    }

    id<MTLTexture> mapPlaneForSoftwareFrame(AVFrame* frame, int planeIndex)
    {
        const AVPixFmtDescriptor* formatDesc = av_pix_fmt_desc_get((AVPixelFormat)frame->format);
        if (!formatDesc) {
            // This shouldn't be possible but handle it anyway
            SDL_assert(formatDesc);
            return nil;
        }

        SDL_assert(planeIndex < MAX_VIDEO_PLANES);

        NSUInteger planeWidth = planeIndex ? AV_CEIL_RSHIFT(frame->width, formatDesc->log2_chroma_w) : frame->width;
        NSUInteger planeHeight = planeIndex ? AV_CEIL_RSHIFT(frame->height, formatDesc->log2_chroma_h) : frame->height;

        auto texture = m_SwMappingTextures[m_CurrentBuffer][planeIndex];

        // Recreate the texture if the plane size changes
        if (texture && (texture.width != planeWidth || texture.height != planeHeight)) {
            [texture release];
            texture = nil;
        }

        if (!texture) {
            MTLPixelFormat metalFormat;

            switch (formatDesc->comp[planeIndex].step) {
            case 1:
                metalFormat = MTLPixelFormatR8Unorm;
                break;
            case 2:
                metalFormat = MTLPixelFormatR16Unorm;
                break;
            default:
                SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                             "Unhandled plane step: %d (plane: %d)",
                             formatDesc->comp[planeIndex].step,
                             planeIndex);
                SDL_assert(false);
                return nil;
            }

            auto texDesc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:metalFormat
                                                                              width:planeWidth
                                                                             height:planeHeight
                                                                          mipmapped:NO];
            texDesc.cpuCacheMode = MTLCPUCacheModeWriteCombined;
            texDesc.storageMode = MTLStorageModeManaged;
            texDesc.usage = MTLTextureUsageShaderRead;

            texture = [m_MetalLayer.device newTextureWithDescriptor:texDesc];
            if (!texture) {
                SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                             "Failed to allocate software frame texture");
                return nil;
            }
            m_SwMappingTextures[m_CurrentBuffer][planeIndex] = texture;
        }

        [texture replaceRegion:MTLRegionMake2D(0, 0, planeWidth, planeHeight)
                   mipmapLevel:0
                     withBytes:frame->data[planeIndex]
                   bytesPerRow:frame->linesize[planeIndex]];

        return texture;
    }

    bool createTexturesFromFrame(AVFrame* frame, std::array<CVMetalTextureRef, MAX_VIDEO_PLANES>& cvMetalTextures)
    {
        SDL_assert(frame->format == AV_PIX_FMT_VIDEOTOOLBOX);

        CVPixelBufferRef pixBuf = reinterpret_cast<CVPixelBufferRef>(frame->data[3]);
        size_t planes = getFramePlaneCount(frame);

        // Create Metal textures for the planes of the CVPixelBuffer
        for (size_t i = 0; i < planes; i++) {
            MTLPixelFormat fmt;

            switch (CVPixelBufferGetPixelFormatType(pixBuf)) {
            case kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange:
            case kCVPixelFormatType_444YpCbCr8BiPlanarVideoRange:
            case kCVPixelFormatType_420YpCbCr8BiPlanarFullRange:
            case kCVPixelFormatType_444YpCbCr8BiPlanarFullRange:
                fmt = (i == 0) ? MTLPixelFormatR8Unorm : MTLPixelFormatRG8Unorm;
                break;

            case kCVPixelFormatType_420YpCbCr10BiPlanarFullRange:
            case kCVPixelFormatType_444YpCbCr10BiPlanarFullRange:
            case kCVPixelFormatType_420YpCbCr10BiPlanarVideoRange:
            case kCVPixelFormatType_444YpCbCr10BiPlanarVideoRange:
                fmt = (i == 0) ? MTLPixelFormatR16Unorm : MTLPixelFormatRG16Unorm;
                break;

            default:
                SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                             "Unknown pixel format: %x",
                             CVPixelBufferGetPixelFormatType(pixBuf));
                return false;
            }

            CVReturn err = CVMetalTextureCacheCreateTextureFromImage(kCFAllocatorDefault, m_TextureCache, pixBuf, nullptr, fmt,
                                                                     CVPixelBufferGetWidthOfPlane(pixBuf, i),
                                                                     CVPixelBufferGetHeightOfPlane(pixBuf, i),
                                                                     i,
                                                                     &cvMetalTextures[i]);
            if (err != kCVReturnSuccess) {
                for (size_t j = 0; j < i; j++) {
                    CFRelease(cvMetalTextures[j]);
                }
                SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                             "CVMetalTextureCacheCreateTextureFromImage() failed: %d",
                             err);
                return false;
            }
        }

        return true;
    }

    bool testRenderFrame(AVFrame *frame) override
    { @autoreleasepool {
        if (frame->format == AV_PIX_FMT_VIDEOTOOLBOX) {
            std::array<CVMetalTextureRef, MAX_VIDEO_PLANES> cvMetalTextures;
            size_t planes = getFramePlaneCount(frame);
            SDL_assert(planes <= MAX_VIDEO_PLANES);

            // Test that we can actually create Metal textures from the CVPixelBufferRef
            if (!createTexturesFromFrame(frame, cvMetalTextures)) {
                return false;
            }

            for (size_t i = 0; i < planes; i++) {
                CFRelease(cvMetalTextures[i]);
            }
        }
        else {
            // Mapping software frames should always work
        }

        return true;
    }}

    // Caller frees frame after we return
    virtual void renderFrameIntoDrawable(AVFrame* frame, id<CAMetalDrawable> drawable)
    { @autoreleasepool {
        std::array<CVMetalTextureRef, MAX_VIDEO_PLANES> cvMetalTextures;
        size_t planes = getFramePlaneCount(frame);
        SDL_assert(planes <= MAX_VIDEO_PLANES);

        if (frame->format == AV_PIX_FMT_VIDEOTOOLBOX) {
            if (!createTexturesFromFrame(frame, cvMetalTextures)) {
                return;
            }
        }

        m_RenderPassDescriptor.colorAttachments[0].texture = drawable.texture;
        auto commandBuffer = [m_CommandQueue commandBuffer];
        auto renderEncoder = [commandBuffer renderCommandEncoderWithDescriptor:m_RenderPassDescriptor];

        // Bind textures and buffers then draw the video region
        [renderEncoder setRenderPipelineState:m_VideoPipelineState];
        if (frame->format == AV_PIX_FMT_VIDEOTOOLBOX) {
            for (size_t i = 0; i < planes; i++) {
                [renderEncoder setFragmentTexture:CVMetalTextureGetTexture(cvMetalTextures[i]) atIndex:i];
            }
        }
        else {
            for (size_t i = 0; i < planes; i++) {
                [renderEncoder setFragmentTexture:mapPlaneForSoftwareFrame(frame, i) atIndex:i];
            }
        }
        [renderEncoder setFragmentBuffer:m_CscParamsBuffer offset:0 atIndex:0];
        [renderEncoder setVertexBuffer:m_VideoVertexBuffer offset:0 atIndex:0];
        [renderEncoder drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4];

        // Now draw any overlays that are enabled
        for (int i = 0; i < Overlay::OverlayMax; i++) {
            id<MTLTexture> overlayTexture = nullptr;

            // Try to acquire a reference on the overlay texture
            SDL_AtomicLock(&m_OverlayLock);
            overlayTexture = [m_OverlayTextures[i] retain];
            SDL_AtomicUnlock(&m_OverlayLock);

            if (overlayTexture) {
                SDL_FRect renderRect = {};
                if (i == Overlay::OverlayStatusUpdate) {
                    // Bottom Left
                    renderRect.x = 0;
                    renderRect.y = 0;
                }
                else if (i == Overlay::OverlayDebug) {
                    // Top left
                    renderRect.x = 0;
                    renderRect.y = m_LastDrawableHeight - overlayTexture.height;
                }

                renderRect.w = overlayTexture.width;
                renderRect.h = overlayTexture.height;

                // Convert screen space to normalized device coordinates
                StreamUtils::screenSpaceToNormalizedDeviceCoords(&renderRect, m_LastDrawableWidth, m_LastDrawableHeight);

                Vertex verts[] =
                {
                    { { renderRect.x, renderRect.y, 0.0f, 1.0f }, { 0.0f, 1.0f } },
                    { { renderRect.x, renderRect.y+renderRect.h, 0.0f, 1.0f }, { 0.0f, 0} },
                    { { renderRect.x+renderRect.w, renderRect.y, 0.0f, 1.0f }, { 1.0f, 1.0f} },
                    { { renderRect.x+renderRect.w, renderRect.y+renderRect.h, 0.0f, 1.0f }, { 1.0f, 0} },
                };

                [renderEncoder setRenderPipelineState:m_OverlayPipelineState];
                [renderEncoder setFragmentTexture:overlayTexture atIndex:0];
                [renderEncoder setVertexBytes:verts length:sizeof(verts) atIndex:0];
                [renderEncoder drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4];

                [overlayTexture release];
            }
        }

        [renderEncoder endEncoding];
        m_RenderPassDescriptor.colorAttachments[0].texture = nil;

        [drawable addPresentedHandler:^(id<MTLDrawable> d) {
            // The obvious way of measuring frametime, but see below. Note that presentedTime can be 0.0
            // if the frame was missed for some reason.
            CFTimeInterval lastPresented = (CFTimeInterval)m_LastPresented.load();
            FQLog(@"[%f] dID %lu d.presentedTime %f, lastPresented %f, frametime %.3fms",
                CACurrentMediaTime(), (unsigned long)d.drawableID, d.presentedTime, lastPresented, (d.presentedTime - lastPresented) * 1000.0);
            if (lastPresented > 0.0 && d.presentedTime > 0.0) {
                //CFTimeInterval frametime = d.presentedTime - lastPresented;
            }
            if (d.presentedTime > 0.0) {
                m_LastPresented.store((double)d.presentedTime);
            }
        }];

        __block dispatch_semaphore_t block_semaphore = m_FrameReady;
        [commandBuffer addCompletedHandler:^(id<MTLCommandBuffer> cb) {
            // this error indicates we've been moved to the background and should stop Metal activity
            if (cb.status == MTLCommandBufferStatusError) {
                SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION,
                            "Metal command buffer error: %lu",
                            (unsigned long)cb.error);
            }

            // signal that the next frame can begin processing
            dispatch_semaphore_signal(block_semaphore);

            // Found this in a comment in MTLDrawable.h:
            // "Be careful when you use delta between this presentedTime and previous frame's presentedTime
            // to animate next frame. If the frame was presented using presentAfterMinimumDuration or presentAtTime,
            // the presentedTime might includes delays to meet your specified present time. If you want to measure
            // how much frame you can achieve, use GPUStartTime in the first command buffer of your frame rendering
            // and GPUEndTime of your last frame rendering to calculate the frame interval."
            CFTimeInterval lastGPUStartTime = (CFTimeInterval)m_LastGPUStartTime.load();
            FQLog(@"[%f] dID %lu commandBuffer completed GPUEndTime %f, lastGPUStartTime %f",
                    CACurrentMediaTime(), (unsigned long)drawable.drawableID, cb.GPUEndTime, lastGPUStartTime);
            if (lastGPUStartTime > 0.0) {
                //CFTimeInterval frametime = cb.GPUEndTime - lastGPUStartTime;
            }
            m_LastGPUStartTime.store((double)cb.GPUStartTime);

            if (m_HwAccel) {
                // Free textures after completion of rendering per CVMetalTextureCache requirements
                for (size_t i = 0; i < planes; i++) {
                    CFRelease(cvMetalTextures[i]);
                }
            }
        }];

        if (m_UsingVRR.load()) {
            // VRR: Use the previous frame's duration, while staying within the VRR range
            // Note that the Vsync checkbox is ignored when VRR is detected.
            CFTimeInterval duration = m_MinRefreshInterval;
            if (m_lastPts > 0) {
                int64_t deltaPts = frame->pts - m_lastPts;
                if (deltaPts > 0) {
                    double delta = static_cast<double>(deltaPts) / 90000.0;
                    duration = std::clamp(delta, m_MinRefreshInterval, m_MaxRefreshInterval);

                    FQLog(@"[VRR] pts %lld, deltaMs %.3f, max/min %.3f/%.3f, present afterMinimumDuration:%.3f ms",
                        frame->pts, delta * 1000.0, m_MaxRefreshInterval * 1000.0, m_MinRefreshInterval * 1000.0, duration * 1000.0);
                }
            }
            m_lastPts = frame->pts;
            FQLog(@"[%f] dID %lu VRR presentDrawable afterMinimumDuration:%f pts %.3fms",
                CACurrentMediaTime(), (unsigned long)drawable.drawableID, duration * 1000.0, frame->pts / 90.0);
            [commandBuffer presentDrawable:drawable afterMinimumDuration:duration];
        }
        else if (m_MetalLayer.displaySyncEnabled) {
            // Vsync enabled, lock to the refresh rate. A server that supports
            // clientRefreshRateX100 is needed if the refresh rate is fractional.
            CFTimeInterval duration = m_MinRefreshInterval;

            FQLog(@"[%f] dID %lu vsync presentDrawable afterMinimumDuration:%f",
                CACurrentMediaTime(), (unsigned long)drawable.drawableID, duration * 1000.0);

            // Don't delay the first frame
            if (drawable.drawableID == 0) {
                [commandBuffer presentDrawable:drawable];
            }
            else {
                [commandBuffer presentDrawable:drawable afterMinimumDuration:duration];
            }
        }
        else {
            // Vsync disabled could look like this but this is a stuttery mess
            //[commandBuffer presentDrawable:drawable];

            // Can we still use afterMinimumDuration without setting displaySyncEnabled?
            CFTimeInterval duration = m_MinRefreshInterval;
            FQLog(@"[%f] dID %lu no-vsync presentDrawable afterMinimumDuration:%f",
                CACurrentMediaTime(), (unsigned long)drawable.drawableID, duration * 1000.0);

            // Don't delay the first frame
            if (drawable.drawableID == 0) {
                [commandBuffer presentDrawable:drawable];
            }
            else {
                [commandBuffer presentDrawable:drawable afterMinimumDuration:duration];
            }
        }
        [commandBuffer commit];
    }}

    virtual void waitToRender() override
    {
        // Wait for a previous drawable to be presented and ready for reuse
        dispatch_semaphore_wait(m_FrameReady, DISPATCH_TIME_FOREVER);
    }

    // Caller frees frame after we return
    virtual void renderFrame(AVFrame* frame) override
    { @autoreleasepool {
        if (m_IsPaused.load()) {
            // we're paused due to being hidden or off screen,
            // we can just throw away the frame and bump the semaphore.
            dispatch_semaphore_signal(m_FrameReady);
            return;
        }

        m_CurrentBuffer = (m_CurrentBuffer + 1) % m_MaxFramesInFlight;

        // Handle changes to the frame's colorspace from last time we rendered
        if (!updateColorSpaceForFrame(frame)) {
            // Trigger the main thread to recreate the decoder
            SDL_Event event;
            event.type = SDL_RENDER_DEVICE_RESET;
            SDL_PushEvent(&event);
            return;
        }

        // Handle changes to the video size or drawable size
        if (!updateVideoRegionSizeForFrame(frame)) {
            // Trigger the main thread to recreate the decoder
            SDL_Event event;
            event.type = SDL_RENDER_DEVICE_RESET;
            SDL_PushEvent(&event);
            return;
        }

        // Render to the next drawable
        id<CAMetalDrawable> drawable = [m_MetalLayer nextDrawable];
        if (drawable == nullptr) {
            return;
        }

        renderFrameIntoDrawable(frame, drawable);
    }}

    id<MTLDevice> getMetalDevice() {
        StreamingPreferences *prefs = StreamingPreferences::get();
        if (prefs->renderer == StreamingPreferences::RENDERER_AVSAMPLEBUFFER || qgetenv("VT_FORCE_METAL") == "0") {
            SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                        "Avoiding Metal renderer due to VT_FORCE_METAL=0 override.");
            return nullptr;
        }

        NSArray<id<MTLDevice>> *devices = [MTLCopyAllDevices() autorelease];
        if (devices.count == 0) {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                         "No Metal device found!");
            return nullptr;
        }

        // First, try to find a low power (Intel) or unified memory (Apple Silicon) GPU
        for (id<MTLDevice> device in devices) {
            // choose iGPU on Intel Macs
            if (device.isLowPower || device.hasUnifiedMemory) {
                return device;
            }
        }

        // Next, we'll just try to pick something internal to the system
        for (id<MTLDevice> device in devices) {
            if (!device.isRemovable) {
                return device;
            }
        }

        // Use the system-default device
        return [MTLCreateSystemDefaultDevice() autorelease];
    }

    virtual bool initialize(PDECODER_PARAMETERS params) override
    { @autoreleasepool {
        int err;

        m_Window = params->window;
        m_FrameRateRange = CAFrameRateRangeMake(params->frameRate, params->frameRate, params->frameRate);

        id<MTLDevice> device = getMetalDevice();
        if (!device) {
            m_InitFailureReason = InitFailureReason::NoSoftwareSupport;
            return false;
        }

        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "Selected Metal device: %s",
                    device.name.UTF8String);

        if (m_HwAccel && !checkDecoderCapabilities(device, params)) {
            return false;
        }

        err = av_hwdevice_ctx_create(&m_HwContext,
                                     AV_HWDEVICE_TYPE_VIDEOTOOLBOX,
                                     nullptr,
                                     nullptr,
                                     0);
        if (err < 0) {
            SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION,
                        "av_hwdevice_ctx_create() failed for VT decoder: %d",
                        err);
            m_InitFailureReason = InitFailureReason::NoSoftwareSupport;
            return false;
        }

        // Create the Metal texture cache for our CVPixelBuffers
        CFStringRef keys[] = { kCVMetalTextureUsage };
        NSUInteger usage = MTLTextureUsageShaderRead;
        CFNumberRef usageNumber = CFNumberCreate(kCFAllocatorDefault, kCFNumberNSIntegerType, &usage);
        const void* values[] = { usageNumber };
        auto cacheAttributes = CFDictionaryCreate(kCFAllocatorDefault, (const void**)keys, values, 1, nullptr, nullptr);
        err = CVMetalTextureCacheCreate(kCFAllocatorDefault, cacheAttributes, device, nullptr, &m_TextureCache);
        CFRelease(cacheAttributes);
        CFRelease(usageNumber);

        if (err != kCVReturnSuccess) {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                         "CVMetalTextureCacheCreate() failed: %d",
                         err);
            return false;
        }

        // Compile our shaders
        QString shaderSource = QString::fromUtf8(Path::readDataFile("vt_renderer.metal"));
        m_ShaderLibrary = [device newLibraryWithSource:shaderSource.toNSString() options:nullptr error:nullptr];
        if (!m_ShaderLibrary) {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                         "Failed to compile shaders");
            return false;
        }

        // Create a command queue for submission
        m_CommandQueue = [device newCommandQueue];

        // we'll reuse one renderPassDescriptor by changing its texture
        m_RenderPassDescriptor = [[MTLRenderPassDescriptor alloc] init];
        m_RenderPassDescriptor.colorAttachments[0].loadAction = MTLLoadActionClear;
        m_RenderPassDescriptor.colorAttachments[0].clearColor = MTLClearColorMake(0, 0, 0, 0);
        m_RenderPassDescriptor.colorAttachments[0].storeAction = MTLStoreActionStore;

        // Add the Metal view to the window if we're not in test-only mode
        //
        // NB: Test-only renderers may be created on a non-main thread, so
        // we don't want to touch the view hierarchy in that context.
        if (!params->testOnly) {
            m_MetalView = SDL_Metal_CreateView(m_Window);
            if (!m_MetalView) {
                SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                             "SDL_Metal_CreateView() failed: %s",
                             SDL_GetError());
                return false;
            }

            m_MetalLayer = (CAMetalLayer*)SDL_Metal_GetLayer(m_MetalView);

            // Choose a device
            m_MetalLayer.device = device;
            m_MetalLayer.maximumDrawableCount = m_MaxFramesInFlight;
            m_MetalLayer.allowsNextDrawableTimeout = YES;

            // Allow EDR content if we're streaming in a 10-bit format
            m_MetalLayer.wantsExtendedDynamicRangeContent = !!(params->videoFormat & VIDEO_FORMAT_MASK_10BIT);

            // check fullscreen state, VRR, and refresh rate
            refreshWindowMetadata();

            // SDL's display change detection will call notifyWindowChanged() but there are some Apple-specific
            // notifications we need to watch out for, such as moving off screen or into the background.
            m_Observer = [[VTMetalObserver alloc] initWithRenderer:this];

            // I would completely remove the ability to run this renderer without vsync
            // but someone may want it for something.
            m_MetalLayer.displaySyncEnabled = YES;
            if (m_IsFullScreen) {
                if (!params->enableVsync) {
                    // Allow v-sync disabled only in fullscreen
                    m_MetalLayer.displaySyncEnabled = NO;
                    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                                "V-sync disabled per request in fullscreen");
                }
            }
            else {
                SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                            "V-sync enforced when running in a window");
            }
        }

        return true;
    }}

    // static
    // void displayLinkCallback(const double timestamp, const double targetTimestamp, void* ctx)
    // {
    //     auto me = reinterpret_cast<VTMetalRenderer*>(ctx);
    //     dispatch_semaphore_signal(me->m_VsyncReady);
    // }

    virtual void prepareToRender() override
    {
        // May want this in the future.
        // Register an extra vsync callback with DisplayLinkSource
        // DisplayLinkSource::instance().setExtraCallback(displayLinkCallback, this);
    }

    virtual void notifyOverlayUpdated(Overlay::OverlayType type) override
    { @autoreleasepool {
        SDL_Surface* newSurface = Session::get()->getOverlayManager().getUpdatedOverlaySurface(type);
        bool overlayEnabled = Session::get()->getOverlayManager().isOverlayEnabled(type);
        if (newSurface == nullptr && overlayEnabled) {
            // The overlay is enabled and there is no new surface. Leave the old texture alone.
            return;
        }

        SDL_AtomicLock(&m_OverlayLock);
        auto oldTexture = m_OverlayTextures[type];
        m_OverlayTextures[type] = nullptr;
        SDL_AtomicUnlock(&m_OverlayLock);

        [oldTexture release];

        // If the overlay is disabled, we're done
        if (!overlayEnabled) {
            showMetalHud(false);
            SDL_FreeSurface(newSurface);
            return;
        }

        if (StreamingPreferences::get()->showMetalPerformanceHud) {
            showMetalHud(true);
        }

        // Create a texture to hold our pixel data
        SDL_assert(!SDL_MUSTLOCK(newSurface));
        SDL_assert(newSurface->format->format == SDL_PIXELFORMAT_ARGB8888);
        auto texDesc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm
                                                                          width:newSurface->w
                                                                         height:newSurface->h
                                                                      mipmapped:NO];
        texDesc.cpuCacheMode = MTLCPUCacheModeWriteCombined;
        texDesc.storageMode = m_MetalLayer.device.hasUnifiedMemory ? MTLStorageModeShared : MTLStorageModeManaged;
        texDesc.usage = MTLTextureUsageShaderRead;
        auto newTexture = [m_MetalLayer.device newTextureWithDescriptor:texDesc];

        // Load the pixel data into the new texture
        [newTexture replaceRegion:MTLRegionMake2D(0, 0, newSurface->w, newSurface->h)
                      mipmapLevel:0
                        withBytes:newSurface->pixels
                      bytesPerRow:newSurface->pitch];

        // The surface is no longer required
        SDL_FreeSurface(newSurface);
        newSurface = nullptr;

        SDL_AtomicLock(&m_OverlayLock);
        m_OverlayTextures[type] = newTexture;
        SDL_AtomicUnlock(&m_OverlayLock);
    }}

    virtual bool prepareDecoderContext(AVCodecContext* context, AVDictionary**) override
    {
        if (m_HwAccel) {
            context->hw_device_ctx = av_buffer_ref(m_HwContext);
        }

        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "Using Metal renderer with %s decoding",
                    m_HwAccel ? "hardware" : "software");

        return true;
    }

    int getDecoderColorspace() override
    {
        // macOS seems to handle Rec 601 best
        return COLORSPACE_REC_601;
    }

    int getDecoderCapabilities() override
    {
        return CAPABILITY_REFERENCE_FRAME_INVALIDATION_HEVC |
               CAPABILITY_REFERENCE_FRAME_INVALIDATION_AV1;
    }

    int getRendererAttributes() override
    {
        // Require frame pacing until we switch to our own pacer
        return RENDERER_ATTRIBUTE_HDR_SUPPORT | RENDERER_ATTRIBUTE_FORCE_PACING;
    }

    bool isPixelFormatSupported(int videoFormat, AVPixelFormat pixelFormat) override
    {
        if (m_HwAccel) {
            return pixelFormat == AV_PIX_FMT_VIDEOTOOLBOX;
        }
        else {
            if (pixelFormat == AV_PIX_FMT_VIDEOTOOLBOX) {
                // VideoToolbox frames are always supported
                return true;
            }
            else {
                // Otherwise it's supported if we can map it
                const int expectedPixelDepth = (videoFormat & VIDEO_FORMAT_MASK_10BIT) ? 10 : 8;
                const int expectedLog2ChromaW = (videoFormat & VIDEO_FORMAT_MASK_YUV444) ? 0 : 1;
                const int expectedLog2ChromaH = (videoFormat & VIDEO_FORMAT_MASK_YUV444) ? 0 : 1;

                const AVPixFmtDescriptor* formatDesc = av_pix_fmt_desc_get(pixelFormat);
                if (!formatDesc) {
                    // This shouldn't be possible but handle it anyway
                    SDL_assert(formatDesc);
                    return false;
                }

                int planes = av_pix_fmt_count_planes(pixelFormat);
                return (planes == 2 || planes == 3) &&
                       formatDesc->comp[0].depth == expectedPixelDepth &&
                       formatDesc->log2_chroma_w == expectedLog2ChromaW &&
                       formatDesc->log2_chroma_h == expectedLog2ChromaH;
            }
        }
    }

    void refreshWindowMetadata()
    {
        // Get the current window from SDL if it wasn't passed in
        NSWindow* nswindow = getNSWindow();
        if (nswindow) {
            NSScreen* screen = [nswindow screen];
            if (!screen) {
                SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                            "Output display: not found");
                return;
            }

            m_MinRefreshInterval = screen.minimumRefreshInterval; // highest Hz
            m_MaxRefreshInterval = screen.maximumRefreshInterval; // lowest Hz

            bool isVRR = m_MinRefreshInterval != m_MaxRefreshInterval;
            m_IsFullScreen = ((nswindow.styleMask & NSWindowStyleMaskFullScreen) == NSWindowStyleMaskFullScreen);
            bool isProMotion = screen.displayUpdateGranularity > 0.0;

            if (isProMotion) {
                // ProMotion displays like MacBook Pro are detected as VRR but behave
                // badly since they can only operate at 120hz, 60hz, and a few lower rates.
                // This should also handle the low power use case where it will run at 60hz.
                SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                            "ProMotion display detected, treating as a fixed %.2f Hz display",
                            1.0f / m_MinRefreshInterval);
                isVRR = false;
            }

            m_UsingVRR.store(false);

            if (isVRR) {
                if (m_IsFullScreen) {
                    m_UsingVRR.store(true);
                    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                        "Output display: %s / VRR active, refresh range %.2f-%.2f Hz",
                        [screen.localizedName UTF8String],
                        1.0f / m_MaxRefreshInterval, 1.0f / m_MinRefreshInterval);
                }
                else {
                    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                        "Output display: %s @ %.2f Hz / VRR inactive (not fullscreen)",
                        [screen.localizedName UTF8String], 1.0f / m_MinRefreshInterval);
                }
            }
            else {
                SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "Output display: %s @ %.2f Hz fixed",
                    [screen.localizedName UTF8String], 1.0f / m_MinRefreshInterval);
            }
        }
    }

    bool notifyWindowChanged(PWINDOW_STATE_CHANGE_INFO info) override
    {
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "Metal renderer is handling window change: %dx%d on display %d",
                    info->width, info->height, info->displayIndex);

        refreshWindowMetadata();

        auto unhandledStateFlags = info->stateChangeFlags;

        // We can always handle size changes
        unhandledStateFlags &= ~WINDOW_STATE_CHANGE_SIZE;

        // We can handle monitor changes
        unhandledStateFlags &= ~WINDOW_STATE_CHANGE_DISPLAY;

        // If nothing is left, we handled everything
        return unhandledStateFlags == 0;
    }

    NSWindow* getNSWindow()
    {
        NSWindow *nswindow = nil;
        SDL_SysWMinfo info;
        SDL_VERSION(&info.version);
        if (SDL_GetWindowWMInfo(m_Window, &info) && info.subsystem == SDL_SYSWM_COCOA) {
            nswindow = (__bridge NSWindow *)info.info.cocoa.window;
        }
        return nswindow;
    }

    void setPaused(bool isPaused)
    {
        m_IsPaused.store(isPaused);

        if (isPaused) {
            SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Metal renderer paused, window not visible");
        }
        else {
            SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Metal renderer resumed");
        }
    }


private:
    bool m_HwAccel;
    SDL_Window* m_Window;
    AVBufferRef* m_HwContext;
    CAMetalLayer* m_MetalLayer;
    CAFrameRateRange m_FrameRateRange;
    dispatch_semaphore_t m_FrameReady;
    dispatch_semaphore_t m_VsyncReady;
    CVMetalTextureCacheRef m_TextureCache;
    id<MTLBuffer> m_CscParamsBuffer;
    id<MTLBuffer> m_VideoVertexBuffer;
    id<MTLTexture> m_OverlayTextures[Overlay::OverlayMax];
    SDL_SpinLock m_OverlayLock;
    MTLRenderPassDescriptor* m_RenderPassDescriptor;
    id<MTLRenderPipelineState> m_VideoPipelineState;
    id<MTLRenderPipelineState> m_OverlayPipelineState;
    id<MTLLibrary> m_ShaderLibrary;
    id<MTLCommandQueue> m_CommandQueue;
    uint32_t m_CurrentBuffer;
    id<MTLTexture> m_SwMappingTextures[MAX_FRAMES_IN_FLIGHT][MAX_VIDEO_PLANES];
    SDL_MetalView m_MetalView;
    int m_LastFrameWidth;
    int m_LastFrameHeight;
    int m_LastDrawableWidth;
    int m_LastDrawableHeight;

    int m_MaxFramesInFlight;
    int64_t m_lastPts;
    bool m_IsFullScreen;
    CFTimeInterval m_MinRefreshInterval;
    CFTimeInterval m_MaxRefreshInterval;
    std::atomic<double> m_LastPresented;
    std::atomic<double> m_LastGPUStartTime;
    std::atomic<bool> m_UsingVRR;
    std::atomic<bool> m_IsPaused;
    VTMetalObserver* m_Observer;
};

IFFmpegRenderer* VTMetalRendererFactory::createRenderer(bool hwAccel) {
    return new VTMetalRenderer(hwAccel);
}

@implementation VTMetalObserver {
    VTMetalRenderer* _renderer;
    id _note;
    id _note2;
}

- (id)initWithRenderer:(VTMetalRenderer *)renderer {
    self = [super init];
    if (self) {
        _renderer = renderer;

        void (^pauseIfHidden)(NSNotification*) = ^(NSNotification*) {
            NSWindow* window = _renderer->getNSWindow();
            bool shouldRender =
                window.isVisible &&
                !window.isMiniaturized &&
                (window.occlusionState & NSWindowOcclusionStateVisible) != 0;
            _renderer->setPaused( !shouldRender );
        };

        // Pause rendering when off screen or hidden
        _note = [[NSNotificationCenter defaultCenter] addObserverForName:NSWindowDidChangeOcclusionStateNotification
                                                                  object:nil queue:nil
                                                              usingBlock:pauseIfHidden];
        _note2 = [[NSNotificationCenter defaultCenter] addObserverForName:NSApplicationWillBecomeActiveNotification
                                                                  object:nil queue:nil
                                                              usingBlock:pauseIfHidden];

        // Some notifications we may want
        //NSApplicationShouldBeginSuppressingHighDynamicRangeContentNotification
        //NSApplicationShouldEndSuppressingHighDynamicRangeContentNotification
    }
    return self;
}

- (void)stop {
    if (_note) [[NSNotificationCenter defaultCenter] removeObserver:_note];
    if (_note2) [[NSNotificationCenter defaultCenter] removeObserver:_note2];
}

@end
