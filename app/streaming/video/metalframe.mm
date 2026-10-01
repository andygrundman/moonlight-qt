#include "metalframe.h"

#include <new>

namespace {
void freeMetalVideoFrame(void*, uint8_t* data)
{
    auto textures = reinterpret_cast<MetalVideoFrame*>(data);
    for (auto texture : textures->planes) {
        [texture release];
    }
    delete textures;
}
}

MetalVideoFramePool::MetalVideoFramePool(id<MTLDevice> device, int width, int height,
                                       bool yuv444, size_t capacity, bool tenBit)
    : m_Device([device retain]), m_Width(width), m_Height(height),
      m_Yuv444(yuv444), m_TenBit(tenBit), m_Capacity(capacity)
{
    m_Buffers.reserve(capacity);
}

MetalVideoFramePool::~MetalVideoFramePool()
{
    for (auto buffer : m_Buffers) {
        av_buffer_unref(&buffer);
    }
    [m_Device release];
}

bool MetalVideoFramePool::exhausted() const
{
    if (m_Buffers.size() < m_Capacity) {
        return false;
    }
    for (auto buffer : m_Buffers) {
        if (av_buffer_get_ref_count(buffer) == 1) {
            return false;
        }
    }
    return true;
}

AVFrame* MetalVideoFramePool::acquire()
{ @autoreleasepool {
    AVFrame* frame = av_frame_alloc();
    if (!frame) {
        return nullptr;
    }

    AVBufferRef* owner = nullptr;
    for (auto buffer : m_Buffers) {
        if (av_buffer_get_ref_count(buffer) == 1) {
            owner = buffer;
            break;
        }
    }

    if (!owner && m_Buffers.size() < m_Capacity) {
        auto textures = new (std::nothrow) MetalVideoFrame;
        if (textures) {
            for (int i = 0; i < 3; i++) {
                int divisor = i && !m_Yuv444 ? 2 : 1;
                auto desc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:(m_TenBit ? MTLPixelFormatR16Unorm : MTLPixelFormatR8Unorm)
                                                                             width:m_Width / divisor
                                                                            height:m_Height / divisor
                                                                         mipmapped:NO];
                desc.storageMode = MTLStorageModePrivate;
                desc.usage = MTLTextureUsageShaderRead | MTLTextureUsageShaderWrite;
                textures->planes[i] = [m_Device newTextureWithDescriptor:desc];
                if (!textures->planes[i]) {
                    freeMetalVideoFrame(nullptr, reinterpret_cast<uint8_t*>(textures));
                    textures = nullptr;
                    break;
                }
            }
        }
        if (textures) {
            owner = av_buffer_create(reinterpret_cast<uint8_t*>(textures), sizeof(*textures),
                                     freeMetalVideoFrame, nullptr, AV_BUFFER_FLAG_READONLY);
            if (owner) {
                m_Buffers.push_back(owner);
            }
            else {
                freeMetalVideoFrame(nullptr, reinterpret_cast<uint8_t*>(textures));
            }
        }
    }

    frame->buf[0] = owner ? av_buffer_ref(owner) : nullptr;
    if (!frame->buf[0]) {
        av_frame_free(&frame);
        return nullptr;
    }
    frame->opaque = frame->buf[0]->data;
    // The format describes the stream's bit depth for color conversion. Native
    // R16 textures hold normalized samples, rather than CPU YUV10's low bits.
    frame->format = m_TenBit ? (m_Yuv444 ? AV_PIX_FMT_YUV444P10LE : AV_PIX_FMT_YUV420P10LE) :
                             (m_Yuv444 ? AV_PIX_FMT_YUV444P : AV_PIX_FMT_YUV420P);
    frame->width = m_Width;
    frame->height = m_Height;
    return frame;
}}
