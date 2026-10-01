#pragma once

#import <Metal/Metal.h>

#include <vector>

extern "C" {
#include <libavutil/frame.h>
}

// Native textures are owned by buf[0]. opaque_ref belongs to the frame pacer.
struct MetalVideoFrame {
    static constexpr uint64_t Magic = 0x4d4c4d4554414c31;
    uint64_t magic = Magic;
    id<MTLTexture> planes[3] = {};
};

inline MetalVideoFrame* getMetalVideoFrame(const AVFrame* frame)
{
    if (!frame || !frame->buf[0] || frame->buf[0]->size != sizeof(MetalVideoFrame) ||
            frame->opaque != frame->buf[0]->data) {
        return nullptr;
    }
    auto textures = reinterpret_cast<MetalVideoFrame*>(frame->buf[0]->data);
    return textures->magic == MetalVideoFrame::Magic ? textures : nullptr;
}

// Only the decode thread acquires frames. AVBuffer references held by the pacer
// and render command buffers prevent reuse until all consumers have finished.
class MetalVideoFramePool {
public:
    MetalVideoFramePool(id<MTLDevice> device, int width, int height, bool yuv444,
                        size_t capacity, bool tenBit = false);
    ~MetalVideoFramePool();
    MetalVideoFramePool(const MetalVideoFramePool&) = delete;
    MetalVideoFramePool& operator=(const MetalVideoFramePool&) = delete;

    AVFrame* acquire();
    bool exhausted() const;

private:
    id<MTLDevice> m_Device;
    int m_Width;
    int m_Height;
    bool m_Yuv444;
    bool m_TenBit;
    size_t m_Capacity;
    std::vector<AVBufferRef*> m_Buffers;
};
