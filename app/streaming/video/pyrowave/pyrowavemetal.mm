#include "pyrowavemetal.h"
#include "pyrowavedecoder.h"
#include "path.h"
#include "streaming/video/ffmpeg-renderers/vt_colorspace.h"
#include <pyrowave_metal.h>
#include <SDL.h>
#include <array>
#include <atomic>
#include <deque>
#include <mutex>
#include <new>
#include <simd/simd.h>

namespace {
struct SurfacePool {
    std::mutex lock;
    std::array<std::array<id<MTLTexture>, 3>, 8> textures = {};
    std::deque<unsigned> available;
    ~SurfacePool() {
        for (auto& planes : textures) for (auto texture : planes) [texture release];
    }
    bool acquire(unsigned& index) {
        std::lock_guard<std::mutex> guard(lock);
        if (available.empty()) return false;
        index = available.front(); available.pop_front(); return true;
    }
    void release(unsigned index) {
        std::lock_guard<std::mutex> guard(lock);
        available.push_back(index);
    }
};
struct FrameOwner {
    std::atomic<unsigned> references {1};
    PyroWaveMetalFrame frame;
    std::shared_ptr<SurfacePool> pool;
    unsigned index;
};
void releaseOwner(FrameOwner* owner) {
    if (owner->references.fetch_sub(1, std::memory_order_acq_rel) != 1) return;
    [owner->frame.completion release];
    [owner->frame.readyEvent release];
    owner->pool->release(owner->index);
    delete owner;
}
void releaseFrame(void* opaque, uint8_t*) {
    releaseOwner(static_cast<FrameOwner*>(opaque));
}

void logMessage(void*, const char* message) {
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "PyroWave Metal: %s", message);
}
}

bool pyroWaveMetalSupported() { @autoreleasepool {
    auto device = MTLCreateSystemDefaultDevice();
    bool supported = pyrowave_device_is_supported((void*)device);
    [device release];
    return supported;
}}

PyroWaveMetalFrame* pyroWaveMetalFrame(const AVFrame* frame) {
    if (!frame || !frame->buf[0] || frame->buf[0]->size != sizeof(PyroWaveMetalFrame) ||
        frame->data[0] != frame->buf[0]->data) return nullptr;
    auto ref = reinterpret_cast<PyroWaveMetalFrame*>(frame->buf[0]->data);
    return ref->magic == UINT64_C(0x50574d4554414c31) ? ref : nullptr;
}
bool pyroWaveMetalWait(const PyroWaveMetalFrame* frame) {
    [frame->completion waitUntilCompleted];
    if (frame->completion.status != MTLCommandBufferStatusCompleted) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PyroWave Metal decode failed: %s",
                     frame->completion.error.localizedDescription.UTF8String);
        return false;
    }
    return true;
}

struct PyroWaveMetalDecoder::Impl {
    Config config;
    pyrowave_device device = nullptr;
    pyrowave_decoder decoder = nullptr;
    id<MTLCommandQueue> queue = nil;
    id<MTLCommandBuffer> lastSubmission = nil;
    id<MTLSharedEvent> readyEvent = nil;
    uint64_t readyValue = 0;
    std::shared_ptr<SurfacePool> pool;
    ~Impl() {
        [lastSubmission waitUntilCompleted];
        [lastSubmission release];
        if (decoder) pyrowave_decoder_destroy(decoder);
        if (device) pyrowave_device_destroy(device);
        [queue release];
        [readyEvent release];
    }
};
PyroWaveMetalDecoder::PyroWaveMetalDecoder() = default;
PyroWaveMetalDecoder::~PyroWaveMetalDecoder() = default;

bool PyroWaveMetalDecoder::initialize(const Config& config) { @autoreleasepool {
    m_Impl.reset(); m_LastError.clear();
    auto impl = std::make_unique<Impl>();
    impl->config = config;
    auto device = (id<MTLDevice>)config.metalDevice;
    if (!pyrowave_device_is_supported((void*)device)) {
        m_LastError = "PyroWave requires an Apple Silicon Metal GPU";
        return false;
    }
    pyrowave_device_create_info deviceInfo = {};
    deviceInfo.mtl_device = (void*)device;
    deviceInfo.message_callback = logMessage;
    auto result = pyrowave_device_create(&deviceInfo, &impl->device);
    if (result != PYROWAVE_SUCCESS) {
        m_LastError = pyrowave_result_to_string(result); return false;
    }
    pyrowave_decoder_create_info decoderInfo = {};
    decoderInfo.device = impl->device;
    decoderInfo.width = config.width; decoderInfo.height = config.height;
    decoderInfo.chroma = config.chroma444 ? PYROWAVE_CHROMA_SUBSAMPLING_444 : PYROWAVE_CHROMA_SUBSAMPLING_420;
    result = pyrowave_decoder_create(&decoderInfo, &impl->decoder);
    if (result != PYROWAVE_SUCCESS) {
        m_LastError = pyrowave_result_to_string(result); return false;
    }
    impl->queue = [device newCommandQueue];
    impl->readyEvent = [device newSharedEvent];
    impl->pool = std::make_shared<SurfacePool>();
    if (!impl->queue || !impl->readyEvent) { m_LastError = "Could not create Metal decode queue"; return false; }
    for (unsigned i = 0; i < impl->pool->textures.size(); ++i) {
        for (int plane = 0; plane < 3; ++plane) {
            const int divisor = plane && !config.chroma444 ? 2 : 1;
            auto descriptor = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:
                config.tenBit ? MTLPixelFormatR16Unorm : MTLPixelFormatR8Unorm
                width:config.width / divisor height:config.height / divisor mipmapped:NO];
            descriptor.storageMode = MTLStorageModePrivate;
            descriptor.usage = MTLTextureUsageShaderWrite | MTLTextureUsageShaderRead;
            impl->pool->textures[i][plane] = [device newTextureWithDescriptor:descriptor];
            if (!impl->pool->textures[i][plane]) {
                m_LastError = "Could not allocate Metal video planes"; return false;
            }
        }
        impl->pool->available.push_back(i);
    }
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "PyroWave decoder ready: native Metal on %s, %dx%d %s %d-bit",
        device.name.UTF8String, config.width, config.height, config.chroma444 ? "4:4:4" : "4:2:0", config.tenBit ? 10 : 8);
    m_Impl = std::move(impl);
    return true;
}}

bool PyroWaveMetalDecoder::decode(const uint8_t* data, size_t size,
    const std::vector<PyroWaveFraming::Segment>& packets, size_t criticalPackets,
    AVFrame* frame) { @autoreleasepool {
    m_LastError.clear();
    if (!m_Impl || !frame) { m_LastError = "Decoder is not initialized"; return false; }
    auto& impl = *m_Impl;
    PyroWaveFraming::Frame parsed;
    if (!PyroWaveFraming::parse(data, size, packets, criticalPackets,
        {impl.config.width, impl.config.height, impl.config.chroma444}, parsed, m_LastError)) return false;
    pyrowave_decoder_clear(impl.decoder);
    for (const auto& span : parsed.spans) {
        auto result = pyrowave_decoder_push_packet(impl.decoder, data + span.offset, span.size);
        if (result != PYROWAVE_SUCCESS) {
            m_LastError = std::string("Decoder rejected packet: ") + pyrowave_result_to_string(result); return false;
        }
    }
    if (parsed.partial ? (!parsed.coarseLevelIntact ||
        !pyrowave_decoder_decode_is_ready_with_sideband(impl.decoder, true, 0, 0.0f, nullptr, 0)) :
        !pyrowave_decoder_decode_is_ready(impl.decoder, false)) {
        m_LastError = "Frame is incomplete or missing the coarsest wavelet level"; return false;
    }
    unsigned index;
    if (!impl.pool->acquire(index)) { m_LastError = "All Metal video surfaces are in use"; return false; }
    auto owner = new (std::nothrow) FrameOwner;
    if (!owner) { impl.pool->release(index); m_LastError = "Out of memory"; return false; }
    owner->pool = impl.pool; owner->index = index;
    for (int i = 0; i < 3; ++i) owner->frame.textures[i] = impl.pool->textures[index][i];
    owner->frame.completion = [[impl.queue commandBuffer] retain];
    auto buffer = av_buffer_create(reinterpret_cast<uint8_t*>(&owner->frame), sizeof(owner->frame),
                                  releaseFrame, owner, AV_BUFFER_FLAG_READONLY);
    if (!buffer || !owner->frame.completion) {
        if (buffer) av_buffer_unref(&buffer); else releaseFrame(owner, nullptr);
        m_LastError = "Could not allocate frame completion handle"; return false;
    }
    pyrowave_gpu_buffers output = {};
    for (int i = 0; i < 3; ++i) output.planes[i] = (void*)owner->frame.textures[i];
    auto result = pyrowave_decoder_decode_gpu_buffer(impl.decoder, (void*)owner->frame.completion, &output);
    if (result != PYROWAVE_SUCCESS) {
        av_buffer_unref(&buffer); m_LastError = pyrowave_result_to_string(result); return false;
    }
    owner->frame.readyEvent = [impl.readyEvent retain];
    owner->frame.readyValue = ++impl.readyValue;
    [owner->frame.completion encodeSignalEvent:owner->frame.readyEvent value:owner->frame.readyValue];
    // Keep the planes alive if the frame pacer drops the AVFrame during decode.
    owner->references.fetch_add(1, std::memory_order_relaxed);
    [owner->frame.completion addCompletedHandler:^(id<MTLCommandBuffer>) { releaseOwner(owner); }];
    [owner->frame.completion commit];
    [impl.lastSubmission release];
    impl.lastSubmission = [owner->frame.completion retain];
    av_frame_unref(frame);
    frame->buf[0] = buffer; frame->data[0] = buffer->data;
    frame->width = impl.config.width; frame->height = impl.config.height;
    frame->format = impl.config.tenBit ?
        (impl.config.chroma444 ? AV_PIX_FMT_YUV444P10 : AV_PIX_FMT_YUV420P10) :
        (impl.config.chroma444 ? AV_PIX_FMT_YUV444P : AV_PIX_FMT_YUV420P);
    frame->chroma_location = AVCHROMA_LOC_CENTER;
    frame->flags |= AV_FRAME_FLAG_KEY;
    return true;
}}

#ifdef PYROWAVE_METAL_TEST
struct PyroWaveMetalCalibrationRenderer::Impl {
    id<MTLDevice> device = nil;
    id<MTLCommandQueue> queue = nil;
    id<MTLLibrary> library = nil;
    id<MTLRenderPipelineState> pipeline = nil;
    id<MTLTexture> target = nil;
    int displayWidth = 0, displayHeight = 0;
    ~Impl() { [target release]; [pipeline release]; [library release]; [queue release]; [device release]; }
};
PyroWaveMetalCalibrationRenderer::PyroWaveMetalCalibrationRenderer() : m_Impl(new Impl) {}
PyroWaveMetalCalibrationRenderer::~PyroWaveMetalCalibrationRenderer() = default;
void* PyroWaveMetalCalibrationRenderer::device() const { return (void*)m_Impl->device; }
bool PyroWaveMetalCalibrationRenderer::create(int width, int height) { @autoreleasepool {
    auto& impl = *m_Impl;
    impl.displayWidth = width; impl.displayHeight = height;
    impl.device = MTLCreateSystemDefaultDevice();
    if (!pyrowave_device_is_supported((void*)impl.device)) return false;
    impl.queue = [impl.device newCommandQueue];
    QString source = QString::fromUtf8(Path::readDataFile("vt_renderer.metal"));
    impl.library = [impl.device newLibraryWithSource:source.toNSString() options:nil error:nil];
    return impl.queue && impl.library;
}}
bool PyroWaveMetalCalibrationRenderer::prepare(int width, int height, bool, bool hdr) { @autoreleasepool {
    auto& impl = *m_Impl;
    [impl.target release]; impl.target = nil;
    [impl.pipeline release]; impl.pipeline = nil;
    auto texture = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:
        hdr ? MTLPixelFormatBGR10A2Unorm : MTLPixelFormatBGRA8Unorm
        width:impl.displayWidth > 0 ? impl.displayWidth : width
        height:impl.displayHeight > 0 ? impl.displayHeight : height mipmapped:NO];
    texture.storageMode = MTLStorageModePrivate; texture.usage = MTLTextureUsageRenderTarget;
    impl.target = [impl.device newTextureWithDescriptor:texture];
    auto descriptor = [[MTLRenderPipelineDescriptor alloc] init];
    descriptor.vertexFunction = [[impl.library newFunctionWithName:@"vs_draw"] autorelease];
    descriptor.fragmentFunction = [[impl.library newFunctionWithName:@"ps_draw_triplanar"] autorelease];
    descriptor.colorAttachments[0].pixelFormat = texture.pixelFormat;
    impl.pipeline = [impl.device newRenderPipelineStateWithDescriptor:descriptor error:nil];
    [descriptor release];
    return impl.target && impl.pipeline;
}}
bool PyroWaveMetalCalibrationRenderer::present(AVFrame* frame, bool hdr) { @autoreleasepool {
    auto ref = pyroWaveMetalFrame(frame);
    if (!ref) return false;
    auto& impl = *m_Impl;
    struct Vertex { simd_float4 position; simd_float2 uv; };
    const Vertex vertices[] = {{{-1, -1, 0, 1}, {0, 1}}, {{1, -1, 0, 1}, {1, 1}},
                              {{-1, 1, 0, 1}, {0, 0}}, {{1, 1, 0, 1}, {1, 0}}};
    VTMetalCscParams params = {};
    // Match the stream's limited-range BT.709/BT.2020 conversion.
    const int bits = hdr ? 10 : 8, range = 1 << bits, factor = 1 << (bits - 8);
    const float yScale = float(range - 1) / (219 * factor);
    const float uvScale = float(range - 1) / (224 * factor);
    params.matrix = simd_matrix(simd_make_float3(yScale, 0, (hdr ? 1.4746f : 1.5748f) * uvScale),
        simd_make_float3(yScale, (hdr ? -0.1646f : -0.1873f) * uvScale, (hdr ? -0.5714f : -0.4681f) * uvScale),
        simd_make_float3(yScale, (hdr ? 1.8814f : 1.8556f) * uvScale, 0));
    params.offsets = simd_make_float3(float(16 * factor) / (range - 1),
        float(range / 2) / (range - 1), float(range / 2) / (range - 1));
    params.bitnessScaleFactor = 1;
    auto pass = [MTLRenderPassDescriptor renderPassDescriptor];
    pass.colorAttachments[0].texture = impl.target;
    pass.colorAttachments[0].loadAction = MTLLoadActionDontCare;
    pass.colorAttachments[0].storeAction = MTLStoreActionStore;
    auto command = [impl.queue commandBuffer];
    [command encodeWaitForEvent:ref->readyEvent value:ref->readyValue];
    auto encoder = [command renderCommandEncoderWithDescriptor:pass];
    [encoder setRenderPipelineState:impl.pipeline];
    [encoder setVertexBytes:vertices length:sizeof(vertices) atIndex:0];
    [encoder setFragmentBytes:&params length:sizeof(params) atIndex:0];
    for (int i = 0; i < 3; ++i) [encoder setFragmentTexture:ref->textures[i] atIndex:i];
    [encoder drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4];
    [encoder endEncoding]; [command commit]; [command waitUntilCompleted];
    return command.status == MTLCommandBufferStatusCompleted;
}}

#endif
