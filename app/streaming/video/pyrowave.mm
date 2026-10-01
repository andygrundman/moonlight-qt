#include "pyrowave.h"

#ifdef HAVE_PYROWAVE

#include "pyrowavebitstream.h"
#include "pyrowavecolor.h"
#include "metalframe.h"
#include "ffmpeg-renderers/vt.h"
#include "ffmpeg-renderers/framepacing/framepacer.h"
#include "streaming/session.h"
#include "utils.h"
#include <pyrowave_metal.h>

#include <mutex>

namespace {
// Five queued frames, one current frame, up to three GPU readers, one decode.
constexpr size_t OutputSurfaceCount = 10;
constexpr int FailedDecodesResetThreshold = 20;

void logPyroWave(void*, const char* message)
{
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "PyroWave: %s", message);
}

struct PyroWaveDevice {
    pyrowave_device handle = nullptr;
    uint64_t registryId = 0;
    ~PyroWaveDevice() { pyrowave_device_destroy(handle); }
};

std::shared_ptr<PyroWaveDevice> getSharedDevice(id<MTLDevice> mtlDevice)
{
    // Pipeline compilation is expensive; preserve it across probes and reconnects.
    static std::mutex lock;
    static std::shared_ptr<PyroWaveDevice> cached;
    std::lock_guard<std::mutex> guard(lock);
    if (cached && cached->registryId == mtlDevice.registryID) {
        return cached;
    }
    auto device = std::make_shared<PyroWaveDevice>();
    pyrowave_device_create_info info = {};
    info.mtl_device = (void*)mtlDevice;
    info.message_callback = logPyroWave;
    auto result = pyrowave_device_create(&info, &device->handle);
    if (result != PYROWAVE_SUCCESS) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PyroWave device creation failed: %s",
                     pyrowave_result_to_string(result));
        return nullptr;
    }
    device->registryId = mtlDevice.registryID;
    cached = device;
    return device;
}
}

struct PyroWaveVideoDecoder::Impl {
    explicit Impl(bool testing) : testOnly(testing) {}
    ~Impl()
    {
        pyrowave_decoder_destroy(decoder);
        [queue release];
        delete renderer;
    }

    bool decode(AVFrame* frame)
    {
        auto textures = getMetalVideoFrame(frame);
        pyrowave_gpu_buffers buffers = {};
        for (int i = 0; i < 3; i++) {
            buffers.planes[i] = (void*)textures->planes[i];
        }
        auto commandBuffer = [queue commandBuffer];
        if (!commandBuffer) {
            return false;
        }
        commandBuffer.label = @"PyroWave decode";
        auto result = pyrowave_decoder_decode_gpu_buffer(decoder, (void*)commandBuffer, &buffers);
        // Drain even partially encoded work before parser/scratch buffers are reused.
        [commandBuffer commit];
        [commandBuffer waitUntilCompleted];
        if (result != PYROWAVE_SUCCESS || commandBuffer.status != MTLCommandBufferStatusCompleted) {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PyroWave GPU decode failed: %s (%s)",
                         pyrowave_result_to_string(result),
                         commandBuffer.error.localizedDescription.UTF8String ?: "no GPU error");
            return false;
        }
        return true;
    }

    bool testOnly;
    bool pacerInitialized = false;
    bool overlayAttached = false;
    IFFmpegRenderer* renderer = nullptr;
    std::shared_ptr<PyroWaveDevice> device;
    pyrowave_decoder decoder = nullptr;
    id<MTLCommandQueue> queue = nil;
    std::unique_ptr<MetalVideoFramePool> pool;
    std::vector<uint32_t> bitstream;
    std::vector<PyroWaveFraming::Segment> packets;
    PyroWaveFraming::Frame parsed;
    SDL_Thread* thread = nullptr;
    SDL_atomic_t stopping = {};
    uint32_t lastFrameNumber = 0;
    bool haveLastFrameNumber = false;
    int failedDecodes = 0;
    int colorRange = COLOR_RANGE_FULL;
    int width = 0;
    int height = 0;
    bool yuv444 = false;
    bool tenBit = false;
    uint64_t nextPartialLogTimeUs = 0;
    unsigned partialFrames = 0;
};

PyroWaveVideoDecoder::PyroWaveVideoDecoder(bool testOnly)
    : m_Impl(new Impl(testOnly))
{
}

PyroWaveVideoDecoder::~PyroWaveVideoDecoder()
{
    auto& impl = *m_Impl;
    if (impl.thread) {
        SDL_AtomicSet(&impl.stopping, 1);
        LiWakeWaitForVideoFrame();
        SDL_WaitThread(impl.thread, nullptr);
    }
    if (impl.pacerInitialized) {
        FramePacer::instance().deinit();
    }
    if (impl.overlayAttached) {
        Session::get()->getOverlayManager().setOverlayRenderer(nullptr);
        Stats::instance().LogGlobalVideoStats();
    }
    // Impl frees the renderer after the pacer has relinquished its frames. GPU
    // readers hold independent buffer references and never touch the decoder.
}

bool PyroWaveVideoDecoder::initialize(PDECODER_PARAMETERS params)
{ @autoreleasepool {
    auto& impl = *m_Impl;
    if ((params->videoFormat != VIDEO_FORMAT_PYROWAVE && params->videoFormat != VIDEO_FORMAT_PYROWAVE_444 &&
            params->videoFormat != VIDEO_FORMAT_PYROWAVE10_420 && params->videoFormat != VIDEO_FORMAT_PYROWAVE10_444) ||
            params->vds == StreamingPreferences::VDS_FORCE_SOFTWARE ||
            params->width <= 0 || params->height <= 0 ||
            params->width > 16384 || params->height > 16384 ||
            impl.renderer || params->testOnly != impl.testOnly) {
        return false;
    }
    bool yuv444 = (params->videoFormat & VIDEO_FORMAT_MASK_YUV444) != 0;
    if (!yuv444 && ((params->width | params->height) & 1)) {
        return false;
    }
    impl.width = params->width;
    impl.height = params->height;
    impl.yuv444 = yuv444;
    impl.tenBit = (params->videoFormat & VIDEO_FORMAT_MASK_10BIT) != 0;
    impl.renderer = VTMetalRendererFactory::createTextureRenderer();
    if (!impl.renderer->initialize(params)) {
        return false;
    }
    auto mtlDevice = (id<MTLDevice>)VTMetalRendererFactory::getDevice(impl.renderer);
    if (!pyrowave_device_is_supported((void*)mtlDevice)) {
        SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "PyroWave requires an Apple7 or newer Metal GPU");
        return false;
    }
    impl.device = getSharedDevice(mtlDevice);
    if (!impl.device) {
        return false;
    }
    pyrowave_decoder_create_info info = {};
    info.device = impl.device->handle;
    info.width = params->width;
    info.height = params->height;
    info.chroma = yuv444 ? PYROWAVE_CHROMA_SUBSAMPLING_444 : PYROWAVE_CHROMA_SUBSAMPLING_420;
    auto result = pyrowave_decoder_create(&info, &impl.decoder);
    if (result != PYROWAVE_SUCCESS) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PyroWave decoder creation failed: %s",
                     pyrowave_result_to_string(result));
        return false;
    }
    impl.queue = [mtlDevice newCommandQueue];
    if (!impl.queue) {
        return false;
    }
    impl.pool.reset(new MetalVideoFramePool(mtlDevice, params->width, params->height, yuv444,
                                           OutputSurfaceCount, impl.tenBit));
    // The override helper writes zero even when the variable is absent or invalid.
    // Preserve full range in that case so negotiation and frame metadata agree.
    if (!Utils::getEnvironmentVariableOverride("COLOR_RANGE_OVERRIDE", &impl.colorRange)) {
        impl.colorRange = COLOR_RANGE_FULL;
    }

    // A complete frame with no active wavelet blocks decodes to neutral gray.
    // Exercise real GPU decoding at the requested dimensions without a connection.
    uint32_t testFrame[2] = {
        uint32_t(params->width - 1) | (uint32_t(params->height - 1) << 14) | 0x80000000u,
        yuv444 ? (1u << 26) : 0u,
    };
    AVFrame* frame = impl.pool->acquire();
    if (!frame) {
        return false;
    }
    result = pyrowave_decoder_push_packet(impl.decoder, testFrame, sizeof(testFrame));
    bool success = result == PYROWAVE_SUCCESS &&
                   pyrowave_decoder_decode_is_ready(impl.decoder, false) &&
                   impl.decode(frame) && impl.renderer->testRenderFrame(frame);
    av_frame_free(&frame);
    pyrowave_decoder_clear(impl.decoder);
    if (!success || impl.testOnly) {
        return success;
    }

    Session::get()->getOverlayManager().setOverlayRenderer(impl.renderer);
    impl.overlayAttached = true;
    impl.renderer->prepareToRender();
    impl.pacerInitialized = true; // initialization may partially acquire resources
    if (!FramePacer::instance().initialize(impl.renderer, params)) {
        return false;
    }
    impl.thread = SDL_CreateThread(decoderThread, "PyroWaveDecoder", this);
    return impl.thread != nullptr;
}}

int PyroWaveVideoDecoder::decoderThread(void* context)
{
    auto decoder = static_cast<PyroWaveVideoDecoder*>(context);
    auto& impl = *decoder->m_Impl;
    while (!SDL_AtomicGet(&impl.stopping)) {
        VIDEO_FRAME_HANDLE handle;
        PDECODE_UNIT du;
        if (LiWaitForNextVideoFrame(&handle, &du)) {
            int result = SDL_AtomicGet(&impl.stopping) ? DR_OK : decoder->submitDecodeUnit(du);
            LiCompleteVideoFrame(handle, result);
        }
    }
    return 0;
}

int PyroWaveVideoDecoder::dropFrame(const char* reason)
{
    SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "PyroWave frame dropped: %s", reason);
    pyrowave_decoder_clear(m_Impl->decoder);
    Stats::instance().SubmitDroppedFrame(1);
    // Every frame is independent. A dropped frame needs neither an IDR request
    // nor a renderer reset, even when several network frames are malformed.
    return DR_OK;
}

int PyroWaveVideoDecoder::decodeFailed(const char* reason)
{
    auto& impl = *m_Impl;
    SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "PyroWave decode failed: %s", reason);
    pyrowave_decoder_clear(impl.decoder);
    Stats::instance().SubmitDroppedFrame(1);
    if (++impl.failedDecodes == FailedDecodesResetThreshold) {
        SDL_Event event = {};
        event.type = SDL_RENDER_DEVICE_RESET;
        SDL_PushEvent(&event);
        SDL_AtomicSet(&impl.stopping, 1);
    }
    return DR_OK;
}

int PyroWaveVideoDecoder::submitDecodeUnit(PDECODE_UNIT du)
{ @autoreleasepool {
    auto& impl = *m_Impl;
    SDL_assert(!impl.testOnly);
    uint32_t dropped = 0;
    uint32_t delta = uint32_t(du->frameNumber) - impl.lastFrameNumber;
    if (impl.haveLastFrameNumber && delta > 1 && delta < 0x80000000u) {
        dropped = delta - 1;
    }
    impl.lastFrameNumber = du->frameNumber;
    impl.haveLastFrameNumber = true;
    Stats::instance().SubmitVideoBytesAndReassemblyTime(du, dropped);
    auto& overlay = Session::get()->getOverlayManager();
    if (Stats::instance().ShouldUpdateDisplay(overlay.isOverlayEnabled(Overlay::OverlayDebug),
                                             overlay.getOverlayText(Overlay::OverlayDebug),
                                             overlay.getOverlayMaxTextLength())) {
        overlay.setOverlayTextUpdated(Overlay::OverlayDebug);
    }
    if (du->hdrActive && !impl.tenBit) {
        return dropFrame("HDR frame received on an 8-bit stream");
    }
    if (!assemblePyroWaveDecodeUnit(du, impl.bitstream, &impl.packets)) {
        return dropFrame("invalid frame framing");
    }
    std::string parseError;
    if (!PyroWaveFraming::parse(reinterpret_cast<const uint8_t*>(impl.bitstream.data()),
                               impl.bitstream.size() * sizeof(uint32_t), impl.packets,
                               du->pyrowaveCriticalPackets, {impl.width, impl.height, impl.yuv444},
                               impl.parsed, parseError)) {
        return dropFrame(parseError.c_str());
    }
    if (impl.parsed.partial && !impl.parsed.coarseLevelIntact) {
        return dropFrame("part of the coarsest wavelet level was lost");
    }
    // Each Moonlight DU represents an independent frame. Reset the three-bit sequence
    // tracker so network gaps cannot make a newer frame look like an old packet.
    pyrowave_decoder_clear(impl.decoder);
    for (const auto& span : impl.parsed.spans) {
        auto result = pyrowave_decoder_push_packet(impl.decoder,
                         reinterpret_cast<const uint8_t*>(impl.bitstream.data()) + span.offset, span.size);
        if (result != PYROWAVE_SUCCESS) {
            return dropFrame("codec rejected a record");
        }
    }
    // Match vrr18: an intact coarse prefix is sufficient; missing finer blocks
    // become zero coefficients. The codec's pristine-band check cannot tell a
    // lost block from a zero block the encoder never sent.
    bool ready = impl.parsed.partial ?
        pyrowave_decoder_decode_is_ready_with_sideband(impl.decoder, true, 0, 0.0f, nullptr, 0) :
        pyrowave_decoder_decode_is_ready(impl.decoder, false);
    if (!ready) {
        return dropFrame("corrupt or incomplete bitstream");
    }
    AVFrame* frame = impl.pool->acquire();
    if (!frame) {
        if (impl.pool->exhausted()) {
            Stats::instance().SubmitDroppedFrame(1);
            return DR_OK; // all frames are independent; no keyframe is necessary
        }
        return decodeFailed("output frame allocation failed");
    }
    if (!impl.decode(frame)) {
        av_frame_free(&frame);
        return decodeFailed("GPU command failed");
    }
    impl.failedDecodes = 0;
    frame->pts = du->rtpTimestamp;
    frame->pkt_dts = LiGetMicroseconds();
    if (impl.parsed.partial) {
        impl.partialFrames++;
        if (uint64_t(frame->pkt_dts) >= impl.nextPartialLogTimeUs) {
            SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "PyroWave decoded %u partial frames; latest retained %u/%u block records",
                        impl.partialFrames, impl.parsed.blockRecords, impl.parsed.announcedBlocks);
            impl.partialFrames = 0;
            impl.nextPartialLogTimeUs = frame->pkt_dts + 1000000;
        }
    }
    // Leave pict_type unset: every frame is independent, but the pacing queue
    // gives I-frames special treatment which would defeat normal queue limits.
    setPyroWaveFrameColors(frame, du, impl.colorRange);
    Stats::instance().SubmitDecodeTimeUs(frame->pkt_dts - du->enqueueTimeUs);
    // DU metadata stays valid until the pull loop completes the handle.
    FramePacer::instance().submitFrame(frame, du);
    return DR_OK;
}}

void PyroWaveVideoDecoder::renderFrameOnMainThread()
{
    FramePacer::instance().renderOnMainThread();
}

void PyroWaveVideoDecoder::setHdrMode(bool enabled)
{
    if (m_Impl->renderer) {
        m_Impl->renderer->setHdrMode(enabled && m_Impl->tenBit);
    }
}

bool PyroWaveVideoDecoder::isHdrSupported()
{
    return m_Impl->renderer &&
           (m_Impl->renderer->getRendererAttributes() & RENDERER_ATTRIBUTE_HDR_SUPPORT);
}

int PyroWaveVideoDecoder::getDecoderColorRange()
{
    return m_Impl->colorRange;
}

bool PyroWaveVideoDecoder::notifyWindowChanged(PWINDOW_STATE_CHANGE_INFO info)
{
    return m_Impl->renderer->notifyWindowChanged(info);
}

#endif
