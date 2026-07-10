/**
 * @file app/streaming/video/pyrowave.h
 * @brief PyroWave (Vulkan wavelet, intra-only) client decoder for Moonlight.
 *
 * Stage B4 (GPU zero-copy present): PyroWave decodes on its own (headless) Vulkan device into
 * exportable dmabuf plane images; those planes are imported into a libplacebo Vulkan device which
 * does YUV->RGB + swapchain present. No CPU readback.
 *
 * Compiled only when HAVE_PYROWAVE is defined.
 */
#pragma once

#ifdef HAVE_PYROWAVE

  #include "decoder.h"
  #include "overlaymanager.h"
  #include "../bandwidth.h"

  #include <SDL.h>
  #include <SDL_vulkan.h>
  #include <vulkan/vulkan.h>

  #include <libplacebo/vulkan.h>
  #include <libplacebo/renderer.h>

  #include <atomic>
  #include <mutex>
  #include <vector>

struct pyrowave_device_opaque;
struct pyrowave_decoder_opaque;
struct pyrowave_sync_object_opaque;

class PyroWaveVideoDecoder : public IVideoDecoder, public Overlay::IOverlayRenderer {
public:
    PyroWaveVideoDecoder(bool testOnly);
    virtual ~PyroWaveVideoDecoder() override;

    virtual bool initialize(PDECODER_PARAMETERS params) override;
    virtual bool isHardwareAccelerated() override;
    virtual bool isAlwaysFullScreen() override;
    virtual bool isHdrSupported() override;
    virtual int getDecoderCapabilities() override;
    virtual int getDecoderColorspace() override;
    virtual int getDecoderColorRange() override;
    virtual QSize getDecoderMaxResolution() override;
    virtual int submitDecodeUnit(PDECODE_UNIT du) override;
    virtual void renderFrameOnMainThread() override;
    virtual void setHdrMode(bool enabled) override;
    virtual bool notifyWindowChanged(PWINDOW_STATE_CHANGE_INFO info) override;

    // Overlay::IOverlayRenderer
    virtual void notifyOverlayUpdated(Overlay::OverlayType type) override;

private:
    bool createOverlay(pl_overlay* overlay, SDL_Surface* surface);
    static void overlayUploadComplete(void* opaque);
    void addVideoStats(VIDEO_STATS& src, VIDEO_STATS& dst);
    void stringifyVideoStats(VIDEO_STATS& stats, char* output, int length);
    // One exportable R8 decode-plane image on PyroWave's device, imported into libplacebo.
    struct Plane {
        VkImage image = VK_NULL_HANDLE;
        VkDeviceMemory mem = VK_NULL_HANDLE;
        VkImageView view = VK_NULL_HANDLE;   // for PyroWave decode target
        int w = 0, h = 0;
        VkDeviceSize size = 0;
        uint64_t modifier = 0;
        VkDeviceSize offset = 0, pitch = 0;
        int fd = -1;
        pl_tex plTex = nullptr;              // libplacebo import of the same dmabuf
    };

    bool createPyroDevice();
    bool createPlanes();
    bool createLibplacebo(PDECODER_PARAMETERS params);
    bool importPlanes();
    bool createSharedTimeline();  // libplacebo-created timeline sem, imported into PyroWave

    bool m_TestOnly;
    int m_Width;
    int m_Height;
    bool m_YUV444;
    bool m_TenBit;
    std::atomic<bool> m_HdrEnabled;      // host HDR state (control stream); read on render thread
    pl_color_space m_LastColorspace;     // render thread only; drives swapchain colorspace hints
    SDL_Window* m_Window;

    // PyroWave decode side (its own headless device).
    pyrowave_device_opaque* m_PyroDevice;
    pyrowave_decoder_opaque* m_Decoder;
    VkDevice m_VkDev;
    VkPhysicalDevice m_VkPhys;
    VkQueue m_VkQueue;
    uint32_t m_VkFamily;
    VkCommandPool m_VkPool;
    VkCommandBuffer m_VkCmd;
    Plane m_Planes[3];
    PFN_vkGetMemoryFdKHR m_GetMemoryFd;
    PFN_vkGetImageDrmFormatModifierPropertiesEXT m_GetImgMod;

    // libplacebo present side (its own device + swapchain on the window surface).
    pl_log m_Log;
    pl_vk_inst m_PlVkInstance;
    VkSurfaceKHR m_VkSurface;
    pl_vulkan m_Vulkan;
    pl_swapchain m_Swapchain;
    pl_renderer m_Renderer;
    PFN_vkDestroySurfaceKHR m_DestroySurface;

    // Shared timeline semaphore for cross-device decode->present sync (replaces vkDeviceWaitIdle).
    // Created+exported by libplacebo, imported into PyroWave; both signal/wait the same timeline.
    VkSemaphore m_PlSem;                       // libplacebo-side handle
    pyrowave_sync_object_opaque* m_SyncObj;    // PyroWave import
    VkSemaphore m_PwSem;                        // PyroWave-side handle
    std::atomic<uint64_t> m_TlNext;            // monotonic value dispenser
    std::atomic<uint64_t> m_LastHoldVal;       // last value libplacebo hold_ex will signal
    std::atomic<uint64_t> m_LastReleaseVal;    // last value PyroWave decode release signals
    bool m_TimelineReady;

    std::mutex m_FrameLock;
    bool m_FrameReady;

    // Performance-overlay stats (compact version of FFmpegVideoDecoder's). Decode/network stats are
    // updated on the decode thread in submitDecodeUnit; render stats come from the main thread via
    // atomics folded in at each 1-second window flip.
    VIDEO_STATS m_ActiveWndVideoStats = {};
    VIDEO_STATS m_LastWndVideoStats = {};
    BandwidthTracker m_BwTracker;
    std::vector<uint8_t> m_PacketScratch;  // for packets straddling buffer-chain entries
    uint32_t m_LastFrameNumber;
    std::atomic<uint32_t> m_RenderedFrames;
    std::atomic<uint64_t> m_TotalRenderTimeUs;

    // Performance/status overlay compositing (same model as PlVkRenderer): notifyOverlayUpdated
    // uploads text surfaces to staging textures; renderFrameOnMainThread promotes + composites them.
    SDL_SpinLock m_OverlayLock;
    struct OverlayState {
        bool hasOverlay = false;
        pl_overlay overlay = {};
        bool hasStagingOverlay = false;
        pl_overlay stagingOverlay = {};
    } m_Overlays[Overlay::OverlayMax];
};

#endif  // HAVE_PYROWAVE
