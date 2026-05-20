// A port of the frame pacer from Moonlight for Xbox

// TODO:
// pacer functionality we need:
// vsync thread
// render thread
// SDL callback when frame done

#pragma once

#include "framecadence.h"
#include "streaming/floatbuffer.h"
#include "streaming/stats.h"
#include "streaming/streamutils.h"
#include "streaming/video/ffmpeg-renderers/renderer.h"

#include <atomic>
#include <deque>
#include <set>
#include <thread>
#include <utility>

extern "C"
{
#include <libavcodec/avcodec.h>
}

// The maximum number of frames pacer will ever hold is:
// - 3 frames in the pacing queue
// - 1 frame removed from the render queue in the process of rendering
// - 1 frame for deferred free
#define PACER_MAX_OUTSTANDING_FRAMES (3 + 1 + 1)

// Additional metadata attached to AVFrame
typedef struct MLFrameData {
    int64_t prevPts; // previous frame's pts
} MLFrameData;

class IVsyncSource {
public:
    virtual ~IVsyncSource() {}
    virtual bool initialize(SDL_Window* window, int displayFps) = 0;

    // Asynchronous sources produce callbacks on their own, while synchronous
    // sources require calls to waitForVsync().
    virtual bool isAsync() = 0;

    // Optional method called before destruction
    virtual void stop() = 0;

    // Call this method after receiving m_VsyncSignalled or after
    // waitForVsync() returns to obtain the remaining time in the current
    // vblank interval. A negative value means the interval was missed.
    virtual double remainingMilliseconds() = 0;

    virtual void waitForVsync() {
        // Synchronous sources must implement waitForVsync()!
        SDL_assert(false);
    }
};

class IFramePacer {
public:
    virtual ~IFramePacer() = default;
    virtual void submitFrame(AVFrame* frame) = 0;
    virtual bool initialize(IFFmpegRenderer* renderer, PDECODER_PARAMETERS params) = 0;
    virtual void signalVsync() { };
    virtual void signalVsyncTS(double, double) { };
    virtual bool renderOnMainThread() = 0;
};

class FramePacer: public IFramePacer
{
  public:
    // Singleton
    static FramePacer& instance();

    virtual void submitFrame(AVFrame* frame) override;
    virtual bool initialize(IFFmpegRenderer* renderer, PDECODER_PARAMETERS params) override;
    virtual bool renderOnMainThread() override;
    virtual void signalVsyncTS(double timestamp, double deadline) override;

    void deinit();
    void initPacingMode(int pacingMode);
    void waitForFrame(double timeoutMs);
    bool waitBeforePresent(uint64_t targetQpc, uint64_t nudgeQpc);
    int64_t getCurrentFramePts();
    uint64_t getNextVBlankQpc(uint64_t* now);
    void waitUntilVsync();

    void SetPacingMode(int pacingMode)
    {
        m_FramePacingMode.store(pacingMode);
    }

    int GetPacingMode()
    {
        return m_FramePacingMode.load();
    }

    double GetVsyncTimestamp()
    {
        return m_VsyncTimestamp.load();
    }

    inline bool stopping() const noexcept
    {
        return m_Stopping.load();
    }

  private:
    FramePacer();
    FramePacer(const FramePacer&) = delete;
    FramePacer& operator=(const FramePacer&) = delete;

    static int renderThread(void* context);
    bool renderModeImmediate();
    bool renderModeDisplayLocked();
    static int vsyncThread(void* context);

    IFFmpegRenderer* m_Renderer; // owned by ffmpeg.cpp
    RefreshRateRational m_RefreshRate;
    std::atomic<bool> m_Stopping {false};
    int m_StreamFps;
    int m_PendingPacingMode = -1;
    std::atomic<int> m_FramePacingMode{StreamingPreferences::FRAME_PACING_IMMEDIATE};
    int m_RendererAttributes;
    int64_t m_RemoteAnchorPts;
    uint64_t m_LocalAnchorUs;

    SDL_Thread* m_RenderThread;
    SDL_Thread* m_VsyncThread;
    IVsyncSource* m_VsyncSource;
    AVFrame* m_CurrentFrame = nullptr;
    std::atomic<int64_t> m_CurrentFramePts {0};

    std::mutex m_FrameStatsLock;
    std::condition_variable m_WaitVsync;
    uint64_t m_LastSyncQpc;
    std::atomic<uint64_t> m_LastSyncTargetQpc {0};
    std::atomic<double> m_VsyncTimestamp {0};
    uint64_t m_VsyncIntervalQpc;
    double m_ewmaVsyncDriftQpc;
};
