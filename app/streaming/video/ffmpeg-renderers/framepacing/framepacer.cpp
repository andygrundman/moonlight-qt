
#include "framepacer.h"

#include "framequeue.h"
#include "imgui/devui.h"
#include "imgui/imgui_plots.h"
#include "streaming/qpc.h"

#include <algorithm>
#include <chrono>
#include <thread>

extern "C"
{
#include <libavutil/rational.h>
}

#ifdef Q_OS_WIN32
    #define WIN32_LEAN_AND_MEAN
    #include <Windows.h>
    #include "../pacer/dxvsyncsource.h"
#endif

#ifdef HAS_WAYLAND
    #include "../pacer/waylandvsyncsource.h"
#endif

#ifdef Q_OS_DARWIN
    #include "../pacer/displaylink_source.h"
#endif

#include <SDL_syswm.h>

// Frame Pacing operation
//
// 3 threads use this class:
//
// Decoder thread (run from moonlight-common-c because DIRECT_SUBMIT)
//   * calls submitFrame() to queue a new AVFrame to FrameQueue class via FrameQueue::instance().enqueue(frame)
//   * Frames are dropped at enqueue time, in an alternating manner, when high water mark (default 2 + 1) is exceeded
//   * IDR frames are never dropped
//
// vsyncThread thread:
//   * low-priority background thread responsible for tracking accurate vsync stats via GetFrameStatistics()
//
// main render loop thread:
//   * calls waitForFrame() with a timeout, to wait for new frames to become available in FrameQueue
//   * calls renderOnMainThread to render decoded video frame via VideoRenderer
//   * calls waitBeforePresent() using vsync timing data to align with the next vblank interval (or half-vblank for
//   120hz on Xbox)
//
// Calls to FQLog() and functions called within FQLog() are no-op unless you define FRAME_QUEUE_VERBOSE in pch.h
// and build in Debug mode.

constexpr int FRAME_QUEUE_LOW = 1;
constexpr int FRAME_QUEUE_HIGH = 3;

using steady_clock = std::chrono::steady_clock;

FramePacer& FramePacer::instance()
{
    static FramePacer inst;
    return inst;
}

FramePacer::FramePacer():
    m_StreamFps(0),
    m_RemoteAnchorPts(0),
    m_LocalAnchorUs(0),
    m_RenderThread(nullptr),
    m_VsyncThread(nullptr),
    m_LastSyncQpc(0),
    m_VsyncIntervalQpc(0),
    m_ewmaVsyncDriftQpc(MsToQpc(0.0001))
{}

void FramePacer::initPacingMode(int pacingMode)
{
    m_PendingPacingMode = pacingMode;
}

bool FramePacer::initialize(IFFmpegRenderer* renderer, PDECODER_PARAMETERS params)
{
    m_Renderer = renderer;

    SDL_Window* window = params->window;
    m_RefreshRate = StreamUtils::getDisplayRefreshRateRational(window);
    m_StreamFps = params->frameRate;
    m_RendererAttributes = m_Renderer->getRendererAttributes();
    m_Stopping.store(false);

    // The frame pacing mode comes from Prefs, and is set prior to us being called
    int pacingMode = StreamingPreferences::FRAME_PACING_IMMEDIATE;
    if (m_PendingPacingMode != -1) {
        pacingMode = m_PendingPacingMode;
        m_FramePacingMode.store(m_PendingPacingMode);
        m_PendingPacingMode = -1;
    }

    // cache some misc info we show in stats but don't update often
    Stats::instance().SetMetadata(params->videoFormat, params->width, params->height);

    FrameCadence::instance().init(m_RefreshRate.hz, static_cast<double>(m_StreamFps));

#ifndef IMGUI_DISABLE
    // populate dev UI settings
    DevUISettings::instance().SetConfig([=](DevUIConfig& config) {
        config.frameQueueHigh = FRAME_QUEUE_HIGH;
        config.pacingMode = pacingMode;
    });
#endif

    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                "Frame Pacer init: mode %s, streamFps %d, refreshRate %.2f\n",
                pacingMode == StreamingPreferences::FRAME_PACING_IMMEDIATE ? "immediate" : "display-locked",
                m_StreamFps,
                m_RefreshRate.hz);

    // Start FrameQueue so it's ready to receive new frames
    FrameQueue::instance().setHighWaterMark(FRAME_QUEUE_HIGH);
    FrameQueue::instance().start();

    if (m_Renderer->isVsyncTimingSupported()) {
        SDL_SysWMinfo info;
        SDL_VERSION(&info.version);
        if (!SDL_GetWindowWMInfo(window, &info)) {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "SDL_GetWindowWMInfo() failed: %s", SDL_GetError());
            return false;
        }

        switch (info.subsystem) {
#ifdef Q_OS_WIN32
            case SDL_SYSWM_WINDOWS:
                SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Frame pacing: using D3D WaitForVerticalBlankEvent");
                m_VsyncSource = new DxVsyncSource();
                break;
#endif

#if defined(SDL_VIDEO_DRIVER_WAYLAND) && defined(HAS_WAYLAND)
            case SDL_SYSWM_WAYLAND:
                SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Frame pacing: using Wayland frame callbacks");
                m_VsyncSource = new WaylandVsyncSource();
                break;
#endif

#ifdef Q_OS_DARWIN
            case SDL_SYSWM_COCOA:
                SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Frame pacing: using macOS DisplayLink");
                m_VsyncSource = new DisplayLinkSource();
                break;
#endif

            default:
                // Platforms without a VsyncSource will just render frames
                // immediately like they used to.
                SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                            "Frame pacing: no vsync source on this platform, pacing will be less effective.");
                break;
        }

        SDL_assert(m_VsyncSource != nullptr || !(m_RendererAttributes & RENDERER_ATTRIBUTE_FORCE_PACING));

        if (m_VsyncSource != nullptr && !m_VsyncSource->initialize(window, (int) m_RefreshRate.hz)) {
            SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION,
                        "Vsync source failed to initialize. Frame pacing will not be available!");
            delete m_VsyncSource;
            m_VsyncSource = nullptr;
        }

        if (m_VsyncSource != nullptr) {
            m_VsyncThread = SDL_CreateThread(FramePacer::vsyncThread, "FramePacerVsync", this);
        }
    }

    if (m_Renderer->isRenderThreadSupported()) {
        m_RenderThread = SDL_CreateThread(FramePacer::renderThread, "FramePacerRender", this);
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "FramePacerRender thread started as thread ID %lu",
                    SDL_GetThreadID(m_RenderThread));
    }

    return true;
}

void FramePacer::deinit()
{
    m_Stopping.store(true);

    // Stop and clear out FrameQueue
    FrameQueue::instance().stop();

    // Stop the render thread
    if (m_Renderer != nullptr) {
        if (m_Renderer->isRenderThreadSupported()) {
            if (m_RenderThread != nullptr) {
                SDL_WaitThread(m_RenderThread, nullptr);
                m_RenderThread = nullptr;
            }
        }
        else {
            // Notify the main thread renderer that it is being destroyed soon
            m_Renderer->cleanupRenderContext();
        }
        m_Renderer = nullptr;
    }

    // Stop the vsync thread (it stops after checking m_Stopping)
    m_WaitVsync.notify_all();
    if (m_VsyncThread != nullptr) {
        SDL_WaitThread(m_VsyncThread, nullptr);
        m_VsyncThread = nullptr;
    }

    if (m_CurrentFrame) {
        av_frame_free(&m_CurrentFrame);
        m_CurrentFrame = nullptr;
    }

    delete m_VsyncSource;
    m_VsyncSource = nullptr;

    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "FramePacer: deinit\n");
}

int FramePacer::renderThread(void* context)
{
    FramePacer* me = reinterpret_cast<FramePacer*>(context);

    if (SDL_SetThreadPriority(SDL_THREAD_PRIORITY_HIGH) < 0) {
        SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "Unable to set render thread to high priority: %s", SDL_GetError());
    }

#ifndef IMGUI_DISABLE
    ImGuiPlots::instance().ImGui_init(me->m_Renderer);
#endif

    // All timestamps in this function use Qpc
    // t0: top of loop
    //    waitToRender()
    // t1: GPU is ready, front of vsync interval
    //    waitForFrame()
    // t2: frame is ready in queue
    //    renderModeImmediate() || renderModeDisplayLocked()
    // t3: render done
    //    wait
    uint64_t t0 = 0, t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0;
    int64_t lastFramePts = 0, lastPresentTime = 0;
    double frametimeMs = 0.0, hostFrametimeMs = 0.0;
    const double bufferMs = 1.0;  // safety wait time to avoid missing deadline
    const double alphaUp = 0.25;  // react faster when renderMs spikes upward
    const double alphaDown = 0.05;  // decay slowly when renderMs drops
    double ewmaRenderMs = 2.0;  // Initial guess for render cost
    double ewmaPresentAccuracyMs = 0.001;

    // Don't run the main loop until the first frame is available
    while (!FrameQueue::instance().count() && !me->stopping()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    while (!me->stopping()) {
        // Get overall deadline we must hit to not overshoot Present
        uint64_t deadline = me->getNextVBlankQpc(&t0);

        me->m_Renderer->waitToRender();
        t1 = QpcNow();

        // wait for a frame + avg render time (display-locked mode only)
        double maxWaitMs = std::max(0.0, QpcToMs(deadline - t1) - ewmaRenderMs - bufferMs);

        // Immediate mode may need to wait a while for a new frame, don't tie it to vsync interval
        bool isImmediate = me->m_FramePacingMode.load() == StreamingPreferences::FRAME_PACING_IMMEDIATE;
        if (isImmediate) {
            maxWaitMs = 1000.0;
        }
        me->waitForFrame(maxWaitMs);
        if (me->stopping()) {
            break;
        }
        t2 = QpcNow();

        bool hitDeadline = true;
        uint64_t presentTargetQpc = deadline;

        bool didRender = isImmediate ? me->renderModeImmediate() : me->renderModeDisplayLocked();
        t3 = QpcNow();
        if (!didRender) {
            // If Immediate mode, this means We're receiving a lower framerate and no frame was available,
            // we don't call Present here and the previous frame will be re-displayed by the OS.
            // In Display-locked this should not happen.
            continue;
        }

        // When running without vsync, try an experimental sleep to hit vblank. In theory this should be the
        // lowest possible latency, but everything depends on the sleep being very precise.
        bool vsyncEnabled = me->m_Renderer->isVsyncEnabled();
        if (!vsyncEnabled) {
            hitDeadline = me->waitBeforePresent(presentTargetQpc, MsToQpc(ewmaPresentAccuracyMs));
        }
        t4 = QpcNow();

        // Present
        me->m_Renderer->presentFrame(me->m_CurrentFrame, presentTargetQpc);
        t5 = QpcNow();

        // Graph frametime only for new frames
        bool isRepeatFrame = true;
        int64_t currentFramePts = me->getCurrentFramePts();
        if (currentFramePts != lastFramePts) {
            if (lastPresentTime > 0) {
                hostFrametimeMs = ((double) currentFramePts - lastFramePts) / 90.0;
                frametimeMs = QpcToMs(t4 - lastPresentTime);
                // presentIntervalMs is used for PLOT_FRAMETIME since this one is much more jittery and makes things look worse than they are
                //ImGuiPlots::instance().observeFloat(PLOT_FRAMETIME, static_cast<float>(frametimeMs));
            }
            lastPresentTime = t4;
            lastFramePts = currentFramePts;
            isRepeatFrame = false;
        }

        // Weighted avg of time spent in render functions, more weight given to a slower render time
        // If we missed our present deadline this frame, aggressively weight this higher so maxWaitMs is smaller.
        double renderMs = QpcToMs(t3 - t2);
        double clampedRenderMs = std::clamp(renderMs, 0.0, QpcToMs(deadline - t0));
        double alpha = (clampedRenderMs > ewmaRenderMs) ? alphaUp : alphaDown;
        if (!hitDeadline) {
            alpha *= 2.0;
        }
        ewmaRenderMs = (clampedRenderMs * alpha) + (ewmaRenderMs * (1.0 - alpha));

        double renderLoopTotalMs = QpcToMs(t4 - t0); // does not include t5 which is post-present
        double renderLoopBudgetMs = QpcToMs(deadline - t0);
        double waitGPUMs = QpcToMs(t1 - t0);
        double waitFrameMs = QpcToMs(t2 - t1);
        double waitBeforePresentMs = QpcToMs(t4 - t3);
        double waitPostMs = QpcToMs(t5 - t4);

        double presentAccuracyMs = 0.0;
        if (!vsyncEnabled) {
            presentAccuracyMs = std::abs(QpcToMs(t4 - presentTargetQpc));
            const double pAlpha = 0.25;
            ewmaPresentAccuracyMs = (presentAccuracyMs * pAlpha) + (ewmaPresentAccuracyMs * (1.0 - pAlpha));
        }

        FQLog(
            "loop: %.3f/%.3f %s%s%s [pts:%.3fs] [client:%02.3fms host:%02.3fms] "
            "[waitGPU %.3fms] [waitFrame %.3fms/max %.3fms] render %.3fms [waitPost %.3fms]",
            renderLoopTotalMs,  // loop time
            renderLoopBudgetMs,  // deadline time window until next vblank
            hitDeadline ? " " : "M",  // missed deadline?
            isRepeatFrame ? "R" : " ",  // repeated frame?
            waitFrameMs > maxWaitMs ? "W" : " ",  // we waited too long for a frame
            (double) currentFramePts / 90000.0,  // host's timestamp (in seconds)
            frametimeMs,  // effective client frametime not counting repeated frames
            hostFrametimeMs,  // host frametime
            waitGPUMs, // time spent waiting for GPU to finish previous frame
            waitFrameMs,  // time spent waiting for new frame to arrive
            maxWaitMs,  // max wait allowed this frame
            renderMs,  // render time this frame
            waitPostMs
        );

        Stats::instance().SubmitRenderStats(waitFrameMs, renderMs, hitDeadline);

    #ifndef IMGUI_DISABLE
        DevUISettings::instance().UpdateMetrics([&](DevUIMetrics& metrics) {
            metrics.currentPtsSeconds = (double) currentFramePts / 90000.0;
            metrics.streamFps = FrameCadence::instance().streamFps();
            metrics.displayHz = FrameCadence::instance().displayHz();
            metrics.renderLoopBudgetMs.add(renderLoopBudgetMs);
            metrics.renderLoopTotalMs.add(renderLoopTotalMs);
            metrics.renderLoopWaitGPUMs.add(waitGPUMs);
            metrics.renderLoopWaitFrameMs.add(waitFrameMs);
            metrics.renderLoopMaxWaitMs.add(maxWaitMs);
            metrics.renderLoopRenderMs.add(renderMs);
            metrics.renderLoopWaitBeforePresentMs.add(waitBeforePresentMs);
            metrics.renderLoopWaitPostMs.add(waitPostMs);
            metrics.frametimeClientMs.add(frametimeMs);
            metrics.frametimeHostMs.add(hostFrametimeMs);
            if (!vsyncEnabled) metrics.presentAccuracyMs = ewmaPresentAccuracyMs;
            if (!vsyncEnabled && !hitDeadline) ++metrics.presentMissed;
        });
    #endif
    }

    // Notify the renderer that it is being destroyed soon
    // NB: This must happen on the same thread that calls renderFrame().
    me->m_Renderer->cleanupRenderContext();

#ifndef IMGUI_DISABLE
    ImGuiPlots::instance().ImGui_deinit(me->m_Renderer);
#endif

    return 0;
}

// Main render thread

void FramePacer::waitForFrame(double timeoutMs)
{
    if (stopping()) return;

    // Wait for a decoded frame to be available
    const int queueHas = 1;
    FrameQueue::instance().waitForEnqueue(queueHas, timeoutMs);
}

// called by main thread when renderer can't use a thread
bool FramePacer::renderOnMainThread()
{
    if (stopping()) return false;

    // Ignore this call for renderers that work on a dedicated render thread
    if (m_RenderThread != nullptr) {
        return false;
    }

    if (m_FramePacingMode.load() == StreamingPreferences::FRAME_PACING_IMMEDIATE) {
        return renderModeImmediate();
    }
    else {
        return renderModeDisplayLocked();
    }
}

// Dequeue a new frame if available and immediately render it. When no new frame is available
// skips Present and relies on the system to continue showing the previous frame.
bool FramePacer::renderModeImmediate()
{
    AVFrame* newFrame = FrameQueue::instance().dequeue();
    if (!newFrame) {
        return false;
    }

    int droppedCount = 0;
    int queueDepth = static_cast<int>(FrameQueue::instance().count());

    // If we are behind, catch up by taking one more frame and dropping the older one.
    if (queueDepth > FRAME_QUEUE_LOW) {
        AVFrame* newerFrame = FrameQueue::instance().dequeue();
        if (newerFrame) {
            FrameQueue::instance().dropFrame(newFrame);
            newFrame = newerFrame;
            droppedCount = 1;
            --queueDepth;
        }
    }

    if (m_CurrentFrame) {
        av_frame_free(&m_CurrentFrame);
    }

    m_CurrentFrame = newFrame;
    m_CurrentFramePts.store(m_CurrentFrame->pts);

    if (droppedCount > 0) {
        Stats::instance().SubmitDroppedFrame(droppedCount);
        ImGuiPlots::instance().observeFloat(PLOT_DROPPED_PACER, static_cast<float>(droppedCount));
    }

    // Count time spent in FrameQueue
    uint64_t beforeRenderUs = LiGetMicroseconds();
    Stats::instance().SubmitPacerTime(beforeRenderUs - static_cast<uint64_t>(m_CurrentFrame->pkt_dts));

    m_Renderer->renderFrame(m_CurrentFrame);

    FQLog("> Frame rendered [mode immediate] [pts: %.3fs] [%.3ffps] [%.3fhz] [queued %d] [dropped %d]\n",
          m_CurrentFrame->pts / 90000.0,
          FrameCadence::instance().streamFps(),
          FrameCadence::instance().displayHz(),
          queueDepth,
          droppedCount);

    // Keep m_CurrentFrame alive until the next frame. It is used to calculate frametime.
    return true;
}

// Attempt to pace rendering based on observed framerate from host pts data.
// Always presents at max refresh rate, using either a new frame or a cached previous frame.
bool FramePacer::renderModeDisplayLocked()
{
    // Consume frame(s) according to cadence
	int advanceCount =  FrameCadence::instance().decideAdvanceCount();

	// if the queue has too many frames in it, break the cadence and render or drop one extra
	int queueDepth = FrameQueue::instance().count();
	if (queueDepth > FRAME_QUEUE_LOW) {
		advanceCount++;
	}

	for (int i = 0; i < advanceCount; ++i) {
		AVFrame *newFrame = FrameQueue::instance().dequeue();
		if (!newFrame) {
			break;
		}

		if (m_CurrentFrame) {
			if (i > 0) {
				// advanceCount was > 1, so this is a dropped frame
                Stats::instance().SubmitDroppedFrame(1);
				ImGuiPlots::instance().observeFloat(PLOT_DROPPED_PACER, 1.0);
			}
			av_frame_free(&m_CurrentFrame);
		}
		m_CurrentFrame = newFrame;
        m_CurrentFramePts.store(m_CurrentFrame->pts);
	}

	if (!m_CurrentFrame) {
		// No frame available yet
		return false;
	}

    // Count time spent in FrameQueue
    uint64_t beforeRenderUs = LiGetMicroseconds();
    Stats::instance().SubmitPacerTime(beforeRenderUs - static_cast<uint64_t>(m_CurrentFrame->pkt_dts));

    // Render it
    m_Renderer->renderFrame(m_CurrentFrame);

    FQLog(
        "> Frame rendered [mode display-locked] [pts: %.3fs] [stream %.3ffps] [display %.3fhz] "
        "[advanceCount %d] [queueDepth %d]\n",
        m_CurrentFrame->pts / 90000.0,
        FrameCadence::instance().streamFps(),
        FrameCadence::instance().displayHz(),
        advanceCount,
        queueDepth
    );

    // Keep m_CurrentFrame alive in case we need to reuse it on the next present
    return true;
}

// called by render thread, returns true if we waited, false if we missed the target
bool FramePacer::waitBeforePresent(uint64_t targetQpc, uint64_t nudgeQpc) {
    if (stopping()) return false;

    // wait until vblank before presenting
	uint64_t now = QpcNow();
	if (targetQpc == 0) {
		targetQpc = getNextVBlankQpc(&now);
	}

	m_LastSyncTargetQpc.store(targetQpc);

    if (nudgeQpc < targetQpc) {
        targetQpc -= nudgeQpc;
    }

	if (targetQpc > now) {
		SleepUntilQpc(targetQpc);
        FQLog("SleepUntilQpc slept %.3fms and landed %.3fms away (target %llu nudge %llu)",
            QpcToMs(targetQpc - now), QpcToMs(QpcNow() - targetQpc), targetQpc, nudgeQpc);
		return true;
	}

	return false;
}

// called by render thread
int64_t FramePacer::getCurrentFramePts()
{
    return m_CurrentFramePts.load();
}

// end main thread

static inline int frameAttachUserdata(AVFrame* frame, int64_t prevPts)
{
    if (!frame) {
        return AVERROR(EINVAL);
    }

    if (frame->opaque_ref) {
        av_buffer_unref(&frame->opaque_ref);
    }

    AVBufferRef* buf = av_buffer_allocz(sizeof(MLFrameData));
    if (!buf) {
        return AVERROR(ENOMEM);
    }

    MLFrameData* data = (MLFrameData*) buf->data;
    data->prevPts = prevPts;
    frame->opaque_ref = buf;

    return 0;
}

// called by decoder thread
void FramePacer::submitFrame(AVFrame* frame)
{
    if (stopping()) {
        av_frame_free(&frame);
        return;
    }

    // Update cadence from pts, and store the previous frame's pts with this frame,
    // which gives us the ability to accurately pace frames on a VRR display.
    if (frame->pts) {
        int64_t prevPts = FrameCadence::instance().observeFramePts(frame->pts);
        frameAttachUserdata(frame, prevPts);

        // If this is the first frame we've seen, set our anchor point
        if (m_RemoteAnchorPts == 0) {
            m_RemoteAnchorPts = frame->pts;
            m_LocalAnchorUs = LiGetMicroseconds();
        }
    }

    int dropCount = FrameQueue::instance().enqueue(frame);
    if (dropCount) {
        Stats::instance().SubmitDroppedFrame(dropCount);
    }

    // Sometimes a frame will be dequeued right away, causing count() to be 0.
    // Use min of 1.0 for a nicer graph.
    int count = std::max((int)FrameQueue::instance().count(), 1);

    // plot a smoother queue size average
    static FloatBuffer avgQueueFB{256};
    avgQueueFB.push((float)count);
    float avgQueueDepth = avgQueueFB.average();

    ImGuiPlots::instance().observeFloat(PLOT_DROPPED_PACER, (float) dropCount);
    ImGuiPlots::instance().observeFloat(PLOT_QUEUED_FRAMES, avgQueueDepth);
    Stats::instance().SubmitAvgQueueSize(avgQueueDepth);

    if (m_RenderThread == nullptr) {
        SDL_Event event;

        // For main thread rendering, we'll push an event to trigger a callback
        event.type = SDL_USEREVENT;
        event.user.code = SDL_CODE_FRAME_READY;
        SDL_PushEvent(&event);
    }
}

///

// Vsync data collection - each platform is responsible for calling us at each vsync with an OS-provided
// timestamp of the most recent vsync (the start of the current display refresh interval)

// Some platforms like Win32 don't provide this, in that case the module will use other means such as
// GetFrameStatistics() to obtain the timestamp from a few frames in the past, and use this to estimate the current
// timestamp.

int FramePacer::vsyncThread(void* context)
{
    FramePacer* me = reinterpret_cast<FramePacer*>(context);

    if (SDL_SetThreadPriority(SDL_THREAD_PRIORITY_HIGH) < 0) {
        SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "Unable to set vsync thread to high priority: %s", SDL_GetError());
    }

    bool async = me->m_VsyncSource->isAsync();
    while (!me->stopping()) {
        if (async) {
            // Wait for the VSync source to invoke signalVsyncTS()
            me->waitUntilVsync();
        }
        else {
            // Let the VSync source wait in the context of our thread
            me->m_VsyncSource->waitForVsync();
        }

        if (me->stopping()) {
            break;
        }
    }

    // XXX vsync thread should update:
    // FrameCadence::instance().setDisplayHz() to track real performance of the display
    // m_LastSyncQpc: last vsync timestamp provided by the system
    // m_VsyncIntervalQpc - estimated average vsync interval
    // m_ewmaVsyncDriftQpc - moving average of drift, if targeting present to vblank interval (m_LastSyncQpc -
    // m_LastSyncTarget)

    // Win32 D3D: use GetFrameStatistics() to look at recent frame data
    // Mac: DisplayLink callback provides timestamps

    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "vsyncThread stopped");

    return 0;
}

void FramePacer::signalVsyncTS(double timestamp, double deadline)
{
    if (stopping()) return;

    // Inform the renderer about the native timestamps
    m_Renderer->notifyVsyncTimestamps(timestamp, deadline);

    m_VsyncTimestamp.store(timestamp);

    // we need to translate the OS-provided timestamps in seconds into Qpc
    std::scoped_lock<std::mutex> lock(m_FrameStatsLock);
    m_VsyncIntervalQpc = MsToQpc((deadline - timestamp) * 1000.0);
    m_LastSyncQpc = MsToQpc(timestamp * 1000.0);

    // compare with the last sync target we used in waitBeforePresent
    int64_t driftQpc = 0;
    uint64_t lastTargetQpc = m_LastSyncTargetQpc.load();
    if (lastTargetQpc > 0) {
        driftQpc = m_LastSyncQpc - lastTargetQpc;
        if ((uint64_t)std::llabs(driftQpc) < MsToQpc(0.03)) {
            const double alpha = 0.05; // slow moving average
            m_ewmaVsyncDriftQpc = (1.0 - alpha) * m_ewmaVsyncDriftQpc + alpha * static_cast<double>(driftQpc);
        }
    }

    static double lastDeadline = 0.0;
    if (lastDeadline > 0.0) {
        double interval = deadline - lastDeadline;
        FQLog("signalVsyncTS(): interval %.3fms timestamp %f, deadline %f, drift %.3f (avg %.3f)\n",
            interval * 1000.0, timestamp, deadline, QpcToMs(driftQpc), QpcToMs(m_ewmaVsyncDriftQpc));
    }
    lastDeadline = deadline;

    m_WaitVsync.notify_all();
}

// wait until the next vsync
void FramePacer::waitUntilVsync()
{
    std::unique_lock<std::mutex> lock(m_FrameStatsLock);
    uint64_t lastSyncQpc = m_LastSyncQpc;
    m_WaitVsync.wait(lock, [&]() {
        return m_LastSyncQpc > lastSyncQpc || stopping();
    });
}

// Caller often needs now and the vsync interval, since this needs locking
// the logic is confined to this function.
uint64_t FramePacer::getNextVBlankQpc(uint64_t* now)
{
    std::scoped_lock<std::mutex> lock(m_FrameStatsLock);
    uint64_t target = 0, interval = 0;
    *now = QpcNow();

    if (m_LastSyncQpc == 0 || m_VsyncIntervalQpc == 0) {
        // Fallback until vsync events start to come in
        double rr = m_RefreshRate.hz > 0.0 ? m_RefreshRate.hz : 60.0;
        interval = MsToQpc(1000.0 / rr);
        target = *now + interval;
    }
    else {
        interval = m_VsyncIntervalQpc;
        uint64_t next = m_LastSyncQpc + static_cast<int64_t>(m_ewmaVsyncDriftQpc);

        while (next < *now) {
            next += interval;
        }
        target = next;
    }

    // Keep true refresh rate synced with cadence
    FrameCadence::instance().setDisplayHz(1000.0 / QpcToMs(interval));

    return target;
}
