#pragma once

#include "imgui.h"
#include "imgui/plotdesc.h"
#include "implot.h"
#include "settings/streamingpreferences.h"
#include "streaming/floatbuffer.h"
#include "streaming/stats.h"

#include <chrono>
#include <mutex>

using Clock = std::chrono::steady_clock;

enum DisplayOutputFormat {
    OUTPUT_IS_SDR = 0,
    OUTPUT_IS_PQ = 1,
    OUTPUT_IS_LINEAR = 2
};

struct RunningStat {
    uint64_t count = 0;
    float sum = 0.0;
    float min = 0.0;
    float max = 0.0;
    float last = 0.0;

    void add(float v)
    {
        last = v;
        if (count == 0) {
            min = v;
            max = v;
        }
        else {
            if (v < min) {
                min = v;
            }
            if (v > max) {
                max = v;
            }
        }
        ++count;
        sum += v;
    }

    float avg() const
    {
        return count ? (sum / static_cast<float>(count)) : 0.0;
    }

    void reset()
    {
        count = 0;
        sum = 0.0;
        min = 0.0;
        max = 0.0;
    }
};

struct DevUIConfig {
    int metricsUpdateRate = 15;
    int maxFramesInFlight = 2;
    int frameQueueHigh = 3;
    bool flushQueue = false;
    bool resetCounters = false;
    bool isFullscreen = false;
    bool isVRR = false;
    int pacingMode = StreamingPreferences::FRAME_PACING_IMMEDIATE;
    int presentMode = StreamingPreferences::PRESENT_AUTO;
    int windowMode = StreamingPreferences::WM_WINDOWED;
    bool showStats = true;
    bool showGraphs = true;
    bool enableDevUI = false;
    bool usePTSForVRR = false;

    int audioRenderer = StreamingPreferences::AUDIO_RENDERER_SDL;

#ifdef __APPLE__
    bool isProMotion = false;
    bool showMetalHud = false;
    bool proMotionAllowsVRR = false;
    bool captureGPUTrace = false;
    bool useEDR = false;
    bool isReferenceModeDisplay = false;
    float referenceWhite = 203.0f;
    float minNits = 0.0f;
    float maxNits = 0.0f;
    int spatialAudio = StreamingPreferences::SAC_DISABLED;
    bool useHeadTracking = false;
#endif
};

// metrics are read-only by the UI
struct DevUIMetrics {
    double currentPtsSeconds = 0.0;
    RunningStat renderLoopBudgetMs = {};
    RunningStat renderLoopTotalMs = {};
    RunningStat renderLoopWaitFrameMs = {};
    RunningStat renderLoopMaxWaitMs = {};
    RunningStat renderLoopRenderMs = {};
    RunningStat renderLoopWaitGPUMs = {};
    RunningStat renderLoopWaitPostMs = {};
    RunningStat renderLoopWaitBeforePresentMs = {};
    RunningStat frametimeClientMs = {};
    RunningStat frametimeHostMs = {};
    int totalFrames = 0;
    int networkDroppedFrames = 0;
    int pacerDroppedFrames = 0;
    int presentMode = StreamingPreferences::PRESENT_AUTO;
    int windowMode = StreamingPreferences::WM_WINDOWED;
    double presentAtTime = 0.0;
    RunningStat presentLateMs = {};
    int presentMissed = 0;
    int displayLockedMissed = 0;
    RunningStat presentDelayMs = {};
    RunningStat presentIntervalMs = {};
    RunningStat presentAMDHistory = {};
    RunningStat submittedIntervalMs = {};
    double presentAccuracyMs = 0.0;
    float streamFps = 0.0;
    float displayHz = 0.0;

#ifdef __APPLE__
    // EDR
    float currentEDR = 1.0f;
    float maxPotentialEDR = 1.0f;
    float maxReferenceEDR = 1.0f;
    float referenceWhite = 203.0f;
    float minNits = 0.0f;
    float maxNits = 0.0f;
#endif

    // audio
    char audioOutputDeviceName[32] = {};
    double audioSampleRate = 48000.0;
    int audioFrameDurationMs = 10;
    int opusChannelCount = 2;
    int audioChannels = 2;
    int spatialAudio = StreamingPreferences::SAC_DISABLED;
    bool audioPersonalizedHRTF = false;
    bool audioHeadTracking = false;
    char audioOutputTransportType[5] = {};
    char audioOutputDataSource[5] = {};
    double audioTotalSoftwareLatency = 0.0;
    double audioOutputHardwareLatency = 0.0;
    int audioDropCount = 0;
    float audioInBufferMs = 0.0;

    void reset()
    {
        totalFrames = 0;
        networkDroppedFrames = 0;
        pacerDroppedFrames = 0;
        presentAccuracyMs = 0.0;
        presentMissed = 0;
        displayLockedMissed = 0;

        renderLoopBudgetMs.reset();
        renderLoopTotalMs.reset();
        renderLoopWaitFrameMs.reset();
        renderLoopMaxWaitMs.reset();
        renderLoopRenderMs.reset();
        renderLoopWaitGPUMs.reset();
        renderLoopWaitPostMs.reset();
        renderLoopWaitBeforePresentMs.reset();
        frametimeClientMs.reset();
        frametimeHostMs.reset();
        presentLateMs.reset();
        presentDelayMs.reset();
        presentIntervalMs.reset();
        presentAMDHistory.reset();

        audioDropCount = 0;
    }
};

struct DevUISnapshot {
    DevUIConfig config;
    DevUIMetrics metrics;
};

class DevUISettings
{
  public:
    // Singleton
    static DevUISettings& instance();

    void InitFromPrefs(StreamingPreferences& prefs);

    DevUISnapshot snapshot() const;
    DevUIConfig GetConfig() const;
    void ChangeAndApplyConfig(DevUIConfig& config);

    template<typename Fn> void SetConfig(Fn&& cb)
    {
        std::lock_guard<std::mutex> lock(m_Mutex);
        cb(m_Config);
    }

    void SetMetricsUpdateRate(int updatesPerSecond)
    {
        m_MetricsUpdateRateHz.store(std::clamp(updatesPerSecond, 1, 120));
    }

    template<typename Fn> void UpdateMetrics(Fn&& cb)
    {
        // metrics callbacks only need to run if panel is open
        if (!m_panelOpen || !m_Enabled.load()) {
            return;
        }

        std::lock_guard<std::mutex> lock(m_Mutex);
        cb(m_Metrics);
    }

    bool IsVisible();
    void SetPanelOpen(bool isOpen);
    void Toggle();
    void Render();
    void ResetCounters();

  private:
    DevUISettings();
    DevUISettings(const DevUISettings&) = delete;
    DevUISettings& operator=(const DevUISettings&) = delete;

    void DrawFrameTimingBar(const char* label,
                            float waitFrameMs,
                            const ImVec4& preWait4,
                            float renderMs,
                            const ImVec4& render4,
                            float waitPresentMs,
                            const ImVec4& waitPresent4,
                            float waitEndMs,
                            const ImVec4& waitEnd);
    void DrawCornerText(const char* text, float scaling = 3.0f);

    mutable std::mutex m_Mutex;
    DevUIConfig m_Config;
    DevUIMetrics m_Metrics;
    std::atomic<bool> m_Enabled {true};
    std::atomic<bool> m_panelOpen {false};
    std::atomic<int> m_MetricsUpdateRateHz {15};
    std::atomic<bool> m_SettingsLoaded {false};  // set after settings have been loaded from Qt Prefs
};

// This is from MangoHud
class DevColors
{
  public:
    void InitColors(DisplayOutputFormat outputFormat);

    struct ui_colors {
        ImVec4 preWait, render, waitPresent, waitEnd;
        ImVec4 plotBg, plotLine;
    } colors {};
};

extern DevColors DevUIColors;
