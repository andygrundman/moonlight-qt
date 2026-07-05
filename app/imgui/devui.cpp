#include "devui.h"

#include "imgui.h"
#include "imgui/IconsFontAwesome7.h"
#include "implot.h"
#include "settings/streamingpreferences.h"
#include "streaming/qpc.h"
#include "streaming/session.h"
#include "streaming/video/ffmpeg-renderers/framepacing/framecadence.h"
#include "streaming/video/ffmpeg-renderers/framepacing/framepacer.h"
#include "streaming/video/ffmpeg-renderers/framepacing/framequeue.h"

#include <algorithm>
#include <cstdio>

#ifdef __APPLE__
#include <dispatch/dispatch.h>
#endif

DevUISettings& DevUISettings::instance()
{
    static DevUISettings inst;
    return inst;
}

DevUISettings::DevUISettings() {}

void DevUISettings::InitFromPrefs(StreamingPreferences& prefs)
{
    DevUIConfig cfg;
    cfg.pacingMode = prefs.framePacingMode;
    cfg.presentMode = prefs.presentMode;
    cfg.windowMode = prefs.windowMode;
    cfg.showStats = prefs.showPerformanceOverlay;
    cfg.showGraphs = prefs.showPerformanceGraphs;
    cfg.audioRenderer = prefs.audioRenderer;

#ifdef __APPLE__
    cfg.maxFramesInFlight = std::clamp(prefs.vtMetalFramesInFlight, 2, 3);
    cfg.spatialAudioConfig = prefs.spatialAudioConfig;
    cfg.useHeadTracking = prefs.spatialAudioConfig == StreamingPreferences::SAC_HEAD_TRACKED;
#endif

    // These settings need to be primed from prefs
    FramePacer::instance().SetPacingMode(cfg.pacingMode);
    Stats::instance().SetShowGraphs(cfg.showGraphs);

    std::lock_guard<std::mutex> lock(m_Mutex);
    m_Config = cfg;
}

DevUISnapshot DevUISettings::snapshot() const
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    return DevUISnapshot {m_Config, m_Metrics};
}

DevUIConfig DevUISettings::GetConfig() const
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    return m_Config;
}

void DevUISettings::ChangeAndApplyConfig(DevUIConfig& config)
{
    auto old = GetConfig();

    if (old.metricsUpdateRate != config.metricsUpdateRate) {
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "DevUI: Changed metricsUpdateRate from %d to %d",
                    old.metricsUpdateRate,
                    config.metricsUpdateRate);
        SetMetricsUpdateRate(config.metricsUpdateRate);
    }

    if (old.frameQueueHigh != config.frameQueueHigh) {
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "DevUI: Changed frameQueueHigh from %d to %d and flushed queue",
                    old.frameQueueHigh,
                    config.frameQueueHigh);
        FrameQueue::instance().setHighWaterMark(config.frameQueueHigh);
        FrameQueue::instance().clear();
        ResetCounters();
    }

    if (old.pacingMode != config.pacingMode) {
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "DevUI: Changed pacingMode from %d to %d",
                    old.pacingMode,
                    config.pacingMode);
        FramePacer::instance().SetPacingMode(config.pacingMode);
        ResetCounters();
    }

    if (old.presentMode != config.presentMode) {
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "DevUI: Pending change of presentMode from %d to %d",
                    old.presentMode,
                    config.presentMode);
        // picked up by renderer (vt_metal applyDevUIConfig)
    }

    if (old.windowMode != config.windowMode) {
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "DevUI: Pending change of windowMode from %d to %d",
                    old.windowMode,
                    config.windowMode);

        // We have to let session.cpp handle the mode, because it needs to recreate the decoder
        SDL_Event event;
        event.type = SDL_USEREVENT;
        event.user.code = SDL_CODE_SET_WINDOW_MODE;
        event.user.data1 = (void*)(uintptr_t)config.windowMode;
        SDL_PushEvent(&event);
    }

    if (old.flushQueue != config.flushQueue) {
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "DevUI: Flushed frame queue");
        FrameQueue::instance().clear();
        config.flushQueue = false;
    }

    if (old.resetCounters != config.resetCounters) {
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "DevUI: Reset DevUI counters");
        ResetCounters();
        config.resetCounters = false;
    }

    if (old.showStats != config.showStats) {
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "DevUI: Changed showStats from %d to %d",
                    old.showStats,
                    config.showStats);
        Session::get()->getOverlayManager().setOverlayState(
            Overlay::OverlayDebug,
            !Session::get()->getOverlayManager().isOverlayEnabled(Overlay::OverlayDebug)
        );
    }

    if (old.showGraphs != config.showGraphs) {
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "DevUI: Changed showGraphs from %d to %d",
                    old.showGraphs,
                    config.showGraphs);
        Stats::instance().SetShowGraphs(config.showGraphs);
    }

#ifdef __APPLE__
    if (old.showMetalHud != config.showMetalHud) {
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "DevUI: Pending change of showMetalHud from %d to %d",
                    old.showMetalHud,
                    config.showMetalHud);
        // picked up by renderer (vt_metal applyDevUIConfig)
    }

    if (old.proMotionAllowsVRR != config.proMotionAllowsVRR) {
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "DevUI: Pending change of proMotionAllowsVRR from %d to %d",
                    old.proMotionAllowsVRR,
                    config.proMotionAllowsVRR);
        // picked up by renderer (vt_metal applyDevUIConfig)
    }

    if (old.maxFramesInFlight != config.maxFramesInFlight) {
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "DevUI: Pending change of maxFramesInFlight from %d to %d",
                    old.maxFramesInFlight,
                    config.maxFramesInFlight);
        // picked up by renderer (vt_metal applyDevUIConfig)
    }

    if (old.useEDR != config.useEDR) {
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "DevUI: Pending change of useEDR from %d to %d",
                    old.useEDR,
                    config.useEDR);
        // picked up by renderer (vt_metal applyDevUIConfig)
    }

    if (old.referenceWhite != config.referenceWhite) {
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
            "DevUI: Pending change of referenceWhite from %.2f to %.2f",
            old.referenceWhite,
            config.referenceWhite);
        // picked up by renderer (vt_metal applyDevUIConfig)
    }

    if (old.minNits != config.minNits) {
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
            "DevUI: Pending change of minNits from %.2f to %.2f",
            old.minNits,
            config.minNits);
        // picked up by renderer (vt_metal applyDevUIConfig)
    }

    if (old.maxNits != config.maxNits) {
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
            "DevUI: Pending change of maxNits from %.2f to %.2f",
            old.maxNits,
            config.maxNits);
        // picked up by renderer (vt_metal applyDevUIConfig)
    }

    if (old.useHeadTracking != config.useHeadTracking) {
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "DevUI: Pending change of head tracking from %d to %d",
                    old.useHeadTracking,
                    config.useHeadTracking);

        if (Session::get()->getAudioRenderer() != nullptr) {
             Session::get()->getAudioRenderer()->setHeadTracking(config.useHeadTracking);
        }
    }
#endif

    std::lock_guard<std::mutex> lock(m_Mutex);
    m_Config = config;
}

static float SRGBToLinear(float in)
{
    if (in <= 0.04045f) {
        return in / 12.92f;
    }
    else {
        return powf((in + 0.055f) / 1.055f, 2.4f);
    }
}

static ImVec4 SRGBToLinear(ImVec4 col)
{
    col.x = SRGBToLinear(col.x);
    col.y = SRGBToLinear(col.y);
    col.z = SRGBToLinear(col.z);
    // Alpha component is already linear

    return col;
}

static float LinearToPQ(float in)
{
    const float m1 = 0.1593017578125f;
    const float m2 = 78.84375f;
    const float c1 = 0.8359375f;
    const float c2 = 18.8515625f;
    const float c3 = 18.6875f;
    /* target 200 cd/m^2 as our maximum rather than 10000 cd/m^2 */
    const float targetL = 200.f;
    const float maxL = 10000.0f;

    in = powf(in * (targetL / maxL), m1);
    in = (c1 + c2 * in) / (1.0f + c3 * in);
    return powf(in, m2);
}

static double dot(const ImVec4& a, const ImVec4& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

static ImVec4 SRGBtoBT2020(ImVec4 col)
{
    const ImVec4 to2020[3] = {
        {0.627392, 0.32903, 0.0432691, 0.0},
        {0.0691229, 0.9195232, 0.0113204, 0.0},
        {0.0164229, 0.088042, 0.8956166, 0.0}
    };

    col.x = dot(to2020[0], col);
    col.y = dot(to2020[1], col);
    col.z = dot(to2020[2], col);

    return col;
}

static ImVec4 LinearToPQ(ImVec4 col)
{
    col = SRGBtoBT2020(col);

    col.x = LinearToPQ(col.x);
    col.y = LinearToPQ(col.y);
    col.z = LinearToPQ(col.z);

    return col;
}

ImVec4 convert_color(unsigned color)
{
    ImVec4 fc = ImGui::ColorConvertU32ToFloat4(color);
    fc.w = 1.0f;  // alpha

    // output colorspace is PQ
    fc = SRGBToLinear(fc);
    return LinearToPQ(fc);

    // output colorspace is SRGB
    return fc;
}

// Helper to display a little (?) mark which shows a tooltip when hovered.
static void HelpMarker(const char* desc)
{
    ImGui::TextDisabled(ICON_FA_QUESTION);
    if (ImGui::BeginItemTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

bool DevUISettings::IsVisible()
{
    return m_panelOpen.load();
}

void DevUISettings::SetPanelOpen(bool isOpen)
{
    m_panelOpen.store(isOpen);
}

void DevUISettings::Toggle()
{
    m_panelOpen.store( !m_panelOpen.load() );
}

void DevUISettings::Render()
{
    static DevUISnapshot snap = {};
    static auto nextUpdate = Clock::now();

    // updating every frame is too fast, update at 30fps with a slider
    const auto now = Clock::now();
    const auto interval =
        std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / m_MetricsUpdateRateHz.load()));

    if (now >= nextUpdate) {
        snap = snapshot();
        nextUpdate += interval;
        if (nextUpdate <= now) {
            nextUpdate = now + interval;
        }
    }

    DevUIConfig cfg = snap.config;
    DevUIMetrics metrics = snap.metrics;
    bool configChanged = false;

    ImGuiViewport* vp = ImGui::GetMainViewport();
    if (!vp) {
        return;
    }

    bool panelOpen = m_panelOpen.load();

    const float panelWidth = 500.0f;

    const ImVec2 workPos = vp->WorkPos;
    const ImVec2 workSize = vp->WorkSize;

    const float panelX = workPos.x + workSize.x - panelWidth - 16.0f;
    const float panelY = workPos.y + 64.0f;
    const float panelHeight = 0.0f;

    ImGuiWindowFlags commonFlags = 0;

    // Panel
    if (panelOpen) {
    #ifdef __APPLE__
        // allow keyboard in advanced panel for things like entering numbers
        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        dispatch_async(dispatch_get_main_queue(), ^{
            SDL_StartTextInput();
        });
    #endif

        ImGui::SetNextWindowPos(ImVec2(panelX, panelY), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(panelWidth, panelHeight), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowBgAlpha(0.85f);

        if (ImGui::Begin("Advanced Controls", &panelOpen, commonFlags)) {
            ImGui::SeparatorText("Video");
            ImGui::PushTextWrapPos();

            ImGui::SetNextItemWidth(-ImGui::GetContentRegionAvail().x * 0.5f);
            static const char* pacingModeItems[] = {"Immediate", "Display-locked"};
            configChanged |=
                ImGui::Combo("Frame pacing type", &cfg.pacingMode, pacingModeItems, IM_ARRAYSIZE(pacingModeItems));
            ImGui::SameLine();
            HelpMarker(
                "Immediate renders frames as they arrive.\nDisplay-locked renders at the display refresh rate and "
                "shows new or repeated frames based on PTS timestamps."
            );

            ImGui::SetNextItemWidth(-ImGui::GetContentRegionAvail().x * 0.5f);
            static const char* windowModeItems[] = {"Fullscreen Exclusive", "Borderless Fullscreen", "Windowed"};
            configChanged |=
                ImGui::Combo("Window mode", &cfg.windowMode, windowModeItems, IM_ARRAYSIZE(windowModeItems));

            ImGui::SetNextItemWidth(-ImGui::GetContentRegionAvail().x * 0.5f);
            static const char* presentModeItems[] = {
                "Auto",
                "Fixed Vsync",
                "VRR",
                "No Vsync"};
            configChanged |=
                ImGui::Combo("Present mode", &cfg.presentMode, presentModeItems, IM_ARRAYSIZE(presentModeItems));

    #ifdef __APPLE__
            if (cfg.isFullscreen && cfg.isProMotion) {
                configChanged |= ImGui::Checkbox("Allow VRR on ProMotion display", &cfg.proMotionAllowsVRR);
                ImGui::SameLine();
                HelpMarker(
                    "ProMotion displays like on MacBook Pro are not true VRR displays and only operate at a few fixed rates (120, 60, 50, 48). "
                    "This option lets Moonlight treat it like a VRR display and use variable present durations. This may result in stuttering. "
                    "Use an external adaptive refresh display to fully utilize VRR."
                );
            }
    #endif

            if (metrics.presentMode == StreamingPreferences::PRESENT_VRR) {
                configChanged |= ImGui::Checkbox("Pace VRR frames using PTS", &cfg.usePTSForVRR);
                ImGui::SameLine();
                HelpMarker(
                    "If enabled, uses the host's frame timestamp (PTS) to determine how long each frame should be "
                    "displayed for. May not work with all capture backends.\n\nThe default (unchecked) paces based on "
                    "the average GPU time per frame."
                );
            }

            ImGui::SetNextItemWidth(-ImGui::GetContentRegionAvail().x * 0.5f);
            static const char* maxFIFItems[] = {"2 (double-buffered)", "3 (triple-buffered)"};
            static int item_selected_idx = cfg.maxFramesInFlight - 2;
            const char* maxFIFSelectedValue = maxFIFItems[item_selected_idx];
            if (ImGui::BeginCombo("Max frames in flight", maxFIFSelectedValue, 0)) {
                for (int n = 0; n < IM_COUNTOF(maxFIFItems); n++) {
                    const bool is_selected = (item_selected_idx == n);
                    if (ImGui::Selectable(maxFIFItems[n], is_selected)) {
                        item_selected_idx = n;
                        cfg.maxFramesInFlight = n + 2;
                        configChanged = true;
                    }
                    if (is_selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            // There is no benefit from tweaking the queue high watermark
            // configChanged |= ImGui::SliderInt("Frame queue size", &cfg.frameQueueHigh, 1, 3);

            // This doesn't really need to be changed
            // configChanged |= ImGui::SliderInt("Metrics update rate", &cfg.metricsUpdateRate, 1, 120);

            configChanged |= ImGui::Checkbox("Show Stats", &cfg.showStats);
            ImGui::SameLine();
            configChanged |= ImGui::Checkbox("Show Performance Graphs", &cfg.showGraphs);

    #ifdef __APPLE__
            configChanged |= ImGui::Checkbox("Show Metal HUD", &cfg.showMetalHud);
            ImGui::SameLine();
            HelpMarker(
                "Shift-F10: cycle other HUD views\n"
                "Shift-F11: reset HUD counters\n"
                "More options in the Metal HUD app menu.\n"
                "If run with MTL_CAPTURE_ENABLED=1, a GPU Capture checkbox will allow capture of an Xcode GPU trace. Traces can be loaded into Xcode for Metal and shader debugging.\n"
            );

            // Run Moonlight with MTL_CAPTURE_ENABLED=1 to enable this option
            if (const char* env_capture = std::getenv("MTL_CAPTURE_ENABLED");
                env_capture && std::strcmp(env_capture, "1") == 0) {
                ImGui::SameLine();
                configChanged |= ImGui::Checkbox("GPU Capture", &cfg.captureGPUTrace);
                ImGui::SameLine();
                HelpMarker(
                    "Click to begin capturing GPU activity to /tmp/moonlight.gputrace. Report may consume 1GB/sec or "
                    "more and performance will be very slow while capturing.\nClick again to stop. You can open the "
                    "trace using Xcode."
                );
            }
    #endif

            static bool show_demo_window = false;
            ImGui::Checkbox("ImGui Demo", &show_demo_window);
            if (show_demo_window) {
                ImGui::ShowDemoWindow(&show_demo_window);
            }

        // This HDR stuff is pretty half-baked and not very useful at the moment
        // #ifdef __APPLE__
        //     if (ImGui::CollapsingHeader("HDR")) {
        //         ImGui::Text("EDR headroom: %.1fx SDR", metrics.currentEDR);

        //         ImGui::SetNextItemWidth(-ImGui::GetContentRegionAvail().x * 0.5f);
        //         if (cfg.isReferenceModeDisplay) ImGui::BeginDisabled(); // reference mode uses 100 nits
        //         configChanged |= ImGui::SliderFloat("Reference/SDR White", &cfg.referenceWhite, 100.0f, 203.0f, "%.1f nits");
        //         ImGui::SameLine();
        //         HelpMarker("This value represents SDR peak white. If your display is in reference mode (such as MacBook Pro's 'HDR Video' preset), SDR peak is 100 nits. In other modes, 203 nits is the default.");
        //         if (cfg.isReferenceModeDisplay) ImGui::EndDisabled();

        //         ImGui::SetNextItemWidth(-ImGui::GetContentRegionAvail().x * 0.5f);
        //         configChanged |= ImGui::SliderFloat("Min Luminance", &cfg.minNits, 0.0f, 6.5535f, "%.4f nits");

        //         ImGui::SetNextItemWidth(-ImGui::GetContentRegionAvail().x * 0.5f);
        //         configChanged |= ImGui::SliderFloat("Max Luminance", &cfg.maxNits, 1.0f, 10000.0f, "%.2f nits");

        //         configChanged |= ImGui::Checkbox("Enable EDR tone-mapping (experimental)", &cfg.useEDR);
        //         ImGui::SameLine();
        //         HelpMarker(
        //             "macOS will render HDR content according to your current display brightness and display preset. HDR will be tone-mapped to SDR when viewed on a non-HDR display.\n\n"
        //             "If disabled, HDR is rendered at its native brightness.");
        //     }
        // #endif

            if (ImGui::CollapsingHeader("Audio")) {
                if (Session::get()->getAudioRenderer() != nullptr) {
                    Session::get()->getAudioRenderer()->updateMetrics();
                }

                ImGui::Text("Input stream: %s-channel Opus low-delay @ 48 kHz",
                    metrics.opusChannelCount == 6 ? "5.1" : metrics.opusChannelCount == 8 ? "7.1" : "2");
            #ifdef __APPLE__
                if (cfg.audioRenderer == StreamingPreferences::AUDIO_RENDERER_COREAUDIO) {
                    ImGui::Text("Output: CoreAudio: %s @ %.1f kHz, %u-channel", metrics.audioOutputDeviceName, metrics.audioSampleRate / 1000.0, metrics.audioChannels);
                    ImGui::Text("Audio buffer: %.2fms", metrics.audioInBufferMs);
                    ImGui::Text("Drops (behind/underrun): %d/%d", metrics.audioDropCount, metrics.audioDropCountUnderrun);
                    ImGui::SameLine();
                    HelpMarker("To prevent latency, an audio block is considered \"behind\" and skipped if Moonlight has over 30ms of pending audio. An audio underrun occurs when there is not enough received audio data in the buffer to supply the audio device. Both of these situations cause an audible glitch.");

                    ImGui::Text("Render mode: %s %s %s %s",
                        metrics.spatialAudioActive
                            ? (metrics.audioPersonalizedHRTF ? "personalized spatial audio" : "spatial audio")
                            : "passthrough",
                        metrics.spatialAudioActive && cfg.spatialAudioConfig == StreamingPreferences::SAC_HEAD_TRACKED
                            ? "with head-tracking for"
                            : "for",
                        !strcmp(metrics.audioOutputTransportType, "blue") ? "Bluetooth"
                            : !strcmp(metrics.audioOutputTransportType, "bltn") ? "built-in"
                            : !strcmp(metrics.audioOutputTransportType, "usb ") ? "USB"
                            : !strcmp(metrics.audioOutputTransportType, "hdmi") ? "HDMI"
                            : !strcmp(metrics.audioOutputTransportType, "airp") ? "AirPlay"
                            : metrics.audioOutputTransportType,
                        !strcmp(metrics.audioOutputDataSource      , "hdpn") ? "headphones"
                            : !strcmp(metrics.audioOutputDataSource, "ispk") ? "internal speakers"
                            : !strcmp(metrics.audioOutputDataSource, "espk") ? "external speakers"
                            : metrics.audioOutputDataSource);
                    ImGui::Text("Latency: %0.1fms (buffers %.1fms, hardware: %.1fms)",
                        (metrics.audioTotalSoftwareLatency + metrics.audioOutputHardwareLatency) * 1000.0,
                        metrics.audioTotalSoftwareLatency * 1000.0, metrics.audioOutputHardwareLatency * 1000.0);
                    if (metrics.spatialAudioActive && strcmp(metrics.audioOutputTransportType, "blue")) {
                        configChanged |= ImGui::Checkbox("Enable head-tracking", &cfg.useHeadTracking);
                        ImGui::SameLine();
                        HelpMarker(
                            "Head-tracking works best when Game Mode is active and Moonlight is running fullscreen. Game Mode reduces Bluetooth latency in AirPods from 160ms to about 80ms, although this number is not reflected in the hardware latency number.\n\n"
                            "Audio glitches are more common when head-tracking is enabled.");
                    }
                } else
            #endif
                {
                    ImGui::Text("Output: SDL: %s @ %.1f kHz, %u-channel", metrics.audioOutputDeviceName, metrics.audioSampleRate / 1000.0, metrics.audioChannels);
                    ImGui::Text("Audio buffer: %.2fms", metrics.audioInBufferMs);
                    ImGui::Text("Drop count: %d", metrics.audioDropCount);
                    ImGui::SameLine();
                    HelpMarker("To prevent latency, an audio block is dropped if Moonlight's outgoing audio buffer rises above 30ms. This causes an audible glitch.");
                }
            }

            // Live metrics
            static bool metricsVisible = true;
            ImGui::SetNextItemOpen(metricsVisible);
            if (ImGui::CollapsingHeader("Metrics"), &metricsVisible) {
                ImGui::Text("FPS: %.02f / Display: %.02f Hz", metrics.streamFps, metrics.displayHz);
                ImGui::Text("Frame queue: %zu/%d", FrameQueue::instance().count(), cfg.frameQueueHigh);
                ImGui::SameLine();
                HelpMarker(
                    "Ideal value: 1.0.\n\nThis small queue holds frames from arrival until they are processed by "
                    "waitFrame. If the render loop falls behind, "
                    "frames can build up in this queue and eventually the oldest frames will be dropped. One reason for "
                    "falling behind can be receiving too many frames from the host. "
                    "Even a 0.1% difference, such as a 59.94hz client receiving 60fps will fall behind, requiring a "
                    "dropped frame several times a minute."
                );
                ImGui::SameLine();
                if (ImGui::Button("Flush queue")) {
                    cfg.flushQueue = true;
                    configChanged = true;
                }

                ImGui::Text("Frame count: %06d, lost: %02d, dropped: %02d",
                            metrics.totalFrames,
                            metrics.networkDroppedFrames,
                            metrics.pacerDroppedFrames);
                ImGui::SameLine();
                if (ImGui::Button("Reset counters")) {
                    cfg.resetCounters = true;
                    configChanged = true;
                }

                // This turned out less useful than I hoped
                // DrawFrameTimingBar("Frame",
                //                     metrics.renderLoopWaitFrameMs.last,
                //                     DevUIColors.colors.preWait,
                //                     metrics.renderLoopRenderMs.last,
                //                     DevUIColors.colors.render,
                //                     metrics.renderLoopWaitBeforePresentMs.last,
                //                     DevUIColors.colors.waitPresent,
                //                     metrics.renderLoopWaitPostMs.last,
                //                     DevUIColors.colors.waitEnd);

                ImGui::Text("Render total         %.3f ms", metrics.renderLoopTotalMs.last);
                // ImGui::Text("  waitGPU            %.3f ms", metrics.renderLoopWaitGPUMs.last);
                ImGui::TextColored(DevUIColors.colors.preWait,
                                    "  waitFrame          %.3f ms",
                                    metrics.renderLoopWaitFrameMs.last);
                ImGui::SameLine();
                HelpMarker(
                    "Time spent waiting for a new frame to arrive from the host, if necessary. Optimal value is anything "
                    "above 0, meaning the frame queue is running at 1.0 and not adding latency."
                );
                ImGui::TextColored(DevUIColors.colors.render,
                                    "  render             %.3f ms",
                                    metrics.renderLoopRenderMs.last);
                ImGui::SameLine();
                HelpMarker(
                    "Time spent drawing everything in the frame: the host's video frame, stat overlays, graphs, this UI. "
                    "Typical value: < 1ms "
                );
                ImGui::TextColored(DevUIColors.colors.waitPresent,
                                    "  waitPresent        %.3f ms",
                                    metrics.renderLoopWaitBeforePresentMs.last);
                ImGui::SameLine();
                HelpMarker(
                    "Time spent waiting to align with the vblank interval. A very accurate sleep is performed so that "
                    "Present is called shortly before the vblank."
                );
                ImGui::TextColored(DevUIColors.colors.waitEnd,
                                    "  waitEnd            %.3f ms",
                                    metrics.renderLoopWaitPostMs.last);
                ImGui::SameLine();
                HelpMarker(
                    "Typical value: most of 1 frame\n\nTime spent waiting to obtain the next frame's drawable texture from "
                    "the GPU. This delay is the main driver of frame pacing."
                );

                switch (metrics.presentMode) {
                    case StreamingPreferences::PRESENT_AUTO:  // won't happen
                    case StreamingPreferences::PRESENT_VRR:
                        ImGui::Text("Mode:                VRR, min. duration %.3fms", metrics.presentAMDHistory.last * 1000.0);
                        HelpMarker(
                            "VRR mode, which requires borderless fullscreen, delivers frames with varying frametimes "
                            "based on the range of the display.\n"
                            "If 'Pace VRR frames using PTS' is enabled, each frame is scheduled according to the length of "
                            "time it was displayed on the host. Disable this option if it doesn't appear smooth."
                        );
                        break;
                    case StreamingPreferences::PRESENT_FIXED:
                        ImGui::Text("Mode:                Fixed @ %.02f Hz", metrics.displayHz);
                        ImGui::SameLine();
                        HelpMarker(
                            "In Immediate mode, frames are aligned to vsync timestamps when at full framerate. At lower "
                            "framerates, frames are scheduled with a duration matching the refresh rate.\n\n"
                            "In Display-locked mode, every frame is aligned using vsync timestamps. Incoming frames are "
                            "displayed or repeated as needed to best match their original timing."
                        );
                        break;
                    case StreamingPreferences::PRESENT_NO_VSYNC:
                        ImGui::Text("Mode:                No Vsync");
                        ImGui::SameLine();
                        HelpMarker(
                            "With vsync disabled, frames are presented with no regard to timing. Screen tearing and "
                            "stuttering will occur, but this mode should have the lowest latency."
                        );
                        break;
                }

                ImGui::Text("Present delay:       %.3f ms", metrics.presentDelayMs.last);
                ImGui::SameLine();
                HelpMarker(
                    "How long it takes for a presented frame to actually be shown on screen. Greatly affected by whether we're in composited or direct mode, and whether we're on a fixed refresh or VRR dislpay."
                );
                ImGui::Text("Present interval:    %.3f ms", metrics.presentIntervalMs.last);
                ImGui::SameLine();
                HelpMarker(
                    "Present interval is the interval between frames being displayed. This metric is used for the client frametime graph. This will be a multiple of the refresh rate on fixed displays, or a variable value when using VRR."
                );

                if (metrics.presentMode == StreamingPreferences::PRESENT_NO_VSYNC) {
                    ImGui::Text("Present accuracy:    %.3f ms", metrics.presentAccuracyMs);
                    ImGui::SameLine();
                    HelpMarker(
                        "This is an experimental mode. When vsync is disabled, we try to wait and send presents as close as possible to the display refresh.\n\n"
                        "If done perfectly, it is possible to avoid screen tearing while also having very low latency (see SpecialK and Moonlight for Xbox).\n\n"
                        "Present accuracy is an average of how close we got without going over, and is used to adjust future sleeps.\n\n");
                    ImGui::Text("Presents missed (N-V):      %5d", metrics.presentMissed);
                    ImGui::SameLine();
                    HelpMarker("The number of times no-vsync overslept the sleep target.");
                }
                if (cfg.pacingMode == StreamingPreferences::FRAME_PACING_DISPLAY_LOCKED) {
                    ImGui::Text("Presents missed (D-L):  %5d", metrics.displayLockedMissed);
                    ImGui::SameLine();
                    HelpMarker("How many times the display-locked renderer missed a screen refresh. This can be our fault or the system's.");
                }
            }
            ImGui::PopTextWrapPos();
        }
        ImGui::End();
    }
    else {
    #ifdef __APPLE__
        // no keyboard when panel is closed
        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags &= ~ImGuiConfigFlags_NavEnableKeyboard;
        dispatch_async(dispatch_get_main_queue(), ^{
            SDL_StopTextInput();
        });
    #endif
    }

    if (configChanged) {
        ChangeAndApplyConfig(cfg);
    }
    m_panelOpen.store(panelOpen);
}

void DevUISettings::ResetCounters()
{
    UpdateMetrics([](DevUIMetrics& metrics) {
        metrics.reset();
    });
}

void DevUISettings::DrawFrameTimingBar(const char* label,
                                       float waitFrameMs,
                                       const ImVec4& preWait4,
                                       float renderMs,
                                       const ImVec4& render4,
                                       float waitPresentMs,
                                       const ImVec4& waitPresent4,
                                       float waitEndMs,
                                       const ImVec4& waitEnd4)
{
    const ImGuiStyle& style = ImGui::GetStyle();

    ImVec2 size = ImVec2(0.0f, 18.0f);
    if (size.x <= 0.0f) {
        size.x = ImGui::GetContentRegionAvail().x;
    }

    const float totalMs = waitFrameMs + renderMs + waitPresentMs + waitEndMs;
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    const ImVec2 barSize(size.x, size.y);
    const ImVec2 barEnd(pos.x + barSize.x, pos.y + barSize.y);

    ImDrawList* drawList = ImGui::GetWindowDrawList();

    const ImU32 bgColor = ImGui::GetColorU32(ImGuiCol_FrameBg);
    const ImU32 borderColor = ImGui::GetColorU32(ImGuiCol_Border);
    const ImU32 textColor = ImGui::GetColorU32(ImGuiCol_Text);
    const ImU32 preWait = ImGui::GetColorU32(preWait4);
    const ImU32 render = ImGui::GetColorU32(render4);
    const ImU32 waitPresent = ImGui::GetColorU32(waitPresent4);
    const ImU32 waitEnd = ImGui::GetColorU32(waitEnd4);

    const float rounding = style.FrameRounding;

    drawList->AddRectFilled(pos, barEnd, bgColor, rounding);

    if (totalMs > 0.0f) {
        const float frac1 = std::max(0.0f, waitFrameMs / totalMs);
        const float frac2 = std::max(0.0f, renderMs / totalMs);
        const float frac3 = std::max(0.0f, waitPresentMs / totalMs);
        // const float frac4 = std::max(0.0f, waitEndMs / totalMs);

        const float x0 = pos.x;
        const float x1 = x0 + barSize.x * frac1;
        const float x2 = x1 + barSize.x * frac2;
        const float x3 = x2 + barSize.x * frac3;
        const float x4 = pos.x + barSize.x;

        if (x1 > x0) {
            drawList->AddRectFilled(ImVec2(x0, pos.y),
                                    ImVec2(x1, barEnd.y),
                                    preWait,
                                    rounding,
                                    ImDrawFlags_RoundCornersLeft);
        }

        if (x2 > x1) {
            drawList->AddRectFilled(ImVec2(x1, pos.y), ImVec2(x2, barEnd.y), render);
        }

        if (x3 > x2) {
            drawList->AddRectFilled(ImVec2(x2, pos.y),
                                    ImVec2(x3, barEnd.y),
                                    waitPresent,
                                    rounding,
                                    ImDrawFlags_RoundCornersRight);
        }

        if (x4 > x3) {
            drawList->AddRectFilled(ImVec2(x3, pos.y),
                                    ImVec2(x4, barEnd.y),
                                    waitEnd,
                                    rounding,
                                    ImDrawFlags_RoundCornersRight);
        }
    }

    drawList->AddRect(pos, barEnd, borderColor, rounding);

    if (label && label[0] != '\0') {
        char overlay[128];
        std::snprintf(overlay,
                      sizeof(overlay),
                      "%s  %.2f / %.2f / %.2f / %.2fms",
                      label,
                      waitFrameMs,
                      renderMs,
                      waitPresentMs,
                      waitEndMs);

        const ImVec2 textSize = ImGui::CalcTextSize(overlay);
        const ImVec2 textPos(pos.x + (barSize.x - textSize.x) * 0.5f, pos.y + (barSize.y - textSize.y) * 0.5f);

        drawList->AddText(textPos, textColor, overlay);
    }

    ImGui::Dummy(barSize);
}

// Colors
#define RGBGetBValue(rgb) (rgb & 0x000000FF)
#define RGBGetGValue(rgb) ((rgb >> 8) & 0x000000FF)
#define RGBGetRValue(rgb) ((rgb >> 16) & 0x000000FF)

inline ImVec4 ImLerp(const ImVec4& a, const ImVec4& b, float t) {
    return ImVec4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t);
}

void DevColors::InitColors(DisplayOutputFormat outputFormat)
{
    // ImGui colors are much too bright when output in a PQ colorspace
    // This code from MangoHud converts them from sRGB to PQ
    auto convert = [outputFormat](unsigned color, float alpha = 1.0f) -> ImVec4 {
        ImVec4 fc = ImGui::ColorConvertU32ToFloat4(
            IM_COL32(RGBGetRValue(color), RGBGetGValue(color), RGBGetBValue(color), 255)
        );
        fc.w = alpha;
        if (outputFormat == OUTPUT_IS_PQ) {
            fc = SRGBToLinear(fc);
            return LinearToPQ(fc);
        }
        else if (outputFormat == OUTPUT_IS_LINEAR) {
            return SRGBToLinear(fc);
        }
        return fc;
    };

    auto convert4 = [outputFormat](ImVec4 fc) -> ImVec4 {
        if (outputFormat == OUTPUT_IS_PQ) {
            fc = SRGBToLinear(fc);
            return LinearToPQ(fc);
        }
        else if (outputFormat == OUTPUT_IS_LINEAR) {
            return SRGBToLinear(fc);
        }
        return fc;
    };

    // ImGui colors, we could convert all of them but it ends up looking quite bad
    // This only deals with dimming the white text and green lines.
    ImGuiStyle& style = ImGui::GetStyle();
    //style.Colors[ImGuiCol_PlotLines]        = convert(0x00FF00);
    //style.Colors[ImGuiCol_WindowBg]         = convert(0x000000, 0.50f);
    style.Colors[ImGuiCol_Button]           = convert4(ImVec4(0.26f, 0.59f, 0.98f, 0.40f));
    style.Colors[ImGuiCol_ButtonHovered]    = convert4(ImVec4(0.26f, 0.59f, 0.98f, 1.00f));
    style.Colors[ImGuiCol_ButtonActive]     = convert4(ImVec4(0.06f, 0.53f, 0.98f, 1.00f));
    style.Colors[ImGuiCol_PlotLines]        = convert4(ImVec4(0.61f, 0.61f, 0.61f, 1.00f));
    style.Colors[ImGuiCol_PlotLinesHovered] = convert4(ImVec4(1.00f, 0.43f, 0.35f, 1.00f));

    style.AntiAliasedLines = false;

    style.Colors[ImGuiCol_Text]                   = convert4(ImVec4(1.00f, 1.00f, 1.00f, 1.00f));
    style.Colors[ImGuiCol_TextDisabled]           = convert4(ImVec4(0.50f, 0.50f, 0.50f, 1.00f));
    style.Colors[ImGuiCol_WindowBg]               = convert4(ImVec4(0.06f, 0.06f, 0.06f, 0.94f));
    style.Colors[ImGuiCol_ChildBg]                = convert4(ImVec4(0.00f, 0.00f, 0.00f, 0.00f));
    style.Colors[ImGuiCol_PopupBg]                = convert4(ImVec4(0.08f, 0.08f, 0.08f, 0.94f));
    style.Colors[ImGuiCol_Border]                 = convert4(ImVec4(0.43f, 0.43f, 0.50f, 0.50f));
    style.Colors[ImGuiCol_BorderShadow]           = convert4(ImVec4(0.00f, 0.00f, 0.00f, 0.00f));
    style.Colors[ImGuiCol_FrameBg]                = convert4(ImVec4(0.16f, 0.29f, 0.48f, 0.54f));
    style.Colors[ImGuiCol_FrameBgHovered]         = convert4(ImVec4(0.26f, 0.59f, 0.98f, 0.40f));
    style.Colors[ImGuiCol_FrameBgActive]          = convert4(ImVec4(0.26f, 0.59f, 0.98f, 0.67f));
    style.Colors[ImGuiCol_TitleBg]                = convert4(ImVec4(0.04f, 0.04f, 0.04f, 1.00f));
    style.Colors[ImGuiCol_TitleBgActive]          = convert4(ImVec4(0.16f, 0.29f, 0.48f, 1.00f));
    style.Colors[ImGuiCol_TitleBgCollapsed]       = convert4(ImVec4(0.00f, 0.00f, 0.00f, 0.51f));
    style.Colors[ImGuiCol_MenuBarBg]              = convert4(ImVec4(0.14f, 0.14f, 0.14f, 1.00f));
    // style.Colors[ImGuiCol_ScrollbarBg]            = convert4(ImVec4(0.02f, 0.02f, 0.02f, 0.53f));
    // style.Colors[ImGuiCol_ScrollbarGrab]          = convert4(ImVec4(0.31f, 0.31f, 0.31f, 1.00f));
    // style.Colors[ImGuiCol_ScrollbarGrabHovered]   = convert4(ImVec4(0.41f, 0.41f, 0.41f, 1.00f));
    // style.Colors[ImGuiCol_ScrollbarGrabActive]    = convert4(ImVec4(0.51f, 0.51f, 0.51f, 1.00f));
    style.Colors[ImGuiCol_CheckMark]              = convert4(ImVec4(0.26f, 0.59f, 0.98f, 1.00f));
    style.Colors[ImGuiCol_SliderGrab]             = convert4(ImVec4(0.24f, 0.52f, 0.88f, 1.00f));
    style.Colors[ImGuiCol_SliderGrabActive]       = convert4(ImVec4(0.26f, 0.59f, 0.98f, 1.00f));
    style.Colors[ImGuiCol_Button]                 = convert4(ImVec4(0.26f, 0.59f, 0.98f, 0.40f));
    style.Colors[ImGuiCol_ButtonHovered]          = convert4(ImVec4(0.26f, 0.59f, 0.98f, 1.00f));
    style.Colors[ImGuiCol_ButtonActive]           = convert4(ImVec4(0.06f, 0.53f, 0.98f, 1.00f));
    style.Colors[ImGuiCol_Header]                 = convert4(ImVec4(0.26f, 0.59f, 0.98f, 0.31f));
    style.Colors[ImGuiCol_HeaderHovered]          = convert4(ImVec4(0.26f, 0.59f, 0.98f, 0.80f));
    style.Colors[ImGuiCol_Separator]              = style.Colors[ImGuiCol_Border];
    style.Colors[ImGuiCol_HeaderActive]           = convert4(ImVec4(0.26f, 0.59f, 0.98f, 1.00f));
    style.Colors[ImGuiCol_SeparatorHovered]       = convert4(ImVec4(0.10f, 0.40f, 0.75f, 0.78f));
    style.Colors[ImGuiCol_SeparatorActive]        = convert4(ImVec4(0.10f, 0.40f, 0.75f, 1.00f));

    style.Colors[ImGuiCol_InputTextCursor]        = style.Colors[ImGuiCol_Text];
    style.Colors[ImGuiCol_TabHovered]             = style.Colors[ImGuiCol_HeaderHovered];
    style.Colors[ImGuiCol_Tab]                    = ImLerp(style.Colors[ImGuiCol_Header],       style.Colors[ImGuiCol_TitleBgActive], 0.80f);
    style.Colors[ImGuiCol_TabSelected]            = ImLerp(style.Colors[ImGuiCol_HeaderActive], style.Colors[ImGuiCol_TitleBgActive], 0.60f);
    style.Colors[ImGuiCol_TabSelectedOverline]    = style.Colors[ImGuiCol_HeaderActive];
    style.Colors[ImGuiCol_TabDimmed]              = ImLerp(style.Colors[ImGuiCol_Tab],          style.Colors[ImGuiCol_TitleBg], 0.80f);
    style.Colors[ImGuiCol_TabDimmedSelected]      = ImLerp(style.Colors[ImGuiCol_TabSelected],  style.Colors[ImGuiCol_TitleBg], 0.40f);
    style.Colors[ImGuiCol_TabDimmedSelectedOverline] = convert4(ImVec4(0.50f, 0.50f, 0.50f, 0.00f));
    //style.Colors[ImGuiCol_DockingPreview]         = style.Colors[ImGuiCol_HeaderActive] * ImVec4(1.0f, 1.0f, 1.0f, 0.7f);

    // style.Colors[ImGuiCol_ResizeGrip]             = convert4(ImVec4(0.26f, 0.59f, 0.98f, 0.20f));
    // style.Colors[ImGuiCol_ResizeGripHovered]      = convert4(ImVec4(0.26f, 0.59f, 0.98f, 0.67f));
    // style.Colors[ImGuiCol_ResizeGripActive]       = convert4(ImVec4(0.26f, 0.59f, 0.98f, 0.95f));
    // style.Colors[ImGuiCol_TabDimmedSelectedOverline] = convert4(ImVec4(0.50f, 0.50f, 0.50f, 0.00f));
    // style.Colors[ImGuiCol_DockingEmptyBg]         = convert4(ImVec4(0.20f, 0.20f, 0.20f, 1.00f));
    style.Colors[ImGuiCol_PlotLines]              = convert4(ImVec4(0.61f, 0.61f, 0.61f, 1.00f));
    style.Colors[ImGuiCol_PlotLinesHovered]       = convert4(ImVec4(1.00f, 0.43f, 0.35f, 1.00f));
    style.Colors[ImGuiCol_PlotHistogram]          = convert4(ImVec4(0.90f, 0.70f, 0.00f, 1.00f));
    style.Colors[ImGuiCol_PlotHistogramHovered]   = convert4(ImVec4(1.00f, 0.60f, 0.00f, 1.00f));
    // style.Colors[ImGuiCol_TableHeaderBg]          = convert4(ImVec4(0.19f, 0.19f, 0.20f, 1.00f));
    // style.Colors[ImGuiCol_TableBorderStrong]      = convert4(ImVec4(0.31f, 0.31f, 0.35f, 1.00f));
    // style.Colors[ImGuiCol_TableBorderLight]       = convert4(ImVec4(0.23f, 0.23f, 0.25f, 1.00f));
    // style.Colors[ImGuiCol_TableRowBg]             = convert4(ImVec4(0.00f, 0.00f, 0.00f, 0.00f));
    // style.Colors[ImGuiCol_TableRowBgAlt]          = convert4(ImVec4(1.00f, 1.00f, 1.00f, 0.06f));
    // style.Colors[ImGuiCol_TextSelectedBg]         = convert4(ImVec4(0.26f, 0.59f, 0.98f, 0.35f));
    style.Colors[ImGuiCol_DragDropTarget]         = convert4(ImVec4(1.00f, 1.00f, 0.00f, 0.90f));
    style.Colors[ImGuiCol_DragDropTargetBg]       = convert4(ImVec4(0.00f, 0.00f, 0.00f, 0.00f));
    // style.Colors[ImGuiCol_UnsavedMarker]          = convert4(ImVec4(1.00f, 1.00f, 1.00f, 1.00f));
    // style.Colors[ImGuiCol_NavCursor]              = convert4(ImVec4(0.26f, 0.59f, 0.98f, 1.00f));
    // style.Colors[ImGuiCol_NavWindowingHighlight]  = convert4(ImVec4(1.00f, 1.00f, 1.00f, 0.70f));
    // style.Colors[ImGuiCol_NavWindowingDimBg]      = convert4(ImVec4(0.80f, 0.80f, 0.80f, 0.20f));
    // style.Colors[ImGuiCol_ModalWindowDimBg]       = convert4(ImVec4(0.80f, 0.80f, 0.80f, 0.35f));

    // Our custom colors
    DevUIColors.colors.preWait = convert(0x05668d);
    DevUIColors.colors.render = convert(0x028090);
    DevUIColors.colors.waitPresent = convert(0x00a896);
    DevUIColors.colors.waitEnd = convert(0x02c39a);

    // Colors used by graphs in stats.cpp
    DevUIColors.colors.plotBg = convert(0x020202, 0.2f);
    DevUIColors.colors.plotLine = convert4(ImVec4(0, 1, 0, 1));
}

DevColors DevUIColors;
