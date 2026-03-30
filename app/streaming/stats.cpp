#include "stats.h"

#include "streaming/video/ffmpeg-renderers/framepacing/framepacer.h"
#include "imgui.h"
#include "imgui/devui.h"
#include "imgui/imgui_plots.h"
#include "implot.h"
#include "SDL_compat.h"

// Log something only once, safe to use in hot areas of the code
#define CONCAT(a, b) CONCAT2(a, b)
#define CONCAT2(a, b) a##b
#define LogOnce(fmt, ...) \
    do { \
        static std::once_flag CONCAT(_onceFlag_, __LINE__); \
        std::call_once(CONCAT(_onceFlag_, __LINE__), [&] { \
            SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, fmt, ##__VA_ARGS__); \
        }); \
    } while (0)

Stats& Stats::instance()
{
    static Stats inst;
    return inst;
}

Stats::Stats():
    m_avgQueueSize(0.0),
    m_avgMbpsSmoothed(0.0),
    m_VideoFormat(0),
    m_Width(0),
    m_Height(0),
    m_ShowGraphs {false}
{
    SDL_zero(m_ActiveWndVideoStats);
    SDL_zero(m_LastWndVideoStats);
    SDL_zero(m_GlobalVideoStats);

    m_ActiveWndVideoStats.measurementStartUs = LiGetMicroseconds();
}

void Stats::SetMetadata(int videoFormat, int width, int height)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_VideoFormat = videoFormat;
    m_Width = width;
    m_Height = height;
}

bool Stats::GetShowGraphs()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_ShowGraphs;
}

void Stats::SetShowGraphs(bool enabled)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_ShowGraphs = enabled;
}

// Called every frame, if true is returned, the stats text is refreshed
bool Stats::ShouldUpdateDisplay(bool isVisible, char* output, size_t length)
{
    bool shouldUpdate = false;

    if (isVisible && ImGuiPlots::instance().isEnabled()) {
        const double alpha = 0.1f;
        m_avgMbpsSmoothed = (1 - alpha) * m_avgMbpsSmoothed + alpha * m_bwTracker.GetAverageMbps();
        ImGuiPlots::instance().observeFloat(PLOT_BANDWIDTH, (float) m_avgMbpsSmoothed);
    }

    // Process stats once per second
    if (LiGetMicroseconds() > m_ActiveWndVideoStats.measurementStartUs + 1000000) {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (isVisible) {
            // Display using data from the last 2 window periods
            VIDEO_STATS lastTwoWndStats = {};
            addVideoStats(m_LastWndVideoStats, lastTwoWndStats);
            addVideoStats(m_ActiveWndVideoStats, lastTwoWndStats);

            formatVideoStats(lastTwoWndStats, output, length);
            shouldUpdate = true;
        }

        // Accumulate these values into the global stats
        addVideoStats(m_ActiveWndVideoStats, m_GlobalVideoStats);

        // Move this window into the last window slot and clear it for next window
        memcpy(&m_LastWndVideoStats, &m_ActiveWndVideoStats, sizeof(VIDEO_STATS));
        SDL_zero(m_ActiveWndVideoStats);
        m_ActiveWndVideoStats.measurementStartUs = LiGetMicroseconds();
    }

    return shouldUpdate;
}

void Stats::LogGlobalVideoStats()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_GlobalVideoStats.renderedFps > 0 || m_GlobalVideoStats.renderedFrames != 0) {
        char videoStatsStr[1024];
        formatVideoStats(m_GlobalVideoStats, videoStatsStr, sizeof(videoStatsStr));

        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "\n%s\n------------------\n%s", "Global video stats", videoStatsStr);
    }
}

/// Hooks for stat producers, where possible these are combined into one call

// 1. The size in bytes of one video frame, we use this to also increment frame counters.
// 2. Time in milliseconds from first packet of a frame until fully reassembled frame is ready for decoding
//    Includes time spent in FEC reassembly
// 3. Host processing latency (encode time)
// 4. network packet loss (caller reports frame sequence number holes)
void Stats::SubmitVideoBytesAndReassemblyTime(PDECODE_UNIT decodeUnit, uint32_t droppedFrames)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_ActiveWndVideoStats.receivedFrames++;
    m_ActiveWndVideoStats.totalFrames++;

    // bandwidth
    m_bwTracker.AddBytes(decodeUnit->fullLength);

    // reassembly time
    uint32_t reassemblyUs = (uint32_t) (decodeUnit->enqueueTimeUs - decodeUnit->receiveTimeUs);
    m_ActiveWndVideoStats.totalReassemblyTimeUs += reassemblyUs;

    // Host processing latency
    uint16_t frameHPL = decodeUnit->frameHostProcessingLatency;
    if (frameHPL != 0) {
        if (m_ActiveWndVideoStats.minHostProcessingLatency != 0) {
            m_ActiveWndVideoStats.minHostProcessingLatency =
                std::min(m_ActiveWndVideoStats.minHostProcessingLatency, frameHPL);
        }
        else {
            m_ActiveWndVideoStats.minHostProcessingLatency = frameHPL;
        }
        m_ActiveWndVideoStats.framesWithHostProcessingLatency += 1;
        m_ActiveWndVideoStats.maxHostProcessingLatency =
            std::max(m_ActiveWndVideoStats.maxHostProcessingLatency, frameHPL);
        m_ActiveWndVideoStats.totalHostProcessingLatency += frameHPL;
    }

    // Network packet loss
    if (droppedFrames > 0) {
        m_ActiveWndVideoStats.networkDroppedFrames += droppedFrames;
        m_ActiveWndVideoStats.totalFrames += droppedFrames;
    }
    ImGuiPlots::instance().observeFloat(PLOT_DROPPED_NETWORK, (float) droppedFrames);

    // Host frametime graph, uses raw 90kHz units to avoid rounding errors
    static uint32_t lastHostPts = 0;
    if (lastHostPts != 0) {
        const uint32_t delta90k = (uint32_t) (decodeUnit->rtpTimestamp - lastHostPts);
        ImGuiPlots::instance().observeFloat(PLOT_HOST_FRAMETIME, (float) (delta90k / 90.0f));
    }
    lastHostPts = (uint32_t) decodeUnit->rtpTimestamp;

#ifndef IMGUI_DISABLE
    DevUISettings::instance().UpdateMetrics([&](DevUIMetrics& metrics) {
        if (droppedFrames > 0) {
            metrics.totalFrames += droppedFrames;
            metrics.networkDroppedFrames += droppedFrames;
        }
        metrics.totalFrames++;
    });
#endif
}

// Time in milliseconds we spent decoding one frame, it is added up to later be divided by decodedFrames
void Stats::SubmitDecodeTimeUs(uint64_t decodeUs)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_ActiveWndVideoStats.totalDecodeTimeUs += decodeUs;
    m_ActiveWndVideoStats.decodedFrames++;
}

void Stats::SubmitDroppedFrame(int count)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_ActiveWndVideoStats.pacerDroppedFrames += count;

    // Note: pacer dropped frame(s) have already been included in totalFrames
    // by SubmitVideoBytesAndReassemblyTime since they were otherwise normal frames.

#ifndef IMGUI_DISABLE
    DevUISettings::instance().UpdateMetrics([&](DevUIMetrics& metrics) {
        metrics.pacerDroppedFrames += count;
    });
#endif
}

void Stats::SubmitAvgQueueSize(float avgQueueSize)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_avgQueueSize = avgQueueSize;
}

// Time in microseconds we spent in the frame pacer, and time for rendering the frame.
// Also increments the rendered frame count.
void Stats::SubmitPacerTime(uint64_t pacerTimeUs)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_ActiveWndVideoStats.totalPacerTimeUs += pacerTimeUs;
}

// Present-to-display latency: time in microseconds from present submit to being shown on display
void Stats::SubmitPresentTimeUs(uint64_t presentTimeUs, int presentMode)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_ActiveWndVideoStats.totalPresentTimeUs += presentTimeUs;
    m_ActiveWndVideoStats.presentMode = presentMode;
}

// High-level render loop timings
void Stats::SubmitRenderStats(double preWaitTimeMs, double renderTimeMs, bool hitDeadline)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_ActiveWndVideoStats.totalRenderTimeUs += static_cast<uint64_t>(renderTimeMs * 1000);
    m_ActiveWndVideoStats.renderedFrames++;

    if (hitDeadline) {
        m_ActiveWndVideoStats.hitDeadlines++;
    }
    else {
        m_ActiveWndVideoStats.missedDeadlines++;
    }

    // Only shown in debug builds
    m_ActiveWndVideoStats.totalPreWaitTimeUs += static_cast<uint64_t>(preWaitTimeMs * 1000);
}

/// private methods

void Stats::addVideoStats(VIDEO_STATS& src, VIDEO_STATS& dst)
{
    dst.receivedFrames += src.receivedFrames;
    dst.decodedFrames += src.decodedFrames;
    dst.renderedFrames += src.renderedFrames;
    dst.totalFrames += src.totalFrames;
    dst.networkDroppedFrames += src.networkDroppedFrames;
    dst.pacerDroppedFrames += src.pacerDroppedFrames;
    dst.hitDeadlines += src.hitDeadlines;
    dst.missedDeadlines += src.missedDeadlines;
    dst.totalReassemblyTimeUs += src.totalReassemblyTimeUs;
    dst.totalDecodeTimeUs += src.totalDecodeTimeUs;
    dst.totalPacerTimeUs += src.totalPacerTimeUs;
    dst.totalRenderTimeUs += src.totalRenderTimeUs;
    dst.totalPreWaitTimeUs += src.totalPreWaitTimeUs;
    dst.totalPresentTimeUs += src.totalPresentTimeUs;
    dst.presentMode = src.presentMode;

    if (dst.minHostProcessingLatency == 0) {
        dst.minHostProcessingLatency = src.minHostProcessingLatency;
    }
    else if (src.minHostProcessingLatency != 0) {
        dst.minHostProcessingLatency = std::min(dst.minHostProcessingLatency, src.minHostProcessingLatency);
    }
    dst.maxHostProcessingLatency = std::max(dst.maxHostProcessingLatency, src.maxHostProcessingLatency);
    dst.totalHostProcessingLatency += src.totalHostProcessingLatency;
    dst.framesWithHostProcessingLatency += src.framesWithHostProcessingLatency;

    if (!LiGetEstimatedRttInfo(&dst.lastRtt, &dst.lastRttVariance)) {
        dst.lastRtt = 0;
        dst.lastRttVariance = 0;
    }
    else {
        // Our logic to determine if RTT is valid depends on us never
        // getting an RTT of 0. ENet currently ensures RTTs are >= 1.
        SDL_assert(dst.lastRtt > 0);
    }

    // Initialize the measurement start point if this is the first video stat window
    if (!dst.measurementStartUs) {
        dst.measurementStartUs = src.measurementStartUs;
    }

    // The following code assumes the global measure was already started first
    SDL_assert(dst.measurementStartUs <= src.measurementStartUs);

    double timeDiffSecs = (double) (LiGetMicroseconds() - dst.measurementStartUs) / 1000000.0;
    dst.totalFps = (double) dst.totalFrames / timeDiffSecs;
    dst.receivedFps = (double) dst.receivedFrames / timeDiffSecs;
    dst.decodedFps = (double) dst.decodedFrames / timeDiffSecs;
    dst.renderedFps = (double) dst.renderedFrames / timeDiffSecs;
}

void Stats::formatVideoStats(VIDEO_STATS& stats, char* output, size_t length)
{
    int offset = 0;
    const char* codecString;
    int ret = -1;

    // Start with an empty string
    output[offset] = 0;

    switch (m_VideoFormat) {
        case VIDEO_FORMAT_H264:
            codecString = "H.264";
            break;

        case VIDEO_FORMAT_H264_HIGH8_444:
            codecString = "H.264 4:4:4";
            break;

        case VIDEO_FORMAT_H265:
            codecString = "HEVC";
            break;

        case VIDEO_FORMAT_H265_REXT8_444:
            codecString = "HEVC 4:4:4";
            break;

        case VIDEO_FORMAT_H265_MAIN10:
            if (LiGetCurrentHostDisplayHdrMode()) {
                codecString = "HEVC 10-bit HDR";
            }
            else {
                codecString = "HEVC 10-bit SDR";
            }
            break;

        case VIDEO_FORMAT_H265_REXT10_444:
            if (LiGetCurrentHostDisplayHdrMode()) {
                codecString = "HEVC 10-bit HDR 4:4:4";
            }
            else {
                codecString = "HEVC 10-bit SDR 4:4:4";
            }
            break;

        case VIDEO_FORMAT_AV1_MAIN8:
            codecString = "AV1";
            break;

        case VIDEO_FORMAT_AV1_HIGH8_444:
            codecString = "AV1 4:4:4";
            break;

        case VIDEO_FORMAT_AV1_MAIN10:
            if (LiGetCurrentHostDisplayHdrMode()) {
                codecString = "AV1 10-bit HDR";
            }
            else {
                codecString = "AV1 10-bit SDR";
            }
            break;

        case VIDEO_FORMAT_AV1_HIGH10_444:
            if (LiGetCurrentHostDisplayHdrMode()) {
                codecString = "AV1 10-bit HDR 4:4:4";
            }
            else {
                codecString = "AV1 10-bit SDR 4:4:4";
            }
            break;

        default:
            codecString = "UNKNOWN";
            break;
    }

    if (stats.receivedFps > 0) {
        ret = snprintf(&output[offset],
                       length - offset,
                       "Video stream: %dx%d %.2f FPS (%s)\n",
                       m_Width,
                       m_Height,
                       stats.totalFps,
                       codecString);
        if (ret < 0 || (size_t) ret >= (length - offset)) {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Error: Stats::formatVideoStats length overflow");
            return;
        }

        offset += ret;

        double avgVideoMbps = m_bwTracker.GetAverageMbps();
        double peakVideoMbps = m_bwTracker.GetPeakMbps();
        const RTP_VIDEO_STATS* rtpVideoStats = LiGetRTPVideoStats();
        float fecOverhead = (float) rtpVideoStats->packetCountFec * 1.0 /
                            (rtpVideoStats->packetCountVideo + rtpVideoStats->packetCountFec);

        ret = snprintf(&output[offset],
                       length - offset,
                       "Bitrate: %.1f Mbps, +%.0f%% FEC, Peak (%us): %.1f\n"
                       "Incoming frame rate from network: %.2f FPS\n"
                       "Decoding frame rate: %.2f FPS\n"
                       "Rendering frame rate: %.2f FPS (%s, %s)\n",
                       avgVideoMbps,
                       fecOverhead * 100.0,
                       m_bwTracker.GetWindowSeconds(),
                       peakVideoMbps,
                       stats.receivedFps,
                       stats.decodedFps,
                       stats.renderedFps,
                       FramePacer::instance().GetPacingMode() == StreamingPreferences::FRAME_PACING_IMMEDIATE ?
                           "immediate" :
                           "display-locked",
                       stats.presentMode == StreamingPreferences::PRESENT_VRR      ? "VRR" :
                       stats.presentMode == StreamingPreferences::PRESENT_FIXED    ? "fixed vsync" :
                       stats.presentMode == StreamingPreferences::PRESENT_NO_VSYNC ? "no vsync" :
                                                                                     "-");
        if (ret < 0 || (size_t) ret >= (length - offset)) {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Error: Stats::formatVideoStats length overflow");
            return;
        }

        offset += ret;
    }

    if (stats.framesWithHostProcessingLatency > 0) {
        ret = snprintf(&output[offset],
                       length - offset,
                       "Host processing latency min/max/average: %.1f/%.1f/%.1f ms\n",
                       (double) stats.minHostProcessingLatency / 10,
                       (double) stats.maxHostProcessingLatency / 10,
                       (double) stats.totalHostProcessingLatency / 10 / stats.framesWithHostProcessingLatency);
        if (ret < 0 || (size_t) ret >= (length - offset)) {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Error: Stats::formatVideoStats length overflow");
            return;
        }

        offset += ret;
    }
    else {
        // If all frames are duplicates this can happen, but let's avoid having the whole stats area change height
        ret = snprintf(&output[offset], length - offset, "Host processing latency min/max/avg: -/-/- ms\n");
        if (ret < 0 || (size_t) ret >= (length - offset)) {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Error: Stats::formatVideoStats length overflow");
            return;
        }

        offset += ret;
    }

    if (stats.renderedFrames != 0) {
        char rttString[32];

        if (stats.lastRtt != 0) {
            snprintf(rttString, sizeof(rttString), "%u ms (variance: %u ms)", stats.lastRtt, stats.lastRttVariance);
        }
        else {
            snprintf(rttString, sizeof(rttString), "N/A");
        }

        ret = snprintf(&output[offset],
                       length - offset,
                       "Frames dropped by your network connection: %.2f%%\n"
                       "Frames dropped due to network jitter: %.2f%%\n"
                       "Average network latency: %s\n"
                       "Average reassembly/decoding time: %.2f/%.2f ms\n"
                       "Average frames in queue: %.1f\n"
                       "Average frame queue/render/present delay: %.2f/%.2f/%.2f ms\n",
                       stats.totalFrames ? (double) stats.networkDroppedFrames / stats.totalFrames * 100 : 0.0f,
                       stats.totalFrames ? (double) stats.pacerDroppedFrames / stats.totalFrames * 100 : 0.0f,
                       rttString,
                       stats.decodedFrames ? (double) stats.totalReassemblyTimeUs / 1000.0 / stats.decodedFrames : 0.0f,
                       stats.decodedFrames ? (double) stats.totalDecodeTimeUs / 1000.0 / stats.decodedFrames : 0.0f,
                       m_avgQueueSize,
                       stats.renderedFrames ? (double) stats.totalPacerTimeUs / 1000.0 / stats.renderedFrames : 0.0f,
                       stats.renderedFrames ? (double) stats.totalRenderTimeUs / 1000.0 / stats.renderedFrames : 0.0f,
                       stats.renderedFrames ? (double) stats.totalPresentTimeUs / 1000.0 / stats.renderedFrames : 0.0f);
        if (ret < 0 || (size_t) ret >= (length - offset)) {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Error: Stats::formatVideoStats length overflow");
            return;
        }

        offset += ret;
    }
}

#ifndef IMGUI_DISABLE

struct clampData {
    const float* values;
    float maxVal;
};

static inline ImPlotPoint clampGetter(int idx, void* data)
{
    const clampData* c = static_cast<const clampData*>(data);
    return ImPlotPoint(idx, std::min(c->values[idx], c->maxVal));
}

void Stats::RenderGraphs()
{
    if (!m_ShowGraphs) {
        return;
    }

    // we malloc a buffer for each stat only once and reuse it each frame
    // for performance.
    SDL_assert(PlotCount == 7);
    static float* buffers[7] = {
        (float*) malloc(sizeof(float) * 512),
        (float*) malloc(sizeof(float) * 512),
        (float*) malloc(sizeof(float) * 512),
        (float*) malloc(sizeof(float) * 512),
        (float*) malloc(sizeof(float) * 512),
        (float*) malloc(sizeof(float) * 512),
        (float*) malloc(sizeof(float) * 512)
    };

    static int selectedPlot = -1;
    static bool showSelectedPlot = false;

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImVec2 workSize = vp->WorkSize;  // usable size

    // Scale relative to 4K
    float scale = std::min(workSize.x / 3840.0f, workSize.y / 2160.0f);
    float graphW = 850.0f * scale;
    float graphH = 150.0f * scale;
    float opacity = 0.7f;

    // Row 1: 3 graphs
    // Row 2: 3 graphs
    float itemSpacingX = ImGui::GetStyle().ItemSpacing.x;
    float itemSpacingY = ImGui::GetStyle().ItemSpacing.y;
    float row1Width = (3 * graphW) + (2 * itemSpacingX);
    float totalHeight = (2 * graphH) + (2 * itemSpacingY) + 25;

    // Anchor to top-right
    ImVec2 windowPos(workSize.x - 10.0f, 0.0f);  // 10px margin
    ImGui::SetNextWindowPos(windowPos, ImGuiCond_Always, ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowSize(ImVec2(row1Width, totalHeight), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoNavFocus |
                             ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoSavedSettings;
    ImGui::Begin("##Stats", nullptr, flags);

    auto draw_plot = [&](int i, float width, float height) {
        Plot& plot = ImGuiPlots::instance().get(i);

        float minY = 0.0f;
        float maxY = 0.0f;
        std::size_t countF = plot.buffer.copyInto(buffers[i], 512, minY, maxY);
        float avgF = plot.buffer.average();
        if (!countF) {
            return;
        }

        char label[64];
        switch (plot.desc.labelType) {
            case PLOT_LABEL_MIN_MAX_AVG:
                snprintf(label,
                         sizeof(label),
                         "%s  %.1f / %.1f / %.1f %s",
                         plot.desc.title,
                         minY,
                         maxY,
                         avgF,
                         plot.desc.unit);
                break;
            case PLOT_LABEL_MIN_MAX_AVG_INT:
                snprintf(label,
                         sizeof(label),
                         "%s  %d / %d / %.1f %s",
                         plot.desc.title,
                         (int) minY,
                         (int) maxY,
                         avgF,
                         plot.desc.unit);
                break;
            case PLOT_LABEL_TOTAL_INT:
                snprintf(label, sizeof(label), "%s  %d %s", plot.desc.title, (int) plot.buffer.sum(), plot.desc.unit);
                break;
        }
        float scaleMin = FLT_MAX;
        float scaleMax = FLT_MAX;
        if (!std::isnan(plot.desc.scaleTarget)) {
            // optionally center the graph on a target such as the ideal frametime
            float ideal = (float) plot.desc.scaleTarget;
            scaleMin = ideal - (2 * ideal);
            scaleMax = ideal + (2 * ideal);
        }
        if (!std::isnan(plot.desc.scaleMin)) {
            scaleMin = plot.desc.scaleMin;
        }
        if (!std::isnan(plot.desc.scaleMax)) {
            scaleMax = plot.desc.scaleMax;
        }

        ImGui::PushID(i);

        ImVec2 plotPos = ImGui::GetCursorScreenPos();

        ImPlot::PushStyleColor(ImPlotCol_PlotBg, DevUIColors.colors.plotBg);
        ImPlot::PushStyleColor(ImPlotCol_FrameBg, DevUIColors.colors.plotBg);
        ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding, ImVec2(0, 0));
        ImPlot::PushStyleVar(ImPlotStyleVar_PlotBorderSize, 0.0f);

        ImPlotFlags plotFlags = ImPlotFlags_CanvasOnly | ImPlotFlags_NoInputs;
        ImPlotAxisFlags axisFlags = ImPlotAxisFlags_NoDecorations;

        if (ImPlot::BeginPlot(label, ImVec2(width, height), plotFlags)) {
            ImPlot::SetupAxes(nullptr, nullptr, axisFlags, axisFlags);
            ImPlot::SetupAxisLimits(ImAxis_X1, 0.0, (double) countF - 1.0, ImGuiCond_Always);

            float labelY = 0.0f;
            if (scaleMin != FLT_MAX && scaleMax != FLT_MAX) {
                ImPlot::SetupAxisLimits(ImAxis_Y1, scaleMin, scaleMax, ImGuiCond_Always);
                labelY = scaleMax;
            }
            else {
                double pad = std::max(1.0f, (maxY - minY) * 0.1f);
                ImPlot::SetupAxisLimits(ImAxis_Y1, minY - pad, maxY + pad, ImGuiCond_Always);
                labelY = maxY + pad;
            }
            ImPlot::SetupFinish();

            ImPlotSpec spec;
            spec.LineColor = DevUIColors.colors.plotLine;  // green
            clampData ctx {buffers[i], plot.desc.clampMax};
            ImPlot::PlotLineG("values", clampGetter, &ctx, (int) countF, spec);

            // Plot the label over the graph to save space.
            ImPlot::PlotText(label, (double) countF / 2.0, labelY * 0.80);

            ImPlot::EndPlot();
        }

        ImPlot::PopStyleVar(2);
        ImPlot::PopStyleColor(2);

        // Overlay a click target over the plot that was just drawn.
        ImGui::SetCursorScreenPos(plotPos);
        if (ImGui::InvisibleButton("plot_button", ImVec2(width, height))) {
            selectedPlot = i;
            showSelectedPlot = true;
        }

        ImGui::PopID();
    };

    const int row1[3] = {PLOT_FRAMETIME, PLOT_DROPPED_NETWORK, PLOT_PRESENT_DELAY};
    for (int c = 0; c < 3; ++c) {
        if (c > 0) {
            ImGui::SameLine(0.0f, itemSpacingX);
        }
        draw_plot(row1[c], graphW, graphH);
    }

    ImGui::Dummy(ImVec2(1.0f, itemSpacingY));
    const int row2[3] = {PLOT_HOST_FRAMETIME, PLOT_DROPPED_PACER, PLOT_BANDWIDTH};
    for (int c = 0; c < 3; ++c) {
        if (c > 0) {
            ImGui::SameLine(0.0f, itemSpacingX);
        }
        draw_plot(row2[c], graphW, graphH);
    }

    ImGui::End();

    if (showSelectedPlot && selectedPlot >= 0) {
        Plot& plot = ImGuiPlots::instance().get(selectedPlot);

        float minY = 0.0f;
        float maxY = 0.0f;
        std::size_t countF = plot.buffer.copyInto(buffers[selectedPlot], 512, minY, maxY);

        if (!countF) {
            showSelectedPlot = false;
            return;
        }

        float scaleMin = FLT_MAX;
        float scaleMax = FLT_MAX;

        if (!std::isnan(plot.desc.scaleTarget)) {
            float ideal = (float) plot.desc.scaleTarget;
            scaleMin = ideal - (2 * ideal);
            scaleMax = ideal + (2 * ideal);
        }
        if (!std::isnan(plot.desc.scaleMin)) {
            scaleMin = plot.desc.scaleMin;
        }
        if (!std::isnan(plot.desc.scaleMax)) {
            scaleMax = plot.desc.scaleMax;
        }

        ImGui::SetNextWindowSize(ImVec2(900.0f, 300.0f), ImGuiCond_FirstUseEver);

        char windowTitle[128];
        snprintf(windowTitle, sizeof(windowTitle), "%s###StatsPlotDetail", plot.desc.title);

        if (ImGui::Begin(windowTitle, &showSelectedPlot, ImGuiWindowFlags_None)) {
            ImVec2 plotSize = ImGui::GetContentRegionAvail();
            plotSize.y = std::max(plotSize.y, 200.0f);

            if (ImPlot::BeginPlot("##DetailPlot", plotSize, ImPlotFlags_NoMouseText)) {
                ImPlotSpec spec;
                spec.LineColor = DevUIColors.colors.plotLine;
                spec.LineWeight = 1.5f;

                ImPlotStyle& style = ImPlot::GetStyle();
                style.Colors[ImPlotCol_PlotBg] = ImVec4(0.92f, 0.92f, 0.95f, 0.00f);
                style.Colors[ImPlotCol_AxisGrid] = ImVec4(0.0f, 0.0f, 0.0f, 1.0f);
                style.Colors[ImPlotCol_AxisTick] = ImVec4(0.0f, 0.0f, 0.0f, 1.0f);

                int axFlags = ImPlotAxisFlags_NoLabel | ImPlotAxisFlags_NoSideSwitch | ImPlotAxisFlags_NoHighlight;
                ImPlot::SetupAxes(nullptr, nullptr, axFlags, axFlags);
                ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0f, 65.0f, ImGuiCond_Always);

                clampData ctx {buffers[selectedPlot], plot.desc.clampMax};
                ImPlot::PlotLineG("values", clampGetter, &ctx, (int) countF, spec);

                ImPlot::EndPlot();
            }
        }

        ImGui::End();
    }
}

#endif
