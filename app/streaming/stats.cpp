#include "stats.h"

#include "streaming/video/ffmpeg-renderers/framepacing/framepacer.h"
#include "streaming/video/ffmpeg-renderers/framepacing/framequeue.h"
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
    SDL_zero(m_ArrivalStats);

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

    if (ImGuiPlots::instance().isEnabled()) {
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

// TODO: figure this out: add drops from pacing, compare with Xbox, try 119.88
/*
00:01:01 - SDL Info (0): FQrx 16.00 host 15.99 jit 2.3 max 65.7 burst 0 | q 0>0 avg 1.0 lost 0
00:01:06 - SDL Info (0): FQrx 29.60 host 29.25 jit 2.8 max 66.0 burst 6 | q 0>2 avg 1.0 lost 0
00:01:11 - SDL Info (0): FQrx 62.06 host 62.06 jit 6.1 max 66.9 burst 22 | q 2>0 avg 1.1 lost 0
00:01:16 - SDL Info (0): FQrx 50.12 host 50.11 jit 6.2 max 66.2 burst 16 | q 0>0 avg 1.1 lost 0
00:01:21 - SDL Info (0): FQrx 114.45 host 114.46 jit 1.4 max 58.5 burst 33 | q 0>1 avg 1.9 lost 0
00:01:26 - SDL Info (0): FQrx 119.76 host 120.00 jit 1.3 max 30.1 burst 36 | q 1>0 avg 1.9 lost 0
00:01:31 - SDL Info (0): FQrx 120.25 host 120.00 jit 1.3 max 23.4 burst 38 | q 0>1 avg 1.9 lost 0
00:01:36 - SDL Info (0): FQrx 119.82 host 120.00 jit 1.2 max 25.7 burst 31 | q 1>0 avg 1.9 lost 0
00:01:41 - SDL Info (0): FQrx 120.17 host 119.99 jit 1.0 max 24.3 burst 24 | q 0>1 avg 1.9 lost 0
00:01:46 - SDL Info (0): FQrx 119.99 host 120.00 jit 0.8 max 19.3 burst 15 | q 1>1 avg 1.9 lost 0
00:01:51 - SDL Info (0): FQrx 119.80 host 120.00 jit 0.9 max 19.8 burst 15 | q 1>0 avg 1.9 lost 0
00:01:56 - SDL Info (0): FQrx 120.20 host 120.01 jit 1.0 max 19.4 burst 19 | q 0>1 avg 1.9 lost 0
00:02:01 - SDL Info (0): FQrx 120.00 host 119.99 jit 1.2 max 104.0 burst 28 | q 1>1 avg 1.9 lost 0
00:02:06 - SDL Info (0): FQrx 120.00 host 120.01 jit 1.4 max 26.7 burst 30 | q 1>1 avg 1.9 lost 0
00:02:11 - SDL Info (0): FQrx 120.00 host 120.00 jit 1.3 max 24.4 burst 26 | q 1>1 avg 1.9 lost 0
00:02:16 - SDL Info (0): FQrx 120.00 host 119.99 jit 1.3 max 24.4 burst 28 | q 1>1 avg 1.8 lost 0
00:02:21 - SDL Info (0): FQrx 120.02 host 120.01 jit 1.3 max 21.3 burst 24 | q 1>1 avg 1.8 lost 0
00:02:26 - SDL Info (0): FQrx 119.99 host 119.99 jit 1.3 max 25.7 burst 29 | q 1>1 avg 1.8 lost 0
00:02:31 - SDL Info (0): FQrx 119.91 host 120.00 jit 1.4 max 25.0 burst 32 | q 1>1 avg 1.9 lost 0
00:02:36 - SDL Info (0): FQrx 120.09 host 120.00 jit 1.4 max 27.8 burst 31 | q 1>1 avg 1.9 lost 0
00:02:41 - SDL Info (0): FQrx 34.62 host 35.06 jit 1.9 max 66.0 burst 4 | q 1>0 avg 1.7 lost 0
00:02:46 - SDL Info (0): FQrx 16.00 host 16.00 jit 2.7 max 75.5 burst 0 | q 0>0 avg 1.4 lost 0
00:02:51 - SDL Info (0): FQrx 16.00 host 16.00 jit 2.5 max 66.6 burst 0 | q 0>0 avg 1.1 lost 0
00:02:56 - SDL Info (0): FQrx 16.00 host 16.00 jit 2.3 max 68.8 burst 0 | q 0>0 avg 1.0 lost 0
*/

// Tracks frame arrival timing, before reassembly and decoding, using the receive
// time of the first packet of each frame. Once per second
// a compact summary is written to the debug log:
//
//   FQrx 59.94 host 59.96 jit 0.6 max 3.1 burst 0 | q 1>2 avg 1.4 loss 0
//
//   FQrx  - measured arrival rate in fps
//   host  - frame rate implied by RTP timestamp deltas, i.e. the server's send pacing.
//           host > display refresh rate means the queue must grow without drops
//   jit   - mean |arrival delta - rtp delta| in ms; how much the network distorts
//           the server's spacing
//   max   - largest gap between consecutive arrivals in ms
//   burst - frames that arrived at less than half their rtp spacing (back-to-back)
//   q     - FrameQueue depth at window start > end, and the pacer's running average
//   loss  - frames lost on the network during the window
void Stats::TrackFrameArrival(AVFrame *frame, int droppedFramesPacer)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    uint64_t rxUs = 0;
    if (frame->opaque_ref) {
        auto *data = reinterpret_cast<MLFrameData *>(frame->opaque_ref->data);
        rxUs = data->receiveTimeUs;
    }
    const uint32_t rtpTs = frame->pts;
    ARRIVAL_STATS& a = m_ArrivalStats;

    if (rxUs == 0) {
        return;
    }

    if (a.windowStartUs == 0) {
        // First frame of the stream seeds the window
        a.windowStartUs = rxUs;
        a.firstRxUs = a.lastRxUs = rxUs;
        a.firstRtpTs = a.lastRtpTs = rtpTs;
        a.frames = 1;
        a.queueAtWindowStart = (int) FrameQueue::instance().count();
        return;
    }

    const double rxDeltaMs = (double) (rxUs - a.lastRxUs) / 1000.0;
    const double rtpDeltaMs = (double) (uint32_t) (rtpTs - a.lastRtpTs) / 90.0;  // wrap-safe

    a.frames++;
    a.drops += droppedFramesPacer;
    a.deltaMaxMs = std::max(a.deltaMaxMs, rxDeltaMs);
    a.jitterSumMs += std::abs(rxDeltaMs - rtpDeltaMs);
    if (rtpDeltaMs > 0.0 && rxDeltaMs < rtpDeltaMs * 0.5) {
        a.bursts++;
    }
    if (rxDeltaMs >= rtpDeltaMs * 2.0) {
        a.stalls++;
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "  stall %u rxDeltaMs %.3f rtpDeltaMs %.3f drop %u",
            a.stalls, rxDeltaMs, rtpDeltaMs, droppedFramesPacer);
    }
    a.lastRxUs = rxUs;
    a.lastRtpTs = rtpTs;

    //ImGuiPlots::instance().observeFloat(PLOT_RX_FRAMETIME, (float) rxDeltaMs);

    if (rxUs - a.windowStartUs < 2 * 1000000) {
        return;
    }

    // Window complete, summarize and reset
    const uint32_t rxIntervals = a.frames - 1;
    const uint64_t rxSpanUs = a.lastRxUs - a.firstRxUs;
    const double rxFps = rxSpanUs ? (double) rxIntervals * 1e6 / (double) rxSpanUs : 0.0;

    // Pacer-dropped frames still advance the rtp clock, count them as intervals
    const uint32_t rtpIntervals = rxIntervals + a.drops;
    const uint32_t rtpSpan90k = (uint32_t) (a.lastRtpTs - a.firstRtpTs);  // wrap-safe
    const double hostFps = rtpSpan90k ? (double) rtpIntervals * 90000.0 / (double) rtpSpan90k : 0.0;

    const int queueNow = (int) FrameQueue::instance().count();

    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
        "FQrx %.2f host %.2f jit %.1f max %.1f burst %u stall %u | q %d>%d avg %.1f drop %u",
        rxFps,
        hostFps,
        rxIntervals ? a.jitterSumMs / rxIntervals : 0.0,
        a.deltaMaxMs,
        a.bursts,
        a.stalls,
        a.queueAtWindowStart,
        queueNow,
        m_avgQueueSize,
        a.drops);

    // Current frame becomes the first sample of the next window
    a.windowStartUs = rxUs;
    a.firstRxUs = rxUs;
    a.firstRtpTs = rtpTs;
    a.frames = 1;
    a.drops = 0;
    a.bursts = 0;
    a.stalls = 0;
    a.jitterSumMs = 0.0;
    a.deltaMaxMs = 0.0;
    a.queueAtWindowStart = queueNow;
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
            ImPlot::PlotLineG(plot.desc.unit, clampGetter, &ctx, (int) countF, spec);

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
                ImPlot::PlotLineG(plot.desc.unit, clampGetter, &ctx, (int) countF, spec);

                ImPlot::EndPlot();
            }
        }

        ImGui::End();
    }
}

#endif
