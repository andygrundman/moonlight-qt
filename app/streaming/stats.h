#pragma once

#include "bandwidth.h"
#include "floatbuffer.h"
#include "qpc.h"
#include "settings/streamingpreferences.h"

#include <mutex>
#include <string>

extern "C"
{
#include "Limelight.h"
#include <libavcodec/avcodec.h>
}

typedef struct _VIDEO_STATS {
    uint32_t receivedFrames;
    uint32_t decodedFrames;
    uint32_t renderedFrames;
    uint32_t totalFrames;
    uint32_t networkDroppedFrames;
    uint32_t pacerDroppedFrames;
    uint32_t hitDeadlines;
    uint32_t missedDeadlines;
    uint16_t minHostProcessingLatency;
    uint16_t maxHostProcessingLatency;
    uint32_t totalHostProcessingLatency;
    uint32_t framesWithHostProcessingLatency;
    uint32_t totalReassemblyTimeUs;
    uint64_t totalDecodeTimeUs;
    uint64_t totalPacerTimeUs;
    uint64_t totalPreWaitTimeUs;
    uint64_t totalRenderTimeUs;
    uint64_t totalPresentTimeUs;
    int presentMode;
    uint32_t lastRtt;
    uint32_t lastRttVariance;
    double totalFps;
    double receivedFps;
    double decodedFps;
    double renderedFps;
    uint64_t measurementStartUs;
} VIDEO_STATS, *PVIDEO_STATS;

// Pre-decode frame arrival tracking, one window per second of receive time.
// All times come from DECODE_UNIT receiveTimeUs (first packet of each frame),
// so this measures the true network input rate before reassembly and decode.
typedef struct _ARRIVAL_STATS {
    uint64_t windowStartUs;  // receiveTimeUs at window start
    uint64_t firstRxUs;  // receiveTimeUs of first frame in window
    uint64_t lastRxUs;  // receiveTimeUs of most recent frame
    uint32_t firstRtpTs;  // rtpTimestamp (90kHz) of first frame in window
    uint32_t lastRtpTs;  // rtpTimestamp of most recent frame
    uint32_t frames;  // frames received this window
    uint32_t drops;  // frames lost on the network this window
    uint32_t bursts;  // frames that arrived at < half their rtp spacing
    uint32_t stalls;  // frames that arrived at > 2x rtp spacing
    double jitterSumMs;  // sum of |arrival delta - rtp delta|
    double deltaMaxMs;  // largest arrival delta this window
    int queueAtWindowStart;  // FrameQueue depth when the window began
} ARRIVAL_STATS;

class Stats
{
  public:
	// Singleton
    static Stats& instance();

	void SetMetadata(int videoFormat, int width, int height);
    bool GetShowGraphs();
    void SetShowGraphs(bool enabled);
    bool ShouldUpdateDisplay(bool isVisible, char* output, size_t length);
	void LogGlobalVideoStats();
	void RenderGraphs();
    void DrawPlotLarge(int index);

    // submitters for various types of data
    void SubmitVideoBytesAndReassemblyTime(PDECODE_UNIT decodeUnit, uint32_t droppedFrames);
    void SubmitDecodeTimeUs(uint64_t decodeUs);
    void SubmitDroppedFrame(int count);
    void SubmitAvgQueueSize(float avgQueueSize);
    void SubmitPacerTime(uint64_t pacerTimeUs);
    void SubmitPresentTimeUs(uint64_t presentTimeUs, int presentMode);
    void SubmitRenderStats(double preWaitTimeMs, double renderTimeMs, bool hitDeadline);
    void TrackFrameArrival(AVFrame *frame, int droppedFramesPacer);

  private:
	Stats();
	Stats(const Stats&) = delete;
	Stats& operator=(const Stats&) = delete;

    void addVideoStats(VIDEO_STATS& src, VIDEO_STATS& dst);
    void formatVideoStats(VIDEO_STATS& stats, char* output, size_t length);

    std::mutex m_mutex;

    // Moonlight stats overlay
    VIDEO_STATS m_ActiveWndVideoStats;
    VIDEO_STATS m_LastWndVideoStats;
    VIDEO_STATS m_GlobalVideoStats;
    ARRIVAL_STATS m_ArrivalStats;
    BandwidthTracker m_bwTracker;
    float m_avgQueueSize;
    double m_avgMbpsSmoothed;
	int m_VideoFormat;
	int m_Width;
	int m_Height;
    bool m_ShowGraphs;
};
