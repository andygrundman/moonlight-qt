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
    BandwidthTracker m_bwTracker;
    float m_avgQueueSize;
    double m_avgMbpsSmoothed;
	int m_VideoFormat;
	int m_Width;
	int m_Height;
    bool m_ShowGraphs;
};
