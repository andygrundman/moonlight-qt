#pragma once

#include <Limelight.h>
#include "SDL_compat.h"

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

extern "C"
{
#include <libavcodec/avcodec.h>
}

class FrameQueue
{
  public:
    struct SelectResult {
        AVFrame* frame = nullptr;  // ownership transferred to caller
        int droppedCount = 0;  // number of queued/selected frames dropped
        int queueDepth = 0;  // queue depth after selection
        int64_t selectedPts90k = 0;
        uint64_t selectedEnqueueUs = 0;
        bool reusedCurrent = true;  // true if no new frame was selected
        bool hadReadyFrame = false;  // true if at least one queued frame was ready
    };

    // Singleton
    static FrameQueue& instance();

    void clear();
    std::size_t count() const;
    bool isEmpty() const;
    void setHighWaterMark(int hwm);
    void start();
    void stop();

    int enqueue(AVFrame* frame);
    AVFrame* dequeue();
    AVFrame* dequeueWithTimeout(double timeoutSeconds);
    void dropFrame(AVFrame* frame);
    void waitForEnqueue(int num = 1, double timeoutMs = 1000.0 / 60);

    SelectResult selectForTargetPts(int64_t targetPts90k, int maxAdvance = 2);
    void setReadyTolerance90k(int64_t ticks);
    void setDropLateTolerance90k(int64_t ticks);

  private:
    struct Entry {
        AVFrame* frame = nullptr;
        int64_t pts90k = 0;
        uint64_t enqueueTimeUs = 0;
    };

    FrameQueue();
    FrameQueue(const FrameQueue&) = delete;
    FrameQueue& operator=(const FrameQueue&) = delete;

    void setPaused(bool paused);

    bool paused() const
    {
        return m_Paused.load();
    }

    // Internal helpers (must be called with _mutex held)
    void pushEntry(const Entry& entry);
    Entry popEntry();
    Entry popOldestReadyEntry(int64_t targetPts90k, bool* hadReadyFrame);
    int unsafeEnqueue(AVFrame* frame, int frameDropTarget);

    bool isReadyPts(int64_t pts90k, int64_t targetPts90k) const;
    bool isTooLatePts(int64_t pts90k, int64_t targetPts90k) const;
    bool isFrameIDR(AVFrame* frame) const;

    mutable std::mutex m_Mutex;
    std::condition_variable m_Cv;

    std::vector<Entry> m_Buffer;
    int m_Capacity;
    int m_Count;
    int m_Head;
    int m_Tail;

    bool m_DroppedLast;
    int m_HighWaterMark;
    std::atomic<bool> m_Paused;

    int64_t m_ReadyTolerance90k;
    int64_t m_DropLateTolerance90k;
};

// Frame queue debugging, uncomment FRAME_QUEUE_VERBOSE or FRAME_QUEUE_VERBOSE_LIMITED
// When FQLog is enabled, the log volume can be intense, so LIMITED only logs data for a short time
#if !defined(NDEBUG)
// #define FRAME_QUEUE_VERBOSE
// #define FRAME_QUEUE_VERBOSE_LIMITED
#endif

#define FQLogForce(fmt, ...) SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "[tid %lu] [v %.3f] " fmt, \
                                                                       SDL_ThreadID(), \
                                                                       FramePacer::instance().GetVsyncTimestamp(), \
                                                                       ##__VA_ARGS__)

#ifdef FRAME_QUEUE_VERBOSE
#define FQLog(fmt, ...) FQLogForce(fmt, ...)
#else
    #ifdef FRAME_QUEUE_VERBOSE_LIMITED
        #include <atomic>
static std::atomic<int> g_fqlog_counter {0};
        #define FQLog(fmt, ...) \
            if (++g_fqlog_counter > 200 && g_fqlog_counter < 1000) \
            SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, fmt, ##__VA_ARGS__)
    #else
        #if defined(_MSC_VER)
            #define FQLog(...) __noop
        #else
            #define FQLog(fmt, ...) \
                do { \
                } while (0)
        #endif
    #endif
#endif
