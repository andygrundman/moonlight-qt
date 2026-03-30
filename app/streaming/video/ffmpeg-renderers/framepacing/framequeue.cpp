#include "framequeue.h"

#include "SDL_compat.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <Limelight.h>

static inline double pts90kToSeconds(int64_t pts90k)
{
    return static_cast<double>(pts90k) / 90000.0;
}

FrameQueue& FrameQueue::instance()
{
    static FrameQueue inst;
    return inst;
}

FrameQueue::FrameQueue():
    m_Capacity(5),
    m_Count(0),
    m_Head(0),
    m_Tail(0),
    m_DroppedLast(false),
    m_HighWaterMark(3),
    m_Paused(true),
    m_ReadyTolerance90k(900), // 10ms
    m_DropLateTolerance90k(900) // 10ms
{
    m_Buffer.assign(m_Capacity, Entry {});
}

bool FrameQueue::isFrameIDR(AVFrame* frame) const
{
    return frame && frame->pict_type == AV_PICTURE_TYPE_I;
}

void FrameQueue::setPaused(bool paused)
{
    m_Paused.store(paused);
    if (paused) {
        // Wake any waiters so they can exit
        m_Cv.notify_all();
    }
}

void FrameQueue::start()
{
    clear();
    setPaused(false);
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "FrameQueue started");
}

void FrameQueue::stop()
{
    setPaused(true);
    clear();
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "FrameQueue stopped");
}

std::size_t FrameQueue::count() const
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    return static_cast<std::size_t>(m_Count);
}

bool FrameQueue::isEmpty() const
{
    return count() == 0;
}

void FrameQueue::clear()
{
    std::lock_guard<std::mutex> lock(m_Mutex);

    while (m_Count > 0) {
        Entry entry = popEntry();
        FQLog("! dropped frame in clear() [pts: %.3fs]\n", pts90kToSeconds(entry.frame->pts));
        dropFrame(entry.frame);
    }

    m_Head = 0;
    m_Tail = 0;
    m_Count = 0;
    m_DroppedLast = false;
    std::fill(m_Buffer.begin(), m_Buffer.end(), Entry {});
}

void FrameQueue::setHighWaterMark(int hwm)
{
    std::lock_guard<std::mutex> lock(m_Mutex);

    if (hwm < 1) {
        hwm = 1;
    }
    if (hwm > m_Capacity) {
        hwm = m_Capacity;
    }

    m_HighWaterMark = hwm;
}

void FrameQueue::setReadyTolerance90k(int64_t ticks)
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    m_ReadyTolerance90k = std::max<int64_t>(0, ticks);
}

void FrameQueue::setDropLateTolerance90k(int64_t ticks)
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    m_DropLateTolerance90k = std::max<int64_t>(0, ticks);
}

void FrameQueue::pushEntry(const Entry& entry)
{
    m_Buffer[m_Tail] = entry;
    m_Tail = (m_Tail + 1) % m_Capacity;
    ++m_Count;

    m_Cv.notify_one();

    FQLog("[-> %s pts: %.3fs] enqueue frame, queue size %d/%d\n",
          isFrameIDR(entry.frame) ? "IDR" : "P",
          pts90kToSeconds(entry.pts90k),
          m_Count,
          m_HighWaterMark);
}

FrameQueue::Entry FrameQueue::popEntry()
{
    if (m_Count == 0) {
        return {};
    }

    Entry entry = m_Buffer[m_Head];
    m_Buffer[m_Head] = {};
    m_Head = (m_Head + 1) % m_Capacity;
    --m_Count;
    return entry;
}

void FrameQueue::dropFrame(AVFrame* frame)
{
    if (frame) {
        //FQLog("! dropped frame [pts: %.3fs]\n", pts90kToSeconds(frame->pts));
        av_frame_free(&frame);
    }
}

int FrameQueue::unsafeEnqueue(AVFrame* frame, int frameDropTarget)
{
    int dropCount = 0;

    Entry entry;
    entry.frame = frame;
    entry.pts90k = frame ? frame->pts : 0;
    entry.enqueueTimeUs = LiGetMicroseconds();

    // - always accept IDR
    // - below HWM, accept normally
    // - above HWM, alternate between dropping newest and replacing oldest
    if (isFrameIDR(frame) || m_Count < frameDropTarget) {
        if (m_Count == m_Capacity) {
            Entry oldest = popEntry();
            if (oldest.frame) {
                FQLog("! dropped oldest frame, enqueue full [pts: %.3fs]\n", pts90kToSeconds(oldest.frame->pts));
                dropFrame(oldest.frame);
                dropCount = 1;
            }
        }

        pushEntry(entry);
        m_DroppedLast = false;
    }
    else {
        if (!m_DroppedLast) {
            FQLog("! dropped newest frame, enqueue alternate [pts: %.3fs]\n", pts90kToSeconds(frame->pts));
            dropFrame(frame);
            dropCount = 1;
            m_DroppedLast = true;
        }
        else {
            Entry oldest = popEntry();
            if (oldest.frame) {
                FQLog("! dropped oldest frame, enqueue alternate [pts: %.3fs]\n", pts90kToSeconds(oldest.frame->pts));
                dropFrame(oldest.frame);
                dropCount = 1;
            }

            pushEntry(entry);
            m_DroppedLast = false;
        }
    }

    return dropCount;
}

int FrameQueue::enqueue(AVFrame* frame)
{
    if (!frame) {
        return 0;
    }

    std::lock_guard<std::mutex> lock(m_Mutex);
    return unsafeEnqueue(frame, m_HighWaterMark);
}

void FrameQueue::waitForEnqueue(int num, double timeoutMs)
{
    std::unique_lock<std::mutex> lock(m_Mutex);

    if (m_Paused.load()) {
        return;
    }

    auto deadline =
        std::chrono::steady_clock::now() + std::chrono::microseconds(static_cast<long long>(timeoutMs * 1000.0));

    m_Cv.wait_until(lock, deadline, [&] {
        return m_Count >= num || m_Paused.load();
    });
}

AVFrame* FrameQueue::dequeue()
{
    std::lock_guard<std::mutex> lock(m_Mutex);

    if (m_Count == 0) {
        return nullptr;
    }

    Entry entry = popEntry();
    if (entry.frame) {
        FQLog("[<- pts: %.3fs] dequeue frame, queue size %d/%d\n",
              pts90kToSeconds(entry.pts90k),
              m_Count,
              m_HighWaterMark);
    }

    return entry.frame;
}

AVFrame* FrameQueue::dequeueWithTimeout(double timeoutSeconds)
{
    std::unique_lock<std::mutex> lock(m_Mutex);

    if (m_Paused.load()) {
        return nullptr;
    }

    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::microseconds(static_cast<long long>(timeoutSeconds * 1000000.0));

    while (m_Count == 0 && !m_Paused.load()) {
        if (m_Cv.wait_until(lock, deadline) == std::cv_status::timeout) {
            return nullptr;
        }
    }

    if (m_Count == 0) {
        return nullptr;
    }

    Entry entry = popEntry();
    if (entry.frame) {
        FQLog("[<- pts: %.3fs] dequeue frame, queue size %d/%d\n",
              pts90kToSeconds(entry.pts90k),
              m_Count,
              m_HighWaterMark);
    }

    return entry.frame;
}

bool FrameQueue::isReadyPts(int64_t pts90k, int64_t targetPts90k) const
{
    if (targetPts90k == 0) {
         // unknown target pts is accepted during startup
        return true;
    }
    return pts90k <= (targetPts90k + m_ReadyTolerance90k);
}

bool FrameQueue::isTooLatePts(int64_t pts90k, int64_t targetPts90k) const
{
    if (targetPts90k == 0) {
        // unknown target pts is accepted during startup
        return false;
    }
    return pts90k < (targetPts90k - m_DropLateTolerance90k);
}

FrameQueue::Entry FrameQueue::popOldestReadyEntry(int64_t targetPts90k, bool* hadReadyFrame)
{
    if (hadReadyFrame) {
        *hadReadyFrame = false;
    }

    if (m_Count == 0) {
        return {};
    }

    // FIFO queue: only examine the front entry.
    Entry& front = m_Buffer[m_Head];
    if (!front.frame) {
        return {};
    }

    if (!isReadyPts(front.pts90k, targetPts90k)) {
        return {};
    }

    if (hadReadyFrame) {
        *hadReadyFrame = true;
    }

    return popEntry();
}

FrameQueue::SelectResult FrameQueue::selectForTargetPts(int64_t targetPts90k, int maxAdvance)
{
    std::lock_guard<std::mutex> lock(m_Mutex);

    SelectResult result;
    result.reusedCurrent = true;

    if (m_Paused.load()) {
        result.queueDepth = m_Count;
        return result;
    }

    if (maxAdvance < 1) {
        maxAdvance = 1;
    }

    int droppedCount = 0;
    Entry selected {};

    // Drop stale frames from the front while there is at least one newer frame queued.
    // This avoids throwing away the only available candidate.
    while (m_Count > 1) {
        Entry& front = m_Buffer[m_Head];
        if (!front.frame || !isTooLatePts(front.pts90k, targetPts90k)) {
            break;
        }

        Entry dropped = popEntry();

        FQLog("! dropped stale queued frame [pts: %.3fs] [target: %.3fs]\n",
              pts90kToSeconds(dropped.pts90k),
              pts90kToSeconds(targetPts90k));

        dropFrame(dropped.frame);
        ++droppedCount;
    }

    // Advance through up to maxAdvance ready frames.
    // If more than one ready frame is consumed, older selected candidates are dropped.
    for (int i = 0; i < maxAdvance; ++i) {
        bool hadReadyFrame = false;
        Entry next = popOldestReadyEntry(targetPts90k, &hadReadyFrame);
        if (!next.frame) {
            break;
        }

        result.hadReadyFrame = result.hadReadyFrame || hadReadyFrame;

        if (selected.frame) {
            FQLog("! dropped superseded frame [pts: %.3fs] [target: %.3fs]\n",
                  pts90kToSeconds(selected.pts90k),
                  pts90kToSeconds(targetPts90k));

            dropFrame(selected.frame);
            ++droppedCount;
        }

        selected = next;
    }

    if (selected.frame) {
        result.frame = selected.frame;
        result.selectedPts90k = selected.pts90k;
        result.selectedEnqueueUs = selected.enqueueTimeUs;
        result.reusedCurrent = false;
    }

    result.droppedCount = droppedCount;
    result.queueDepth = m_Count;

    return result;
}
