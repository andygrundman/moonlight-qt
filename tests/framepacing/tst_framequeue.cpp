#include "streaming/video/ffmpeg-renderers/framepacing/framequeue.h"
#include <atomic>
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <thread>
extern "C" {
#include <libavutil/mem.h>
}

extern "C" uint64_t LiGetMicroseconds()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

namespace {
std::atomic<int> allocated{0}, released{0};

void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

AVFrame* frame(int64_t pts)
{
    auto f = av_frame_alloc();
    require(f != nullptr, "allocate frame");
    auto data = static_cast<uint8_t*>(av_malloc(1));
    f->buf[0] = av_buffer_create(data, 1, [](void*, uint8_t* bytes) {
        ++released;
        av_free(bytes);
    }, nullptr, 0);
    require(f->buf[0] != nullptr, "allocate frame buffer");
    ++allocated;
    f->data[0] = data;
    f->linesize[0] = 1;
    f->format = AV_PIX_FMT_GRAY8;
    f->width = f->height = 1;
    f->pts = pts;
    // Already decoded IDR frames are also safe to discard before presentation.
    f->pict_type = AV_PICTURE_TYPE_I;
    return f;
}
}

int main()
{
    try {
        auto& q = FrameQueue::instance();
        q.start();
        int dropped = 99;
        require(q.dequeueLatest(dropped) == nullptr && dropped == 0, "empty selection");

        q.enqueue(frame(1));
        auto selected = q.dequeueLatest(dropped);
        require(selected && selected->pts == 1 && dropped == 0 && q.isEmpty(), "single frame");
        av_frame_free(&selected);

        const int before = released;
        q.enqueue(frame(2)); q.enqueue(frame(3)); q.enqueue(frame(4));
        selected = q.dequeueLatest(dropped);
        require(selected && selected->pts == 4 && dropped == 2 && q.isEmpty(), "freshest of a backlog");
        require(released == before + 2, "discarded frames released exactly once");
        av_frame_free(&selected);

        // Normal dequeue remains FIFO for display-locked pacing.
        q.enqueue(frame(5)); q.enqueue(frame(6));
        selected = q.dequeue();
        require(selected && selected->pts == 5 && q.count() == 1, "FIFO selection unchanged");
        av_frame_free(&selected);
        q.clear();

        // Force the ring's head/tail to wrap and exercise full-queue replacement.
        for (int i = 0; i < 40; ++i) {
            for (int j = 0; j < 7; ++j) q.enqueue(frame(i * 10 + j));
            selected = q.dequeueLatest(dropped);
            require(selected && selected->pts == i * 10 + 6 && dropped == 4, "wrapped full queue");
            av_frame_free(&selected);
        }

        // A GPU/renderer reference must survive dropping the queue's reference.
        auto retained = frame(1000);
        auto gpuReference = av_frame_clone(retained);
        require(gpuReference != nullptr, "retain renderer frame");
        const int retainedBefore = released;
        q.enqueue(retained); q.enqueue(frame(1001));
        selected = q.dequeueLatest(dropped);
        require(selected && selected->pts == 1001 && released == retainedBefore, "retained surface survives drop");
        av_frame_free(&selected); av_frame_free(&gpuReference);
        require(released == retainedBefore + 2, "last references release both surfaces");

        // Enqueue and selection must not duplicate or leak ownership under contention.
        std::atomic<bool> finished{false};
        std::thread producer([&] {
            for (int i = 1; i <= 10000; ++i) q.enqueue(frame(i));
            finished.store(true);
        });
        int64_t last = 0;
        bool monotonic = true;
        while (!finished.load() || !q.isEmpty()) {
            selected = q.dequeueLatest(dropped);
            if (selected) {
                monotonic = monotonic && selected->pts > last;
                last = selected->pts;
                av_frame_free(&selected);
            }
        }
        producer.join();
        require(monotonic, "concurrent selection is monotonic");
        require(last == 10000, "concurrent selection reaches newest frame");
        q.stop();
        require(allocated == released, "all frame buffers released exactly once");
        std::puts("Frame queue freshness, FIFO, wraparound, retained surfaces and concurrency passed");
    }
    catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return 1;
    }
    return 0;
}
