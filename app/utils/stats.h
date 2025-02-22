#pragma once

#include <chrono>
#include <deque>
#include <mutex>

class BandwidthTracker
{
public:
    BandwidthTracker(int windowDuration=30);
    void addBytes(size_t bytes);
    double getAverageMbps();
    double getPeakMbps();

private:
    void _cleanOldEntries(std::chrono::time_point<std::chrono::steady_clock> now);
    double _getAverageMbpsFrom(std::chrono::time_point<std::chrono::steady_clock> now);

    std::deque< std::tuple< std::chrono::steady_clock::time_point, size_t, double> > events;
    std::chrono::seconds windowDuration;
    std::mutex mtx;
};
