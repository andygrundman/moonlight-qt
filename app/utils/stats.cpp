// BandwidthTracker is a simple class that provides more accurate bandwidth stats compared to
// the current+last 2-second model used by other Moonlight stats.
//
// Each data point is stored as a timestamp/bytes/average tuple within a deque (earliest in
// front and newest in back) and expires after a configurable number of window seconds (default 30).
//
// getAverageMbps() calculates the average of collected bytes, minus the most recent 500ms
// of data which could skew the result too high.
//
// getPeakMbps() returns the highest average value from the collected data points. This is
// done so that peak decays after the window seconds have elapsed.

#include "stats.h"

using namespace std::chrono;

BandwidthTracker::BandwidthTracker(int windowDuration)
  : windowDuration(seconds(windowDuration))
{}

void BandwidthTracker::addBytes(size_t bytes)
{
    std::lock_guard<std::mutex> lock(mtx);
    auto now = steady_clock::now();
    _cleanOldEntries(now);

    // store new data point, including avg of past points
    double avg = _getAverageMbpsFrom(now);
    events.emplace_back(now, bytes, avg);
}

double BandwidthTracker::getAverageMbps()
{
    std::lock_guard<std::mutex> lock(mtx);
    auto now = steady_clock::now();
    _cleanOldEntries(now);

    return _getAverageMbpsFrom(now);
}

double BandwidthTracker::getPeakMbps()
{
    std::lock_guard<std::mutex> lock(mtx);
    auto now = steady_clock::now();
    _cleanOldEntries(now);

    double peak = 0.0;
    for (const auto& entry : events) {
        double avg = std::get<2>(entry); // the average recorded at this time
        if (avg > peak) {
            peak = avg;
        }
    }
    return peak;
}

/// private methods

void BandwidthTracker::_cleanOldEntries(time_point<steady_clock> now)
{
    while (!events.empty() && (now - std::get<0>(events.front())) > windowDuration) {
        events.pop_front();
    }
}

double BandwidthTracker::_getAverageMbpsFrom(time_point<steady_clock> now)
{
    uint64_t total = 0;
    steady_clock::time_point oldestTime = {};

    for (const auto& entry : events) {
        // don't include very recent data which can skew the average
        auto timestamp = std::get<0>(entry);
        if (now - timestamp > milliseconds(500)) {
            total += std::get<1>(entry);
            if (oldestTime == steady_clock::time_point{}) {
                oldestTime = timestamp;
            }
        }
    }

    if (total > 0) {
        auto elapsed = duration<double>(now - oldestTime).count();
        return (total * 8.0 / 1000000.0 / elapsed);
    }
    else {
        return 0.0;
    }
}
