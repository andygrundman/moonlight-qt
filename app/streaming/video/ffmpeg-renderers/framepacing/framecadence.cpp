#include "framecadence.h"
#include "framequeue.h"

#include <algorithm>

FrameCadence& FrameCadence::instance()
{
    static FrameCadence inst;
    return inst;
}

FrameCadence::FrameCadence():
    m_displayPeriodMs(1000.0 / 60.0),
    m_streamPeriodMs(1000.0 / 60.0)
{}

void FrameCadence::init(double hz, double fps)
{
    // Initialize the display period from the provided refresh rate.
    setDisplayHz(hz);

    if (fps <= 0.0) {
        fps = 60.0;
    }

    double periodMs = 1000.0 / fps;
    periodMs = std::clamp(periodMs, m_minStreamPeriodMs, m_maxStreamPeriodMs);

    m_streamPeriodEwmaMs = periodMs;
    m_lastPts90k = 0;

    // Initialize the stream period.
    m_streamPeriodMs.store(periodMs);

    // Start cadence phase at zero.
    m_phase = 0.0;
}

// Called by Pacer getNextVBlankQpc (main thread)
void FrameCadence::setDisplayHz(double hz)
{
    if (hz <= 0.0) {
        hz = 60.0;
    }

    const double periodMs = 1000.0 / hz;
    m_displayPeriodMs.store(periodMs);
}

// Decoder thread only
int64_t FrameCadence::observeFramePts(int64_t pts90k)
{
    if (m_lastPts90k > 0) {
        const int64_t deltaPts = pts90k - m_lastPts90k;
        if (deltaPts > 0) {
            double deltaMs = static_cast<double>(deltaPts) / 90.0;
            deltaMs = std::clamp(deltaMs, m_minStreamPeriodMs, m_maxStreamPeriodMs);

            // EWMA smoothing of stream period.
            m_streamPeriodEwmaMs = deltaMs * m_streamEwmaAlpha + m_streamPeriodEwmaMs * (1.0 - m_streamEwmaAlpha);

            // Publish the updated stream period.
            m_streamPeriodMs.store(m_streamPeriodEwmaMs);
            FQLog("[Cadence] new frame pts:%.3fs host delta %.3fms, streamPeriodMs %.3fms",
                (double)pts90k / 90.0, deltaMs, m_streamPeriodEwmaMs);
        }
    }

    int64_t last = m_lastPts90k;
    m_lastPts90k = pts90k;
    return last;
}

int FrameCadence::decideAdvanceCount()
{
    // Main thread only.
    // Uses a fractional phase accumulator to decide how many frames to advance.

    double streamMs = m_streamPeriodMs.load();
    double displayMs = m_displayPeriodMs.load();

    // Defensive fallbacks.
    if (streamMs <= 0.0) {
        streamMs = 1000.0 / 60.0;
    }
    if (displayMs <= 0.0) {
        displayMs = 1000.0 / 60.0;
    }

    // Stream frames per present interval.
    const double framesPerPresent = displayMs / streamMs;

    // Accumulate fractional frames owed.
    m_phase += framesPerPresent;

    FQLog("[Cadence] fpp %.3f, phase %.3f", framesPerPresent, m_phase);

    // Consume whole frames, clamped to the maximum allowed per present.
    int advanceCount = static_cast<int>(m_phase);
    if (advanceCount < 0) {
        advanceCount = 0;
    }
    else if (advanceCount > m_maxAdvancePerPresent) {
        advanceCount = m_maxAdvancePerPresent;
    }

    m_phase -= advanceCount;

    if (m_phase < 0.0) {
        m_phase = 0.0;
    }
    else if (m_phase >= 1.0) {
        m_phase -= static_cast<int>(m_phase);
    }

    FQLog("[Cadence] advanceCount %d, phase %.3f", advanceCount, m_phase);

    return advanceCount;
}

// Accessors

double FrameCadence::displayHz() const
{
    const double periodMs = m_displayPeriodMs.load();
    return (periodMs > 0.0) ? (1000.0 / periodMs) : 0.0;
}

double FrameCadence::streamFps() const
{
    const double periodMs = m_streamPeriodMs.load();
    return (periodMs > 0.0) ? (1000.0 / periodMs) : 0.0;
}
