#pragma once

#include "../framepacing/framepacer.h"

#include <atomic>
#include <mutex>

struct SDL_Window;

class DisplayLinkSource : public IVsyncSource
{
public:
    DisplayLinkSource();
    virtual ~DisplayLinkSource();

    virtual bool initialize(SDL_Window* window, int displayFps) override;
    virtual void stop() override;
    virtual bool isAsync() override;
    virtual double remainingMilliseconds() override;

    void displayLinkUpdate(double timestamp, double targetTimestamp);

private:
    DisplayLinkSource(const DisplayLinkSource &) = delete;
    DisplayLinkSource &operator=(const DisplayLinkSource &) = delete;

    std::mutex m_mtx;
    void* m_DisplayLinkTarget;
    std::atomic<double> m_TargetTimestamp;
};
