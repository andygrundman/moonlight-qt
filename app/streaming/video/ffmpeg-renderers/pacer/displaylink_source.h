#pragma once

#include "pacer.h"

#include <atomic>
#include <mutex>

struct SDL_Window;

class DisplayLinkSource : public IVsyncSource
{
public:
    // Singleton (may switch to this)
	// static DisplayLinkSource& instance();

    DisplayLinkSource(Pacer* pacer);
    virtual ~DisplayLinkSource();

    virtual bool initialize(SDL_Window* window, int displayFps) override;
    virtual void setPacer(Pacer* pacer) override;
    virtual void stop() override;
    virtual bool isAsync() override;
    virtual void setExtraCallback(VsyncCallback cb, void *ctx) override;
    virtual double remainingMilliseconds() override;

    void displayLinkUpdate(double timestamp, double targetTimestamp);

private:
    DisplayLinkSource(const DisplayLinkSource &) = delete;
    DisplayLinkSource &operator=(const DisplayLinkSource &) = delete;

    std::mutex m_mtx;
    Pacer* m_Pacer;
    void* m_DisplayLinkTarget;
    std::atomic<double> m_TargetTimestamp;
    VsyncCallbackData m_Callback;
};
