// Avoid conflict between AVFoundation and
// libavutil both defining AVMediaType
#define AVMediaType AVMediaType_FFmpeg
#include "displaylink_source.h"
#include "pacer.h"
#undef AVMediaType

#include <SDL_syswm.h>

#import <Cocoa/Cocoa.h>
#import <QuartzCore/CADisplayLink.h>

@interface DisplayLinkTarget : NSObject
{
    DisplayLinkSource* _source;
    CADisplayLink* _displayLink API_AVAILABLE(macos(14.0));
}

- (instancetype)initWithSource:(DisplayLinkSource*)source
                     forWindow:(NSWindow*)nswindow
                           fps:(int)fps;
- (void)link:(CADisplayLink *)update API_AVAILABLE(macos(14.0));
- (void)stop;

@end

// DisplayLinkSource& DisplayLinkSource::instance() {
// 	static DisplayLinkSource inst;
// 	return inst;
// }

DisplayLinkSource::DisplayLinkSource(Pacer* pacer)
    : m_Pacer(pacer),
      m_DisplayLinkTarget(nullptr),
      m_TargetTimestamp(0.0)
{
}

DisplayLinkSource::~DisplayLinkSource()
{
    stop();
}

bool DisplayLinkSource::initialize(SDL_Window* window, int fps)
{
    stop();

    SDL_SysWMinfo info;
    SDL_VERSION(&info.version);
    if (!SDL_GetWindowWMInfo(window, &info)) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                     "DisplayLinkSource: SDL_GetWindowWMInfo() failed: %s",
                     SDL_GetError());
        return false;
    }

    NSWindow* nswindow = (__bridge NSWindow *)info.info.cocoa.window;
    if (!nswindow) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                     "DisplayLinkSource: Cocoa window is null");
        return false;
    }

    DisplayLinkTarget* target =
        [[DisplayLinkTarget alloc] initWithSource:this
                                        forWindow:nswindow
                                              fps:fps];
    if (!target) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                     "DisplayLinkSource: error creating DisplayLink");
        return false;
    }

    m_DisplayLinkTarget = target;
    m_TargetTimestamp.store(0.0);

    {
        std::lock_guard<std::mutex> lock(m_mtx);
        m_Callback.callback = nullptr;
        m_Callback.ctx = nullptr;
    }

    return true;
}

void DisplayLinkSource::setPacer(Pacer* pacer) {
    std::lock_guard<std::mutex> lock(m_mtx);
    m_Pacer = pacer;
}

void DisplayLinkSource::stop()
{
    {
        std::lock_guard<std::mutex> lock(m_mtx);
        m_Callback.callback = nullptr;
        m_Callback.ctx = nullptr;
    }

    m_TargetTimestamp.store(0.0);

    DisplayLinkTarget* target = (DisplayLinkTarget*)m_DisplayLinkTarget;
    if (target) {
        [target stop];
        [target release];
        m_DisplayLinkTarget = nullptr;
    }
}

bool DisplayLinkSource::isAsync()
{
    return true;
}

void DisplayLinkSource::setExtraCallback(VsyncCallback cb, void *ctx)
{
    std::lock_guard<std::mutex> lock(m_mtx);
    m_Callback.callback = cb;
    m_Callback.ctx = ctx;
}

void DisplayLinkSource::displayLinkUpdate(double timestamp, double targetTimestamp)
{
    std::lock_guard<std::mutex> lock(m_mtx);
    m_TargetTimestamp.store(targetTimestamp);

    if (m_Pacer) {
        m_Pacer->signalVsync();
    }

    if (m_Callback.callback) {
        m_Callback.callback(timestamp, targetTimestamp, m_Callback.ctx);
    }
}

double DisplayLinkSource::remainingMilliseconds()
{
    const double targetTimestamp = m_TargetTimestamp.load();
    return (targetTimestamp - CACurrentMediaTime()) * 1000.0;
}

///////

@implementation DisplayLinkTarget

- (instancetype)initWithSource:(DisplayLinkSource*)source
                     forWindow:(NSWindow*)window
                            fps:(int)fps
{
    self = [super init];
    if (self) {
        _source = source;
        _displayLink = [window displayLinkWithTarget:self
                                            selector:@selector(link:)];
        if (!_displayLink) {
            [self release];
            return nil;
        }

        _displayLink.preferredFrameRateRange = CAFrameRateRangeMake(fps, fps, fps);
        [_displayLink addToRunLoop:[NSRunLoop mainRunLoop]
                           forMode:NSRunLoopCommonModes];
    }
    return self;
}

- (void)link:(CADisplayLink*)update
{
    DisplayLinkSource* source = _source;
    if (source) {
        source->displayLinkUpdate((double)update.timestamp, (double)update.targetTimestamp);
    }
}

- (void)stop
{
    _source = nullptr;

    if (_displayLink) {
        [_displayLink invalidate];
        _displayLink = nil;
    }
}

- (void)dealloc
{
    [self stop];
    [super dealloc];
}

@end
