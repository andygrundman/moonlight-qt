#import <Cocoa/Cocoa.h>
#import <CoreVideo/CoreVideo.h>

#include <SDL_syswm.h>
#include "streamutils.h"

extern "C" {
#include <libavutil/rational.h>
}

RefreshRateRational StreamUtils::getDisplayRefreshRateRational(SDL_Window* window)
{
    NSWindow* nsWindow = nil;
    RefreshRateRational result {0, 0, 0.0, false};

    SDL_SysWMinfo info;
    SDL_VERSION(&info.version);
    if (SDL_GetWindowWMInfo(window, &info) && info.subsystem == SDL_SYSWM_COCOA) {
        nsWindow = (__bridge NSWindow *)info.info.cocoa.window;
    }

    NSScreen* screen = nsWindow ? nsWindow.screen : NSScreen.mainScreen;
    if (screen == nil) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "NSScreen is nil");
        return result;
    }

    NSNumber* screenNumber = screen.deviceDescription[@"NSScreenNumber"];
    if (screenNumber == nil) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "NSScreenNumber is nil");
        return result;
    }

    CGDirectDisplayID displayID = (CGDirectDisplayID)screenNumber.unsignedIntValue;
    CVDisplayLinkRef displayLink = nullptr;
    CVReturn err = CVDisplayLinkCreateWithCGDisplay(displayID, &displayLink);
    if (err != kCVReturnSuccess || displayLink == nullptr) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Failed to create CVDisplayLink: %d", err);
        return result;
    }

    // CVDisplayLink is deprecated but I can't find another API with precise numerator/denominator
    CVTime period = CVDisplayLinkGetNominalOutputVideoRefreshPeriod(displayLink);
    CVDisplayLinkRelease(displayLink);

    if ((period.flags & kCVTimeIsIndefinite) ||
        period.timeValue <= 0 ||
        period.timeScale <= 0) {
        return result;
    }

    // NTSC 59.94 is represented on macOS as 24000000/400400, so try to reduce it
    av_reduce(
        &result.numerator, &result.denominator,
        period.timeScale, period.timeValue,
        120000);
    result.hz = (double)period.timeScale / (double)period.timeValue;
    result.valid = true;
    return result;
}
