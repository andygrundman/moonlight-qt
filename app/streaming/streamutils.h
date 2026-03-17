#pragma once

#include "SDL_compat.h"

struct RefreshRateRational {
    int32_t numerator;
    int32_t denominator;
    double hz;
    bool valid;
};

class StreamUtils
{
public:
    static
    Uint32 getPlatformWindowFlags();

    static
    SDL_Window* createTestWindow();

    static
    void scaleSourceToDestinationSurface(SDL_Rect* src, SDL_Rect* dst);

    static
    void screenSpaceToNormalizedDeviceCoords(SDL_FRect* rect, int viewportWidth, int viewportHeight);

    static
    void screenSpaceToNormalizedDeviceCoords(SDL_Rect* src, SDL_FRect* dst, int viewportWidth, int viewportHeight);

    static
    bool getNativeDesktopMode(int displayIndex, SDL_DisplayMode* mode, SDL_Rect* safeArea);

    static
    int getDisplayRefreshRate(SDL_Window* window);

    static
    RefreshRateRational getDisplayRefreshRateRational(SDL_Window* window);

    static
    bool hasFastAes();

    static
    int getDrmFdForWindow(SDL_Window* window, bool* needsClose);

    static
    int getDrmFd(bool preferRenderNode);

    static
    void enterAsyncLoggingMode();

    static
    void exitAsyncLoggingMode();
};
