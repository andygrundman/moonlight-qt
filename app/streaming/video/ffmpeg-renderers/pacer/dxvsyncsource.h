#pragma once

#include "../framepacing/framepacer.h"
#include "streaming/video/ffmpeg-renderers/d3d11va.h"

#include <SDL_syswm.h>
#include <atomic>

// from <D3dkmthk.h>
typedef LONG NTSTATUS;
typedef UINT D3DKMT_HANDLE;
typedef UINT D3DDDI_VIDEO_PRESENT_SOURCE_ID;
#define STATUS_SUCCESS ((NTSTATUS)0x00000000L)
#define STATUS_GRAPHICS_PRESENT_OCCLUDED ((NTSTATUS)0xC01E0006L)
typedef struct _D3DKMT_OPENADAPTERFROMHDC {
  HDC hDc;
  D3DKMT_HANDLE hAdapter;
  LUID AdapterLuid;
  D3DDDI_VIDEO_PRESENT_SOURCE_ID VidPnSourceId;
} D3DKMT_OPENADAPTERFROMHDC;
typedef struct _D3DKMT_CLOSEADAPTER {
  D3DKMT_HANDLE hAdapter;
} D3DKMT_CLOSEADAPTER;
typedef struct _D3DKMT_WAITFORVERTICALBLANKEVENT {
  D3DKMT_HANDLE hAdapter;
  D3DKMT_HANDLE hDevice;
  D3DDDI_VIDEO_PRESENT_SOURCE_ID VidPnSourceId;
} D3DKMT_WAITFORVERTICALBLANKEVENT;
typedef NTSTATUS(APIENTRY* PFND3DKMTOPENADAPTERFROMHDC)(D3DKMT_OPENADAPTERFROMHDC*);
typedef NTSTATUS(APIENTRY* PFND3DKMTCLOSEADAPTER)(D3DKMT_CLOSEADAPTER*);
typedef NTSTATUS(APIENTRY* PFND3DKMTWAITFORVERTICALBLANKEVENT)(D3DKMT_WAITFORVERTICALBLANKEVENT*);

class DxVsyncSource : public IVsyncSource
{
public:
    DxVsyncSource(IFFmpegRenderer* renderer);

    virtual ~DxVsyncSource();

    virtual bool initialize(SDL_Window* window, int) override;

    virtual bool isAsync() override;

    virtual void waitForVsync() override;

private:
    void updateFrameStats();

    D3D11VARenderer* m_Renderer;
    HMODULE m_Gdi32Handle;
    HWND m_Window;
    HMONITOR m_LastMonitor;
    D3DKMT_WAITFORVERTICALBLANKEVENT m_WaitForVblankEventParams;

    PFND3DKMTOPENADAPTERFROMHDC m_D3DKMTOpenAdapterFromHdc;
    PFND3DKMTCLOSEADAPTER m_D3DKMTCloseAdapter;
    PFND3DKMTWAITFORVERTICALBLANKEVENT m_D3DKMTWaitForVerticalBlankEvent;

    static constexpr int VSYNC_HISTORY_SIZE = 10;
    std::mutex m_FrameStatsLock;
    UINT m_LastSyncRefreshCount;
    int64_t m_LastSyncQpc = 0;
    int64_t m_VsyncIntervalQpc; // also tracked in framepacer
    std::array<int64_t, VSYNC_HISTORY_SIZE> m_vhistory{};
    int m_vhcount = 0;
    int m_vhidx = 0;
    int64_t m_vhsum = 0;
    double m_ewmaVsyncDriftQpc = 1.0;
};
