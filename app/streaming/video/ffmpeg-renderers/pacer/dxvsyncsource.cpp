#include "dxvsyncsource.h"
#include "streaming/qpc.h"
#include "streaming/video/ffmpeg-renderers/framepacing/framequeue.h"

// Useful references:
// https://bugs.chromium.org/p/chromium/issues/detail?id=467617
// https://chromium.googlesource.com/chromium/src.git/+/c564f2fe339b2b2abb0c8773c90c83215670ea71/gpu/ipc/service/gpu_vsync_provider_win.cc

DxVsyncSource::DxVsyncSource(IFFmpegRenderer* renderer) :
    m_Renderer(static_cast<D3D11VARenderer*>(renderer)),
    m_Gdi32Handle(nullptr),
    m_LastMonitor(nullptr)
{
    SDL_zero(m_WaitForVblankEventParams);
}

DxVsyncSource::~DxVsyncSource()
{
    if (m_WaitForVblankEventParams.hAdapter != 0) {
        D3DKMT_CLOSEADAPTER closeAdapterParams = {};
        closeAdapterParams.hAdapter = m_WaitForVblankEventParams.hAdapter;
        m_D3DKMTCloseAdapter(&closeAdapterParams);
    }

    if (m_Gdi32Handle != nullptr) {
        FreeLibrary(m_Gdi32Handle);
    }
}

bool DxVsyncSource::initialize(SDL_Window* window, int)
{
    m_Gdi32Handle = LoadLibraryA("gdi32.dll");
    if (m_Gdi32Handle == nullptr) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                     "Failed to load gdi32.dll: %d",
                     GetLastError());
        return false;
    }

    m_D3DKMTOpenAdapterFromHdc = (PFND3DKMTOPENADAPTERFROMHDC)GetProcAddress(m_Gdi32Handle, "D3DKMTOpenAdapterFromHdc");
    m_D3DKMTCloseAdapter = (PFND3DKMTCLOSEADAPTER)GetProcAddress(m_Gdi32Handle, "D3DKMTCloseAdapter");
    m_D3DKMTWaitForVerticalBlankEvent = (PFND3DKMTWAITFORVERTICALBLANKEVENT)GetProcAddress(m_Gdi32Handle, "D3DKMTWaitForVerticalBlankEvent");

    if (m_D3DKMTOpenAdapterFromHdc == nullptr ||
            m_D3DKMTCloseAdapter == nullptr ||
            m_D3DKMTWaitForVerticalBlankEvent == nullptr) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                     "Missing required function in gdi32.dll");
        return false;
    }

    SDL_SysWMinfo info;

    SDL_VERSION(&info.version);

    if (!SDL_GetWindowWMInfo(window, &info)) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                     "SDL_GetWindowWMInfo() failed: %s",
                     SDL_GetError());
        return false;
    }

    // Pacer should only create us on Win32
    SDL_assert(info.subsystem == SDL_SYSWM_WINDOWS);

    m_Window = info.info.win.window;

    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "vsync stats thread started, qpcFreq=%lld ticksPerMs=%lld",
	            QpcFreq(), MsToQpc(1.0));

    return true;
}

bool DxVsyncSource::isAsync()
{
    // We wait in the context of the Pacer thread
    return false;
}

void DxVsyncSource::waitForVsync()
{
    NTSTATUS status;

    // // If the monitor has changed from last time, open the new adapter
    // HMONITOR currentMonitor = MonitorFromWindow(m_Window, MONITOR_DEFAULTTONEAREST);
    // if (currentMonitor != m_LastMonitor) {
    //     MONITORINFOEXA monitorInfo = {};
    //     monitorInfo.cbSize = sizeof(monitorInfo);
    //     if (!GetMonitorInfoA(currentMonitor, &monitorInfo)) {
    //         SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
    //                      "GetMonitorInfo() failed: %d",
    //                      GetLastError());
    //         return;
    //     }

    //     DEVMODEA monitorMode;
    //     monitorMode.dmSize = sizeof(monitorMode);
    //     if (!EnumDisplaySettingsA(monitorInfo.szDevice, ENUM_CURRENT_SETTINGS, &monitorMode)) {
    //         SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
    //                      "EnumDisplaySettings() failed: %d",
    //                      GetLastError());
    //         return;
    //     }

    //     SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
    //                 "Monitor changed: %s %d Hz",
    //                 monitorInfo.szDevice,
    //                 monitorMode.dmDisplayFrequency);

    //     // Close the old adapter
    //     if (m_WaitForVblankEventParams.hAdapter != 0) {
    //         D3DKMT_CLOSEADAPTER closeAdapterParams = {};
    //         closeAdapterParams.hAdapter = m_WaitForVblankEventParams.hAdapter;
    //         m_D3DKMTCloseAdapter(&closeAdapterParams);
    //     }

    //     D3DKMT_OPENADAPTERFROMHDC openAdapterParams = {};
    //     openAdapterParams.hDc = CreateDCA(nullptr, monitorInfo.szDevice, nullptr, nullptr);
    //     if (!openAdapterParams.hDc) {
    //         SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
    //                      "CreateDC() failed: %d",
    //                      GetLastError());
    //         return;
    //     }

    //     // Open the new adapter
    //     status = m_D3DKMTOpenAdapterFromHdc(&openAdapterParams);
    //     DeleteDC(openAdapterParams.hDc);

    //     if (status != STATUS_SUCCESS) {
    //         SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
    //                      "D3DKMTOpenAdapterFromHdc() failed: %x",
    //                      status);
    //         return;
    //     }

    //     m_WaitForVblankEventParams.hAdapter = openAdapterParams.hAdapter;
    //     m_WaitForVblankEventParams.hDevice = 0;
    //     m_WaitForVblankEventParams.VidPnSourceId = openAdapterParams.VidPnSourceId;

    //     m_LastMonitor = currentMonitor;
    // }

    // status = m_D3DKMTWaitForVerticalBlankEvent(&m_WaitForVblankEventParams);
    // if (status != STATUS_SUCCESS) {
    //     SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
    //                  "D3DKMTWaitForVerticalBlankEvent() failed: %x",
    //                  status);
    //     return;
    // }

    int64_t t0 = QpcNow();

    HRESULT hr = m_Renderer->GetDXGIOutput()->WaitForVBlank();
    if (FAILED(hr)) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "WaitForVblank() failed: %x", hr);
    }

    if (m_LastSyncQpc && m_VsyncIntervalQpc) {
        // now is close to the vsync timestamp, but we can use historical timestamps
        // to estimate the true value as best we can. "now" should be within 1 interval of nowAligned.
        int64_t now = QpcNow();
        int64_t nowAligned = m_LastSyncQpc + static_cast<int64_t>(m_ewmaVsyncDriftQpc);
        while (nowAligned < now - m_VsyncIntervalQpc) {
            nowAligned += m_VsyncIntervalQpc;
        }

        // Report using the macOS-inspired convention of a pair of double seconds (timestamp of current vsync interval, deadline)
        double timestamp = QpcToMs(nowAligned) / 1000.0;
        double deadline = QpcToMs(nowAligned + m_VsyncIntervalQpc) / 1000.0;
        FramePacer::instance().signalVsyncTS(timestamp, deadline);

        FQLog("WaitForVBlank took %.3f nowAligned %.3f / timestamp %.6f deadline %.6f",
            QpcToMs(now - t0), QpcToMs(now - nowAligned), timestamp, deadline);
    }

    // stats are always a few frames behind, so update them last
    updateFrameStats();
}

void DxVsyncSource::updateFrameStats()
{
    // After we've presented a couple of frames, we can obtain the true vsync interval
    DXGI_FRAME_STATISTICS stats;
    HRESULT hr;
    if (m_Renderer != nullptr) {
        hr = m_Renderer->GetSwapChain()->GetFrameStatistics(&stats);
        if (FAILED(hr)) {
            SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "GetFrameStatistics() failed: %x", hr);
            return;
        }

        if (stats.SyncRefreshCount == 0 && stats.SyncQPCTime.QuadPart == 0ULL) {
            return;
        }

        UINT srcPassed = 0;
        if (stats.SyncRefreshCount && m_LastSyncRefreshCount) {
            srcPassed = stats.SyncRefreshCount - m_LastSyncRefreshCount;
        }
        m_LastSyncRefreshCount = stats.SyncRefreshCount;

        int64_t sqtPassed = 0;
        if (stats.SyncQPCTime.QuadPart && m_LastSyncQpc) {
            sqtPassed = stats.SyncQPCTime.QuadPart - m_LastSyncQpc;
        }
        m_LastSyncQpc = stats.SyncQPCTime.QuadPart;

        // compare with the last sync target we used in waitBeforePresent
        int64_t driftQpc = 0;
        int64_t lastTargetQpc = FramePacer::instance().GetLastSyncTargetQpc();
        if (lastTargetQpc) {
            driftQpc = m_LastSyncQpc - lastTargetQpc;
            if (std::llabs(driftQpc) < (int64_t)MsToQpc(0.03)) {
                const double alpha = 0.05;  // slow moving average
                m_ewmaVsyncDriftQpc = (1.0 - alpha) * m_ewmaVsyncDriftQpc + alpha * static_cast<double>(driftQpc);
            }
        }

        FQLog("GetFrameStatistics: PresentCount %lu PresentRefreshCount %lu SyncRefreshCount %lu SyncQPCTime %lld\n"
              "  srcPassed %u sqtPassed %lld lastTargetQpc %lld driftQpc %lld",
            stats.PresentCount, stats.PresentRefreshCount, stats.SyncRefreshCount, stats.SyncQPCTime.QuadPart,
            srcPassed, sqtPassed, lastTargetQpc, driftQpc);

        // If any vsyncs have passed, we can calculate a very accurate interval
        if (srcPassed && sqtPassed) {
            const int64_t intervalQpc = sqtPassed / srcPassed;

            // sanity check that interval is between 1ms and 1000ms
            static const int64_t ONE_MS_QPC = MsToQpc(1.0);
            static const int64_t ONE_SEC_QPC = MsToQpc(1000.0);
            if (intervalQpc > ONE_MS_QPC && intervalQpc < ONE_SEC_QPC) {
                return;
            }

            // use average from past 10 intervals
            if (m_vhcount == VSYNC_HISTORY_SIZE) {
                m_vhsum -= m_vhistory[m_vhidx];
            }
            else {
                ++m_vhcount;
            }
            m_vhsum += intervalQpc;
            m_vhistory[m_vhidx] = intervalQpc;
            m_vhidx = (m_vhidx + 1) % VSYNC_HISTORY_SIZE;

            // Compute average
            m_VsyncIntervalQpc = m_vhsum / m_vhcount;

            FQLog(
                "updateFrameStats(): LastSyncQpc %lld, Estimated vsync interval: %.3fms (%.2f Hz) (%lld ticks), "
                "driftQpc %lld (%fms), driftAvg %lld\n",
                m_LastSyncQpc,
                QpcToMs(m_VsyncIntervalQpc),
                1000.0 / QpcToMs(m_VsyncIntervalQpc),
                m_VsyncIntervalQpc,
                driftQpc,
                QpcToMs(driftQpc),
                static_cast<int64_t>(m_ewmaVsyncDriftQpc)
            );
        }
    }
}
