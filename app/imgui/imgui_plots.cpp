#include "imgui_plots.h"
#include "plotdesc.h"
#include "IconsFontAwesome7.h"
#include "fa-solid-900.h"
#include "devui.h"

#include "imgui.h"
#include "implot.h"

#include <cassert>
#include <numeric>
#include <vector>

Plot::Plot(const PlotDesc& d, std::size_t capacity):
    desc(d),
    buffer(capacity)
{}

// singleton
ImGuiPlots& ImGuiPlots::instance()
{
    static ImGuiPlots s;
    return s;
}

ImGuiPlots::ImGuiPlots():
    plots_ {{
        Plot(kPlotDescs[PLOT_FRAMETIME]),
        Plot(kPlotDescs[PLOT_HOST_FRAMETIME]),
        Plot(kPlotDescs[PLOT_DROPPED_NETWORK]),
        Plot(kPlotDescs[PLOT_DROPPED_PACER]),
        Plot(kPlotDescs[PLOT_QUEUED_FRAMES]),
        Plot(kPlotDescs[PLOT_BANDWIDTH]),
        Plot(kPlotDescs[PLOT_PRESENT_DELAY]),
    }},
    m_isEnabled(true)
{}

ImGuiPlots::~ImGuiPlots() = default;

void ImGuiPlots::clearData()
{
    for (auto& p : plots_) {
        p.buffer.clear();
    }
}

void ImGuiPlots::observeFloat(int plotId, float value)
{
    if (!m_isEnabled) {
        return;
    }
    assert(plotId >= 0 && plotId < PlotCount);
    plots_[static_cast<std::size_t>(plotId)].buffer.push(static_cast<float>(value));
}

float ImGuiPlots::observeFloatReturnAvg(int plotId, float value)
{
    // still runs even if !m_isEnabled, caller needs the return average
    assert(plotId >= 0 && plotId < PlotCount);
    plots_[static_cast<std::size_t>(plotId)].buffer.push(static_cast<float>(value));
    return plots_[static_cast<std::size_t>(plotId)].buffer.average();
}

#ifndef IMGUI_DISABLE

void ImGuiPlots::ImGui_init(IFFmpegRenderer* renderer)
{
    // Setup ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();

    // io.ConfigFlags |= ImGuiConfigFlags_NoMouse;
    // io.ConfigFlags |= ImGuiConfigFlags_NoKeyboard;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;   // Enable Gamepad Controls

    io.ConfigFlags |= ImGuiConfigFlags_NoMouseCursorChange;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigMacOSXBehaviors = false; // Don't swap ctrl and super
    ImGui::StyleColorsDark();

    ImFontConfig mainConfig;
    mainConfig.SizePixels = 13.0f;
    io.Fonts->AddFontDefaultVector(&mainConfig);
    ImFontConfig iconConfig;
    iconConfig.MergeMode = true;
    iconConfig.GlyphMinAdvanceX = 13.0f; // make the icon monospaced
    io.Fonts->AddFontFromMemoryCompressedTTF(FA_Solid_compressed_data, FA_Solid_compressed_size, 13.0f, &iconConfig);

    renderer->ImGui_initBackend();
}

void ImGuiPlots::ImGui_deinit(IFFmpegRenderer* renderer)
{
    renderer->ImGui_deinitBackend();

    ImPlot::DestroyContext();
    ImGui::DestroyContext();
}

#endif
