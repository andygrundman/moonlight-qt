#pragma once
#include "plotdesc.h"
#include "imgui_backend.h"
#include "streaming/floatbuffer.h"

#include <array>

struct Plot {
    const PlotDesc& desc;
    FloatBuffer buffer;

    explicit Plot(const PlotDesc& d, std::size_t capacity = 512);
};

class ImGuiPlots
{
  public:
    static ImGuiPlots& instance();

#ifndef IMGUI_DISABLE
    void ImGui_init(IImGuiBackend* renderer);
    void ImGui_deinit(IImGuiBackend* renderer);
#endif

    bool isEnabled()
    {
        return m_isEnabled;
    }

    void setEnabled(bool enabled)
    {
        m_isEnabled = enabled;
    }

    void clearData();
    void observeFloat(int plotId, float value);
    float observeFloatReturnAvg(int plotId, float value);

    Plot& get(int plotId)
    {
        return plots_[static_cast<std::size_t>(plotId)];
    }

    const Plot& get(int plotId) const
    {
        return plots_[static_cast<std::size_t>(plotId)];
    }

    std::array<Plot, PlotCount>& plots()
    {
        return plots_;
    }

    const std::array<Plot, PlotCount>& plots() const
    {
        return plots_;
    }

  private:
    ImGuiPlots();
    ~ImGuiPlots();

    std::array<Plot, PlotCount> plots_;
    bool m_isEnabled;
};
