// Defines available plots.

#pragma once
#include <array>
#include <limits>

enum PlotType {
    PLOT_FRAMETIME = 0,
    PLOT_HOST_FRAMETIME,
    PLOT_DROPPED_NETWORK,
    PLOT_DROPPED_PACER,
    PLOT_QUEUED_FRAMES,
    PLOT_BANDWIDTH,
    PLOT_PRESENT_DELAY,
    PlotCount
};

enum PlotLabelType {
    PLOT_LABEL_MIN_MAX_AVG = 0,
    PLOT_LABEL_MIN_MAX_AVG_INT,
    PLOT_LABEL_TOTAL_INT
};

struct PlotDesc {
    const char* title;
    PlotLabelType labelType;
    const char* unit;
    float scaleMin;
    float scaleMax;
    float scaleTarget;
    float clampMax;
};

constexpr float kNoValue = std::numeric_limits<float>::quiet_NaN();

// clang-format off
inline constexpr std::array<PlotDesc, PlotCount> kPlotDescs = {{
    {"Frametime",                 PLOT_LABEL_MIN_MAX_AVG,   "ms", -0.1f, 65.0f,  kNoValue, 64.0f},
    {"Host Frametime",            PLOT_LABEL_MIN_MAX_AVG,   "ms", -0.1f, 65.0f,  kNoValue, 64.0f},
    {"Dropped frames (network)",  PLOT_LABEL_TOTAL_INT,       "", -1.0f, 3.0f,   kNoValue, kNoValue},
    {"Dropped frames (pacing)",   PLOT_LABEL_TOTAL_INT,       "", -1.0f, 3.0f,   kNoValue, kNoValue},
	{"Frames queued",             PLOT_LABEL_MIN_MAX_AVG_INT, "",  0.0f, 4.0f,   kNoValue, kNoValue},
    {"Video stream",              PLOT_LABEL_MIN_MAX_AVG, "Mbps", -0.1f, 200.0f, kNoValue, kNoValue},
    {"Present delay",             PLOT_LABEL_MIN_MAX_AVG,   "ms", -0.1f, 65.0f,  kNoValue, 64.0f},
}};

static_assert(kPlotDescs.size() == PlotCount, "Plot descriptors out of sync with PlotType enum");
