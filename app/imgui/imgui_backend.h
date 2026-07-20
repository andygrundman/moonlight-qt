#pragma once

// Rendering-backend hooks for the ImGui dev UI. ImGuiPlots::ImGui_init() creates the ImGui
// context and then calls ImGui_initBackend() on whichever renderer drives the screen: an
// IFFmpegRenderer (Metal/D3D11) or the PyroWave decoder's libplacebo Vulkan pipeline.
class IImGuiBackend {
public:
    virtual ~IImGuiBackend() {}

    virtual void ImGui_initBackend() {
    }

    virtual void ImGui_deinitBackend() {
    }
};
