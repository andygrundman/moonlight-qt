#include "imgui_input.h"

#ifndef IMGUI_DISABLE
#include "imgui.h"
#include "imgui_impl_sdl2.h"

ImGuiInput& ImGuiInput::instance()
{
    static ImGuiInput input;
    return input;
}

void ImGuiInput::beginSession()
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    m_Events.clear();
    m_FrameEvents.clear();
    m_WantKeyboard.store(false);
    m_WantMouse.store(false);
    m_Active = true;
}

void ImGuiInput::endSession()
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    m_Active = false;
    m_Events.clear();
    m_FrameEvents.clear();
    m_WantKeyboard.store(false);
    m_WantMouse.store(false);
}

void ImGuiInput::enqueue(const SDL_Event& event)
{
    // These are the events consumed by the SDL backend. Their payloads are
    // stored inline, so none can retain SDL-owned strings after event handling.
    switch (event.type) {
    case SDL_MOUSEMOTION:
    case SDL_MOUSEWHEEL:
    case SDL_MOUSEBUTTONDOWN:
    case SDL_MOUSEBUTTONUP:
    case SDL_TEXTINPUT:
    case SDL_KEYDOWN:
    case SDL_KEYUP:
    case SDL_DISPLAYEVENT:
    case SDL_WINDOWEVENT:
    case SDL_CONTROLLERDEVICEADDED:
    case SDL_CONTROLLERDEVICEREMOVED:
        break;
    default:
        return;
    }

    std::lock_guard<std::mutex> lock(m_Mutex);
    if (m_Active) {
        m_Events.push_back(event);
    }
}

void ImGuiInput::processPendingEvents()
{
    m_FrameEvents.clear();
    {
        std::lock_guard<std::mutex> lock(m_Mutex);
        if (!m_Active) {
            return;
        }
        m_FrameEvents.swap(m_Events);
    }
    for (const auto& event : m_FrameEvents) {
        ImGui_ImplSDL2_ProcessEvent(&event);
    }
}

void ImGuiInput::updateCapture()
{
    const ImGuiIO& io = ImGui::GetIO();
    m_WantKeyboard.store(io.WantCaptureKeyboard);
    m_WantMouse.store(io.WantCaptureMouse);
}
#endif
