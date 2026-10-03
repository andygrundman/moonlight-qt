#pragma once

#include <SDL.h>

#include <atomic>
#include <mutex>
#include <vector>

// ImGui's context and input queue belong to the rendering thread. The SDL
// event thread only copies supported events and reads published capture flags.
class ImGuiInput
{
public:
    static ImGuiInput& instance();

    // Called by the context owner after backend initialization and before
    // backend destruction. Events from an inactive session are discarded.
    void beginSession();
    void endSession();

    void enqueue(const SDL_Event& event);
    void processPendingEvents();
    void updateCapture();

    bool wantsKeyboard() const { return m_WantKeyboard.load(); }
    bool wantsMouse() const { return m_WantMouse.load(); }

private:
    std::mutex m_Mutex;
    std::vector<SDL_Event> m_Events;
    std::vector<SDL_Event> m_FrameEvents; // Only touched by the context owner.
    bool m_Active = false;
    std::atomic<bool> m_WantKeyboard{false};
    std::atomic<bool> m_WantMouse{false};
};
