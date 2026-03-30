#pragma once

#include "imgui.h"
#include "settings/streamingpreferences.h"

#include "SDL_compat.h"

class GamepadMenu
{
  public:
    // Singleton
    static GamepadMenu& instance();

    void SetVisible(bool visible, SDL_JoystickID jsid);
    bool IsVisible();
    void Render();

  private:
    GamepadMenu();
    GamepadMenu(const GamepadMenu&) = delete;
    GamepadMenu& operator=(const GamepadMenu&) = delete;

    std::atomic<bool> m_Visible {false};
    std::atomic<SDL_JoystickID> m_ActiveJoystickID {-1};
};
