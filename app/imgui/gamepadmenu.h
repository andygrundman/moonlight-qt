#pragma once

#include "imgui.h"
#include "SDL_compat.h"
#include "settings/streamingpreferences.h"

class GamepadMenu
{
  public:
    // Singleton
    static GamepadMenu& instance();

    bool IsVisible();
    void SetActiveJoystickID(SDL_JoystickID jsid);
    void Render();
    void CloseMenu();

  private:
    GamepadMenu();
    GamepadMenu(const GamepadMenu&) = delete;
    GamepadMenu& operator=(const GamepadMenu&) = delete;

    std::atomic<bool> m_Visible {false};
    int m_SelectedIndex = 0;
    ImVec2 m_MousePosOnOpen = {0, 0};
    std::atomic<SDL_JoystickID> m_ActiveJoystickID {-1};
};
