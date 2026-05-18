#include "gamepadmenu.h"

#include "imgui.h"
#include "imgui/devui.h"
#include "imgui/IconsFontAwesome7.h"
#include "streaming/session.h"

GamepadMenu& GamepadMenu::instance()
{
    static GamepadMenu inst;
    return inst;
}

GamepadMenu::GamepadMenu() {}

bool GamepadMenu::IsVisible()
{
    return m_Visible.load();
}

void GamepadMenu::SetActiveJoystickID(SDL_JoystickID jsid)
{
    m_ActiveJoystickID.store(jsid);
}

struct MenuItem {
    const char* label;
    std::function<void()> callback;
};

void GamepadMenu::Render()
{
    ImGuiViewport* vp = ImGui::GetMainViewport();
    if (!vp) {
        return;
    }

    // Setup 2 global ImGui shortcuts (key chord handlers) for bringing up the quick menu:
    // Select + Start on gamepad, or Ctrl-Shift-Alt-Space
    bool panelOpen = m_Visible.load();
    if (ImGui::Shortcut(ImGuiKey_Space | ImGuiMod_Ctrl | ImGuiMod_Alt | ImGuiMod_Shift, ImGuiInputFlags_RouteGlobal)) {
        panelOpen = !panelOpen;

        // Clear keys down so we aren't holding keys while in the menu
        Session::get()->getInputHandler()->raiseAllKeys();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_GamepadStart) && ImGui::IsKeyPressed(ImGuiKey_GamepadBack)) {
        panelOpen = !panelOpen;

        // Clear buttons down on all gamepads so we aren't holding the buttons while in the menu
        Session::get()->getInputHandler()->raiseAllButtons();
    }
    m_Visible.store(panelOpen);

    if (panelOpen) {
        // When this panel is open, block all keyboard and gamepad events
        // ImGui doesn't have a "want capture gamepad", so we disable gamepad events in SDL instead
        ImGui::SetNextFrameWantCaptureKeyboard(true);
        SDL_GameControllerEventState(SDL_IGNORE);

        // Always center this window when appearing
        ImGui::SetNextWindowPos(vp->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

        ImGui::SetNextWindowSize(ImVec2(std::max(vp->WorkSize.x / 3.5f, 320.0f), 0.0f));
        //ImGui::SetNextWindowBgAlpha(0.92f);

        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(16.0f, 12.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,   ImVec2(0.0f, 2.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,  ImVec2(10.0f, 8.0f));

        ImGui::PushStyleColor(ImGuiCol_Border, IM_COL32(255, 255, 255, 40));

        ImGuiStyle& style = ImGui::GetStyle();
        ImGui::PushFont(NULL, style.FontSizeBase * 1.5f);

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar;

        if (ImGui::Begin("##GamepadMenu", nullptr, flags)) {
            ImGui::TextDisabled("  " ICON_FA_GAMEPAD "  Quick Menu");
            ImGui::Separator();
            ImGui::Spacing();

            // last used SDL gamepad
            SDL_JoystickID jsid = m_ActiveJoystickID.load();

            MenuItem items[] = {
                {ICON_FA_COMPUTER_MOUSE " Enter Mouse Mode",
                 [jsid]() {
                    if (jsid >= 0) {
                        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "toggleMouseEmulation for SDL_JoystickID %d", jsid);
                        Session::get()->toggleMouseEmulation(jsid);
                    }
                 }},
                {ICON_FA_CHART_BAR " Toggle stats",
                 []() {
                     Session::get()->getOverlayManager().setOverlayState(
                         Overlay::OverlayDebug,
                         !Session::get()->getOverlayManager().isOverlayEnabled(Overlay::OverlayDebug)
                     );
                 }},
                {ICON_FA_CHART_LINE " Toggle graphs",
                 []() {
                     Stats::instance().SetShowGraphs(!Stats::instance().GetShowGraphs());
                 }},
                {ICON_FA_SLIDERS " Toggle advanced controls",
                 []() {
                    DevUISettings::instance().Toggle();
                    // XXX some way to let gamepad control move over to DevUI window
                 }},
                {ICON_FA_MINUS " Hide all overlays",
                 []() {
                    Session::get()->getOverlayManager().setOverlayState(Overlay::OverlayDebug, false);
                    Stats::instance().SetShowGraphs(false);
                    DevUISettings::instance().SetPanelOpen(false);
                #ifdef __APPLE__
                    DevUISettings::instance().SetConfig([=](DevUIConfig& config) {
                        config.showMetalHud = false;
                    });
                #endif
                 }},
                {ICON_FA_PAUSE " Disconnect",
                 []() {
                     SDL_Event event;
                     event.type = SDL_QUIT;
                     event.quit.timestamp = SDL_GetTicks();
                     SDL_PushEvent(&event);
                 }},
                {ICON_FA_STOP " Disconnect and close", []() {
                     Session::get()->setShouldExit(true);
                     SDL_Event event;
                     event.type = SDL_QUIT;
                     event.quit.timestamp = SDL_GetTicks();
                     SDL_PushEvent(&event);
                 }}
            };

            const int itemCount = IM_COUNTOF(items);

            // Reset selection and mouse position when the menu first opens
            if (ImGui::IsWindowAppearing()) {
                m_SelectedIndex = 0;
                m_MousePosOnOpen = ImGui::GetMousePos();
            }

            // Keyboard and gamepad navigation
            bool navUp = ImGui::IsKeyPressed(ImGuiKey_UpArrow) || ImGui::IsKeyPressed(ImGuiKey_GamepadDpadUp) ||
                         ImGui::IsKeyPressed(ImGuiKey_GamepadLStickUp);
            bool navDown = ImGui::IsKeyPressed(ImGuiKey_DownArrow) || ImGui::IsKeyPressed(ImGuiKey_GamepadDpadDown) ||
                           ImGui::IsKeyPressed(ImGuiKey_GamepadLStickDown);

            if (navUp) {
                m_SelectedIndex = (m_SelectedIndex - 1 + itemCount) % itemCount;
            }
            if (navDown) {
                m_SelectedIndex = (m_SelectedIndex + 1) % itemCount;
            }

            // Activate selected item with Enter or gamepad A (FaceDown)
            bool navActivate = ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_GamepadFaceDown);
            if (navActivate) {
                items[m_SelectedIndex].callback();
                CloseMenu();
            }

            // Render items, highlighting the currently selected one
            for (int i = 0; i < itemCount; i++) {
                const auto& item = items[i];
                ImGui::PushID(i);

                const bool selected = (i == m_SelectedIndex);
                if (selected) {
                    ImGui::PushStyleColor(ImGuiCol_Header, IM_COL32(60, 100, 180, 180));
                    ImGui::PushStyleColor(ImGuiCol_HeaderHovered, IM_COL32(60, 100, 180, 200));
                }

                ImGui::CollapsingHeader(item.label,
                                        ImGuiTreeNodeFlags_Leaf |
                                        ImGuiTreeNodeFlags_NoTreePushOnOpen |
                                        ImGuiTreeNodeFlags_SpanAvailWidth);
                if (selected) {
                    ImGui::PopStyleColor(2);
                }

                // Mouse click also works; update selection so state stays consistent
                if (ImGui::IsItemClicked()) {
                    m_SelectedIndex = i;
                    item.callback();
                    CloseMenu();
                }

                // Only sync mouse hover to selection if the mouse has moved since the menu opened
                if (ImGui::IsItemHovered() && ImGui::GetMousePos().x != m_MousePosOnOpen.x &&
                    ImGui::GetMousePos().y != m_MousePosOnOpen.y) {
                    m_SelectedIndex = i;
                }

                ImGui::PopID();
            }

            // Close menu with Escape or gamepad B (FaceRight)
            if (ImGui::IsKeyPressed(ImGuiKey_Escape) || ImGui::IsKeyPressed(ImGuiKey_GamepadFaceRight)) {
                CloseMenu();
            }
        }

        ImGui::End();
        ImGui::PopFont();
        ImGui::PopStyleColor(1);
        ImGui::PopStyleVar(3);
    }
}

void GamepadMenu::CloseMenu()
{
    m_Visible.store(false);

    // Re-enable SDL gamepad when menu is closed
    SDL_GameControllerEventState(SDL_ENABLE);

    Session::get()->getInputHandler()->raiseAllButtons();

    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "CloseMenu()");
}
