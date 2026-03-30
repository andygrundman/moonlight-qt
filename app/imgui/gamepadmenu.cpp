#include "gamepadmenu.h"

#include "imgui.h"
#include "imgui/IconsFontAwesome7.h"
#include "imgui/devui.h"
#include "imgui/style.h"
#include "streaming/session.h"

GamepadMenu& GamepadMenu::instance()
{
    static GamepadMenu inst;
    return inst;
}

GamepadMenu::GamepadMenu() {}

void GamepadMenu::SetVisible(bool visible, SDL_JoystickID jsid)
{
    ImGuiIO& io = ImGui::GetIO();
    if (visible) {
        // Enable ImGui Gamepad handling
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

        // cache the active controller ID, which will be needed if
        // mouse mode is enabled.
        m_ActiveJoystickID.store(jsid);
    }
    else {
        io.ConfigFlags &= ~ImGuiConfigFlags_NavEnableGamepad;
        io.ConfigFlags &= ~ImGuiConfigFlags_NavEnableKeyboard;
        m_ActiveJoystickID.store(-1);
    }

    m_Visible.store(visible);
}

bool GamepadMenu::IsVisible()
{
    return m_Visible.load();
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

    bool panelOpen = m_Visible.load();
    if (panelOpen) {
        //ImGui::SetNextWindowPos(ImVec2(400, 400), ImGuiCond_Always, ImVec2(1, 1));
        ImGui::SetNextWindowPos(ImVec2(vp->WorkSize.x - 40.0f, 0.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
        ImGui::SetNextWindowBgAlpha(0.70f);

        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 6.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 4.0f));

        ImGui::PushStyleColor(ImGuiCol_WindowBg, IM_COL32(25, 30, 28, 65));
        ImGui::PushStyleColor(ImGuiCol_Border, IM_COL32(255, 255, 255, 35));

        ImGuiStyle& style = ImGui::GetStyle();
        ImGui::PushFont(NULL, style.FontSizeBase * 2.0f);

        ImGuiWindowFlags flags = 0;
            // ImGuiWindowFlags_AlwaysAutoResize |
            // ImGuiWindowFlags_NoTitleBar |
            // ImGuiWindowFlags_NoSavedSettings;
            // ImGuiWindowFlags_NoResize |
            // ImGuiWindowFlags_NoMove |

        if (ImGui::Begin("##GamepadMenu", nullptr, flags)) {
            auto jsid = m_ActiveJoystickID.load();

            MenuItem items[] = {
                {ICON_FA_COMPUTER_MOUSE " Enter Mouse Mode", [jsid](){
                    Session::get()->toggleMouseEmulation(jsid);
                }},

                {nullptr, [](){ ImGui::Separator(); }},

                {ICON_FA_CHART_BAR " Toggle Stats", [](){
                    Session::get()->getOverlayManager().setOverlayState(Overlay::OverlayDebug,
                        !Session::get()->getOverlayManager().isOverlayEnabled(Overlay::OverlayDebug));
                }},

                {ICON_FA_CHART_LINE " Toggle Graphs", [](){
                    Stats::instance().SetShowGraphs(
                        !Stats::instance().GetShowGraphs());
                }},

                {ICON_FA_SLIDERS " Toggle advanced controls", [](){
                    DevUISettings::instance().Toggle();
                }},

                {nullptr, [](){ ImGui::Separator(); }},

                {ICON_FA_PAUSE " Disconnect", [](){
                    // Push a quit event to the main loop
                    SDL_Event event;
                    event.type = SDL_QUIT;
                    event.quit.timestamp = SDL_GetTicks();
                    SDL_PushEvent(&event);
                }},

                {ICON_FA_STOP " Disconnect and Close", [](){
                    // Indicate that we want to exit afterwards
                    Session::get()->setShouldExit(true);

                    // Push a quit event to the main loop
                    SDL_Event event;
                    event.type = SDL_QUIT;
                    event.quit.timestamp = SDL_GetTicks();
                    SDL_PushEvent(&event);
                }}
            };

            for (int i = 0; i < IM_COUNTOF(items); i++) {
                auto item = items[i];

                if (item.label != nullptr) {
                    ImGui::PushID(i);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    if (WinUI::RoundedButton(item.label)) {
                        item.callback();
                    }
                    ImGui::PopID();
                }
                else {
                    // callback-only for Separator
                    item.callback();
                }
            }
        }

        ImGui::End();
        ImGui::PopFont();
        ImGui::PopStyleColor(2);
        ImGui::PopStyleVar(2);
    }
}
