#include "imgui/imgui_input.h"
#include "imgui.h"
#include "imgui_impl_sdl2.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <thread>

static void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

static SDL_Event key(SDL_Window* window, bool down)
{
    SDL_Event event = {};
    event.type = down ? SDL_KEYDOWN : SDL_KEYUP;
    event.key.windowID = SDL_GetWindowID(window);
    event.key.state = down ? SDL_PRESSED : SDL_RELEASED;
    event.key.keysym.sym = SDLK_q;
    event.key.keysym.scancode = SDL_SCANCODE_Q;
    event.key.keysym.mod = down ? KMOD_CTRL | KMOD_ALT | KMOD_SHIFT : KMOD_NONE;
    return event;
}

static void createContext(SDL_Window* window)
{
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;
    ImGui::GetIO().ConfigInputTrickleEventQueue = false;
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NoMouseCursorChange;
    ImGui::GetIO().ConfigMacOSXBehaviors = false; // Match the streaming overlay.
    unsigned char* pixels;
    int width, height;
    ImGui::GetIO().Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
    require(ImGui_ImplSDL2_InitForOther(window), "SDL input backend initialization");
    ImGuiInput::instance().beginSession();
}

static void frame()
{
    ImGuiInput::instance().processPendingEvents();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();
    ImGui::EndFrame();
    ImGui::Render();
    ImGuiInput::instance().updateCapture();
}

static void destroyContext()
{
    ImGuiInput::instance().endSession();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
}

int main()
{
    // The dummy driver has no native window, device input or desktop effects.
    SDL_setenv("SDL_VIDEODRIVER", "dummy", 1);
    SDL_SetMainReady();
    try {
        require(SDL_InitSubSystem(SDL_INIT_VIDEO) == 0, SDL_GetError());
        SDL_Window* window = SDL_CreateWindow("Input regression", 0, 0, 64, 64, SDL_WINDOW_HIDDEN);
        require(window != nullptr, SDL_GetError());
        auto& input = ImGuiInput::instance();
        input.enqueue(key(window, true)); // No context exists yet.
        createContext(window);
        frame();
        require(!ImGui::IsKeyDown(ImGuiKey_Q), "inactive events leaked into a new context");
        input.enqueue(key(window, true));
        frame();
        require(ImGui::IsKeyDown(ImGuiKey_Q), "queued keyboard press was lost");
        require(ImGui::GetIO().KeyCtrl && ImGui::GetIO().KeyAlt && ImGui::GetIO().KeyShift,
                "quit shortcut modifiers were lost");
        input.enqueue(key(window, false));
        frame();
        require(!ImGui::IsKeyDown(ImGuiKey_Q), "queued keyboard release was lost");
        ImGui::SetNextFrameWantCaptureKeyboard(true);
        ImGui::SetNextFrameWantCaptureMouse(true);
        frame();
        require(input.wantsKeyboard() && input.wantsMouse(), "overlay capture was not published");
        destroyContext();
        require(!input.wantsKeyboard() && !input.wantsMouse(), "capture survived context shutdown");

        unsigned int consumedFrames = 0;
        for (int cycle = 0; cycle < 20; ++cycle) {
            createContext(window);
            std::atomic<bool> done{false};
            std::thread producer([&] {
                for (int n = 0; n < 20000; ++n) {
                    input.enqueue(key(window, (n & 1) == 0));
                    if ((n & 63) == 63) std::this_thread::sleep_for(std::chrono::microseconds(100));
                }
                done.store(true);
            });
            do {
                frame();
                ++consumedFrames;
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            } while (!done.load());
            producer.join();
            frame();
            require(!ImGui::IsKeyDown(ImGuiKey_Q), "concurrent event delivery lost the final release");

            // Events arriving while the context is destroyed must never touch it.
            done.store(false);
            std::thread duringShutdown([&] {
                for (int n = 0; n < 2000; ++n) input.enqueue(key(window, true));
                done.store(true);
            });
            destroyContext();
            duringShutdown.join();
            require(!input.wantsKeyboard() && !input.wantsMouse(), "shutdown left capture active");
            createContext(window);
            frame();
            require(!ImGui::IsKeyDown(ImGuiKey_Q), "shutdown events leaked into the next stream");
            destroyContext();
        }
        SDL_DestroyWindow(window);
        SDL_QuitSubSystem(SDL_INIT_VIDEO);
        std::printf("PASS: key/modifier delivery, capture reset, 20 context restarts, 440000 concurrent events, %u frames\n",
                    consumedFrames);
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
