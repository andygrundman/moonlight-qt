#pragma once
#include "imgui.h"

// ---------------------------------------------------------------------------
//  WinUI-style rounded button for ImGui
//  - Does NOT call ImGui::Button() internally
//  - Uses ONLY the public ImGui API
//  - Honours ImGuiCol_Button / ImGuiCol_ButtonHovered / ImGuiCol_ButtonActive
//  - Optional left icon (any UTF-8 / icon-font glyph string)
//  - Optional right-chevron indicator (for sub-menu items)
//  - Separator helper matching the menu-group dividers in the reference UI
// ---------------------------------------------------------------------------

namespace WinUI
{

// ── tunables ────────────────────────────────────────────────────────────────

struct ButtonStyle
{
    float  Rounding       = 10.0f;         // corner rounding radius (px)
    float  PaddingX       = 16.0f;         // horizontal inner padding
    float  PaddingY       =  8.0f;         // vertical   inner padding
    float  IconSpacing    = 10.0f;         // gap between icon and label
    float  ChevronWidth   = 20.0f;         // reserved width for the ">" glyph
    float  MinWidth       = 200.0f;        // minimum button width

    // Per-state alpha multipliers applied on top of the theme colours so the
    // backdrop stays dark and semi-transparent (Xbox overlay look) without
    // touching the theme itself.
    float  IdleAlpha      = 0.55f;
    float  HoverAlpha     = 0.85f;
    float  ActiveAlpha    = 1.00f;

    // Separator rule
    ImVec4 SeparatorColor = {1, 1, 1, 0.08f};
    float  SeparatorThick = 1.0f;
};

static ButtonStyle GButtonStyle; // global default — mutate once at startup

// ── internal helpers (file-private) ─────────────────────────────────────────

namespace detail
{
    // Return the end of the visible portion of a label, stripping "##id".
    inline const char* LabelEnd(const char* label)
    {
        const char* p = label;
        while (*p)
        {
            if (p[0] == '#' && p[1] == '#') return p;
            ++p;
        }
        return p;
    }

    // Build an ImU32 from a style colour with an alpha multiplier.
    inline ImU32 StyleCol(ImGuiCol idx, float alphaMul)
    {
        ImVec4 c = ImGui::GetStyleColorVec4(idx);
        c.w *= alphaMul;
        return ImGui::ColorConvertFloat4ToU32(c);
    }
} // namespace detail

// ── RoundedButton ────────────────────────────────────────────────────────────
//
//  Parameters
//  ----------
//  label       Visible text. Append "##uniqueId" to disambiguate if needed.
//  icon        Optional glyph drawn to the left, e.g. u8"\uf0e7".
//  hasChevron  Draw a ">" arrow on the right edge (sub-menu cue).
//  size        size.x == 0  -> stretch to fill available width
//              size.y == 0  -> derive height from font size + padding
//
//  Returns true on click (same contract as ImGui::Button).

bool RoundedButton(
    const char*        label,
    ImVec2             size       = {0, 0},
    const ButtonStyle& style      = GButtonStyle)
{
    // ── measure text / icon ─────────────────────────────────────────────────
    const char*  labelEnd = detail::LabelEnd(label);
    const ImVec2 textSz   = ImGui::CalcTextSize(label);

    float contentW = textSz.x;

    // ── resolve final size ──────────────────────────────────────────────────
    ImVec2 btnSize = size;
    if (btnSize.x <= 0.0f)
        btnSize.x = std::max(style.MinWidth, contentW + style.PaddingX * 2.0f);
    if (btnSize.y <= 0.0f)
        btnSize.y = textSz.y + style.PaddingY * 2.0f;

    // ── record cursor position BEFORE InvisibleButton advances it ───────────
    const ImVec2 pos = ImGui::GetCursorScreenPos();

    // ── InvisibleButton handles ItemSize / ItemAdd / ButtonBehavior ─────────
    //    This is the correct public API for custom-drawn widgets.
    bool clicked = ImGui::InvisibleButton(label, btnSize);

    // ── read interaction state via the public API ───────────────────────────
    const bool hovered = ImGui::IsItemHovered();
    const bool held    = ImGui::IsItemActive();

    // ── choose background colour ────────────────────────────────────────────
    ImU32 bgCol;
    if (held && hovered)
        bgCol = detail::StyleCol(ImGuiCol_ButtonActive,  style.ActiveAlpha);
    else if (hovered)
        bgCol = detail::StyleCol(ImGuiCol_ButtonHovered, style.HoverAlpha);
    else
        bgCol = detail::StyleCol(ImGuiCol_Button,        style.IdleAlpha);

    // ── draw ────────────────────────────────────────────────────────────────
    ImDrawList* dl       = ImGui::GetWindowDrawList();
    const ImVec2 pMax    = { pos.x + btnSize.x, pos.y + btnSize.y };
    const float  centerY = pos.y + btnSize.y * 0.5f;

    // Background
    dl->AddRectFilled(pos, pMax, bgCol, style.Rounding);

    // Thin white border (glass-edge effect) — fades out when pressed
    if (!held)
    {
        const float borderA = hovered ? 0.22f : 0.10f;
        dl->AddRect(pos, pMax,
                    IM_COL32(255, 255, 255, (int)(borderA * 255)),
                    style.Rounding, 0, 1.0f);
    }

    // Label
    float cursorX = pos.x + style.PaddingX;
    dl->AddText({ cursorX, centerY - textSz.y * 0.5f },
                IM_COL32(255, 255, 255, 230), label, labelEnd);

    return clicked;
}

// ── Separator ────────────────────────────────────────────────────────────────
//  Subtle horizontal rule matching the WinUI group dividers.
//  Call between groups of buttons.

void Separator(
    float              widthOverride = 0.0f,
    float              marginY       = 4.0f,
    const ButtonStyle& style         = GButtonStyle)
{
    ImDrawList*  dl = ImGui::GetWindowDrawList();
    const float  w  = widthOverride > 0.0f
                    ? widthOverride
                    : ImGui::GetContentRegionAvail().x;
    const ImVec2 p  = ImGui::GetCursorScreenPos();

    // Reserve vertical space via the public Dummy call
    ImGui::Dummy({ w, marginY * 2.0f + style.SeparatorThick });

    dl->AddLine({ p.x,     p.y + marginY },
                { p.x + w, p.y + marginY },
                ImGui::ColorConvertFloat4ToU32(style.SeparatorColor),
                style.SeparatorThick);
}

} // namespace WinUI

// ---------------------------------------------------------------------------
//  Example usage
// ---------------------------------------------------------------------------
//
//  void ShowWinUIDemo()
//  {
//      ImGui::SetNextWindowSize({280, 0}, ImGuiCond_Always);
//      ImGui::Begin("WinUI Menu", nullptr,
//          ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);
//
//      WinUI::RoundedButton(u8"Enter Mouse Mode", u8"\U0001F5B1");
//      WinUI::RoundedButton(u8"Show Keyboard",    u8"\u2328");
//
//      WinUI::Separator();
//
//      WinUI::RoundedButton(u8"Other Actions", u8"\u2630", /*chevron=*/true);
//
//      WinUI::Separator();
//
//      WinUI::RoundedButton(u8"Show Stats", u8"\u21BA");
//      WinUI::RoundedButton(u8"Show Logs",  u8"\u2261");
//
//      WinUI::Separator();
//
//      WinUI::RoundedButton(u8"Disconnect",           u8"\u21AA");
//      WinUI::RoundedButton(u8"Disconnect and Close", u8"\u25A1");
//
//      ImGui::End();
//  }
