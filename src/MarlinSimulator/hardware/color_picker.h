#pragma once

//
// One color swatch + picker popup, shared by everything that lets the user choose a filament color
// (single extruders, mixing extruder components, MMU / Chameleon slots), and the "#rrggbb" helpers
// the agent interface uses for the same colors.
//

#include <cstdio>
#include <string>

#include <glm/glm.hpp>
#include <imgui.h>

namespace color_picker {

  // The same palette for every extruder and every filament slot (T0 red, T1 green, ...)
  inline glm::vec3 default_color(const size_t i) {
    static const glm::vec3 defaults[8] = {
      {1.0f, 0.0f, 0.0f}, {0.0f, 0.6f, 0.0f}, {0.0f, 0.2f, 1.0f}, {1.0f, 0.9f, 0.0f},
      {1.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 1.0f}, {1.0f, 0.5f, 0.0f}, {1.0f, 1.0f, 1.0f}
    };
    return defaults[i & 7];
  }

  inline std::string to_hex(const glm::vec3& c) {
    char buf[8];
    snprintf(buf, sizeof(buf), "#%02x%02x%02x", int(c.r * 255 + 0.5f), int(c.g * 255 + 0.5f), int(c.b * 255 + 0.5f));
    return buf;
  }

  inline bool parse_hex(const std::string& hex, glm::vec3& out) {
    unsigned rgb;
    if (hex.size() != 7 || hex[0] != '#' || sscanf(hex.c_str() + 1, "%x", &rgb) != 1) return false;
    out = { ((rgb >> 16) & 0xFF) / 255.0f, ((rgb >> 8) & 0xFF) / 255.0f, (rgb & 0xFF) / 255.0f };
    return true;
  }

  enum class Mark { None, Selected, Active };

  // A square swatch that opens a picker when clicked. The caption goes under it. Mark draws a border:
  // Active is thick and white (loaded / in use), Selected is thin and yellow (chosen, not yet loaded).
  // Returns true when the color changed this frame. Call inside a PushID() scope.
  inline bool swatch(const char* caption, glm::vec3& color, const Mark mark = Mark::None, const float size = 40.0f) {
    bool changed = false;
    ImGui::BeginGroup();
    const ImVec4 col(color.r, color.g, color.b, 1.0f);
    if (ImGui::ColorButton("##swatch", col, ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoBorder, ImVec2(size, size)))
      ImGui::OpenPopup("##picker");
    if (mark != Mark::None) {
      const ImVec2 lo = ImGui::GetItemRectMin(), hi = ImGui::GetItemRectMax();
      const bool active = mark == Mark::Active;
      ImGui::GetWindowDrawList()->AddRect(ImVec2(lo.x - 3, lo.y - 3), ImVec2(hi.x + 3, hi.y + 3),
        active ? IM_COL32(255, 255, 255, 255) : IM_COL32(255, 220, 0, 255), 2.0f, 0, active ? 3.0f : 1.5f);
    }
    ImGui::Text(" %s", caption);
    ImGui::EndGroup();
    if (ImGui::BeginPopup("##picker")) {
      ImGui::Text("%s color", caption);
      changed = ImGui::ColorPicker3("##color", &color.x, ImGuiColorEditFlags_PickerHueWheel | ImGuiColorEditFlags_NoSidePreview);
      ImGui::EndPopup();
    }
    return changed;
  }

} // namespace color_picker
