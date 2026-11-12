#include "ExtruderColors.h"

#include <algorithm>
#include <string>

#include <imgui.h>

#include "../agent/json.h"
#include "color_picker.h"

ExtruderColors* ExtruderColors::instance = nullptr;

// A blend is taken after this much filament has gone forward. It is long enough to span several of the
// mixer's own step patterns (so the blend is not just whichever stepper stepped last) and short enough
// to follow a gradient or a vtool change within a fraction of a millimeter of extrusion.
static constexpr double blend_window_mm = 0.2;

ExtruderColors::ExtruderColors(size_t count, bool mixing)
  : VirtualPrinter::Component("ExtruderColors"), count(std::min(count, max_slots)), mixing(mixing) {
  for (size_t i = 0; i < max_slots; ++i) color[i] = color_picker::default_color(i);
  instance = this;
}

ExtruderColors::~ExtruderColors() { if (instance == this) instance = nullptr; }

glm::vec3 ExtruderColors::blend() const {
  glm::vec3 c {};
  for (size_t i = 0; i < count; ++i) c += color[i] * float(mix[i]);
  return c;
}

glm::vec3 ExtruderColors::color_of(size_t extruder) {
  if (!instance) return color_picker::default_color(extruder);
  std::scoped_lock lock(instance->mutex);
  if (instance->mixing) return instance->mixed_color;
  return instance->color[extruder & (max_slots - 1)];
}

void ExtruderColors::observe(const std::vector<double>& stepper_mm) {
  if (!instance || !instance->mixing) return;
  ExtruderColors& s = *instance;
  std::scoped_lock lock(s.mutex);
  const size_t n = std::min(s.count, stepper_mm.size());
  if (!s.primed) {   // The first sample only sets the baseline
    for (size_t i = 0; i < n; ++i) s.last_mm[i] = stepper_mm[i];
    s.primed = true;
    return;
  }
  double pushed = 0;
  for (size_t i = 0; i < n; ++i) {
    const double d = stepper_mm[i] - s.last_mm[i];
    s.last_mm[i] = stepper_mm[i];
    if (d > 0) { s.window_mm[i] += d; s.total_mm[i] += d; pushed += d; }
  }
  double window = 0;
  for (size_t i = 0; i < n; ++i) window += s.window_mm[i];
  if (window < blend_window_mm) return;
  for (size_t i = 0; i < n; ++i) { s.mix[i] = s.window_mm[i] / window; s.window_mm[i] = 0; }
  s.mixed_color = s.blend();
  (void)pushed;
}

void ExtruderColors::ui_widget() {
  std::scoped_lock lock(mutex);
  if (mixing) ImGui::TextWrapped("Mixing extruder: %d filaments through one nozzle. The extrusion is drawn in the blend the E steppers actually delivered.", int(count));
  else ImGui::TextDisabled("Click a color to change it.");
  for (size_t i = 0; i < count; ++i) {
    ImGui::PushID(int(i));
    const std::string caption = (mixing ? "E" : "T") + std::to_string(i);
    color_picker::swatch(caption.c_str(), color[i]);
    ImGui::PopID();
    if (i + 1 < count) ImGui::SameLine(0, 12);
  }
  if (mixing) {
    ImGui::SeparatorText("Delivered mix");
    ImGui::ColorButton("##mixed", ImVec4(mixed_color.r, mixed_color.g, mixed_color.b, 1.0f), ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoBorder, ImVec2(40, 40));
    ImGui::SameLine();
    ImGui::BeginGroup();
    for (size_t i = 0; i < count; ++i) {
      const float f = float(mix[i]);
      char text[48];
      snprintf(text, sizeof(text), "E%d  %.0f%%   (%.1f mm)", int(i), f * 100.0f, total_mm[i]);
      ImGui::ProgressBar(f, ImVec2(220, 0), text);
    }
    ImGui::EndGroup();
  }
}

void ExtruderColors::serialize(agent::JsonWriter& writer) const {
  std::scoped_lock lock(mutex);
  writer.begin_object();
  writer.member("mixing", mixing);
  writer.key("colors");
  writer.begin_array();
  for (size_t i = 0; i < count; ++i) writer.value(color_picker::to_hex(color[i]));
  writer.end_array();
  if (mixing) {
    writer.member("mixed_color", color_picker::to_hex(mixed_color));
    writer.key("mix");   // The last measured proportions of the steppers (0..1)
    writer.begin_array();
    for (size_t i = 0; i < count; ++i) writer.value(mix[i]);
    writer.end_array();
    writer.key("total_mm");   // Filament pushed forward by each stepper since the start
    writer.begin_array();
    for (size_t i = 0; i < count; ++i) writer.value(total_mm[i]);
    writer.end_array();
  }
  writer.end_object();
}

// {"slot": 1, "color": "#ff8800"} sets the color of an extruder (mixing: of a mixing stepper's filament)
bool ExtruderColors::apply(const agent::JsonValue& body, std::string& error) {
  double slot;
  std::string hex;
  glm::vec3 rgb;
  if (!body.get_double("slot", slot) || !body.get_string("color", hex)) {
    error = "'slot' (number) and 'color' (\"#rrggbb\") are required";
    return false;
  }
  if (slot < 0 || size_t(slot) >= count || !color_picker::parse_hex(hex, rgb)) {
    error = "slot out of range or color is not \"#rrggbb\"";
    return false;
  }
  std::scoped_lock lock(mutex);
  color[size_t(slot)] = rgb;
  mixed_color = blend();
  return true;
}
