#pragma once

//
// Colors of the filament(s) on a printer that has no filament changer.
//
// Every extruder (or, on a mixing extruder, every mixing stepper) has a color the user picks in the UI
// (or sets from the agent interface, {"slot":N,"color":"#rrggbb"}). The extrusion is drawn in the color
// of its extruder.
//
// A MIXING_EXTRUDER pushes all its filaments through one nozzle. The simulator watches how much filament
// each E stepper really moves (the stepper step counts, not what Marlin believes it asked for), and the
// extrusion takes the color blended in that proportion. So a mix that Marlin gets wrong (wrong steps
// per stepper, a stepper that stops stepping, a mix that never changes) shows up in the viewport, and
// the measured proportions and totals are reported through the agent interface.
//

#include <array>
#include <mutex>
#include <vector>

#include <glm/glm.hpp>

#include "../virtual_printer.h"

class ExtruderColors : public VirtualPrinter::Component {
public:
  static constexpr size_t max_slots = 8;

  ExtruderColors(size_t count, bool mixing);
  ~ExtruderColors() override;

  // The color of extruder i (for a mixing extruder, the blend of the mix that is being extruded)
  static glm::vec3 color_of(size_t extruder);

  // Called from the kinematic update with the signed filament moved so far by each E stepper (mm)
  static void observe(const std::vector<double>& stepper_mm);

  void ui_widget() override;
  void serialize(agent::JsonWriter& writer) const override;
  bool apply(const agent::JsonValue& body, std::string& error) override;

private:
  static ExtruderColors* instance;

  glm::vec3 blend() const;

  const size_t count;
  const bool mixing;
  mutable std::mutex mutex;
  std::array<glm::vec3, max_slots> color {};

  // Mixing: what the steppers delivered
  std::array<double, max_slots> last_mm {};     // The stepper position at the last observation
  std::array<double, max_slots> window_mm {};   // Filament pushed since the last blend was taken
  std::array<double, max_slots> total_mm {};    // Filament pushed forward since the start (retractions are not counted)
  std::array<double, max_slots> mix {};         // The last measured proportions (they add up to 1)
  bool primed = false;
  glm::vec3 mixed_color { 0.3f, 0.3f, 0.3f };
};
