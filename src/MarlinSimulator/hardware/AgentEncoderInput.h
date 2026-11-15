#pragma once

#include <cstdint>
#include <cstdlib>
#include <algorithm>

#include "../execution_control.h"

/**
 * Encoder rotation and button presses from the agent interface (POST /encoder,
 * POST /button), for the LCD panels with an encoder.
 *
 * This is kept apart from the panel's keyboard state, which the UI rewrites
 * every frame. Whenever Marlin reads a pin, the panel adds position() to its
 * own encoder position and ORs pressed() into its button.
 *
 * Rotation steps through the quadrature states in simulated time, one every
 * interval_ms, so Marlin samples every state. Presses are held for hold_ms of
 * simulated time and until Marlin has read the pin at least once.
 *
 * All calls are on the simulation thread.
 */
class AgentEncoderInput {
public:
  enum Button : uint8_t { KILL, ENCODER, BACK, COUNT };

  // Start turning by 'pulses' quadrature states. False if still turning.
  bool rotate(const int32_t pulses, const uint32_t interval_ms) {
    if (rotating()) return false;
    committed = position();
    total = pulses;
    step_ms = std::max<uint32_t>(interval_ms, 1);
    start_ms = now();
    return true;
  }

  // Quadrature states turned so far, for the panel to add to its own position
  int32_t position() const {
    const int32_t n = int32_t(std::min<uint64_t>(std::abs(total), (now() - start_ms) / step_ms));
    return committed + (total < 0 ? -n : n);
  }

  bool rotating() const { return (now() - start_ms) / step_ms < uint64_t(std::abs(total)); }

  uint64_t remaining_ms() const {
    const uint64_t end = start_ms + uint64_t(std::abs(total)) * step_ms, t = now();
    return end > t ? end - t : 0;
  }

  // Start pressing a button. False if it's still held from the last press.
  bool press(const Button b, const uint32_t hold_ms) {
    if (held[b]) return false;
    held[b] = true;
    release_ms[b] = now() + hold_ms;
    read[b] = false;
    return true;
  }

  // Called when Marlin reads the button's pin: true while the press is held
  bool pressed(const Button b) {
    if (!held[b]) return false;
    if (now() >= release_ms[b] && read[b]) {
      held[b] = false;
      return false;
    }
    read[b] = true;
    return true;
  }

  bool holding(const Button b) const { return held[b]; }

private:
  // Simulated time, never going backwards. ISRs run at their scheduled time, which
  // can be earlier than the time a request was handled in the main loop.
  uint64_t now() const {
    const uint64_t t = Kernel::SimulationRuntime::millis();
    if (t > latest_ms) latest_ms = t;
    return latest_ms;
  }
  mutable uint64_t latest_ms = 0;

  int32_t committed = 0, total = 0;
  uint32_t step_ms = 1;
  uint64_t start_ms = 0;

  bool held[COUNT] = {};
  uint64_t release_ms[COUNT] = {};
  bool read[COUNT] = {};    // Marlin has seen the press
};
