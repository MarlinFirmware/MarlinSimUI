#include <array>
#include <thread>
#include <vector>
#include <algorithm>

#include "../execution_control.h"
#include "ExternalInterrupts.h"   // Gpio.h before Arduino.h (pin_data::State is LOW, HIGH)

#include <Arduino.h>              // CHANGE, RISING, FALLING, HIGH, LOW

namespace ExternalInterrupts {

  // Index of the "External Interrupts" entry in Kernel::Timers::timers
  constexpr size_t timer_index = 4;

  // Sweep rate for changes that don't come from motion (UI, agent)
  constexpr uint64_t sweep_frequency = 1000;

  struct Line {
    void (*isr)() = nullptr;
    uint32_t mode = CHANGE;
    bool level = false;
    bool pending = false;
  };

  static std::array<Line, Gpio::pin_count> lines;
  static std::vector<pin_type> attached;    // Pins with a handler, in attach order
  static std::thread::id sim_thread;        // Thread that attached the handlers (Marlin's)

  static inline bool valid(const pin_type pin) { return pin >= 0 && pin < Gpio::pin_count; }

  static bool read_level(const pin_type pin) { return Gpio::get(pin) != 0; }

  // Run the pseudo-ISR as soon as the scheduler allows
  static void request_isr() {
    auto &timer = Kernel::Timers::timers[timer_index];
    const uint64_t now = Kernel::TimeControl::getTicks();
    timer.source_offset = now > timer.compare ? now - timer.compare : 0;
  }

  void attach(const pin_type pin, void (*isr)(), const uint32_t mode) {
    if (!valid(pin) || !isr) return;

    sim_thread = std::this_thread::get_id();

    auto &line = lines[pin];
    line.isr = isr;
    line.mode = mode;
    line.level = read_level(pin);
    line.pending = false;
    if (std::find(attached.begin(), attached.end(), pin) == attached.end()) attached.push_back(pin);

    auto &timer = Kernel::Timers::timers[timer_index];
    if (!timer.enabled()) {
      timer.timer_frequency = Kernel::TimeControl::frequency;
      timer.start(Kernel::TimeControl::getTicks(), sweep_frequency);
      timer.enable();
    }
  }

  void detach(const pin_type pin) {
    if (!valid(pin)) return;
    lines[pin] = Line{};
    attached.erase(std::remove(attached.begin(), attached.end(), pin), attached.end());
    if (attached.empty()) Kernel::Timers::timers[timer_index].disable();
  }

  void sample() {
    if (attached.empty()) return;

    // Kinematic updates also come from UI sliders. Leave those to the timer sweep.
    if (std::this_thread::get_id() != sim_thread) return;

    bool any = false;
    for (const auto pin : attached) {
      auto &line = lines[pin];
      const bool level = read_level(pin);
      if (level == line.level) continue;
      line.level = level;
      const bool fire = line.mode == CHANGE
                     || (line.mode == RISING  &&  level)
                     || (line.mode == FALLING && !level)
                     || (line.mode == HIGH    &&  level)
                     || (line.mode == LOW     && !level);
      if (fire) line.pending = any = true;
    }
    if (any) request_isr();
  }

  void isr() {
    sample();
    for (const auto pin : attached) {
      auto &line = lines[pin];
      if (line.pending && line.isr) {
        line.pending = false;
        line.isr();
      }
    }
  }

} // ExternalInterrupts

void EXTI_IRQHandler() { ExternalInterrupts::isr(); }
