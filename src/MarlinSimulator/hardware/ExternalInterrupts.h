#pragma once

/**
 * External (pin change) interrupts for attachInterrupt() / detachInterrupt().
 *
 * Simulated inputs such as endstops and probes compute their level when the pin is read,
 * so a level change never arrives as a GPIO event. Instead the attached pins are sampled:
 *  - right after every kinematic update (a step moved the machine), and
 *  - by the "External Interrupts" kernel timer, which also catches changes made from the UI
 *    or the agent interface (manual endstop override, probe deploy, ...).
 *
 * A matching edge marks the line pending and schedules the External Interrupts pseudo-ISR to run
 * at once. Like an EXTI line it waits while a higher priority ISR (the stepper) is running or while
 * interrupts are disabled, then calls each pending handler.
 */

#include <cstdint>

#include "Gpio.h"

namespace ExternalInterrupts {
  // Arduino attachInterrupt() / detachInterrupt()
  void attach(const pin_type pin, void (*isr)(), const uint32_t mode);
  void detach(const pin_type pin);

  // Look for edges on attached pins. Safe to call from any thread: only the simulation thread
  // samples, and the timer sweep picks up changes made from other threads.
  void sample();

  // The External Interrupts pseudo-ISR (Kernel timer)
  void isr();
}
