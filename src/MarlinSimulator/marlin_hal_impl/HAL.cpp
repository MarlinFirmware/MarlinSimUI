/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include <src/inc/MarlinConfig.h>
#include <src/HAL/shared/Delay.h>

MSerialT serial_stream_0(false);
MSerialT serial_stream_1(false);
MSerialT serial_stream_2(false);
MSerialT serial_stream_3(false);

// U8glib required functions
extern "C" {
  void u8g_xMicroDelay(uint16_t val) { DELAY_US(val); }
  void u8g_MicroDelay()              { u8g_xMicroDelay(1); }
  void u8g_10MicroDelay()            { u8g_xMicroDelay(10); }
  void u8g_Delay(uint16_t val)       { delay(val); }
}

//************************//

// return free heap space
int freeMemory() {
  return 0;
}

// ------------------------
// ADC
// ------------------------

void MarlinHAL::adc_init() {

}

void MarlinHAL::adc_enable(const uint8_t ch) {

}

uint8_t MarlinHAL::active_ch = 0;
void MarlinHAL::adc_start(const uint8_t ch) {
  active_ch = ch;
}

bool MarlinHAL::adc_ready() {
  return true;
}

uint16_t MarlinHAL::adc_value() {
  pin_t pin = analogInputToDigitalPin(active_ch);
  if (!isValidPin(pin)) return 0;
  uint16_t data = ((Gpio::get(pin) >> 2) & 0x3FF);
  return data;    // return 10bit value as Marlin expects
}

void MarlinHAL::reboot() {
  Kernel::shutdown();
}

void MarlinHAL::idletask() {  Kernel::yield(); };

//
// Watchdog emulation
//
// Marlin enables the watchdog (USE_WATCHDOG) and promises to call hal.watchdog_refresh() often. A real
// board resets itself when that doesn't happen for WDT_TIMEOUT. The simulator can't restart Marlin, so it
// reports the reset (stdout and the agent /kernel state) and requests a quit, which stops Marlin the
// same way a reboot would. Time is Marlin's clock, so the result doesn't depend on realtime_scale.
// Set MARLIN_SIM_WATCHDOG=0 to turn the emulation off.
//
namespace SimWatchdog {
  constexpr uint64_t timeout_ms = TERN(WATCHDOG_DURATION_8S, 8000, 4000);
  bool enabled = false;               // Marlin has called watchdog_init() and the emulation is on
  uint64_t last_refresh_ms = 0;       // Marlin time of the latest refresh
  uint64_t max_gap_ms = 0;            // Longest time seen between refreshes
  uint32_t resets = 0;                // Number of watchdog resets reported
  uint64_t reset_gap_ms = 0;          // Gap that caused the latest reset
}

void MarlinHAL::watchdog_refresh() {
  if (SimWatchdog::enabled) {
    const uint64_t now = Kernel::TimeControl::millis();
    // Marlin's clock can read a few ms behind a timer ISR's, so compare signed
    const int64_t gap = int64_t(now - SimWatchdog::last_refresh_ms);
    if (gap > 0) NOLESS(SimWatchdog::max_gap_ms, uint64_t(gap));
    SimWatchdog::last_refresh_ms = now;
  }
  Kernel::yield();
}

void MarlinHAL::watchdog_init() {
  const char *env = getenv("MARLIN_SIM_WATCHDOG");
  SimWatchdog::enabled = !(env && env[0] == '0');
  SimWatchdog::last_refresh_ms = Kernel::TimeControl::millis();
}

// Called every millisecond (independent of what Marlin is doing)
static void watchdog_check() {
  if (!SimWatchdog::enabled) return;
  const uint64_t now = Kernel::TimeControl::millis();
  const int64_t sgap = int64_t(now - SimWatchdog::last_refresh_ms);   // Signed: see watchdog_refresh()
  if (sgap <= int64_t(SimWatchdog::timeout_ms)) return;
  const uint64_t gap = uint64_t(sgap);
  SimWatchdog::enabled = false;
  SimWatchdog::resets++;
  SimWatchdog::reset_gap_ms = gap;
  NOLESS(SimWatchdog::max_gap_ms, gap);
  printf("WATCHDOG RESET: no watchdog_refresh() for %llu ms (limit %llu ms) at %llu ms\n",
         (unsigned long long)gap, (unsigned long long)SimWatchdog::timeout_ms, (unsigned long long)now);
  fflush(stdout);
  Kernel::quit_requested = true;
}

// Maple Compatibility
volatile uint32_t systick_uptime_millis = 0;
systickCallback_t systick_user_callback;
void systick_attach_callback(systickCallback_t cb) { systick_user_callback = cb; }
void SYSTICK_IRQHandler() {
  systick_uptime_millis++;
  watchdog_check();
  if (systick_user_callback) systick_user_callback();
}
