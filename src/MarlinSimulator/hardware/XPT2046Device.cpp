#include <mutex>
#include <fstream>
#include <cmath>
#include <random>
#include <algorithm>
#include "Gpio.h"

#include <gl.h>

#include "XPT2046Device.h"
#include "../agent/json.h"

#include "../paths.h"
#include MARLIN_HAL_PATH(tft/xpt2046.h)
#if ENABLED(TOUCH_SCREEN)
  #include <src/lcd/tft/touch.h>
  #if ENABLED(TOUCH_SCREEN_CALIBRATION)
    #include <src/lcd/tft_io/touch_calibration.h>
  #endif
#else
  #define MINIMUM_HOLD_TIME 15
#endif

void XPT2046Device::onByteReceived(uint8_t _byte) {
  SPISlavePeripheral::onByteReceived(_byte);
  switch (_byte) {
    //TODO: touch hold
    case XPT2046_Z1:
      // A Z1 request after Y was read means a new getRawPoint() started, i.e.
      // Marlin completed a full poll of this device.
      if (saw_y) { saw_y = false; if (poll_count < 0xFF) poll_count++; }
      if (dirty) {
        setResponse16(XPT2046_Z1_THRESHOLD); // respond that we have data to send
      }
      else {
        setResponse16(0);
      }
      break;

    case XPT2046_X:
      setResponse16(lastClickX);
      break;

    case XPT2046_Y:
      saw_y = true;
      setResponse16(lastClickY);
      break;

    default:
      break;
  }
}

void XPT2046Device::onEndTransaction() {
  SPISlavePeripheral::onEndTransaction();
  if (!dirty) return;

  // Marlin's Touch::idle() only acts on a control when it has TWO consecutive
  // polls reporting a touch: the first records the point, the second sees
  // (x,y) != 0 and activates. A real mouse click is far shorter than the time
  // between polls, so releasing on the elapsed-time rule alone dropped the
  // touch after a single poll and the click was silently ignored.
  //
  // Hold the touch until Marlin has actually polled it MIN_TOUCH_POLLS times
  // (counted in onByteReceived -- see the note in the header), not merely for
  // a span of simulation time.
  if (poll_count < MIN_TOUCH_POLLS) return;

  if (held) return;   // Still pressed: keep reporting until released.

  const auto now = Kernel::SimulationRuntime::millis();
  if (now < inject_until_ms) return;   // Agent-injected press still held
  if ((now - touch_time) > (MINIMUM_HOLD_TIME * 3)) dirty = false;
};

bool XPT2046Device::inject_touch(float rx, float ry, uint32_t hold_ms) {
  rx = std::clamp(rx, 0.0f, 1.0f);
  ry = std::clamp(ry, 0.0f, 1.0f);
  last_ratio_x = rx; last_ratio_y = ry;
  lastClickX = uint16_t(1024.0f * rx);
  lastClickY = uint16_t(1024.0f * ry);
  poll_count = 0;
  touch_time = Kernel::SimulationRuntime::millis();
  inject_until_ms = touch_time + hold_ms;
  injected_touches++;
  dirty = true;
  return true;
}

void XPT2046Device::serialize(agent::JsonWriter& writer) const {
  writer.begin_object();
  writer.member("raw_x", double(lastClickX));
  writer.member("raw_y", double(lastClickY));
  writer.member("dirty", dirty);
  writer.member("held", held);
  writer.member("polls", double(poll_count));
  writer.member("injected_touches", double(injected_touches));
  writer.member("touch_frames", double(touch_frames));
  #if ALL(TOUCH_SCREEN, TOUCH_SCREEN_CALIBRATION)
    const auto &c = touch_calibration.calibration;
    writer.member("cal_x", double(c.x));
    writer.member("cal_y", double(c.y));
    writer.member("offset_x", double(c.offset_x));
    writer.member("offset_y", double(c.offset_y));
    writer.member("orientation", double(c.orientation));
  #endif
  writer.end_object();
}

void XPT2046Device::ui_widget() {
  ImGui::Text("raw: %d, %d", lastClickX, lastClickY);
  ImGui::Text("held: %s  dirty: %s  polls: %u", last_held ? "YES" : "no", dirty ? "YES" : "no", poll_count);
  ImGui::Text("ratio: %.3f, %.3f", last_ratio_x, last_ratio_y);
  ImGui::Text("touch frames: %u", touch_frames);

  #if ALL(TOUCH_SCREEN, TOUCH_SCREEN_CALIBRATION)
    // The stored transform Marlin applies to the raw values above. If touches
    // work uncalibrated but not after calibrating, the answer is here.
    const auto &c = touch_calibration.calibration;
    ImGui::Separator();
    ImGui::Text("cal x,y: %d, %d", (int)c.x, (int)c.y);
    ImGui::Text("offset : %d, %d", (int)c.offset_x, (int)c.offset_y);
    ImGui::Text("orient : %u", (unsigned)c.orientation);
    // Marlin: v = ((raw * cal) >> 16) + offset
    ImGui::Text("-> screen: %d, %d",
      (int)(int16_t((int32_t(lastClickX) * c.x) >> 16) + c.offset_x),
      (int)(int16_t((int32_t(lastClickY) * c.y) >> 16) + c.offset_y));
  #endif
}

void XPT2046Device::ui_callback(const bool is_held, const ImVec2 rect_min, const ImVec2 rect_max) {
  last_held = is_held;
  held = is_held;
  if (!is_held) return;

  const float w = rect_max.x - rect_min.x, h = rect_max.y - rect_min.y;
  if (w <= 0.0f || h <= 0.0f) return;

  const ImVec2 mouse = ImGui::GetMousePos();

  // A drag can travel outside the screen rect while the button still holds the
  // mouse. Clamp so the panel edge behaves like a real touchscreen edge instead
  // of reporting out-of-range values to Marlin.
  const float rx = std::clamp((mouse.x - rect_min.x) / w, 0.0f, 1.0f),
              ry = std::clamp((mouse.y - rect_min.y) / h, 0.0f, 1.0f);

  last_ratio_x = rx; last_ratio_y = ry;
  touch_frames++;

  lastClickX = uint16_t(1024.0f * rx);
  lastClickY = uint16_t(1024.0f * ry);

  // A new press starts a fresh poll budget; a continuing hold keeps the one
  // already in progress so dragging isn't restarted every frame.
  if (!dirty) poll_count = 0;

  // Refreshed every frame the touch is held, so onEndTransaction's decay timer
  // only starts running once the mouse is actually released.
  dirty = true;
  touch_time = Kernel::SimulationRuntime::millis();
}
