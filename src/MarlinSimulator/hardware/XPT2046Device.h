#pragma once

#include <SDL2/SDL.h>
#include "../user_interface.h"

#include <list>
#include <deque>
#include "SPISlavePeripheral.h"

class XPT2046Device: public SPISlavePeripheral {
public:
  XPT2046Device(SpiBus& spi_bus, pin_type cs) : SPISlavePeripheral(spi_bus, cs) {}
  virtual ~XPT2046Device() {};

  void update() {}
  void ui_widget();

  // 'held' and the screen rect are passed explicitly rather than read from
  // ImGui's "last item" state: ui_callback() is called well after the screen
  // button is submitted, and any item added in between (or a frame where the
  // button is skipped entirely) would silently retarget IsItemActive() at the
  // wrong widget.
  void ui_callback(const bool held, const ImVec2 rect_min, const ImVec2 rect_max);

  void onByteReceived(uint8_t _byte) override;
  void onEndTransaction() override;

  uint16_t lastClickX = 0;
  uint16_t lastClickY = 0;
  bool dirty = false;
  uint64_t touch_time;

  // Marlin needs two consecutive polls reporting a touch before it activates a
  // control, so a press is held for at least this many polls regardless of how
  // briefly the mouse button was actually down.
  //
  // NOTE: one Marlin poll (XPT2046::getRawPoint) performs THREE SPI
  // transactions -- Z1, X, then Y -- so onEndTransaction cannot be used to
  // count polls. A poll boundary is detected in onByteReceived instead: a Z1
  // request arriving after Y has been read means a new getRawPoint began.
  static constexpr uint8_t MIN_TOUCH_POLLS = 2;
  uint8_t poll_count = 0;
  bool saw_y = false;
  bool held = false;

  // Agent-injected touch: held (like a mouse press) until this sim time.
  uint64_t inject_until_ms = 0;
  uint32_t injected_touches = 0;

  bool inject_touch(float rx, float ry, uint32_t hold_ms) override;
  void serialize(agent::JsonWriter& writer) const override;

  // Diagnostics for the Components panel
  bool  last_held = false;
  float last_ratio_x = 0.0f, last_ratio_y = 0.0f;
  uint32_t touch_frames = 0;
};
