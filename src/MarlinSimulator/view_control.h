#pragma once

//
// Thread-safe mailbox between the agent server and the Viewport camera.
// The agent posts a request; Visualisation::update() applies it on the UI thread
// and publishes the resulting view.
//

#include <mutex>
#include <string>

namespace view_control {

struct Request {
  bool has_yaw = false, has_pitch = false, has_distance = false, has_target = false, has_follow = false;
  float yaw = 0, pitch = 0, distance = 0;
  float target[3] {};  // Marlin coordinates (mm)
  bool follow = false;
  std::string preset;  // home, front, right, back, left, top, iso
  std::string machine; // bedslinger, cube, delta
};

struct State {
  float yaw = 0, pitch = 0, distance = 0;
  float target[3] {};  // Marlin coordinates (mm)
  bool follow = false;
  bool turntable = true;
  std::string machine;
};

inline std::mutex mutex;
inline bool pending = false;
inline Request request;
inline State state;

inline void post(const Request& r) {
  std::scoped_lock lock(mutex);
  request = r;
  pending = true;
}

inline State current() {
  std::scoped_lock lock(mutex);
  return state;
}

} // namespace view_control
