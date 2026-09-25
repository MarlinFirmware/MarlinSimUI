#include "agent_interface.h"
#include "json.h"
#include "serial_log.h"
#include "png_writer.h"

#include "../execution_control.h"
#include "../virtual_printer.h"

#include "serial.h"
#include <src/gcode/queue.h>
#include <src/module/planner.h>

extern MSerialT serial_stream_3;

#include <cstring>
#include <chrono>

namespace agent {

AgentServer server;

namespace {

const char* timer_state(const KernelTimer& timer) {
  if (timer.running) return "running";
  return timer.active ? "enabled" : "disabled";
}

void write_kernel_state(JsonWriter& writer) {
  writer.begin_object();

  writer.member("ticks", Kernel::TimeControl::getTicks());
  writer.member("realtime_ticks", Kernel::TimeControl::getRealtimeTicks());
  writer.member("tick_frequency", uint64_t(Kernel::TimeControl::frequency));
  writer.member("realtime_scale", Kernel::TimeControl::realtime_scale.load());
  writer.member("sim_nanos", Kernel::SimulationRuntime::nanos());
  writer.member("sim_seconds", Kernel::SimulationRuntime::seconds());
  writer.member("isr_timing_error_ns", Kernel::isr_timing_error.load());
  writer.member("timers_active", Kernel::timers_active);
  writer.member("quit_requested", Kernel::quit_requested);

  writer.key("timers");
  writer.begin_array();
  for (auto& timer : Kernel::Timers::timers) {
    writer.begin_object();
    writer.member("name", timer.name);
    writer.member("state", timer_state(timer));
    writer.member("priority", timer.priority);
    writer.member("compare", timer.compare);
    writer.member("timer_frequency", timer.timer_frequency);
    writer.end_object();
  }
  writer.end_array();

  writer.end_object();
}

// GET /kernel — kernel clock, timers and scheduler health.
void handle_get_kernel(const Request&, Response& response) {
  JsonWriter writer;
  write_kernel_state(writer);
  response.json(writer.str());
}

// POST /kernel/control — time and execution control.
//
// Registered with Affinity::Direct: it touches only atomics, and MUST remain
// serviceable while the simulation thread is blocked — otherwise a client that
// sets realtime_scale to 0 can never resume it.
//
//   {"realtime_scale": 100.0}  fast-forward (0 freezes, 1.0 is wall clock)
//   {"break": true}            trigger Kernel::execution_break()
//   {"quit": true}             request clean shutdown
void handle_post_kernel_control(const Request& request, Response& response) {
  JsonValue body;
  if (!body.parse(request.body)) {
    response.error(400, "body must be a flat JSON object");
    return;
  }

  bool changed = false;

  double scale = 0.0;
  if (body.get_double("realtime_scale", scale)) {
    // 0 is legal (freeze). Negative would run time backwards.
    if (scale < 0.0 || scale > 1000.0) {
      response.error(400, "realtime_scale must be between 0 and 1000");
      return;
    }
    Kernel::TimeControl::realtime_scale = float(scale);
    changed = true;
  }

  bool flag = false;
  if (body.get_bool("break", flag) && flag) {
    Kernel::execution_break();
    changed = true;
  }

  if (body.get_bool("quit", flag) && flag) {
    Kernel::quit_requested = true;
    changed = true;
  }

  if (!changed) {
    response.error(400, "no recognised field (realtime_scale, break, quit)");
    return;
  }

  // Report only atomics — reading the timer array here would race the
  // simulation thread. Use GET /kernel for the full picture.
  JsonWriter writer;
  writer.begin_object();
  writer.member("ok", true);
  writer.member("realtime_scale", Kernel::TimeControl::realtime_scale.load());
  writer.member("ticks", Kernel::TimeControl::getTicks());
  writer.end_object();
  response.json(writer.str());
}

// GET /state — every registered component's state in one snapshot.
void handle_get_state(const Request&, Response& response) {
  JsonWriter writer;
  writer.begin_object();

  writer.key("kernel");
  write_kernel_state(writer);

  writer.key("components");
  VirtualPrinter::serialize_all(writer);

  writer.end_object();
  response.json(writer.str());
}

// GET /state/<component-name> — one component, by registry name.
// Names contain spaces and parentheses (e.g. "Endstop(X Min)"), so the path
// segment is percent-decoded before lookup.
void handle_get_state_component(const Request& request, Response& response) {
  constexpr const char* prefix = "/state/";
  const size_t prefix_length = strlen(prefix);

  if (request.path.size() <= prefix_length) {
    response.error(400, "missing component name");
    return;
  }

  std::string name = AgentServer::percent_decode(request.path.substr(prefix_length));

  JsonWriter writer;
  if (!VirtualPrinter::serialize_one(name, writer)) {
    response.error(404, "no component named '" + name + "' (see GET /components)");
    return;
  }
  response.json(writer.str());
}

// GET /components — the list of names usable with GET /state/<name>.
void handle_get_components(const Request&, Response& response) {
  JsonWriter writer;
  writer.begin_object();
  writer.key("components");
  writer.begin_array();
  for (auto const& name : VirtualPrinter::component_names()) writer.value(name);
  writer.end_array();
  writer.end_object();
  response.json(writer.str());
}

// GET /serial?since=<seq>&limit=<n>&stream=<0-3>
//
// Registered Affinity::Direct: the log has its own mutex, and it must stay
// readable when the simulation is frozen or wedged -- that is exactly when you
// want the last thing the firmware printed.
void handle_get_serial(const Request& request, Response& response) {
  uint64_t since = 0;
  size_t limit = 200;
  int stream = -1;

  std::string value;
  if (request.query_param("since", value))  since = strtoull(value.c_str(), nullptr, 10);
  if (request.query_param("limit", value))  limit = size_t(strtoull(value.c_str(), nullptr, 10));
  if (request.query_param("stream", value)) stream = atoi(value.c_str());

  if (limit > 5000) limit = 5000;  // bound the response size

  auto lines = serial_log.query(since, limit, stream);

  JsonWriter writer;
  writer.begin_object();
  writer.key("lines");
  writer.begin_array();
  for (auto const& line : lines) {
    writer.begin_object();
    writer.member("seq", double(line.sequence));
    writer.member("stream", double(line.stream));
    writer.member("sim_seconds", line.sim_seconds);
    writer.member("text", line.text);
    writer.end_object();
  }
  writer.end_array();

  // Resume point for the next poll. This MUST be one past the last line
  // actually RETURNED, not the global counter -- with limit-capped or
  // stream-filtered results those differ, and using the counter would silently
  // skip every line the page didn't include.
  uint64_t next = since;
  if (!lines.empty()) next = lines.back().sequence + 1;

  writer.member("next_seq", double(next));
  // Where the log itself is up to; next_seq < head_seq means more is pending.
  writer.member("head_seq", double(serial_log.next_sequence()));
  writer.member("returned", double(lines.size()));
  writer.member("retained", double(serial_log.size()));
  // Oldest sequence still held. If your ?since= is below this, the lines in
  // between were evicted and you have a gap.
  writer.member("oldest_seq", double(serial_log.oldest_sequence()));
  writer.member("max_lines", double(serial_log.max_lines()));
  // Non-zero means output was evicted before being read: the agent has a gap.
  writer.member("dropped", double(serial_log.dropped_lines()));
  writer.end_object();
  response.json(writer.str());
}

// POST /serial — log maintenance.
//   {"clear": true}        drop retained lines (sequence numbers keep rising)
//   {"max_lines": 50000}   resize retention
void handle_post_serial(const Request& request, Response& response) {
  JsonValue body;
  if (!body.parse(request.body)) {
    response.error(400, "body is not valid JSON");
    return;
  }

  bool flag = false;
  if (body.get_bool("clear", flag) && flag) serial_log.clear();

  double limit = 0;
  if (body.get_double("max_lines", limit)) {
    if (limit < 0 || limit > 1000000) {
      response.error(400, "max_lines must be 0..1000000");
      return;
    }
    serial_log.set_max_lines(size_t(limit));
  }

  JsonWriter writer;
  writer.begin_object();
  writer.member("ok", true);
  writer.member("retained", double(serial_log.size()));
  writer.member("max_lines", double(serial_log.max_lines()));
  writer.member("next_seq", double(serial_log.next_sequence()));
  writer.end_object();
  response.json(writer.str());
}

// POST /screenshot — write a display capture to a file.
//   {"path": "/tmp/lcd.png"}                  first display found
//   {"path": "...", "display": "<name>"}      a specific display
//
// Writing to a caller-supplied path is the point: an agent asks for its own
// tmp dir, then reads the PNG back with its normal file tools.
void handle_post_screenshot(const Request& request, Response& response) {
  JsonValue body;
  if (!body.parse(request.body)) {
    response.error(400, "body is not valid JSON");
    return;
  }

  std::string path;
  if (!body.get_string("path", path) || path.empty()) {
    response.error(400, "'path' is required, e.g. {\"path\": \"/tmp/lcd.png\"}");
    return;
  }

  std::string display;
  body.get_string("display", display);

  std::vector<uint8_t> rgb;
  uint32_t width = 0, height = 0;
  std::string matched;

  if (!VirtualPrinter::capture_display(display, rgb, width, height, matched)) {
    response.error(404, display.empty()
      ? "no display component found (see GET /displays)"
      : "no display named '" + display + "' (see GET /displays)");
    return;
  }

  std::string error;
  if (!write_png_rgb(path, width, height, rgb.data(), error)) {
    response.error(500, error);
    return;
  }

  JsonWriter writer;
  writer.begin_object();
  writer.member("ok", true);
  writer.member("path", path);
  writer.member("display", matched);
  writer.member("width", double(width));
  writer.member("height", double(height));
  writer.member("bytes", double(size_t(width) * size_t(height) * 3));
  writer.end_object();
  response.json(writer.str());
}

// GET /displays — registry names of components that can be captured.
void handle_get_displays(const Request&, Response& response) {
  JsonWriter writer;
  writer.begin_object();
  writer.key("displays");
  writer.begin_array();
  for (auto const& name : VirtualPrinter::display_names()) writer.value(name);
  writer.end_array();
  writer.end_object();
  response.json(writer.str());
}

// POST /gcode — submit G-code the way a host would.
//   {"command": "G28"}
//
// Writes into the serial RX buffer rather than GCodeQueue::inject(), which has
// a single 64-char slot that silently overwrites a pending command.
//
// This NEVER blocks. A handler cannot wait for motion to finish: it is serviced
// from inside the marlin_loop ISR, and execute_loop only fires timers with
// priority < the active one (execution_control.cpp), so Kernel::yield() cannot
// re-enter marlin_loop. The G-code queue would never be pumped and the wait
// would deadlock the simulation thread permanently -- taking every route with
// it, since the accept loop is single-threaded.
//
// To wait, poll GET /idle until {"idle": true}.
void handle_post_gcode(const Request& request, Response& response) {
  JsonValue body;
  if (!body.parse(request.body)) {
    response.error(400, "body is not valid JSON");
    return;
  }

  std::string command;
  if (!body.get_string("command", command) || command.empty()) {
    response.error(400, "'command' is required, e.g. {\"command\": \"G28\"}");
    return;
  }
  if (command.find('\n') != std::string::npos) {
    response.error(400, "'command' must be a single line");
    return;
  }

  // Sequence before submitting, so the caller can read exactly the output this
  // command produced via GET /serial?since=<serial_from>.
  const uint64_t serial_from = serial_log.next_sequence();

  command += '\n';
  serial_stream_3.receive_buffer.write((uint8_t*)command.data(), command.size());

  JsonWriter writer;
  writer.begin_object();
  writer.member("ok", true);
  writer.member("command", command.substr(0, command.size() - 1));
  writer.member("serial_from", double(serial_from));
  writer.member("hint", std::string("poll GET /idle until idle:true, then GET /serial?since=serial_from"));
  writer.end_object();
  response.json(writer.str());
}

// GET /idle — is the machine done with everything submitted so far?
//
// Cheap and non-blocking, so an agent can poll it between moves. rx_pending
// matters: immediately after POST /gcode the bytes are still in the serial
// receive buffer and both the queue and planner are empty, so a check that
// omitted it would report idle before the command had even been parsed.
void handle_get_idle(const Request&, Response& response) {
  const bool rx_pending   = serial_stream_3.receive_buffer.available() != 0;
  const bool queued       = queue.has_commands_queued();
  const bool blocks       = planner.has_blocks_queued();
  const bool planner_busy = planner.busy();

  JsonWriter writer;
  writer.begin_object();
  writer.member("idle", !(rx_pending || queued || blocks || planner_busy));
  writer.member("rx_pending", rx_pending);
  writer.member("commands_queued", queued);
  writer.member("blocks_queued", blocks);
  writer.member("planner_busy", planner_busy);
  writer.member("moves_queued", double(planner.movesplanned()));
  writer.end_object();
  response.json(writer.str());
}

// POST /touch — press the touchscreen, as a user would with the mouse.
//   {"x": 0.5, "y": 0.5}                 tap at panel ratios 0..1
//   {"x": 0.5, "y": 0.5, "hold_ms": 1500} press-and-hold (simulated ms)
//
// Never blocks: the press is released by the device once hold_ms has elapsed
// AND Marlin has polled it enough times to act on it. Take a screenshot
// afterwards to see the result.
void handle_post_touch(const Request& request, Response& response) {
  JsonValue body;
  if (!body.parse(request.body)) {
    response.error(400, "body is not valid JSON");
    return;
  }

  double x = -1.0, y = -1.0, hold = 0.0;
  if (!body.get_double("x", x) || !body.get_double("y", y)) {
    response.error(400, "'x' and 'y' (ratios 0..1) are required");
    return;
  }
  if (x < 0.0 || x > 1.0 || y < 0.0 || y > 1.0) {
    response.error(400, "'x' and 'y' must be between 0 and 1");
    return;
  }
  body.get_double("hold_ms", hold);
  if (hold < 0.0 || hold > 60000.0) {
    response.error(400, "'hold_ms' must be between 0 and 60000");
    return;
  }

  std::string matched;
  if (!VirtualPrinter::inject_touch(float(x), float(y), uint32_t(hold), matched)) {
    response.error(404, "no touch device found");
    return;
  }

  JsonWriter writer;
  writer.begin_object();
  writer.member("ok", true);
  writer.member("device", matched);
  writer.member("x", x);
  writer.member("y", y);
  writer.member("hold_ms", hold);
  writer.end_object();
  response.json(writer.str());
}

// GET /ping — liveness probe that proves the simulation thread is servicing
// requests (it can only be answered from execute_loop).
void handle_get_ping(const Request&, Response& response) {
  JsonWriter writer;
  writer.begin_object();
  writer.member("ok", true);
  writer.member("sim_seconds", Kernel::SimulationRuntime::seconds());
  writer.end_object();
  response.json(writer.str());
}

} // namespace

void register_routes() {
  server.route("GET /ping", handle_get_ping);
  server.route("GET /kernel", handle_get_kernel);
  server.route("GET /state", handle_get_state);
  server.route("GET /components", handle_get_components);
  server.route("POST /kernel/control", handle_post_kernel_control, Affinity::Direct);

  // Serial log: own mutex, and must stay readable while the sim is frozen.
  server.route("GET /serial", handle_get_serial, Affinity::Direct);
  server.route("POST /serial", handle_post_serial, Affinity::Direct);

  // Simulation-thread routes: these touch Marlin/component state.
  // POST /gcode never blocks, so it needs no special budget.
  server.route("POST /gcode", handle_post_gcode);
  server.route("GET /idle", handle_get_idle);
  server.route("POST /screenshot", handle_post_screenshot);
  server.route("GET /displays", handle_get_displays);
  server.route("POST /touch", handle_post_touch);

  // Prefix route: any GET /state/<name>.
  server.route_prefix("GET /state/", handle_get_state_component);
}

} // namespace agent
