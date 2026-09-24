#pragma once

/**
 * AgentServer — HTTP/JSON control endpoint for automated (agent) control of
 * the simulator.
 *
 * THREADING CONTRACT (this is the whole point of the class, do not break it):
 *
 *   1. The server thread only ever parses HTTP and touches its own queue. It
 *      MUST NOT read or write Marlin state, the planner, thermalManager, or
 *      any VirtualPrinter::Component. Those live on the simulation thread and
 *      are raced by the stepper ISR.
 *   2. A parsed request is pushed onto `pending` with a std::promise, and the
 *      server thread blocks on the future.
 *   3. Kernel::execute_loop() calls AgentServer::service() on the simulation
 *      thread, at the same point it drains net_serial, where no ISR is on the
 *      isr_stack. Handlers run there and fulfil the promise.
 *   4. The server thread wakes, serialises the response and writes the socket.
 *
 * service() drains at most `max_requests_per_service` per call so a chatty
 * client cannot starve the stepper ISR.
 *
 * The listener binds 127.0.0.1 only. This endpoint can execute G-code and
 * (later) override pin state; it must not be reachable off-host.
 */

#include <cstdint>
#include <cstddef>
#include <string>
#include <map>
#include <deque>
#include <mutex>
#include <thread>
#include <atomic>
#include <future>
#include <memory>
#include <functional>

namespace agent {

enum class Method { GET, POST, UNSUPPORTED };

struct Request {
  Method method = Method::UNSUPPORTED;
  std::string path;   // path with query string stripped
  std::string query;  // raw query string, may be empty
  std::string body;

  // Look up a query-string parameter. Returns false if absent.
  bool query_param(const std::string& name, std::string& out) const;
};

struct Response {
  int status = 200;
  std::string content_type = "application/json";
  std::string body;

  void json(const std::string& payload) { body = payload; }
  void error(int code, const std::string& message);
};

// Handlers run ON THE SIMULATION THREAD. Reading Marlin state here is safe.
using Handler = std::function<void(const Request&, Response&)>;

// Where a route's handler is executed.
enum class Affinity {
  Simulation,  // queued and serviced by Kernel::execute_loop (default; safe for Marlin state)
  Direct       // run immediately on the server thread — ONLY for handlers that
               // touch nothing but atomics. Required for any control that must
               // work while the simulation thread is blocked (e.g. unfreezing
               // realtime_scale == 0, which otherwise deadlocks).
};

class AgentServer {
public:
  ~AgentServer();

  // Start listening on 127.0.0.1:port. Returns false if the socket could not
  // be opened (the simulator continues without an agent interface rather than
  // aborting, unlike RawSocketSerial which calls exit()).
  bool start(uint16_t port);
  void stop();

  bool running() const { return thread_active.load(); }
  uint16_t port() const { return listen_port; }

  // Register a handler for an exact "METHOD /path" route, e.g. "GET /kernel".
  void route(const std::string& key, Handler handler, Affinity affinity = Affinity::Simulation,
             uint32_t timeout_ms = 0);

  // Register a handler matching any path starting with the given
  // "METHOD /prefix/", e.g. "GET /state/" catches "GET /state/Bed Heater".
  // Exact routes always win; prefix routes are tried longest-first.
  void route_prefix(const std::string& key, Handler handler, Affinity affinity = Affinity::Simulation);

  // Percent-decode a URL path segment ("%20" -> ' ', '+' left as-is).
  static std::string percent_decode(const std::string& text);

  // Called from Kernel::execute_loop() on the simulation thread.
  void service();

  static constexpr uint32_t max_requests_per_service = 8;

private:
  struct PendingRequest {
    Request request;
    std::promise<Response> response;
  };

  struct Route {
    Handler handler;
    Affinity affinity = Affinity::Simulation;
    // Per-route service budget in ms. 0 uses the default. Routes that block on
    // purpose (POST /gcode with wait) need more than the default, but raising
    // the default globally would mask a genuinely frozen simulation.
    uint32_t timeout_ms = 0;
  };

  void execute();                              // server thread entry
  bool read_request(void* socket, Request&);   // socket fd as intptr_t; keeps platform headers out
  void dispatch(const Request&, Response&);    // simulation thread
  const Route* find_route(const Request&) const;

  std::map<std::string, Route> handlers;
  std::map<std::string, Route> prefix_handlers;
  std::deque<std::shared_ptr<PendingRequest>> pending;
  std::mutex queue_mutex;

  std::thread server_thread;
  std::atomic_bool thread_active { false };
  uint16_t listen_port = 0;

  void* listen_socket = nullptr;  // listening socket fd, stored as intptr_t
};

} // namespace agent
