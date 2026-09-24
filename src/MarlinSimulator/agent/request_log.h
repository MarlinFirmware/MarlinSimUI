#pragma once

/**
 * Bounded log of recent agent HTTP requests, for the Agent Activity UI panel.
 *
 * Its own mutex, like SerialLog: the server thread appends while the render
 * thread reads, and neither may block the other. Entries are plain values so
 * the UI never dereferences anything owned by the server thread.
 */

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace agent {

struct RequestRecord {
  uint64_t    sequence = 0;
  double      sim_seconds = 0.0;
  std::string method;
  std::string path;
  std::string query;
  int         status = 0;
  double      duration_ms = 0.0;
  bool        direct = false;   // Affinity::Direct (ran on the server thread)
};

class RequestLog {
public:
  static constexpr size_t default_max_entries = 500;

  void append(const RequestRecord& record);
  void clear();

  // Snapshot for the UI. Copies under the lock so the caller can render at
  // leisure without holding it.
  std::vector<RequestRecord> snapshot() const;

  size_t   size() const;
  uint64_t total_requests() const;
  void     set_max_entries(size_t max_entries);

private:
  mutable std::mutex       mutex;
  std::deque<RequestRecord> entries;
  size_t                   max_entries = default_max_entries;
  uint64_t                 sequence_counter = 0;
};

extern RequestLog request_log;

} // namespace agent
