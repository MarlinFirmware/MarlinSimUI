#include "request_log.h"

namespace agent {

RequestLog request_log;

void RequestLog::append(const RequestRecord& record) {
  std::scoped_lock lock(mutex);
  entries.push_back(record);
  entries.back().sequence = sequence_counter++;
  while (entries.size() > max_entries) entries.pop_front();
}

void RequestLog::clear() {
  std::scoped_lock lock(mutex);
  entries.clear();
  // Deliberately does NOT reset sequence_counter, matching SerialLog: a
  // monotonic counter means "total requests" stays meaningful across a clear.
}

std::vector<RequestRecord> RequestLog::snapshot() const {
  std::scoped_lock lock(mutex);
  return std::vector<RequestRecord>(entries.begin(), entries.end());
}

size_t RequestLog::size() const {
  std::scoped_lock lock(mutex);
  return entries.size();
}

uint64_t RequestLog::total_requests() const {
  std::scoped_lock lock(mutex);
  return sequence_counter;
}

void RequestLog::set_max_entries(size_t max) {
  std::scoped_lock lock(mutex);
  max_entries = max ? max : 1;
  while (entries.size() > max_entries) entries.pop_front();
}

} // namespace agent
