#pragma once

/**
 * Minimal JSON serialisation helpers for the agent interface.
 *
 * Deliberately tiny: the agent interface only ever *emits* JSON, and only
 * emits objects, arrays, numbers, bools and strings. Pulling in a full JSON
 * library would add a vendored dependency and inflate an already slow native
 * build for no benefit.
 *
 * Request parsing uses JsonValue, which is an equally minimal reader covering
 * the flat `{"key": <scalar>}` bodies the POST endpoints accept.
 */

#include <string>
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <map>

namespace agent {

class JsonWriter {
public:
  void begin_object() { separate(); buffer += '{'; first = true; }
  void end_object()   { buffer += '}'; first = false; }
  void begin_array()  { separate(); buffer += '['; first = true; }
  void end_array()    { buffer += ']'; first = false; }

  void key(const std::string& k) {
    separate();
    write_string(k);
    buffer += ':';
    first = true;  // value follows immediately, no comma
  }

  void value(const std::string& v) { separate(); write_string(v); }
  void value(const char* v)        { separate(); write_string(v); }
  void value(bool v)               { separate(); buffer += v ? "true" : "false"; }

  void value(double v) {
    separate();
    if (std::isfinite(v)) {
      char tmp[40];
      snprintf(tmp, sizeof(tmp), "%.6g", v);
      buffer += tmp;
    }
    else buffer += "null";  // JSON has no NaN/Infinity
  }

  void value(float v)    { value(double(v)); }
  void value(uint64_t v) { separate(); buffer += std::to_string(v); }
  void value(int64_t v)  { separate(); buffer += std::to_string(v); }
  void value(uint32_t v) { value(uint64_t(v)); }
  void value(int v)      { value(int64_t(v)); }

  // Convenience: emit "key": value in one call.
  template<typename T> void member(const std::string& k, T v) { key(k); value(v); }

  const std::string& str() const { return buffer; }
  void clear() { buffer.clear(); first = true; }

private:
  void separate() {
    if (!first) buffer += ',';
    first = false;
  }

  void write_string(const std::string& s) {
    buffer += '"';
    for (char c : s) {
      switch (c) {
        case '"':  buffer += "\\\""; break;
        case '\\': buffer += "\\\\"; break;
        case '\n': buffer += "\\n";  break;
        case '\r': buffer += "\\r";  break;
        case '\t': buffer += "\\t";  break;
        default:
          if ((unsigned char)c < 0x20) {
            char tmp[8];
            snprintf(tmp, sizeof(tmp), "\\u%04x", c);
            buffer += tmp;
          }
          else buffer += c;
      }
    }
    buffer += '"';
  }

  std::string buffer;
  bool first = true;
};

/**
 * Flat JSON object reader. Parses `{"a": 1, "b": true, "c": "str"}` into a
 * string->string map, keeping the raw token for each value. Nested objects and
 * arrays are not supported and are reported as a parse failure; no current
 * endpoint needs them.
 */
class JsonValue {
public:
  bool parse(const std::string& text);

  bool has(const std::string& key) const { return members.count(key) != 0; }

  bool get_double(const std::string& key, double& out) const;
  bool get_bool(const std::string& key, bool& out) const;
  bool get_string(const std::string& key, std::string& out) const;

private:
  std::map<std::string, std::string> members;  // value stored as raw token
};

} // namespace agent
