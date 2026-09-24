#include "json.h"

#include <cstdlib>

namespace agent {

namespace {

void skip_space(const std::string& s, size_t& i) {
  while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) ++i;
}

// Parse a JSON string literal starting at s[i] == '"'. Returns false on error.
bool parse_string(const std::string& s, size_t& i, std::string& out) {
  if (i >= s.size() || s[i] != '"') return false;
  ++i;
  out.clear();
  while (i < s.size()) {
    char c = s[i++];
    if (c == '"') return true;
    if (c == '\\') {
      if (i >= s.size()) return false;
      char e = s[i++];
      switch (e) {
        case '"':  out += '"';  break;
        case '\\': out += '\\'; break;
        case '/':  out += '/';  break;
        case 'n':  out += '\n'; break;
        case 'r':  out += '\r'; break;
        case 't':  out += '\t'; break;
        case 'b':  out += '\b'; break;
        case 'f':  out += '\f'; break;
        case 'u': {
          if (i + 4 > s.size()) return false;
          // Only the ASCII range is preserved; the agent interface has no
          // need for full UTF-16 surrogate decoding.
          unsigned code = (unsigned)strtoul(s.substr(i, 4).c_str(), nullptr, 16);
          i += 4;
          if (code < 0x80) out += char(code); else out += '?';
        } break;
        default: return false;
      }
    }
    else out += c;
  }
  return false;  // unterminated
}

// Capture a scalar token (number, true, false, null) verbatim.
bool parse_scalar(const std::string& s, size_t& i, std::string& out) {
  size_t start = i;
  while (i < s.size() && s[i] != ',' && s[i] != '}') ++i;
  out = s.substr(start, i - start);
  // trim trailing whitespace
  while (!out.empty() && (out.back() == ' ' || out.back() == '\t' || out.back() == '\n' || out.back() == '\r'))
    out.pop_back();
  return !out.empty();
}

} // namespace

bool JsonValue::parse(const std::string& text) {
  members.clear();

  size_t i = 0;
  skip_space(text, i);
  if (i >= text.size() || text[i] != '{') return false;
  ++i;

  skip_space(text, i);
  if (i < text.size() && text[i] == '}') return true;  // empty object

  while (i < text.size()) {
    skip_space(text, i);

    std::string key;
    if (!parse_string(text, i, key)) return false;

    skip_space(text, i);
    if (i >= text.size() || text[i] != ':') return false;
    ++i;
    skip_space(text, i);

    if (i >= text.size()) return false;

    std::string value;
    if (text[i] == '"') {
      std::string unescaped;
      if (!parse_string(text, i, unescaped)) return false;
      value = "\"" + unescaped;  // leading quote marks this as a string token
    }
    else if (text[i] == '{' || text[i] == '[') {
      return false;  // nested structures unsupported by design
    }
    else if (!parse_scalar(text, i, value)) return false;

    members[key] = value;

    skip_space(text, i);
    if (i >= text.size()) return false;
    if (text[i] == ',') { ++i; continue; }
    if (text[i] == '}') return true;
    return false;
  }

  return false;
}

bool JsonValue::get_double(const std::string& key, double& out) const {
  auto it = members.find(key);
  if (it == members.end() || it->second.empty()) return false;
  if (it->second[0] == '"') return false;  // a string, not a number

  char* end = nullptr;
  double v = strtod(it->second.c_str(), &end);
  if (end == it->second.c_str()) return false;
  out = v;
  return true;
}

bool JsonValue::get_bool(const std::string& key, bool& out) const {
  auto it = members.find(key);
  if (it == members.end()) return false;
  if (it->second == "true")  { out = true;  return true; }
  if (it->second == "false") { out = false; return true; }
  return false;
}

bool JsonValue::get_string(const std::string& key, std::string& out) const {
  auto it = members.find(key);
  if (it == members.end() || it->second.empty()) return false;
  if (it->second[0] != '"') return false;
  out = it->second.substr(1);
  return true;
}

} // namespace agent
