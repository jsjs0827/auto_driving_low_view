#pragma once
#include "json.hpp"
#include <algorithm>
#include <iomanip>
#include <optional>

namespace perception_compare::detail {
namespace fs = std::filesystem;
using Json = r33::json::JsonValue;
inline Json object() {
  Json j;
  j.kind = Json::Kind::Object;
  return j;
}
inline Json array() {
  Json j;
  j.kind = Json::Kind::Array;
  return j;
}
inline Json str(const std::string &s) {
  Json j;
  j.kind = Json::Kind::String;
  j.text = s;
  return j;
}
inline Json boolean(bool b) {
  Json j;
  j.kind = Json::Kind::Boolean;
  j.boolean = b;
  return j;
}
inline Json number(double n) {
  if (!std::isfinite(n))
    return {};
  Json j;
  j.kind = Json::Kind::Number;
  std::ostringstream s;
  s << std::setprecision(17) << n;
  j.text = s.str();
  return j;
}
inline Json integer(std::uint64_t n) {
  Json j;
  j.kind = Json::Kind::Number;
  j.text = std::to_string(n);
  return j;
}
inline Json obj(std::initializer_list<std::pair<const std::string, Json>> fields) {
  auto j = object();
  for (const auto &field : fields)
    j.object.emplace(field);
  return j;
}
inline const Json &field(const Json &j, const std::string &key) {
  static const Json null;
  const auto i = j.object.find(key);
  return i == j.object.end() ? null : i->second;
}
inline std::string textValue(const Json &j, const std::string &fallback = {}) {
  return j.kind == Json::Kind::String ? j.text : fallback;
}
inline std::optional<double> numeric(const Json &j) {
  if (j.kind != Json::Kind::Number)
    return {};
  try {
    std::size_t used = 0;
    const auto n = std::stod(j.text, &used);
    if (used == j.text.size() && std::isfinite(n) && n >= 0)
      return n;
  } catch (const std::exception &) {
  }
  return {};
}
inline std::uint64_t uint64(const Json &j, const std::string &name, bool allow_zero = true) {
  if (j.kind != Json::Kind::Number || j.text.empty() ||
      j.text.find_first_not_of("0123456789") != std::string::npos)
    throw std::runtime_error(name + " must be an unsigned JSON integer");
  std::size_t used = 0;
  const auto n = std::stoull(j.text, &used);
  if (used != j.text.size() || (!allow_zero && !n))
    throw std::runtime_error(name + " must be positive");
  return n;
}
inline std::string readText(const fs::path &path) {
  std::ifstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error("Cannot read " + path.u8string());
  std::ostringstream s;
  s << f.rdbuf();
  if (f.bad())
    throw std::runtime_error("Read failed " + path.u8string());
  auto text = s.str();
  if (text.compare(0, 3, "\xef\xbb\xbf") == 0)
    text.erase(0, 3);
  return text;
}
inline Json parse(const std::string &s, const std::string &where) {
  return r33::json::JsonParser(s, where).parse();
}
inline Json readJson(const fs::path &p) { return parse(readText(p), p.u8string()); }
inline std::string quote(const std::string &s) {
  std::ostringstream out;
  out << '"';
  for (unsigned char c : s) {
    if (c == '"')
      out << "\\\"";
    else if (c == '\\')
      out << "\\\\";
    else if (c < 32 || c == '<' || c == '>' || c == '&')
      out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(c) << std::dec;
    else
      out << char(c);
  }
  out << '"';
  return out.str();
}
inline std::string dump(const Json &j) {
  switch (j.kind) {
  case Json::Kind::Null:
    return "null";
  case Json::Kind::Boolean:
    return j.boolean ? "true" : "false";
  case Json::Kind::Number:
    return j.text;
  case Json::Kind::String:
    return quote(j.text);
  case Json::Kind::Array: {
    std::string s = "[";
    for (const auto &v : j.array) {
      if (s.size() > 1)
        s += ',';
      s += dump(v);
    }
    return s + "]";
  }
  case Json::Kind::Object: {
    std::string s = "{";
    for (const auto &[k, v] : j.object) {
      if (s.size() > 1)
        s += ',';
      s += quote(k) + ":" + dump(v);
    }
    return s + "}";
  }
  }
  throw std::runtime_error("Invalid JSON kind");
}
inline void writeText(const fs::path &p, const std::string &s) {
  std::ofstream f(p, std::ios::binary | std::ios::trunc);
  if (!f)
    throw std::runtime_error("Cannot write " + p.u8string());
  f << s;
  f.close();
  if (!f)
    throw std::runtime_error("Write failed " + p.u8string());
}
inline void writeJson(const fs::path &p, const Json &j) { writeText(p, dump(j) + "\n"); }
inline fs::path regular(const fs::path &p) {
  const auto c = fs::canonical(p);
  if (!fs::is_regular_file(c))
    throw std::runtime_error("Expected file: " + c.u8string());
  if (c.u8string().find_first_of("\t\r\n") != std::string::npos)
    throw std::runtime_error("Control character in file path");
  return c;
}
inline bool contained(const fs::path &root, const fs::path &p) {
  auto a = root.begin(), b = p.begin();
  for (; a != root.end(); ++a, ++b)
    if (b == p.end() || *a != *b)
      return false;
  return true;
}
std::string sha256(const fs::path &path);
Json actualModels(const fs::path &root, const Json &run, const Json &full);
Json buildViewer(const fs::path &root, const Json &pairs, const Json &summary,
                 std::size_t max_points);
} // namespace perception_compare::detail
