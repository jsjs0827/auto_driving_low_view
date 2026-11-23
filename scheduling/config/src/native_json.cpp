#include "native_config_detail.hpp"
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <stdexcept>
#if !defined(_WIN32)
#include <unistd.h>
#endif
namespace v3 {
Json readJson(const Path &path) { return r33::json::load(path.u8string()); }
Json readJson(const std::string &path) { return readJson(std::filesystem::u8path(path)); }
namespace {
void encode(std::ostream &out, const Json &value, int indentation, int depth) {
  using K = Json::Kind;
  auto newline = [&](int d) {
    if (indentation > 0)
      out << '\n' << std::string(static_cast<std::size_t>(d * indentation), ' ');
  };
  switch (value.kind) {
  case K::Null:
    out << "null";
    break;
  case K::Boolean:
    out << (value.boolean ? "true" : "false");
    break;
  case K::Number:
    r33::json::number(value);
    out << value.text;
    break;
  case K::String:
    out << r33::json::quote(value.text);
    break;
  case K::Array:
    out << '[';
    for (std::size_t i = 0; i < value.array.size(); ++i) {
      if (i)
        out << ',';
      newline(depth + 1);
      encode(out, value.array[i], indentation, depth + 1);
    }
    if (!value.array.empty())
      newline(depth);
    out << ']';
    break;
  case K::Object:
    out << '{';
    {
      std::size_t i = 0;
      for (const auto &item : value.object) {
        if (i++)
          out << ',';
        newline(depth + 1);
        out << r33::json::quote(item.first) << ":" << (indentation > 0 ? " " : "");
        encode(out, item.second, indentation, depth + 1);
      }
    }
    if (!value.object.empty())
      newline(depth);
    out << '}';
    break;
  }
}
std::string expand(const std::string &input) {
  std::string result;
  for (std::size_t i = 0; i < input.size();) {
    if (input[i] == '$') {
      std::size_t end = i + 1;
      std::string name;
      if (end < input.size() && input[end] == '{') {
        end = input.find('}', end + 1);
        if (end == std::string::npos)
          throw std::invalid_argument("Unclosed environment variable: " + input);
        name = input.substr(i + 2, end - i - 2);
        ++end;
      } else {
        while (end < input.size() &&
               (std::isalnum(static_cast<unsigned char>(input[end])) || input[end] == '_'))
          ++end;
        name = input.substr(i + 1, end - i - 1);
      }
      if (name.empty())
        throw std::invalid_argument("Unresolved environment variable in path: " + input);
      const char *value = std::getenv(name.c_str());
      if (!value || !*value)
        throw std::invalid_argument("Missing environment variable: " + name);
      result += value;
      i = end;
    } else if (input[i] == '%' && input.find('%', i + 1) != std::string::npos) {
      const auto end = input.find('%', i + 1);
      const auto name = input.substr(i + 1, end - i - 1);
      const char *value = std::getenv(name.c_str());
      if (!value || !*value)
        throw std::invalid_argument("Missing environment variable: " + name);
      result += value;
      i = end + 1;
    } else
      result += input[i++];
  }
  if (!result.empty() && result[0] == '~' &&
      (result.size() == 1 || result[1] == '/' || result[1] == '\\')) {
    const char *home = std::getenv("HOME");
    if (!home)
      home = std::getenv("USERPROFILE");
    if (!home)
      throw std::invalid_argument("Cannot expand home directory");
    result = std::string(home) + result.substr(1);
  }
  if (std::regex_search(result, std::regex(R"(\$\{?[A-Za-z_]|%[A-Za-z_][A-Za-z0-9_]*%)")))
    throw std::invalid_argument("Unresolved environment variable in path: " + result);
  return result;
}
} // namespace
std::string jsonText(const Json &value, int indentation) {
  std::ostringstream out;
  encode(out, value, indentation, 0);
  return out.str();
}
void writeJson(const Path &path, const Json &value) {
  if (std::filesystem::exists(path))
    throw std::invalid_argument("Refusing to overwrite: " + path.u8string());
  std::ofstream out(path, std::ios::binary);
  if (!out)
    throw std::runtime_error("Cannot write: " + path.u8string());
  out << jsonText(value) << '\n';
  if (!out)
    throw std::runtime_error("Cannot complete write: " + path.u8string());
}
std::string resolvePath(const Json &value, const Path &base, const std::string &name,
                        const std::string &kind, bool allow_empty) {
  auto raw = detail::text(value, name, allow_empty);
  if (raw.empty() && allow_empty)
    return {};
  auto path = std::filesystem::u8path(expand(raw));
  if (path.is_relative())
    path = base / path;
  path = std::filesystem::absolute(path).lexically_normal();
  if (kind == "file" && !std::filesystem::is_regular_file(path))
    throw std::invalid_argument(name + " file does not exist: " + path.u8string());
  if (kind == "directory" && !std::filesystem::is_directory(path))
    throw std::invalid_argument(name + " directory does not exist: " + path.u8string());
  return path.u8string();
}
int launch(const std::vector<std::string> &arguments,
           const std::map<std::string, std::string> &environment) {
  if (arguments.empty())
    throw std::invalid_argument("No executable selected");
#if defined(_WIN32)
  (void)environment;
  throw std::runtime_error(
      "Launching Linux/ROS binaries is unavailable on Windows; use --prepare-only or WSL");
#else
  for (const auto &item : environment)
    if (setenv(item.first.c_str(), item.second.c_str(), 1))
      throw std::runtime_error("Cannot set environment: " + item.first);
  std::vector<char *> argv;
  for (const auto &item : arguments)
    argv.push_back(const_cast<char *>(item.c_str()));
  argv.push_back(nullptr);
  execv(argv[0], argv.data());
  throw std::runtime_error("Cannot execute " + arguments[0] + ": " + std::strerror(errno));
#endif
}
int launch(const PreparedPipeline &pipeline) { return launch(pipeline.argv, pipeline.environment); }
namespace detail {
Json object() {
  Json j;
  j.kind = Json::Kind::Object;
  return j;
}
Json array() {
  Json j;
  j.kind = Json::Kind::Array;
  return j;
}
Json str(const std::string &v) {
  Json j;
  j.kind = Json::Kind::String;
  j.text = v;
  return j;
}
Json num(double v) {
  if (!std::isfinite(v))
    throw std::invalid_argument("Finite number required");
  Json j;
  j.kind = Json::Kind::Number;
  std::ostringstream s;
  s << std::setprecision(17) << v;
  j.text = s.str();
  return j;
}
Json integer(int v) {
  Json j;
  j.kind = Json::Kind::Number;
  j.text = std::to_string(v);
  return j;
}
Json boolean(bool v) {
  Json j;
  j.kind = Json::Kind::Boolean;
  j.boolean = v;
  return j;
}
bool has(const Json &v, const std::string &k) { return v.object.count(k) != 0; }
bool explicitValue(const Json &v, const std::string &k) {
  return has(v, k) && v.object.at(k).kind != Json::Kind::Null;
}
const Json &get(const Json &v, const std::string &k) {
  auto i = v.object.find(k);
  if (i == v.object.end())
    throw std::invalid_argument("Missing configuration field: " + k);
  return i->second;
}
Json get(const Json &v, const std::string &k, Json d) {
  return has(v, k) ? v.object.at(k) : std::move(d);
}
void keys(const Json &v, const std::set<std::string> &allowed, const std::string &name) {
  if (v.kind != Json::Kind::Object)
    throw std::invalid_argument(name + " must be an object");
  for (const auto &item : v.object) {
    if (item.first == "_help") {
      if (item.second.kind != Json::Kind::String && item.second.kind != Json::Kind::Object)
        throw std::invalid_argument(name + "._help must be a string or object");
    } else if (!allowed.count(item.first))
      throw std::invalid_argument(name + " has unknown field: " + item.first);
  }
}
void keys(const Json &v, std::initializer_list<const char *> allowed, const std::string &name) {
  std::set<std::string> s;
  for (auto k : allowed)
    s.insert(k);
  keys(v, s, name);
}
std::string text(const Json &v, const std::string &name, bool empty) {
  if (v.kind != Json::Kind::String ||
      (!empty && v.text.find_first_not_of(" \t") == std::string::npos) ||
      v.text.find_first_of("\r\n") != std::string::npos || v.text.find('\0') != std::string::npos)
    throw std::invalid_argument(name + " must be a single-line string" +
                                (empty ? "" : " (nonempty)"));
  return v.text;
}
std::string choice(const Json &v, std::initializer_list<const char *> allowed,
                   const std::string &name) {
  auto s = text(v, name);
  for (auto a : allowed)
    if (s == a)
      return s;
  throw std::invalid_argument(name + " has unsupported value: " + s);
}
bool flag(const Json &v, const std::string &name) {
  if (v.kind != Json::Kind::Boolean)
    throw std::invalid_argument(name + " must be boolean");
  return v.boolean;
}
int count(const Json &v, const std::string &name, int low, int high) {
  try {
    if (v.kind != Json::Kind::Number || v.text.find_first_of(".eE") != std::string::npos)
      throw std::invalid_argument("integer representation required");
    return r33::json::integer(v, low, high);
  } catch (const std::exception &) {
    throw std::invalid_argument(name + " must be an integer in [" + std::to_string(low) + "," +
                                std::to_string(high) + "]");
  }
}
double number(const Json &v, const std::string &name, double low, double high, bool exclusive) {
  double d;
  try {
    d = r33::json::number(v);
  } catch (const std::exception &) {
    throw std::invalid_argument(name + " must be finite numeric");
  }
  if ((exclusive ? d <= low : d < low) || d > high)
    throw std::invalid_argument(name + " is out of range");
  return d;
}
std::string topic(const Json &v, const std::string &name) {
  auto s = text(v, name);
  if (!std::regex_match(s, std::regex("/[A-Za-z_][A-Za-z0-9_/]*")) ||
      s.find("//") != std::string::npos || s.back() == '/')
    throw std::invalid_argument(name + " must be an absolute ROS topic");
  return s;
}
void merge(Json &target, const Json &source) {
  if (target.kind != Json::Kind::Object)
    target = object();
  for (const auto &item : source.object)
    target.object[item.first] = item.second;
}
Path projectRoot(const Path &config) {
  if (const char *env = std::getenv("PERCEPTION_PROJECT_ROOT"))
    return std::filesystem::absolute(std::filesystem::u8path(env));
  for (Path p = std::filesystem::absolute(config).parent_path(); !p.empty();) {
    if (std::filesystem::is_directory(p / "features") &&
        std::filesystem::is_directory(p / "scheduling"))
      return p;
    auto parent = p.parent_path();
    if (parent == p)
      break;
    p = parent;
  }
#ifdef PERCEPTION_NATIVE_PROJECT_ROOT
  return std::filesystem::path(PERCEPTION_NATIVE_PROJECT_ROOT);
#else
  throw std::invalid_argument("Cannot find project source root; set PERCEPTION_PROJECT_ROOT");
#endif
}
} // namespace detail
} // namespace v3
