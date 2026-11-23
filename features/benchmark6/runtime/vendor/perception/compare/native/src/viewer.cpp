#include "ply.hpp"
#include "viewer_template.hpp"
#include <cstring>
#include <set>

namespace perception_compare::detail {
namespace {
struct Scalar {
  std::string type, name;
  std::size_t bytes = 0;
};
std::size_t typeBytes(const std::string &t) {
  if (t == "char" || t == "int8" || t == "uchar" || t == "uint8")
    return 1;
  if (t == "short" || t == "int16" || t == "ushort" || t == "uint16")
    return 2;
  if (t == "int" || t == "int32" || t == "uint" || t == "uint32" || t == "float" || t == "float32")
    return 4;
  if (t == "double" || t == "float64")
    return 8;
  throw std::runtime_error("Unsupported scalar PLY type " + t);
}
double binaryScalar(std::istream &f, const Scalar &p, bool big) {
  std::uint64_t bits = 0;
  for (std::size_t i = 0; i < p.bytes; ++i) {
    const int c = f.get();
    if (c == EOF)
      throw std::runtime_error("Truncated binary PLY");
    bits |= std::uint64_t(c) << (8 * (big ? p.bytes - 1 - i : i));
  }
  if (p.type == "float" || p.type == "float32") {
    const auto b = std::uint32_t(bits);
    float n;
    std::memcpy(&n, &b, 4);
    return n;
  }
  if (p.type == "double" || p.type == "float64") {
    double n;
    std::memcpy(&n, &bits, 8);
    return n;
  }
  if (p.type == "char" || p.type == "int8")
    return std::int8_t(bits);
  if (p.type == "short" || p.type == "int16")
    return std::int16_t(bits);
  if (p.type == "int" || p.type == "int32")
    return std::int32_t(bits);
  return double(bits);
}
std::string base64(const std::vector<unsigned char> &bytes) {
  constexpr const char *chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((bytes.size() + 2) / 3 * 4);
  for (std::size_t i = 0; i < bytes.size(); i += 3) {
    const unsigned a = bytes[i], b = i + 1 < bytes.size() ? bytes[i + 1] : 0,
                   c = i + 2 < bytes.size() ? bytes[i + 2] : 0;
    out += chars[a >> 2];
    out += chars[((a & 3) << 4) | (b >> 4)];
    out += i + 1 < bytes.size() ? chars[((b & 15) << 2) | (c >> 6)] : '=';
    out += i + 2 < bytes.size() ? chars[c & 63] : '=';
  }
  return out;
}
Json triple(const std::array<double, 3> &xyz) {
  auto j = array();
  for (double n : xyz)
    j.array.push_back(number(n));
  return j;
}
Json preview(const std::string &file, std::size_t max_points) {
  if (file.empty())
    return obj({{"status", str("missing")}, {"message", str("未保存此点云")}});
  if (!fs::is_regular_file(fs::u8path(file)))
    return obj({{"status", str("missing")}, {"message", str("PLY文件不存在")}});
  try {
    const auto ply = readPly(fs::u8path(file));
    std::vector<std::size_t> finite;
    bool black = !ply.vertices.empty();
    std::array<double, 3> low = {INFINITY, INFINITY, INFINITY},
                          high = {-INFINITY, -INFINITY, -INFINITY};
    for (std::size_t i = 0; i < ply.vertices.size(); ++i) {
      const auto &v = ply.vertices[i];
      for (auto c : v.rgb)
        if (c)
          black = false;
      if (std::isfinite(v.xyz[0]) && std::isfinite(v.xyz[1]) && std::isfinite(v.xyz[2])) {
        finite.push_back(i);
        for (int d = 0; d < 3; ++d) {
          low[d] = std::min(low[d], double(v.xyz[d]));
          high[d] = std::max(high[d], double(v.xyz[d]));
        }
      }
    }
    const auto count = std::min(max_points, finite.size());
    std::vector<unsigned char> xyz, rgb;
    xyz.reserve(count * 12);
    rgb.reserve(count * 3);
    for (std::size_t i = 0; i < count; ++i) {
      const auto rank = count > 1 ? i * (finite.size() - 1) / (count - 1) : 0;
      const auto &v = ply.vertices[finite[rank]];
      for (float n : v.xyz) {
        std::uint32_t bits;
        std::memcpy(&bits, &n, 4);
        for (int b = 0; b < 4; ++b)
          xyz.push_back((bits >> (8 * b)) & 255);
      }
      rgb.insert(rgb.end(), v.rgb.begin(), v.rgb.end());
    }
    auto bounds = array();
    if (!finite.empty()) {
      bounds.array.push_back(triple(low));
      bounds.array.push_back(triple(high));
    }
    auto properties = array();
    for (const auto &p : ply.properties)
      properties.array.push_back(str(p));
    return obj({{"status", str("ok")},
                {"file", str(fs::u8path(file).filename().u8string())},
                {"total_points", integer(ply.vertices.size())},
                {"finite_points", integer(finite.size())},
                {"displayed_points", integer(count)},
                {"invalid_points", integer(ply.vertices.size() - finite.size())},
                {"bounds", finite.empty() ? Json{} : bounds},
                {"properties", properties},
                {"has_rgb", boolean(ply.has_rgb)},
                {"encoding", str(ply.encoding)},
                {"rgb_all_zero", boolean(black)},
                {"xyz", str(base64(xyz))},
                {"rgb", str(base64(rgb))}});
  } catch (const std::exception &e) {
    return obj(
        {{"status", str("error")}, {"message", str(std::string("PLY无法预览：") + e.what())}});
  }
}
} // namespace

Ply readPly(const fs::path &path) {
  std::ifstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error("Cannot read PLY " + path.u8string());
  std::string line;
  std::getline(f, line);
  if (!line.empty() && line.back() == '\r')
    line.pop_back();
  if (line != "ply")
    throw std::runtime_error("Not a PLY file");
  Ply out;
  std::vector<Scalar> properties;
  std::uint64_t count = 0, header = 4;
  bool vertex = false, found = false, ended = false;
  std::set<std::string> names;
  while (std::getline(f, line)) {
    header += line.size() + 1;
    if (header > 1048576)
      throw std::runtime_error("Excessive PLY header");
    std::istringstream s(line);
    std::string key;
    s >> key;
    if (key == "end_header") {
      ended = true;
      break;
    }
    if (key == "format") {
      std::string version;
      s >> out.encoding >> version;
      if (version != "1.0")
        throw std::runtime_error("Unsupported PLY version");
    } else if (key == "element") {
      std::string name, n;
      s >> name >> n;
      if (n.empty() || n.find_first_not_of("0123456789") != std::string::npos)
        throw std::runtime_error("Invalid PLY element count");
      const auto number = std::stoull(n);
      if (!found && name != "vertex" && number)
        throw std::runtime_error("Nonempty PLY element precedes vertex data");
      vertex = name == "vertex";
      if (vertex) {
        if (found)
          throw std::runtime_error("Duplicate PLY vertex element");
        count = number;
        found = true;
      }
    } else if (key == "property" && vertex) {
      Scalar p;
      s >> p.type >> p.name;
      if (p.type == "list")
        throw std::runtime_error("List vertex properties unsupported");
      p.bytes = typeBytes(p.type);
      if (p.name.empty() || !names.insert(p.name).second)
        throw std::runtime_error("Invalid/duplicate PLY property");
      properties.push_back(p);
      out.properties.push_back(p.name);
    }
  }
  if (!ended || !found || !names.count("x") || !names.count("y") || !names.count("z"))
    throw std::runtime_error("PLY requires vertex x/y/z");
  if (out.encoding != "ascii" && out.encoding != "binary_little_endian" &&
      out.encoding != "binary_big_endian")
    throw std::runtime_error("Unsupported PLY encoding");
  std::size_t rowbytes = 0;
  for (const auto &p : properties)
    rowbytes += p.bytes;
  const auto start = f.tellg();
  if (start < 0)
    throw std::runtime_error("Invalid PLY stream position");
  const auto remaining = fs::file_size(path) - std::uint64_t(start);
  const auto minimum =
      out.encoding == "ascii" ? std::max<std::size_t>(1, properties.size() * 2 - 1) : rowbytes;
  if (count > 50000000 || (count && count > (remaining + 1) / minimum))
    throw std::runtime_error("Truncated or excessive PLY vertices");
  out.has_rgb = (names.count("red") && names.count("green") && names.count("blue")) ||
                (names.count("r") && names.count("g") && names.count("b"));
  out.vertices.reserve(std::size_t(count));
  for (std::uint64_t i = 0; i < count; ++i) {
    Vertex v;
    std::istringstream ascii;
    if (out.encoding == "ascii") {
      if (!std::getline(f, line))
        throw std::runtime_error("Truncated ASCII PLY");
      ascii.str(line);
    }
    for (const auto &p : properties) {
      double value = 0;
      if (out.encoding == "ascii") {
        std::string token;
        if (!(ascii >> token))
          throw std::runtime_error("Missing PLY scalar");
        std::size_t used = 0;
        value = std::stod(token, &used);
        if (used != token.size())
          throw std::runtime_error("Invalid PLY scalar");
      } else
        value = binaryScalar(f, p, out.encoding == "binary_big_endian");
      if (p.name == "x")
        v.xyz[0] = float(value);
      else if (p.name == "y")
        v.xyz[1] = float(value);
      else if (p.name == "z")
        v.xyz[2] = float(value);
      else if (p.name == "red" || p.name == "r")
        v.rgb[0] = std::isfinite(value) ? std::clamp(value, 0.0, 255.0) : 0;
      else if (p.name == "green" || p.name == "g")
        v.rgb[1] = std::isfinite(value) ? std::clamp(value, 0.0, 255.0) : 0;
      else if (p.name == "blue" || p.name == "b")
        v.rgb[2] = std::isfinite(value) ? std::clamp(value, 0.0, 255.0) : 0;
    }
    if (out.encoding == "ascii") {
      std::string extra;
      if (ascii >> extra)
        throw std::runtime_error("Extra ASCII PLY scalar");
    }
    out.vertices.push_back(v);
  }
  return out;
}
Json buildViewer(const fs::path &root, const Json &pairs, const Json &summary,
                 std::size_t max_points) {
  auto prepared = array();
  std::uint64_t errors = 0;
  for (const auto &pair : pairs.array) {
    auto item = obj(
        {{"stem", field(pair, "stem")}, {"repeat", field(pair, "repeat")}, {"clouds", object()}});
    for (const auto &[type, sides] : field(pair, "clouds").object) {
      auto entry = object();
      std::array<double, 3> low = {INFINITY, INFINITY, INFINITY},
                            high = {-INFINITY, -INFINITY, -INFINITY};
      bool has_bounds = false;
      for (const auto *side : {"original", "stereo"}) {
        auto cloud = preview(textValue(field(sides, side)), max_points);
        if (textValue(field(cloud, "status")) == "error")
          ++errors;
        const auto &bounds = field(cloud, "bounds");
        if (bounds.array.size() == 2) {
          has_bounds = true;
          for (int d = 0; d < 3; ++d) {
            low[d] = std::min(low[d], std::stod(bounds.array[0].array[d].text));
            high[d] = std::max(high[d], std::stod(bounds.array[1].array[d].text));
          }
        }
        entry.object[side] = std::move(cloud);
      }
      std::array<double, 3> center = {0, 0, 0};
      double radius = 1;
      if (has_bounds) {
        double square = 0;
        for (int d = 0; d < 3; ++d) {
          center[d] = (low[d] + high[d]) / 2;
          square += (high[d] - low[d]) * (high[d] - low[d]);
        }
        radius = std::max(std::sqrt(square) / 2, .001);
      }
      entry.object["center"] = triple(center);
      entry.object["radius"] = number(radius);
      item.object["clouds"].object[type] = std::move(entry);
    }
    prepared.array.push_back(std::move(item));
  }
  const auto data =
      obj({{"pairs", prepared}, {"summary", summary}, {"max_points", integer(max_points)}});
  std::string html = viewer_template;
  const auto at = html.find("__DATA__");
  if (at == std::string::npos)
    throw std::runtime_error("Missing viewer template placeholder");
  html.replace(at, 8, dump(data));
  writeText(root / "index.html", html);
  return obj({{"path", str((root / "index.html").u8string())},
              {"pair_count", integer(prepared.array.size())},
              {"max_points_per_cloud", integer(max_points)},
              {"cloud_errors", integer(errors)}});
}
} // namespace perception_compare::detail
