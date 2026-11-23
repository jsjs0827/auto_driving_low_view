#pragma once
#include "json_util.hpp"
#include <array>
namespace perception_compare::detail {
struct Vertex {
  std::array<float, 3> xyz{};
  std::array<unsigned char, 3> rgb = {180, 180, 180};
};
struct Ply {
  std::vector<Vertex> vertices;
  std::vector<std::string> properties;
  std::string encoding;
  bool has_rgb = false;
};
Ply readPly(const fs::path &path);
} // namespace perception_compare::detail
