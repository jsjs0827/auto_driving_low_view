#pragma once
#include "json.hpp"
#include <filesystem>
#include <stdexcept>
#include <string>

namespace v3 {
// Match the recharge backend's accepted standalone/wrapped/start-config forms.
// Call only for an active recharge stage that otherwise has no mapping input;
// disabled features must not introduce file or calibration dependencies.
inline bool rechargeUsesFrameIntrinsics(const r33::json::JsonValue &full,
                                        const std::filesystem::path &fullBase) {
  using namespace r33::json;
  const auto &recharge = required(full, "recharge");
  if (const auto *enabled = optional(recharge, "enabled"))
    if (!boolean(*enabled))
      return false;
  auto config = std::filesystem::u8path(string(required(recharge, "config")));
  if (!config.is_absolute())
    config = fullBase / config;
  for (unsigned depth = 0; depth != 4; ++depth) {
    const auto document = load(config.u8string());
    if (document.kind != JsonValue::Kind::Object)
      throw std::invalid_argument("Recharge configuration must be a JSON object");
    const auto *wrapped = optional(document, "recharge_mark");
    const auto &mark = wrapped && wrapped->kind == JsonValue::Kind::Object ? *wrapped : document;
    const auto isMark = [&](const JsonValue &value) {
      return optional(value, "enabled") || optional(value, "publish_enabled") ||
             optional(value, "roi") || optional(value, "aruco");
    };
    if (wrapped && wrapped->kind == JsonValue::Kind::Object && optional(mark, "config_file") &&
        !isMark(mark)) {
      auto next = std::filesystem::u8path(string(required(mark, "config_file")));
      config = next.is_absolute() ? next : config.parent_path() / next;
      continue;
    }
    if (!isMark(mark) && !isMark(document) &&
        (optional(document, "sdk_so_dir") || optional(document, "dnn") ||
         optional(document, "map"))) {
      config = config.parent_path() / "recharge_mark.json";
      continue;
    }
    const auto *source = optional(mark, "intrinsics_source");
    // The retained backend defaults missing/non-string settings to dataset.
    return source && source->kind == JsonValue::Kind::String && source->text != "dataset";
  }
  throw std::invalid_argument("Recharge configuration references are cyclic or too deep");
}
inline bool rechargeUsesFrameIntrinsics(const std::filesystem::path &fullPath) {
  return rechargeUsesFrameIntrinsics(r33::json::load(fullPath.u8string()), fullPath.parent_path());
}
// One calibration contract is shared by the native config frontend and the ROS
// offline source. The file is either the seven fields below or recorder metadata
// with those fields in its `intrinsics` object. Values describe source pixels;
// fx/fy are pixels and bxf is pixel * millimetres. No implicit scaling is applied.
inline r33::json::JsonValue loadIntrinsicsFile(const std::filesystem::path &path,
                                               bool require640544 = false) {
  using namespace r33::json;
  try {
    const auto document = load(path.u8string());
    const auto *nested = optional(document, "intrinsics");
    const auto &intrinsics = nested ? *nested : document;
    fields(intrinsics, {"width", "height", "fx", "fy", "cx", "cy", "bxf", "_help"});
    if (const auto *help = optional(intrinsics, "_help"))
      if (help->kind != JsonValue::Kind::String && help->kind != JsonValue::Kind::Object)
        throw std::invalid_argument("intrinsics._help must be a string or object");
    const int width = integer(required(intrinsics, "width"), 2, 4096);
    const int height = integer(required(intrinsics, "height"), 2, 4096);
    if ((width & 1) || (height & 1))
      throw std::invalid_argument("Intrinsics source dimensions must be even");
    if (require640544 && (width != 640 || height != 544))
      throw std::invalid_argument("Intrinsics file must describe 640x544 source images");
    for (auto key : {"fx", "fy", "bxf"})
      if (number(required(intrinsics, key)) <= 0)
        throw std::invalid_argument(std::string("intrinsics.") + key + " must be positive");
    const double cx = number(required(intrinsics, "cx"));
    const double cy = number(required(intrinsics, "cy"));
    if (cx < 0 || cx >= width || cy < 0 || cy >= height)
      throw std::invalid_argument("Intrinsics principal point is outside source image");
    JsonValue resolved;
    resolved.kind = JsonValue::Kind::Object;
    for (auto key : {"width", "height", "fx", "fy", "cx", "cy", "bxf"})
      resolved.object[key] = required(intrinsics, key);
    return resolved;
  } catch (const std::exception &error) {
    throw std::invalid_argument("Invalid intrinsics file '" + path.u8string() +
                                "': " + error.what());
  }
}
} // namespace v3
