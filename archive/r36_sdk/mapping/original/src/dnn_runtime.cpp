/**
 * @file dnn_runtime.cpp
 * @brief obs/detection：仿 obs/segmentation 接入 Horizon 检测，发布检测结果与匹配点云
 */
#include "percep_demo/percep_demo.h"
#include "percep_demo/pcl_common.hpp"

#if defined(PERCEP_DNN_ENABLE)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <errno.h>
#include <exception>
#include <iostream>
#include <limits>
#include <new>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unordered_map>
#include <utility>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "third_party/stb_image_write.h"

using namespace std;
using namespace rapidjson;
using namespace cv;

namespace ai_node_percep_demo {

namespace {

static const uint8_t kDetPalette[12][3] = {
    {255, 56, 56},   {255, 157, 151}, {255, 112, 31}, {255, 178, 29},
    {207, 210, 49},  {72, 249, 10},   {0, 128, 255},  {128, 0, 255},
    {255, 0, 128},   {0, 255, 200},   {180, 180, 40}, {100, 200, 255},
};

void BlendSegMaskOnRgb(uint8_t* rgb, int w, int h,
                       const std::vector<uint8_t>& seg_map, float alpha,
                       const std::string& class_names_csv) {
  if (!rgb || w <= 0 || h <= 0 || seg_map.empty()) return;
  if (static_cast<int>(seg_map.size()) < w * h) return;
  const float a = std::max(0.0f, std::min(1.0f, alpha));
  const float ia = 1.f - a;
  const auto names = ParseSegClassNames(class_names_csv);
  for (int i = 0; i < w * h; ++i) {
    const int cls = seg_map[static_cast<size_t>(i)];
    if (cls < 0) continue;
    const std::string name =
        (cls < static_cast<int>(names.size())) ? names[static_cast<size_t>(cls)]
                                               : std::string();
    uint8_t cr = 0, cg = 0, cb = 0;
    SegClassVisRgb(cls, name, cr, cg, cb);
    uint8_t* p = rgb + static_cast<size_t>(i) * 3;
    p[0] = static_cast<uint8_t>(p[0] * ia + cr * a);
    p[1] = static_cast<uint8_t>(p[1] * ia + cg * a);
    p[2] = static_cast<uint8_t>(p[2] * ia + cb * a);
  }
}

inline uint8_t ClipU8(int v) {
  return static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v));
}

inline void Yuv601ToRgb(uint8_t y, uint8_t u, uint8_t v, uint8_t* r, uint8_t* g,
                        uint8_t* b) {
  const int c = static_cast<int>(y) - 16;
  const int d = static_cast<int>(u) - 128;
  const int e = static_cast<int>(v) - 128;
  const int c298 = 298 * c;
  *r = ClipU8((c298 + 409 * e + 128) >> 8);
  *g = ClipU8((c298 - 100 * d - 208 * e + 128) >> 8);
  *b = ClipU8((c298 + 516 * d + 128) >> 8);
}

constexpr std::size_t kMaxOverlayRgbBytes =
    2U * kMaxCameraPayloadBytes;
constexpr int kMaxOverlayDimension = 8192;

bool CheckedOverlayRgbBytes(int width, int height,
                            std::size_t* bytes) noexcept {
  if (width <= 0 || height <= 0 || width > kMaxOverlayDimension ||
      height > kMaxOverlayDimension || bytes == nullptr) {
    return false;
  }
  std::size_t pixels = 0;
  return checkedMultiply(static_cast<std::size_t>(width),
                         static_cast<std::size_t>(height), &pixels) &&
         checkedMultiply(pixels, 3U, bytes) &&
         *bytes <= kMaxOverlayRgbBytes;
}

bool Yuv420NearestToRgb24(const uint8_t* yuv, int src_w, int src_h,
                          int stride_y, bool src_is_nv12, int dst_w,
                          int dst_h, std::vector<uint8_t>* rgb) {
  if (!yuv || !rgb || src_w <= 0 || src_h <= 0 || dst_w <= 0 || dst_h <= 0 ||
      (src_w % 2) != 0 || (src_h % 2) != 0) {
    return false;
  }
  if (stride_y < src_w) stride_y = src_w;
  std::size_t rgb_bytes = 0;
  if (!CheckedOverlayRgbBytes(dst_w, dst_h, &rgb_bytes)) return false;
  try {
    rgb->resize(rgb_bytes);
  } catch (const std::bad_alloc&) {
    return false;
  }
  const uint8_t* y_plane = yuv;
  const uint8_t* chroma = yuv + static_cast<size_t>(stride_y) * src_h;
  const int chroma_stride = stride_y / 2;
  const uint8_t* u_plane = chroma;
  const uint8_t* v_plane =
      chroma + static_cast<size_t>(chroma_stride) * (src_h / 2);
  uint8_t* dst = rgb->data();
  for (int y = 0; y < dst_h; ++y) {
    const int sy = std::min(src_h - 1, y * src_h / dst_h);
    const uint8_t* yrow = y_plane + static_cast<size_t>(sy) * stride_y;
    const uint8_t* uvrow =
        chroma + static_cast<size_t>(sy / 2) * stride_y;
    const uint8_t* urow =
        u_plane + static_cast<size_t>(sy / 2) * chroma_stride;
    const uint8_t* vrow =
        v_plane + static_cast<size_t>(sy / 2) * chroma_stride;
    uint8_t* out = dst + static_cast<size_t>(y) * dst_w * 3;
    for (int x = 0; x < dst_w; ++x) {
      const int sx = std::min(src_w - 1, x * src_w / dst_w);
      const int chroma_x = sx / 2;
      const uint8_t u =
          src_is_nv12 ? uvrow[chroma_x * 2] : urow[chroma_x];
      const uint8_t v =
          src_is_nv12 ? uvrow[chroma_x * 2 + 1] : vrow[chroma_x];
      Yuv601ToRgb(yrow[sx], u, v, &out[x * 3], &out[x * 3 + 1],
                  &out[x * 3 + 2]);
    }
  }
  return true;
}

void ScaleDetsToOverlay(const std::vector<DnnDetection>& src, int src_w,
                        int src_h, int dst_w, int dst_h,
                        std::vector<DnnDetection>* dst) {
  dst->clear();
  if (src_w <= 0 || src_h <= 0) return;
  const float sx = static_cast<float>(dst_w) / static_cast<float>(src_w);
  const float sy = static_cast<float>(dst_h) / static_cast<float>(src_h);
  dst->reserve(src.size());
  for (const auto& d : src) {
    DnnDetection o = d;
    o.x1 = d.x1 * sx;
    o.y1 = d.y1 * sy;
    o.x2 = d.x2 * sx;
    o.y2 = d.y2 * sy;
    dst->push_back(o);
  }
}

bool MkDirRecursive(const std::string& path) {
  if (path.empty()) return false;
  std::string cur;
  cur.reserve(path.size());
  for (size_t i = 0; i < path.size(); ++i) {
    cur.push_back(path[i]);
    if (path[i] == '/' || i + 1 == path.size()) {
      if (cur.size() == 1 && cur[0] == '/') continue;
      std::string d = cur;
      while (d.size() > 1 && d.back() == '/') d.pop_back();
      if (d.empty() || d == "/") continue;
      struct stat st {};
      if (::stat(d.c_str(), &st) == 0) {
        if (!S_ISDIR(st.st_mode)) return false;
        continue;
      }
      if (::mkdir(d.c_str(), 0755) != 0) {
        if (::stat(d.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) return false;
      }
    }
  }
  return true;
}

void MaybeSaveDnnOverlay(const uint8_t* rgb, int w, int h,
                         const std::string& time_str, const DnnConfig& cfg,
                         std::uint64_t* tick, std::uint64_t* saved,
                         bool* limit_logged) {
  if (!cfg.save_overlay || !rgb || w <= 0 || h <= 0 || !tick || !saved ||
      !limit_logged) {
    return;
  }
  if (cfg.save_overlay_dir.empty()) return;
  ++(*tick);
  if (cfg.save_overlay_every_n > 1 &&
      ((*tick) % cfg.save_overlay_every_n) != 0) {
    return;
  }
  if (cfg.save_overlay_max_count > 0 &&
      *saved >= cfg.save_overlay_max_count) {
    if (!*limit_logged) {
      std::cerr << "[PercepDemo] save_overlay reached max_count="
                << cfg.save_overlay_max_count << ", stop" << std::endl;
      *limit_logged = true;
    }
    return;
  }
  if (!MkDirRecursive(cfg.save_overlay_dir)) {
    std::cerr << "[PercepDemo] mkdir save_overlay_dir failed: "
              << cfg.save_overlay_dir << std::endl;
    return;
  }
  const std::string ts = time_str.empty()
                             ? std::to_string(daye::getSystemTimeMs())
                             : time_str;
  char name[384];
  std::snprintf(name, sizeof(name), "%s/%s.jpg",
                cfg.save_overlay_dir.c_str(), ts.c_str());
  const int q = std::max(1, std::min(100, cfg.overlay_jpeg_quality));
  if (!stbi_write_jpg(name, w, h, 3, rgb, q)) {
    std::cerr << "[PercepDemo] stbi_write_jpg failed: " << name << std::endl;
    return;
  }
  ++(*saved);
  std::cout << "[PercepDemo] save_overlay " << w << "x" << h << " -> " << name
            << " (" << *saved << "/"
            << (cfg.save_overlay_max_count > 0
                    ? std::to_string(cfg.save_overlay_max_count)
                    : std::string("inf"))
            << ")" << std::endl;
}

std::string DnnOverlayFrameStem(const FrameKey& key, DnnInferenceLane lane) {
  const std::int64_t timestamp_ns = key.camera_timestamp_ns > 0
                                        ? key.camera_timestamp_ns
                                        : key.transport_timestamp_ns;
  std::ostringstream stem;
  stem << (lane == DnnInferenceLane::kMap ? "map" : "recharge") << '_';
  if (timestamp_ns > 0) {
    stem << timestamp_ns;
  } else {
    stem << "time_unknown";
  }
  stem << "_cam" << key.camera_frame_id << "_seq"
       << key.transport_sequence;
  return stem.str();
}

void DrawHLine(uint8_t* rgb, int w, int h, int x0, int x1, int y,
               uint8_t r, uint8_t g, uint8_t b) {
  if (y < 0 || y >= h) return;
  if (x0 > x1) std::swap(x0, x1);
  x0 = std::max(0, x0);
  x1 = std::min(w - 1, x1);
  uint8_t* row = rgb + static_cast<size_t>(y) * w * 3;
  for (int x = x0; x <= x1; ++x) {
    row[x * 3] = r;
    row[x * 3 + 1] = g;
    row[x * 3 + 2] = b;
  }
}

void DrawVLine(uint8_t* rgb, int w, int h, int x, int y0, int y1,
               uint8_t r, uint8_t g, uint8_t b) {
  if (x < 0 || x >= w) return;
  if (y0 > y1) std::swap(y0, y1);
  y0 = std::max(0, y0);
  y1 = std::min(h - 1, y1);
  for (int y = y0; y <= y1; ++y) {
    uint8_t* p = rgb + (static_cast<size_t>(y) * w + x) * 3;
    p[0] = r;
    p[1] = g;
    p[2] = b;
  }
}

// 5x7 点阵（bit4=左列 … bit0=右列），覆盖 a-z / 0-9 / _-
// 不依赖 OpenCV putText，避免板端叠加不可见
static const uint8_t kFont5x7[38][7] = {
    // 0-9
    {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E},
    {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},
    {0x0E, 0x11, 0x01, 0x06, 0x08, 0x10, 0x1F},
    {0x0E, 0x11, 0x01, 0x06, 0x01, 0x11, 0x0E},
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02},
    {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E},
    {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E},
    {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E},
    {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C},
    // a-z
    {0x00, 0x00, 0x0E, 0x01, 0x0F, 0x11, 0x0F},
    {0x10, 0x10, 0x1E, 0x11, 0x11, 0x11, 0x1E},
    {0x00, 0x00, 0x0E, 0x11, 0x10, 0x11, 0x0E},
    {0x01, 0x01, 0x0F, 0x11, 0x11, 0x11, 0x0F},
    {0x00, 0x00, 0x0E, 0x11, 0x1F, 0x10, 0x0E},
    {0x06, 0x08, 0x08, 0x1E, 0x08, 0x08, 0x08},
    {0x00, 0x00, 0x0F, 0x11, 0x0F, 0x01, 0x0E},
    {0x10, 0x10, 0x1E, 0x11, 0x11, 0x11, 0x11},
    {0x04, 0x00, 0x0C, 0x04, 0x04, 0x04, 0x0E},
    {0x02, 0x00, 0x06, 0x02, 0x02, 0x12, 0x0C},
    {0x10, 0x10, 0x12, 0x14, 0x18, 0x14, 0x12},
    {0x0C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E},
    {0x00, 0x00, 0x1A, 0x15, 0x15, 0x15, 0x15},
    {0x00, 0x00, 0x1E, 0x11, 0x11, 0x11, 0x11},
    {0x00, 0x00, 0x0E, 0x11, 0x11, 0x11, 0x0E},
    {0x00, 0x00, 0x1E, 0x11, 0x1E, 0x10, 0x10},
    {0x00, 0x00, 0x0F, 0x11, 0x0F, 0x01, 0x01},
    {0x00, 0x00, 0x16, 0x19, 0x10, 0x10, 0x10},
    {0x00, 0x00, 0x0F, 0x10, 0x0E, 0x01, 0x1E},
    {0x08, 0x08, 0x1E, 0x08, 0x08, 0x09, 0x06},
    {0x00, 0x00, 0x11, 0x11, 0x11, 0x11, 0x0F},
    {0x00, 0x00, 0x11, 0x11, 0x11, 0x0A, 0x04},
    {0x00, 0x00, 0x11, 0x15, 0x15, 0x15, 0x0A},
    {0x00, 0x00, 0x11, 0x0A, 0x04, 0x0A, 0x11},
    {0x00, 0x00, 0x11, 0x11, 0x0F, 0x01, 0x0E},
    {0x00, 0x00, 0x1F, 0x02, 0x04, 0x08, 0x1F},
    // _ -
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F},
    {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00},
};

inline int FontGlyphIndex(char ch) {
  if (ch >= '0' && ch <= '9') return ch - '0';
  if (ch >= 'a' && ch <= 'z') return 10 + (ch - 'a');
  if (ch >= 'A' && ch <= 'Z') return 10 + (ch - 'A');
  if (ch == '_') return 36;
  if (ch == '-') return 37;
  return -1;
}

void SetRgbPixel(uint8_t* rgb, int w, int h, int x, int y,
                 uint8_t r, uint8_t g, uint8_t b) {
  if (x < 0 || y < 0 || x >= w || y >= h) return;
  uint8_t* p = rgb + (static_cast<size_t>(y) * w + x) * 3;
  p[0] = r;
  p[1] = g;
  p[2] = b;
}

// 框内左上角画白字（黑描边，保证任意底上可见）
void DrawLabelInsideBox(uint8_t* rgb, int w, int h, int x1, int y1, int x2,
                        int y2, const std::string& label) {
  if (!rgb || label.empty()) return;
  const int scale = (h >= 480) ? 2 : 1;  // 大图 2x，缩小发布后仍可读
  const int gw = 5 * scale;
  const int gh = 7 * scale;
  const int gap = scale;
  const int pad = 2 * scale;
  const int n = static_cast<int>(label.size());
  const int tw = n * (gw + gap) - gap;
  const int th = gh;

  // 起点：检测框内侧左上，紧贴框线内
  int ox = x1 + 3;
  int oy = y1 + 3;
  if (ox + tw + pad > x2) ox = std::max(x1 + 1, x2 - tw - pad);
  if (oy + th + pad > y2) oy = std::max(y1 + 1, y2 - th - pad);
  if (ox < 0) ox = 0;
  if (oy < 0) oy = 0;

  auto draw_glyph = [&](int gx, int gy, int gi, uint8_t r, uint8_t g,
                        uint8_t b) {
    if (gi < 0) return;
    for (int row = 0; row < 7; ++row) {
      const uint8_t bits = kFont5x7[gi][row];
      for (int col = 0; col < 5; ++col) {
        // bit4 为最左列，避免字母左右镜像
        if (!(bits & (1u << (4 - col)))) continue;
        for (int sy = 0; sy < scale; ++sy) {
          for (int sx = 0; sx < scale; ++sx) {
            SetRgbPixel(rgb, w, h, gx + col * scale + sx,
                        gy + row * scale + sy, r, g, b);
          }
        }
      }
    }
  };

  for (int i = 0; i < n; ++i) {
    const int gi = FontGlyphIndex(label[static_cast<size_t>(i)]);
    const int gx = ox + i * (gw + gap);
    const int gy = oy;
    // 黑描边
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;
        draw_glyph(gx + dx, gy + dy, gi, 0, 0, 0);
      }
    }
    // 白字
    draw_glyph(gx, gy, gi, 255, 255, 255);
  }
}

void DrawDetBoxesOnRgb(uint8_t* rgb, int w, int h,
                       const std::vector<DnnDetection>& dets) {
  if (!rgb || w <= 0 || h <= 0) return;
  for (const auto& det : dets) {
    const auto& c = kDetPalette[(det.class_id >= 0 ? det.class_id : 0) % 12];
    const int x1 = std::max(0, std::min(w - 1, static_cast<int>(det.x1)));
    const int y1 = std::max(0, std::min(h - 1, static_cast<int>(det.y1)));
    const int x2 = std::max(0, std::min(w - 1, static_cast<int>(det.x2)));
    const int y2 = std::max(0, std::min(h - 1, static_cast<int>(det.y2)));
    for (int t = 0; t < 2; ++t) {
      DrawHLine(rgb, w, h, x1, x2, y1 + t, c[0], c[1], c[2]);
      DrawHLine(rgb, w, h, x1, x2, y2 - t, c[0], c[1], c[2]);
      DrawVLine(rgb, w, h, x1 + t, y1, y2, c[0], c[1], c[2]);
      DrawVLine(rgb, w, h, x2 - t, y1, y2, c[0], c[1], c[2]);
    }

    // class_name 白字画在框内左上角（框贴顶也能看到）
    const std::string label =
        det.class_name.empty() ? ("c" + std::to_string(det.class_id))
                               : det.class_name;
    DrawLabelInsideBox(rgb, w, h, x1, y1, x2, y2, label);
  }
}

std::string ReadJsonString(const rapidjson::Value& obj, const char* key,
                           const std::string& def) {
  if (!obj.HasMember(key) || !obj[key].IsString()) return def;
  return obj[key].GetString();
}

float ReadJsonFloat(const rapidjson::Value& obj, const char* key, float def) {
  if (!obj.HasMember(key)) return def;
  if (obj[key].IsNumber()) {
    const double value = obj[key].GetDouble();
    const double float_max =
        static_cast<double>(std::numeric_limits<float>::max());
    if (!std::isfinite(value) || value < -float_max || value > float_max) {
      // Callers validate all numeric configuration fields. Returning NaN
      // preserves the distinction between a missing key and an invalid number
      // without performing an out-of-range floating-point conversion.
      return std::numeric_limits<float>::quiet_NaN();
    }
    return static_cast<float>(value);
  }
  return def;
}

uint32_t ReadJsonUInt(const rapidjson::Value& obj, const char* key, uint32_t def) {
  if (!obj.HasMember(key)) return def;
  if (obj[key].IsUint()) return obj[key].GetUint();
  if (obj[key].IsUint64() &&
      obj[key].GetUint64() <= std::numeric_limits<uint32_t>::max()) {
    return static_cast<uint32_t>(obj[key].GetUint64());
  }
  if (obj[key].IsInt() && obj[key].GetInt() >= 0) {
    return static_cast<uint32_t>(obj[key].GetInt());
  }
  return def;
}

}  // namespace

bool PercepDemo::parseDnnConfig(const rapidjson::Value& root) {
  if (!root.HasMember("dnn") || !root["dnn"].IsObject()) {
    return true;
  }
  const rapidjson::Value& dnn = root["dnn"];
  dnn_cfg_.enabled = readJsonBool(dnn, "enabled", dnn_cfg_.enabled);
  if (dnn.HasMember("run_every_n") && dnn["run_every_n"].IsInt()) {
    dnn_cfg_.run_every_n = std::max(1, dnn["run_every_n"].GetInt());
  }
  dnn_cfg_.publish_overlay =
      readJsonBool(dnn, "publish_overlay", dnn_cfg_.publish_overlay);
  if (dnn.HasMember("save_overlay") && dnn["save_overlay"].IsBool()) {
    dnn_cfg_.save_overlay = dnn["save_overlay"].GetBool();
  } else if (dnn.HasMember("save_overlay") && dnn["save_overlay"].IsObject()) {
    const rapidjson::Value& so = dnn["save_overlay"];
    dnn_cfg_.save_overlay =
        readJsonBool(so, "enabled", dnn_cfg_.save_overlay);
    dnn_cfg_.save_overlay_dir =
        ReadJsonString(so, "dir", dnn_cfg_.save_overlay_dir);
    dnn_cfg_.save_overlay_every_n = ReadJsonUInt(
        so, "every_n", dnn_cfg_.save_overlay_every_n);
    if (dnn_cfg_.save_overlay_every_n < 1) dnn_cfg_.save_overlay_every_n = 1;
    dnn_cfg_.save_overlay_max_count = ReadJsonUInt(
        so, "max_count", dnn_cfg_.save_overlay_max_count);
    if (so.HasMember("quality") && so["quality"].IsNumber()) {
      const double quality = so["quality"].GetDouble();
      if (!std::isfinite(quality)) {
        std::cerr << "[PercepDemo] save_overlay.quality must be finite"
                  << std::endl;
        return false;
      }
      dnn_cfg_.overlay_jpeg_quality = static_cast<int>(
          std::clamp(quality, 1.0, 100.0));
    }
  }
  dnn_cfg_.overlay_blend_seg =
      readJsonBool(dnn, "overlay_blend_seg", dnn_cfg_.overlay_blend_seg);
  dnn_cfg_.overlay_seg_alpha =
      ReadJsonFloat(dnn, "overlay_seg_alpha", dnn_cfg_.overlay_seg_alpha);
  dnn_cfg_.overlay_use_original_resolution = readJsonBool(
      dnn, "overlay_use_original_resolution",
      dnn_cfg_.overlay_use_original_resolution);
  dnn_cfg_.overlay_pub_width =
      ReadJsonUInt(dnn, "overlay_pub_width", dnn_cfg_.overlay_pub_width);
  dnn_cfg_.overlay_pub_height =
      ReadJsonUInt(dnn, "overlay_pub_height", dnn_cfg_.overlay_pub_height);
  dnn_cfg_.left_yuv_format =
      ReadJsonString(dnn, "left_yuv_format", dnn_cfg_.left_yuv_format);
  {
    std::string& f = dnn_cfg_.left_yuv_format;
    for (char& c : f) {
      if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    }
    if (f == "yuv420p") f = "i420";
    if (f != "nv12" && f != "i420" && f != "auto") f = "nv12";
  }

  det_match_cfg_.enabled =
      readJsonBool(dnn, "publish_det_cloud", det_match_cfg_.enabled);
  det_match_cfg_.score_min =
      ReadJsonFloat(dnn, "det_match_score_min", det_match_cfg_.score_min);
  det_match_cfg_.depth_span_m =
      ReadJsonFloat(dnn, "det_depth_span_m", det_match_cfg_.depth_span_m);
  det_match_cfg_.max_depth_m =
      ReadJsonFloat(dnn, "det_max_depth_m", det_match_cfg_.max_depth_m);

  if (dnn.HasMember("detect") && dnn["detect"].IsObject()) {
    const rapidjson::Value& det = dnn["detect"];
    dnn_cfg_.detect.enabled =
        readJsonBool(det, "enabled", dnn_cfg_.detect.enabled);
    dnn_cfg_.detect.model_path =
        ReadJsonString(det, "model_path", dnn_cfg_.detect.model_path);
    dnn_cfg_.detect.score_threshold =
        ReadJsonFloat(det, "score_threshold", dnn_cfg_.detect.score_threshold);
    dnn_cfg_.detect.nms_threshold =
        ReadJsonFloat(det, "nms_threshold", dnn_cfg_.detect.nms_threshold);
    if (det.HasMember("nms_top_k") && det["nms_top_k"].IsInt()) {
      dnn_cfg_.detect.nms_top_k = det["nms_top_k"].GetInt();
    }
    if (det.HasMember("classes_num") && det["classes_num"].IsInt()) {
      dnn_cfg_.detect.classes_num = det["classes_num"].GetInt();
    }
    if (det.HasMember("reg") && det["reg"].IsInt()) {
      dnn_cfg_.detect.reg = det["reg"].GetInt();
    }
    if (det.HasMember("resize_type") && det["resize_type"].IsInt()) {
      dnn_cfg_.detect.resize_type = det["resize_type"].GetInt();
    }
    dnn_cfg_.detect.class_names =
        ReadJsonString(det, "class_names", dnn_cfg_.detect.class_names);
    dnn_cfg_.detect.keep_classes =
        ReadJsonString(det, "keep_classes", dnn_cfg_.detect.keep_classes);
  } else {
    dnn_cfg_.detect.enabled = false;
  }

  if (dnn.HasMember("seg") && dnn["seg"].IsObject()) {
    const rapidjson::Value& seg = dnn["seg"];
    dnn_cfg_.seg.enabled = readJsonBool(seg, "enabled", dnn_cfg_.seg.enabled);
    dnn_cfg_.seg.model_path =
        ReadJsonString(seg, "model_path", dnn_cfg_.seg.model_path);
    if (seg.HasMember("output_index") && seg["output_index"].IsInt()) {
      dnn_cfg_.seg.output_index = seg["output_index"].GetInt();
    }
    if (seg.HasMember("num_classes") && seg["num_classes"].IsInt()) {
      dnn_cfg_.seg.num_classes = seg["num_classes"].GetInt();
    }
    if (seg.HasMember("resize_type") && seg["resize_type"].IsInt()) {
      dnn_cfg_.seg.resize_type = seg["resize_type"].GetInt();
    }
    dnn_cfg_.seg.bgr_input =
        readJsonBool(seg, "bgr_input", dnn_cfg_.seg.bgr_input);
    dnn_cfg_.seg.swap_classes =
        readJsonBool(seg, "swap_classes", dnn_cfg_.seg.swap_classes);
    dnn_cfg_.seg.align_corners =
        readJsonBool(seg, "align_corners", dnn_cfg_.seg.align_corners);
    dnn_cfg_.seg.argmax_mode =
        ReadJsonString(seg, "argmax_mode", dnn_cfg_.seg.argmax_mode);
    for (char& c : dnn_cfg_.seg.argmax_mode) {
      if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    }
    if (dnn_cfg_.seg.argmax_mode != "direct" &&
        dnn_cfg_.seg.argmax_mode != "bilinear") {
      dnn_cfg_.seg.argmax_mode = "direct";
    }
    dnn_cfg_.seg.class_names =
        ReadJsonString(seg, "class_names", dnn_cfg_.seg.class_names);
    {
      const auto names = ParseSegClassNames(dnn_cfg_.seg.class_names);
      if (!names.empty()) {
        const int from_names = static_cast<int>(names.size());
        if (seg.HasMember("num_classes") && seg["num_classes"].IsInt() &&
            seg["num_classes"].GetInt() != from_names) {
          std::cerr << "[PercepDemo] dnn.seg.num_classes="
                    << seg["num_classes"].GetInt()
                    << " != class_names count=" << from_names
                    << ", use class_names; tensor will confirm at init"
                    << std::endl;
        }
        dnn_cfg_.seg.num_classes = from_names;
        dnn_cfg_.seg.class_names = JoinCsv(names);
      }
    }
  } else {
    dnn_cfg_.seg.enabled = false;
  }

  dnn_cfg_.parallel_det_seg =
      readJsonBool(dnn, "parallel_det_seg", dnn_cfg_.parallel_det_seg);

  if (dnn_cfg_.enabled && !dnn_cfg_.detect.enabled && !dnn_cfg_.seg.enabled) {
    std::cerr << "[PercepDemo] dnn.enabled but detect/seg both off" << std::endl;
  }
  if (dnn_cfg_.enabled && dnn_cfg_.detect.enabled &&
      dnn_cfg_.detect.model_path.empty()) {
    std::cerr << "[PercepDemo] detect enabled but model_path empty" << std::endl;
    return false;
  }
  if (dnn_cfg_.enabled && dnn_cfg_.seg.enabled &&
      dnn_cfg_.seg.model_path.empty()) {
    std::cerr << "[PercepDemo] seg enabled but model_path empty" << std::endl;
    return false;
  }
  if (dnn_cfg_.detect.enabled &&
      (!(dnn_cfg_.detect.score_threshold > 0.0F &&
         dnn_cfg_.detect.score_threshold < 1.0F) ||
       !std::isfinite(dnn_cfg_.detect.score_threshold) ||
       !(dnn_cfg_.detect.nms_threshold >= 0.0F &&
         dnn_cfg_.detect.nms_threshold <= 1.0F) ||
       !std::isfinite(dnn_cfg_.detect.nms_threshold) ||
       dnn_cfg_.detect.reg <= 0 || dnn_cfg_.detect.reg > 256 ||
       dnn_cfg_.detect.classes_num < 0 ||
       dnn_cfg_.detect.classes_num > 256 ||
       dnn_cfg_.detect.nms_top_k <= 0 ||
       dnn_cfg_.detect.nms_top_k > 4096 ||
       (dnn_cfg_.detect.resize_type != 0 &&
        dnn_cfg_.detect.resize_type != 1))) {
    std::cerr << "[PercepDemo] invalid detect postprocess configuration"
              << std::endl;
    return false;
  }
  if (dnn_cfg_.seg.enabled &&
      ((dnn_cfg_.seg.resize_type != 0 && dnn_cfg_.seg.resize_type != 1) ||
       dnn_cfg_.seg.output_index < -1)) {
    std::cerr << "[PercepDemo] invalid segmentation configuration"
              << std::endl;
    return false;
  }
  if (dnn_cfg_.overlay_pub_width >
          static_cast<uint32_t>(kMaxOverlayDimension) ||
      dnn_cfg_.overlay_pub_height >
          static_cast<uint32_t>(kMaxOverlayDimension) ||
      !std::isfinite(dnn_cfg_.overlay_seg_alpha) ||
      dnn_cfg_.overlay_seg_alpha < 0.0F ||
      dnn_cfg_.overlay_seg_alpha > 1.0F) {
    std::cerr << "[PercepDemo] invalid DNN overlay configuration"
              << std::endl;
    return false;
  }
  if (dnn_cfg_.overlay_pub_width > 0 &&
      dnn_cfg_.overlay_pub_height > 0) {
    std::size_t overlay_bytes = 0;
    if (!CheckedOverlayRgbBytes(
            static_cast<int>(dnn_cfg_.overlay_pub_width),
            static_cast<int>(dnn_cfg_.overlay_pub_height), &overlay_bytes)) {
      std::cerr << "[PercepDemo] DNN overlay dimensions exceed memory budget"
                << std::endl;
      return false;
    }
  }
  if (!std::isfinite(det_match_cfg_.score_min) ||
      det_match_cfg_.score_min < 0.0F || det_match_cfg_.score_min > 1.0F ||
      !std::isfinite(det_match_cfg_.depth_span_m) ||
      det_match_cfg_.depth_span_m < 0.0F ||
      !std::isfinite(det_match_cfg_.max_depth_m) ||
      det_match_cfg_.max_depth_m < 0.0F) {
    std::cerr << "[PercepDemo] invalid DNN/cloud matching configuration"
              << std::endl;
    return false;
  }
  return true;
}


void PercepDemo::parseLabelFusionConfig(const rapidjson::Value& root) {
  LabelFusionConfig next = label_fusion_cfg_;
  if (root.HasMember("label_fusion") &&
      root["label_fusion"].IsObject()) {
    const rapidjson::Value& lf = root["label_fusion"];
    next.enabled = readJsonBool(lf, "enabled", next.enabled);
    next.fuse_into_grid =
        readJsonBool(lf, "fuse_into_grid", next.fuse_into_grid);
    next.nongrass_seg_as_obstacle = readJsonBool(
        lf, "nongrass_seg_as_obstacle", next.nongrass_seg_as_obstacle);
    next.passable_classes =
        ReadJsonString(lf, "passable_classes", next.passable_classes);
    next.det_score_min =
        ReadJsonFloat(lf, "det_score_min", next.det_score_min);
    next.max_range_m = ReadJsonFloat(lf, "max_range_m", next.max_range_m);
    next.max_height_m =
        ReadJsonFloat(lf, "max_height_m", next.max_height_m);
    next.det_depth_span_m =
        ReadJsonFloat(lf, "det_depth_span_m", next.det_depth_span_m);
    next.det_depth_gap_m =
        ReadJsonFloat(lf, "det_depth_gap_m", next.det_depth_gap_m);
    if (lf.HasMember("det_depth_min_pts") &&
        lf["det_depth_min_pts"].IsInt()) {
      next.det_depth_min_pts = lf["det_depth_min_pts"].GetInt();
    }
    next.keep_below_cam_m =
        ReadJsonFloat(lf, "keep_below_cam_m", next.keep_below_cam_m);
    next.cluster_filter =
        readJsonBool(lf, "cluster_filter", next.cluster_filter);
    next.cluster_voxel_m =
        ReadJsonFloat(lf, "cluster_voxel_m", next.cluster_voxel_m);
    if (lf.HasMember("cluster_min_component_pts") &&
        lf["cluster_min_component_pts"].IsInt()) {
      next.cluster_min_component_pts =
          lf["cluster_min_component_pts"].GetInt();
    }
    next.seg_mask_postprocess =
        readJsonBool(lf, "seg_mask_postprocess", next.seg_mask_postprocess);
    if (lf.HasMember("seg_mask_majority_radius") &&
        lf["seg_mask_majority_radius"].IsInt()) {
      next.seg_mask_majority_radius =
          lf["seg_mask_majority_radius"].GetInt();
    }
    if (lf.HasMember("seg_mask_close_radius") &&
        lf["seg_mask_close_radius"].IsInt()) {
      next.seg_mask_close_radius = lf["seg_mask_close_radius"].GetInt();
    }
    if (lf.HasMember("seg_mask_close_iterations") &&
        lf["seg_mask_close_iterations"].IsInt()) {
      next.seg_mask_close_iterations =
          lf["seg_mask_close_iterations"].GetInt();
    }
    next.seg_cluster_filter =
        readJsonBool(lf, "seg_cluster_filter", next.seg_cluster_filter);
    if (lf.HasMember("seg_mask_erode_px") &&
        lf["seg_mask_erode_px"].IsInt()) {
      next.seg_mask_erode_px = lf["seg_mask_erode_px"].GetInt();
    }
    next.seg_depth_gap_m =
        ReadJsonFloat(lf, "seg_depth_gap_m", next.seg_depth_gap_m);
    if (lf.HasMember("seg_ray_bin_px") && lf["seg_ray_bin_px"].IsInt()) {
      next.seg_ray_bin_px = lf["seg_ray_bin_px"].GetInt();
    }
    if (lf.HasMember("seg_min_component_pts") &&
        lf["seg_min_component_pts"].IsInt()) {
      next.seg_min_component_pts = lf["seg_min_component_pts"].GetInt();
    }
  }
  const auto finite_between = [](float value, float lower,
                                 float upper) noexcept {
    return std::isfinite(value) && value >= lower && value <= upper;
  };
  const bool config_valid =
      next.passable_classes.size() <= 4096U &&
      finite_between(next.det_score_min, 0.0F, 1.0F) &&
      finite_between(next.max_range_m, 0.1F, 100.0F) &&
      finite_between(next.max_height_m, 0.0F, 20.0F) &&
      finite_between(next.det_depth_span_m, 0.0F, 100.0F) &&
      finite_between(next.det_depth_gap_m, 0.0F, 100.0F) &&
      next.det_depth_min_pts >= 1 && next.det_depth_min_pts <= 1000000 &&
      finite_between(next.keep_below_cam_m, 0.0F, 20.0F) &&
      finite_between(next.cluster_voxel_m, 0.001F, 10.0F) &&
      next.cluster_min_component_pts >= 1 &&
      next.cluster_min_component_pts <= 1000000 &&
      next.seg_mask_majority_radius >= 0 &&
      next.seg_mask_majority_radius <= 4 &&
      next.seg_mask_close_radius >= 0 && next.seg_mask_close_radius <= 4 &&
      next.seg_mask_close_iterations >= 1 &&
      next.seg_mask_close_iterations <= 3 &&
      next.seg_mask_erode_px >= 0 && next.seg_mask_erode_px <= 8 &&
      finite_between(next.seg_depth_gap_m, 0.0F, 100.0F) &&
      next.seg_ray_bin_px >= 1 && next.seg_ray_bin_px <= 4096 &&
      next.seg_min_component_pts >= 1 &&
      next.seg_min_component_pts <= 1000000;
  if (!config_valid) {
    std::cerr << "[PercepDemo] invalid label_fusion configuration; disable it"
              << std::endl;
    label_fusion_cfg_.enabled = false;
    return;
  }
  if (!dnn_cfg_.enabled) next.enabled = false;
  if (!dnn_cfg_.detect.enabled && !dnn_cfg_.seg.enabled) {
    next.enabled = false;
  }
  label_fusion_cfg_ = std::move(next);
}

Yuv420Format PercepDemo::resolveLeftYuvFormat(const Nv12Image& image,
                                              DnnImageSource source) {
  std::lock_guard<std::mutex> lock(yuv_format_mtx_);
  bool& format_known = source == DnnImageSource::kRgbd
                           ? rgbd_yuv_format_known_
                           : gdc_yuv_format_known_;
  bool& is_nv12 = source == DnnImageSource::kRgbd ? rgbd_yuv_is_nv12_
                                                  : gdc_yuv_is_nv12_;
  if (format_known) {
    return is_nv12 ? Yuv420Format::kNv12 : Yuv420Format::kI420;
  }
  const std::string& fmt = dnn_cfg_.left_yuv_format;
  YuvFormatProbe probe{};
  if (fmt == "auto") {
    ProbeYuv420Format(image.bytes.data(), image.width, image.height,
                      image.stride_y, image.bytes.size(), &probe);
    is_nv12 =
        std::string(probe.likely_format ? probe.likely_format : "unknown") !=
        "i420";
  } else {
    is_nv12 = (fmt != "i420");
  }
  format_known = true;
  std::cout << "[PercepDemo][yuv] source="
            << (source == DnnImageSource::kRgbd ? "rgbd" : "gdc")
            << " locked=" << (is_nv12 ? "nv12" : "i420")
            << " decision=" << (fmt == "auto" ? "probe" : "config");
  if (fmt == "auto") {
    std::cout << " center_nv12=" << static_cast<int>(probe.center_r_nv12)
              << "," << static_cast<int>(probe.center_g_nv12) << ","
              << static_cast<int>(probe.center_b_nv12)
              << " center_i420=" << static_cast<int>(probe.center_r_i420)
              << "," << static_cast<int>(probe.center_g_i420) << ","
              << static_cast<int>(probe.center_b_i420);
  }
  std::cout << std::endl;
  return is_nv12 ? Yuv420Format::kNv12 : Yuv420Format::kI420;
}

void PercepDemo::initDnnRuntime() {
  if (!dnn_cfg_.enabled) {
    std::cout << "[PercepDemo] dnn disabled" << std::endl;
    return;
  }
  if (dnn_cfg_.publish_overlay) {
    // 可视化图像用 Best Effort，避免 Reliable/SystemDefault 堆积导致 RViz 延迟
    const auto overlay_qos =
        rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
    dnn_overlay_pub_ = daye::CreatePublish<sensor_msgs::msg::Image>(
        "/percep_demo/dnn_overlay", overlay_qos);
    std::cout << "[PercepDemo] dnn_overlay qos=best_effort depth=5" << std::endl;
  }
  if (det_match_cfg_.enabled) {
    det_cloud_pub_ = daye::CreatePublish<sensor_msgs::msg::PointCloud2>(
        "/percep_demo/det_cloud", rclcpp::SensorDataQoS());
  }
  // 分割着色统一发 /percep_demo/dnn_overlay，不再保留无接线的独立 seg_mask 开关。
  dnn_runner_.reset(new HorizonDnnRunner());
  if (!dnn_runner_->init(dnn_cfg_)) {
    std::cerr << "[PercepDemo] HorizonDnnRunner init failed, disable dnn"
              << std::endl;
    dnn_cfg_.enabled = false;
    dnn_runner_.reset();
    return;
  }
  dnn_overlay_save_tick_ = 0;
  dnn_overlay_saved_count_ = 0;
  dnn_overlay_save_limit_logged_ = false;
  dnn_fps_window_start_ = std::chrono::steady_clock::now();
  dnn_fps_window_count_ = 0;
  dnn_fps_ema_ = 0.0;
  const DnnConfig resolved_dnn_cfg = dnn_runner_->config();
  dnn_cfg_.detect = resolved_dnn_cfg.detect;
  dnn_cfg_.seg = resolved_dnn_cfg.seg;
  if (label_fusion_cfg_.enabled) {
    initLabelFusionClassIds();
  }
  std::cout << "[PercepDemo] dnn ready detect="
            << (dnn_cfg_.detect.enabled ? "on" : "off")
            << " seg=" << (dnn_cfg_.seg.enabled ? "on" : "off")
            << " det_model=" << dnn_cfg_.detect.model_path
            << " seg_model=" << dnn_cfg_.seg.model_path
            << " seg_classes=" << dnn_cfg_.seg.num_classes
            << " [" << dnn_cfg_.seg.class_names << "]"
            << " overlay=" << (dnn_cfg_.publish_overlay ? "on" : "off")
            << " save_overlay=" << (dnn_cfg_.save_overlay ? "on" : "off")
            << " save_dir=" << dnn_cfg_.save_overlay_dir
            << " det_cloud=" << (det_match_cfg_.enabled ? "on" : "off")
            << " label_fusion=" << (label_fusion_cfg_.enabled ? "on" : "off")
            << " fuse_grid=" << (label_fusion_cfg_.fuse_into_grid ? "on" : "off")
            << " overlay_size="
            << (dnn_cfg_.overlay_use_original_resolution
                    ? "orig"
                    : (std::to_string(dnn_cfg_.overlay_pub_width) + "x" +
                       std::to_string(dnn_cfg_.overlay_pub_height)))
            << std::endl;
}

bool PercepDemo::consumeDnnCadence(DnnInferenceLane lane) {
  std::uint64_t& call_index =
      lane == DnnInferenceLane::kMap ? map_dnn_call_index_
                                    : recharge_dnn_call_index_;
  const std::uint64_t current_call = call_index++;
  return periodicFrameDue(current_call, std::max(1, dnn_cfg_.run_every_n));
}

std::optional<DnnFrameResult> PercepDemo::runDnnInference(
    const Nv12Image& image, DnnInferenceLane lane, bool run_seg) {
  try {
  if (shutting_down_.load(std::memory_order_acquire)) return std::nullopt;
  if (!dnn_runner_ || !image.valid()) return std::nullopt;
  if (!frameKeyHasIdentity(image.key)) {
    LOG_WARN("[PercepDemo] skip DNN: image has no usable frame identity");
    return std::nullopt;
  }

  const int img_w = image.width;
  const int img_h = image.height;
  const std::size_t bytes_used = image.bytes.size();
  const int stride_y = image.stride_y;

  const bool source_is_nv12 = image.format != Yuv420Format::kI420;

  DnnFrameResult result;
  result.key = image.key;
  result.image_width = img_w;
  result.image_height = img_h;
  std::vector<DnnDetection>* det_ptr =
      dnn_cfg_.detect.enabled ? &result.detections : nullptr;
  std::vector<uint8_t>* seg_ptr =
      (run_seg && dnn_cfg_.seg.enabled) ? &result.segmentation_mask : nullptr;
  DnnInferStatus infer_status;
  // 推理不填 overlay（避免 runner 黑底）；底图用已验证的 nv12→rgb24
  if (!dnn_runner_->inferNv12(
          image.bytes.data(), img_w, img_h, stride_y, bytes_used,
          source_is_nv12, det_ptr,
          seg_ptr, /*overlay_rgb=*/nullptr, &result.timing, &infer_status)) {
    return std::nullopt;
  }

  result.detection_requested = infer_status.detect_requested;
  result.detection_succeeded = infer_status.detect_succeeded;
  result.segmentation_requested = infer_status.segmentation_requested;
  result.segmentation_succeeded = infer_status.segmentation_succeeded;

  if (result.segmentation_succeeded &&
      !result.segmentation_mask.empty()) {
    postprocessSegMask(result.segmentation_mask, img_w, img_h);
  }

  const size_t det_n = result.detections.size();
  // runner 已在候选生成/NMS/top-k 前应用 keep_classes。
  {
    std::ostringstream oss;
    oss << "[PercepDemo][det] frame=" << frame_count_ << " det_n=" << det_n
        << " " << img_w << "x" << img_h;
    constexpr size_t kMaxLogDets = 12;
    for (size_t i = 0; i < result.detections.size() && i < kMaxLogDets; ++i) {
      const auto& d = result.detections[i];
      oss << " ["
          << (d.class_name.empty() ? ("c" + std::to_string(d.class_id))
                                   : d.class_name)
          << " id=" << d.class_id
          << " conf=" << static_cast<int>(d.confidence * 100.f + 0.5f) << "%"
          << " xyxy=" << static_cast<int>(d.x1) << ","
          << static_cast<int>(d.y1) << "," << static_cast<int>(d.x2) << ","
          << static_cast<int>(d.y2) << "]";
    }
    if (result.detections.size() > kMaxLogDets) {
      oss << " ...+" << (result.detections.size() - kMaxLogDets);
    }
    const std::string line = oss.str();
    LOG_INFO("%s", line.c_str());
    std::cout << line << std::endl;
  }
  double overlay_ms = 0;
  double overlay_pub_ms = 0;
  double overlay_save_ms = 0;
  const bool want_overlay =
      (dnn_cfg_.publish_overlay || dnn_cfg_.save_overlay) && pclCommonPtr;
  if (want_overlay) {
    using clock = std::chrono::steady_clock;
    const auto t_ov = clock::now();
    int pub_w = img_w;
    int pub_h = img_h;
    if (!dnn_cfg_.overlay_use_original_resolution &&
        dnn_cfg_.overlay_pub_width > 0) {
      pub_w = static_cast<int>(dnn_cfg_.overlay_pub_width);
      if (dnn_cfg_.overlay_pub_height > 0) {
        pub_h = static_cast<int>(dnn_cfg_.overlay_pub_height);
      } else {
        pub_h = std::max(1, img_h * pub_w / std::max(1, img_w));
      }
    }
    std::size_t overlay_rgb_bytes = 0;
    if (!CheckedOverlayRgbBytes(pub_w, pub_h, &overlay_rgb_bytes)) {
      LOG_WARN("[PercepDemo] invalid/oversized overlay target %dx%d; "
               "fall back to source resolution",
               pub_w, pub_h);
      pub_w = img_w;
      pub_h = img_h;
      if (!CheckedOverlayRgbBytes(pub_w, pub_h, &overlay_rgb_bytes)) {
        LOG_ERROR("[PercepDemo] source image is too large for RGB overlay");
        return std::optional<DnnFrameResult>{std::move(result)};
      }
    }
    const bool need_full_rgb =
        run_seg && dnn_cfg_.overlay_blend_seg && dnn_cfg_.seg.enabled &&
        !result.segmentation_mask.empty();
    const std::vector<uint8_t>* pub_buf = &dnn_overlay_buf_;
    std::vector<DnnDetection> overlay_dets = result.detections;
    bool ok = false;
    if (need_full_rgb) {
      // 叠分割时必须全分辨率 RGB，避免掩码边缘被预缩放弄锯齿
      ok = source_is_nv12
               ? pclCommonPtr->yuv_nv12_to_rgb24(
                     image.bytes.data(), img_w, img_h, stride_y,
                     &dnn_overlay_buf_)
               : Yuv420NearestToRgb24(
                     image.bytes.data(), img_w, img_h, stride_y,
                     /*src_is_nv12=*/false, img_w, img_h,
                     &dnn_overlay_buf_);
      if (ok) {
        BlendSegMaskOnRgb(dnn_overlay_buf_.data(), img_w, img_h,
                          result.segmentation_mask, dnn_cfg_.overlay_seg_alpha,
                          dnn_cfg_.seg.class_names);
        DrawDetBoxesOnRgb(dnn_overlay_buf_.data(), img_w, img_h,
                          result.detections);
        if (pub_w != img_w || pub_h != img_h) {
          if (pclCommonPtr->rgb8_resize(dnn_overlay_buf_, img_w, img_h,
                                        dnn_overlay_scaled_buf_, pub_w, pub_h)) {
            pub_buf = &dnn_overlay_scaled_buf_;
          } else {
            pub_w = img_w;
            pub_h = img_h;
            pub_buf = &dnn_overlay_buf_;
          }
        }
      }
    } else if (pub_w != img_w || pub_h != img_h) {
      // 仅可视化：在发布分辨率取样 NV12，不改 det_cloud / seg_mask
      ok = Yuv420NearestToRgb24(
          image.bytes.data(), img_w, img_h, stride_y, source_is_nv12,
          pub_w, pub_h, &dnn_overlay_scaled_buf_);
      if (ok) {
        ScaleDetsToOverlay(result.detections, img_w, img_h, pub_w, pub_h,
                           &overlay_dets);
        DrawDetBoxesOnRgb(dnn_overlay_scaled_buf_.data(), pub_w, pub_h,
                          overlay_dets);
        pub_buf = &dnn_overlay_scaled_buf_;
      }
    } else {
      ok = source_is_nv12
               ? pclCommonPtr->yuv_nv12_to_rgb24(
                     image.bytes.data(), img_w, img_h, stride_y,
                     &dnn_overlay_buf_)
               : Yuv420NearestToRgb24(
                     image.bytes.data(), img_w, img_h, stride_y,
                     /*src_is_nv12=*/false, img_w, img_h,
                     &dnn_overlay_buf_);
      if (ok) {
        DrawDetBoxesOnRgb(dnn_overlay_buf_.data(), img_w, img_h,
                          result.detections);
      }
    }
    overlay_ms = std::chrono::duration<double, std::milli>(clock::now() - t_ov)
                     .count();
    if (!ok) {
      std::cerr << "[PercepDemo] dnn overlay nv12→rgb failed" << std::endl;
    } else {
      if (dnn_cfg_.publish_overlay) {
        const auto t_pub = clock::now();
        sensor_msgs::msg::Image msg;
        const std::int64_t timestamp_ns =
            image.key.camera_timestamp_ns > 0
                ? image.key.camera_timestamp_ns
                : image.key.transport_timestamp_ns;
        msg.header.stamp = timestamp_ns > 0
                               ? rclcpp::Time(timestamp_ns, RCL_ROS_TIME)
                               : rclcpp::Clock().now();
        msg.header.frame_id = "percep_demo";
        msg.encoding = "rgb8";
        msg.is_bigendian = false;
        msg.width = static_cast<uint32_t>(pub_w);
        msg.height = static_cast<uint32_t>(pub_h);
        msg.step = static_cast<uint32_t>(pub_w * 3);
        msg.data = *pub_buf;
        dnn_overlay_pub_.publish(msg);
        overlay_pub_ms =
            std::chrono::duration<double, std::milli>(clock::now() - t_pub)
                .count();
      }
      if (dnn_cfg_.save_overlay) {
        const auto t_sv = clock::now();
        MaybeSaveDnnOverlay(pub_buf->data(), pub_w, pub_h,
                            DnnOverlayFrameStem(result.key, lane), dnn_cfg_,
                            &dnn_overlay_save_tick_,
                            &dnn_overlay_saved_count_,
                            &dnn_overlay_save_limit_logged_);
        overlay_save_ms =
            std::chrono::duration<double, std::milli>(clock::now() - t_sv)
                .count();
      }
    }
  }

  // 每次推理写 LOG（带 fps / 业务检测数量）；便于确认新 so 与过滤结果
  {
    using clock = std::chrono::steady_clock;
    ++dnn_fps_window_count_;
    const auto now = clock::now();
    const double dt =
        std::chrono::duration<double>(now - dnn_fps_window_start_).count();
    if (dt >= 1.0) {
      dnn_fps_ema_ = static_cast<double>(dnn_fps_window_count_) / dt;
      dnn_fps_window_count_ = 0;
      dnn_fps_window_start_ = now;
    }
    std::ostringstream oss;
    oss << "[PercepDemo] dnn frame=" << frame_count_
        << " camera_frame=" << result.key.camera_frame_id
        << " lane="
        << (lane == DnnInferenceLane::kMap ? "map" : "recharge")
        << " det_ok=" << (result.detection_succeeded ? 1 : 0)
        << " seg_ok=" << (result.segmentation_succeeded ? 1 : 0)
        << " det_ms=" << static_cast<int>(result.timing.detect_ms)
        << " det_pre=" << static_cast<int>(result.timing.detect_pre_ms)
        << " det_bpu=" << static_cast<int>(result.timing.detect_bpu_ms)
        << " seg_ms=" << static_cast<int>(result.timing.seg_ms)
        << " seg_pre=" << static_cast<int>(result.timing.seg_pre_ms)
        << " seg_bpu=" << static_cast<int>(result.timing.seg_bpu_ms)
        << " seg_post=" << static_cast<int>(result.timing.seg_post_ms)
        << " overlay_ms=" << static_cast<int>(overlay_ms)
        << " overlay_pub_ms=" << static_cast<int>(overlay_pub_ms)
        << " overlay_save_ms=" << static_cast<int>(overlay_save_ms)
        << " fps=" << static_cast<int>(dnn_fps_ema_ + 0.5)
        << " det_n=" << result.detections.size();
    for (size_t i = 0; i < result.detections.size() && i < 8; ++i) {
      const auto& d = result.detections[i];
      oss << " ["
          << (d.class_name.empty() ? std::to_string(d.class_id) : d.class_name)
          << ":" << static_cast<int>(d.confidence * 100) << "%]";
    }
    if (result.detections.size() > 8) oss << " ...";
    const std::string line = oss.str();
    LOG_INFO("%s", line.c_str());
    std::cout << line << std::endl;
  }
  return std::optional<DnnFrameResult>{std::move(result)};
  } catch (const std::exception& error) {
    LOG_ERROR("[PercepDemo] DNN host processing exception: %s", error.what());
    return std::nullopt;
  } catch (...) {
    LOG_ERROR("[PercepDemo] DNN host processing unknown exception");
    return std::nullopt;
  }
}

void PercepDemo::matchAndPublishDetCloud(const PointCloudT::Ptr& cloud,
                                         const Nv12Image& image,
                                         const DnnFrameResult& result,
                                         float fx, float fy, float cx,
                                         float cy) {
  try {
  if (!result.matches(image)) {
    LOG_WARN("[PercepDemo] skip DNN/cloud fusion: frame result mismatch");
    return;
  }
  const auto& detections = result.detections;
  const int img_w = image.width;
  const int img_h = image.height;
  if (label_fusion_cfg_.enabled) {
    fuseSemanticLabels(cloud, detections, result.segmentation_mask,
                       result.image_width, result.image_height, img_w, img_h,
                       fx, fy, cx, cy);
    return;
  }
  if (!det_match_cfg_.enabled) return;
  if (!cloud || cloud->empty()) {
    if (det_match_cfg_.enabled) {
      PointCloudT::Ptr empty(new PointCloudT);
      last_semantic_cloud_ = empty;
      publish_ros_rgbcloud(empty, det_cloud_pub_, parent_frame, false);
    }
    return;
  }
  if (detections.empty()) {
    PointCloudT::Ptr unlabeled(new PointCloudT(*cloud));
    for (auto& q : unlabeled->points) {
      q.label = -1;
      q.confidence = 0.f;
    }
    PointCloudT::Ptr det_pub(new PointCloudT(*unlabeled));
    if (pclCommonPtr) {
      Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
      pclCommonPtr->EulerToMatrix(cali, T);
      pcl::transformPointCloud(*det_pub, *det_pub, T);
    }
    last_semantic_cloud_ = det_pub;
    if (det_match_cfg_.enabled) {
      publish_ros_rgbcloud(det_pub, det_cloud_pub_, parent_frame, false);
    }
    return;
  }
  if (img_w <= 0 || img_h <= 0 || fx <= 1e-6f) return;

  const float score_min = det_match_cfg_.score_min;
  // max_depth_m<=0：不截断（点云范围由 point_cloud_pub.box_* 约束）；
  // >0：相机系深度上限。旧默认 0.6m 会丢掉 2m+ 检测障碍。
  const bool limit_depth = det_match_cfg_.max_depth_m > 0.05f;
  const float max_depth = limit_depth ? det_match_cfg_.max_depth_m : 1e9f;
  std::vector<std::vector<size_t>> box_pts(detections.size());
  std::vector<std::vector<float>> box_z(detections.size());

  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto& p = cloud->points[i];
    if (p.z <= 0.05f || p.z > max_depth) continue;
    const float u = fx * (p.x / p.z) + cx;
    const float v = fy * (p.y / p.z) + cy;
    if (u < 0 || v < 0 || u >= img_w || v >= img_h) continue;

    int best = -1;
    float best_conf = score_min;
    for (size_t di = 0; di < detections.size(); ++di) {
      const auto& d = detections[di];
      if (d.confidence < score_min) continue;
      if (u >= d.x1 && u <= d.x2 && v >= d.y1 && v <= d.y2) {
        if (d.confidence >= best_conf) {
          best_conf = d.confidence;
          best = static_cast<int>(di);
        }
      }
    }
    if (best >= 0) {
      box_pts[best].push_back(i);
      box_z[best].push_back(p.z);
    }
  }

  // 每框：取点数最多的连续深度簇（同点数取更近），抑制背景拖尾；
  // 不再固定从最近端起切，避免近处杂点抢走远处主体
  const float span = std::max(0.05f, det_match_cfg_.depth_span_m);
  const float gap = 0.25f;
  std::vector<float> zmin(detections.size(), -1e9f);
  std::vector<float> zmax(detections.size(), 1e9f);
  for (size_t d = 0; d < detections.size(); ++d) {
    auto& zs = box_z[d];
    if (zs.size() < 5) continue;
    std::sort(zs.begin(), zs.end());
    size_t best_s = 0, best_e = 0;
    int best_n = -1;
    double best_w = -1.0;
    size_t i = 0;
    while (i < zs.size()) {
      size_t j = i;
      while (j + 1 < zs.size() && (zs[j + 1] - zs[j]) <= gap) ++j;
      const int run_n = static_cast<int>(j - i + 1);
      double run_w = 0.0;
      for (size_t k = i; k <= j; ++k) {
        const double z = static_cast<double>(zs[k]);
        run_w += z * z;  // 透视密度补偿，避免近密远疏抢走远处主体
      }
      const bool better =
          (best_n < 0) || (run_w > best_w * 1.05) ||
          (run_w >= best_w * 0.95 && zs[i] < zs[best_s]);
      if (better) {
        best_s = i;
        best_e = j;
        best_n = run_n;
        best_w = run_w;
      }
      i = j + 1;
    }
    if (best_n < 0) continue;
    float z0 = zs[best_s];
    float z1 = zs[best_e];
    if (z1 - z0 > span) z1 = z0 + span;
    zmin[d] = z0 - 0.03f;
    zmax[d] = z1 + 0.03f;
  }

  PointCloudT::Ptr det_cloud(new PointCloudT(*cloud));
  for (auto& q : det_cloud->points) {
    q.label = -1;
    q.confidence = 0.f;
  }
  for (size_t di = 0; di < detections.size(); ++di) {
    for (size_t idx : box_pts[di]) {
      auto& p = det_cloud->points[idx];
      if (p.z < zmin[di] || p.z > zmax[di]) continue;
      const int cid = detections[di].class_id;
      if (cid < 0) continue;
      const auto& col = kDetPalette[(cid >= 0 ? cid : 0) % 12];
      p.r = col[0];
      p.g = col[1];
      p.b = col[2];
      p.label = EncodeDetLabel(cid);
      p.confidence = detections[di].confidence;
    }
  }

  // 与主点云一致：先乘外参 cali（相机系 Y 朝下 → base_link），再 publish
  // 否则 RViz 里目标会头朝下（相机光学系未转）
  PointCloudT::Ptr det_pub(new PointCloudT(*det_cloud));
  if (pclCommonPtr) {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    pclCommonPtr->EulerToMatrix(cali, T);
    pcl::transformPointCloud(*det_pub, *det_pub, T);
  }
  last_semantic_cloud_ = det_pub;

  if (det_match_cfg_.enabled) {
    publish_ros_rgbcloud(det_pub, det_cloud_pub_, parent_frame, false);
  }

  std::cout << "[PercepDemo] det_cloud pts=" << det_cloud->size()
            << " boxes=" << detections.size() << std::endl;
  } catch (const std::exception& error) {
    LOG_ERROR("[PercepDemo] DNN/cloud fusion exception: %s", error.what());
  } catch (...) {
    LOG_ERROR("[PercepDemo] DNN/cloud fusion unknown exception");
  }
}

}  // namespace ai_node_percep_demo

#endif  // PERCEP_DNN_ENABLE
