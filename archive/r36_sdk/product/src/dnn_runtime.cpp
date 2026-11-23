/**
 * @file dnn_runtime.cpp
 * @brief obs/detection：仿 obs/segmentation 接入 Horizon 检测，发布检测结果与匹配点云
 */
#include "percep_demo/percep_demo.h"

#if defined(PERCEP_DNN_ENABLE)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <errno.h>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unordered_map>
#include <unordered_set>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include "stb_image_write.h"

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
  const float a = std::max(0.05f, std::min(0.80f, alpha));
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

void Nv12NearestToRgb24(const uint8_t* nv12, int src_w, int src_h, int stride_y,
                        int dst_w, int dst_h, std::vector<uint8_t>* rgb) {
  if (!nv12 || !rgb || src_w <= 0 || src_h <= 0 || dst_w <= 0 || dst_h <= 0) {
    return;
  }
  if (stride_y < src_w) stride_y = src_w;
  rgb->resize(static_cast<size_t>(dst_w) * dst_h * 3);
  const uint8_t* y_plane = nv12;
  const uint8_t* uv_plane = nv12 + static_cast<size_t>(stride_y) * src_h;
  uint8_t* dst = rgb->data();
  for (int y = 0; y < dst_h; ++y) {
    const int sy = std::min(src_h - 1, y * src_h / dst_h);
    const uint8_t* yrow = y_plane + static_cast<size_t>(sy) * stride_y;
    const uint8_t* uvrow =
        uv_plane + static_cast<size_t>(sy / 2) * stride_y;
    uint8_t* out = dst + static_cast<size_t>(y) * dst_w * 3;
    for (int x = 0; x < dst_w; ++x) {
      const int sx = std::min(src_w - 1, x * src_w / dst_w);
      const int uv_off = (sx / 2) * 2;
      Yuv601ToRgb(yrow[sx], uvrow[uv_off], uvrow[uv_off + 1], &out[x * 3],
                  &out[x * 3 + 1], &out[x * 3 + 2]);
    }
  }
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

void ClearOverlayDir(const std::string& dir) {
  if (dir.empty() || dir == "/" || dir == ".") return;
  namespace fs = std::filesystem;
  std::error_code ec;
  if (!fs::exists(dir, ec) || !fs::is_directory(dir, ec)) return;
  size_t n = 0;
  for (fs::directory_iterator it(dir, ec), end; it != end && !ec; it.increment(ec)) {
    if (ec) break;
    if (!it->is_regular_file(ec)) continue;
    fs::remove(it->path(), ec);
    if (!ec) ++n;
  }
  std::cout << "[PercepDemo] save_overlay cleared " << n
            << " files in " << dir << std::endl;
}

void MaybeSaveDnnOverlay(const uint8_t* rgb, int w, int h,
                         const std::string& time_str, const DnnConfig& cfg) {
  if (!cfg.save_overlay || !rgb || w <= 0 || h <= 0) return;
  if (cfg.save_overlay_dir.empty()) {
    r28::Status status;status.code=2002;status.feature="product.output.ai_overlay";
    status.detail="save_overlay enabled with empty output directory";throw r28::Failure(status);
  }
  static uint64_t tick = 0;
  static uint64_t saved = 0;
  ++tick;
  if (cfg.save_overlay_every_n > 1 &&
      (tick % cfg.save_overlay_every_n) != 0) {
    return;
  }
  if (cfg.save_overlay_max_count > 0 &&
      saved >= cfg.save_overlay_max_count) {
    if (saved == cfg.save_overlay_max_count) {
      std::cerr << "[PercepDemo] save_overlay reached max_count="
                << cfg.save_overlay_max_count << ", stop" << std::endl;
      ++saved;
    }
    return;
  }
  if (!MkDirRecursive(cfg.save_overlay_dir)) {
    std::cerr << "[PercepDemo] mkdir save_overlay_dir failed: "
              << cfg.save_overlay_dir << std::endl;
    r28::Status status;status.code=2002;status.feature="product.output.ai_overlay";
    status.detail="cannot create AI overlay directory";throw r28::Failure(status);
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
    r28::Status status;status.code=2002;status.feature="product.output.ai_overlay";
    status.detail="AI overlay JPEG write failed";throw r28::Failure(status);
  }
  ++saved;
  std::cout << "[PercepDemo] save_overlay " << w << "x" << h << " -> " << name
            << " (" << saved << "/"
            << (cfg.save_overlay_max_count > 0
                    ? std::to_string(cfg.save_overlay_max_count)
                    : std::string("inf"))
            << ")" << std::endl;
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
  if (obj[key].IsNumber()) return static_cast<float>(obj[key].GetDouble());
  return def;
}

uint32_t ReadJsonUInt(const rapidjson::Value& obj, const char* key, uint32_t def) {
  if (!obj.HasMember(key)) return def;
  if (obj[key].IsUint()) return obj[key].GetUint();
  if (obj[key].IsInt() && obj[key].GetInt() >= 0)
    return static_cast<uint32_t>(obj[key].GetInt());
  if (obj[key].IsNumber() && obj[key].GetDouble() >= 0)
    return static_cast<uint32_t>(obj[key].GetDouble());
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
  dnn_cfg_.publish_seg_mask = false;  // 统一叠到 dnn_overlay，不再单独发 seg_mask
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
      dnn_cfg_.overlay_jpeg_quality =
          std::max(1, std::min(100, static_cast<int>(so["quality"].GetDouble())));
    }
  }
  dnn_cfg_.overlay_blend_seg =
      readJsonBool(dnn, "overlay_blend_seg", dnn_cfg_.overlay_blend_seg);
  dnn_cfg_.overlay_seg_alpha =
      ReadJsonFloat(dnn, "overlay_seg_alpha", dnn_cfg_.overlay_seg_alpha);
  dnn_cfg_.overlay_use_original_resolution = readJsonBool(
      dnn, "overlay_use_original_resolution",
      dnn_cfg_.overlay_use_original_resolution);
  if (dnn.HasMember("overlay_pub_width") && dnn["overlay_pub_width"].IsNumber()) {
    dnn_cfg_.overlay_pub_width =
        static_cast<uint32_t>(dnn["overlay_pub_width"].GetInt());
  }
  if (dnn.HasMember("overlay_pub_height") && dnn["overlay_pub_height"].IsNumber()) {
    dnn_cfg_.overlay_pub_height =
        static_cast<uint32_t>(dnn["overlay_pub_height"].GetInt());
  }
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
  det_match_cfg_.publish_markers =
      readJsonBool(dnn, "publish_det_markers", det_match_cfg_.publish_markers);
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
  return true;
}


void PercepDemo::parseLabelFusionConfig(const rapidjson::Value& root) {
  if (root.HasMember("label_fusion") &&
        root["label_fusion"].IsObject()) {
        const rapidjson::Value& lf = root["label_fusion"];
        label_fusion_cfg_.enabled =
            readJsonBool(lf, "enabled", label_fusion_cfg_.enabled);
        label_fusion_cfg_.fuse_into_grid =
            readJsonBool(lf, "fuse_into_grid", label_fusion_cfg_.fuse_into_grid);
        label_fusion_cfg_.publish_debug_cloud =
            readJsonBool(lf, "publish_debug_cloud",
                         label_fusion_cfg_.publish_debug_cloud);
        label_fusion_cfg_.nongrass_seg_as_obstacle =
            readJsonBool(lf, "nongrass_seg_as_obstacle",
                         label_fusion_cfg_.nongrass_seg_as_obstacle);
        label_fusion_cfg_.passable_classes = ReadJsonString(
            lf, "passable_classes", label_fusion_cfg_.passable_classes);
        label_fusion_cfg_.det_score_min = ReadJsonFloat(lf, "det_score_min", label_fusion_cfg_.det_score_min);
        if (lf.HasMember("min_obstacle_points_per_cell") &&
            lf["min_obstacle_points_per_cell"].IsInt()) {
            label_fusion_cfg_.min_obstacle_points_per_cell =
                std::max(1, lf["min_obstacle_points_per_cell"].GetInt());
        }
        if (lf.HasMember("min_semantic_points_per_cell") &&
            lf["min_semantic_points_per_cell"].IsInt()) {
            label_fusion_cfg_.min_semantic_points_per_cell =
                std::max(1, lf["min_semantic_points_per_cell"].GetInt());
        }
        label_fusion_cfg_.max_range_m = ReadJsonFloat(lf, "max_range_m", label_fusion_cfg_.max_range_m);
        label_fusion_cfg_.max_height_m = ReadJsonFloat(lf, "max_height_m", label_fusion_cfg_.max_height_m);
        label_fusion_cfg_.match_downsample = ReadJsonString(
            lf, "match_downsample", label_fusion_cfg_.match_downsample);
        {
            std::string& m = label_fusion_cfg_.match_downsample;
            for (char& c : m) {
                if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
            }
            if (m != "stride" && m != "voxel" && m != "uniform") {
                std::cout << "[PercepDemo][label_fusion] unknown match_downsample=\""
                     << m << "\", fallback stride" << std::endl;
                m = "stride";
            }
        }
        label_fusion_cfg_.match_voxel_m = ReadJsonFloat(lf, "match_voxel_m", label_fusion_cfg_.match_voxel_m);
        label_fusion_cfg_.max_match_points = ReadJsonUInt(
            lf, "max_match_points",
            static_cast<uint32_t>(label_fusion_cfg_.max_match_points));
        label_fusion_cfg_.det_depth_span_m = ReadJsonFloat(lf, "det_depth_span_m", label_fusion_cfg_.det_depth_span_m);
        label_fusion_cfg_.det_depth_gap_m = ReadJsonFloat(lf, "det_depth_gap_m", label_fusion_cfg_.det_depth_gap_m);
        if (lf.HasMember("det_depth_min_pts") && lf["det_depth_min_pts"].IsInt()) {
            label_fusion_cfg_.det_depth_min_pts =
                std::max(1, lf["det_depth_min_pts"].GetInt());
        }
        label_fusion_cfg_.keep_below_cam_m = ReadJsonFloat(lf, "keep_below_cam_m", label_fusion_cfg_.keep_below_cam_m);
        label_fusion_cfg_.cluster_filter =
            readJsonBool(lf, "cluster_filter", label_fusion_cfg_.cluster_filter);
        label_fusion_cfg_.cluster_voxel_m = ReadJsonFloat(lf, "cluster_voxel_m", label_fusion_cfg_.cluster_voxel_m);
        if (lf.HasMember("cluster_min_pts") && lf["cluster_min_pts"].IsInt()) {
            label_fusion_cfg_.cluster_min_pts =
                std::max(1, lf["cluster_min_pts"].GetInt());
        }
        if (lf.HasMember("cluster_min_component_pts") &&
            lf["cluster_min_component_pts"].IsInt()) {
            label_fusion_cfg_.cluster_min_component_pts =
                std::max(1, lf["cluster_min_component_pts"].GetInt());
        }
        label_fusion_cfg_.seg_mask_postprocess = readJsonBool(
            lf, "seg_mask_postprocess", label_fusion_cfg_.seg_mask_postprocess);
        if (lf.HasMember("seg_mask_majority_radius") &&
            lf["seg_mask_majority_radius"].IsInt()) {
            label_fusion_cfg_.seg_mask_majority_radius =
                std::max(0, lf["seg_mask_majority_radius"].GetInt());
        }
        if (lf.HasMember("seg_mask_close_radius") &&
            lf["seg_mask_close_radius"].IsInt()) {
            label_fusion_cfg_.seg_mask_close_radius =
                std::max(0, lf["seg_mask_close_radius"].GetInt());
        }
        if (lf.HasMember("seg_mask_close_iterations") &&
            lf["seg_mask_close_iterations"].IsInt()) {
            label_fusion_cfg_.seg_mask_close_iterations =
                std::max(1, lf["seg_mask_close_iterations"].GetInt());
        }
        label_fusion_cfg_.seg_cluster_filter = readJsonBool(
            lf, "seg_cluster_filter", label_fusion_cfg_.seg_cluster_filter);
        if (lf.HasMember("seg_mask_erode_px") && lf["seg_mask_erode_px"].IsInt()) {
            label_fusion_cfg_.seg_mask_erode_px =
                std::max(0, lf["seg_mask_erode_px"].GetInt());
        }
        label_fusion_cfg_.seg_depth_gap_m = ReadJsonFloat(lf, "seg_depth_gap_m", label_fusion_cfg_.seg_depth_gap_m);
        if (lf.HasMember("seg_ray_bin_px") && lf["seg_ray_bin_px"].IsInt()) {
            label_fusion_cfg_.seg_ray_bin_px =
                std::max(1, lf["seg_ray_bin_px"].GetInt());
        }
        if (lf.HasMember("seg_min_component_pts") &&
            lf["seg_min_component_pts"].IsInt()) {
            label_fusion_cfg_.seg_min_component_pts =
                std::max(1, lf["seg_min_component_pts"].GetInt());
        }
    }
  if (!dnn_cfg_.enabled) label_fusion_cfg_.enabled = false;
  if (!dnn_cfg_.detect.enabled && !dnn_cfg_.seg.enabled) {
    label_fusion_cfg_.enabled = false;
  }
}

void PercepDemo::ensureLeftYuvFormat() {
  if (left_yuv_format_known_) return;
  const std::string& fmt = dnn_cfg_.left_yuv_format;
  left_yuv_is_nv12_ = (fmt != "i420");
  left_yuv_format_known_ = true;
  std::cout << "[PercepDemo][yuv] locked="
            << (left_yuv_is_nv12_ ? "nv12" : "i420")
            << " source=config" << std::endl;
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
  // 检测框+分割着色统一发 /percep_demo/dnn_overlay；不再发 seg_mask/det_markers/labeled_cloud
  det_match_cfg_.publish_markers = false;
  label_fusion_cfg_.publish_debug_cloud = false;

  if (dnn_cfg_.save_overlay && !dnn_cfg_.save_overlay_dir.empty()) {
    if (!MkDirRecursive(dnn_cfg_.save_overlay_dir)) throw std::runtime_error("cannot create overlay directory");
  }

  dnn_runner_.reset(new r28::product::ExactDnnRunner(product_exact_profiles_));
  if (!dnn_runner_->init(dnn_cfg_)) {
    std::cerr << "[PercepDemo] HorizonDnnRunner init failed, disable dnn"
              << std::endl;
    const auto status=dnn_runner_->lastStatus();
    dnn_runner_.reset(); throw r28::Failure(status);
  }
  dnn_cfg_.seg = dnn_runner_->config().seg;
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

void PercepDemo::runDnnInference(const PicData& meta, const uint8_t* yuv,
                                 bool run_seg) {
  if (shutting_down_.load(std::memory_order_acquire)) return;
  last_dnn_dets_.clear();
  last_dnn_seg_mask_.clear();
  last_dnn_img_w_ = 0;
  last_dnn_img_h_ = 0;
  if (!dnn_runner_) {
    throw r28::Failure({1011,0,"product.ai.exact","dnn","product DNN runner is unavailable",product_frame_});
  }

  const int img_w = meta.width > 0 ? static_cast<int>(meta.width) : WIDTH;
  const int img_h = meta.height > 0 ? static_cast<int>(meta.height) : HEIGHT;
  const size_t bytes_used =
      meta.bytes_used > 0 ? static_cast<size_t>(meta.bytes_used)
                          : static_cast<size_t>(PIC_SIZE);
  // cmd=2 可能只有 GDC 1280；YOLO letterbox 可吃 640 或 1280
  const size_t max_yuv = (img_w >= GDC_WIDTH) ? static_cast<size_t>(GDC_PIC_SIZE)
                                              : static_cast<size_t>(PIC_SIZE);
  perception::FrameInfo frame; frame.id=meta.frame_id; frame.timestamp_ns=meta.time_ns;
  const int stride_y = r28::product::productYuvStride(
      yuv,img_w,img_h,bytes_used,max_yuv,frame);

  // run_every_n 固定为 1（业务要求每帧推理，勿改为节流手段）
  const int every = 1;
  (void)dnn_cfg_.run_every_n;
  if (++dnn_run_tick_ % static_cast<uint64_t>(every) != 0) return;

  ensureLeftYuvFormat();
  dnn_runner_->setFrameInfo(frame);

  std::vector<DnnDetection> detections;
  std::vector<uint8_t> seg_mask;
  DnnInferTiming timing;
  std::vector<DnnDetection>* det_ptr =
      dnn_cfg_.detect.enabled ? &detections : nullptr;
  std::vector<uint8_t>* seg_ptr =
      (run_seg && dnn_cfg_.seg.enabled) ? &seg_mask : nullptr;
  // 推理不填 overlay（避免 runner 黑底）；底图用已验证的 nv12→rgb24
  if (!dnn_runner_->inferNv12(
          yuv, img_w, img_h, stride_y, bytes_used, left_yuv_is_nv12_, det_ptr,
          seg_ptr, /*overlay_rgb=*/nullptr, &timing)) {
    throw r28::Failure(dnn_runner_->lastStatus());
  }

  if (run_seg && dnn_cfg_.seg.enabled && !seg_mask.empty()) {
    postprocessSegMask(seg_mask, img_w, img_h);
  }

  last_dnn_dets_ = std::move(detections);
  const size_t raw_det_n = last_dnn_dets_.size();
  // 模型原始检测结果（keep_classes 过滤前）写入 LOG
  {
    std::ostringstream oss;
    oss << "[PercepDemo][det] frame=" << frame_count_ << " raw_n=" << raw_det_n
        << " " << img_w << "x" << img_h;
    constexpr size_t kMaxLogDets = 12;
    for (size_t i = 0; i < last_dnn_dets_.size() && i < kMaxLogDets; ++i) {
      const auto& d = last_dnn_dets_[i];
      oss << " ["
          << (d.class_name.empty() ? ("c" + std::to_string(d.class_id))
                                   : d.class_name)
          << " id=" << d.class_id
          << " conf=" << static_cast<int>(d.confidence * 100.f + 0.5f) << "%"
          << " xyxy=" << static_cast<int>(d.x1) << ","
          << static_cast<int>(d.y1) << "," << static_cast<int>(d.x2) << ","
          << static_cast<int>(d.y2) << "]";
    }
    if (last_dnn_dets_.size() > kMaxLogDets) {
      oss << " ...+" << (last_dnn_dets_.size() - kMaxLogDets);
    }
    const std::string line = oss.str();
    LOG_INFO("%s", line.c_str());
    std::cout << line << std::endl;
  }
  // 只保留 keep_classes（默认 person/cat/dog/charger）；空串=不过滤
  if (!dnn_cfg_.detect.keep_classes.empty() && !last_dnn_dets_.empty()) {
    std::unordered_set<std::string> keep;
    {
      std::string s = dnn_cfg_.detect.keep_classes;
      size_t start = 0;
      while (start <= s.size()) {
        size_t comma = s.find(',', start);
        std::string tok =
            (comma == std::string::npos) ? s.substr(start) : s.substr(start, comma - start);
        // trim
        while (!tok.empty() && (tok.front() == ' ' || tok.front() == '\t'))
          tok.erase(tok.begin());
        while (!tok.empty() && (tok.back() == ' ' || tok.back() == '\t'))
          tok.pop_back();
        if (!tok.empty()) keep.insert(tok);
        if (comma == std::string::npos) break;
        start = comma + 1;
      }
    }
    if (!keep.empty()) {
      std::vector<DnnDetection> filtered;
      filtered.reserve(last_dnn_dets_.size());
      for (auto& d : last_dnn_dets_) {
        if (keep.count(d.class_name)) filtered.push_back(std::move(d));
      }
      last_dnn_dets_.swap(filtered);
    }
  }
  last_dnn_seg_mask_ = std::move(seg_mask);
  last_dnn_img_w_ = img_w;
  last_dnn_img_h_ = img_h;

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
    const bool need_full_rgb =
        run_seg && dnn_cfg_.overlay_blend_seg && dnn_cfg_.seg.enabled &&
        !last_dnn_seg_mask_.empty();
    const std::vector<uint8_t>* pub_buf = &dnn_overlay_buf_;
    static std::vector<uint8_t> overlay_scaled_;
    std::vector<DnnDetection> overlay_dets = last_dnn_dets_;
    bool ok = false;
    if (need_full_rgb) {
      // 叠分割时必须全分辨率 RGB，避免掩码边缘被预缩放弄锯齿
      ok = pclCommonPtr->yuv_nv12_to_rgb24(yuv, img_w, img_h, stride_y,
                                           &dnn_overlay_buf_);
      if (ok) {
        BlendSegMaskOnRgb(dnn_overlay_buf_.data(), img_w, img_h,
                          last_dnn_seg_mask_, dnn_cfg_.overlay_seg_alpha,
                          dnn_cfg_.seg.class_names);
        DrawDetBoxesOnRgb(dnn_overlay_buf_.data(), img_w, img_h,
                          last_dnn_dets_);
        if (pub_w != img_w || pub_h != img_h) {
          if (pclCommonPtr->rgb8_resize(dnn_overlay_buf_, img_w, img_h,
                                        overlay_scaled_, pub_w, pub_h)) {
            pub_buf = &overlay_scaled_;
          } else {
            pub_w = img_w;
            pub_h = img_h;
            pub_buf = &dnn_overlay_buf_;
          }
        }
      }
    } else if (pub_w != img_w || pub_h != img_h) {
      // 仅可视化：在发布分辨率取样 NV12，不改 det_cloud / seg_mask
      Nv12NearestToRgb24(yuv, img_w, img_h, stride_y, pub_w, pub_h,
                         &overlay_scaled_);
      ScaleDetsToOverlay(last_dnn_dets_, img_w, img_h, pub_w, pub_h,
                         &overlay_dets);
      DrawDetBoxesOnRgb(overlay_scaled_.data(), pub_w, pub_h, overlay_dets);
      pub_buf = &overlay_scaled_;
      ok = true;
    } else {
      ok = pclCommonPtr->yuv_nv12_to_rgb24(yuv, img_w, img_h, stride_y,
                                           &dnn_overlay_buf_);
      if (ok) {
        DrawDetBoxesOnRgb(dnn_overlay_buf_.data(), img_w, img_h,
                          last_dnn_dets_);
      }
    }
    overlay_ms = std::chrono::duration<double, std::milli>(clock::now() - t_ov)
                     .count();
    if (!ok) {
      std::cerr << "[PercepDemo] dnn overlay nv12→rgb failed" << std::endl;
      r28::Status status;status.code=2002;status.feature="product.output.ai_overlay";
      status.detail="AI overlay RGB conversion failed";throw r28::Failure(status);
    } else {
      if (dnn_cfg_.publish_overlay) {
        const auto t_pub = clock::now();
        sensor_msgs::msg::Image msg;
        msg.header.stamp = meta.time_ns > 0
                               ? rclcpp::Time(static_cast<int64_t>(meta.time_ns),
                                              RCL_ROS_TIME)
                               : rclcpp::Clock().now();
        msg.header.frame_id = "percep_demo";
        msg.encoding = "rgb8";
        msg.is_bigendian = false;
        msg.width = static_cast<uint32_t>(pub_w);
        msg.height = static_cast<uint32_t>(pub_h);
        msg.step = static_cast<uint32_t>(pub_w * 3);
        msg.data = *pub_buf;
        try {dnn_overlay_pub_.publish(msg);}
        catch(const std::exception& error) {
          throw r28::Failure(r28::statusFromException(error,2003,"product.output.ai_overlay"));
        }
        overlay_pub_ms =
            std::chrono::duration<double, std::milli>(clock::now() - t_pub)
                .count();
      }
      if (dnn_cfg_.save_overlay) {
        const auto t_sv = clock::now();
        MaybeSaveDnnOverlay(pub_buf->data(), pub_w, pub_h, frame_save_time_str_,
                            dnn_cfg_);
        overlay_save_ms =
            std::chrono::duration<double, std::milli>(clock::now() - t_sv)
                .count();
      }
    }
  }

  // 每次推理写 LOG（带 fps / keep 后数量）；便于确认新 so 与过滤结果
  {
    using clock = std::chrono::steady_clock;
    static clock::time_point win_t0 = clock::now();
    static uint64_t win_n = 0;
    static double fps_ema = 0.0;
    ++win_n;
    const auto now = clock::now();
    const double dt = std::chrono::duration<double>(now - win_t0).count();
    if (dt >= 1.0) {
      fps_ema = static_cast<double>(win_n) / dt;
      win_n = 0;
      win_t0 = now;
    }
    std::ostringstream oss;
    oss << "[PercepDemo] dnn frame=" << frame_count_
        << " det_ms=" << static_cast<int>(timing.detect_ms)
        << " det_pre=" << static_cast<int>(timing.detect_pre_ms)
        << " det_bpu=" << static_cast<int>(timing.detect_bpu_ms)
        << " seg_ms=" << static_cast<int>(timing.seg_ms)
        << " seg_pre=" << static_cast<int>(timing.seg_pre_ms)
        << " seg_bpu=" << static_cast<int>(timing.seg_bpu_ms)
        << " seg_post=" << static_cast<int>(timing.seg_post_ms)
        << " overlay_ms=" << static_cast<int>(overlay_ms)
        << " overlay_pub_ms=" << static_cast<int>(overlay_pub_ms)
        << " overlay_save_ms=" << static_cast<int>(overlay_save_ms)
        << " fps=" << static_cast<int>(fps_ema + 0.5)
        << " raw_n=" << raw_det_n << " keep_n=" << last_dnn_dets_.size();
    for (size_t i = 0; i < last_dnn_dets_.size() && i < 8; ++i) {
      const auto& d = last_dnn_dets_[i];
      oss << " ["
          << (d.class_name.empty() ? std::to_string(d.class_id) : d.class_name)
          << ":" << static_cast<int>(d.confidence * 100) << "%]";
    }
    if (last_dnn_dets_.size() > 8) oss << " ...";
    const std::string line = oss.str();
    LOG_INFO("%s", line.c_str());
    std::cout << line << std::endl;
  }
}

void PercepDemo::matchAndPublishDetCloud(const PointCloudT::Ptr& cloud,
                                         float fx, float fy, float cx,
                                         float cy, int img_w, int img_h) {
  if (label_fusion_cfg_.enabled) {
    if (img_w <= 0) img_w = last_dnn_img_w_;
    if (img_h <= 0) img_h = last_dnn_img_h_;
    fuseSemanticLabels(cloud, last_dnn_dets_, last_dnn_seg_mask_,
                       last_dnn_img_w_, last_dnn_img_h_, img_w, img_h);
    return;
  }
  if (!det_match_cfg_.enabled && !det_match_cfg_.publish_markers) return;
  if (!cloud || cloud->empty()) {
    if (det_match_cfg_.enabled) {
      PointCloudT::Ptr empty(new PointCloudT);
      last_semantic_cloud_ = empty;
      publish_ros_rgbcloud(empty, det_cloud_pub_, parent_frame, false);
    }
    if (det_match_cfg_.publish_markers) {
      visualization_msgs::msg::MarkerArray clear;
      visualization_msgs::msg::Marker m;
      m.header.frame_id = parent_frame;
      m.header.stamp = rclcpp::Clock().now();
      m.ns = "det";
      m.id = 0;
      m.action = visualization_msgs::msg::Marker::DELETEALL;
      clear.markers.push_back(m);
      det_markers_pub_.publish(clear);
    }
    return;
  }
  if (last_dnn_dets_.empty()) {
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
    if (det_match_cfg_.publish_markers) {
      visualization_msgs::msg::MarkerArray clear;
      visualization_msgs::msg::Marker m;
      m.header.frame_id = parent_frame;
      m.header.stamp = rclcpp::Clock().now();
      m.ns = "det";
      m.id = 0;
      m.action = visualization_msgs::msg::Marker::DELETEALL;
      clear.markers.push_back(m);
      det_markers_pub_.publish(clear);
    }
    return;
  }
  if (img_w <= 0) img_w = last_dnn_img_w_;
  if (img_h <= 0) img_h = last_dnn_img_h_;
  if (img_w <= 0 || img_h <= 0 || fx <= 1e-6f) return;

  const float score_min = det_match_cfg_.score_min;
  // max_depth_m<=0：不截断（点云范围由 point_cloud_pub.box_* 约束）；
  // >0：相机系深度上限。旧默认 0.6m 会丢掉 2m+ 检测障碍。
  const bool limit_depth = det_match_cfg_.max_depth_m > 0.05f;
  const float max_depth = limit_depth ? det_match_cfg_.max_depth_m : 1e9f;
  std::vector<std::vector<size_t>> box_pts(last_dnn_dets_.size());
  std::vector<std::vector<float>> box_z(last_dnn_dets_.size());

  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto& p = cloud->points[i];
    if (p.z <= 0.05f || p.z > max_depth) continue;
    const float u = fx * (p.x / p.z) + cx;
    const float v = fy * (p.y / p.z) + cy;
    if (u < 0 || v < 0 || u >= img_w || v >= img_h) continue;

    int best = -1;
    float best_conf = score_min;
    for (size_t di = 0; di < last_dnn_dets_.size(); ++di) {
      const auto& d = last_dnn_dets_[di];
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
  std::vector<float> zmin(last_dnn_dets_.size(), -1e9f);
  std::vector<float> zmax(last_dnn_dets_.size(), 1e9f);
  for (size_t d = 0; d < last_dnn_dets_.size(); ++d) {
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
  std::vector<Eigen::Vector3f> centers(last_dnn_dets_.size(),
                                       Eigen::Vector3f::Zero());
  std::vector<int> center_n(last_dnn_dets_.size(), 0);

  for (size_t di = 0; di < last_dnn_dets_.size(); ++di) {
    for (size_t idx : box_pts[di]) {
      auto& p = det_cloud->points[idx];
      if (p.z < zmin[di] || p.z > zmax[di]) continue;
      const int cid = last_dnn_dets_[di].class_id;
      if (cid < 0) continue;
      const auto& col = kDetPalette[(cid >= 0 ? cid : 0) % 12];
      p.r = col[0];
      p.g = col[1];
      p.b = col[2];
      p.label = EncodeDetLabel(cid);
      p.confidence = last_dnn_dets_[di].confidence;
      centers[di] += Eigen::Vector3f(p.x, p.y, p.z);
      center_n[di] += 1;
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

  if (det_match_cfg_.publish_markers) {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = parent_frame;
    clear.header.stamp = rclcpp::Clock().now();
    clear.ns = "det";
    clear.id = 0;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    Eigen::Matrix4f Tm = Eigen::Matrix4f::Identity();
    if (pclCommonPtr) {
      pclCommonPtr->EulerToMatrix(cali, Tm);
    }

    for (size_t di = 0; di < last_dnn_dets_.size(); ++di) {
      if (center_n[di] <= 0) continue;
      Eigen::Vector3f c = centers[di] / static_cast<float>(center_n[di]);
      Eigen::Vector4f ch(c.x(), c.y(), c.z(), 1.f);
      Eigen::Vector4f cw = Tm * ch;
      // publish_ros_rgbcloud 还会做 mapWidth 平移；marker 与 TF 子系 percep_demo 对齐
      const float ros_x = cw.x();
      const float ros_y = cw.y();
      const float ros_z = cw.z();
      const auto& d = last_dnn_dets_[di];
      const auto& col = kDetPalette[(d.class_id >= 0 ? d.class_id : 0) % 12];

      visualization_msgs::msg::Marker sphere;
      sphere.header.frame_id = "percep_demo";
      sphere.header.stamp = clear.header.stamp;
      sphere.ns = "det";
      sphere.id = static_cast<int>(di) + 1;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position.x = ros_x;
      sphere.pose.position.y = ros_y;
      sphere.pose.position.z = ros_z;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.12;
      sphere.color.r = col[0] / 255.f;
      sphere.color.g = col[1] / 255.f;
      sphere.color.b = col[2] / 255.f;
      sphere.color.a = 0.85f;
      sphere.lifetime = rclcpp::Duration::from_seconds(0.5);
      arr.markers.push_back(sphere);

      visualization_msgs::msg::Marker text = sphere;
      text.id = static_cast<int>(di) + 1000;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.pose.position.z = ros_z + 0.15;
      text.scale.x = text.scale.y = 0.0;
      text.scale.z = 0.12;
      std::ostringstream oss;
      oss << (d.class_name.empty() ? ("c" + std::to_string(d.class_id))
                                   : d.class_name)
          << " " << static_cast<int>(d.confidence * 100) << "%"
          << " n=" << center_n[di];
      text.text = oss.str();
      arr.markers.push_back(text);
    }
    det_markers_pub_.publish(arr);
  }

  std::cout << "[PercepDemo] det_cloud pts=" << det_cloud->size()
            << " boxes=" << last_dnn_dets_.size() << std::endl;
}

}  // namespace ai_node_percep_demo

#endif  // PERCEP_DNN_ENABLE
