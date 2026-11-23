/**
 * @file dnn_runtime.cpp
 * @brief obs/detection：仿 obs/segmentation 接入 Horizon 检测，发布检测结果与匹配点云
 */
#include "percep_main/percep_main.h"


#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <errno.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <exception>
#include <memory>
#include <sys/stat.h>
#include <sys/types.h>
#include <unordered_map>
#include <unordered_set>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include "det_cloud.h"
#include "third_party/stb_image_write.h"

namespace perception {

namespace {
bool readJsonBool(const rapidjson::Value& obj, const char* key,
                            bool default_val) {
    if (!obj.HasMember(key) || !obj[key].IsBool()) return default_val;
    return obj[key].GetBool();
}
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

// 5x7 点阵（bit4=左列 … bit0=右列），覆盖 a-z / 0-9 / _- 和 .
// 不依赖 OpenCV putText，避免板端叠加不可见
// 空格及其它不支持字符由 draw_glyph 的 gi<0 分支留白并照常推进位置
static const uint8_t kFont5x7[39][7] = {
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
    // .  （置信度小数点，底部 2x2）
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C},
};

inline int FontGlyphIndex(char ch) {
  if (ch >= '0' && ch <= '9') return ch - '0';
  if (ch >= 'a' && ch <= 'z') return 10 + (ch - 'a');
  if (ch >= 'A' && ch <= 'Z') return 10 + (ch - 'A');
  if (ch == '_') return 36;
  if (ch == '-') return 37;
  if (ch == '.') return 38;
  return -1;   // 含空格：draw_glyph 留白，但位置照常推进
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

// 框内文字内容：类名 + 置信度。位图字体没有 '%'，用 "person 0.87" 形式。
std::string FormatDetLabel(const DnnDetection& det) {
  std::string s = det.class_name.empty()
                      ? ("c" + std::to_string(det.class_id))
                      : det.class_name;
  float c = det.confidence;
  if (c < 0.f) c = 0.f;
  if (c > 1.f) c = 1.f;
  char buf[16];
  std::snprintf(buf, sizeof(buf), " %.2f", static_cast<double>(c));
  s += buf;
  return s;
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

    // 类名+置信度，白字黑描边画在框内左上角（框贴顶也能看到）
    DrawLabelInsideBox(rgb, w, h, x1, y1, x2, y2, FormatDetLabel(det));
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

bool PercepMain::parseDnnConfig(const rapidjson::Value& root) {
  if (!root.HasMember("dnn") || !root["dnn"].IsObject()) {
    return true;
  }
  const rapidjson::Value& dnn = root["dnn"];
  dnn_cfg_.enabled = readJsonBool(dnn, "enabled", dnn_cfg_.enabled);
  {
    std::string error;
    if (!ParseDnnPreprocessConfig(dnn, &dnn_preprocess_cfg_, &error)) {
      LOG_ERROR("[PercepMain][preprocess] config error: %s", error.c_str());
      return false;
    }
  }
  if (dnn.HasMember("run_every_n") && dnn["run_every_n"].IsInt()) {
    dnn_cfg_.run_every_n = std::max(1, dnn["run_every_n"].GetInt());
  }
  // 可视化话题开关统一受 debug.visualize 管控（配置文件中不再单独配置）：
  // publish_overlay / publish_det_cloud / publish_det_markers / publish_debug_cloud
  dnn_cfg_.publish_overlay = visualize;
  dnn_cfg_.publish_seg_mask = false;  // 统一叠到 dnn_overlay，不再单独发 seg_mask
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

  det_match_cfg_.enabled = visualize;
  det_match_cfg_.publish_markers = false;  // 统一发 dnn_overlay，不再单独发 det_markers
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
    {
      std::string error;
      if (!ParseDnnResizeType(det, "dnn.detect", dnn_cfg_.detect.resize_type,
                              &dnn_cfg_.detect.resize_type, &error)) {
        LOG_ERROR("[PercepMain][preprocess] mode config error: %s", error.c_str());
        return false;
      }
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
    {
      std::string error;
      if (!ParseDnnResizeType(seg, "dnn.seg", dnn_cfg_.seg.resize_type,
                              &dnn_cfg_.seg.resize_type, &error)) {
        LOG_ERROR("[PercepMain][preprocess] mode config error: %s", error.c_str());
        return false;
      }
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
          LOG_WARN("[PercepMain] dnn.seg.num_classes=%d != class_names count=%d, "
                   "use class_names; tensor will confirm at init",
                   seg["num_classes"].GetInt(), from_names);
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
    LOG_WARN("[PercepMain] dnn.enabled but detect/seg both off");
  }
  if (dnn_cfg_.enabled && dnn_cfg_.detect.enabled &&
      dnn_cfg_.detect.model_path.empty()) {
    LOG_ERROR("[PercepMain] detect enabled but model_path empty");
    return false;
  }
  if (dnn_cfg_.enabled && dnn_cfg_.seg.enabled &&
      dnn_cfg_.seg.model_path.empty()) {
    LOG_ERROR("[PercepMain] seg enabled but model_path empty");
    return false;
  }
  return true;
}


void PercepMain::parseLabelFusionConfig(const rapidjson::Value& root) {
  if (root.HasMember("label_fusion") &&
        root["label_fusion"].IsObject()) {
        const rapidjson::Value& lf = root["label_fusion"];
        label_fusion_cfg_.enabled =
            readJsonBool(lf, "enabled", label_fusion_cfg_.enabled);
        label_fusion_cfg_.fuse_into_grid =
            readJsonBool(lf, "fuse_into_grid", label_fusion_cfg_.fuse_into_grid);
        label_fusion_cfg_.publish_debug_cloud = false;  // 统一发 dnn_overlay，不再单独发 labeled_cloud
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
                LOG_WARN("[PercepMain][label_fusion] unknown match_downsample=\"%s\", "
                         "fallback stride", m.c_str());
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
  // 可视化话题仅受 debug.visualize 管控：visualize=true 才发 overlay/det_cloud；
  // det_markers/labeled_cloud 统一叠到 dnn_overlay，永久关闭。
  dnn_cfg_.publish_overlay = visualize;
  det_match_cfg_.enabled = visualize;
  det_match_cfg_.publish_markers = false;
  label_fusion_cfg_.publish_debug_cloud = false;
  if (!dnn_cfg_.enabled) label_fusion_cfg_.enabled = false;
  if (!dnn_cfg_.detect.enabled && !dnn_cfg_.seg.enabled) {
    label_fusion_cfg_.enabled = false;
  }
}

void PercepMain::ensureLeftYuvFormat() {
  if (left_yuv_format_known_) return;
  const std::string& fmt = dnn_cfg_.left_yuv_format;
  left_yuv_is_nv12_ = (fmt != "i420");
  left_yuv_format_known_ = true;
  LOG_INFO("[PercepMain][yuv] locked=%s source=config",
           left_yuv_is_nv12_ ? "nv12" : "i420");
}

void PercepMain::initDnnRuntime() {
  if (!dnn_cfg_.enabled) {
    LOG_INFO("[PercepMain] dnn disabled");
    return;
  }
  if (dnn_cfg_.publish_overlay) {
    // 可视化图像用 Best Effort，避免 Reliable/SystemDefault 堆积导致 RViz 延迟
    const auto overlay_qos =
        rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
    dnn_overlay_pub_ = daye::CreatePublish<sensor_msgs::msg::Image>(
        "/percep_main/dnn_overlay", overlay_qos);
    LOG_INFO("[PercepMain] dnn_overlay qos=best_effort depth=5");
  }
  if (det_match_cfg_.enabled) {
    det_cloud_pub_ = daye::CreatePublish<sensor_msgs::msg::PointCloud2>(
        "/percep_main/det_cloud", rclcpp::SensorDataQoS());
  }
  // visualize 已在解析阶段同步到各开关；此处再强制关停已下线话题
  det_match_cfg_.publish_markers = false;
  label_fusion_cfg_.publish_debug_cloud = false;

  // Keep the existing wrapper contract while replacing the implementation
  // with ds_create_ex and ds_process[_parallel] from det_seg_cloud_sdk_x5.
  dnn_runner_.reset(new DetSegCloudAdapter());
  if (!dnn_runner_->init(dnn_cfg_)) {
    LOG_ERROR("[PercepMain][det_seg_cloud] init failed: %s; disable dnn", dnn_runner_->lastError().c_str());
    dnn_cfg_.enabled = false;
    dnn_runner_.reset();
    return;
  }
  {
    int det_w = 0, det_h = 0, seg_w = 0, seg_h = 0;
    std::string error;
    if ((dnn_cfg_.detect.enabled && !dnn_runner_->modelInputSize(DS_DETECTION, &det_w, &det_h, &error)) ||
        (dnn_cfg_.seg.enabled && !dnn_runner_->modelInputSize(DS_SEGMENTATION, &seg_w, &seg_h, &error)) ||
        !ValidateDnnPreprocessModelSizes(dnn_preprocess_cfg_,
            dnn_cfg_.detect.enabled, det_w, det_h, dnn_cfg_.seg.enabled, seg_w, seg_h, &error)) {
      LOG_ERROR("[PercepMain][preprocess] model size validation failed: %s", error.c_str());
      dnn_cfg_.enabled = false;
      dnn_runner_.reset();
      return;
    }
    LOG_INFO("[PercepMain][preprocess] source_expected=%dx%d (0x0=auto) "
             "det_output=%dx%d seg_output=%dx%d (actual BIN inputs)",
             dnn_preprocess_cfg_.source_width, dnn_preprocess_cfg_.source_height,
             det_w, det_h, seg_w, seg_h);
  }
  dnn_cfg_.seg = dnn_runner_->config().seg;
  dnn_cfg_.parallel_det_seg = dnn_runner_->config().parallel_det_seg;
  if (label_fusion_cfg_.enabled) {
    initLabelFusionClassIds();
  }
  LOG_INFO("[PercepMain] dnn ready detect=%s seg=%s det_model=%s seg_model=%s "
           "seg_classes=%d [%s] overlay=%s det_cloud=%s label_fusion=%s "
           "fuse_grid=%s overlay_size=%s",
           dnn_cfg_.detect.enabled ? "on" : "off",
           dnn_cfg_.seg.enabled ? "on" : "off",
           dnn_cfg_.detect.model_path.c_str(),
           dnn_cfg_.seg.model_path.c_str(),
           dnn_cfg_.seg.num_classes,
           dnn_cfg_.seg.class_names.c_str(),
           dnn_cfg_.publish_overlay ? "on" : "off",
           det_match_cfg_.enabled ? "on" : "off",
           label_fusion_cfg_.enabled ? "on" : "off",
           label_fusion_cfg_.fuse_into_grid ? "on" : "off",
           dnn_cfg_.overlay_use_original_resolution
               ? "orig"
               : (std::to_string(dnn_cfg_.overlay_pub_width) + "x" +
                  std::to_string(dnn_cfg_.overlay_pub_height))
                  .c_str());
}

void PercepMain::runDnnInference(const PicData& meta, const uint8_t* yuv,
                                 bool run_seg) {
  last_dnn_dets_.clear();
  last_dnn_raw_dets_.clear();
  last_dnn_seg_mask_.clear();
  last_dnn_img_w_ = 0;
  last_dnn_img_h_ = 0;
  if (!dnn_runner_ || !yuv) return;

  const int img_w = meta.width > 0 ? static_cast<int>(meta.width) : WIDTH;
  const int img_h = meta.height > 0 ? static_cast<int>(meta.height) : HEIGHT;
  const size_t bytes_used =
      meta.bytes_used > 0 ? static_cast<size_t>(meta.bytes_used)
                          : static_cast<size_t>(PIC_SIZE);
  // cmd=2 可能只有 GDC 1280；YOLO letterbox 可吃 640 或 1280
  const size_t max_yuv = (img_w >= GDC_WIDTH) ? static_cast<size_t>(GDC_PIC_SIZE)
                                              : static_cast<size_t>(PIC_SIZE);
  if (img_w <= 0 || img_h <= 0 || bytes_used == 0 || bytes_used > max_yuv) {
    return;
  }

  {
    std::string error;
    if (!ValidateDnnPreprocessSource(dnn_preprocess_cfg_, img_w, img_h, &error)) {
      LOG_ERROR("[PercepMain][preprocess] source size validation failed: %s", error.c_str());
      return;
    }
  }

  const int stride_y = static_cast<int>(bytes_used / (img_h * 1.5f));
  if (stride_y < img_w) return;

  // run_every_n 固定为 1（业务要求每帧推理，勿改为节流手段）
  const int every = 1;
  (void)dnn_cfg_.run_every_n;
  if (++dnn_run_tick_ % static_cast<uint64_t>(every) != 0) return;

  ensureLeftYuvFormat();

  std::vector<DnnDetection> detections;
  std::vector<uint8_t> seg_mask;
  DnnInferTiming timing;
  std::vector<DnnDetection>* det_ptr =
      dnn_cfg_.detect.enabled ? &detections : nullptr;
  std::vector<uint8_t>* seg_ptr =
      (run_seg && dnn_cfg_.seg.enabled) ? &seg_mask : nullptr;
  // Adapter uses the SDK serial/parallel entry and copies borrowed outputs before releasing
  // ds_result. Overlay remains a separate application-level operation.
  if (!dnn_runner_->inferNv12(
          yuv, img_w, img_h, stride_y, bytes_used, left_yuv_is_nv12_, det_ptr,
          seg_ptr, /*overlay_rgb=*/nullptr, &timing)) {
    LOG_ERROR("[PercepMain][det_seg_cloud] inference failed: %s", dnn_runner_->lastError().c_str());
    return;
  }

  if (run_seg && dnn_cfg_.seg.enabled && !seg_mask.empty()) {
    postprocessSegMask(seg_mask, img_w, img_h);
  }

  last_dnn_dets_ = std::move(detections);
  const size_t raw_det_n = last_dnn_dets_.size();
  // 存档需要模型全部类别的原始结果，必须在 keep_classes 过滤前留一份副本。
  last_dnn_raw_dets_ = last_dnn_dets_;
  // 模型原始检测结果（keep_classes 过滤前）写入 LOG
  {
    std::ostringstream oss;
    oss << "[PercepMain][det] frame=" << frame_count_ << " raw_n=" << raw_det_n
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

  // overlay 渲染只服务于实时发布：publish_overlay 开时才在主循环生成，
  // 纯存档场景把原料塞进 SaveMapTask，着色画框挪到 saveLogLoop 异步做
  dnn_overlay_w_ = 0;
  dnn_overlay_h_ = 0;
  double overlay_ms = 0;
  double overlay_pub_ms = 0;
  const bool want_overlay =
      dnn_cfg_.publish_overlay && pclCommonPtr;
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
        // 掩码要在全分辨率上混合（保边缘质量），但框线和文字必须画在**发布
        // 分辨率**上：先画后缩会把 5x7 位图字重采样成一团糊，类名和置信度
        // 就读不出来了。所以顺序是 混合 → 缩放 → 按缩放后坐标画框画字。
        bool scaled = false;
        if (pub_w != img_w || pub_h != img_h) {
          if (pclCommonPtr->rgb8_resize(dnn_overlay_buf_, img_w, img_h,
                                        overlay_scaled_, pub_w, pub_h)) {
            pub_buf = &overlay_scaled_;
            scaled = true;
          } else {
            pub_w = img_w;
            pub_h = img_h;
            pub_buf = &dnn_overlay_buf_;
          }
        }
        if (scaled) {
          ScaleDetsToOverlay(last_dnn_dets_, img_w, img_h, pub_w, pub_h,
                             &overlay_dets);
          DrawDetBoxesOnRgb(overlay_scaled_.data(), pub_w, pub_h, overlay_dets);
        } else {
          DrawDetBoxesOnRgb(dnn_overlay_buf_.data(), img_w, img_h,
                            last_dnn_dets_);
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
      LOG_ERROR("[PercepMain] dnn overlay nv12→rgb failed");
    } else {
      dnn_overlay_w_ = pub_w;
      dnn_overlay_h_ = pub_h;
      if (dnn_cfg_.publish_overlay) {
        const auto t_pub = clock::now();
        sensor_msgs::msg::Image msg;
        msg.header.stamp = meta.time_ns > 0
                               ? rclcpp::Time(static_cast<int64_t>(meta.time_ns),
                                              RCL_ROS_TIME)
                               : rclcpp::Clock().now();
        msg.header.frame_id = "percep_main";
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
      // 存档快照：落盘图与发布图同源；只拷贝一次，批量存档错误到 run_compute
      last_dnn_overlay_save_ = *pub_buf;
      last_dnn_overlay_save_w_ = pub_w;
      last_dnn_overlay_save_h_ = pub_h;
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
    oss << std::fixed << std::setprecision(3)
        << "[PercepMain] dnn frame=" << frame_count_
        << " det_ms=" << timing.detect_ms
        << " det_pre=" << timing.detect_pre_ms
        << " det_bpu=" << timing.detect_bpu_ms
        << " det_post=" << timing.detect_post_ms
        << " seg_ms=" << timing.seg_ms
        << " seg_pre=" << timing.seg_pre_ms
        << " seg_bpu=" << timing.seg_bpu_ms
        << " seg_post=" << timing.seg_post_ms
        << " overlay_ms=" << static_cast<int>(overlay_ms)
        << " overlay_pub_ms=" << static_cast<int>(overlay_pub_ms)
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
  }
}

// 保留接口兼容性；存档数据由saveLogLoop以原始数据格式写盘。
void PercepMain::saveAiOverlayAsync(const SaveMapTask& task)
{
    (void)task;
    return;
#if 0
    const int img_w = task.dnn_img_w;
    if (img_w <= 0 || img_h <= 0 ||
        task.rgb.size() != static_cast<size_t>(img_w) * img_h * 3) {
        return;
    }
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
    std::vector<uint8_t> buf = task.rgb;  // 底图副本，不动任务数据
    const bool blend_seg = dnn_cfg_.overlay_blend_seg && dnn_cfg_.seg.enabled &&
                           !task.seg_mask.empty();
    if (blend_seg) {
        BlendSegMaskOnRgb(buf.data(), img_w, img_h, task.seg_mask,
                          dnn_cfg_.overlay_seg_alpha, dnn_cfg_.seg.class_names);
    }
    std::vector<uint8_t> scaled;
    std::vector<DnnDetection> dets = task.dets;
    uint8_t* out = buf.data();
    int out_w = img_w, out_h = img_h;
    if ((pub_w != img_w || pub_h != img_h) && pclCommonPtr &&
        pclCommonPtr->rgb8_resize(buf, img_w, img_h, scaled, pub_w, pub_h)) {
        out = scaled.data();
        out_w = pub_w;
        out_h = pub_h;
        ScaleDetsToOverlay(task.dets, img_w, img_h, pub_w, pub_h, &dets);
    }
    if (!dets.empty()) {
        DrawDetBoxesOnRgb(out, out_w, out_h, dets);
    }
    std::vector<uint8_t> out_buf(out, out + static_cast<size_t>(out_w) * out_h * 3);
    if (visualizePtr &&
        visualizePtr->saveOverlay(out_buf, out_w, out_h, task.time_str)) {
        pclCommonPtr->keepNewestFiles(output_dir + "/data/ai_det/", 30000);
        pclCommonPtr->keepNewestFiles(output_dir + "/data/ai_mask/", 30000);
    }
#endif
}

// Keep the original entry point; only its matching implementation uses SDK.
void PercepMain::matchAndPublishDetCloud(const PointCloudT::Ptr& cloud,
                                         const std::vector<uint32_t>& u_vec,
                                         const std::vector<uint32_t>& v_vec,
                                         float fx, float fy, float cx,
                                         float cy, int img_w, int img_h) {
  // The extended SDK accepts same-frame source pixel coordinates for fusion.
  // The box-only path still uses the original floating-point reprojection.

  auto clear_frame = [this](bool publish_empty) {
    // AiFusion::initData dereferences this pointer even if there are no points.
    // Never leave either the previous frame or a null pointer for the consumer.
    if (!last_semantic_cloud_) last_semantic_cloud_.reset(new PointCloudT);
    else last_semantic_cloud_->clear();
    if (publish_empty && det_match_cfg_.enabled) {
      publish_ros_rgbcloud(last_semantic_cloud_, det_cloud_pub_, parent_frame);
    }
    if (publish_empty && det_match_cfg_.publish_markers && !label_fusion_cfg_.enabled) {
      visualization_msgs::msg::MarkerArray clear;
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = parent_frame;
      marker.header.stamp = rclcpp::Clock().now();
      marker.ns = "det";
      marker.id = 0;
      marker.action = visualization_msgs::msg::Marker::DELETEALL;
      clear.markers.push_back(marker);
      det_markers_pub_.publish(clear);
    }
  };

  if (!label_fusion_cfg_.enabled && !det_match_cfg_.enabled &&
      !det_match_cfg_.publish_markers) {
    clear_frame(false);
    return;
  }
  if (!cloud || cloud->empty()) {
    clear_frame(true);
    return;
  }
  if (!pclCommonPtr) {
    LOG_ERROR("[PercepMain][det_cloud_sdk] missing camera-to-output transform helper");
    clear_frame(true);
    return;
  }

  // Fusion uses the segmentation grid; the legacy box-only path uses the
  // caller's image dimensions and the caller's unscaled intrinsics instead.
  const int source_w = img_w > 0 ? img_w : last_dnn_img_w_;
  const int source_h = img_h > 0 ? img_h : last_dnn_img_h_;
  const int grid_w = label_fusion_cfg_.enabled && last_dnn_img_w_ > 0
                         ? last_dnn_img_w_ : source_w;
  const int grid_h = label_fusion_cfg_.enabled && last_dnn_img_h_ > 0
                         ? last_dnn_img_h_ : source_h;
  if (grid_w <= 0 || grid_h <= 0 || grid_w > 4096 || grid_h > 4096 ||
      cloud->size() > 2000000) {
    LOG_ERROR("[PercepMain][det_cloud_sdk] invalid grid %dx%d or cloud size=%llu",
              grid_w, grid_h, static_cast<unsigned long long>(cloud->size()));
    clear_frame(true);
    return;
  }
  if (label_fusion_cfg_.enabled && !last_dnn_seg_mask_.empty() &&
      (last_dnn_img_w_ != grid_w || last_dnn_img_h_ != grid_h ||
       last_dnn_seg_mask_.size() != static_cast<size_t>(grid_w) * grid_h)) {
    LOG_ERROR("[PercepMain][det_cloud_sdk] mask does not match AI image grid");
    clear_frame(true);
    return;
  }
  // Source intrinsics are calibrated for camera_params.width/height. Reuse
  // the old fusion's scaling rule when the AI grid uses another resolution.
  double calibration_width = 0, calibration_height = 0;
  if (stereo_cfg_.enabled) {
    // Pair the dimensions with the same calibration snapshot as fx/fy/cx/cy.
    calibration_width = stereo_frame_intrinsics_.width;
    calibration_height = stereo_frame_intrinsics_.height;
  } else {
    std::lock_guard<std::mutex> lock(camera_params_mtx_);
    calibration_width = camera_params.width;
    calibration_height = camera_params.height;
  }
  if (label_fusion_cfg_.enabled && calibration_width > 0 && calibration_height > 0 &&
      (calibration_width != grid_w || calibration_height != grid_h)) {
    const float sx = static_cast<float>(grid_w) / calibration_width;
    const float sy = static_cast<float>(grid_h) / calibration_height;
    fx *= sx;
    cx *= sx;
    fy *= sy;
    cy *= sy;
  }

  struct CloudResultDeleter {
    void operator()(ds_cloud_result* result) const noexcept {
      ds_cloud_result_release(&result);
    }
  };
  try {
    std::vector<ds_cloud_point> points;
    points.reserve(cloud->size());
    for (const auto& p : cloud->points) {
      ds_cloud_point q{};
      q.x = p.x; q.y = p.y; q.z = p.z;
      q.r = p.r; q.g = p.g; q.b = p.b;
      q.label = p.label; q.confidence = p.confidence;
      points.push_back(q);
    }

    std::vector<ds_detection> boxes;
    boxes.reserve(last_dnn_dets_.size());
    const float box_sx = label_fusion_cfg_.enabled && source_w > 0
                             ? static_cast<float>(grid_w) / source_w : 1.f;
    const float box_sy = label_fusion_cfg_.enabled && source_h > 0
                             ? static_cast<float>(grid_h) / source_h : 1.f;
    for (const auto& source : last_dnn_dets_) {
      ds_detection d{};
      d.class_id = source.class_id; d.score = source.confidence;
      // Old fusion compares its grid u/v after scaling back to source_w/h.
      // Scale only these temporary box copies into the SDK grid so the mask
      // and box projection share a coordinate system. Stored/overlay boxes
      // retain their original values. Floating-point edge rounding can differ.
      d.x1 = source.x1 * box_sx; d.y1 = source.y1 * box_sy;
      d.x2 = source.x2 * box_sx; d.y2 = source.y2 * box_sy;
      std::snprintf(d.class_name, sizeof(d.class_name), "%s",
                    source.class_name.c_str());
      boxes.push_back(d);
    }

    ds_result_view ai{};
    ai.width = static_cast<uint32_t>(grid_w);
    ai.height = static_cast<uint32_t>(grid_h);
    ai.detections = boxes.empty() ? nullptr : boxes.data();
    ai.detection_count = boxes.size();
    const bool use_mask = label_fusion_cfg_.enabled && !last_dnn_seg_mask_.empty();
    ai.mask = use_mask ? last_dnn_seg_mask_.data() : nullptr;
    ai.mask_bytes = use_mask ? last_dnn_seg_mask_.size() : 0;
    // The adapter ID is the last successful inference's local monotonic
    // sequence. The old function signature has no hardware capture timestamp;
    // matching these synthetic IDs is not proof of physical camera sync.
    ai.frame_id = dnn_runner_ ? dnn_runner_->lastFrameId() : 0;
    ai.timestamp_ns = dnn_runner_ ? dnn_runner_->lastTimestampNs() : 0;

    ds_cloud_input input{};
    input.points = points.data();
    input.point_count = points.size();
    input.image_width = ai.width;
    input.image_height = ai.height;
    input.fx = fx; input.fy = fy; input.cx = cx; input.cy = cy;
    input.frame_id = ai.frame_id;
    input.timestamp_ns = ai.timestamp_ns;

    ds_cloud_config cfg{};
    ds_cloud_default_config(&cfg);
    cfg.fusion_enabled = label_fusion_cfg_.enabled ? 1u : 0u;
    cfg.publish_cloud = det_match_cfg_.enabled ? 1u : 0u;
    cfg.publish_markers = det_match_cfg_.publish_markers ? 1u : 0u;
    // runDnnInference already applied keep_classes. 0 means no second filter;
    // SDK's default mask would otherwise silently remove additional classes.
    cfg.keep_detection_class_mask = 0;
    cfg.output_frame = parent_frame.c_str();
    cfg.passable_classes = label_fusion_cfg_.passable_classes.c_str();
    Eigen::Matrix4f camera_to_output = Eigen::Matrix4f::Identity();
    pclCommonPtr->EulerToMatrix(cali, camera_to_output);
    // Eigen storage is column-major by default; the C API is explicitly row-major.
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        cfg.camera_to_output[4 * r + c] = camera_to_output(r, c);
    cfg.match_score_min = det_match_cfg_.score_min;
    // Original <=0.05 means no depth limit; normalize negative sentinels to 0.
    cfg.match_max_depth_m = det_match_cfg_.max_depth_m > 0.05f
                                ? det_match_cfg_.max_depth_m : 0.f;
    cfg.match_depth_span_m = det_match_cfg_.depth_span_m;
    cfg.det_score_min = label_fusion_cfg_.det_score_min;
    cfg.max_range_m = label_fusion_cfg_.max_range_m;
    cfg.max_height_m = label_fusion_cfg_.max_height_m;
    cfg.keep_below_cam_m = label_fusion_cfg_.keep_below_cam_m;
    cfg.sensor_height_m = static_cast<float>(cali.z);
    cfg.det_depth_span_m = label_fusion_cfg_.det_depth_span_m;
    cfg.det_depth_gap_m = label_fusion_cfg_.det_depth_gap_m;
    cfg.det_depth_min_pts = label_fusion_cfg_.det_depth_min_pts;
    cfg.nongrass_seg_as_obstacle = label_fusion_cfg_.nongrass_seg_as_obstacle ? 1u : 0u;
    cfg.cluster_filter = label_fusion_cfg_.cluster_filter ? 1u : 0u;
    cfg.cluster_voxel_m = label_fusion_cfg_.cluster_voxel_m;
    cfg.cluster_min_component_pts = label_fusion_cfg_.cluster_min_component_pts;
    // The stored mask has already passed postprocessSegMask in runDnnInference.
    cfg.seg_mask_postprocess = 0;
    cfg.seg_cluster_filter = label_fusion_cfg_.seg_cluster_filter ? 1u : 0u;
    cfg.seg_mask_erode_px = label_fusion_cfg_.seg_mask_erode_px;
    cfg.seg_depth_gap_m = label_fusion_cfg_.seg_depth_gap_m;
    cfg.seg_ray_bin_px = label_fusion_cfg_.seg_ray_bin_px;
    cfg.seg_min_component_pts = label_fusion_cfg_.seg_min_component_pts;

    ds_model_contract contract{};
    ds_default_model_contract(&contract);
    if (dnn_cfg_.detect.enabled && dnn_cfg_.detect.classes_num > 0 &&
        !dnn_cfg_.detect.class_names.empty()) {
      contract.detection_classes = static_cast<uint32_t>(dnn_cfg_.detect.classes_num);
      contract.detection_reg = static_cast<uint32_t>(dnn_cfg_.detect.reg);
      contract.detection_resize_type = static_cast<uint32_t>(dnn_cfg_.detect.resize_type);
      contract.detection_class_names = dnn_cfg_.detect.class_names.c_str();
    }
    if (dnn_cfg_.seg.enabled && !dnn_cfg_.seg.class_names.empty()) {
      contract.segmentation_classes = dnn_cfg_.seg.num_classes > 0
                                          ? static_cast<uint32_t>(dnn_cfg_.seg.num_classes) : 3u;
      contract.segmentation_resize_type = static_cast<uint32_t>(dnn_cfg_.seg.resize_type);
      contract.segmentation_align_corners = dnn_cfg_.seg.align_corners ? 1u : 0u;
      contract.segmentation_argmax_mode = dnn_cfg_.seg.argmax_mode.c_str();
      contract.segmentation_class_names = dnn_cfg_.seg.class_names.c_str();
    }

    // The source u/v values correspond to img_w/img_h. Original fusion scales
    // them into the segmentation grid before truncation; the SDK consumes
    // u/v in its input grid. Keep out-of-grid coordinates as grid-size
    // sentinels, so individual points remain unknown rather than failing a frame.
    std::vector<uint32_t> scaled_u, scaled_v;
    const uint32_t* source_u = nullptr;
    const uint32_t* source_v = nullptr;
    size_t source_uv_count = 0;
    if (u_vec.size() == cloud->size() && v_vec.size() == cloud->size()) {
      source_u = u_vec.data();
      source_v = v_vec.data();
      source_uv_count = cloud->size();
      if (source_w != grid_w || source_h != grid_h) {
        scaled_u.resize(source_uv_count);
        scaled_v.resize(source_uv_count);
        for (size_t i = 0; i < source_uv_count; ++i) {
          const float u = static_cast<float>(u_vec[i]) * grid_w / source_w;
          const float v = static_cast<float>(v_vec[i]) * grid_h / source_h;
          scaled_u[i] = u >= grid_w ? static_cast<uint32_t>(grid_w)
                                     : static_cast<uint32_t>(u);
          scaled_v[i] = v >= grid_h ? static_cast<uint32_t>(grid_h)
                                     : static_cast<uint32_t>(v);
        }
        source_u = scaled_u.data();
        source_v = scaled_v.data();
      }
    }

    char error[512] = {};
    ds_cloud_result* raw = nullptr;
    const int rc = ds_match_det_cloud_ex(&cfg, &input, &ai, &contract,
                                        source_u, source_v, source_uv_count,
                                        &raw, error, sizeof(error));
    std::unique_ptr<ds_cloud_result, CloudResultDeleter> result(raw);
    if (rc != DS_OK || !result) {
      LOG_ERROR("[PercepMain][det_cloud_sdk] match failed code=%d detail=%s", rc, error);
      clear_frame(true);
      return;
    }
    ds_cloud_view view{};
    if (ds_cloud_result_get_view(result.get(), &view) != DS_OK ||
        view.point_count != cloud->size() ||
        (view.point_count && !view.points) ||
        (view.marker_count && !view.markers)) {
      LOG_ERROR("[PercepMain][det_cloud_sdk] invalid result or point count changed");
      clear_frame(true);
      return;
    }
    PointCloudT::Ptr output(new PointCloudT(*cloud));
    size_t legacy_unknown_to_background = 0;
    const bool background_is_passable =
        std::find(passable_seg_ids_.begin(), passable_seg_ids_.end(), 0) !=
        passable_seg_ids_.end();
    for (size_t i = 0; i < view.point_count; ++i) {
      const auto& source = view.points[i];
      auto& dest = output->points[i];
      // SDK already applied camera_to_output; do not apply cali a second time.
      dest.x = source.x; dest.y = source.y; dest.z = source.z;
      dest.r = source.r; dest.g = source.g; dest.b = source.b;
      dest.label = source.label; dest.confidence = source.confidence;
      // The public SDK deliberately reports unknown as -1/0. The original
      // application's fusion path instead assigns no_grass label 100 and
      // confidence 1, including for gated or out-of-image points. Preserve
      // that downstream ai_fusion contract here, without changing SDK output.
      if (label_fusion_cfg_.enabled && source.label < 0) {
        dest.label = EncodeSegLabel(0);
        dest.confidence = 1.f;
        // Legacy fusion computed the color after assigning the fallback
        // segmentation label, so class 0 can be passable as well.
        if (background_is_passable) {
          dest.r = 0; dest.g = 200; dest.b = 0;
        }
        ++legacy_unknown_to_background;
      }
    }
    output->is_dense = false;
    last_semantic_cloud_ = output;
    if (det_match_cfg_.enabled) {
      publish_ros_rgbcloud(last_semantic_cloud_, det_cloud_pub_, parent_frame);
    }

    // Current parseLabelFusionConfig forces this flag off. Keep the original
    // DELETEALL/sphere/text behavior if the flag is enabled in a later build.
    if (det_match_cfg_.publish_markers && view.clear_markers) {
      visualization_msgs::msg::MarkerArray markers;
      visualization_msgs::msg::Marker clear;
      clear.header.frame_id = parent_frame;
      clear.header.stamp = rclcpp::Clock().now();
      clear.ns = "det";
      clear.id = 0;
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      markers.markers.push_back(clear);
      for (size_t i = 0; i < view.marker_count; ++i) {
        const auto& item = view.markers[i];
        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = "percep_main";
        sphere.header.stamp = clear.header.stamp;
        sphere.ns = "det";
        sphere.id = item.detection_index + 1;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position.x = item.x;
        sphere.pose.position.y = item.y;
        sphere.pose.position.z = item.z;
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.12;
        sphere.color.r = item.r / 255.f;
        sphere.color.g = item.g / 255.f;
        sphere.color.b = item.b / 255.f;
        sphere.color.a = 0.85f;
        sphere.lifetime = rclcpp::Duration::from_seconds(0.5);
        markers.markers.push_back(sphere);
        visualization_msgs::msg::Marker text = sphere;
        text.id = item.detection_index + 1000;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.pose.position.z += 0.15;
        text.scale.x = text.scale.y = 0;
        text.scale.z = 0.12;
        text.text = item.text;
        markers.markers.push_back(text);
      }
      det_markers_pub_.publish(markers);
    }
    LOG_INFO("[PercepMain][det_cloud_sdk] in=%llu out=%llu detection=%llu segmentation=%llu sdk_unknown=%llu legacy_background=%llu matching=%.3fms",
             static_cast<unsigned long long>(points.size()),
             static_cast<unsigned long long>(view.point_count),
             static_cast<unsigned long long>(view.detection_labeled),
             static_cast<unsigned long long>(view.segmentation_labeled),
             static_cast<unsigned long long>(view.unknown),
             static_cast<unsigned long long>(legacy_unknown_to_background), view.matching_ms);
  } catch (const std::exception& e) {
    LOG_ERROR("[PercepMain][det_cloud_sdk] exception: %s", e.what());
    clear_frame(true);
  } catch (...) {
    LOG_ERROR("[PercepMain][det_cloud_sdk] unknown exception");
    clear_frame(true);
  }
}

}  // namespace perception

