#include "percep_demo/api_recharge_mark.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <deque>
#include <exception>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "third_party/stb_image_write.h"

#include "percep_demo/mark_recharge_detect.h"
#include "percep_demo/mark_aruco_detect.h"
#include "recharge_mark_config.h"
#include "recharge_mark_logger.h"

#include "shm_struct/camera_struct.h"


namespace {

using SteadyClock = std::chrono::steady_clock;

std::mutex g_api_mtx;
bool g_cam_override = false;
double g_fx = 0, g_fy = 0, g_cx = 0, g_cy = 0;
aiper::recharge::DetectOptions g_opt{};
aiper::aruco::ArucoOptions g_aruco_opt{};
int g_max_pnp_try = 96;
bool g_skip_bc_on_a_fail = false;
int g_unlock_fail_streak = 0;

bool g_save_raw = false;
std::string g_save_raw_dir;
uint32_t g_save_every_n = 1;
uint32_t g_save_max = 0;
uint64_t g_save_tick = 0;
uint64_t g_save_count = 0;

bool g_build_overlay = false;
int g_overlay_max_width = 640;

bool g_sdk_k_cached = false;
double g_sdk_fx = 0, g_sdk_fy = 0, g_sdk_cx = 0, g_sdk_cy = 0;

Aiper_Recharge::RechargeHostConfig g_host{};
bool g_mark_yaw_have = false;
float g_mark_yaw_pub = 0.f;
bool g_x_have = false;
float g_x_pub = 0.f;
std::deque<float> g_x_hist;

// 在线异步：单槽队列，忙则丢最新之前的 pending
struct RuntimeJob {
    std::vector<uint8_t> nv12;
    int width = 0;
    int height = 0;
    int stride_y = 0;
    uint64_t frame_id = 0;
    int64_t timestamp_ns = 0;
};
std::mutex g_rt_mtx;
std::condition_variable g_rt_cv;
std::thread g_rt_thread;
std::atomic<bool> g_rt_stop{true};
std::atomic<bool> g_rt_running{false};
std::atomic<bool> g_rt_busy{false};
std::atomic<bool> g_rt_joinable{false};
bool g_rt_pending = false;
RuntimeJob g_rt_job;
Aiper_Recharge::RechargeMarkApi::ResultCallback g_rt_cb = nullptr;
void* g_rt_cb_ud = nullptr;
std::chrono::steady_clock::time_point g_rt_last_start{};
uint64_t g_rt_tick = 0;
uint64_t g_rt_drop = 0;
std::atomic<uint32_t> g_rt_min_interval_ms{200};
std::atomic<uint32_t> g_rt_run_every_n{1};

static inline uint8_t ClampU8(int v) {
    return static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v));
}

static inline double ElapsedMs(SteadyClock::time_point t0) {
    return std::chrono::duration<double, std::milli>(SteadyClock::now() - t0)
        .count();
}

bool MkDirRecursive(const std::string& path) {
    if (path.empty()) return false;
    std::string cur;
    cur.reserve(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        char c = path[i];
        cur.push_back(c);
        if (c == '/' || i + 1 == path.size()) {
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
                if (::stat(d.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
                    return false;
                }
            }
        }
    }
    return true;
}

void MaybeSaveRawBgr(const cv::Mat& bgr, uint64_t frame_id) {
    if (!g_save_raw || g_save_raw_dir.empty() || bgr.empty()) return;
    ++g_save_tick;
    if (g_save_every_n > 1 && (g_save_tick % g_save_every_n) != 0) return;
    if (g_save_max > 0 && g_save_count >= g_save_max) {
        if (g_save_count == g_save_max) {
            RM_LOG_INFO("[recharge_mark] save_raw reached max_count=%u, stop",
                        g_save_max);
            ++g_save_count;  // 只打一次
        }
        return;
    }
    if (!MkDirRecursive(g_save_raw_dir)) {
        RM_LOG_ERROR("[recharge_mark] mkdir save_raw_dir failed: %s",
                     g_save_raw_dir.c_str());
        return;
    }

    std::time_t now = std::time(nullptr);
    std::tm tm_buf {};
    localtime_r(&now, &tm_buf);
    char name[256];
    std::snprintf(name, sizeof(name), "left_%02d%02d%02d_%02d%02d_%06llu.jpg",
                  (tm_buf.tm_year + 1900) % 100, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                  tm_buf.tm_hour, tm_buf.tm_min,
                  static_cast<unsigned long long>(frame_id ? frame_id : g_save_tick));
    const std::string path = g_save_raw_dir + "/" + name;
    cv::Mat contiguous;
    if (bgr.isContinuous() && bgr.type() == CV_8UC3) {
        contiguous = bgr;
    } else {
        bgr.copyTo(contiguous);
    }
    cv::Mat rgb;
    rgb.create(contiguous.rows, contiguous.cols, CV_8UC3);
    for (int y = 0; y < contiguous.rows; ++y) {
        const uint8_t* s = contiguous.ptr<uint8_t>(y);
        uint8_t* d = rgb.ptr<uint8_t>(y);
        for (int x = 0; x < contiguous.cols; ++x) {
            d[x * 3 + 0] = s[x * 3 + 2];
            d[x * 3 + 1] = s[x * 3 + 1];
            d[x * 3 + 2] = s[x * 3 + 0];
        }
    }
    if (!stbi_write_jpg(path.c_str(), rgb.cols, rgb.rows, 3, rgb.data, 90)) {
        RM_LOG_ERROR("[recharge_mark] stbi_write_jpg failed: %s",
                     path.c_str());
        return;
    }
    ++g_save_count;
    RM_LOG_INFO("[recharge_mark] save_raw %dx%d -> %s (%llu/%u)",
                rgb.cols, rgb.rows, path.c_str(),
                static_cast<unsigned long long>(g_save_count), g_save_max);
}

cv::Rect MakeRoiRect(int W, int H, const aiper::recharge::DetectOptions& opt) {
    if (!opt.enable_search_crop || W <= 0 || H <= 0) {
        return cv::Rect(0, 0, W, H);
    }
    int x0 = static_cast<int>(W * opt.search_x_min_ratio);
    int x1 = static_cast<int>(W * opt.search_x_max_ratio);
    int y0 = static_cast<int>(H * opt.search_y_min_ratio);
    int y1 = static_cast<int>(H * opt.search_y_max_ratio);
    x0 = std::max(0, std::min(W - 1, x0));
    y0 = std::max(0, std::min(H - 1, y0));
    x1 = std::max(x0 + 1, std::min(W, x1));
    y1 = std::max(y0 + 1, std::min(H, y1));
    // NV12 UV 对齐
    x0 &= ~1;
    y0 &= ~1;
    x1 = std::min(W, (x1 + 1) & ~1);
    y1 = std::min(H, (y1 + 1) & ~1);
    if (x1 <= x0) x1 = std::min(W, x0 + 2);
    if (y1 <= y0) y1 = std::min(H, y0 + 2);
    return cv::Rect(x0, y0, x1 - x0, y1 - y0);
}

bool Nv12RoiToBgr24(const uint8_t* data, int full_w, int full_h, int stride_y,
                    const cv::Rect& roi, std::vector<uint8_t>* bgr_out) {
    if (!data || !bgr_out || full_w <= 0 || full_h <= 0) return false;
    if (stride_y < full_w) stride_y = full_w;
    if (roi.width <= 0 || roi.height <= 0 || roi.x < 0 || roi.y < 0 ||
        roi.x + roi.width > full_w || roi.y + roi.height > full_h) {
        return false;
    }
    const int rw = roi.width;
    const int rh = roi.height;
    bgr_out->resize(static_cast<size_t>(rw) * static_cast<size_t>(rh) * 3u);
    uint8_t* dst = bgr_out->data();
    const uint8_t* y_plane = data;
    const uint8_t* uv_plane = data + static_cast<size_t>(stride_y) * full_h;
    for (int y = 0; y < rh; ++y) {
        const int sy = roi.y + y;
        for (int x = 0; x < rw; ++x) {
            const int sx = roi.x + x;
            const int Y = y_plane[sy * stride_y + sx];
            const int uv_off = (sy / 2) * stride_y + (sx / 2) * 2;
            const int U = uv_plane[uv_off];
            const int V = uv_plane[uv_off + 1];
            const int C = Y - 16;
            const int D = U - 128;
            const int E = V - 128;
            const int R = (298 * C + 409 * E + 128) >> 8;
            const int G = (298 * C - 100 * D - 208 * E + 128) >> 8;
            const int B = (298 * C + 516 * D + 128) >> 8;
            const size_t o = (static_cast<size_t>(y) * rw + x) * 3u;
            dst[o + 0] = ClampU8(B);
            dst[o + 1] = ClampU8(G);
            dst[o + 2] = ClampU8(R);
        }
    }
    return true;
}

bool Nv12ToBgr24(const uint8_t* data, int w, int h, int stride_y,
                 std::vector<uint8_t>* bgr_out) {
    return Nv12RoiToBgr24(data, w, h, stride_y, cv::Rect(0, 0, w, h), bgr_out);
}

// 与 point-cloud-processingtest 相同：NV12 Y 平面即灰度，包装 ROI 后 clone。
cv::Mat MakeGrayRoiFromNv12(const uint8_t* data, int full_w, int full_h,
                            int stride_y, const cv::Rect& roi) {
    if (!data || full_w <= 0 || full_h <= 0 || roi.width <= 0 ||
        roi.height <= 0 || roi.x < 0 || roi.y < 0 ||
        roi.x + roi.width > full_w || roi.y + roi.height > full_h) {
        return {};
    }
    if (stride_y < full_w) stride_y = full_w;
    const uint8_t* y_off =
        data + static_cast<size_t>(roi.y) * stride_y + roi.x;
    return cv::Mat(roi.height, roi.width, CV_8UC1,
                   const_cast<uint8_t*>(y_off), stride_y);
}

cv::Mat MakeK(double fx, double fy, double cx, double cy) {
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = fx;
    K.at<double>(1, 1) = fy;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;
    return K;
}

cv::Mat CurrentCameraMatrix(int img_w = 0, int img_h = 0) {
    if (g_cam_override && g_fx > 1.0 && g_fy > 1.0) {
        return MakeK(g_fx, g_fy, g_cx, g_cy);
    }
    if (g_sdk_k_cached && g_sdk_fx > 1.0 && g_sdk_fy > 1.0) {
        return MakeK(g_sdk_fx, g_sdk_fy, g_sdk_cx, g_sdk_cy);
    }
    cv::Mat K = aiper::recharge::DefaultCameraMatrix();
    constexpr double kRefW = 1280.0;
    constexpr double kRefH = 1088.0;
    if (img_w > 0 && img_h > 0 &&
        (img_w != static_cast<int>(kRefW) || img_h != static_cast<int>(kRefH))) {
        K.at<double>(0, 2) *= static_cast<double>(img_w) / kRefW;
        K.at<double>(1, 2) *= static_cast<double>(img_h) / kRefH;
    }
    return K;
}

void ResetDockPoseFiltersLocked() {
    g_mark_yaw_have = false;
    g_mark_yaw_pub = 0.f;
    g_x_have = false;
    g_x_pub = 0.f;
    g_x_hist.clear();
}

void ApplyParsedLocked(const recharge_cfg::Parsed& p) {
    g_host = p.host;
    g_rt_min_interval_ms.store(g_host.min_interval_ms, std::memory_order_relaxed);
    g_rt_run_every_n.store(g_host.run_every_n == 0 ? 1 : g_host.run_every_n,
                           std::memory_order_relaxed);
    g_opt.enable_search_crop = p.roi_enabled;
    g_opt.search_x_min_ratio = p.roi_x_min;
    g_opt.search_x_max_ratio = p.roi_x_max;
    g_opt.search_y_min_ratio = p.roi_y_min;
    g_opt.search_y_max_ratio = p.roi_y_max;
    g_opt.enable_aspect_ratio_check = p.check_aspect;
    g_opt.enable_strict_symmetry_check = p.check_sym;
    g_opt.enable_area_perimeter_structure_check = p.check_area;
    g_opt.enable_ground_pose_refine = p.pose_refine;
    if (p.camera_height_mm > 50.0) g_opt.camera_height_mm = p.camera_height_mm;
    if (p.mark_height_mm > 10.0) g_opt.mark_height_mm = p.mark_height_mm;
    g_opt.camera_pitch_deg = p.camera_pitch_deg;
    g_build_overlay = p.host.publish_overlay;
    g_overlay_max_width =
        p.host.overlay_max_width > 0 ? p.host.overlay_max_width : 640;
    g_save_raw_dir = p.save_raw ? p.save_raw_dir : "";
    g_save_raw = p.save_raw && !g_save_raw_dir.empty();
    g_save_every_n = p.save_every_n > 0 ? p.save_every_n : 1;
    g_save_max = p.save_max;
    g_save_tick = 0;
    g_save_count = 0;
    g_aruco_opt.dictionary_id = p.aruco_dictionary_id;
    g_aruco_opt.target_marker_id = p.aruco_target_id;
    g_aruco_opt.dual_marker = p.aruco_dual_marker;
    g_aruco_opt.layout_horizontal = p.aruco_layout_horizontal;
    g_aruco_opt.marker_id_left = p.aruco_id_left;
    g_aruco_opt.marker_id_right = p.aruco_id_right;
    g_aruco_opt.marker_id_top = p.aruco_id_top;
    g_aruco_opt.marker_id_bottom = p.aruco_id_bottom;
    if (p.aruco_marker_size_m > 0.001) {
        g_aruco_opt.marker_size_m = p.aruco_marker_size_m;
    }
    if (p.aruco_gap_m > 0.0) g_aruco_opt.gap_m = p.aruco_gap_m;
    if (p.aruco_margin_m > 0.0) g_aruco_opt.margin_m = p.aruco_margin_m;
    if (p.aruco_card_w_m > 0.01) g_aruco_opt.card_w_m = p.aruco_card_w_m;
    if (p.aruco_card_h_m > 0.01) g_aruco_opt.card_h_m = p.aruco_card_h_m;
    if (p.aruco_max_reproj_px > 0.1) {
        g_aruco_opt.max_reprojection_error_px = p.aruco_max_reproj_px;
    }
    g_aruco_opt.enable_lm_refine = p.aruco_enable_lm_refine;
    g_aruco_opt.adaptive_thresh_win_size_min = p.aruco_thresh_win_min;
    g_aruco_opt.adaptive_thresh_win_size_max = p.aruco_thresh_win_max;
    g_aruco_opt.adaptive_thresh_win_size_step = p.aruco_thresh_win_step;
    g_aruco_opt.min_marker_perimeter_rate = p.aruco_min_perimeter_rate;
    g_aruco_opt.max_marker_perimeter_rate = p.aruco_max_perimeter_rate;
    g_aruco_opt.polygonal_approx_accuracy_rate = p.aruco_poly_approx_rate;
    g_aruco_opt.min_corner_distance_rate = p.aruco_min_corner_dist_rate;
    g_aruco_opt.min_distance_to_border = p.aruco_min_border;
    g_aruco_opt.perspective_remove_pixel_per_cell = p.aruco_pixel_per_cell;
    g_aruco_opt.corner_refinement_method = p.aruco_refine_method;
    g_aruco_opt.corner_refinement_win_size = p.aruco_refine_win;
    g_aruco_opt.corner_refinement_max_iterations = p.aruco_refine_iters;
    g_aruco_opt.corner_refinement_min_accuracy = p.aruco_refine_min_acc;
    ResetDockPoseFiltersLocked();
}

float YawAbsDiff(float a, float b) {
    float d = a - b;
    while (d > static_cast<float>(M_PI)) d -= 2.f * static_cast<float>(M_PI);
    while (d < -static_cast<float>(M_PI)) d += 2.f * static_cast<float>(M_PI);
    return std::fabs(d);
}

void AppendFilterReason(char* buf, size_t n, const char* tag) {
    if (!buf || n == 0 || !tag || !tag[0]) return;
    if (buf[0] == 0) {
        std::snprintf(buf, n, "%s", tag);
        return;
    }
    char tmp[48];
    std::snprintf(tmp, sizeof(tmp), "%s+%s", buf, tag);
    std::snprintf(buf, n, "%s", tmp);
}

void ApplyDockPosePublishFilter(Aiper_Recharge::MarkPoseResult* out) {
    if (!out) return;
    std::lock_guard<std::mutex> lock(g_api_mtx);
    out->dist_jump_hold = false;
    out->yaw_jump_hold = false;
    out->x_filter_reason[0] = 0;
    if (!out->pose_ok) {
        return;
    }

    // 符号只来自 PnP dock_x（码面系 cam_x + 左目 30mm）和竖边梯面。
    // |x| 较大时禁止单帧翻号；|x|≤center 跟测过零。禁止画面左右翻号。
    const float dist = std::fabs(out->post.y);
    const float yaw_raw = out->post.yaw;
    float x = out->post.x;
    out->raw_x_m = x;
    const int trap = out->trap_sign;
    if (trap != 0 && std::fabs(x) > 0.08f &&
        static_cast<float>(trap) * x < 0.f) {
        x = std::copysign(std::fabs(x), static_cast<float>(trap));
        AppendFilterReason(out->x_filter_reason, sizeof(out->x_filter_reason),
                           "trap");
    }
    constexpr float kHold = 0.10f;
    constexpr float kCenter = 0.08f;
    constexpr size_t kHist = 7;
    if (g_x_have) {
        if (std::fabs(x) > kHold && std::fabs(g_x_pub) > kHold &&
            x * g_x_pub < 0.f) {
            x = std::copysign(std::fabs(x), g_x_pub);
            AppendFilterReason(out->x_filter_reason, sizeof(out->x_filter_reason),
                               "sign_hold");
        } else {
            int npos = 0, nneg = 0, noff = 0;
            for (float v : g_x_hist) {
                if (std::fabs(v) <= kHold) continue;
                ++noff;
                if (v > 0.f) ++npos;
                else ++nneg;
            }
            if (std::fabs(x) > kHold && std::fabs(g_x_pub) > kCenter &&
                noff >= 3 && npos != nneg) {
                const float maj = (npos > nneg) ? 1.f : -1.f;
                if (x * maj < 0.f) {
                    x = std::copysign(std::fabs(x), maj);
                    AppendFilterReason(out->x_filter_reason,
                                       sizeof(out->x_filter_reason), "sign_maj");
                }
            }
        }
        const float lim = (dist > 0.70f) ? 0.10f : 0.055f;
        const float dx = x - g_x_pub;
        if (std::fabs(x) > 0.05f && std::fabs(dx) > lim) {
            x = g_x_pub + std::copysign(lim, dx);
            AppendFilterReason(out->x_filter_reason, sizeof(out->x_filter_reason),
                               "x_rate");
        }
    }
    g_x_have = true;
    g_x_pub = x;
    g_x_hist.push_back(x);
    while (g_x_hist.size() > kHist) g_x_hist.pop_front();
    out->post.x = x;

    float yaw = yaw_raw;
    if (std::fabs(x) <= 0.04f && std::fabs(yaw_raw) > 0.05f &&
        std::fabs(yaw_raw) <= 0.20f) {
        yaw = std::copysign(0.05f, yaw_raw);
    }
    if (g_mark_yaw_have) {
        const float dy = YawAbsDiff(yaw, g_mark_yaw_pub);
        if (dy > 0.12f && std::fabs(yaw_raw) <= 0.20f) {
            const float cap = std::fabs(g_mark_yaw_pub) + 0.06f;
            if (std::fabs(yaw) > cap) {
                yaw = std::copysign(cap, yaw_raw);
            }
        }
    }
    g_mark_yaw_have = true;
    g_mark_yaw_pub = yaw;
    out->post.yaw = yaw;
}

void FillPoint(Aiper_Recharge::MarkPoseResult* out, const char* name,
               const std::map<std::string, cv::Point2f>& pts) {
    auto it = pts.find(name);
    if (it == pts.end()) return;
    if (std::strcmp(name, "TL") == 0) {
        out->has_tl = true; out->tl_u = it->second.x; out->tl_v = it->second.y;
    } else if (std::strcmp(name, "TR") == 0) {
        out->has_tr = true; out->tr_u = it->second.x; out->tr_v = it->second.y;
    } else if (std::strcmp(name, "BR") == 0) {
        out->has_br = true; out->br_u = it->second.x; out->br_v = it->second.y;
    } else if (std::strcmp(name, "BL") == 0) {
        out->has_bl = true; out->bl_u = it->second.x; out->bl_v = it->second.y;
    } else if (std::strcmp(name, "TOP") == 0) {
        out->has_top = true; out->top_u = it->second.x; out->top_v = it->second.y;
    } else if (std::strcmp(name, "BOT") == 0) {
        out->has_bot = true; out->bot_u = it->second.x; out->bot_v = it->second.y;
    }
}

void OffsetDetectToFull(aiper::recharge::MarkDetectResult* det, int ox, int oy) {
    if (!det || (ox == 0 && oy == 0)) return;
    det->center.x += ox;
    det->center.y += oy;
    det->mark_box.x += ox;
    det->mark_box.y += oy;
    det->search_crop.x += ox;
    det->search_crop.y += oy;
    for (auto& kv : det->feature_points) {
        kv.second.x += static_cast<float>(ox);
        kv.second.y += static_cast<float>(oy);
    }
    for (auto& r : det->debug_candidate_boxes) {
        r.x += ox;
        r.y += oy;
    }
    for (auto& r : det->debug_scaffold_boxes) {
        r.x += ox;
        r.y += oy;
    }
    if (det->pose.ok) {
        det->pose.center_image_px.x += ox;
        det->pose.center_image_px.y += oy;
    }
}

// AI 紧裁：把裁切 overlay 贴回全幅，并画 charger 框
cv::Mat PasteOverlayToFull(const cv::Mat& crop_ov, int full_w, int full_h,
                           int ox, int oy) {
    cv::Mat full(std::max(1, full_h), std::max(1, full_w), CV_8UC3,
                 cv::Scalar(40, 40, 40));
    if (crop_ov.empty() || full_w <= 0 || full_h <= 0) return full;
    const int cw = crop_ov.cols;
    const int ch = crop_ov.rows;
    const int dx0 = std::max(0, ox);
    const int dy0 = std::max(0, oy);
    const int sx0 = dx0 - ox;
    const int sy0 = dy0 - oy;
    const int dx1 = std::min(full_w, ox + cw);
    const int dy1 = std::min(full_h, oy + ch);
    const int rw = dx1 - dx0;
    const int rh = dy1 - dy0;
    if (rw > 0 && rh > 0 && sx0 >= 0 && sy0 >= 0 &&
        sx0 + rw <= cw && sy0 + rh <= ch) {
        crop_ov(cv::Rect(sx0, sy0, rw, rh))
            .copyTo(full(cv::Rect(dx0, dy0, rw, rh)));
    }
    cv::rectangle(full, cv::Rect(ox, oy, cw, ch), cv::Scalar(0, 255, 255), 2);
    const int ty = std::max(16, oy - 6);
    cv::putText(full, "ai_crop", cv::Point(ox + 4, ty),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1,
                cv::LINE_AA);
    return full;
}

void AttachOverlayBgr(Aiper_Recharge::MarkPoseResult* out, const cv::Mat& bgr) {
    if (!out || bgr.empty() || bgr.type() != CV_8UC3) return;
    cv::Mat src = bgr;
    if (g_overlay_max_width > 0 && src.cols > g_overlay_max_width) {
        const double s =
            static_cast<double>(g_overlay_max_width) / src.cols;
        cv::resize(src, src, cv::Size(), s, s, cv::INTER_AREA);
    }
    out->overlay_w = src.cols;
    out->overlay_h = src.rows;
    out->overlay_step = src.cols * 3;
    const size_t nbytes =
        static_cast<size_t>(out->overlay_step) * static_cast<size_t>(out->overlay_h);
    out->overlay_bgr = new uint8_t[nbytes];
    if (src.isContinuous()) {
        std::memcpy(out->overlay_bgr, src.data, nbytes);
    } else {
        for (int y = 0; y < src.rows; ++y) {
            std::memcpy(out->overlay_bgr + static_cast<size_t>(y) * out->overlay_step,
                        src.ptr(y), static_cast<size_t>(out->overlay_step));
        }
    }
}

cv::Mat BuildOverlayImage(const cv::Mat& bgr,
                          const aiper::recharge::MarkDetectResult& det) {
    cv::Mat out;
    bgr.copyTo(out);
    // 仅成功框才画点/框；失败不画杂点，避免误导
    if (!det.ok) {
        return out;
    }
    if (det.mark_box.area() > 0) {
        cv::rectangle(out, det.mark_box, cv::Scalar(255, 0, 0), 2);
    }
    for (const auto& kv : det.feature_points) {
        const cv::Point p(static_cast<int>(std::lround(kv.second.x)),
                          static_cast<int>(std::lround(kv.second.y)));
        cv::drawMarker(out, p, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 16, 2);
        cv::putText(out, kv.first, p + cv::Point(4, -4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 255), 1,
                    cv::LINE_AA);
    }
    cv::drawMarker(out, det.center, cv::Scalar(255, 0, 255),
                   cv::MARKER_CROSS, 24, 2);
    char line[192];
    std::snprintf(line, sizeof(line),
                  "ok pts=%d cand=%d scaf=%d score=%.1f",
                  det.detected_point_count, det.debug_initial_candidates,
                  det.debug_scaffold_count, det.score);
    cv::putText(out, line, cv::Point(8, 24), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    if (det.pose.ok) {
        std::snprintf(line, sizeof(line), "dist=%.0fmm heading=%.1f",
                      det.pose.distance_mm, det.pose.heading_y_deg);
        cv::putText(out, line, cv::Point(8, 48), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
    }
    return out;
}

Aiper_Recharge::MarkPoseResult* FromDetect(
    const aiper::recharge::MarkDetectResult& det, int w, int h,
    uint64_t frame_id) {
    auto* out = new Aiper_Recharge::MarkPoseResult{};
    out->frame_id = frame_id;
    out->image_w = w;
    out->image_h = h;
    out->ok = det.ok;
    out->detected_point_count = det.detected_point_count;
    out->center_u = det.center.x;
    out->center_v = det.center.y;
    out->image_angle_deg = det.image_angle_deg;
    out->score = det.score;
    out->debug_candidates = det.debug_initial_candidates;
    out->debug_scaffolds = det.debug_scaffold_count;
    {
        std::string err = det.error;
        if (!det.ok) {
            if (err.empty()) err = "no pose";
            if (!det.detect_branch.empty()) {
                err = det.detect_branch + ":" + err;
            }
        }
        if (!err.empty()) {
            std::snprintf(out->error, sizeof(out->error), "%s", err.c_str());
        }
    }
    if (!det.pnp_error.empty()) {
        std::snprintf(out->pnp_error, sizeof(out->pnp_error), "%s",
                      det.pnp_error.c_str());
    }
    for (const char* n : {"TL", "TR", "BR", "BL", "TOP", "BOT"}) {
        FillPoint(out, n, det.feature_points);
    }
    if (det.pose.ok) {
        out->pose_ok = true;
        out->tvec_mm[0] = det.pose.tvec_mm[0];
        out->tvec_mm[1] = det.pose.tvec_mm[1];
        out->tvec_mm[2] = det.pose.tvec_mm[2];
        out->rvec[0] = det.pose.rvec[0];
        out->rvec[1] = det.pose.rvec[1];
        out->rvec[2] = det.pose.rvec[2];
        out->distance_mm = det.pose.distance_mm;
        out->reprojection_rmse_px = det.pose.reprojection_rmse_px;
        out->roll_x_deg = det.pose.roll_x_deg;
        out->pitch_y_deg = det.pose.pitch_y_deg;
        out->yaw_z_deg = det.pose.yaw_z_deg;
        out->heading_y_deg = det.pose.heading_y_deg;
        out->tilt_x_deg = det.pose.tilt_x_deg;
        out->image_roll_z_deg = det.pose.image_roll_z_deg;
        // x<0 车在桩轴左。cam_x 用码面系相机 X（对准码时 tvec.x≈0，C_x 仍是横向偏置）
        // 左目在车体中心左侧 30mm：body_x = cam_x + 30。禁止画面 u 翻号。
        constexpr double kCamLateralOffsetMm = 30.0;
        const double cam_x =
            std::isfinite(det.pose.cam_x_mm) ? det.pose.cam_x_mm
                                             : det.pose.tvec_mm[0];
        out->post.x = static_cast<float>((cam_x + kCamLateralOffsetMm) / 1000.0);
        // 码面比对接面板靠后约 6cm：发布 y 为到面板距（贴板≈0）。
        // distance_mm 是码面垂直距 |C_z|。
        constexpr double kPanelAheadM = 0.06;
        const double code_m =
            det.pose.distance_mm > 1.0
                ? det.pose.distance_mm / 1000.0
                : std::fabs(det.pose.tvec_mm[2] / 1000.0);
        out->post.y = static_cast<float>(
            -(std::max(0.0, code_m - kPanelAheadM)));
        out->post.yaw = static_cast<float>(det.pose.heading_y_deg * M_PI / 180.0);
        out->post.time_stamp = 0;
        out->trap_sign = det.pose.trap_sign;
        out->cam_x_mm = det.pose.cam_x_mm;
        out->raw_x_m = out->post.x;
    }
    return out;
}

Aiper_Recharge::MarkPoseResult* ProcessBgrMat(cv::Mat img, int full_w, int full_h,
                                              int crop_ox, int crop_oy,
                                              uint64_t frame_id,
                                              bool already_cropped,
                                              bool skip_save = false,
                                              bool trust_detection_crop = false) {
    const auto t0 = SteadyClock::now();
    cv::Mat K;
    aiper::recharge::DetectOptions opt;
    aiper::aruco::ArucoOptions aruco_opt;
    bool want_overlay = false;
    const bool is_bgr = (img.type() == CV_8UC3);
    {
        std::lock_guard<std::mutex> lock(g_api_mtx);
        K = CurrentCameraMatrix(full_w, full_h);
        opt = g_opt;
        aruco_opt = g_aruco_opt;
        opt.max_scaffolds_pnp_try = g_max_pnp_try > 0 ? g_max_pnp_try : 96;
        opt.skip_bc_on_a_fail = g_skip_bc_on_a_fail;
        want_overlay = g_build_overlay && is_bgr;
        if (g_save_raw && !skip_save && is_bgr) {
            MaybeSaveRawBgr(img, frame_id);
        }
    }

    // YOLO/静态 ROI 已裁：主点平移后在裁图上做 ArUco。
    if (already_cropped) {
        K.at<double>(0, 2) -= static_cast<double>(crop_ox);
        K.at<double>(1, 2) -= static_cast<double>(crop_oy);
        opt.enable_search_crop = false;
        opt.reference_image_width = full_w;
        opt.reference_image_height = full_h;
    }
    opt.trust_detection_crop = trust_detection_crop;

    // 与当时单码板端一致：TBB 用 2 线程，允许灰度或 BGR。
    cv::setNumThreads(2);
    if (img.empty() || (img.type() != CV_8UC3 && img.type() != CV_8UC1)) {
        auto* r = new Aiper_Recharge::MarkPoseResult{};
        std::snprintf(r->error, sizeof(r->error), "ProcessBgrMat needs BGR8 or GRAY8");
        return r;
    }
    cv::Mat bgr = img.isContinuous() ? img : img.clone();
    RM_LOG_INFO("[recharge_mark] ProcessBgrMat -> DetectArucoMarkCenter "
                "(DICT_4X4 dual=%d layout=%s idL=%d idR=%d size=%.3fm)",
                aruco_opt.dual_marker ? 1 : 0,
                aruco_opt.layout_horizontal ? "horizontal" : "vertical",
                aruco_opt.marker_id_left, aruco_opt.marker_id_right,
                aruco_opt.marker_size_m);
    auto det = aiper::aruco::DetectArucoMarkCenter(bgr, aruco_opt, K);

    cv::Mat overlay;
    if (want_overlay) {
        overlay = BuildOverlayImage(img, det);
        if (trust_detection_crop && already_cropped && full_w > 0 &&
            full_h > 0) {
            overlay =
                PasteOverlayToFull(overlay, full_w, full_h, crop_ox, crop_oy);
        }
    }

    if (already_cropped) {
        OffsetDetectToFull(&det, crop_ox, crop_oy);
    }

    auto* out = FromDetect(det, full_w, full_h, frame_id);
    out->process_ms = ElapsedMs(t0);
    if (want_overlay && !overlay.empty()) {
        AttachOverlayBgr(out, overlay);
    }
    if (det.pose.ok) {
        std::lock_guard<std::mutex> lock(g_api_mtx);
        g_opt.axis_x_prior_mm = det.pose.tvec_mm[0];
        g_opt.have_axis_x_prior = true;
        if (det.pose.distance_mm > 1.0) {
            g_opt.last_distance_mm = det.pose.distance_mm;
        }
        g_unlock_fail_streak = 0;
    } else {
        std::lock_guard<std::mutex> lock(g_api_mtx);
        ++g_unlock_fail_streak;
        // 连续失败后丢掉远距残留，启动按 ~1m 重新排 scaffold。
        // 上桩带（≤1.2m）必须留下距离：清掉后 A 支回到 800/60，80cm 正视再也提不到真 L。
        if (g_unlock_fail_streak >= 8) {
            const bool keep_approach =
                std::isfinite(g_opt.last_distance_mm) &&
                g_opt.last_distance_mm > 1.0 &&
                g_opt.last_distance_mm <= 1200.0;
            if (!keep_approach) {
                g_opt.last_distance_mm =
                    std::numeric_limits<double>::quiet_NaN();
                g_opt.have_axis_x_prior = false;
                g_opt.axis_x_prior_mm =
                    std::numeric_limits<double>::quiet_NaN();
            }
        }
    }
    ApplyDockPosePublishFilter(out);
    return out;
}

int ApiLoadConfig(const char* json_path) {
    recharge_cfg::Parsed parsed;
    std::string err;
    if (!recharge_cfg::LoadFile(json_path, &parsed, &err)) {
        RM_LOG_ERROR("[recharge_mark] loadConfig failed: %s", err.c_str());
        return -1;
    }
    std::lock_guard<std::mutex> lock(g_api_mtx);
    ApplyParsedLocked(parsed);
    RM_LOG_INFO("[recharge_mark] loadConfig %s interval=%u "
                "use_hires=%d overlay=%d dual=%d layout=%s size=%.3fm",
                json_path ? json_path : "",
                g_host.min_interval_ms, g_host.use_hires ? 1 : 0,
                g_host.publish_overlay ? 1 : 0,
                g_aruco_opt.dual_marker ? 1 : 0,
                g_aruco_opt.layout_horizontal ? "horizontal" : "vertical",
                g_aruco_opt.marker_size_m);
    return 0;
}

void ApiGetHostConfig(Aiper_Recharge::RechargeHostConfig* out) {
    if (!out) return;
    std::lock_guard<std::mutex> lock(g_api_mtx);
    *out = g_host;
}

int ApiInit(const char* sdk_so_path, const char* cfg_path) {
    (void)sdk_so_path;
    cv::setNumThreads(2);
    if (!cfg_path || !cfg_path[0]) {
        return 0;
    }
    const char* dot = std::strrchr(cfg_path, '.');
    if (!dot || std::strcmp(dot, ".json") != 0) {
        RM_LOG_WARN("[recharge_mark] init skip non-json cfg_path=%s", cfg_path);
        return 0;
    }
    return ApiLoadConfig(cfg_path);
}

void ApiSetCameraMatrix(double fx, double fy, double cx, double cy) {
    std::lock_guard<std::mutex> lock(g_api_mtx);
    g_cam_override = true;
    g_fx = fx; g_fy = fy; g_cx = cx; g_cy = cy;
}

void ApiUseSdkCameraMatrix() {
    std::lock_guard<std::mutex> lock(g_api_mtx);
    g_cam_override = false;
    g_sdk_k_cached = false;
}

void ApiSetSearchRoi(bool enabled, double x_min, double x_max, double y_min,
                     double y_max) {
    std::lock_guard<std::mutex> lock(g_api_mtx);
    g_opt.enable_search_crop = enabled;
    if (!enabled) return;
    auto clamp01 = [](double v) {
        return std::max(0.0, std::min(1.0, v));
    };
    x_min = clamp01(x_min);
    x_max = clamp01(x_max);
    y_min = clamp01(y_min);
    y_max = clamp01(y_max);
    if (x_max <= x_min) {
        x_min = 0.0;
        x_max = 1.0;
    }
    if (y_max <= y_min) {
        y_min = 0.0;
        y_max = 1.0;
    }
    g_opt.search_x_min_ratio = x_min;
    g_opt.search_x_max_ratio = x_max;
    g_opt.search_y_min_ratio = y_min;
    g_opt.search_y_max_ratio = y_max;
    RM_LOG_INFO("[recharge_mark] search_roi=on x=[%.2f,%.2f] y=[%.2f,%.2f]",
                x_min, x_max, y_min, y_max);
}

void ApiSetMaxPnpTry(int n) {
    std::lock_guard<std::mutex> lock(g_api_mtx);
    g_max_pnp_try = n > 0 ? n : 96;
}

void ApiSetSkipBcOnAFail(bool skip) {
    std::lock_guard<std::mutex> lock(g_api_mtx);
    g_skip_bc_on_a_fail = skip;
}

void ApiResetSession() {
    std::lock_guard<std::mutex> lock(g_api_mtx);
    g_opt.last_distance_mm = std::numeric_limits<double>::quiet_NaN();
    g_opt.have_axis_x_prior = false;
    g_opt.axis_x_prior_mm = std::numeric_limits<double>::quiet_NaN();
    g_unlock_fail_streak = 0;
    ResetDockPoseFiltersLocked();
}

void ApiSetDetectChecks(bool aspect_ratio, bool strict_symmetry,
                        bool area_perimeter) {
    std::lock_guard<std::mutex> lock(g_api_mtx);
    g_opt.enable_aspect_ratio_check = aspect_ratio;
    g_opt.enable_strict_symmetry_check = strict_symmetry;
    g_opt.enable_area_perimeter_structure_check = area_perimeter;
    RM_LOG_INFO("[recharge_mark] checks aspect=%d strict_sym=%d area_perim=%d",
                aspect_ratio ? 1 : 0, strict_symmetry ? 1 : 0,
                area_perimeter ? 1 : 0);
}

void ApiSetPoseRefine(bool enabled, double camera_height_mm,
                      double mark_height_mm, double camera_pitch_deg) {
    std::lock_guard<std::mutex> lock(g_api_mtx);
    g_opt.enable_ground_pose_refine = enabled;
    if (camera_height_mm > 50.0) g_opt.camera_height_mm = camera_height_mm;
    if (mark_height_mm > 10.0) g_opt.mark_height_mm = mark_height_mm;
    g_opt.camera_pitch_deg = camera_pitch_deg;
    RM_LOG_INFO("[recharge_mark] pose_refine=%s cam_h=%.0f mark_h=%.0f pitch=%.1f",
                enabled ? "on" : "off", g_opt.camera_height_mm,
                g_opt.mark_height_mm, g_opt.camera_pitch_deg);
}

void ApiSetBuildOverlay(bool enabled, int max_width) {
    std::lock_guard<std::mutex> lock(g_api_mtx);
    g_build_overlay = enabled;
    g_overlay_max_width = max_width > 0 ? max_width : 640;
    RM_LOG_INFO("[recharge_mark] overlay=%s max_w=%d",
                enabled ? "on" : "off", g_overlay_max_width);
}

void ApiSetSaveRaw(const char* dir, uint32_t every_n, uint32_t max_count) {
    std::lock_guard<std::mutex> lock(g_api_mtx);
    g_save_raw_dir = dir ? dir : "";
    g_save_raw = !g_save_raw_dir.empty();
    g_save_every_n = every_n > 0 ? every_n : 1;
    g_save_max = max_count;
    g_save_tick = 0;
    g_save_count = 0;
    RM_LOG_INFO("[recharge_mark] setSaveRaw %s dir=%s every_n=%u max=%u",
                g_save_raw ? "ON" : "OFF", g_save_raw_dir.c_str(),
                g_save_every_n, g_save_max);
}

void ApiSetArucoConfig(int dictionary_id, int target_marker_id,
                       double marker_size_m, double max_reproj_px) {
    std::lock_guard<std::mutex> lock(g_api_mtx);
    g_aruco_opt.dictionary_id = dictionary_id;
    g_aruco_opt.target_marker_id = target_marker_id;
    if (marker_size_m > 0.001) g_aruco_opt.marker_size_m = marker_size_m;
    if (max_reproj_px > 0.1) g_aruco_opt.max_reprojection_error_px = max_reproj_px;
    RM_LOG_INFO("[recharge_mark] setArucoConfig dict=%d id=%d size=%.3fm "
                "reproj<=%.1fpx",
                g_aruco_opt.dictionary_id, g_aruco_opt.target_marker_id,
                g_aruco_opt.marker_size_m, g_aruco_opt.max_reprojection_error_px);
}

Aiper_Recharge::MarkPoseResult* ApiProcessBgr(const uint8_t* bgr, int width,
                                              int height, int step,
                                              uint64_t frame_id) {
    if (!bgr || width <= 0 || height <= 0 || width > 4096 || height > 4096) {
        auto* r = new Aiper_Recharge::MarkPoseResult{};
        std::snprintf(r->error, sizeof(r->error),
                      "invalid bgr buffer w=%d h=%d", width, height);
        return r;
    }
    if (step <= 0) step = width * 3;
    cv::Mat img(height, width, CV_8UC3);
    for (int y = 0; y < height; ++y) {
        std::memcpy(img.ptr(y), bgr + static_cast<size_t>(y) * step,
                    static_cast<size_t>(width) * 3u);
    }
    return ProcessBgrMat(img, width, height, 0, 0, frame_id,
                         /*already_cropped=*/false);
}

Aiper_Recharge::MarkPoseResult* ApiProcessBgrCrop(const uint8_t* bgr, int width,
                                                  int height, int step,
                                                  int crop_x, int crop_y,
                                                  int crop_w, int crop_h,
                                                  uint64_t frame_id) {
    if (!bgr || width <= 0 || height <= 0 || crop_w < 4 || crop_h < 4) {
        auto* r = new Aiper_Recharge::MarkPoseResult{};
        std::snprintf(r->error, sizeof(r->error), "invalid bgr crop");
        return r;
    }
    if (step <= 0) step = width * 3;
    int x0 = std::max(0, crop_x) & ~1;
    int y0 = std::max(0, crop_y) & ~1;
    int x1 = std::min(width, crop_x + crop_w);
    int y1 = std::min(height, crop_y + crop_h);
    x1 = std::min(width, (x1 + 1) & ~1);
    y1 = std::min(height, (y1 + 1) & ~1);
    if (x1 - x0 < 4 || y1 - y0 < 4) {
        auto* r = new Aiper_Recharge::MarkPoseResult{};
        std::snprintf(r->error, sizeof(r->error), "bgr crop too small");
        return r;
    }
    cv::Mat full(height, width, CV_8UC3);
    for (int y = 0; y < height; ++y) {
        std::memcpy(full.ptr(y), bgr + static_cast<size_t>(y) * step,
                    static_cast<size_t>(width) * 3u);
    }
    cv::Mat roi = full(cv::Rect(x0, y0, x1 - x0, y1 - y0)).clone();
    return ProcessBgrMat(roi, width, height, x0, y0, frame_id,
                         /*already_cropped=*/true, /*skip_save=*/false,
                         /*trust_detection_crop=*/true);
}

Aiper_Recharge::MarkPoseResult* ApiProcessLeftNv12(const uint8_t* nv12,
                                                   int width, int height,
                                                   int stride_y,
                                                   uint64_t frame_id) {
    if (!nv12 || width <= 0 || height <= 0) {
        auto* r = new Aiper_Recharge::MarkPoseResult{};
        std::snprintf(r->error, sizeof(r->error), "invalid nv12 buffer");
        return r;
    }
    bool do_save = false;
    bool want_overlay = false;
    {
        std::lock_guard<std::mutex> lock(g_api_mtx);
        do_save = g_save_raw;
        want_overlay = g_build_overlay;
    }
    if (do_save) {
        std::vector<uint8_t> full_bgr;
        if (Nv12RoiToBgr24(nv12, width, height, stride_y,
                           cv::Rect(0, 0, width, height), &full_bgr)) {
            cv::Mat full(height, width, CV_8UC3, full_bgr.data());
            std::lock_guard<std::mutex> lock(g_api_mtx);
            MaybeSaveRawBgr(full, frame_id);
        }
    }
    aiper::recharge::DetectOptions opt;
    {
        std::lock_guard<std::mutex> lock(g_api_mtx);
        opt = g_opt;
    }
    const cv::Rect roi = MakeRoiRect(width, height, opt);
    const bool cropped =
        !(roi.x == 0 && roi.y == 0 && roi.width == width && roi.height == height);
    // 与当时单码一致：overlay 关时用 NV12 Y 平面，存图不影响检测路径。
    if (!want_overlay) {
        cv::Mat gray =
            MakeGrayRoiFromNv12(nv12, width, height, stride_y, roi);
        if (!gray.empty()) {
            auto* rr = ProcessBgrMat(gray.clone(), width, height, roi.x, roi.y,
                                     frame_id, cropped, true, false);
            if (rr) {
                std::snprintf(rr->crop_src, sizeof(rr->crop_src), "static_roi");
            }
            return rr;
        }
    }
    std::vector<uint8_t> bgr;
    if (!Nv12RoiToBgr24(nv12, width, height, stride_y, roi, &bgr)) {
        auto* r = new Aiper_Recharge::MarkPoseResult{};
        std::snprintf(r->error, sizeof(r->error), "nv12 roi to bgr failed");
        return r;
    }
    cv::Mat img(roi.height, roi.width, CV_8UC3, bgr.data());
    auto* rr = ProcessBgrMat(img.clone(), width, height, roi.x, roi.y, frame_id,
                             cropped, true, false);
    if (rr) {
        std::snprintf(rr->crop_src, sizeof(rr->crop_src), "static_roi");
    }
    return rr;
}

Aiper_Recharge::MarkPoseResult* ApiProcessLeftNv12Crop(
    const uint8_t* nv12, int width, int height, int stride_y, int crop_x,
    int crop_y, int crop_w, int crop_h, uint64_t frame_id) {
    if (!nv12 || width <= 0 || height <= 0 || crop_w < 4 || crop_h < 4) {
        auto* r = new Aiper_Recharge::MarkPoseResult{};
        std::snprintf(r->error, sizeof(r->error), "invalid nv12 crop");
        return r;
    }
    int x0 = std::max(0, crop_x) & ~1;
    int y0 = std::max(0, crop_y) & ~1;
    int x1 = std::min(width, crop_x + crop_w);
    int y1 = std::min(height, crop_y + crop_h);
    x1 = std::min(width, (x1 + 1) & ~1);
    y1 = std::min(height, (y1 + 1) & ~1);
    if (x1 - x0 < 4 || y1 - y0 < 4) {
        auto* r = new Aiper_Recharge::MarkPoseResult{};
        std::snprintf(r->error, sizeof(r->error), "crop too small after align");
        return r;
    }
    const cv::Rect roi(x0, y0, x1 - x0, y1 - y0);

    bool do_save = false;
    {
        std::lock_guard<std::mutex> lock(g_api_mtx);
        do_save = g_save_raw;
    }
    if (do_save) {
        std::vector<uint8_t> full_bgr;
        if (Nv12RoiToBgr24(nv12, width, height, stride_y,
                           cv::Rect(0, 0, width, height), &full_bgr)) {
            cv::Mat full(height, width, CV_8UC3, full_bgr.data());
            std::lock_guard<std::mutex> lock(g_api_mtx);
            MaybeSaveRawBgr(full, frame_id);
        }
    }
    cv::Mat gray = MakeGrayRoiFromNv12(nv12, width, height, stride_y, roi);
    if (!gray.empty()) {
        return ProcessBgrMat(gray.clone(), width, height, roi.x, roi.y,
                             frame_id, true, true, true);
    }
    std::vector<uint8_t> bgr;
    if (!Nv12RoiToBgr24(nv12, width, height, stride_y, roi, &bgr)) {
        auto* r = new Aiper_Recharge::MarkPoseResult{};
        std::snprintf(r->error, sizeof(r->error), "nv12 crop to bgr failed");
        return r;
    }
    cv::Mat img(roi.height, roi.width, CV_8UC3, bgr.data());
    return ProcessBgrMat(img.clone(), width, height, roi.x, roi.y, frame_id,
                         true, true, true);
}

Aiper_Recharge::MarkPoseResult* ApiCaptureAndProcess() {
    // feature/obs 无 captureFrame；在线请用 processLeftNv12
    auto* out = new Aiper_Recharge::MarkPoseResult{};
    out->ok = false;
    std::snprintf(out->error, sizeof(out->error),
                  "captureAndProcess unsupported on feature/obs; use processLeftNv12");
    return out;
}

void ApiFreeResult(Aiper_Recharge::MarkPoseResult* result) {
    if (!result) return;
    delete[] result->overlay_bgr;
    result->overlay_bgr = nullptr;
    delete result;
}

int64_t RuntimeStampNs(int64_t timestamp_ns) {
    if (timestamp_ns > 0) {
        return timestamp_ns;
    }
    timespec ts{};
#if defined(CLOCK_BOOTTIME)
    if (clock_gettime(CLOCK_BOOTTIME, &ts) == 0) {
        return static_cast<int64_t>(ts.tv_sec) * 1000000000LL +
               static_cast<int64_t>(ts.tv_nsec);
    }
#endif
    if (clock_gettime(CLOCK_MONOTONIC, &ts) == 0) {
        return static_cast<int64_t>(ts.tv_sec) * 1000000000LL +
               static_cast<int64_t>(ts.tv_nsec);
    }
    return 0;
}

bool RuntimeIntervalBlockedLocked() {
    const uint32_t min_ms =
        g_rt_min_interval_ms.load(std::memory_order_relaxed);
    if (min_ms == 0 || g_rt_last_start.time_since_epoch().count() == 0) {
        return false;
    }
    const double since_ms = std::chrono::duration<double, std::milli>(
                                SteadyClock::now() - g_rt_last_start)
                                .count();
    return since_ms < static_cast<double>(min_ms);
}

bool ApiCanSubmit() {
    if (!g_rt_running.load(std::memory_order_acquire)) {
        return false;
    }
    if (g_rt_busy.load(std::memory_order_acquire)) {
        return false;
    }
    std::lock_guard<std::mutex> lk(g_rt_mtx);
    if (g_rt_pending) {
        return false;
    }
    return !RuntimeIntervalBlockedLocked();
}

void ApiSetResultCallback(Aiper_Recharge::RechargeMarkApi::ResultCallback cb,
                          void* userdata) {
    std::lock_guard<std::mutex> lk(g_rt_mtx);
    g_rt_cb = cb;
    g_rt_cb_ud = userdata;
}

void RuntimeWorkerLoop() {
    while (!g_rt_stop.load(std::memory_order_acquire)) {
        RuntimeJob job;
        {
            std::unique_lock<std::mutex> lk(g_rt_mtx);
            g_rt_cv.wait(lk, [&] {
                return g_rt_stop.load(std::memory_order_acquire) || g_rt_pending;
            });
            if (g_rt_stop.load(std::memory_order_acquire) && !g_rt_pending) {
                break;
            }
            job = std::move(g_rt_job);
            g_rt_pending = false;
            g_rt_last_start = SteadyClock::now();
            g_rt_busy.store(true, std::memory_order_release);
        }
        auto notify_result = [](Aiper_Recharge::MarkPoseResult* res) {
            Aiper_Recharge::RechargeMarkApi::ResultCallback cb = nullptr;
            void* ud = nullptr;
            {
                std::lock_guard<std::mutex> lk(g_rt_mtx);
                cb = g_rt_cb;
                ud = g_rt_cb_ud;
            }
            if (cb) {
                cb(ud, res);
            }
        };
        if (job.nv12.empty() || job.width <= 0 || job.height <= 0) {
            notify_result(nullptr);
            g_rt_busy.store(false, std::memory_order_release);
            continue;
        }

        const uint32_t every_n =
            g_rt_run_every_n.load(std::memory_order_relaxed);
        ++g_rt_tick;
        if (every_n > 1 && (g_rt_tick % every_n) != 0) {
            notify_result(nullptr);
            g_rt_busy.store(false, std::memory_order_release);
            continue;
        }

        Aiper_Recharge::MarkPoseResult* r = nullptr;
        try {
            r = ApiProcessLeftNv12(job.nv12.data(), job.width, job.height,
                                   job.stride_y, job.frame_id);
            if (r) {
                r->post.time_stamp = RuntimeStampNs(job.timestamp_ns);
            }
        } catch (const std::exception& ex) {
            RM_LOG_ERROR("[recharge_mark] worker exception: %s", ex.what());
        } catch (...) {
            RM_LOG_ERROR("[recharge_mark] worker unknown exception");
        }
        notify_result(r);
        ApiFreeResult(r);
        // min_interval 按「距上次开始」计时。cmd=2 宿主 wait settle 已 1:1
        // 串行；若不把 last_start 拨回，pose 发出后还要空等到满间隔（板上
        // 300ms 时表现为 pose 后再隔 ~300ms 才有下一帧 det）。
        {
            const uint32_t min_ms =
                g_rt_min_interval_ms.load(std::memory_order_relaxed);
            std::lock_guard<std::mutex> lk(g_rt_mtx);
            g_rt_last_start =
                SteadyClock::now() - std::chrono::milliseconds(min_ms);
        }
        g_rt_busy.store(false, std::memory_order_release);
    }
}

void ApiStopWorker() {
    g_rt_running.store(false, std::memory_order_release);
    g_rt_stop.store(true, std::memory_order_release);
    g_rt_cv.notify_all();
    if (g_rt_joinable.exchange(false, std::memory_order_acq_rel)) {
        if (g_rt_thread.joinable()) {
            g_rt_thread.join();
        }
    }
    {
        std::lock_guard<std::mutex> lk(g_rt_mtx);
        g_rt_pending = false;
        g_rt_job = RuntimeJob{};
    }
    g_rt_busy.store(false, std::memory_order_release);
}

int ApiStartWorker() {
    if (g_rt_running.load(std::memory_order_acquire) &&
        g_rt_joinable.load(std::memory_order_acquire)) {
        return 0;
    }
    ApiStopWorker();
    g_rt_stop.store(false, std::memory_order_release);
    g_rt_running.store(true, std::memory_order_release);
    g_rt_thread = std::thread(&RuntimeWorkerLoop);
    g_rt_joinable.store(true, std::memory_order_release);
    RM_LOG_INFO("[recharge_mark] worker=on min_interval_ms=%u",
                g_rt_min_interval_ms.load(std::memory_order_relaxed));
    return 0;
}

int ApiSubmitLeftNv12(const uint8_t* nv12, int width, int height, int stride_y,
                      uint64_t frame_id, int64_t timestamp_ns) {
    if (!nv12 || width <= 0 || height <= 0) {
        return 1;
    }
    if (stride_y < width) {
        stride_y = width;
    }
    const size_t bytes =
        static_cast<size_t>(stride_y) * static_cast<size_t>(height) * 3u / 2u;
    if (bytes == 0 || bytes > (8u * 1024u * 1024u)) {
        return 1;
    }
    if (!ApiCanSubmit()) {
        ++g_rt_drop;
        return 1;
    }

    RuntimeJob job;
    job.width = width;
    job.height = height;
    job.stride_y = stride_y;
    job.frame_id = frame_id;
    job.timestamp_ns = timestamp_ns;
    job.nv12.assign(nv12, nv12 + bytes);

    {
        std::lock_guard<std::mutex> lk(g_rt_mtx);
        if (!g_rt_running.load(std::memory_order_acquire) || g_rt_pending ||
            g_rt_busy.load(std::memory_order_relaxed) ||
            RuntimeIntervalBlockedLocked()) {
            ++g_rt_drop;
            return 1;
        }
        g_rt_job = std::move(job);
        g_rt_pending = true;
    }
    g_rt_cv.notify_one();
    return 0;
}

void ScaleCameraK(int img_w, int img_h, const Aiper_Recharge::RechargeCameraK& k,
                  double* fx, double* fy, double* cx, double* cy) {
    *fx = k.fx;
    *fy = k.fy;
    *cx = k.cx;
    *cy = k.cy;
    if (k.cal_width <= 0 || k.cal_height <= 0) {
        return;
    }
    if (k.cal_width == img_w && k.cal_height == img_h) {
        return;
    }
    const double sx = static_cast<double>(img_w) / static_cast<double>(k.cal_width);
    const double sy = static_cast<double>(img_h) / static_cast<double>(k.cal_height);
    *fx *= sx;
    *fy *= sy;
    *cx *= sx;
    *cy *= sy;
}

int SubmitLeftFromPic(const PicData& meta, const uint8_t* data, int def_w,
                      int def_h, size_t max_bytes,
                      const Aiper_Recharge::RechargeCameraK* k,
                      bool apply_k) {
    if (!data) {
        return 1;
    }
    const int w = meta.width > 0 ? static_cast<int>(meta.width) : def_w;
    const int h = meta.height > 0 ? static_cast<int>(meta.height) : def_h;
    const size_t bytes =
        meta.bytes_used > 0 ? static_cast<size_t>(meta.bytes_used) : max_bytes;
    if (w <= 0 || h <= 0 || bytes == 0 || bytes > max_bytes) {
        return 1;
    }
    int stride_y = static_cast<int>(bytes / (static_cast<float>(h) * 1.5f));
    if (stride_y < w) {
        stride_y = w;
    }
    if (apply_k && k && k->fx > 1.0 && k->fy > 1.0) {
        double fx = 0, fy = 0, cx = 0, cy = 0;
        ScaleCameraK(w, h, *k, &fx, &fy, &cx, &cy);
        ApiSetCameraMatrix(fx, fy, cx, cy);
    }
    return ApiSubmitLeftNv12(data, w, h, stride_y, meta.frame_id,
                             static_cast<int64_t>(meta.time_ns));
}

int ApiSubmitFromShm(const void* rgbd, std::size_t rgbd_bytes, const void* gdc,
                     std::size_t gdc_bytes, const Aiper_Recharge::RechargeCameraK* k) {
    (void)rgbd;
    (void)rgbd_bytes;
    bool sdk_scaled = false;
    {
        std::lock_guard<std::mutex> lock(g_api_mtx);
        sdk_scaled =
            std::strcmp(g_host.intrinsics_source, "sdk_scaled") == 0;
    }
    if (!gdc || gdc_bytes != sizeof(CameraGdcData)) {
        return 1;
    }
    const auto* frame = static_cast<const CameraGdcData*>(gdc);
    return SubmitLeftFromPic(frame->left, frame->left_data, GDC_WIDTH,
                             GDC_HEIGHT, GDC_PIC_SIZE, k, sdk_scaled);
}

void ApiDestroy(bool destroy_camera) {
    (void)destroy_camera;
    ApiStopWorker();
    std::lock_guard<std::mutex> lock(g_api_mtx);
    g_cam_override = false;
    g_save_raw = false;
    g_save_raw_dir.clear();
    g_build_overlay = false;
}

}  // namespace

namespace Aiper_Recharge {

RechargeMarkApi* createRechargeMarkApi() {
    auto* api = new RechargeMarkApi();
    api->handle = nullptr;
    api->init = &ApiInit;
    api->setCameraMatrix = &ApiSetCameraMatrix;
    api->useSdkCameraMatrix = &ApiUseSdkCameraMatrix;
    api->setSearchRoi = &ApiSetSearchRoi;
    api->setDetectChecks = &ApiSetDetectChecks;
    api->setPoseRefine = &ApiSetPoseRefine;
    api->setBuildOverlay = &ApiSetBuildOverlay;
    api->setSaveRaw = &ApiSetSaveRaw;
    api->setArucoConfig = &ApiSetArucoConfig;
    api->setMaxPnpTry = &ApiSetMaxPnpTry;
    api->setSkipBcOnAFail = &ApiSetSkipBcOnAFail;
    api->captureAndProcess = &ApiCaptureAndProcess;
    api->processLeftNv12 = &ApiProcessLeftNv12;
    api->processLeftNv12Crop = &ApiProcessLeftNv12Crop;
    api->processBgr = &ApiProcessBgr;
    api->processBgrCrop = &ApiProcessBgrCrop;
    api->freeResult = &ApiFreeResult;
    api->destroy = &ApiDestroy;
    api->resetSession = &ApiResetSession;
    api->loadConfig = &ApiLoadConfig;
    api->getHostConfig = &ApiGetHostConfig;
    api->setResultCallback = &ApiSetResultCallback;
    api->startWorker = &ApiStartWorker;
    api->stopWorker = &ApiStopWorker;
    api->canSubmit = &ApiCanSubmit;
    api->submitLeftNv12 = &ApiSubmitLeftNv12;
    api->submitFromShm = &ApiSubmitFromShm;
    return api;
}

}  // namespace Aiper_Recharge
