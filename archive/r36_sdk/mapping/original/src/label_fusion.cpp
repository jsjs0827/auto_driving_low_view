/**
 * @file label_fusion.cpp
 * @brief fastdnn 语义融合移植：检测框+分割掩码给输入点云打标签（点数与输入一致）
 */
#include "percep_demo/percep_demo.h"
#include "percep_demo/pcl_common.hpp"

#if defined(PERCEP_DNN_ENABLE)

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <optional>
#include <queue>
#include <string>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <Eigen/Core>
#include <pcl/common/transforms.h>

using namespace std;

namespace ai_node_percep_demo {

namespace {

using SteadyClock = std::chrono::steady_clock;

inline double ElapsedMs(SteadyClock::time_point t0) {
  return std::chrono::duration<double, std::milli>(SteadyClock::now() - t0)
      .count();
}

std::vector<int> ResolveClassIds(const std::string& want_csv,
                                 const std::string& all_csv) {
  auto split = [](const std::string& s) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
      if (c == ',') {
        const std::string tok = TrimCsvToken(cur);
        if (!tok.empty()) out.push_back(tok);
        cur.clear();
      } else {
        cur.push_back(c);
      }
    }
    const std::string tok = TrimCsvToken(cur);
    if (!tok.empty()) out.push_back(tok);
    return out;
  };
  const std::vector<std::string> want = split(want_csv);
  const std::vector<std::string> all = ParseSegClassNames(all_csv);
  std::vector<int> ids;
  for (const auto& w : want) {
    int numeric_id = -1;
    const auto parsed =
        std::from_chars(w.data(), w.data() + w.size(), numeric_id);
    if (parsed.ec == std::errc{} && parsed.ptr == w.data() + w.size()) {
      if (numeric_id >= 0 &&
          numeric_id < static_cast<int>(all.size())) {
        ids.push_back(numeric_id);
      }
      continue;
    }
    for (size_t i = 0; i < all.size(); ++i) {
      if (all[i] == w) {
        ids.push_back(static_cast<int>(i));
        break;
      }
    }
  }
  return ids;
}

}  // namespace

void PercepDemo::initLabelFusionClassIds() {
  passable_seg_ids_.clear();
  passable_det_ids_.clear();
  passable_seg_ids_ =
      ResolveClassIds(label_fusion_cfg_.passable_classes, dnn_cfg_.seg.class_names);
  passable_det_ids_ =
      ResolveClassIds(label_fusion_cfg_.passable_classes, dnn_cfg_.detect.class_names);
  if (aiFusionPtr) {
    aiFusionPtr->setLabelMap(detail::BuildAiGridLabelMap(
        ParseSegClassNames(dnn_cfg_.detect.class_names),
        ParseSegClassNames(dnn_cfg_.seg.class_names), passable_det_ids_,
        passable_seg_ids_, label_fusion_cfg_.nongrass_seg_as_obstacle));
  }
  std::cout << "[PercepDemo][label_fusion] passable seg_ids=";
  for (size_t i = 0; i < passable_seg_ids_.size(); ++i) {
    if (i) std::cout << ',';
    std::cout << passable_seg_ids_[i];
  }
  std::cout << " det_ids=";
  for (size_t i = 0; i < passable_det_ids_.size(); ++i) {
    if (i) std::cout << ',';
    std::cout << passable_det_ids_[i];
  }
  std::cout << " classes=\"" << label_fusion_cfg_.passable_classes << "\""
            << " seg_names=\"" << dnn_cfg_.seg.class_names << "\""
            << std::endl;
}

void PercepDemo::postprocessSegMask(std::vector<uint8_t>& seg_mask,
                                    int width, int height) {
    if (!label_fusion_cfg_.seg_mask_postprocess || width <= 0 || height <= 0 ||
        seg_mask.size() != static_cast<size_t>(width) * height) {
        return;
    }
    auto is_passable = [&](int c) {
        for (int id : passable_seg_ids_) {
            if (id == c) return true;
        }
        return false;
    };

    const int majority_r =
        std::min(4, std::max(0, label_fusion_cfg_.seg_mask_majority_radius));
    if (majority_r > 0) {
        const std::vector<uint8_t> src = seg_mask;
        std::vector<int> counts(256, 0);
        std::vector<uint8_t> touched;
        touched.reserve(static_cast<size_t>((majority_r * 2 + 1) *
                                            (majority_r * 2 + 1)));
        for (int y = 0; y < height; ++y) {
            const int y0 = std::max(0, y - majority_r);
            const int y1 = std::min(height - 1, y + majority_r);
            for (int x = 0; x < width; ++x) {
                const int x0 = std::max(0, x - majority_r);
                const int x1 = std::min(width - 1, x + majority_r);
                touched.clear();
                for (int yy = y0; yy <= y1; ++yy) {
                    const size_t row = static_cast<size_t>(yy) * width;
                    for (int xx = x0; xx <= x1; ++xx) {
                        const uint8_t c = src[row + xx];
                        if (counts[c]++ == 0) touched.push_back(c);
                    }
                }
                const uint8_t center =
                    src[static_cast<size_t>(y) * width + x];
                uint8_t best = center;
                int best_n = counts[center];
                for (uint8_t c : touched) {
                    if (counts[c] > best_n) {
                        best = c;
                        best_n = counts[c];
                    }
                }
                seg_mask[static_cast<size_t>(y) * width + x] = best;
                for (uint8_t c : touched) counts[c] = 0;
            }
        }
    }

    const int close_r =
        std::min(4, std::max(0, label_fusion_cfg_.seg_mask_close_radius));
    const int close_iters =
        std::min(3, std::max(1, label_fusion_cfg_.seg_mask_close_iterations));
    if (close_r <= 0) return;

    const size_t pixels = static_cast<size_t>(width) * height;
    std::vector<uint8_t> binary(pixels), dilated(pixels), closed(pixels);
    for (size_t i = 0; i < pixels; ++i) {
        binary[i] = is_passable(seg_mask[i]) ? 0 : 1;
    }
    for (int iter = 0; iter < close_iters; ++iter) {
        std::fill(dilated.begin(), dilated.end(), 0);
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                bool hit = false;
                for (int dy = -close_r; !hit && dy <= close_r; ++dy) {
                    const int yy = y + dy;
                    if (yy < 0 || yy >= height) continue;
                    for (int dx = -close_r; dx <= close_r; ++dx) {
                        const int xx = x + dx;
                        if (xx < 0 || xx >= width) continue;
                        if (binary[static_cast<size_t>(yy) * width + xx]) {
                            hit = true;
                            break;
                        }
                    }
                }
                dilated[static_cast<size_t>(y) * width + x] = hit ? 1 : 0;
            }
        }
        std::fill(closed.begin(), closed.end(), 0);
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                bool all = true;
                for (int dy = -close_r; all && dy <= close_r; ++dy) {
                    const int yy = y + dy;
                    if (yy < 0 || yy >= height) continue;
                    for (int dx = -close_r; dx <= close_r; ++dx) {
                        const int xx = x + dx;
                        if (xx < 0 || xx >= width) continue;
                        if (!dilated[static_cast<size_t>(yy) * width + xx]) {
                            all = false;
                            break;
                        }
                    }
                }
                closed[static_cast<size_t>(y) * width + x] = all ? 1 : 0;
            }
        }
        binary.swap(closed);
    }

    // 闭运算只填障碍凹口/小孔，不删除原障碍；新增像素继承邻近障碍类别。
    const std::vector<uint8_t> labels = seg_mask;
    const int search_r = std::max(1, close_r * 2);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const size_t idx = static_cast<size_t>(y) * width + x;
            if (!binary[idx] || !is_passable(seg_mask[idx])) continue;
            uint8_t replacement = 0;
            bool found = false;
            for (int r = 1; !found && r <= search_r; ++r) {
                const int y0 = std::max(0, y - r);
                const int y1 = std::min(height - 1, y + r);
                const int x0 = std::max(0, x - r);
                const int x1 = std::min(width - 1, x + r);
                for (int yy = y0; !found && yy <= y1; ++yy) {
                    for (int xx = x0; xx <= x1; ++xx) {
                        const uint8_t c =
                            labels[static_cast<size_t>(yy) * width + xx];
                        if (!is_passable(c)) {
                            replacement = c;
                            found = true;
                            break;
                        }
                    }
                }
            }
            if (found) seg_mask[idx] = replacement;
        }
    }
}

void PercepDemo::fuseSemanticLabels(const PointCloudT::Ptr& cloud,
                                    const std::vector<DnnDetection>& dets,
                                    const std::vector<uint8_t>& seg_mask,
                                    int seg_w, int seg_h, int img_w,
                                    int img_h, float fx, float fy, float cx,
                                    float cy) {
    if (!cloud || cloud->empty()) {
      // 本帧无可用点云也发空 det_cloud，避免可视化残留
      if (det_match_cfg_.enabled) {
        PointCloudT::Ptr empty(new PointCloudT);
        last_semantic_cloud_ = empty;
        publish_ros_rgbcloud(empty, det_cloud_pub_, parent_frame, false);
      } else {
        last_semantic_cloud_.reset(new PointCloudT);
      }
      return;
    }
    const auto t_fuse_start = SteadyClock::now();
    const size_t n_cloud = cloud->size();

    const int proj_w = (seg_w > 0) ? seg_w : img_w;
    const int proj_h = (seg_h > 0) ? seg_h : img_h;
    if (proj_w != img_w || proj_h != img_h || !std::isfinite(fx) ||
        !std::isfinite(fy) || !std::isfinite(cx) || !std::isfinite(cy) ||
        fx <= 0.0F || fy <= 0.0F) {
      LOG_ERROR("[PercepDemo] semantic fusion skipped: invalid intrinsics");
      last_semantic_cloud_.reset(new PointCloudT);
      return;
    }
    const bool have_seg =
        (!seg_mask.empty() && proj_w > 0 && proj_h > 0 &&
         seg_mask.size() ==
             static_cast<size_t>(proj_w) * static_cast<size_t>(proj_h));

    auto is_passable_seg = [&](int c) {
        for (int id : passable_seg_ids_) if (id == c) return true;
        return false;
    };
    auto is_passable_det = [&](int c) {
        for (int id : passable_det_ids_) if (id == c) return true;
        return false;
    };

    // 语义: 0=unknown,1=passable,2=det障碍,3=seg障碍
    enum Sem { SEM_UNKNOWN = 0, SEM_PASS = 1, SEM_OBS_DET = 2, SEM_OBS_SEG = 3 };

    size_t n_pass = 0, n_obs_det = 0, n_obs_seg = 0, n_unknown = 0;
    size_t n_gate_range = 0, n_gate_height = 0, n_valid = 0, n_gate_depth = 0;

    // 有效点云门：前方距离 ≤ max_range_m、离地高度 ≤ min(max_height_m, keep_below_cam_m)
    const double max_range_mm =
        static_cast<double>(label_fusion_cfg_.max_range_m) * 1000.0;
    double max_height_mm =
        static_cast<double>(label_fusion_cfg_.max_height_m) * 1000.0;
    if (label_fusion_cfg_.keep_below_cam_m > 0.0f) {
        const double kb =
            static_cast<double>(label_fusion_cfg_.keep_below_cam_m) * 1000.0;
        if (kb < max_height_mm) max_height_mm = kb;  // 相机高度以下 19.5cm 上限
    }
    const double sensor_h_mm = patchwork_cfg_.sensor_height * 1000.0;

    // 输出与输入一一对应：全量回投影打标签，超距/过高点保留为 unknown，不丢点
    struct VP {
      float X, Y, Z;
      int seg_c;
      int best_det;
      int u, v;
      size_t src_i;
      bool gated;
    };
    std::vector<VP> vps;
    vps.reserve(n_cloud);
    std::vector<std::vector<float>> box_z(dets.size());

    for (size_t pi = 0; pi < n_cloud; ++pi) {
        const auto& p = cloud->points[pi];
        const double X = (p.x * 1000.0), Y = (p.y * 1000.0), Z = (p.z * 1000.0);  // mm, 左相机系(Y下为正)
        if (!std::isfinite(X) || !std::isfinite(Y) || !std::isfinite(Z) ||
            Z <= 1.0 || Z > max_range_mm || std::abs(X) > max_range_mm ||
            std::abs(Y) > max_range_mm) {
            ++n_gate_range;
            vps.push_back({static_cast<float>(X), static_cast<float>(Y),
                           static_cast<float>(Z), -1, -1, -1, -1, pi, true});
            continue;
        }
        const double height_mm = sensor_h_mm - Y;  // 离地高度(向上为正)
        if (height_mm > max_height_mm) {
            ++n_gate_height;
            vps.push_back({static_cast<float>(X), static_cast<float>(Y),
                           static_cast<float>(Z), -1, -1, -1, -1, pi, true});
            continue;
        }
        ++n_valid;
        const double projected_u =
            static_cast<double>(fx) * (X / Z) + static_cast<double>(cx);
        const double projected_v =
            static_cast<double>(fy) * (Y / Z) + static_cast<double>(cy);
        if (!std::isfinite(projected_u) || !std::isfinite(projected_v) ||
            projected_u < static_cast<double>(std::numeric_limits<int>::min()) ||
            projected_u > static_cast<double>(std::numeric_limits<int>::max()) ||
            projected_v < static_cast<double>(std::numeric_limits<int>::min()) ||
            projected_v > static_cast<double>(std::numeric_limits<int>::max())) {
            ++n_gate_range;
            vps.push_back({static_cast<float>(X), static_cast<float>(Y),
                           static_cast<float>(Z), -1, -1, -1, -1, pi, true});
            continue;
        }
        int u = static_cast<int>(projected_u);
        int v = static_cast<int>(projected_v);

        int seg_c = -1;
        int best_det = -1;
        if (u >= 0 && u < proj_w && v >= 0 && v < proj_h) {
            if (have_seg) seg_c = seg_mask[static_cast<size_t>(v) * proj_w + u];
            if (!(have_seg && is_passable_seg(seg_c))) {
                const float ub = (proj_w == img_w)
                                     ? static_cast<float>(u)
                                     : static_cast<float>(u) * img_w / proj_w;
                const float vb = (proj_h == img_h)
                                     ? static_cast<float>(v)
                                     : static_cast<float>(v) * img_h / proj_h;
                float best_conf = label_fusion_cfg_.det_score_min;
                for (size_t di = 0; di < dets.size(); ++di) {
                    const auto& d = dets[di];
                    if (d.confidence < label_fusion_cfg_.det_score_min) continue;
                    if (ub >= d.x1 && ub <= d.x2 && vb >= d.y1 && vb <= d.y2) {
                        if (d.confidence >= best_conf) {
                            best_conf = d.confidence;
                            best_det = static_cast<int>(di);
                        }
                    }
                }
                // 障碍候选（非可通行检测类）→ 收集框内深度，供近深度门估计
                if (best_det >= 0 && !is_passable_det(dets[best_det].class_id)) {
                    box_z[best_det].push_back(static_cast<float>(Z));
                }
            }
        } else {
            u = -1;
            v = -1;
        }
        vps.push_back({static_cast<float>(X), static_cast<float>(Y),
                       static_cast<float>(Z), seg_c, best_det, u, v, pi, false});
    }

    // 每检测框：深度排序 → 取 Σ(z²) 最大的连续主体簇（空隙>gap 切断）→
    // 再夹 span；抑制射线拖尾，同时避免近密远疏抢走 2m+ 主体
    const float span_mm =
        std::max(0.f, label_fusion_cfg_.det_depth_span_m) * 1000.f;
    const float gap_mm =
        std::max(0.05f, label_fusion_cfg_.det_depth_gap_m) * 1000.f;
    const float z_margin_mm = 30.f;  // 主体边界留少许余量
    std::vector<float> box_zmin(dets.size(), -1e30f);
    std::vector<float> box_zmax(dets.size(), 1e30f);  // 默认不设门
    const bool do_depth_gate =
        (label_fusion_cfg_.det_depth_span_m > 1e-3f ||
         label_fusion_cfg_.det_depth_gap_m > 1e-3f);
    if (do_depth_gate) {
        for (size_t d = 0; d < dets.size(); ++d) {
            auto& zs = box_z[d];
            if (static_cast<int>(zs.size()) < label_fusion_cfg_.det_depth_min_pts)
                continue;
            std::sort(zs.begin(), zs.end());
            // 扫描连续深度簇；用 Σ(z²) 加权（补偿透视：近密远疏），避免近处杂点
            // 抢走 2m+ 主体；权重接近时取更近簇以抑制射线拖尾
            size_t best_s = 0, best_e = 0;
            int best_n = -1;
            double best_w = -1.0;
            const int need = std::max(
                3, std::min(label_fusion_cfg_.det_depth_min_pts,
                            static_cast<int>(zs.size()) / 10));
            size_t i = 0;
            while (i < zs.size()) {
                size_t j = i;
                while (j + 1 < zs.size() &&
                       (zs[j + 1] - zs[j]) <= gap_mm) {
                    ++j;
                }
                const int run_n = static_cast<int>(j - i + 1);
                if (run_n >= need) {
                    double run_w = 0.0;
                    for (size_t k = i; k <= j; ++k) {
                        const double zm = static_cast<double>(zs[k]) * 1e-3;
                        run_w += zm * zm;  // 透视密度补偿
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
                }
                i = j + 1;
            }
            if (best_n < 0) {
                // 回退：中位数为中心 ± span/2（比近端 10% 更稳，远处主体不易丢）
                const size_t k = zs.size() / 2;
                const float half =
                    (span_mm > 1.f ? span_mm : 800.f) * 0.5f;
                box_zmin[d] = zs[k] - half - z_margin_mm;
                box_zmax[d] = zs[k] + half + z_margin_mm;
                continue;
            }
            float z0 = zs[best_s];
            float z1 = zs[best_e];
            if (span_mm > 1.f && (z1 - z0) > span_mm) {
                z1 = z0 + span_mm;
            }
            box_zmin[d] = z0 - z_margin_mm;
            box_zmax[d] = z1 + z_margin_mm;
        }
    }

    // Pass 2a: 定语义（含深度门），并缓存检测障碍的体素键。
    std::vector<uint8_t> sem_arr(vps.size(), SEM_UNKNOWN);
    std::vector<int> lbl_arr(vps.size(), -1);
    const bool do_cluster = label_fusion_cfg_.cluster_filter &&
                            label_fusion_cfg_.cluster_voxel_m > 1e-3f;
    const double vox_mm =
        static_cast<double>(label_fusion_cfg_.cluster_voxel_m) * 1000.0;
    const double inv_vox_mm = vox_mm > 1e-9 ? 1.0 / vox_mm : 0.0;
    auto vox_key = [&](double X, double Y, double Z) -> int64_t {
        const int64_t ix = static_cast<int64_t>(std::floor(X * inv_vox_mm)) + 100000;
        const int64_t iy = static_cast<int64_t>(std::floor(Y * inv_vox_mm)) + 100000;
        const int64_t iz = static_cast<int64_t>(std::floor(Z * inv_vox_mm)) + 100000;
        return (ix * 262144LL + iy) * 262144LL + iz;
    };
    // 每点体素键只算一次，Pass 2a/2b 复用，避免重复 floor/除法与哈希键重建
    std::vector<int64_t> vox_key_arr;
    if (do_cluster) {
        vox_key_arr.assign(vps.size(), 0);
    }

    for (size_t i = 0; i < vps.size(); ++i) {
        const auto& q = vps[i];
        int sem = SEM_UNKNOWN;
        int label_class = -1;  // 点云 label：det=0..255，seg=kAiSegLabelBase+模型id
        if (q.gated) {
            sem_arr[i] = static_cast<uint8_t>(SEM_UNKNOWN);
            lbl_arr[i] = -1;
            continue;
        }
        if (have_seg && is_passable_seg(q.seg_c)) {
            sem = SEM_PASS;
            label_class = EncodeSegLabel(q.seg_c);
        } else if (q.best_det >= 0) {
            const auto& d = dets[q.best_det];
            if (is_passable_det(d.class_id)) {
                sem = SEM_PASS;
                label_class = EncodeDetLabel(d.class_id);
            } else if (static_cast<float>(q.Z) >= box_zmin[q.best_det] &&
                       static_cast<float>(q.Z) <= box_zmax[q.best_det]) {
                sem = SEM_OBS_DET;
                label_class = EncodeDetLabel(d.class_id);
            } else {
                ++n_gate_depth;  // 不在近处主体深度簇内 → 丢弃拖尾/射线离散点
            }
        } else if (have_seg && label_fusion_cfg_.nongrass_seg_as_obstacle &&
                   q.seg_c >= 0 && !is_passable_seg(q.seg_c)) {
            // 掩码腐蚀：边缘像素邻域含草/越界则不标分割障碍，收紧轮廓鬼影
            bool solid = true;
            const int erode = label_fusion_cfg_.seg_mask_erode_px;
            if (erode > 0) {
                solid = false;
                if (q.u >= erode && q.v >= erode &&
                    q.u + erode < proj_w && q.v + erode < proj_h) {
                    solid = true;
                    for (int dy = -erode; solid && dy <= erode; ++dy) {
                        for (int dx = -erode; solid && dx <= erode; ++dx) {
                            const int c = seg_mask[static_cast<size_t>(q.v + dy) *
                                                       proj_w +
                                                   (q.u + dx)];
                            if (c < 0 || is_passable_seg(c)) solid = false;
                        }
                    }
                }
            }
            if (solid) {
                sem = SEM_OBS_SEG;
                label_class = EncodeSegLabel(q.seg_c);
            }
        }
        sem_arr[i] = static_cast<uint8_t>(sem);
        lbl_arr[i] = label_class;
        // 体素密度只统计检测障碍（分割背景结构不在此剔除）
        if (do_cluster && sem == SEM_OBS_DET) {
            const int64_t vk = vox_key(q.X, q.Y, q.Z);
            vox_key_arr[i] = vk;
        }
    }

    // Pass 2b: 稀疏体素剔除 → 每检测框连通域只留最大主体 → 栅格/调试点云
    size_t n_cluster_removed = 0;
    // 检测拖尾/离散点剔除后保持 unknown，不回落分割障碍，避免边缘错误点参与避障
    auto demote_det = [&](size_t i) {
        sem_arr[i] = static_cast<uint8_t>(SEM_UNKNOWN);
        lbl_arr[i] = -1;
    };

    if (do_cluster) {
        // 每检测框：体素 26-邻域连通，只保留点数最多的主体；
        // 远处框总点数 < min_component 时仍保留最大连通域，避免整框被清空
        const int min_comp = label_fusion_cfg_.cluster_min_component_pts;
        for (size_t di = 0; di < dets.size(); ++di) {
            std::unordered_map<int64_t, std::vector<size_t>> vox_pts;
            int box_total = 0;
            for (size_t i = 0; i < vps.size(); ++i) {
                if (sem_arr[i] != SEM_OBS_DET) continue;
                if (vps[i].best_det != static_cast<int>(di)) continue;
                vox_pts[vox_key_arr[i]].push_back(i);
                ++box_total;
            }
            if (vox_pts.empty()) continue;

            std::unordered_set<int64_t> visited;
            int best_count = 0;
            std::vector<size_t> best_pts;
            for (const auto& kv : vox_pts) {
                if (visited.count(kv.first)) continue;
                std::queue<int64_t> qv;
                std::vector<size_t> comp_pts;
                qv.push(kv.first);
                visited.insert(kv.first);
                while (!qv.empty()) {
                    const int64_t vk = qv.front();
                    qv.pop();
                    auto it = vox_pts.find(vk);
                    if (it != vox_pts.end()) {
                        comp_pts.insert(comp_pts.end(), it->second.begin(),
                                        it->second.end());
                    }
                    // 解包体素坐标（与 vox_key 互逆）
                    int64_t t = vk;
                    const int64_t iz = t % 262144LL;
                    t /= 262144LL;
                    const int64_t iy = t % 262144LL;
                    const int64_t ix = t / 262144LL;
                    for (int dx = -1; dx <= 1; ++dx) {
                        for (int dy = -1; dy <= 1; ++dy) {
                            for (int dz = -1; dz <= 1; ++dz) {
                                if (dx == 0 && dy == 0 && dz == 0) continue;
                                const int64_t nk =
                                    ((ix + dx) * 262144LL + (iy + dy)) *
                                        262144LL +
                                    (iz + dz);
                                if (!vox_pts.count(nk) || visited.count(nk))
                                    continue;
                                visited.insert(nk);
                                qv.push(nk);
                            }
                        }
                    }
                }
                const int n = static_cast<int>(comp_pts.size());
                if (n > best_count) {
                    best_count = n;
                    best_pts = std::move(comp_pts);
                }
            }

            std::unordered_set<size_t> keep;
            // 远处 stride 后常被打成多块，最大块可能 < min_comp 但仍是真障碍：
            // - 达到 min_comp：正常保留
            // - 框内总点 < min_comp：稀疏远框，保留最大块
            // - 最大块已占框内半数以上：主体被打散时仍保留，勿整框清空
            const bool keep_sparse_box =
                (box_total > 0 && box_total < min_comp && best_count > 0);
            const bool keep_majority =
                (best_count >= 3 && best_count * 2 >= box_total);
            if (best_count >= min_comp || keep_sparse_box || keep_majority) {
                keep.insert(best_pts.begin(), best_pts.end());
            }
            for (const auto& kv : vox_pts) {
                for (size_t i : kv.second) {
                    if (!keep.count(i)) {
                        ++n_cluster_removed;
                        demote_det(i);
                    }
                }
            }
        }
    }

    // Pass 2c: 分割障碍拖尾 —— 同射线近端连续深度簇 + 连通域最小点数
    size_t n_seg_removed = 0;
    auto demote_seg = [&](size_t i) {
        if (sem_arr[i] != SEM_OBS_SEG) return;
        sem_arr[i] = static_cast<uint8_t>(SEM_UNKNOWN);
        lbl_arr[i] = -1;
        ++n_seg_removed;
    };
    if (label_fusion_cfg_.seg_cluster_filter) {
        const float seg_gap_mm =
            std::max(0.05f, label_fusion_cfg_.seg_depth_gap_m) * 1000.f;
        const int bin_px = std::max(1, label_fusion_cfg_.seg_ray_bin_px);
        std::unordered_map<int64_t, std::vector<std::pair<float, size_t>>> rays;
        rays.reserve(vps.size() / 8 + 1);
        for (size_t i = 0; i < vps.size(); ++i) {
            if (sem_arr[i] != SEM_OBS_SEG) continue;
            const auto& q = vps[i];
            if (q.u < 0 || q.v < 0) continue;
            const int64_t bu = q.u / bin_px;
            const int64_t bv = q.v / bin_px;
            const int64_t key = (bu << 32) | static_cast<uint32_t>(bv & 0xffffffffLL);
            rays[key].emplace_back(q.Z, i);
        }
        for (auto& kv : rays) {
            auto& pts = kv.second;
            if (pts.size() < 2) continue;
            std::sort(pts.begin(), pts.end(),
                      [](const auto& a, const auto& b) { return a.first < b.first; });
            // 近端起取第一段连续簇，空隙 > gap 后的拖尾全部降级
            size_t end = 0;
            while (end + 1 < pts.size() &&
                   (pts[end + 1].first - pts[end].first) <= seg_gap_mm) {
                ++end;
            }
            for (size_t k = end + 1; k < pts.size(); ++k) {
                demote_seg(pts[k].second);
            }
        }

        // 连通域：保留所有 ≥ min 的主体（多障碍并存），丢弃孤立离散团
        if (label_fusion_cfg_.cluster_voxel_m > 1e-3f) {
            const int min_seg_comp = label_fusion_cfg_.seg_min_component_pts;
            std::unordered_map<int64_t, std::vector<size_t>> vox_pts;
            for (size_t i = 0; i < vps.size(); ++i) {
                if (sem_arr[i] != SEM_OBS_SEG) continue;
                vox_pts[vox_key(vps[i].X, vps[i].Y, vps[i].Z)].push_back(i);
            }
            std::unordered_set<int64_t> visited;
            for (const auto& seed : vox_pts) {
                if (visited.count(seed.first)) continue;
                std::queue<int64_t> qv;
                std::vector<size_t> comp_pts;
                qv.push(seed.first);
                visited.insert(seed.first);
                while (!qv.empty()) {
                    const int64_t vk = qv.front();
                    qv.pop();
                    auto it = vox_pts.find(vk);
                    if (it != vox_pts.end()) {
                        comp_pts.insert(comp_pts.end(), it->second.begin(),
                                        it->second.end());
                    }
                    int64_t t = vk;
                    const int64_t iz = t % 262144LL;
                    t /= 262144LL;
                    const int64_t iy = t % 262144LL;
                    const int64_t ix = t / 262144LL;
                    for (int dx = -1; dx <= 1; ++dx) {
                        for (int dy = -1; dy <= 1; ++dy) {
                            for (int dz = -1; dz <= 1; ++dz) {
                                if (dx == 0 && dy == 0 && dz == 0) continue;
                                const int64_t nk =
                                    ((ix + dx) * 262144LL + (iy + dy)) *
                                        262144LL +
                                    (iz + dz);
                                if (!vox_pts.count(nk) || visited.count(nk))
                                    continue;
                                visited.insert(nk);
                                qv.push(nk);
                            }
                        }
                    }
                }
                if (static_cast<int>(comp_pts.size()) < min_seg_comp) {
                    for (size_t i : comp_pts) demote_seg(i);
                }
            }
        }
    }

    PointCloudT::Ptr labeled(new PointCloudT(*cloud));
    labeled->width = static_cast<uint32_t>(n_cloud);
    labeled->height = 1;
    labeled->is_dense = cloud->is_dense;
    size_t n_labeled = 0;
    for (size_t i = 0; i < vps.size(); ++i) {
        auto& pt = labeled->points[vps[i].src_i];
        const auto& q = vps[i];
        const int sem = sem_arr[i];
        int label_class = lbl_arr[i];

        if (sem == SEM_PASS) ++n_pass;
        else if (sem == SEM_OBS_DET) ++n_obs_det;
        else if (sem == SEM_OBS_SEG) ++n_obs_seg;
        else ++n_unknown;

        // 输出点云优先级：框内仅草可覆盖检测；保留的检测标签不被非草覆盖。
        const bool is_grass =
            have_seg && q.seg_c >= 0 && is_passable_seg(q.seg_c);
        if (is_grass) {
            label_class = EncodeSegLabel(q.seg_c);
        } else if (label_class >= 0 && !IsSegPointLabel(label_class)) {
            // 检测有效类（人/猫/狗/桩）保留
        } else if (label_class < 0) {
            if (have_seg && q.seg_c >= 0) {
                label_class = EncodeSegLabel(q.seg_c);
            }
        }
        if (label_class >= 0) ++n_labeled;
        uint8_t r = 90, g = 90, b = 90;
        if (sem == SEM_PASS ||
            (IsSegPointLabel(label_class) &&
             is_passable_seg(DecodeSegClassId(label_class)))) {
            r = 0;
            g = 200;
            b = 0;
        } else if (sem == SEM_OBS_DET) {
            DetObstacleVisRgb(label_class, dnn_cfg_.detect.class_names, r, g, b);
        } else if (sem == SEM_OBS_SEG) {
            r = 240;
            g = 140;
            b = 0;
        } else {
            r = 40;
            g = 40;
            b = 40;
        }
        pt.r = r;
        pt.g = g;
        pt.b = b;
        pt.label = label_class;
        pt.confidence = label_class >= 0 ? 1.f : 0.f;
    }

    auto apply_cali = [&](PointCloudT::Ptr& cam_cloud) {
      PointCloudT::Ptr out(new PointCloudT);
      if (cam_cloud && !cam_cloud->empty()) {
        *out = *cam_cloud;
        if (pclCommonPtr) {
          Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
          pclCommonPtr->EulerToMatrix(cali, T);
          pcl::transformPointCloud(*out, *out, T);
        }
      }
      last_semantic_cloud_ = out;  // base_link，点数与输入一致
      if (det_match_cfg_.enabled) {
        publish_ros_rgbcloud(out, det_cloud_pub_, parent_frame, false);
      }
    };
    apply_cali(labeled);
    {
      ++label_fusion_log_tick_;
      const size_t n_out =
          last_semantic_cloud_ ? last_semantic_cloud_->size() : 0;
      std::cout << "[PercepDemo][label_fusion]"
                << " in=" << n_cloud
                << " out=" << n_out
                << " labeled=" << n_labeled
                << " pass=" << n_pass
                << " unknown=" << n_unknown
                << " obs_det=" << n_obs_det
                << " obs_seg=" << n_obs_seg
                << " time=" << static_cast<int>(ElapsedMs(t_fuse_start)) << "ms"
                << std::endl;
      LOG_INFO("[PercepDemo][label_fusion] in=%llu out=%llu labeled=%llu pass=%llu "
               "unknown=%llu obs_det=%llu obs_seg=%llu time=%dms",
               static_cast<unsigned long long>(n_cloud),
               static_cast<unsigned long long>(n_out),
               static_cast<unsigned long long>(n_labeled),
               static_cast<unsigned long long>(n_pass),
               static_cast<unsigned long long>(n_unknown),
               static_cast<unsigned long long>(n_obs_det),
               static_cast<unsigned long long>(n_obs_seg),
               static_cast<int>(ElapsedMs(t_fuse_start)));
    }
}

}  // namespace ai_node_percep_demo

#endif  // PERCEP_DNN_ENABLE
