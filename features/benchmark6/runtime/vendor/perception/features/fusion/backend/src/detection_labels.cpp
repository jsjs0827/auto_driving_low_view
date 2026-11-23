#include "mapping_algorithms.hpp"
#include <cstdint>
namespace ai_node_percep_demo {
namespace {
// Original dnn_runtime.cpp file-local detection palette, retained with the
// extracted matching routine so class-id modulo 12 keeps its original RGB.
static const uint8_t kDetPalette[12][3] = {
    {255, 56, 56},   {255, 157, 151}, {255, 112, 31}, {255, 178, 29},
    {207, 210, 49},  {72, 249, 10},   {0, 128, 255},  {128, 0, 255},
    {255, 0, 128},   {0, 255, 200},   {180, 180, 40}, {100, 200, 255},
};
}  // namespace
void MappingAlgorithms::matchDetectionLabels(const PointCloudT::Ptr& cloud,
    const std::vector<DnnDetection>& detections, int img_w, int img_h,
    float fx, float fy, float cx, float cy) {
  if (!det_match_cfg_.enabled) return;
  if (!cloud || cloud->empty()) {
    if (det_match_cfg_.enabled) {
      PointCloudT::Ptr empty(new PointCloudT);
      last_semantic_cloud_ = empty;

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

  }

  std::cout << "[PercepDemo] det_cloud pts=" << det_cloud->size()
            << " boxes=" << detections.size() << std::endl;
}
}
