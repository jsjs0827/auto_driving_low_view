#pragma once
#include "percep_demo/ai_fusion.hpp"
#include "percep_demo/horizon_dnn_runner.hpp"
#include "percep_demo/percep_demo_config.hpp"

namespace ai_node_percep_demo {
// V0 configuration defaults. These gates run on full-resolution Q8 before
// sampling/projecting the cloud, rather than on the already sparse 3D cloud.
struct DispFilterConfig {
  bool enabled = true;
  float min_disp = 2.0f, max_disp = 144.0f;
  bool grad_check = true;
  int grad_radius = 1;
  float grad_max_px = 2.0f, grad_max_ratio = 0.02f;
  bool speckle_check = true;
  int min_valid_neighbors = 4;
  float min_valid_ratio = 0.05f;
  int log_every_n = 0;
};
struct ExposureGuardConfig {
  bool enabled = true;
  float exp_soft_ms = 8.0f, exp_hard_ms = 20.0f;
  float gain_soft = 4.0f, gain_hard = 16.0f;
  float min_confidence = 0.35f;
  std::string hard_action = "skip_obstacle";
  bool skip_ai_obstacle = true;
  float blur_grad_scale = 0.6f;
  int log_every_n = 0;
};
// Owns the genuine algorithm members without inheriting daye::Function or ROS
// runtime state. Extracted numeric methods retain the original member names.
class MappingAlgorithms {
 public:
  void initLabelFusionClassIds();
  void postprocessSegMask(std::vector<uint8_t>& mask, int width, int height);
  void fuseSemanticLabels(const PointCloudT::Ptr& cloud,
      const std::vector<DnnDetection>& detections,
      const std::vector<uint8_t>& seg_mask, int seg_w, int seg_h,
      int img_w, int img_h, float fx, float fy, float cx, float cy,
      const std::vector<uint32_t>* u_vec = nullptr,
      const std::vector<uint32_t>* v_vec = nullptr);
  void matchDetectionLabels(const PointCloudT::Ptr& cloud,
      const std::vector<DnnDetection>& detections, int img_w, int img_h,
      float fx, float fy, float cx, float cy);
  void parseLabelFusionConfig(const rapidjson::Value& root);
  void runGroundSegmentation(const PointCloudT::Ptr& raw_pts);
  // Numerical port of the reviewed 1104 V0 per-frame ground adaptation.
  Trans adaptGroundTrans(const PointCloudT& cam_cloud);
  bool solveExtrinsicsFromGround(const PointCloudT& cam_cloud, double max_range,
      Trans& out, double& out_height, double& out_tilt_deg,
      double& out_resid_cm, size_t& out_inliers);
  static bool readJsonBool(const rapidjson::Value& v, const char* name, bool fallback) {
    return v.HasMember(name) && v[name].IsBool() ? v[name].GetBool() : fallback;
  }
  DnnConfig dnn_cfg_;
  LabelFusionConfig label_fusion_cfg_;
  PatchworkConfig patchwork_cfg_;
  DetMatchConfig det_match_cfg_;
  DispFilterConfig disp_filter_cfg_;
  ExposureGuardConfig exp_guard_cfg_;
  float frame_exp_ms_ = 0.f, frame_gain_ = 0.f, frame_blur_t_ = 0.f;
  float frame_confidence_ = 1.f;
  bool frame_skip_obstacle_ = false, frame_drop_ = false;
  Trans cali, cali_default;
  double obs_height_thr = 0.05;
  // V0 defaults. cali_default is initialized once from the static extrinsics;
  // cali and the EMA state persist between frames, including rejected fits.
  bool gadapt_enable = true;
  double gadapt_ema_alpha = 1.0;
  double gadapt_max_roll_dev = 10.0;
  double gadapt_max_pitch_dev = 10.0;
  double gadapt_max_yaw_dev = 5.0;  // V0 retains this field; yaw guard is inactive.
  double gadapt_max_h_dev = 0.1;
  double gadapt_max_resid_cm = 4.0;
  int gadapt_min_inliers = 3000;
  bool gadapt_verbose = true;
  double cali_max_range = 4.0;
  Trans gadapt_cur_;
  bool gadapt_inited_ = false;
  size_t gadapt_ok_ = 0, gadapt_rej_ = 0;
  std::shared_ptr<PclCommon> pclCommonPtr{new PclCommon};
  std::shared_ptr<AiFusion> aiFusionPtr;
  std::unique_ptr<patchwork::PatchWorkpp> patchwork_;
  PointCloudT::Ptr ground_cloud{new PointCloudT};
  PointCloudT::Ptr no_ground_cloud{new PointCloudT};
  PointCloudT::Ptr obs_cloud{new PointCloudT};
  PointCloudT::Ptr last_semantic_cloud_{new PointCloudT};
  std::vector<int> passable_seg_ids_, passable_det_ids_;
  std::uint64_t label_fusion_log_tick_ = 0;
};
}  // namespace ai_node_percep_demo
