#pragma once
#include "percep_demo/ai_fusion.hpp"
#include "percep_demo/horizon_dnn_runner.hpp"
#include "percep_demo/percep_demo_config.hpp"

namespace ai_node_percep_demo {
// Owns the genuine algorithm members without inheriting daye::Function or ROS
// runtime state. Extracted numeric methods retain the original member names.
class MappingAlgorithms {
 public:
  void initLabelFusionClassIds();
  void postprocessSegMask(std::vector<uint8_t>& mask, int width, int height);
  void fuseSemanticLabels(const PointCloudT::Ptr& cloud,
      const std::vector<DnnDetection>& detections,
      const std::vector<uint8_t>& seg_mask, int seg_w, int seg_h,
      int img_w, int img_h, float fx, float fy, float cx, float cy);
  void matchDetectionLabels(const PointCloudT::Ptr& cloud,
      const std::vector<DnnDetection>& detections, int img_w, int img_h,
      float fx, float fy, float cx, float cy);
  void parseLabelFusionConfig(const rapidjson::Value& root);
  void runGroundSegmentation(const PointCloudT::Ptr& raw_pts);
  static bool readJsonBool(const rapidjson::Value& v, const char* name, bool fallback) {
    return v.HasMember(name) && v[name].IsBool() ? v[name].GetBool() : fallback;
  }
  DnnConfig dnn_cfg_;
  LabelFusionConfig label_fusion_cfg_;
  PatchworkConfig patchwork_cfg_;
  DetMatchConfig det_match_cfg_;
  Trans cali;
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
