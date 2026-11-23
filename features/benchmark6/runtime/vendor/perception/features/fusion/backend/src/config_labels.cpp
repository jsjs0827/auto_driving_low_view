#include "mapping_algorithms.hpp"
namespace ai_node_percep_demo {
namespace {
std::string ReadJsonString(const rapidjson::Value& v, const char* n, const std::string& f) {
  return v.HasMember(n) && v[n].IsString() ? v[n].GetString() : f;
}
float ReadJsonFloat(const rapidjson::Value& v, const char* n, float f) {
  return v.HasMember(n) && v[n].IsNumber() ? static_cast<float>(v[n].GetDouble()) : f;
}
}
void MappingAlgorithms::parseLabelFusionConfig(const rapidjson::Value& root) {
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

}
