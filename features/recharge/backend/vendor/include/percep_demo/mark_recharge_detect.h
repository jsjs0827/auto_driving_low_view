#ifndef PERCEP_DEMO_MARK_RECHARGE_DETECT_H_
#define PERCEP_DEMO_MARK_RECHARGE_DETECT_H_

#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace aiper::recharge {

// G 流水线提块方式：严格HSV → 放宽HSV → Otsu
enum class CandidateExtractMode {
  HsvRelaxed = 0,
  HsvStrict = 1,
  Otsu = 2,
};

struct DetectOptions {
  bool enable_roi = true;
  bool enable_aspect_ratio_check = false;
  bool enable_strict_symmetry_check = false;
  bool enable_area_perimeter_structure_check = false;
  int min_detected_feature_points = 4;
  bool enable_search_crop = true;
  double search_x_min_ratio = 0.12;
  double search_x_max_ratio = 0.88;
  double search_y_min_ratio = 0.18;
  double search_y_max_ratio = 0.98;
  int reference_image_width = 0;
  int reference_image_height = 0;
  // 仅姿态后处理：不改图像识别；y 用 PnP，x/yaw 用轴分解
  bool enable_ground_pose_refine = true;
  double camera_height_mm = 200.0;
  double mark_height_mm = 100.0;
  double camera_pitch_deg = 12.0;
  // Python 推荐方案 G：级联检测 + 多 scaffold PnP RMSE 门
  bool enable_detect_cascade = true;
  CandidateExtractMode extract_mode = CandidateExtractMode::HsvRelaxed;
  double max_accept_reproj_rmse_px = 3.0;
  int max_scaffolds_pnp_try = 300;  // 对齐 Python scored[:300]
  // 上一帧 refine 后 tvec_x(mm)= -cam_x，左右符号保持 / x 尖峰门
  double axis_x_prior_mm = NAN;
  bool have_axis_x_prior = false;
  // 上一帧成功距离(mm)。≤450 时四角优先：当前 Python 在该近距带几乎组不出 10 点。
  double last_distance_mm = NAN;
  // AI charger 框内识别时跳过部分全幅占幅门；主路径默认 false
  bool trust_detection_crop = false;
  // 静态 ROI 已有 last_good / YOLO 回退时：A 失败不再跑 B_q/C_q（远距 40 点空扫 ~650ms）
  bool skip_bc_on_a_fail = false;
};

struct MarkPose {
  bool ok = false;
  cv::Vec3d rvec;
  cv::Vec3d tvec_mm;
  cv::Matx33d R;
  cv::Vec3d center_camera_mm;
  cv::Point2d center_image_px;
  double distance_mm = 0;
  double roll_x_deg = 0;
  double pitch_y_deg = 0;
  double yaw_z_deg = 0;
  double heading_y_deg = 0;
  double tilt_x_deg = 0;
  double image_roll_z_deg = 0;
  double reprojection_rmse_px = 0;
  double cam_x_mm = NAN;
  int trap_sign = 0;
  bool have_side_cues = false;
  std::vector<std::string> point_names;
};

struct MarkDetectResult {
  bool ok = false;
  std::string error;
  cv::Point2i center;
  cv::Rect mark_box;
  std::map<std::string, cv::Point2f> feature_points;
  int detected_point_count = 0;
  double image_angle_deg = NAN;
  double score = 0;
  MarkPose pose;
  std::string pnp_error;
  std::string detect_branch;
  cv::Rect search_crop;
  int debug_initial_candidates = 0;
  int debug_scaffold_count = 0;
  std::vector<cv::Rect> debug_candidate_boxes;
  std::vector<cv::Rect> debug_scaffold_boxes;
};

cv::Mat DefaultCameraMatrix();

}  // namespace aiper::recharge

#endif  // PERCEP_DEMO_MARK_RECHARGE_DETECT_H_
