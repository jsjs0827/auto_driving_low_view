#ifndef PERCEP_DEMO_MARK_ARUCO_DETECT_H_
#define PERCEP_DEMO_MARK_ARUCO_DETECT_H_

#include <opencv2/core.hpp>

#include "percep_demo/mark_recharge_detect.h"

namespace aiper {
namespace aruco {

// ArUco 检测+PnP。默认横贴双码：DICT_4X4_50，ID0 左 / ID1 右，黑格 7cm，卡 20x8cm。
struct ArucoOptions {
  int dictionary_id = 0;  // cv::aruco::DICT_4X4_50 = 0
  int target_marker_id = 0;
  bool dual_marker = true;
  bool layout_horizontal = true;  // false=竖贴（上/下）
  int marker_id_left = 0;
  int marker_id_right = 1;
  int marker_id_top = 0;
  int marker_id_bottom = 1;
  double marker_size_m = 0.07;
  double gap_m = 0.05;
  double margin_m = 0.005;
  double card_w_m = 0.20;
  double card_h_m = 0.08;
  int adaptive_thresh_win_size_min = 3;
  int adaptive_thresh_win_size_max = 23;
  int adaptive_thresh_win_size_step = 10;
  double min_marker_perimeter_rate = 0.03;
  double max_marker_perimeter_rate = 4.0;
  double polygonal_approx_accuracy_rate = 0.05;
  double min_corner_distance_rate = 0.05;
  int min_distance_to_border = 3;
  int perspective_remove_pixel_per_cell = 4;
  // OpenCV 4.5 SUBPIX 会把角点拉到错误边缘（远距 x 偏 20–30cm）。
  // 4.5 用 CONTOUR；与 Python 4.13 SUBPIX 位姿对齐。
  int corner_refinement_method = 2;  // cv::aruco::CORNER_REFINE_CONTOUR
  int corner_refinement_win_size = 5;
  int corner_refinement_max_iterations = 20;
  double corner_refinement_min_accuracy = 0.01;
  double max_reprojection_error_px = 5.0;
  bool enable_lm_refine = true;
};

// 产出与当前 MarkDetectResult 兼容的四角 + PnP；DockPose 换算在 api 层。
aiper::recharge::MarkDetectResult DetectArucoMarkCenter(
    const cv::Mat& bgr, const ArucoOptions& opt, const cv::Mat& camera_matrix);

}  // namespace aruco
}  // namespace aiper

#endif  // PERCEP_DEMO_MARK_ARUCO_DETECT_H_
