#include "percep_demo/mark_aruco_detect.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace aiper {
namespace aruco {

using aiper::recharge::MarkDetectResult;
using aiper::recharge::MarkPose;

namespace {

struct CardLayout {
  double size = 70.0;
  double half = 35.0;
  double gap = 50.0;
  double margin = 5.0;
  double card_w = 200.0;
  double card_h = 80.0;
  bool horizontal = true;
  double cx_left = -60.0;
  double cx_right = 60.0;
  double cy_top = -60.0;
  double cy_bot = 60.0;
  int id_left = 0;
  int id_right = 1;
  int id_top = 0;
  int id_bot = 1;
};

CardLayout MakeCardLayout(const ArucoOptions& opt) {
  CardLayout L;
  L.size = opt.marker_size_m * 1000.0;
  L.half = 0.5 * L.size;
  L.gap = opt.gap_m * 1000.0;
  L.margin = opt.margin_m * 1000.0;
  L.horizontal = opt.layout_horizontal;
  L.card_w = opt.card_w_m * 1000.0;
  L.card_h = opt.card_h_m * 1000.0;
  if (L.card_w < 1.0) L.card_w = L.horizontal ? 200.0 : 80.0;
  if (L.card_h < 1.0) L.card_h = L.horizontal ? 80.0 : 200.0;
  const double along =
      0.5 * (L.horizontal ? L.card_w : L.card_h) - L.margin - L.half;
  L.cx_left = -along;
  L.cx_right = +along;
  L.cy_top = -along;
  L.cy_bot = +along;
  L.id_left = opt.marker_id_left;
  L.id_right = opt.marker_id_right;
  L.id_top = opt.marker_id_top;
  L.id_bot = opt.marker_id_bottom;
  return L;
}

std::vector<cv::Point3d> LocalMarkerCornersMm(double half) {
  return {{-half, -half, 0.0},
          {half, -half, 0.0},
          {half, half, 0.0},
          {-half, half, 0.0}};
}

std::vector<cv::Point3d> MarkerCornersInCardMm(const ArucoOptions& opt,
                                               const CardLayout& L, int mid) {
  auto out = LocalMarkerCornersMm(L.half);
  if (!opt.dual_marker) return out;
  if (L.horizontal) {
    double dx = 0.0;
    if (mid == L.id_left) dx = L.cx_left;
    else if (mid == L.id_right) dx = L.cx_right;
    for (auto& p : out) p.x += dx;
    return out;
  }
  double dy = 0.0;
  if (mid == L.id_top) dy = L.cy_top;
  else if (mid == L.id_bot) dy = L.cy_bot;
  for (auto& p : out) p.y += dy;
  return out;
}

double ReprojectionRmse(const std::vector<cv::Point3d>& obj,
                        const std::vector<cv::Point2d>& img, const cv::Mat& rvec,
                        const cv::Mat& tvec, const cv::Mat& K,
                        const cv::Mat& dist) {
  std::vector<cv::Point2d> proj;
  cv::projectPoints(obj, rvec, tvec, K, dist, proj);
  double s = 0.0;
  for (size_t i = 0; i < img.size(); ++i) {
    const double dx = proj[i].x - img[i].x;
    const double dy = proj[i].y - img[i].y;
    s += dx * dx + dy * dy;
  }
  return std::sqrt(s / static_cast<double>(img.size()));
}

struct PoseAngles {
  double roll_x_deg = 0;
  double pitch_y_deg = 0;
  double yaw_z_deg = 0;
  double heading_y_deg = 0;
  double tilt_x_deg = 0;
  double image_roll_z_deg = 0;
};

PoseAngles DecomposePose(const cv::Matx33d& R) {
  PoseAngles a;
  const double r00 = R(0, 0), r10 = R(1, 0), r20 = R(2, 0);
  const double r21 = R(2, 1), r22 = R(2, 2);
  const double r11 = R(1, 1), r12 = R(1, 2);
  const double sy = std::hypot(r00, r10);
  const bool singular = sy < 1e-8;
  double roll_x, pitch_y, yaw_z;
  if (!singular) {
    roll_x = std::atan2(r21, r22);
    pitch_y = std::atan2(-r20, sy);
    yaw_z = std::atan2(r10, r00);
  } else {
    roll_x = std::atan2(-r12, r11);
    pitch_y = std::atan2(-r20, sy);
    yaw_z = 0.0;
  }
  a.roll_x_deg = roll_x * 180.0 / CV_PI;
  a.pitch_y_deg = pitch_y * 180.0 / CV_PI;
  a.yaw_z_deg = yaw_z * 180.0 / CV_PI;
  const double nx = R(0, 2), ny = R(1, 2), nz = R(2, 2);
  a.heading_y_deg = std::atan2(nx, nz) * 180.0 / CV_PI;
  a.tilt_x_deg = std::atan2(-ny, std::hypot(nx, nz)) * 180.0 / CV_PI;
  a.image_roll_z_deg = std::atan2(r10, r00) * 180.0 / CV_PI;
  return a;
}

// 右竖更长 → +1（车在桩轴右）；左竖更长 → -1。不用画面左右。
int TrapSignFromQuad(const std::vector<cv::Point2f>& pts, double z_mm) {
  if (pts.size() < 4) return 0;
  const double len_l = std::hypot(pts[3].x - pts[0].x, pts[3].y - pts[0].y);
  const double len_r = std::hypot(pts[2].x - pts[1].x, pts[2].y - pts[1].y);
  const double mean = 0.5 * (len_l + len_r);
  const double trap = len_r - len_l;
  const double thr = std::max(6.0, 0.08 * mean);
  (void)z_mm;
  if (trap > thr) return 1;
  if (trap < -thr) return -1;
  return 0;
}

cv::Ptr<cv::aruco::Dictionary> MakeDictionary(int dict_id) {
  static std::map<int, cv::Ptr<cv::aruco::Dictionary>> cache;
  auto it = cache.find(dict_id);
  if (it != cache.end()) return it->second;
  auto dict = cv::aruco::getPredefinedDictionary(
      static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(dict_id));
  cache[dict_id] = dict;
  return dict;
}

cv::Vec3d CameraInMarkerMm(const cv::Matx33d& R, const cv::Mat& tvec) {
  const cv::Vec3d t(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
  return -(R.t() * t);
}

bool MatToVec3d(const cv::Mat& src, cv::Mat* dst64) {
  if (!dst64 || src.empty()) return false;
  cv::Mat tmp;
  src.convertTo(tmp, CV_64F);
  if (tmp.total() < 3) return false;
  *dst64 = tmp.reshape(1, 3).clone();
  return std::isfinite(dst64->at<double>(0)) &&
         std::isfinite(dst64->at<double>(1)) &&
         std::isfinite(dst64->at<double>(2));
}

struct PnpPick {
  bool ok = false;
  double score = 1e9;
  double rmse = 1e9;
  cv::Mat rvec;
  cv::Mat tvec;
  cv::Matx33d R;
  cv::Vec3d C;
  const char* name = "";
};

bool KeepLmRefine(const cv::Vec3d& C, const cv::Vec3d& C_r) {
  // 角点走 CONTOUR 后 C_y 与 Python 同号；4.5 的 8 点 LM 仍可能把解拽进
  // 左码盆地（C_x 跳十几厘米），必须拦住。
  const bool x_flip = (C[0] < 0.0) != (C_r[0] < 0.0) &&
                      std::fabs(C[0]) >= 40.0 && std::fabs(C_r[0]) >= 40.0;
  const bool x_jump = std::fabs(C_r[0] - C[0]) > 80.0;
  const bool y_flip = (C[1] < 0.0) != (C_r[1] < 0.0) && std::fabs(C[1]) >= 30.0;
  return !x_flip && !x_jump && !y_flip;
}

void ConsiderSolution(const std::vector<cv::Point3d>& obj_pts,
                      const std::vector<cv::Point2d>& img_pts, const cv::Mat& K,
                      const cv::Mat& dist, const cv::Mat& rvec_in,
                      const cv::Mat& tvec_in, bool refine, const char* name,
                      PnpPick* best) {
  cv::Mat rvec, tvec;
  if (!MatToVec3d(rvec_in, &rvec) || !MatToVec3d(tvec_in, &tvec)) return;
  if (tvec.at<double>(2) <= 1.0) return;

  cv::Matx33d R;
  cv::Rodrigues(rvec, R);
  cv::Vec3d C = CameraInMarkerMm(R, tvec);
  if (refine) {
    cv::Mat rvec_r = rvec.clone();
    cv::Mat tvec_r = tvec.clone();
    try {
      cv::solvePnPRefineLM(obj_pts, img_pts, K, dist, rvec_r, tvec_r);
      cv::Matx33d R_r;
      cv::Rodrigues(rvec_r, R_r);
      const cv::Vec3d C_r = CameraInMarkerMm(R_r, tvec_r);
      if (KeepLmRefine(C, C_r)) {
        rvec = rvec_r;
        tvec = tvec_r;
        R = R_r;
        C = C_r;
      }
    } catch (const cv::Exception&) {
    }
  }
  const double rmse = ReprojectionRmse(obj_pts, img_pts, rvec, tvec, K, dist);
  // 与 Python 一致：相机应在码上方（码面 +Y 向下 → C_y<0）。错误平面解 C_y 反号。
  const double score = rmse + (C[1] < 20.0 ? 0.0 : 25.0);
  if (!best->ok || score < best->score) {
    best->ok = true;
    best->score = score;
    best->rmse = rmse;
    best->rvec = rvec;
    best->tvec = tvec;
    best->R = R;
    best->C = C;
    best->name = name;
  }
}

PnpPick SolveBoardPnp(const std::vector<cv::Point3d>& obj_pts,
                      const std::vector<cv::Point2d>& img_pts, const cv::Mat& K,
                      const cv::Mat& dist, bool refine) {
  PnpPick best;
  struct Flag {
    cv::SolvePnPMethod id;
    const char* name;
  };
  std::vector<Flag> flags;
  flags.push_back({cv::SOLVEPNP_IPPE, "IPPE"});
  flags.push_back({cv::SOLVEPNP_ITERATIVE, "ITERATIVE"});

  for (const Flag& fl : flags) {
    std::vector<cv::Mat> rvecs, tvecs;
    bool got = false;
    try {
      const int n = cv::solvePnPGeneric(obj_pts, img_pts, K, dist, rvecs, tvecs,
                                        false, fl.id);
      got = n > 0 && !rvecs.empty() && !tvecs.empty();
    } catch (const cv::Exception&) {
      got = false;
    }
    if (!got) {
      cv::Mat rvec, tvec;
      try {
        if (cv::solvePnP(obj_pts, img_pts, K, dist, rvec, tvec, false, fl.id)) {
          rvecs.push_back(rvec);
          tvecs.push_back(tvec);
          got = true;
        }
      } catch (const cv::Exception&) {
        got = false;
      }
    }
    if (!got) continue;
    const int nsol = static_cast<int>(
        std::min(rvecs.size(), tvecs.size()));
    for (int i = 0; i < nsol; ++i) {
      ConsiderSolution(obj_pts, img_pts, K, dist, rvecs[i], tvecs[i], refine,
                       fl.name, &best);
    }
    if (best.ok && std::strcmp(fl.name, "IPPE") == 0) break;
  }
  return best;
}

}  // namespace

aiper::recharge::MarkDetectResult DetectArucoMarkCenter(
    const cv::Mat& bgr, const ArucoOptions& opt, const cv::Mat& camera_matrix) {
  MarkDetectResult det;
  det.ok = false;
  if (bgr.empty()) {
    det.error = "empty image";
    return det;
  }

  cv::Mat gray;
  if (bgr.channels() == 1) {
    gray = bgr;
  } else {
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  }

  cv::Ptr<cv::aruco::Dictionary> dict = MakeDictionary(opt.dictionary_id);
  cv::Ptr<cv::aruco::DetectorParameters> params =
      cv::aruco::DetectorParameters::create();
  params->adaptiveThreshWinSizeMin = opt.adaptive_thresh_win_size_min;
  params->adaptiveThreshWinSizeMax = opt.adaptive_thresh_win_size_max;
  params->adaptiveThreshWinSizeStep = opt.adaptive_thresh_win_size_step;
  params->minMarkerPerimeterRate = opt.min_marker_perimeter_rate;
  params->maxMarkerPerimeterRate = opt.max_marker_perimeter_rate;
  params->polygonalApproxAccuracyRate = opt.polygonal_approx_accuracy_rate;
  params->minCornerDistanceRate = opt.min_corner_distance_rate;
  params->minDistanceToBorder = opt.min_distance_to_border;
  params->perspectiveRemovePixelPerCell = opt.perspective_remove_pixel_per_cell;
  {
    int method = opt.corner_refinement_method;
    if (method < 0 || method > 3) {
      method = 2;  // CORNER_REFINE_CONTOUR
    }
    params->cornerRefinementMethod = method;
  }
  params->cornerRefinementWinSize = opt.corner_refinement_win_size;
  params->cornerRefinementMaxIterations = opt.corner_refinement_max_iterations;
  params->cornerRefinementMinAccuracy = opt.corner_refinement_min_accuracy;

  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  cv::aruco::detectMarkers(gray, dict, corners, ids, params);

  det.debug_initial_candidates = static_cast<int>(corners.size());
  det.debug_scaffold_count = static_cast<int>(corners.size());
  if (ids.empty()) {
    det.error = "no aruco detected";
    return det;
  }

  const CardLayout lay = MakeCardLayout(opt);
  const int id_a = opt.dual_marker
                       ? (lay.horizontal ? lay.id_left : lay.id_top)
                       : opt.target_marker_id;
  const int id_b = opt.dual_marker
                       ? (lay.horizontal ? lay.id_right : lay.id_bot)
                       : opt.target_marker_id;

  std::map<int, std::vector<cv::Point2f>> found;
  for (size_t i = 0; i < ids.size(); ++i) {
    const int mid = ids[i];
    const bool want =
        opt.dual_marker ? (mid == id_a || mid == id_b) : (mid == id_a);
    if (!want || found.count(mid)) continue;
    found[mid] = corners[i];
  }
  if (found.empty()) {
    det.error = opt.dual_marker ? "target id pair not found"
                                : "target id not found";
    return det;
  }

  std::vector<int> ordered;
  if (found.count(id_a)) ordered.push_back(id_a);
  if (opt.dual_marker && id_b != id_a && found.count(id_b)) {
    ordered.push_back(id_b);
  }

  std::vector<cv::Point3d> obj_pts;
  std::vector<cv::Point2d> img_pts;
  for (int mid : ordered) {
    const auto obj = MarkerCornersInCardMm(opt, lay, mid);
    const auto& pts = found[mid];
    for (int k = 0; k < 4; ++k) {
      obj_pts.push_back(obj[static_cast<size_t>(k)]);
      img_pts.emplace_back(pts[k].x, pts[k].y);
    }
  }

  std::vector<cv::Point2f> outer(4);
  if (opt.dual_marker && found.count(id_a) && found.count(id_b) &&
      id_a != id_b) {
    const auto& p0 = found[id_a];
    const auto& p1 = found[id_b];
    if (lay.horizontal) {
      outer = {p0[0], p1[1], p1[2], p0[3]};
    } else {
      outer = {p0[0], p0[1], p1[2], p1[3]};
    }
  } else {
    outer = found[ordered[0]];
  }
  const char* names[] = {"TL", "TR", "BR", "BL"};
  for (int i = 0; i < 4; ++i) {
    det.feature_points[names[i]] = outer[i];
  }
  det.detected_point_count = static_cast<int>(img_pts.size());
  det.debug_scaffold_count = static_cast<int>(ordered.size());

  cv::Point2d center(0.0, 0.0);
  for (const auto& p : outer) {
    center.x += p.x;
    center.y += p.y;
  }
  center.x /= 4.0;
  center.y /= 4.0;
  det.center = cv::Point2i(static_cast<int>(center.x), static_cast<int>(center.y));

  int xmin = INT_MAX, ymin = INT_MAX, xmax = 0, ymax = 0;
  for (const auto& p : outer) {
    xmin = std::min(xmin, static_cast<int>(p.x));
    ymin = std::min(ymin, static_cast<int>(p.y));
    xmax = std::max(xmax, static_cast<int>(p.x));
    ymax = std::max(ymax, static_cast<int>(p.y));
  }
  det.mark_box = cv::Rect(xmin, ymin, xmax - xmin, ymax - ymin);
  {
    const cv::Point2f tl = outer[0], tr = outer[1];
    det.image_angle_deg =
        std::atan2(static_cast<double>(tr.y - tl.y),
                   static_cast<double>(tr.x - tl.x)) *
        180.0 / CV_PI;
  }

  cv::Mat dist = cv::Mat::zeros(4, 1, CV_64F);
  const PnpPick picked =
      SolveBoardPnp(obj_pts, img_pts, camera_matrix, dist, opt.enable_lm_refine);
  if (!picked.ok) {
    det.pnp_error = "solvePnP failed";
    return det;
  }
  if (picked.rmse > opt.max_reprojection_error_px) {
    det.pnp_error = "reproj too large";
    det.pose.reprojection_rmse_px = picked.rmse;
    return det;
  }

  std::vector<cv::Point3d> origin = {{0.0, 0.0, 0.0}};
  std::vector<cv::Point2d> origin_uv;
  cv::projectPoints(origin, picked.rvec, picked.tvec, camera_matrix, dist,
                    origin_uv);
  if (!origin_uv.empty()) {
    center = origin_uv[0];
    det.center = cv::Point2i(static_cast<int>(std::lround(center.x)),
                             static_cast<int>(std::lround(center.y)));
  }

  PoseAngles ang = DecomposePose(picked.R);
  const cv::Vec3d& C = picked.C;
  const double dist_plane_mm = std::fabs(C[2]);

  det.ok = true;
  det.score = 100.0 - picked.rmse;
  det.detect_branch = picked.name ? picked.name : "aruco";
  if (opt.dual_marker) {
    det.detect_branch = std::string("aruco_dual_") + det.detect_branch;
  } else {
    det.detect_branch = std::string("aruco_") + det.detect_branch;
  }

  MarkPose& pose = det.pose;
  pose.ok = true;
  pose.rvec = cv::Vec3d(picked.rvec.at<double>(0), picked.rvec.at<double>(1),
                        picked.rvec.at<double>(2));
  pose.tvec_mm = cv::Vec3d(picked.tvec.at<double>(0), picked.tvec.at<double>(1),
                           picked.tvec.at<double>(2));
  pose.R = picked.R;
  pose.center_image_px = center;
  pose.distance_mm = dist_plane_mm > 1.0 ? dist_plane_mm
                                         : std::fabs(pose.tvec_mm[2]);
  pose.roll_x_deg = ang.roll_x_deg;
  pose.pitch_y_deg = ang.pitch_y_deg;
  pose.yaw_z_deg = ang.yaw_z_deg;
  // heading_y 是码法向在相机里的方位；车体相对桩轴航向取反。
  pose.heading_y_deg = -ang.heading_y_deg;
  pose.tilt_x_deg = ang.tilt_x_deg;
  pose.image_roll_z_deg = ang.image_roll_z_deg;
  pose.reprojection_rmse_px = picked.rmse;
  pose.cam_x_mm = C[0];
  pose.trap_sign = TrapSignFromQuad(outer, pose.distance_mm);
  pose.have_side_cues = pose.trap_sign != 0;
  pose.point_names = {"TL", "TR", "BR", "BL"};
  pose.center_camera_mm = C;
  return det;
}

}  // namespace aruco
}  // namespace aiper
