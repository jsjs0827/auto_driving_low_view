#ifndef RECHARGE_MARK_CONFIG_INTERNAL_H_
#define RECHARGE_MARK_CONFIG_INTERNAL_H_

#include "percep_demo/api_recharge_mark.h"
#include "percep_demo/mark_recharge_detect.h"

#include <string>

namespace recharge_cfg {

struct Parsed {
    Aiper_Recharge::RechargeHostConfig host{};
    bool roi_enabled = true;
    double roi_x_min = 0.12;
    double roi_x_max = 0.88;
    double roi_y_min = 0.18;
    double roi_y_max = 0.98;
    bool check_aspect = false;
    bool check_sym = false;
    bool check_area = false;
    bool pose_refine = true;
    double camera_height_mm = 200.0;
    double mark_height_mm = 100.0;
    double camera_pitch_deg = 12.0;
    bool save_raw = false;
    std::string save_raw_dir = "/userdata/datasets/recharge_mark/left";
    uint32_t save_every_n = 1;
    uint32_t save_max = 200;
    int aruco_dictionary_id = 0;
    int aruco_target_id = 0;
    bool aruco_dual_marker = true;
    bool aruco_layout_horizontal = true;
    int aruco_id_left = 0;
    int aruco_id_right = 1;
    int aruco_id_top = 0;
    int aruco_id_bottom = 1;
    double aruco_marker_size_m = 0.07;
    double aruco_gap_m = 0.05;
    double aruco_margin_m = 0.005;
    double aruco_card_w_m = 0.20;
    double aruco_card_h_m = 0.08;
    double aruco_max_reproj_px = 5.0;
    bool aruco_enable_lm_refine = true;
    int aruco_thresh_win_min = 3;
    int aruco_thresh_win_max = 23;
    int aruco_thresh_win_step = 10;
    double aruco_min_perimeter_rate = 0.03;
    double aruco_max_perimeter_rate = 4.0;
    double aruco_poly_approx_rate = 0.05;
    double aruco_min_corner_dist_rate = 0.05;
    int aruco_min_border = 3;
    int aruco_pixel_per_cell = 4;
    int aruco_refine_method = 2;  // CORNER_REFINE_CONTOUR
    int aruco_refine_win = 5;
    int aruco_refine_iters = 20;
    double aruco_refine_min_acc = 0.01;
};

// 读 JSON；根对象或 {"recharge_mark":{...}} 均可。失败返回 false。
bool LoadFile(const char* path, Parsed* out, std::string* err);

// 解析 recharge_mark.json 路径。
// start_or_cfg 空：读板端 start_config.json 的 recharge_mark.config_file；
// 也可直接传 start_config.json 或 recharge_mark.json。
std::string ResolveMarkJsonPath(const char* start_or_cfg, std::string* err);

}  // namespace recharge_cfg

#endif  // RECHARGE_MARK_CONFIG_INTERNAL_H_
