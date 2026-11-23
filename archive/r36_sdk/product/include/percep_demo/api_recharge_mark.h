/*
 * @Description: 回充二维码（标志）识别接口
 *
 * 在线路径：感知仓把 SHM 帧指针交给 submitFromShm；解 NV12 / 选 1280|640 在 so 内。
 * 算法、配置、DockPose 滤波、异步 worker 都在本 so 内；感知仓只取图/发 ROS。
 */

#ifndef AIPER_RECHARGE_MARK_API_H
#define AIPER_RECHARGE_MARK_API_H

#include <cstddef>
#include <cstdint>

namespace Aiper_Recharge {

// 中间位姿（percep_demo 转 common_interface/DockPose 发布）
// y: 到对接面板距离 m（恒负；码面距减 6cm，贴板≈0）；x: 到充电轴垂距 m；yaw: 桩法向相对相机前向 rad
// time_stamp: 开机域时间 ns（优先相机帧 TimeStamp，否则 CLOCK_BOOTTIME）
// x<0：车在桩轴左侧（往右纠）；x>0：车在桩轴右侧（往左纠）
struct Post {
    float x = 0.f;
    float y = 0.f;
    float yaw = 0.f;
    int64_t time_stamp = 0;
};

// 感知仓调度用（取图节奏/overlay topic）；算法参数不经过此结构
struct RechargeHostConfig {
    bool enabled = false;
    bool publish_enabled = true;
    uint32_t run_every_n = 1;
    uint32_t min_interval_ms = 200;
    bool use_hires = true;
    bool publish_overlay = false;
    int overlay_max_width = 640;
    char intrinsics_source[32] = "dataset";
    // 回充任务：YOLO charger 扩框辅助裁图（感知仓 setExternalCropHint）。
    bool yolo_assist_crop = true;
    float yolo_crop_scale = 2.0f;
};

struct MarkPoseResult {
    bool ok = false;
    bool pose_ok = false;
    uint64_t frame_id = 0;
    int image_w = 0;
    int image_h = 0;
    int detected_point_count = 0;
    int center_u = 0;
    int center_v = 0;
    double image_angle_deg = 0;
    double score = 0;
    double tl_u = 0, tl_v = 0;
    double tr_u = 0, tr_v = 0;
    double br_u = 0, br_v = 0;
    double bl_u = 0, bl_v = 0;
    double top_u = 0, top_v = 0;
    double bot_u = 0, bot_v = 0;
    bool has_tl = false, has_tr = false, has_br = false, has_bl = false;
    bool has_top = false, has_bot = false;
    double tvec_mm[3] = {0, 0, 0};
    double rvec[3] = {0, 0, 0};
    double distance_mm = 0;
    double reprojection_rmse_px = 0;
    double roll_x_deg = 0;
    double pitch_y_deg = 0;
    double yaw_z_deg = 0;
    double heading_y_deg = 0;
    double tilt_x_deg = 0;
    double image_roll_z_deg = 0;
    Post post;
    double process_ms = 0;
    int debug_candidates = 0;
    int debug_scaffolds = 0;
    uint8_t* overlay_bgr = nullptr;
    int overlay_w = 0;
    int overlay_h = 0;
    int overlay_step = 0;
    char error[256] = {0};
    char pnp_error[256] = {0};
    // ABI 保留（L2 已移除，恒为 0 / 空）
    bool strip_l2 = false;
    float strip_dx_m = 0.f;
    float strip_du_px = 0.f;
    float strip_ms = 0.f;
    char strip_status[48] = {0};
    // 四角竖边梯面：+1 右竖更长（车在轴右），-1 左竖更长；0 不明。
    int trap_sign = 0;
    // ABI 保留（y/yaw hold 已不用，恒 false）
    bool dist_jump_hold = false;
    bool yaw_jump_hold = false;
    char x_filter_reason[48] = {0};
    double cam_x_mm = 0;
    float raw_x_m = 0.f;
    char crop_src[16] = {0};
};

struct RechargeCameraK {
    int cal_width = 0;
    int cal_height = 0;
    double fx = 0;
    double fy = 0;
    double cx = 0;
    double cy = 0;
};

struct RechargeMarkApi {
    void* handle = nullptr;
    // sdk_so_path 保留兼容（nocam 可传空）；cfg_path 为 recharge_mark.json
    int (*init)(const char* sdk_so_path, const char* cfg_path) = nullptr;
    void (*setCameraMatrix)(double fx, double fy, double cx, double cy) = nullptr;
    void (*useSdkCameraMatrix)() = nullptr;
    void (*setSearchRoi)(bool enabled, double x_min, double x_max, double y_min,
                         double y_max) = nullptr;
    void (*setDetectChecks)(bool aspect_ratio, bool strict_symmetry,
                            bool area_perimeter) = nullptr;
    void (*setPoseRefine)(bool enabled, double camera_height_mm,
                          double mark_height_mm,
                          double camera_pitch_deg) = nullptr;
    void (*setBuildOverlay)(bool enabled, int max_width) = nullptr;
    void (*setSaveRaw)(const char* dir, uint32_t every_n, uint32_t max_count) = nullptr;
    // ArUco：字典 / 目标 id / 码边长(m) / 重投影门限(px)
    void (*setArucoConfig)(int dictionary_id, int target_marker_id,
                            double marker_size_m, double max_reproj_px) = nullptr;
    void (*setMaxPnpTry)(int n) = nullptr;
    void (*setSkipBcOnAFail)(bool skip) = nullptr;
    MarkPoseResult* (*captureAndProcess)() = nullptr;
    MarkPoseResult* (*processLeftNv12)(const uint8_t* nv12, int width, int height,
                                       int stride_y, uint64_t frame_id) = nullptr;
    MarkPoseResult* (*processLeftNv12Crop)(const uint8_t* nv12, int width,
                                           int height, int stride_y, int crop_x,
                                           int crop_y, int crop_w, int crop_h,
                                           uint64_t frame_id) = nullptr;
    MarkPoseResult* (*processBgr)(const uint8_t* bgr, int width, int height,
                                  int step, uint64_t frame_id) = nullptr;
    MarkPoseResult* (*processBgrCrop)(const uint8_t* bgr, int width, int height,
                                      int step, int crop_x, int crop_y,
                                      int crop_w, int crop_h,
                                      uint64_t frame_id) = nullptr;
    void (*freeResult)(MarkPoseResult* result) = nullptr;
    void (*destroy)(bool destroy_camera) = nullptr;
    void (*resetSession)() = nullptr;
    // 读 recharge_mark.json：ROI/检测/安装几何；可在 init 后再调
    int (*loadConfig)(const char* json_path) = nullptr;
    void (*getHostConfig)(RechargeHostConfig* out) = nullptr;

    // 在线异步：so 内单槽队列 + worker。回调在 worker 线程，返回前勿 freeResult。
    using ResultCallback = void (*)(void* userdata, MarkPoseResult* result);
    void (*setResultCallback)(ResultCallback cb, void* userdata) = nullptr;
    int (*startWorker)() = nullptr;
    void (*stopWorker)() = nullptr;
    bool (*canSubmit)() = nullptr;
    // 0=入队成功，1=忙/间隔未到/未 start。nv12 由 so 深拷贝。
    int (*submitLeftNv12)(const uint8_t* nv12, int width, int height,
                          int stride_y, uint64_t frame_id,
                          int64_t timestamp_ns) = nullptr;
    // 从 SHM 帧提交左目 NV12。rgbd/gdc 可为 nullptr（按 host.use_hires 选）。
    // 不在头文件暴露 CameraData；k 仅 intrinsics_source=sdk_scaled 时使用。
    int (*submitFromShm)(const void* rgbd, std::size_t rgbd_bytes,
                         const void* gdc, std::size_t gdc_bytes,
                         const RechargeCameraK* k) = nullptr;
    // 外部裁框：感知仓 YOLO charger 扩边后的全图窗。w<=0 清除。src: 1=yolo。
    void (*setExternalCropHint)(int crop_x, int crop_y, int crop_w, int crop_h,
                                int src) = nullptr;
};

RechargeMarkApi* createRechargeMarkApi();

}  // namespace Aiper_Recharge

#endif  // AIPER_RECHARGE_MARK_API_H
