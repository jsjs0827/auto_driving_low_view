/*
 * @Description: 回充二维码（标志）识别接口（API + 宿主，单头文件）
 *
 * 在线路径：感知仓把 GDC SHM 帧交给 Tick()；解 NV12 在 so 内。
 * 算法、配置、DockPose 滤波、异步 worker、独立日志都在本 so 内；
 * 感知仓只取图。ROS 发布（DockPose / overlay）由 so 内宿主完成。
 * 对外头不暴露 ROS 类型；Init 的 rclcpp_node 传 rclcpp::Node*（可空）。
 *
 * 日志：so 内独立管理，异步写入 /userdata/log/ai_node/recharge_mark.log
 * （10MB×5 大小轮转，行格式仿 ai_node：[时间][级别] 内容），
 * 不依赖感知仓 LOG_INFO；setLogSink 仅作为可选镜像。
 *
 * ============================ 生命周期 ============================
 * 一个任务的完整流程就是下面 5 步，感知仓只需要这几行：
 *
 *   auto host = Aiper_Recharge::createRechargeMarkHost();    // 创建（unique_ptr 持有）
 *   host->Init(so_dir, nullptr, &daye::getHandle());         // ① 初始化
 *   host->Start();                                           // ② 开始任务
 *   //  每帧: host->Tick(&latest_gdc, sizeof(CameraGdcData)); // ③ 喂数据
 *   host->Stop();                                            // ④ 停止任务
 *   // ⑤ 析构：host 离开作用域自动完成，无需 destroyRechargeMarkHost
 *
 * ①②③④⑤ 之外的操作（配置解析、内参回退、ROS 发布器创建、
 * 日志文件打开、worker 线程管理）全部在 so 内自动完成，外部无需关心。
 * 所有权由 std::unique_ptr 管理（RechargeMarkHostPtr）：删除器 =
 * destroyRechargeMarkHost（Stop → 释放 api → 关日志线程），空删除器借
 * EBO 与裸指针同 sizeof、零开销；老代码可用 createRechargeMarkHostRaw()。
 */

#ifndef AIPER_RECHARGE_MARK_API_H
#define AIPER_RECHARGE_MARK_API_H

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>

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

// 感知仓调度用（取图节奏/overlay topic）；算法参数不经过此结构。
// 全部由 recharge_mark.json 读入（Init 内部完成），外部只读不写。
// 二维码识别是必需功能，没有总开关；publish_enabled 只控制 ROS 发布。
struct RechargeHostConfig {
    bool publish_enabled = true;     // json "publish_enabled"：false 时不建
                                     // DockPose/overlay 发布器（识别照跑、日志照写）
    uint32_t run_every_n = 1;        // 每 N 帧识别一次（1=每帧）
    uint32_t min_interval_ms = 200;  // 两次识别最小间隔 ms
    bool use_hires = true;           // true=GDC 1280×1088 高清识别
    bool publish_overlay = false;    // 是否发布 /percep_demo/recharge_mark_overlay
    int overlay_max_width = 640;     // overlay 图最大宽
    char intrinsics_source[32] = "dataset";  // "dataset"=用仓内标定内参（默认，
                                     // 不需要外部 K）；"sdk_scaled"=用感知仓
                                     // CameraParam 按分辨率缩放后作内参
};

struct MarkPoseResult {
    bool ok = false;                 // 本帧识别流程跑通（有检测）
    bool pose_ok = false;            // 解算出可用 DockPose（post 有效）
    uint64_t frame_id = 0;           // 相机帧号
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
    Post post;                       // 发布侧滤波后的 DockPose（含时间戳）
    double process_ms = 0;           // 本帧算法耗时
    int debug_candidates = 0;
    int debug_scaffolds = 0;
    uint8_t* overlay_bgr = nullptr;  // overlay 图（BGR），freeResult 释放
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

// 相机内参（标定分辨率下的值）。仅 intrinsics_source=sdk_scaled 时参与解算；
// dataset 模式下写入也不会影响识别。fx/fy<=1 视为无效，内部自动忽略。
struct RechargeCameraK {
    int cal_width = 0;               // 内参对应的标定图像宽
    int cal_height = 0;              // 内参对应的标定图像高
    double fx = 0;
    double fy = 0;
    double cx = 0;
    double cy = 0;
};

// ---------------------------------------------------------------------------
// 离线/底层 C 风格 API（宿主在 so 内使用；离线评测工具直接用这一套）。
// 在线感知仓不用关心本结构，直接用上面的 RechargeMarkHost。
// ---------------------------------------------------------------------------
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
    // 结果图本帧时间戳（仅命名用，与 percep gdc/disp/map 同名对齐）；空=旧命名。
    void (*setSaveResultTimeStr)(const char* time_str) = nullptr;
    // 结果图存盘开关，由主仓 debug.save_log 统一控制。
    void (*setSaveResultEnable)(bool enabled) = nullptr;
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
    // 从 SHM 提交 GDC 左目 NV12。rgbd 参数保留兼容，在线路径忽略。
    int (*submitFromShm)(const void* rgbd, std::size_t rgbd_bytes,
                         const void* gdc, std::size_t gdc_bytes,
                         const RechargeCameraK* k) = nullptr;
};

// RechargeMarkApi 的 unique_ptr 句柄：删除器先调 destroy(false)（停 worker、
// 释放内部资源）再 delete，对应旧的"手动 destroy + delete"两步。
struct RechargeMarkApiDeleter {
    void operator()(RechargeMarkApi* p) const {
        if (!p) return;
        if (p->destroy) p->destroy(false);
        delete p;
    }
};
using RechargeMarkApiPtr = std::unique_ptr<RechargeMarkApi, RechargeMarkApiDeleter>;

// raw 工厂（旧用法：配 freeResult / destroy 手动管理；离线评测工具用）。
RechargeMarkApi* createRechargeMarkApi();

// ---------------------------------------------------------------------------
// 宿主（Host）：任务级生命周期接口。
// 初始化 / 开始任务 / 停止任务 / 析构 = 一个回充识别任务的完整流程，
// 每步只调一个接口，其余（配置、话题、日志、内参回退）全部 so 内完成。
// 实现在本仓 recharge_mark_host.cpp。
// ---------------------------------------------------------------------------

inline constexpr const char* kDockPoseTopic = "/percep_demo/recharge_mark_pose";
inline constexpr const char* kOverlayTopic =
    "/percep_demo/recharge_mark_overlay";

class RechargeMarkHost {
public:
    // 供 unique_ptr 自定义删除器使用；裸指针持有者显式调 Destroy 释放。
    // 内部顺序：Stop → 释放 api → 关日志线程；对象内存由 delete 释放。
    static void Destroy(RechargeMarkHost* p);

    ~RechargeMarkHost();

    // ======================= ① 初始化（一步完成） =======================
    // 一句话完成：打开独立日志(/userdata/log/ai_node/recharge_mark.log) →
    // 解析配置(start_or_cfg，失败回退板端默认 recharge_mark.json) → 读入
    // ROI/检测/aruco/调度参数 → 注册结果回调 → 创建 DockPose/overlay 发布。
    // 无需再调 setLogSink/loadConfig/建话题等任何其它接口。
    //
    // 参数：
    //   sdk_so_dir   兼容保留，在线可传进程 so 目录或空串，不参与逻辑。
    //   start_or_cfg 可传 start_config.json / recharge_mark.json / 空串(板端
    //                默认 /app/daye/config/ai_node/recharge_mark.json)。
    //   rclcpp_node  rclcpp::Node*；非空且 publish_enabled 时建 ROS 发布
    //                (kDockPoseTopic / kOverlayTopic)；空=离线，只识别不发布。
    //
    // 返回 0=成功；非 0=配置/算法初始化失败（详情见日志）。
    // 线程：进程启动期调用一次；可重复调用重入。
    int Init(const char* sdk_so_dir, const char* start_or_cfg = nullptr,
             void* rclcpp_node = nullptr);

    // ======================= ② 开始任务 =======================
    // 开 worker 线程 + 清空上次会话滤波状态（resetSession），此后 Tick()
    // 开始提交识别、结果回调开始发布 DockPose。
    // 对应感知服务 ToPercepCmd cmd=2(只发二维码)/cmd=3(同时发地图)。
    // 返回 true=任务已开始（幂等：运行中重复调用直接 true）。
    bool Start();

    // ======================= ③ 每帧喂数据（唯一数据接口） =======================
    // 传入一帧 GDC SHM 数据，一步完成"校验→内参→深拷贝→入队"。
    //   gdc_frame        CameraGdcData 指针（GDC 左目 NV12 1280×1088）；
    //                    传 null/bytes 非法时安全返回 1，不会崩溃。
    //   gdc_bytes        上者的 sizeof。
    //   settle_timeout_ms>0：阻塞至本帧处理完（仅二维码模式 1:1 串行上报用）；
    //                    0（默认）：异步返回，结果经回调发布。
    //   save_time_str    本帧存图时间戳（percep frame_save_time_str_，与
    //                    gdc/disp/map 落盘同名）；空=结果图用旧命名。仅命名用。
    // 未 Start（worker 未运行）/ 忙（上一帧未处理完）时直接返回 1 丢帧，
    // 不阻塞感知定时器。返回 0=已入队，1=跳过（原因见日志）。
    // 线程：感知 timer 线程调用。
    int Tick(const void* gdc_frame, std::size_t gdc_bytes,
             int settle_timeout_ms = 0, const char* save_time_str = nullptr);

    // ======================= ④ 停止任务 =======================
    // 停 worker、丢弃未处理帧、结束发布；ROS 发布器保留（Init 不重来）。
    // 对应 ToPercepCmd cmd=21。幂等。
    void Stop();

    // ============ 辅助接口（可选，均有安全缺省，不调也可） ============
    // 外部内参写入：收到 CameraParam 话题时调用（sdk_scaled 模式才生效，
    // dataset 模式写入不参与解算）。fx/fy<=1 自动忽略；可随时反复调用。
    void setCameraK(int cal_width, int cal_height, double fx, double fy,
                    double cx, double cy);
    // 日志镜像（可选）：每行日志同时回调给外部（如镜像进感知仓日志）。
    // 缺省不镜像——日志已由 so 独立写入 recharge_mark.log，无需注册。
    using LogSink = void (*)(const char* line);
    void setLogSink(LogSink fn);
    // 会话日志落盘（供点云仓 percep 显式驱动，与感知存图时间戳对齐）：
    //   enable=true  开启：以 timestamp_str 命名新建
    //                /userdata/log/ai_node/recharge_<timestamp>.log，随后所有
    //                回充日志（host + api/检测 Worker 全量，logger 层镜像）
    //                追加写入；timestamp 为空/非法取当前毫秒，非法字符转下划线。
    //   enable=false 关闭：幂等，关闭后文件保留不删除。
    // 返回实际打开的文件全路径；打开失败返回空串（主日志不受影响）。
    // 线程安全；析构自动关闭。暂不接入点云仓主代码，先在子仓实现。
    void setLogCapture(bool enable, const char* timestamp_str = nullptr);

    // 结果图存盘开关，由主仓 start_config 的 debug.save_log 统一控制。
    void setSaveResultEnabled(bool enable);

    // ---- 状态查询 ----
    const char* configPath() const { return cfg_path_.c_str(); }
    const RechargeHostConfig& hostConfig() const { return host_; }
    uint32_t minIntervalMs() const { return host_.min_interval_ms; }
    const char* poseTopic() const { return kDockPoseTopic; }

private:
    friend RechargeMarkHost* createRechargeMarkHostRaw();
    RechargeMarkHost() = default;
    RechargeMarkHost(const RechargeMarkHost&) = delete;
    RechargeMarkHost& operator=(const RechargeMarkHost&) = delete;

    void onResult(MarkPoseResult* r);
    static void OnResultThunk(void* userdata, MarkPoseResult* result);
    void logLine(const char* line);
    void logf(const char* fmt, ...);
    bool waitSettled(uint64_t done_before, int timeout_ms);
    void publishPose(float x, float y, float yaw, int64_t stamp_ns);
    void publishInvalidPose(const char* why);
    void publishOverlay(const uint8_t* bgr, int w, int h, int step,
                        int64_t stamp_ns);
    bool needsSdkScaledK() const {
        return std::strcmp(host_.intrinsics_source, "sdk_scaled") == 0;
    }

    RechargeMarkApiPtr api_;
    std::string cfg_path_;
    RechargeHostConfig host_{};
    bool startup_save_cleanup_done_ = false;
    RechargeCameraK cam_k_{};
    bool have_cam_k_ = false;
    LogSink log_sink_ = nullptr;
    std::atomic<bool> abort_{false};
    uint64_t overlay_warn_tick_ = 0;
    std::atomic<uint64_t> done_seq_{0};
    std::mutex done_mtx_;
    std::condition_variable done_cv_;

    struct RosOut;
    std::unique_ptr<RosOut> ros_;
};

// 所有权句柄：默认删除器 = destroyRechargeMarkHost（Stop → 释放 api →
// 关日志线程）。空删除器借 EBO 与裸指针同 sizeof、零开销。
// 仅析构可隐式转裸指针（成员访问用 operator->）；如需 release 到旧接口用 .release()。
struct RechargeMarkHostDeleter {
    void operator()(RechargeMarkHost* p) const {
        if (p) RechargeMarkHost::Destroy(p);
    }
};
using RechargeMarkHostPtr =
    std::unique_ptr<RechargeMarkHost, RechargeMarkHostDeleter>;

// 推荐：返回 unique_ptr，出作用域自动 Stop/释放/关日志。
RechargeMarkHostPtr createRechargeMarkHost();
// 过渡：裸指针版本，须配 destroyRechargeMarkHost 释放。
RechargeMarkHost* createRechargeMarkHostRaw();
void destroyRechargeMarkHost(RechargeMarkHost* p);

}  // namespace Aiper_Recharge

#endif  // AIPER_RECHARGE_MARK_API_H
