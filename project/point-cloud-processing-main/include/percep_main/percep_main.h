/**
 * @file self_check.h
 * @brief Minimal collect node function example.
 *
 * Copyright 2026 Daye Technology Co.Ltd. All rights reserved.
 */
#ifndef AI_NODE_PERCEP_DEMO_INCLUDE_PERCEP_DEMO_H_
#define AI_NODE_PERCEP_DEMO_INCLUDE_PERCEP_DEMO_H_

#include <std_msgs/msg/u_int8.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "common_interface/msg/grid_map_info.hpp"
#include "common_interface/msg/dock_pose.hpp"
#include "common_interface/msg/camera_param.hpp"
#include "common_interface/srv/to_percep_cmd.hpp"
#include "common_interface/srv/to_camera_cmd.hpp"
#include "common_interface/srv/to_app_cmd.hpp"
#include "common_interface/srv/get_percep_status.hpp"

#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"


#include <pcl_conversions/pcl_conversions.h>

#include "percep_main/pcl_common.hpp"
#include "percep_main/ai_fusion.hpp"
#include "topic_macros.h"
#include "shm_struct/camera_struct.h"
#include "percep_main/patchworkpp.hpp"
#include "percep_main/horizon_dnn_runner.hpp"
#include "percep_main/visualize.hpp"
#include "percep_main/rampRecovery.h"
// api_recharge_mark.h 统一取 recharge_mark 独立包/内嵌子仓的对外头
// （find_package(recharge_mark) 的 include 或本仓 recharge_mark/include 均可解析）
#include "percep_demo/api_recharge_mark.h"

using namespace std;
using namespace rapidjson;
using namespace cv;
const int rgb_width = 640;
const int rgb_height = 544;
const int gdc_width = 1280;
const int gdc_height = 1088;
namespace perception {
// 检测框 → 点云匹配（像素回投影；label_fusion 关闭时的简化路径）
struct DetMatchConfig {
  bool enabled = true;           // 发布 /percep_main/det_cloud
  bool publish_markers = true;   // 发布 /percep_main/det_markers
  float score_min = 0.25f;
  float depth_span_m = 0.80f;    // 框内主体深度跨度
  float max_depth_m = 3.0f;      // 相机系深度上限（米）；<=0 表示不截断
};

// 图像-点云语义融合（fastdnn）：检测框+分割掩码回投影，滤拖尾/收轮廓
struct LabelFusionConfig {
  bool enabled = false;
  bool fuse_into_grid = false;  // 预留：AI 语义写栅格交给下游，本进程不写
  bool publish_debug_cloud = true;  // /percep_main/labeled_cloud
  std::string passable_classes = "grass";  // 可通行分割类（草），点云 label=100+seg_id
  bool nongrass_seg_as_obstacle = false;  // false：非草分割不作为障碍（只留 det keep + 草）
  int min_semantic_points_per_cell = 2;  // 预留：下游写栅格时每格最少命中点数
  float det_score_min = 0.30f;
  int min_obstacle_points_per_cell = 2;
  float max_range_m = 3.0f;
  float max_height_m = 1.0f;
  std::string match_downsample = "stride";
  float match_voxel_m = 0.05f;
  size_t max_match_points = 20000;
  float det_depth_span_m = 0.60f;
  float det_depth_gap_m = 0.25f;
  int det_depth_min_pts = 5;
  float keep_below_cam_m = 0.0f;
  bool cluster_filter = true;
  float cluster_voxel_m = 0.05f;
  int cluster_min_pts = 4;
  int cluster_min_component_pts = 20;
  bool seg_mask_postprocess = false;
  int seg_mask_majority_radius = 1;
  int seg_mask_close_radius = 1;
  int seg_mask_close_iterations = 1;
  bool seg_cluster_filter = true;
  int seg_mask_erode_px = 1;
  float seg_depth_gap_m = 0.20f;
  int seg_ray_bin_px = 2;
  int seg_min_component_pts = 15;
};

// 视差域拖影/飞点抑制：在反投影成点云之前，把遮挡边界上的过渡视差和孤立噪声
// 剔除掉。3D 域的 Radius/Statistical 滤波对成片的边缘飞点无效（飞点自己就能聚
// 成一片、密度过关），所以必须在视差域拦。
struct DispFilterConfig {
    bool  enabled = true;
    // 有效视差范围(px)：低于 min_disp 的点深度已超出可信量程，基本都是无匹配噪声
    float min_disp = 2.0f;
    float max_disp = 144.0f;
    // 视差梯度门控（抑制边缘胖化/拖尾）：邻域内视差跳变超阈值 → 该点位于遮挡
    // 边界，丢弃。阈值 = max(grad_max_px, disp * grad_max_ratio)；近处视差大、
    // 斜面本身梯度也大，用相对项避免误杀近处地面。
    bool  grad_check = true;
    int   grad_radius = 1;        // 1 → 3x3；邻域在全分辨率视差图上取，不受 sample_step 影响
    float grad_max_px = 2.0f;
    float grad_max_ratio = 0.02f;
    // 邻域有效点数门控（speckle）：邻域内有效视差邻居太少 → 孤立噪点
    bool  speckle_check = true;
    int   min_valid_neighbors = 4;
    // 帧级有效率下限：整帧有效视差占比过低（暗光/运动模糊/镜头遮挡）时直接判本
    // 帧不可用，避免糊掉的帧污染下游栅格图。<=0 表示不检查。
    float min_valid_ratio = 0.05f;
    // 每 N 帧打一次剔除统计，便于调参。<=0 关闭。
    int   log_every_n = 0;
};

// 曝光自适应守护：长曝光 + 运动 = 图像模糊 → 左右匹配重心偏移 → 点云沿深度方向
// 拉长/抖动。曝光时间和增益是每帧都能从 SHM 拿到的（PicData.exp_time / analog_gain），
// 用它给本帧点云打一个可信度，并在超过硬阈值时降级处理。
// 注意：曝光长只是模糊的必要条件，静止时长曝光并不糊。没有里程计/IMU 参与时这是
// 一个偏保守的代理指标，阈值要按实测调，别一上来就设太紧。
struct ExposureGuardConfig {
    bool  enabled = true;
    // 曝光(ms)：<= soft 视为清晰(置信度 1.0)；(soft, hard) 线性降权；>= hard 触发 hard_action
    float exp_soft_ms = 8.0f;
    float exp_hard_ms = 20.0f;
    // 模拟增益：高增益说明暗光，匹配噪声大，同样降权。hard<=soft 表示不看增益
    float gain_soft = 4.0f;
    float gain_hard = 16.0f;
    // 降权下限，写入点云 confidence 字段
    float min_confidence = 0.35f;
    // 达到硬阈值时的动作：
    //   "none"          仅降权，几何与障碍照常输出
    //   "skip_obstacle" 降权 + 本帧不产出障碍点云（地面/高度仍然更新），防误判
    //   "drop_frame"    整帧点云丢弃；注意会走 onTimer 的失败分支，
    //                   打印"点云生成失败"并把 percep_status 置 -1
    std::string hard_action = "skip_obstacle";
    // skip_obstacle 时是否连 AI 语义/检测障碍(last_semantic_cloud_)一起跳过。
    // 图像糊了检测框同样不准，默认一起跳。
    bool  skip_ai_obstacle = true;
    // 模糊帧收紧视差梯度阈值的系数(<1 更严格，按 blur 程度线性插值)。<=0 表示不收紧
    float blur_grad_scale = 0.6f;
    // 每 N 帧打一次曝光/降权日志。<=0 关闭
    int   log_every_n = 0;
};

struct SaveMapTask
{
    OccMapData map_data;
    std::vector<uint8_t> rgb;
    std::vector<float> disp;
    std::vector<uint32_t> u;
    std::vector<uint32_t> v;
    std::unique_ptr<CameraGdcData> gdc;
    PointCloudT::Ptr raw_cloud;
    // 与disp/u/v逐点对齐的稀疏类别：0=不绘制，1=检测障碍，2=非grass分割。
    std::vector<uint8_t> ai_cloud_overlay_classes;
    int ai_cloud_overlay_w = 0;
    int ai_cloud_overlay_h = 0;
    // AI overlay 已渲染快照（仅 publish_overlay 开启时顺带产出，可直接落盘）
    std::vector<uint8_t> overlay;
    int overlay_w = 0;
    int overlay_h = 0;
    // 未发布时的渲染原料：全分辨率 RGB 底图(=rgb) + 分割掩码 + 检测框，
    // 着色画框在 saveLogLoop 异步完成，不占主循环
    std::vector<uint8_t> seg_mask;
    // 模型原始检测结果（keep_classes 过滤前），用于离线存档与分析。
    std::vector<DnnDetection> raw_dets;
    // 过滤后结果，仅用于在线融合，存盘不依赖它。
    std::vector<DnnDetection> dets;
    int dnn_img_w = 0;
    int dnn_img_h = 0;
    std::string time_str;
    int rows;
    int cols;
};

enum class HealthCode : int32_t {
    RgbNotReceived = 400,
    DispNotReceived  = 401,
    RgbError  = 402,
    DispError = 403,
    GdcNotReceived = 404,
    GdcError = 405,
    StartConfigMiss = 406,
    StartConfigError = 407,
    CalibrateMiss = 408,
    CalibrateError = 409,
    CameraParamsMiss = 410,
    CameraParamsError = 411,
    ModelError = 412,
    BpuApplyError = 413,
    AiInferDataError = 414,
    QRcodeDetectError = 415,
    LogWarn = 416,
    CameraBlockedWarn = 417,
    CameraExposureWarn =418

};
//各错误码状态，true表示曾经发生过且故障持续存在，清障时需要判断
struct HealthCodeStatus {
    std::atomic<bool> RgbNotReceived{false};
    std::atomic<bool> DispNotReceived{false};
    std::atomic<bool> GdcNotReceived{false};    
    std::atomic<bool> RgbError{false};
    std::atomic<bool> DispError{false};
    std::atomic<bool> GdcError{false};
    std::atomic<bool> StartConfigMiss{false};
    std::atomic<bool> StartConfigError{false};
    std::atomic<bool> CalibrateMiss{false};
    std::atomic<bool> CalibrateError{false};
    std::atomic<bool> CameraParamsMiss{false};
    std::atomic<bool> CameraParamsError{false};
    std::atomic<bool> ModelError{false};
    std::atomic<bool> BpuApplyError{false};
    std::atomic<bool> AiInferDataError{false};
    std::atomic<bool> QRcodeDetectError{false};
    std::atomic<bool> LogWarn{false};
    std::atomic<bool> CameraBlockedWarn{false};
    std::atomic<bool> CameraExposureWarn{false};
};
//各故障码上报标志，true表示已上报过一次
struct HealthCodeReportedStatus {
    bool RgbNotReceived = false;
    bool DispNotReceived  = false;
    bool RgbError  = false;
    bool DispError = false;
    bool GdcNotReceived = false;
    bool GdcError = false;
    bool StartConfigMiss = false;
    bool StartConfigError = false;
    bool CalibrateMiss = false;
    bool CalibrateError = false;
    bool CameraParamsMiss = false;
    bool CameraParamsError = false;
    bool ModelError = false;
    bool BpuApplyError = false;
    bool AiInferDataError = false;
    bool QRcodeDetectError = false;
    bool LogWarn = false;
    bool CameraBlockedWarn = false;
    bool CameraExposureWarn =false;
};
class PercepMain : public daye::Function {
public:
  explicit PercepMain(const rapidjson::Value& ai_node_json);
  ~PercepMain() override;

private:
  bool initInterface();
  bool loadExtrinsicsConfig(rapidjson::Document &extrinsics_json);
  bool initExtrinsicsParams(rapidjson::Document &extrinsics_json);
  bool loadStartConfig(rapidjson::Document &start_json);
  bool initStartConfig(rapidjson::Document &ai_node_json);
  void offline_loop(); 
  bool initCameraParams();
  bool load_files();
  bool checkCamParams();

  void run_compute(CameraData &latest_rgb,CameraGdcData &latest_gdc);
  bool playFrame(size_t idx);
  void publish_ros_rgbcloud(PointCloudT::Ptr &cloud_in, daye::RosPublisher& pub,string frame_id);
  void publish_ros_cloud(PointCloudT::Ptr &cloud_in,daye::RosPublisher& pub,string frame_id);
  bool dense2sparse_pointcloud(CameraData& camera_data, int sample_step, PointCloudT::Ptr &out_sparse_cloud);
  void estimateGrassHeight();
  void runGroundSegmentation(const PointCloudT::Ptr& raw_pts); 
  // 依据本帧曝光/增益评估运动模糊程度，刷新 frame_* 状态
  void evaluateFrameBlur(const CameraData& camera_data);
  void postprocess(PointCloudT::Ptr& pcl_cloud,const std::vector<uint8_t>& rgb_buf, int img_w, int img_h,float fx,float fy,float cx,float cy);
  Trans adaptGroundTrans(const PointCloudT& cam_cloud);
  bool solveExtrinsicsFromGround(const PointCloudT& cam_cloud,double max_range,
                                  Trans& out,double& out_height,
                                  double& out_tilt_deg,double& out_resid_cm,
                                  size_t& out_inliers);
  bool percepStatus_callback(common_interface::srv::GetPercepStatus::Request& req,common_interface::srv::GetPercepStatus::Response& res);
  bool percepCmd_callback(common_interface::srv::ToPercepCmd::Request& req,common_interface::srv::ToPercepCmd::Response& res);
  void callAppService(int32_t cmd, int32_t status,double timeout_us);
  bool NV12ToRgbRos(const CameraData& camera_data, sensor_msgs::msg::Image& ros_img, std::vector<uint8_t>& rgb_buf);
  bool GdcToRgbRos(const CameraGdcData& camera_data,sensor_msgs::msg::Image& ros_img,std::vector<uint8_t>& rgb_buf);
  bool ResizeRosImage(sensor_msgs::msg::Image& ros_img, std::vector<uint8_t> &publish_rgb_buf_,uint32_t dst_w, uint32_t dst_h);
  void convertGridMap(int count);
  bool convertDispBin(const string& disp_path,PointCloudT::Ptr &sparse_cloud,std::vector<uint32_t> &u_buf,std::vector<uint32_t> &v_buf);
  bool convertDispRaw(const string& disp_path,PointCloudT::Ptr &sparse_cloud,std::vector<uint32_t> &u_buf,std::vector<uint32_t> &v_buf);
  bool convertGDC(const std::string& nv12_path,std::vector<uint8_t> &rgb_buf,std::unique_ptr<CameraData>* out_camera_data,std::unique_ptr<CameraGdcData>* out_gdc_data);
  bool waitStdinLine(std::string& out);
  void publishColoredMap(const std::vector<MapPoint>& map_points,string header,int index);
  void publish3dTF(Trans &cali,const PointCloudT::Ptr& pcl_cloud,string frame_id,bool pubTf);
  void onCameraFrame(const uint8_t* data, std::size_t size, const daye::FrameInfo& info);
  void onGdcFrame(const uint8_t* data, std::size_t size, const daye::FrameInfo& info);
  void autocali(PointCloudT::Ptr &pcl_cloud);
  void saveLogLoop();
  // saveLogLoop 线程专用：用任务里的 rgb 底图+seg_mask+dets 异步着色画框并落盘，
  // 仅 publish_overlay 关闭时走这条路径，主循环不做任何 overlay 渲染
  void saveAiOverlayAsync(const SaveMapTask& task);
  // 任务暂停(cmd=21)时的真清理：停回充 worker、清 SHM 队列/DNN 与点云缓存并 trim，
  // 避免回充高水位(Detector 常驻除外)带到下一轮后继续累高。
  void releaseTaskResources();
  void algThread();
  void healthCodeThread();
  void processSingleCode(std::atomic<bool> &codeHappen,bool &reportedStatus,bool clear_cond,perception::HealthCode healthCode,daye::HealthStatus errLevel);
  bool saveCamParams(const common_interface::msg::CameraParam &params);
  void onCamParams(const common_interface::msg::CameraParam &params);
  bool parseDnnConfig(const rapidjson::Value& root);
  void parseLabelFusionConfig(const rapidjson::Value& root);
  void ensureLeftYuvFormat();
  void initDnnRuntime();
  void initLabelFusionClassIds();
  void postprocessSegMask(std::vector<uint8_t>& seg_mask, int width, int height);
  void runDnnInference(const PicData& meta, const uint8_t* yuv,
                       bool run_seg = true);
  void matchAndPublishDetCloud(const PointCloudT::Ptr& cloud,
                               const std::vector<uint32_t>& u_vec,
                               const std::vector<uint32_t>& v_vec,
                               float fx, float fy, float cx, float cy,
                               int img_w, int img_h);
  void fuseSemanticLabels(const PointCloudT::Ptr& cloud,
                          const std::vector<uint32_t>& u_vec,
                          const std::vector<uint32_t>& v_vec,
                          const std::vector<DnnDetection>& dets,
                          const std::vector<uint8_t>& seg_mask, int seg_w,
                          int seg_h, int img_w, int img_h);

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster{nullptr};
  std::shared_ptr<Visualize> visualizePtr{nullptr};
  std::shared_ptr<AiFusion> aiFusionPtr{nullptr};   
  std::shared_ptr<patchwork::PatchWorkpp> patchwork_{nullptr};
  std::shared_ptr<PclCommon> pclCommonPtr{nullptr};
  std::shared_ptr<RampRecovery> recoveryPtr{nullptr};
  // recharge_mark_new 智能指针句柄：出作用域/析构自动 Stop→释放 api→关日志线程
  // （自定义删除器 RechargeMarkHostDeleter = RechargeMarkHost::Destroy）。
  Aiper_Recharge::RechargeMarkHostPtr recharge_;
    
  rapidjson::Document global_json;
  HealthCodeStatus healthCodeStatus;
  std::unique_ptr<CameraGdcData> gdc_forsave;
  std::thread saveLogThreadHandle,algThreadHandle,healthCodeHandle,offlineThreadHandle;
  std::mutex save_map_mtx_;
  std::condition_variable cv_save_map;
  std::optional<std::unique_ptr<SaveMapTask>> pending_task_; // 仅保存一份待处理任务，永远最新帧
  // 保存任务统计仅用于评估异步 bin 落盘是否跟得上感知线程。
  std::atomic<uint64_t> save_submit_count_{0};
  std::atomic<uint64_t> save_interval_drop_{0};
  std::atomic<uint64_t> save_busy_drop_{0};
  std::atomic<uint64_t> save_done_count_{0};
  std::atomic<uint64_t> save_last_task_ms_{0};
  std::atomic<uint64_t> save_max_task_ms_{0};
  std::atomic<uint64_t> save_total_task_ms_{0};
  std::atomic<bool> running{true};
  std::atomic<bool> stop_percep{false};
  //统一true时才恢复
  std::atomic<bool> receive_params{false};
  std::atomic<bool> params_ok{false};
  std::atomic<bool> rgb_ok{false};
  std::atomic<bool> gdc_ok{false};
  std::atomic<bool> disp_ok{false};
  std::atomic<bool> rgb_rec{false};
  std::atomic<bool> gdc_rec{false};
  std::atomic<bool> disp_rec{false};
  std::atomic<bool> log_ok{false};
  std::atomic<bool> cam_clear{false};
  std::atomic<bool> exposure_ok{false};
  string sdk_so_dir,cfg_path,output_dir,rgb_dir,disp_dir,camparams_dir,type;
  int percep_status = 0;
  int log_count = 0;
  int sample_step = 4;
  double obs_height_thr = 0.05;
  bool log_on = false;
  std::atomic<bool> percep_map{false};
  std::atomic<bool> percep_station{false};
  std::atomic<bool> offline_running{true};
  float box_min_x = -3.0f;
  float box_max_x =  3.0f;
  float box_min_y = -3.0f;
  float box_max_y =  3.0f;
  float box_min_z =  0.1f;
  float box_max_z =  3.0f;
  DispFilterConfig disp_filter_cfg_;
  ExposureGuardConfig exp_guard_cfg_;
  // 本帧曝光评估结果，由 evaluateFrameBlur() 在 dense2sparse_pointcloud 开头刷新
  float frame_exp_ms_ = 0.f;        // 本帧曝光(ms)，<=0 表示 SHM 未提供
  float frame_gain_ = 0.f;          // 本帧模拟增益
  float frame_blur_t_ = 0.f;        // 0=清晰 … 1=达到硬阈值
  float frame_confidence_ = 1.f;    // 写入点云 confidence 字段
  bool  frame_skip_obstacle_ = false;
  bool  frame_drop_ = false;

  // 车体本身在俯仰，静态外参不足以让地面始终水平（实测残余倾斜 0.7~3.0deg 跨帧变化）。
  // 打开后每帧从地面反解外参，替代静态 cali 去做 base_link 变换，
  // 于是 patchwork 的 sensor_height、AiFusion 的 min_h/max_h、box_min_z
  // 这些"假设地面在 z≈0"的地方全部自动成立。
  //
  // 合理性门限必须有：单帧可能把墙面当地面，偏离静态外参太多就丢弃。
  //
  // ema_alpha 默认 1.0（即不平滑，完全跟随本帧）。实测 12 帧变换后地面倾斜：
  //   静态外参      平均1.184deg 最大2.737deg 标准差0.853deg
  //   alpha=0.25    平均1.189deg 最大2.583deg 标准差0.653deg
  //   alpha=0.5     平均0.989deg 最大1.871deg 标准差0.422deg
  //   alpha=1.0     平均0.758deg 最大1.113deg 标准差0.338deg   <-- 最好
  // 原本担心逐帧跟随会让栅格地图抖动，但 AiFusion::integrate() 结尾是
  // clean_variable()，地图每帧重建、没有跨帧累积，所以没有会被抖动的状态，
  // EMA 只会白白损失跟随能力。
  // 一旦以后引入跨帧累积(时序占据栅格之类)，再把 alpha 调到 0.3~0.5。
  bool   gadapt_enable      = true;
  double gadapt_ema_alpha   = 1.0;   // 1.0=完全跟随本帧，越小越平滑
  double gadapt_max_roll_dev  = 10.0; // 相对静态 cali 的最大允许偏差(度)
  double gadapt_max_pitch_dev = 10.0;
  double gadapt_max_yaw_dev   = 5.0;
  double gadapt_max_h_dev     = 0.1; // 相对静态 cali.z 的最大允许偏差(m)
  double gadapt_max_resid_cm  = 4.0;  // 地面拟合残差上限
  int    gadapt_min_inliers   = 3000;
  bool   gadapt_verbose       = true;
  double cali_max_range = 4.0;
  // EMA 状态
  Trans  gadapt_cur_;
  bool   gadapt_inited_ = false;
  size_t gadapt_ok_ = 0, gadapt_rej_ = 0;

  // 队列/锁必须比 SHM subscriber 寿命更长：subscriber 析构先停回调线程
  std::mutex rgb_mtx_;
  std::mutex gdc_mtx_;
  std::condition_variable rgb_data_cond_;
  std::condition_variable gdc_data_cond_;
  std::queue<CameraData> rgb_buffer_queue;
  std::queue<CameraGdcData> gdc_buffer_queue;
  daye::ShmSubscriber subCamera;
  daye::ShmSubscriber subCameraGdc;
  
  common_interface::msg::CameraParam camera_params;
  int patchwork_frame_count_{0};
  
  std::vector<uint8_t> pcl_proj_rgb_buf_;       // GDC 缩放到 RGBD 分辨率，供投影
  uint64_t frame_count_{0};                     // 低频心跳日志计数
  int cali_count = 0;
  float sum_roll,sum_pitch,sum_h;
  // 本帧存图时间戳（getSystemTimeMs），overlay / rgb / map 共用同一文件名
  std::string frame_save_time_str_;
  Trans cali,cali_default;
  PointCloudT::Ptr ground_cloud,no_ground_cloud,obs_cloud,ori_cloud,ramp_cloud;
  double mapWidth,mapHeight,mapResolution;
  std::vector<float> disp_vec;
  std::vector<uint32_t> u_vec,v_vec;
 
  bool visualize = false,save_log = false,auto_loop = false;
  // 保存节流：save_interval_ms<=0 时每帧保存；>0 时按时间间隔提交保存任务；
  // 保存的是“最新完成帧”的全量数据，不是跳过感知计算的跳帧。
  int64_t save_interval_ms = 0;
  uint64_t last_saved_frame_ms_ = 0;
  bool auto_cali = false,offline = false,interactive=false,self_test=false,ramp_detect = false;
  vector<string> nv12_files,nv12_names,disp_files,disp_names;

private:
  std::vector<uint8_t> frame_rgb_buf_;          // RGBD 左目 RGB
  daye::RosPublisher grid_map_pub_;
  daye::RosPublisher ori_cloud_pub_,ground_cloud_pub_,nonground_cloud_pub_,obs_cloud_pub_,ramp_cloud_pub_,step_cloud_pub_,grass_cloud_pub_;
  daye::RosPublisher rgb_img_pub_,disp_img_pub_;
  daye::RosPublisher pclmap_pub_,aimap_pub_,fusionmap_pub_;
  daye::RosSubscriber cam_params_sub_;
  daye::RosServer percepStatus_server_;
  daye::RosServer percepCmd_server_,cameraCmd_server_;
  daye::RosClient appCmd_client_;

  DnnConfig dnn_cfg_;
  DetMatchConfig det_match_cfg_;
  LabelFusionConfig label_fusion_cfg_;
  std::unique_ptr<HorizonDnnRunner> dnn_runner_;
  uint64_t dnn_run_tick_{0};
  std::vector<DnnDetection> last_dnn_dets_;
  // keep_classes 过滤前的原始检测结果，仅供存档使用。
  std::vector<DnnDetection> last_dnn_raw_dets_;
  std::vector<uint8_t> last_dnn_seg_mask_;
  int last_dnn_img_w_{0};
  int last_dnn_img_h_{0};
  std::vector<int> passable_seg_ids_;
  std::vector<int> passable_det_ids_;

  PointCloudT::Ptr offline_sparse_cloud;
  PointCloudT::Ptr last_semantic_cloud_;
  uint64_t label_fusion_log_tick_{0};
  std::vector<uint8_t> dnn_overlay_buf_;
  int dnn_overlay_w_{0};
  int dnn_overlay_h_{0};
  // 本帧 overlay 存档快照：runDnnInference 生成，convertGridMap 随 SaveMapTask 落盘
  std::vector<uint8_t> last_dnn_overlay_save_;
  int last_dnn_overlay_save_w_{0};
  int last_dnn_overlay_save_h_{0};
  daye::RosPublisher dnn_overlay_pub_;
  daye::RosPublisher dnn_seg_mask_pub_;
  daye::RosPublisher det_cloud_pub_;
  daye::RosPublisher det_markers_pub_;
  daye::RosPublisher labeled_cloud_pub_;
  bool left_yuv_is_nv12_{true};
  bool left_yuv_format_known_{false};

};

}  // namespace perception

#endif  // AI_NODE_PERCEP_DEMO_INCLUDE_PERCEP_DEMO_H_
