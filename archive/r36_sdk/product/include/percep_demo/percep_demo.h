/**
 * @file self_check.h
 * @brief Minimal collect node function example.
 *
 * Copyright 2026 Daye Technology Co.Ltd. All rights reserved.
 */
#ifndef AI_NODE_PERCEP_DEMO_INCLUDE_PERCEP_DEMO_H_
#define AI_NODE_PERCEP_DEMO_INCLUDE_PERCEP_DEMO_H_

#ifndef PERCEPTION_PRODUCT_WITH_RECHARGE
#define PERCEPTION_PRODUCT_WITH_RECHARGE 1
#endif

#include <std_msgs/msg/u_int8.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "common_interface/msg/grid_map_info.hpp"
#if PERCEPTION_PRODUCT_WITH_RECHARGE
#include "common_interface/msg/dock_pose.hpp"
#endif
#include "common_interface/msg/camera_param.hpp"
#include "common_interface/srv/to_percep_cmd.hpp"
#include "common_interface/srv/to_camera_cmd.hpp"
#include "common_interface/srv/to_app_cmd.hpp"
#include "common_interface/srv/get_percep_status.hpp"

#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "base_framework/function_factory.h"
#include "base_framework/node_factory.h"
#include "base_framework/daye_shm.h"
#include <pcl_conversions/pcl_conversions.h>

#if PERCEPTION_PRODUCT_WITH_RECHARGE
#include "percep_demo/api_recharge_mark.h"
#endif
#include "percep_demo/pcl_common.hpp"
#include "percep_demo/ai_fusion.hpp"
#include "topic_macros.h"

#include "shm_struct/camera_struct.h"
#include "percep_demo/patchworkpp.hpp"
#include "percep_demo/visualize.hpp"
#include "product/product_dag.hpp"
#include "product/product_events.hpp"

#if defined(PERCEP_DNN_ENABLE)
#include "product/exact_dnn_runner.hpp"
#endif


using namespace std;
using namespace rapidjson;
using namespace cv;
const int rgb_width = 640;
const int rgb_height = 544;
const int gdc_width = 1280;
const int gdc_height = 1088;
namespace ai_node_percep_demo {

#if PERCEPTION_PRODUCT_WITH_RECHARGE
// 回充位姿/可视化话题：common_interface 未收录前在本包内定义
// AI 回充与静态 ROI 共用同一 pose / overlay 话题与 publish_overlay 开关
inline constexpr const char* kRechargeMarkPoseTopic =
    "/percep_demo/recharge_mark_pose";
inline constexpr const char* kRechargeMarkOverlayTopic =
    "/percep_demo/recharge_mark_overlay";
#endif

// PatchWork++ 地面分割配置
struct PatchworkConfig {
    bool enabled = false;
    bool verbose = false;
    bool publish_ground = true;
    bool publish_nonground = true;
    double sensor_height = 0.23;
    double min_range = 0.15;
    double max_range = 3.0;
    double th_dist = 0.05;
    double th_seeds = 0.05;
    int log_every_n = 10;
};

// 图像发布配置
struct ImagePubConfig {
    bool resize_enabled = false;
    uint32_t width = 640;
    uint32_t height = 480;
};

struct PointCloudColorConfig {
    bool enabled = true;
    // true=用 GDC 矫正图着色（与 RViz 发布图像一致）；false=用 RGBD 左目
    bool use_gdc_image = true;
    bool height_color_fallback = false;
};

#if PERCEPTION_PRODUCT_WITH_RECHARGE
// AI 回充：检测模型定位 charger 面板 → 仅在该框内做二维码识别
struct AiRechargeConfig {
    bool enabled = false;                 // 总开关；关则走静态比率 ROI
    std::string charger_class = "charger";
    float score_min = 0.25f;
    float pad_ratio = 0.08f;              // 相对框宽高扩边
    int pad_px = 16;                      // 最小扩边像素（检测图系，映射后按尺度）
    int min_box_w = 40;
    int min_box_h = 40;
    // 有有效 charger 才走 crop；无框时见 fallback_static_roi
    bool require_charger = true;
    // true：AI 开但未检出有效 charger → 回退静态 ROI；false → 本帧跳过码识别
    bool fallback_static_roi = true;
};

// 回充二维码：enabled=模块可用；真正检测需 ToPercepCmd cmd=2
// 算法/滤波/异步 worker 在二维码 so；感知只取图并发 ROS
using RechargeHostConfig = Aiper_Recharge::RechargeHostConfig;
#endif


// 检测框 → 点云匹配（像素回投影；label_fusion 关闭时的简化路径）
struct DetMatchConfig {
  bool enabled = true;           // 发布 /percep_demo/det_cloud
  bool publish_markers = true;   // 发布 /percep_demo/det_markers
  float score_min = 0.25f;
  float depth_span_m = 0.80f;    // 框内主体深度跨度
  float max_depth_m = 3.0f;      // 相机系深度上限（米）；<=0 表示不截断
};

// 图像-点云语义融合（fastdnn）：检测框+分割掩码回投影，滤拖尾/收轮廓
struct LabelFusionConfig {
  bool enabled = false;
  bool fuse_into_grid = false;  // 预留：AI 语义写栅格交给下游，本进程不写
  bool publish_debug_cloud = true;  // /percep_demo/labeled_cloud
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

struct PointCloudPubConfig {
    size_t max_points = 50000;
    float box_min_x = -3.0f;
    float box_max_x =  3.0f;
    float box_min_y = -3.0f;
    float box_max_y =  3.0f;
    float box_min_z =  0.1f;
    float box_max_z =  3.0f;
};

struct SaveMapTask
{
    perception::FrameInfo frame;
    std::uint64_t run_id=0;
    OccMapData map_data;
    std::vector<uint8_t> rgb;
    std::unique_ptr<CameraGdcData> gdc;
    std::vector<float> disp;
    std::vector<uint32_t> u;
    std::vector<uint32_t> v;
    std::string time_str;
    int rows;
    int cols;
};

class PercepDemo : public daye::Function {
public:
  explicit PercepDemo(const rapidjson::Value& ai_node_json);
  ~PercepDemo() override;

private:
  void onTimer();
  bool initParams();
#if PERCEPTION_PRODUCT_WITH_RECHARGE
  bool resolveRechargeMarkConfigPath(const std::string& start_config_path,
                                     const rapidjson::Document& ai_node_json);
#endif
  bool initInterface();
  void cleanupProductResources() noexcept;
  void buildProductMaps();
  void publishProductMaps(int count);
  void recordProductMaps();
  std::uint32_t runProductBindings(const r28::product::ProductDag::Bindings&, perception::FrameInfo);
  std::unique_ptr<r28::product::ProductDag> product_dag_;
  std::uint32_t runProductEvent(const std::string&, const r30::product::ProductEvents::Bindings&,
                                perception::FrameInfo frame = {}, std::uint64_t parent_run_id = 0,
                                r28::Status* completion_status = nullptr);
  std::unique_ptr<r30::product::ProductEvents> product_events_;
  OccMapData product_maps_;
  std::string product_dag_path_;
  std::string product_events_path_;
  std::string product_exact_profiles_;
  std::string product_metrics_dir_;
  perception::FrameInfo product_frame_;
  std::string product_extrinsics_path_ = "/app/daye/config/ai_node/extrinsics.json";
  std::string offline_rgb_dir_ = "/app/daye/bin/offline_data/rgbd_left";
  std::string offline_disparity_dir_ = "/app/daye/bin/offline_data/rgbd_disp";
  void offline_loop(); 
  bool load_files();
  void playFrame(size_t idx);
  void publish_ros_rgbcloud(PointCloudT::Ptr &cloud_in, daye::RosPublisher& pub,string frame_id,bool height_color = false);
  void publish_ros_cloud(PointCloudT::Ptr &cloud_in,daye::RosPublisher& pub,string frame_id);
  bool dense2sparse_pointcloud(CameraData& camera_data, int sample_step, PointCloudT::Ptr &out_sparse_cloud,const PointCloudPubConfig& box);
  void runGroundSegmentation(const PointCloudT::Ptr& raw_pts); 
  void postprocess(PointCloudT::Ptr& pcl_cloud,
                   const std::vector<uint8_t>& rgb_buf, int img_w, int img_h,float fx,float fy,float cx,float cy);
  bool percepStatus_callback(common_interface::srv::GetPercepStatus::Request& req,common_interface::srv::GetPercepStatus::Response& res);
  bool percepCmd_callback(common_interface::srv::ToPercepCmd::Request& req,common_interface::srv::ToPercepCmd::Response& res);
  void callAppService(int32_t cmd, int32_t status,double timeout_us);
  bool NV12ToRgbRos(const CameraData& camera_data, sensor_msgs::msg::Image& ros_img, std::vector<uint8_t>& rgb_buf);
  bool GdcToRgbRos(const CameraGdcData& camera_data,sensor_msgs::msg::Image& ros_img,std::vector<uint8_t>& rgb_buf);
  bool ResizeRosImage(sensor_msgs::msg::Image& ros_img, std::vector<uint8_t> &publish_rgb_buf_,uint32_t dst_w, uint32_t dst_h);
  static double readJsonDouble(const rapidjson::Value& obj, const char* key, double default_val);
  static uint32_t readJsonUInt(const rapidjson::Value& obj, const char* key, uint32_t default_val);
  // REP-103 高度着色：贴地绿 -> 越高越红（RViz 选 RGB8）
  static void heightToRgb(float ros_z, float sensor_h,
                          uint8_t& r, uint8_t& g, uint8_t& b);
  void convertGridMap(int count);
  std_msgs::msg::ColorRGBA createColorRGBA(uint8_t r, uint8_t g, uint8_t b);
  void publishColoredMap(const std::vector<MapPoint>& map_points,string header,int cols,int rows,int index);
  void publish3dTF(Trans &cali,const PointCloudT::Ptr& pcl_cloud,string frame_id,bool pubTf);
  void onCameraFrame(const uint8_t* data, std::size_t size, const daye::FrameInfo& info);
  void onGdcFrame(const uint8_t* data, std::size_t size, const daye::FrameInfo& info);
  void autocali(PointCloudT::Ptr &pcl_cloud);
  void scaleIntrinsics(int img_w, int img_h,
                      const common_interface::msg::CameraParam& cam,
                      float& fx, float& fy, float& cx, float& cy);
  void saveLogLoop();
  bool saveCamParams(const common_interface::msg::CameraParam &params);
#if PERCEPTION_PRODUCT_WITH_RECHARGE
  // 回充：SHM 帧交给 so；本仓只发 ROS。0=已入队，非 0=丢弃
  int submitRechargeMarkFromShm(const CameraData* rgbd, bool rgb_valid,
                                const CameraGdcData* gdc, bool gdc_valid);
  void waitRechargeMarkSettled(uint64_t done_before, int timeout_ms);
  bool startRechargeMarkWorker();
  void stopRechargeMarkWorker();
  void loadAiRechargeConfig(const std::string& json_path);
  void applyYoloAssistCropHint();
  void publishRechargeMark(const Aiper_Recharge::MarkPoseResult* r);
  static void OnRechargeMarkResult(void* userdata,
                                   Aiper_Recharge::MarkPoseResult* result);
#endif
  static bool readJsonBool(const rapidjson::Value& obj, const char* key, bool default_val);
#if defined(PERCEP_DNN_ENABLE)
  bool parseDnnConfig(const rapidjson::Value& root);
  void parseLabelFusionConfig(const rapidjson::Value& root);
  void ensureLeftYuvFormat();
  void initDnnRuntime();
  void initLabelFusionClassIds();
  void postprocessSegMask(std::vector<uint8_t>& seg_mask, int width, int height);
  void runDnnInference(const PicData& meta, const uint8_t* yuv,
                       bool run_seg = true);
  void matchAndPublishDetCloud(const PointCloudT::Ptr& cloud,
                               float fx, float fy, float cx, float cy,
                               int img_w, int img_h);
  void fuseSemanticLabels(const PointCloudT::Ptr& cloud,
                          const std::vector<DnnDetection>& dets,
                          const std::vector<uint8_t>& seg_mask, int seg_w,
                          int seg_h, int img_w, int img_h);
#endif
private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster;
  std::shared_ptr<Visualize> visualizePtr;
  std::shared_ptr<AiFusion> aiFusionPtr;   
  std::shared_ptr<patchwork::PatchWorkpp> patchwork_;
  std::shared_ptr<PclCommon> pclCommonPtr;
  rapidjson::Document global_json;
  std::unique_ptr<CameraGdcData> gdc_forsave;
  std::thread saveLogThreadHandle;
  std::mutex save_map_mtx_;
  std::mutex timer_run_mtx_;
  std::atomic<bool> shutting_down_{false};
  bool timer_started_=false;
  std::condition_variable cv_save_map;
  std::optional<std::unique_ptr<SaveMapTask>> pending_task_; // 仅保存一份待处理任务，永远最新帧
  std::atomic<bool> save_map_exit_{false};

  string sdk_so_dir,cfg_path,output_dir,jpg_dir;
  int percep_status = -1,camera_status = 0;
  int log_count = 0;
  int sample_step = 4;
  bool percep_map = false,percep_station = false,log_on = false,receive_prams=false;
  PatchworkConfig patchwork_cfg_;
  ImagePubConfig image_pub_cfg_;
  PointCloudPubConfig pcl_pub_cfg_;
  PointCloudColorConfig pcl_color_cfg_;
#if PERCEPTION_PRODUCT_WITH_RECHARGE
  RechargeHostConfig recharge_host_;
  AiRechargeConfig ai_recharge_;
  std::string recharge_mark_config_path_ =
      "/app/daye/config/ai_node/recharge_mark.json";
#endif

  // 队列/锁必须比 SHM subscriber 寿命更长：subscriber 析构先停回调线程
  std::mutex rgb_mtx_;
  std::mutex gdc_mtx_;
  std::condition_variable rgb_data_cond_;
  std::condition_variable gdc_data_cond_;
  std::queue<CameraData> rgb_buffer_queue;
  std::queue<CameraGdcData> gdc_buffer_queue;
  daye::ShmSubscriber subCamera;
  daye::ShmSubscriber subCameraGdc;
  void onCamParams(const common_interface::msg::CameraParam &params);
  common_interface::msg::CameraParam camera_params;
  int patchwork_frame_count_{0};
  std::atomic<bool> g_running{true};
  std::vector<uint8_t> pcl_proj_rgb_buf_;       // GDC 缩放到 RGBD 分辨率，供投影
  uint64_t frame_count_{0};                     // 低频心跳日志计数
  int cali_count = 0;
  float sum_roll=0,sum_pitch=0,sum_h=0;
  // 本帧存图时间戳（getSystemTimeMs），overlay / rgb / map 共用同一文件名
  std::string frame_save_time_str_;
  Trans cali;
  PointCloudT::Ptr ground_cloud,no_ground_cloud,obs_cloud,ori_cloud;
  double mapWidth,mapHeight,mapResolution;
  std::vector<float> disp_vec;
  std::vector<uint32_t> u_vec,v_vec;
  string parent_frame = "base_link";
  bool visualize = false,save_map = false,save_data = false,save_log = false;
  bool auto_cali = false,offline = false,interactive=false,self_test=false;
  vector<string> nv12_files,nv12_names,disp_files,disp_names;

private:
  rclcpp::CallbackGroup::SharedPtr sub_callback_group_;
  std::vector<uint8_t> frame_rgb_buf_;          // RGBD 左目 RGB
  daye::RosPublisher grid_map_pub_,camera_request_pub;
#if PERCEPTION_PRODUCT_WITH_RECHARGE
  daye::RosPublisher recharge_mark_pub_;
  daye::RosPublisher recharge_mark_overlay_pub_;
  Aiper_Recharge::RechargeMarkApi* recharge_api_{nullptr};
  uint64_t recharge_overlay_warn_tick_{0};
  std::atomic<bool> mark_runtime_on_{false};
  std::mutex mark_done_mtx_;
  std::condition_variable mark_done_cv_;
  std::atomic<uint64_t> mark_done_seq_{0};
  r28::Status mark_pending_status_;
  perception::FrameInfo mark_submitted_frame_;
  std::uint64_t mark_submitted_run_id_=0;
  // 未检出/无有效位姿时 DockPose 哨兵值，导航侧视为无效
  static constexpr float kDockPoseInvalidSentinel = 999.f;
#endif
  daye::RosPublisher ori_cloud_pub_;
  daye::RosPublisher ground_cloud_pub_;
  daye::RosPublisher nonground_cloud_pub_;
  daye::RosPublisher rgb_img_pub_,disp_img_pub_,gdc_img_pub_;
  daye::RosPublisher pclmap_pub_,aimap_pub_,fusionmap_pub_;
  daye::RosSubscriber cam_params_sub_;
  daye::RosTimer timer_;
  daye::RosServer percepStatus_server_;
  daye::RosServer percepCmd_server_,cameraCmd_server_;
  daye::RosClient appCmd_client_;
#if defined(PERCEP_DNN_ENABLE)
  DnnConfig dnn_cfg_;
  DetMatchConfig det_match_cfg_;
  LabelFusionConfig label_fusion_cfg_;
  std::unique_ptr<r28::product::ExactDnnRunner> dnn_runner_;
  uint64_t dnn_run_tick_{0};
  std::vector<DnnDetection> last_dnn_dets_;
  std::vector<uint8_t> last_dnn_seg_mask_;
  int last_dnn_img_w_{0};
  int last_dnn_img_h_{0};
  std::vector<int> passable_seg_ids_;
  std::vector<int> passable_det_ids_;
  // 最近一帧带 AI 标签的障碍点云（base_link，AiperPclPoint）
  // label：检测=class_id（dnn.detect.class_names 下标，keep_classes 过滤）；
  //        分割=100+seg_id（dnn.seg.class_names 下标，如 grass 的 id）；confidence=置信度
  PointCloudT::Ptr last_semantic_cloud_;
  uint64_t label_fusion_log_tick_{0};
  std::vector<uint8_t> dnn_overlay_buf_;
  daye::RosPublisher dnn_overlay_pub_;
  daye::RosPublisher dnn_seg_mask_pub_;
  daye::RosPublisher det_cloud_pub_;
  daye::RosPublisher det_markers_pub_;
  daye::RosPublisher labeled_cloud_pub_;
  bool left_yuv_is_nv12_{true};
  bool left_yuv_format_known_{false};
#endif
};

}  // namespace ai_node_percep_demo

#endif  // AI_NODE_PERCEP_DEMO_INCLUDE_PERCEP_DEMO_H_
