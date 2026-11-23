#ifndef AI_NODE_PERCEP_DEMO_PERCEP_DEMO_STATE_HPP_
#define AI_NODE_PERCEP_DEMO_PERCEP_DEMO_STATE_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <rapidjson/document.h>

#include "base_framework/daye_shm.h"
#include "base_framework/node_factory.h"
#include "percep_demo/common.hpp"
#include "percep_demo/latest_frame_channel.hpp"
#include "percep_demo/percep_demo_config.hpp"
#include "percep_demo/pre_pointcloud_types.hpp"
#include "percep_demo/recharge_api_handle.hpp"
#include "percep_demo/save_map_task.hpp"

#if defined(PERCEP_DNN_ENABLE)
#include "percep_demo/horizon_dnn_runner.hpp"
#endif

class AiFusion;
class PclCommon;
class Visualize;

namespace patchwork {
class PatchWorkpp;
}

namespace tf2_ros {
class StaticTransformBroadcaster;
}

#if defined(PERCEP_DNN_ENABLE)
namespace ai_node_percep_demo {

enum class DnnInferenceLane {
    kMap,
    kRecharge,
};

enum class DnnImageSource {
    kRgbd,
    kGdc,
};

// A DNN result is meaningful only together with the exact image identity and
// geometry that produced it. It is intentionally returned by value and kept
// in the current timer invocation instead of being shared mutable node state.
struct DnnFrameResult {
    FrameKey key;
    int image_width{0};
    int image_height{0};
    bool detection_requested{false};
    bool detection_succeeded{false};
    bool segmentation_requested{false};
    bool segmentation_succeeded{false};
    std::vector<DnnDetection> detections;
    std::vector<std::uint8_t> segmentation_mask;
    DnnInferTiming timing;

    bool matches(const Nv12Image& image) const noexcept {
        return image.valid() && sameFrame(key, image.key) &&
               image_width == image.width && image_height == image.height;
    }
};

}  // namespace ai_node_percep_demo
#endif

namespace ai_node_percep_demo::detail {

// Synchronization and coarse process state shared by callbacks and the timer.
struct RuntimeState {
    std::mutex timer_run_mtx_;
    std::mutex mode_transition_mtx_;
    std::atomic<bool> shutting_down_{false};
    std::atomic<int> percep_status_{-1};
    std::atomic<WorkMode> work_mode_{WorkMode::kIdle};
    std::atomic<bool> g_running{true};
    int log_count{0};
    std::uint64_t frame_count_{0};
    std::uint64_t skip_mark_yolo_log_tick_{0};
    std::uint64_t skip_combined_mark_log_tick_{0};
    std::uint64_t no_map_dnn_log_tick_{0};
};

// Parsed configuration. Defaults are valid, deterministic fallbacks; parsing
// may replace them only with values that pass the relevant validation.
struct ConfigurationState {
    std::string sdk_so_dir;
    std::string cfg_path;
    std::string output_dir;
    std::string jpg_dir;
    std::string recharge_mark_config_path_{
        "/app/daye/config/ai_node/recharge_mark.json"};
    std::string parent_frame{"base_link"};
    int sample_step{4};
    PatchworkConfig patchwork_cfg_;
    ImagePubConfig image_pub_cfg_;
    PointCloudPubConfig pcl_pub_cfg_;
    PointCloudColorConfig pcl_color_cfg_;
    RechargeHostConfig recharge_host_;
    AiRechargeConfig ai_recharge_;
    bool recharge_allow_source_fallback_{true};
    bool visualize{false};
    bool save_map{false};
    bool save_data{false};
    bool save_log{false};
    bool auto_cali{false};
    bool offline{false};
    bool interactive{false};
    bool self_test{false};
    double mapWidth{1.6};
    double mapHeight{2.0};
    double mapResolution{0.02};
    rapidjson::Document global_json;
};

// Stateless or internally synchronized helpers used by the processing stages.
struct ProcessingServicesState {
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster;
    std::shared_ptr<Visualize> visualizePtr;
    std::shared_ptr<AiFusion> aiFusionPtr;
    std::shared_ptr<patchwork::PatchWorkpp> patchwork_;
    std::shared_ptr<PclCommon> pclCommonPtr;
};

struct CameraIngressState {
    // Declaration order is intentional. Members are destroyed in reverse, so
    // subscribers stop/join before the channels they may publish into vanish.
    LatestFrameChannel<RgbdFramePtr> rgb_frame_channel_;
    LatestFrameChannel<GdcFramePtr> gdc_frame_channel_;
    std::atomic<bool> rgb_camera_ready_{false};
    std::atomic<bool> gdc_camera_ready_{false};
    std::atomic<std::uint64_t> invalid_rgb_frames_{0};
    std::atomic<std::uint64_t> invalid_gdc_frames_{0};
    daye::ShmSubscriber rgb_subscriber_;
    daye::ShmSubscriber gdc_subscriber_;
    mutable std::mutex camera_params_mtx_;
    std::optional<CameraIntrinsics> camera_intrinsics_;
};

struct PointCloudState {
    Trans cali{};
    int cali_count{0};
    float sum_roll{0.0F};
    float sum_pitch{0.0F};
    float sum_h{0.0F};
    PointCloudT::Ptr ground_cloud;
    PointCloudT::Ptr no_ground_cloud;
    PointCloudT::Ptr obs_cloud;
    PointCloudT::Ptr ori_cloud;
    // Kept outside DnnState because DNN-off map builds still consume an empty
    // semantic cloud at the map-fusion boundary.
    PointCloudT::Ptr last_semantic_cloud_;
    std::vector<std::uint8_t> frame_rgb_buf_;
    std::vector<std::uint8_t> pcl_proj_rgb_buf_;
    std::vector<float> disp_vec;
    std::vector<std::uint32_t> u_vec;
    std::vector<std::uint32_t> v_vec;
    GdcFramePtr gdc_for_save_;
    int patchwork_frame_count_{0};
};

struct SaveRuntimeState {
    ~SaveRuntimeState() {
        // Backstop for constructor unwinding. The normal PercepDemo destructor
        // performs the same shutdown earlier, making this path idempotent.
        save_map_exit_.store(true, std::memory_order_release);
        cv_save_map.notify_all();
        if (saveLogThreadHandle.joinable()) {
            saveLogThreadHandle.join();
        }
    }

    std::thread saveLogThreadHandle;
    std::mutex save_map_mtx_;
    std::condition_variable cv_save_map;
    std::optional<std::unique_ptr<SaveMapTask>> pending_task_;
    std::atomic<bool> save_map_exit_{false};
    std::string frame_save_time_str_;
    std::vector<std::string> nv12_files;
    std::vector<std::string> nv12_names;
    std::vector<std::string> disp_files;
    std::vector<std::string> disp_names;
};

struct RechargeRuntimeState {
    RechargeApiPtr recharge_api_;
    std::uint64_t recharge_overlay_warn_tick_{0};
    std::atomic<bool> mark_runtime_on_{false};
    std::mutex mark_done_mtx_;
    std::condition_variable mark_done_cv_;
    std::atomic<std::uint64_t> mark_done_seq_{0};
    static constexpr float kDockPoseInvalidSentinel = 999.0F;
};

struct RosEndpointsState {
    rclcpp::CallbackGroup::SharedPtr sub_callback_group_;
    daye::RosPublisher grid_map_pub_;
    daye::RosPublisher camera_request_pub;
    daye::RosPublisher recharge_mark_pub_;
    daye::RosPublisher recharge_mark_overlay_pub_;
    daye::RosPublisher ori_cloud_pub_;
    daye::RosPublisher ground_cloud_pub_;
    daye::RosPublisher nonground_cloud_pub_;
    daye::RosPublisher rgb_img_pub_;
    daye::RosPublisher disp_img_pub_;
    daye::RosPublisher gdc_img_pub_;
    daye::RosPublisher pclmap_pub_;
    daye::RosPublisher aimap_pub_;
    daye::RosPublisher fusionmap_pub_;
    daye::RosSubscriber cam_params_sub_;
    daye::RosTimer timer_;
    daye::RosServer percepStatus_server_;
    daye::RosServer percepCmd_server_;
    daye::RosClient appCmd_client_;
};

#if defined(PERCEP_DNN_ENABLE)
struct DnnRuntimeState {
    DnnConfig dnn_cfg_;
    DetMatchConfig det_match_cfg_;
    LabelFusionConfig label_fusion_cfg_;
    std::unique_ptr<HorizonDnnRunner> dnn_runner_;
    // Map and recharge may both infer during one timer tick. Independent
    // cadence counters prevent run_every_n from starving one lane forever.
    std::uint64_t map_dnn_call_index_{0};
    std::uint64_t recharge_dnn_call_index_{0};
    std::vector<int> passable_seg_ids_;
    std::vector<int> passable_det_ids_;
    std::uint64_t label_fusion_log_tick_{0};
    std::vector<std::uint8_t> dnn_overlay_buf_;
    std::vector<std::uint8_t> dnn_overlay_scaled_buf_;
    std::uint64_t dnn_overlay_save_tick_{0};
    std::uint64_t dnn_overlay_saved_count_{0};
    bool dnn_overlay_save_limit_logged_{false};
    std::uint64_t yolo_crop_miss_log_tick_{0};
    std::uint64_t yolo_crop_hit_log_tick_{0};
    std::chrono::steady_clock::time_point dnn_fps_window_start_{
        std::chrono::steady_clock::now()};
    std::uint64_t dnn_fps_window_count_{0};
    double dnn_fps_ema_{0.0};
    std::mutex yuv_format_mtx_;
    daye::RosPublisher dnn_overlay_pub_;
    daye::RosPublisher det_cloud_pub_;
    bool rgbd_yuv_is_nv12_{true};
    bool rgbd_yuv_format_known_{false};
    bool gdc_yuv_is_nv12_{true};
    bool gdc_yuv_format_known_{false};
};
#else
// Keeps the PercepDemo base list stable and readable in DNN-off builds. Empty
// base optimization means this placeholder does not need per-object storage.
struct DnnRuntimeState {};
#endif

}  // namespace ai_node_percep_demo::detail

#endif  // AI_NODE_PERCEP_DEMO_PERCEP_DEMO_STATE_HPP_
