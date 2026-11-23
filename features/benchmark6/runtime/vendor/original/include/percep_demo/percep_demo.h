/**
 * @file percep_demo.h
 * @brief Perception orchestration node.
 *
 * The class remains the integration boundary required by daye::Function.
 * Configuration records and worker task payloads live in dedicated headers;
 * declarations below are grouped by subsystem so the runtime flow is visible.
 *
 * Copyright 2026 Daye Technology Co.Ltd. All rights reserved.
 */
#ifndef AI_NODE_PERCEP_DEMO_INCLUDE_PERCEP_DEMO_H_
#define AI_NODE_PERCEP_DEMO_INCLUDE_PERCEP_DEMO_H_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <rapidjson/document.h>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "base_framework/daye_shm.h"
#include "base_framework/function_factory.h"
#include "base_framework/node_factory.h"
#include "common_interface/msg/camera_param.hpp"
#include "common_interface/srv/get_percep_status.hpp"
#include "common_interface/srv/to_percep_cmd.hpp"
#include "percep_demo/common.hpp"
#include "percep_demo/percep_demo_config.hpp"
#include "percep_demo/percep_demo_state.hpp"
#include "percep_demo/pre_pointcloud_types.hpp"
#include "percep_demo/recharge_api_handle.hpp"

#if defined(PERCEP_DNN_ENABLE)
#include "percep_demo/horizon_dnn_runner.hpp"
#endif

namespace ai_node_percep_demo {

class PercepDemo : public daye::Function,
                   private detail::RuntimeState,
                   private detail::ConfigurationState,
                   private detail::ProcessingServicesState,
                   private detail::PointCloudState,
                   private detail::SaveRuntimeState,
                   private detail::RechargeRuntimeState,
                   private detail::DnnRuntimeState,
                   private detail::CameraIngressState,
                   private detail::RosEndpointsState {
public:
    explicit PercepDemo(const rapidjson::Value& ai_node_json);
    ~PercepDemo() override;

private:
    // ---------------------------------------------------------------------
    // Lifecycle, configuration and interface construction
    // ---------------------------------------------------------------------
    bool initParams();
    bool resolveRechargeMarkConfigPath(
        const std::string& start_config_path,
        const rapidjson::Document& ai_node_json);
    bool initInterface();
    bool startOnlineRuntime();

    static double readJsonDouble(const rapidjson::Value& object,
                                 const char* key, double default_value);
    static std::uint32_t readJsonUInt(const rapidjson::Value& object,
                                      const char* key,
                                      std::uint32_t default_value);
    static bool readJsonBool(const rapidjson::Value& object,
                             const char* key, bool default_value);

    // ---------------------------------------------------------------------
    // Camera ingress and calibration
    // ---------------------------------------------------------------------
    void onCameraFrame(const std::uint8_t* data, std::size_t size,
                       const daye::FrameInfo& info);
    void onGdcFrame(const std::uint8_t* data, std::size_t size,
                    const daye::FrameInfo& info);
    void onCamParams(const common_interface::msg::CameraParam& params);
    std::optional<CameraIntrinsics> cameraIntrinsicsSnapshot() const;

    // ---------------------------------------------------------------------
    // Runtime control and one-frame orchestration
    // ---------------------------------------------------------------------
    WorkMode currentWorkMode() const noexcept;
    void onTimer();
    bool percepStatus_callback(
        common_interface::srv::GetPercepStatus::Request& request,
        common_interface::srv::GetPercepStatus::Response& response);
    bool percepCmd_callback(
        common_interface::srv::ToPercepCmd::Request& request,
        common_interface::srv::ToPercepCmd::Response& response);
    void callAppService(std::int32_t command, std::int32_t status,
                        double timeout_us);

    // ---------------------------------------------------------------------
    // Image conversion
    // ---------------------------------------------------------------------
    bool decodeYuv420ToRgb(const Nv12Image& image,
                           std::vector<std::uint8_t>& rgb_buffer);
    static void fillRgbRosImage(
        const Nv12Image& image,
        const std::vector<std::uint8_t>& rgb_buffer,
        sensor_msgs::msg::Image& ros_image);
    bool ResizeRosImage(sensor_msgs::msg::Image& ros_image,
                        std::vector<std::uint8_t>& publish_rgb_buffer,
                        std::uint32_t destination_width,
                        std::uint32_t destination_height);

    // ---------------------------------------------------------------------
    // Point-cloud generation and processing
    // ---------------------------------------------------------------------
    bool dense2sparse_pointcloud(const PrePointCloudInput& input,
                                 int sample_step,
                                 PointCloudT::Ptr& output_cloud,
                                 const PointCloudPubConfig& bounds);
    void runGroundSegmentation(const PointCloudT::Ptr& raw_points);
    void postprocess(PointCloudT::Ptr& cloud,
                     const std::vector<std::uint8_t>& rgb_buffer,
                     int image_width, int image_height, float fx, float fy,
                     float cx, float cy);
    void autocali(PointCloudT::Ptr& cloud);
    static void heightToRgb(float ros_z, float sensor_height,
                            std::uint8_t& red, std::uint8_t& green,
                            std::uint8_t& blue);

    // ---------------------------------------------------------------------
    // Map, point-cloud and TF output
    // ---------------------------------------------------------------------
    void convertGridMap(int count);
    void publishColoredMap(const std::vector<MapPoint>& map_points,
                           std::string header, int columns, int rows,
                           int index);
    std_msgs::msg::ColorRGBA createColorRGBA(std::uint8_t red,
                                              std::uint8_t green,
                                              std::uint8_t blue);
    void publish_ros_rgbcloud(PointCloudT::Ptr& cloud,
                              daye::RosPublisher& publisher,
                              std::string frame_id,
                              bool height_color = false);
    void publish_ros_cloud(PointCloudT::Ptr& cloud,
                           daye::RosPublisher& publisher,
                           std::string frame_id);
    void publish3dTF(Trans& calibration, const PointCloudT::Ptr& cloud,
                     std::string frame_id, bool publish_tf);

    // ---------------------------------------------------------------------
    // Persistence and offline playback
    // ---------------------------------------------------------------------
    void saveLogLoop();
    bool saveCamParams(const common_interface::msg::CameraParam& params);
    void offline_loop();
    bool load_files();
    void playFrame(std::size_t index);

    // ---------------------------------------------------------------------
    // Recharge-marker runtime
    // ---------------------------------------------------------------------
    bool loadAiRechargeConfig(const std::string& json_path);
    bool startRechargeMarkWorker();
    void stopRechargeMarkWorker();
    int submitRechargeMark(
        const Nv12Image& image,
        const std::optional<CameraIntrinsics>& calibration);
    void waitRechargeMarkSettled(std::uint64_t done_before, int timeout_ms);
    void publishRechargeMark(const Aiper_Recharge::MarkPoseResult* result);
    static void OnRechargeMarkResult(
        void* userdata, Aiper_Recharge::MarkPoseResult* result);

#if defined(PERCEP_DNN_ENABLE)
    // ---------------------------------------------------------------------
    // Detect, segmentation and semantic fusion
    // ---------------------------------------------------------------------
    bool parseDnnConfig(const rapidjson::Value& root);
    void parseLabelFusionConfig(const rapidjson::Value& root);
    Yuv420Format resolveLeftYuvFormat(const Nv12Image& image,
                                      DnnImageSource source);
    bool consumeDnnCadence(DnnInferenceLane lane);
    void initDnnRuntime();
    void initLabelFusionClassIds();
    void postprocessSegMask(std::vector<std::uint8_t>& segmentation_mask,
                            int width, int height);
    std::optional<DnnFrameResult> runDnnInference(
        const Nv12Image& image, DnnInferenceLane lane,
        bool run_segmentation = true);
    void applyYoloAssistCropHint(const Nv12Image& image,
                                 const DnnFrameResult* result);
    void matchAndPublishDetCloud(const PointCloudT::Ptr& cloud,
                                 const Nv12Image& image,
                                 const DnnFrameResult& result,
                                 float fx, float fy, float cx, float cy);
    void fuseSemanticLabels(const PointCloudT::Ptr& cloud,
                            const std::vector<DnnDetection>& detections,
                            const std::vector<std::uint8_t>& segmentation_mask,
                            int segmentation_width, int segmentation_height,
                            int image_width, int image_height, float fx,
                            float fy, float cx, float cy);
#endif

};

}  // namespace ai_node_percep_demo

#endif  // AI_NODE_PERCEP_DEMO_INCLUDE_PERCEP_DEMO_H_
