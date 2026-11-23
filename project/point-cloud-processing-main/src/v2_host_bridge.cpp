#include "percep_main/percep_main.h"
#include "percep_main/v2_frame_input.hpp"

#include "common_interface/msg/dock_pose.hpp"

#include <cstdlib>
#include <string_view>

namespace perception {
namespace {

bool readJson(const std::filesystem::path& path, rapidjson::Document& json)
{
    std::ifstream file(path, std::ios::binary);
    if (!file) return false;
    const std::string text{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
    return !json.Parse(text.c_str()).HasParseError() && json.IsObject();
}

template<typename Params>
psdk_camera_intrinsics convertIntrinsics(const Params& params)
{
    return {uint32_t(params.width), uint32_t(params.height),
            params.fx, params.fy, params.cx, params.cy, params.bxf};
}

}  // namespace

std::string PercepMain::configDirectory() const
{
    const char* path = std::getenv("PERCEP_CONFIG_DIR");
    return path && *path ? path : "/app/daye/config/ai_node";
}

bool PercepMain::parseV2Config(const rapidjson::Value& root)
{
    if (!root.HasMember("v2_backend")) return true;
    const auto& backend = root["v2_backend"];
    const auto invalid = [] { LOG_ERROR("[V2] invalid v2_backend configuration"); return false; };
    if (!backend.IsObject()) return invalid();
    for (const char* name : {"map_enabled", "recharge_enabled"}) {
        if (backend.HasMember(name) && !backend[name].IsBool()) return invalid();
    }
    v2_map_ = backend.HasMember("map_enabled") && backend["map_enabled"].GetBool();
    v2_recharge_ = backend.HasMember("recharge_enabled") && backend["recharge_enabled"].GetBool();
    if (v2_recharge_ && !v2_map_) return invalid();
    if (!v2_map_) return true;
    if (offline || stereo_cfg_.enabled || auto_cali) {
        LOG_ERROR("[V2] this integration requires board input, SDK Q8 and auto_cali=false");
        return false;
    }
    const auto path = [this](const char* default_name) {
        return (std::filesystem::path(configDirectory()) / "v2_bridge" / default_name).string();
    };
    v2_map_config_ = path("map_q8.json");
    v2_recharge_config_ = path("recharge_gdc.json");
    return true;
}

bool PercepMain::initV2Backend()
{
    if (!v2_map_) return true;
    rapidjson::Document profile;
    if (!readJson(v2_map_config_, profile) || !profile.HasMember("input") || !profile["input"].IsObject()
        || !profile["input"].HasMember("disparity_source") || !profile["input"]["disparity_source"].IsString()
        || std::string(profile["input"]["disparity_source"].GetString()) != "original_q8") {
        LOG_ERROR("[V2] map profile must use original_q8");
        return false;
    }
    std::string error;
    if (!v2_map_runtime_.open(v2_map_config_, error)) {
        LOG_ERROR("[V2] map initialization: %s", error.c_str());
        return false;
    }
    if (!v2_recharge_) return true;
    rapidjson::Document recharge;
    if (!readJson(std::filesystem::path(configDirectory()) / "recharge_mark.json", recharge)
        || !recharge.HasMember("publish_enabled") || !recharge["publish_enabled"].IsBool()
        || !recharge.HasMember("use_hires") || !recharge["use_hires"].IsTrue()
        || (recharge.HasMember("publish_overlay") && !recharge["publish_overlay"].IsFalse())) {
        LOG_ERROR("[V2] recharge requires GDC and publish_overlay=false");
        return false;
    }
    if (!v2_recharge_runtime_.open(v2_recharge_config_, error)) {
        LOG_ERROR("[V2] recharge initialization: %s", error.c_str());
        return false;
    }
    v2_publish_pose_ = recharge["publish_enabled"].GetBool();
    if (v2_publish_pose_) {
        v2_pose_pub_ = daye::CreatePublish<common_interface::msg::DockPose>(
            "/percep_demo/recharge_mark_pose", rclcpp::SensorDataQoS());
    }
    return true;
}

int PercepMain::switchV2Task(int command)
{
    std::lock_guard<std::mutex> lock(v2_task_mtx_);
    if (command == v2_command_) return 1;
    // A mode change and its queue flush are serialized with computation and publication.
    std::string error;
    const bool old_map_stopped = v2_command_ != 1 || v2_map_runtime_.command(21, error);
    const bool old_recharge_stopped = v2_command_ != 2 || !v2_recharge_
        || v2_recharge_runtime_.command(21, error);
    releaseTaskResources();
    {
        std::lock_guard<std::mutex> rgb_lock(rgb_mtx_);
        std::queue<CameraData>().swap(rgb_buffer_queue);
    }
    v2_command_ = 21;
    int status = 2;
    if (old_map_stopped && old_recharge_stopped) {
        bool started = true;
        if (command == 1) started = v2_map_runtime_.command(1, error);
        if (command == 2) {
            if (v2_recharge_) {
                started = v2_recharge_runtime_.command(2, error);
            } else {
#if PERCEP_ENABLE_RECHARGE
                started = recharge_ && recharge_->Start();
#else
                started = false;
#endif
                if (!started) error = "legacy recharge start failed";
            }
        }
        if (started) {
            v2_command_ = command;
            status = 1;
        }
    }
    if (status != 1) LOG_ERROR("[V2] command %d failed: %s", command, error.c_str());
    percep_map.store(v2_command_ == 1);
    percep_station.store(v2_command_ == 2);
    stop_percep.store(v2_command_ == 21);
    percep_status = v2_command_ == 21 ? 0 : v2_command_;
    ++v2_generation_;
    rgb_data_cond_.notify_all();
    gdc_data_cond_.notify_all();
    return status;
}

bool PercepMain::runV2Map(const CameraData& packet)
{
    psdk_camera_intrinsics intrinsics{};
    {
        std::lock_guard<std::mutex> lock(camera_params_mtx_);
        intrinsics = convertIntrinsics(camera_params);
    }
    psdk_disparity_q8 disparity{};
    psdk_full_frame frame{};
    if (!makeV2MapInput(packet, intrinsics, disparity, frame)) {
        LOG_ERROR("[V2] rejected NV12/Q8/CameraParam frame %llu", static_cast<unsigned long long>(packet.left.frame_id));
        return false;
    }
    psdk_full_result_view view{};
    std::string error;
    auto result = v2_map_runtime_.process(frame, view, error);
    if (!result) {
        LOG_ERROR("[V2] map process: %s", error.c_str());
        return false;
    }
    const auto map_end = view.maps ? view.maps + view.map_count : nullptr;
    const auto fusion = view.maps
        ? std::find_if(view.maps, map_end, [](const auto& map) { return std::string_view(map.name) == "fusion"; })
        : nullptr;
    if (fusion == map_end || !view.grid_count || !view.avg_h_cm || !view.min_h_cm
        || !view.max_h_cm || !view.grid_types || view.grid_count != uint64_t(fusion->rows) * fusion->cols
        || fusion->rows > UINT16_MAX || fusion->cols > UINT16_MAX || fusion->resolution_mm > UINT16_MAX) {
        LOG_ERROR("[V2] incomplete fusion grid");
        return false;
    }
    common_interface::msg::GridMapInfo msg;
    const auto stamp_ms = daye::getSteadyTimeMs();
    msg.header.stamp.sec = stamp_ms / 1000;
    msg.header.stamp.nanosec = (stamp_ms % 1000) * 1000000;
    msg.header.frame_id = parent_frame;
    msg.rows = fusion->rows;
    msg.cols = fusion->cols;
    msg.resolution_mm = fusion->resolution_mm;
    msg.avg_h_cm.assign(view.avg_h_cm, view.avg_h_cm + view.grid_count);
    msg.min_h_cm.assign(view.min_h_cm, view.min_h_cm + view.grid_count);
    msg.max_h_cm.assign(view.max_h_cm, view.max_h_cm + view.grid_count);
    msg.type.assign(view.grid_types, view.grid_types + view.grid_count);
    grid_map_pub_.publish(msg);
    if (visualize) {
        sensor_msgs::msg::Image image;
        if (NV12ToRgbRos(packet, image, frame_rgb_buf_)) {
            image.header.frame_id = "percep_main";
            rgb_img_pub_.publish(image);
        }
        for (const auto* map = view.maps; map != map_end; ++map) {
            if (!map->cells || map->cell_count != uint64_t(map->rows) * map->cols) continue;
            std::vector<MapPoint> cells;
            cells.reserve(map->cell_count);
            for (uint64_t i = 0; i < map->cell_count; ++i) {
                const auto& cell = map->cells[i];
                cells.push_back({cell.min_h_m, cell.max_h_m, cell.avg_h_m, cell.type});
            }
            publishColoredMap(cells, map->name, frame_count_);
        }
    }
    if (save_log) {
        const uint64_t now_ms = daye::getSystemTimeMs();
        if (save_interval_ms <= 0 || !last_saved_frame_ms_
            || now_ms - last_saved_frame_ms_ >= uint64_t(save_interval_ms)) {
            const auto dir = std::filesystem::path(output_dir) / "v2" /
                (std::to_string(frame.left.timestamp_ns) + "-" + std::to_string(frame.left.frame_id));
            {
                std::lock_guard<std::mutex> lock(save_map_mtx_);
                pending_v2_save_.emplace(std::move(result), dir.string());
            }
            last_saved_frame_ms_ = now_ms;
            cv_save_map.notify_one();
        }
    }
    return true;
}

bool PercepMain::runV2Recharge(const CameraGdcData& packet)
{
    psdk_camera_intrinsics intrinsics{};
    {
        std::lock_guard<std::mutex> lock(camera_params_mtx_);
        intrinsics = convertIntrinsics(camera_params);
    }
    psdk_frame hires{};
    psdk_full_frame frame{};
    if (!makeV2RechargeInput(packet, intrinsics, hires, frame)) {
        LOG_ERROR("[V2] rejected recharge GDC frame %llu", static_cast<unsigned long long>(packet.left.frame_id));
        return false;
    }
    psdk_full_result_view view{};
    std::string error;
    const auto result = v2_recharge_runtime_.process(frame, view, error);
    if (!result) {
        LOG_ERROR("[V2] recharge process: %s", error.c_str());
        return false;
    }
    if (!view.recharge.executed || !v2_publish_pose_) return true;
    const auto values = v2DockValues(view.recharge);
    const uint64_t stamp = view.recharge.timestamp_ns > 0
        ? uint64_t(view.recharge.timestamp_ns) : hires.timestamp_ns;
    common_interface::msg::DockPose msg;
    msg.header.stamp.sec = stamp / 1000000000ull;
    msg.header.stamp.nanosec = stamp % 1000000000ull;
    msg.header.frame_id = "base_link";
    msg.x = values.x;
    msg.y = values.y;
    msg.yaw = values.yaw;
    v2_pose_pub_.publish(msg);
    return true;
}

}  // namespace perception
