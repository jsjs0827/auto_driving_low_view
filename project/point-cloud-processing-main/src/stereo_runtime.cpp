#include "percep_main/percep_main.h"

#include <chrono>
#include <cmath>
#include <limits>

namespace perception {
namespace {

bool readStereoBool(const rapidjson::Value& object, const char* key, bool* value) {
    if (!object.HasMember(key)) return true;
    if (!object[key].IsBool()) return false;
    *value = object[key].GetBool();
    return true;
}

bool readStereoInt(const rapidjson::Value& object, const char* key, int* value) {
    if (!object.HasMember(key)) return true;
    if (!object[key].IsInt()) return false;
    *value = object[key].GetInt();
    return true;
}

bool readStereoString(const rapidjson::Value& object, const char* key, std::string* value) {
    if (!object.HasMember(key)) return true;
    if (!object[key].IsString()) return false;
    *value = object[key].GetString();
    return true;
}

bool stereoImage(const PicData& meta, const uint8_t* data, size_t capacity,
                 const std::string& format, int expected_width, int expected_height,
                 StereoImageView* image, std::string* error) {
    const auto fail = [&](const char* message) {
        if (error) *error = message;
        return false;
    };
    if (!data || meta.width != static_cast<uint32_t>(expected_width) ||
        meta.height != static_cast<uint32_t>(expected_height) ||
        expected_width < 2 || expected_height < 2 ||
        (expected_width & 1) || (expected_height & 1)) {
        return fail("left/right metadata must match the configured full-source dimensions");
    }
    const size_t bytes = static_cast<size_t>(meta.bytes_used);
    const size_t rows = static_cast<size_t>(expected_height) * 3 / 2;
    if (bytes == 0 || bytes > capacity || bytes % rows != 0) {
        return fail("left/right YUV byte count exceeds its buffer or cannot describe a YUV420 stride");
    }
    const size_t stride = bytes / rows;
    if (stride < static_cast<size_t>(expected_width) || (stride & 1) ||
        stride > static_cast<size_t>(std::numeric_limits<int>::max())) {
        return fail("left/right YUV stride must be even and at least the visible width");
    }
    image->data = data;
    image->bytes = bytes;
    image->width = expected_width;
    image->height = expected_height;
    image->stride = static_cast<int>(stride);
    image->nv12 = format == "nv12";
    image->frame_id = static_cast<uint64_t>(meta.frame_id);
    image->timestamp_ns = static_cast<uint64_t>(meta.time_ns);
    return true;
}

bool stereoImages(const CameraData& frame, const StereoRuntimeConfig& config,
                  StereoImageView* left, StereoImageView* right, std::string* error) {
    if (!stereoImage(frame.left, frame.left_data, sizeof(frame.left_data),
                     config.left_yuv_format, config.source_width, config.source_height,
                     left, error) ||
        !stereoImage(frame.right, frame.right_data, sizeof(frame.right_data),
                     config.right_yuv_format, config.source_width, config.source_height,
                     right, error)) return false;
    if ((left->frame_id != 0 || right->frame_id != 0) &&
        (left->frame_id == 0 || right->frame_id == 0 || left->frame_id != right->frame_id)) {
        if (error) *error = "left/right frame IDs are missing on one side or differ";
        return false;
    }
    if ((left->timestamp_ns != 0 || right->timestamp_ns != 0) &&
        (left->timestamp_ns == 0 || right->timestamp_ns == 0 ||
         left->timestamp_ns != right->timestamp_ns)) {
        if (error) *error = "left/right capture timestamps are missing on one side or differ";
        return false;
    }
    return true;
}

} // namespace

bool PercepMain::parseStereoConfig(const rapidjson::Value& root) {
    stereo_cfg_ = StereoRuntimeConfig{};
    if (!root.HasMember("stereo")) return true;
    if (!root["stereo"].IsObject()) {
        LOG_ERROR("[Stereo] stereo must be a JSON object");
        return false;
    }
    const auto& object = root["stereo"];
    auto& c = stereo_cfg_;
    if (!readStereoBool(object, "enabled", &c.enabled) ||
        !readStereoBool(object, "crop", &c.crop) ||
        !readStereoBool(object, "rectified", &c.rectified) ||
        !readStereoInt(object, "source_width", &c.source_width) ||
        !readStereoInt(object, "source_height", &c.source_height) ||
        !readStereoInt(object, "crop_height", &c.crop_height) ||
        !readStereoString(object, "full_model_path", &c.full_model_path) ||
        !readStereoString(object, "crop_model_path", &c.crop_model_path) ||
        !readStereoString(object, "left_yuv_format", &c.left_yuv_format) ||
        !readStereoString(object, "right_yuv_format", &c.right_yuv_format) ||
        !readStereoString(object, "right_rgb_dir", &c.right_rgb_dir)) {
        LOG_ERROR("[Stereo] invalid stereo field type (use JSON booleans, integer dimensions and string paths/formats)");
        return false;
    }
    if (!c.enabled) return true;
    // Existing RGB, semantic projection and map buffers use the original full grid.
    if (c.source_width != rgb_width || c.source_height != rgb_height ||
        c.crop_height < 2 || (c.crop_height & 1) || c.crop_height > c.source_height) {
        LOG_ERROR("[Stereo] map integration requires the original %dx%d grid and an even crop_height within it",
                  rgb_width, rgb_height);
        return false;
    }
    if (!c.rectified) {
        LOG_ERROR("[Stereo] enabled stereo requires rectified=true for a genuinely rectified same-capture pair");
        return false;
    }
    // The original map RGB conversion consumes NV12. The SDK itself also
    // supports I420, but accepting I420 on this application's left lane would
    // silently make RGB/overlay conversion inconsistent with model input.
    if (c.left_yuv_format != "nv12" ||
        (c.right_yuv_format != "nv12" && c.right_yuv_format != "i420")) {
        LOG_ERROR("[Stereo] map left_yuv_format must be nv12; right_yuv_format must be nv12 or i420");
        return false;
    }
    if (dnn_cfg_.enabled && dnn_cfg_.left_yuv_format != c.left_yuv_format) {
        LOG_ERROR("[Stereo] DNN and stereo must interpret the same left YUV format");
        return false;
    }
    if ((c.crop ? c.crop_model_path : c.full_model_path).empty()) {
        LOG_ERROR("[Stereo] selected stereo model path is empty");
        return false;
    }
    if (offline && c.right_rgb_dir.empty()) {
        LOG_ERROR("[Stereo] offline stereo requires stereo.right_rgb_dir with matching left/right stems");
        return false;
    }
    return true;
}

bool PercepMain::initStereoRuntime() {
    stereo_frame_ = StereoDepthResult{};
    stereo_frame_intrinsics_ = StereoIntrinsics{};
    stereo_runner_.reset();
    if (!stereo_cfg_.enabled) {
        LOG_INFO("[Stereo] disabled; map uses the existing SDK/offline disparity path");
        return true;
    }
    StereoDepthConfig config;
    config.model_path = stereo_cfg_.crop ? stereo_cfg_.crop_model_path : stereo_cfg_.full_model_path;
    config.crop = stereo_cfg_.crop;
    config.rectified = stereo_cfg_.rectified;
    config.source_width = stereo_cfg_.source_width;
    config.source_height = stereo_cfg_.source_height;
    config.crop_height = stereo_cfg_.crop_height;
    auto runner = std::make_unique<StereoDepthRunner>();
    if (!runner->init(config)) {
        LOG_ERROR("[Stereo] init failed: %s", runner->lastError().c_str());
        return false; // An explicitly requested stereo path never silently falls back.
    }
    LOG_INFO("[Stereo] ready model=%s crop=%s source=%dx%d crop_height=%d",
              config.model_path.c_str(), config.crop ? "bottom" : "off",
              config.source_width, config.source_height, config.crop_height);
    LOG_INFO("[Stereo] model contract: %s", runner->contractJson().c_str());
    stereo_runner_ = std::move(runner);
    return true;
}

bool PercepMain::validateStereoFrame(const CameraData& frame, std::string* error) const {
    StereoImageView left, right;
    return stereoImages(frame, stereo_cfg_, &left, &right, error);
}

bool PercepMain::generateStereoPointCloud(CameraData& frame, int step, PointCloudT::Ptr& cloud) {
    if (!cloud) cloud.reset(new PointCloudT);
    cloud->clear();
    u_vec.clear(); v_vec.clear(); disp_vec.clear();
    stereo_frame_ = StereoDepthResult{};
    stereo_frame_intrinsics_ = StereoIntrinsics{};
    if (!stereo_runner_) {
        LOG_ERROR("[Stereo] map requested stereo before successful initialization");
        return false;
    }
    StereoImageView left, right;
    std::string error;
    if (!stereoImages(frame, stereo_cfg_, &left, &right, &error)) {
        LOG_ERROR("[Stereo] invalid same-frame pair: %s", error.c_str());
        return false;
    }
    StereoIntrinsics intrinsics;
    {
        std::lock_guard<std::mutex> lock(camera_params_mtx_);
        const double width = camera_params.width, height = camera_params.height;
        if (!std::isfinite(width) || !std::isfinite(height) ||
            width != stereo_cfg_.source_width || height != stereo_cfg_.source_height) {
            LOG_ERROR("[Stereo] calibration dimensions must match the full-source pair");
            return false;
        }
        intrinsics.width = stereo_cfg_.source_width;
        intrinsics.height = stereo_cfg_.source_height;
        intrinsics.fx = camera_params.fx; intrinsics.fy = camera_params.fy;
        intrinsics.cx = camera_params.cx; intrinsics.cy = camera_params.cy;
        intrinsics.bxf_px_mm = camera_params.bxf;
    }
    StereoDepthResult result;
    if (!stereo_runner_->inferPair(left, right, intrinsics, &result)) {
        LOG_ERROR("[Stereo] inference failed: %s", stereo_runner_->lastError().c_str());
        return false;
    }
    if (result.width != intrinsics.width || result.height != intrinsics.height ||
        result.q8.size() != static_cast<size_t>(result.width) * result.height) {
        LOG_ERROR("[Stereo] SDK returned a disparity grid inconsistent with full-image calibration");
        return false;
    }
    const auto& t = result.timing;
    LOG_INFO("[Stereo] pre_ms=%.3f bpu_submit_ms=%.3f bpu_wait_ms=%.3f "
             "bpu_submit_wait_ms=%.3f post_ms=%.3f total_ms=%.3f crop_y=%d "
             "bpu_scope=host_submit_and_wait_including_queue",
             t.pre_ms, t.bpu_submit_ms, t.bpu_wait_ms, t.bpu_submit_wait_ms,
             t.post_ms, t.total_ms, result.crop_y);
    const auto start = std::chrono::steady_clock::now();
    const bool sparse_ok = dense2sparse_pointcloud(frame, step, cloud, result.q8.data(),
                                                  result.width, result.height, &intrinsics);
    const double sparse_ms = std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now() - start).count();
    LOG_INFO("[Stereo] sparse_ms=%.3f points=%zu status=%s",
             sparse_ms, cloud->size(), sparse_ok ? "ok" : "failed");
    if (!sparse_ok) return false;
    stereo_frame_intrinsics_ = intrinsics;
    stereo_frame_ = std::move(result);
    return true;
}

} // namespace perception
