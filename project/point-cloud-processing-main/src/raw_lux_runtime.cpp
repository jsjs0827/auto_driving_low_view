#include "percep_main/percep_main.h"
#include "raw_lux_io.hpp"

#include <algorithm>
#include <stdexcept>

namespace perception {
namespace {
std::uint64_t captureStamp(const sensor_msgs::msg::Image& image) {
    if (image.header.stamp.sec < 0 || image.header.stamp.nanosec >= 1000000000U) return 0;
    return static_cast<std::uint64_t>(image.header.stamp.sec)*1000000000ULL + image.header.stamp.nanosec;
}
void checkRawImage(const sensor_msgs::msg::Image& image) {
    if (image.width != raw_lux_io::kWidth || image.height != raw_lux_io::kHeight ||
        image.step != raw_lux_io::kStride || image.data.size() != raw_lux_io::kFrameBytes || image.is_bigendian != 0)
        throw std::invalid_argument("RAW must be native1088x1280, stride2176, unpacked uint16LE, exactly2785280 bytes");
    if (image.encoding != "bayer_bggr16" && image.encoding != "mono16" && image.encoding != "16UC1")
        throw std::invalid_argument("RAW requires BGGR16 or an explicitly declared native single16 container; YUV/RGB is not RAW");
    if (!captureStamp(image)) throw std::invalid_argument("RAW capture timestamp is missing or invalid");
}
} // namespace

bool PercepMain::parseRawLuxConfig(const rapidjson::Value& root) {
    raw_lux_cfg_ = RawLuxRuntimeConfig{};
    if (!root.HasMember("raw_lux")) return true;
    if (!root["raw_lux"].IsObject()) { LOG_ERROR("[RawLux] raw_lux must be an object"); return false; }
    const auto& object = root["raw_lux"];
    auto& c = raw_lux_cfg_;
    if (object.HasMember("enabled")) {
        if (!object["enabled"].IsBool()) { LOG_ERROR("[RawLux] enabled must be a JSON boolean"); return false; }
        c.enabled = object["enabled"].GetBool();
    }
    if (!c.enabled) return true;
    if (!raw_lux_io::stringField(object, "model_name", &c.algorithm.model_name) ||
        !raw_lux_io::stringField(object, "metadata_source", &c.algorithm.metadata_source) ||
        !raw_lux_io::stringField(object, "raw_left_topic", &c.raw_left_topic) ||
        !raw_lux_io::stringField(object, "raw_right_topic", &c.raw_right_topic) ||
        !raw_lux_io::stringField(object, "raw_left_dir", &c.raw_left_dir) ||
        !raw_lux_io::stringField(object, "raw_right_dir", &c.raw_right_dir) ||
        !raw_lux_io::stringField(object, "metadata_sidecar_dir", &c.metadata_sidecar_dir)) {
        LOG_ERROR("[RawLux] model/source/topic/directory fields must be strings"); return false;
    }
    if (c.algorithm.model_name != "raw_hist_org_logistic" && c.algorithm.model_name != "raw_only_logistic") {
        LOG_ERROR("[RawLux] unknown model_name"); return false;
    }
    const bool main_model = c.algorithm.model_name == "raw_hist_org_logistic";
    if (offline) {
        if (c.raw_left_dir.empty() || c.raw_right_dir.empty() || (main_model && c.metadata_sidecar_dir.empty())) {
            LOG_ERROR("[RawLux] offline requires both RAW directories; main model also requires metadata_sidecar_dir"); return false;
        }
    } else {
        if (c.raw_left_topic.empty() || c.raw_right_topic.empty() || c.raw_left_topic == c.raw_right_topic) {
            LOG_ERROR("[RawLux] online requires two distinct native RAW topics"); return false;
        }
        if (main_model && c.algorithm.metadata_source != "org_proxy") {
            LOG_ERROR("[RawLux] this online Image/PicData adapter only supplies org_proxy metadata; verified_raw requires an actual RAW telemetry source");
            return false;
        }
    }
    return true; // The SDK validates the exact model/metadata-source combination at init.
}

bool PercepMain::initRawLuxRuntime() {
    raw_lux_frame_ = RawLuxResult{};
    raw_lux_left_sub_.reset(); raw_lux_right_sub_.reset(); raw_lux_mailbox_.reset(); raw_lux_runner_.reset();
    if (!raw_lux_cfg_.enabled) return true;
    auto runner = std::make_unique<RawLuxRunner>();
    if (!runner->init(raw_lux_cfg_.algorithm)) {
        LOG_ERROR("[RawLux] init failed: %s", runner->lastError().c_str()); return false;
    }
    raw_lux_runner_ = std::move(runner);
    if (!offline) {
        raw_lux_mailbox_ = std::make_shared<RawLuxMailbox>();
        const auto mailbox = raw_lux_mailbox_;
        raw_lux_left_sub_.reset(new daye::RosSubscriber(
            daye::CreateSubscribe<sensor_msgs::msg::Image>(raw_lux_cfg_.raw_left_topic,
                rclcpp::SensorDataQoS().keep_last(2), [mailbox](const sensor_msgs::msg::Image& image) {
                    PercepMain::onRawLuxFrame(mailbox, 0, image);
                })));
        raw_lux_right_sub_.reset(new daye::RosSubscriber(
            daye::CreateSubscribe<sensor_msgs::msg::Image>(raw_lux_cfg_.raw_right_topic,
                rclcpp::SensorDataQoS().keep_last(2), [mailbox](const sensor_msgs::msg::Image& image) {
                    PercepMain::onRawLuxFrame(mailbox, 1, image);
                })));
    }
    LOG_INFO("[RawLux] enabled model=%s metadata_source=%s profile=BGGR_1088x1280_uint16LE_DN0_1023 fixed_roi=435,128,1033,1152 continuous_lux=false",
             raw_lux_cfg_.algorithm.model_name.c_str(), raw_lux_cfg_.algorithm.metadata_source.c_str());
    return true;
}

void PercepMain::onRawLuxFrame(const std::shared_ptr<RawLuxMailbox>& mailbox, std::size_t eye,
                              const sensor_msgs::msg::Image& image) {
    if (!mailbox || eye > 1) return;
    try {
        checkRawImage(image);
        {
            std::lock_guard<std::mutex> lock(mailbox->mutex);
            if (!mailbox->accepting) return;
        }
        auto owned = std::make_shared<sensor_msgs::msg::Image>(image);
        std::lock_guard<std::mutex> lock(mailbox->mutex);
        if (!mailbox->accepting) return;
        auto& frames = mailbox->frames[eye];
        const auto stamp = captureStamp(image);
        frames.erase(std::remove_if(frames.begin(), frames.end(), [stamp](const auto& prior) {
            return captureStamp(*prior) == stamp;
        }), frames.end());
        while (frames.size() >= 2) frames.pop_front();
        frames.push_back(std::move(owned)); mailbox->last_errors[eye].clear();
    } catch (const std::exception& error) {
        std::lock_guard<std::mutex> lock(mailbox->mutex);
        if (mailbox->accepting) mailbox->last_errors[eye] = error.what();
    }
}

void PercepMain::runRawLuxAssessment(const CameraData& frame) {
    raw_lux_frame_ = RawLuxResult{};
    if (!raw_lux_cfg_.enabled) return;
    const auto unknown = [&](const std::string& reason) {
        raw_lux_frame_ = RawLuxResult{}; raw_lux_frame_.reason = reason;
        LOG_INFO("[RawLux] class=UNKNOWN valid=false model=%s metadata_source=%s reason=%s",
                 raw_lux_cfg_.algorithm.model_name.c_str(), raw_lux_cfg_.algorithm.metadata_source.c_str(), reason.c_str());
    };
    try {
        if (!raw_lux_runner_) throw std::logic_error("RAW lux runner is not initialized");
        RawLuxImageView left, right;
        std::vector<std::uint8_t> left_file, right_file;
        std::array<std::shared_ptr<const sensor_msgs::msg::Image>, 2> messages;
        const bool main_model = raw_lux_cfg_.algorithm.model_name == "raw_hist_org_logistic";
        std::string group;
        if (offline) {
            raw_lux_io::readOfflinePair(raw_lux_cfg_.raw_left_dir, raw_lux_cfg_.raw_right_dir,
                raw_lux_cfg_.metadata_sidecar_dir, raw_lux_offline_stem_, main_model,
                raw_lux_cfg_.algorithm.metadata_source, &left, &right, &left_file, &right_file);
            group = raw_lux_offline_stem_; // replay association, not a hardware same-capture claim
        } else {
            const auto left_stamp = static_cast<std::uint64_t>(frame.left.time_ns);
            const auto right_stamp = static_cast<std::uint64_t>(frame.right.time_ns);
            if (!left_stamp || left_stamp != right_stamp)
                throw std::invalid_argument("selected left/right proxy frames need equal nonzero capture timestamps");
            if (!raw_lux_mailbox_) throw std::logic_error("RAW mailbox is not initialized");
            std::array<std::string,2> receive_errors;
            {
                std::lock_guard<std::mutex> lock(raw_lux_mailbox_->mutex);
                for (std::size_t eye = 0; eye < 2; ++eye) {
                    for (auto it = raw_lux_mailbox_->frames[eye].rbegin(); it != raw_lux_mailbox_->frames[eye].rend(); ++it) {
                        if (captureStamp(**it) == left_stamp) { messages[eye] = *it; break; }
                    }
                    receive_errors[eye] = raw_lux_mailbox_->last_errors[eye];
                }
            }
            if (!messages[0] || !messages[1])
                throw std::invalid_argument("missing left/right native RAW at the exact current capture timestamp; " + receive_errors[0] + "; " + receive_errors[1]);
            const auto fill = [](const sensor_msgs::msg::Image& message, RawLuxImageView* view) {
                view->data = message.data.data(); view->bytes = message.data.size();
                view->width = static_cast<int>(message.width); view->height = static_cast<int>(message.height);
                view->stride_bytes = message.step; view->timestamp_ns = captureStamp(message);
            };
            fill(*messages[0], &left); fill(*messages[1], &right);
            left.frame_id = static_cast<std::uint64_t>(frame.left.frame_id);
            right.frame_id = static_cast<std::uint64_t>(frame.right.frame_id);
            if (main_model) {
                // PicData is an Org/YUV proxy. Its exposure unit is ms; the trained model requires us.
                left.exposure_us = raw_lux_io::orgProxyExposureUs(frame.left.exp_time);
                right.exposure_us = raw_lux_io::orgProxyExposureUs(frame.right.exp_time);
                left.analog_gain = frame.left.analog_gain; right.analog_gain = frame.right.analog_gain;
            }
            group = "capture_" + std::to_string(left_stamp);
        }
        RawLuxResult result;
        if (!raw_lux_runner_->predictPair(group, left, right, &result)) {
            unknown(raw_lux_runner_->lastError()); return;
        }
        raw_lux_frame_ = std::move(result);
        const auto& r = raw_lux_frame_;
        LOG_INFO("[RawLux] class=%s valid=true model=%s metadata_source=%s source=%s group=%s p_not_low=%.9f p_low=%.9f p_very_low=%.9f roi_p10_dn=%.3f roi_p50_dn=%.3f left_frames=%zu right_frames=%zu outside_training=%zu single_sample_accuracy_unvalidated=%s probabilities_calibrated=false metadata_source_transfer_unvalidated=%s feature_ms=%.3f classification_ms=%.3f total_ms=%.3f",
                 r.predicted_class.c_str(), r.model_name.c_str(), r.metadata_source.c_str(),
                 offline ? "offline_stem_replay" : (main_model ? "same_capture_timestamp_org_proxy" : "same_capture_timestamp_raw_only"), r.group_id.c_str(),
                 r.probabilities[0], r.probabilities[1], r.probabilities[2],
                 r.features.size() > 0 ? r.features[0] : 0, r.features.size() > 1 ? r.features[1] : 0,
                 r.frame_counts[0], r.frame_counts[1], r.outside_training.size(),
                 r.single_sample_accuracy_unvalidated ? "true" : "false",
                 r.metadata_source_transfer_unvalidated ? "true" : "false",
                 r.timing.feature_ms, r.timing.classification_ms, r.timing.total_ms);
    } catch (const std::exception& error) { unknown(error.what()); }
}
} // namespace perception
