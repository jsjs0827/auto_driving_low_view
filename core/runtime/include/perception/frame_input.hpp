#pragma once
#include "perception_full_sdk.h"
#include "perception/types.hpp"
#include "perception/task_graph.hpp"
#include <optional>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace perception::input {
// A synchronous input packet: descriptors are immutable, pixel buffers belong
// to the adapter/caller and MUST outlive Graph::runContext. No input key may be
// exported as a cross-frame artifact; History creates its own owned copies.
struct FrameBundle {
    psdk_frame left{};
    std::optional<psdk_frame> right;
    std::optional<psdk_frame> rear;
    std::optional<psdk_imu_input> imu;
    std::optional<psdk_wheel_input> wheel;
    std::optional<psdk_radar_input> radar;
    std::optional<psdk_raw_stereo_input> raw_light;
    std::optional<psdk_disparity_q8> disparity;
    psdk_camera_intrinsics intrinsics{};

    static void validateImage(const psdk_frame& image, bool is_left) {
        if (image.struct_size != sizeof(image) || image.abi_version != PSDK_ABI_VERSION ||
            !image.data || !image.width || !image.height || image.width > 8192 || image.height > 8192)
            throw std::invalid_argument("Input image ABI, storage or dimensions are invalid");
        const bool yuv = image.format == PSDK_FORMAT_NV12 || image.format == PSDK_FORMAT_I420;
        const bool packed = image.format == PSDK_FORMAT_BGR || image.format == PSDK_FORMAT_RGB;
        if ((!yuv && !packed) || (is_left && !yuv))
            throw std::invalid_argument("Left input requires NV12/I420; auxiliary images accept YUV420/BGR/RGB");
        const std::uint64_t minimum_stride = std::uint64_t(image.width) * (yuv ? 1 : 3);
        if (image.stride < minimum_stride || image.stride > 16U*1024U*1024U ||
            (yuv && ((image.width | image.height | image.stride) & 1U)))
            throw std::invalid_argument("Input image stride or YUV420 geometry is invalid");
        const std::uint64_t base = std::uint64_t(image.stride) * image.height;
        const std::uint64_t needed = base + (yuv ? base/2 : 0);
        if (image.bytes < needed || image.bytes > std::numeric_limits<std::size_t>::max())
            throw std::invalid_argument("Input image buffer is shorter than its layout");
    }
    template<class T> static void validateBatch(const T& input) {
        if (input.struct_size != sizeof(input) || input.abi_version != PSDK_SENSOR_INPUT_ABI_VERSION ||
            input.reserved || input.count > PSDK_SENSOR_MAX_SAMPLES || (input.count && !input.samples))
            throw std::invalid_argument("Sensor batch ABI, count or storage is invalid");
        std::uint64_t previous = 0;
        for (std::uint32_t i = 0; i < input.count; ++i) {
            const auto timestamp = input.samples[i].timestamp_ns;
            if (!timestamp || timestamp <= previous)
                throw std::invalid_argument("Sensor sample timestamps must be nonzero and strictly increasing");
            previous = timestamp;
        }
    }
    static void validateImu(const psdk_imu_input& input) {
        validateBatch(input);
        for (std::uint32_t i = 0; i < input.count; ++i)
            for (int axis = 0; axis < 3; ++axis)
                if (!std::isfinite(input.samples[i].linear_acceleration_mps2[axis]) ||
                    !std::isfinite(input.samples[i].angular_velocity_radps[axis]))
                    throw std::invalid_argument("IMU input contains a nonfinite value");
    }
    static void validateWheel(const psdk_wheel_input& input) {
        validateBatch(input);
        for (std::uint32_t i = 0; i < input.count; ++i) {
            const auto& sample = input.samples[i];
            if (sample.struct_size != sizeof(sample) || sample.abi_version != PSDK_SENSOR_INPUT_ABI_VERSION ||
                sample.reserved[0] || sample.reserved[1] || !std::isfinite(sample.linear_speed_mps) ||
                !std::isfinite(sample.yaw_rate_radps))
                throw std::invalid_argument("Wheel input sample ABI or value is invalid");
        }
    }
    static void validateRadar(const psdk_radar_input& input) {
        const auto known = PSDK_RADAR_HAS_RADIAL_VELOCITY | PSDK_RADAR_HAS_INTENSITY;
        if (input.struct_size != sizeof(input) || input.abi_version != PSDK_SENSOR_INPUT_ABI_VERSION ||
            input.reserved || (input.fields & ~known) || !input.timestamp_ns ||
            input.point_count > PSDK_RADAR_MAX_POINTS || (input.point_count && !input.points))
            throw std::invalid_argument("Radar input ABI, timestamp, count or storage is invalid");
        for (std::uint64_t i = 0; i < input.point_count; ++i) {
            const auto& point = input.points[i];
            if (!std::isfinite(point.x_m) || !std::isfinite(point.y_m) || !std::isfinite(point.z_m) ||
                ((input.fields & PSDK_RADAR_HAS_RADIAL_VELOCITY) && !std::isfinite(point.radial_velocity_mps)) ||
                ((input.fields & PSDK_RADAR_HAS_INTENSITY) && !std::isfinite(point.intensity)))
                throw std::invalid_argument("Radar input contains a nonfinite value");
        }
    }
    // This layer validates descriptor ABI and safe byte layout only. Missing
    // metadata/profile/synchronization are per-algorithm results, not a reason
    // to abort unrelated stereo/detection work for the whole DAG frame.
    static void validateRawImage(const psdk_raw_image& image) {
        if (!image.data && !image.bytes) return;
        if (image.struct_size != sizeof(image) || image.abi_version != PSDK_RAW_INPUT_ABI_VERSION ||
            !image.data || !image.width || !image.height || image.width > 8192 || image.height > 8192 ||
            image.format != PSDK_RAW_FORMAT_UINT16_LE || image.stride_bytes < std::uint64_t(image.width)*2 ||
            image.stride_bytes > 16U*1024U*1024U || (image.stride_bytes & 1U) ||
            image.bytes < std::uint64_t(image.stride_bytes)*image.height ||
            image.bytes > std::numeric_limits<std::size_t>::max())
            throw std::invalid_argument("RAW image ABI, storage or uint16 byte layout is invalid");
    }
    static void validateOrgDescriptor(const psdk_org_exposure& input) {
        if (!input.present) return;
        if (input.present != 1 || input.struct_size != sizeof(input) ||
            input.abi_version != PSDK_RAW_INPUT_ABI_VERSION || input.reserved)
            throw std::invalid_argument("ORG exposure descriptor ABI/presence is invalid");
        // Positive exposure/gain and capture matching are checked by raw_lux.
    }
    static void validateRawInput(const psdk_raw_stereo_input& input) {
        if (input.struct_size != sizeof(input) || input.abi_version != PSDK_RAW_INPUT_ABI_VERSION ||
            input.reserved || input.metadata_source > PSDK_RAW_METADATA_ORG_PROXY)
            throw std::invalid_argument("RAW stereo input ABI or metadata source is invalid");
        validateRawImage(input.left); validateRawImage(input.right);
        validateOrgDescriptor(input.left_org); validateOrgDescriptor(input.right_org);
    }
    static FrameBundle borrowed(const psdk_full_frame& frame) {
        if (frame.abi_version != PSDK_FULL_ABI_VERSION ||
            (frame.struct_size != PSDK_FULL_FRAME_V1_SIZE && frame.struct_size != PSDK_FULL_FRAME_V2_SIZE &&
             frame.struct_size != PSDK_FULL_FRAME_V3_SIZE && frame.struct_size != sizeof(frame)))
            throw std::invalid_argument("Invalid full input frame ABI/struct_size");
        FrameBundle value;
        validateImage(frame.left, true);
        value.left = frame.left;
        value.intrinsics = frame.intrinsics;
        if (frame.struct_size >= PSDK_FULL_FRAME_V2_SIZE && frame.right) {
            validateImage(*frame.right, false);
            if (frame.right->frame_id != frame.left.frame_id || frame.right->timestamp_ns != frame.left.timestamp_ns ||
                frame.right->width != frame.left.width || frame.right->height != frame.left.height)
                throw std::invalid_argument("Right input must have the same capture identity and dimensions as left");
            value.right = *frame.right;
        }
        if (frame.struct_size >= PSDK_FULL_FRAME_V3_SIZE) {
            if (frame.rear) {
                validateImage(*frame.rear, false);
                if (!frame.rear->timestamp_ns)
                    throw std::invalid_argument("Rear input requires a capture timestamp");
                value.rear = *frame.rear;
            }
            if (frame.imu) { validateImu(*frame.imu); value.imu = *frame.imu; }
            if (frame.wheel) { validateWheel(*frame.wheel); value.wheel = *frame.wheel; }
            if (frame.radar) { validateRadar(*frame.radar); value.radar = *frame.radar; }
        }
        if (frame.struct_size == sizeof(frame) && frame.raw_light) {
            validateRawInput(*frame.raw_light); value.raw_light = *frame.raw_light;
        }
        if (frame.disparity) {
            const auto& q = *frame.disparity;
            if (q.struct_size != sizeof(q) || q.abi_version != PSDK_FULL_ABI_VERSION || q.reserved || !q.data ||
                q.width != frame.left.width || q.height != frame.left.height || q.stride_bytes < std::uint64_t(q.width)*2 ||
                (q.stride_bytes & 1U) || q.bytes < std::uint64_t(q.stride_bytes)*q.height ||
                q.bytes > std::numeric_limits<std::size_t>::max() ||
                q.frame_id != frame.left.frame_id || q.timestamp_ns != frame.left.timestamp_ns)
                throw std::invalid_argument("Original Q8.8 requires valid storage and the exact left capture identity");
            value.disparity = q;
        }
        return value;
    }
    void seed(runtime::ArtifactStore& artifacts) const {
        artifacts.put("input.left", left);
        if (right) artifacts.put("input.right", *right);
        if (rear) artifacts.put("input.rear", *rear);
        if (imu) artifacts.put("input.imu", *imu);
        if (wheel) artifacts.put("input.wheel", *wheel);
        if (radar) artifacts.put("input.radar", *radar);
        if (raw_light) artifacts.put("input.raw_light", *raw_light);
        if (disparity) artifacts.put("input.disparity_q8", *disparity);
        artifacts.put("input.intrinsics", intrinsics);
        artifacts.put("input.frame", FrameInfo{left.frame_id, left.timestamp_ns});
    }
};
} // namespace perception::input
