#ifndef AI_NODE_PERCEP_DEMO_PRE_POINTCLOUD_TYPES_HPP_
#define AI_NODE_PERCEP_DEMO_PRE_POINTCLOUD_TYPES_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace ai_node_percep_demo {

inline constexpr int kMaxCameraFrameDimension = 16384;
inline constexpr std::size_t kMaxCameraPayloadBytes =
    128U * 1024U * 1024U;

struct FrameKey {
    std::uint64_t transport_sequence{0};
    std::int64_t transport_timestamp_ns{0};
    std::uint64_t camera_frame_id{0};
    std::int64_t camera_timestamp_ns{0};
};

inline bool operator==(const FrameKey& lhs, const FrameKey& rhs) noexcept {
    return lhs.transport_sequence == rhs.transport_sequence &&
           lhs.transport_timestamp_ns == rhs.transport_timestamp_ns &&
           lhs.camera_frame_id == rhs.camera_frame_id &&
           lhs.camera_timestamp_ns == rhs.camera_timestamp_ns;
}

inline bool operator!=(const FrameKey& lhs, const FrameKey& rhs) noexcept {
    return !(lhs == rhs);
}

// A result may be associated with a frame only when the producer supplied at
// least one usable identity field. Treating two all-zero keys as "the same
// frame" would turn an unknown identity into a false proof of synchronization.
inline bool frameKeyHasIdentity(const FrameKey& key) noexcept {
    return key.transport_sequence != 0U || key.transport_timestamp_ns > 0 ||
           key.camera_frame_id != 0U || key.camera_timestamp_ns > 0;
}

inline bool sameFrame(const FrameKey& lhs, const FrameKey& rhs) noexcept {
    return frameKeyHasIdentity(lhs) && frameKeyHasIdentity(rhs) && lhs == rhs;
}

// `call_index` is zero based so a cadence always admits the first available
// frame. Keeping this decision independent of mutable runtime state makes the
// sampling rule easy to test and lets map/recharge maintain separate counters.
inline bool periodicFrameDue(std::uint64_t call_index,
                             int every_n) noexcept {
    const std::uint64_t period =
        every_n > 1 ? static_cast<std::uint64_t>(every_n) : 1U;
    return (call_index % period) == 0U;
}

// RGB and disparity are expected to come from the same SHM envelope. Their
// transport identity must therefore match exactly. A camera frame id of zero
// means "not supplied"; two non-zero camera ids must match.
inline bool rgbdFrameKeysCompatible(const FrameKey& left,
                                    const FrameKey& disparity) noexcept {
    if (left.transport_sequence != disparity.transport_sequence ||
        left.transport_timestamp_ns !=
            disparity.transport_timestamp_ns) {
        return false;
    }
    return left.camera_frame_id == 0 || disparity.camera_frame_id == 0 ||
           left.camera_frame_id == disparity.camera_frame_id;
}

enum class FrameValidationError {
    kNone,
    kNullData,
    kInvalidDimensions,
    kOddDimensions,
    kPayloadEmpty,
    kPayloadTooSmall,
    kPayloadTooLarge,
    kSizeOverflow,
    kStrideNotIntegral,
    kStrideTooSmall,
    kOddStride,
    kElementSizeMismatch,
    kDimensionMismatch,
    kFrameKeyMismatch,
    kInvalidIntrinsics,
    kAllocationFailure,
};

inline const char* frameValidationErrorName(
    FrameValidationError error) noexcept {
    switch (error) {
        case FrameValidationError::kNone:
            return "none";
        case FrameValidationError::kNullData:
            return "null_data";
        case FrameValidationError::kInvalidDimensions:
            return "invalid_dimensions";
        case FrameValidationError::kOddDimensions:
            return "odd_dimensions";
        case FrameValidationError::kPayloadEmpty:
            return "payload_empty";
        case FrameValidationError::kPayloadTooSmall:
            return "payload_too_small";
        case FrameValidationError::kPayloadTooLarge:
            return "payload_too_large";
        case FrameValidationError::kSizeOverflow:
            return "size_overflow";
        case FrameValidationError::kStrideNotIntegral:
            return "stride_not_integral";
        case FrameValidationError::kStrideTooSmall:
            return "stride_too_small";
        case FrameValidationError::kOddStride:
            return "odd_stride";
        case FrameValidationError::kElementSizeMismatch:
            return "element_size_mismatch";
        case FrameValidationError::kDimensionMismatch:
            return "dimension_mismatch";
        case FrameValidationError::kFrameKeyMismatch:
            return "frame_key_mismatch";
        case FrameValidationError::kInvalidIntrinsics:
            return "invalid_intrinsics";
        case FrameValidationError::kAllocationFailure:
            return "allocation_failure";
    }
    return "unknown";
}

inline bool checkedMultiply(std::size_t lhs, std::size_t rhs,
                            std::size_t* result) noexcept {
    if (result == nullptr) {
        return false;
    }
    if (lhs != 0 && rhs > std::numeric_limits<std::size_t>::max() / lhs) {
        return false;
    }
    *result = lhs * rhs;
    return true;
}

struct Nv12Layout {
    std::size_t stride_y{0};
    std::size_t payload_bytes{0};
};

struct Nv12Validation {
    FrameValidationError error{FrameValidationError::kInvalidDimensions};
    Nv12Layout layout;

    explicit operator bool() const noexcept {
        return error == FrameValidationError::kNone;
    }
};

inline Nv12Validation validateNv12Buffer(const void* data, int width,
                                         int height,
                                         std::size_t bytes_used) noexcept {
    if (width <= 0 || height <= 0 || width > kMaxCameraFrameDimension ||
        height > kMaxCameraFrameDimension) {
        return {FrameValidationError::kInvalidDimensions, {}};
    }
    // NV12 is YUV 4:2:0. Each chroma sample covers a 2x2 luma block.
    if ((width % 2) != 0 || (height % 2) != 0) {
        return {FrameValidationError::kOddDimensions, {}};
    }
    if (data == nullptr) {
        return {FrameValidationError::kNullData, {}};
    }
    if (bytes_used == 0) {
        return {FrameValidationError::kPayloadEmpty, {}};
    }
    if (bytes_used > kMaxCameraPayloadBytes) {
        return {FrameValidationError::kPayloadTooLarge, {}};
    }

    const auto rows = static_cast<std::size_t>(height) +
                      static_cast<std::size_t>(height) / 2U;
    std::size_t tight_bytes = 0;
    if (!checkedMultiply(static_cast<std::size_t>(width), rows,
                         &tight_bytes)) {
        return {FrameValidationError::kSizeOverflow, {}};
    }
    if (bytes_used < tight_bytes) {
        return {FrameValidationError::kPayloadTooSmall, {}};
    }
    // Without an explicit SDK stride field, bytes_used must describe an
    // integral number of equally-strided Y and UV rows. Guessing here could
    // make the decoder read the UV plane at the wrong address.
    if ((bytes_used % rows) != 0U) {
        return {FrameValidationError::kStrideNotIntegral, {}};
    }

    const std::size_t stride_y = bytes_used / rows;
    if (stride_y < static_cast<std::size_t>(width)) {
        return {FrameValidationError::kStrideTooSmall, {}};
    }
    if ((stride_y % 2U) != 0U) {
        return {FrameValidationError::kOddStride, {}};
    }
    return {FrameValidationError::kNone, {stride_y, bytes_used}};
}

struct DisparityLayout {
    std::size_t stride_samples{0};
    std::size_t sample_count{0};
};

struct DisparityValidation {
    FrameValidationError error{FrameValidationError::kInvalidDimensions};
    DisparityLayout layout;

    explicit operator bool() const noexcept {
        return error == FrameValidationError::kNone;
    }
};

inline DisparityValidation validateDisparityBuffer(
    const void* data, int width, int height,
    std::size_t bytes_used) noexcept {
    if (width <= 0 || height <= 0 || width > kMaxCameraFrameDimension ||
        height > kMaxCameraFrameDimension) {
        return {FrameValidationError::kInvalidDimensions, {}};
    }
    if (data == nullptr) {
        return {FrameValidationError::kNullData, {}};
    }
    if (bytes_used == 0) {
        return {FrameValidationError::kPayloadEmpty, {}};
    }
    if (bytes_used > kMaxCameraPayloadBytes) {
        return {FrameValidationError::kPayloadTooLarge, {}};
    }
    if ((bytes_used % sizeof(std::uint16_t)) != 0U) {
        return {FrameValidationError::kElementSizeMismatch, {}};
    }

    const std::size_t sample_count = bytes_used / sizeof(std::uint16_t);
    std::size_t tight_samples = 0;
    if (!checkedMultiply(static_cast<std::size_t>(width),
                         static_cast<std::size_t>(height),
                         &tight_samples)) {
        return {FrameValidationError::kSizeOverflow, {}};
    }
    if (sample_count < tight_samples) {
        return {FrameValidationError::kPayloadTooSmall, {}};
    }
    if ((sample_count % static_cast<std::size_t>(height)) != 0U) {
        return {FrameValidationError::kStrideNotIntegral, {}};
    }

    const std::size_t stride_samples =
        sample_count / static_cast<std::size_t>(height);
    if (stride_samples < static_cast<std::size_t>(width)) {
        return {FrameValidationError::kStrideTooSmall, {}};
    }
    return {FrameValidationError::kNone,
            {stride_samples, sample_count}};
}

struct BorrowedNv12Image {
    FrameKey key;
    int width{0};
    int height{0};
    std::size_t bytes_used{0};
    const std::uint8_t* data{nullptr};
};

struct BorrowedDisparityImage {
    FrameKey key;
    int width{0};
    int height{0};
    std::size_t bytes_used{0};
    const void* data{nullptr};
};

enum class Yuv420Format {
    kUnknown,
    kNv12,
    kI420,
};

struct Nv12Image {
    FrameKey key;
    int width{0};
    int height{0};
    int stride_y{0};
    // Kept in this legacy-named type so every consumer interprets the same
    // owning bytes consistently. Camera adapters initially leave it unknown;
    // the source-aware runtime resolves it before publishing the frame.
    Yuv420Format format{Yuv420Format::kUnknown};
    std::vector<std::uint8_t> bytes;

    bool valid() const noexcept {
        const auto validation = validateNv12Buffer(
            bytes.data(), width, height, bytes.size());
        return validation &&
               validation.layout.stride_y ==
                   static_cast<std::size_t>(stride_y);
    }
};

// Normalize a validated I420 image to tightly packed NV12 for APIs whose ABI
// explicitly accepts only NV12 (for example recharge_mark). The destination is
// committed only after the complete conversion succeeds.
inline bool convertI420ToNv12(const Nv12Image& source,
                              Nv12Image* destination) noexcept {
    if (destination == nullptr || !source.valid() ||
        source.format != Yuv420Format::kI420) {
        return false;
    }
    try {
        Nv12Image converted;
        converted.key = source.key;
        converted.width = source.width;
        converted.height = source.height;
        converted.stride_y = source.width;
        converted.format = Yuv420Format::kNv12;

        const std::size_t y_bytes =
            static_cast<std::size_t>(source.width) * source.height;
        converted.bytes.resize(y_bytes + y_bytes / 2U);

        for (int y = 0; y < source.height; ++y) {
            std::memcpy(converted.bytes.data() +
                            static_cast<std::size_t>(y) * source.width,
                        source.bytes.data() +
                            static_cast<std::size_t>(y) * source.stride_y,
                        static_cast<std::size_t>(source.width));
        }

        const std::size_t source_y_bytes =
            static_cast<std::size_t>(source.stride_y) * source.height;
        const int source_uv_stride = source.stride_y / 2;
        const std::uint8_t* source_u = source.bytes.data() + source_y_bytes;
        const std::uint8_t* source_v =
            source_u + static_cast<std::size_t>(source_uv_stride) *
                           (source.height / 2);
        std::uint8_t* destination_uv = converted.bytes.data() + y_bytes;
        for (int y = 0; y < source.height / 2; ++y) {
            for (int x = 0; x < source.width / 2; ++x) {
                const std::size_t source_offset =
                    static_cast<std::size_t>(y) * source_uv_stride + x;
                const std::size_t destination_offset =
                    static_cast<std::size_t>(y) * source.width + x * 2U;
                destination_uv[destination_offset] = source_u[source_offset];
                destination_uv[destination_offset + 1U] =
                    source_v[source_offset];
            }
        }
        *destination = std::move(converted);
    } catch (...) {
        return false;
    }
    return true;
}

struct DisparityImage {
    FrameKey key;
    int width{0};
    int height{0};
    int stride_samples{0};
    std::vector<std::uint16_t> samples;

    bool valid() const noexcept {
        std::size_t bytes_used = 0;
        if (!checkedMultiply(samples.size(), sizeof(std::uint16_t),
                             &bytes_used)) {
            return false;
        }
        const auto validation = validateDisparityBuffer(
            samples.data(), width, height, bytes_used);
        return validation &&
               validation.layout.stride_samples ==
                   static_cast<std::size_t>(stride_samples);
    }
};

inline FrameValidationError copyNv12Image(const BorrowedNv12Image& source,
                                          Nv12Image* output) noexcept {
    if (output == nullptr) {
        return FrameValidationError::kNullData;
    }
    const auto validation = validateNv12Buffer(
        source.data, source.width, source.height, source.bytes_used);
    if (!validation) {
        return validation.error;
    }

    try {
        Nv12Image copy;
        copy.key = source.key;
        copy.width = source.width;
        copy.height = source.height;
        copy.stride_y = static_cast<int>(validation.layout.stride_y);
        copy.bytes.assign(source.data, source.data + source.bytes_used);
        *output = std::move(copy);
    } catch (...) {
        return FrameValidationError::kAllocationFailure;
    }
    return FrameValidationError::kNone;
}

inline FrameValidationError copyDisparityImage(
    const BorrowedDisparityImage& source,
    DisparityImage* output) noexcept {
    if (output == nullptr) {
        return FrameValidationError::kNullData;
    }
    const auto validation = validateDisparityBuffer(
        source.data, source.width, source.height, source.bytes_used);
    if (!validation) {
        return validation.error;
    }

    try {
        DisparityImage copy;
        copy.key = source.key;
        copy.width = source.width;
        copy.height = source.height;
        copy.stride_samples =
            static_cast<int>(validation.layout.stride_samples);
        copy.samples.resize(validation.layout.sample_count);
        std::memcpy(copy.samples.data(), source.data, source.bytes_used);
        *output = std::move(copy);
    } catch (...) {
        return FrameValidationError::kAllocationFailure;
    }
    return FrameValidationError::kNone;
}

struct OwnedRgbdFrame {
    Nv12Image left;
    DisparityImage disparity;

    bool valid() const noexcept {
        if (!left.valid() || !disparity.valid()) {
            return false;
        }
        if (left.width != disparity.width ||
            left.height != disparity.height) {
            return false;
        }
        return rgbdFrameKeysCompatible(left.key, disparity.key);
    }
};

struct OwnedGdcFrame {
    Nv12Image left;

    bool valid() const noexcept {
        return left.valid();
    }
};

using RgbdFramePtr = std::shared_ptr<const OwnedRgbdFrame>;
using GdcFramePtr = std::shared_ptr<const OwnedGdcFrame>;

inline FrameValidationError copyRgbdFrame(
    const BorrowedNv12Image& left,
    const BorrowedDisparityImage& disparity,
    OwnedRgbdFrame* output) noexcept {
    if (output == nullptr) {
        return FrameValidationError::kNullData;
    }

    // Validate both borrowed views before allocating/copying either
    // potentially large plane. The copy helpers intentionally validate again
    // at their own public boundary.
    const auto left_validation = validateNv12Buffer(
        left.data, left.width, left.height, left.bytes_used);
    if (!left_validation) {
        return left_validation.error;
    }
    const auto disparity_validation = validateDisparityBuffer(
        disparity.data, disparity.width, disparity.height,
        disparity.bytes_used);
    if (!disparity_validation) {
        return disparity_validation.error;
    }
    if (left.width != disparity.width || left.height != disparity.height) {
        return FrameValidationError::kDimensionMismatch;
    }
    if (!rgbdFrameKeysCompatible(left.key, disparity.key)) {
        return FrameValidationError::kFrameKeyMismatch;
    }

    OwnedRgbdFrame copy;
    auto error = copyNv12Image(left, &copy.left);
    if (error != FrameValidationError::kNone) {
        return error;
    }
    error = copyDisparityImage(disparity, &copy.disparity);
    if (error != FrameValidationError::kNone) {
        return error;
    }
    if (copy.left.width != copy.disparity.width ||
        copy.left.height != copy.disparity.height) {
        return FrameValidationError::kDimensionMismatch;
    }
    if (!rgbdFrameKeysCompatible(copy.left.key, copy.disparity.key)) {
        return FrameValidationError::kFrameKeyMismatch;
    }
    *output = std::move(copy);
    return FrameValidationError::kNone;
}

struct CameraIntrinsics {
    int calibration_width{0};
    int calibration_height{0};
    float fx{0.0F};
    float fy{0.0F};
    float cx{0.0F};
    float cy{0.0F};
    // The legacy CameraParam expresses bxf in pixel-millimetres. This member
    // stores it in pixel-metres, matching Z[m] = bxf_m / disparity[pixel].
    float bxf_m{0.0F};
    float baseline_m{0.0F};

    bool pinholeValid() const noexcept {
        if (calibration_width <= 0 || calibration_height <= 0 ||
            !std::isfinite(fx) || !std::isfinite(fy) || fx <= 0.0F ||
            fy <= 0.0F || !std::isfinite(cx) || !std::isfinite(cy)) {
            return false;
        }
        return cx >= 0.0F && cy >= 0.0F &&
               cx < static_cast<float>(calibration_width) &&
               cy < static_cast<float>(calibration_height);
    }

    bool depthValid() const noexcept {
        return pinholeValid() && std::isfinite(bxf_m) && bxf_m > 0.0F;
    }

    std::optional<CameraIntrinsics> scaledTo(int width,
                                             int height) const noexcept {
        if (!pinholeValid() || width <= 0 || height <= 0 ||
            width > kMaxCameraFrameDimension ||
            height > kMaxCameraFrameDimension) {
            return std::nullopt;
        }
        const float sx = static_cast<float>(width) /
                         static_cast<float>(calibration_width);
        const float sy = static_cast<float>(height) /
                         static_cast<float>(calibration_height);
        CameraIntrinsics scaled = *this;
        scaled.calibration_width = width;
        scaled.calibration_height = height;
        scaled.fx *= sx;
        scaled.fy *= sy;
        scaled.cx *= sx;
        scaled.cy *= sy;
        // bxf contains fx, so it must follow the horizontal scale too.
        scaled.bxf_m *= sx;
        // Even finite source values can overflow after scaling. Do not return
        // an optional containing an object that already violates its own
        // value contract.
        if (!scaled.pinholeValid() || !std::isfinite(scaled.bxf_m)) {
            return std::nullopt;
        }
        return scaled;
    }
};

struct PrePointCloudInput {
    RgbdFramePtr frame;
    CameraIntrinsics intrinsics;

    bool valid() const noexcept {
        return frame && frame->valid() && intrinsics.depthValid() &&
               intrinsics.calibration_width == frame->disparity.width &&
               intrinsics.calibration_height == frame->disparity.height;
    }
};

enum class WorkMode {
    kIdle,
    kMap,
    kRecharge,
    kMapAndRecharge,
};

struct FrameDemandOptions {
    bool recharge_prefers_gdc{true};
    bool save_gdc_for_map{false};
};

struct FrameDemand {
    bool run_map{false};
    bool run_recharge{false};
    bool run_segmentation{false};
    bool wait_rgbd{false};
    bool wait_gdc{false};
    bool try_rgbd{false};
    bool try_gdc{false};
};

constexpr FrameDemand demandFor(
    WorkMode mode, FrameDemandOptions options = {}) noexcept {
    const bool run_map =
        mode == WorkMode::kMap || mode == WorkMode::kMapAndRecharge;
    const bool run_recharge =
        mode == WorkMode::kRecharge || mode == WorkMode::kMapAndRecharge;
    const bool recharge_needs_rgbd =
        run_recharge && !options.recharge_prefers_gdc;
    const bool recharge_needs_gdc =
        run_recharge && options.recharge_prefers_gdc;

    FrameDemand demand;
    demand.run_map = run_map;
    demand.run_recharge = run_recharge;
    demand.run_segmentation = run_map;
    demand.wait_rgbd = run_map || recharge_needs_rgbd;
    // Saving a debug GDC image is best-effort; it must not turn an optional
    // diagnostic into a blocking dependency of the map pipeline.
    demand.wait_gdc = recharge_needs_gdc;
    // The non-primary recharge source is still consumed opportunistically so
    // a configured fallback can continue when the preferred source is absent.
    demand.try_rgbd = demand.wait_rgbd || run_recharge;
    demand.try_gdc = demand.wait_gdc || run_recharge ||
                     (run_map && options.save_gdc_for_map);
    return demand;
}

enum class RechargeFrameSource {
    kNone,
    kRgbd,
    kGdc,
};

constexpr RechargeFrameSource selectRechargeFrameSource(
    bool prefer_gdc, bool allow_fallback, bool has_rgbd,
    bool has_gdc) noexcept {
    if (prefer_gdc) {
        if (has_gdc) {
            return RechargeFrameSource::kGdc;
        }
        return allow_fallback && has_rgbd ? RechargeFrameSource::kRgbd
                                          : RechargeFrameSource::kNone;
    }
    if (has_rgbd) {
        return RechargeFrameSource::kRgbd;
    }
    return allow_fallback && has_gdc ? RechargeFrameSource::kGdc
                                     : RechargeFrameSource::kNone;
}

}  // namespace ai_node_percep_demo

#endif  // AI_NODE_PERCEP_DEMO_PRE_POINTCLOUD_TYPES_HPP_
