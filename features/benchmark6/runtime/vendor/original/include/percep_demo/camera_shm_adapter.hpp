#ifndef AI_NODE_PERCEP_DEMO_CAMERA_SHM_ADAPTER_HPP_
#define AI_NODE_PERCEP_DEMO_CAMERA_SHM_ADAPTER_HPP_

#include <cstddef>
#include <cstdint>

#include "percep_demo/pre_pointcloud_types.hpp"

namespace ai_node_percep_demo {

enum class FrameCopyStatus {
    kOk,
    kSdkUnavailable,
    kNullInput,
    kWrongMessageSize,
    kInvalidMetadata,
    kInvalidPayload,
    kAllocationFailure,
};

struct FrameCopyResult {
    FrameCopyStatus status{FrameCopyStatus::kInvalidMetadata};
    FrameValidationError validation_error{
        FrameValidationError::kInvalidDimensions};

    explicit operator bool() const noexcept {
        return status == FrameCopyStatus::kOk;
    }
};

struct CameraShmLayout {
    bool sdk_available{false};
    std::size_t rgbd_message_bytes{0};
    std::size_t gdc_message_bytes{0};
};

const char* frameCopyStatusName(FrameCopyStatus status) noexcept;
CameraShmLayout cameraShmLayout() noexcept;

// `data` is a borrowed SHM callback view. These functions must be called
// before the callback returns; successful output owns every byte it exposes.
FrameCopyResult copyRgbdFrameFromShm(
    const std::uint8_t* data, std::size_t size,
    std::uint64_t transport_sequence,
    std::int64_t transport_timestamp_ns,
    OwnedRgbdFrame* output) noexcept;

FrameCopyResult copyGdcFrameFromShm(
    const std::uint8_t* data, std::size_t size,
    std::uint64_t transport_sequence,
    std::int64_t transport_timestamp_ns,
    OwnedGdcFrame* output) noexcept;

}  // namespace ai_node_percep_demo

#endif  // AI_NODE_PERCEP_DEMO_CAMERA_SHM_ADAPTER_HPP_
