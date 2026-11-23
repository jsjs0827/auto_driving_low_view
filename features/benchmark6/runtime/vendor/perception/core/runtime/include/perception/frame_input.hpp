#pragma once
#include "perception_full_sdk.h"
#include "perception/types.hpp"
#include "perception/task_graph.hpp"
#include <optional>
#include <limits>
#include <stdexcept>

namespace perception::input {
// A synchronous input packet: descriptors are immutable, pixel buffers belong
// to the adapter/caller and MUST outlive Graph::runContext. No input key may be
// exported as a cross-frame artifact; History creates its own owned copies.
struct FrameBundle {
    psdk_frame left{};
    std::optional<psdk_frame> right;
    std::optional<psdk_disparity_q8> disparity;
    psdk_camera_intrinsics intrinsics{};

    static void validateImage(const psdk_frame& image, bool is_left) {
        if (image.struct_size != sizeof(image) || image.abi_version != PSDK_ABI_VERSION ||
            !image.data || !image.width || !image.height || image.width > 8192 || image.height > 8192)
            throw std::invalid_argument("Input image ABI, storage or dimensions are invalid");
        const bool yuv = image.format == PSDK_FORMAT_NV12 || image.format == PSDK_FORMAT_I420;
        const bool packed = image.format == PSDK_FORMAT_BGR || image.format == PSDK_FORMAT_RGB;
        if ((!yuv && !packed) || (is_left && !yuv))
            throw std::invalid_argument("Left input requires NV12/I420; right accepts YUV420/BGR/RGB");
        const std::uint64_t minimum_stride = std::uint64_t(image.width) * (yuv ? 1 : 3);
        if (image.stride < minimum_stride || image.stride > 16U*1024U*1024U ||
            (yuv && ((image.width | image.height | image.stride) & 1U)))
            throw std::invalid_argument("Input image stride or YUV420 geometry is invalid");
        const std::uint64_t base = std::uint64_t(image.stride) * image.height;
        const std::uint64_t needed = base + (yuv ? base/2 : 0);
        if (image.bytes < needed || image.bytes > std::numeric_limits<std::size_t>::max())
            throw std::invalid_argument("Input image buffer is shorter than its layout");
    }
    static FrameBundle borrowed(const psdk_full_frame& frame) {
        if (frame.abi_version != PSDK_FULL_ABI_VERSION ||
            (frame.struct_size != PSDK_FULL_FRAME_V1_SIZE && frame.struct_size != sizeof(frame)))
            throw std::invalid_argument("Invalid full input frame ABI/struct_size");
        FrameBundle value;
        validateImage(frame.left, true);
        value.left = frame.left;
        value.intrinsics = frame.intrinsics;
        if (frame.struct_size == sizeof(frame) && frame.right) {
            validateImage(*frame.right, false);
            if (frame.right->frame_id != frame.left.frame_id || frame.right->timestamp_ns != frame.left.timestamp_ns ||
                frame.right->width != frame.left.width || frame.right->height != frame.left.height)
                throw std::invalid_argument("Right input must have the same capture identity and dimensions as left");
            value.right = *frame.right;
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
        if (disparity) artifacts.put("input.disparity_q8", *disparity);
        artifacts.put("input.intrinsics", intrinsics);
        artifacts.put("input.frame", FrameInfo{left.frame_id, left.timestamp_ns});
    }
};
} // namespace perception::input
