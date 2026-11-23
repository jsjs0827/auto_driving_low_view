#pragma once
#include "percep_demo/horizon_dnn_runner.hpp"
#include "runtime_status.hpp"
#include <memory>
#include <limits>

namespace r28::product {
struct ExactLaneSelection {
    bool detection = false, segmentation = false;
    const char* profile = nullptr;
};
// Original inferNv12 always runs enabled detection, and runs enabled
// segmentation only when the caller requests its mask.
inline ExactLaneSelection selectExactLanes(const ai_node_percep_demo::DnnConfig& config,
                                           bool request_mask) {
    ExactLaneSelection result{config.enabled && config.detect.enabled,
                              config.enabled && config.seg.enabled && request_mask};
    if (!result.detection && !result.segmentation)
        throw std::invalid_argument("exact product inference has no enabled requested lane");
    result.profile = result.detection && result.segmentation
        ? (config.parallel_det_seg ? "dag_exact.json" : "dag_exact_serial.json")
        : (result.detection ? "dag_exact_det_only.json" : "dag_exact_seg_only.json");
    return result;
}
inline void validateProductYuv(const std::uint8_t* data, int width, int height,
                               int stride, std::size_t bytes,
                               perception::FrameInfo frame = {}) {
    if (!data || width <= 0 || height <= 0 || width % 2 || height % 2 ||
        stride < width || stride % 2 ||
        std::size_t(stride) > std::numeric_limits<std::size_t>::max() /
                                   (std::size_t(height) + std::size_t(height) / 2) ||
        bytes < std::size_t(stride) * (std::size_t(height) + std::size_t(height) / 2)) {
        throw r28::Failure({1002, 0, "product.input.image", "dnn",
                            "NV12/I420 source has invalid data, geometry, stride or byte count", frame});
    }
}
// SHM PicData carries total YUV bytes rather than a stride field. Derive the
// original stride only from a complete, even YUV420 plane layout.
inline int productYuvStride(const std::uint8_t* data, int width, int height,
                            std::size_t bytes, std::size_t capacity,
                            perception::FrameInfo frame = {}) {
    if (height <= 0 || height % 2 || !bytes || bytes > capacity)
        throw r28::Failure({1002, 0, "product.input.image", "dnn",
                            "product YUV image geometry or byte count is invalid", frame});
    const auto rows = std::size_t(height) + std::size_t(height) / 2;
    if (bytes % rows || bytes / rows > std::size_t(std::numeric_limits<int>::max()))
        throw r28::Failure({1002, 0, "product.input.image", "dnn",
                            "product YUV image has incomplete planes or invalid stride", frame});
    const int stride = static_cast<int>(bytes / rows);
    validateProductYuv(data, width, height, stride, bytes, frame);
    return stride;
}
// Product-compatible inference boundary. Enabled original models load once;
// each invocation selects the original requested DAG lane set.
class ExactDnnRunner {
    struct Impl;
    std::unique_ptr<Impl> impl_;
public:
    explicit ExactDnnRunner(std::string profiles);
    ~ExactDnnRunner();
    bool init(const ai_node_percep_demo::DnnConfig&);
    bool inferNv12(const std::uint8_t*,int,int,int,std::size_t,bool,
        std::vector<ai_node_percep_demo::DnnDetection>*,std::vector<std::uint8_t>*,
        std::vector<std::uint8_t>*,ai_node_percep_demo::DnnInferTiming* = nullptr);
    const ai_node_percep_demo::DnnConfig& config() const;
    const r28::Status& lastStatus() const;
    void setFrameInfo(perception::FrameInfo);
};
}
