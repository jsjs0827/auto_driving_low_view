#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace perception {
struct FrameInfo {
    std::uint64_t id = 0, timestamp_ns = 0; // Capture time, monotonic in one camera stream.
};
enum class PixelFormat { RGB, BGR, NV12, I420 };
enum class Layout { NCHW, NHWC };
enum class Task { Detection, Segmentation };
enum class Device { CPU, CUDA, BPU };
enum class BackendState { Ready, Running, Quarantined, Closed };
const char *stateName(BackendState) noexcept;
struct ImageView {
    const std::uint8_t *data = nullptr;
    std::size_t bytes = 0, stride = 0;
    int width = 0, height = 0;
    PixelFormat format = PixelFormat::RGB;
};
struct Geometry {
    int source_w, source_h, input_w, input_h, resized_w, resized_h, left, top;
};
struct TensorShape {
    int channels = 0, height = 0, width = 0;
    bool resolved() const noexcept { return channels > 0 && height > 0 && width > 0; }
    void validate(bool allow_dynamic = false) const;
    bool operator==(const TensorShape &b) const noexcept {
        return channels == b.channels && height == b.height && width == b.width;
    }
};
struct Tensor {
    int channels = 0, height = 0, width = 0;
    std::vector<float> data; // Owned, dequantized, contiguous CHW; no borrowed SDK memory.
    TensorShape shape() const noexcept { return {channels, height, width}; }
    void validate() const;
    float at(int c, int y, int x) const;
};
struct ModelInfo {
    int width = 0, height = 0;
    std::vector<TensorShape> outputs; // -1 dimensions are unresolved until the first run.
    std::vector<std::string> output_names;
    bool native_nv12 = false;
};
struct Detection {
    int class_id;
    float score, x1, y1, x2, y2;
};
struct Timing {
    double preprocess_ms = 0, inference_ms = 0, postprocess_ms = 0;
};
struct DetectionResult {
    std::vector<Detection> detections;
    int width = 0, height = 0;
    std::size_t rejected_anchors = 0;
    Timing timing;
};
struct SegmentationResult {
    std::vector<std::uint8_t> mask; // Original image size, raw IDs; zero is a valid class.
    int width = 0, height = 0;
    Timing timing;
};
} // namespace perception
