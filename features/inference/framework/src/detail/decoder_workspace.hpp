#pragma once

#include "perception/model_plan.hpp"
#include <vector>

namespace perception::detail {

struct NativeTensorView;

struct DetectionDecoderWorkspace {
    std::vector<Detection> boxes;
    std::vector<Detection> sort_buffer;
    std::vector<std::vector<std::size_t>> groups;
    std::vector<std::uint8_t> chosen;
};

struct SegmentationAxisSample {
    int first = 0, second = 0;
    double weight = 0;
};

struct SegmentationAxisKey {
    int source_w = 0, source_h = 0, resized_w = 0, resized_h = 0;
    int left = 0, top = 0, input_w = 0, input_h = 0;
    int logits_w = 0, logits_h = 0;
    bool align_corners = false, bilinear = false;
    int rounding_mode = 0;
    bool operator==(const SegmentationAxisKey &other) const noexcept {
        return source_w == other.source_w && source_h == other.source_h &&
               resized_w == other.resized_w && resized_h == other.resized_h &&
               left == other.left && top == other.top && input_w == other.input_w &&
               input_h == other.input_h && logits_w == other.logits_w &&
               logits_h == other.logits_h && align_corners == other.align_corners &&
               bilinear == other.bilinear && rounding_mode == other.rounding_mode;
    }
};

struct PointSummary {
    double margin = 0, max_abs = 0;
    int winner = -1;
};

struct SegmentationDecoderWorkspace {
    SegmentationAxisKey axis_key{};
    bool axis_key_valid = false;
    std::vector<SegmentationAxisSample> columns, rows;
    std::vector<PointSummary> upper_points, lower_points;
    std::vector<int> cell_winner;
};

struct DecoderWorkspace {
    DetectionDecoderWorkspace detection;
    SegmentationDecoderWorkspace segmentation;
};

// These entry points consume a batch already validated once at the pipeline
// boundary. Public decoders retain their complete validation contract.
void decodeDetectionValidated(const std::vector<Tensor> &, const ModelPlan &, const Geometry &,
                              DetectionDecoderWorkspace &, std::vector<Detection> &,
                              std::size_t &rejected);
void decodeDetectionNativeValidated(const std::vector<NativeTensorView> &, const ModelPlan &,
                                     const Geometry &, DetectionDecoderWorkspace &,
                                     std::vector<Detection> &, std::size_t &rejected);
void decodeSegmentationValidated(const std::vector<Tensor> &, const ModelPlan &, const Geometry &,
                                 SegmentationDecoderWorkspace &, std::vector<std::uint8_t> &);

} // namespace perception::detail
