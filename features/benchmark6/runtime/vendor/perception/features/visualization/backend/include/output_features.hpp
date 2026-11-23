#pragma once
#include "perception/runtime_engine.hpp"
#include "runtime_status.hpp"
#include <functional>
#include <string>
#include <vector>

namespace r28 {
struct OutputFile {
    std::string layer, path;
    int width = 0, height = 0;
};
struct OutputVisualization {
    perception::FrameInfo frame;
    bool skipped = false;
    std::uint64_t invocation = 0;
    double render_ms = 0, encode_write_ms = 0, output_ms = 0;
    std::vector<OutputFile> files;
    std::string metadata_path;
};

// Registers an output-only Feature. It reads the current graph's immutable
// input/result artifacts and writes output.visualization. Registration and
// skipped invocations do not create directories or files.
// Parameters: dir, every (Nth invocation), layers (comma-separated raw,
// detection,segmentation,overlay,disparity), mask_alpha, require_identity.
// Optional ai.detection.frame / ai.segmentation.frame artifacts are checked
// against input.frame. require_identity rejects a missing result identity.
void registerOutputFeatures(perception::runtime::FeatureRegistry&,
                            std::function<void(const Status&)> on_error = {});
}
