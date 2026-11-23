#pragma once

#include "perception/config.hpp"
#include <utility>
#include <vector>

namespace perception::detail {

// Non-owning view valid until the next call on the same workspace.  ModelSession
// serializes calls, so the backing storage can be retained and reused per model.
struct PreparedInputView {
    Geometry geometry{};
    const std::uint8_t *data = nullptr;
    std::size_t bytes = 0;
    bool nv12 = false;
    bool borrowed = false;
};

struct LinearSampleF {
    int first = 0, second = 0;
    float weight = 0, inverse_weight = 1;
};

struct LinearSampleD {
    int first = 0, second = 0;
    double weight = 0, inverse_weight = 1;
};

struct ResizeKey {
    int source_w = 0, source_h = 0, input_w = 0, input_h = 0;
    int resized_w = 0, resized_h = 0, left = 0, top = 0;
    int rounding_mode = 0;
    bool operator==(const ResizeKey &other) const noexcept {
        return source_w == other.source_w && source_h == other.source_h &&
               input_w == other.input_w && input_h == other.input_h &&
               resized_w == other.resized_w && resized_h == other.resized_h &&
               left == other.left && top == other.top &&
               rounding_mode == other.rounding_mode;
    }
};

class PreprocessWorkspace {
  public:
    PreparedInputView execute(const ImageView &, const PreprocessConfig &, bool native_nv12,
                              bool allow_borrowed);
    const std::vector<std::uint8_t> &ownedBytes() const noexcept { return output_; }
    std::vector<std::uint8_t> releaseBytes() { return std::move(output_); }

  private:
    PreparedInputView prepareRgb(const ImageView &, const PreprocessConfig &, bool allow_borrowed);
    PreparedInputView prepareNative(const ImageView &, const PreprocessConfig &,
                                    bool allow_borrowed);
    void ensureRgbMap(const Geometry &);
    void ensureNv12Map(const Geometry &);

    std::vector<std::uint8_t> converted_rgb_;
    std::vector<std::uint8_t> output_;
    ResizeKey output_key_{};
    bool output_key_valid_ = false;
    bool output_is_nv12_ = false;
    std::uint8_t output_pad_ = 0;

    ResizeKey rgb_key_{};
    bool rgb_key_valid_ = false;
    bool direct_rgb_rows_ = false;
    std::vector<LinearSampleF> rgb_columns_, rgb_rows_;

    ResizeKey nv12_key_{};
    bool nv12_key_valid_ = false;
    std::vector<LinearSampleD> luma_columns_, luma_rows_;
    std::vector<LinearSampleD> chroma_columns_, chroma_rows_;
};

// Exact r10 RGB->NV12 arithmetic, but writes into caller-owned storage.  The
// BPU adapter uses this to write RGB preprocessing output directly into its
// persistent input allocation instead of allocating and copying an NV12 vector.
void rgbToNv12Into(const std::uint8_t *rgb, std::size_t rgb_bytes, int width, int height,
                   std::uint8_t *target, std::size_t target_bytes);

} // namespace perception::detail
