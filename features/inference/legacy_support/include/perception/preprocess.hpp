#pragma once
#include "perception/config.hpp"
namespace perception {
struct PreparedImage {
    Geometry geometry;
    std::vector<std::uint8_t> rgb;
};
struct PreparedNv12 {
    Geometry geometry;
    std::vector<std::uint8_t> nv12;
};
PreparedImage prepare(const ImageView &, int width, int height, bool letterbox, std::uint8_t pad);
PreparedNv12 prepareNv12(const ImageView &, int width, int height, bool letterbox,
                         std::uint8_t pad);
std::vector<std::uint8_t> rgbToNv12(const std::vector<std::uint8_t> &, int width, int height);
std::vector<float> normalize(const std::vector<std::uint8_t> &, const PreprocessConfig &);
struct PreparedInput {
    Geometry geometry;
    std::vector<std::uint8_t> bytes;
    bool nv12 = false;
};
// Stores immutable per-model choices; image dimensions/strides are validated every frame.
class PreprocessPlan {
  public:
    PreprocessPlan(PreprocessConfig config, bool native_nv12);
    PreparedInput execute(const ImageView &) const;

  private:
    PreprocessConfig config_;
    bool native_nv12_;
};
} // namespace perception
