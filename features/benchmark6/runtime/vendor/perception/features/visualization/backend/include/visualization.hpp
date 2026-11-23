#pragma once
// Caller/example support only: no symbols are added to the shared SDK ABI.
#include "image_input.hpp"
#include <filesystem>

namespace demo_vis {
struct Style {
    float mask_alpha = .4f;
    std::vector<std::string> detection_names, segmentation_names;
};

// Returns owned, tightly packed RGB at the input image dimensions (up to 4096).
// The input and results are never modified. Results must already be in source
// image coordinates; a size mismatch is rejected rather than silently resized.
// Each non-null result contributes an overlay. Raw class 0 is also colored.
// The embedded ASCII font displays lowercase names in uppercase; unsupported
// characters are displayed as '?', while the numeric class ID is always shown.
std::vector<std::uint8_t> renderRgb(
    const example::Image&, const perception::DetectionResult*,
    const perception::SegmentationResult*, const Style& = {});

// Output-only conversion; no resize or write into model buffers. Caller-owned
// RGB storage is reused across frames. overlayRgb mutates only this RGB copy.
void convertRgbInto(const perception::ImageView&, std::vector<std::uint8_t>& rgb);
void validateOverlay(int width, int height,
                     const perception::DetectionResult*, const perception::SegmentationResult*,
                     const Style& = {});
void overlayRgb(std::vector<std::uint8_t>& rgb, int width, int height,
                const perception::DetectionResult*, const perception::SegmentationResult*,
                const Style& = {});
void saveRgbPng(const std::filesystem::path&, const std::vector<std::uint8_t>& rgb,
                int width, int height);

// Encodes RGB PNG independently of the filename extension. Parent directories
// must exist. Creates the destination exclusively: existing files/symlinks are
// never overwritten. Rendering, encoding, short-write and close errors throw.
// A failed write can leave a partial NEW file for diagnostics; it is never
// treated as success. Distinct output paths may be saved concurrently.
void savePng(const std::filesystem::path&, const example::Image&,
             const perception::DetectionResult*, const perception::SegmentationResult*,
             const Style& = {});
} // namespace demo_vis
