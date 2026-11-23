#ifndef PERCEPTION_EXAMPLE_IMAGE_INPUT_HPP_INCLUDED
#define PERCEPTION_EXAMPLE_IMAGE_INPUT_HPP_INCLUDED
// Example/CLI support only. This is not part of the public SDK ABI.
#include "perception/types.hpp"
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace example {
struct ImageInputOptions {
    std::string format = "image";
    int width = 0, height = 0;
    std::size_t stride = 0;
};
struct Image {
    std::vector<std::uint8_t> bytes;
    int width = 0, height = 0;
    std::size_t stride = 0;
    perception::PixelFormat format = perception::PixelFormat::RGB;
    std::string decoded_format;
    int source_channels = 0, source_bit_depth = 8;
    // File I/O, validation and decoding, performed once before inference timing.
    double decode_ms = 0;
    perception::ImageView view() const {
        return {bytes.data(), bytes.size(), stride, width, height, format};
    }
};
// image/auto sniffs P6/JPEG/PNG content. ppm accepts only strict 8-bit P6.
// Encoded images become RGB8; alpha is discarded, gray replicated, 16-bit PNG
// uses the high byte. EXIF orientation and color profiles are not applied.
Image loadImage(const std::string &path, const ImageInputOptions &options = {});
void writeInputPpm(const Image &image, const std::string &path);
} // namespace example
#endif
