#include "image_input.hpp"
#include "perception/error.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>

#ifdef PERCEPTION_IMAGE_CODECS
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_ONLY_PNG
#define STBI_NO_STDIO
#define STBI_MAX_DIMENSIONS 8192
// Keep third-party symbols local to this CLI support translation unit.
#define STB_IMAGE_STATIC
#include "stb_image.h"
#endif

namespace example {
namespace {
constexpr std::size_t max_bytes = 256U * 1024U * 1024U;
void dimensions(int width, int height, std::size_t bytes_per_pixel = 3) {
    if (width <= 0 || height <= 0 || width > 8192 || height > 8192 ||
        std::size_t(width) * std::size_t(height) > max_bytes / bytes_per_pixel)
        throw std::runtime_error("invalid image dimensions or decoded image exceeds 256 MiB");
}
std::string token(std::istream &stream) {
    std::string out;
    for (;;) {
        stream >> std::ws;
        if (stream.peek() != '#')
            break;
        std::getline(stream, out);
    }
    if (!(stream >> out))
        throw std::runtime_error("truncated PPM header");
    return out;
}
int dimension(const std::string &text) {
    std::size_t used = 0;
    const int value = std::stoi(text, &used);
    if (used != text.size() || value <= 0 || value > 8192)
        throw std::runtime_error("invalid image dimensions");
    return value;
}
void readExact(std::istream &stream, std::vector<std::uint8_t> &bytes) {
    if (!stream.read(reinterpret_cast<char *>(bytes.data()), std::streamsize(bytes.size())))
        throw std::runtime_error("truncated image data");
    if (stream.peek() != std::char_traits<char>::eof())
        throw std::runtime_error("extra image data: check format/stride/dimensions");
}
#ifdef PERCEPTION_IMAGE_CODECS
std::uint32_t big32(const std::uint8_t *p) {
    return (std::uint32_t(p[0]) << 24) | (std::uint32_t(p[1]) << 16) |
           (std::uint32_t(p[2]) << 8) | p[3];
}
std::uint32_t crc32(const std::uint8_t *p, std::size_t count) {
    static const auto table = [] {
        std::array<std::uint32_t, 256> result{};
        for (std::uint32_t i = 0; i < 256; ++i) {
            std::uint32_t c = i;
            for (int bit = 0; bit < 8; ++bit)
                c = (c >> 1) ^ ((c & 1) ? 0xedb88320U : 0U);
            result[i] = c;
        }
        return result;
    }();
    std::uint32_t crc = 0xffffffffU;
    for (std::size_t i = 0; i < count; ++i)
        crc = table[(crc ^ p[i]) & 255] ^ (crc >> 8);
    return ~crc;
}
void validatePng(const std::vector<std::uint8_t> &bytes) {
    bool header = false, data = false;
    std::size_t offset = 8;
    while (offset < bytes.size()) {
        if (bytes.size() - offset < 12)
            throw std::runtime_error("truncated PNG chunk");
        const auto length = std::size_t(big32(bytes.data() + offset));
        if (length > bytes.size() - offset - 12)
            throw std::runtime_error("truncated PNG chunk data");
        const auto *type = bytes.data() + offset + 4;
        if (crc32(type, length + 4) != big32(type + length + 4))
            throw std::runtime_error("PNG chunk CRC mismatch");
        const std::string name(reinterpret_cast<const char *>(type), 4);
        if (!header && (name != "IHDR" || length != 13))
            throw std::runtime_error("PNG must start with IHDR");
        if (name == "IHDR") {
            if (header || length != 13)
                throw std::runtime_error("invalid duplicate PNG header");
            header = true;
        }
        if (name == "IDAT")
            data = true;
        offset += length + 12;
        if (name == "IEND") {
            if (!data || length != 0 || offset != bytes.size())
                throw std::runtime_error("invalid PNG end or trailing data");
            return;
        }
    }
    throw std::runtime_error("PNG is missing IEND");
}
// stb tolerates some truncated JPEG streams. Require well-formed marker lengths,
// at least one scan and an explicit EOI before passing data to its decoder.
void validateJpeg(const std::vector<std::uint8_t> &bytes) {
    std::size_t offset = 2;
    bool scan = false, entropy = false;
    while (offset < bytes.size()) {
        if (entropy) {
            while (offset < bytes.size() && bytes[offset] != 0xff)
                ++offset;
        } else if (bytes[offset] != 0xff) {
            throw std::runtime_error("invalid JPEG marker");
        }
        if (offset == bytes.size())
            break;
        while (offset < bytes.size() && bytes[offset] == 0xff)
            ++offset;
        if (offset == bytes.size())
            break;
        const auto marker = bytes[offset++];
        if (entropy && (marker == 0 || (marker >= 0xd0 && marker <= 0xd7)))
            continue;
        entropy = false;
        if (marker == 0xd9) {
            if (!scan)
                throw std::runtime_error("JPEG has no scan");
            return; // JPEG permits trailing metadata/padding after EOI.
        }
        if (marker == 0 || marker == 0xd8 || (marker >= 0xd0 && marker <= 0xd7))
            throw std::runtime_error("unexpected JPEG marker");
        if (marker == 1)
            continue;
        if (bytes.size() - offset < 2)
            break;
        const std::size_t length = (std::size_t(bytes[offset]) << 8) | bytes[offset + 1];
        if (length < 2 || length > bytes.size() - offset)
            throw std::runtime_error("truncated JPEG segment");
        offset += length;
        if (marker == 0xda)
            scan = entropy = true;
    }
    throw std::runtime_error("truncated JPEG: missing end marker");
}
#endif
} // namespace

Image loadImage(const std::string &path, const ImageInputOptions &options) try {
    const auto start = std::chrono::steady_clock::now();
    Image image;
    const auto &format = options.format;
    if (format != "image" && format != "auto" && format != "ppm" && format != "nv12" &&
        format != "i420")
        throw std::invalid_argument("format must be image/auto/ppm/nv12/i420");
    std::ifstream stream(std::filesystem::u8path(path), std::ios::binary | std::ios::ate);
    if (!stream)
        throw perception::PerceptionException(perception::ErrorCode::CameraAcquisitionFailed,
                                              "cannot open image: " + path, "camera.file");
    const auto length = stream.tellg();
    if (length <= 0 || static_cast<std::uint64_t>(length) > max_bytes)
        throw std::runtime_error("empty image or input file exceeds 256 MiB");
    stream.seekg(0);
    if (format == "nv12" || format == "i420") {
        image.width = options.width;
        image.height = options.height;
        image.stride = options.stride ? options.stride : std::size_t(std::max(0, image.width));
        dimensions(image.width, image.height);
        if (image.width % 2 || image.height % 2 || image.stride % 2 ||
            image.stride < std::size_t(image.width) || image.stride > 65536)
            throw std::invalid_argument("raw YUV needs even width/height/stride, stride >= width and <= 65536");
        const auto size = image.stride * std::size_t(image.height) * 3 / 2;
        if (size > max_bytes)
            throw std::runtime_error("raw input buffer exceeds 256 MiB");
        image.format = format == "nv12" ? perception::PixelFormat::NV12 : perception::PixelFormat::I420;
        image.decoded_format = format;
        image.source_channels = 3;
        image.bytes.resize(size);
        readExact(stream, image.bytes);
    } else {
        std::array<unsigned char, 8> magic{};
        stream.read(reinterpret_cast<char *>(magic.data()), magic.size());
        stream.clear();
        stream.seekg(0);
        if (magic[0] == 'P' && magic[1] == '6') {
            if (token(stream) != "P6")
                throw std::runtime_error("PPM must be 8-bit P6");
            image.width = dimension(token(stream));
            image.height = dimension(token(stream));
            dimensions(image.width, image.height);
            if (token(stream) != "255")
                throw std::runtime_error("PPM must be 8-bit P6");
            const int delimiter = stream.get();
            if (delimiter == '\r' && stream.peek() == '\n')
                stream.get();
            else if (delimiter != ' ' && delimiter != '\n' && delimiter != '\t' &&
                     delimiter != '\r')
                throw std::runtime_error("missing PPM raster delimiter");
            image.stride = std::size_t(image.width) * 3;
            image.decoded_format = "ppm";
            image.source_channels = 3;
            image.bytes.resize(image.stride * image.height);
            readExact(stream, image.bytes);
        } else {
            if (format == "ppm")
                throw std::runtime_error("--format ppm requires binary RGB P6 PPM");
#ifdef PERCEPTION_IMAGE_CODECS
            const bool png = magic == std::array<unsigned char, 8>{137, 80, 78, 71, 13, 10, 26, 10};
            const bool jpeg = magic[0] == 0xff && magic[1] == 0xd8;
            if (!png && !jpeg)
                throw std::runtime_error("unsupported image content: expected P6 PPM, JPEG or PNG");
            std::vector<std::uint8_t> encoded(static_cast<std::size_t>(length));
            readExact(stream, encoded);
            if (png)
                validatePng(encoded);
            else
                validateJpeg(encoded);
            int channels = 0;
            if (!stbi_info_from_memory(encoded.data(), int(encoded.size()), &image.width,
                                       &image.height, &channels))
                throw std::runtime_error("cannot parse image header");
            image.source_bit_depth = png ? encoded[24] : 8;
            dimensions(image.width, image.height);
            dimensions(image.width, image.height,
                       std::size_t(std::max(channels, 4)) * (image.source_bit_depth == 16 ? 2 : 1));
            using Pixels = std::unique_ptr<stbi_uc, decltype(&stbi_image_free)>;
            Pixels pixels(stbi_load_from_memory(encoded.data(), int(encoded.size()), &image.width,
                                                &image.height, &channels, 3), stbi_image_free);
            if (!pixels)
                throw std::runtime_error(std::string("cannot decode image: ") + stbi_failure_reason());
            image.source_channels = channels;
            image.decoded_format = png ? "png" : "jpeg";
            image.stride = std::size_t(image.width) * 3;
            image.bytes.assign(pixels.get(), pixels.get() + image.stride * image.height);
#else
            throw std::runtime_error("JPEG/PNG support was disabled; rebuild with PERCEPTION_WITH_IMAGE_CODECS=ON");
#endif
        }
    }
    image.decode_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    return image;
} catch (const std::exception &e) {
    throw perception::PerceptionException(perception::errorFromException(
        perception::ErrorCode::CameraDataInvalid, e, "camera.decode"));
}

void writeInputPpm(const Image &image, const std::string &path) {
    if (image.format != perception::PixelFormat::RGB || image.stride != std::size_t(image.width) * 3 ||
        image.bytes.size() != image.stride * std::size_t(image.height))
        throw std::invalid_argument("--dump-input supports decoded RGB images; raw YUV must be supplied as an image");
    const auto output = std::filesystem::u8path(path);
    if (std::filesystem::exists(output))
        throw std::invalid_argument("--dump-input destination already exists");
    if (!output.parent_path().empty())
        std::filesystem::create_directories(output.parent_path());
    std::ofstream stream(output, std::ios::binary);
    stream << "P6\n" << image.width << ' ' << image.height << "\n255\n";
    stream.write(reinterpret_cast<const char *>(image.bytes.data()), std::streamsize(image.bytes.size()));
    stream.flush();
    if (!stream)
        throw std::runtime_error("cannot write decoded PPM");
}
} // namespace example
