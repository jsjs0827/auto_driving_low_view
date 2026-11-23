#include "visualization.hpp"
#include "perception/error.hpp"
#include "perception/preprocess.hpp"
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <locale>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#include <sys/stat.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

// Keep all encoder symbols/options private to this translation unit. Never
// change stb's compression/filter/flip globals: concurrent calls only read
// their fixed defaults, and all per-image scratch data is independently owned.
#define STB_IMAGE_WRITE_STATIC
#define STBI_WRITE_NO_STDIO
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

namespace demo_vis {
namespace {
using Color = std::array<std::uint8_t, 3>;

Color classColor(int id) {
    // The built-in segmentation classes: no_grass=coral, grass=green,
    // flagstone=blue. Further IDs use a stable bit-interleaved palette.
    if (id == 0) return {235, 90, 70};
    if (id == 1) return {35, 205, 95};
    if (id == 2) return {80, 140, 245};
    auto bits = static_cast<std::uint32_t>(id) + 1U;
    Color out{};
    for (unsigned shift = 0; shift < 8; ++shift) {
        for (unsigned channel = 0; channel < 3; ++channel) {
            out[channel] |= static_cast<std::uint8_t>((bits & 1U) << (7U - shift));
            bits >>= 1U;
        }
    }
    return out;
}

// Original 5x7 ASCII glyphs, stored as rows with the high five bits in use.
// This deliberately needs no external font files or platform font services.
std::array<std::uint8_t, 7> glyph(char input) {
    const auto ch = input >= 'a' && input <= 'z' ? char(input - 'a' + 'A') : input;
    switch (ch) {
    case 'A': return {14,17,17,31,17,17,17};
    case 'B': return {30,17,17,30,17,17,30};
    case 'C': return {14,17,16,16,16,17,14};
    case 'D': return {30,17,17,17,17,17,30};
    case 'E': return {31,16,16,30,16,16,31};
    case 'F': return {31,16,16,30,16,16,16};
    case 'G': return {14,17,16,23,17,17,15};
    case 'H': return {17,17,17,31,17,17,17};
    case 'I': return {14,4,4,4,4,4,14};
    case 'J': return {7,2,2,2,2,18,12};
    case 'K': return {17,18,20,24,20,18,17};
    case 'L': return {16,16,16,16,16,16,31};
    case 'M': return {17,27,21,21,17,17,17};
    case 'N': return {17,25,21,19,17,17,17};
    case 'O': return {14,17,17,17,17,17,14};
    case 'P': return {30,17,17,30,16,16,16};
    case 'Q': return {14,17,17,17,21,18,13};
    case 'R': return {30,17,17,30,20,18,17};
    case 'S': return {15,16,16,14,1,1,30};
    case 'T': return {31,4,4,4,4,4,4};
    case 'U': return {17,17,17,17,17,17,14};
    case 'V': return {17,17,17,17,17,10,4};
    case 'W': return {17,17,17,21,21,21,10};
    case 'X': return {17,17,10,4,10,17,17};
    case 'Y': return {17,17,10,4,4,4,4};
    case 'Z': return {31,1,2,4,8,16,31};
    case '0': return {14,17,19,21,25,17,14};
    case '1': return {4,12,4,4,4,4,14};
    case '2': return {14,17,1,2,4,8,31};
    case '3': return {30,1,1,14,1,1,30};
    case '4': return {2,6,10,18,31,2,2};
    case '5': return {31,16,16,30,1,1,30};
    case '6': return {14,16,16,30,17,17,14};
    case '7': return {31,1,2,4,8,8,8};
    case '8': return {14,17,17,14,17,17,14};
    case '9': return {14,17,17,15,1,1,14};
    case '.': return {0,0,0,0,0,6,6};
    case ':': return {0,6,6,0,6,6,0};
    case '-': return {0,0,0,31,0,0,0};
    case '_': return {0,0,0,0,0,0,31};
    case '/': return {1,2,2,4,8,8,16};
    case '%': return {25,25,2,4,8,19,19};
    case '+': return {0,4,4,31,4,4,0};
    case '[': return {14,8,8,8,8,8,14};
    case ']': return {14,2,2,2,2,2,14};
    case ' ': return {};
    default: return {14,17,1,2,4,0,4};
    }
}

struct Canvas {
    std::vector<std::uint8_t>& rgb;
    int width, height;
    void pixel(int x, int y, Color color) {
        if (x < 0 || y < 0 || x >= width || y >= height) return;
        auto* dst = rgb.data() + (std::size_t(y) * width + x) * 3;
        std::copy(color.begin(), color.end(), dst);
    }
    void rectangle(int x1, int y1, int x2, int y2, Color color) {
        x1 = std::clamp(x1, 0, width); x2 = std::clamp(x2, 0, width);
        y1 = std::clamp(y1, 0, height); y2 = std::clamp(y2, 0, height);
        for (int y = y1; y < y2; ++y)
            for (int x = x1; x < x2; ++x) pixel(x, y, color);
    }
    void text(int x, int y, const std::string& value, Color color, int scale) {
        for (const char ch : value) {
            if (x + 5 * scale > width) break;
            const auto rows = glyph(ch);
            for (int row = 0; row < 7; ++row)
                for (int column = 0; column < 5; ++column)
                    if (rows[row] & (1U << (4 - column)))
                        rectangle(x + column*scale, y + row*scale,
                                  x + (column+1)*scale, y + (row+1)*scale, color);
            x += 6 * scale;
        }
    }
};

std::string label(int id, const std::vector<std::string>& names) {
    std::string out = std::to_string(id);
    if (static_cast<std::size_t>(id) < names.size() && !names[id].empty())
        out += " " + names[id].substr(0, 80);
    return out;
}

void require(bool ok, const char* why) {
    if (!ok) throw std::invalid_argument(why);
}

struct FileDescriptor {
    int value = -1;
    ~FileDescriptor() {
        if (value >= 0) {
#ifdef _WIN32
            _close(value);
#else
            ::close(value);
#endif
        }
    }
};
[[noreturn]] void fileError(const char* operation) {
    throw std::system_error(errno, std::generic_category(), operation);
}

void writeExclusive(const std::filesystem::path& path, const unsigned char* png, int bytes) {
    FileDescriptor file;
#ifdef _WIN32
    file.value = _wopen(path.c_str(), _O_WRONLY | _O_CREAT | _O_EXCL | _O_BINARY,
                       _S_IREAD | _S_IWRITE);
#else
    file.value = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0666);
#endif
    if (file.value < 0) fileError("cannot create visualization PNG (destination must be new)");
    int offset = 0;
    while (offset < bytes) {
#ifdef _WIN32
        const int written = _write(file.value, png + offset, static_cast<unsigned>(bytes - offset));
#else
        const auto written = ::write(file.value, png + offset, static_cast<std::size_t>(bytes - offset));
#endif
        if (written < 0) {
            if (errno == EINTR) continue;
            fileError("cannot write visualization PNG");
        }
        if (written == 0) throw std::runtime_error("short write while saving visualization PNG");
        offset += static_cast<int>(written); // bounded by the requested remaining byte count
    }
    const int descriptor = file.value;
    file.value = -1; // A failed close must not be retried against a reused descriptor.
#ifdef _WIN32
    if (_close(descriptor) != 0) fileError("cannot close visualization PNG");
#else
    if (::close(descriptor) != 0) fileError("cannot close visualization PNG");
#endif
}
} // namespace

void convertRgbInto(const perception::ImageView& image, std::vector<std::uint8_t>& rgb) {
    perception::validateCameraImage(image);
    require(image.width <= 4096 && image.height <= 4096, "visualization dimensions exceed 4096");
    rgb.resize(std::size_t(image.width) * image.height * 3);
    const bool yuv = image.format == perception::PixelFormat::NV12 || image.format == perception::PixelFormat::I420;
    const auto base = image.stride * image.height;
    const auto rounded = [](float value) {
        return static_cast<std::uint8_t>(std::clamp(std::lround(value), 0L, 255L));
    };
    for (int y = 0; y < image.height; ++y) {
        auto* target = rgb.data() + std::size_t(y) * image.width * 3;
        const auto* row = image.data + std::size_t(y) * image.stride;
        if (!yuv && image.format == perception::PixelFormat::RGB) {
            std::copy_n(row, std::size_t(image.width) * 3, target);
            continue;
        }
        for (int x = 0; x < image.width; ++x) {
            auto* dst = target + std::size_t(x) * 3;
            if (!yuv) {
                dst[0] = row[x*3+2]; dst[1] = row[x*3+1]; dst[2] = row[x*3];
                continue;
            }
            std::size_t ui = base + std::size_t(y/2) * image.stride + std::size_t(x/2) * 2;
            std::size_t vi = ui + 1;
            if (image.format == perception::PixelFormat::I420) {
                ui = base + std::size_t(y/2) * (image.stride/2) + x/2;
                vi = ui + base/4;
            }
            const float yy = float(row[x]) - 16;
            const float u = float(image.data[ui]) - 128, v = float(image.data[vi]) - 128;
            dst[0] = rounded(1.164383f * yy + 1.596027f * v);
            dst[1] = rounded(1.164383f * yy - .391762f * u - .812968f * v);
            dst[2] = rounded(1.164383f * yy + 2.017232f * u);
        }
    }
}

void validateOverlay(int width, int height,
                const perception::DetectionResult* detection,
                const perception::SegmentationResult* segmentation, const Style& style) {
    require(width > 0 && height > 0 && width <= 4096 && height <= 4096,
            "invalid visualization dimensions");
    require(std::isfinite(style.mask_alpha) && style.mask_alpha >= 0 && style.mask_alpha <= 1,
            "mask alpha must be finite and within [0,1]");
    const auto pixels = std::size_t(width) * height;
    if (segmentation) {
        require(segmentation->width == width && segmentation->height == height,
                "segmentation visualization dimensions differ from input");
        require(segmentation->mask.size() == pixels, "segmentation visualization mask size mismatch");
    }
    if (detection) {
        require(detection->width == width && detection->height == height,
                "detection visualization dimensions differ from input");
        require(detection->detections.size() <= 4096, "too many detections for visualization");
        for (const auto& d : detection->detections) {
            require(d.class_id >= 0 && std::isfinite(d.score) && d.score >= 0 && d.score <= 1,
                    "invalid detection visualization class or score");
            require(std::isfinite(d.x1) && std::isfinite(d.x2) &&
                        std::isfinite(d.y1) && std::isfinite(d.y2) && d.x1 < d.x2 && d.y1 < d.y2,
                    "invalid detection visualization coordinates");
        }
    }
}

void overlayRgb(std::vector<std::uint8_t>& rgb, int width, int height,
                const perception::DetectionResult* detection,
                const perception::SegmentationResult* segmentation, const Style& style) {
    validateOverlay(width,height,detection,segmentation,style);
    require(rgb.size() == std::size_t(width) * height * 3, "visualization RGB size mismatch");
    const auto pixels = std::size_t(width) * height;
    std::set<int> mask_ids, detection_ids;
    if (segmentation) {
        std::array<Color, 256> palette{};
        std::array<bool, 256> present{};
        for (int id = 0; id < 256; ++id) palette[id] = classColor(id);
        for (std::size_t i = 0; i < pixels; ++i) {
            const int id = segmentation->mask[i];
            present[id] = true;
            const auto color = palette[id];
            for (int c = 0; c < 3; ++c) {
                const float blended = (1 - style.mask_alpha) * rgb[i*3+c] + style.mask_alpha * color[c];
                rgb[i*3+c] = static_cast<std::uint8_t>(std::clamp(blended + .5f, 0.f, 255.f));
            }
        }
        for (int id = 0; id < 256; ++id) if (present[id]) mask_ids.insert(id);
    }
    Canvas canvas{rgb, width, height};
    const int scale = width >= 640 && height >= 320 ? 2 : 1;
    if (detection) {
        for (const auto& d : detection->detections) {
            // Clamp finite coordinates BEFORE float-to-int conversion; even
            // FLT_MAX boxes are harmless. Completely outside boxes are skipped.
            const float left = std::clamp(d.x1, 0.f, float(width));
            const float right = std::clamp(d.x2, 0.f, float(width));
            const float top = std::clamp(d.y1, 0.f, float(height));
            const float bottom = std::clamp(d.y2, 0.f, float(height));
            if (left >= right || top >= bottom) continue;
            const int x1 = static_cast<int>(std::floor(left));
            const int y1 = static_cast<int>(std::floor(top));
            const int x2 = static_cast<int>(std::ceil(right));
            const int y2 = static_cast<int>(std::ceil(bottom));
            const auto color = classColor(d.class_id);
            const int line = std::min({2 * scale, x2-x1, y2-y1});
            canvas.rectangle(x1,y1,x2,y1+line,color);
            canvas.rectangle(x1,y2-line,x2,y2,color);
            canvas.rectangle(x1,y1,x1+line,y2,color);
            canvas.rectangle(x2-line,y1,x2,y2,color);
            std::ostringstream text;
            text.imbue(std::locale::classic());
            text << "DET " << label(d.class_id, style.detection_names) << ' '
                 << std::fixed << std::setprecision(2) << d.score;
            const auto caption = text.str();
            const int label_width = std::min(width-x1, int(caption.size()) * 6 * scale + 4);
            const int label_y = y1 >= 9*scale+2 ? y1 - 9*scale - 2 : y1+line;
            canvas.rectangle(x1,label_y,x1+label_width,label_y+9*scale,{18,21,25});
            canvas.text(x1+2,label_y+scale,caption,{255,255,255},scale);
            detection_ids.insert(d.class_id);
        }
    }
    // Compact legend; keep at least two thirds of the frame free of the panel.
    // Very small images omit the panel instead of obscuring the entire mask.
    std::vector<std::pair<std::string, Color>> legend;
    for (const int id : mask_ids) legend.push_back({"SEG " + label(id,style.segmentation_names),classColor(id)});
    for (const int id : detection_ids) legend.push_back({"DET " + label(id,style.detection_names),classColor(id)});
    const int row_height = 10*scale;
    const int row_limit = std::min(8, (height/3 - 8) / row_height);
    if (!legend.empty() && width >= 100 && row_limit > 0) {
        const auto original_count = legend.size();
        legend.resize(std::min<std::size_t>(legend.size(), std::size_t(row_limit)));
        if (original_count > legend.size())
            legend.back() = {"+" + std::to_string(original_count - legend.size() + 1) + " MORE",{220,220,220}};
        std::size_t longest = 0;
        for (const auto& row : legend) longest = std::max(longest, row.first.size());
        const int panel_width = std::min(width-8, int(longest)*6*scale+18*scale);
        canvas.rectangle(4,4,4+panel_width,8+int(legend.size())*row_height,{18,21,25});
        int y = 7;
        for (const auto& row : legend) {
            canvas.rectangle(7,y,7+6*scale,y+7*scale,row.second);
            canvas.text(7+9*scale,y,row.first,{245,245,245},scale);
            y += row_height;
        }
    }
}

std::vector<std::uint8_t> renderRgb(const example::Image& image,
    const perception::DetectionResult* detection,
    const perception::SegmentationResult* segmentation, const Style& style) {
    std::vector<std::uint8_t> rgb;
    convertRgbInto(image.view(), rgb);
    overlayRgb(rgb, image.width, image.height, detection, segmentation, style);
    return rgb;
}

void saveRgbPng(const std::filesystem::path& path, const std::vector<std::uint8_t>& rgb,
                int width, int height) {
    require(width > 0 && height > 0 && width <= 4096 && height <= 4096,
            "invalid PNG dimensions");
    require(rgb.size() == std::size_t(width) * height * 3, "PNG RGB size mismatch");
    int bytes = 0;
    using Buffer = std::unique_ptr<unsigned char, decltype(&std::free)>;
    Buffer encoded(stbi_write_png_to_mem(rgb.data(), width*3, width,
                                         height, 3, &bytes), &std::free);
    if (!encoded || bytes <= 0) throw std::runtime_error("cannot encode visualization PNG");
    writeExclusive(path, encoded.get(), bytes);
}

void savePng(const std::filesystem::path& path, const example::Image& image,
             const perception::DetectionResult* detection,
             const perception::SegmentationResult* segmentation, const Style& style) {
    const auto rgb = renderRgb(image, detection, segmentation, style);
    saveRgbPng(path, rgb, image.width, image.height);
}
} // namespace demo_vis
