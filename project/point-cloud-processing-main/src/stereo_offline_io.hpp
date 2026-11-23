#pragma once

// Pure offline input helpers: no ROS messages, CameraData or shared-memory ABI.
// Both encoded views use the same BT.601 limited-range integer conversion.
// Its coefficients/rounding match the previously tested stereo offline loader.
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rapidjson/document.h>

namespace perception::offline_stereo {
namespace fs = std::filesystem;

struct Pair { std::string stem, left, right; };
struct Image {
    int width = 0, height = 0;
    std::vector<uint8_t> nv12, rgb;
};
struct Calibration {
    int width = 0, height = 0;
    double fx = 0, fy = 0, cx = 0, cy = 0, bxf_px_mm = 0, baseline_mm = 0;
};

inline std::string extension(const fs::path& path) {
    std::string ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return ext;
}
inline bool supported(const fs::path& path) {
    const auto ext = extension(path);
    return ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp" ||
           ext == ".ppm" || ext == ".pnm" || ext == ".pgm" || ext == ".nv12";
}
inline std::map<std::string, std::string> images(const std::string& directory) {
    if (directory.empty() || !fs::is_directory(directory))
        throw std::invalid_argument("Stereo offline image directory does not exist: " + directory);
    std::map<std::string, std::string> result;
    for (const auto& item : fs::directory_iterator(directory)) {
        if (!item.is_regular_file() || !supported(item.path())) continue;
        const std::string stem = item.path().stem().string();
        if (stem.empty() || !result.emplace(stem, item.path().string()).second)
            throw std::invalid_argument("Duplicate stereo offline image stem: " + stem);
    }
    if (result.empty()) throw std::invalid_argument("No supported stereo images in: " + directory);
    return result;
}
inline std::vector<Pair> pairDirectories(const std::string& left, const std::string& right) {
    const auto l = images(left), r = images(right);
    if (l.size() != r.size())
        throw std::invalid_argument("Stereo left/right image counts differ");
    std::vector<Pair> output;
    output.reserve(l.size());
    for (const auto& item : l) {
        const auto found = r.find(item.first);
        if (found == r.end())
            throw std::invalid_argument("Missing matching right image stem: " + item.first);
        output.push_back({item.first, item.second, found->second});
    }
    return output;
}
inline int floorDivide(int value, int divisor) {
    return value >= 0 ? value / divisor : -((-value + divisor - 1) / divisor);
}
inline uint8_t byte(int value) { return static_cast<uint8_t>(std::clamp(value, 0, 255)); }
inline std::vector<uint8_t> rgbToNv12(const std::vector<uint8_t>& rgb, int width, int height) {
    if (width < 2 || height < 2 || (width & 1) || (height & 1) ||
        rgb.size() != static_cast<size_t>(width) * height * 3)
        throw std::invalid_argument("Stereo RGB-to-NV12 requires exact even RGB dimensions");
    const size_t plane = static_cast<size_t>(width) * height;
    std::vector<uint8_t> output(plane * 3 / 2);
    for (size_t i = 0; i < plane; ++i) {
        const auto* p = rgb.data() + i * 3;
        output[i] = byte((66 * int(p[0]) + 129 * int(p[1]) + 25 * int(p[2]) + 128) / 256 + 16);
    }
    for (int y = 0; y < height; y += 2) for (int x = 0; x < width; x += 2) {
        int r = 0, g = 0, b = 0;
        for (int dy = 0; dy < 2; ++dy) for (int dx = 0; dx < 2; ++dx) {
            const auto* p = rgb.data() + (static_cast<size_t>(y + dy) * width + x + dx) * 3;
            r += p[0]; g += p[1]; b += p[2];
        }
        const size_t cell = static_cast<size_t>(y / 2) * (width / 2) + x / 2;
        output[plane + cell * 2] = byte(floorDivide(-38 * r - 74 * g + 112 * b + 512, 1024) + 128);
        output[plane + cell * 2 + 1] = byte(floorDivide(112 * r - 94 * g - 18 * b + 512, 1024) + 128);
    }
    return output;
}
inline Image loadImage(const std::string& path, int width, int height) {
    if (width < 2 || height < 2 || width > 4096 || height > 4096 || (width & 1) || (height & 1))
        throw std::invalid_argument("Stereo offline image dimensions must be even and in [2,4096]");
    if (!supported(path) || !fs::is_regular_file(path))
        throw std::invalid_argument("Unsupported or missing stereo offline image: " + path);
    const auto bytes = fs::file_size(path);
    if (!bytes || bytes > 128U * 1024U * 1024U)
        throw std::invalid_argument("Stereo offline image must be nonempty and at most 128 MiB: " + path);
    Image output;
    output.width = width; output.height = height;
    cv::Mat rgb;
    if (extension(path) == ".nv12") {
        const size_t expected = static_cast<size_t>(width) * height * 3 / 2;
        if (bytes != expected)
            throw std::invalid_argument("Raw stereo NV12 byte count differs from exact source dimensions: " + path);
        output.nv12.resize(expected);
        std::ifstream input(path, std::ios::binary);
        if (!input.read(reinterpret_cast<char*>(output.nv12.data()), static_cast<std::streamsize>(expected)))
            throw std::runtime_error("Cannot read complete stereo NV12 input: " + path);
        // Only the RGB/GDC visualization is decoded. Inference retains the exact raw bytes.
        cv::Mat yuv(height * 3 / 2, width, CV_8UC1, output.nv12.data());
        cv::cvtColor(yuv, rgb, cv::COLOR_YUV2RGB_NV12);
    } else {
        const cv::Mat bgr = cv::imread(path, cv::IMREAD_COLOR);
        if (bgr.empty() || bgr.type() != CV_8UC3 || bgr.cols != width || bgr.rows != height)
            throw std::invalid_argument("Stereo images must already be rectified at the exact source dimensions; no resize: " + path);
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    }
    if (!rgb.isContinuous()) rgb = rgb.clone();
    output.rgb.assign(rgb.data, rgb.data + static_cast<size_t>(width) * height * 3);
    if (output.nv12.empty()) output.nv12 = rgbToNv12(output.rgb, width, height);
    return output;
}
inline double number(const rapidjson::Value& object, const char* name) {
    if (!object.HasMember(name) || !object[name].IsNumber())
        throw std::invalid_argument(std::string("Missing numeric stereo intrinsics field: ") + name);
    const double value = object[name].GetDouble();
    if (!std::isfinite(value))
        throw std::invalid_argument(std::string("Nonfinite stereo intrinsics field: ") + name);
    return value;
}
inline Calibration loadCalibration(const std::string& path, int width, int height) {
    if (width < 2 || height < 2 || width > 4096 || height > 4096 || (width & 1) || (height & 1))
        throw std::invalid_argument("Stereo calibration source dimensions must be even and in [2,4096]");
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::invalid_argument("Cannot open stereo intrinsics file: " + path);
    std::ostringstream content; content << input.rdbuf();
    rapidjson::Document root;
    root.Parse(content.str().c_str());
    if (root.HasParseError() || !root.IsObject())
        throw std::invalid_argument("Stereo intrinsics file is not a JSON object: " + path);
    const rapidjson::Value& k = root.HasMember("intrinsics") ? root["intrinsics"] : root;
    if (!k.IsObject()) throw std::invalid_argument("Stereo intrinsics must be an object");
    if (k.HasMember("bxf_unit") && (!k["bxf_unit"].IsString() ||
        std::string(k["bxf_unit"].GetString()) != "px_mm"))
        throw std::invalid_argument("Stereo bxf must use explicit px_mm contract");
    if (k.HasMember("baseline_unit") && (!k["baseline_unit"].IsString() ||
        std::string(k["baseline_unit"].GetString()) != "mm"))
        throw std::invalid_argument("Stereo base_line must be millimetres");
    const double kw = number(k, "width"), kh = number(k, "height");
    if (kw != width || kh != height)
        throw std::invalid_argument("Stereo intrinsics dimensions differ from unchanged source images");
    Calibration output;
    output.width = width; output.height = height;
    output.fx = number(k, "fx"); output.fy = number(k, "fy");
    output.cx = number(k, "cx"); output.cy = number(k, "cy");
    if (!(output.fx > 0) || !(output.fy > 0))
        throw std::invalid_argument("Stereo focal lengths must be finite and positive");
    const bool bxf = k.HasMember("bxf"), baseline = k.HasMember("base_line");
    if (!bxf && !baseline) throw std::invalid_argument("Stereo intrinsics require bxf(px_mm) or base_line(mm)");
    output.bxf_px_mm = bxf ? number(k, "bxf") : number(k, "base_line") * output.fx;
    output.baseline_mm = baseline ? number(k, "base_line") : output.bxf_px_mm / output.fx;
    if (!(output.bxf_px_mm > 0) || !(output.baseline_mm > 0) ||
        !std::isfinite(output.bxf_px_mm) || !std::isfinite(output.baseline_mm))
        throw std::invalid_argument("Stereo bxf and baseline must be finite and positive");
    // The original saveCamParams rounds these fields to two decimals and its
    // online validator allows 2 pixel*mm. Keep that explicit compatibility
    // tolerance; use the supplied bxf itself for projection, never re-scale it.
    if (bxf && baseline && std::abs(output.bxf_px_mm - output.baseline_mm * output.fx) >
        std::max(2.0, 1e-5 * std::abs(output.bxf_px_mm)))
        throw std::invalid_argument("Stereo bxf differs from base_line(mm)*fx");
    return output;
}
} // namespace perception::offline_stereo
