#pragma once
// Actual production offline RAW reader. No ROS or camera shared-memory ABI.
#include "percep_main/raw_lux_runner.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <rapidjson/document.h>

namespace perception::raw_lux_io {
constexpr int kWidth = 1088, kHeight = 1280;
constexpr std::size_t kStride = 2176, kFrameBytes = 2785280;

inline double orgProxyExposureUs(double exposure_ms) {
    const double exposure_us = exposure_ms * 1000.0;
    if (!std::isfinite(exposure_ms) || exposure_ms <= 0 || !std::isfinite(exposure_us))
        throw std::invalid_argument("Org proxy exposure must be finite positive milliseconds before conversion to us");
    return exposure_us;
}

inline bool stringField(const rapidjson::Value& object, const char* key, std::string* value) {
    if (!object.HasMember(key)) return true;
    if (!object[key].IsString()) return false;
    *value = object[key].GetString(); return true;
}

inline double positiveNumber(const rapidjson::Value& object, const char* key) {
    if (!object.HasMember(key) || !object[key].IsNumber())
        throw std::invalid_argument(std::string("Missing numeric RAW metadata: ") + key);
    const double value = object[key].GetDouble();
    if (!std::isfinite(value) || value <= 0)
        throw std::invalid_argument(std::string("RAW metadata must be finite and positive: ") + key);
    return value;
}

inline void readRaw(const std::filesystem::path& path, RawLuxImageView* view,
                    std::vector<std::uint8_t>* owned) {
    if (!std::filesystem::is_regular_file(path) || std::filesystem::file_size(path) != kFrameBytes)
        throw std::invalid_argument("RAW must contain exactly2785280 bytes: " + path.string());
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("Cannot open RAW: " + path.string());
    owned->resize(kFrameBytes);
    if (!input.read(reinterpret_cast<char*>(owned->data()), static_cast<std::streamsize>(owned->size())))
        throw std::runtime_error("Cannot read complete RAW: " + path.string());
    if (input.peek() != std::char_traits<char>::eof())
        throw std::runtime_error("RAW size changed while reading: " + path.string());
    view->data = owned->data(); view->bytes = owned->size();
    view->width = kWidth; view->height = kHeight; view->stride_bytes = kStride;
    view->format = RawLuxPixelFormat::UInt16LE;
}

inline void readOfflinePair(const std::string& left_directory, const std::string& right_directory,
                            const std::string& metadata_directory, const std::string& stem,
                            bool requires_metadata, const std::string& configured_source,
                            RawLuxImageView* left, RawLuxImageView* right,
                            std::vector<std::uint8_t>* left_owned,
                            std::vector<std::uint8_t>* right_owned) {
    namespace fs = std::filesystem;
    *left = {}; *right = {};
    if (stem.empty() || fs::path(stem).filename().string() != stem || stem == "." || stem == "..")
        throw std::invalid_argument("Offline RAW requires the current input image's simple stem");
    const fs::path left_path = fs::path(left_directory)/(stem+".raw");
    const fs::path right_path = fs::path(right_directory)/(stem+".raw");
    if (fs::is_regular_file(left_path) && fs::is_regular_file(right_path) && fs::equivalent(left_path, right_path))
        throw std::invalid_argument("Left and right RAW paths must not resolve to the same file");
    readRaw(left_path, left, left_owned); readRaw(right_path, right, right_owned);
    // A stem pairs replay files; it is not proof of hardware timestamps or capture IDs.
    if (!requires_metadata) return;
    const fs::path path = fs::path(metadata_directory)/(stem+".json");
    if (!fs::is_regular_file(path) || fs::file_size(path) > 65536)
        throw std::invalid_argument("Missing/oversized RAW metadata sidecar: " + path.string());
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("Cannot open RAW metadata sidecar");
    std::string text(65537, '\0');
    input.read(text.data(), static_cast<std::streamsize>(text.size()));
    const auto count = input.gcount();
    if (count > 65536 || input.bad() || (!input.eof() && input.fail()))
        throw std::invalid_argument("RAW metadata sidecar is oversized or cannot be read completely");
    text.resize(static_cast<std::size_t>(count));
    rapidjson::Document d; d.Parse(text.c_str(), text.size());
    if (d.HasParseError() || !d.IsObject())
        throw std::invalid_argument("RAW metadata sidecar must be a JSON object");
    std::string source;
    if (!stringField(d, "metadata_source", &source) || source != configured_source)
        throw std::invalid_argument("RAW sidecar metadata_source must equal the explicitly configured source");
    left->exposure_us = positiveNumber(d, "left_exposure_us");
    left->analog_gain = positiveNumber(d, "left_analog_gain");
    right->exposure_us = positiveNumber(d, "right_exposure_us");
    right->analog_gain = positiveNumber(d, "right_analog_gain");
}
} // namespace perception::raw_lux_io
