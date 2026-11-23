#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace raw_lux1 {
constexpr int width = 1088, height = 1280;
constexpr std::array<int, 4> roi = {435, 128, 1033, 1152};
inline const std::array<std::string, 3> classes = {"NOT_LOW_LIGHT", "LOW_LIGHT", "VERY_LOW_LIGHT"};

struct Model {
    std::string name, source_sha256;
    bool requires_metadata;
    std::vector<std::string> feature_names;
    std::vector<std::vector<double>> coefficients;
    std::vector<double> intercepts, training_min, training_max;
    std::string warning;
    std::vector<std::string> label_definitions;
};

struct Record {
    std::string group_id, eye;
    std::vector<double> values;  // P10/P50, 64 bins, optional linear exposure/gain
};

struct Prediction {
    std::string group_id;
    std::array<std::size_t, 2> frame_counts{};
    std::vector<double> features;  // log2 metadata after aggregation
    std::array<double, 3> probabilities{};
    std::vector<std::string> outside_training;
    std::size_t class_index = 0;
};

inline double positive(double value, const std::string& name) {
    if (!std::isfinite(value) || value <= 0)
        throw std::runtime_error(name + " must be positive and finite");
    return value;
}

inline std::vector<double> extract_features(const std::filesystem::path& path) {
    constexpr std::size_t frame_bytes = static_cast<std::size_t>(width) * height * 2;
    if (std::filesystem::file_size(path) != frame_bytes)
        throw std::runtime_error("RAW must contain exactly 2785280 bytes: " + path.u8string());
    std::ifstream stream(path, std::ios::binary);
    std::vector<unsigned char> data(frame_bytes);
    if (!stream.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(data.size())))
        throw std::runtime_error("cannot read complete RAW: " + path.u8string());
    if (stream.peek() != std::char_traits<char>::eof())
        throw std::runtime_error("RAW size changed while reading");
    std::array<std::uint64_t, 1024> counts{};
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const auto index = static_cast<std::size_t>(y) * width + x;
            if (index == 0) continue;  // embedded first uint16 is not a pixel
            const unsigned value = data[index * 2] | (unsigned(data[index * 2 + 1]) << 8);
            if (value > 1023)
                throw std::runtime_error("RAW contains values above 1023 outside first uint16");
            if (x >= roi[0] && x < roi[2] && y >= roi[1] && y < roi[3]) ++counts[value];
        }
    }
    std::array<std::uint64_t, 1024> cumulative{};
    std::partial_sum(counts.begin(), counts.end(), cumulative.begin());
    const auto n = cumulative.back();
    auto percentile = [&](double fraction) {
        const double rank = static_cast<double>(n - 1) * fraction;
        const auto lo = static_cast<std::uint64_t>(std::floor(rank));
        const auto hi = static_cast<std::uint64_t>(std::ceil(rank));
        const auto a = std::upper_bound(cumulative.begin(), cumulative.end(), lo) - cumulative.begin();
        const auto b = std::upper_bound(cumulative.begin(), cumulative.end(), hi) - cumulative.begin();
        return static_cast<double>(a) + (rank - static_cast<double>(lo)) * static_cast<double>(b - a);
    };
    std::vector<double> features{percentile(0.1), percentile(0.5)};
    for (int bin = 0; bin < 64; ++bin) {
        const auto count = std::accumulate(counts.begin() + bin * 16, counts.begin() + (bin + 1) * 16,
                                           std::uint64_t{0});
        features.push_back(static_cast<double>(count) / static_cast<double>(n));
    }
    return features;
}

inline double median(std::vector<double> values) {
    if (values.empty()) throw std::runtime_error("cannot aggregate an empty eye");
    std::sort(values.begin(), values.end());
    const auto mid = values.size() / 2;
    return values.size() % 2 ? values[mid] : (values[mid - 1] + values[mid]) * 0.5;
}

// Callers with camera buffers can supply Record.values in the documented order.
// Do not replace a missing eye by duplicating the other eye.
inline Prediction predict(const Model& model, const std::string& group_id,
                          const std::vector<Record>& records) {
    const auto dimension = model.feature_names.size();
    if (dimension != (model.requires_metadata ? 68u : 66u)
            || model.coefficients.size() != 3 || model.intercepts.size() != 3
            || model.training_min.size() != dimension || model.training_max.size() != dimension)
        throw std::runtime_error("invalid model dimensions");
    Prediction result;
    result.group_id = group_id;
    result.features.assign(dimension, 0.0);
    for (std::size_t eye = 0; eye < 2; ++eye) {
        std::vector<const Record*> selected;
        for (const auto& row : records) {
            if (row.group_id != group_id) continue;
            if (row.eye != "left" && row.eye != "right") throw std::runtime_error("invalid eye");
            if (row.eye == (eye == 0 ? "left" : "right")) {
                if (row.values.size() != dimension) throw std::runtime_error("invalid feature dimensions");
                for (std::size_t j = 0; j < dimension; ++j) {
                    const double v = row.values[j];
                    if (!std::isfinite(v) || (j < 2 && (v < 0 || v > 1023))
                            || (j >= 2 && j < 66 && (v < 0 || v > 1))
                            || (j >= 66 && v <= 0)) throw std::runtime_error("invalid feature value");
                }
                if (row.values[0] > row.values[1]) throw std::runtime_error("P10 exceeds P50");
                selected.push_back(&row);
            }
        }
        if (selected.empty()) throw std::runtime_error("group requires both left and right eyes: " + group_id);
        result.frame_counts[eye] = selected.size();
        for (std::size_t j = 0; j < dimension; ++j) {
            std::vector<double> column;
            for (const auto* row : selected) column.push_back(row->values[j]);
            result.features[j] += median(std::move(column)) * 0.5;
        }
    }
    if (model.requires_metadata) {
        result.features[66] = std::log2(positive(result.features[66], "exposure_us"));
        result.features[67] = std::log2(positive(result.features[67], "analog_gain"));
    }
    for (std::size_t j = 0; j < dimension; ++j) {
        if (result.features[j] < model.training_min[j] || result.features[j] > model.training_max[j])
            result.outside_training.push_back(model.feature_names[j]);
    }
    for (std::size_t k = 0; k < 3; ++k) {
        if (model.coefficients[k].size() != dimension) throw std::runtime_error("invalid coefficient dimensions");
        double score = model.intercepts[k];
        for (std::size_t j = 0; j < dimension; ++j) score += model.coefficients[k][j] * result.features[j];
        if (!std::isfinite(score)) throw std::runtime_error("non-finite score");
        result.probabilities[k] = score;
    }
    const auto max_score = *std::max_element(result.probabilities.begin(), result.probabilities.end());
    double total = 0;
    for (auto& value : result.probabilities) { value = std::exp(value - max_score); total += value; }
    for (auto& value : result.probabilities) value /= total;
    result.class_index = static_cast<std::size_t>(std::max_element(result.probabilities.begin(),
                                                                 result.probabilities.end()) - result.probabilities.begin());
    return result;
}
}  // namespace raw_lux1
