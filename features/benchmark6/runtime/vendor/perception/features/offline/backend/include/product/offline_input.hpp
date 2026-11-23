#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace r30::product {
// File names and paths crossing this boundary are UTF-8. JPEG decoding remains
// the original OpenCV product operation; this helper never substitutes PCL/AI.
struct OfflineFrame {
    std::string stem;
    std::string rgb_path;
    std::string disparity_path;
};
std::size_t offlineDisparityBytes(int width, int height);
std::vector<OfflineFrame> indexOfflineInput(const std::string& rgb_dir,
    const std::string& disparity_dir, int width, int height);
std::vector<std::uint8_t> readOfflineImage(const std::string& path);
std::vector<std::uint8_t> readOfflineDisparity(const std::string& path,
    int width, int height);
// Accepts exactly "n <zero-based index>" (case-insensitive n). On failure the
// caller's index is unchanged. No wraparound, negative or trailing input.
bool parseOfflineJump(const std::string& command, std::size_t count,
    std::size_t& index) noexcept;
}
