#include "product/offline_input.hpp"
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>

namespace r30::product {
namespace {
namespace fs = std::filesystem;
using Paths = std::map<std::string, fs::path>;
Paths scan(const std::string& directory, bool rgb) {
    const auto root = fs::u8path(directory);
    if (directory.empty() || !fs::is_directory(root))
        throw std::runtime_error("offline directory is missing: " + directory);
    Paths result;
    for (const auto& entry : fs::directory_iterator(root)) {
        if (!entry.is_regular_file()) continue;
        auto extension = entry.path().extension().u8string();
        std::transform(extension.begin(), extension.end(), extension.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (rgb ? (extension != ".jpg" && extension != ".jpeg") : extension != ".raw") continue;
        auto stem = entry.path().stem().u8string();
        if (stem.empty() || !result.emplace(stem, entry.path()).second)
            throw std::runtime_error("duplicate offline frame stem: " + stem);
    }
    if (result.empty()) throw std::runtime_error("offline input directory has no frames: " + directory);
    return result;
}
std::vector<std::uint8_t> read(const std::string& path, std::size_t expected) {
    std::ifstream stream(fs::u8path(path), std::ios::binary | std::ios::ate);
    if (!stream) throw std::runtime_error("cannot open offline input: " + path);
    const auto length = stream.tellg();
    if (length <= 0 || static_cast<std::uintmax_t>(length) >
            static_cast<std::uintmax_t>(std::numeric_limits<std::streamsize>::max()) ||
        static_cast<std::uintmax_t>(length) > std::numeric_limits<std::size_t>::max())
        throw std::runtime_error("invalid offline input size: " + path);
    const auto bytes = static_cast<std::size_t>(length);
    if (expected && bytes != expected)
        throw std::runtime_error("offline disparity size mismatch: " + path);
    std::vector<std::uint8_t> data(bytes);
    stream.seekg(0, std::ios::beg);
    stream.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(bytes));
    if (!stream || stream.peek() != std::char_traits<char>::eof())
        throw std::runtime_error("offline input changed or read incomplete: " + path);
    return data;
}
}
std::size_t offlineDisparityBytes(int width, int height) {
    if (width <= 0 || height <= 0 || static_cast<std::size_t>(width) >
        std::numeric_limits<std::size_t>::max() / sizeof(std::uint16_t) / static_cast<std::size_t>(height))
        throw std::invalid_argument("offline disparity dimensions must be positive and fit memory");
    return static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * sizeof(std::uint16_t);
}
std::vector<OfflineFrame> indexOfflineInput(const std::string& rgb_dir,
    const std::string& disparity_dir, int width, int height) {
    const auto expected = offlineDisparityBytes(width, height);
    const auto images = scan(rgb_dir, true);
    const auto disparities = scan(disparity_dir, false);
    for (const auto& item : disparities)
        if (!images.count(item.first))
            throw std::runtime_error("offline RGB frame missing for stem: " + item.first);
    std::vector<OfflineFrame> result;
    for (const auto& item : images) {
        const auto match = disparities.find(item.first);
        if (match == disparities.end())
            throw std::runtime_error("offline disparity frame missing for stem: " + item.first);
        if (fs::file_size(item.second) == 0)
            throw std::runtime_error("offline RGB frame is empty: " + item.second.u8string());
        if (fs::file_size(match->second) != expected)
            throw std::runtime_error("offline disparity size mismatch: " + match->second.u8string());
        result.push_back({item.first, item.second.u8string(), match->second.u8string()});
    }
    return result;
}
std::vector<std::uint8_t> readOfflineImage(const std::string& path) { return read(path, 0); }
std::vector<std::uint8_t> readOfflineDisparity(const std::string& path, int width, int height) {
    return read(path, offlineDisparityBytes(width, height));
}
bool parseOfflineJump(const std::string& command, std::size_t count, std::size_t& index) noexcept {
    if (command.size() < 3 || (command[0] != 'n' && command[0] != 'N') || command[1] != ' ') return false;
    std::size_t pos = 2;
    while (pos < command.size() && command[pos] == ' ') ++pos;
    if (pos == command.size()) return false;
    std::size_t value = 0;
    for (; pos < command.size() && command[pos] >= '0' && command[pos] <= '9'; ++pos) {
        const auto digit = static_cast<std::size_t>(command[pos] - '0');
        if (value > (std::numeric_limits<std::size_t>::max() - digit) / 10) return false;
        value = value * 10 + digit;
    }
    while (pos < command.size() && command[pos] == ' ') ++pos;
    if (pos != command.size() || value >= count) return false;
    index = value;
    return true;
}
}
