#include "perception/config.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace perception {
namespace {
void require(bool ok, const char *message) {
    if (!ok)
        throw std::invalid_argument(message);
}
void validateClasses(int classes, const std::vector<std::string> &names) {
    require(classes > 0 && classes <= 256, "classes must be explicit, 1..256");
    require(names.empty() || names.size() == std::size_t(classes), "class_names count mismatch");
    for (const auto &name : names)
        require(!name.empty(), "empty class name");
}
std::string trim(const std::string &s) {
    const auto start = s.find_first_not_of(" \t\r\n");
    return start == std::string::npos ? std::string{}
                                      : s.substr(start, s.find_last_not_of(" \t\r\n") - start + 1);
}
std::vector<std::string> split(const std::string &s) {
    std::istringstream stream(s);
    std::string part;
    std::vector<std::string> out;
    while (std::getline(stream, part, ','))
        out.push_back(trim(part));
    if (!s.empty() && s.back() == ',')
        out.emplace_back();
    return out;
}
template <class T> T parseValue(const std::string &s) {
    if constexpr (std::is_same_v<T, std::string>)
        return s;
    else if constexpr (std::is_same_v<T, bool>) {
        if (s == "true")
            return true;
        if (s == "false")
            return false;
        throw std::invalid_argument("boolean must be true/false");
    } else if constexpr (std::is_same_v<T, int> || std::is_same_v<T, float>) {
        std::size_t used = 0;
        T result{};
        if constexpr (std::is_same_v<T, int>)
            result = std::stoi(s, &used);
        else
            result = std::stof(s, &used);
        if (used != s.size())
            throw std::invalid_argument("invalid numeric value: " + s);
        return result;
    } else
        static_assert(!sizeof(T), "unsupported config field type");
}
struct Entry {
    std::string value;
    int line;
};
class Reader {
    std::map<std::string, Entry> entries_;
    std::string path_;

  public:
    explicit Reader(const std::string &path) : path_(path) {
        std::ifstream stream(std::filesystem::u8path(path));
        if (!stream)
            throw std::runtime_error("cannot open profile: " + path);
        std::string line;
        int line_number = 0;
        while (std::getline(stream, line)) {
            ++line_number;
            if (line_number == 1 && line.compare(0, 3, "\xEF\xBB\xBF") == 0)
                line.erase(0, 3);
            line = trim(line);
            if (line.empty() || line.front() == '#')
                continue;
            const auto pos = line.find('=');
            if (pos == std::string::npos)
                throw std::runtime_error(path + ":" + std::to_string(line_number) +
                                         ": expected key=value");
            auto key = trim(line.substr(0, pos));
            if (!entries_.emplace(key, Entry{trim(line.substr(pos + 1)), line_number}).second)
                throw std::runtime_error(path + ":" + std::to_string(line_number) +
                                         ": duplicate key: " + key);
        }
    }
    bool has(const std::string &key) const { return entries_.count(key) != 0; }
    template <class T> void field(const std::string &key, T &target) {
        auto it = entries_.find(key);
        if (it == entries_.end())
            return;
        try {
            target = parseValue<T>(it->second.value);
        } catch (const std::exception &e) {
            throw std::runtime_error(path_ + ":" + std::to_string(it->second.line) + ": " + key +
                                     ": " + e.what());
        }
        entries_.erase(it);
    }
    void finish() const {
        if (!entries_.empty())
            throw std::runtime_error(
                path_ + ":" + std::to_string(entries_.begin()->second.line) +
                ": unknown or inapplicable field for this task: " + entries_.begin()->first);
    }
};
} // namespace
void BackendConfig::validate() const {
    require(device == Device::CPU || device == Device::CUDA || device == Device::BPU,
            "invalid device");
    require(cpu_threads > 0 && cpu_threads <= 256 && device_id >= 0 && bpu_timeout_ms > 0,
            "invalid threads/device/timeout");
}
void PreprocessConfig::validate() const {
    require(width >= 0 && height >= 0 && width <= 4096 && height <= 4096 &&
                ((width == 0) == (height == 0)),
            "set both width/height, or neither");
    for (int c = 0; c < 3; ++c)
        require(std::isfinite(mean[c]) && std::isfinite(stddev[c]) && stddev[c] > 0,
                "invalid normalization");
}
void ModelConfig::validate() const {
    backend.validate();
    preprocess.validate();
    require(output_layout == Layout::NCHW || output_layout == Layout::NHWC,
            "invalid output layout");
}
void DetectionConfig::validate() const {
    validateClasses(classes, class_names);
    require(reg_bins >= 2 && reg_bins <= 256, "reg_bins must be 2..256");
    require(std::isfinite(score_threshold) && score_threshold > 0 && score_threshold < 1,
            "score_threshold must be in (0,1)");
    require(std::isfinite(iou_threshold) && iou_threshold >= 0 && iou_threshold <= 1,
            "iou_threshold must be in [0,1]");
    require(max_detections > 0 && max_detections <= 4096 && candidate_budget >= max_detections &&
                candidate_budget >= classes && candidate_budget <= 16384,
            "invalid candidate budget/max_detections");
    const bool automatic = std::all_of(heads.begin(), heads.end(), [](int i) { return i == -1; });
    require(automatic ||
                (std::all_of(heads.begin(), heads.end(), [](int i) { return i >= 0 && i < 6; }) &&
                 std::set<int>(heads.begin(), heads.end()).size() == 6),
            "set six distinct valid head indices or auto");
}
void SegmentationConfig::validate() const {
    validateClasses(classes, class_names);
    require(seg_output >= 0, "seg_output must be explicit");
}
void Profile::validate() const {
    model.validate();
    std::visit([](const auto &c) { c.validate(); }, task);
}
Profile loadProfile(const std::string &path) {
    Reader r(path);
    Profile p;
    auto &m = p.model;
    auto &b = m.backend;
    auto &pre = m.preprocess;
    std::string task = "detection", device = "cpu", layout = "nhwc", names;
    r.field("task", task);
    r.field("device", device);
    r.field("output_layout", layout);
    if (device == "cpu")
        b.device = Device::CPU;
    else if (device == "cuda")
        b.device = Device::CUDA;
    else if (device == "bpu")
        b.device = Device::BPU;
    else
        throw std::invalid_argument("unknown device");
    if (layout == "nchw")
        m.output_layout = Layout::NCHW;
    else if (layout == "nhwc")
        m.output_layout = Layout::NHWC;
    else
        throw std::invalid_argument("unknown output layout");
    r.field("model", m.source.path);
    r.field("model_name", m.source.model_name);
    r.field("cpu_threads", b.cpu_threads);
    r.field("device_id", b.device_id);
    r.field("bpu_timeout_ms", b.bpu_timeout_ms);
    r.field("width", pre.width);
    r.field("height", pre.height);
    r.field("letterbox", pre.letterbox);
    r.field("input_bgr", pre.input_bgr);
    int pad = pre.pad;
    r.field("pad", pad);
    require(pad >= 0 && pad <= 255, "pad must be 0..255");
    pre.pad = static_cast<std::uint8_t>(pad);
    auto triple = [&](const char *key, std::array<float, 3> &values) {
        if (!r.has(key))
            return;
        std::string raw;
        r.field(key, raw);
        auto parts = split(raw);
        require(parts.size() == 3, "normalization needs 3 values");
        for (int i = 0; i < 3; ++i)
            values[i] = parseValue<float>(parts[i]);
    };
    triple("mean", pre.mean);
    triple("stddev", pre.stddev);
    r.field("class_names", names);
    if (task == "detection") {
        DetectionConfig d;
        d.class_names = split(names);
        r.field("classes", d.classes);
        r.field("reg_bins", d.reg_bins);
        r.field("score_threshold", d.score_threshold);
        r.field("iou_threshold", d.iou_threshold);
        r.field("max_detections", d.max_detections);
        r.field("candidate_budget", d.candidate_budget);
        std::string raw = "auto";
        r.field("heads", raw);
        if (raw != "auto") {
            auto parts = split(raw);
            require(parts.size() == 6, "heads requires 6 indices");
            for (int i = 0; i < 6; ++i)
                d.heads[i] = parseValue<int>(parts[i]);
        }
        p.task = std::move(d);
    } else if (task == "segmentation") {
        SegmentationConfig s;
        s.class_names = split(names);
        r.field("classes", s.classes);
        r.field("seg_output", s.seg_output);
        r.field("align_corners", s.align_corners);
        r.field("bilinear_logits", s.bilinear_logits);
        p.task = std::move(s);
    } else
        throw std::invalid_argument("unknown task");
    r.finish();
    p.validate();
    return p;
}
} // namespace perception
