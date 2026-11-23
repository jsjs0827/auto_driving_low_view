#pragma once
#include "json.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>
namespace ab {
namespace fs = std::filesystem;
namespace j = r33::json;
using Json = j::JsonValue;
inline std::string str(const Json &x, const char *k) {
    return j::string(j::required(x, k));
}
inline int num(const Json &x, const char *k, int a = 0, int b = 1000000) {
    return j::integer(j::required(x, k), a, b);
}
inline double real(const Json &x, const char *k) {
    auto &v = j::required(x, k);
    if (v.kind != Json::Kind::Number)
        throw std::invalid_argument(std::string(k) + " must be numeric");
    double d = std::stod(v.text);
    if (!std::isfinite(d))
        throw std::invalid_argument("nonfinite number");
    return d;
}
inline std::string dump(const Json &v) {
    switch (v.kind) {
    case Json::Kind::Null:
        return "null";
    case Json::Kind::Boolean:
        return v.boolean ? "true" : "false";
    case Json::Kind::Number:
        return v.text;
    case Json::Kind::String:
        return j::quote(v.text);
    case Json::Kind::Array: {
        std::string s = "[";
        for (auto &x : v.array) {
            if (s.size() > 1)
                s += ",";
            s += dump(x);
        }
        return s + "]";
    }
    case Json::Kind::Object: {
        std::string s = "{";
        for (auto &x : v.object) {
            if (s.size() > 1)
                s += ",";
            s += j::quote(x.first) + ":" + dump(x.second);
        }
        return s + "}";
    }
    }
    throw std::logic_error("invalid JSON kind");
}
inline fs::path resolve(const fs::path &parent, const std::string &s) {
    auto p = fs::u8path(s);
    return fs::absolute(p.is_absolute() ? p : parent / p).lexically_normal();
}
inline std::uint64_t u64(const std::string &s) {
    if (s.empty() || s.find_first_not_of("0123456789") != std::string::npos)
        throw std::invalid_argument("Frame identity must be an unsigned decimal string");
    std::size_t n = 0;
    auto v = std::stoull(s, &n);
    if (n != s.size())
        throw std::invalid_argument("Invalid frame identity");
    return v;
}
struct Roi {
    int x = 0, y = 192, w = 640, h = 352;
};
struct Intrinsics {
    int w = 640, h = 544;
    double fx = 0, fy = 0, cx = 0, cy = 0, bxf = 0;
};
inline void validate(const Intrinsics &k) {
    if (k.w <= 0 || k.h <= 0 || (k.w | k.h) & 1 || !std::isfinite(k.fx) || !std::isfinite(k.fy) ||
        !std::isfinite(k.cx) || !std::isfinite(k.cy) || !std::isfinite(k.bxf) || k.fx <= 0 ||
        k.fy <= 0 || k.bxf <= 0)
        throw std::invalid_argument("Invalid rectified intrinsics; bxf unit must be pixel_mm");
}
struct Config {
    fs::path file, base, manifest, intrinsics, det, seg, stereo, mapping, extrinsics;
    Json root;
    std::string source = "offline", camera_sdk, camera_config;
    std::string left_topic, right_topic, q8_topic, left_info_topic, right_info_topic;
    std::string ros_transport = "dds", bridge_host = "127.0.0.1";
    int bridge_port = 19091;
    int width = 640, height = 544, frames = 90, warmup = 10, sample_step = 4, save_every = 0,
        timeout_ms = 5000, cache_size = 16;
    double ros_publish_hz = 5;
    bool ros_reliable = true;
    bool retain_clouds = true, render_maps = true, run_mapping = true;
    bool ros_acknowledged = false;
    std::string ros_ack_topic = "/benchmark6/replay_ack";
    Roi roi;
    std::vector<std::string> methods, suites;
};
inline Config load(const fs::path &path) {
    Config c;
    c.file = fs::absolute(path).lexically_normal();
    c.base = c.file.parent_path();
    c.root = j::load(c.file.u8string());
    j::fields(c.root, {"schema_version", "input", "models", "pipeline", "experiment", "ros"});
    if (num(c.root, "schema_version", 1, 1) != 1)
        throw std::invalid_argument("Unsupported schema");
    const auto &in = j::required(c.root, "input");
    j::fields(in, {"source", "width", "height", "offline", "camera"});
    c.source = str(in, "source");
    if (c.source != "offline" && c.source != "camera" && c.source != "ros")
        throw std::invalid_argument("source must be offline/camera/ros");
    c.width = num(in, "width", 2, 8192);
    c.height = num(in, "height", 2, 8192);
    if ((c.width | c.height) & 1)
        throw std::invalid_argument("YUV dimensions must be even");
    const auto &off = j::required(in, "offline");
    j::fields(off, {"manifest", "intrinsics"});
    c.manifest = resolve(c.base, str(off, "manifest"));
    c.intrinsics = resolve(c.base, str(off, "intrinsics"));
    const auto &cam = j::required(in, "camera");
    j::fields(cam, {"sdk_library", "vendor_config"});
    c.camera_sdk = resolve(c.base, str(cam, "sdk_library")).u8string();
    c.camera_config = resolve(c.base, str(cam, "vendor_config")).u8string();
    const auto &m = j::required(c.root, "models");
    j::fields(m, {"detection", "segmentation", "stereo"});
    c.det = resolve(c.base, str(m, "detection"));
    c.seg = resolve(c.base, str(m, "segmentation"));
    c.stereo = resolve(c.base, str(m, "stereo"));
    const auto &p = j::required(c.root, "pipeline");
    j::fields(p, {"mapping_config", "extrinsics", "sample_step", "retain_clouds", "render_maps", "run_mapping"});
    c.mapping = resolve(c.base, str(p, "mapping_config"));
    c.extrinsics = resolve(c.base, str(p, "extrinsics"));
    c.sample_step = num(p, "sample_step", 1, 8192);
    c.retain_clouds = j::boolean(j::required(p, "retain_clouds"));
    c.render_maps = j::boolean(j::required(p, "render_maps"));
    if (auto v = j::optional(p, "run_mapping")) c.run_mapping = j::boolean(*v);
    const auto &e = j::required(c.root, "experiment");
    j::fields(
        e, {"methods", "suites", "frames", "warmup", "save_every_n", "timeout_ms", "stereo_roi"});
    for (auto &x : j::required(e, "methods").array) {
        auto s = j::string(x);
        if (s != "original" && s != "optimized_q8" && s != "optimized_stereo")
            throw std::invalid_argument("Unknown method");
        if (std::find(c.methods.begin(), c.methods.end(), s) != c.methods.end())
            throw std::invalid_argument("Duplicate method");
        c.methods.push_back(s);
    }
    for (auto &x : j::required(e, "suites").array) {
        auto s = j::string(x);
        if (s != "native" && s != "matched_roi")
            throw std::invalid_argument("Unknown suite");
        if (std::find(c.suites.begin(), c.suites.end(), s) != c.suites.end())
            throw std::invalid_argument("Duplicate suite");
        c.suites.push_back(s);
    }
    if (c.methods.empty() || c.suites.empty())
        throw std::invalid_argument("methods/suites must be nonempty arrays");
    c.frames = num(e, "frames", 1, 1000000);
    c.warmup = num(e, "warmup", 0, 1000000);
    c.save_every = num(e, "save_every_n", 0, 1000000);
    c.timeout_ms = num(e, "timeout_ms", 100, 600000);
    auto &r = j::required(e, "stereo_roi");
    j::fields(r, {"x", "y", "width", "height"});
    c.roi = {num(r, "x"), num(r, "y"), num(r, "width", 2), num(r, "height", 2)};
    if (c.roi.x != 0 || c.roi.w != c.width || c.roi.y + c.roi.h != c.height ||
        (c.roi.y | c.roi.h) & 1)
        throw std::invalid_argument(
            "Stereo ROI must be an even bottom full-width crop inside source");
    auto &ros = j::required(c.root, "ros");
    j::fields(ros,
              {"left_topic", "right_topic", "q8_topic", "left_info_topic", "right_info_topic",
               "reliable", "cache_size", "publish_hz", "transport", "bridge_host", "bridge_port", "acknowledged_replay", "ack_topic"});
    if (auto v = j::optional(ros, "transport"))
        c.ros_transport = j::string(*v);
    if (c.ros_transport != "dds" && c.ros_transport != "tcp")
        throw std::invalid_argument("ROS transport must be dds or tcp");
    if (auto v = j::optional(ros, "bridge_host"))
        c.bridge_host = j::string(*v);
    if (auto v = j::optional(ros, "bridge_port"))
        c.bridge_port = j::integer(*v, 1024, 65535);
    if (auto v = j::optional(ros, "acknowledged_replay")) c.ros_acknowledged = j::boolean(*v);
    if (auto v = j::optional(ros, "ack_topic")) c.ros_ack_topic = j::string(*v);
    c.left_topic = str(ros, "left_topic");
    c.right_topic = str(ros, "right_topic");
    c.q8_topic = str(ros, "q8_topic");
    c.left_info_topic = str(ros, "left_info_topic");
    c.right_info_topic = str(ros, "right_info_topic");
    c.ros_reliable = j::boolean(j::required(ros, "reliable"));
    c.cache_size = num(ros, "cache_size", 2, 4096);
    c.ros_publish_hz = real(ros, "publish_hz");
    if (c.ros_publish_hz <= 0)
        throw std::invalid_argument("publish_hz must be positive");
    if (c.run_mapping && c.save_every && !c.retain_clouds)
        throw std::invalid_argument("PLY saving requires retain_clouds=true");
    return c;
}
inline Roi region(const Config &c, const std::string &method, const std::string &suite) {
    return method == "optimized_stereo" || suite == "matched_roi" ? c.roi
                                                                  : Roi{0, 0, c.width, c.height};
}
inline double quantile(std::vector<double> a, double q) {
    if (a.empty())
        throw std::invalid_argument("empty quantile");
    std::sort(a.begin(), a.end());
    double i = (a.size() - 1) * q;
    auto lo = std::size_t(std::floor(i)), hi = std::size_t(std::ceil(i));
    return a[lo] + (a[hi] - a[lo]) * (i - lo);
}
inline std::string stats(const std::vector<double> &a) {
    if (a.empty())
        return "null";
    std::ostringstream s;
    s.precision(15);
    s << "{\"count\":" << a.size()
      << ",\"mean\":" << std::accumulate(a.begin(), a.end(), 0.0) / a.size()
      << ",\"p95\":" << quantile(a, .95) << ",\"max\":" << *std::max_element(a.begin(), a.end())
      << '}';
    return s.str();
}
} // namespace ab
