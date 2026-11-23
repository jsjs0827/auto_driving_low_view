#include "ab/runtime.hpp"
#include "perception_ros_so_demo/mo_camera.hpp"
#include "r36_stereo/nv12.hpp"
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <cstring>
namespace ab {
std::atomic<bool> running{true};
static std::vector<std::uint8_t> bytes(const fs::path &p, std::size_t n) {
    if (!fs::is_regular_file(p) || fs::file_size(p) != n)
        throw std::invalid_argument("Wrong raw byte count: " + p.u8string());
    std::vector<std::uint8_t> v(n);
    std::ifstream f(p, std::ios::binary);
    f.read(reinterpret_cast<char *>(v.data()), std::streamsize(n));
    if (!f)
        throw std::runtime_error("Read failed: " + p.u8string());
    return v;
}
cv::Mat loadImage(const fs::path &p, int w, int h) {
    auto ext = p.extension().u8string();
    if (ext == ".nv12" || ext == ".i420") {
        auto data = bytes(p, std::size_t(w) * h * 3 / 2);
        return r36stereo::copy_yuv420_to_nv12(data.data(), w, h, w, data.size(), ext == ".nv12");
    }
    auto bgr = cv::imread(p.u8string(), cv::IMREAD_COLOR);
    if (bgr.empty() || bgr.cols != w || bgr.rows != h)
        throw std::invalid_argument("Offline rectified image must match calibration " +
                                    std::to_string(w) + "x" + std::to_string(h) + ": " +
                                    p.u8string());
    return r36stereo::bgr_to_nv12(bgr);
}
std::vector<std::uint16_t> readQ8(const fs::path &p, int w, int h) {
    auto b = bytes(p, std::size_t(w) * h * 2);
    std::vector<std::uint16_t> out(std::size_t(w) * h);
    for (std::size_t i = 0; i < out.size(); ++i)
        out[i] = std::uint16_t(b[i * 2]) | (std::uint16_t(b[i * 2 + 1]) << 8);
    return out;
}
void maskQ8(std::vector<std::uint16_t> &q, int w, int h, Roi r) {
    if (q.size() != std::size_t(w) * h || r.x < 0 || r.y < 0 || r.x + r.w > w || r.y + r.h > h)
        throw std::invalid_argument("Q8 geometry/ROI mismatch");
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            if (x < r.x || x >= r.x + r.w || y < r.y || y >= r.y + r.h)
                q[std::size_t(y) * w + x] = 0;
}
class Offline final : public Source {
    Config c_;
    bool need_;
    Intrinsics k_;
    Json manifest_;
    std::size_t next_ = 0;

  public:
    Offline(const Config &c, bool n) : c_(c), need_(n) {
        auto doc = j::load(c.intrinsics.u8string());
        j::fields(doc, {"width", "height", "fx", "fy", "cx", "cy", "bxf", "bxf_unit"});
        if (str(doc, "bxf_unit") != "pixel_mm")
            throw std::invalid_argument("intrinsics.bxf_unit must be pixel_mm");
        k_ = {num(doc, "width", 2), num(doc, "height", 2), real(doc, "fx"), real(doc, "fy"),
              real(doc, "cx"),      real(doc, "cy"),       real(doc, "bxf")};
        validate(k_);
        if (k_.w != c.width || k_.h != c.height)
            throw std::invalid_argument(
                "Offline calibration dimensions differ from configured images");
        manifest_ = j::load(c.manifest.u8string());
        j::fields(manifest_, {"schema_version", "frames"});
        if (num(manifest_, "schema_version", 1, 1) != 1)
            throw std::invalid_argument("Invalid manifest schema");
        auto &frames = j::required(manifest_, "frames");
        if (frames.kind != Json::Kind::Array ||
            frames.array.size() < std::size_t(c.frames + c.warmup))
            throw std::invalid_argument(
                "Manifest needs at least warmup+frames unique captures; no silent repetition");
        std::set<std::uint64_t> stamps;
        for (auto &f : frames.array) {
            j::fields(f, {"key", "frame_id", "timestamp_ns", "left", "right", "q8"});
            auto &stamp = j::required(f, "timestamp_ns");
            if (stamp.kind != Json::Kind::String)
                throw std::invalid_argument(
                    "Manifest timestamps are decimal strings, avoiding JSON precision loss");
            auto ns = u64(j::string(stamp));
            if (!ns || !stamps.insert(ns).second)
                throw std::invalid_argument("Manifest timestamps must be nonzero and unique");
            if (need_ && str(f, "q8").empty())
                throw std::invalid_argument("Original/optimized_q8 requires actual same-frame Q8; "
                                            "cannot substitute Stereo output");
        }
    }
    Frame next() override {
        auto &a = j::required(manifest_, "frames").array;
        if (next_ >= a.size())
            throw std::runtime_error("Offline dataset exhausted");
        auto &f = a[next_++];
        Frame o;
        o.id = u64(str(f, "frame_id"));
        o.ns = u64(str(f, "timestamp_ns"));
        o.key = str(f, "key");
        o.k = k_;
        auto base = c_.manifest.parent_path();
        o.left = loadImage(resolve(base, str(f, "left")), k_.w, k_.h);
        o.right = loadImage(resolve(base, str(f, "right")), k_.w, k_.h);
        if (need_)
            o.q8 = readQ8(resolve(base, str(f, "q8")), k_.w, k_.h);
        return o;
    }
    std::string metadata() const override {
        return "{\"kind\":\"offline\",\"paired\":true,\"q8_generation_included\":false,\"timestamp_"
               "policy\":\"original_capture_preserved\"}";
    }
};
class CameraSource final : public Source {
    ros_so_demo::raw::Camera camera_;
    Intrinsics k_;
    bool need_;
    std::uint64_t last_ = 0;
    static ros_so_demo::raw::CameraOptions options(const Config &c, bool n) {
        ros_so_demo::raw::CameraOptions o;
        o.sdk = c.camera_sdk;
        o.config = c.camera_config;
        o.need_q8 = n;
        o.disparity_mode = n ? 2 : 1;
        o.timeout_ms = c.timeout_ms;
        return o;
    }

  public:
    CameraSource(const Config &c, bool n) : camera_(options(c, n)), need_(n) {
        auto p = camera_.parameters();
        if (!((p.width == 640 && p.height == 544) || (p.width == 1280 && p.height == 1088)))
            throw std::runtime_error("SDK rectified reference must be verified 640x544 or 1280x1088");
        double sx = double(c.width) / p.width, sy = double(c.height) / p.height;
        k_ = {c.width,
              c.height,
              p.rectified.fx * sx,
              p.rectified.fy * sy,
              p.rectified.cx * sx,
              p.rectified.cy * sy,
              p.bxf * sx};
        validate(k_);
    }
    Frame next() override {
        while (running) {
            auto f = camera_.next([] { return running.load(); });
            if (f.left.capture.ns <= last_)
                continue;
            last_ = f.left.capture.ns;
            Frame o;
            o.id = f.left.capture.id;
            o.ns = f.left.capture.ns;
            o.key = "capture-" + std::to_string(o.id);
            o.k = k_;
            o.left = r36stereo::copy_yuv420_to_nv12(f.left.bytes.data(), k_.w, k_.h, f.left.step,
                                                    f.left.bytes.size(), true);
            o.right = r36stereo::copy_yuv420_to_nv12(f.right.bytes.data(), k_.w, k_.h, f.right.step,
                                                     f.right.bytes.size(), true);
            if (need_) {
                if (f.disparity.bytes.size() != std::size_t(k_.w) * k_.h * 2)
                    throw std::runtime_error("SDK Q8 byte mismatch");
                o.q8.resize(std::size_t(k_.w) * k_.h);
                std::memcpy(o.q8.data(), f.disparity.bytes.data(), f.disparity.bytes.size());
            }
            return o;
        }
        throw std::runtime_error("Camera interrupted");
    }
    std::string metadata() const override {
        return std::string("{\"kind\":\"camera\",\"paired_between_runs\":false,\"intrinsics_"
                           "source\":\"camera_sdk_rectified\",\"camera_mode\":") +
               (need_ ? "2" : "1") +
               ",\"q8_producer_inside_process\":" + (need_ ? "true" : "false") + "}";
    }
};
std::unique_ptr<Source> offlineSource(const Config &c, bool q) {
    return std::make_unique<Offline>(c, q);
}
std::unique_ptr<Source> cameraSource(const Config &c, bool q) {
    if (c.width != 640 || c.height != 544)
        throw std::invalid_argument("Current vendor adapter requires 640x544");
    return std::make_unique<CameraSource>(c, q);
}
std::unique_ptr<Source> rosSource(const Config &c, bool need) {
    if (c.ros_transport == "tcp")
        return tcpSource(c, need);
    return nativeRosSource(c, need);
}
#ifndef AB_WITH_ROS
std::unique_ptr<Source> nativeRosSource(const Config &, bool) {
    throw std::runtime_error("Native DDS was not compiled; use ros.transport=tcp with Ubuntu C++ "
                             "ROS bridge, or build AB_WITH_ROS=ON");
}
void rosPublish(const Config &, const std::string &, bool) {
    throw std::runtime_error("ROS was not compiled");
}
#endif
} // namespace ab
