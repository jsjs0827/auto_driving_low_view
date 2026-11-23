#include "bridge.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
struct Frame {
    int width = 0, height = 0, stride = 0;
    std::vector<std::uint8_t> nv12;
};

std::uint8_t clamp(int value) {
    return static_cast<std::uint8_t>(std::max(0, std::min(255, value)));
}

Frame loadPpm(const char* filename) {
    std::ifstream file(filename, std::ios::binary);
    std::string magic;
    int maximum = 0;
    Frame frame;
    if (!(file >> magic >> frame.width >> frame.height >> maximum) || magic != "P6" ||
        maximum != 255 || frame.width <= 0 || frame.height <= 0 ||
        frame.width > 4096 || frame.height > 4096 || (frame.width & 1) || (frame.height & 1)) {
        throw std::runtime_error(std::string("unsupported P6 PPM: ") + filename);
    }
    const char separator = static_cast<char>(file.get());
    if (separator == '\r' && file.peek() == '\n') file.get();
    const auto pixels = static_cast<std::size_t>(frame.width) * frame.height;
    std::vector<std::uint8_t> rgb(pixels * 3u);
    if (!file.read(reinterpret_cast<char*>(rgb.data()), static_cast<std::streamsize>(rgb.size()))) {
        throw std::runtime_error(std::string("truncated PPM: ") + filename);
    }
    frame.stride = frame.width + 16;  // Exercise actual pitch/padding handling.
    const auto y_bytes = static_cast<std::size_t>(frame.stride) * frame.height;
    frame.nv12.assign(y_bytes * 3u / 2u, 128);
    for (int y = 0; y < frame.height; ++y) {
        for (int x = 0; x < frame.width; ++x) {
            const auto i = (static_cast<std::size_t>(y) * frame.width + x) * 3u;
            const int r = rgb[i], g = rgb[i + 1], b = rgb[i + 2];
            frame.nv12[static_cast<std::size_t>(y) * frame.stride + x] =
                clamp(16 + ((66 * r + 129 * g + 25 * b + 128) >> 8));
        }
    }
    for (int y = 0; y < frame.height; y += 2) {
        for (int x = 0; x < frame.width; x += 2) {
            int r = 0, g = 0, b = 0;
            for (int dy = 0; dy < 2; ++dy) {
                for (int dx = 0; dx < 2; ++dx) {
                    const auto i = (static_cast<std::size_t>(y + dy) * frame.width + x + dx) * 3u;
                    r += rgb[i]; g += rgb[i + 1]; b += rgb[i + 2];
                }
            }
            r = (r + 2) / 4; g = (g + 2) / 4; b = (b + 2) / 4;
            const auto uv = y_bytes + static_cast<std::size_t>(y / 2) * frame.stride + x;
            frame.nv12[uv] = clamp(128 + ((-38 * r - 74 * g + 112 * b + 128) >> 8));
            frame.nv12[uv + 1] = clamp(128 + ((112 * r - 94 * g - 18 * b + 128) >> 8));
        }
    }
    return frame;
}

template <typename T>
bool same(T a, T b) { return a == b; }
template <>
bool same(double a, double b) { return a == b || (std::isnan(a) && std::isnan(b)); }
template <>
bool same(float a, float b) { return a == b || (std::isnan(a) && std::isnan(b)); }

void compare(const r33::recharge::Output& expected, const r33::recharge::Output& actual) {
    const auto& a = expected.pose;
    const auto& b = actual.pose;
#define FIELD(name) if (!same(a.name, b.name)) throw std::runtime_error("result differs: " #name)
    FIELD(ok); FIELD(pose_ok); FIELD(frame_id); FIELD(image_w); FIELD(image_h);
    FIELD(detected_point_count); FIELD(center_u); FIELD(center_v); FIELD(image_angle_deg); FIELD(score);
    FIELD(tl_u); FIELD(tl_v); FIELD(tr_u); FIELD(tr_v); FIELD(br_u); FIELD(br_v); FIELD(bl_u); FIELD(bl_v);
    FIELD(top_u); FIELD(top_v); FIELD(bot_u); FIELD(bot_v);
    FIELD(has_tl); FIELD(has_tr); FIELD(has_br); FIELD(has_bl); FIELD(has_top); FIELD(has_bot);
    FIELD(distance_mm); FIELD(reprojection_rmse_px); FIELD(roll_x_deg); FIELD(pitch_y_deg);
    FIELD(yaw_z_deg); FIELD(heading_y_deg); FIELD(tilt_x_deg); FIELD(image_roll_z_deg);
    FIELD(post.x); FIELD(post.y); FIELD(post.yaw);
    FIELD(debug_candidates); FIELD(debug_scaffolds); FIELD(overlay_w); FIELD(overlay_h); FIELD(overlay_step);
    FIELD(strip_l2); FIELD(strip_dx_m); FIELD(strip_du_px); FIELD(strip_ms);
    FIELD(trap_sign); FIELD(dist_jump_hold); FIELD(yaw_jump_hold); FIELD(cam_x_mm); FIELD(raw_x_m);
    for (int i = 0; i < 3; ++i) { FIELD(tvec_mm[i]); FIELD(rvec[i]); }
#undef FIELD
    if (std::string(a.error) != b.error || std::string(a.pnp_error) != b.pnp_error ||
        std::string(a.strip_status) != b.strip_status ||
        std::string(a.x_filter_reason) != b.x_filter_reason || std::string(a.crop_src) != b.crop_src) {
        throw std::runtime_error("original diagnostics/filter/crop source differs");
    }
    if (expected.overlay_bgr != actual.overlay_bgr) throw std::runtime_error("overlay pixels differ");
}

r33::recharge::CropHint cropFor(const Frame& frame, std::size_t index) {
    if ((index & 1u) == 0) return {};
    return {frame.width / 8, frame.height / 6, frame.width * 3 / 4,
            frame.height * 4 / 5, 1};
}
}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc < 3) throw std::runtime_error("usage: r33_recharge_sequence_parity config.json image.ppm [...]");
        std::vector<Frame> frames;
        for (int i = 2; i < argc; ++i) frames.push_back(loadPpm(argv[i]));
        std::vector<r33::recharge::Output> expected;
        {
            std::unique_ptr<Aiper_Recharge::RechargeMarkApi> api(Aiper_Recharge::createRechargeMarkApi());
            if (api->loadConfig(argv[1]) != 0) throw std::runtime_error("original config failed");
            api->useSdkCameraMatrix();
            api->resetSession();
            for (std::size_t i = 0; i < frames.size(); ++i) {
                const auto& f = frames[i];
                const auto crop = cropFor(f, i);
                // Replacement donor uses its configured ROI, with no external crop.
                auto* result = api->processLeftNv12(f.nv12.data(), f.width, f.height, f.stride, i + 1);
                if (!result) throw std::runtime_error("null original result");
                r33::recharge::Output saved;
                saved.pose = *result;
                saved.pose.overlay_bgr = nullptr;
                if (result->overlay_bgr) {
                    saved.overlay_bgr.assign(result->overlay_bgr, result->overlay_bgr +
                        static_cast<std::size_t>(result->overlay_step) * result->overlay_h);
                }
                expected.push_back(std::move(saved));
                api->freeResult(result);
            }
            api->destroy(false);
        }
        r33::recharge::Engine engine;
        std::string error;
        if (!engine.initialize(argv[1], error)) throw std::runtime_error(error);
        std::size_t detections = 0, poses = 0;
        for (std::size_t i = 0; i < frames.size(); ++i) {
            const auto& f = frames[i];
            const auto crop = cropFor(f, i);
            r33::recharge::Output actual;
            const auto timestamp = 123450000000LL + static_cast<std::int64_t>(i) * 150000000LL;
            if (!engine.processLeftNv12(f.nv12.data(), f.nv12.size(), f.width, f.height,
                                        f.stride, i + 1, timestamp, nullptr, actual, error)) {
                throw std::runtime_error(error);
            }
            compare(expected[i], actual);
            const auto name=std::filesystem::path(argv[i+2]).stem().string();
            const bool negative=name=="blank"||name=="wrong_id";
            if(negative && (actual.pose.ok||actual.pose.pose_ok))throw std::runtime_error("negative fixture produced pose");
            if(!negative && (!actual.pose.ok||!actual.pose.pose_ok))throw std::runtime_error("positive fixture did not produce pose: "+name);
            if(!negative && (name=="dual"||name=="offset"||name=="dual_repeat"||name=="small_dual") && actual.pose.detected_point_count!=8)
                throw std::runtime_error("dual fixture must use both markers (8 corners)");
            if(actual.overlay_bgr.empty())throw std::runtime_error("overlay ownership parity was not exercised");
            if (actual.pose.post.time_stamp != timestamp) throw std::runtime_error("timestamp mismatch");
            detections += actual.pose.ok;
            poses += actual.pose.pose_ok;
            std::cout << "frame=" << i + 1 << " size=" << f.width << 'x' << f.height
                      << " ok=" << actual.pose.ok << " pose=" << actual.pose.pose_ok
                      << " crop=" << actual.pose.crop_src << " parity=exact\n";
        }
        std::cout << "sequence parity: passed frames=" << frames.size() << " detections="
                  << detections << " poses=" << poses
                  << " (all original scalar results, filter/crop diagnostics, and overlay pixels; excluding timing)\n";
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "sequence parity failed: " << exception.what() << '\n';
        return 1;
    }
}
