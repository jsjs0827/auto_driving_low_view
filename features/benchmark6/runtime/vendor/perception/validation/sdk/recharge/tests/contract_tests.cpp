#include "bridge.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void requireEqual(double expected, double actual, const char* message) {
    require((std::isnan(expected) && std::isnan(actual)) || expected == actual, message);
}
}  // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 2, "usage: r33_recharge_contract_tests recharge_mark.json");
        constexpr int width = 64, height = 48, stride = 80;
        const std::size_t y_bytes = stride * height;
        std::vector<std::uint8_t> nv12(y_bytes * 3u / 2u, 128);
        std::fill(nv12.begin(), nv12.begin() + y_bytes, 16);
        // Padding differs from image pixels to catch assumptions of tight stride.
        for (int y = 0; y < height; ++y) {
            std::fill(nv12.begin() + y * stride + width,
                      nv12.begin() + (y + 1) * stride, 255);
        }

        // Save the direct upstream synchronous result before acquiring Engine's
        // global session. This checks adapter fidelity without a fake detector.
        Aiper_Recharge::MarkPoseResult expected;
        {
            std::unique_ptr<Aiper_Recharge::RechargeMarkApi> api(
                Aiper_Recharge::createRechargeMarkApi());
            require(api->loadConfig(argv[1]) == 0, "upstream configuration load");
            api->useSdkCameraMatrix();
            api->resetSession();
            auto* result = api->processLeftNv12(nv12.data(), width, height, stride, 78);
            require(result != nullptr, "upstream blank frame result");
            expected = *result;
            expected.overlay_bgr = nullptr;
            api->freeResult(result);
            api->destroy(false);
        }

        r33::recharge::Engine waiting;
        std::string error;
        {
            r33::recharge::Engine engine;
            require(!engine.initialize("this_config_does_not_exist.json", error),
                    "missing config must fail");
            require(!error.empty(), "missing config diagnostic");
            require(engine.initialize(argv[1], error), "Engine configuration load");
            require(error.empty(), "successful initialization clears diagnostic");
            require(!waiting.initialize(argv[1], error), "reject second process-global Engine");
            require(error.find("process-global") != std::string::npos,
                    "singleton restriction is explicit");

            Aiper_Recharge::RechargeHostConfig host;
            engine.getHostConfig(host);
            require(host.publish_enabled && host.use_hires && host.run_every_n == 1,
                    "original host settings retained");
            require(host.min_interval_ms == 150,
                    "original cadence and crop settings retained");

            r33::recharge::Output output;
            const auto process = [&](const std::uint8_t* data, std::size_t size,
                                     int w, int h, int pitch) {
                return engine.processLeftNv12(data, size, w, h, pitch, 78,
                                              1234567890123LL, nullptr, output, error);
            };
            require(!process(nullptr, nv12.size(), width, height, stride), "null NV12 rejected");
            require(!process(nv12.data(), nv12.size() - 1, width, height, stride),
                    "short NV12 rejected");
            require(!process(nv12.data(), nv12.size(), width - 1, height, stride),
                    "odd NV12 width rejected");
            require(!process(nv12.data(), nv12.size(), width, height - 1, stride),
                    "odd NV12 height rejected");
            require(!process(nv12.data(), nv12.size(), width, height, width - 1),
                    "short NV12 stride rejected");
            require(process(nv12.data(), nv12.size(), width, height, stride),
                    "valid padded NV12 must finish even without a marker");
            require(error.empty(), "no-detection is a completed call");
            require(output.pose.frame_id == 78 && output.pose.post.time_stamp == 1234567890123LL,
                    "result remains attached to the same input frame and timestamp");
            require(output.pose.ok == expected.ok && output.pose.pose_ok == expected.pose_ok,
                    "original detection flags retained");
            require(output.pose.detected_point_count == expected.detected_point_count &&
                    output.pose.debug_candidates == expected.debug_candidates &&
                    output.pose.debug_scaffolds == expected.debug_scaffolds,
                    "original detector metadata retained");
            require(std::string(output.pose.error) == expected.error &&
                    std::string(output.pose.pnp_error) == expected.pnp_error &&
                    std::string(output.pose.crop_src) == expected.crop_src,
                    "original diagnostics and crop retained");
            requireEqual(expected.image_angle_deg, output.pose.image_angle_deg, "original image angle");
            requireEqual(expected.score, output.pose.score, "original detector score");
            requireEqual(expected.post.x, output.pose.post.x, "original DockPose x");
            requireEqual(expected.post.y, output.pose.post.y, "original DockPose y");
            requireEqual(expected.post.yaw, output.pose.post.yaw, "original DockPose yaw");
            require(output.pose.overlay_bgr == nullptr, "raw overlay pointer never escapes ownership");

            require(!engine.setCameraMatrix(NAN, 718, 320, 240, error),
                    "invalid intrinsics rejected");
            require(engine.setCameraMatrix(718, 719, 320, 240, error),
                    "finite frame intrinsics accepted");
            engine.resetSession();
            require(engine.processLeftNv12(nv12.data(), nv12.size(), width, height,
                                           stride, 79, 0, nullptr, output, error),
                    "second synchronous frame");
            require(output.pose.frame_id == 79 && output.pose.post.time_stamp > 0,
                    "fallback monotonic timestamp without stale previous frame");
        }
        require(waiting.initialize(argv[1], error), "singleton released after destruction");
        std::cout << "recharge contract: passed (original parity, config, ownership, validation, singleton)\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& exception) {
        std::cerr << "recharge contract failed: " << exception.what() << '\n';
        return EXIT_FAILURE;
    }
}
