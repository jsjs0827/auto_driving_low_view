#include "bridge.hpp"
#include "v0_runtime.hpp"
#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <unistd.h>
#include <filesystem>
#include <fstream>
#include <cstring>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace fs = std::filesystem;
namespace {
void require(bool value, const char* error) { if (!value) throw std::runtime_error(error); }
bool bits(float a, float b) { return std::memcmp(&a, &b, sizeof(float)) == 0; }
struct Oracle {
    Aiper_Recharge::RechargeMarkApi* api = nullptr;
    void* lease = nullptr;
    ~Oracle() { if (lease) r36_recharge_v0_close(lease); }
};
}
int main(int argc, char** argv) {
    try {
        require(argc == 3, "usage: V0 contract ORIGINAL_CONFIG OWNED_OUTPUT_PARENT");
        const auto output = fs::absolute(argv[2]) / ("run-" + std::to_string(getpid()));
        require(!fs::exists(output), "contract output must be new");
        fs::create_directories(output);
        std::ifstream source(argv[1]);
        require(bool(source), "read original config");
        const std::string text((std::istreambuf_iterator<char>(source)), {});
        rapidjson::Document config;
        config.Parse(text.c_str());
        require(!config.HasParseError() && config.IsObject(), "valid original configuration");
        auto& a = config.GetAllocator();
        config["save_raw"]["enabled"].SetBool(false);
        config["save_raw"]["dir"].SetString((output / "raw").c_str(), a);
        config["save_result"]["dir"].SetString((output / "result").c_str(), a);
        if (config["save_result"].HasMember("enabled")) config["save_result"]["enabled"].SetBool(false);
        else config["save_result"].AddMember("enabled", false, a);
        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer); config.Accept(writer);
        const auto cfg = output / "recharge.json";
        { std::ofstream file(cfg); file << buffer.GetString(); }
        constexpr int width = 64, height = 48, stride = 80;
        std::vector<std::uint8_t> nv12(stride * height * 3 / 2, 128);
        std::fill(nv12.begin(), nv12.begin() + stride * height, 16);
        Aiper_Recharge::MarkPoseResult expected;
        {
            Oracle oracle;
            char error[1024]{};
            require(r36_recharge_v0_open(&oracle.api, &oracle.lease, error, sizeof(error)) == 0, error);
            require(oracle.api->init("", cfg.c_str()) == 0, "real V0 oracle init");
            oracle.api->setSaveResultEnable(false);
            oracle.api->resetSession();
            auto* result = oracle.api->processLeftNv12(nv12.data(), width, height, stride, 78);
            require(result != nullptr, "real V0 oracle result");
            expected = *result; expected.overlay_bgr = nullptr;
            oracle.api->freeResult(result);
        }
        r33::recharge::Engine waiting;
        std::string error;
        {
            r33::recharge::Engine engine;
            require(!engine.initialize("missing.json", error), "missing configuration rejected");
            require(engine.initialize(cfg.c_str(), error), error.c_str());
            require(!waiting.initialize(cfg.c_str(), error), "second global session rejected");
            Aiper_Recharge::RechargeHostConfig host;
            engine.getHostConfig(host);
            require(host.run_every_n == config["run_every_n"].GetUint() &&
                    host.min_interval_ms == config["min_interval_ms"].GetUint() &&
                    host.use_hires == config["use_hires"].GetBool(), "real original configuration retained");
            r33::recharge::Output actual;
            require(!engine.processLeftNv12(nullptr, nv12.size(), width, height, stride, 78, 1, nullptr, actual, error), "null rejected");
            require(!engine.processLeftNv12(nv12.data(), nv12.size() - 1, width, height, stride, 78, 1, nullptr, actual, error), "short buffer rejected");
            require(!engine.processLeftNv12(nv12.data(), nv12.size(), width - 1, height, stride, 78, 1, nullptr, actual, error), "odd dimension rejected");
            require(engine.processLeftNv12(nv12.data(), nv12.size(), width, height, stride, 78, 1234567890123LL, nullptr, actual, error), error.c_str());
            require(actual.pose.ok == expected.ok && actual.pose.pose_ok == expected.pose_ok &&
                    actual.pose.detected_point_count == expected.detected_point_count, "real oracle flags retained");
            require(bits(actual.pose.post.x, expected.post.x) && bits(actual.pose.post.y, expected.post.y) &&
                    bits(actual.pose.post.yaw, expected.post.yaw), "post FP32 bits retained");
            require(actual.pose.frame_id == 78 && actual.pose.post.time_stamp == 1234567890123LL, "DAG routing identity retained");
            require(actual.pose.overlay_bgr == nullptr, "no dangling overlay pointer");
            require(!engine.setCameraMatrix(NAN, 718, 320, 240, error), "bad camera matrix rejected");
            require(engine.setCameraMatrix(718, 719, 320, 240, error), "existing camera override API preserved");
            require(engine.initialize(cfg.c_str(), error), "reinitialization releases old API and reloads original configuration");
        }
        require(waiting.initialize(cfg.c_str(), error), "global session released after destruction");
        std::cout << "PASS real V0 API/Engine ownership, lifecycle, input validation and blank-result transport; no accuracy claim\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; return 1;
    }
}
