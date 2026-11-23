#include "perception_full_sdk.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
namespace fs = std::filesystem;
void require(bool value, const char* detail) {
    if (!value) throw std::runtime_error(detail);
}
void success(std::uint32_t code, const psdk_status& status, const char* operation) {
    if (code) throw std::runtime_error(std::string(operation) + ": code=" +
        std::to_string(code) + " native=" + std::to_string(status.native_code) + " " + status.detail);
}
struct Engine {
    psdk_full_engine* handle = nullptr;
    ~Engine() { psdk_full_destroy(&handle, nullptr); }
};
struct Result {
    psdk_full_result* handle = nullptr;
    ~Result() { psdk_full_result_release(handle); }
};
psdk_full_result_view view(const Result& result) {
    psdk_full_result_view value{};
    value.struct_size = sizeof(value); value.abi_version = PSDK_FULL_ABI_VERSION;
    psdk_status status{};
    success(psdk_full_result_get_view(result.handle, &value, &status), status, "get full view");
    return value;
}
std::uint64_t hash(const std::uint8_t* data, std::uint64_t bytes) {
    std::uint64_t value = 1469598103934665603ULL;
    for (std::uint64_t i = 0; i < bytes; ++i) value = (value ^ data[i]) * 1099511628211ULL;
    return value;
}
psdk_full_config config(const char* path, const psdk_full_hooks* hooks = nullptr) {
    psdk_full_config value{};
    value.struct_size = sizeof(value); value.abi_version = PSDK_FULL_ABI_VERSION;
    value.json_file = path; value.hooks = hooks;
    return value;
}

constexpr std::uint32_t callback_error = 0xf123abcdu;
struct CallbackState {
    enum Mode { Capture, Reenter, Fail } mode = Capture;
    psdk_full_engine* engine = nullptr;
    std::uint64_t expected_id = 0, expected_stamp = 0;
    std::uint32_t calls = 0, reentry_code = 0;
    bool identity_ok = true, reentry_status_ok = false;
    double reentry_ms = -1;
};
std::uint32_t publish(void* userdata, const psdk_full_result_view* result, psdk_status* status) {
    auto& state = *static_cast<CallbackState*>(userdata);
    ++state.calls;
    state.identity_ok = state.identity_ok && result && result->frame_id == state.expected_id &&
                        result->timestamp_ns == state.expected_stamp && result->has_inference;
    if (state.mode == CallbackState::Fail) {
        *status = {};
        status->struct_size = sizeof(*status); status->abi_version = PSDK_ABI_VERSION;
        status->code = callback_error; status->native_code = -321;
        // Intentionally no terminators: a public callback can supply all bytes.
        std::memset(status->feature, 'F', sizeof(status->feature));
        std::memset(status->task, 'T', sizeof(status->task));
        std::memset(status->detail, 'D', sizeof(status->detail));
        return callback_error;
    }
    if (state.mode == CallbackState::Reenter) {
        psdk_status nested{};
        const auto begin = std::chrono::steady_clock::now();
        state.reentry_code = psdk_full_command(state.engine, 21, &nested);
        state.reentry_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - begin).count();
        state.reentry_status_ok = nested.code == state.reentry_code &&
                                  std::string(nested.feature) == "full.callback";
    }
    return 0;
}

std::vector<std::uint8_t> readInput(const char* path) {
    constexpr std::size_t bytes = 640u * 544u * 3u / 2u;
    require(fs::file_size(fs::u8path(path)) == bytes, "input must be640x544 tight NV12");
    std::vector<std::uint8_t> input(bytes);
    std::ifstream file(fs::u8path(path), std::ios::binary);
    require(bool(file.read(reinterpret_cast<char*>(input.data()), input.size())), "complete input read");
    return input;
}
}  // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 5, "usage: full_sdk_contract inference.json recharge.json input.nv12 NEW_OUTPUT_DIR");
        require(psdk_abi_version() == PSDK_ABI_VERSION, "base ABI version");
        require((psdk_full_capabilities() & PSDK_CAP_CPU_ORT) != 0, "requires actual CPU ONNX Runtime build");
        require((psdk_full_capabilities() & PSDK_CAP_RECHARGE) != 0, "requires actual compiled recharge algorithm");
        const auto out = fs::u8path(argv[4]);
        require(!fs::exists(out) && fs::create_directories(out), "output must be a NEW directory");
        auto input = readInput(argv[3]);
        psdk_full_frame frame{};
        frame.struct_size = sizeof(frame); frame.abi_version = PSDK_FULL_ABI_VERSION;
        frame.left.struct_size = sizeof(frame.left); frame.left.abi_version = PSDK_ABI_VERSION;
        frame.left.format = PSDK_FORMAT_NV12; frame.left.width = 640; frame.left.height = 544;
        frame.left.stride = 640; frame.left.data = input.data(); frame.left.bytes = input.size();
        frame.left.frame_id = 901; frame.left.timestamp_ns = 1234567891000ULL;
        frame.intrinsics = {640, 544, 298.404816, 298.404816, 313.862183, 274.359955, 17935.5};
        psdk_status status{};
        CallbackState callbacks;
        psdk_full_hooks hooks{};
        hooks.struct_size = sizeof(hooks); hooks.abi_version = PSDK_FULL_ABI_VERSION;
        hooks.userdata = &callbacks; hooks.publication = &publish;
        auto infer_config = config(argv[1], &hooks);
        Engine inference;
        success(psdk_full_create(&infer_config, &inference.handle, &status), status, "create real inference");
        callbacks.engine = inference.handle;
        const auto process = [&](Result& result) {
            callbacks.expected_id = frame.left.frame_id;
            callbacks.expected_stamp = frame.left.timestamp_ns;
            return psdk_full_process(inference.handle, &frame, &result.handle, &status);
        };
        Result first;
        success(process(first), status, "real full inference");
        const auto first_view = view(first);
        require(first_view.has_inference == 1 && first_view.map_count == 0 && !first_view.recharge.executed,
                "inference-only JSON must run actual inference and expose zero maps/recharge");
        require(first_view.frame_id == frame.left.frame_id && first_view.timestamp_ns == frame.left.timestamp_ns &&
                first_view.inference.frame_id == frame.left.frame_id &&
                first_view.inference.timestamp_ns == frame.left.timestamp_ns, "full/inference same-frame identity");
        require(first_view.inference.segmentation_bytes == 640u * 544u &&
                first_view.inference.segmentation_mask, "real segmentation result");
        require(first_view.inference.detection_count == 0 || first_view.inference.detections,
                "valid real detection storage");
        require(callbacks.calls == 1 && callbacks.identity_ok, "application callback receives same-frame inference");
        require(first_view.task_trace_json && std::string(first_view.task_trace_json).find("dnn") != std::string::npos,
                "actual inference Task is traced");
        const auto first_hash = hash(first_view.inference.segmentation_mask, first_view.inference.segmentation_bytes);
        const std::string first_trace = first_view.task_trace_json;

        // Structural failure must not call inference/publication or retain output.
        Result invalid;
        const auto good_stamp = frame.left.timestamp_ns;
        frame.left.timestamp_ns = 0;
        require(process(invalid) == 1002 && !invalid.handle && status.code == 1002 &&
                std::string(status.feature) == "full.input" && status.frame_id == frame.left.frame_id,
                "zero timestamp preserves full validation error and frame");
        require(callbacks.calls == 1, "invalid input never reaches publication");
        frame.left.timestamp_ns = good_stamp + 1; ++frame.left.frame_id;
        callbacks.mode = CallbackState::Reenter;
        Result second;
        success(process(second), status, "callback reentry guard");
        require(callbacks.reentry_code == 2001 && callbacks.reentry_status_ok &&
                callbacks.reentry_ms >= 0 && callbacks.reentry_ms < 1000,
                "callback command reentry fails promptly without lock deadlock");
        require(hash(first_view.inference.segmentation_mask, first_view.inference.segmentation_bytes) == first_hash &&
                std::string(first_view.task_trace_json) == first_trace,
                "previous result remains immutable after another real inference");

        callbacks.mode = CallbackState::Fail; ++frame.left.frame_id; ++frame.left.timestamp_ns;
        Result rejected;
        require(process(rejected) == callback_error && !rejected.handle && status.code == callback_error &&
                status.native_code == -321 && status.frame_id == frame.left.frame_id,
                "publication uint32 business/native/frame error must survive the DAG and C ABI");
        require(status.feature[sizeof(status.feature) - 1] == 0 &&
                status.task[sizeof(status.task) - 1] == 0 && status.detail[sizeof(status.detail) - 1] == 0,
                "unterminated callback arrays are bounded and terminated");
        require(std::string(status.feature) == std::string(sizeof(status.feature) - 1, 'F') &&
                std::string(status.task) == std::string(sizeof(status.task) - 1, 'T') &&
                std::string(status.detail) == std::string(sizeof(status.detail) - 1, 'D'),
                "bounded callback details retain original content");
        callbacks.mode = CallbackState::Capture;
        success(psdk_full_command(inference.handle, 21, &status), status, "stop command");
        Result stopped;
        require(process(stopped) == 2001 && !stopped.handle &&
                std::string(status.feature) == "product.control.command", "stopped engine produces no stale result");
        success(psdk_full_command(inference.handle, 1, &status), status, "resume map/inference command");
        Result resumed;
        success(process(resumed), status, "recover after stop/publication error");
        success(psdk_full_destroy(&inference.handle, &status), status, "destroy inference");
        require(!inference.handle && hash(view(first).inference.segmentation_mask,
                first_view.inference.segmentation_bytes) == first_hash &&
                view(second).frame_id == 902, "owned results remain valid after engine destruction");
        const auto infer_save = (out / "inference-after-destroy").u8string();
        success(psdk_full_save_result(first.handle, infer_save.c_str(), &status), status, "save owned destroyed-engine result");
        require(fs::is_regular_file(out / "inference-after-destroy/result.json") &&
                fs::is_regular_file(out / "inference-after-destroy/mask.bin"), "saved real inference artifacts");
        require(psdk_full_save_result(first.handle, infer_save.c_str(), &status) == 2002,
                "full result saving never overwrites an existing directory");

        auto recharge_config = config(argv[2]);
        Engine recharge;
        success(psdk_full_create(&recharge_config, &recharge.handle, &status), status, "create real recharge");
        Engine competing;
        require(psdk_full_create(&recharge_config, &competing.handle, &status) == 2004 && !competing.handle &&
                std::string(status.feature) == "product.recharge_mark" &&
                std::string(status.detail).find("process-global") != std::string::npos,
                "second recharge engine rejected with explicit original singleton restriction");
        frame.left.frame_id = 1001; frame.left.timestamp_ns = 2234567891000ULL;
        Result pose;
        success(psdk_full_process(recharge.handle, &frame, &pose.handle, &status), status,
                "real recharge640 NV12 with explicit configured source fallback");
        const auto pose_view = view(pose);
        require(!pose_view.has_inference && pose_view.map_count == 0 && pose_view.recharge.executed,
                "recharge-only mode executes real algorithm without fabricated inference/maps");
        require(pose_view.recharge.frame_id == frame.left.frame_id &&
                pose_view.recharge.timestamp_ns == static_cast<std::int64_t>(frame.left.timestamp_ns) &&
                pose_view.frame_id == frame.left.frame_id, "original recharge same-frame pose metadata");
        require(pose_view.recharge.details_json && pose_view.recharge.crop_source[0],
                "original recharge diagnostics/crop metadata exposed, even for no-marker result");
        success(psdk_full_destroy(&recharge.handle, &status), status, "destroy recharge");
        require(view(pose).recharge.frame_id == 1001, "owned recharge result outlives engine");
        const auto pose_save = (out / "recharge-after-destroy").u8string();
        success(psdk_full_save_result(pose.handle, pose_save.c_str(), &status), status, "save owned recharge result");
        success(psdk_full_create(&recharge_config, &competing.handle, &status), status, "singleton released after destroy");
        success(psdk_full_destroy(&competing.handle, &status), status, "close replacement recharge engine");
        std::ofstream report(out / "contract.json", std::ios::binary);
        report << "{\"passed\":true,\"real_cpu_onnx_runtime\":true,\"real_recharge_algorithm\":true,"
                  "\"mapping_tested\":false,\"accuracy_or_hardware_claim\":false,"
                  "\"publication_error_uint32\":" << callback_error << ",\"recharge_mark_ok\":"
               << pose_view.recharge.mark_ok << ",\"recharge_pose_ok\":" << pose_view.recharge.pose_ok << "}\n";
        require(bool(report), "write contract evidence");
        std::cout << "Full public C ABI passed: real CPU ORT inference, original recharge, ownership, identity,"
                     " full uint32 callback errors, bounded strings, reentry, stop and singleton; no map/hardware/accuracy claim.\n";
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "Full public C ABI contract failed: " << exception.what() << '\n';
        return 1;
    }
}
