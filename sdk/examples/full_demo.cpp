#include "perception_full_sdk.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#if defined(__unix__) || defined(__APPLE__)
#include <sys/resource.h>
#include <time.h>
#endif
#if defined(R33_DEMO_WITH_CAMERA)
#include "stereo_inputs.hpp"
#include "runtime_status.hpp"
#endif

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

namespace {
struct Options {
    fs::path config = "config/full.json", input, disparity, hires, out;
    std::string det_model, seg_model;
    std::string camera_sdk = "/app/data/metoak/lib/libMoGeneralSDK.so";
    std::string camera_config = "/app/data/metoak/demo/config/x5_camera.cfg";
    std::uint32_t width = 640, height = 544, stride = 640, disparity_stride = 0;
    std::uint32_t frames = 120, warmup = 20;
    double fx = 298.404816, fy = 298.404816, cx = 313.862183, cy = 274.359955, bxf = 17935.5;
    bool camera = false, camera_hires = true;
    std::uint32_t acquire_timeout_ms = 3000, retry_interval_ms = 5;
    int command = -1;
};

std::string quote(const std::string& text) {
    std::ostringstream out;
    out << '"';
    for (unsigned char c : text) {
        switch (c) {
        case '"': out << "\\\""; break;
        case '\\': out << "\\\\"; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default:
            if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                            << static_cast<unsigned>(c) << std::dec;
            else out << static_cast<char>(c);
        }
    }
    return out.str() + '"';
}

std::uint32_t number(const std::string& value) {
    if (value.empty() || value.front() == '-') throw std::runtime_error("Invalid integer: " + value);
    std::size_t used = 0;
    const auto n = std::stoull(value, &used);
    if (used != value.size() || n > std::numeric_limits<std::uint32_t>::max())
        throw std::runtime_error("Integer out of range: " + value);
    return static_cast<std::uint32_t>(n);
}

double real(const std::string& value) {
    std::size_t used = 0;
    const double n = std::stod(value, &used);
    if (used != value.size() || !std::isfinite(n)) throw std::runtime_error("Invalid finite number: " + value);
    return n;
}

void help() {
    std::cout <<
        "perception_full_demo --config config/full.json --input LEFT.nv12 [options]\n"
        "  --disparity FILE     Same-capture raw uint16 Q8.8 disparity, little endian\n"
        "  --hires FILE         Same-capture GDC-left NV12, fixed 1280x1088 stride1280\n"
        "  --width N --height N --stride N   Defaults 640,544,640\n"
        "  --disparity-stride N  Bytes per Q8 row; default width*2\n"
        "  --frames N --warmup N Defaults 120 measured calls,20 warmup calls\n"
        "  --fx N --fy N --cx N --cy N --bxf N  Original bxf unit: focal_px*baseline_mm\n"
        "  --det-model FILE --seg-model FILE  Optional original model overrides\n"
        "  --command 1|2|3|21   Map,recharge,both,stop; otherwise use JSON mode\n"
        "  --out NEW_DIRECTORY  Save summary.json,frames.csv and owned last/ artifacts\n"
        "  --camera --sdk SO --camera-config FILE  Real one-handle RGBD+GDC input\n"
        "  --acquire-timeout-ms N --retry-interval-ms N  Camera input wait policy\n"
        "  --camera-no-hires    Explicitly omit GDC for configurations that allow it\n"
        "File mode replays supplied captures. Replay IDs/timestamps identify calls,\n"
        "not original camera capture metadata. Supply files from the same capture.\n"
        "Camera IDs/timestamps come directly from the SDK and are never rewritten.\n";
}

Options parse(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (key == "--help" || key == "-h") { help(); std::exit(0); }
        if (key == "--camera") { o.camera = true; continue; }
        if (key == "--camera-no-hires") { o.camera_hires = false; continue; }
        if (i + 1 >= argc) throw std::runtime_error("Missing value for " + key);
        const std::string value = argv[++i];
        if (key == "--config") o.config = fs::u8path(value);
        else if (key == "--input") o.input = fs::u8path(value);
        else if (key == "--disparity") o.disparity = fs::u8path(value);
        else if (key == "--hires") o.hires = fs::u8path(value);
        else if (key == "--out") o.out = fs::u8path(value);
        else if (key == "--det-model") o.det_model = value;
        else if (key == "--seg-model") o.seg_model = value;
        else if (key == "--sdk") o.camera_sdk = value;
        else if (key == "--acquire-timeout-ms") o.acquire_timeout_ms = number(value);
        else if (key == "--retry-interval-ms") o.retry_interval_ms = number(value);
        else if (key == "--camera-config") o.camera_config = value;
        else if (key == "--width") o.width = number(value);
        else if (key == "--height") o.height = number(value);
        else if (key == "--stride") o.stride = number(value);
        else if (key == "--disparity-stride") o.disparity_stride = number(value);
        else if (key == "--frames") o.frames = number(value);
        else if (key == "--warmup") o.warmup = number(value);
        else if (key == "--fx") o.fx = real(value);
        else if (key == "--fy") o.fy = real(value);
        else if (key == "--cx") o.cx = real(value);
        else if (key == "--cy") o.cy = real(value);
        else if (key == "--bxf") o.bxf = real(value);
        else if (key == "--command") {
            const auto command = number(value);
            if (command != 1 && command != 2 && command != 3 && command != 21)
                throw std::runtime_error("--command must be 1,2,3 or21");
            o.command = static_cast<int>(command);
        } else throw std::runtime_error("Unknown option: " + key);
    }
    if(o.acquire_timeout_ms<1||o.acquire_timeout_ms>60000||o.retry_interval_ms<1||o.retry_interval_ms>1000||o.retry_interval_ms>o.acquire_timeout_ms)throw std::runtime_error("Invalid camera wait/retry interval");
    if (!o.camera && o.input.empty()) throw std::runtime_error("File mode requires --input");
    if (o.camera && (!o.input.empty() || !o.disparity.empty() || !o.hires.empty()))
        throw std::runtime_error("--camera uses SDK frames; file input/disparity/hires cannot be mixed");
    if (!o.width || !o.height || o.width > 4096 || o.height > 4096 ||
        o.stride < o.width || ((o.width | o.height | o.stride) & 1u))
        throw std::runtime_error("NV12 requires even dimensions<=4096 and even stride>=width");
    if (!o.disparity_stride) o.disparity_stride = o.width * 2;
    if (o.disparity_stride < o.width * 2 || (o.disparity_stride & 1u))
        throw std::runtime_error("Q8 disparity row stride must be even and >=width*2");
    if (!o.frames || o.frames > 1000000 || o.warmup > 1000000)
        throw std::runtime_error("Require frames1..1000000 and warmup0..1000000");
    if (o.fx <= 0 || o.fy <= 0 || o.bxf <= 0) throw std::runtime_error("fx,fy,bxf must be positive");
    if (o.camera && (o.width != 640 || o.height != 544 || o.stride != 640))
        throw std::runtime_error("Original RGBD camera adapter requires640x544 stride640");
    if (o.out.empty()) {
        const auto id = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        o.out = fs::path("results") / ("full-demo-" + std::to_string(id));
    }
    return o;
}

std::ofstream output(const fs::path& path) {
    std::ofstream file(path, std::ios::out | std::ios::binary);
    if (!file) throw std::runtime_error("Cannot create output: " + path.u8string());
    file.exceptions(std::ios::badbit | std::ios::failbit);
    file << std::setprecision(12);
    return file;
}

std::vector<std::uint8_t> readBytes(const fs::path& path, std::uint64_t required) {
    if (required > std::numeric_limits<std::size_t>::max() ||
        required > static_cast<std::uint64_t>(std::numeric_limits<std::streamsize>::max()) ||
        fs::file_size(path) != required)
        throw std::runtime_error("Input byte count differs from declared shape: " + path.u8string());
    std::vector<std::uint8_t> bytes(static_cast<std::size_t>(required));
    std::ifstream file(path, std::ios::binary);
    if (!file.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size())))
        throw std::runtime_error("Cannot read complete input: " + path.u8string());
    return bytes;
}

double cpuSeconds() {
#if defined(__unix__) || defined(__APPLE__)
    rusage usage{};
    if (!getrusage(RUSAGE_SELF, &usage)) return usage.ru_utime.tv_sec + usage.ru_utime.tv_usec * 1e-6 +
                                              usage.ru_stime.tv_sec + usage.ru_stime.tv_usec * 1e-6;
#endif
    return -1;
}

std::int64_t memoryField(const char* filename, const char* key) {
#if defined(__linux__)
    std::ifstream file(filename);
    std::string line;
    while (std::getline(file, line)) {
        const std::string prefix = std::string(key) + ':';
        if (line.rfind(prefix, 0) == 0) {
            std::istringstream number(line.substr(prefix.size()));
            std::int64_t kib = -1;
            if (number >> kib) return kib;
        }
    }
#else
    (void)filename; (void)key;
#endif
    return -1;
}

std::int64_t peakRss() {
#if defined(__unix__) || defined(__APPLE__)
    rusage usage{};
    if (!getrusage(RUSAGE_SELF, &usage)) {
#if defined(__APPLE__)
        return usage.ru_maxrss / 1024;
#else
        return usage.ru_maxrss;
#endif
    }
#endif
    return -1;
}

std::uint64_t replayTimestamp() {
#if defined(CLOCK_BOOTTIME)
    timespec stamp{};
    if (!clock_gettime(CLOCK_BOOTTIME, &stamp))
        return static_cast<std::uint64_t>(stamp.tv_sec) * 1000000000ULL + stamp.tv_nsec;
#endif
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        Clock::now().time_since_epoch()).count());
}

struct SdkFailure : std::runtime_error {
    psdk_status status;
    explicit SdkFailure(psdk_status value) : std::runtime_error(value.detail), status(value) {}
};

void check(std::uint32_t code, const psdk_status& status) {
    if (code) { auto failure = status; failure.code = code; throw SdkFailure(failure); }
}

struct Engine {
    psdk_full_engine* handle = nullptr;
    ~Engine() { psdk_full_destroy(&handle, nullptr); }
};
struct Result {
    psdk_full_result* handle = nullptr;
    ~Result() { psdk_full_result_release(handle); }
    void reset() { psdk_full_result_release(handle); handle = nullptr; }
};

void failureFile(const fs::path& directory, const psdk_status& status) {
    std::cerr << "Full SDK error " << status.code << " native=" << status.native_code
              << " frame=" << status.frame_id << " task=" << status.task
              << " feature=" << status.feature << ": " << status.detail << '\n';
    if (directory.empty()) return;
    auto file = output(directory / "failed.json");
    file << "{\"completed\":false,\"code\":" << status.code << ",\"native_code\":" << status.native_code
         << ",\"frame_id\":" << status.frame_id << ",\"timestamp_ns\":" << status.timestamp_ns
         << ",\"feature\":" << quote(status.feature) << ",\"task\":" << quote(status.task)
         << ",\"detail\":" << quote(status.detail) << "}\n";
}

struct Record {
    std::uint64_t frame_id = 0, timestamp_ns = 0;
    double acquire_ms = 0, call_ms = 0, graph_ms = 0, process_ms = 0, cpu_percent = -1;
    std::int64_t rss_kib = -1, cma_total_kib = -1, cma_free_kib = -1;
    std::uint32_t maps = 0, recharge_executed = 0, mark_ok = 0, pose_ok = 0;
    psdk_timings inference{};
};

void nullable(std::ostream& file, double value) {
    if (value < 0 || !std::isfinite(value)) file << "null"; else file << value;
}

void resourceSnapshot(const fs::path& directory, const char* phase, bool result_owned) {
    std::ofstream file(directory / "resource_lifecycle.jsonl", std::ios::binary | std::ios::app);
    if (!file) throw std::runtime_error("Cannot write resource lifecycle");
    file << "{\"phase\":" << quote(phase) << ",\"host_timestamp_ns\":" << replayTimestamp()
         << ",\"owned_last_result_alive\":" << (result_owned ? "true" : "false") << ",\"rss_kib\":";
    nullable(file, static_cast<double>(memoryField("/proc/self/status", "VmRSS")));
    file << ",\"cma_total_kib\":"; nullable(file, static_cast<double>(memoryField("/proc/meminfo", "CmaTotal")));
    file << ",\"cma_free_kib\":"; nullable(file, static_cast<double>(memoryField("/proc/meminfo", "CmaFree")));
    file << "}\n"; file.close(); if (!file) throw std::runtime_error("Resource lifecycle write/close failed");
}

psdk_frame frameView(const std::uint8_t* data, std::uint64_t bytes, std::uint32_t width,
                     std::uint32_t height, std::uint32_t stride,
                     std::uint64_t frame_id, std::uint64_t timestamp_ns) {
    psdk_frame frame{};
    frame.struct_size = sizeof(frame); frame.abi_version = PSDK_ABI_VERSION;
    frame.format = PSDK_FORMAT_NV12; frame.width = width; frame.height = height;
    frame.stride = stride; frame.data = data; frame.bytes = bytes;
    frame.frame_id = frame_id; frame.timestamp_ns = timestamp_ns;
    return frame;
}
}  // namespace

int main(int argc, char** argv) {
    fs::path active_out;
    std::uint32_t fallback_code = 2001;
    std::string fallback_feature = "full_demo.input";
    try {
        const Options options = parse(argc, argv);
        fallback_code = 2002;
        fallback_feature = "full_demo.output";
        if (fs::exists(options.out) || !fs::create_directories(options.out))
            throw std::runtime_error("--out must be a new directory: " + options.out.u8string());
        active_out = options.out;
        resourceSnapshot(options.out, "before_init", false);
        fallback_code = 2001;
        fallback_feature = "full_demo.input";
        if (psdk_abi_version() != PSDK_ABI_VERSION) throw std::runtime_error("SDK ABI version mismatch");
        std::vector<std::uint8_t> pixels, hires;
        std::vector<std::uint16_t> disparity;
        if (!options.camera) {
            pixels = readBytes(options.input, static_cast<std::uint64_t>(options.stride) * options.height * 3u / 2u);
            if (!options.hires.empty()) hires = readBytes(options.hires, 1280ULL * 1088 * 3 / 2);
            if (!options.disparity.empty()) {
                const auto bytes = readBytes(options.disparity,
                    static_cast<std::uint64_t>(options.disparity_stride) * options.height);
                disparity.resize(bytes.size() / 2);
                for (std::size_t i = 0; i < disparity.size(); ++i)
                    disparity[i] = static_cast<std::uint16_t>(bytes[i * 2] | (std::uint16_t(bytes[i * 2 + 1]) << 8));
            }
        }

#if !defined(R33_DEMO_WITH_CAMERA)
        if (options.camera) {
            fallback_code = 1001;
            fallback_feature = "full_demo.camera";
            throw std::runtime_error("This standalone consumer was built without R33_DEMO_WITH_CAMERA");
        }
#endif
        psdk_full_config config{};
        const auto json_path = options.config.u8string();
        config.struct_size = sizeof(config); config.abi_version = PSDK_FULL_ABI_VERSION;
        config.json_file = json_path.c_str();
        config.detection_model = options.det_model.empty() ? nullptr : options.det_model.c_str();
        config.segmentation_model = options.seg_model.empty() ? nullptr : options.seg_model.c_str();
        psdk_status status{};
        status.struct_size = sizeof(status); status.abi_version = PSDK_ABI_VERSION;
        Engine engine;
        check(psdk_full_create(&config, &engine.handle, &status), status);
        if (options.command >= 0) check(psdk_full_command(engine.handle, options.command, &status), status);

#if defined(R33_DEMO_WITH_CAMERA)
        std::unique_ptr<r33::stereo::FullCameraAdapter> camera;
        if (options.camera) {
            fallback_code = 1001;
            fallback_feature = "full_demo.camera";
            r33::stereo::CameraOptions camera_options;
            camera_options.sdk_path = options.camera_sdk;
            camera_options.config_path = options.camera_config;
            r33::stereo::Intrinsics intrinsics;
            intrinsics.width = static_cast<int>(options.width); intrinsics.height = static_cast<int>(options.height);
            intrinsics.fx = static_cast<float>(options.fx); intrinsics.fy = static_cast<float>(options.fy);
            intrinsics.cx = static_cast<float>(options.cx); intrinsics.cy = static_cast<float>(options.cy);
            intrinsics.bxf = options.bxf;
            camera = std::make_unique<r33::stereo::FullCameraAdapter>(
                camera_options, intrinsics, options.camera_hires);
        }
#endif
        psdk_camera_intrinsics reported_intrinsics{options.width,options.height,options.fx,options.fy,options.cx,options.cy,options.bxf};
        resourceSnapshot(options.out, "after_init", false);
        Result result;
        psdk_full_result_view last{};
        std::uint64_t replay_id = 0;
        auto run = [&]() {
            result.reset();
            Record record;
            const double cpu_before = cpuSeconds();
            const auto frame_begin = Clock::now();
            fallback_code = options.camera ? 1001 : 1002;
            fallback_feature = options.camera ? "full_demo.camera" : "full_demo.input";
            psdk_full_frame frame{};
            psdk_frame hires_view{};
            psdk_disparity_q8 disparity_view{};
            frame.struct_size = sizeof(frame); frame.abi_version = PSDK_FULL_ABI_VERSION;
            frame.intrinsics = {options.width, options.height, options.fx, options.fy,
                                options.cx, options.cy, options.bxf};
            disparity_view.struct_size = sizeof(disparity_view); disparity_view.abi_version = PSDK_FULL_ABI_VERSION;
#if defined(R33_DEMO_WITH_CAMERA)
            r33::stereo::Frame camera_frame;
#endif
#if defined(R33_DEMO_WITH_CAMERA)
            if (camera) {
                camera_frame = camera->next();
                const auto& left = *camera_frame.left;
                frame.left = frameView(left.bytes.data(), left.bytes.size(), left.width, left.height,
                                       static_cast<std::uint32_t>(left.stride),
                                       camera_frame.info.id, camera_frame.info.timestamp_ns);
                const auto q8 = camera_frame.disparityView();
                disparity_view.width = q8.width; disparity_view.height = q8.height;
                disparity_view.stride_bytes = static_cast<std::uint32_t>(q8.stride_bytes);
                disparity_view.data = static_cast<const std::uint16_t*>(q8.data); disparity_view.bytes = q8.bytes;
                disparity_view.frame_id = q8.frame.id; disparity_view.timestamp_ns = q8.frame.timestamp_ns;
                frame.disparity = &disparity_view;
                if (camera_frame.hires_left) {
                    const auto& image = *camera_frame.hires_left;
                    hires_view = frameView(image.bytes.data(), image.bytes.size(), image.width, image.height,
                        static_cast<std::uint32_t>(image.stride), camera_frame.hires_info.id,
                        camera_frame.hires_info.timestamp_ns);
                    frame.recharge_hires = &hires_view;
                }
            } else
#endif
            {
                const auto stamp = replayTimestamp();
                frame.left = frameView(pixels.data(), pixels.size(), options.width, options.height,
                                       options.stride, ++replay_id, stamp);
                if (!hires.empty()) {
                    hires_view = frameView(hires.data(), hires.size(), 1280, 1088, 1280, replay_id, stamp);
                    frame.recharge_hires = &hires_view;
                }
                if (!disparity.empty()) {
                    disparity_view.width = options.width; disparity_view.height = options.height;
                    disparity_view.stride_bytes = options.disparity_stride;
                    disparity_view.data = disparity.data(); disparity_view.bytes = disparity.size() * sizeof(std::uint16_t);
                    disparity_view.frame_id = replay_id; disparity_view.timestamp_ns = stamp;
                    frame.disparity = &disparity_view;
                }
            }
            reported_intrinsics=frame.intrinsics;
            record.acquire_ms = std::chrono::duration<double, std::milli>(Clock::now() - frame_begin).count();
            const auto call_begin = Clock::now();
            check(psdk_full_process(engine.handle, &frame, &result.handle, &status), status);
            record.call_ms = std::chrono::duration<double, std::milli>(Clock::now() - call_begin).count();
            last = {};
            last.struct_size = sizeof(last); last.abi_version = PSDK_FULL_ABI_VERSION;
            check(psdk_full_result_get_view(result.handle, &last, &status), status);
            fallback_code = 1002;
            fallback_feature = "full_demo.identity";
            if (last.frame_id != frame.left.frame_id || last.timestamp_ns != frame.left.timestamp_ns)
                throw std::runtime_error("Full result identity differs from supplied input");
            record.frame_id = last.frame_id; record.timestamp_ns = last.timestamp_ns;
            record.graph_ms = last.graph_ms; record.process_ms = last.process_ms;
            record.maps = last.map_count; record.recharge_executed = last.recharge.executed;
            record.mark_ok = last.recharge.mark_ok; record.pose_ok = last.recharge.pose_ok;
            record.inference = last.inference.timings;
            const double interval_ms = std::chrono::duration<double, std::milli>(Clock::now() - frame_begin).count();
            const double cpu_after = cpuSeconds();
            if (cpu_before >= 0 && cpu_after >= cpu_before && interval_ms > 0)
                record.cpu_percent = (cpu_after - cpu_before) * 100000.0 / interval_ms;
            record.rss_kib = memoryField("/proc/self/status", "VmRSS");
            record.cma_total_kib = memoryField("/proc/meminfo", "CmaTotal");
            record.cma_free_kib = memoryField("/proc/meminfo", "CmaFree");
            return record;
        };
        for (std::uint32_t i = 0; i < options.warmup; ++i) run();
        resourceSnapshot(options.out, "after_warmup", result.handle != nullptr);
        std::vector<Record> records;
        records.reserve(options.frames);
        const double cpu_begin = cpuSeconds();
        const auto begin = Clock::now();
        for (std::uint32_t i = 0; i < options.frames; ++i) records.push_back(run());
        const double elapsed_ms = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
        const double cpu_end = cpuSeconds();
        const double cpu_percent = cpu_begin >= 0 && cpu_end >= cpu_begin && elapsed_ms > 0 ?
                                   (cpu_end - cpu_begin) * 100000.0 / elapsed_ms : -1;
        resourceSnapshot(options.out, "after_measurement", result.handle != nullptr);
        fallback_code = 2002;
        fallback_feature = "full_demo.output";
        const auto last_path = (options.out / "last").u8string();
        check(psdk_full_save_result(result.handle, last_path.c_str(), &status), status);
#if defined(R33_DEMO_WITH_CAMERA)
        if (camera) camera->close();
#endif
        resourceSnapshot(options.out, "after_camera_close", true);
        check(psdk_full_destroy(&engine.handle, &status), status);
        resourceSnapshot(options.out, "after_engine_destroy", true);
        {
            auto csv = output(options.out / "frames.csv");
            csv << "frame_id,timestamp_ns,acquire_ms,call_ms,graph_ms,process_ms,det_pre_ms,det_infer_ms,det_post_ms,seg_pre_ms,seg_infer_ms,seg_post_ms,cpu_percent,rss_kib,cma_total_kib,cma_free_kib,map_count,recharge_executed,mark_ok,pose_ok\n";
            for (const auto& r : records) {
                const auto& t = r.inference;
                csv << r.frame_id << ',' << r.timestamp_ns << ',' << r.acquire_ms << ',' << r.call_ms << ','
                    << r.graph_ms << ',' << r.process_ms << ',' << t.detection_pre_ms << ',' << t.detection_infer_ms << ','
                    << t.detection_post_ms << ',' << t.segmentation_pre_ms << ',' << t.segmentation_infer_ms << ','
                    << t.segmentation_post_ms << ',';
                if (r.cpu_percent >= 0) csv << r.cpu_percent;
                csv << ','; if (r.rss_kib >= 0) csv << r.rss_kib;
                csv << ','; if (r.cma_total_kib >= 0) csv << r.cma_total_kib;
                csv << ','; if (r.cma_free_kib >= 0) csv << r.cma_free_kib;
                csv << ',' << r.maps << ',' << r.recharge_executed << ',' << r.mark_ok << ',' << r.pose_ok << '\n';
            }
        }
        {
            auto summary = output(options.out / "summary.json");
            summary << "{\"completed\":true,\"full_abi_version\":" << PSDK_FULL_ABI_VERSION
                    << ",\"capabilities\":" << psdk_full_capabilities() << ",\"config\":" << quote(json_path)
                    << ",\"input_mode\":" << quote(options.camera ? "real_sdk_rgbd_gdc" : "same_file_replay")
                    << ",\"input\":" << quote(options.input.u8string())
                    << ",\"disparity_input\":" << quote(options.disparity.u8string())
                    << ",\"hires_input\":" << quote(options.hires.u8string())
                    << ",\"frames\":" << options.frames << ",\"warmup\":" << options.warmup
                    << ",\"elapsed_ms\":" << elapsed_ms << ",\"fps\":" << options.frames * 1000.0 / elapsed_ms
                    << ",\"cpu_process_percent\":";
            nullable(summary, cpu_percent);
            summary << ",\"rss_kib\":"; nullable(summary, static_cast<double>(records.back().rss_kib));
            summary << ",\"peak_rss_kib\":"; nullable(summary, static_cast<double>(peakRss()));
            summary << ",\"rss_after_engine_destroy_kib\":"; nullable(summary, static_cast<double>(memoryField("/proc/self/status", "VmRSS")));
            summary << ",\"cma_total_kib\":"; nullable(summary, static_cast<double>(records.back().cma_total_kib));
            summary << ",\"cma_free_kib\":"; nullable(summary, static_cast<double>(records.back().cma_free_kib));
            summary << ",\"output_frame_id\":" << last.frame_id << ",\"output_timestamp_ns\":" << last.timestamp_ns
                    << ",\"map_count\":" << last.map_count << ",\"recharge_executed\":" << last.recharge.executed
                    << ",\"mark_ok\":" << last.recharge.mark_ok << ",\"pose_ok\":" << last.recharge.pose_ok
                    << ",\"intrinsics\":{\"width\":" << reported_intrinsics.width << ",\"height\":" << reported_intrinsics.height
                    << ",\"fx\":" << reported_intrinsics.fx << ",\"fy\":" << reported_intrinsics.fy << ",\"cx\":" << reported_intrinsics.cx
                    << ",\"cy\":" << reported_intrinsics.cy << ",\"bxf_px_mm\":" << reported_intrinsics.bxf << "}"
                    << ",\"last_artifacts\":\"last\",\"task_trace\":" << quote(last.task_trace_json ? last.task_trace_json : "")
                    << ",\"effective_config\":" << quote(last.effective_config_json ? last.effective_config_json : "") << "}\n";
        }
        result.reset();
        resourceSnapshot(options.out, "after_result_release", false);
        std::cout << std::fixed << std::setprecision(3) << "Completed " << options.frames << " frames, "
                  << options.frames * 1000.0 / elapsed_ms << " FPS; artifacts: " << options.out.u8string() << '\n';
        return 0;
    } catch (const SdkFailure& failure) {
        try { failureFile(active_out, failure.status); } catch (const std::exception& e) { std::cerr << e.what() << '\n'; }
#if defined(R33_DEMO_WITH_CAMERA)
    } catch (const r28::Failure& failure) {
        const auto& source = failure.status();
        psdk_status status{};
        status.struct_size = sizeof(status); status.abi_version = PSDK_ABI_VERSION;
        status.code = source.code; status.native_code = source.native_code;
        status.frame_id = source.frame.id; status.timestamp_ns = source.frame.timestamp_ns;
        const auto copy = [](char* dest, std::size_t size, const std::string& text) {
            const auto count = std::min(size - 1, text.size());
            std::copy_n(text.data(), count, dest); dest[count] = 0;
        };
        copy(status.feature, sizeof(status.feature), source.feature);
        copy(status.task, sizeof(status.task), source.task);
        copy(status.detail, sizeof(status.detail), source.detail);
        try { failureFile(active_out, status); } catch (const std::exception& e) { std::cerr << e.what() << '\n'; }
#endif
    } catch (const std::exception& exception) {
        psdk_status status{};
        status.struct_size = sizeof(status); status.abi_version = PSDK_ABI_VERSION;
        status.code = fallback_code;
        std::copy_n(fallback_feature.data(), std::min(fallback_feature.size(), sizeof(status.feature) - 1), status.feature);
        const std::string detail = exception.what();
        std::copy_n(detail.data(), std::min(detail.size(), sizeof(status.detail) - 1), status.detail);
        try { failureFile(active_out, status); } catch (const std::exception& e) { std::cerr << e.what() << '\n'; }
    }
    return 1;
}
