#include "perception_sdk.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#if defined(__unix__) || defined(__APPLE__)
#include <sys/resource.h>
#endif

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

// This consumer uses only the public C ABI. It never opens a camera or ROS node.
struct Options {
    fs::path sdk_root = ".";
    fs::path profile, dag, input, out, cpu_profile;
    std::string det_model, seg_model, backend = "bpu", format = "nv12";
    uint32_t width = 640, height = 544, stride = 640;
    uint32_t frames = 120, warmup = 20;
    bool save_png = false;
};

static std::string quote(const std::string& s) {
    std::ostringstream o;
    o << '"';
    for (unsigned char c : s) {
        switch (c) {
        case '"': o << "\\\""; break;
        case '\\': o << "\\\\"; break;
        case '\n': o << "\\n"; break;
        case '\r': o << "\\r"; break;
        case '\t': o << "\\t"; break;
        default:
            if (c < 32) o << "\\u" << std::hex << std::setw(4)
                          << std::setfill('0') << static_cast<unsigned>(c) << std::dec;
            else o << static_cast<char>(c);
        }
    }
    o << '"';
    return o.str();
}

static uint32_t number(const std::string& s) {
    if (s.empty() || s[0] == '-') throw std::runtime_error("Invalid integer: " + s);
    size_t used = 0;
    const auto n = std::stoull(s, &used);
    if (used != s.size() || n > std::numeric_limits<uint32_t>::max())
        throw std::runtime_error("Integer out of range: " + s);
    return static_cast<uint32_t>(n);
}

static void help() {
    std::cout
      << "perception_sdk_demo --input FRAME.nv12 [options]\n"
      << "  --sdk-root DIR        SDK root; default current directory\n"
      << "  --profile FILE        Original model/decoder configuration\n"
      << "  --dag FILE            Same-frame DAG configuration\n"
      << "  --det-model FILE      Override detection model path\n"
      << "  --seg-model FILE      Override segmentation model path\n"
      << "  --width N --height N --stride N  Default 640, 544, 640\n"
      << "  --format nv12|i420    Default nv12; contiguous YUV420 storage\n"
      << "  --frames N --warmup N Default 120 measured, 20 warmup calls\n"
      << "  --backend bpu|cpu     Default bpu; cpu requires --cpu-profile\n"
      << "  --cpu-profile FILE    Explicit ONNX tensor/decoder contract\n"
      << "  --out NEW_DIRECTORY   Refuse to overwrite existing directory\n"
      << "  --save-png            Save final-frame overlay.png\n"
      << "The same input file is replayed; FPS is inference throughput, not camera FPS.\n";
}

static Options parse(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        if (k == "--help" || k == "-h") { help(); std::exit(0); }
        if (k == "--save-png") { o.save_png = true; continue; }
        if (i + 1 == argc) throw std::runtime_error("Missing value for " + k);
        const std::string v = argv[++i];
        if (k == "--sdk-root") o.sdk_root = fs::u8path(v);
        else if (k == "--profile") o.profile = fs::u8path(v);
        else if (k == "--dag") o.dag = fs::u8path(v);
        else if (k == "--input") o.input = fs::u8path(v);
        else if (k == "--out") o.out = fs::u8path(v);
        else if (k == "--cpu-profile") o.cpu_profile = fs::u8path(v);
        else if (k == "--det-model") o.det_model = v;
        else if (k == "--seg-model") o.seg_model = v;
        else if (k == "--backend") o.backend = v;
        else if (k == "--format") o.format = v;
        else if (k == "--width") o.width = number(v);
        else if (k == "--height") o.height = number(v);
        else if (k == "--stride") o.stride = number(v);
        else if (k == "--frames") o.frames = number(v);
        else if (k == "--warmup") o.warmup = number(v);
        else throw std::runtime_error("Unknown option: " + k);
    }
    if (o.input.empty()) throw std::runtime_error("--input is required; see --help");
    if (o.backend != "bpu" && o.backend != "cpu") throw std::runtime_error("--backend must be bpu or cpu");
    if (o.backend == "cpu" && o.cpu_profile.empty()) throw std::runtime_error("CPU requires --cpu-profile");
    if (o.format != "nv12" && o.format != "i420") throw std::runtime_error("--format must be nv12 or i420");
    if (!o.width || !o.height || o.stride < o.width || (o.width | o.height | o.stride) & 1u)
        throw std::runtime_error("YUV420 width, height and stride must be positive/even; stride >= width");
    if (!o.frames || o.frames > 1000000 || o.warmup > 1000000)
        throw std::runtime_error("Require 1..1000000 frames and 0..1000000 warmup calls");
    if (o.profile.empty()) o.profile = o.sdk_root / "profiles" / "original.cfg";
    if (o.dag.empty()) o.dag = o.sdk_root / "profiles" / "dag_exact.json";
    if (o.out.empty()) {
        const auto id = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        o.out = fs::path("results") / ("shared-demo-" + std::to_string(id));
    }
    return o;
}

static double process_cpu_seconds() {
#if defined(__unix__) || defined(__APPLE__)
    rusage r{};
    if (getrusage(RUSAGE_SELF, &r)) return -1;
    return r.ru_utime.tv_sec + r.ru_utime.tv_usec * 1e-6
         + r.ru_stime.tv_sec + r.ru_stime.tv_usec * 1e-6;
#else
    return -1; // No fabricated CPU measurement on unsupported hosts.
#endif
}

static int64_t current_rss_kib() {
#if defined(__linux__)
    std::ifstream in("/proc/self/status");
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("VmRSS:", 0) == 0) {
            std::istringstream v(line.substr(6));
            int64_t kib = -1;
            if (v >> kib) return kib;
        }
    }
#endif
    return -1;
}

static int64_t peak_rss_kib() {
#if defined(__unix__) || defined(__APPLE__)
    rusage r{};
    if (getrusage(RUSAGE_SELF, &r)) return -1;
#if defined(__APPLE__)
    return r.ru_maxrss / 1024;
#else
    return r.ru_maxrss;
#endif
#else
    return -1;
#endif
}

static std::ofstream output(const fs::path& p, bool binary = false) {
    std::ofstream o(p, std::ios::out | (binary ? std::ios::binary : std::ios::openmode(0)));
    if (!o) throw std::runtime_error("Cannot create output: " + p.u8string());
    o.exceptions(std::ios::failbit | std::ios::badbit);
    o << std::setprecision(12);
    return o;
}

struct SdkFailure : std::runtime_error {
    psdk_status status;
    explicit SdkFailure(const psdk_status& s)
      : std::runtime_error(s.detail), status(s) {}
};

static void check(uint32_t code, const psdk_status& status) {
    if (code) throw SdkFailure(status);
}

struct Engine {
    psdk_engine* p = nullptr;
    ~Engine() { psdk_destroy(&p, nullptr); }
};
struct Result {
    psdk_result* p = nullptr;
    ~Result() { psdk_result_release(p); }
    void reset() { psdk_result_release(p); p = nullptr; }
};

struct Record {
    uint64_t frame_id, timestamp_ns;
    psdk_timings t;
    int64_t rss_kib;
};

static void timing_json(std::ostream& o, const psdk_timings& t) {
    o << "{\"pack_ms\":" << t.pack_ms
      << ",\"det_pre_ms\":" << t.detection_pre_ms
      << ",\"det_infer_ms\":" << t.detection_infer_ms
      << ",\"det_post_ms\":" << t.detection_post_ms
      << ",\"seg_pre_ms\":" << t.segmentation_pre_ms
      << ",\"seg_infer_ms\":" << t.segmentation_infer_ms
      << ",\"seg_post_ms\":" << t.segmentation_post_ms
      << ",\"graph_ms\":" << t.graph_ms << ",\"process_ms\":" << t.process_ms << '}';
}

static void save_failure(const fs::path& out, const psdk_status& s) {
    std::cerr << "SDK error " << s.code << ", native=" << s.native_code
              << ", frame=" << s.frame_id << ", task=" << s.task
              << ", feature=" << s.feature << ": " << s.detail << '\n';
    if (out.empty()) return;
    auto o = output(out / "failed.json");
    o << "{\"completed\":false,\"code\":" << s.code << ",\"native_code\":" << s.native_code
      << ",\"frame_id\":" << s.frame_id << ",\"timestamp_ns\":" << s.timestamp_ns
      << ",\"task\":" << quote(s.task) << ",\"feature\":" << quote(s.feature)
      << ",\"detail\":" << quote(s.detail) << "}\n";
}

int main(int argc, char** argv) {
    fs::path active_out;
    try {
        const Options o = parse(argc, argv);
        if (psdk_abi_version() != PSDK_ABI_VERSION) throw std::runtime_error("Shared library ABI version mismatch");
        const uint64_t y_bytes = uint64_t(o.stride) * o.height;
        if (y_bytes > std::numeric_limits<uint64_t>::max() / 3)
            throw std::runtime_error("Input dimensions overflow byte count");
        const uint64_t required = y_bytes * 3 / 2;
        if (required > std::numeric_limits<size_t>::max()
            || required > uint64_t(std::numeric_limits<std::streamsize>::max()))
            throw std::runtime_error("Input is too large for this consumer");
        if (fs::file_size(o.input) != required)
            throw std::runtime_error("Input file bytes differ from stride*height*3/2; check format/size/stride");
        std::vector<uint8_t> pixels(static_cast<size_t>(required));
        std::ifstream in(o.input, std::ios::binary);
        if (!in.read(reinterpret_cast<char*>(pixels.data()), static_cast<std::streamsize>(required)))
            throw std::runtime_error("Cannot read complete input file");
        if (fs::exists(o.out) || !fs::create_directories(o.out))
            throw std::runtime_error("--out must be a new directory: " + o.out.u8string());
        active_out = o.out;

        const std::string profile = o.profile.u8string(), dag = o.dag.u8string(), cpu = o.cpu_profile.u8string();
        psdk_config c{};
        c.struct_size = sizeof(c); c.abi_version = PSDK_ABI_VERSION;
        c.backend = o.backend == "bpu" ? PSDK_BACKEND_BPU : PSDK_BACKEND_CPU_ORT;
        c.original_profile = profile.c_str(); c.dag_profile = dag.c_str();
        c.cpu_profile = cpu.empty() ? nullptr : cpu.c_str();
        c.detection_model = o.det_model.empty() ? nullptr : o.det_model.c_str();
        c.segmentation_model = o.seg_model.empty() ? nullptr : o.seg_model.c_str();
        psdk_status s{};
        s.struct_size = sizeof(s); s.abi_version = PSDK_ABI_VERSION;
        Engine engine;
        check(psdk_create(&c, &engine.p, &s), s);
        Result result;
        psdk_frame f{};
        f.struct_size = sizeof(f); f.abi_version = PSDK_ABI_VERSION;
        f.format = o.format == "nv12" ? PSDK_FORMAT_NV12 : PSDK_FORMAT_I420;
        f.width = o.width; f.height = o.height; f.stride = o.stride;
        f.data = pixels.data(); f.bytes = pixels.size();
        uint64_t next_id = 1;
        auto run = [&] {
            result.reset();
            f.frame_id = next_id++;
            f.timestamp_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
            check(psdk_process(engine.p, &f, &result.p, &s), s);
            psdk_result_view v{};
            v.struct_size = sizeof(v); v.abi_version = PSDK_ABI_VERSION;
            check(psdk_result_get_view(result.p, &v, &s), s);
            if (v.frame_id != f.frame_id || v.timestamp_ns != f.timestamp_ns)
                throw std::runtime_error("Result identity differs from supplied frame");
            return v;
        };
        for (uint32_t i = 0; i < o.warmup; ++i) run();
        std::vector<Record> records;
        records.reserve(o.frames);
        const double cpu_start = process_cpu_seconds();
        const auto begin = Clock::now();
        psdk_result_view last{};
        for (uint32_t i = 0; i < o.frames; ++i) {
            last = run();
            records.push_back({last.frame_id, last.timestamp_ns, last.timings, current_rss_kib()});
        }
        const double elapsed_ms = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
        const double cpu_end = process_cpu_seconds();
        const double cpu_percent = cpu_start < 0 || cpu_end < cpu_start ? -1
                                 : (cpu_end - cpu_start) / (elapsed_ms / 1000) * 100;
        const int64_t rss = current_rss_kib(), peak = peak_rss_kib();
        psdk_timings mean{};
        for (const auto& r : records) {
            mean.pack_ms += r.t.pack_ms;
            mean.detection_pre_ms += r.t.detection_pre_ms;
            mean.detection_infer_ms += r.t.detection_infer_ms;
            mean.detection_post_ms += r.t.detection_post_ms;
            mean.segmentation_pre_ms += r.t.segmentation_pre_ms;
            mean.segmentation_infer_ms += r.t.segmentation_infer_ms;
            mean.segmentation_post_ms += r.t.segmentation_post_ms;
            mean.graph_ms += r.t.graph_ms;
            mean.process_ms += r.t.process_ms;
        }
        const double n = o.frames;
        mean.pack_ms /= n;
        mean.detection_pre_ms /= n; mean.detection_infer_ms /= n; mean.detection_post_ms /= n;
        mean.segmentation_pre_ms /= n; mean.segmentation_infer_ms /= n; mean.segmentation_post_ms /= n;
        mean.graph_ms /= n; mean.process_ms /= n;

        // Dump only the final result, after the measured window.
        {
            auto csv = output(o.out / "frames.csv");
            csv << "frame_id,timestamp_ns,pack_ms,det_pre_ms,det_infer_ms,det_post_ms,seg_pre_ms,seg_infer_ms,seg_post_ms,graph_ms,process_ms,rss_kib\n";
            for (const auto& r : records)
                csv << r.frame_id << ',' << r.timestamp_ns << ',' << r.t.pack_ms << ','
                    << r.t.detection_pre_ms << ',' << r.t.detection_infer_ms << ',' << r.t.detection_post_ms << ','
                    << r.t.segmentation_pre_ms << ',' << r.t.segmentation_infer_ms << ',' << r.t.segmentation_post_ms << ','
                    << r.t.graph_ms << ',' << r.t.process_ms << ',' << r.rss_kib << '\n';
        }
        {
            auto d = output(o.out / "detections.json");
            d << "{\"frame_id\":" << last.frame_id << ",\"timestamp_ns\":" << last.timestamp_ns
              << ",\"width\":" << last.width << ",\"height\":" << last.height << ",\"detections\":[";
            for (uint64_t i = 0; i < last.detection_count; ++i) {
                const auto& v = last.detections[i];
                if (i) d << ',';
                d << "{\"class_id\":" << v.class_id << ",\"class_name\":" << quote(v.class_name)
                  << ",\"score\":" << v.confidence << ",\"x1\":" << v.x1 << ",\"y1\":" << v.y1
                  << ",\"x2\":" << v.x2 << ",\"y2\":" << v.y2 << '}';
            }
            d << "]}\n";
        }
        {
            auto mask = output(o.out / "mask.bin", true);
            if (last.segmentation_bytes)
                mask.write(reinterpret_cast<const char*>(last.segmentation_mask),
                           static_cast<std::streamsize>(last.segmentation_bytes));
        }
        if (o.save_png) {
            const std::string png = (o.out / "overlay.png").u8string();
            check(psdk_save_png(result.p, &f, png.c_str(), &s), s);
        }
        check(psdk_destroy(&engine.p, &s), s);
        const int64_t closed_rss = current_rss_kib();
        {
            auto summary = output(o.out / "summary.json");
            summary << "{\"completed\":true,\"abi_version\":" << psdk_abi_version()
                    << ",\"build_capabilities\":" << psdk_build_capabilities()
                    << ",\"backend\":" << quote(o.backend)
                    << ",\"input\":" << quote(o.input.u8string())
                    << ",\"input_mode\":\"same_file_replay\",\"width\":" << o.width
                    << ",\"height\":" << o.height << ",\"stride\":" << o.stride
                    << ",\"format\":" << quote(o.format)
                    << ",\"frames\":" << o.frames << ",\"warmup\":" << o.warmup
                    << ",\"elapsed_ms\":" << elapsed_ms << ",\"fps\":" << o.frames * 1000.0 / elapsed_ms
                    << ",\"cpu_process_percent\":";
            if (cpu_percent < 0) summary << "null"; else summary << cpu_percent;
            summary << ",\"rss_kib\":";
            if (rss < 0) summary << "null"; else summary << rss;
            summary << ",\"peak_rss_kib\":";
            if (peak < 0) summary << "null"; else summary << peak;
            summary << ",\"rss_after_engine_destroy_kib\":";
            if (closed_rss < 0) summary << "null"; else summary << closed_rss;
            summary << ",\"average\":";
            timing_json(summary, mean);
            summary << ",\"output_frame_id\":" << last.frame_id
                    << ",\"output_timestamp_ns\":" << last.timestamp_ns
                    << ",\"mask_bytes\":" << last.segmentation_bytes
                    << ",\"overlay_saved\":" << (o.save_png ? "true" : "false") << "}\n";
        }
        std::cout << std::fixed << std::setprecision(3)
                  << "Same-file inference: " << o.frames * 1000.0 / elapsed_ms << " FPS, "
                  << mean.process_ms << " ms/process; graph=" << mean.graph_ms << " ms\n"
                  << "Detection pre/infer/post=" << mean.detection_pre_ms << '/' << mean.detection_infer_ms
                  << '/' << mean.detection_post_ms << " ms\n"
                  << "Segmentation pre/infer/post=" << mean.segmentation_pre_ms << '/' << mean.segmentation_infer_ms
                  << '/' << mean.segmentation_post_ms << " ms\n"
                  << "CPU process=" << cpu_percent << "%, RSS=" << rss << " KiB, peak=" << peak << " KiB\n"
                  << "Results: " << fs::absolute(o.out).u8string() << '\n';
        return 0;
    } catch (const SdkFailure& e) {
        try { save_failure(active_out, e.status); }
        catch (const std::exception& save) { std::cerr << "Could not save failed.json: " << save.what() << '\n'; }
        return 1; // Full 32-bit SDK code remains in stderr/failed.json, not the shell exit byte.
    } catch (const std::exception& e) {
        std::cerr << "Demo error: " << e.what() << '\n';
        return 1;
    }
}
