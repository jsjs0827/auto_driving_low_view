#include "ab/runtime.hpp"
#include "platform_metrics.hpp"
#include "bpu_metrics.hpp"
#include <chrono>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <unistd.h>
namespace {
using Clock = std::chrono::steady_clock;
double ms(Clock::time_point t) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}
void stop(int) {
    ab::running = false;
}
std::string digest(const void *data, std::size_t n) {
    auto *b = static_cast<const unsigned char *>(data);
    std::uint64_t h = 14695981039346656037ULL;
    for (std::size_t i = 0; i < n; ++i) {
        h ^= b[i];
        h *= 1099511628211ULL;
    }
    std::ostringstream s;
    s << std::hex << std::setfill('0') << std::setw(16) << h;
    return s.str();
}
double pidCpu(int pid) {
    std::ifstream f("/proc/" + std::to_string(pid) + "/stat");
    std::string s;
    std::getline(f, s);
    auto p = s.rfind(')');
    if (!f || p == std::string::npos)
        throw std::runtime_error("Cannot read publisher process CPU");
    std::istringstream in(s.substr(p + 2));
    std::string field;
    unsigned long long u = 0, v = 0;
    for (int k = 3; k <= 15; ++k) {
        in >> field;
        if (k == 14)
            u = std::stoull(field);
        if (k == 15)
            v = std::stoull(field);
    }
    return (u + v) * 1000.0 / sysconf(_SC_CLK_TCK);
}
} // namespace
int main(int argc, char **argv) {
    ab::fs::path output;
    bool owns_output = false;
    try {
        std::map<std::string, std::string> args;
        bool publish = false, validate_only = false, record = false, bridge = false;
        for (int i = 1; i < argc; ++i) {
            std::string a = argv[i];
            if (a == "--publish") {
                publish = true;
                continue;
            }
            if (a == "--bridge-forward") {
                bridge = true;
                continue;
            }
            if (a == "--validate-input") {
                validate_only = true;
                continue;
            }
            if (a == "--record") {
                record = true;
                continue;
            }
            if (i + 1 >= argc)
                throw std::invalid_argument("Every option requires a value");
            if (!args.emplace(a, argv[++i]).second)
                throw std::invalid_argument("Duplicate argument");
        }
        for (auto &a : args)
            if (a.first != "--config" && a.first != "--method" && a.first != "--suite" &&
                a.first != "--source" && a.first != "--output" && a.first != "--publish-source" &&
                a.first != "--publisher-pid")
                throw std::invalid_argument("Unknown argument: " + a.first);
        if (!args.count("--config"))
            throw std::invalid_argument(
                "ab_benchmark --config CONFIG --method original|optimized_q8|optimized_stereo "
                "--suite native|matched_roi --source offline|camera|ros --output NEW_DIRECTORY");
        auto c = ab::load(args.at("--config"));
        if (args.count("--source"))
            c.source = args.at("--source");
        std::string method = args.count("--method") ? args.at("--method") : "original",
                    suite = args.count("--suite") ? args.at("--suite") : "native";
        if (method != "original" && method != "optimized_q8" && method != "optimized_stereo")
            throw std::invalid_argument("Unknown method");
        if (suite != "native" && suite != "matched_roi")
            throw std::invalid_argument("Unknown suite");
        if (c.source != "offline" && c.source != "camera" && c.source != "ros")
            throw std::invalid_argument("Unknown source");
        bool need_q8 = method != "optimized_stereo";
        std::signal(SIGINT, stop);
        std::signal(SIGTERM, stop);
        if (bridge) {
            ab::bridgeForward(c, need_q8);
            return 0;
        }
        if (publish) {
            ab::rosPublish(c,
                           args.count("--publish-source") ? args.at("--publish-source") : "offline",
                           need_q8);
            return 0;
        }
        if (!args.count("--output"))
            throw std::invalid_argument("--output required");
        output = ab::fs::absolute(ab::fs::u8path(args.at("--output")));
        if (ab::fs::exists(output))
            throw std::invalid_argument("Output already exists; choose a NEW directory");
        if (!ab::fs::create_directories(output))
            throw std::runtime_error("Could not exclusively create output directory");
        owns_output = true;
        std::ofstream config(output / "requested_config.json");
        config << ab::dump(c.root) << '\n';
        config.close();
        if (record)
            need_q8 = true;
        std::unique_ptr<ab::Pipeline> pipeline;
        if (!validate_only && !record)
            pipeline = std::make_unique<ab::Pipeline>(c, method, suite, output);
        auto source = c.source == "offline"  ? ab::offlineSource(c, need_q8)
                      : c.source == "camera" ? ab::cameraSource(c, need_q8)
                                             : ab::rosSource(c, need_q8);
        if (record) {
            for (auto name : {"left", "right", "q8"})
                ab::fs::create_directory(output / name);
            std::ofstream manifest(output / "manifest.json");
            manifest << "{\"schema_version\":1,\"frames\":[";
            for (int i = 0; i < c.frames + c.warmup && ab::running; ++i) {
                auto f = source->next();
                std::ostringstream name;
                name << std::setw(6) << std::setfill('0') << i;
                auto stem = name.str();
                auto binary = [&](const ab::fs::path &path, const void *data, std::size_t n) {
                    std::ofstream b(path, std::ios::binary);
                    b.write(static_cast<const char *>(data), std::streamsize(n));
                    b.close();
                    if (!b)
                        throw std::runtime_error("Recording write failed");
                };
                binary(output / "left" / (stem + ".nv12"), f.left.ptr(), f.left.total());
                binary(output / "right" / (stem + ".nv12"), f.right.ptr(), f.right.total());
                std::vector<std::uint8_t> q(f.q8.size() * 2);
                for (std::size_t n = 0; n < f.q8.size(); ++n) {
                    q[n * 2] = f.q8[n] & 255;
                    q[n * 2 + 1] = f.q8[n] >> 8;
                }
                binary(output / "q8" / (stem + ".u16"), q.data(), q.size());
                if (i)
                    manifest << ',';
                manifest << "{\"key\":" << ab::j::quote(f.key)
                         << ",\"frame_id\":" << ab::j::quote(std::to_string(f.id))
                         << ",\"timestamp_ns\":" << ab::j::quote(std::to_string(f.ns))
                         << ",\"left\":\"left/" << stem << ".nv12\",\"right\":\"right/" << stem
                         << ".nv12\",\"q8\":\"q8/" << stem << ".u16\"}";
                if (i == 0) {
                    std::ofstream k(output / "intrinsics.json");
                    k << std::setprecision(15) << "{\"width\":" << f.k.w << ",\"height\":" << f.k.h
                      << ",\"fx\":" << f.k.fx << ",\"fy\":" << f.k.fy << ",\"cx\":" << f.k.cx
                      << ",\"cy\":" << f.k.cy << ",\"bxf\":" << f.k.bxf
                      << ",\"bxf_unit\":\"pixel_mm\"}\n";
                }
            }
            if (!ab::running)
                throw std::runtime_error("Recording interrupted; manifest is incomplete");
            manifest << "]}\n";
            manifest.close();
            if (!manifest)
                throw std::runtime_error("Manifest write failed");
            std::cout << "Recorded same-frame left/right/Q8/intrinsics: " << output.u8string()
                      << '\n';
            return 0;
        }
        if (validate_only) {
            for (int i = 0; i < std::min(3, c.frames + c.warmup); ++i) {
                auto f = source->next();
                ab::validate(f.k);
                if (need_q8)
                    ab::maskQ8(f.q8, f.k.w, f.k.h, ab::region(c, method, suite));
                std::cout << "Validated " << f.key << " original timestamp=" << f.ns
                          << " left=" << f.left.cols << 'x' << f.k.h << " right=" << f.right.cols
                          << 'x' << f.k.h << '\n';
            }
            std::ofstream f(output / "input_validation.json");
            f << "{\"input_validated\":true,\"inference_executed\":false,\"source\":"
              << source->metadata() << "}\n";
            return 0;
        }
        std::ofstream frames(output / "frames.jsonl");
        if (!frames)
            throw std::runtime_error("Cannot write frame metrics");
        frames << std::setprecision(15);
        std::map<std::string, std::vector<double>> samples;
        int count = 0;
        double wall = 0, cpu = 0, extra_cpu = 0;
        int extra_pid = args.count("--publisher-pid") ? std::stoi(args.at("--publisher-pid")) : 0;
        r29::BpuSampler bpu;
        Clock::time_point window;
        double cpu_start = 0, extra_start = 0;
        for (int index = 0; index < c.warmup + c.frames && ab::running; ++index) {
            if (index == c.warmup) {
                bpu.begin((output / "bpu_samples.jsonl").u8string());
                window = Clock::now();
                cpu_start = r29::processCpuMs();
                if (extra_pid)
                    extra_start = pidCpu(extra_pid);
            }
            auto begin = Clock::now();
            double before_cpu = r29::processCpuMs();
            auto f = source->next();
            double acquire = ms(begin);
            auto compute = Clock::now();
            double compute_cpu = r29::processCpuMs();
            auto result = pipeline->process(f);
            double compute_wall = ms(compute), compute_cpu_ms = r29::processCpuMs() - compute_cpu;
            if (index < c.warmup)
                continue;
            int ordinal = count++;
            double save = 0;
            if (c.save_every && ordinal % c.save_every == 0) {
                auto t = Clock::now();
                std::ostringstream name;
                name << "frame-" << std::setw(6) << std::setfill('0') << ordinal;
                ab::saveOutput(output / name.str(), f, result);
                save = ms(t);
            }
            auto resources = r29::processResources();
            auto rss = resources.find("VmRSS:");
            if (rss == resources.end())
                throw std::runtime_error("Linux RSS unavailable");
            result.ms["acquire"] = acquire;
            result.ms["compute_wall"] = compute_wall;
            result.ms["compute_cpu"] = compute_cpu_ms;
            result.ms["save"] = save;
            result.ms["frame_wall"] = ms(begin);
            result.ms["frame_cpu"] = r29::processCpuMs() - before_cpu;
            samples["rss_kib"].push_back(rss->second);
            auto lhash = digest(f.left.ptr(), f.left.total()),
                 rhash = digest(f.right.ptr(), f.right.total()),
                 qhash = digest(result.q8.data(), result.q8.size() * 2);
            auto roi = ab::region(c, method, suite);
            std::ostringstream artifact; artifact << "frame-" << std::setw(6) << std::setfill('0') << ordinal;
            std::ostringstream detections; detections << std::setprecision(15) << '[';
            for (std::size_t d = 0; d < result.detections.size(); ++d) { if(d) detections << ','; auto &b = result.detections[d];
                detections << "{\"class_id\":" << b.class_id << ",\"score\":" << b.confidence << ",\"x1\":" << b.x1 << ",\"y1\":" << b.y1 << ",\"x2\":" << b.x2 << ",\"y2\":" << b.y2 << '}'; }
            detections << ']';
            frames << "{\"ordinal\":" << ordinal
                   << ",\"frame_id\":" << ab::j::quote(std::to_string(f.id))
                   << ",\"timestamp_ns\":" << ab::j::quote(std::to_string(f.ns))
                   << ",\"key\":" << ab::j::quote(f.key)
                   << ",\"left_digest\":" << ab::j::quote(lhash)
                   << ",\"right_digest\":" << ab::j::quote(rhash) << ",\"mask_digest\":"
                   << ab::j::quote(digest(result.mask.data(), result.mask.size()))
                   << ",\"q8_digest\":" << ab::j::quote(qhash)
                   << ",\"detection_count\":" << result.detections.size()
                   << ",\"width\":" << f.k.w << ",\"height\":" << f.k.h
                   << ",\"detections\":" << detections.str()
                   << ",\"segmentation_mask\":" << ab::j::quote(artifact.str()+"/segmentation.png")
                   << ",\"disparity_q8\":" << ab::j::quote(artifact.str()+"/disparity_q8.u16")
                   << ",\"camera_points\":" << (result.mapping ? result.mapping->camera_points : 0)
                   << ",\"ground_points\":" << (result.mapping ? result.mapping->ground_points : 0)
                   << ",\"obstacle_points\":" << (result.mapping ? result.mapping->obstacle_points : 0) << ",\"roi\":["
                   << roi.x << ',' << roi.y << ',' << roi.w << ',' << roi.h << "],\"intrinsics\":["
                   << f.k.w << ',' << f.k.h << ',' << f.k.fx << ',' << f.k.fy << ',' << f.k.cx
                   << ',' << f.k.cy << ',' << f.k.bxf << "],\"rss_kib\":" << rss->second
                   << ",\"ms\":{";
            bool first = true;
            for (auto &x : result.ms) {
                samples[x.first].push_back(x.second);
                if (!first)
                    frames << ',';
                first = false;
                frames << ab::j::quote(x.first) << ':' << x.second;
            }
            frames << "}}\n";
            if (!frames)
                throw std::runtime_error("Metric write failed");
            if (count == c.frames) {
                wall = ms(window);
                cpu = r29::processCpuMs() - cpu_start;
                if (extra_pid)
                    extra_cpu = pidCpu(extra_pid) - extra_start;
            }
        }
        if (count != c.frames)
            throw std::runtime_error(
                "Interrupted before requested measured frames; no successful benchmark summary");
        bpu.end();
        frames.close();
        if (c.save_every == 1) ab::fs::copy_file(output / "frames.jsonl", output / "predictions.jsonl");
        auto roi = ab::region(c, method, suite);
        std::ofstream summary(output / "summary.json");
        summary << std::setprecision(15)
                << "{\"complete\":true,\"source\":" << ab::j::quote(c.source)
                << ",\"method\":" << ab::j::quote(method) << ",\"suite\":" << ab::j::quote(suite)
                << ",\"roi\":[" << roi.x << ',' << roi.y << ',' << roi.w << ',' << roi.h
                << "],\"sample_step\":" << c.sample_step << ",\"measured_frames\":" << count
                << ",\"warmup_frames\":" << c.warmup << ",\"wall_ms\":" << wall
                << ",\"pipeline_fps\":" << count * 1000.0 / wall << ",\"process_cpu_ms\":" << cpu
                << ",\"process_cpu_percent\":" << cpu / wall * 100
                << ",\"cpu_ms_per_frame\":" << cpu / count << ",\"external_publisher_cpu_ms\":";
        if (extra_pid)
            summary << extra_cpu;
        else
            summary << "null";
        summary << ",\"group_cpu_percent\":";
        if (extra_pid)
            summary << (cpu + extra_cpu) / wall * 100;
        else
            summary << "null";
        summary << ",\"resource_scope\":\"worker_process_including_its_camera_SDK_threads;ROS_"
                   "publisher_separate_unless_PID_given\",\"source_metadata\":"
                << source->metadata() << ",\"pipeline_metadata\":" << pipeline->metadata()
                << ",\"bpu\":" << bpu.summaryJson() << ",\"metrics\":{";
        bool first = true;
        for (auto &x : samples) {
            if (!first)
                summary << ',';
            first = false;
            summary << ab::j::quote(x.first) << ':' << ab::stats(x.second);
        }
        summary << "}}\n";
        summary.close();
        if (!summary)
            throw std::runtime_error("Summary write failed");
        std::cout << method << " completed frames=" << count << " FPS=" << count * 1000.0 / wall
                  << " output=" << output.u8string() << '\n';
        return 0;
    } catch (const std::exception &e) {
        if (owns_output && ab::fs::is_directory(output) &&
            !ab::fs::exists(output / "summary.json")) {
            std::ofstream f(output / "failure.json");
            f << "{\"complete\":false,\"error\":" << ab::j::quote(e.what()) << "}\n";
        }
        std::cerr << "Benchmark error: " << e.what() << '\n';
        return 1;
    }
}
