#include "ab/runtime.hpp"
#include "perception_sdk.h"
#include "r36_stereo/nv12.hpp"
#ifdef AB_WITH_X5
#include "ab/original.hpp"
#include "r36_stereo/horizon_stereo.hpp"
#endif
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
namespace ab {
using Clock = std::chrono::steady_clock;
static double elapsed(Clock::time_point t) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}
static void checked(std::uint32_t code, const psdk_status &s) {
    if (code)
        throw std::runtime_error(std::string(s.feature) + ": " + s.detail);
}
#ifdef AB_WITH_X5
struct Pipeline::Impl {
    Config c;
    std::string method, suite;
    psdk_engine *engine = nullptr;
    std::unique_ptr<r33::MappingPipeline> mapping;
#ifdef AB_WITH_X5
    std::unique_ptr<OriginalInference> original;
    std::unique_ptr<r36stereo::HorizonStereoModel> stereo;
#endif
    ~Impl() {
        psdk_status s{};
        psdk_destroy(&engine, &s);
    }
};
Pipeline::Pipeline(const Config &c, const std::string &method, const std::string &suite,
                   const fs::path &out)
    : p_(new Impl) {
    p_->c = c;
    p_->method = method;
    p_->suite = suite;
    if (!fs::is_regular_file(c.mapping) || !fs::is_regular_file(c.extrinsics))
        throw std::invalid_argument("Missing mapping/extrinsics config");
    r33::MappingConfig m;
    m.start_config_path = c.mapping.u8string();
    m.extrinsics_path = c.extrinsics.u8string();
    m.sample_step = c.sample_step;
    m.retain_clouds = c.retain_clouds;
    m.render_maps = c.render_maps;
    m.run_calibration = true;
    auto mapping = j::load(c.mapping.u8string());
    const auto &scope = j::required(mapping, "dnn");
    if (!j::boolean(j::required(j::required(scope, "detect"), "enabled")) ||
        !j::boolean(j::required(j::required(scope, "seg"), "enabled")) ||
        !j::boolean(j::required(scope, "parallel_det_seg")) || num(scope, "run_every_n") != 1)
        throw std::invalid_argument("Benchmark requires both models every frame with parallel_det_seg=true");
    if (j::boolean(j::required(mapping, "auto_cali")))
        throw std::invalid_argument("Fair benchmark disables auto calibration");
    if (!j::boolean(j::required(j::required(mapping, "dnn"), "enabled")) ||
        !j::boolean(j::required(j::required(mapping, "patchwork"), "enabled")) ||
        !j::boolean(j::required(j::required(mapping, "label_fusion"), "enabled")))
        throw std::invalid_argument(
            "All benchmark methods require det/seg, labels, ground and maps enabled");
    if (c.run_mapping) p_->mapping = std::make_unique<r33::MappingPipeline>(m);
#ifndef AB_WITH_X5
    throw std::runtime_error("Real three-method inference requires X5 BPU build; host binary "
                             "supports validate-input and ROS transport tests only");
#else
    auto &dnn = j::required(mapping, "dnn");
    auto &det = j::required(dnn, "detect");
    auto &seg = j::required(dnn, "seg");
    if (method == "original") {
        p_->original = std::make_unique<OriginalInference>(c);
    } else {
        auto profile = out / "inference.cfg";
        std::ofstream f(profile);
        f << "det_model=" << c.det.u8string() << "\nseg_model=" << c.seg.u8string()
          << "\ndet_classes=" << num(det, "classes_num")
          << "\ndet_names=" << str(det, "class_names") << "\nseg_names=" << str(seg, "class_names")
          << "\nscore_threshold=" << real(det, "score_threshold")
          << "\nnms_threshold=" << real(det, "nms_threshold") << "\nreg_bins=" << num(det, "reg")
          << "\nnms_top_k=" << num(det, "nms_top_k")
          << "\nparallel_det_seg=1\nkeep_classes=\nseg_align_corners="
          << (j::boolean(j::required(seg, "align_corners")) ? 1 : 0)
          << "\nseg_argmax_mode=" << str(seg, "argmax_mode")
          << "\ndet_resize_type=" << num(det, "resize_type")
          << "\nseg_resize_type=" << num(seg, "resize_type")
          << "\ndet_width=864\ndet_height=480\nseg_width=832\nseg_height=480\n";
        f.close();
        if (!f)
            throw std::runtime_error("Cannot write effective inference profile");
        auto dag = out / "inference_dag.json";
        std::ofstream df(dag);
        df << "{\"config_version\":3,\"engine\":{\"worker_threads\":2,\"result_artifacts\":[\"ai."
              "detection\",\"ai.segmentation\",\"exact.detections\"]},\"tasks\":{\"input\":{"
              "\"features\":[\"input.exact.pack\"]},\"detection\":{\"features\":[\"ai.exact.det."
              "preprocess\",\"ai.exact.det.backend\",\"ai.exact.det.postprocess\"]},"
              "\"segmentation\":{\"features\":[\"ai.exact.seg.preprocess\",\"ai.exact.seg."
              "backend\",\"ai.exact.seg.postprocess\"]}},\"dag\":{\"sequence\":[\"input\",{"
              "\"parallel\":[\"detection\",\"segmentation\"]}]}}";
        df.close();
        auto profile_text = profile.u8string(), dag_text = dag.u8string(),
             det_text = c.det.u8string(), seg_text = c.seg.u8string();
        psdk_config cfg{};
        cfg.struct_size = sizeof(cfg);
        cfg.abi_version = PSDK_ABI_VERSION;
        cfg.backend = PSDK_BACKEND_BPU;
        cfg.original_profile = profile_text.c_str();
        cfg.dag_profile = dag_text.c_str();
        cfg.detection_model = det_text.c_str();
        cfg.segmentation_model = seg_text.c_str();
        psdk_status s{};
        checked(psdk_create(&cfg, &p_->engine, &s), s);
    }
    if (method == "optimized_stereo") {
        p_->stereo = std::make_unique<r36stereo::HorizonStereoModel>(c.stereo.u8string(), true);
        if (p_->stereo->input_size() != cv::Size(c.roi.w, c.roi.h))
            throw std::invalid_argument(
                "Stereo dimensions differ from configured crop; hidden resize forbidden");
    }
#endif
}
Pipeline::~Pipeline() = default;
std::string Pipeline::metadata() const {
    std::string s = "{\"inference_scope\":\"shared_det_seg_every_frame;only_disparity_method_varies\",\"schedule\":\"det_seg_parallel_then_disparity\",\"mapping_enabled\":" +
                    std::string(p_->c.run_mapping ? "true" : "false") + ",\"mapping\":" +
                    (p_->mapping ? p_->mapping->effectiveConfigJson() : "null");
#ifdef AB_WITH_X5
    if (p_->stereo)
        s += ",\"stereo_contract\":" + p_->stereo->contract_json();
#endif
    return s + "}";
}
Output Pipeline::process(const Frame &f) {
    Output out;
    auto process_start = Clock::now();
    auto ai_start = Clock::now();
#ifdef AB_WITH_X5
    if (p_->original) {
        p_->original->process(f, out);
    } else {
        psdk_frame in{};
        in.struct_size = sizeof(in);
        in.abi_version = PSDK_ABI_VERSION;
        in.format = PSDK_FORMAT_NV12;
        in.width = f.k.w;
        in.height = f.k.h;
        in.stride = f.left.step;
        in.data = f.left.ptr();
        in.bytes = f.left.total();
        in.frame_id = f.id;
        in.timestamp_ns = f.ns;
        psdk_result *raw = nullptr;
        psdk_status status{};
        checked(psdk_process(p_->engine, &in, &raw, &status), status);
        std::unique_ptr<psdk_result, decltype(&psdk_result_release)> owner(raw,
                                                                           psdk_result_release);
        psdk_result_view v{};
        v.struct_size = sizeof(v);
        v.abi_version = PSDK_ABI_VERSION;
        checked(psdk_result_get_view(raw, &v, &status), status);
        for (std::uint64_t i = 0; i < v.detection_count; ++i) {
            auto &d = v.detections[i];
            out.detections.push_back({d.class_id, d.confidence, d.x1, d.y1, d.x2, d.y2});
        }
        if (v.segmentation_bytes != std::size_t(f.k.w) * f.k.h)
            throw std::runtime_error("Optimized mask shape mismatch");
        out.mask.assign(v.segmentation_mask, v.segmentation_mask + v.segmentation_bytes);
        auto &t = v.timings;
        out.ms["det_pre"] = t.detection_pre_ms;
        out.ms["det_infer_wait"] = t.detection_infer_ms;
        out.ms["det_post"] = t.detection_post_ms;
        out.ms["seg_pre"] = t.segmentation_pre_ms;
        out.ms["seg_infer_wait"] = t.segmentation_infer_ms;
        out.ms["seg_post"] = t.segmentation_post_ms;
    }
#else
    (void)ai_start;
    throw std::runtime_error("X5 inference was not compiled");
#endif
    out.ms["det_seg_pair"] = elapsed(ai_start);
    auto disparity_start = Clock::now();
#ifdef AB_WITH_X5
    if (p_->stereo) {
        auto pre = Clock::now();
        cv::Rect roi(p_->c.roi.x, p_->c.roi.y, p_->c.roi.w, p_->c.roi.h);
        auto l = r36stereo::crop_nv12(f.left, roi), r = r36stereo::crop_nv12(f.right, roi);
        out.ms["stereo_pre"] = elapsed(pre);
        auto backend = Clock::now();
        auto pred = p_->stereo->infer_nv12(l, r);
        out.ms["stereo_backend_wait_decode"] = elapsed(backend);
        auto post = Clock::now();
        out.q8.assign(std::size_t(f.k.w) * f.k.h, 0);
        for (int y = 0; y < pred.rows; ++y)
            for (int x = 0; x < pred.cols; ++x) {
                float d = pred.at<float>(y, x);
                if (std::isfinite(d) && d > 0 && d < 256)
                    out.q8[std::size_t(y + roi.y) * f.k.w + x + roi.x] =
                        std::uint16_t(std::clamp(std::lround(d * 256), 1L, 65535L));
            }
        out.ms["stereo_post"] = elapsed(post);
    } else
#endif
    {
        out.q8 = f.q8;
        maskQ8(out.q8, f.k.w, f.k.h, region(p_->c, p_->method, p_->suite));
    }
    out.ms["disparity_prepare_or_stereo"] = elapsed(disparity_start);
    if (!p_->c.run_mapping) { out.ms["process"] = elapsed(process_start); return out; }
    auto rgb_start = Clock::now();
    cv::Mat rgb;
    cv::cvtColor(f.left, rgb, cv::COLOR_YUV2RGB_NV12);
    out.ms["rgb_convert"] = elapsed(rgb_start);
    r33::MappingFrame m;
    m.frame_id = f.id;
    m.width = f.k.w;
    m.height = f.k.h;
    m.disparity = out.q8.data();
    m.disparity_bytes = out.q8.size() * 2;
    m.disparity_stride_samples = f.k.w;
    m.intrinsics = {float(f.k.fx), float(f.k.fy), float(f.k.cx), float(f.k.cy),
                    float(f.k.bxf / 1000)};
    m.rgb = rgb.ptr();
    m.rgb_bytes = rgb.total() * 3;
    m.semantic_result_valid = true;
    m.semantic_frame_id = f.id;
    m.detections = out.detections;
    m.segmentation_mask = out.mask.data();
    m.segmentation_bytes = out.mask.size();
    m.segmentation_width = f.k.w;
    m.segmentation_height = f.k.h;
    auto timed = [&](const char *name, auto fn) {
        auto t = Clock::now();
        fn();
        out.ms[name] = elapsed(t);
    };
    timed("mapping_begin_copy", [&] { p_->mapping->beginFrame(m); });
    timed("cloud_projection", [&] { p_->mapping->cloud(); });
    timed("calibration_dispatch", [&] { p_->mapping->calibration(); });
    timed("labels", [&] { p_->mapping->labels(); });
    timed("transform", [&] { p_->mapping->transform(); });
    timed("ground", [&] { p_->mapping->ground(); });
    timed("maps", [&] { p_->mapping->maps(); });
    out.mapping = p_->mapping->result();
    out.ms["process"] = elapsed(process_start);
    return out;
}
#else
struct Pipeline::Impl {};
Pipeline::Pipeline(const Config &, const std::string &, const std::string &, const fs::path &) {
    throw std::runtime_error(
        "Host build is input/ROS transport only; real inference and mapping require the X5 build");
}
Pipeline::~Pipeline() = default;
std::string Pipeline::metadata() const {
    return "{}";
}
Output Pipeline::process(const Frame &) {
    throw std::runtime_error("No X5 BPU in host build");
}
#endif
static void ply(const fs::path &p, const std::vector<r33::MappingPoint> &points) {
    std::ofstream f(p);
    f << "ply\nformat ascii 1.0\nelement vertex " << points.size()
      << "\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty "
         "uchar green\nproperty uchar blue\nproperty int label\nproperty float "
         "confidence\nend_header\n"
      << std::setprecision(9);
    for (auto &x : points)
        f << x.x << ' ' << x.y << ' ' << x.z << ' ' << unsigned(x.r) << ' ' << unsigned(x.g) << ' '
          << unsigned(x.b) << ' ' << x.label << ' ' << x.confidence << '\n';
    if (!f)
        throw std::runtime_error("PLY write failed");
}
void saveOutput(const fs::path &p, const Frame &f, const Output &out) {
    if (!fs::create_directory(p))
        throw std::runtime_error("Result directory exists");
    if (out.mapping) {
    ply(p / "camera.ply", out.mapping->camera_cloud);
    ply(p / "semantic.ply", out.mapping->semantic_cloud);
    ply(p / "robot.ply", out.mapping->robot_cloud);
    ply(p / "ground.ply", out.mapping->ground_cloud);
    ply(p / "obstacle.ply", out.mapping->obstacle_cloud);
    }
    std::ofstream q(p / "disparity_q8.u16", std::ios::binary);
    for (auto d : out.q8) {
        q.put(char(d & 255));
        q.put(char(d >> 8));
    }
    q.close();
    if (!q)
        throw std::runtime_error("Q8 save failed");
    auto image = [&](const char *name, const cv::Mat &value) {
        if (!cv::imwrite((p / name).u8string(), value))
            throw std::runtime_error(std::string("Image save failed: ") + name);
    };
    image("segmentation.png",
                cv::Mat(f.k.h, f.k.w, CV_8UC1, const_cast<std::uint8_t *>(out.mask.data())));
    cv::Mat bgr;
    cv::cvtColor(f.left, bgr, cv::COLOR_YUV2BGR_NV12);
    image("left.png", bgr);
    cv::cvtColor(f.right, bgr, cv::COLOR_YUV2BGR_NV12);
    image("right.png", bgr);
    std::ofstream meta(p / "frame.json");
    meta << "{\"frame_id\":" << j::quote(std::to_string(f.id))
         << ",\"timestamp_ns\":" << j::quote(std::to_string(f.ns)) << ",\"key\":" << j::quote(f.key)
         << "}\n";
}
} // namespace ab
