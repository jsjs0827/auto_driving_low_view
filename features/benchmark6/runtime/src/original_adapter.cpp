#include "ab/original.hpp"
#include "ab/runtime.hpp"
#include "percep_demo/horizon_dnn_runner.hpp"
namespace ab {
struct OriginalInference::Impl {
    ai_node_percep_demo::HorizonDnnRunner runner;
};
OriginalInference::OriginalInference(const Config &c) : p_(new Impl) {
    auto mapping = j::load(c.mapping.u8string());
    auto &dnn = j::required(mapping, "dnn");
    auto &det = j::required(dnn, "detect");
    auto &seg = j::required(dnn, "seg");
    ai_node_percep_demo::DnnConfig d;
    d.enabled = true;
    d.parallel_det_seg = true;
    d.left_yuv_format = "nv12";
    d.detect.model_path = c.det.u8string();
    d.seg.model_path = c.seg.u8string();
    d.detect.score_threshold = real(det, "score_threshold");
    d.detect.nms_threshold = real(det, "nms_threshold");
    d.detect.nms_top_k = num(det, "nms_top_k");
    d.detect.classes_num = num(det, "classes_num");
    d.detect.reg = num(det, "reg");
    d.detect.resize_type = num(det, "resize_type", 0, 1);
    d.detect.class_names = str(det, "class_names");
    d.detect.keep_classes = "";
    d.seg.class_names = str(seg, "class_names");
    d.seg.align_corners = j::boolean(j::required(seg, "align_corners"));
    d.seg.argmax_mode = str(seg, "argmax_mode");
    d.seg.resize_type = num(seg, "resize_type", 0, 1);
    d.seg.output_index = -1;
    if (!p_->runner.init(d))
        throw std::runtime_error("Original production HorizonDnnRunner init failed");
}
OriginalInference::~OriginalInference() = default;
void OriginalInference::process(const Frame &f, Output &out) {
    std::vector<ai_node_percep_demo::DnnDetection> detections;
    ai_node_percep_demo::DnnInferTiming t;
    ai_node_percep_demo::DnnInferStatus status;
    if (!p_->runner.inferNv12(f.left.ptr(), f.k.w, f.k.h, int(f.left.step), f.left.total(), true,
                              &detections, &out.mask, nullptr, &t, &status) ||
        !status.allRequestedSucceeded())
        throw std::runtime_error("Original inference failed/partial result");
    for (auto &d : detections)
        out.detections.push_back({d.class_id, d.confidence, d.x1, d.y1, d.x2, d.y2});
    out.ms["det_pre"] = t.detect_pre_ms;
    out.ms["det_infer_wait"] = t.detect_bpu_ms;
    out.ms["det_post"] = t.detect_ms - t.detect_pre_ms - t.detect_bpu_ms;
    out.ms["seg_pre"] = t.seg_pre_ms;
    out.ms["seg_infer_wait"] = t.seg_bpu_ms;
    out.ms["seg_post"] = t.seg_post_ms;
}
} // namespace ab
