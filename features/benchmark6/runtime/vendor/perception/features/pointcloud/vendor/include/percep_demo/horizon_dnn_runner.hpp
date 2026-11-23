/**
 * @file horizon_dnn_runner.hpp
 * @brief Horizon BPU inference wrapper for YOLO detect + PIDNet seg.
 */
#ifndef AI_NODE_PERCEP_DEMO_HORIZON_DNN_RUNNER_HPP_
#define AI_NODE_PERCEP_DEMO_HORIZON_DNN_RUNNER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace ai_node_percep_demo {

struct DnnDetectConfig {
    bool enabled = true;
    std::string model_path;
    float score_threshold = 0.25f;
    float nms_threshold = 0.70f;
    int classes_num = 0;  // 0=从当前模型的分类 head 自动推断
    int reg = 16;
    int resize_type = 1;  // 0=direct, 1=letterbox
    int nms_top_k = 300;
    // 当前 mmyolo0828 为 24 类；id = 逗号分隔顺序下标。
    std::string class_names =
        "cobblestone,puddle,flagstone,tree_root,tree_pit,flower,potted_plant,"
        "leaf_piles,person,cat,dog,hedgehog,stone,fence,step,trapped_furniture,"
        "toy,tool,charger,manhole_cover,shoe,water_pipe,slide,trampoline";
    // runner 初始化时解析；非空时在候选生成/NMS/top-k 前只读取这些类别。
    // 空串保持兼容：处理并返回全部类别。
    std::string keep_classes = "person,cat,dog,charger";
};

struct DnnSegConfig {
    bool enabled = true;
    std::string model_path;
    // -1 selects the legacy shape-based heuristic. Set an explicit index for
    // multi-output models so an auxiliary head cannot be selected by size.
    int output_index = -1;
    // 可省略：优先用 class_names 条数，启动时再以 bin 输出通道数为准
    int num_classes = 0;
    int resize_type = 0;
    // Legacy configuration compatibility. NV12 model input does not consume
    // these two fields; change class_names to alter the class-id mapping.
    bool bgr_input = false;
    bool swap_classes = false;
    bool align_corners = true;
    std::string argmax_mode = "bilinear";
    // 逗号分隔，顺序即 class_id；也可写 "0:no_grass,1:grass,2:flagstone"
    std::string class_names = "no_grass,grass,flagstone";
};

struct DnnConfig {
    bool enabled = false;
    // Validated by runner; frame skipping itself is consumed by dnn_runtime.
    int run_every_n = 1;
    // Either flag also enables overlay generation in infer*(). Supplying a
    // non-null overlay_rgb pointer alone intentionally does not enable it.
    bool publish_overlay = false;
    bool save_overlay = false;
    std::string save_overlay_dir =
        "/userdata/daye/perception_output/dnn_overlay";
    uint32_t save_overlay_every_n = 1;
    uint32_t save_overlay_max_count = 200;
    bool parallel_det_seg = true;
    // true=overlay 按相机原图分辨率发布；false=使用 overlay_pub_width/height
    bool overlay_use_original_resolution = false;
    uint32_t overlay_pub_width = 480;   // 0=不缩放，按原图发布
    uint32_t overlay_pub_height = 360;
    // 宿主保存 overlay JPEG 时使用；当前 ROS overlay publisher 仍发布 raw Image。
    int overlay_jpeg_quality = 70;  // 1-100
    bool overlay_blend_seg = true;     // 在发布分辨率上融合分割着色
    float overlay_seg_alpha = 0.20f;
    // Parsed by the host. inferNv12() receives the resolved src_is_nv12 flag.
    // 左目 YUV 布局：nv12（Metoak RGBD 默认）| i420 | auto（首帧启发式，易误判偏色）
    std::string left_yuv_format = "nv12";
    DnnDetectConfig detect;
    DnnSegConfig seg;
};

struct DnnDetection {
    int class_id = -1;
    float confidence = 0.f;
    float x1 = 0.f;
    float y1 = 0.f;
    float x2 = 0.f;
    float y2 = 0.f;
    std::string class_name;
};

struct DnnInferTiming {
    double detect_ms = 0;
    double detect_pre_ms = 0;
    double detect_bpu_ms = 0;
    double seg_ms = 0;
    double seg_pre_ms = 0;
    double seg_bpu_ms = 0;
    double seg_post_ms = 0;
};

struct DnnInferStatus {
    bool detect_requested{false};
    bool detect_succeeded{false};
    bool segmentation_requested{false};
    bool segmentation_succeeded{false};

    bool anySucceeded() const noexcept {
        return detect_succeeded || segmentation_succeeded;
    }

    bool allRequestedSucceeded() const noexcept {
        return (!detect_requested || detect_succeeded) &&
               (!segmentation_requested || segmentation_succeeded);
    }
};

struct YuvFormatProbe {
    int width = 0;
    int height = 0;
    int stride_y = 0;
    std::size_t bytes_used = 0;
    const char* likely_format = "unknown";
    uint8_t center_r_nv12 = 0;
    uint8_t center_g_nv12 = 0;
    uint8_t center_b_nv12 = 0;
    uint8_t center_r_i420 = 0;
    uint8_t center_g_i420 = 0;
    uint8_t center_b_i420 = 0;
    int center_rgb_max_delta = 0;
};

void ProbeYuv420Format(const uint8_t* data, int w, int h, int stride_y,
                       std::size_t bytes_used, YuvFormatProbe* probe);

class HorizonDnnRunner {
public:
    HorizonDnnRunner();
    ~HorizonDnnRunner();

    HorizonDnnRunner(const HorizonDnnRunner&) = delete;
    HorizonDnnRunner& operator=(const HorizonDnnRunner&) = delete;

    bool init(const DnnConfig& cfg);
    // A ready detector runs on every infer*() call even when detections is
    // null (the result is discarded). Segmentation runs only when seg_mask is
    // non-null, allowing the host to schedule it less frequently. Overlay
    // also requires publish_overlay or save_overlay plus a non-null output.
    // `rgb` must expose at least width*height*3 tightly packed RGB bytes. For
    // camera/SHM input prefer inferNv12(), whose bytes_used can be validated.
    bool infer(const uint8_t* rgb, int width, int height,
               std::vector<DnnDetection>* detections,
               std::vector<uint8_t>* seg_mask,
               std::vector<uint8_t>* overlay_rgb,
               DnnInferTiming* timing = nullptr,
               DnnInferStatus* status = nullptr);
    // Returns true when at least one requested model branch succeeds. Inspect
    // DnnInferStatus::allRequestedSucceeded() to detect a partial result.
    // The output mask, when requested and successful, is width*height uint8
    // class ids; detection coordinates are clipped to the source image.
    bool inferNv12(const uint8_t* nv12, int width, int height, int src_stride_y,
                   std::size_t bytes_used, bool src_is_nv12,
                   std::vector<DnnDetection>* detections,
                   std::vector<uint8_t>* seg_mask,
                   std::vector<uint8_t>* overlay_rgb,
                   DnnInferTiming* timing = nullptr,
                   DnnInferStatus* status = nullptr);

    DnnConfig config() const;
    int input_w_detect() const;
    int input_h_detect() const;
    int input_w_seg() const;
    int input_h_seg() const;
    int seg_num_classes() const;

private:
    DnnConfig cfg_;
    bool detect_ready_{false};
    bool seg_ready_{false};
    int detect_input_w_{0};
    int detect_input_h_{0};
    int seg_input_w_{0};
    int seg_input_h_{0};
    // init_mtx_ serializes session replacement while infer_mtx_ protects the
    // currently committed engines and their reusable tensors.
    std::mutex init_mtx_;
    mutable std::mutex infer_mtx_;

    struct DetectEngine;
    struct SegEngine;
    struct ParallelExecutor;
    std::unique_ptr<DetectEngine> detect_;
    std::unique_ptr<SegEngine> seg_;
    std::unique_ptr<ParallelExecutor> parallel_executor_;
};

}  // namespace ai_node_percep_demo

#endif  // AI_NODE_PERCEP_DEMO_HORIZON_DNN_RUNNER_HPP_
