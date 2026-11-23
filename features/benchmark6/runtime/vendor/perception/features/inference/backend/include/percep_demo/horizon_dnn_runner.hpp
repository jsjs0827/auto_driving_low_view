/**
 * @file horizon_dnn_runner.hpp
 * @brief Horizon BPU inference wrapper for YOLO detect + PIDNet seg.
 */
#ifndef AI_NODE_PERCEP_DEMO_HORIZON_DNN_RUNNER_HPP_
#define AI_NODE_PERCEP_DEMO_HORIZON_DNN_RUNNER_HPP_

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>
#include <memory>
#include "percep_demo/execution_schedule.hpp"

namespace r25 { class NativeStages; } // R25_STAGE_FRIEND
namespace ai_node_percep_demo {

struct DnnDetectConfig {
    bool enabled = true;
    std::string model_path;
    float score_threshold = 0.25f;
    float nms_threshold = 0.70f;
    int classes_num = 21;
    int reg = 16;
    int resize_type = 1;  // 0=direct, 1=letterbox
    int nms_top_k = 300;
    // 新模型 21 类；须与 mmyolo0726.bin 输出维一致；id = 逗号分隔顺序下标
    std::string class_names =
        "cobblestone,puddle,flagstone,tree_root,tree_pit,flower,potted_plant,"
        "leaf_piles,person,cat,dog,hedgehog,stone,fence,step,trapped_furniture,"
        "toy,tool,charger,manhole_cover,shoe";
    // 对外/融合/回充只保留这些类（空=不过滤）
    std::string keep_classes = "person,cat,dog,charger";
};

struct DnnSegConfig {
    // Optional, separate experiment. Empty keeps the original board model and full-size mask.
    std::string fused_model_path;
    bool fused_vse=false;
    bool fused_only=false;
    int mask_width=0,mask_height=0;
    bool enabled = true;
    std::string model_path;
    // 可省略：优先用 class_names 条数，启动时再以 bin 输出通道数为准
    int num_classes = 0;
    int resize_type = 0;
    bool bgr_input = false;
    bool swap_classes = false;  // 未使用；换类请改 class_names
    bool align_corners = true;
    std::string argmax_mode = "bilinear";
    // 逗号分隔，顺序即 class_id；也可写 "0:no_grass,1:grass,2:flagstone"
    std::string class_names =
        "no_grass,grass,fence,manhole_cover,puddle,road,flagstone,tree_pit,"
        "waterpipe";
};

struct DnnConfig {
    bool enabled = false;
    int run_every_n = 1;
    bool publish_overlay = false;
    bool publish_seg_mask = false;
    bool save_overlay = false;
    std::string save_overlay_dir =
        "/userdata/daye/perception_output/dnn_overlay";
    uint32_t save_overlay_every_n = 1;
    uint32_t save_overlay_max_count = 200;
    bool parallel_det_seg = true;
    // Execution only: unchanged input, model and postprocess. No cross-frame overlap.
    std::string execution_schedule = "legacy"; // legacy | free | det-first | seg-first
    int bpu_priority = 0; // Original default. Optional explicit 0..255 experiment.
    // true=overlay 按相机原图分辨率发布；false=使用 overlay_pub_width/height
    bool overlay_use_original_resolution = false;
    uint32_t overlay_pub_width = 480;   // 0=不缩放，按原图发布
    uint32_t overlay_pub_height = 360;
    // JPEG 压缩后发 /percep_demo/dnn_overlay/compressed（大幅降 DDS/网络带宽）
    bool overlay_jpeg = true;
    int overlay_jpeg_quality = 70;  // 1-100
    bool overlay_publish_raw = false;  // 额外发未压缩 rgb8（调试用，费带宽）
    bool overlay_swap_rb = false;      // 交换 R/B 通道（色彩不对时切换验证）
    bool overlay_blend_seg = true;     // 在发布分辨率上融合分割着色
    float overlay_seg_alpha = 0.20f;
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
    r21::BpuTiming detect_api,seg_api;
    double detect_ms = 0;
    double detect_pre_ms = 0;
    double detect_bpu_ms = 0;
    double seg_ms = 0;
    double seg_pre_ms = 0;
    double seg_bpu_ms = 0;
    double seg_post_ms = 0;
};

struct YuvFormatProbe {
    int width = 0;
    int height = 0;
    int stride_y = 0;
    size_t bytes_used = 0;
    const char* likely_format = "unknown";
    uint8_t center_r_nv12 = 0;
    uint8_t center_g_nv12 = 0;
    uint8_t center_b_nv12 = 0;
    uint8_t center_r_i420 = 0;
    uint8_t center_g_i420 = 0;
    uint8_t center_b_i420 = 0;
    int center_rgb_max_delta = 0;
};

void ProbeYuv420Format(const uint8_t* data, int w, int h, size_t bytes_used,
                       YuvFormatProbe* probe);

class HorizonDnnRunner {
public:
    HorizonDnnRunner();
    ~HorizonDnnRunner();

    HorizonDnnRunner(const HorizonDnnRunner&) = delete;
    HorizonDnnRunner& operator=(const HorizonDnnRunner&) = delete;

    bool init(const DnnConfig& cfg);
    void setExecutionSchedule(const std::string& schedule,int priority);
    bool infer(const uint8_t* rgb, int width, int height,
               std::vector<DnnDetection>* detections,
               std::vector<uint8_t>* seg_mask,
               std::vector<uint8_t>* overlay_rgb,
               DnnInferTiming* timing = nullptr);
    bool inferNv12(const uint8_t* nv12, int width, int height, int src_stride_y,
                   size_t bytes_used, bool src_is_nv12,
                   std::vector<DnnDetection>* detections,
                   std::vector<uint8_t>* seg_mask,
                   std::vector<uint8_t>* overlay_rgb,
                   DnnInferTiming* timing = nullptr);

    const DnnConfig& config() const { return cfg_; }
    int input_w_detect() const { return detect_input_w_; }
    int input_h_detect() const { return detect_input_h_; }
    int input_w_seg() const { return seg_input_w_; }
    int input_h_seg() const { return seg_input_h_; }
    int seg_num_classes() const;

private:
    friend class r25::NativeStages; // R25_STAGE_FRIEND
    DnnConfig cfg_;
    bool detect_ready_{false};
    bool seg_ready_{false};
    int detect_input_w_{0};
    int detect_input_h_{0};
    int seg_input_w_{0};
    int seg_input_h_{0};
    std::mutex infer_mtx_;
    std::unique_ptr<r21::PersistentPair> workers_;

    struct DetectEngine;
    struct SegEngine;
    DetectEngine* detect_{nullptr};
    SegEngine* seg_{nullptr};
};

}  // namespace ai_node_percep_demo

#endif  // AI_NODE_PERCEP_DEMO_HORIZON_DNN_RUNNER_HPP_
