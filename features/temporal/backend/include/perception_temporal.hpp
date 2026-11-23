#pragma once
// C++ DAG artifact contract. Pointers are immutable and owned, not camera/BPU loans.
#include "perception_temporal_sdk.h"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
namespace r36::temporal {
inline constexpr const char* input_artifact_key = "full.temporal.input_window";
inline constexpr const char* artifact_key = "full.temporal.window";
struct Settings {
    unsigned capacity=1, frame_interval=1, max_memory_mb=64;
    bool difference=false, match=false;
    unsigned difference_threshold=8, match_max_detections=256;
    double match_iou_threshold=.5;
};
struct Image {
    psdk_frame metadata{};
    std::shared_ptr<const std::vector<std::uint8_t>> storage;
    psdk_frame view()const {auto v=metadata;v.data=storage?storage->data():nullptr;return v;}
};
struct Frame {
    std::uint64_t sequence=0;
    Image left,hires;
    psdk_camera_intrinsics intrinsics{};
    std::shared_ptr<const std::vector<std::uint16_t>> disparity;
    std::vector<std::uint8_t> mask;
    std::vector<psdk_detection> detections;
    psdk_result_view inference{};
    bool has_inference=false;
    std::uint64_t payloadBytes()const;
    psdk_temporal_frame view()const;
};
struct Window {
    std::vector<std::shared_ptr<const Frame>> frames;
    unsigned capacity=1, interval=1;
    bool sampled=false, memory_limited=false;
    std::uint64_t current_id=0,current_timestamp=0,payload_bytes=0;
    std::string reset_reason;
};
struct Analysis {
    bool difference_valid=false,matching_valid=false,matching_truncated=false;
    std::uint64_t previous_id=0,previous_timestamp=0,changed_luma=0,mask_pixels=0,changed_mask=0;
    double mean_abs_luma=0,capture_ms=0,difference_ms=0,match_ms=0;
    std::vector<psdk_temporal_match> matches;
};
// History has one owner: the synchronous engine under full_gate. A Plan is an
// uncommitted preview. Publish/record/extension failures never advance it.
class History {
public:
    struct Plan {
        Window window;
        std::uint64_t unique_count=0,last_id=0,last_timestamp=0;
        bool new_capture=false;
        psdk_camera_intrinsics intrinsics{};
    };
    explicit History(Settings settings={});
    Plan prepare(const psdk_full_frame&)const;
    void complete(Plan&,const psdk_result_view*,
        std::shared_ptr<const std::vector<std::uint16_t>>)const;
    void commit(const Plan&);
    void clear();
private:
    Settings settings_;
    Window stored_;
    std::uint64_t unique_count_=0,last_id_=0,last_timestamp_=0;
    psdk_camera_intrinsics intrinsics_{};
    bool has_last_=false;
    void trim(Window&)const;
};
void difference(const Window&,const Settings&,Analysis&);
void match(const Window&,const Settings&,Analysis&);
std::string json(const Window&,const Analysis&);
// cache/view storage resides in the owning result or callback stack.
psdk_temporal_view view(const Window&,const Analysis&,
    const std::vector<psdk_temporal_frame>&,const std::string&);
}
