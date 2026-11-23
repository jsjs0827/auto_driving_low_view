#pragma once
#include <vector>

namespace ai_node_percep_demo::detail {
// Internal decoded detector values; model sessions remain in shared inference.
struct RawBox {
    float x = 0.f;
    float y = 0.f;
    float w = 0.f;
    float h = 0.f;
};

struct RawDetection {
    int class_id = -1;
    float confidence = 0.f;
    RawBox box;
};

float BoxIoU(const RawBox& a, const RawBox& b);
void NmsPerClass(std::vector<RawDetection>& dets, float nms_threshold,
                 int nms_top_k);
} // namespace ai_node_percep_demo::detail
