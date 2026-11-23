#include "percep_demo/detection_post.hpp"
#include <algorithm>
#include <cstddef>

namespace ai_node_percep_demo::detail {
float BoxIoU(const RawBox& a, const RawBox& b) {
    const float ax2 = a.x + a.w;
    const float ay2 = a.y + a.h;
    const float bx2 = b.x + b.w;
    const float by2 = b.y + b.h;
    const float ix1 = std::max(a.x, b.x);
    const float iy1 = std::max(a.y, b.y);
    const float ix2 = std::min(ax2, bx2);
    const float iy2 = std::min(ay2, by2);
    const float iw = std::max(0.f, ix2 - ix1);
    const float ih = std::max(0.f, iy2 - iy1);
    const float inter = iw * ih;
    const float uni = a.w * a.h + b.w * b.h - inter;
    return uni > 0.f ? inter / uni : 0.f;
}

void NmsPerClass(std::vector<RawDetection>& dets, float nms_threshold,
                 int nms_top_k) {
    std::sort(dets.begin(), dets.end(),
              [](const RawDetection& a, const RawDetection& b) {
                  return a.confidence > b.confidence;
              });
    std::vector<RawDetection> kept;
    std::vector<bool> removed(dets.size(), false);
    for (size_t i = 0; i < dets.size(); ++i) {
        if (removed[i]) continue;
        kept.push_back(dets[i]);
        if (static_cast<int>(kept.size()) >= nms_top_k) break;
        for (size_t j = i + 1; j < dets.size(); ++j) {
            if (removed[j] || dets[i].class_id != dets[j].class_id) continue;
            if (BoxIoU(dets[i].box, dets[j].box) > nms_threshold) {
                removed[j] = true;
            }
        }
    }
    dets.swap(kept);
}
} // namespace ai_node_percep_demo::detail
