#include "percep_demo/segmentation_post.hpp"
#include <cstddef>

namespace ai_node_percep_demo::detail {
void ArgmaxFromChw(const std::vector<float>& chw, int num_classes, int out_h,
                   int out_w, std::vector<uint8_t>& mask) {
    const size_t plane = static_cast<size_t>(out_h) * out_w;
    mask.resize(plane);
    for (int y = 0; y < out_h; ++y) {
        for (int x = 0; x < out_w; ++x) {
            const size_t pix = static_cast<size_t>(y) * out_w + x;
            int best = 0;
            float best_val = chw[pix];
            for (int c = 1; c < num_classes; ++c) {
                const float val = chw[static_cast<size_t>(c) * plane + pix];
                if (val > best_val) {
                    best_val = val;
                    best = c;
                }
            }
            mask[pix] = static_cast<uint8_t>(best);
        }
    }
}
} // namespace ai_node_percep_demo::detail
