#pragma once
#include <cstdint>
#include <vector>

namespace ai_node_percep_demo::detail {
// Caller validates a nonempty CHW tensor, positive geometry, and <=256 classes.
// Equal logits retain the first class, matching the preserved original loop.
void ArgmaxFromChw(const std::vector<float>& chw, int num_classes, int out_h,
                   int out_w, std::vector<uint8_t>& mask);
} // namespace ai_node_percep_demo::detail
