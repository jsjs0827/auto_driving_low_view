#include "percep_demo/detection_post.hpp"
#include "percep_demo/segmentation_post.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace post = ai_node_percep_demo::detail;
namespace {
void require(bool valid, const char* message) {
    if (!valid) throw std::runtime_error(message);
}
post::RawDetection detection(int category, float score, post::RawBox box) {
    return {category, score, box};
}
void detectionCases() {
    const post::RawBox full{0.f, 0.f, 2.f, 2.f};
    const post::RawBox half{0.f, 0.f, 1.f, 2.f};
    require(post::BoxIoU(full, full) == 1.f, "identical boxes have IoU one");
    require(post::BoxIoU(full, half) == .5f, "half overlap gives IoU one half");
    require(post::BoxIoU(full, {3.f, 3.f, 1.f, 1.f}) == 0.f, "disjoint boxes have zero IoU");
    require(post::BoxIoU({}, {}) == 0.f, "degenerate union gives zero IoU");

    const std::vector<post::RawDetection> inputs{
        detection(0, .7f, full), detection(0, .8f, {4.f, 4.f, 1.f, 1.f}),
        detection(1, .85f, full), detection(0, .9f, full)};
    auto kept = inputs;
    post::NmsPerClass(kept, .5f, 10);
    require(kept.size() == 3, "NMS suppresses the lower-score same-class duplicate");
    require(kept[0].confidence == .9f && kept[1].confidence == .85f && kept[2].confidence == .8f,
            "NMS retains descending score order and a disjoint same-class box");
    require(kept[0].class_id == 0 && kept[1].class_id == 1,
            "overlapping different classes must both survive");
    kept = inputs;
    post::NmsPerClass(kept, .5f, 2);
    require(kept.size() == 2 && kept[0].confidence == .9f && kept[1].confidence == .85f,
            "top-k applies across the score-ordered retained candidates");

    kept = {detection(0, .9f, full), detection(0, .8f, half)};
    post::NmsPerClass(kept, .5f, 10);
    require(kept.size() == 2, "IoU equal to the threshold is retained (strict greater-than)");
    kept = {detection(0, .9f, full), detection(0, .8f, half)};
    post::NmsPerClass(kept, .49f, 10);
    require(kept.size() == 1, "IoU above the threshold is suppressed");
}
void segmentationCases() {
    // Three CHW class planes for a two-by-two image. Include equal and negative logits.
    const std::vector<float> logits{1.f, 2.f, -5.f, 3.f,
                                    4.f, 2.f, -4.f, 1.f,
                                    2.f, 1.f, -4.f, 5.f};
    std::vector<std::uint8_t> mask(8, 255);
    post::ArgmaxFromChw(logits, 3, 2, 2, mask);
    require(mask == std::vector<std::uint8_t>({1, 0, 1, 2}),
            "CHW argmax follows pixel geometry, retains the first tied class, and handles negative logits");
    post::ArgmaxFromChw({-2.f, -3.f}, 1, 1, 2, mask);
    require(mask == std::vector<std::uint8_t>({0, 0}), "one-class output remains class zero");
}
} // namespace
int main() {
    try {
        detectionCases(); segmentationCases();
        std::cout << "Real pure detection/segmentation postprocess cases passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; return 1;
    }
}
