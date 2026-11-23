#pragma once

namespace perception::detail {

// Private diagnostic data for the most recent segmentation decode on this
// thread. It is consumed by the A/B benchmark after the timed postprocess.
struct SegmentationDecodeTrace {
    bool valid = false;
    bool shortcut_eligible = false;
    bool fallback = false;
    int fallback_y = -1;
    int fallback_certified = -1;
    int first_128_certified = -1;
};

SegmentationDecodeTrace takeLastSegmentationDecodeTrace() noexcept;

} // namespace perception::detail
