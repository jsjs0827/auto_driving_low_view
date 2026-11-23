#ifndef PERCEPTION_TEMPORAL_SDK_H_INCLUDED
#define PERCEPTION_TEMPORAL_SDK_H_INCLUDED
#include "perception_full_sdk.h"
#ifdef __cplusplus
extern "C" {
#endif
#define PSDK_TEMPORAL_ABI_VERSION 1u
#define PSDK_CAP_TEMPORAL 64u
/* Oldest to newest sampled successful captures, including the current capture
 * only when sampled=1. All pointers belong to the full result until release.
 * Callback pointers are borrowed and expire when the callback returns. */
typedef struct psdk_temporal_frame {
    uint64_t sequence;
    psdk_frame left, hires;
    psdk_camera_intrinsics intrinsics;
    psdk_disparity_q8 disparity;
    uint32_t has_hires, has_disparity, has_inference, reserved;
    psdk_result_view inference;
} psdk_temporal_frame;
typedef struct psdk_temporal_match {
    uint32_t previous_index, current_index;
    int32_t class_id;
    float iou;
} psdk_temporal_match;
typedef struct psdk_temporal_view {
    uint32_t struct_size, abi_version, capacity, frame_interval;
    uint32_t frame_count, sampled, ready, difference_valid;
    uint64_t current_frame_id, current_timestamp_ns, payload_bytes;
    const psdk_temporal_frame* frames;
    uint64_t previous_frame_id, previous_timestamp_ns, changed_luma_pixels;
    double mean_abs_luma;
    uint64_t compared_mask_pixels, changed_mask_pixels;
    const psdk_temporal_match* matches;
    uint32_t match_count, matching_valid, matching_truncated, memory_limited;
    double capture_ms, difference_ms, match_ms;
    const char* reset_reason;
    const char* analysis_json;
} psdk_temporal_view;
/* Executed as product.temporal.extension after all current-frame producers and
 * enabled temporal analysis, before publication/record. A nonzero return fails
 * that DAG invocation; the failed capture is never committed to history.
 * No reentry into process/command/reset/destroy from this callback. */
typedef uint32_t (*psdk_temporal_callback)(void*, const psdk_temporal_view*, psdk_status*);
PSDK_API uint32_t psdk_full_set_temporal_callback(psdk_full_engine*,
    psdk_temporal_callback, void*, psdk_status*);
PSDK_API uint32_t psdk_full_result_get_temporal_view(const psdk_full_result*,
    psdk_temporal_view*, psdk_status*);
PSDK_API uint32_t psdk_full_reset_history(psdk_full_engine*, psdk_status*);
#ifdef __cplusplus
}
#endif
#endif
