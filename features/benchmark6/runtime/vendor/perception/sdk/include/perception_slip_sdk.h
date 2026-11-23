#ifndef PERCEPTION_SLIP_SDK_H_INCLUDED
#define PERCEPTION_SLIP_SDK_H_INCLUDED
#include "perception_full_sdk.h"
#ifdef __cplusplus
extern "C" {
#endif
#define PSDK_SLIP_ABI_VERSION 1u
#define PSDK_CAP_SLIP 128u
#define PSDK_SLIP_DISABLED 0u
#define PSDK_SLIP_INITIALIZING 1u
#define PSDK_SLIP_UNAVAILABLE 2u
#define PSDK_SLIP_NORMAL 3u
#define PSDK_SLIP_SUSPECTED 4u
#define PSDK_SLIP_SLIPPING 5u
#define PSDK_SLIP_RECOVERING 6u
/* Measured wheel/encoder-derived BODY velocity, not commanded velocity.
 * BODY x is forward, yaw is right-handed about BODY z. Timestamp is in the
 * SAME calibrated clock domain as image capture, not receipt/playback time.
 * Current version integrates wheel odometry; it does not fuse IMU samples. */
typedef struct psdk_motion_sample {
    uint32_t struct_size, abi_version;
    uint64_t timestamp_ns;
    double linear_speed_mps, yaw_rate_radps;
    uint32_t reserved[2];
} psdk_motion_sample;
typedef struct psdk_slip_view {
    uint32_t struct_size, abi_version, enabled, state;
    uint32_t visual_valid, reference_valid, body_calibration_confirmed, reserved;
    uint64_t previous_frame_id, frame_id, start_timestamp_ns, end_timestamp_ns;
    double dt_s, visual_forward_m, visual_lateral_m, visual_yaw_rad;
    double reference_forward_m, reference_lateral_m, reference_yaw_rad;
    /* Speed is NaN when the corresponding motion is unavailable; slip_ratio
     * is NaN when no valid slip comparison exists. JSON renders these null.
     * Inspect state/visual_valid/reference_valid before consuming values. */
    double visual_speed_mps, reference_speed_mps, slip_ratio;
    double visual_quality, visual_rmse_m;
    uint32_t matches, inliers;
    double visual_ms, reference_ms, slip_ms;
    char state_name[32], reason[256];
    /* Immutable JSON owned by the result until result_release; never retain
     * borrowed pointers beyond result lifetime. Includes quality diagnostics. */
    const char* details_json;
} psdk_slip_view;
/* Bounded, all-or-nothing batch. Samples must be finite and monotonic.
 * An invalid batch never partially updates the engine's time buffer.
 * Call before process; forbidden from full SDK callbacks. */
PSDK_API uint32_t psdk_full_push_motion_samples(psdk_full_engine*,
    const psdk_motion_sample*, uint32_t count, psdk_status*);
PSDK_API uint32_t psdk_full_result_get_slip_view(const psdk_full_result*,
    psdk_slip_view*, psdk_status*);
#ifdef __cplusplus
}
#endif
#endif
