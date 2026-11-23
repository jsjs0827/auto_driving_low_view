#ifndef PERCEPTION_FULL_SDK_H_INCLUDED
#define PERCEPTION_FULL_SDK_H_INCLUDED
#include "perception_sdk.h"
#include "perception_sensor_input.h"
#include "perception_raw_input.h"
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

#define PSDK_FULL_ABI_VERSION 1u
#define PSDK_CAP_MAPPING 8u
#define PSDK_CAP_RECHARGE 16u
#define PSDK_CAP_RGBD_INPUT 32u
typedef struct psdk_full_engine psdk_full_engine;
typedef struct psdk_full_result psdk_full_result;

/* Original camera calibration unit: bxf = focal length * baseline_mm.
 * Point cloud projection converts bxf to meters exactly once. */
typedef struct psdk_camera_intrinsics {
    uint32_t width, height;
    double fx, fy, cx, cy, bxf;
} psdk_camera_intrinsics;
typedef struct psdk_disparity_q8 {
    uint32_t struct_size, abi_version, width, height, stride_bytes, reserved;
    const uint16_t* data;
    uint64_t bytes, frame_id, timestamp_ns;
} psdk_disparity_q8;
typedef struct psdk_full_frame {
    uint32_t struct_size, abi_version;
    psdk_frame left;
    const psdk_frame* recharge_hires; /* Optional rectified GDC left; same identity required. */
    const psdk_disparity_q8* disparity;
    psdk_camera_intrinsics intrinsics;
    /* V4 additive tail. Same-capture right image, retained by the caller until
     * full_process returns. NV12/I420 or packed BGR/RGB. Old callers may pass
     * PSDK_FULL_FRAME_V1_SIZE; the SDK never reads this tail in that case. */
    const psdk_frame* right;
    /* Optional input slots. Rear image may have independent geometry/capture
     * identity and accepts NV12/I420/BGR/RGB. Sensor timestamps use the camera
     * clock, but these slots do not synchronize or fuse themselves. NULL leaves
     * the corresponding input artifact absent. All storage is caller-owned. */
    const psdk_frame* rear;
    const psdk_imu_input* imu;
    const psdk_wheel_input* wheel;
    const psdk_radar_input* radar;
    /* Optional native RAW pair + ORG exposure proxy for registered raw_lux.
     * Descriptors are copied, image buffers remain borrowed until return. */
    const psdk_raw_stereo_input* raw_light;
} psdk_full_frame;
#define PSDK_FULL_FRAME_V1_SIZE ((uint32_t)offsetof(psdk_full_frame, right))
/* Size of the previous input ABI including right, before optional sensor slots. */
#define PSDK_FULL_FRAME_V2_SIZE ((uint32_t)offsetof(psdk_full_frame, rear))
/* Previous full size with rear/IMU/wheel/radar and no RAW-light tail. */
#define PSDK_FULL_FRAME_V3_SIZE ((uint32_t)offsetof(psdk_full_frame, raw_light))

typedef struct psdk_grid_cell {
    float min_h_m, max_h_m, avg_h_m;
    int32_t type;
} psdk_grid_cell;
typedef struct psdk_topview {
    char name[16]; /* pcl, ai, fusion */
    uint32_t rows, cols, resolution_mm, image_width, image_height;
    uint64_t cell_count;
    const psdk_grid_cell* cells;
    const uint8_t* rgb; /* Optional original rendered RGB image, including original row reversal. */
    uint64_t rgb_bytes;
} psdk_topview;
typedef struct psdk_cloud_point {
    float x, y, z, confidence;
    int32_t label;
    uint8_t r, g, b, reserved;
} psdk_cloud_point;
typedef struct psdk_cloud_view {
    char name[24]; /* camera, robot, ground, obstacle, semantic */
    const psdk_cloud_point* points;
    uint64_t point_count;
} psdk_cloud_view;
typedef struct psdk_dock_pose {
    uint32_t executed, mark_ok, pose_ok, detected_point_count;
    uint64_t frame_id;
    int64_t timestamp_ns;
    float x_m, y_m, yaw_rad;
    double score, distance_mm, reprojection_rmse_px;
    double tvec_mm[3], rvec[3];
    char error[256], pnp_error[256], crop_source[16];
    const uint8_t* overlay_rgb;
    uint32_t overlay_width, overlay_height;
    uint64_t overlay_bytes;
    const char* details_json; /* Additional original MarkPoseResult diagnostics. */
} psdk_dock_pose;
typedef struct psdk_calibration_result {
    uint32_t active, completed, accumulated_frames, reserved;
    double x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg;
} psdk_calibration_result;
typedef struct psdk_full_result_view {
    uint32_t struct_size, abi_version, has_inference, map_count;
    uint64_t frame_id, timestamp_ns, run_id;
    psdk_result_view inference;
    psdk_disparity_q8 disparity; /* Owned tight Q8 data when the disparity stage ran. */
    const uint8_t* rgb;
    uint64_t rgb_bytes;
    uint32_t image_width, image_height;
    const psdk_topview* maps;
    const int16_t* avg_h_cm;
    const int16_t* min_h_cm;
    const int16_t* max_h_cm;
    const uint8_t* grid_types;
    uint64_t grid_count;
    const psdk_cloud_view* clouds;
    uint32_t cloud_count, nearest_obstacle_valid;
    int32_t nearest_obstacle_index;
    float nearest_obstacle_x_m, nearest_obstacle_y_m, nearest_obstacle_distance_m;
    psdk_dock_pose recharge;
    psdk_calibration_result calibration;
    double graph_ms, process_ms;
    const char* task_trace_json;
    const char* effective_config_json;
} psdk_full_result_view;

/* Provider's borrowed disparity remains valid until full_process returns.
 * No proprietary stereo model is fabricated: use camera SDK Q8 data, or
 * implement this callback using the application's actual stereo producer.
 * Callbacks execute within Features and must not reenter full SDK calls. */
typedef uint32_t (*psdk_disparity_provider)(void*, const psdk_frame*,
    psdk_disparity_q8*, psdk_status*);
/* Publication sees completed same-frame payloads. Final graph timing/trace
 * become available in the result returned by full_process after all Tasks. */
typedef uint32_t (*psdk_full_publication)(void*, const psdk_full_result_view*, psdk_status*);
typedef struct psdk_full_hooks {
    uint32_t struct_size, abi_version;
    void* userdata;
    psdk_disparity_provider disparity_provider;
    psdk_full_publication publication;
} psdk_full_hooks;
typedef struct psdk_full_config {
    uint32_t struct_size, abi_version;
    const char* json_file;
    const char* detection_model; /* Optional original model path overrides. */
    const char* segmentation_model;
    const psdk_full_hooks* hooks;
} psdk_full_config;

PSDK_API uint32_t psdk_full_capabilities(void);
PSDK_API uint32_t psdk_full_create(const psdk_full_config*, psdk_full_engine**, psdk_status*);
PSDK_API uint32_t psdk_full_process(psdk_full_engine*, const psdk_full_frame*, psdk_full_result**, psdk_status*);
PSDK_API uint32_t psdk_full_result_get_view(const psdk_full_result*, psdk_full_result_view*, psdk_status*);
/* Borrowed per-task algorithm JSON remains valid until result release.
 * This getter leaves the existing result-view ABI unchanged. */
PSDK_API uint32_t psdk_full_result_get_algorithms_json(const psdk_full_result*, const char**, psdk_status*);
/* Original command meanings: 1 map, 2 recharge, 3 both, 21 stop.
 * This sets application work; camera/ROS command publication stays in caller. */
PSDK_API uint32_t psdk_full_command(psdk_full_engine*, int32_t, psdk_status*);
/* Save an owned result to a NEW directory: maps PNG/raw, pose, trace, clouds.
 * Called explicitly, or by the record Feature if configured every_n > 0. */
PSDK_API uint32_t psdk_full_save_result(const psdk_full_result*, const char*, psdk_status*);
PSDK_API void psdk_full_result_release(psdk_full_result*);
PSDK_API uint32_t psdk_full_destroy(psdk_full_engine**, psdk_status*);

/* Engines process one frame synchronously. Result views own their data until
 * result_release, including after engine destruction. Model parameters and
 * camera/SHM C++ layouts never cross this C ABI. Optional owned history is
 * configured through JSON; see perception_temporal_sdk.h. No batch inference.
 * A recharge-enabled engine is process-exclusive due to original filter state.
 * Destroy and release must never race use of the same handle. */
#ifdef __cplusplus
}
#endif
#endif
