#ifndef DET_CLOUD_H
#define DET_CLOUD_H
#include "det_seg.h"
#ifdef __cplusplus
extern "C" {
#endif
#define DS_CLOUD_ABI_VERSION 1u
#define DS_CLOUD_SEG_LABEL_BASE 100
enum { DS_CLOUD_PUBLISH_FAILED = 4 };

typedef struct ds_cloud_point {
    float x, y, z; /* metres, camera optical coordinates before matching */
    uint8_t r, g, b, reserved;
    int32_t label; /* output: -1 unknown, detection class ID, or 100 + segmentation ID */
    float confidence;
} ds_cloud_point;

typedef struct ds_cloud_input {
    const ds_cloud_point* points;
    size_t point_count;
    uint32_t image_width, image_height;
    float fx, fy, cx, cy; /* calibrated for the exact detection/mask image grid */
    uint64_t frame_id, timestamp_ns; /* must match the ds_result_view */
} ds_cloud_input;

typedef struct ds_cloud_config {
    uint32_t struct_size, abi_version;
    uint32_t fusion_enabled; /* 1: detection + segmentation fusion; 0: original box-only path */
    uint32_t publish_cloud, publish_markers;
    uint32_t keep_detection_class_mask; /* bitmask; 0 means all classes. Default person/cat/dog/charger. */
    const char* output_frame; /* default camera_optical_frame (identity); copied into result */
    const char* passable_classes; /* default grass, names or comma-separated class IDs */
    float camera_to_output[16]; /* ROW-MAJOR 4x4 matrix; default identity. Supply original cali matrix. */
    float match_score_min, match_max_depth_m, match_depth_span_m;
    float det_score_min, max_range_m, max_height_m, keep_below_cam_m;
    float sensor_height_m, det_depth_span_m, det_depth_gap_m;
    int32_t det_depth_min_pts;
    uint32_t nongrass_seg_as_obstacle, cluster_filter;
    float cluster_voxel_m;
    int32_t cluster_min_component_pts;
    uint32_t seg_mask_postprocess;
    int32_t seg_mask_majority_radius, seg_mask_close_radius, seg_mask_close_iterations;
    uint32_t seg_cluster_filter;
    int32_t seg_mask_erode_px;
    float seg_depth_gap_m;
    int32_t seg_ray_bin_px, seg_min_component_pts;
} ds_cloud_config;

typedef struct ds_cloud_marker {
    int32_t detection_index, class_id;
    float x, y, z, score; /* transformed centre, metres */
    uint8_t r, g, b, reserved;
    uint32_t point_count;
    char text[128];
} ds_cloud_marker;

typedef struct ds_cloud_view {
    const ds_cloud_point* points;
    size_t point_count;
    const ds_cloud_marker* markers;
    size_t marker_count;
    uint32_t publish_cloud;
    uint32_t clear_markers; /* box-only path: publish DELETEALL then current sphere/text markers */
    char output_frame[64];
    uint64_t frame_id, timestamp_ns;
    size_t detection_labeled, segmentation_labeled, unknown; /* Counts by label namespace, not obstacle/passability counts. */
    double matching_ms;
} ds_cloud_view;
typedef struct ds_cloud_result ds_cloud_result;
/* Invoked synchronously after matching, without holding inference/cache locks.
 * Return 0 on success. Copy borrowed data if retaining it beyond the call.
 * For ROS2 publication use det_cloud_ros2.hpp; default demo writes PLY + marker JSON.
 */
typedef int (*ds_cloud_publish_callback)(void* userdata, const ds_cloud_view* view);

DS_API void ds_cloud_default_config(ds_cloud_config* config);
/* Thread-independent CPU operation; does not create a model or require BPU hardware.
 * Input points/semantics are borrowed only during the call. Point order/count are preserved.
 * Segmentation is optional; a nonempty mask must cover the complete image grid.
 */
DS_API int ds_match_det_cloud(const ds_cloud_config*, const ds_cloud_input*, const ds_result_view*, ds_cloud_result**, char* error, size_t error_capacity);
/* Extended model contract and optional same-frame source pixel coordinates.
 * source_u/source_v are either both NULL with count 0 or both non-NULL with
 * count equal to point_count. Fusion uses these pixels (out-of-range pixels
 * remain unknown); box-only retains its
 * original floating-point reprojection. All data is borrowed during the call.
 * A nonzero legacy keep-mask can represent class IDs 0..31 only; IDs >=32 are
 * excluded by such a mask. Set it to zero after application-side filtering. */
DS_API int ds_match_det_cloud_ex(const ds_cloud_config*, const ds_cloud_input*,
    const ds_result_view*, const ds_model_contract*, const uint32_t* source_u,
    const uint32_t* source_v, size_t source_uv_count, ds_cloud_result**,
    char* error, size_t error_capacity);
DS_API int ds_cloud_result_get_view(const ds_cloud_result*, ds_cloud_view*);
DS_API void ds_cloud_result_release(ds_cloud_result**);
DS_API int ds_cloud_publish(const ds_cloud_result*, ds_cloud_publish_callback, void* userdata, char* error, size_t error_capacity);
/* Complete matchAndPublishDetCloud equivalent. On publication failure the computed
 * result is STILL returned and must be released; never retry matching blindly.
 */
DS_API int ds_match_and_publish_det_cloud(const ds_cloud_config*, const ds_cloud_input*, const ds_result_view*, ds_cloud_publish_callback, void* userdata, ds_cloud_result**, char* error, size_t error_capacity);
#ifdef __cplusplus
}
#endif
#endif
