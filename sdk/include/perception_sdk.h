#ifndef PERCEPTION_SDK_H_INCLUDED
#define PERCEPTION_SDK_H_INCLUDED

/* Standalone synchronous inference ABI. No ROS or C++ types cross this boundary. */
#include <stdint.h>
#if defined(_WIN32)
# if defined(PERCEPTION_SDK_BUILD)
#  define PSDK_API __declspec(dllexport)
# else
#  define PSDK_API __declspec(dllimport)
# endif
#else
# define PSDK_API __attribute__((visibility("default")))
#endif
#ifdef __cplusplus
extern "C" {
#endif

#define PSDK_ABI_VERSION 1u
#define PSDK_CAP_BPU 1u
#define PSDK_CAP_CPU_ORT 2u
#define PSDK_CAP_PNG 4u
#define PSDK_BACKEND_BPU 1u
#define PSDK_BACKEND_CPU_ORT 2u
#define PSDK_FORMAT_NV12 1u
#define PSDK_FORMAT_I420 2u
/* Packed formats are accepted for the optional full-frame right input only.
 * The standalone detector/segmenter left input remains NV12 or I420. */
#define PSDK_FORMAT_BGR 3u
#define PSDK_FORMAT_RGB 4u

typedef struct psdk_engine psdk_engine;
typedef struct psdk_result psdk_result;

typedef struct psdk_config {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t backend;
    uint32_t reserved_flags; /* Must be zero. */
    const char* original_profile; /* NULL: profiles/original.cfg, relative to working directory. */
    const char* dag_profile;      /* NULL: profiles/dag_exact.json. */
    const char* cpu_profile;      /* Required for the CPU backend; explicit tensor/decoder contract. */
    const char* detection_model;  /* NULL: use selected backend's profile model. */
    const char* segmentation_model;
} psdk_config;

typedef struct psdk_frame {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t format;
    uint32_t width;
    uint32_t height;
    uint32_t stride; /* Y stride; NV12 UV stride=Y stride, I420 U/V stride=Y stride/2. */
    const uint8_t* data;
    uint64_t bytes;
    uint64_t frame_id;
    uint64_t timestamp_ns;
} psdk_frame;

typedef struct psdk_status {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t code; /* Full framework/business code; zero means success. */
    int32_t native_code;
    uint64_t frame_id;
    uint64_t timestamp_ns;
    char feature[96];
    char task[96];
    char detail[512];
} psdk_status;

typedef struct psdk_detection {
    int32_t class_id;
    float confidence;
    float x1, y1, x2, y2; /* Source image coordinates. */
    char class_name[96];
} psdk_detection;

typedef struct psdk_timings {
    double pack_ms;
    double detection_pre_ms, detection_infer_ms, detection_post_ms;
    double segmentation_pre_ms, segmentation_infer_ms, segmentation_post_ms;
    double graph_ms, process_ms;
} psdk_timings;

typedef struct psdk_result_view {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t width;
    uint32_t height;
    uint64_t frame_id;
    uint64_t timestamp_ns;
    const psdk_detection* detections;
    uint64_t detection_count;
    const uint8_t* segmentation_mask; /* Raw class IDs, tightly packed at source dimensions. */
    uint64_t segmentation_bytes;
    psdk_timings timings;
} psdk_result_view;

/* Initialize config/frame/view with {0}, then set struct_size and abi_version.
 * Return values equal status.code; status is optional and overwritten every call.
 * Models load once at create. Process is synchronous: input data may be reused
 * after it returns. Create, process and destroy serialize across SDK engines
 * because the preserved exact-input workspaces are process-global.
 * Detection and segmentation execute in the unchanged same-frame DAG.
 * Each result owns its immutable output and survives later process calls and
 * engine destruction; view pointers expire only at result_release.
 * Coordinate callers: never race destroy/release against any use of that handle.
 * An engine owns no camera, ROS node, input queue or inter-frame double buffer.
 */
PSDK_API uint32_t psdk_abi_version(void);
PSDK_API uint32_t psdk_build_capabilities(void);
PSDK_API uint32_t psdk_create(const psdk_config*, psdk_engine**, psdk_status*);
PSDK_API uint32_t psdk_process(psdk_engine*, const psdk_frame*, psdk_result**, psdk_status*);
PSDK_API uint32_t psdk_result_get_view(const psdk_result*, psdk_result_view*, psdk_status*);
/* Optional overlay output is outside inference timing. The caller supplies the
 * original same-frame YUV data. PNG destinations are created exclusively;
 * existing files/symlinks are never overwritten. Parent directory must exist.
 */
PSDK_API uint32_t psdk_save_png(const psdk_result*, const psdk_frame*, const char*, psdk_status*);
PSDK_API void psdk_result_release(psdk_result*);
/* Clears *engine even if backend cleanup reports an error; repeated NULL destroy succeeds. */
PSDK_API uint32_t psdk_destroy(psdk_engine**, psdk_status*);

#ifdef __cplusplus
}
#endif
#endif
