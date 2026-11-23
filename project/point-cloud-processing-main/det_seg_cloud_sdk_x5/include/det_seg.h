#ifndef DET_SEG_H
#define DET_SEG_H
#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#define DS_API
#else
#define DS_API __attribute__((visibility("default")))
#endif
#ifdef __cplusplus
extern "C" {
#endif

#define DS_ABI_VERSION 1u
#define DS_MODEL_CONTRACT_ABI_VERSION 1u
enum ds_status { DS_OK = 0, DS_INVALID_ARGUMENT = 1, DS_INIT_FAILED = 2, DS_INFERENCE_FAILED = 3 };
enum ds_mode { DS_DETECTION = 1, DS_SEGMENTATION = 2, DS_BOTH = 3 };
enum ds_format { DS_NV12 = 0, DS_I420 = 1 };
typedef struct ds_handle ds_handle;
typedef struct ds_result ds_result;

typedef struct ds_config {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t mode; /* DS_DETECTION, DS_SEGMENTATION or DS_BOTH. */
    const char* detection_model;
    const char* segmentation_model;
    float score_threshold;
    float nms_threshold;
    uint32_t nms_top_k;
} ds_config;

/* Optional model metadata for applications that use a compatible model with
 * a different ordered class list, including the 34-class mmyolo1010 model.
 * All strings are copied by ds_create_ex. IDs >= 100 are reserved for the
 * cloud segmentation label namespace. Current cloud masks remain 3-class. */
typedef struct ds_model_contract {
    uint32_t struct_size, abi_version;
    uint32_t detection_classes, detection_reg, detection_resize_type;
    const char* detection_class_names;
    uint32_t segmentation_classes, segmentation_resize_type, segmentation_align_corners;
    const char* segmentation_argmax_mode;
    const char* segmentation_class_names;
} ds_model_contract;

typedef struct ds_image {
    const uint8_t* data;
    size_t bytes;
    uint32_t width;
    uint32_t height;
    uint32_t stride; /* Y stride; I420 U/V stride = stride / 2. */
    uint32_t format;
    uint64_t frame_id;
    uint64_t timestamp_ns;
} ds_image;

typedef struct ds_detection {
    int32_t class_id;
    float score;
    float x1, y1, x2, y2; /* Coordinates in the original input image. */
    char class_name[64];
} ds_detection;

typedef struct ds_stage_timing {
    double preprocess_ms;
    double inference_ms; /* submit + wait + release wall time, not pure BPU cycles. */
    double postprocess_ms;
} ds_stage_timing;

typedef struct ds_result_view {
    uint32_t width;
    uint32_t height;
    uint64_t frame_id;
    uint64_t timestamp_ns;
    const ds_detection* detections;
    size_t detection_count;
    const uint8_t* mask; /* uint8 class IDs, tightly packed width * height. */
    size_t mask_bytes;
    ds_stage_timing detection_timing;
    ds_stage_timing segmentation_timing;
    double total_ms;
} ds_result_view;

/* Initialize defaults, then set the enabled model paths before ds_create(). */
DS_API void ds_default_config(ds_config* config);
/* Paths are copied during creation. Disabled models need no path or allocation. */
DS_API int ds_create(const ds_config* config, ds_handle** handle, char* error, size_t error_capacity);
DS_API void ds_default_model_contract(ds_model_contract* contract);
/* Versioned extension: ds_create keeps the original packaged 24/3 contract. */
DS_API int ds_create_ex(const ds_config* config, const ds_model_contract* contract,
                        ds_handle** handle, char* error, size_t error_capacity);
/* Query the actual NV12 model input canvas, read from BIN tensor metadata.
 * mode must be exactly DS_DETECTION or DS_SEGMENTATION and enabled on handle.
 * This never changes tensor shapes. Width/height are cleared before validation.
 * Keep handle alive until return; error text may be NULL. No old ABI struct changes.
 */
DS_API int ds_get_model_input_size(const ds_handle* handle, uint32_t mode,
                                  uint32_t* width, uint32_t* height,
                                  char* error, size_t error_capacity);
/* Synchronous: input must remain unchanged until return. No input pointer is retained.
 * Results own their storage and survive subsequent calls and handle destruction.
 * SDK calls serialize across handles because the preserved workspaces are global.
 * Both mode executes detection then segmentation; there is no cross-frame queue.
 * Output pointers are set to NULL on failure. Initialize output slots to NULL;
 * release an existing result before reusing its slot. Error text may be NULL.
 */
DS_API int ds_process(ds_handle* handle, const ds_image* image, ds_result** result, char* error, size_t error_capacity);
/* Parallel detector and segmenter when both are enabled; single-model calls remain sequential. */
DS_API int ds_process_parallel(ds_handle* handle, const ds_image* image, ds_result** result, char* error, size_t error_capacity);
/* Borrowed pointers remain valid until ds_result_release(), including after ds_destroy(). */
DS_API int ds_result_get_view(const ds_result* result, ds_result_view* view);
DS_API void ds_result_release(ds_result** result);
/* Call after all users of this handle have finished. Null pointer is accepted. */
DS_API void ds_destroy(ds_handle** handle);

#ifdef __cplusplus
}
#endif
#endif
