#ifndef PERCEPTION_RAW_INPUT_H_INCLUDED
#define PERCEPTION_RAW_INPUT_H_INCLUDED
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define PSDK_RAW_INPUT_ABI_VERSION 1u
#define PSDK_RAW_FORMAT_UINT16_LE 1u
#define PSDK_RAW_BAYER_BGGR 1u
#define PSDK_RAW_METADATA_UNKNOWN 0u
#define PSDK_RAW_METADATA_ORG_PROXY 1u

/* Borrowed unrotated RAW; stride is BYTES. A null data pointer with bytes=0
 * means this eye is absent. Buffers must remain unchanged until full_process
 * returns. The RAW-light model currently accepts native BGGR1088x1280 LE16
 * containers with 10 valid bits; no rescaling/rotation is performed. */
typedef struct psdk_raw_image {
    uint32_t struct_size, abi_version;
    uint32_t width, height, stride_bytes, format, valid_bits, bayer;
    const uint8_t* data;
    uint64_t bytes, frame_id, timestamp_ns;
    uint32_t sensor_id, stream_type;
} psdk_raw_image;

/* exp_time is the ORG frame's SDK value in MILLISECONDS. analog_gain is the
 * linear multiplier, never dB. Preserve the original ORG capture identity;
 * do not overwrite it with a RAW frame ID to manufacture synchronization. */
typedef struct psdk_org_exposure {
    uint32_t struct_size, abi_version, present, reserved;
    double exposure_ms, analog_gain;
    uint64_t frame_id, timestamp_ns;
    uint32_t sensor_id, stream_type;
} psdk_org_exposure;

typedef struct psdk_raw_stereo_input {
    uint32_t struct_size, abi_version, metadata_source, reserved;
    psdk_raw_image left, right;
    psdk_org_exposure left_org, right_org;
} psdk_raw_stereo_input;

#ifdef __cplusplus
}
#endif
#endif
