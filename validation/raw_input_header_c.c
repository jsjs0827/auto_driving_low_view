#include "perception_full_sdk.h"
int raw_input_header_is_c(void) {
    psdk_full_frame frame={0};psdk_raw_stereo_input raw={0};
    raw.struct_size=(uint32_t)sizeof(raw);raw.abi_version=PSDK_RAW_INPUT_ABI_VERSION;
    raw.metadata_source=PSDK_RAW_METADATA_ORG_PROXY;frame.raw_light=&raw;
    return frame.raw_light==&raw && PSDK_FULL_FRAME_V3_SIZE>PSDK_FULL_FRAME_V2_SIZE &&
        sizeof(frame)>PSDK_FULL_FRAME_V3_SIZE && PSDK_RAW_FORMAT_UINT16_LE==1;
}
