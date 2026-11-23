#include "perception_sdk.h"
/* This translation unit is C, deliberately independent of C++ headers. */
int psdk_header_is_c_compatible(void) {
    psdk_config config = {0};
    psdk_frame frame = {0};
    psdk_result_view view = {0};
    config.struct_size = (uint32_t)sizeof(config);
    config.abi_version = PSDK_ABI_VERSION;
    frame.struct_size = (uint32_t)sizeof(frame);
    view.struct_size = (uint32_t)sizeof(view);
    return config.abi_version == 1u && frame.struct_size && view.struct_size;
}
