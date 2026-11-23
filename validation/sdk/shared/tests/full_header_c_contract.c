#include "perception_full_sdk.h"
/* This translation unit is compiled as C, including all additive ABI structs. */
int r33_full_header_c_contract(void) {
    psdk_full_frame frame = {0};
    psdk_full_result_view result = {0};
    psdk_full_hooks hooks = {0};
    return sizeof(frame) > 0 && sizeof(result) > 0 && sizeof(hooks) > 0 && PSDK_FULL_ABI_VERSION == 1;
}
