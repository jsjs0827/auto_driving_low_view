#include "perception_temporal_sdk.h"
int r36_temporal_header_is_c(void){psdk_temporal_view v={0};v.struct_size=sizeof(v);v.abi_version=PSDK_TEMPORAL_ABI_VERSION;return v.struct_size>0;}
