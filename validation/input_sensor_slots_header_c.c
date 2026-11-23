#include "perception_sensor_input.h"
#include "perception_full_sdk.h"
#include "perception_slip_sdk.h"

int input_sensor_slots_header_is_c(void) {
    psdk_full_frame frame = {0};
    psdk_imu_input imu = {0};
    psdk_wheel_input wheel = {0};
    psdk_radar_input radar = {0};
    psdk_motion_sample sample = {0};
    frame.struct_size = (uint32_t)sizeof(frame);
    frame.imu = &imu;
    frame.wheel = &wheel;
    frame.radar = &radar;
    wheel.samples = &sample;
    return frame.struct_size > PSDK_FULL_FRAME_V2_SIZE &&
        PSDK_FULL_FRAME_V2_SIZE > PSDK_FULL_FRAME_V1_SIZE &&
        PSDK_SLIP_ABI_VERSION == PSDK_SENSOR_INPUT_ABI_VERSION &&
        frame.wheel->samples == &sample;
}
