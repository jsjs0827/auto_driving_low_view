#ifndef PERCEPTION_SENSOR_INPUT_H_INCLUDED
#define PERCEPTION_SENSOR_INPUT_H_INCLUDED
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define PSDK_SENSOR_INPUT_ABI_VERSION 1u
#define PSDK_SENSOR_MAX_SAMPLES 8192u
#define PSDK_RADAR_MAX_POINTS 1048576u
#define PSDK_RADAR_HAS_RADIAL_VELOCITY 1u
#define PSDK_RADAR_HAS_INTENSITY 2u

/* Optional borrowed input slots, not hardware drivers or fusion algorithms.
 * All timestamps use the calibrated camera capture clock, not receipt time.
 * IMU/wheel batches are strictly increasing, with at most MAX_SAMPLES entries.
 * Empty batches/scans are allowed; a NULL slot means the sensor was not supplied.
 * The caller retains every descriptor and backing array until process returns.
 * Coordinates remain in the sensor's documented frame; no implicit extrinsic
 * transform, synchronization, motion-buffer insertion or history copy is done. */
typedef struct psdk_imu_sample {
    uint64_t timestamp_ns;
    double linear_acceleration_mps2[3];
    double angular_velocity_radps[3];
} psdk_imu_sample;

/* Existing wheel/encoder-derived BODY velocity ABI, also used by
 * psdk_full_push_motion_samples. BODY x is forward; yaw is right-handed about z.
 * These are measured velocities, not commanded velocities or raw tick counts. */
typedef struct psdk_motion_sample {
    uint32_t struct_size, abi_version;
    uint64_t timestamp_ns;
    double linear_speed_mps, yaw_rate_radps;
    uint32_t reserved[2];
} psdk_motion_sample;

typedef struct psdk_imu_input {
    uint32_t struct_size, abi_version;
    const psdk_imu_sample* samples;
    uint32_t count, reserved;
} psdk_imu_input;

typedef struct psdk_wheel_input {
    uint32_t struct_size, abi_version;
    const psdk_motion_sample* samples;
    uint32_t count, reserved;
} psdk_wheel_input;

/* Cartesian points/detections. Radial velocity is positive away from sensor;
 * intensity uses the driver's documented scale. Optional fields are meaningful
 * only when the corresponding PSDK_RADAR_HAS_* bit is set on the scan. */
typedef struct psdk_radar_point {
    float x_m, y_m, z_m;
    float radial_velocity_mps, intensity;
} psdk_radar_point;

typedef struct psdk_radar_input {
    uint32_t struct_size, abi_version, fields, reserved;
    uint64_t timestamp_ns;
    const psdk_radar_point* points;
    uint64_t point_count;
} psdk_radar_input;

#ifdef __cplusplus
}
#endif
#endif
