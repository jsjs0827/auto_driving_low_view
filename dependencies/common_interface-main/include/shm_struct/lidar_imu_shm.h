/**
 * @file lidar_imu_shm.h
 * @brief 雷达 IMU 共享内存定长结构（最新帧）。
 */
#pragma once

#include <cstdint>

/** IMU shm channel 名 */
inline constexpr const char *kLidarImuShmChannel = "lidar_imu_shm";

#pragma pack(push, 1)

/** IMU 定长帧：时间戳 + 线加速度 + 角速度 + frame_id */
struct LidarImuShm {
  int64_t stamp_sec = 0;
  uint32_t stamp_nsec = 0;
  char frame_id[32] = {};
  float linear_acceleration[3] = {};
  float angular_velocity[3] = {};
};

#pragma pack(pop)

static_assert(sizeof(LidarImuShm) == 68, "LidarImuShm size mismatch");
