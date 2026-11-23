/**
 * @file lidar_pointcloud_shm.h
 * @brief 雷达点云共享内存定长结构（最新帧）。
 */
#pragma once

#include <cstdint>

/** 点云 shm channel 名 */
inline constexpr const char *kLidarPointCloudShmChannel = "lidar_pointcloud_shm";

/** 单帧最大点数；超出部分丢弃前缀之后的点 */
inline constexpr uint32_t kLidarShmMaxPoints = 50000;

#pragma pack(push, 1)

/** 单点字段，对齐现有 DefPoint / TW48ORI */
struct LidarShmPoint {
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;
  uint16_t intensity = 0;
  uint32_t offset_time = 0;  // ns，相对本帧首点
  uint8_t ring = 0;
  uint8_t tag = 0;
};

/** 点云帧：定长数组 + point_num 表示有效点数 */
struct LidarPointCloudShm {
  int64_t stamp_sec = 0;
  uint32_t stamp_nsec = 0;
  uint32_t point_num = 0;  // 有效点数，<= kLidarShmMaxPoints
  char frame_id[32] = {};
  LidarShmPoint points[kLidarShmMaxPoints];
};

#pragma pack(pop)

static_assert(sizeof(LidarShmPoint) == 20, "LidarShmPoint size mismatch");
static_assert(sizeof(LidarPointCloudShm) == 48 + 20 * 50000,
              "LidarPointCloudShm size mismatch");
