#ifndef COMMON_H
#define COMMON_H

#include <random>
#include <chrono>
#include <filesystem>
#include <condition_variable>
#include <deque>
#include <thread>
#include <mutex>
#include <cstdint>
#include <unordered_set>
#include "spdlog_wrapper/spdlog_wrapper.h"
#include <functional>
#include <iostream>
#include <vector>
#include <cstring>
#include <ctime>
#include <sys/stat.h>
#include <unistd.h>  // for usleep
#include <atomic>
#include <signal.h>
#include <cmath>
#include <omp.h>
#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>
#include <memory>
#include <string>
#include "percep_demo/dnn_palette.hpp"
#include "percep_demo/seg_class_names.hpp"
#include "percep_demo/semantic_point_label.hpp"
#include <base_framework/base_utils.h>
#include "rapidjson/document.h"
#include <rapidjson/writer.h>
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include <rapidjson/filewritestream.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
#include <yaml-cpp/yaml.h>
#pragma GCC diagnostic pop

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/point_traits.h>
#include <pcl/register_point_struct.h>
#include "spdlog_wrapper/spdlog_wrapper.h"
#define rad_to_angle(angle) ((angle)*180.0 / M_PI)
#define angle_to_rad(rad) ((rad)*M_PI / 180.0)

using namespace std;

struct Cube {
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  double min_z = 0.0;
  double max_z = 0.0;
};
struct Trans{
  double x= 0.0;
  double y= 0.0;
  double z= 0.0;
  double roll= 0.0;  //rad
  double pitch= 0.0;  //rad
  double yaw= 0.0;  //rad
};

// 检测障碍点云可视化色：人黄 / 猫蓝 / 狗粉 / 其它统一红
inline void DetObstacleVisRgbByName(const std::string& class_name, uint8_t& r,
                                    uint8_t& g, uint8_t& b) {
  if (class_name == "person") {
    r = 255;
    g = 220;
    b = 0;
  } else if (class_name == "cat") {
    r = 40;
    g = 120;
    b = 255;
  } else if (class_name == "dog") {
    r = 255;
    g = 105;
    b = 180;
  } else {
    r = 230;
    g = 40;
    b = 40;
  }
}

// 按 detect.class_names 下标取色（未知 id → 红）
inline void DetObstacleVisRgb(int det_class_id, const std::string& class_names_csv,
                              uint8_t& r, uint8_t& g, uint8_t& b) {
  r = 230;
  g = 40;
  b = 40;
  if (det_class_id < 0 || class_names_csv.empty()) return;
  int idx = 0;
  std::string cur;
  for (size_t i = 0; i <= class_names_csv.size(); ++i) {
    const char c = (i < class_names_csv.size()) ? class_names_csv[i] : ',';
    if (c == ',') {
      if (!cur.empty()) {
        if (idx == det_class_id) {
          DetObstacleVisRgbByName(cur, r, g, b);
          return;
        }
        ++idx;
        cur.clear();
      }
    } else if (c != ' ') {
      cur.push_back(c);
    }
  }
}
inline double roundDouble(double val, int digits)
{
    double pow10 = std::pow(10.0, digits);
    return std::round(val * pow10) / pow10;
}

struct MapPoint {
  float min_h = 999;
  float max_h = -999;
  float avg_h = 0;
  int type = 0; // 0=未知, 1=地面/可通行, 2=其他障碍，3 =草，4 = 人，5 = 猫， 6 = 狗 ， 7 = 充电站
};

struct OccMapData{
  vector<MapPoint> ai_map;
  vector<MapPoint> pcl_map;
  vector<MapPoint> fusion_map;
  std::vector<int16_t> map_min_h;
  std::vector<int16_t> map_max_h;
  std::vector<int16_t> map_avg_h;
  std::vector<uint8_t> type;
};

struct EIGEN_ALIGN16 AiperPclPoint
{
    PCL_ADD_POINT4D;
    int label;
    float confidence;
    uint8_t r, g, b;          

    AiperPclPoint(float x_ = 0.0f, float y_ = 0.0f, float z_ = 0.0f,
                  uint8_t r_ = 0, uint8_t g_ = 0, uint8_t b_ = 0,
                 int label_ = -1, float confidence_ = 0.0f)
        : label(label_), confidence(confidence_),r(r_), g(g_), b(b_)
    {
        x = x_; y = y_; z = z_; data[3] = 1.0f; // PCL 要求 data[3] = 1
    }
    AiperPclPoint(const AiperPclPoint&) = default;
    AiperPclPoint& operator=(const AiperPclPoint&) = default;
    // 内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//  PCL 宏注册自定义点类型
POINT_CLOUD_REGISTER_POINT_STRUCT(
    AiperPclPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (int, label, label)
    (float, confidence, confidence)
    (uint8_t, r, r)
    (uint8_t, g, g)
    (uint8_t, b, b)    
)
using PointT = AiperPclPoint;
using PointCloudT = pcl::PointCloud<PointT>;

// 点云发布与着色


#endif
