# 板端 RViz 通信操作指南

PC 与板端通过 **ADB + ROS2 localhost + 端口转发** 通信。PC 端准备完成后，按本文在板端操作。

## 架构

```text
[板端 X5]  ai_node  --ROS2(DDS localhost)--> 127.0.0.1:7400
                                              ^
                                              | adb forward
[PC Linux]  rviz2 / ros2 topic  <---------- 127.0.0.1:7400
```

两端必须一致：

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
```

## 一、PC 端部署（adb push）

在 PC 工程目录执行：

```bash
cd ~/ai_root
./adb_push.sh

# RViz 专用配置（可选，比默认 config 更轻量）
adb push ai_ws/src/ai_node/ai_node/config/aarch64/config_rviz.json \
         /app/daye/config/ai_node/config_rviz.json
adb push ai_ws/src/ai_node/point-cloud-processing/config/start_config_rviz.json \
         /app/daye/config/start_config_rviz.json

adb push ai_ws/src/script/aarch64/ros_rviz_env.sh /app/daye/bin/
adb push ai_ws/src/script/aarch64/start_percep_rviz.sh /app/daye/bin/
adb push ai_ws/src/script/aarch64/board_ros_check.sh /app/daye/bin/
adb shell chmod +x /app/daye/bin/*.sh
```

## 二、板端启动（adb shell）

```bash
adb shell
```

### 方式 A：RViz 调试专用（推荐）

使用 RViz 配置 + localhost DDS：

```bash
# 切换到 RViz 配置（只需做一次，或写进启动脚本）
cp /app/daye/config/ai_node/config_rviz.json /app/daye/config/ai_node/config.json
cp /app/daye/config/start_config_rviz.json /app/daye/config/start_config.json

cd /app/daye/bin
./start_percep_rviz.sh
```

### 方式 B：沿用现有 config.json

若已手动开启发布，也可仅加 ROS 环境后启动：

```bash
cd /app/daye/bin
source ./ros_rviz_env.sh
./start_percep.sh
```

### 启动后日志应包含

```text
[PercepMain] image_pub resize=on target=640x480
[PercepMain] working frame=... pub_points=...
```

## 三、板端自检（adb shell 另开窗口）

```bash
adb shell
cd /app/daye/bin
./board_ros_check.sh
```

或手动：

```bash
source /opt/ros/humble/setup.bash
source /app/daye/bin/ros_rviz_env.sh
export NODE_CONFIG_FILE=/app/daye/config
ros2 topic list
ros2 topic hz /percep_main/pcl_cloud
ros2 topic hz /percep_main/gdc_frame
```

若 `topic list` 为空但 ai_node 在跑：

```bash
ros2 daemon stop && ros2 daemon start
ros2 topic list
```

## 四、发布话题一览

| 话题 | 类型 | RViz 用途 |
|------|------|-----------|
| `/percep_main/pcl_cloud` | `sensor_msgs/PointCloud2` | 彩色点云（Fixed Frame: `percep_main`） |
| `/percep_main/gdc_frame` | `sensor_msgs/Image` | 相机图像 |
| `/percep_main/ground_cloud` | `sensor_msgs/PointCloud2` | PatchWork 地面点（可选） |
| `/percep_main/nonground_cloud` | `sensor_msgs/PointCloud2` | PatchWork 非地面点（可选） |

配置文件：`rviz/visiualize.rviz`（PC 端打开）

## 五、config_rviz 与默认 config 差异

| 项 | config.json（默认） | config_rviz.json |
|----|---------------------|------------------|
| fps | 5 | 3 |
| data_loopback | 开启 | **关闭**（减 CPU） |
| patchwork 发布 | 开 | **关**（RViz 看点云即可） |
| image_pub | 原图/可配 | 640×480 缩放 |

## 六、常见问题

**Q: PC 上 `ros2 topic list` 看不到板端话题**

1. 确认板端用 `start_percep_rviz.sh` 启动（含 `ROS_LOCALHOST_ONLY=1`）
2. PC 端也设置了相同 `ROS_DOMAIN_ID` 和 `ROS_LOCALHOST_ONLY=1`
3. PC 端已执行 `adb forward tcp:7400 tcp:7400`（必要时加 7401、7411）
4. 板端 `board_ros_check.sh` 能列出话题后再看 PC

**Q: 有话题但 RViz 无点云**

- Fixed Frame 设为 `percep_main`
- PointCloud2 的 Color Transformer 选 **RGB8**
- QoS 与发布端一致（Best Effort）

**Q: CPU 过高**

- 使用 `config_rviz.json` + `start_config_rviz.json`
- 关闭 patchwork 发布、降低 fps

## 七、恢复生产配置

```bash
adb push ai_ws/src/ai_node/ai_node/config/aarch64/config.json \
         /app/daye/config/ai_node/config.json
adb push ai_ws/src/ai_node/point-cloud-processing/config/start_config.json \
         /app/daye/config/start_config.json
# 重启 ai_node，使用 start_percep.sh（不带 rviz 环境）
```
