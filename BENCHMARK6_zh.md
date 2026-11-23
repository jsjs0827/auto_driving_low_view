# 远程 Ubuntu + ADB：六组测试与 RViz 结果保存

本目录是独立的新版本 `perception-stereo-cpp-v8-dag-v3-benchmark6`。业务配置延续 v2 的 `tasks` + `dag.sequence/parallel`；六组测试使用 `features/benchmark6` 中的专用入口。

**交付状态：已在本机 WSL2 Ubuntu 22.04 完成 Linux AArch64 交叉编译，以及配置、合成指标、ROS传输和可视化保存的主机检查。没有连接 X5，尚无真实相机/模型精度或板端性能数据。** 交付包包含预编译程序，使用时可直接按 README_RUN_zh.md 操作。目录内继承的 v2、v9 或 vendor 验证记录是历史记录；本次构建证据见交付包 BUILD_REPORT.json。真实板端结果由下面的远程命令产生。

## 测试内容

| 输入 | 原始视差 | 新视差 |
| --- | --- | --- |
| camera | 检测 + 分割 + 相机 SDK 原始 Q8 | 检测 + 分割 + crop stereo |
| offline | 检测 + 分割 + 同帧录制 Q8 | 检测 + 分割 + crop stereo |
| ros | 检测 + 分割 + ROS 同帧 Q8 | 检测 + 分割 + ROS 左右图 crop stereo |

检测、分割每帧执行，二者并行，随后执行视差导入或新 stereo；两种方法使用相同检测、分割实现。这里的“原始”只指视差来源。专用入口中 `optimized_q8` 对应原始视差，`optimized_stereo` 对应新视差；不要选旧工具里的 `original`，那个选项同时切换检测、分割实现。

检测、分割用完整左图；视差统一比较底部 ROI `(0,192,640,352)`。新模型直接裁剪 640×544 左右图，不缩放，恢复到原尺寸，ROI 外置零。模型为 `r34_crop_stage2_i12_cheb5_max1_tail_int16_352x640_20261012.bin`。

先用相机录制同帧左右 NV12、原始 Q8 和内参，供 offline 与 ROS 使用。相机各次运行是独立实时采集，不能假定其画面相同。offline、ROS 两种方法保留录制时间戳，汇总时检查时间戳、左右图摘要与内参，只有一致的帧才配对。

每组包含独立的 performance 和 quality 两遍，总计 12 次推理运行，以及初始相机录制。performance 默认预热 20 帧、统计 200 帧，不保存图像；quality 默认预热 20 帧、保存 30 帧。可视化与 rosbag 录制均在所有性能测量结束后进行。不会把后处理、RViz 或图像写盘纳入性能表。

该固定测试调度用于控制变量；任意业务 DAG 的完整吞吐量仍需使用对应业务配置单独测量。专用入口内关闭建图、地面分割与标定计算，保存的三维点云在 Ubuntu 上根据视差和内参派生。

## 1. 将整个项目传到远程 Ubuntu

将源码 ZIP 上传后解压，以下命令均在连接 X5 的 Ubuntu 执行：

```bash
cd ~/perception-stereo-cpp-v8-dag-v3-benchmark6
export PROJECT="$PWD"
adb devices -l
export ADB_SERIAL=你的X5序列号
```

只有一个设备时可省略 `ADB_SERIAL`。设备必须处于 `device` 状态。

Ubuntu 端需要 CMake ≥3.20、Ninja、C++17 编译器、OpenCV 开发库、ROS2 Humble、`sensor_msgs`、`std_msgs`、rosbag2 的 sqlite3 存储插件；显示 RViz 还需要图形桌面与 `rviz2`。已有 ROS 环境时可安装缺少的组件，例如：

```bash
sudo apt-get install cmake ninja-build build-essential libopencv-dev \
  ros-humble-rviz2 ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins
bash tools/build_benchmark6_host.sh
```

该脚本仅编译 Ubuntu 配置/汇总工具、ROS 桥接与结果回放程序，不会启动板端任务或自动跑测试。

## 2. 编译 X5 程序

需要与你板端系统匹配的 AArch64 sysroot、Horizon SDK、OpenCV、PCL、Eigen、Boost 开发文件。不要使用 Ubuntu x86 库代替 ARM 库。

以下路径是可修改的示例，按远程 Ubuntu 或现有交叉编译容器内的实际位置设置：

```bash
export X5_SYSROOT=/work/obs_v004_x5/sysroot_docker
export TARGET_SYSTEM_PREFIX="$X5_SYSROOT/usr_x5"
export STEREO_HORIZON_ROOT="$TARGET_SYSTEM_PREFIX"
export TARGET_OPENCV_DIR="$TARGET_SYSTEM_PREFIX/lib/aarch64-linux-gnu/cmake/opencv4"
export TARGET_PCL_DIR="$TARGET_SYSTEM_PREFIX/lib/aarch64-linux-gnu/cmake/pcl"
export X5_CXX_COMPILER=/usr/bin/aarch64-linux-gnu-g++
bash features/benchmark6/runtime/tools/build_x5.sh
```

编译产物默认在 `features/benchmark6/runtime/install/x5`。若在 Docker 中编译，把该 install/x5 目录复制回 Ubuntu 的相同位置；也可设置 `B6_X5_INSTALL=/实际ARM安装目录`。

板端采用 ADB TCP 接收 Ubuntu 转发的 ROS 帧，因此 X5 编译不需要 ARM ROS 库。ROS 组使用真实 Ubuntu DDS 话题，再通过 C++ 桥接和 ADB TCP 送入 X5。测到的 X5 CPU/RSS 是接收和推理进程的资源，不包含 Ubuntu 发布、DDS 桥接进程，更不能视为“X5 原生 ROS 订阅进程”的资源。

## 3. 修改配置

```bash
editor benchmark6.json
```

主要检查：

- `camera.sdk_library`、`camera.vendor_config` 是 X5 上真实路径；默认 `/app/data/metoak/lib/libMoGeneralSDK.so`、`/app/data/metoak/demo/config/x5_camera.cfg`。
- `board_root` 是本次部署目录，默认 `/userdata/perception_benchmark6_v4`；`board_output_base` 是结果目录的父目录。
- 图像尺寸必须与相机校正图匹配，crop 必须与所用新视差模型输入匹配。
- `frames`、`warmup`、`quality_frames` 控制帧数。首次可先设成 20、3、5 做远程连通性试跑，再用正式帧数新建结果目录。
- `visualization.save_rosbag=true` 默认保存 rosbag；`replay_hz` 仅决定结果回放速度，不决定性能测量帧率。
- 预留磁盘空间：默认相机录制约数百 MB，另有六组图像、点云与 rosbag。

暂定输入话题如下，可在 `ros` 节修改：

```text
/benchmark6/left_nv12          sensor_msgs/Image, nv12
/benchmark6/right_nv12         sensor_msgs/Image, nv12
/benchmark6/disparity_q8       sensor_msgs/Image, 16UC1 Q8.8
/benchmark6/left/camera_info   sensor_msgs/CameraInfo
/benchmark6/right/camera_info  sensor_msgs/CameraInfo
```

所有必需消息按原始采集时间戳严格配对；Q8 是视差而非深度。右相机投影矩阵 P[3] 提供以米为单位的基线乘焦距。默认用 `ROS_DOMAIN_ID=67` 隔离本次测试，回放和 RViz 也使用同一值。确认式回放通过 `/benchmark6/replay_ack` 控制供帧，避免固定低频发布限制吞吐量。

## 4. 一条命令部署、采集、跑完六组并保存可视化

```bash
export ROS_DOMAIN_ID=67
export OUT="$PROJECT/results/benchmark6_$(date +%Y%m%d_%H%M%S)"
bash tools/run_benchmark6_adb.sh "$PROJECT/benchmark6.json" "$OUT"
```

脚本会部署 ARM 程序与模型，录制同帧数据，依次运行 camera、offline、ros 的两种方法，拉回结果，生成可视化并录制 rosbag，最后汇总。每组运行独立进程，默认单次超时 3600 秒，可设置 `B6_RUN_TIMEOUT`。

本版集成旧版 ADB 返回码兼容。部署目录已存在时比较运行清单并校验全部文件；一致则自动复用，内容不同或不完整则保留并停止。修改代码或目录内容后应指定新 board_root。每次输出目录必须全新。

若板端 ldd 提示缺库，查看 `$OUT/logs/board-ldd.txt`。相机平台库 libcam/libvpf/libhbmem/libgdcbin 必须使用板端原安装版本，本包不再覆盖它们。历史默认搜索目录和配置位置见 RUN_X5_zh.md；通用依赖在包内，必要时应针对匹配的开发 sysroot 重新编译、打包。

## 5. 查看与回放 RViz 结果

远程脚本默认无需启动图形界面即可保存 PNG、PLY 和 rosbag。在有 Ubuntu 图形桌面的终端运行：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=67
rviz2 -d "$OUT/runs/offline/stereo_crop/quality/view.rviz"
```

另开一个同环境终端，回放所有六组输出：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=67
ros2 bag play "$OUT/visualization/bag" --loop
```

每个 `view.rviz` 对应一组话题，切换配置即可查看另一组。六组在 bag 中依次出现。bag 的接收时间反映结果回放过程；每条图像/点云 Header 保留原采集时间戳，结果 JSON 保留采集标识。这里保存的是可复现的 RViz 输入数据和布局；PNG 是程序输出图像，不是桌面窗口截图。

也可只回放某一组，不使用 bag：

```bash
features/benchmark6/runtime/install/ubuntu_ros/bin/ab_result_replay \
  "$OUT/runs/offline/stereo_crop/quality" \
  /benchmark6/offline/stereo_crop 5 4 144
```

该命令会生成或更新该组派生图像与点云，并逐帧发布已保存的推理结果，不重新加载模型。

结果目录：

```text
$OUT/
  index.html                     六组性能/精度表与图片预览
  summary.json / summary.csv      完整指标与表格
  REPORT_zh.md                    汇总说明
  run_status.tsv                 每组每遍退出状态
  board-environment.txt          板端系统、CPU、模型 SHA256
  capture/                       同帧左右 NV12、Q8、内参、manifest
  runs/{camera,offline,ros}/{original_q8,stereo_crop}/
    performance/summary.json     性能数据，包含阶段统计和 BPU
    performance/frames.jsonl     逐帧耗时、RSS、采集标识
    performance/bpu_samples.jsonl
    quality/predictions.jsonl    检测框、掩码/Q8路径、图像摘要、内参
    quality/view.rviz            对应 RViz 布局
    quality/frame-000000/
      left.png / right.png
      detection.png
      segmentation.png           原始单通道类别 ID
      segmentation_color.png / segmentation_overlay.png
      disparity_q8.u16            小端 uint16，实际视差=值/256
      disparity_q8.png            无损16位Q8
      disparity_color.png        各组使用相同上限映射颜色
      camera_cloud.ply            视差派生的彩色点云，坐标单位米
  visualization/bag/             sqlite3 rosbag + metadata.yaml
  visualization/*.log            保存与回放日志
  logs/                          采集/推理/ROS桥接等日志
```

输出话题前缀为 `/benchmark6/<输入源>/<视差方法>/`，包含 `left`、`detection`、`segmentation`、`segmentation_overlay`、`disparity_color`、`disparity_q8`、`cloud`、`result`。检测与分割已画在图上，`result` 另保存包含框坐标、分数和采集标识的 JSON。

若性能已完成但可视化中途失败，可保留原始质量结果，先处理已存在的 `visualization/bag`，再独立运行：

```bash
bash tools/save_benchmark6_visuals.sh "$OUT"
build/benchmark6/r36_benchmark6 --report "$OUT"
```

不需要 bag 时可在首次测试前将 `visualization.save_rosbag` 设为 false；仍会保存所有 PNG、Q8、PLY 与 `.rviz`。

## 指标与精度口径

- 性能：整体 FPS、采集等待、计算 wall time、检测/分割前后处理及 backend wait、新 stereo 的前处理/backend wait/post、均值和 P95。
- CPU：Linux 进程 CPU，100% 为一个 CPU 核，包含该进程中的相机 SDK 线程；Ubuntu ROS 发布/桥接/回放与 RViz 不计入 X5 worker。
- RSS：进程 VmRSS，KiB，报告均值与最大值；不把共享库映射简单等同于独占内存。
- BPU：默认读取 `/sys/devices/system/bpu/bpu0/ratio`，100ms 采样，设备共享、驱动定义窗口，报告均值/P95/最大值及样本可用性；不可读时是 null，不能填 0。
- 原始视差来自 SDK 的已有 Q8，无法从 SDK 单独测出其内部生成耗时。camera 采集等待包含相机/SDK等待；offline、ROS 原始 Q8 是预先生成的。表中的 `disparity_prepare_or_stereo` 对原始方法是拷贝/ROI处理，对新方法包含 stereo 计算；不能当作两种算法独立生成耗时的直接对比。
- 新视差与原始 Q8 的差异仅是参考一致性，不是绝对精度；summary.json 将其放在 `same_input_consistency`。`transport_consistency` 另比较相同方法的 offline 与 ROS 输出。

没有提供真值时，AP/mIoU/EPE/D1 都为 NA/null。可以先采集再标注，在 `plan.json` 的 `ground_truth.offline` 和 `ground_truth.ros` 填入同一标注 manifest 的绝对路径，然后只执行 `--report` 重算；实时相机组还需与当次质量帧对应的标注及 `left_digest`。修改 suite 后不会追溯改写既有 plan。

支持如下 GT JSON；也支持每行一个 frame 对象的 JSONL，所有文件路径相对于 manifest：

```json
{
  "frames": [
    {
      "timestamp_ns": "实际采集的十进制时间戳",
      "left_digest": "predictions.jsonl中的left_digest",
      "detections": [{"class_id": 0, "x1": 100, "y1": 200, "x2": 150, "y2": 280}],
      "segmentation_mask": "labels/000001.png",
      "disparity_f32": "labels/000001.f32",
      "disparity_valid_mask": "labels/000001_valid.png"
    }
  ]
}
```

检测标注使用完整左图像素坐标；汇总报告 IoU≥0.5 的 precision/recall 和 101 点插值 AP50，基于模型已输出且已过阈值的框，不等价于完整 COCO 指标。分割标注为原尺寸单通道 uint8 类别 ID PNG 或 `.u8`，255 忽略。独立视差真值为原尺寸小端 float32，单位是原图像素，不能填模型自己的预测；非正数/非有限值忽略，可选 valid mask 排除无效位置。EPE/RMSE基于有效预测交集，并同时报告预测覆盖率；无效预测在 bad3 与 D1 中计为错误。D1阈值为误差>3px且相对误差>5%。视差仅在共同 crop ROI 上评估，检测/分割评估完整左图。

标注字段可部分提供，报告会列出每个任务的标注帧数与覆盖率。没有匹配标注、无效值或运行失败的组不会被当作精度 0 或性能成功。
