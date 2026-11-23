> 此运行库已适配 benchmark6（两种视差、关闭建图、质量结果导出、ROS确认式供帧和RViz回放）。以下是继承的 v9 使用背景；当前入口和配置请以项目根目录 BENCHMARK6_zh.md 为准。本版已完成 WSL 交叉编译和主机检查，尚未上板实测。

# v9：原生产检测分割、优化检测分割与新 Stereo 的受控实验

这是独立项目，不修改 v8 或原项目。算法、输入与统计程序均为 C++；运行脚本使用 Bash。ROS 的 CMake/ament 构建工具可能调用系统 Python，板端实验程序不需要 Python 环境。

## 1. 三组到底比较什么

| 方法 | 检测分割 | 视差 | 后处理 |
|---|---|---|---|
| `original` | 原生产版 `HorizonDnnRunner` 原始源码 | 相机 SDK/同帧录制 Q8 | 统一原数值点云、标签、坐标变换、Patchwork、地图 |
| `optimized_q8` | 重构版 exact-input SDK | 同一来源 Q8 | 同上 |
| `optimized_stereo` | 同一个重构版 SDK | 左右 NV12 crop → 新 BPU 模型 → 恢复 Q8 | 同上 |

**原算法基线是原生产检测分割移入统一实验外壳，不能称为原完整 ROS/SHM 系统。**三组共用输入封装、映射实现、采样、空间边界、外参、地面、地图及保存策略，以隔离检测分割和视差源的差异。原工程专有框架/SHM 的线程开销不在这个实验范围内。回充、历史、打滑和 RViz 发布不参与这次基准，三组都关闭。

原 runner 源文件位于 `vendor/original/horizon_dnn_runner.cpp`，与已有 original-camera-benchmark 的生产源码一致。原 runner 和优化 runner 恰好有相同的 C++ 类名、但内部布局不同，因此只给原 runner 的编译单元设置命名空间宏隔离，不改其数值算法；不能把两个实现直接用同名符号链接。

三组都推理完整左图的检测、分割，类别、阈值与双模型并行开关一致。原检测入口关闭提前 keep_classes 筛选，统一输出所有配置类别，语义融合阶段仍按 mapping.json 的 keep_classes 过滤。调度为检测分割对完成后执行视差，再执行映射；这是用于隔离算法成本的统一调度，不复制 v7 三个推理任务争用 BPU 的调度。

## 2. 同时保留全图和同 ROI 两种实验

默认每种输入执行两套 suite，每套三组：

| suite | 原算法 Q8 | 优化 Q8 | 新 Stereo |
|---|---|---|---|
| `native` | **完整 640×544 Q8** | **完整 640×544 Q8** | 底部 `(0,192,640,352)` |
| `matched_roi` | 同一底部 ROI，外部 Q8 清零 | 同一底部 ROI，外部 Q8 清零 | 同一底部 ROI |

`native` 保留真实应用的覆盖区域，可比较原与优化 Q8。比较三种视差方案时用 `matched_roi`，避免把减少上方区域的工作量算作算法优化。Stereo 的 NV12 Y/UV 直接裁剪，不经过 RGB，不做隐藏 resize；视差贴回 640×544 原坐标，区域外为无效 Q8=0。源图及内参不变。

默认三组 `sample_step=4`、max_points=50000，保留点云、输出地图。每组独立进程、10 帧预热、90 个正式帧，默认不逐帧保存。配置里的 methods/suites 可删减。

## 3. 目录与配置

```text
perception-ab-benchmark-v9/
├─ config/
│  ├─ offline.json       离线输入与实验开关
│  ├─ camera.json        相机直取输入
│  ├─ ros.json           ROS 订阅输入、话题和 QoS
│  ├─ mapping.json       统一点云、标签、地面和地图参数
│  └─ extrinsics.json    同一相机到机器人外参
├─ models/x5/            三个实际 BIN；配置使用相对路径
├─ include/ab/           配置、Frame、Source、Pipeline 接口
├─ src/                  输入、原 runner 适配、流水线、ROS、统计与报告
├─ tests/                区域、时间、格式与真实数值链路测试
├─ tools/                编译和实验矩阵运行入口
├─ vendor/original/      原生产检测分割源码与来源记录
└─ vendor/perception/    v8 数值功能与 SDK 源码快照
```

JSON 的文件路径相对于该 JSON 所在目录。manifest 中图像/Q8 路径相对于 manifest 所在目录。相机 SDK 和厂商 cfg 路径直接设置为板端实际路径。**相机直取自动读取 SDK 矫正内参；离线必须指定真实内参文件；ROS 从同帧 CameraInfo 获取矫正投影矩阵。**示例内参不能替代真实标定。

新模型：`models/x5/r34_crop_stage2_i12_cheb5_max1_tail_int16_352x640_20261012.bin`，左右两个 NV12 输入，模型逻辑 shape 为 `1×3×352×640`。初始化检查实际张量类型和尺寸，不匹配时拒绝运行。

## 4. 离线数据与录制

离线支持 PNG/JPEG、紧凑 `.nv12` 和 `.i420`，解码为 NV12 后再处理。**左右图必须已经矫正，尺寸与标定一致**；PNG 路径无需再声明成 raw NV12。图像尺寸不匹配会报错，不会擅自拉伸双目图。Q8 是 uint16 小端，像素视差=Q8/256，不是深度图。640×544 NV12 为 522240 字节，Q8 为 696320 字节。

先用本项目相机录制器获得同帧左右图、原 Q8、内参：

```bash
bash tools/run_one.sh --config config/camera.json --source camera \
  --record --output /userdata/ab_capture_v9_001
```

每次必须使用新目录，默认录制 warmup+frames=100 帧。将 `config/offline.json` 与 `config/ros.json` 中 `input.offline.manifest`、`input.offline.intrinsics` 分别设置为录制目录的 `manifest.json`、`intrinsics.json`。可以使用绝对路径，也可复制到 data/ 后继续用相对路径。

```json
{
  "schema_version": 1,
  "frames": [{
    "key": "capture-000001", "frame_id": "9",
    "timestamp_ns": "187149751177781",
    "left": "left/000001.nv12", "right": "right/000001.nv12",
    "q8": "q8/000001.u16"
  }]
}
```

ID、纳秒时间用十进制字符串保存，保持真实采集值，避免工具按浮点读取时损失精度。正式数据至少有 warmup+frames 个唯一时间戳，不默默重复同一帧。新 Stereo 单独测试可以不提供 Q8；三组对比必须有真实同帧原 Q8。

## 5. Ubuntu / Docker 交叉编译

沿用此前 Ubuntu 路径及容器名：源码放 `/home/st/ini_code/perception-ab-benchmark-v9`。

```bash
sudo systemctl start docker
sudo docker start obs-v004-x5-build
sudo docker exec obs-v004-x5-build mkdir -p /tmp/perception-ab-benchmark-v9
sudo docker cp /home/st/ini_code/perception-ab-benchmark-v9/. \
  obs-v004-x5-build:/tmp/perception-ab-benchmark-v9/
sudo docker exec -it --workdir /tmp/perception-ab-benchmark-v9 obs-v004-x5-build bash
```

容器内：

```bash
export X5_SYSROOT=/work/obs_v004_x5/sysroot_docker
export X5_CXX_COMPILER=/usr/bin/aarch64-linux-gnu-g++
export TARGET_SYSTEM_PREFIX="$X5_SYSROOT/usr_x5"
export STEREO_HORIZON_ROOT="$TARGET_SYSTEM_PREFIX"
export TARGET_OPENCV_DIR="$TARGET_SYSTEM_PREFIX/lib/aarch64-linux-gnu/cmake/opencv4"
export TARGET_PCL_DIR="$TARGET_SYSTEM_PREFIX/lib/aarch64-linux-gnu/cmake/pcl"
export TARGET_ROS_PREFIX=/opt/ros/humble
export BUILD_JOBS=2

# 编译三种输入，ROS prefix 必须是实际 AArch64 包
bash tools/build_x5.sh --with-ros

# 只编译离线和相机输入时：bash tools/build_x5.sh
```

不要将宿主机 x86 ROS 库传给交叉链接器。脚本不修改 sysroot 软链接、不安装驱动、不忽略未解析符号。产物在 `install/x5/`。

退出容器后取出：

```bash
exit
mkdir -p /home/st/ini_code/perception-ab-benchmark-v9/install
sudo docker cp obs-v004-x5-build:/tmp/perception-ab-benchmark-v9/install/. \
  /home/st/ini_code/perception-ab-benchmark-v9/install/
```

本工作区 WSL 的复现入口为 `bash tools/build_local_x5.sh`；具体已完成的编译和测试记录见 VALIDATION.md。

## 6. X5 上运行

将完整源码目录或 install/x5 运行目录放到板上，例如 `/userdata/perception-ab-benchmark-v9`，进入该目录。运行目录需要 bin/、lib/、config/、models/、tools/。

```bash
cd /userdata/perception-ab-benchmark-v9
export LD_LIBRARY_PATH="$PWD/lib:$PWD/install/x5/lib:/app/data/metoak/lib:/usr/hobot/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
ldd bin/ab_benchmark        # 源码目录则检查 install/x5/bin/ab_benchmark
sha256sum -c SHA256SUMS    # 已打包运行目录内执行；修改配置前校验
```

先关闭其他占用相机的程序，核对 camera.json 的 SDK/cfg 路径。

```bash
# 离线同数据：六组实验，包括两种区域策略
bash tools/run_matrix.sh offline /userdata/ab_v9_offline_001

# 相机直取：六组依次独占相机
bash tools/run_matrix.sh camera /userdata/ab_v9_camera_001

# ROS 原生 DDS：需要 --with-ros 编译，使用 ros_dds.json
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30
bash tools/run_matrix.sh ros /userdata/ab_v9_ros_offline_001 config/ros_dds.json offline

# ROS：发布端从相机 SDK 获取对应数据
bash tools/run_matrix.sh ros /userdata/ab_v9_ros_camera_001 config/ros_dds.json camera

# ROS：已有外部发布节点；它必须提供本次方法所需消息
bash tools/run_matrix.sh ros /userdata/ab_v9_ros_external_001 config/ros_dds.json external
```

单组运行：

```bash
bash tools/run_one.sh --config config/offline.json --source offline \
  --method optimized_q8 --suite native --output /userdata/ab_v9_q8_full_001
```

## 7. ROS 数据约定

| 话题默认值 | 消息 | 必须包含 |
|---|---|---|
| /ab/left_nv12 | sensor_msgs/Image | 左矫正图，nv12/i420/rgb8/bgr8 |
| /ab/right_nv12 | sensor_msgs/Image | 同帧右矫正图 |
| /ab/disparity_q8 | sensor_msgs/Image | 原 Q8.8，encoding=16UC1；两种 Q8 方法需要 |
| /ab/left/camera_info | sensor_msgs/CameraInfo | 矫正左投影矩阵 P |
| /ab/right/camera_info | sensor_msgs/CameraInfo | 矫正右 P，P[3]=-fx×baseline_m |

所需消息必须有完全相同的非零采集时间戳；缓存有界，超时错误会提示话题/QoS/配对。右 P[3] 的单位使用 ROS 标准米，输入适配层换算成统一 bxf pixel_mm。`reliable` 必须与外部发布者匹配：订阅 RELIABLE 无法匹配只提供 BEST_EFFORT 的发布者。

矩阵脚本自带 ROS 发布者默认 5 Hz，刻意避免最慢组堆积。**这时 FPS 可能受发布频率限制，判断算法快慢看 process_ms。**需要测极限吞吐时提高 publish_hz，并检查缓存淘汰及是否完整收到指定帧。相机直取三次并不是同一批捕获；只用固定场景比较性能，精度和输出差异优先用离线或 ROS 录制回放。

自动启动的同机 ROS 发布进程会作为 --publisher-pid 传给 worker，记录同一个测量窗口的发布进程 CPU，以及 group_cpu_percent。外部/异机发布者没有可用 PID 时该字段为 null，不能把 worker 单进程资源当作整个系统资源。

## 8. 输出、计时与效果比较

```text
输出根目录/
├─ index.html / report_zh.md / summary.csv
├─ model_sha256.txt / system.txt
├─ native/{original,optimized_q8,optimized_stereo}/
└─ matched_roi/{original,optimized_q8,optimized_stereo}/
   ├─ requested_config.json
   ├─ frames.jsonl            唯一采集 ID、时间、左右输入摘要、结果摘要、各阶段时间
   ├─ summary.json            mean/p95/max、FPS、CPU、RSS、BPU 与实际算法配置
   ├─ bpu_samples.jsonl
   └─ frame-000000/           save_every_n > 0 时
      ├─ left.png / segmentation.png / disparity_q8.u16
      └─ camera.ply / semantic.ply / robot.ply / ground.ply / obstacle.ply
```

计时先分 acquisition、compute、save，compute 内再分检测分割对、视差导入/新模型、RGB 转换、mapping_begin_copy、cloud_projection、labels、transform、ground、maps。det_infer_wait、seg_infer_wait 与 stereo_backend_wait_decode 均包含主机提交/等待，不能解释成纯 BPU 核时间。

RSS 为进程驻留内存，BPU 是整个设备的利用率快照；原 Q8 的 SDK 内部生成时间不能从 acquire_ms 单独推断。离线 Q8 不包括生成成本；相机 mode2 的生成线程计入本进程 CPU/BPU。CPU 100% 表示一个核。

性能轮保持 save_every_n=0。效果轮单独使用新目录，设置 save_every_n=1，统一保存所有方法同序号的结果。比较输出前先核对左右图摘要和 timestamp_ns，相同序号并不自动表示相机在线三组是同一帧。matched_roi 的 PLY 才能直接比较相同区域；全图 Q8 与 Stereo ROI 的覆盖差异应保留说明。Q8 是参考而不是精度真值。

## 9. 主机验证

Windows 可使用 `cmake -S . -B build/windows -G "Visual Studio 17 2022" -A x64 -DAB_CONTRACT_ONLY=ON` 编译配置和统计测试。Ubuntu/WSL 使用 `bash tools/build_host.sh --contract`。带 ROS 的真实输入传输和 OpenCV 格式验证用 `bash tools/build_host.sh --ros`。主机版本拒绝执行 X5 BIN 推理，不会用假模型制造性能结果。

`ab_benchmark --validate-input` 只验证前三帧的真实输入适配，不推理；`ab_input_tests NEW_DIR` 生成的是 SYNTHETIC TRANSPORT ONLY 夹具，只用于 ROS 格式与时序测试，不能作为模型效果或 X5 性能数据。

## 10. 无 ARM ROS 开发包时：Ubuntu ROS → ADB → X5

本轮提供的 X5 默认运行包支持 `ros.transport=tcp`，Ubuntu 端使用已编译的 C++ ROS2 订阅转发器。数据链为：

```text
Ubuntu 的 ROS 图像/CameraInfo 话题
  → C++ exact-stamp 配对、NV12 规范化
  → TCP（可经过 adb forward）
  → X5 C++ Frame 输入
  → 原检测分割 / 优化检测分割 / 新 Stereo
```

每个包明确携带原 frame_id、采集 timestamp_ns、左右 NV12、实际 Q8 和矫正内参，包含字节数、尺寸和版本校验。X5 端无需安装 ROS。也可通过局域网连接，bridge_host 设置为 X5 IPv4。转发与图像转换的开销放在 ROS 输入获取阶段，算法处理计时从收到完整 Frame 后开始。

Ubuntu/WSL 编译转发器：

```bash
bash tools/build_host.sh --ros
```

先把 X5 运行包部署到 `/userdata/perception-ab-benchmark-v9`。在 Ubuntu 的 `config/ros.json` 设置录制数据的真实 manifest/intrinsics 路径；在 X5 的同名配置中保持 transport=tcp、port=19091。Ubuntu 上有 adb、可访问板子且装有 ROS Humble 时，一条指令顺序执行六组：

```bash
export ROS_DOMAIN_ID=30
bash tools/run_ros_adb.sh \
  /userdata/perception-ab-benchmark-v9 \
  /userdata/ab_v9_ros_adb_001 \
  config/ros.json
```

这个脚本只用 Ubuntu 的录制数据发布 ROS 消息，再转发到 X5 执行真实模型，不能把它称为 X5 相机实时录制。真实 ROS 相机话题也可以接到 Ubuntu 转发器，话题/CameraInfo/Q8 约定同第7节。应按当前方法启动正确的相机 mode1/mode2 生产者。

已有真实 ROS 话题时，分别启动：

```bash
# X5
bash tools/run_one.sh --config config/ros.json --source ros \
  --method optimized_stereo --suite matched_roi --output /userdata/ab_ros_live_001

# Ubuntu
adb forward tcp:19091 tcp:19091
install/ubuntu_ros/bin/ab_benchmark --config config/ros.json \
  --method optimized_stereo --bridge-forward
```

X5 的 ros.json 不读取离线内参文件，内参随 ROS Frame 传入。Ubuntu bridge_host=127.0.0.1 表示经 ADB 转发；直连局域网则填写 X5 IP。ROS 发布端、转发器在另一台机器时，X5 summary 的 CPU/RSS 只表示 X5 算法进程，不合并两台机器的资源。原生 DDS 方式使用 ros_dds.json 并通过 --with-ros 构建。

报告新增 pairing.json：只把原时间戳、左右图字节摘要和内参一致的记录判为同输入，并统计分割/Q8 的逐帧精确一致数。不同相机实时捕获不会被错误当成配对精度样本。

补充：本基准固定检测、分割同时开启，每帧执行且双分支并行；不支持通过 mapping.json 静默关闭其中一个分支。两端 ROS 配置的 frames、warmup、尺寸、话题与 bridge_port 应一致。segmentation.png 保存原始类别 ID，直接查看时可能较暗；它用于逐像素核对，语义颜色可在 semantic.ply 查看。

本轮交叉产物的 OpenCV SONAME 为 libopencv_core.so.4.5d、libopencv_imgproc.so.4.5d、libopencv_imgcodecs.so.4.5d，PCL 为 1.12。运行包提供本项目 SDK 与模型，使用板端已有的匹配 OpenCV/PCL/Horizon 运行库；上板先执行 ldd，详细动态依赖见 ELF_DEPENDENCIES.txt。若板端库版本不同，应使用板端匹配 sysroot 按第5节重新编译。
