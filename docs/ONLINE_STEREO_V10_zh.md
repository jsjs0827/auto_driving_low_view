> **历史材料（v1/v2）**：本文保留用于理解原算法与历史验证。v3 的配置、路径、构建及运行入口已经改为纯 C++；本文旧命令和旧 DAG 配置不能直接用于本版。请以 [09_纯CPP输入与Graph迁移](09_纯CPP输入与Graph迁移.md) 为准。

# v10 在线双目 Stereo：JSON 配置与运行

v10 在同一算法节点中增加了新 Stereo 的实时相机直取、实时 ROS 双目订阅和离线双目输入。检测、分割、原点云反投影、地面分割、语义融合和地图继续由完整 SDK 执行；新算法仅替换视差来源。相机、Stereo 模型和完整 SDK 在启动时初始化，单独的算法工作线程逐帧同步调用 SDK，停止时释放。

本次配置整理后，日常只改 `config/pipeline.dag.json`；完整模板在 `config/examples/`，旧工具配置在 `config/legacy/`。各文件用途、切换方法和保存路径见 [配置入口](../config/README_zh.md)。下面命令使用整理后的路径，硬件验证范围仍见 [验证记录](../validation/V10_VALIDATION_zh.md)。

## 一份 JSON 控制输入和视差来源

统一入口默认读取 `config/pipeline.dag.json`，出厂配置为相机直取＋新 Stereo 640×352：

```bash
bash tools/run_pipeline.sh \
  --run-dir /userdata/stereo_results/live_camera_$(date +%Y%m%d_%H%M%S)
```

`run_pipeline.py` 本身只依赖 Python 3 标准库，使用板载 `/usr/bin/python3`，不需要虚拟环境、PyYAML 或 ONNX Runtime。节点内的新 Stereo 直接调用 X5 C++ BPU 推理实现。`recording` 仍委托原对比工具，该工具需要板载 NumPy/OpenCV。

| `input.mode` | 实际输入 | 支持的视差来源 |
|---|---|---|
| `camera` | MoGeneralSDK 同帧左/右 RGBD 图像 | 新 Stereo 或 SDK 原 Q8.8 |
| `ros` | 同时间戳图像消息及相应内参 | 左右图→新 Stereo，或左图＋原 Q8.8 |
| `offline` | 明确配对的左右图文件列表 | 新 Stereo；原 Q8.8 回放使用 `recording` |
| `recording` | 录制目录中的左右 NV12、原 Q8.8、内参和 manifest | 原有离线对比流程 |

视差在 `dag` 中选择：

```json
{"id":"disparity","feature":"disparity.raft_stereo","depends_on":["input"]}
```

原方法改为 `disparity.original_q8`，同时删除仅用于新模型的 `stereo` 对象。相机原方法必须使用 `input.camera.disparity_mode: 2`。ROS 原方法将 `right_topic` 换为 `q8_topic`。

这些切换已提供完整示例：`config/examples/camera_original.dag.json` 和 `config/examples/ros_original.dag.json`。建议复制所需模板为 `config/pipeline.dag.json` 后继续使用统一入口，也可在启动命令中直接指定模板。恢复相机新方法使用 `config/examples/camera_stereo.dag.json`。

当前外层 DAG 是固定四节点契约：`input → disparity → perception → report`，校验节点名称和依赖。它不是任意增加任务的调度器。输入节点功能必须对应实际模式，如 `input.camera` 或 `input.ros`；在线输出节点为 `output.online`，原离线比较配置继续为 `output.compare`。真正的检测/分割与视差任务并行执行仍在 SDK 内部。

## 输入尺寸、校正和模型格式

原始双目图和融合坐标保持 **640×544**，使用该尺寸的真实校正内参。模型适配独立配置：

```json
"stereo": {
  "model": "${STEREO_MODEL}",
  "input_mode": "resize",
  "input_width": 640,
  "input_height": 352
}
```

新模型对外显示 NCHW `1×3×352×640`；实际送入方式以模型运行时的图像类型、布局和对齐元数据为准。当前 NV12 模型使用 NV12 数据，不能仅凭 NCHW 把输入改成三通道 RGB float。程序按模型元数据打包，左右图使用相同的缩放，输出视差恢复到原图尺寸，并根据水平方向缩放比例恢复视差量纲。检测、分割和点云内参不跟着模型高度改成 352。

`input_mode: strict` 拒绝图像尺寸和模型输入不一致；`resize` 明确允许模型支路的缩放。宽高必须成对设置，且为正偶数；不填写宽高时由模型元数据读取。

相机直取使用 `input.intrinsics: "camera_sdk"`，从 SDK 获得真实校正参数。ROS 推荐 `"camera_info"`，读取左右 `CameraInfo.P`，并由右目投影矩阵确定基线。ROS 和离线新 Stereo 必须明确设置：

```json
"rectified": true,
"calibration_confirmed": true
```

这两个开关表示你确认图像已经双目校正、内参可信，程序不会用开关替代校正。若输入未校正，应先使用真实标定完成左右校正。显式内参格式为 `{"width":640,"height":544,"fx":...,"fy":...,"cx":...,"cy":...,"bxf":...}`；`bxf = 焦距像素 × 基线毫米`，不要填基线米或随意复制别的相机内参。

离线模板 `config/examples/offline_stereo.dag.json` 故意将内参设为 0、确认开关设为 false；必须填写该数据的真实校正参数后运行。`left_files` 和 `right_files` 按索引逐帧对应，数量必须一致，不能同一文件充当左右图；源图必须是校正后的 640×544，不能先做未知裁剪或拉伸再套用旧内参。

## Ubuntu 交叉编译

沿用已有容器和 SDK 路径：

```bash
cd /home/st/ini_code/perception-r36-ros-so-call-demo-stereo-online-v10
bash tools/build_in_container.sh \
  --container obs-v004-x5-build \
  --sysroot /work/obs_v004_x5/sysroot_docker \
  --sdk-root /tmp/r12-v36-20261008-215622-3654808/install/perception_full_shared_sdk \
  --stereo-horizon-root /work/obs_v004_x5/sysroot_docker/usr_x5 \
  --out /home/st/ini_code/r36_ros_demo_release_v10
```

将生成的 `perception-r36-ros-so-demo-stereo-online-v10-x5.tar.gz` 复制到 X5 `/userdata` 并解压。完整 ROS 运行包包含 `demo`、`raw_camera` 和 JSON 前端；只含比较 runner 的独立运行包不能代替此包运行在线 ROS 节点。

## X5 相机直取：新 Stereo

```bash
cd /userdata
tar -xzf perception-r36-ros-so-demo-stereo-online-v10-x5.tar.gz
cd /userdata/perception-r36-ros-so-demo-stereo-online-v10-x5
python3 tools/verify_files.py

export SDK_ROOT=/userdata/ai_infer_r12_v36/perception-core-x5-r12-v36/install/perception_full_shared_sdk
export DET_MODEL=/userdata/ai_infer/data/mmyolo0828.bin
export SEG_MODEL=/userdata/ai_infer/data/pidnet_0825_3classes.bin
export STEREO_MODEL=/userdata/stereo_pro/r34_tail_pointwise_shuffle2_grouped_grid_352x640_nv12_mixed.bin

# 恢复相机新算法模板；已修改的主配置需要时先备份
cp config/examples/camera_stereo.dag.json config/pipeline.dag.json
bash tools/run_pipeline.sh \
  --run-dir /userdata/stereo_results/live_camera_$(date +%Y%m%d_%H%M%S)
```

此模式由算法进程直接开相机，不再另启 `raw_camera`。相机由同一 SDK 获取同帧左右图。新模型分支不给完整 SDK 传入旧 Q8.8，避免 SDK 优先消费旧视差而跳过新模型。

新方法默认 `input.camera.disparity_mode: 1`，请求相机 SDK 关闭其内部原视差计算，代码不调用原 `calculate`；模型使用 SDK 的左右图。若板载 SDK 在模式 1 不提供左右图，可改为 2，保留相机内部原计算，融合依然使用新模型输出。原方法固定模式 2，并使用 SDK 原 Q8.8：

```bash
bash tools/run_pipeline.sh config/examples/camera_original.dag.json \
  --run-dir /userdata/stereo_results/live_original_$(date +%Y%m%d_%H%M%S)
```

每次只运行一个相机拥有者。用所属终端的 Ctrl+C 正常停止，再切换算法。

## X5 ROS 双目订阅：新 Stereo

终端一先运行原始相机发布节点：

```bash
cd /userdata/perception-r36-ros-so-demo-stereo-online-v10-x5
bash tools/run_camera_x5.sh infer \
  -p camera_disparity_mode:=1 \
  -p publish_camera_info:=true \
  -p input_reliability:=best_effort
```

`infer` 的相机内部视差模式默认为 1，上面显式写出便于检查。它发布同一次采集的左右图和真实校正 `CameraInfo`。如果板载 SDK 在模式 1 不提供左右图，将命令改为 `-p camera_disparity_mode:=2` 重试；这保留相机原视差计算，但 `infer` 不发布 Q8，算法节点仍使用新 Stereo 输出。终端二设置上述四个模型/SDK 环境变量后运行：

```bash
cd /userdata/perception-r36-ros-so-demo-stereo-online-v10-x5
bash tools/run_pipeline.sh config/examples/ros_stereo.dag.json \
  --run-dir /userdata/stereo_results/live_ros_$(date +%Y%m%d_%H%M%S)
```

默认订阅：

| 内容 | topic |
|---|---|
| 左图 | `/camera/rgbd_left_nv12` |
| 右图 | `/camera/rgbd_right_nv12` |
| 左内参 | `/stereo/left/camera_info` |
| 右内参 | `/stereo/right/camera_info` |

左右图必须为 `nv12/i420`、640×544，属于同次采集，`header.stamp` 完全相同且非零；左右 `CameraInfo` 也必须具有相同采集时间。左图与左内参的 `frame_id` 一致，右图与右内参的 `frame_id` 一致，左右光学坐标系名称应明确。不能用接收时刻重打时间戳，也不能把普通 depth-mm 图冒充 Q8.8。

`queue_capacity` 是有限配对缓存，允许 1～16，默认 4。配齐后进入最新帧待处理槽；算法忙时，新帧替换尚未开始处理的旧帧，避免延迟持续积累。`metrics` 分别记录未配齐淘汰和待处理替换数量。增大缓存不会提高推理速度。订阅 `best_effort` 可接 best-effort 或 reliable 发布者；订阅 `reliable` 需要 reliable 发布者。所有终端需使用一致的 ROS domain 和可互通的 DDS 环境。

检查通信：

```bash
ros2 node list
ros2 topic info /camera/rgbd_left_nv12 --verbose
ros2 topic info /camera/rgbd_right_nv12 --verbose
ros2 topic echo /perception_ros_so_demo/result --once --field data
ros2 topic echo /perception_ros_so_demo/metrics --once --field data
```

结果、掩码、地图和点云由算法节点发布，点云 topic 为 `/perception_ros_so_demo/clouds/{camera,robot,ground,obstacle,semantic}`。可在 RViz2 添加对应 PointCloud2，设置适当 Fixed Frame，或使用已保存 PLY 的离线查看器。

切换 ROS 原算法时，先用 Ctrl+C 停止这两个节点。终端一改为 `map`，取得并发布 SDK 原始 Q8.8：

```bash
bash tools/run_camera_x5.sh map \
  -p camera_disparity_mode:=2 \
  -p publish_camera_info:=true \
  -p input_reliability:=best_effort
```

终端二使用原算法 JSON：

```bash
bash tools/run_pipeline.sh config/examples/ros_original.dag.json \
  --run-dir /userdata/stereo_results/live_ros_original_$(date +%Y%m%d_%H%M%S)
```

此时算法订阅左图、原 Q8.8 和左右 `CameraInfo`，不加载新 Stereo 模型。左右图新方法和原 Q8.8 方法的结果来源会在 `resolved.json` 和每帧性能记录中明确标识。

## 保存路径、性能数据和配置审计

`--run-dir` 必须是不存在的新目录。该目录保存请求 JSON、`resolved.json`、ROS `parameters.yaml`、`effective_full.json` 和保留原有融合参数的 `original_start.json`。默认 `output.metrics: true` 还逐帧写入 `metrics.jsonl`，便于查看检测、分割、Stereo、SDK 图耗时、CPU、RSS 和处理 FPS。

指标中 `det_*_ms/seg_*_ms` 是 SDK 的检测/分割分阶段耗时，`stereo_ms` 包含新视差的输入适配、模型推理和视差恢复，`task_trace` 保留 SDK 各任务耗时。`completed_fps_since_ready` 是节点就绪后完成帧数除以墙钟时间，包含等待、传输、调度和保存。`process_cpu_ms` 与 CPU 百分比统计整个进程，包括取帧/DDS 线程，可能超过单核 100%；`rss_kib` 为进程常驻内存。不要把这些字段当作 BPU 纯推理耗时。

`output.save_root: ""` 禁止 SDK 结果和 PLY 保存，自动将 `save_every_n` 置零，即使示例中的 `every_n/ply` 仍开启。它不禁止运行目录里的配置和性能诊断。`metrics: false` 禁止 JSONL 性能文件。需要保存点云时，例如：

```json
"output": {
  "save_root": "/userdata/stereo_results/frames",
  "every_n": 10,
  "ply": true,
  "publish_clouds": true,
  "publish_maps": true,
  "metrics": true
}
```

实际 SDK 结果位于 `save_root/运行目录名/frame-帧号-时间戳/`，避免不同测试共用帧目录。SDK 标准结果文件与各点云 PLY 保存在对应帧目录。保存和 ROS 发布会增加端到端开销；比较计算性能时保持采样步长、地图开关、保存频率和发布设置一致。

在线指标的处理 FPS 与原 `recording` 重复回放的 benchmark FPS 口径不同。相机直取涉及取帧等待；ROS 输入存在传输、调度和缓存丢帧；保存 PLY/地图有额外开销。SDK 原视差内部计算耗时不能从新模型回调计时中推断，应结合端到端输出 FPS、CPU/RSS 和相同配置比较。

ROS 模式的 CPU/RSS 只统计算法节点进程，不包含另一个 `raw_camera` 进程；相机直取模式包含本进程内的相机 SDK 线程。比较新旧算法时应保持输入模式一致；需要整个 ROS 系统资源时，还应单独统计相机发布进程。

可只校验并生成配置，不开相机、不订阅、不推理：

```bash
/usr/bin/python3 tools/run_pipeline.py \
  --run-dir /userdata/stereo_results/prepared_$(date +%Y%m%d_%H%M%S) \
  --prepare-only
```

`resolved.json` 会明确记录 `evidence: prepared_only`；这不是硬件运行成功或性能测量结果。正式运行时启动器直接替换为真实算法进程，Ctrl+C/SIGTERM 传给节点处理，关闭相机并释放 SDK。

原离线 DAG 继续可用，例如：

```bash
export CAPTURE_ROOT=/userdata/stereo_compare/capture_20261010_103121
/usr/bin/python3 tools/run_pipeline.py config/examples/compare_stereo.dag.json \
  --run-dir /userdata/stereo_results/recording_$(date +%Y%m%d_%H%M%S)
```

需要同时比较两种视差时，继续使用原入口：

```bash
/usr/bin/python3 compare/run_compare.py config/examples/compare_stereo.dag.json \
  --output /userdata/stereo_results/ab_$(date +%Y%m%d_%H%M%S) --both
```
