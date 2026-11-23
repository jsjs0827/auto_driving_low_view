# v6 可选传感器输入槽位

本次在当前 v6 源码中增加输入契约，不接入新的驱动、ROS 订阅或融合算法。已有的双目右图继续使用原入口。

## 数据传递

```text
psdk_full_frame → FrameBundle::borrowed → ArtifactStore → FeatureContext::get<T>()
```

| full_frame 成员 | artifact 数据键 | C 类型 | 约定 |
| --- | --- | --- | --- |
| left | input.left | psdk_frame | 原左图，NV12/I420 |
| right | input.right | psdk_frame | 原双目右图，必须与左图同帧、同尺寸 |
| rear | input.rear | psdk_frame | 后视图，NV12/I420/BGR/RGB，可独立尺寸、帧号和采集时间 |
| imu | input.imu | psdk_imu_input | 时间戳、三轴加速度 m/s²、三轴角速度 rad/s 的样本批次 |
| wheel | input.wheel | psdk_wheel_input | 时间戳、实测车体前向速度 m/s、偏航角速度 rad/s 的样本批次 |
| radar | input.radar | psdk_radar_input | 一次扫描的时间戳及 XYZ 米坐标；可选径向速度、强度 |

雷达槽位是笛卡尔点/目标集合，可由后续适配器转换毫米波或激光雷达数据后填入；没有原始扫描驱动或自动坐标转换。雷达坐标保留在传感器坐标系中。IMU 保留传感器轴系；轮速沿用现有 BODY 前向速度和绕 z 轴右手偏航约定。

所有可选指针默认 NULL。未提供的槽位不会产生对应 artifact；显式传入合法的 count=0 批次或 point_count=0 扫描表示一次空观测，与未提供传感器不同。空雷达扫描仍需有效时间戳。

## 调用方填充

新增类型在 `sdk/include/perception_sensor_input.h`，由 `perception_full_sdk.h` 自动包含。旧 `perception_slip_sdk.h` 的 psdk_motion_sample 类型移到公共输入头文件，字段、布局和 ABI 值保持不变。

```cpp
psdk_full_frame frame{};
frame.struct_size = sizeof(frame);
frame.abi_version = PSDK_FULL_ABI_VERSION;
frame.left = left_image; // 调用方已准备好的有效 psdk_frame

// 仅填已有数据，其余指针保持 nullptr。
frame.right = &right_image;
frame.rear = &rear_image;
frame.imu = &imu_batch;
frame.wheel = &wheel_batch;
frame.radar = &radar_scan;

psdk_full_process(engine, &frame, &result, &status);
```

各批次/扫描必须初始化 struct_size 和 PSDK_SENSOR_INPUT_ABI_VERSION；psdk_wheel_input 内的每个 psdk_motion_sample 也必须初始化 struct_size/abi_version。reserved 字段置零。IMU 单个样本为固定字段结构，不带单独 ABI 头。

样本批次最多 8192 项，雷达扫描最多 1048576 个点。时间戳必须非零，批次内部严格递增；有效数值必须有限。雷达的 fields 使用 PSDK_RADAR_HAS_RADIAL_VELOCITY / PSDK_RADAR_HAS_INTENSITY，未声明的可选数值字段不读取。

## 节点读取和配置

```cpp
auto imu = context.get<psdk_imu_input>("input.imu");
auto wheel = context.get<psdk_wheel_input>("input.wheel");
auto radar = context.get<psdk_radar_input>("input.radar");
auto rear = context.get<psdk_frame>("input.rear");
```

缺失可选输入时，先调用 context.contains()，或由算法规定该端口必须提供。算法应在自己的 algorithm.json 中声明输入端口，再通过任务 inputs 绑定上述数据键；仅修改 JSON 不会启动传感器采集。

旧 det/seg 内置节点的 image 绑定仍固定为 input.left；本次没有将它们改成任意图像输入。scene_cls 可绑定 image=input.rear，但其执行器目前只接受 NV12/I420。

## 所有权、时间和兼容性

- 描述结构复制到 FrameBundle/ArtifactStore；像素、样本和点数组继续借用调用方存储，必须保留到同步 psdk_full_process 返回，调用期间不能修改。
- 各传感器应使用经过校准的相机采集时钟域。框架不能从时间戳数值验证时钟是否已经标定；本次不自动同步、插值或变换外参。
- input.wheel 只提供给 DAG 消费，不自动写入既有 MotionBuffer；原打滑链继续使用 psdk_full_push_motion_samples，避免重复注入。
- 新槽位不自动进入 History，也不自动写入结果文件。原多帧左图、视差、检测/分割和分析逻辑保持原状。
- 在 psdk_full_frame 末尾追加字段；保留 ABI version=1。新 SDK 接受 PSDK_FULL_FRAME_V1_SIZE（无右图）、PSDK_FULL_FRAME_V2_SIZE（原右图尾部）和新 sizeof(psdk_full_frame)。按 struct_size 判断后才读取对应尾部。
- 使用新槽位的调用方必须部署本次重新编译的 SDK；旧版 SDK 不支持新的结构大小。

验证入口：v6_optional_sensor_input_slots（C 头文件、旧尺寸兼容、DAG 读取、空/缺失输入和非法输入）；algorithm_plugin_real_onnx_runtime 同时覆盖真实 SDK 的可选槽位传递和非法后视图拒绝。数据全部来自内存测试，不代表硬件已经接入。
