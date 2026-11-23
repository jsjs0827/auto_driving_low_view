# v10 在线输入验证

日期：2026-10-10。本文件区分实际编译/通信验证与尚未进行的硬件测试；旧 v9/v9p1 验证文档及证据保留为历史记录。

## 本次实际完成

| 检查 | 结果 |
|---|---|
| WSL Ubuntu 原生完整构建 | demo、raw_camera、在线 provider、Horizon runner、compare runner、capture 全部编译链接并安装 |
| C++ CTest | 6/6：相机生命周期、ROS 标定/点云、YUV/padding 输入、精确同步/最新帧、张量 codec、视差几何契约 |
| JSON 前端配置 | 29/29：模式/算法、严格类型、校正与内参、模型尺寸、左右配对、QoS、环境变量、空保存目录、运行配置生成 |
| 原录制对比配置回归 | 17/17，通过；原报告测试 12/12，通过 |
| 实际 ROS + CPU SDK | 真实 r36 CPU ONNX 检测/分割及原点云融合；不匹配 Q8 时间戳被拒绝，配齐后得到同帧结果、指标和五类 PLY |
| 相机接口/线程回归 | 使用明确标记的相机 ABI 测试 fixture，经真实 adapter、采集线程、worker 和真实 CPU SDK，验证拥有数据副本、真实参数 API、最新帧丢帧统计和 SIGINT 正常关闭 |
| raw_camera 通信 | 相机 ABI fixture 验证同帧左右图和左右 CameraInfo，右投影 Tx 从 SDK bxf 换算；不冒充实际硬件采集 |
| ARM 源码编译 | mo_camera/offline_image/stereo_provider/demo/raw_camera 五个实际源文件生成 AArch64 对象 |
| ARM 链接 | 真实 Horizon 在线 provider 链接检查、compare/capture/stereo runner 编译链接成功；ARM 可执行程序未在本机执行 |
| 历史版本保护 | v9p1 原 SHA256 清单 114 个文件保持一致 |

新 provider 输入测试实际检查 NV12/I420 的行填充转换、输入不被修改、错误身份/格式/缓冲区拒绝，以及 640×352→640×544 的水平视差比例。Horizon 关闭时也进行了明确拒绝新模型的检查；没有使用假模型输出作为算法或性能证据。

实际 ROS 回归使用既有录制 NV12/Q8 数据和原 CPU ONNX 模型。相机 fixture 用于验证 vendor ABI、线程和生命周期；由它产生的图像和标定不代表真实相机标定或算法精度。

## 实测范围限制

- 本机没有 X5 相机与 BPU，未执行新 BIN 的实际在线推理，未测量 X5 新旧算法 FPS、CPU、RSS 或点云精度。
- 本机缺少完整 ARM ROS 库。五个新源文件的 ARM 编译使用目标 SDK/Horizon/OpenCV 头文件和架构无关的公开 ROS 接口头文件，没有链接 x86 ROS 库；这不等同于完成完整 ARM ROS 节点链接。
- 因此本次交付源码压缩包，不将 x86 ELF 或独立 ARM 比较 runner 冒充完整 X5 在线运行包。用文档中的已有 Ubuntu 容器构建后，生成完整 `perception-r36-ros-so-demo-stereo-online-v10-x5.tar.gz`。
- 新模型与 HBRT 的既有版本差异未通过输入接入消除，实际模型元数据和适配信息会写入在线 metrics 中。没有修改厂商运行库或模型内容。

本次日志和机器可读证据见 `evidence/v10_ctest.log`、`evidence/v10_online_native.json`、`evidence/v10_arm_sources.json` 和 `evidence/v10_arm_provider_readelf.log`。

## 可复现检查

```bash
python3 -B validation/test_pipeline_config.py
# 下列原对比测试需要系统 NumPy/OpenCV：
python3 -B validation/compare_input_test.py
python3 -B validation/test_compare_report.py

# 已有本机 ROS/SDK 时，完整编译并执行 CTest：
export SDK_ROOT=/absolute/path/to/native/perception_full_shared_sdk
bash tools/build_native.sh
```

`--prepare-only` 仅证明配置可准备，不是成功订阅、开相机或运行算法。X5 验收应分别运行 camera 和 ROS 新算法示例，检查右图订阅、非空模型元数据、Stereo 计时、连续时间戳、丢帧指标、点云话题和所选保存间隔的 PLY。
