# Windows v7 调试入口验证

验证环境：Windows x64、Visual Studio 2022 Community / MSVC 19.44、Windows SDK 10.0.26100、CMake 3.30.3、项目内 ONNX Runtime 1.16.3。

## 构建与调试符号

- 当前 v7 源码通过 MSVC 全工程 Debug 构建。
- 实际解决方案：`build/vs2022/perception_stereo_cpp_v7.sln`。
- 调试入口：`perception_windows_debug`；默认主配置为根目录 `windows_debug.json`。
- Debug 使用 `/Zi /Od /Ob0 /RTC1`，链接使用 `/debug`，生成入口和 SDK 的 PDB；SDK DLL/PDB 与 ORT DLL复制到入口目录。
- 入口直接调用当前配置库和 C ABI，没有通过子进程绕开断点。

## 真实模型运行

运行 `windows/data/sample_left.png`：解码 RGB → 调整至640×544 → 显式 RGB→NV12 → 验证522240字节 → 完整 SDK/DAG → CPU ONNX 检测/分割 → 保存。

带调试符号的运行记录在 `build/vs2022/smoke_real_onnx_debug_pdb`：正常退出，完成1帧，检测结果2个，掩码348160字节；检测与分割任务均为 succeeded，`source.yuv`为522240字节，生成`rgb.png`和`overlay.png`。这只证明本机实际推理与调试链路运行成功，不能替代X5性能或模型精度测试。

最初的 `smoke_real_onnx_02` 误用了通用 `prepareNv12` 的 RGB 返回路径，已标记无效；该目录不得用于正确性或质量判断。最终入口采用明确的`prepare(...).rgb → rgbToNv12`转换，并检查存储大小。

## 现有契约测试

首次完整 CTest 共41项，其中38项通过，3项因Windows路径字符串形式或临时目录短路径检查失败。调整测试中的路径语义比较及临时目录边界检查后，这3项全部复测通过：

- `v3_temporal_analysis_JSON_without_extension`
- `native_config_contracts`
- `camera_benchmark_config_contracts`

测试继续验证实际模型路径相对主JSON解析、移动目录不依赖当前工作目录、缺失模型拒绝、相机实验图配置和时序JSON。临时目录必须为新建目录，检查名称、类型及解析后的父目录后才清理。未重复运行耗时数分钟的完整像素校准回归；首轮该项已通过。

## 范围

此 profile 开启真实 CPU ONNX、配置、调度、离线输入与结果保存，关闭mapping/recharge/BPU/ROS及X5相机实验控制器。未验证Windows上的X5硬件或点云功能。原X5运行压缩包保持为此前交付的独立快照。
