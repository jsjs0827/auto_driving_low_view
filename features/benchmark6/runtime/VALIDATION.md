> Historical v9 evidence only. The modified benchmark6 runtime has now been cross-compiled in WSL Ubuntu 22.04 and checked with host-side synthetic fixtures. Current evidence is in the delivery BUILD_REPORT.json; X5 hardware execution has not been performed. See ../DELIVERY_STATUS_zh.md and the project BENCHMARK6_zh.md.

# v9 编译与验证记录

日期：2026-10-15。这个记录只陈述实际完成的编译和主机测试，不包含 X5 真机性能结论。

## 已完成

- WSL Ubuntu 22.04、GNU 11.4.0：X5 AArch64 Release 交叉编译成功。原生产检测分割、优化 SDK、Horizon Stereo、原数值点云/标签/Patchwork/地图均编入实际程序。
- X5 产物：`install/x5/bin/ab_benchmark`、`ab_report`、`ab_config_check`，以及 `lib/libperception_sdk.so.1.0.36`。三种输入通过相同 Frame/Source 接口进入基准。
- Ubuntu x86_64 ROS Humble：C++ ROS 订阅、发布、TCP 转发程序编译成功，位于 `install/ubuntu_ros/bin/`。这个主机版本不执行 X5 模型，不用假模型输出性能结果。
- 四项 CTest 全部通过：配置/区域/统计约束；PNG/NV12/crop 与禁止隐藏 resize；同时间戳但不同图像不能误配对；相机 ABI 缓冲区所有权、SDK 1280×1088 内参缩放到 640×544、mode1/mode2、正常关闭。
- 实际 ROS 通信验证通过：离线 NV12/Q8 → ROS Image/CameraInfo → exact-stamp 配对；以及 ROS → C++ TCP 转发 → Frame 输入。收到原始时间戳 1000000000、1040000000、1080000000，没有替换为接收时间。
- 已有输出目录拒绝覆盖，错误路径不会向旧目录补写 failure.json。
- 原生产 `horizon_dnn_runner.cpp` 与原 benchmark 副本 SHA256 一致，源码正文未改。原、优化 runner 使用隔离的 C++ 符号，防止同名实现互相覆盖。

## 证据

源码工程中的 `build/evidence/`：

- `x5_build_initial.log`：完整数值库与 SDK 首次交叉构建。
- `x5_build.log`：包含最终输入适配与报告代码的增量交叉构建。
- `host_final.log`：Ubuntu 最终构建、四项 CTest 与安装。
- `transport_final.log`、`transport-slPHc2eq/`：实际 DDS/TCP 输入传输与旧目录保护检查。
- `elf_audit.txt`：ELF 架构、动态依赖、原 runner 隔离符号及 Bash 语法检查。

通信、相机 ABI 测试使用明确标注的合成夹具，只验证格式、时间、内参、模式和内存所有权。它们不是实际相机画质、模型精度、Stereo 效果或 X5 FPS 数据。

## 尚未执行

- 没有连接真实 X5 相机执行本版模型推理、三组性能测量或 PLY 质量对照。
- USB ADB 实体链路未实测；同样的 C++ ROS/TCP 帧协议已在 Linux 回环上通过。
- 当前 X5 运行包的 ROS 输入采用 Ubuntu C++ ROS 订阅转发 → TCP/ADB → X5，X5 端无需 ROS。
- X5 原生 DDS 的 CMake 入口和源代码已提供，但本机缺少完整 ARM ROS 开发包，未生成原生 DDS ARM 版本；`--with-ros` 必须指向真实 AArch64 ROS prefix，脚本会拒绝 x86 ROS 库。

## 比较范围

`original` 是原生产检测分割算法进入统一实验外壳，再接同一套原数值映射功能；不包含旧 ROS/SHM 专有框架、回充、历史、打滑或 RViz。统一采样、区域策略、模型、阈值、地图和保存设置后，比较原检测分割 → 优化检测分割 → 替换 Stereo 的影响。

`native` 保留 Q8 全图，Stereo 为其实际 ROI；`matched_roi` 才统一三组点云区域。离线 Q8 已提前生成，不能把回放耗时当作原 SDK 视差生成时间。真实相机三次采集不是同帧，需查看 pairing.json 再做输出一致性判断。
