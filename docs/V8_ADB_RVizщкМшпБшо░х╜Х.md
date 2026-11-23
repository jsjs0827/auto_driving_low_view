# ADB / RViz 新增功能验证记录

验证日期：2026-10-15。此记录对应 `features/rviz`，没有用本机回环测试代替真实 USB 或板端测试。

| 环境 / 检查 | 实际结果 |
|---|---|
| Windows VS 2022 / x64 Debug | 完整解决方案编译成功；RViz 三项测试及相关配置两项测试 5/5 通过 |
| Ubuntu 22.04 / GCC 11.4 / 原生 SDK 发布入口 | 编译成功；显示适配、传输及既有输入/配置回归 9/9 通过 |
| Ubuntu 22.04 / ROS 2 Humble 接收端 | 编译、安装成功；配置/模拟 ADB 生命周期、socket、ROS 消息、TCP→ROS 闭环 4/4 通过 |
| AArch64 / GCC 11.4 / 匹配 X5 sysroot | 原生发布器及真实 Horizon Stereo provider 编译、链接、安装成功 |
| 有界帧预算 | 复制前拒绝超限图像/地图，按剩余字节裁剪点云；边界值和失败限频测试通过 |
| ASan / UBSan | SDK 显示适配器测试通过；包含超限预检查与真实客户端失败限频 |
| Socket 生命周期 | 断线、重连、新 session、超时清理通过；transport 测试额外连续 10 次通过 |
| 既有 ROS 应用 | 增加可选 `rviz_config` 后，`perception_node` 与 `demo` 编译、链接成功 |

完整入口补充验证使用已有合成 NV12/Q8 时间线、真实完整 SDK、多帧视觉运动与轮速参考，再经真实 TCP 和本机 ROS DDS 订阅。该用例关闭检测分割与点云计算，不用于评价模型精度或板端性能。检测框/分割配色/点云字段另由显示适配与 ROS 消息测试验证。

本次完整入口处理 12 帧、提交 12 个显示帧，完成统计采样时已发送 11 帧、替换 0、拒绝 0。显示异步发送，`display_submitted` 和 `display_sent` 分别计数；完成时的快照不代表每个提交帧都已经送达接收器。

收到的首帧 metadata 保留 `frame_number=1001`、`capture_timestamp_ns=1700000000000000000`，接收 4 路数据且没有拒绝：`input_rgb`、`disparity_q8`、`disparity_rgb`、`slip`。首帧打滑状态为 `initializing / insufficient_history`，两种运动有效标志均为 false、`slip_ratio=null`，接收器没有把初始化状态显示成正常。

以下仍需目标环境验证：实际 X5 相机/BPU 运行、真实 USB ADB 连接与带宽、Ubuntu 图形桌面的 RViz 画面。当前没有连接板子，Ubuntu 环境也没有运行 RViz GUI。发布包依赖板上已有的相机 SDK、匹配 Horizon 运行库及 PCL/OpenCV 系统库；Ubuntu 端依赖当地安装的 ROS 2 Humble 与 RViz。
