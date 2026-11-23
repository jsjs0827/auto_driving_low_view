> **DAG v3 / benchmark6 新增入口：** 六组测试、远程 Ubuntu ADB 指令及 RViz 结果保存见 [BENCHMARK6_zh.md](BENCHMARK6_zh.md)。本版已在本机 WSL2 Ubuntu 22.04 完成交叉编译和主机检查，尚未进行 X5 板端实测；构建方式见 WSL_BUILD_zh.md，当前检查结果见交付包 BUILD_REPORT.json。

本版为 **dag-v2 无 from/to 配置版**：15 份用户主配置仅使用 tasks + dag.sequence/parallel；详见 [DAG_FORMAT_zh.md](DAG_FORMAT_zh.md)。

# v8 DAG 配置版

本版使用 `tasks` + `dag.sequence/parallel` 自由组合已有功能；旧 `graph` 配置继续兼容。当前配置格式、限制和编译命令请先阅读 [DAG_FORMAT_zh.md](DAG_FORMAT_zh.md)。

以下为继承自 v8 的功能说明及历史验证；其中旧 `graph` 用户示例应按上面的 DAG 文档使用。

# perception-stereo-cpp-v8：多帧视差与打滑检测

在独立 v8 目录中保留 v7 的检测、分割、Stereo、点云、地图与三组相机实验，新增纯 C++ 的视觉运动、轮速参考和打滑判断。v7 目录及旧发布包没有改动。

```text
相机 / ROS / 带原时间戳的录制
        ↓ FrameBundle（同帧图像、视差、内参、身份）
disparity → history（拥有数据的最近5帧窗口）
                ├→ visual_motion（当前使用最近2帧图像+视差估计3D运动）
轮速 / Odometry → motion_reference（积分到视觉的同一时间区间）
                └────────→ slip_detection（质量检查、打滑率、时间滞回）
```

三个功能是同一张 DAG 中的独立节点，参数分别放在 `features/visual_motion/config.json`、`features/motion_reference/config.json`、`features/slip_detection/config.json`。历史帧、轮速缓冲与状态均有界；本帧整张图成功后才提交历史和打滑状态。失败帧不累计滞回时间。

**默认外参尚未确认、未接入轮速，因此先输出视觉运动和打滑“不可用”原因。** 必须提供真实相机到车体外参、同一采集时钟的测量轮速，才可以判断打滑。缺参考、弱纹理、大转弯、低速和异常时间间隔均不当作正常。当前没有融合 IMU，也没有五帧联合优化或全局 SLAM。

阅读与运行入口：

- [原理、参数、DAG、输入格式和扩展说明](docs/V8_多帧视差与打滑检测.md)
- [Docker、Ubuntu 交叉编译、X5 与 Windows 运行指令](docs/V8_编译与运行.md)
- [实际验证记录及适用范围](docs/V8_验证记录.md)
- [RViz 可视化图解：话题、配置、操作和限制](docs/V8_RViz可视化图解.md)
- [新增：X5 发布，经 USB ADB 在 Ubuntu RViz 显示，JSON 配置及两端命令](docs/V8_ADB_RViz配置与运行.md)

USB 显示使用 `features/rviz` 中的独立 C++ 模块。X5 的 `r36_rviz_publisher` 运行原有 SDK DAG，Ubuntu 的 `r36_rviz_launcher` 管理 ADB 转发、接收器和 RViz；两端共用根目录 `rviz.json`。默认 `rviz_pipeline.json` 显示相机图像、检测分割叠加、彩色视差和相机点云。发送使用有界异步缓冲；无客户端时跳过显示转换；断线或超时后清除旧画面。显示抽样不改变算法输入和原采集时间。

| 根配置 | 输入 | 内参来源 | 参考适配器 |
|---|---|---|---|
| `slip_camera.json` | SDK 左 NV12 + 原 Q8，或左右图 Stereo | 自动从相机 SDK 获取 | CSV；实时轮速可在嵌入 SDK 的应用中推送 |
| `slip_recording.json` | timeline 原采集时间、图像与视差 | 显式 `recording.intrinsics_file` | 同钟轮速 CSV |
| `slip_ros.json` | 同时间戳图像、Q8 或左右图、CameraInfo | 同帧 ROS CameraInfo | `nav_msgs/msg/Odometry` |

静态单张离线图片仍可使用原 `pipeline.json` / Windows 调试入口测试检测和分割。估计运动需要有真实采集间隔的图片序列，应使用 recording 时间线，不能用回放速度或文件序号编造采集时间。

结果保存到启动时显式指定的全新 `--output` 目录，包含逐帧 `slip.jsonl`、`metrics.jsonl`、设备共享 `bpu_samples.jsonl`、实际配置及 `complete.json` / `partial.json`。是否处理完成和是否得到有效打滑判断分别计数。

旧功能说明保留在 [v7 相机三组实验说明](docs/V7_X5相机直取三组实验.md) 和其他历史文档中；其中旧版本目录及发布包名称是历史记录。v8 的当前构建与部署以本 README 和 V8 文档为准。
