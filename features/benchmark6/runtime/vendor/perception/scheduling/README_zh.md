# scheduling：C++ 调度层

本层把 `pipeline.json` 编译成生效配置，建立功能图，再按依赖执行这一帧的功能。算法在 `features/`，对外 C ABI 在 `sdk/`。日常配置只有根文件中的 `input` 和 `graph`，不再有 Python 配置或启动程序。

用户模型路径集中指向根 `models/`；用户 JSON 中的相对路径以主 `pipeline.json` 为基准。SDK 原始配置/profile 内部相对路径以其自身文件所在目录为基准，配置器会先解析，再将本次运行快照中的模型路径写为绝对路径，便于检查。快照中的绝对路径不表示用户需要恢复模型环境变量。

内参同样有明确来源：camera 输入从 SDK 读取；ROS 默认从同帧 `CameraInfo` 读取；offline/recording 点云由配置器检查显式 `intrinsics_file`，加载独立内参 JSON 或 `metadata.json` 的 `intrinsics`，再传给输入/回放处理。路径相对主配置，且必须匹配当前处理图像的几何尺寸。内参模板在根 `calibration/`。

| 目录 | 当前职责 | 主要入口 |
|---|---|---|
| `config/` | 解析用户 JSON、检查参数与功能依赖，生成 SDK/profile/ROS 参数 | `src/native_pipeline.cpp`、`src/native_features.cpp`、`src/full_config.cpp` |
| `dag/` | 产品功能注册及数据依赖检查；构造和执行用户图 | `include/product_graph.hpp`、`src/task_graph.cpp`、`src/full_graph.cpp` |
| `runtime/` | 执行接口、状态与资源管理 | `include/perception/runtime_engine.hpp`、`src/engine.cpp` |
| `pipeline/` | 当前帧上下文、功能绑定、结果视图及生命周期 | `src/full_pipeline.cpp`、`builtin/runtime_features.cpp` |
| `inference/` | 检测/分割功能内部的预处理、模型执行、后处理 | `src/dag_pipeline.cpp`、`src/inference_session.cpp` |
| `product/` | 保留的原产品入口及状态/身份兼容 | `src/product_dag.cpp` |
| `launch/` | C++ 命令行配置、相机、回放启动 | `src/perception_config.cpp` |

`graph` 真正决定执行的功能、依赖及工作线程。检测/分割内部图由 C++ 自动生成；各自的 `enabled` 决定是否加载和执行该模型。共享点云工作区要求点云、标定、语义、坐标变换、地面及地图阶段按已选择阶段的依赖顺序执行，不能任意并发重排。

新增功能须实现算法、参数映射、产品功能注册和数据契约，再配置图；JSON 不能调用未编译的代码。ROS 发布与应用 PLY 在同步 SDK 返回后进行，发布/保存开关会映射到应用。运行中配置热加载未实现，修改配置后重新启动。

完整说明、配置图、编译和 X5 命令见 [纯 C++ v3 指南](../docs/09_纯CPP输入与Graph迁移.md)。历史 `docs/07` 可参考设计背景，但其中旧三层图和 Python 入口不用于本版。
