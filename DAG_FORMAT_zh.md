# v8 DAG 配置版：流程配置与编译

本版目录名为 `perception-stereo-cpp-v8-dag-v2`，基于 v8 的完整产品调度框架生成。用户主配置改为 `tasks` 加 `dag.sequence/parallel`。原 v8 的 `graph.nodes/depends_on` 仍可读取，但同一份配置不能混用两种入口。算法和模型没有因配置迁移而替换。

## 最小配置

下面是根目录相对路径下可用的检测、分割并行配置。模型等参数继续读取 `features/*/config.json`；`SDK_ROOT` 应指向重新编译的本版 SDK。

```json
{
  "schema_version": 5,
  "input": {"mode": "camera", "disparity_source": "original_q8"},
  "sdk": {"root": "${SDK_ROOT}", "base_config": null},
  "engine": {"worker_threads": 3},
  "tasks": {
    "det": {"feature": "inference", "algorithm": "det"},
    "seg": {"feature": "inference", "algorithm": "seg"}
  },
  "dag": {"parallel": ["det", "seg"]}
}
```

输入由原来的 camera/ros/offline/recording 适配器提供，默认不需要在任务表再定义 `input` 节点。原内部模型阶段名如 `input.exact.pack` 不等于产品功能，不能直接移入这份用户配置。

## 顺序、并行和嵌套

任务字符串引用 `tasks` 中的名字，也可以写成 `{"task":"det"}`。任务只引用一次，多个消费者共享这次执行的结果。

```json
{
  "sequence": [
    {"parallel": ["det", "seg", {"sequence": ["stereo", "cloud"]}]},
    "labels",
    "transform",
    "ground",
    "maps",
    "publish"
  ]
}
```

这段内容放入主配置的 `dag`。`sequence` 在相邻阶段之间建立依赖，后一个阶段等待前一个阶段的全部出口；`parallel` 的分支互不增加依赖，可以在可用工作线程上并行。示例中点云只等 Stereo，检测、分割可以与该分支重叠；标签融合等待三个分支汇合。

`tasks` 每项可使用 `feature`、`algorithm`、`inputs`、`enabled`。`feature` 也可简写为任务值字符串，例如 `"cloud":"cloud"`。为兼容之前的单功能任务写法，也接受 `"features":["product.ai.detection"]`；一个产品任务只包含一个功能，不接受未实现的自定义函数名。推理功能的 `algorithm` 取 `det`、`seg`、`stereo`；图像绑定沿用 v8 支持的同帧输入。

## 不再需要 from/to

本版的 15 份用户主配置均没有 `edges`、`from`、`to`。使用 `sequence` 表达先后，使用 `parallel` 表达分支，并通过嵌套表达汇合；日常只需维护一份 `dag`。解析器仍兼容旧版可选 `edges`，便于读取已有配置。

`enabled:false` 的任务不执行。如果它出现在 `sequence` 中，调度器会跨过控制节点保留前后依赖；但不能因此删掉仍需要的数据生产者。定义但没有引用的任务必须设为 `enabled:false`。要启用回充、历史等尚未加入默认流程的可选功能，需同时将任务加入 `dag` 并组织好依赖，不能只改 enabled。

本轮保持全部已启用任务的执行依赖与 v1/旧 graph 一致。未启用任务的旧边不再写在用户配置中；新增流程仍受原有数据依赖校验约束。循环、重复引用、未知任务和缺少生产者在启动前报错。

已有功能包括检测、分割、视差、点云、标定、语义标签、变换、地面、地图、回充、历史、多帧差异、视觉运动、轮速参考、打滑、发布和录制。每个产品功能每帧只支持一个实例。共享映射工作区仍要求已选阶段按点云、标定、标签、变换、地面、地图的顺序组织。修改配置后重启，不支持运行中热加载。

## 当前配置与运行快照

根 `pipeline.json`、`camera_*.json`、`slip_*.json`、`rviz_*.json`、`benchmark.json`、`crop_compare.json`、`windows_debug.json` 和 `config/examples/` 的产品主配置已迁移。底层 SDK 的模型阶段配置和生成的 `effective_full.json` 仍使用现有内部接口，不需要用户手动维护两张图。

运行目录保存原始 `requested_pipeline.json`、编译后的 `normalized_pipeline.json` 和 `resolved.json.graph`。前者保留用户的 DAG 写法，后两者便于检查实际依赖。`validation/dag_legacy_configs/` 仅用于旧配置等价性回归。

## 编译与验证

Windows 已安装 VS2022 C++ 工具链时，在本项目根目录执行：

```powershell
cmake --preset dag-config-only
cmake --build --preset dag-config-release
ctest --preset dag-config-release
```

这组命令构建实际 C++ 配置器和调度测试，不需要加载模型、相机、OpenCV 或 ROS。生成的 `perception_config.exe` 可用 `--prepare-only` 检查真实路径及配置，但仍需把 `SDK_ROOT` 设置为有本版 SDK 配置的安装目录。

Windows 完整 CPU SDK 和调试程序沿用 `windows-vs2022` preset。X5 完整构建沿用 `bash tools/build_x5_v8.sh`，Ubuntu 容器流程见原 `docs/V8_编译与运行.md`，将源码目录名替换为本版目录即可；必须重新编译本版配置器，不能使用旧 v8 可执行文件读取 schema 5。`--with-ros` 仍要求真实 AArch64 ROS 开发包。

本轮测试结果见 `DAG_VALIDATION_zh.md`。主机 DAG 验证不等于 X5 相机、BPU 性能或模型精度已验证。
