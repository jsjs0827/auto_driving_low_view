> 本版用户配置已经迁移为 tasks + dag.sequence/parallel；请以根目录 DAG_FORMAT_zh.md 为准。以下旧 graph 说明用于理解内部依赖和兼容格式。

v6 质量入口是根 crop_compare.json，纯性能入口是 benchmark.json；说明见 ../docs/V6_CROP_同区域对比与X5运行.md。默认 Stereo 使用同宽底部crop。以下是通用配置规则。

# 配置目录

v4 的日常配置是项目根 `pipeline.json`：`input` 选择数据来源与视差方法，`graph` 选择功能及依赖。检测、分割、Stereo 分别使用 `feature: "inference"` 和 `algorithm: "det" / "seg" / "stereo"`。模型和功能参数放在 `features/<功能>/config.json`；`camera.json`、`visualization.json` 在项目根。

权重统一存放在根 `models/x5/`、`models/cpu/`。功能配置的 `model` 使用相对主 `pipeline.json` 的路径，例如 `models/x5/mmyolo0828.bin`，不相对功能 JSON 的目录，也不依赖启动时的工作目录。替换模型直接修改相应 `model` 字段，详见 [模型管理](../models/README_zh.md)。

相机直取的内参自动从 SDK 获取；ROS 默认使用同帧 `CameraInfo`。离线图像/录制数据需要点云时，分别设置 `features/offline/config.json` 的 `intrinsics_file` 或主配置 `recording.intrinsics_file`，引用真实内参 JSON 或录制 `metadata.json`；不再把离线内参数值内嵌在输入配置中。路径相对主配置，模板和规则见 [calibration/](../calibration/README_zh.md)。

`examples/` 有三个可直接交给配置器的 v4 ROS 示例：

- `det_only_ros.json`：只订阅左图、运行检测，不需要右图和内参。
- `stereo_cloud_ros.json`：左右图生成视差、几何点云，不运行检测和分割。
- `full_ros.json`：检测、分割、Stereo、点云和语义融合的默认流程。

这些主配置引用同目录的 `detection.json`、`segmentation.json`、`stereo.json`，模型路径用 `../../models/`。其他功能配置引用项目 `features/`。所有路径按各主配置所在目录解析，迁移整个项目后仍有效。

运行前设置已编译 SDK 的绝对路径，再选一个新输出目录：

```bash
export SDK_ROOT=/userdata/perception-stereo-cpp-v4/install/perception_full_shared_sdk
bash tools/run_pipeline.sh --config "$PWD/config/examples/det_only_ros.json" \
  --prepare-only --run-dir /userdata/results/det_check_001
bash tools/run_pipeline.sh --config "$PWD/config/examples/det_only_ros.json" \
  --run-dir /userdata/results/det_run_001
```

`--prepare-only` 只检查配置，不运行相机或模型。ROS 完整流程需要同帧左右图和 CameraInfo。`input.disparity_source: "original_q8"` 切换为真实原始视差：不加载 Stereo 模型、不要求右图，ROS 订阅 Q8 和 CameraInfo。RGB 按需转换并缓存，主图不用单独配置 `rgb`。

旧 DAG、ROS YAML 和旧 Stereo RViz 文件集中到 `archive/config_v1/`，只用于追溯历史。schema 3 的单 `inference` 节点仍兼容，但不能和 det/seg 实例混用。

字段到源码的对应关系见根 `parameter_map.json`。C++ 配置器自动生成内部模型阶段图和 ROS 参数文件，用户只维护一张产品 DAG。旧版 v3 文档保留作迁移对照。
