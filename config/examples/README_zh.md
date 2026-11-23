> 本版用户配置已经迁移为 tasks + dag.sequence/parallel；请以根目录 DAG_FORMAT_zh.md 为准。以下旧 graph 说明用于理解内部依赖和兼容格式。

# v4 流程示例

以下文件与主 pipeline.json 使用相同配置器，模型路径相对各示例文件所在目录，已经通过配置准备与迁移测试。

| 文件 | 功能 |
|---|---|
| det_only_ros.json | ROS 输入，仅检测和发布 |
| stereo_cloud_ros.json | ROS 左右图，Stereo 和相机坐标几何点云，不运行检测/分割 |
| full_ros.json | ROS 输入，独立检测、分割、Stereo，语义点云及地图 |

detection.json、segmentation.json、stereo.json 是这些示例的模型配置，相对路径为 ../../models/...。其余配置引用项目根目录对应文件。复制示例到不同目录时须同时调整相对路径。

```bash
export SDK_ROOT=/path/to/rebuilt/v4/sdk
bash tools/run_pipeline.sh --config "$PWD/config/examples/full_ros.json" \
  --prepare-only --run-dir /tmp/v4_example_check_001
```

去掉 prepare-only 并换一个新的 run-dir 才实际运行。运行需要匹配的 X5 SDK、应用和 ROS 发布节点；配置准备通过并不代表模型已经执行。

新 Stereo 需要同帧左右 Image 及 CameraInfo。改为 original_q8 则订阅真实 Q8，不加载 Stereo 模型、不要求右图。det-only 不要求几何内参。查看和修改话题请使用 features/ros/config.json。

保存点云需增加启用的 record 节点并等待 publish，同时设置根 visualization.json 的非空 save_root、save_every_n>0、save_ply=true。
