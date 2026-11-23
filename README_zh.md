# 算法注册与通用配置版 v6

本版入口：[README_PLUGINS_zh.md](README_PLUGINS_zh.md)。新增算法使用统一接口、单份注册和 DAG 配置；原有 v4 流程继续兼容。

分类示例：[scene_cls_camera.json](scene_cls_camera.json)；主机验证示例：[scene_cls_test.json](scene_cls_test.json)。验证结果见 [PLUGIN_VALIDATION_zh.md](PLUGIN_VALIDATION_zh.md)。

可选输入槽位：[INPUT_SLOTS_zh.md](INPUT_SLOTS_zh.md)，包含右图、后视图、IMU、轮速计及雷达的结构与 DAG 数据键。

检测、分割与两种视差的 ROS、camera、离线图片六组测试：[编译与运行教程](docs/V6_三种输入六组推理测试.md)。Ubuntu 宿主机使用 `tools/build_inference_matrix_in_container.sh` 复用已有 Docker 容器，编译并取回 `perception-v6-matrix-x5.tar.gz`；该入口无需先编旧 benchmark6 主机工具或执行 `--host-only`。配置位于 `config/inference_matrix/`，离线原视差支持真实 16 位 Q8 文件。

以下保留 v4 的 Ubuntu/X5 基础环境说明。

当前入口说明：[README_UBUNTU_zh.md](README_UBUNTU_zh.md)。

交叉编译：[BUILD_UBUNTU_zh.md](BUILD_UBUNTU_zh.md)。

X5 运行与历史默认路径：[RUN_X5_zh.md](RUN_X5_zh.md)。

DAG 格式：[DAG_FORMAT_zh.md](DAG_FORMAT_zh.md)。

完整精度与指标定义：[BENCHMARK6_zh.md](BENCHMARK6_zh.md)。
