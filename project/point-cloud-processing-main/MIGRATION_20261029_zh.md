# 检测、分割、融合迁移说明

原始输入是 `point-cloud-processing-main.zip`，新功能来自
`percep-demo-parallel-x5-20261029.tar.gz`。原始压缩包保留不变，当前目录为迁移后的源码。

## 实际集成

- 保留 `perception` 命名空间、`percep_main` 包名和原节点接口，新增 `dnn_sdk`，主工程从源码构建并链接 `libpercep_dnn.so`，不使用新包中 ABI 不匹配的预编译库。
- 保留原配置和模型：34 类 `mmyolo1010.bin`、3 类 `pidnet_1020_3classes.bin`；检测阈值、类别顺序、外参和业务标签不变。
- 两模型使用独立 Tensor 和缓存，保留串行、并行和单模型调用；推理失败时释放 task。
- 分割采用缓存坐标和连续区间后处理。X5 实测发现新包优化预处理与原生产 `-ffast-math` 输入有字节差异，因此正式路径使用原预处理内核及原浮点编译选项。实验性 NEON 预处理代码保留供独立研究，正式路径不启用，不能将其初版加速数字当成本次交付性能。
- 修复分割后处理 `ResizeCoord` 在动态库中因外部函数调用产生的 1 ULP 坐标差异，保持原插值运算和 `>` 平局规则。
- 融合迁入按 `Σ(z²)` 选择检测深度簇的算法，保留当前生产版本的 UV 对齐、坡道、扩展类别、距离/高度门及稀疏点保护。新包较旧的 `ai_fusion.hpp` 没有覆盖当前版本。

## 计时与一致性

检测和分割分别直接采集前处理、BPU API 推理、后处理耗时。新增
`DnnInferTiming::detect_post_ms`；节点日志保留三位小数。BPU 阶段是
`hbDNNInfer`、等待和 task 释放的墙钟时间，包含调度和排队；输出 cache invalidate
计入后处理，不等同于硬件纯执行周期。

公开计时结构增加了字段，所有使用该头文件的调用方必须重新编译；不能给旧调用方直接替换 so。

`benchmark_tools` 用相同程序分别链接原始归档实现和新实现，原始基线只加计时观测，不改变算法。
测试输入先载入内存，预热不计入统计；效果图在计时结束后另外生成。
`compare.py --require-exact` 严格比较检测类别、名称、置信度、坐标和 mask，并检查重复稳定性。
没有使用人工标注，也不报告 mAP/mIoU 等真值精度；一致性结论仅适用于报告中的模型、配置、运行库和测试输入。

## Docker 与 X5 复现

交付包顶层包含 `tools` 和 `benchmark_tools`。本机已有构建容器为
`obs-v004-x5-build`，挂载 `/home/st/ini_code` 到 `/work/obs_v004_x5`。
独立模型测试构建：

```bash
python3 tools/package_delivery.py --output x5_validation/build-input.tar.gz
sudo -v
USE_SUDO=1 bash tools/build_x5_docker.sh
```

板端测试入口为 `tools/run_board_suite.sh` 与 `benchmark_tools/run_x5.sh`，在独立目录运行。
配置中的模型绝对路径必须对应测试部署目录。计时输出为 `frames.jsonl`，原始 mask 为 PGM。
`benchmark_tools/report_suite.py` 汇总完整测试组，严格不一致时返回失败。

## ROS 节点和回充

默认 `PERCEP_ENABLE_RECHARGE=ON` 保持原行为。本次按要求跳过回充：

```bash
sudo -v
USE_SUDO=1 bash tools/build_full_node_x5_docker.sh
```

脚本使用 `-DPERCEP_ENABLE_RECHARGE=OFF`；该选项通过 target 与 ament 导出。
直接包含 `PercepMain` 公共头的调用方也必须使用相同宏重编译，避免类布局不匹配。
OFF 时回充命令明确返回失败，地图功能不受回充 SDK 缺失影响。

Docker 中的 AArch64 ROS2 Humble、PCL、OpenCV 等标准依赖已找到，完整节点已实际进入编译；
当前本机和板端旧 `common_interface` 均缺少当前源码需要的
`common_interface/msg/camera_param.hpp`、`CameraGdcData` 和 GDC SHM 定义，
相机共享内存布局也不同。因此尚不能交付已编译的完整 ROS 感知库或声称完成融合整链路场景验收。
需要原生产版本匹配的 `common_interface` 消息源、相机 SHM 头及对应库，不能用临时结构体替代 ABI。
详细证据见 `validation/full_node_dependencies.json` 及交付报告中的完整节点构建日志。
