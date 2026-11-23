# 算法注册与通用配置验证报告

验证日期：2026-10-20。项目独立复制自 perception-stereo-cpp-v8-dag-v4-ubuntu，原目录未修改。

后续在同一 v6 目录增加了可选传感器输入槽位；最新验证为 **48/48 通过**，具体见文末“可选输入槽位增量验证”。下文 47 项为此前算法注册版本的验证记录。

## 已交付的行为

- 新算法使用 runtime::Feature / FeatureContext 统一接口。
- 每个算法的一份 features/*/algorithm.json 同时驱动配置目录、参数规则和执行工厂生成；CMake 自动发现。
- 配置按任务实例保存于 SDK algorithms 容器，可为同一种算法创建多个实例并分别覆盖参数。
- 自动处理模型绝对路径、默认参数、类型/范围/枚举校验、未知字段和图配置一致性。
- 新算法可以独立运行，也可以与原有节点顺序或并行组合。发布/记录节点必须等待所有生产实例。
- 输出通过统一 JSON 容器和新增 C ABI getter 提供，不扩展现有结果结构的布局。
- scene_cls 是可执行的分类实现，调用真实 Backend；公共配置/图源码中没有 scene_cls 专用字段或分支，Context 头文件与 v4 相同。

## 实际构建与测试

主机：Windows x64，MSVC 19.44.35217，CMake 3.30.3，Ninja。CPU ONNX Runtime 1.16.3。

构建设置：R29_WITH_ORT=ON，R20_WITH_BPU=OFF，R33_WITH_MAPPING=OFF，R33_WITH_RECHARGE=OFF。完整主机构建成功。

最终 CTest：**47/47 通过，0 失败，总耗时 165.54 秒**。

| 范围 | 结果 |
| --- | --- |
| 原有 45 项回归，包括 DAG、配置、C ABI、时序、输入所有权和后处理 | 全部通过 |
| algorithm_plugin_configuration | 通过 |
| algorithm_plugin_real_onnx_runtime | 通过 |
| 关闭 SDK、没有 ORT/BPU 的纯配置工具独立构建 | 通过 |

配置测试实际覆盖：两个同类算法实例、独立参数覆盖、模型绝对路径、分类独立运行时不启动旧 det/seg、未知参数拒绝、缺失模型拒绝、禁用实例不要求模型、图/算法配置不匹配拒绝，以及发布节点没有等待全部同类实例时的拒绝。

实际推理测试通过完整 SDK API 加载自建 ONNX 测试模型并处理内存 NV12 图像：红色输入选出 class 0，蓝色输入选出 class 2；两个实例分别使用 top_k=1、top_k=2。测试还验证了销毁 engine 后结果仍可读取，以及无效 getter 请求清空输出指针。

测试模型仅计算 RGB 各通道的空间均值，是输入相关的框架测试模型，不是训练后的业务场景分类器。没有以测试模型代替业务精度结论。

## 可复查文件

- validation/evidence/plugin_test_results.xml：最终 47 项 JUnit 结果。
- validation/evidence/plugin_ctest.log：最终 CTest 日志。
- validation/evidence/plugin_host_build.log：主机构建日志。
- validation/evidence/plugin_catalog_only_build.log：不包含推理后端的配置工具构建日志。
- validation/create_scene_cls_fixture.py：测试 ONNX 模型的可复现生成器。
- validation/algorithm_config_test.cpp、validation/algorithm_runtime_test.cpp：新增测试。

## 未验证的范围

没有在本机完成 Docker/AArch64 交叉编译、X5 实机推理、相机或 DDS/ROS 实际输入测试，也没有新的板端 CPU/BPU/RSS 性能与业务分类精度数据。BPU 路径需真实 scene_cls.bin 及与板端匹配的 SDK 环境验证。原 benchmark6 的历史结果不作为本次新算法验证证据。

后续新增 C++ 算法需要重新编译；本版是编译期注册，不是共享库热加载。

## 可选输入槽位增量验证

2026-10-20 在当前 v6 源码中追加 psdk_full_frame 可选尾部，经 FrameBundle 传递到 ArtifactStore。右图沿用 input.right，新增 input.rear、input.imu、input.wheel、input.radar。原 v4 目录及历史 benchmark6 内嵌副本未修改。

MSVC x64 / ONNX Runtime 1.16.3 增量构建成功。完整 CTest **48/48 通过，0 失败，170.81 秒**，其中原有 47 项全部通过。

- v6_optional_sensor_input_slots：C 头文件兼容；右图、独立后视图和各传感器批次的实际 DAG 读取；借用存储指针保持；缺失与空观测区分；旧 V1/V2 结构尺寸兼容；非法大小、时间戳、数量、数值和保留位拒绝。
- algorithm_plugin_real_onnx_runtime：通过真实 SDK 传入全部可选槽位，分类继续随图像变化；非法后视缓冲区返回输入错误；原结果所有权检查继续通过。
- history、多帧 JSON、轮速参考、视差、DAG 和原有 C ABI 回归继续通过。

证据：validation/evidence/input_slots_build.log、input_slots_ctest.log、input_slots_test_results.xml。

本次仅提供可选输入契约及其校验/传递，没有接入新增硬件、自动同步或融合。新槽位不会自动缓存进 history；input.wheel 不会自动写入已有 MotionBuffer。X5/AArch64 及实际传感器仍未验证。类型、初始化和时间约定见 INPUT_SLOTS_zh.md。
