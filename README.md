# V2 感知 DAG

此分支采用本机 `perception-stereo-cpp-v8-dag-v6-algorithm-plugins` 的 2026-11-12 运行时优化版本，对应同日 X5 的 SDK、俯视图逐格对齐、回充和真实相机长测。配置支持离线回放以及由宿主提供输入，保留检测、分割、双目、暗场相关 RAW 输入、回充和地图功能。

- [工程说明](README_zh.md)
- [Docker 交叉编译](BUILD_DOCKER_zh.md)和 [X5 运行](RUN_X5_zh.md)
- [DAG 格式](DAG_FORMAT_zh.md)和 [输入接口](INPUT_SLOTS_zh.md)
- [测试报告](test-reports/README.md)

本分支源码按交付配置整理。构建校验会拒绝开启仅供本机实验的相机输入选项。V1 的 11 月 23 日集成验证使用的是同一份 X5 V2 SDK，报告中的 Q8 地图和回充结论对应 V1 宿主调用路径。
