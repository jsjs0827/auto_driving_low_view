> 此文件为继承的 v2 历史验证记录，未覆盖本版 benchmark6 修改。本版另已完成 WSL 交叉编译和主机检查，未进行板端实测；当前交付状态见 features/benchmark6/DELIVERY_STATUS_zh.md 和交付包 BUILD_REPORT.json。

# DAG v2 无 from/to 配置版验证

日期：2026-10-19。项目：`perception-stereo-cpp-v8-dag-v2`，基于 v8 DAG v1 的独立副本。

- 15 份用户主配置与示例均已删除 `edges`，没有 `from/to` 字段。解析器继续兼容旧的可选 edges。
- 已启用任务的算法、输入、开关、线程数及依赖可达关系与旧配置一致；未启用任务的旧边不再保留在用户配置中。启用新任务时须将它加入 dag 并配置依赖。
- MSVC 19.44 x64、Ninja、Release 主机编译成功。
- CTest **5/5 通过，0 失败**；结构化 DAG 测试 447 项断言通过。
- 验证实际调度器串行、并行、汇合和失败传播，以及原有运动算法、主配置、运动配置和相机基准配置回归。
- 本轮原始日志：[validation/evidence/dag_v2/](validation/evidence/dag_v2/)。继承的 dag_v1 日志为上一版历史证据。

本轮没有执行 X5 交叉编译、真实相机、BPU 模型或 FPS 测量。新项目保留原有完整构建入口；配置格式与命令见 [DAG_FORMAT_zh.md](DAG_FORMAT_zh.md)。v8、v9、dag-v1 目录及已发布包均未被本轮修改。
