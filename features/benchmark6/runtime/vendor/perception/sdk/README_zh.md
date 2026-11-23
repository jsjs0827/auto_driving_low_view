> **历史材料（v2）**：目录内算法原理仍可参考；旧配置生成与启动入口已在 v3 退役。当前入口、原生工具和编译命令见 [纯 C++ v3 指南](../docs/09_纯CPP输入与Graph迁移.md)。

# SDK 接口与构建入口

本目录负责公开接口和构建安装，数值算法在 `features/`，配置/DAG/业务编排在 `scheduling/`。主构建入口是 `CMakeLists.txt`，可通过项目根 `tools/build_backend_sdk.sh` 使用已有依赖构建到独立安装目录。

| 目录 | 作用 |
|---|---|
| `include/` | 公开 C ABI 头文件，保留既有结构版本与函数名 |
| `src/perception_sdk.cpp` | 8 个基础推理 C 入口的薄转发 |
| `src/perception_full_sdk.cpp` | 11 个 full/时序 C 入口的薄转发 |
| `internal/` | 转发到 `perception::sdk_internal::impl_*` 的内部声明，不是新的公开 API |
| `cmake/` | 安装/package 配置、目标工具链及依赖查找 |
| `examples/` | 公开接口调用示例 |

一帧 full 调用为：

```text
features/sdk_bridge::Caller::process
  → sdk/src/perception_full_sdk.cpp::psdk_full_process
  → scheduling/pipeline/src/full_pipeline.cpp
     ::perception::sdk_internal::impl_psdk_full_process
  → scheduling/dag/src/full_graph.cpp::Graph::runContext
  → TaskGraph → features 中的算法
```

基础推理入口同样转到 `scheduling/inference/src/inference_session.cpp`。本轮把原函数体归入会话/业务层，并保留 C ABI 名称和错误边界；薄转发本身没有另写一套校验/算法逻辑。SDK view 中的借用指针寿命仍由结果对象决定。

安装前缀仍提供 `profiles/`；基础 C ABI 中空指针默认路径仍相对进程工作目录，分别为 `profiles/original.cfg` 和 `profiles/dag_exact.json`，没有改成源码路径。源码模板现在位于 `scheduling/config/profiles/`。直接源码运行应显式传真实配置/profile 路径；根前端会展开并传入实际生效路径。

默认 ROS/比较程序加载 `SDK_ROOT` 指定的已安装库。修改这里或调度/功能源码后，必须重编 SDK 并切换安装前缀；只重编应用不会更新原 `.so`。完整命令见 [交付与验证](../docs/00_交付与验证.md)，结构、线程和剩余问题见 [调度层设计](../docs/07_调度层设计.md)。
