> **历史材料（v1/v2）**：本文保留用于理解原算法与历史验证。v3 的配置、路径、构建及运行入口已经改为纯 C++；本文旧命令和旧 DAG 配置不能直接用于本版。请以 [09_纯CPP输入与Graph迁移](09_纯CPP输入与Graph迁移.md) 为准。

# r36 ROS SO demo 链接签名修复 v5

本次日志表明目标 ROS、typesupport、OpenSSL、FastRTPS、Threads 和 sensor_msgs 已找到。
停止原因是示例自身的 CMake 链接声明，发生在配置阶段，还没有编译 demo.cpp。

原示例先调用 `target_link_libraries(demo PRIVATE perception::sdk)`，使用关键字签名；
随后 Humble 的 `ament_target_dependencies(demo rclcpp sensor_msgs std_msgs)`
内部使用普通签名调用同一个目标。CMake 不允许两种签名混用。

修正为：

```cmake
target_link_libraries(demo perception::sdk)
ament_target_dependencies(demo rclcpp sensor_msgs std_msgs)
```

demo 是最终可执行程序。此次仅统一链接声明，不改 SO、模型、算法调用、图像输入输出或依赖查找策略。
无需重装 ROS、修改 sysroot 或重新编译 r36 SO。使用 README 中相同的 Ubuntu 宿主机包装脚本重新构建，
脚本为本次 consumer 创建独立的容器临时目录，不复用旧 CMake 缓存。

本机使用真实 CMake 与原生 MSVC/Ninja 做针对性回归：复现旧的关键字/普通签名冲突；
统一为普通签名后，配置、生成和最小程序链接通过，SDK/ROS 三个依赖的目标链接列表均保留。
该测试中的依赖均为明确的 INTERFACE 测试目标，程序是空 main，未编译真实 ROS 节点，
未进行 AArch64 链接或 X5 运行。真实目标构建仍须在用户已有的 Ubuntu 容器中执行。
前几版依赖检查的历史报告保留原有来源和验证范围。

官方说明：

- [Humble ament_target_dependencies 实现](https://github.com/ament/ament_cmake/blob/humble/ament_cmake_target_dependencies/cmake/ament_target_dependencies.cmake)
- [CMake target_link_libraries](https://cmake.org/cmake/help/latest/command/target_link_libraries.html)
