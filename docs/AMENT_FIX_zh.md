> **历史材料（v1/v2）**：本文保留用于理解原算法与历史验证。v3 的配置、路径、构建及运行入口已经改为纯 C++；本文旧命令和旧 DAG 配置不能直接用于本版。请以 [09_纯CPP输入与Graph迁移](09_纯CPP输入与Graph迁移.md) 为准。

# ament-fix-v2

用户日志的首个致命错误是 CMake 配置阶段的 `No 'rosidl_typesupport_c' found`；尚未编译 demo.cpp、链接 SO 或打包。`rcutils` 的 `dl` 查找提示目前是警告。

上一版 cross_build.sh 在加载 ROS 环境后清空了 AMENT_PREFIX_PATH。即使随后设置 CMAKE_PREFIX_PATH，ament 的资源索引查找仍从环境变量 AMENT_PREFIX_PATH 获取路径，因此 CMake 可以找到 rclcppConfig.cmake，却枚举不到 typesupport 实现。此错误不能据此认定 ROS 开发包都缺失。

修复范围仅为 ROS 调用示例的构建辅助脚本：

- 保留对 rclcpp、sensor_msgs、std_msgs 的真实 ARM 库检查。
- 补查 rosidl_typesupport_c / rosidl_typesupport_cpp 索引及对应的 ARM 调度库、实现库；不伪造资源文件。
- 从已核验的目标前缀设置 AMENT_PREFIX_PATH，同时写入初始 CMake cache 的环境，避免直接 CMake 调用再次丢失。
- 每次宿主机脚本使用全新的临时构建目录，避免复用失败的 CMake cache。

官方实现依据：

- https://github.com/ros2/rosidl_typesupport/blob/humble/rosidl_typesupport_c/cmake/get_used_typesupports.cmake
- https://github.com/ament/ament_cmake/blob/humble/ament_cmake_core/cmake/index/ament_index_get_prefix_path.cmake

如果新预检仍报告索引缺失，才进一步检查完整目标 ROS 安装；通过 --target-ros-prefix 传入实际 ARM 前缀。不能通过禁用 typesupport 检查、创建空占位索引、混入 x86 库解决。

本机没有实际 ROS2/ARM 编译环境。回归测试只验证脚本、资源索引选择及 CMake 环境传递，不表示已完成用户容器的完整交叉链接。
