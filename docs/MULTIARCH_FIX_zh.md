> **历史材料（v1/v2）**：本文保留用于理解原算法与历史验证。v3 的配置、路径、构建及运行入口已经改为纯 C++；本文旧命令和旧 DAG 配置不能直接用于本版。请以 [09_纯CPP输入与Graph迁移](09_纯CPP输入与Graph迁移.md) 为准。

# multiarch-fix-v4

用户 v3 日志已确认 OpenSSL 3.0.2、FastRTPS 和 rmw_fastrtps_cpp 被找到。新的致命错误是 rcl_logging_spdlog 导出的 spdlog 开发配置未被发现，仍发生在 CMake 配置阶段。

本机同套 X5 sysroot 的实际文件为：

- usr_x5/lib/aarch64-linux-gnu/cmake/spdlog/spdlogConfig.cmake
- usr_x5/lib/aarch64-linux-gnu/cmake/fmt/fmt-config.cmake
- 两份配置引用的 ARM libspdlog.so.1.9.2 / libfmt.so.8.1.1

这是本机文件检查，不能单凭这一检查保证容器拥有全部相同文件。v4 会在容器预检中确认这些开发配置及它们引用的 ARM 库，缺失或受保护的文件会输出准确路径。

修复内容：

- 设置 CMAKE_LIBRARY_ARCHITECTURE=aarch64-linux-gnu，使 lib/<arch>/cmake 的标准查找规则可用。
- 显式设置 spdlog_DIR 和 fmt_DIR，避免依赖交叉编译器的架构目录自动推断。
- 读取目标包导出的 IMPORTED_LOCATION，检查实际引用的共享库存在且为 AArch64 ELF；保留原 CMake 包及其版本检查，不伪造 spdlog/fmt 配置。
- 保留前面已经通过的 AMENT 资源索引、OpenSSL 和 dl 设置。

官方搜索规则：https://cmake.org/cmake/help/v3.22/command/find_package.html#config-mode-search-procedure

验证仅覆盖本机包配置发现、ARM 库元数据和脚本回归。完整 ROS/ARM 编译与 X5 运行仍需用户容器和板端验证；不修改任何现有算法实现、SO、模型、ROS 安装或 sysroot。
