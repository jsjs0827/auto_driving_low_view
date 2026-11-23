> **历史材料（v1/v2）**：本文保留用于理解原算法与历史验证。v3 的配置、路径、构建及运行入口已经改为纯 C++；本文旧命令和旧 DAG 配置不能直接用于本版。请以 [09_纯CPP输入与Graph迁移](09_纯CPP输入与Graph迁移.md) 为准。

# sysroot-fix-v3

第二次日志已经能够枚举 C/C++ typesupport，并找到 rmw_fastrtps_cpp。当前致命错误是 FastDDS 的 find_package(OpenSSL) 未找到 OPENSSL_CRYPTO_LIBRARY。

用户随后给出的 file -L 检查确认，usr_x5/lib/aarch64-linux-gnu 中的 libcrypto.so.3、libssl.so.3 是真实 AArch64 ELF，且无版本后缀的 .so 对应这两个文件。该目录同时有 libdl.a 和 ARM libdl.so.2。因此当前问题是目标系统依赖查找路径不完整，不能据此判定 ROS 安装缺失。

上一版仅把此目录用于链接器 rpath-link。CMake 的 find_library/find_package 不会因此把该目录作为库搜索位置。

修复仅影响独立调用示例的构建配置：

- 将 usr_x5 加入 CMAKE_PREFIX_PATH；将其目标库目录加入 CMAKE_LIBRARY_PATH。
- 将 usr_x5/include 及其 aarch64-linux-gnu 子目录用于目标头文件搜索与 demo 编译。
- 检查 OpenSSL 3 的两个版本化文件为 AArch64 ELF，目标 opensslv.h 的主版本为 3，再明确设置 OPENSSL_CRYPTO_LIBRARY / OPENSSL_SSL_LIBRARY / OPENSSL_INCLUDE_DIR。
- libdl.a 由 CMake 在 X5 目录正常查找；不将其他 SoC 的 libdl.so 拿来替代。
- 保留 v2 的目标 AMENT_PREFIX_PATH 及 typesupport 检查。

不修改容器系统、sysroot、ROS 安装、现有 SO、算法、相机配置或任何模型。OpenSSL 系统文件不打进本源码包。

编译成功后，仍需在实际 X5 ROS 环境验证运行时库与消息输入。详见 README_zh.md；本机的 CMake 查找测试不等于完整 ROS/ARM 编译。
