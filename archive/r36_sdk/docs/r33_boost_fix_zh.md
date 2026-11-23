# r33：PCL 查找 Boost 失败的修复

日志中的 Eigen 3.4.0 已找到。实际失败是 PCL 的 `find_package(Boost)` 未定位目标 sysroot 多架构目录里的五个 Boost 库。

用户已在 Ubuntu 容器确认 `usr_x5/lib/aarch64-linux-gnu` 中 `system`、`filesystem`、`date_time`、`iostreams`、`serialization` 的 Boost 1.74.0 库及 `.so` 链接均为有效 AArch64 ELF，因此无需安装 Boost。

本修复只调整构建：在 PCL 配置之前读取目标 Boost 版本，检查对应五个真实 AArch64 动态库与 CMake 组件配置，生成构建目录内的 `x5_boost.cmake`；显式指定目标 `Boost_DIR`、组件目录、头文件与库目录，采用动态 Release 库并允许 FindBoost 转入 BoostConfig。保留上一版共享库依赖目录传播修复。检测、分割、点云、视差、回充、DAG 和数值源码不改动。

## Ubuntu 宿主机安装补丁并编译

下载补丁及 `.sha256` 到 `/home/st/ini_code`。`SRC` 指向你实际编译的 r33 源码目录，以下沿用上一条指令创建的位置：

```bash
cd /home/st/ini_code
sha256sum -c perception-x5-r33-boost-fix-v1.tar.gz.sha256
tar -xzf perception-x5-r33-boost-fix-v1.tar.gz

SRC=/home/st/ini_code/r33_linkfix_source/perception-x5-full-shared-r12-v33

python3 perception-x5-r33-boost-fix-v1/install.py --source-root "$SRC"
cd "$SRC"
sudo bash tools/build_in_container.sh \
  --container obs-v004-x5-build \
  --sysroot /work/obs_v004_x5/sysroot_docker \
  --out /home/st/ini_code/r33_shared_release
```

如果原源码直接位于 `/home/st/ini_code/perception-x5-full-shared-r12-v33`，只需将 `SRC` 改成该路径。不要把 `SRC` 指向板端 SDK。补丁验证现有校验清单、保存备份、更新构建文件与清单，兼容原 r33 源码和上一版链接修复源码。

脚本仍复用已有容器，并创建新的容器构建目录；无需进入 Docker、修改第三方 PCL/Boost 文件、手工清除 CMakeCache 或安装宿主机 x86 Boost。

预期出现：

```text
Target Boost 1.74.0: /work/obs_v004_x5/sysroot_docker/usr_x5/lib/aarch64-linux-gnu/cmake/Boost-1.74.0
Verified 5 genuine AArch64 Boost shared libraries; PCL will use their target CMake configurations.
```

生成的 SDK 名称和 X5 运行指令不变，见 `README_SHARED_zh.md`。

## 真正缺少开发配置文件时

本修复同时需要同套 sysroot 中的 `Boost-1.74.0/BoostConfig.cmake`、`boost_headers-1.74.0` 和五个组件的 CMake 配置。若实际容器缺少这些文件，检查器会打印确切路径并停止；此时需补齐同版本 ARM Boost 开发配置，不能用 Ubuntu 宿主的 x86 库替代。用户提供的 `file` 输出只验证了库，未验证 CMake 配置文件是否存在。

## 验证范围

使用本地真实 AArch64 Boost 1.74.0 文件检查架构、版本和路径；通过真实 CMake FindBoost/BoostConfig 配置读取检查各组件的 IMPORTED_LOCATION_RELEASE，并验证 UTF-8/LF、源码保留清单及补丁交付。配置验证不执行 ARM 链接，也不等同于 Ubuntu 容器完整构建或 X5 运行成功。
