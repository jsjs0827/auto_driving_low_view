# r33：FLANN / PCL 目标库查找修复

Boost 配置已经通过。此次失败是 PCL 的 `search` 必需依赖 FLANN 没有被找到。用户已确认容器中的 `libflann_cpp.so.1.9.1`、`.so.1.9` 和 `.so` 都是有效 AArch64 ELF。

PCL 的 FindFLANN 在这套非标准 `/usr_x5` sysroot 中未定位多架构库目录。原 FLANN pkg-config 配置还可能保留 `/usr` 前缀。新增构建检查明确指定目标 FLANN 头文件、版本化动态库和 LZ4 依赖，并检查当前 PCL 组件的完整闭包。

当前 7 个请求模块展开为 11 个组件：9 个共享库，以及 header-only 的 `2d`、`geometry`。它们的必需外部依赖为 Eigen、Boost、FLANN。VTK 是可选依赖，Qhull 不在当前闭包；保留现有缺少 VTK 时的行为，不新增 Qt/VTK 链。

修复保留前两轮共享库链接和 Boost 配置修复。检测、分割、视差、回充、点云的数值源码与 DAG 调度不变。

## Ubuntu 宿主机安装和编译

下载补丁及 `.sha256` 到 `/home/st/ini_code`。以下 `SRC` 沿用前面交叉编译指令的源码位置：

```bash
cd /home/st/ini_code
sha256sum -c perception-x5-r33-pcl-deps-fix-v1.tar.gz.sha256
tar -xzf perception-x5-r33-pcl-deps-fix-v1.tar.gz

SRC=/home/st/ini_code/r33_linkfix_source/perception-x5-full-shared-r12-v33

python3 perception-x5-r33-pcl-deps-fix-v1/install.py --source-root "$SRC"
cd "$SRC"
sudo bash tools/build_in_container.sh \
  --container obs-v004-x5-build \
  --sysroot /work/obs_v004_x5/sysroot_docker \
  --out /home/st/ini_code/r33_shared_release
```

如果源码直接在 `/home/st/ini_code/perception-x5-full-shared-r12-v33`，只修改 `SRC`。补丁兼容原 r33、链接修复版和 Boost 修复版；验证旧清单、保存备份后更新指定构建文件及最终字节清单。

仍复用原 Docker 容器，每次创建新的构建目录。成功后 SDK 文件名和 X5 启动指令不变，见 `README_SHARED_zh.md`。

## 本次构建修复

- `tools/check_x5_pcl.py` 读取实际 PCL 配置的组件闭包，核验对应版本库的 AArch64 架构、SONAME 和 SHA256，生成构建目录内的 `x5_pcl.cmake`。
- 指定真实 `FLANN_LIBRARY_SHARED`，并通过 `FLANN::FLANN` 显式传递真实目标 LZ4 库，覆盖 FLANN 头文件直接调用 LZ4 的链接需求。
- 交叉编译进程隔离宿主机 pkg-config 路径。所需目标头文件、库和 CMake 包路径由预加载配置提供；不读入 x86 库配置。
- PCL 库直接指向同版本真实载荷，不依赖 Windows 拷贝可能损坏的文本别名；不修改第三方 sysroot 文件。

VTK 的两条可选功能警告可能仍然出现。判定配置通过应查看后续 `Configuring done` / `Generating done`，而非要求没有任何 warning。

## 验证范围

本地使用真实 AArch64 PCL 1.12.1、FLANN 1.9.1 和 LZ4 ELF 检查所需库；用真实 PCLConfig/FindFLANN 配置探针验证查找结果和 LZ4 传递依赖，执行源码保留、UTF-8/LF、压缩包与补丁安装校验。仅配置检查，不等同于用户 Ubuntu 容器完整 ARM 编译或 X5 运行通过。
