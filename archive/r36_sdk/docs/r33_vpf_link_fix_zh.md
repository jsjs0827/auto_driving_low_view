# r33：X5 VPF 媒体库链接修复

此次失败在生成 `libperception_sdk.so.1.0.33` 的链接阶段，`hardware_routes` 引用的 14 个 `hbn_vflow_*` / `hbn_vnode_*` 函数未能解析。

目标 sysroot 存在两种同名的 `libvpf`：

| 路径（相对 `usr_x5`） | SONAME | 实际用途 |
| --- | --- | --- |
| `hobot/lib/libvpf.so.1.0.0` | `libvpf.so.1` | X5 媒体处理，导出所需的 14 个 `hbn_*` 函数 |
| `lib/libvpf.so.4.1` | `libvpf.so.4.1` | 地理矢量数据处理，未导出上述函数 |

原查找仅使用 `HINTS`，CMake 仍会先搜索 `CMAKE_PREFIX_PATH` 对应的标准 `lib` 目录。修复将媒体库查找限定到目标 BSP 位置，并防止旧缓存把错误的同名库带回链接。

该修复只调整构建依赖。默认精确 CPU 前处理、原始检测 letterbox、分割前后处理、原模型及单帧 DAG/Task/Feature 路径保持原配置。新包累积保留之前的链接、Boost、FLANN/PCL/LZ4 以及检测点云配色修复。

## Ubuntu 宿主机安装与交叉编译

将修复包和 `.sha256` 下载到 `/home/st/ini_code`。`SRC` 设置为你此前使用的 Ubuntu 宿主机源码目录；不要指向容器中的临时 `/tmp/.../source`。

```bash
cd /home/st/ini_code
sha256sum -c perception-x5-r33-vpf-link-fix-v1.tar.gz.sha256
tar -xzf perception-x5-r33-vpf-link-fix-v1.tar.gz

SRC=/home/st/ini_code/r33_linkfix_source/perception-x5-full-shared-r12-v33
python3 perception-x5-r33-vpf-link-fix-v1/install.py --source-root "$SRC"

cd "$SRC"
sudo bash tools/build_in_container.sh \
  --container obs-v004-x5-build \
  --sysroot /work/obs_v004_x5/sysroot_docker \
  --out /home/st/ini_code/r33_shared_release
```

安装程序先验证现有清单和源码版本、保存备份，再更新指定源码与最终字节清单。补丁支持此前原始、链接修复、Boost 修复、PCL 修复和配色修复的 r33 源码包。

脚本复用现有 Docker 容器，创建新的构建目录。

配置日志应出现以下检查结果（省略了路径前缀）：

```text
X5 media VPF: .../usr_x5/hobot/lib/libvpf.so.1.0.0 (libvpf.so.1; 14 required hbn exports)
```

构建成功后的文件为：

```text
/home/st/ini_code/r33_shared_release/perception-sdk-x5-r12-v33-full-shared.tar.gz
/home/st/ini_code/r33_shared_release/perception-sdk-x5-r12-v33-full-shared.tar.gz.sha256
```

## X5 运行

将上述两个产物复制到 `/userdata/ai_infer_r12_v33`。已有同名解压目录时先改名备份，然后执行：

```bash
cd /userdata/ai_infer_r12_v33
sha256sum -c perception-sdk-x5-r12-v33-full-shared.tar.gz.sha256
tar -xzf perception-sdk-x5-r12-v33-full-shared.tar.gz
cd perception-sdk-x5-r12-v33-full-shared
bash tools/check_full_x5.sh

R="/userdata/r33_results/full-$(date +%Y%m%d-%H%M%S)"
bash tools/run_full_x5.sh \
  --camera \
  --sdk /app/data/metoak/lib/libMoGeneralSDK.so \
  --camera-config /app/data/metoak/demo/config/x5_camera.cfg \
  --det-model /userdata/ai_infer/data/mmyolo0828.bin \
  --seg-model /userdata/ai_infer/data/pidnet_0825_3classes.bin \
  --frames 120 --warmup 20 --out "$R"
```

相机由本测试独占。预热 20 帧、测量 120 帧后关闭退出。默认 `map_and_recharge` 保留同帧 GDC 输入。

## 验证范围

验证证据见 `validation/r33_vpf_link_fix.json`：核对真实 AArch64 库的 SONAME 和动态符号，并检查 CMake 查找与生成的链接依赖。另验证原始计算源码、修复安装、最终字节哈希及 UTF-8/LF。完整 GCC11 ARM 链接与 X5 运行以 Ubuntu 容器及板端实际执行为准。
