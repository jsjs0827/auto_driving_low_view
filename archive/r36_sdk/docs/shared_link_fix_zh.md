# X5 共享库调用示例的链接修复 v1

本次报错发生在 `perception_sdk_demo` 的最终链接。`original_runner` 的 PUBLIC `-rpath-link` 通过 PRIVATE 依赖进入共享库，但不会继续传播到只链接共享库的 demo。链接器因此找不到 `libvpf`、`libdnn` 的间接依赖 `libalog.so.1`、`libgdcbin.so`、`libcnn_intf.so.1`。

修复适用于 r32 和 r33：共享库公开 BUILD_INTERFACE 链接选项，供所有构建树内消费者使用；补齐 `sysroot/usr/lib` 等目录。外部调用示例增加 `PSDK_LINK_DEPENDENCY_DIRS`，可在单独交叉编译时指定相同依赖目录。保持原运行时 RPATH、原模型、数学实现、DAG 和单帧调度；不打包替换相机或 Horizon 运行库，不放宽未定义符号检查。

## 已有源码安装小补丁

把补丁 tar.gz 和 .sha256 复制到 Ubuntu 宿主机 `/home/st/ini_code`。以下命令在宿主机执行，`--source-root` 指向此次编译失败的源码根目录。

```bash
cd /home/st/ini_code
sha256sum -c perception-x5-shared-link-fix-v1.tar.gz.sha256
tar -xzf perception-x5-shared-link-fix-v1.tar.gz
python3 perception-x5-shared-link-fix-v1/install.py \
  --source-root /home/st/ini_code/perception-x5-shared-sdk-r12-v32
cd /home/st/ini_code/perception-x5-shared-sdk-r12-v32
sudo bash tools/build_in_container.sh \
  --container obs-v004-x5-build \
  --sysroot /work/obs_v004_x5/sysroot_docker \
  --out /home/st/ini_code/r32_shared_release
```

如果编译的是 r33，上面两处源码目录改成 `perception-x5-full-shared-r12-v33`，输出目录改成 `/home/st/ini_code/r33_shared_release`。安装器自动识别 r32/r33，先验证旧文件与清单，保存备份，再更新指定构建文件及最终字节校验清单。不要指向 X5 上解压后的 SDK。

沿用已有 `obs-v004-x5-build` 容器，不拉镜像、不进入交互 shell。封装脚本每次使用新的容器临时构建目录，成功后仍输出原 SDK 文件名：

```text
r32_shared_release/perception-sdk-x5-r12-v32-shared.tar.gz
r32_shared_release/perception-sdk-x5-r12-v32-shared.tar.gz.sha256
r33_shared_release/perception-sdk-x5-r12-v33-full-shared.tar.gz
r33_shared_release/perception-sdk-x5-r12-v33-full-shared.tar.gz.sha256
```

## 使用完整修复源码包

也可将对应完整源码包解压到新的目录，直接运行同一条 `tools/build_in_container.sh` 命令，不再安装补丁。不应把新 tar 覆盖解压到正在使用的旧源码目录。

## 若仍报缺少相同三个库

在 Ubuntu 宿主机执行下面的只读检查。这检查的是容器里的实际 sysroot，不是 Windows 上的拷贝：

```bash
sudo docker exec obs-v004-x5-build bash -lc '
for name in libalog.so.1 libgdcbin.so libcnn_intf.so.1; do
  p=/work/obs_v004_x5/sysroot_docker/usr_x5/hobot/lib/$name
  printf "\n%s\n" "$p"
  ls -l -- "$p"
  file -L -- "$p"
  readelf -h -- "$p"
done
'
```

期望为有效的 AArch64 ELF，或能正确解析到该 ELF 的 Linux 软链接。若只是包含目标文件名的几字节普通文本，是软链接在 Windows 拷贝时丢失；需要从同版本 X5 SDK/板端补齐真实依赖及软链接。不要拿其他平台、旧 VPF ABI 或任意版本同名库替代。该情形需要单独修复 sysroot，增加搜索目录不会把文本变成库。

## 验证范围

已检查真实本地 X5 ELF 的依赖关系，并执行 CMake 生成链接参数检查、Windows 原生构建和契约测试、UTF-8/LF 与最终压缩包字节校验。Windows 本机不能执行用户的 Ubuntu Docker 交叉编译或 X5 相机测试；这些检查不等同于 ARM 构建和板端验收。
