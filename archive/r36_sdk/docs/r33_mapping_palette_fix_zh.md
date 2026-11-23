# r33：检测点云配色声明修复

本次确定的编译错误是 `mapping/src/detection_labels.cpp` 使用了未声明的 `kDetPalette`。原项目在 `dnn_runtime.cpp` 的匿名命名空间中定义这张 12 色表；将检测点云匹配方法拆到独立 `.cpp` 后，原定义不再处于同一编译单元。

修复恢复原项目的 12 色定义和类别取模规则。原有 `dnn_palette.hpp` 包含另一套 16 色定义，不能直接替换。检测与分割输入、模型、前后处理，以及视差、回充、DAG/Task/Feature 调度保持原方案。

新包累积保留之前共享库链接、Boost、FLANN/PCL/LZ4 的构建修复。安装程序验证已有源码和补丁清单、保存备份，再更新指定文件和最终字节校验清单。

## Ubuntu 宿主机安装与编译

将修复包和同名 `.sha256` 下载到 `/home/st/ini_code`。以下 `SRC` 沿用之前的源码路径；如实际路径不同，只修改这一行。

```bash
cd /home/st/ini_code
sha256sum -c perception-x5-r33-mapping-palette-fix-v1.tar.gz.sha256
tar -xzf perception-x5-r33-mapping-palette-fix-v1.tar.gz

SRC=/home/st/ini_code/r33_linkfix_source/perception-x5-full-shared-r12-v33
python3 perception-x5-r33-mapping-palette-fix-v1/install.py --source-root "$SRC"

cd "$SRC"
sudo bash tools/build_in_container.sh \
  --container obs-v004-x5-build \
  --sysroot /work/obs_v004_x5/sysroot_docker \
  --out /home/st/ini_code/r33_shared_release
```

脚本复用现有容器，创建新的构建目录。请修改 Ubuntu 宿主机上的源码，不要只修改上次构建的 `/tmp/.../source` 临时副本。

成功后输出文件为：

```text
/home/st/ini_code/r33_shared_release/perception-sdk-x5-r12-v33-full-shared.tar.gz
/home/st/ini_code/r33_shared_release/perception-sdk-x5-r12-v33-full-shared.tar.gz.sha256
```

## X5 有限帧测试

将上述两个文件复制到 X5 的 `/userdata/ai_infer_r12_v33`。已有同名 SDK 目录时先将其改名备份，再解压新包。

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

相机需由本测试独占。预热 20 帧、测量 120 帧后关闭退出。默认 `map_and_recharge` 使用同帧 GDC，请保留默认相机选项。

## 验证范围

本地核对恢复的 12×3 个 RGB 字节与原项目一致，并验证原始数值源码、源码溯源、修复安装、压缩包及 UTF-8/LF 最终字节清单。具体检查证据见 `mapping/validation/r33_mapping_palette_fix.json`。完整 Ubuntu AArch64 编译与 X5 运行仍以用户实际执行结果为准。

另外对 8 个映射编译单元执行了基于真实 PCL/Eigen/OpenCV 头的 Clang 作用域检查，未发现其他确定的拆分符号遗漏，证据见 `validation/r33_mapping_palette_syntax.json`。该检查禁用了 SIMD/Neon 并延迟解析第三方模板，仅用于发现源码作用域错误，不替代 GCC11 ARM 编译、目标链接或运行验证。
