# r12-v33：原业务算法共享库

本版本把本地 `charing-main` 回充算法和 `point-cloud-processing-main` 的视差消费、点云、自动标定、语义标注、Patchwork++、栅格融合接入现有 DAG / Task / Feature。输出 `pcl`、`ai`、`fusion` 三张俯视图、原始栅格数组、可选点云与回充姿态。

检测仍用原模型和 864×480 letterbox，分割仍用原模型、832×480 stretch 和已优化的原 CPU 后处理；保留精确缩放与直接输入写入。单帧同步处理，无跨帧双缓冲。视差来自原相机 SDK 内部的 BPU，本库接收同帧 Q8.8 视差，也提供真实相机 SDK 获取示例。

公开入口：`shared/include/perception_full_sdk.h`。调用示例：`shared/examples/full_demo.cpp`；安装后在 `examples/full/full_demo.cpp`。旧 `perception_sdk.h` 推理 ABI 保留。完整业务链在 `full/graph.cpp`，每帧返回 Task 状态、耗时和完整 uint32 错误码、原生返回码、Feature、Task 与帧信息。

本机真实 OpenCV / ONNX Runtime 的编译、回充原接口对照和公开 ABI 测试已通过。Windows 本机缺少 PCL，完整地图模块、X5 SO 和实际相机端到端等价尚未完成目标机验收。此源码包不包含伪造的 ARM SO。Ubuntu 下面的命令生成真正的 `.so` 和 X5 SDK。

## Ubuntu 宿主交叉编译

将源码压缩包和同名 `.sha256` 放在 `/home/st/ini_code` 后：

```bash
cd /home/st/ini_code
sha256sum -c perception-x5-full-shared-r12-v33.tar.gz.sha256
tar -xzf perception-x5-full-shared-r12-v33.tar.gz
cd perception-x5-full-shared-r12-v33
sudo bash tools/build_in_container.sh \
  --container obs-v004-x5-build \
  --sysroot /work/obs_v004_x5/sysroot_docker \
  --out /home/st/ini_code/r33_shared_release
```

沿用已有容器；不拉镜像、不进入交互 Docker、不编译 ONNX、不依赖受保护的 ROS/daye SHM 头文件或旧回充二进制。需要 sysroot 中真实的 AArch64 PCL、Eigen、动态 OpenCV 和 Horizon BPU SDK；缺失会在配置阶段明确报错。

成功输出：

```text
/home/st/ini_code/r33_shared_release/perception-sdk-x5-r12-v33-full-shared.tar.gz
/home/st/ini_code/r33_shared_release/perception-sdk-x5-r12-v33-full-shared.tar.gz.sha256
```

SDK 中包含 `lib/libperception_sdk.so`、两个公开头文件、`bin/perception_full_demo`、调用源码、配置和板端检查程序。仅打包实际链接的数学运行库及其依赖，保留系统/Horizon/相机运行库由板端提供，避免覆盖相机媒体栈。版本与依赖记录在 `math_runtime_provenance.json`；所有文件和 tar 都校验最终字节。

## X5 首次检查与在线测试

先停止占用相机的原程序。把生成的 SDK 两个文件复制到 `/userdata/ai_infer_r12_v33`：

```bash
cd /userdata/ai_infer_r12_v33
sha256sum -c perception-sdk-x5-r12-v33-full-shared.tar.gz.sha256
tar -xzf perception-sdk-x5-r12-v33-full-shared.tar.gz
cd perception-sdk-x5-r12-v33-full-shared
bash tools/check_full_x5.sh
bash tools/run_full_x5.sh \
  --camera \
  --sdk /app/data/metoak/lib/libMoGeneralSDK.so \
  --camera-config /app/data/metoak/demo/config/x5_camera.cfg \
  --det-model /userdata/ai_infer/data/mmyolo0828.bin \
  --seg-model /userdata/ai_infer/data/pidnet_0825_3classes.bin \
  --frames 120 --warmup 20 \
  --out /userdata/r33_results/full-$(date +%Y%m%d-%H%M%S)
```

默认同帧 `640×544 NV12` RGBD 左目、16 位 Q8.8 视差；回充按原配置使用同帧 `1280×1088` GDC 左目。SDK 帧身份与时间戳严格配对，不取相近时间戳代替同帧。默认内参来自本次原相机配置，其他设备须传入其实际 `--fx/--fy/--cx/--cy/--bxf`；bxf 为 pixel-mm。

`frames.csv` / `summary.json` 记录实际处理 FPS、CPU、RSS、CMA 和分支耗时。`last/` 保存 `pcl.png`、`ai.png`、`fusion.png`、`overlay.png`、原始地图数组、视差、回充诊断、Task 轨迹与完成标记。回充原配置 `publish_overlay=false` 时不生成回充图；原配置 `label_fusion.fuse_into_grid=false` 时 AI 地图为未知，融合地图沿用几何结果。本包保留这两个原始开关，不擅自开启。

## 配置与其他程序调用

入口 `config/full.json` 引用原算法配置。不要修改已校验 SDK 包内文件；生成包外的用户 JSON，路径会自动展开，之后可直接编辑：

```bash
python3 tools/make_full_config.py \
  --output /userdata/r33_user_config.json \
  --mode map_and_recharge \
  --det-model /userdata/ai_infer/data/mmyolo0828.bin \
  --seg-model /userdata/ai_infer/data/pidnet_0825_3classes.bin
bash tools/run_full_x5.sh --config /userdata/r33_user_config.json \
  --camera --frames 120 --warmup 20 \
  --out /userdata/r33_results/custom-$(date +%Y%m%d-%H%M%S)
```

地图/外参/回充高级参数分别通过 `--original-config`、`--extrinsics`、`--recharge-config` 引用包外配置副本。仅建图可选择 `--mode map`，并在相机 Demo 上加 `--camera-no-hires`，无需获取回充 GDC。

宿主程序负责 ROS/SHM 或其他图片来源，把同帧图像、视差和内参填入 `psdk_full_frame`。调用 `create → process → result_get_view → result_release → destroy`；结果持有自己的内存，销毁引擎后仍有效。publication 回调可接回原程序发布流程，所有回调禁止重入完整 SDK。相机控制与原 ROS wire 协议仍由宿主负责，本库不伪造缺失的 middleware ABI。

在 X5 原生编译纯公共接口示例：

```bash
cmake -S examples/full -B /userdata/r33_consumer_build -DPSDK_ROOT="$PWD"
cmake --build /userdata/r33_consumer_build -j2
```

独立示例以调用方提供的同帧文件输入；带相机 SDK 接入的完整 demo 已随 SDK 编译。详细功能、API、单位、验收范围见 `docs/r12_v33_full_zh.md` 和 `FUNCTIONAL_SCOPE_R33.json`。
