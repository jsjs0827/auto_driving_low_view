# 模型统一管理

所有模型权重集中在本目录。当前包已经包含以下 6 个实际权重（含 v4 旧 Stereo 保留权重）；`manifest.json` 记录相对路径、大小和 SHA256，便于确认复制、部署和替换的文件。清单只负责登记，运行时选择模型的是功能配置的 `model` 字段。

```text
项目根/                              主 pipeline.json 所在目录
├── pipeline.json
├── features/
│   ├── detection/config.json         model → models/x5/mmyolo0828.bin
│   ├── segmentation/config.json      model → models/x5/pidnet_0825_3classes.bin
│   ├── stereo/config.json            model → models/x5/r34_...352x640...bin
│   └── inference/cpu_profile.json    CPU 输入/输出和预后处理契约
└── models/
    ├── manifest.json
    ├── x5/
    │   ├── mmyolo0828.bin
    │   ├── pidnet_0825_3classes.bin
    │   └── r34_tail_pointwise_shuffle2_grouped_grid_352x640_nv12_mixed.bin
    └── cpu/
        ├── case2_20260821_V2.1_epoch_160_dy.onnx
        └── train_20260924_3class_aug_flagstone.onnx
```

## 路径规则

`features/*/config.json` 的 `model` 虽然写在功能目录中，**相对路径统一以主 `pipeline.json` 所在目录为基准**，不相对功能 JSON，也不相对终端工作目录。复制或部署时一起保留 `pipeline.json`、`features/` 和 `models/` 的结构即可。模型配置不使用模型路径环境变量。

SDK 原始 profile 内部路径遵循另一条已有规则：相对该 profile 文件所在目录。源树 `scheduling/config/profiles/` 中因此写 `../../../models/...`；SDK 安装时生成适合安装目录的 `../models/...`，并复制所需模型目录到 SDK 前缀。用户日常修改功能配置即可，无需同步修改这些 SDK 默认值。原生配置器会把本次运行的最终模型路径写入 `effective_full.json`、`inference_cpu.json` 或 `parameters.yaml`，运行快照中出现绝对路径是正常的。

## X5 默认模型与替换

v6 默认 Stereo 使用 r34_crop_stage2_i12_cheb5_max1_tail_int16_352x640_20261012.bin；旧 Stereo 权重仅保留用于回归。当前默认配置直接使用 `models/x5/` 内的检测、分割和 Stereo 模型。进入项目目录并设置匹配的 SDK/ROS 环境后即可运行，不需要导出检测、分割或 Stereo 模型环境变量：

```bash
export SDK_ROOT="$PWD/install/backend_sdk_x5"
export INSTALL_ROOT="$PWD/install/perception_ros_so_demo_x5"
export ROS_SETUP=/app/humble/setup.bash
bash tools/run_pipeline.sh --config "$PWD/pipeline.json" \
  --prepare-only --run-dir /userdata/stereo_results/model_check_001
```

例如替换 Stereo，将新 BIN 放入 `models/x5/`，修改 `features/stereo/config.json` 的 `model`：

```json
{
  "model": "models/x5/your_stereo.bin",
  "image_transform": "crop",
  "input_width": 640,
  "input_height": 352
}
```

当前模型契约是 NCHW `[1,3,352,640]`、NV12 双目输入。替换模型时同时核对尺寸、dtype、量化参数、输出布局及视差含义；仅修改文件名不能使任意模型兼容。尺寸改变时同步修改 `input_width/input_height`，在线provider可同时设为 `null` 由模型元数据读取；同ROI录制对比必须显式声明尺寸，以便原始Q8组不加载新模型也能确定裁剪区域。检测/分割模型还须匹配各自的类别数、输出和预后处理契约。

修改配置后重启进程。使用新文件名能保留原权重用于对比；替换权重后相应更新清单及发布 SHA256，再进行同数据回归。

## x86 CPU 检测/分割

在 `features/inference/config.json` 设置：

```json
{
  "backend": "cpu",
  "original_profile": null,
  "cpu_profile": "features/inference/cpu_profile.json"
}
```

同时将 `features/detection/config.json` 的 `model` 改为：

```json
"model": "models/cpu/case2_20260821_V2.1_epoch_160_dy.onnx"
```

将 `features/segmentation/config.json` 的 `model` 改为：

```json
"model": "models/cpu/train_20260924_3class_aug_flagstone.onnx"
```

保留这两个配置中的其他字段。`cpu_profile.json` 已包含本包 ONNX 的真实输入/输出名称、预处理与解码契约；其中模型默认路径相对 profile 自身，是 `../../models/cpu/...`。上述功能模型字段会明确覆盖这些默认值。

CPU 仍需启用 ONNX Runtime 后端并使用重新编译的 CPU SDK。本版没有新增 CPU Stereo 推理器：x86 上检测/分割可用离线左图；点云对比中的原方法使用录制的 Q8.8，新 Stereo 仍需 Horizon/X5 环境。图中没有启用某个模型通路时，不要求加载对应权重。

`scheduling/config/profiles/cpu_exact.json`、`cpu_found_models_trial.json` 保留给底层 worker 调试，输入占位路径是 `../../../data/source.nv12`，需要自行提供该文件；日常运行优先使用根 `pipeline.json` 配置 offline/recording 输入。

## v6验证

见 [本轮验证记录](../docs/V6_验证记录.md)：C++工具6/6、codec/NV12两项、真实ONNX集成三种情况及AArch64编译。当前没有本轮X5 crop模型实测。
