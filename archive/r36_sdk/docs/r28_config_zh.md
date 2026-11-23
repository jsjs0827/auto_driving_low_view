# r28：一个 JSON 配置入口

用户填写简写 JSON，程序冻结为原框架读取的完整 JSON，再启动实际执行程序。检测 letterbox、分割直接缩放、原 BIN、精确输入写入和原数值后处理保持原约定。默认单帧、无双缓冲；配置工具不修改相机 SDK、不编译模型、不切换视差模式。

## 两个实际运行目标

| `target` | 实际执行路径 | 可配置内容 |
|---|---|---|
| `exact` | `bin/dag_flow_worker` → 原 Engine / DAG / Task / Feature | 已验证的检测与分割路径、冻结输入/相机输入、PNG 输出 Feature |
| `product` | 原 daye / ROS2 launcher → 可选编译的 `lib/ai_node/libpercep_demo.so` | 完整原业务、地面提取、语义融合、自动标定、回充、地图记录与原 JPG / ROS 可视化 |

`exact` 不冒充完整业务。`product` 必须有真实插件和原业务依赖；本包没有发明一个独立的 `product_flow_worker`。完整业务生命周期、地图/回充模式仍由原 launcher 和原命令接口控制。

完整业务输入继续由原相机节点及 SHM 提供端交付。对于 `product`，本工具只展开并透传 `camera.sdk/config` 路径、检查依赖文件；填写它们不会启动新相机、重新配置原相机节点或切换 SDK 取帧策略。

## 首次运行精确检测/分割

SDK 内的模板参与完整性校验，请复制到 SDK 外再编辑，避免修改已校验的发行文件。

```bash
cd /userdata/ai_infer_r12_v28/perception-sdk-x5-r12-v28-product-dag
cp profiles/product_exact.json /userdata/r28_exact.json
```

修改 `/userdata/r28_exact.json` 中相机和模型路径。默认保留之前的原 BIN 路径。完整例子见模板；最小文件也可以这样写：

```json
{
  "schema_version": 1,
  "run": {
    "out": "/userdata/r12_v28_res/exact-01",
    "frames": 300
  },
  "visualization": {
    "enabled": true,
    "every_n": 10,
    "layers": ["raw", "detection", "segmentation", "overlay"]
  }
}
```

先预览，再运行：

```bash
bash tools/run_product_x5.sh --config /userdata/r28_exact.json --dry-run
bash tools/run_product_x5.sh --config /userdata/r28_exact.json --run
```

预览不创建目录、不打开相机、不加载模型。运行前检查实际二进制、原模型和相机 SDK/config；缺少文件返回 JSON 错误，不悄悄替换 BIN 或运行其他分支。`run.out` 必须为新目录，每次测试修改末尾编号。

性能测试时将 `visualization.enabled` 设为 `false`；可视化测试单独开启。PNG 编码和磁盘写入会增加实际总耗时，报告分别记录输出工作，不把输出时间当成 BPU 推理时间。

`layers` 还支持 `disparity`。该层仅限实时 RGBD 输入：在 SDK 释放帧前复制已有的同帧视差，不额外触发视差 BPU 推理、不切换模式。复制开销计入取帧 `frame_copy_ms`；三路 ID、时间戳或布局不完整时明确报错，避免绘制错帧。默认不复制视差。`raw/list` 输入没有配对视差，简写配置拒绝该层；离线可用 `tools/view_results.py` 读取具有可信同帧元数据的已保存视差。`preprocess` 模式没有推理结果，不能同时启用可视化。

## 字段说明

| 字段 | 默认值或限制 |
|---|---|
| `schema_version` | 必填，整数 `1` |
| `target` | `exact`；完整业务填 `product` |
| `camera.sdk` / `camera.config` | `/app/data/metoak/lib/libMoGeneralSDK.so` / `/app/data/metoak/demo/config/x5_camera.cfg` |
| `models.detection` / `models.segmentation` | `/userdata/ai_infer/data/mmyolo0828.bin` / `/userdata/ai_infer/data/pidnet_0825_3classes.bin` |
| `run.out` | 输出目录；生产配置建议填写绝对路径 |
| `run.frames` / `run.warmup` | `exact` 专用，默认 `300` / `20`；范围 `1..100000` / `0..10000` |
| `run.mode` | `exact` 专用，默认 `optimized`；也支持 `verify`、`baseline`、`preprocess` |
| `input.kind` / `input.path` | `exact` 专用；默认 `camera`；`raw` 或 `list` 必须提供路径 |
| `features.detection` / `features.segmentation` | 本包要求都启用，不能关闭其中一个后继续声称同等工作负载 |

相对路径统一相对于简写 JSON 所在目录解析。原始 `raw` 输入固定为紧凑的 **640×544 NV12**，不接受把 1280×1088、RGB 或其他布局静默当成原输入。`list` 使用现有原输入 TSV 格式。

不需要在简写 JSON 里写 Task 名、Feature 名、并行节点或每个模型的数值参数。预处理固定 `compact_direct`，检测/分割在同一帧并行；一帧完成后再处理下一帧。相机参数 `disparity-mode=0` 的含义为不调用 SDK 视差模式设置 API，不代表关闭 SDK 内部原有的视差处理。

## 可视化配置

`exact` 使用 PNG 输出 Feature：

```json
"visualization": {
  "enabled": true,
  "every_n": 10,
  "layers": ["overlay"],
  "alpha": 0.35,
  "dir": "/userdata/r12_v28_res/exact-01/visualization"
}
```

`layers` 可选 `raw`、`detection`、`segmentation`、`overlay`、`disparity`，不得重复。视差层需要当前帧确实有视差数据；工具不会为了画图修改 SDK 模式。只有上述真实实现的 PNG 参数，不能填写未实现的 JPEG `quality`。

`product` 使用原业务的 JPG 保存和 ROS overlay 发布：

```json
"visualization": {
  "enabled": true,
  "publish": false,
  "every_n": 10,
  "max_count": 200,
  "quality": 70,
  "blend_seg": true,
  "alpha": 0.35,
  "dir": "/userdata/r12_v28_res/product-01/overlay"
}
```

其中 `enabled` 对应原 `dnn.save_overlay.enabled`，`publish` 对应 `dnn.publish_overlay`。完整业务保留原保存目录初始化行为，使用新的会话目录。`layers` 仅属于轻量 PNG 路径，完整业务填写该字段会明确报错。

## 完整业务配置与启动

```bash
cp profiles/product_full.json /userdata/r28_product.json
bash tools/run_product_x5.sh --config /userdata/r28_product.json
```

默认仅生成兼容原业务的 `start_config.json`，不会启动未知 launcher。配置工具将外参和回充配置复制到冻结目录，并将真实业务图、精确 profiles 路径写入 `product.dag_config` / `product.exact_profiles`。

`product` 不支持轻量 worker 的有限帧运行参数。显式填写 `run.frames`、`run.warmup`、`run.mode` 或 `input` 会返回 `2001 CONFIG_INVALID`，不会接受后忽略。完整业务按原 launcher 的启动、模式命令与停止机制运行，不能据此认为它会在 300 帧后自动停止。

完整业务的逐 Task 耗时、进程 CPU/RSS 与全局 CMA 指标写入 `product.metrics_dir`，默认 `run.out/metrics`；可在 `product` 对象里显式指定该目录。准备配置不运行任何业务，也不产生实际性能结果。

最常用的其他功能开关：

```json
"features": {
  "ground": true,
  "semantic_fusion": true,
  "auto_calibration": false,
  "recharge": true
},
"recording": {
  "logs": false,
  "maps": false,
  "data": false
}
```

对应原 `patchwork.enabled`、`label_fusion.enabled`、`auto_cali`、回充配置顶层 `enabled`，以及 `debug.save_log/save_map/save_data`。点云和栅格地图是原地图模式的必要链路，不能用简写开关删掉其中一步。原业务需要的高级参数继续在原 `start_config.json` 等完整配置中填写，通过 `product.base_config`、`product.extrinsics`、`product.recharge_config` 引入；生成器保留其他业务参数，并拒绝改变已冻结的检测/分割数值流程。

确认原 launcher 的真实命令后才填写 `launch.argv`，它必须是参数数组，工具使用 `subprocess` 直接启动，不经过 shell。不提供 launch 时 `--run` 明确拒绝。不要把 `.so` 当成独立程序执行。

启动时工具设置 `PERCEPTION_PRODUCT_CONFIG` 指向生成的完整配置，供新插件读取；可选的整参数占位符 `{product_config}`、`{plugin}` 只在真实 launcher 需要这些参数时使用。插件的加载路径仍需按原 daye / ROS2 launcher 的机制配置，本工具不会替换 `/app/daye` 里的原插件或伪造外部依赖。

## 冻结记录与错误返回

默认冻结目录为 `run.out + ".prepared"`，也可以传 `--prepare-dir NEW_DIR`。文件包括：

- `effective_config.json`：补齐默认值并解析路径后的简写配置。
- `dag_expanded.json` + `original.cfg`：轻量目标的实际 DAG 和原数值 profile；原 profile 按字节复制。
- `start_config.json`、`product_dag.json`、`extrinsics.json`、`recharge_mark.json`：完整业务目标的实际输入配置。
- `launch_plan.json`：执行参数数组、环境变量、源配置与最终文件 SHA256；实际可读取的运行文件另外记录 SHA256，缺失文件列明，准备成功不代表运行依赖齐全。

保留原完整 DAG JSON 时，轻量目标可通过 `dag_profile` 引用；它不会被隐式改写。外部 DAG 如需可视化，必须显式配置输出 Feature，不能同时开启简写的 `visualization.enabled`。

未知字段、重复字段、错误类型、越界参数、缺少运行依赖均返回完整 JSON，例如：

```json
{"status":"failed","code":2001,"name":"CONFIG_INVALID","feature":"configuration","native_code":0,"detail":"..."}
```

配置错误为 `2001`，写入失败为 `2002`。子进程的 `failed.json` 有结构化业务错误时，工具保留完整业务码和 SDK 原始返回值；命令行进程退出为 `1`。POSIX 退出状态只有八位，不能将 `1001` 等完整业务码直接当成进程退出值读取。

全部新增 JSON、Python、Shell 和文档为 UTF-8 无 BOM、LF。冻结文件使用确定的 UTF-8/LF 写入，校验和在最终字节上计算；工具不通过重写 manifest 来掩盖源文件差异。
