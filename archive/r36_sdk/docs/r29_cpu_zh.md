# r12-v29：x86 CPU 推理

本版增加 ONNX Runtime CPU 后端。它复用原 DAG／Task／Feature、精确 NV12 缩放、检测 letterbox、分割插值＋ArgMax、同帧结果标识和可视化输出；X5 BIN 后端仍保留，默认行为不变。CPU 后端支持 Windows x64 和 Ubuntu x86_64 原生构建。

## 支持范围

| 项目 | x86 CPU | X5 BPU |
| --- | --- | --- |
| 推理模型 | 原始 ONNX，两个模型路径必须明确设置 | 原始 BIN |
| 输入 | 保存的 640×544 NV12 单帧或 TSV 图片集 | 原相机 SDK 或保存的 NV12 |
| 检测与分割 | 同一帧，两分支在原 DAG 内并行 | 保留现有路径 |
| 前处理 | 精确 resize；检测 letterbox、分割直接缩放；再执行显式 YUV→RGB/BGR＋归一化 | 保留现有 NV12 输入流程 |
| 后处理 | 明确输出布局的检测解码、NMS；分割插值＋ArgMax | 保留原检测/分割后处理 |
| 输出 | 检测 JSON、分割掩码、叠加 PNG、逐帧指标、完整错误码 | 现有输出 |
| 相机/视差/完整 ROS2 产品插件 | 本 CPU 入口未接入 | 原功能仍在 |

CPU 路径不加载 Horizon BIN，也不使用 X5 相机、VSE 或视差 BPU。完整产品插件依赖原 ROS2、SHM、点云和回充库，本次扩展是检测／分割 DAG 的可运行 CPU 后端。其他算法库在 x86 上是否可编译，应由各库的依赖和 ABI 单独确认。

## 1. 准备 ONNX 与显式契约

需要两个 ONNX：检测和分割。现有 `.bin` 不能交给 ONNX Runtime。当前模型输入限定为单批次 float32 NCHW，输出布局按契约选择。项目中 `profiles/cpu_exact.json` 和 `profiles/cpu_contract.example.json` 都是配置模板，路径、张量名称及归一化示例不代表已经匹配你的生产模型。

复制并修改 `cpu_exact.json`：

```json
{
  "schema_version": 1,
  "target": "cpu",
  "models": {
    "detection": "/absolute/path/detection.onnx",
    "segmentation": "/absolute/path/segmentation.onnx"
  },
  "cpu": { "profile": "/absolute/path/cpu_contract.json" },
  "input": { "kind": "raw", "path": "/absolute/path/source.nv12" },
  "run": { "out": "/absolute/path/results/cpu-01", "frames": 100, "warmup": 5 },
  "visualization": { "enabled": true, "every_n": 10, "layers": ["overlay"] }
}
```

Windows JSON 路径可写成 `D:/models/detection.onnx`，避免反斜杠转义。相对路径以这个 JSON 的目录为准。每次测试使用新的 `run.out`；准备目录默认是 `run.out` 后加 `.prepared`，也必须为新目录。

`cpu_contract.json` 必须写清：

- 模型输入张量名称、输入尺寸、RGB/BGR 通道顺序、`scale`、三通道 `mean`/`std`。
- YUV 转换契约 `bt601_limited_integer`；按有限范围整数系数转换，结果截断到 0..255。
- 浮点归一化公式为 `(channel * scale - mean[c]) / std[c]`。均值的单位取决于 `scale`，不能将 0..255 的均值与 0..1 的均值混用。
- 检测原始 DFL 的六个输出名称及 NCHW/NHWC 布局、类别数、`reg`、阈值和 NMS；或者明确的 `decoded_xyxy` 输出 `[x1,y1,x2,y2,score,class]`，坐标单位为模型输入像素。
- 分割输出名称、NCHW/NHWC 布局、类别名称；本路径接受 logits 后执行既有分割插值和 ArgMax。
- `runtime.intra_threads` 和 `runtime.inter_threads`。两分支已由 DAG 并行执行，先用 1/1 建立基线，再受控增加线程，避免线程争抢。

模型的颜色顺序与归一化必须来自训练／导出配置，不能仅通过 ONNX 的维度或 BIN 文件可靠推断。本次改动不保证浮点 CPU 与量化 BPU 零像素差。`verify` 比较的是同一 CPU 模型下的前处理优化一致性；比较 CPU/BPU 精度时，要固定源帧并比较各自输出，不能把 CPU `verify` 通过解释为跨平台精度已通过。

### 图片集输入

将 `input` 改为 `{"kind":"list","path":"/absolute/path/frames.tsv"}`。TSV 无表头，每行六列，用实际制表符隔开：

```text
1	640	544	640	nv12	/absolute/path/frame001.nv12
2	640	544	640	nv12	/absolute/path/frame002.nv12
```

以上展示中的列间是制表符；不能写成两个字符 `\t`。路径使用绝对路径，格式必须为 `nv12`，宽高维持 640×544，stride 至少 640 且满足原 NV12 约束。单帧 raw 文件在 stride=640 时应为 522240 字节。按源文件重复推理不代表在线相机 FPS。

## 2. Windows x64 构建与运行

依赖：Visual Studio 2022 的 C++ x64 工具、CMake 3.20 以上、Python 3，以及包含 `include/onnxruntime_cxx_api.h`、`lib/onnxruntime.lib`、`lib/onnxruntime.dll` 的 x64 ONNX Runtime SDK。本地验证使用的 Runtime 版本记录在验证报告中。

Windows 可执行文件嵌入 `activeCodePage=UTF-8` manifest，确保 JSON 的 UTF-8 路径与窄字符 argv／文件路径一致。该机制要求 **Windows 10 1903 或更新版本**；模型文件通过宽字符路径交给 ONNX Runtime。配置和脚本仍保存为 UTF-8 无 BOM、LF，不能依赖手工改编码来修复路径。

打开 **x64 Native Tools Command Prompt for VS 2022**，在源码根目录执行（将 SDK 路径换成实际目录）：

```bat
tools\build_cpu_windows.cmd D:\deps\onnxruntime-win-x64-1.16.3
```

生成目录：`cpu_runtime`。构建脚本会将实际执行需要的 ONNX Runtime 动态库装入运行目录；不需要 X5 sysroot、Docker 或交叉编译器。脚本报错会返回终端，不要双击脚本运行。

修改自己的 JSON 后，在 CMD 或 PowerShell 中执行：

```powershell
python .\cpu_runtime\tools\product_config.py --config D:/configs/cpu_exact.json --dry-run
python .\cpu_runtime\tools\product_config.py --config D:/configs/cpu_exact.json --run
```

使用安装目录中的工具，它会调用同目录的 `bin/dag_flow_worker.exe`。源码目录中的工具不会自动猜测任意 build 目录。

### 已编译的 Windows 包与本机试跑配置

`perception-sdk-cpu-windows-x64-r12-v29.tar.gz` 已含 x64 程序、ONNX Runtime 1.16.3 DLL 和许可证，解压后无需重新编译，也无需另外安装 Python 的 onnxruntime 包。先在解压根目录检查，再运行：

```powershell
python tools/integrity.py .
python tools/product_config.py --config profiles/cpu_found_models_trial.json --dry-run
python tools/product_config.py --config profiles/cpu_found_models_trial.json --run
```

这份 `trial` JSON 已填写本机找到的两个 ONNX 的张量名称，以及当前 `E:/new_data/daye` 下模型和原始 NV12 的路径。运行 10 帧、预热 2 帧，检测和分割同帧并行，每帧保存叠加图到 `results/cpu-found-trial/visualization`。再次运行前修改 `run.out` 为新目录。若文件换了位置，只修改启动 JSON 中的模型和输入路径；CPU 契约中的模型路径由启动参数覆盖。

该配置的颜色和归一化取自既有导出／推理配置作为试跑参数，**训练配置尚未确认**；检测 ONNX 与当前 X5 BIN 的权重对应关系也未证明。因此可以验证功能和观察输出，不能直接作为生产精度验收。包内不附模型和原始图像；通用接入时使用前述契约模板填写训练参数。

## 3. Ubuntu x86_64 构建与运行

使用 **Linux x64** 的 ONNX Runtime SDK；不能复用 Windows DLL。确认 SDK 包含 `include/onnxruntime_cxx_api.h` 和 `lib/libonnxruntime.so`。

```bash
cd /absolute/path/perception-x5-cpu-dag-r12-v29
bash tools/build_cpu_ubuntu.sh /absolute/path/onnxruntime-linux-x64-1.16.3
python3 cpu_runtime/tools/product_config.py --config /absolute/path/cpu_exact.json --dry-run
bash cpu_runtime/tools/run_cpu.sh --config /absolute/path/cpu_exact.json --run
```

这是 Ubuntu 本机编译，不进入原 X5 Docker，不设置 `X5_SYSROOT`。如需改变构建／安装目录，先设置 `CPU_BUILD_DIR`、`CPU_INSTALL_DIR`；默认分别为 `build/cpu-ubuntu` 和 `cpu_runtime`。`CPU_BUILD_JOBS` 默认 4。

## 4. 输出与验收

准备目录保存 effective JSON、展开后的 DAG、CPU 契约、完整 argv 及 SHA256。启动前检查模型、输入和可执行文件；C++ 再严格验证完整 CPU 契约及实际 ONNX 张量，错误不会静默退回 BPU 或虚拟推理。

结果目录沿用 `complete.json`、`frames.csv`、`graph_frames.csv`、`dag_trace.jsonl` 和按需可视化。出现错误时读取 `failed.json` 的完整业务码；进程退出码仅表示成功/失败。`visualization.every_n=1` 会引入 PNG 编码和写盘开销，测推理 FPS 时先关闭可视化，再单独测打开后的总开销。

CPU 指标含义如下：

- `det_infer_ms`／`seg_infer_ms` 为后端中立的 **DAG 推理 Feature 时间**，包含推理调用、输出检查及必要的复制／布局整理，不等于纯模型计算时间。
- `det_cpu_infer_ms`／`seg_cpu_infer_ms` 是 CPU 模式下上述 Feature 时间的明确标记。
- `det_ort_run_ms`／`seg_ort_run_ms` 单独包围 ONNX Runtime `Session::Run` 调用，包含 Runtime 调度及输出分配，不包含之后的检查和复制。
- YUV 转 RGB/BGR 与归一化属于前处理，计入相应 `*_pre_ms`。兼容旧 CSV 的 `*_bpu_ms` 在 CPU 路径保持 0；不得将其解释为 CPU 计算耗时。
- `cpu_pct` 是整个进程占用，相当于一个逻辑核满载为 100%，并行时可超过 100%。RSS 在 Windows 取进程 WorkingSetSize，在 Linux 取 VmRSS，统一以 KiB 输出；它不是模型权重大小。

`run.mode=verify` 同一帧会执行基线与优化两次推理，并保存验收证据。该模式的 FPS、CPU 和输出开销不能当作正常单次推理性能；一致性通过后使用 `optimized` 关闭证据保存来测速。

建议验证顺序：

1. 在固定原始 NV12 上检查尺寸、颜色、张量名称和类别配置；保存检测框与分割图。
2. 使用 `run.mode=verify` 检查同一 CPU 后端的精确前处理优化；成功后再测 `optimized`。
3. 关闭可视化，固定输入集和线程数，记录 FPS、整帧延迟、CPU、RSS；然后打开可视化复测。
4. 同一批源帧与 X5 BPU 结果比较输入字节、检测匹配、掩码差异和每类 IoU。浮点与量化差异需独立归因，不能用速度测试替代精度验证。

已在 Windows x64 上实际运行找到的检测 ONNX 和三类 PIDNet，并通过同一 CPU 后端的基线／优化输入零字节差、分割零像素差、检测结果逐位一致检查。另以六类可控 ONNX 覆盖布局、DFL、NMS 和异常输出，验证中文路径、可视化及完整错误码。详见 `validation/r29_cpu_cli_validation.json` 和 `validation/r29_model_inventory.json`。

Ubuntu 构建入口已提供，本次未在 Ubuntu 实际编译或运行；X5 路径保留，本次未重新板测。CPU 与 BPU 的准确率差异及生产实时 FPS 尚未验收。
