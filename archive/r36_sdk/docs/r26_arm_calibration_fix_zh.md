# r26 ARM 校准修复 v1：保留原输入，严格复测零差与性能

本包修复 `original_input_contract_tests` 在 X5 上出现的 `Original-map direct input bytes differ`。交付的是**完整源码包**，需在原 Ubuntu C++ 交叉编译环境中生成 X5 SDK；不是已经编译的板端 SDK。

## 问题与修复

板端日志表明，640 stride 的检测和分割测试已通过，失败发生在 656 stride 的带行补齐检测输入测试。此前用初始黑帧和五种通用探针校准 FMA 顺序，在新增的 640×544 输入上，仍可能同时保留模式 3 和模式 4。两种模式对大多数像素产生相同字节，但在插值值非常接近半整数时，会相差一个字节；之后复用先选出的模式，就可能在其他帧暴露差异。

独立的主机显式九模式 oracle 已复现这一校准不足：旧校准选择 Y/U/V=4/4/3 后，656 stride 的后续随机帧出现一个字节差。该复现证明旧探针不能可靠区分这些模式；板端原日志没有保存差异坐标，不能把主机复现坐标当作原板端失败坐标。

本次修改如下：

1. 对 640×544 → 检测有效区 564×480，在启动校准阶段增加三组专门区分半整数舍入的探针。由当前构建中的原始 scalar 例程产生参考字节，再筛选逐通道 FMA 模式；不硬编码某个模式作为所有平台的答案。
2. ARM 原始输入一致性测试改为调用实际 `original_runner` 的参考回调，并保留原 stride/格式上下文，和生产路径一致。黑帧、白帧、后续随机帧仍逐字节验收。
3. 新增 `calibration_640_tests`，使用独立显式九模式 oracle 覆盖首次黑帧校准、后续不同帧及带行补齐输入。
4. 测试失败时直接显示日志位置和末 30 行；另保存 `preflight_failure.json`。原始输入一致性测试通过 `--diagnostics-dir` 保存差异证据，方便继续定位。

新增探针只在 workspace 建立/输入几何变化时参与校准，稳定逐帧循环继续使用已选模式。它不替换原插值公式、不改变舍入规则，也不绕过验收。启动校准耗时会增加，稳态 FPS 仍需板端重新测量。

## 保持原流程

| 部分 | 本包行为 |
| --- | --- |
| 相机输入 | 原地图 RGBD 左目 640×544，保留实际 stride；按原配置解释为 NV12 |
| 检测前处理 | 原 letterbox 到 864×480，有效区 564×480，左右各 150；Y=127、UV=128 补边 |
| 检测模型与后处理 | 原 mmyolo0828 BIN、解码、阈值、坐标还原、NMS |
| 分割前处理 | 原直接缩放到 832×480 |
| 分割模型与后处理 | 原三类 PIDNet BIN；保留已验证的同类区域填充与边界原双线性插值/ArgMax，输出 640×544 mask |
| 精确加速 | 紧凑查表、固定补边复用、直接写 BPU 输入、每帧 cache clean |
| 框架与调度 | 原 DAG／Task／Feature，单帧 batch=1，无双缓冲；同帧检测和分割可并行 |
| 相机 SDK 与配置 | 继续使用板端原文件，不修改 SDK 二进制、视差模式和相机配置 |

同帧检测/分割 prepared NV12 和实际 BPU input storage 必须零字节差，分割零像素差，检测结果 bit 一致。任何一项失败即停止性能测试。没有标注时，结果比较代表一致性，不代表 mAP/mIoU。

## Ubuntu 宿主机交叉编译

继续使用原来的 `bash tools/build_in_container.sh`，在 Ubuntu **宿主机**执行，无需手动进入 Docker、无需拉镜像、无需重新编译模型。`obs-v004-x5-build` 是已有 C++ 容器名；如实际使用的是 `x5-build`，仅替换 `--container`。

将源码包及同名 `.sha256` 放入 `/home/st/ini_code`：

```bash
cd /home/st/ini_code
sha256sum -c perception-x5-original-input-r12-v26-arm-calibration-fix-v1.tar.gz.sha256
tar -xzf perception-x5-original-input-r12-v26-arm-calibration-fix-v1.tar.gz
cd perception-x5-original-input-r12-v26-arm-calibration-fix-v1

sudo docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
sudo bash tools/build_in_container.sh \
  --container obs-v004-x5-build \
  --sysroot /work/obs_v004_x5/sysroot_docker \
  --out /home/st/ini_code/r26_arm_calibration_fix_release
```

直接用 `bash` 执行，不要 `source` 脚本，不要在交互终端增加 `set -e` 或 `exit`。脚本会复制源码到容器，交叉编译并将 SDK 拷回宿主机。

生成文件：

```text
/home/st/ini_code/r26_arm_calibration_fix_release/perception-sdk-x5-r12-v26-arm-calibration-fix-v1.tar.gz
/home/st/ini_code/r26_arm_calibration_fix_release/perception-sdk-x5-r12-v26-arm-calibration-fix-v1.tar.gz.sha256
```

若已经位于正确的 C++ 交叉编译容器中，在新源码根目录也可执行：

```bash
export X5_SYSROOT=/work/obs_v004_x5/sysroot_docker
export X5_HORIZON_ROOT=/work/obs_v004_x5/sysroot_docker/usr_x5
bash tools/build_x5_sdk.sh
```

这个入口生成到新源码目录的 `releases/`。不要将 AI 工具链镜像名当成容器名。

## X5 解压与运行

将 Ubuntu 生成的两个 **SDK** 文件复制到 `/userdata/ai_infer_r12_v26_fix`。使用新目录，避免与旧包混用。先关闭其他正在使用相机的程序。

```bash
mkdir -p /userdata/ai_infer_r12_v26_fix
cd /userdata/ai_infer_r12_v26_fix
sha256sum -c perception-sdk-x5-r12-v26-arm-calibration-fix-v1.tar.gz.sha256
tar -xzf perception-sdk-x5-r12-v26-arm-calibration-fix-v1.tar.gz
cd perception-sdk-x5-r12-v26-arm-calibration-fix-v1

bash tools/run_x5.sh \
  --sdk /app/data/metoak/lib/libMoGeneralSDK.so \
  --camera-config /app/data/metoak/demo/config/x5_camera.cfg \
  --det-model /userdata/ai_infer/data/mmyolo0828.bin \
  --seg-model /userdata/ai_infer/data/pidnet_0825_3classes.bin \
  --stream rgbd --format nv12 --disparity-mode 0 \
  --frames 120 --warmup 20 --capture-frames 10 --rounds 2 \
  --out /userdata/r12_v26_fix_res
```

模型、SDK、配置路径继续使用之前能运行的原文件；实际位置不同则修改参数，不能用融合 BIN 代替原模型。默认有限帧结束，不会一直采集。

全部 ARM 预检通过后，程序才会采集原 RGBD 帧、冻结回放并进行严格同帧验证；随后进行原始/r21/compact_direct 两轮正反顺序的固定帧及在线性能对比。测试 FPS、CPU、RSS、CMA 和检测/分割六阶段耗时，保持主性能阶段不保存图片。

长测和重开仍为可选。在完整运行命令后增加 `--long-seconds 600 --reopen-count 1 --reopen-frames 30` 即可；本补丁不声称修复相机 SDK 重开资源滞留。

## 结果与失败证据

结果目录为 `/userdata/r12_v26_fix_res/original-input-ab-日期时间-进程号/`，主要文件包括 `report.html`、`report.md`、`performance_summary.json`、`exact_acceptance.json`、`resource_summary.json`、`framework_summary.json` 和 `run.json`。可优先回传 `metrics_only.tar.gz` 与同名 `.sha256`，完整张量证据保留在结果目录中。

预检失败时不会跳过验收。终端直接显示失败测试、日志路径与末 30 行；`preflight_failure.json` 和 `run.json` 记录失败名称。若原输入测试仍失败，请保留并回传：

```text
original_input_contract_tests_arm.log
original_input_contract_tests_evidence/
preflight_failure.json
run.json
metrics_only.tar.gz
metrics_only.tar.gz.sha256
```

证据目录可能含较大的原始输入和 prepared 字节，指标包不包含这些二进制文件，需要单独复制。其他预检失败则保留相应的 `测试名_arm.log`。不要删除严格测试、改成只比较输出，或重新生成清单掩盖文件变化。

## 本机验证结果

- 17 项 CTest 全部通过，包括实际原始 runner 参考函数、DAG 任务图、输入写入和分割后处理回归。
- 独立显式 FMA 回归：9 种统一模式及 3 组跨通道模式，48 个上下文、1248 次 cached/direct 比较，零字节差。保留黑→白→随机顺序，并追加 Y/U/V 半整数边界及四组独立随机范围。
- 13 项 Python 测试通过；成功/失败/超时三种预检行为通过。人工注入 1 字节差异的 4 例全部严格拒绝，证据文件复核通过。
- CMake 构建出的新增测试另以模式 4 跑了 104 次比较，全部零差。原输入入口的有效/拒绝用例通过。

以上均为 Windows x64 主机验证；完整记录见 `validation/r26_fix_host_validation.json`。尚未执行新 SDK 的 Ubuntu 交叉编译和 X5 实测，不据此给出板端 FPS 或准确率结论。

## 验证边界与字符一致性

Windows 主机回归可以证明本地明确构造的模式歧义被消除、独立 oracle 用例与测试/报告逻辑通过。主机不是 X5：真实 ARM 编译器的 FMA 选择、原 BIN 的 BPU 输入/输出一致性，以及 FPS/CPU/RSS/CMA 必须由新 SDK 的板端结果确认。本包不提供尚未测得的板端提速结论。

源码、脚本与文档统一使用 UTF-8 无 BOM、LF。最终清单按文件实际字节生成，归档后逐成员核对，再生成外部 `.sha256`。Ubuntu/X5 直接解压，不经过会自动换行的编辑器。旧文档保留作历史资料；本修复包以当前文件与 `r26_arm_calibration_fix_zh.html` 为入口。
