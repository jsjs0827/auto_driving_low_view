# r12-v33 完整共享库：回充、视差、点云与三张俯视图

本版本在现有 r32 推理共享库之上增加完整业务入口。回充代码来自本地 `charing-main`，点云、标定、Patchwork++、语义点云和栅格融合代码来自 `point-cloud-processing-main`。各步通过现有 DAG、Task、Feature 执行，输出每帧任务状态和耗时。

当前交付状态是源码移植和接口集成。原始数值代码保留情况可通过哈希和差异文件核对；完整地图模块尚未在具备真实 PCL 的环境中完成编译和运行对比，不能据此宣称与原项目已经通过端到端功能等价验收。目标环境应运行下述原始算法对照测试，并使用同一组实际相机数据与原程序比较。

## 三张地图与输出

| 输出 | 数据来源 | 内容 |
| --- | --- | --- |
| `pcl` | 地面、非地面点云 | 原始 PCL 地面/障碍栅格 |
| `ai` | 本帧检测和分割回投影后的语义点云 | 原始模型类别到栅格类型的映射 |
| `fusion` | 原始 `AiFusion::integrate` | 原始融合、补洞和邻域去噪结果 |

结果同时包含三张地图的逐格 `min_h/max_h/avg_h` 米值、融合地图的 `int16` 厘米高度数组、`uint8` 类型数组、行列数、毫米分辨率和最近障碍物信息。启用 `retain_clouds` 后还包含相机、车体、地面、障碍物、语义五组点云。

栅格按 `row * cols + col` 存储：行递增表示车体前方 `+x`，列递增表示车体右方 `-y`。PNG 保留原始 `Visualize::saveMap` 的行翻转、调色板、坐标箭头、最近邻放大和网格线；PNG 像素与原始格子数组的行方向不同。栅格厘米数组中的空格高度按原始安全转换输出零；逐格米值仍保留原始默认哨兵 `999/-999`。

原始 `start_config.json` 的 `label_fusion.fuse_into_grid` 为 `false`。保持此值时仍会产生三张地图，但 `ai` 地图保持未知，语义点云不参与栅格融合。需要检测/分割参与地图时，显式改为 `true`；类别、可通行策略和过滤阈值继续由原始 JSON 决定。

## 输入契约

默认相机输入是同帧 `640×544 NV12` 左目和 `uint16 Q8` 视差。视差像素值为 `sample / 256.0`；零视差不生成点。可以传入带行填充的视差，SDK 校验后逐行复制为紧凑数据。左目、视差和语义结果必须具有同一帧身份；视差还必须具有相同时间戳。SDK 不用伪造视差或恒定深度填补缺失输入。

公开 C ABI 的 `psdk_camera_intrinsics.bxf` 使用原始相机单位：`fx × baseline_mm`，不是米。地图模块只进行一次 `/1000.0F` 转换，得到 `bxf_m` 后以 `Z[m] = bxf_m / disparity[pixel]` 生成点云。传入的内参宽高必须与左目/视差一致；调用方应先按图像尺寸缩放 `fx/fy/cx/cy`，并按水平比例缩放 `bxf`。

回充高分辨率输入通过 `recharge_hires` 提供，按原始配置选择 GDC 左目，保持同帧身份。原始 `point_cloud_color.use_gdc_image` 虽被解析，但所选原项目的点云颜色路径实际使用 RGBD 左目；本次保留该行为。ROS 图像发布缩放、Marker、TF 和应用消息传输由调用方完成，完整共享库提供原始数值结果与 publication 回调。

## 框架执行与配置

完整业务图包含 `dnn → recharge → rgb → cloud → calibration → labels → transform → ground → maps → publish → record`，另有真实视差输入校验/复制的 `disparity` Feature。依赖关系允许视差准备与推理并行，地图的各个数值阶段按顺序执行。禁用分支从运行图中移除；请求未编译的地图或回充能力时返回明确错误。

简单包装配置是 `config/full.json`。它引用原始推理配置、原始地图 `original_start.json`、`extrinsics.json` 和回充 JSON，不要求用户编写底层 Task 参数。路径相对包装 JSON 所在目录解析。保留高级 DAG 配置供现有调用方式使用。

| 配置 | 作用 |
| --- | --- |
| `mode` | `map`、`recharge`、`map_and_recharge`、`stopped` |
| `mapping.original_config/extrinsics` | 原始地图参数与外参文件 |
| `mapping.sample_step` | 原始视差采样步长，默认 4 |
| `mapping.retain_clouds/render_maps` | 保留点云、生成原始风格 RGB 俯视图 |
| `mapping.calibration_output` | 自动标定结果的写入位置 |
| `recharge.config` | 原始回充配置文件 |
| `output.directory/every_n` | 录制目录与帧间隔；零表示关闭自动录制 |

原始 `auto_cali=true` 是标定模式：每帧使用 PCL 平面拟合，累计六次后按原始规则平均、保留两位小数并写外参；该帧流程不生成地图。内存 JSON 调用没有输出路径时可从结果读取标定值。平面拟合失败返回错误，避免原代码在失败路径中把未初始化角度写入文件。

原始命令 `1/2/3/21` 分别表示建图、回充、两者、停止。通过 `psdk_full_command` 切换；相机启停和外部 ROS/Socket 命令转发由宿主负责。

## 构建 Linux SO

默认完整构建启用 `R33_WITH_MAPPING=ON` 和 `R33_WITH_RECHARGE=ON`，地图要求真实 PCL、Eigen、OpenCV，回充要求真实 OpenCV。地图代码需要的 PCL 组件是 `common/filters/features/segmentation/search/sample_consensus/kdtree`；保留的 `PclCommon` 未使用 PCL IO 导出符号。配置阶段缺少依赖会失败。

数学运行库可通过 `tools/stage_math_runtime.py` 按实际链接清单和递归 `DT_NEEDED` 打包到 SDK 的 `lib`：每个文件均检查真实 AArch64 ELF、精确 SONAME 和 SHA-256，并在 `math_runtime_provenance.json` 记录来源及依赖。明确链接文件优先；未固定版本时若同 SONAME 对应不同内容，或必需数学依赖缺失，打包失败。glibc、libstdc++、libgcc、动态加载器、Horizon、相机、HBRT/hbmem/vpf、cjson 和 ROS 库始终使用目标环境提供的版本，打包工具不会覆盖这些系统或硬件 ABI。

在已有 X5 工具链容器的 Ubuntu 宿主执行：

```bash
bash tools/build_in_container.sh \
  --container obs-v004-x5-build \
  --sysroot /work/obs_v004_x5/sysroot_docker \
  --out ../r33_shared_release
```

沿用已有容器和 sysroot，无需为本模块生成替代 SDK 头文件。预期交付包为 `perception-sdk-x5-r12-v33-full-shared.tar.gz`；以构建脚本实际打印的成功结果和导出符号检查为准。完整地图未编译成功之前，任何关闭地图构建产生的库都不应标作完整版本。

独立编译地图模块和验收测试：

```bash
cmake -S mapping -B build-mapping \
  -DCMAKE_BUILD_TYPE=Release \
  -DR33_MAPPING_BUILD_TESTS=ON
cmake --build build-mapping -j2
ctest --test-dir build-mapping --output-on-failure
```

交叉编译时增加实际工具链参数；交叉生成的测试可执行文件需要在相应 AArch64 环境运行。`r33_mapping_original_oracle` 覆盖带填充 Q8 视差的投影、真实 Patchwork/PCL 后处理、原始 AiFusion 对照、跨帧语义清空和 PNG 行方向/颜色。

## Demo 与 C ABI 调用

文件模式示例，输入文件必须来自同一帧：

```bash
bash tools/run_full_x5.sh \
  --config config/full.json \
  --input left.nv12 --disparity disparity_q8.u16 \
  --hires gdc_left.nv12 \
  --width 640 --height 544 --stride 640 \
  --frames 1 --out output/full-demo-001
```

相机模式示例：

```bash
bash tools/run_full_x5.sh --config config/full.json \
  --camera --sdk /app/data/metoak/lib/libMoGeneralSDK.so \
  --camera-config /app/data/metoak/demo/config/x5_camera.cfg \
  --frames 120 --out output/full-camera-001
```

命令行还应传入与设备一致的实际内参；以 Demo `--help` 中的内参选项为准。高分辨率回充输入不可仅凭相近时间戳配对；缺少相同身份的 GDC 输入时，由原始回充策略和显式 `allow_source_fallback` 决定是否使用左目。

新增公开头文件是 `shared/include/perception_full_sdk.h`，旧 `perception_sdk.h` ABI 保留。调用顺序：

```c
psdk_full_config config = {0};
config.struct_size = sizeof(config);
config.abi_version = PSDK_FULL_ABI_VERSION;
config.json_file = "config/full.json";
psdk_full_engine *engine = NULL;
psdk_status status = {0};
if (psdk_full_create(&config, &engine, &status)) { /* 处理 status */ }

/* 填写同帧 left、disparity、intrinsics；可选 recharge_hires。 */
psdk_full_frame frame = {0};
frame.struct_size = sizeof(frame);
frame.abi_version = PSDK_FULL_ABI_VERSION;
/* frame.left 的 struct_size/abi_version 与原有 psdk_frame 契约相同。 */
/* frame.disparity->stride_bytes 使用字节；bxf 使用 pixel-mm。 */
psdk_full_result *result = NULL;
if (!psdk_full_process(engine, &frame, &result, &status)) {
    psdk_full_result_view view = {0};
    view.struct_size = sizeof(view);
    view.abi_version = PSDK_FULL_ABI_VERSION;
    if (!psdk_full_result_get_view(result, &view, &status)) {
        /* 读取 view.maps、grid_types、高度、点云和 recharge。 */
    }
    psdk_full_result_release(result);
}
psdk_full_destroy(&engine, &status);
```

结果视图中的指针由 `psdk_full_result` 持有，在 `psdk_full_result_release` 前有效；销毁 engine 不会提前销毁已经返回的结果。publication/视差 provider 回调执行期间不可重入完整 SDK。回充受原始全局滤波状态约束，一个进程内只允许一个回充实例。

保存结果会写入新目录，包含 `pcl.png/ai.png/fusion.png`、逐格米值、厘米高度、类型数组、回充诊断、任务轨迹、有效配置，以及按配置保留的点云。录制完成标记是 `complete.json`；原始输入保存由 `save_inputs` 控制。二进制结构布局以公开 ABI 头文件为准。

## 可核对的移植依据

`mapping/original` 保留原始文件快照。`mapping/SOURCE_PROVENANCE.json` 记录快照和编译文件 SHA-256，`SOURCE_DELTAS.patch` 列出为移除 middleware 依赖而做的改动。`Patchwork++`、`AiFusion`、安全栅格计算、语义标签编码等数值源文件保持逐字一致，仅统一文本编码和换行。语义融合及地面后处理由脚本从原函数抽出，移除 ROS 发布调用并替换成员所属类。

```bash
python3 tools/verify_full_sources.py
```

完整功能范围和尚未完成的验收见 `FUNCTIONAL_SCOPE_R33.json`。原相机/地面场景的数据回放、X5 BPU 实际模型运行、回充实物定位以及三张俯视图的逐格对比，均应在真实目标依赖和相机条件下确认。


完整的解压、Ubuntu 宿主交叉编译、X5 首次检查与包外 JSON 配置命令见根目录 README_SHARED_zh.md。本机验证报告位于 validation/r33_native_validation.json，实测范围与未验证范围分开记录。

资源报告：`resource_lifecycle.jsonl` 分别记录初始化前、预热后、测量后、相机关闭、引擎销毁和最终结果释放后的 RSS/CMA。引擎销毁时仍有一个调用方持有的结果；应结合 `after_result_release` 判断生命周期，不能把该内存误当作泄漏。Windows 没有 Linux procfs 的字段为 null。
