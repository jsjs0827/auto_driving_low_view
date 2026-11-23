# 原产品业务接入与验证边界

r30：命令、CameraParam 验收/保存、后台实际 I/O、回充回调和离线读取已增加六个原 TaskGraph 事件 Feature。见 [本轮对比](../docs/r30_original_comparison_zh.md)，下文原有事件边界描述以该报告为准。

本目录从 `point-cloud-processing-main-ini/point-cloud-processing-main` 迁移真实业务源码。`BASELINE_PROVENANCE.json` 记录原始字节 SHA256；v27 的 `reference/original_product` 五个基准文件与该原始项目一致。旁边名称相近的 `point-cloud-processing-main` 是另一份后续重构，未作为本次业务基准。

业务注册仍使用原 `perception::runtime::FeatureRegistry` 和 `TaskGraph`。每帧闭包直接调用下面列出的原 `PercepDemo`、PCL、PatchWork++ 和 `AiFusion` 函数。产品层只允许一个 graph worker，保持可变点云/地图的依赖顺序；精确 AI 内部 DAG 独立执行双模型 lane。图不能删去业务 Feature、禁用 task、削弱必要依赖或使用未知绑定。关闭某一帧不需要的步骤时，保留穿过该步骤的传递依赖，保证失败不会发布旧地图。

## 原功能完整清单和落点

| 原产品功能 | 当前真实落点 | 验证边界 |
|---|---|---|
| det/seg 模型、原预处理/后处理、保持类别、语义 mask 后处理 | `product.ai.exact` → `runDnnInference` → `ExactDnnRunner` → 根精确 DAG/NativeStages | 冻结数值文件保持不变；产品 NativeStages 桥接尚待 ARM 原依赖编译和板端比较 |
| YOLO charger 辅助裁框、静态 ROI、二维码 SDK 异步提交和结果发布 | `product.recharge_mark` → `applyYoloAssistCropHint`/`submitRechargeMarkFromShm` | 实际二维码算法仍在匹配的原 `librecharge_mark.so`；busy/interval 跳帧正常，缺所选相机输入返回 1001，SDK 失败/等待超时返回 2004 |
| RGB NV12 转换、ROS 图像、尺寸配置 | `product.image.rgb` → 原转换和发布函数 | 可视化关闭不构建地图用 RGB 图片；不改变输入像素或 SDK |
| Q8.8 视差采样、内参回投影、FOV 筛选 | `product.pointcloud.from_disparity` → `dense2sparse_pointcloud` | host 对 3 个坐标 × 65536 个原始视差字逐位验证标量投影；完整 PCL/地图字节比较待板端 |
| 自动地面外参标定、写 extrinsics | `product.calibration.auto` → 原 `autocali` | 原标定模式不继续地图发布；需要原 PCL 算法及实际标定数据验收 |
| 检测框点云匹配/marker，分割形态学、深度层、体素连通、射线/簇过滤和语义点云 | `product.fusion.semantic_labels` → 原 `matchAndPublishDetCloud`/`fuseSemanticLabels` | 关闭 label_fusion 仍执行原 det_cloud 兼容路径；原 `fuse_into_grid`/`min_semantic_points_per_cell` 为原预留字段，不承诺写入栅格 |
| 点云着色、外参变换、TF、原点云 ROS 发布 | `product.pointcloud.transform_color` → 原 `postprocess`/`publish3dTF` | 数学变换保留；只有真实需要可视化时构造颜色/ROS 点云 |
| PatchWork++ 地面分割、原 PCL 去噪/聚类 | `product.pointcloud.patchwork` → 原 `runGroundSegmentation`/`PclCommon` | 使用真实原 PCL/PatchWork++，没有 portable 替代；板端 native 编译、点云输出仍待验证 |
| 原点云投影、AI 图、融合图、孔洞填补、噪声/高度过滤 | `product.fusion.ai_fusion` → 原 `AiFusion::initData/integrate` | 迁移原 `ai_fusion.hpp`；未用 framework portable fusion 冒充原产品算法 |
| GridMapInfo、地图彩色 marker、RViz | `product.output.ros_maps` → `publishProductMaps` | GridMapInfo 是地图业务必需输出；marker 仅 visualize 时构造/发布 |
| 最新槽 map/GDC/采样视差数据记录、文件保留 | `product.output.record_latest` → `recordProductMaps`；后台 `saveLogLoop` → `product.output.record_io` 完成实际 I/O | 入队成功仅表示最新槽接收；后续失败通过 C API 与 JSONL 返回 2002。被新帧替换的待保存任务及关闭时未开始的任务沿用原 latest-only 语义 |
| ROS2/daye 插件注册、相机命令、服务状态/启动/停止、自测 | 保留原 REGISTER_FUNCTION 和原服务入口 | r30 命令执行进入 product.control.command 事件图；旧 ROS 服务协议保持，完整码由响应/C API/JSON 返回 |
| RGBD/GDC SHM 订阅、CameraParam、帧选择/等待/暂停 | 保留原真实 daye/CameraData ABI 和回调 | 每路只保留最新一份 metadata；SHM 像素所有权仍由原 SDK 提供，未造相机 ABI 或复制一套 SDK |
| JPG + raw disparity 离线/交互回放 | 原文件加载/交互循环 → 同一 cloud/transform/ground/maps/publish/record DAG | 基准离线原本没有运行 AI/语义匹配；当前保留几何回放，可 save_map。离线 save_data 缺原始 GDC 上下文，初始化明确拒绝（2001） |
| 相机内外参和 FOV JSON 保存 | product.input.camera_params → product.output.camera_params → 原 saveCamParams | 控制面输出检查 fopen/flush/close，失败 2002 |
| 原 det/seg overlay JPG、overlay/image/pointcloud/map ROS、地图 PNG | 保留原 dnn_runtime renderer、ROS 和 Visualize | 使用原配置开关；根 exact PNG 输出 Feature 属于独立 exact profile，不宣称 full 产品已自动输出其 PNG |

v27 框架本身的 17 个 portable Feature 是真实通用算法；但默认精确 worker 只接入 input + det 三段 + seg 三段 + stereo.demand，stereo.demand 当时只提供视差信息，没有原始视差像素。v27 `ProductAiFusionAdapter` 只有声明没有可链接实现。本目录的原业务接入与那个声明无关，实际调用上述原产品对象。

## 构建与启动

默认 host 构建产生可移植 `product_dag` 和业务编排、事件、命令、离线契约测试，不依赖 ROS/PCL，不编译整个原产品。该测试验证的是真实框架编排、依赖失败跳过、模式切换、完整错误码与 Q8.8 标量数学，不验证 vendor/ROS/AI/PCL ABI。

```sh
cmake -S . -B build-host -DBUILD_TESTING=ON
cmake --build build-host --target product_dag_tests
ctest --test-dir build-host -R product_original_framework --output-on-failure
```

在具有原 Ubuntu/X5 sysroot 和 ROS2 工作空间的主机显式构建全量插件：

```sh
sudo bash tools/build_in_container.sh --container obs-v004-x5-build \
  --sysroot /work/obs_v004_x5/sysroot_docker --out /home/st/ini_code/r28_product_release \
  --product --deps-root /work/obs_v004_x5/ai_ws/src \
  --recharge-root /path/inside/container/to/matched/recharge_mark
```

CMake 变量为 `PERCEPTION_BUILD_PRODUCT=ON`、`PRODUCT_DEPS_ROOT`、`RECHARGE_SDK_ROOT`，还需要 `R20_WITH_BPU=ON`、原 AArch64 runner、编译器/sysroot/Horizon SDK 和真实 ROS2/common_interface/PCL/Eigen/OpenCV/yaml-cpp/OpenMP。全量目标 `product_percep_demo`，输出 `<build>/product/libpercep_demo.so`；安装至 `lib/ai_node/libpercep_demo.so`。根工具链负责 C/C++ 交叉编译；原依赖头和库由产品 CMake 检查。

若原 ROS2/common_interface 的 AArch64 install 在 sysroot 外，显式设置 `PRODUCT_CROSS_PREFIX_PATH="/原ARM/ros2/install;/原ARM/ai_ws/install/common_interface"`（可多前缀、分号分隔）。full 子目录同时把这些目录加入 package prefixes 与查找根，保留 exact 默认 root-only 查找，不自动搜索 Ubuntu host 的 x86 库。变量必须是容器内真实绝对目录；只有明确的原 daye/recharge 库路径不重定位，并检查 ELF AArch64。

插件由已有 ROS2/daye ai_node launcher 加载，注册名仍为 `percep_demo`。原工程未提供可验证统一 launcher 命令，所以不编造一个新 executable。简化入口 `tools/product_config.py --config ...` 可 prepare/dry-run；实际 `--run` 要在 JSON 给出真实 `launch.argv` 数组，并检查已编译插件存在。launcher 环境包含 `PERCEPTION_PRODUCT_CONFIG=<冻结 start_config.json 绝对路径>`，插件读取该文件。配置 `product.dag_config`、`product.exact_profiles`、`product.metrics_dir` 相对 start_config.json 解析；未设置 metrics_dir 不写指标。exact profiles 默认安装于 `/app/daye/share/perception_x5/profiles`。

full 配置可开关 ground、semantic_fusion、auto_calibration、recharge；原地图所需 pointcloud/map 链继续执行。r30 恢复原独立 det/seg 开关，按请求选择 det-only、seg-only 或双 lane，并遵循 parallel_det_seg。只检查启用模型；分割启用时 class_names/num_classes 必须完整。整产品 `dnn.enabled=false` 保留原关闭入口，但不会获得 AI 结果。host 适配契约已测，真实 BPU/ROS 待验收。

## 已发现的原依赖阻塞

本机 `gecao/ai_ws/src/thirdparty/include/base_framework/*.h`、`common_interface/include/shm_struct/camera_struct.h` 等文件是受保护二进制内容，不是可编译 UTF8 C++ 头。它们虽存在于原项目，不能用于真实 ABI 编译。CMake 提前 fail-fast，需在原 Ubuntu 工程使用可编译源头副本；禁止用 fake_camera_sdk/自造结构代替生产 ABI。

`charing-apriltag/librecharge_mark.so` 确认是 AArch64 ELF，但对应头与本产品保留 API 不一致（新头去掉原末尾 setExternalCropHint 等）；不能只复制 .so 然后按旧函数表调用。应提供原版本匹配 header/.so。`tools/probe_product_dependencies.py` 只读检查文件、ELF、头 ABI；检查通过仍需真实 CMake 编译、链接和板端 dlopen。新 `product_image_codec.cpp` 提供实际外部 JPEG encoder，插件链接设置 `--no-undefined`，避免原 stdio stbi_write_jpg 未定义符号。

因此本次 host 通过不能作为 full native build/hardware 通过证据。完整插件、回充 SDK、PCL/ROS 输出和实际性能验收仍 pending，发布能力元数据必须如实标明。

## 状态、指标和验收

`perception_product_last_status` 声明在公开 `include/perception/product_status.h`，返回完整 uint32 业务码；先以 null/0 查询 JSON 所需字节（含 NUL），再提供足够缓冲。缓冲不足保持原内容。JSON 包含原供应商 native_code、Feature、task、frame_id、timestamp 和 detail；C ABI 捕获异常，不跨 ABI 抛出。原 1001…1015 保留，配置/输出/传输/回充/标定/清理分别 2001…2006。后台保存/回充异常也进入该状态接口，不把排队成功写成落盘成功。

配置 metrics_dir 时生成 `product_metrics.csv` 和 `product_status.jsonl`。CSV 包含 run_id、frame_id、timestamp_ns、各 task state/ms、整帧 wall/process CPU ms、RSS KiB、CMA free KiB；r30 Windows 使用实际进程 CPU 时间与 WorkingSet，CMA 不可用。整帧数值在各 task 行重复，聚合应先按 run_id 去重，不能直接按行求和。run_id 区分离线同 frame_id 的重复回放；异步记录完成/失败使用受理帧的同 run_id。记录和回充仍为原异步边界，主图 CSV task_ms 衡量入队/等待，后续实际执行由 product_event_metrics.csv 和 product_event_status.jsonl 记录，event_id/parent_run_id 区分关联。

落实的非 AI 改动：仅可视化时构造全尺寸 float disparity/灰度图；仅 save_data 时收集采样 disp/u/v；投影直接读取原 Q8.8 一次，表达式/运算次序保持；只在需要真实图像输出时做 RGB 转换；marker 按 visualize 开关；map-only 且不记录原数据时不等待无关 GDC；SHM metadata 队列每路限制最新一份。host 标量逐位回归证明投影数学，未给未经实测的提速比例。插件使用严格浮点编译；原产品 CMake 曾使用 fast-math，PCL/完整地图与原编译产物的逐点差异需在同一编译设置/输入下实测，不能凭源码宣称全部字节相等。

板端验收应使用同一固定 SHM/离线输入，依次验证 map-only、recharge-only、combined、auto-cali、offline；分别比较 det 有序结果/分割 mask、采样点坐标和 label/confidence、ground/nonground、GridMapInfo 字节和保存图尺寸。关闭 label_fusion 仍应出现 det_cloud；GDC 超时不得落盘旧帧或空指针；错误注入覆盖输入缺失、SDK native 错误、保存不可写、回充超时和任务失败后下游 skip。性能用同板同频同输入/开关采集至少稳定连续帧，对比 task/total wall、CPU、RSS/CMA，记录 p50/p95 和输出一致性。当前没有该板端测量证据。
