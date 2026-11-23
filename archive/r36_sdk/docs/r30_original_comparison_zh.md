# r12-v30：原始项目对照与业务接入报告

本轮以 `point-cloud-processing-main-ini/point-cloud-processing-main` 为原始基准，对照 r29，生成独立 r30。结论：主要算法在 r29 已迁入；本轮补的是外围业务执行、错误处理和配置兼容，不能把“源码已接入”直接当成“完整产品已在 X5 验收”。

保留原 640×544 NV12 地图输入、回充所选 GDC 输入、864×480 检测 letterbox、832×480 分割直接缩放、原模型及前后处理、单帧和无双缓冲。未用 VSE 或融合 BIN 替代原流程。

## 1. 逐项比较

| 功能 | 原项目 | r29 状态 | r30 处理 |
|---|---|---|---|
| 检测、分割 | 原 Horizon runner，独立模型开关，可串行/并行 | 精确 Feature 已接入；产品适配却要求两个模型，忽略串行开关 | 修复独立 det/seg 加载、按请求选 lane、遵循 `parallel_det_seg` |
| 精确前处理 | 原 NV12/I420 缩放、检测补边 | 已验证精确缩放、紧凑表、直接模型输入写入 | 保留；只修改 adapter 启用检查，不改数值计算体 |
| 检测/分割后处理 | 检测解码/NMS；原完整尺寸 mask | 已保留原解码、插值/ArgMax 与优化 | 保留，未切换近似路径 |
| 相机输入 | daye SHM RGBD/GDC，CameraParam | 实际 ABI、最新 metadata 和模式选择已保留 | 参数验收/更新/保存纳入事件图；SHM 接收仍为输入适配器 |
| 视差转点云 | Q8.8、原内参回投影与 FOV | `product.pointcloud.from_disparity` | 保留原算法和按需构造中间数据 |
| 自动标定 | 原 `autocali`、外参保存 | `product.calibration.auto` | 保留 |
| 检测点云/语义融合 | 原框匹配、语义形态学、深度/体素/射线过滤 | `product.fusion.semantic_labels` | 保留原 label_fusion，不替换为通用近似算法 |
| 坐标变换/着色/TF | 原 `postprocess` 等 | `product.pointcloud.transform_color` | 保留 |
| 地面分割 | 原 PCL + PatchWork++ | `product.pointcloud.patchwork` | 保留真实原库算法 |
| 地图融合 | 原 AiFusion、孔洞/高度/噪声处理 | `product.fusion.ai_fusion` | 保留 |
| 地图/ROS 输出 | GridMapInfo、marker、TF、点云、图像 | `product.output.ros_maps` 及原业务 Feature 内发布 | 保留原消息和发布路径 |
| 回充识别提交 | YOLO 辅助 ROI、静态 ROI、SDK 异步识别 | `product.recharge_mark` 已接入提交/等待 | 保留；回调发布新增独立事件 Feature，关联实际所选源帧 |
| 回充结果发布 | SDK 回调直接发布 | 图外执行 | 新增 `product.output.recharge`，异常保留 native_code，通知等待者 |
| 命令控制 | 服务命令 1/2/3/21 | 原回调在图外 | 新增 `product.control.command`；保留协议；未知命令/不可用回充明确拒绝 |
| CameraParam | 回调赋值、可选写 JSON | 图外执行 | 新增参数验收和保存两个 Feature，保存依赖验收成功 |
| 数据记录 | 最新槽入队，后台保存/文件保留 | `product.output.record_latest` 只包含入队；实际 I/O 图外 | 新增 `product.output.record_io` 执行真实保存及保留，失败 2002 |
| 离线回放 | JPG + RAW 分别排序后配对，原几何回放 | 原读取/交互流程在图外 | 新增 `product.input.offline`；严格同 stem 配对、尺寸检查、UTF-8 文件读取、支持 `n 帧号` |
| 配置 | 大型 start_config JSON | 简化 JSON 冻结并映射原配置 | 增加离线目录和事件图 profile；继续原 JSON parser/scheduler |
| x86 CPU 推理 | 无可直接运行的这套 CPU 后端 | r29 增加真实 ONNX Runtime CPU | 保留并复测独立检测/分割入口；完整 ROS 产品插件尚非 CPU 版 |
| 错误码 | 多处日志/返回值分散 | 已有完整 Status/C API | 补齐实际 I/O、命令、输入错误、回调和 worker 显式关闭错误 |

`product/BASELINE_PROVENANCE.json` 保存原文件 SHA256。本轮补回原字节 `MoTypes.h` 和 `MoCommon.h`，不再把这两个可恢复文件误列为外部缺失依赖。

## 2. 框架如何组织

每帧主业务仍由原 `TaskGraph` 执行 11 个产品 Feature：AI、回充提交、RGB、点云、标定、语义标签、变换、地面、地图、发布、记录入队。检测/分割内部仍走原 Engine 的精确 DAG；同一帧内可并行，下一帧不会排成 AI 双缓冲。

新增 6 个事件 Feature，由同一个原 FeatureRegistry/TaskGraph 实现：

| 事件 | Feature | 执行内容 |
|---|---|---|
| command | `product.control.command` | 原命令执行及失败状态 |
| camera_params | `product.input.camera_params` | 检查内参/几何并更新 |
| camera_params | `product.output.camera_params` | 按原开关保存 JSON |
| record_io | `product.output.record_io` | 原后台实际文件写入及保留 |
| recharge_result | `product.output.recharge` | 原 SDK 回调结果发布 |
| offline_read | `product.input.offline` | 文件索引、同帧配对、读取/解码 |

事件 Feature 在原回调或原后台线程中同步等待图执行完成。没有增加新的图像缓存或 AI 帧队列。参数保存依赖参数验收；同类事件串行，不同事件图独立，避免主图等待回充回调、回调又等待主图同一锁的循环。

这里的“17 个产品 Feature”是 11+6，不能与框架自带的 portable Feature 或 AI 内部八个 Feature 重复计数。源码中每个绑定调用真实业务代码；host 编排测试中的替身只用于测试，不会打包成生产 SDK。

## 3. 本轮纠正的行为差异

1. 原 runner 可以只启用检测或分割。r29 产品 adapter 错误地强制两模型同时加载，r30 恢复原语义；回充不需要 mask 时不执行 seg，地图按启用项执行。
2. 原配置允许 `parallel_det_seg=false`。r30 选择串行 profile，双模型开启并行时继续并行；数值计算不变。
3. 非法图像或 stride 不再静默返回后继续发布旧结果。返回 1002，清空本次结果；后续有效帧能恢复。
4. 回充推理、提交和结果身份使用实际选中的 PicData；combined 模式也不会把 GDC 回充结果标成 RGB 图像的帧号。
5. 启动命令失败恢复本地原模式，只停止本次新启动的 worker；原错误和 native_code 不被回滚错误覆盖。停止命令已有外部副作用时保持停止状态，在详情中说明后续通知失败，不假装已撤销。
6. 实际保存失败与“入队成功”分别记录。最新槽覆盖和原关闭策略继续保留，未承诺每帧都落盘。
7. 原离线按排序位置配对可能将错帧融合。r30 在执行前拒绝遗漏、重复 stem 或错误 RAW 大小。原几何处理和 OpenCV 颜色转换/缩放保持；未自动增加离线 AI 分支。
8. 精确 worker 显式调用 `tryClose`，模型清理失败可返回 2006/native_code，避免析构吞掉错误后仍写成功。完整产品宿主析构的错误传播尚未全面验证。

`R30_STAGE_ENABLE_COMPAT.json` 列出 runner adapter 七处精确、可逆的启用检查修改；反向还原后必须恢复 r29 整个 runner SHA256。历史数值清单和 53 个原框架文件不修改。

## 4. 仍需完成或验收的部分

| 项目 | 当前状态 | 下一步 |
|---|---|---|
| 全量 native 插件 | 真实 ROS/PCL/原 SDK 源码已接入，当前机器不能完成其编译 | 使用原 Ubuntu 可编译的 daye/common_interface 头及匹配回充库 |
| 原地图和点云完整一致性 | 标量投影/编排已测；PCL/ROS/地图字节尚未板测 | 固定同一 RGBD/GDC 输入，逐层保存点云、标签、ground/nonground、地图 |
| 原配置预留字段 | `fuse_into_grid`、`min_semantic_points_per_cell` 等原项目本身未实际应用 | 明确定义语义后另做功能，不能算作本轮已经生效 |
| `use_gdc_image` / `max_points` | 原项目部分仅解析/记录 | 需要专门实现和回归；本轮未改变点云来源或抽样策略 |
| 关闭 PatchWork 的地图策略 | 原项目没有完整的地面/障碍替代语义 | 应先明确停发地图还是用其他分类，不能擅自把全部点当障碍 |
| 可视化细粒度 Task | 原 overlay、TF、marker、点云发布保留，部分仍嵌在业务 Feature 内 | 可进一步拆分和分别计时；现有功能没有因未单独拆 Task 而丢失 |
| 输入所有权 | SHM 像素仍遵循原 ABI 生命周期；业务多为帧内闭包 | 可逐步增加有类型、可持有的结果 artifact；当前不声称所有业务数据都已独立拥有 |
| 视差产物 | 原相机/产品点云分支使用真实视差；standalone `stereo.demand` 仅有元数据 | 不可把该元数据 Feature 当成完整可持有的视差图输出 |
| 全产品 x86 | ONNX CPU 的独立 AI DAG 已支持 | ROS/PCL/daye/recharge 依赖与后端仍需单独移植；不能只开 ORT 就宣称全部产品 CPU 化 |
| 稳定性/CMA/重开 | 本轮没有 X5 新长测 | 同板进行 map/recharge/combined、离线、长测和重开验收 |

本机预检发现五个 daye/common_interface 头为受保护二进制内容；真实 AArch64 base_framework 库存在。现有 `charing-apriltag` 回充头与保留 API 不同，不能直接用不匹配的函数表调用。详情见 `validation/r30_local_dependency_inventory.json`。没有生成假 ABI 头来绕过这些问题。

## 5. 指标及验证范围

主业务 `product_metrics.csv` 包含 run_id、frame_id、timestamp、task 状态/耗时、整图 wall/process CPU ms、RSS KiB 和 CMA free KiB；`product_status.jsonl` 保留完整错误码。新增 `product_event_metrics.csv` / `product_event_status.jsonl` 包含 event_id、parent_run_id 和事件源帧。离线读发生在主图分配 run_id 前，parent 为 0，使用 event_id 与 frame_id 追踪，不能伪造主图归属。

CPU 使用进程用户态＋内核态时间；Windows RSS 使用 WorkingSet，Linux 使用 VmRSS。单逻辑核为 100%，并行可超过 100%。Windows 无 CMA 时保持不可用。整图指标在 task 行重复，必须按 run_id/event_id 去重；并发事件的进程 CPU 时间可能覆盖同一时间段，不能把事件 CPU 值相加当作总 CPU。

本轮 29 项 C++ CTest、34 项配置测试、4 项依赖预检测试、8 项 CPU CLI 检查均通过。验证记录见 `validation/r30_host_validation.json` 和 `validation/r30_cpu_cli_validation.json`。host 测试覆盖真实原框架编排、事件实际文件 I/O、完整错误、回调隔离、单模型/串并行适配、原缩放和后处理、真实 ONNX CPU 同帧路径。Horizon API fixture 测试不执行 BPU；完整 ROS/PCL 插件和 X5 性能仍未测。

本轮主要是补齐功能边界，没有同板受控性能数据，不能给 r30 相对原项目或 r29 的 FPS、CPU、RSS 提升百分比。此前 AI 精确缩放的板端结论也不能自动外推成加入全部点云/地图/保存后的产品性能。

## 6. 后续顺序

先完成真实全插件编译与同帧正确性，再测性能：map-only、recharge-only、combined、auto-cali、offline；随后覆盖 det-only、seg-only、双模型串/并行以及保存/可视化开关。比较原模型实际输入、检测结果、mask、点云坐标/标签和地图消息；只有一致性通过后才比较 FPS、p50/p95、CPU、RSS/CMA 和回充延迟。最后做长测与相机重开。

当前建议继续以原精确 CPU 前处理＋直接输入写入作为默认，原 PCL/PatchWork++/AiFusion 作为产品算法。优先补真实依赖和完整业务验收，随后依据每个 Task 的实测耗时优化其他 Feature；不要为了表面统一或速度更换输入、模型或数值流程。
