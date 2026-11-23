# 最小功能替换与新旧 SO 对照报告

日期：2026-11-03。

**当前状态：源码最小融合完成，算法库 Docker 交叉编译和 X5 模型对照完成；完整 ROS 节点 `libpercep_main.so` 尚未生成，也未在 X5 启动。** 完整节点实际进入源码编译后，停在缺少 `common_interface/msg/camera_param.hpp`；还需要匹配本源码的 PicData、CameraGdcData 和 CameraData 共享内存布局。状态见 `BUILD_STATUS.json`，实际日志见 `evidence/build/full-node-build.log`。本报告不把算法库测试写成完整节点/整帧测试。

## 1. 最小融合范围

以 `/home/st/ini_code/pcl_Ini/point-cloud-processing-main/` 为工程基线，在独立副本 `project/point-cloud-processing-main/` 集成优化 SDK。经逐文件明文比较，只修改4个已有文件：

| 文件 | 修改用途 |
|---|---|
| `CMakeLists.txt` | 以 DetSegCloud::Adapter 替换原 dnn_sdk/percep_dnn 链接；保留边界校验所需浮点选项；限定实际使用PCL组件，避免额外GUI依赖 |
| `include/percep_main/percep_main.h` | 引入适配器头、替换私有runner类型 |
| `src/dnn_runtime.cpp` | 在原 runDnnInference、matchAndPublishDetCloud 内接入检测/分割和点云SDK |
| `src/percep_main.cpp` | 离线线程先结束，再销毁模型句柄 |

新增与 `src/` 同级的 `det_seg_cloud_sdk_x5/` 为精简二进制 SDK，包含头文件、src中的两份SO、必要Horizon运行库、配置示例和CMake导入。适配器已封装到SO，不再编译 adapter.cpp。

原两个函数的名称、参数、默认参数及调用位置一致；`percep_compute.cpp`、`label_fusion.cpp`、模型及 `config/start_config.json` 完全不改。原工程目录本身未被覆盖。详见 `CHANGES.json`、`MINIMAL_CHANGES.patch` 和 `validation/check_original_entrypoints.py`。

## 2. 新旧基线身份

| 对象 | 身份 |
|---|---|
| `run/engineering/libpercep_dnn.so` | 当前原工程实际使用的旧算法SO，原样保留，SHA `2843ad9ee16fdb7e949d61b3a5e7170ca62315dde5bdccc74d30311e7c18029b` |
| `run/engineering/libpercep_engineering_baseline.so` | 仅为上述旧SO增加C API测试桥，旧算法二进制未重新编译/修改 |
| `run/original/libpercep_original_baseline.so` | 从工程留存原始runner源码的计时快照重新编译的对照库；源 `src/horizon_dnn_runner.cpp` SHA为 `cce3184d859e3fe881917d2e7e542f6d95b8bb054983028e6e78ce516cc2306b`，只加计时和接口转换，不是完整ROS节点 |
| `run/new/libdet_seg_cloud.so` | 优化后检测/分割/点云核心库，SHA `e226b652bb492b744ea6853140a5fa57936e085060a27e9b26fffd15c48a3c19` |
| `run/new/libdet_seg_cloud_adapter.so` | 原适配器库，SHA `ae99821d7687e54618ae78b55dcf1d4f96d0136afc983631a960150c805e11ec`，本轮未改 |

此前 `4fab71…` 是中间SDK基线，本次没有把它当成原工程旧SO。另找到 `/home/st/reconstract_code/point-cloud-processing-main.zip`，但它是不同的PercepDemo/24类版本，不能与本轮当前工程混用；历史ZIP路径仅作原快照来源记录，本轮未重新核验那个历史归档。来源证据见 `validation/ORIGINAL_BASELINE_PROVENANCE.json`、`ENGINEERING_BASELINE_PROVENANCE.json`。

## 3. 构建情况

使用现有 Docker 镜像 `pc_tros_ubuntu22.04:v1.0.0`、GCC 11.4.0 AArch64交叉编译器及 `sysroot_docker`。

- 优化核心库本轮 clean rebuild 成功，SHA仍为e226，和实际跑测/工程引用的库一致。
- 留存原始runner对照SO、现有工程旧SO测试桥、对比工具及融合接口回归程序，均在Docker交叉编译成功。
- 完整节点 CMake配置成功并进入源文件编译；已显式指定真实daye第三方目录，spdlog_wrapper缺项已解决。当前首个阻塞是 `common_interface/msg/camera_param.hpp`。
- 本地和X5现有common_interface未找到该消息定义/生成头；本地共享内存相机头是旧布局。需要与当前源码匹配的common_interface源/安装工作区，不能从旧结构体推定新ABI。
- 按此前用户指示，验证构建关闭recharge（`PERCEP_ENABLE_RECHARGE=OFF`），原生产默认选项未改。

Docker脚本在 `scripts/build_full_node_in_docker.sh`；可通过 X5_SYSROOT、THIRDPARTY_ROOT、COMMON_INTERFACE_ROOT、COMMON_INTERFACE_INSTALL、ROS_PREFIX 指定实际依赖。补齐依赖后还需完成全部编译/链接和节点运行验证，不能仅凭本次配置成功认定全工程可运行。

## 4. X5 对照条件与计时边界

两组独立对照，各100张不同640×544 NV12图片，分别做3轮，共300次稳态配对。每轮两版本各预热2次，调用顺序逐帧交替，第二轮反转。冷首帧独立记录，未混入稳态。每个API地址通过dladdr核对实际SO归属，避免误加载同一库。

统一使用当前工程的 `mmyolo1010.bin`（34类、864×480输入、letterbox、score0.5、NMS0.7、top-k300）与 `pidnet_1020_3classes.bin`（3类、832×480输入、direct resize、bilinear、align_corners=true）。模型文件与原工程校验值相同；不把换模型混入实现对比。

图像先读入内存；wall包含算法库调用、测试桥和结果转换/复制/释放，写JSON不计入；旧版测试桥转换开销没有单独扣除。因此wall是本对照边界的实际调用耗时，不是旧节点函数的纯算法周期。前处理、BPU、后处理由各实现内部计时。BPU包括提交、排队等待与释放，并非纯硬件周期。两模型并行，六项不能简单相加。

不包含相机采集、双目、点云融合、ROS发布、overlay、保存或整帧处理。X5测试在独立目录加载模型库，完整ROS节点并未启动。

## 5. 速度：当前工程实际旧 SO → 新 SO（主要对照）

单位ms，300次稳态配对均值：

| 阶段 | 工程实际旧SO | 新SO |
|---|---:|---:|
| 检测前处理 | 15.994 | 3.223 |
| 检测 BPU 提交/等待/释放 | 13.776 | 13.764 |
| 检测后处理 | 2.691 | 2.649 |
| 分割前处理 | 23.019 | 4.534 |
| 分割 BPU 提交/等待/释放 | 11.382 | 17.178 |
| 分割后处理 | 2.257 | 2.085 |
| 两模型实际调用 wall | 37.071 | 24.003 |

调用均值 **37.071 → 24.003ms**，减少 **35.25%**。P95 **37.571 → 24.269ms**；最大值 **38.145 → 24.931ms**。

## 6. 速度：留存原始 runner 源码 → 新 SO（补充对照）

| 阶段 | 留存原源码对照SO | 新SO |
|---|---:|---:|
| 检测前处理 | 15.978 | 3.226 |
| 检测 BPU 提交/等待/释放 | 13.781 | 13.741 |
| 检测后处理 | 2.691 | 2.651 |
| 分割前处理 | 22.969 | 4.524 |
| 分割 BPU 提交/等待/释放 | 11.409 | 17.179 |
| 分割后处理 | 6.143 | 2.073 |
| 两模型实际调用 wall | 40.939 | 23.977 |

调用均值 **40.939 → 23.977ms**，减少 **41.43%**。P95 **41.402 → 24.233ms**；最大值 **41.854 → 24.523ms**。

两组不能合并成一个“旧版”。主要收益来自精确NEON前处理、坐标/权重缓存、减少中间复制及分割角点复用。分割BPU等待项增多，与前处理更早结束后排队时序变化一致；未用硬件profiler拆开纯核周期，不称BPU模型核加速。

## 7. 冷帧与输入切换

| 组别 | 旧版冷首帧ms | 新版冷首帧ms |
|---|---:|---:|
| 留存原源码 | 42.062–43.659 | 337.968–339.882 |
| 工程实际旧SO | 37.746–39.443 | 338.096–339.019 |

新版首次配置需要校准和完整字节验收，因此冷帧更慢。尺寸、stride、格式等切换也可能重新校准，不能把24ms稳态结论套用到频繁切换配置的场景。建议用实际输入参数预热1–2帧后进入实时循环。初始化耗时另存原始JSON，不据同进程先后加载模型的缓存差异声称初始化加速。

## 8. 精度：输出一致性，而非标注准确率

两组均为：

- 分割mask **300/300配对完全一致，差异像素0**。
- 检测框数量、类别ID/名称及分数完全一致，最大分数差0。
- 检测数值严格bit一致 **288/300配对**；每轮相同4帧的单个坐标存在浮点末位差，最大 **0.000030517578125像素**。
- 按 `1e-4` 像素容差，**300/300配对一致**；没有通过取整输出或重排检测框隐藏差异。
- 差异来自帧48的y2、54的y2、73的y1、87的y2；详细原值、bit和坐标差保存在两组aggregate JSON。
- 100张不同图片中49张有检测，共88框；三轮是重复测量，不是300张独立图片。两版本各自跨轮输出稳定。

这些数字不是mAP、召回率或mIoU；没有人工标注，不能据此宣称所有场景精度绝对不退化。

此外，本轮直接录制原留存runner与新库真实提交给 `hbDNNInfer` 的输入：100稳态帧加2次预热、两个模型共204对张量，**124,554,240字节及shape/stride/padding完全一致**。这一步验证letterbox在实际两份二进制中的输入等价，诊断计时有写盘污染，不用于任何性能结论。

## 9. 点云融合与完整节点范围

新SO的扩展融合接口已在X5通过合成回归，覆盖34类含高类别ID、原u/v坐标、box-only、参数校验和稀疏深度筛选等；日志见 `validation/cloud-ex-checks-x5.log`。原两个应用入口及调用方通过接口检查。

**未完成真实点云逐点对比、完整ROS节点链接/启动、相机在线运行、整帧耗时及端到端精度测试。** 缺失匹配相机接口是实际阻塞。不能把上述合成接口检查写成完整matchAndPublishDetCloud节点链路通过。

## 10. 运行与交付

- `project/point-cloud-processing-main/`：完整最小融合源工程，nested SDK保持头文件＋SO形式。
- `run/new/`：新核心和适配器SO；`run/engineering/`：原样实际旧SO及测试桥；`run/original/`：留存源码对照SO。
- `run/data/`：本次100图及相同模型，`run/runtime/`：同一套Horizon运行库。
- `run/bin/run_all_comparisons.sh`：X5独立模型复测；退出状态3表示存在严格浮点差异，应检查容差/JSON，不表示进程崩溃。
- `results_engineering/`、`results_original_source/`：本轮完整三轮逐帧数据；`validation/*aggregate.json`：独立汇总。
- `evidence/build/`：实际Docker构建记录，含完整节点失败日志；`BUILD_STATUS.json`明确区分已完成与待依赖项目。

SDK运行日志仍有原模型构建HBRT3.15.58与运行时HBRT3.15.55版本告警；两版对照使用相同DNN1.24.5/HBRT3.15.55，本轮未换运行库或模型。正式节点部署时应统一匹配版本并复验。
