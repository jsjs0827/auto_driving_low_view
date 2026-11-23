# RAW 光照度分类算法

本功能沿用已验证的 `raw_hist_org_logistic` 模型与优化 RAW 特征内核，作为 V2 通用算法插件注册。它输出 `NOT_LOW_LIGHT`、`LOW_LIGHT`、`VERY_LOW_LIGHT` 三类及概率，不输出经过光度标定的 lux 数值。

```json
{
  "tasks": {
    "lux": {"feature": "raw_lux"},
    "publish": {"feature": "publish"}
  },
  "dag": {"sequence": ["lux", "publish"]}
}
```

该片段需要合入完整主配置，不能单独覆盖整份配置。算法参数在本目录 `config.json`；任务中的 `parameters` 可以覆盖 `include_features`。模型名固定为 `raw_hist_org_logistic`，元数据来源固定为 `org_proxy`，不静默降级到 raw-only 模型。插件和数值后端不依赖共享内存，可用于任何提供合规 RAW 输入的适配器。

输入端口 `raw` 默认绑定 `input.raw_light`，类型为公共 C POD `psdk_raw_stereo_input`，通过 `psdk_full_frame.raw_light` 传入。框架复制描述结构，像素仍借用调用方存储；必须保留到同步 `psdk_full_process` 返回。接口在 `sdk/include/perception_raw_input.h`。

每眼 RAW 要求：未旋转的 BGGR，宽 1088、高 1280、字节 stride 2176、连续 unpacked little-endian uint16 容器、10 位有效 DN，完整每眼 2785280 字节。保留原先“排除首个 uint16”与地面 ROI `[435,128,1033,1152)` 的规则。插件不做 resize、旋转、黑电平扣除、位移或 RAW8 归一化。

每眼对应的 ORG 元数据保留原 SDK `exp_time` 的毫秒单位和线性模拟增益；模型入口将毫秒乘以 1000 转为微秒。默认要求左右 RAW、各自 ORG，以及当前 DAG 左图的帧号与时间戳都非零且一致，不能用改写身份来伪造同步。相机 SDK 的跨流时间戳有已记录偏移时，可以显式采用下面的有界关联策略；左右 RAW 与左右 ORG 的各自配对始终保持精确匹配。模型仍标 `org_proxy`，不声称该参数已经验证为 RAW 实际曝光。

RAW/ORG 缺失、错帧、曝光无效、DN 越界或不支持的 profile 会产生独立算法结果，例如：

```json
{"algorithm":"raw_lux","valid":false,"status":"missing_input","reason":"RAW/ORG input artifact is absent"}
```

这些属于本算法的无效观测，任务仍成功返回，因此不会阻断同帧检测、分割或双目支路。输入描述结构本身存在 ABI 错误或越界存储时，框架仍按非法输入拒绝，避免读取不安全内存。

成功结果包含预测类别、三类概率、`model_source_sha256`、输入身份、元数据来源、训练范围诊断和 `timing_ms.feature/classification/total/lock_wait`。`include_features=true` 时还包含特征字典。结果自动进入既有 `psdk_full_result_get_algorithms_json` 与保存的 `result.json.algorithms`，不改变公共结果 view ABI。

现有 `psdk_full_frame` 采用尾部追加字段：保留 V1、V2、V3 旧输入尺寸及旧字段偏移，只有声明新完整尺寸的调用才读取 `raw_light`。使用新输入的调用方必须部署重新编译的 SDK。

数值后端来源、原文件 SHA 与仅 include 路径调整记录在 `backend/PROVENANCE.json`。AArch64 little-endian 自动使用 NEON4；其他架构使用原标量替代路径。编译该后端时保留 `-O3 -fno-fast-math -ffp-contract=fast`；模型系数与插值、直方图、百分位和分类算术未更改。

验证入口为 `validation/raw_input_test.cpp`、`validation/raw_lux_plugin_test.cpp`。后者使用已有左右 RAW 示例，与直接 `RawLuxRunner` 的特征和概率逐位比较，并验证缺 RAW、错 ORG、单位、非法 DN 和邻居任务继续执行。它不证明真实相机三类接口已经同步，也不证明单帧精度经过人工标注评测。

## 显式的跨流时间关联

默认参数为 `cross_stream_sync=exact_timestamps`、`max_cross_stream_skew_ns=0`，不容忍不同时间戳。硬件实测中，SDK 1.4.4 的同一公共 FrameId 可能在 RAW 与 YUV/ORG 中使用不同阶段的时间戳；例如现场一组原始 RAW 时间为 649478243955236 ns，ORG/YUV 为 649478233730111 ns，差 10.225125 ms。不能把 RAW 时间强行改为 YUV 时间。

仅在明确接受这种未验证关联、并依据现场记录选择上界时，设置：

```json
{
  "cross_stream_sync": "sdk_frame_id_bounded",
  "max_cross_stream_skew_ns": 15000000,
  "capture_frame_period_ns": 40000000
}
```

以上 15 ms / 40 ms 对应本次约 10.23 ms 偏移、25 fps 的现场试验，不是自动默认值。配置上界为 20 ms，且必须严格小于调用方明确声明的实际帧周期；未配置正上界/周期或错误配置会被拒绝。改变相机 FPS 时必须同步核实帧周期。传入较大周期不能替代现场证据。

规则只对 RAW↔YUV、ORG↔YUV、每眼 RAW↔ORG 生效：各流必须使用相同非零公共 FrameId、非零原始 timestamp，且每一项绝对时间差都不超过配置上界。左右 RAW 的 FrameId/timestamp 必须精确一致，左右 ORG 同样必须精确一致；不会对左右双目图像引入容差。不同 FrameId 即使时间很近也不能匹配，不按最近时间自动拼帧。

SDK 的公共 FrameId 存在底层 counter 分组的实现，因此同公共 ID 加时间窗仍不是同物理曝光的证明。输出 `capture_association.association=bounded_unverified`、`cross_stream_sync_verified=false`，保留 RAW/ORG/YUV 的全部原始身份、带符号 `observed_skew_ns`、policy/max/period/provenance。无论策略如何，`metadata_source=org_proxy` 与 `stereo_same_exposure_verified=false` 保持不变；不会因有界关联声称曝光已经验证。

RAW profile 仍固定 16 位 little-endian 容器、10 位有效 DN。驱动配置文件名包含 raw8 不能替代 buffer 证据：应核实宽高、每眼 2785280 字节、stride2176、DN 范围、低位分布和 Bayer/旋转文档。原样例 DN 最大值 1023，含大量大于 255 和低两位非零的值；禁止未经确认进行位移或归一化。现场 DN 范围可作为输入域诊断，不能单独证明 Bayer 排列、传感器物理位深或光照度精度。

新增验证 `validation/raw_lux_skew_test.cpp` 覆盖默认严格模式拒绝偏移、显式同 ID 有界关联、原始时间戳不变、概率位不变、左右 RAW/ORG 保持精确、不同 ID/过界/错误周期拒绝，以及各跨流边的独立上界检查。
