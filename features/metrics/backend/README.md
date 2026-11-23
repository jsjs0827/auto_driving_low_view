# BPU 利用率采样

`bpu_metrics.hpp` / `bpu_metrics.cpp` 提供 `r29::BpuSampler`，由比较 runner
在预热结束后调用 `begin()`，测量结束后调用 `end()`。

```cpp
r29::BpuSamplerOptions options;
options.sample_interval_ms = 100;
options.paths = {"/sys/devices/system/bpu/bpu0/ratio"};
r29::BpuSampler bpu(options);
// 先完成模型初始化和预热。
bpu.begin("bpu_samples.jsonl");
// 执行本组需要计入结果的帧。
bpu.end();
const std::string group_summary = bpu.summaryJson();
```

默认只读取 X5 的 `bpu0/ratio`。显式配置多个路径时，第一个路径提供顶层结果，
`paths` 保留每个路径的独立统计；不会把总线 ratio 与核心 ratio 相加或平均。

格式依据是 D-Robotics 官方 X5 内核中的
[bpu_debug.c](https://github.com/D-Robotics/x5-kernel/blob/main/drivers/soc/hobot/bpu/bpu_debug.c)：
`bpu_core_ratio_show` / `bpu_ratio_show` 输出十进制整数加换行。
实现严格解析完整的 `0` 至 `100` 整数文本，可以有首尾空白。错误文字、多个数值、
百分号、负数、超过 100 的数字均为 `invalid_format`，不会从错误消息中提取数字。
文件不能读取为 `unreadable`，读取失败为 `read_error`，超过 4096 字节为 `too_large`。
缺失值写作 JSON `null`；实际读到 `0` 时保留数值 `0`。

`begin()` 和 `end()` 各记录一次边界观测，中间由后台线程周期读取。
JSONL 在开始测量前创建，在停止采样后统一写入，减少测量期间的磁盘干扰。
原始观测暂存内存，适用于有界的性能比较任务；运行时间越长，占用的内存越多。
复用同一对象再次 `begin()` 会清空上一组数据。调用者负责串行调用 `begin/end`，
`summaryJson/latestJson` 可以与采样线程并发调用。

摘要包含均值、P95（排序后的线性插值）、最大值、总观测数、有效观测数和缺失数。
统计是观测值的算术均值，未进行时间加权。`latestJson()` 是最近一次采样及其年龄，
不能解释为某一帧自身的 BPU 利用率。

`scope=device_shared` 表明读数属于设备共享 BPU，其他进程使用 BPU 也会影响它。
开始测量不会重置内核计数器。预热帧不进入采样器观测列表，但驱动内部的统计窗口
可能仍覆盖更早的工作，所以输出同时注明 `kernel_window=driver_defined_not_reset`
和 `warmup_filter=observations_after_begin_only`。它不提供按模型划分的 BPU 时间。

新增测试：`validation/sdk/tests/bpu_metrics_tests.cpp`；CTest 名称为
`bpu_ratio_sampling_and_missing_data_contract`。
