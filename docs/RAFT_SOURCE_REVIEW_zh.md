> **历史材料（v1/v2）**：本文保留用于理解原算法与历史验证。v3 的配置、路径、构建及运行入口已经改为纯 C++；本文旧命令和旧 DAG 配置不能直接用于本版。请以 [09_纯CPP输入与Graph迁移](09_纯CPP输入与Graph迁移.md) 为准。

# RAFT runner 源码审查与 v8p1 接口边界

本说明记录 2026-10-09 对用户提供的可读 C++ 源码及本地 Horizon API 头文件的核对结果。已发布 v8 保持原样；v8p1 将可构建的 runner 源码与 Python 几何处理分开。源码阅读不代表原 RAFT 模型已经在 X5 上完成推理或精度验收。

## 审查依据与来源

用户原附件路径：

```text
C:/Users/Administrator/.codex/attachments/41ee0a67-b690-4f04-a061-529e72a21c95/已粘贴的文本.txt
```

下文“原源码行号”均指该附件的 459 行内容。原附件字节哈希、换行归一化哈希及随包文件映射由来源清单记录；本说明不替代来源清单。

另实际读取了本工作区的 API 头文件：

```text
D:/projects/python_pro/gpt_pro/reconstract_code/tmp/r21-real-api-headers/dnn/hb_dnn.h
```

该头文件声明版本 `1.24.5`，含 `validShape/alignedShape/tensorLayout/quantizeAxis/stride` 以及 SCALE 零点字段。这里对枚举和公式的判断限于这份实际读取的 API 定义；板端构建仍须使用与板端 runtime 匹配的 SDK。

**可读附件不能证明现有外部 ELF 就由这份源码编译。** 文件名相同、CLI 字符串相似或模型能加载，都不能替代源码、构建环境、命令和产物哈希的对应记录。v8p1 新构建的 runner 应作为新产物记录，不能追溯性地宣称旧 ELF 已通过源码审计。

## 原 runner 的实际 CLI 与配对

原源码第 33–46、282–317 行定义以下接口：

| 参数 | 实际作用 |
| --- | --- |
| `--model_file` | 必填，加载 Horizon `.bin` 模型 |
| `--left`、`--right` | 单对图像，二者都需填写 |
| `--left_dir`、`--right_dir` | 批量模式，与单对模式互斥 |
| `--output` | 单对 NPY 路径，默认 `disparity.npy` |
| `--output_dir` | 批量 NPY 目录；默认在左目录名后加 `_result_disparity` |
| `--output_png_dir` | 可选 JET 视差图片，不是定量深度 |
| `--gt_dir` | 可选浮点视差 GT，仅用于评测 |
| `--calibration` | 可选 colon 配置，仅用于后面的 depth PNG 计算 |
| `--restore_original_resolution` | 默认 false；仅影响 depth PNG 分支，不修改输出 NPY |
| `--depth_min`、`--depth_max` | 声明单位米，但旧实现比较时存在单位错误 |

原图像扫描支持 jpg/jpeg/png/bmp/webp，先按完整文件名查找右图，找不到时按 stem 搜索第一个匹配项；缺右图只警告并跳过，重复 stem 没有消歧检查（第 59–79、292–305 行）。

v8p1 的 Python 外层先要求左右目录精确 stem 一一对应，重复 stem 或任一侧缺失立即失败；随后生成同名无损 PNG。这样外层不会继承原 runner 静默跳过或任取重复项的行为。PNG 输入受原 runner 明确支持。外层继续检查预测 stem 集合与选中图像集合完全相同。

## 原预处理：必须看分支，不能只读头注释

原源码第 183–239 行实际执行 `imread`、resize、颜色转换和张量写入。**没有 stereoRectify、去畸变 map 或 remap。** 传入 `--calibration` 也不会校正左右图。

| 输入分支 | 源码实际行为 | 边界 |
| --- | --- | --- |
| 数值 `tensorType==1` | BGR resize → OpenCV I420 → 单 buffer Y + UV | 与读取的头文件中 NV12 枚举一致 |
| 数值 `tensorType==2` | 单 buffer Y + VU，代码假设这是 NV21 | 与读取的头文件矛盾，见下面缺陷表 |
| `HB_DNN_IMG_TYPE_RGB/BGR` | BGR→RGB，resize，再写 uint8 RGB 平面 | 两个枚举都写成 RGB；不能据此声称 BGR 模型已正确支持 |
| 其他类型 fallback | 写 float32 RGB 平面，值为 0–255，不做 `/255` | 没有限定必须是 F32；不能视为通用数值张量适配 |

第 3–6 行头注释声称 float32 输入，但第 209–226 行 RGB 图像分支明确写 **uint8**，并说明归一化由模型内预处理完成。v8p1 文档应记录实际支持的张量类型及模型预处理约定；不能混写“统一 float 输入”“统一 NV12 输入”或对未知模型自动猜测。

原源码第 348–349 行固定按 NCHW 的第 2/3 维取输入高宽，第二路也使用第一路的尺寸；只有输出侧第 382 行针对 NHWC 调整高宽。因此输入布局、左右 shape 一致性及内存布局必须由新 runner 显式核对。

## 原 NPY、视差单位与 restore

原源码第 82–97、381–390 行将模型输出保存为二维 `HxW`、little-endian `<f4`、C 顺序 NPY。负值先置 0；没有把结果量化为 Q8，也没有把视差换算成米或毫米。NaN/Inf 没有在这里主动清理。

对本项目声明的 640 宽 RAFT 输出，值为横向像素视差。原评测分支也使用 `GT宽度/预测宽度` 恢复视差尺度（第 250–254 行）。具体模型仍须证明自己的视差值所对应的像素画幅；数组分辨率本身不能证明一个任意模型的视差单位。

`--restore_original_resolution` 的名字容易误导：NPY 已在第 390 行保存；第 406–408 行的 resize 和宽度倍率仅作用于临时 `disp_for_depth`。该开关不会让 NPY 变成 1280×1088。v8p1 不依赖这条旧 depth 导出分支。

## 已确认缺陷及影响条件

| 原源码位置 | 已观察到的错误 | 影响条件与 v8p1 处理边界 |
| --- | --- | --- |
| 406–418 | restore 把视差放大到 1280×1088，却仍用原 `h,w` 分配 `depth_m`；循环按放大画幅写入 | 输出画幅小于放大画幅时可能越界。深度应由 Python 在一致的目标画幅计算 |
| 416–417 | `fx*baseline_m/d` 得到米，却与 `depth_min/max*1000` 比较 | 默认 0.1–30m 被当成 100–30000m，正常深度大量清零。Python 保持米单位 |
| 188–203 | 将数值类型 2 当成 NV21/VU 单 buffer | 本地 1.24.5 头文件第 59–65 行定义类型 2 是 `HB_DNN_IMG_TYPE_NV12_SEPARATE`，不是 NV21；必须按真实 API 处理或明确拒绝 |
| 207–224 | RGB、BGR 图像枚举均填 RGB 平面 | 若实际模型声明 BGR，通道语义不一致；新实现必须区分或拒绝 |
| 228–237 | 未限制 tensorType 的 fallback 强制按 float 指针写 | 对 U8/S8 等非 F32 张量可能写错格式，且写入大小可能超过其存储；必须限制支持范围 |
| 337、348–349 | 改写 alignedShape，并固定按 NCHW 解释输入 | 不能证明 padded/strided 或 NHWC 输入正确；新实现应校验真实属性并拒绝未支持组合 |
| 148–169、384 | 输出按扁平连续索引读取，未处理实际 stride/量化轴；SCALE 没有减零点 | 本地 API 头第 93–107 行明确要求有零点时 `(raw-zeroPoint)*scale`，第 124/126 行提供 quantizeAxis/stride。相关模型输出可能错误，Python 几何无法修复错误视差 |
| 130–145、400–411 | 配置只做宽松数值解析，depth 只取原始 left.fx 和 `abs(Tx)`；宽度固定假设 1280 | 不等价于完整双目标定/Q，不能作为已校正点云的内参来源 |

上述是源码和实际头文件可直接确认的条件性问题，不表示每个模型都会命中全部分支。已有两个结果目录的内容、旧 ELF 与源码的对应关系、候选标定是否属于该相机，仍分别需要来源证据。

## v8p1 的统一接口与 P/Q8 后处理

新 runner 的接口边界是：读取左右两幅 prepared 图像，执行两输入模型，导出单通道浮点像素视差 NPY；输出宽度与模型输入宽度一致。张量类型、布局、尺寸、步长及量化组合应显式验证，未支持的组合报错。runner 不承担标定校正、米制深度、PLY 或 ROS 发布。

已逐行核对新后端的 `stereo/runner/src/horizon_stereo.cpp`、`tensor_codec.cpp` 及 `include/r36_stereo/horizon_stereo.hpp`、`tensor_codec.hpp`，实际实现边界如下：

- 模型类只接受与模型输入宽高完全一致的 `CV_8UC3 BGR`，内部不隐藏 resize；尺寸转换由调用层显式完成。要求单模型包、两输入、一输出，左右输入名称须能无歧义地识别 left/right，左右宽高相同。
- 支持四维 NCHW/NHWC、有效和 aligned batch 均为 1，保留原 aligned 属性。数值张量只接受标准 dense aligned byte stride 或零/缺省 stride；不支持的 stride 明确报错。大小乘法及分配限制在 512MiB 内。
- 输入支持 RGB/BGR uint8、RGB F32 0–255 和 NV12/NV12_SEPARATE；输入量化不猜测。separate NV12 分开分配 Y/UV，保持 UV 顺序，并填充 limited-range 黑色/中性 chroma。NV12 要求零/缺省 stride、偶数有效宽高、aligned 宽度为 16 的倍数、aligned 高度为偶数，无 aligned channel padding。
- 输出要求有效 C=1、W 等于模型输入宽度，H 可以不同；支持 F32/S8/U8/S16/U16/S32。量化仅支持 NONE、单个 per-tensor SCALE（可有单个零点）或 SHIFT；多元素 per-axis 元数据报错。SCALE 使用 `(raw-zeroPoint)*scale`，SHIFT 使用 `ldexp`，没有旧的位移溢出写法。
- 输出复制到拥有独立存储的 float Mat。成功提交后的局部 Task 在输入/输出 owner 之前回收；模型、tensor plane 由 RAII 管理，运行时失败后的会话标记为不可复用。header/runtime 的二进制 ABI 仍必须匹配；成员检测不使不同版本的库自动兼容。

这些是已读源码的处理规则；输出宽度检查不能自动证明训练模型本身的视差物理语义，真实模型仍须确认 left-minus-right、输入宽度像素单位及板端执行结果。构建和测试状态见对应日志，不以本节替代。

Python `stereo/stereo_core.py` 继续负责几何，不需要因旧 runner 的 depth 缺陷改变 P/Q8 算法：

1. 严格读取左右内参、八参数畸变、Rodrigues 旋转、完整平移及标定画幅，按配置的 mm/m 转成米。
2. 未校正图像真实 remap 到原标定画幅，再 stretch 到 640×544；P/Q 使用与 resize 一致的像素中心转换。已校正图像必须显式声明与该标定的 rectified projection 一致。
3. 模型浮点视差恢复到 640×544 后乘 `640/预测宽度`，前提是上面的 runner 视差宽度单位契约成立。保留无效掩码，避免 NaN/零值稀释有效插值。
4. `Q8=floor(d*256+0.5)`；非有限、非正值、下溢、溢出均置 0，不做 uint16 回绕。SDK `bxf=fx*baseline_m*1000`，单位 pixel·mm。
5. 从同一量化 Q8 重投影，深度保持米，颜色来自同一 prepared 左图。默认为校正左光学坐标 X右/Y下/Z前；显示用 OpenGL 翻转不能改写导出坐标。

`recorded_npy` 仅回放既有预测，必须显式 `already_rectified: true`，且原图、预测、标定需对应同一校正网格；不能在预测后再 remap 左右图。新源码证明原 runner 没有执行校正，**仍不能单凭源码断言上游采集图是否已经校正**。校正声明及候选标定未确认时，结果应保留该状态，不能宣称物理测距精度。

## 验证结论的范围

几何单测可以验证投影/重投影、尺度、单位、Q8、PLY 与无效处理；源码审查可以确认 CLI 和分支行为。它们均不等于 X5 BPU 推理成功、旧 ELF 来源证明、训练模型精度或实体相机稳定性验收。新 runner 的实际构建、模型初始化、输出属性及硬件推理结果应以 v8p1 对应构建日志和板端记录为准。
