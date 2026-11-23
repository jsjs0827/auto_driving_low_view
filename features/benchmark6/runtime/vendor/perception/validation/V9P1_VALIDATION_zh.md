# v9p1 新模型输入适配验证

日期：2026-10-10。此目录记录本版真实构建与测试，旧 V9_VALIDATION_zh.md 仍为历史记录。

## 本次修改

- 新配置 compare_stereo_352.dag.json 显式启用 resize 并断言 640×352。
- 用户确认模型布局 NCHW [1,3,352,640]；布局和图像类型仍从实际 Horizon 元数据读取，不根据三通道猜 RGB。
- provider 内两目同样 INTER_LINEAR，原 NV12/标定保留；输出恢复到源图尺寸，视差只乘宽比。640×544→640×352 的水平倍率为 1。
- 根 run.json 记录请求参数，方法 run.json 记录实际模型输入、原始输出、恢复尺寸及完整张量契约。resize/打包/推理/恢复均计入 provider/compute 计时。
- 保留原默认 strict 及原视差回放；包含之前 v9 Horizon 传递依赖的链接修复。

## 已执行检查

| 检查 | 结果与范围 |
|---|---|
| 原生 ROS 集成构建 | demo/raw_camera/stereo_model_runner/r36_compare_runner/r36_stereo_capture 成功编译安装 |
| CTest | 3/3，通过 ROS 辅助、张量 codec、比较契约 |
| 352 NCHW NV12 打包 | 实际执行全尺寸测试，检查 Y/UV 边界和末字节，正确大小为 337920 字节，元数据不修改 |
| 视差几何 | 640×352 和 640×176 的 8px 平面恢复到 640×544 后仍为 8px；固定 fx/baseline 下深度仍为 4.5m；横向缩放另行验证 |
| Python 配置/输入 | 17/17；成对尺寸、strict 默认、非法值拒绝、源 NV12/Q8 与内参保留、只向 stereo 传参数 |
| Python 报告 | 12/12；实际元数据展示与空输出处理、统计及 PLY 合同 |
| 原视差回归 | 真实 r36 ORT+mapping SDK 和检测/分割 ONNX：预热 1 帧、测量 2 帧，每帧保存 SDK 五类实际 PLY、JSON 与报告 |
| AArch64 交叉编译 | compare、capture、stereo 以及两种测试 ELF 均成功编译链接；ARM 测试未在 x86 上执行 |
| ELF 依赖闭包 | compare 147、capture 5、stereo 136 个依赖，缺失 0；无 RPATH/RUNPATH |
| HTML | 生成报告及 Node 脚本语法检查通过；本次未声称完成浏览器渲染验收 |

本机原视差回归使用先前外部记录 NPY 转成的 Q8 验证输入，不是新录制的相机原视差基线，也不代表新模型推理。实际相机 A/B 必须通过 capture_stereo.sh 同时录制左右图及 SDK Q8。配置测试中的 mock 子进程和占位模型文件只用于参数传递验证，不用于算法输出或性能数据。

本机系统 Python 缺少 cv2，回归复用了工作区已有测试解释器（OpenCV 4.10.0、NumPy 1.26.4），未创建新环境。X5 交付指令使用 /usr/bin/python3；系统安装 python3-numpy、python3-opencv 即可。

## 模型与实测限制

指定 BIN 为 r34_tail_pointwise_shuffle2_grouped_grid_352x640_nv12_mixed.bin，20384471 字节，SHA256：

```text
3d00263ac8a721f8d1ffd10063d94e6dee13a0fe06310ac218d9b6b6665c9558
```

模型文件未修改。用户给出的输入形状作为配置断言；本次未在本机执行该模型，本机已有的旧 Horizon 环境不作为新模型兼容性证明。未完成本次真实模型元数据探测或新模型推理，更未测量 X5 FPS/CPU/RSS 或视觉精度。板端运行时将真实张量元数据写入 stereo/run.json。

外部 result352640shuffke2mixed 中 100 个 NPY 均为 544×640 float32，且与 result0909/result_disparity 中的对应文件全部字节相同，不能将其当作本次新模型独立推理证据。详见 evidence/v9p1_model_audit.json。

HBRT3.15.54.0 与模型编译版本3.15.58.0.post0.dev202601130140 的警告没有通过本次尺寸适配消除。需用匹配模型编译包的运行库检查板端结果；本版不替换厂商运行库。

构建及回归证据见 evidence/v9p1_arm_build.json、v9p1_native_regression.json、v9p1_report_validation.json 和附带测试日志。源码包与运行包均提供完整 SHA256 清单、UTF-8 无 BOM/LF 检查。
