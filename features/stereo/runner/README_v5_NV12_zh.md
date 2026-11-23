# v5 Stereo 的直接 NV12 输入

在线相机、ROS、录制回放已有 NV12 时，模型输入保留原 Y/UV 值；不再先转 BGR 再转回 NV12。I420 输入仅把 U/V 交错为 NV12。PNG/JPEG 必须先解码为 BGR，再一次转换到 NV12。

`NCHW [1,3,352,640]` 是模型维度元信息。实际图像存储由 Horizon `tensorType` 决定；NV12 的字节数为 `640*352*3/2=337920`，不是三通道 RGB 的字节数。

## C++ 接口

```cpp
#include "r36_stereo/nv12.hpp"
#include "r36_stereo/horizon_stereo.hpp"

// true：启动时要求左右模型输入都是 NV12 / NV12_SEPARATE。
r36stereo::HorizonStereoModel model(model_file, true);
auto left = r36stereo::copy_yuv420_to_nv12(
    left_data, source_width, source_height, source_stride, left_bytes, true);
auto right = r36stereo::copy_yuv420_to_nv12(
    right_data, source_width, source_height, source_stride, right_bytes, true);
left = r36stereo::resize_nv12(left, model.input_size());
right = r36stereo::resize_nv12(right, model.input_size());
cv::Mat disparity = model.infer_nv12(left, right);
```

NV12 Mat 类型为 `CV_8UC1`，`rows=H*3/2`、`cols=W`；前 H 行是 Y，后 H/2 行是交错 UV。支持输入 Mat 的行 padding。`copy_yuv420_to_nv12` 的最后一个参数为 false 时，源数据按 I420 解释，U/V stride 均为 Y stride/2。

`resize_nv12` 将 Y 作为单通道、UV 作为双通道分别用 `INTER_LINEAR` 缩放。同尺寸返回共享 Mat，不做色彩转换。它执行显式 stretch；没有自动裁剪。原帧及原内参不变，点云链路继续把视差恢复到原尺寸，并仅按横向缩放比例修正视差值。

主机不额外 `/255`。新模型的 YAML 声明 `input_type_rt=nv12`、训练输入 RGB、`data_scale=1/255`；该预处理由编译模型负责。输出类型、layout、aligned shape 与量化比例从实际运行时读取；不能根据 `tail_int16` 文件名硬编码 INT16 或 `/256`。S16 输出按有符号读取，支持单尺度 SCALE/SHIFT；不支持的多轴量化会明确拒绝。

## X5 独立检查

```bash
stereo_model_runner --model_file MODEL.bin --model_info

stereo_model_runner --model_file MODEL.bin \
  --left left.nv12 --right right.nv12 \
  --input_format nv12 --input_width 640 --input_height 544 \
  --output new_disparity.npy

stereo_model_runner --model_file MODEL.bin \
  --left left.png --right right.png --output new_disparity.npy
```

原始文件必须是紧密 NV12，文件字节数与指定源尺寸完全一致。输出为模型分辨率的 float32 NPY；应用的 Q8 恢复和点云生成由在线/对比 pipeline 完成。`--model_info` 只加载并检查模型、打印运行时 tensor 契约，不执行图像推理。输出路径必须新建。

## CPU 可执行的检查

在已经安装 OpenCV 和 CMake 的 Ubuntu 中，用真实 Horizon 头文件编译纯数据处理测试；不需要 BPU，也不会链接 AArch64 libdnn：

```bash
cmake -S features/stereo/runner -B build/stereo_codec \
  -DSTEREO_BUILD_HORIZON_RUNNER=OFF \
  -DSTEREO_BUILD_CODEC_TESTS=ON \
  -DHORIZON_ROOT=/path/to/matching/horizon/prefix \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/stereo_codec --parallel 4
ctest --test-dir build/stereo_codec --output-on-failure
```

测试覆盖原 Y/UV 字节保留、NV12/I420 源 padding、UV 通道隔离、640×544 到 640×352 边界、NCHW/NHWC、NV12_SEPARATE、aligned padding、S16 SCALE/SHIFT 和输出地址。CPU 测试验证数据处理，不代表已经执行新 BIN 或验证了板端精度、速度。
