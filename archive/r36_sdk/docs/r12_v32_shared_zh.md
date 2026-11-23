# r12-v32 独立推理 SO 与调用示例

## 交付范围

其他程序可以直接链接 `libperception_sdk.so`，向 `psdk_process` 传入一帧 NV12/I420，获得检测框、分割类别掩码、帧身份和耗时。公开头 `perception_sdk.h` 不暴露 ROS、BPU、相机 SDK 或 C++ STL 类型，调用程序不需要包含这些内部头。

| 项目 | 本版行为 |
|---|---|
| 调度 | 保留当前原 DAG / Task / Feature；同一帧检测与分割并行 |
| 检测 | 原 BIN、letterbox、原解码及 NMS；精确缩放与直接输入写入 |
| 分割 | 原 BIN、原拉伸输入、原精确后处理；保留已经验证的加速路径 |
| 跨帧 | 同步单帧调用，无输入队列、无跨帧双缓冲 |
| VSE / 融合 BIN | 本独立入口不启用；不改变原数值流程 |
| 相机与视差 | 调用方取帧；库不打开相机、不计算视差 |
| 可视化 | 公开 `psdk_save_png`，绘制同帧结果并保存 PNG |
| 错误 | 返回完整 `uint32_t` 状态码，并附原生码、Task、Feature、帧身份和说明 |
| x86 CPU | 编译 ONNX Runtime 后可用；需要真实 ONNX 和明确的输入/输出配置，BIN 不能直接在 x86 执行 |

`native_code` 只在底层阶段提供时有意义；保留的部分 BPU 实现仅抛出通用错误，原生码可能为 0。调用方应同时记录完整状态码、Task、Feature 和 detail，不能据此假定所有 Horizon 返回码都已经保留。

这个 SO 封装检测与分割。它没有导出依赖缺失原始 ABI 的地图、点云、Patchwork、标定、AiFusion、回充与 ROS 协议。原产品源码仍保留，不能声称这些业务功能已经被此独立接口替代。

## Ubuntu 宿主机交叉编译

在解压后的 `perception-x5-shared-sdk-r12-v32` 项目根目录执行；不必进入 Docker：

```bash
sudo bash tools/build_shared_in_container.sh \
  --container obs-v004-x5-build \
  --sysroot /work/obs_v004_x5/sysroot_docker \
  --out "$PWD/../r32_shared_release"
```

`obs-v004-x5-build` 是现有 C++ 交叉编译容器的名称；不是模型工具链镜像名。`--sysroot` 是容器内部已经使用的 X5 sysroot。该入口不需要 `--deps-root`、`--recharge-root`、原 ROS 开发头或重新转换模型。原来的同类构建入口也由项目包装到本版 shared 构建流程，实际选项以 `--help` 为准。

成功后的宿主机目录：

```text
../r32_shared_release/
  perception-sdk-x5-r12-v32-shared.tar.gz
  perception-sdk-x5-r12-v32-shared.tar.gz.sha256
```

SDK 解压内容：

```text
perception-sdk-x5-r12-v32-shared/
  lib/libperception_sdk.so       # SONAME: libperception_sdk.so.1
  include/perception_sdk.h
  bin/perception_sdk_demo
  examples/demo.cpp
  examples/CMakeLists.txt
  profiles/original.cfg
  profiles/dag_exact.json
  tools/integrity.py
  SDK_RELEASE.json
  SHA256SUMS
```

X5 的 BPU/媒体依赖使用板端原生库；本包不复制另一套相机媒体运行库。源代码和清单以 UTF-8 无 BOM、LF 打包；不要通过文本编辑器重新保存 `.so` 或二进制输入。

## X5 首次测试

把 SDK 压缩包和 `.sha256` 放到 X5 的同一目录。以下命令假定当前目录中有这两个文件，并且尚不存在同名解压目录：

```bash
sha256sum -c perception-sdk-x5-r12-v32-shared.tar.gz.sha256
tar -xzf perception-sdk-x5-r12-v32-shared.tar.gz
cd perception-sdk-x5-r12-v32-shared
python3 tools/integrity.py .
```

使用板端真实的一帧输入和原始模型：

```bash
bin/perception_sdk_demo \
  --input /userdata/test_input/source.nv12 \
  --width 640 --height 544 --stride 640 --format nv12 \
  --det-model /userdata/ai_infer/data/mmyolo0828.bin \
  --seg-model /userdata/ai_infer/data/pidnet_0825_3classes.bin \
  --frames 120 --warmup 20 \
  --out /userdata/r12_v32_res/shared-test-01 \
  --save-png
```

输入路径需要替换为实际文件；格式是原始 YUV420，不是 JPEG/PNG。默认输入契约为原项目 RGBD 左目 `640×544`。NV12 的 Y 和 UV 行跨度都是 `stride`；I420 的 U/V 跨度是 `stride/2`。三个平面或两个平面必须按约定连续存储，文件字节数为 `stride × height × 3 / 2`；宽、高和 stride 为偶数。取 GDC 大图进行测试时不能把该测试直接当作原项目输入一致性证明。

`--out` 必须不存在；重新测试使用另一个目录。`--profile`、`--dag` 可以指定配置；默认取 `--sdk-root` 下的 `profiles/original.cfg` 和 `profiles/dag_exact.json`，`--sdk-root` 默认当前目录。已有模型路径也可以直接写入 `original.cfg`。不要用融合 BIN 替代原始 BIN 来宣称原流程一致。

## 结果和计时

| 文件 | 内容 |
|---|---|
| `summary.json` | 完成状态、后端、输入、FPS、平均分阶段时间、CPU 和 RSS |
| `frames.csv` | 每次测量的帧身份、打包、两个分支的前处理/推理/后处理、DAG 和 process 时间、RSS |
| `detections.json` | 最后一次调用的检测框、类别、分数和帧身份，框为输入图坐标 |
| `mask.bin` | 最后一次调用的逐像素 `uint8` 类别 ID，紧凑存储，尺寸为输入宽高 |
| `overlay.png` | 可选，同帧检测与分割结果叠加图 |
| `failed.json` | SDK 调用失败时的完整错误码与定位信息 |

本 demo 重复推理同一个文件，用递增帧 ID 和调用时间标记每次调用。因此测到的是固定输入推理吞吐，不能用它代替相机新帧交付 FPS。暖机不计入测量；检测/分割并行，两个分支的分阶段时间不能直接相加作为整帧时间。`graph_ms` 是 DAG 执行时间，`process_ms` 是同步 API 的处理时间；FPS 使用测量窗口的真实墙钟时间，还包含结果视图读取、轻量记录和 RSS 采样。

PNG、CSV 和最终结果写入在测量窗口结束后执行，不计入该 FPS。Linux CPU 是整个 demo 进程的用户态加内核态时间除以墙钟时间，使用两个核时可以超过 100%。RSS 来自 `/proc/self/status`；峰值来自 `getrusage`，包含模型、运行库、输入、结果和 demo 记录。非支持平台返回空值，不伪造指标。`rss_after_engine_destroy_kib` 仍包含最后一帧结果和 demo 自身，不能仅凭单次下降或未下降判断泄漏；本 demo 不测 CMA。

## 最小调用代码

完整可运行版本在 `examples/demo.cpp`。实际在线程序应创建一次 engine，在帧循环中多次调用 `psdk_process`，退出时销毁一次；不要每帧重新加载模型。下面函数只演示一帧的完整生命周期和错误处理，`demo.cpp` 已把创建/销毁放在暖机和测量循环之外。配置字符串至少在 `psdk_create` 返回前有效：

```cpp
#include "perception_sdk.h"

uint32_t run_one_frame_example(const uint8_t* caller_owned_nv12,
                               uint64_t camera_frame_id,
                               uint64_t camera_timestamp_ns,
                               psdk_status& status) {
    psdk_config config{};
    config.struct_size = sizeof(config);
    config.abi_version = PSDK_ABI_VERSION;
    config.backend = PSDK_BACKEND_BPU;
    config.original_profile = "/userdata/sdk/profiles/original.cfg";
    config.dag_profile = "/userdata/sdk/profiles/dag_exact.json";
    config.detection_model = "/userdata/ai_infer/data/mmyolo0828.bin";
    config.segmentation_model = "/userdata/ai_infer/data/pidnet_0825_3classes.bin";

    psdk_engine* engine = nullptr;
    uint32_t code = psdk_create(&config, &engine, &status);
    if (code != 0) return code; // status retains the original creation failure.

    psdk_frame frame{};
    frame.struct_size = sizeof(frame);
    frame.abi_version = PSDK_ABI_VERSION;
    frame.format = PSDK_FORMAT_NV12;
    frame.width = 640;
    frame.height = 544;
    frame.stride = 640;
    frame.data = caller_owned_nv12;
    frame.bytes = 640ull * 544 * 3 / 2;
    frame.frame_id = camera_frame_id;
    frame.timestamp_ns = camera_timestamp_ns;

    psdk_result* result = nullptr;
    code = psdk_process(engine, &frame, &result, &status);
    if (code == 0) {
        psdk_result_view view{};
        view.struct_size = sizeof(view);
        view.abi_version = PSDK_ABI_VERSION;
        code = psdk_result_get_view(result, &view, &status);
        if (code == 0) {
            // Publish/copy view.detections and view.segmentation_mask here.
            // Preserve view.frame_id and view.timestamp_ns when publishing.
        }
    }
    psdk_result_release(result);
    // Keep the primary processing error if cleanup also reports an error.
    psdk_status cleanup_status{};
    const uint32_t cleanup_code = psdk_destroy(&engine, &cleanup_status);
    if (code != 0) return code;
    status = cleanup_status;
    return cleanup_code;
}
```

函数参数中的像素和帧身份由宿主程序提供；返回非零时，调用方记录 `status.code/native_code/task/feature/detail`。完整 demo 还展示了 RAII 释放和异常分支。

`psdk_process` 返回前，调用方必须保留输入帧且不能修改像素。返回后可以归还相机/SHM 输入；结果由 `psdk_result` 独立持有，不依赖输入存储，也不依赖 engine 的存活。`view` 指针在 `psdk_result_release` 后失效，不要用 `free/delete` 释放。每个 engine 同步处理，精确流水线之间也串行保护，不支持通过多个调用并发处理不同帧来提高 FPS；检测与分割只在同一帧的 DAG 内部并行。

如果需要 `psdk_save_png`，还要保留该结果对应的原始输入，直到 PNG 保存返回；API 会检查帧 ID、时间戳和尺寸，且拒绝覆盖已有 PNG。相机缓冲在 process 之后归还、随后还想画图时，应由调用方先复制一份同帧原始数据。

## 接回现有 ROS 或其他程序

原相机节点和共享内存读取代码继续留在能够编译运行的宿主程序中：读取 RGBD 左目 → 保留同帧内存 → `psdk_process` → 按原消息协议发布框和类别图 → 释放结果与输入。每帧通过 `frame_id/timestamp_ns` 和结果一一对应，不使用跨帧双缓冲。

本库不需要看到原始 `CameraData` 类型，但宿主负责把真实像素地址、格式、尺寸、stride 和有效字节数转换为 `psdk_frame`。这避免了库本身编译依赖受保护的原始头；若宿主也缺少它自身使用的 ROS/SHM 开发头，仅提供这个 SO 并不会替宿主生成缺失 ABI。

## 独立编译调用 demo

在 X5 的 SDK 解压目录中，使用本机 C++ 编译器：

```bash
cmake -S examples -B demo-build -DPSDK_ROOT="$PWD" -DCMAKE_BUILD_TYPE=Release
cmake --build demo-build -j2
demo-build/my_demo --help
```

也可以直接编译：

```bash
c++ -std=c++17 examples/demo.cpp \
  -Iinclude -Llib -lperception_sdk \
  -Wl,-rpath,'$ORIGIN/../lib' \
  -Wl,-rpath-link,/usr/hobot/lib \
  -o bin/my_demo
```

调用方只链接公开 SO，不需要原业务开发头。板端必须有与原模型匹配的 BPU 运行库；如 X5 的原生库在其他目录，应使用该板子的真实库路径。Ubuntu x86 主机不能直接运行 AArch64 demo；在主机交叉编译调用程序时也需使用相同 AArch64 工具链和 sysroot。

Windows CPU 调用程序部署时，`perception_sdk.dll` 和匹配的 `onnxruntime.dll` 应放在调用程序 EXE 的同一目录；仅把 SDK 目录加入 PATH，可能被系统目录中的另一版本 ONNX Runtime 抢先加载。随包的独立示例 CMake 会在构建后复制 SDK `bin/` 中已有的 DLL 到 `my_demo.exe` 旁边，并为 MSVC 调用程序嵌入 UTF-8 清单。移动 EXE 时需要一起移动这些 DLL。

## x86 CPU 构建

使用原生 x86-64 编译器和匹配的 ONNX Runtime C++ SDK：

```bash
cmake -S . -B build/shared-cpu \
  -DR29_WITH_ORT=ON -DORT_ROOT=/actual/onnxruntime/sdk \
  -DR20_WITH_BPU=OFF -DPERCEPTION_BUILD_PRODUCT=OFF \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/shared-cpu --target perception_sdk perception_sdk_demo -j2
```

`ORT_ROOT` 必须换为本机真实路径。运行时传 `--backend cpu --cpu-profile <真实配置.json>`。配置应描述 ONNX 输入名、布局、颜色转换/归一化、输出名、解码格式等；包内 `profiles/cpu_contract.example.json` 含有待替换占位符，不能直接作为实测配置。CPU 与 BPU 原模型精度一致性仍需使用同源模型和同帧数据验收。

## 验收与限制

本机测试可以覆盖公开 ABI、错误分支、结果生命周期、CPU 真推理和包内文件校验；这不等于已经在你的 Ubuntu 容器生成了 X5 SO，也不等于板端精度和性能已经通过。板端首先用原始同帧数据检查 detector/segmenter 实际输入与原基线逐字节相等、分割零像素差、检测结果一致，再比较稳定 FPS、CPU 和 RSS。封装成 SO 本身不保证额外加速；主要价值是获得清晰可调用边界，并继续复用之前的优化实现。
