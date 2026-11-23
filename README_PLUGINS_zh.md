# v6 算法注册与通用配置

可选传感器输入：右图沿用 input.right，新增 input.rear、input.imu、input.wheel、input.radar。类型、初始化和兼容性见 [INPUT_SLOTS_zh.md](INPUT_SLOTS_zh.md)。这些槽位尚未接入新的硬件驱动或融合算法。

本项目从 v4 Ubuntu 版本独立复制，保留原有检测、分割、视差及 DAG 配置。新增算法通过一份注册文件接入；不再逐个向 Context、SDK Config 和公共配置白名单添加专用字段。

## 新增算法只维护自己的目录

以 `scene_cls` 为例：

```text
features/scene_cls/
  algorithm.json          注册名称、参数规则、输入、工厂和源码
  config.json             可选的公共默认参数
  include/scene_cls.hpp    工厂声明
  src/scene_cls.cpp        runtime::Feature 的实现
```

CMake 自动扫描 `features/*/algorithm.json`，从同一份注册生成配置目录和执行工厂。添加 C++ 算法后需要重新编译；已注册算法的模型路径、参数和 DAG 组合可直接通过配置调整。

注册文件见 `features/scene_cls/algorithm.json`。复制为其他算法时，修改 id、feature、factory、header、sources、inputs、parameters 即可，无需修改公共配置结构。配置阶段只读取注册元数据，不加载或运行模型。

## 统一接口

```cpp
std::unique_ptr<perception::runtime::Feature>
create(const perception::algorithms::Invocation& invocation);
```

`Invocation` 包含算法名称、任务实例 id、已校验并补齐默认值的参数、输入端口绑定。模型文件和目录参数已转为绝对路径；所有用户相对路径以主 pipeline.json 所在目录为基准。

实现 `Feature::execute(FeatureContext&)`，用 `context.get<T>(绑定的 artifact 名称)` 读取输入，用 `context.set(...)` 发布不可变输出。框架共享的图像输入类型为 SDK `psdk_frame`，`input.left` 与 `input.right` 由输入适配器提供；算法参数不会直接持有相机或 ROS 对象。

每个任务必须发布统一结果：

```cpp
context.set(
    perception::algorithms::resultKey(context.taskId()),
    perception::algorithms::Output{"{\"value\":123}"});
```

输出必须是有效 JSON 对象。写入采用已有 DAG 的任务事务，后续节点在依赖成功后才能读取。实例拥有自己的模型对象，结果保留在对应 SDK result 中。

## 配置算法与组合流程

`features.scene_cls` 可引用公共配置文件；任务中的 `parameters` 覆盖公共默认值。以两个独立分类实例并行为例：

```json
{
  "tasks": {
    "near": {
      "feature": "scene_cls",
      "parameters": {"model": "models/x5/scene_cls.bin", "top_k": 1}
    },
    "far": {
      "feature": "scene_cls",
      "parameters": {"model": "models/x5/other_scene.bin", "top_k": 3}
    },
    "publish": {"feature": "publish"}
  },
  "dag": {"sequence": [{"parallel": ["near", "far"]}, "publish"]}
}
```

以上是合入现有主配置的 tasks/dag 部分。也支持 `feature: inference, algorithm: scene_cls`。完整示例为 `scene_cls_camera.json`、`scene_cls_test.json`。同类算法可以出现多个任务实例；发布、记录节点必须等待所有启用的生产实例。

开关统一使用任务的 `enabled`。不必再维护一个算法配置里的第二个 enabled；禁用的任务不会加载模型，也不要求模型文件存在。输入端口绑定和输出类型仍必须符合算法契约，DAG 依赖不会替代真实的数据来源。

## 参数规则与 SDK 配置

注册支持 string、integer、number、boolean、numbers、strings、file、directory，以及 required/default/min/max/enum/length。未知参数、类型错误、越界值和缺失的活动模型会报错。文件路径支持 `${ENV_NAME}`，环境变量缺失会报错。

生成的 effective_full.json 使用固定的通用容器，以任务 id 为键：

```json
"algorithms": {
  "near": {
    "algorithm": "scene_cls",
    "parameters": {"model": "/userdata/project/models/x5/scene_cls.bin", "top_k": 1}
  }
}
```

实际输出会包含全部参数默认值。SDK 再次校验容器与图的一致性；缺少启用任务的配置、算法名不匹配、多余实例都会拒绝。仅运行新算法也被视为有效工作，不会启动旧检测/分割链。

调用 `psdk_full_result_get_algorithms_json(result, &json, &status)` 获取按任务 id 组织的结果。指针在 result release 前有效，销毁 engine 不影响已返回的 result。`psdk_full_save_result` 的 result.json 同样包含 algorithms；原有 C ABI 结果结构布局保持不变。

## scene_cls 模型要求

实现直接调用真实 Backend，不返回占位分类。支持 CPU ONNX 和编译了 BPU 后端的 X5 BIN。CPU 输入为 float32 NCHW，输出为 `[1,C]` 或 `[1,C,1,1]`；BPU 使用现有 Backend 的 NV12 输入，分类输出需为四维 batch=1、空间尺寸 1x1 的张量，量化与布局须符合其约定。softmax=true 将 logits 转成概率；softmax=false 要求模型已输出概率。输入尺寸可从静态模型解析。

mean/stddev 和 input_bgr 用于 RGB 浮点输入路径。BPU 的原生 NV12 路径由编译模型定义颜色与归一化处理，不能通过这些 CPU 预处理参数改写模型内部预处理。

需要自行提供真实的 `models/x5/scene_cls.bin` 并配置与训练一致的 mean/stddev、input_bgr、letterbox、class_names。配置模板不会伪造业务模型。

`validation/fixtures/scene_cls_rgb_mean.onnx` 是自建的、随图像改变输出的三通道均值测试模型，专门验证配置到 SDK 执行的完整链路，不是经过训练的场景分类器。可用 `python validation/create_scene_cls_fixture.py` 重建。

## 编译与验证

Windows 可沿用 CMakePresets 中的 windows-vs2022，或使用正常的 MSVC/Ninja 开发环境。CPU 推理需 R29_WITH_ORT=ON，ORT_ROOT 指向随包的 windows/deps/onnxruntime-win-x64-1.16.3。纯配置编译可关闭 PERCEPTION_BUILD_SDK，目录和参数校验无需加载 ORT/BPU。

```powershell
cmake --preset windows-vs2022
cmake --build --preset windows-release --parallel 4
ctest --test-dir build/vs2022 -C Release -R algorithm_plugin --output-on-failure
```

`tools/host_cmake.py --msvc` 可在 Windows 子进程中初始化 VS 2022 Community 环境，并避免启动器 PATH/Path 重复；不修改系统环境变量。

X5 仍可复用历史 Docker 容器 `obs-v004-x5-build` 和 `tools/build_in_container.sh`。源码包不携带旧 build/install 缓存；Docker 打包入口要求的 Ubuntu ROS 配套工具，可先在远程 Ubuntu 执行 `bash tools/build_ubuntu_x5.sh --host-only` 生成。新算法源码由根 CMake/SDK 构建自动带入，实际分类模型需使用与板端 SDK 匹配的工具链编译。六组 benchmark6 是既有固定实验入口，本次不把其历史 det/seg/视差指标冒充为分类精度测试。

配置工具示例：

```bash
export SDK_ROOT="$PWD/install/perception_ros_so_demo"
"$SDK_ROOT/bin/perception_config" run \
  --config scene_cls_camera.json \
  --run-dir results/scene_cls_prepare --prepare-only
```

run-dir 必须是新目录。相机模式的准备会检查板端 SDK 路径，应在对应 X5 环境执行。`--prepare-only` 仅生成配置。真实 SDK 相机执行可使用编译后的 `perception_full_demo --config results/scene_cls_prepare/effective_full.json --camera`，相机库路径沿用 `/app/data/metoak/lib/libMoGeneralSDK.so`，运行时库搜索路径需包含板端 Horizon、传感器及相机依赖。

测试结果见交付后的 `PLUGIN_VALIDATION_zh.md`；未完成的硬件测试会明确列出。
