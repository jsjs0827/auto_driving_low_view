# r12-v3 严格 JSON DAG：架构、配置、扩展与部署

本文对应 `perception_inference` 3.3.0、ABI/SONAME 5、
`profiles/runtime_r12_v3_default.json`、`profiles/runtime_r12_v3.schema.json`、三份 X5 AI-only
JSON 和 `src/runtime_config_json.cpp`。

r12-v3 没有完整照搬 MMlab/MMEngine 配置。它使用项目自有、无代码执行能力的严格 JSON，把
“Task 的定义”和“Task 的编排”分开：

- `tasks` 定义每个 Task 包含哪些 Feature、参数和启用状态；
- `dag` 只引用 Task ID，用嵌套 `sequence` / `parallel` 表达主结构；
- 可选 `edges` 增加无法自然写进结构树的横向约束；
- 解析器把结构展平为现有 `TaskGraphConfig.depends_on`，后续仍使用原 TaskGraph 校验、线程池和
  事务式 artifact 提交。

板端只执行 C++，不嵌入 CPython，不依赖 MMEngine，也不动态加载 JSON Schema 引擎。

## 1. 默认业务图

默认文件定义六个业务 Task：

```text
detection ---------\
segmentation -------+--> semantic_mapping --> publish
stereo_geometry ----/                     \--> ai_fusion
```

| Task | Feature 顺序 | 结构前继 | 作用 |
|---|---|---|---|
| `detection` | `ai.preprocess` → `ai.backend` → `ai.det.postprocess` | 无 | 检测完整推理 |
| `segmentation` | `ai.preprocess` → `ai.backend` → `ai.seg.postprocess` | 无 | 分割完整推理 |
| `stereo_geometry` | `pointcloud.disparity_to_cloud` → `pointcloud.project_to_image` | 无 | 视差点云与图像投影 |
| `semantic_mapping` | 匹配 → 深度 → 体素 → 连通域 → 语义点云 → 两个坐标变换 | 前三个 Task | 生成 `base_link` 和 `percep_main` 语义点云 |
| `publish` | `output.semantic_cloud_markers` | `semantic_mapping` | 构造发布点云和 Marker 载荷 |
| `ai_fusion` | `fusion.ai_fusion` | `semantic_mapping` | 生成 `map.ai`、`map.fusion` |

DAG 边表示严格完成屏障，不表示前继的全部输出会自动传给后继。Feature 仍按 artifact key 从有类型的
`ArtifactStore` 读取输入。任一必需前继失败，后继不会在输入不完整时继续执行。

## 2. 顶层严格 schema

JSON 顶层只允许：

| 字段 | 必需 | 含义 |
|---|---:|---|
| `$schema` | 否 | 编辑器提示；必须是非空字符串，C++ 运行时不会读取其目标文件 |
| `config_version` | 是 | 必须为整数 `3` |
| `engine` | 是 | worker 数和返回 artifact 白名单 |
| `tasks` | 是 | Task 定义对象，至少一个、最多 512 个 |
| `dag` | 是 | 引用全部 Task 的结构树 |
| `edges` | 否 | 额外的显式有向边数组 |

`profiles/runtime_r12_v3.schema.json` 使用 JSON Schema 2020-12，可供 IDE 补全和离线工具检查；
部署时真正具有约束力的是编译进 SDK 的严格解析器、`TaskGraphConfig::validate()` 和已注册 Feature
参数 schema。不要把修改外部 schema 文件当作放宽运行时规则的方法。

解析器会拒绝重复 object key、未知字段、错误 JSON 类型、尾逗号、尾随内容、非法 UTF-8 和不合法
Unicode。UTF-8 BOM 可以存在。为避免不受控的内存或调度开销，运行时还实施以下硬上限：

| 资源 | 上限 |
|---|---:|
| 输入文件 | 4 MiB |
| JSON/DAG 嵌套深度 | 64 层 |
| 单份配置中的 JSON 值节点总数 | 65536 |
| 单个字符串（UTF-8 字节数） | 65536 字节 |
| 单个 JSON 数组 | 16384 项 |
| 单个 JSON 对象 | 1024 个成员 |
| Task 定义 | 512 个 |
| 展平后的依赖边总数（含显式 `edges`） | 16384 条 |
| 单个 Task 的 `features` | 256 项 |
| 单个 `params` 对象 | 256 个成员 |
| 参数中的扁平数值数组 | 4096 项 |
| `engine.worker_threads` | 1～256 |

达到上限仍合法，超过上限会直接拒绝，不会截断或降级。

### 2.1 `engine`

```json
"engine": {
  "worker_threads": 3,
  "result_artifacts": [
    "ai.detection",
    "ai.segmentation",
    "cloud.semantic",
    "map.ai",
    "map.fusion"
  ]
}
```

- `worker_threads` 缺省为 3，最终有效范围为 1～256；
- `result_artifacts` 是成功运行后允许返回给调用方的 artifact 白名单；
- key 必须非空、不得重复，也不能以保留的 `input.` 前缀开头。

## 3. `tasks`：定义与引用分离

`tasks` 是以 Task ID 为 key 的对象。ID 长度为 1～128，首字符只能是字母或 `_`，其余字符可为
字母、数字、`_`、`-`、`.`。Task 定义只允许 `enabled`、`features`、`params`：

```json
"semantic_mapping": {
  "enabled": true,
  "features": [
    "fusion.match_detection_segmentation",
    {
      "type": "fusion.depth_filter",
      "params": {
        "input_key": "fusion.matches",
        "output_key": "fusion.depth_matches"
      }
    }
  ],
  "params": {
    "detection_score_min": 0.30,
    "detection_include_ids": [8, 9, 10, 18],
    "filter_segmentation": true
  }
}
```

- `features` 必需且至少一个，按数组顺序在该 Task 内串行执行；
- Feature 可简写成注册名字符串，也可写成 `{"type":..., "params":...}`；
- Task 级 `params` 对该 Task 内所有 Feature 可见；
- Feature 级 `params` 覆盖同名 Task 参数，适合同一 Feature 多次使用或不同阶段使用不同 key；
- `enabled` 缺省为 `true`；
- 参数值只能是字符串、布尔值、JSON number 或一维纯数字数组。数组在内部转换为逗号分隔值；
  `null`、对象、混合数组和嵌套数组均被拒绝；
- JSON 层先校验形状，图校验阶段再根据注册表拒绝未知 Feature、拼错的参数和错误参数类型。

Task 定义本身不表示执行顺序，也不允许写 `depends_on`。依赖统一由 `dag` 和 `edges` 产生，避免同一
条边在两处维护。

## 4. `dag` 节点和展平规则

一个 DAG 节点只能是以下四种形式之一：

```json
"task_id"
```

```json
{"task": "task_id"}
```

```json
{"sequence": [NODE]}
```

```json
{"parallel": [NODE, NODE]}
```

字符串和 `{"task":...}` 完全等价。`sequence` 至少含一个子节点；`parallel` 至少含两个分支。
对象节点必须且只能含 `task`、`sequence`、`parallel` 中的一个，不能在同一对象中同时写两个操作符。

每个 `tasks` 中定义的 ID 必须在 `dag` 中出现且只出现一次：未知引用、重复引用和未引用定义都会
失败。最终 `TaskGraphConfig.tasks` 顺序由 DAG 的深度优先首次遍历决定，而不是 `tasks` 对象的文本
或字典顺序。

展平时，每个子树计算一组入口 Task 和出口 Task：

- `sequence: [X, Y]`：给 Y 的每个入口增加对 X 每个出口的依赖；
- `parallel: [X, Y]`：X、Y 之间不自动增加边，入口/出口集合取两个分支的并集；
- 嵌套结构递归应用同样规则，因此 fork/join 不需要手写多条边。

例如：

```json
"dag": {
  "sequence": [
    "a",
    {
      "parallel": [
        {"sequence": ["b", "c"]},
        "d"
      ]
    },
    "e"
  ]
}
```

展平结果为：

```text
a -> b -> c -\
 \-> d --------+-> e
```

即 `b` 依赖 `a`，`c` 依赖 `b`，`d` 依赖 `a`，`e` 同时依赖 `c`、`d`。

### 4.1 完整结构示例

下面的示例完整展示所有顶层区域和两种 Task 引用写法。它用于说明配置结构；真实端到端运行仍应从
默认文件复制，因为 Feature 需要匹配的输入 artifact、模型和标定。

```json
{
  "$schema": "runtime_r12_v3.schema.json",
  "config_version": 3,
  "engine": {
    "worker_threads": 3,
    "result_artifacts": ["cloud.projected", "map.ai", "map.fusion"]
  },
  "tasks": {
    "cloud": {
      "features": [
        {
          "type": "pointcloud.disparity_to_cloud",
          "params": {
            "disparity_key": "input.disparity",
            "image_key": "input.image",
            "cloud_key": "cloud.raw"
          }
        }
      ],
      "params": {
        "fx": 288.129,
        "fy": 288.129,
        "cx": 308.542,
        "cy": 278.213,
        "baseline_m": 0.0601156,
        "calibration_width": 640,
        "calibration_height": 544
      }
    },
    "project": {
      "features": ["pointcloud.project_to_image"],
      "params": {
        "input_cloud_key": "cloud.raw",
        "image_key": "input.image",
        "output_cloud_key": "cloud.projected",
        "fx": 288.129,
        "fy": 288.129,
        "cx": 308.542,
        "cy": 278.213,
        "calibration_width": 640,
        "calibration_height": 544
      }
    },
    "ai_map": {
      "features": ["fusion.ai_map"],
      "params": {
        "cloud_key": "cloud.semantic",
        "grid_rows": 100,
        "grid_cols": 80,
        "grid_resolution_m": 0.02,
        "grid_origin_x_m": 0.0,
        "grid_origin_y_m": -0.8,
        "output_key": "map.ai"
      }
    },
    "fusion_map": {
      "features": ["fusion.fusion_map"],
      "params": {
        "ai_key": "map.ai",
        "geometry_key": "input.geometric_map",
        "output_key": "map.fusion"
      }
    }
  },
  "dag": {
    "sequence": [
      {"task": "cloud"},
      {
        "parallel": ["project", "ai_map"]
      },
      "fusion_map"
    ]
  },
  "edges": []
}
```

`profiles/runtime_r12_v3_default.json` 才是六业务 Task 的完整部署模板，并包含全部 artifact key、
过滤阈值、两个坐标变换和 AiFusion 参数。

## 5. 可选 `edges`

结构树不方便表达的额外约束可写为：

```json
"edges": [
  {"from": "calibration_ready", "to": "stereo_geometry"},
  {"from": "health_gate", "to": "publish"}
]
```

边方向为 `from -> to`，即 `to` 增加对 `from` 的依赖。显式边会在结构展平后追加，然后对整个图做
统一校验。以下情况会失败：

- `from`/`to` 未定义；
- 自环；
- 同一显式边重复；
- 显式边与结构树已生成的边重复；
- 追加后形成环；
- 启用 Task 依赖禁用 Task。

`parallel` 表示“不自动增加边”，不是禁止 `edges` 在两个分支之间增加约束。若加了横向 edge，实际
可并行度会相应降低。

## 6. 禁用语义：保留节点，不自动旁路

`enabled=false` 是 fail-closed 语义。结构展平时禁用 Task 与启用 Task 一样参与入口、出口和边生成；
调度时它被报告为 `Skipped/disabled`，但解析器不会把它的前继直接连接到后继。

因此：

```json
{"sequence": ["a", "off", "b"]}
```

若 `off.enabled=false`、`b.enabled=true`，配置会因“启用节点依赖禁用节点”而被拒绝。这样可以避免
关闭安全检查或数据生产 Task 后，下游悄悄绕过它继续运行。

以下形式适合禁用可选叶子：

```json
{
  "sequence": [
    "semantic_mapping",
    {"parallel": ["publish", "ai_fusion"]}
  ]
}
```

当 `ai_fusion.enabled=false` 且没有启用后继时，它仍保留并被明确跳过，`publish` 不受影响。要移除
中间节点，应同时删除或重写相应 Task 定义和 DAG 引用；不要依赖旁路推断。环检测也覆盖禁用节点。

## 7. 线程与执行模型

`engine.worker_threads` 控制一个有界共享线程池：

- 线程池随 `TaskGraph` / `PerceptionEngine` 构造一次，跨帧复用；
- 每帧只提交当前已就绪的 Task，不逐帧创建线程；
- Task 内 Feature 串行执行，Task 间并行由依赖和 worker 数共同决定；
- worker 少于就绪 Task 时，其余工作排队，不会额外无限创建线程；
- Engine 关闭时统一停止并 join worker；
- 同一个 Engine 的帧级 `run()` 当前串行保护，单帧内部才按 DAG 并行。

默认 3 个 worker 与三个根分支匹配，但最终值必须在真实 X5 上比较 2/3/4 的延迟、吞吐、CPU、
内存带宽和温度。配置校验还会拒绝无序并行 Task 共享同一个独占模型 session/workspace；若确需两个
模型实例，设置不同 `model_instance`，或在 DAG 中显式串行。

Feature 输出先写入 Task 的事务上下文；Task 成功后才提交到共享 artifact store，失败不会留下
可被后继误读的半成品。

## 8. 默认模板与安全占位

默认 JSON 使用 X5/BPU profile，并把以下安全条件保持为真：

- 两个 `pointcloud.transform` 均为 identity 占位，同时
  `require_non_identity=true`；构造 Engine 时会失败，直到写入实测外参；
- `ai_fusion.require_product_adapter=true`；缺少产品 adapter 会明确失败；
- 产品 adapter 存在时还要求 `input.ground_cloud`、`input.obstacle_cloud` 和语义点云都在
  `base_link`；
- 源码包不包含模型，检测/分割 profile 必须增加真实 `model=`。

Windows/离线调试时应复制而不是直接修改交付模板：

1. 把 `detection_x5.cfg` / `segmentation_x5.cfg` 换成 ONNX profile；
2. 在 profile 写真实 `.onnx` 路径；
3. 写实测外参；
4. 禁用 `ai_fusion` 叶子，或设 `require_product_adapter=false`。便携 fallback 可在没有几何地图时
   使用同网格的空几何层，但不能作为产品 AiFusion 验收结果。

主 JSON 中的相对 profile 路径由 `PerceptionEngine` 相对 JSON 所在目录解析，包括 Task 级参数和
Feature 级覆盖。

## 9. v2 与 legacy 配置兼容

入口按扩展名和内容确定：

- `.json`（大小写不敏感）固定进入 v3 严格 JSON 解析器，要求 `config_version: 3`；
- 非 JSON 文件先按 `key=value` 读取；存在 v2 标志时展开 r12-v2 `semantic_perception` preset；
- 同时没有 `config_version` 和 `pipeline.preset` 时，交给 r12 显式 Task/Feature 配置解析器。

保留文件：

- `profiles/runtime_r12_v2_default.cfg`：六业务 Task 的 v2 简化 preset；
- `profiles/runtime_r12_default.cfg`：13 个细 Task 的 r12 显式配置。

迁移到 v3 时，把 Task 定义搬入 `tasks`，把 `depends_on` 关系改写成结构化 `dag`，只有结构树表达
不自然的边才放入 `edges`。不要仅把旧文件改成 `.json` 后缀。

## 10. 扩展 Task 和 Feature

### 10.1 使用已有 Feature 增加 Task

1. 在 `tasks` 增加唯一 ID、`features` 和参数；
2. 在 `dag` 恰好引用一次；
3. 用 `sequence` 表达屏障、`parallel` 表达独立分支；
4. 必要时增加非重复 `edges`；
5. 运行 `perception_runtime_demo --explain-config`，确认展平依赖符合预期；
6. 用真实/合成输入验证 artifact key 和错误传播。

通用 schema 不需要为每个新 Task 增加字段。Task ID 只是配置级名称，真正实现来自 Feature 注册表。

### 10.2 增加编译型 Feature

1. 实现 `perception::runtime::Feature`，明确输入/输出 artifact 和失败行为；
2. 在 `registerBuiltinFeatures()` 路径注册工厂；
3. 注册严格参数 schema 和独占资源声明；
4. 增加正常、边界、未知参数、资源释放和并发冲突测试；
5. 再在 JSON Task 中引用注册名；
6. 若产生交付结果，把 key 加入合适配置的 `engine.result_artifacts`。

Feature 不应自行创建逐帧线程；并发统一由 TaskGraph 管理。增加 Feature 后还要确认 Windows 与
GCC 9/AArch64 C++17 都能编译，并检查是否引入新的目标运行库及许可证。

## 11. Windows / Visual Studio 2022

```powershell
cd D:\projects\python_pro\gpt_pro\reconstract_code\perception-x5-unified-r12-v3\point-cloud-processing-perception-x5-v3
Set-ExecutionPolicy -Scope Process Bypass
$env:ORT_ROOT = 'D:\deps\onnxruntime-win-x64-1.16.3'
.\windows\generate_vs2022.ps1
.\windows\build_and_test.ps1 -Configuration Release
```

生成：

```text
<r12-v3 外层目录>\vs2022-r12-v3\PerceptionR12V3.sln
```

只查看展开图：

```powershell
..\vs2022-r12-v3\Release\perception_runtime_demo.exe `
  --explain-config .\profiles\runtime_r12_v3_default.json
```

针对性配置回归可重跑：

```powershell
ctest --test-dir ..\vs2022-r12-v3 -C Release `
  -R '^(runtime_json_config_tests|runtime_config_tests|perception_runtime_demo_explain_v3)$' `
  --output-on-failure
```

这条命令只覆盖三个配置相关测试，不是全量回归。发布前仍需执行不带 `-R` 的完整 CTest，并保存
最终冻结日志。

## 12. Ubuntu Docker / X5 交叉编译

若已有容器：

```bash
export X5_CONTAINER=your_x5_build_container
docker start "$X5_CONTAINER"
docker exec -it "$X5_CONTAINER" bash
```

新建容器时按实际镜像和宿主目录替换：

```bash
export X5_IMAGE=your_x5_build_image
export HOST_R12_V3=/absolute/host/path/perception-x5-unified-r12-v3
docker run -it --name perception-x5-r12-v3 \
  -v "$HOST_R12_V3:/work/unified-r12-v3" \
  -v /absolute/host/path/x5work:/x5work \
  "$X5_IMAGE" bash
```

容器内：

```bash
cd /work/unified-r12-v3/point-cloud-processing-perception-x5-v3

export X5_CXX=/usr/bin/aarch64-linux-gnu-g++
export X5_SYSROOT=/x5work/gecao/sysroot_docker
export X5_BPU_DEPS="$X5_SYSROOT/usr_x5/hobot/lib"
export SDK_BUILD_DIR=/x5work/build/unified-r12-v3
export SDK_OUTPUT_DIR=/x5work/releases
export SDK_NAME=perception-sdk-x5-3.3.0-unified-r12-v3
export SDK_JOBS=4

export ENABLE_MO_CAMERA=0
unset MO_CAMERA_INCLUDE_DIR
export ENABLE_PRODUCT_AI_FUSION=0
unset PRODUCT_SOURCE_ROOT
unset PRODUCT_AI_FUSION_SUPPORT_LIBRARY

bash tools/build_x5_sdk.sh
```

检查：

```bash
test -f "$SDK_OUTPUT_DIR/$SDK_NAME.tar.gz"
test -f "$SDK_OUTPUT_DIR/$SDK_NAME.tar.gz.sha256"
readelf -h "$SDK_OUTPUT_DIR/$SDK_NAME/bin/new_main" | grep Machine
readelf -h "$SDK_OUTPUT_DIR/$SDK_NAME/bin/perception_runtime_demo" | grep Machine
readelf -h "$SDK_OUTPUT_DIR/$SDK_NAME/lib/libperception_inference.so.5" | grep Machine
tar -tzf "$SDK_OUTPUT_DIR/$SDK_NAME.tar.gz" | grep 'profiles/runtime_r12_v3_default.json'
tar -tzf "$SDK_OUTPUT_DIR/$SDK_NAME.tar.gz" | grep 'profiles/runtime_r12_v3.schema.json'
tar -tzf "$SDK_OUTPUT_DIR/$SDK_NAME.tar.gz" | grep 'profiles/runtime_r12_v3_x5_.*json'
tar -tzf "$SDK_OUTPUT_DIR/$SDK_NAME.tar.gz" | grep 'tools/run_x5_ai_unit.sh'
sha256sum -c "$SDK_OUTPUT_DIR/$SDK_NAME.tar.gz.sha256"
```

三个 ELF 应显示 AArch64。切换 toolchain/sysroot 时使用新 `SDK_BUILD_DIR`；新发布名必须使用新的
`SDK_NAME`，打包器会拒绝覆盖同名已有包。

只调试 X5 BPU 检测/分割时，不需要完整六 Task 的视差、外参和 AiFusion 输入。使用
`runtime_r12_v3_x5_detection.json`、`runtime_r12_v3_x5_segmentation.json` 或
`runtime_r12_v3_x5_det_seg_parallel.json`，并通过 `tools/run_x5_ai_unit.sh` 调用
`perception_runtime_demo`。命令、输出和速度测试边界见
`docs/x5_detection_segmentation_json_test_zh.md`。

### 12.1 可选直接相机

只有拿到与板端 `libMoGeneralSDK.so` 完全匹配的厂商头文件后才启用：

```bash
export ENABLE_MO_CAMERA=1
export MO_CAMERA_INCLUDE_DIR=/path/to/matching/MoGeneralSDK/include
bash tools/build_x5_sdk.sh
```

目录必须包含 `MoTypes.h`、`MoCommon.h`、`MoException.h`、`MoLocalSdkInterface.h`。不能手写私有
结构体猜测 ABI，也不能在产品相机服务已独占设备时同时直接打开相机。

### 12.2 可选产品 AiFusion

独立 X5 SDK 构建需要真实产品源码、支持库及其传递依赖：

```bash
export ENABLE_PRODUCT_AI_FUSION=1
export PRODUCT_SOURCE_ROOT=/work/product/point-cloud-processing
export PRODUCT_AI_FUSION_SUPPORT_LIBRARY=/work/product/lib/libpercep_demo.so
export PRODUCT_AI_FUSION_EXTRA_INCLUDE_DIRS='/work/thirdparty/include;/work/common_interface/include'
export PRODUCT_AI_FUSION_EXTRA_LIBRARIES='/work/product/lib/libdependency.so'
bash tools/build_x5_sdk.sh
```

产品库若隐藏 `PclCommon` 等符号，推荐按 `integrations/README.md` 把 adapter 源直接编入现有产品目标；
仅有头文件不代表最终链接可行。Windows 的便携 fallback 不能替代这个链接和板端行为验证。

## 13. X5 部署与运行

在板端先核对 tar 的 sidecar：

```bash
cd /userdata/releases
sha256sum -c perception-sdk-x5-3.3.0-unified-r12-v3.tar.gz.sha256
mkdir -p /userdata/perception-r12-v3
tar -xzf perception-sdk-x5-3.3.0-unified-r12-v3.tar.gz \
  -C /userdata/perception-r12-v3 --strip-components=1
cd /userdata/perception-r12-v3
```

先运行不需要模型的契约检查：

```bash
./run.sh bin/runtime_json_config_tests
./run.sh bin/task_graph_tests
./run.sh bin/staged_inference_tests
./run.sh bin/semantic_fusion_tests
./run.sh bin/runtime_config_tests "$PWD/profiles/runtime_r12_v3_default.json"
./run.sh bin/product_ai_fusion_contract_tests
./run.sh bin/perception_runtime_demo \
  --explain-config "$PWD/profiles/runtime_r12_v3_default.json"
```

随后复制默认 JSON、schema 和 X5 profiles 到新的产品配置目录，填写真实 BIN、外参和 adapter 策略，
再运行冻结样例。不要为了让默认模板启动而关闭标定检查，然后把错误坐标系结果当作有效输出。

板端验收至少记录：模型/BSP/hbDNN 版本和哈希、六 Task 状态、检测/分割精度、点云投影与地图
对照、p50/p95/p99、吞吐、线程数、RSS/PSS、CMA/DMA-BUF、fd、温度、2h/12h/24h 斜率、
相机/视差时间同步及真实 Product AiFusion 结果。

## 14. 源码交付和版本边界

计划中的源码归档为：

```text
dist/perception-x5-perception-engine-3.3.0-r12-v3.zip
```

必须在所有源码、测试、文档和最终日志冻结后运行 `python tools/package_source.py`。归档应具有固定的
`perception-x5-perception-engine-3.3.0-r12-v3/` 根目录、内部 `SOURCE_SHA256SUMS` 和相邻
`.zip.sha256`。打包后任何内容变化都要求重新生成，不能沿用旧摘要。

r12-v3 的 JSON 解析实现未改变既有公开对象布局或虚表，因此继续 ABI/SONAME 5。版本 3.3.0、
头文件、库、CMake package、配置、schema 和 adapter 示例应作为同一交付使用。

## 15. 当前验证边界

截至 2026-09-24，VS2022 x64 Release + ONNX Runtime 的完整 CTest 已 **53/53 通过**，包含安装树、
默认/schema/Windows/X5 AI-only v3 JSON、X5 runner 和外部 3.3 consumer 验证；源码包选择/确定性/manifest 测试亦通过。最终日志见
`docs/logs/r12_v3_ctest_msvc_vs.txt`，源码 ZIP 与 sidecar 位于 `dist`。

X5 交叉编译、SDK tar、真实 BPU、相机、产品 AiFusion、精度、性能和长期稳定性仍为
`NOT VERIFIED`。详细证据与边界见 `docs/verification_r12_v3.md`；Windows 结果不能外推为板端验收。
