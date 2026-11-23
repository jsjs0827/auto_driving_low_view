# r28：业务 DAG、可视化、错误返回和简化 JSON

保留原模型、检测 letterbox、分割前后处理、精确 CPU 缩放和直接 BPU 输入写入。单帧执行，检测和分割同帧并行，无 AI 双缓冲。原框架 53 个文件和数值基线 14 个文件保持原字节。

默认构建独立的精确检测/分割程序、输出 Feature 和业务 DAG 调度测试；加 `--product` 构建复用原 ROS2/daye 环境的完整业务插件。默认 SDK 不包含未编译的完整业务插件。`PRODUCT_CAPABILITIES.json` 明确记录实际构建内容。

## 修改内容

| 请求 | 实现 |
|---|---|
| 优化其他 Feature | 关闭视差可视化时省去整幅 float/灰度图构建；保存辅助向量、地图和图像按实际消费者生成；SHM metadata 仅保留最新一份；不需要原图的地图模式省去 GDC 等待；业务图按运行模式缓存；每个业务 Task 单独计时 |
| 可视化 | `output.visualize` 支持原图、检测、分割、叠加、视差 PNG；同帧元数据、抽帧输出、RGB 工作区复用、离线 gallery；完整业务保留 ROS/RViz/JPG |
| 错误返回 | C++ `Pipeline::tryRun` 返回完整 uint32_t；插件 C ABI 查询错误和 JSON；携带 Feature/Task/帧号/时间戳/native_code；CLI 输出 failed.json |
| 原业务集中 | `product/` 迁入原地图、PatchWork++、语义匹配、AiFusion、标定、回充、发布、记录、离线入口，实际业务函数绑定原 TaskGraph |
| 简化 JSON | 常用配置只写输入、模型、功能开关和输出；自动冻结完整 JSON/启动参数；拒绝未知字段、无效类型和未支持组合 |

完整业务保留原 SHM 订阅、服务、launcher 作为宿主适配器，负责供帧、业务模式和生命周期；每帧算法及结果输出进入 DAG。回充库、异步保存仍有自己的完成过程，不能把提交成功当成最终完成。没有使用 portable 近似算法替代原 PatchWork++/AiFusion。

新增代码的 X5 FPS、CPU、RSS、完整业务输出一致性需要板端测量。历史 r26 的 42.25 FPS 是冻结输入的检测/分割结果，不能当作本版完整业务 FPS。

## Ubuntu 宿主机交叉编译

沿用已有 C++ 编译容器，在 Ubuntu 宿主机执行；不手动进入 Docker，不转换模型。使用 `bash`，不要 `source`。

```bash
cd /home/st/ini_code
sha256sum -c perception-x5-product-dag-r12-v28.tar.gz.sha256
tar -xzf perception-x5-product-dag-r12-v28.tar.gz
cd perception-x5-product-dag-r12-v28

sudo docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
sudo bash tools/build_in_container.sh \
  --container obs-v004-x5-build \
  --sysroot /work/obs_v004_x5/sysroot_docker \
  --out /home/st/ini_code/r28_product_release
```

实际容器名是 `x5-build` 时，只修改 `--container`。默认输出：

```text
/home/st/ini_code/r28_product_release/perception-sdk-x5-r12-v28-product-dag.tar.gz
/home/st/ini_code/r28_product_release/perception-sdk-x5-r12-v28-product-dag.tar.gz.sha256
```

**完整业务插件**需要原工程的真实依赖：

```bash
sudo bash tools/build_in_container.sh \
  --container obs-v004-x5-build \
  --sysroot /work/obs_v004_x5/sysroot_docker \
  --product \
  --deps-root /work/obs_v004_x5/ai_ws/src \
  --recharge-root /work/obs_v004_x5/ai_ws/src/ai_node/point-cloud-processing/recharge_mark \
  --out /home/st/ini_code/r28_full_product_release
```

两个依赖参数均为容器内路径，实际布局不同则填写真实路径。原交叉编译环境还需要可发现的 ROS2/common_interface、PCL、Eigen、OpenCV、yaml-cpp 等。缺失会报告具体依赖，不生成冒充完整业务的包。

如果原工程的 AArch64 ROS2 或 workspace install 位于 sysroot 外，追加 `--cross-prefix "/容器内/ARM_ROS前缀;/容器内/ARM工作空间/install"`，填写实际目录。该参数仅为完整产品增加显式搜索根，不使用 Ubuntu 宿主机的 x86 库。

本机找到 `gecao/ai_ws/src/thirdparty`、`common_interface` 和 AArch64 `libbase_framework.so`，但部分 `.h` 为二进制，无法编译。`charing-apriltag/librecharge_mark.so` 的配套 API 布局与原产品不同，未自动替换。记录见 `validation/r28_local_dependency_inventory.json`。完整业务应使用 Ubuntu 原工程中可编译的头文件和匹配的回充 header/so 对。

容器内只读预检：

```bash
python3 tools/probe_product_dependencies.py \
  --deps-root /work/obs_v004_x5/ai_ws/src \
  --recharge-root /work/obs_v004_x5/ai_ws/src/ai_node/point-cloud-processing/recharge_mark
```

## X5 精确路径和可视化测试

复制宿主机生成的两个 SDK 文件到 X5，再执行：

```bash
mkdir -p /userdata/ai_infer_r12_v28
cd /userdata/ai_infer_r12_v28
sha256sum -c perception-sdk-x5-r12-v28-product-dag.tar.gz.sha256
tar -xzf perception-sdk-x5-r12-v28-product-dag.tar.gz
cd perception-sdk-x5-r12-v28-product-dag

cp profiles/product_exact.json /userdata/r28_exact.json
bash tools/run_product_x5.sh --config /userdata/r28_exact.json --dry-run
bash tools/run_product_x5.sh --config /userdata/r28_exact.json --run
```

修改 SDK 外的 JSON，不要修改受清单校验的发行文件。已有相机和模型路径正确时，最短配置：

```json
{
  "schema_version": 1,
  "run": {"out": "/userdata/r12_v28_res/exact-01", "frames": 300},
  "visualization": {"enabled": true, "every_n": 10, "layers": ["overlay"]}
}
```

实际输入仍为原地图 RGBD 左目 640×544 NV12，配置工具不修改 SDK 交付频率或视差模式。该精确路径有限帧结束。每次用新的 `run.out`。性能组关闭可视化，另开一组测输出开销。

`wall_ms` 从图开始计到 AI 分支完成，`dag_graph_ms` 包含输出 Task；`output_ms` 包含 DAG 可视化和外部保存。AI 各阶段不含 PNG 编码/写盘；实际 FPS 包含整个测量循环工作。

## 完整业务使用

```bash
cp profiles/product_full.json /userdata/r28_product.json
bash tools/run_product_x5.sh --config /userdata/r28_product.json
```

此命令只生成兼容原业务的冻结配置。完整业务由原 daye/ROS2 launcher 加载新 `lib/ai_node/libpercep_demo.so`。需在 JSON `launch.argv` 填实际原启动程序，再加 `--run`；工具不猜测 launcher、不自动覆盖 `/app/daye` 已运行文件。完整业务通过原服务/launcher 停止，不能用精确测试的 `run.frames` 限制其时长。

详细字段见 [JSON 配置说明](docs/r28_config_zh.md)。原功能逐项落点、迁移和依赖见 [完整业务说明](product/README_zh.md)。

## 返回错误码

```cpp
r25::Output output;
r28::Status status;
std::uint32_t code = pipeline.tryRun(image, frame, options, output, status);
if (code != 0) std::cerr << r28::statusJson(status) << '\n';
```

失败时清空返回的推理对象。完整插件 C ABI 声明在 `include/perception/product_status.h`：

```c
uint32_t perception_product_last_status(char *json, size_t capacity, size_t *required);
```

旧 1001..1015 不改号；新增 2001 配置、2002 输出、2003 传输、2004 回充、2005 标定、2006 资源释放错误。Linux 进程退出只有低 8 位，因此 shell 使用 0/1，完整码从 API、JSON 和日志读取。

## 验收

1. 板端先运行原 ARM 预检及同帧输入/输出比较，要求输入零字节差、分割零像素差、检测 bit 一致。
2. 完整业务另比较点云、地图、回充、发布和实际落盘；主机调度测试不能替代这些验证。
3. `product.metrics_dir/product_metrics.csv` 记录各 Task 耗时、进程 CPU 时间/RSS、全局 CMA。同一帧有多行，不要把重复整帧值求和当多个样本。
4. 分开测关闭可视化、抽帧保存、完整可视化，再长测、重开及错误注入。全局 CMA 波动不直接等于进程泄漏。

所有源码/脚本采用 UTF-8 无 BOM、LF，清单按最终字节生成，tar 内逐文件复核。直接解压，不执行自动换行或批量编码转换。
