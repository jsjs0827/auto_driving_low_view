> **历史材料（v1/v2）**：本文保留用于理解原算法与历史验证。v3 的配置、路径、构建及运行入口已经改为纯 C++；本文旧命令和旧 DAG 配置不能直接用于本版。请以 [09_纯CPP输入与Graph迁移](09_纯CPP输入与Graph迁移.md) 为准。

> 历史说明：本页保留 v8/v8p1 原始操作记录；v10 配置入口和迁移位置见 [配置说明](../config/README_zh.md)，日常启动使用 `bash tools/run_pipeline.sh`。

# 双目源码接入 v8p1

根据本次可读 `run_raft_stereo.cc` 接入可编译的 Horizon 双目推理库和 `stereo_model_runner`。v8 已发布源码、包和验证结果保留。原 `offline/camera/ros` 三种算法输入及 r36 C ABI 保留。

## 源码修正

附件的 UTF-8/LF 副本位于 `stereo/runner/upstream/run_raft_stereo.cc`，附件hash及规范化说明在同目录 `PROVENANCE.json`。这个副本保留原始缺陷，仅供审查，不参加编译；实际编译 `stereo/runner/src/`。详细审查见 `docs/RAFT_SOURCE_REVIEW_zh.md`。

- 原代码把type=2当NV21；真实API定义为NV12_SEPARATE，需分开的Y、UV内存。
- RGB/BGR图像类型实际使用uint8；只有F32张量写入0..255 float，不额外除255。新实现按模型真实类型/布局处理，模型须与此预处理一致。
- 新代码检查NCHW/NHWC、batch、通道、左右尺寸、aligned shape/stride和输出量化；无法明确支持的格式直接报错。NV12与分离NV12分别分配正确内存，SCALE处理零点，SHIFT不用危险整数左移。
- 原depth PNG代码混用米/毫米，restore后还按旧尺寸分配、按新尺寸写入。新runner只输出原生画幅浮点NPY，校正、尺度、Q8和米制点云统一由Python几何模块处理。
- 原 `--calibration` 不执行图像校正；`--restore_original_resolution` 不改变先前已保存的NPY。这两个原选项不再作为新runner接口。
- RAII管理packed model、tensor memory、task；API/写文件失败返回非零。缺失/重复/额外图像拒绝，已有结果不覆盖。

可读源码不能证明之前的外部ELF由它编译。新程序由本包源码构建，运行时保存真实 `model_contract.json`；Python记录程序与模型SHA256。API依据：[D-Robotics X5 BPU SDK](https://developer.d-robotics.cc/oe_x5_doc/en/runtime/source/bpu_sdk_api/bpu_sdk_api.html#hbdnntensorproperties)。

## 输入与ROS链路

```text
左右同stem图 + 实际标定
  → Python校正/统一视场、准备640x544左右图
  → C++明确resize至模型画幅、按真实张量契约执行双输入推理
  → 原生模型画幅float32 NPY（横向像素视差）
  → Python缩放视差、安全Q8、同stamp的左NV12/Q8/左右CameraInfo
  → 现有ROS算法节点 → r36 SO → 检测/分割/result/mask
  → /stereo/points + 每帧彩色PLY + 离线交互HTML
```

库不偷偷resize/remap，输入图须匹配模型画幅；CLI显式INTER_LINEAR resize。模型输入名字须能识别一left一right，模糊命名拒绝。输出须单batch/单通道且宽等于输入宽；高度可不同，Python会恢复目标高度。张量尺寸不能自动证明模型视差单位，实际模型仍须确认输出是该画幅横向像素视差。

PLY/PointCloud2坐标为rectified left optical：X右/Y下/Z前，单位米；HTML仅显示时翻转Y/Z。有WebGL及Canvas2D模式。无安装外参/位姿时输出逐帧相机点云，不是全局融合地图。

| 配置 | backend | 行为 |
|---|---|---|
| config/stereo_recorded.yaml | recorded_npy | 已有NPY回放，未重跑RAFT |
| config/stereo_sgbm.yaml | sgbm_cpu | 真实CPU SGBM，与RAFT不同 |
| config/stereo_x5.yaml | x5_runner | 本次源码编译的AArch64 runner |
| config/input_stereo_ros.yaml | 算法ROS输入 | 同帧消费左图和标定；map/both还需Q8 |

配置实际左右路径、标定、平移单位mm/m及新输出目录。YAML中的相对路径以YAML所在目录为准；命令行 `--output` 相对路径以当前工作目录为准。图像尺寸须符合标定。原图未校正为 `already_rectified: false`，确已校正为true；已有预测回放必须true，不能在预测后再remap。候选2533标定身份及这批图是否已校正仍未确认，两个confirmed字段保持false，测试不能证明物理精度。

新X5样例 `runner.implementation: v8p1_source` 要求产出model_contract.json。若仍用旧外部程序，填其真实路径并明确改为external，不把旧ELF冒称本次构建。

## Windows回放/CPU点云

```powershell
cd D:\projects\python_pro\gpt_pro\reconstract_code\perception-r36-ros-so-call-demo-stereo-v8p1
# 先编辑YAML中的实际数据、标定及输出路径。
python stereo/stereo_source.py --config config/stereo_recorded.yaml --limit 2
python stereo/stereo_source.py --config config/stereo_sgbm.yaml --limit 2
```

依赖numpy/opencv/PyYAML，可安装 `stereo/requirements.txt`。Windows不执行Horizon BPU或本次ROS联测。`--output NEW_DIR` 覆盖输出目录；`--limit 0`为全部，输出必须是新目录。

## Linux/X5编译runner

需要相同架构且匹配模型的Horizon头文件/libdnn、OpenCV开发包、CMake及C++17编译器；这些外部依赖不随源码包附带。X5本机示例：

```bash
cd /userdata/perception-r36-ros-so-call-demo-stereo-v8p1
cmake -S stereo/runner -B build/stereo_runner -DCMAKE_BUILD_TYPE=Release \
  -DHORIZON_ROOT=/actual/matching/horizon/runtime \
  -DSTEREO_RUNNER_INSTALL_DIR=lib/perception_ros_so_demo \
  -DCMAKE_INSTALL_PREFIX="$PWD/install/perception_ros_so_demo"
cmake --build build/stereo_runner --parallel 4
ctest --test-dir build/stereo_runner --output-on-failure
cmake --install build/stereo_runner
export LD_LIBRARY_PATH=/actual/matching/horizon/runtime/lib:$LD_LIBRARY_PATH
```

源码本机构建后，runner.executable设为 `$PWD/install/perception_ros_so_demo/lib/perception_ros_so_demo/stereo_model_runner`；样例默认采用完整交叉运行包目录，须按实际安装位置修改。

include/lib不在同一prefix时传 `-DHORIZON_INCLUDE_DIR=/actual/include`、`-DHORIZON_DNN_LIBRARY=/actual/lib/libdnn.so`。不要新头链接旧库；x86仿真与AArch64不能混用。

```bash
install/perception_ros_so_demo/lib/perception_ros_so_demo/stereo_model_runner \
  --model_file=/actual/stereo.bin --left_dir=/actual/left --right_dir=/actual/right \
  --output_dir=/actual/new_disparity
# 只检查配对与解码，不执行推理：
install/perception_ros_so_demo/lib/perception_ros_so_demo/stereo_model_runner \
  --left_dir=/actual/left --right_dir=/actual/right --validate_pairs
```

支持批量4参数及单对 `--left/--right/--output`；不接受原GT/旧depth导出选项。目录输出 `stem.npy` 和model_contract.json；新建或空目录才可使用。

## ROS编译与运行

普通Linux/WSL节点继续使用 `SDK_ROOT=/actual/native/perception_full_shared_sdk bash tools/build_native.sh`。同时编译runner可用 `STEREO_HORIZON_ROOT=/actual/matching/horizon/runtime SDK_ROOT=/actual/native/perception_full_shared_sdk bash tools/build_native.sh`；直接CMake构建则传 `-DR36_BUILD_HORIZON_STEREO_RUNNER=ON` 及匹配Horizon include/lib，也可独立编译后装至同一prefix。

已有X5交叉容器流程可额外编译并打包runner：

```bash
sudo bash tools/build_in_container.sh \
  --container obs-v004-x5-build --sysroot /work/obs_v004_x5/sysroot_docker \
  --sdk-root /actual/container/install/perception_full_shared_sdk \
  --stereo-horizon-root /actual/container/matching/horizon/runtime \
  --out /home/st/ini_code/r36_ros_demo_release_v8p1
```

全新构建时省略stereo-horizon-root只构建原ROS节点；RELEASE.json的source_stereo_runner_included如实记录。交叉编译还需要目标toolchain能找到OpenCV。源码包不含预编译X5运行程序。

板上先校验SHA256，再编辑配置。终端一：

```bash
export SDK_ROOT=/userdata/ai_infer_r12_v36/perception-core-x5-r12-v36/install/perception_full_shared_sdk
export ROS_DOMAIN_ID=0
bash tools/run_input.sh config/input_stereo_ros.yaml
```

终端二同ROS环境：

```bash
export ROS_DOMAIN_ID=0
export STEREO_PYTHON=/actual/ros-capable/python
bash tools/run_stereo.sh config/stereo_x5.yaml --ros \
  --output /userdata/stereo_results/source_run_02 --limit 2
```

Python须可import rclpy，可用带system-site-packages的venv。左右图/Q8/CameraInfo同stamp；双目样例两端input_reliability均reliable，原相机默认best_effort保留。点云/pair采用reliable+transient-local。消费者须先启动，队列/帧率须与处理速度匹配。

主要话题是 `/camera/rgbd_left_nv12`、`/camera/disparity_q8`、`/stereo/left/camera_info`、`/stereo/right/camera_info`、`/stereo/points`、`/stereo/pair`；算法输出为 `/perception_ros_so_demo/result`、`mask`。RViz使用config/stereo.rviz。

SDK mapping仍须真实MAPON SDK、正确安装外参、mapping.enabled/retain_clouds=true及算法map/both、publish_clouds/save_clouds_ply/save_every_n/save_root配置；它与源立体点云分别验收。

## 结果及证据

每帧有cloud.ply/json、cloud.html、左右PNG、disparity.npy、disparity_q8.u16、left.nv12、frame.json。runner_disparity/model_contract.json记录真实张量，runner.json记录其hash及程序/模型hash。双击HTML可离线查看及下载同目录PLY。

validation/stereo_v8_logs和native_v8_logs是v8历史证据，不是v8p1新runner推理成功证据。v8p1日志放validation/stereo_v8p1_logs；独立测试在共享工作区local_env/stereo_v8_test保存新结果。X5实际BPU、原模型物理精度及MAPON需要设备和真实标定验收。

## 本工作区的WSL测试入口

现有 Ubuntu-22.04 / ROS2 Humble 环境复用了 v8 Python venv 和 v6 CPU ONNX SDK；本版本节点、runner均在新目录编译，不覆盖历史发布包。仓库根目录下的机器配置及结果位于 `local_env/x86_ros_v8p1`，独立验收位于 `local_env/stereo_v8_test`。这些机器路径不随便携源码包附带。

```bash
cd /mnt/d/projects/python_pro/gpt_pro/reconstract_code
source local_env/x86_ros_v8p1/env.sh
# 离线CPU点云（每次换新的输出目录）：
bash perception-r36-ros-so-call-demo-stereo-v8p1/tools/run_stereo.sh   local_env/x86_ros_v8p1/config/wsl/stereo_sgbm.yaml   --output local_env/x86_ros_v8p1/results/sgbm_manual_01 --limit 2
# 检查原始100对图像；这个命令不推理：
LD_LIBRARY_PATH="$PWD/gecao/sysroot_docker/usr_x86/lib:$LD_LIBRARY_PATH"   local_env/x86_ros_v8p1/install/lib/perception_ros_so_demo/stereo_model_runner   --left_dir=/mnt/e/pro/python_pro/stereo_data100.tar/stereo_data100/left   --right_dir=/mnt/e/pro/python_pro/stereo_data100.tar/stereo_data100/right --validate_pairs
```

要跑ROS算法，在两个终端都source上述env；终端一调用 `bash perception-r36-ros-so-call-demo-stereo-v8p1/tools/run_input.sh local_env/x86_ros_v8p1/config/wsl/input_stereo_ros.yaml`，终端二在上述run_stereo命令后添加 `--ros`。当前机器配置使用已有真实CPU ONNX检测/分割模型，SDK为MAPOFF，源双目PointCloud2/PLY仍可生成。

本机x86 Horizon仿真库为DNN 1.12.3 / HBRT 3.14.25，原两份BIN的builder为1.24.4；新runner实际加载两份模型均返回 `hbDNNInitializeFromFiles=-6000002` / `hbrtErrorIllegalHBM`，没有生成模型推理结果。需要与模型相匹配的运行时或X5设备后才能验证RAFT实际输出；当前可运行的点云测试来自已有NPY或CPU SGBM。

## v8p1已完成的验证

- 完整x86 runner真实链接本地Horizon/OpenCV；ROS包联合构建并安装新runner，CTest 2/2通过。
- 纯张量编码/反量化使用真实旧x86头和X5头分别55/59项断言，ASan/UBSan及泄漏检查无诊断；新头未冒配旧libdnn。
- 独立CLI 29项检查及100对原图配对/解码通过；12项核心检查、18项Python拒绝和Q8哨兵、JSON往返、产品NPY写入格式检查通过。这些不执行或模拟BPU推理。
- 完整同帧ROS回归回放3帧+SGBM2帧使用历史v8节点；最终使用新v8p1节点又分别补测1帧，真实CPU ONNX检测/分割、NV12/Q8/左右CameraInfo/PointCloud2/pair/result/mask/input同stamp，PLY与ROS云一致；5项CameraInfo活体拒绝来自首轮，报告分别保留。
- 两个非法confirmed字段在创建目录前被拒绝，修正后同一路径重试成功。
- 打包回归8项通过，覆盖UTF-8/LF恢复、SHA校验、缺项拒绝及可选runner的真实AArch64架构检查。该打包测试使用实际交叉编译的最小ELF夹具，不是完整X5节点构建或设备运行证据。

原模型成功推理、真实X5 BPU、MAPON及物理深度精度未验收。日志和受测源码/程序哈希在 `validation/stereo_v8p1_logs`；原始独立报告在工作区 `local_env/stereo_v8_test/V8P1_REPORT.md`。

## 独立AArch64 runner交付

新源码已使用现有真实X5 Horizon头/库和目标OpenCV4.5.4完成AArch64交叉编译、严格链接及安装；未执行ARM程序。交付的独立包为 `stereo-model-runner-v8p1-aarch64.tar.gz`，与源码包分开，只有runner及其BUILD/SHA/ELF证据，不是完整ROS运行包，不捆绑模型或系统/Horizon/OpenCV运行库。

安装ELF SHA256：`ce490bb617ec1953237acef6af88d5077a851d378860d40f3ca89bdbc10659d6`。它使用 `/lib/ld-linux-aarch64.so.1`，直接依赖OpenCV imgcodecs/imgproc/core `.so.4.5d`、libdnn及C/C++系统库，无内嵌RPATH/RUNPATH。板上先执行 `sha256sum -c SHA256SUMS` 和 `ldd ./stereo_model_runner` 确认依赖，再用 `--validate_pairs` 检查图片；该选项不执行推理。Python配置中的runner.executable改成解压后绝对路径，运行库路径填到runner.library_dirs。

交叉构建视图仅在本地build目录中恢复原Windows副本的文本软链接和依赖路径，未修改gecao原sysroot。构建、初始依赖错误、最终ELF日志及本机复现脚本放 `validation/stereo_v8p1_logs/parent`。这个编译结果不能代替真实X5模型推理验收。

最终交叉依赖审计解析136个真实AArch64动态库，missing为空；只核验构建侧ELF与依赖闭包，没有执行硬件。安装runner为115360字节，相关路径/hash及源码hash在 `validation/stereo_v8p1_logs/parent/cross_dependency_audit.json`。
