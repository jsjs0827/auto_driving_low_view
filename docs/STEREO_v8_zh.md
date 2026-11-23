> **历史材料（v1/v2）**：本文保留用于理解原算法与历史验证。v3 的配置、路径、构建及运行入口已经改为纯 C++；本文旧命令和旧 DAG 配置不能直接用于本版。请以 [09_纯CPP输入与Graph迁移](09_纯CPP输入与Graph迁移.md) 为准。

> 历史说明：本页保留 v8/v8p1 原始操作记录；v10 配置入口和迁移位置见 [配置说明](../config/README_zh.md)，日常启动使用 `bash tools/run_pipeline.sh`。

# 双目输入、ROS 通信与点云输出 v8（历史说明）

本文描述已发布 v8 的外部 runner 方案，保留其验证历史。当前 v8p1 已根据新提供的可读源码加入可编译 runner，编译、配置及限制以 `STEREO_v8p1_zh.md` 和根目录 `README_zh.md` 为准。

v8 在 v7p1 上增加双目数据入口、逐帧 CameraInfo、彩色点云、PLY 与离线 HTML 查看器。原有 `offline/camera/ros` 三种输入和 r36 SDK C ABI 保留。双目数据通过新增 `stereo_source` 发布给算法节点的 `ros` 模式；不会把左图和右图各自走一次普通图片 letterbox。

## 原项目分析

分析对象为 `E:\pro\python_pro\stereo_data100.tar\stereo_data100`：

1. `left/`、`right/` 各 100 张 JPEG，文件 stem 一一对应；首对为 1280×1088。
2. `run_stereo/run_raft_stereo` 是 AArch64 ELF，使用 Horizon `libdnn` 执行 `.bin` 双输入模型。`code/06_raft_stereo/run_raft_stereo` 是 x86 ELF，同样需要 Horizon 推理/仿真依赖，不是 ONNX Runtime。
3. `run_simple.sh` 传入 `--model_file/--left_dir/--right_dir/--output_dir`，产生按图像 stem 命名的浮点像素视差 `.npy`。实际检查的预测形状为 `(544,640)`，不能从模型名/目录名推断输出形状。
4. `convert_result.py` 将视差恢复至标定画幅，并乘 `标定宽度/预测宽度`；`disp_to_pc.py` 用左右内参、八参数畸变、Rodrigues 旋转和平移计算 Q，再重投影为 RGB 点云。原实现导出时翻转 Y/Z，声明 x右/y上/z后、米单位。
5. 原后处理只计算 Q，没有对输入图执行 remap；输入是否已校正必须由采集端确认。v8 显式区分 raw 和 already_rectified，禁止对已有预测的图像再次校正。
6. 原包没有标定文件/ONNX 模型。已找到同名候选标定 `E:\new_data\daye\org_binocular_camera_2533.conf`，数学解析得到基线约 0.0604368m；它是否属于这批图以及图像是否已校正，需要采集信息确认。配置和结果记录确认状态，不能用未确认标定宣称物理测距精度。
7. 唯一 `run_raft_stereo.cc` 当前不是可读文本；用户确认无其它可读副本。v8 因而复用现有可执行程序的 CLI，没有假称合并或重编译其内部模型推理代码。

两个现有视差结果目录的 100 个同名 NPY 实际逐字节相同，不能当作两个模型的独立精度对比。独立测试也实际尝试了原 x86 runner 的 Horizon 仿真路径，但当前旧 runtime 初始化这两个模型失败（`hbDNNInitializeFromFiles rc=-6000002`）；本机可执行验收使用记录视差及 SGBM，未声称重跑原 RAFT 模型成功。

## 三种双目 backend

| backend | 输入与行为 | 当前电脑可用 |
|---|---|---|
| `recorded_npy` | 左右同名图 + 已有浮点视差；复用原项目预测做几何/ROS/导出验收，不再执行模型 | Windows / Linux x86 |
| `sgbm_cpu` | 左右图经校正，再由 OpenCV SGBM 计算真实视差；用于 CPU 调试，是不同算法 | Windows / Linux x86 |
| `x5_runner` | 校正左右图 → 保存同名无损 PNG → 调原 AArch64 runner → 检查全部 NPY 输出配对 | X5 Linux + 原 BPU runtime |

双目样例分别在 `config/stereo_recorded.yaml`、`stereo_sgbm.yaml`、`stereo_x5.yaml`。配置路径相对 YAML 所在目录解析；推荐绝对路径。输出目录必须是新目录，避免覆盖既有结果。`--limit 2` 只运行前两对；0 表示全部。

## 几何与单位

标定文件采用原项目的 colon 键格式，平移单位由 `calibration_unit: mm/m` 明确指定，不按数值猜测。读取左右 `fx/fy/cx/cy`、k0..k5/p0..p1、rotation_r0..2、translation_t0..2、resolution_width/height；缺项、非有限值、零基线或图像尺寸不符立即失败。

未校正图像使用 OpenCV `stereoRectify(CALIB_ZERO_DISPARITY, alpha=0)`、`initUndistortRectifyMap` 和 `remap`，随后在同一个视场下 stretch 为 640×544。P/K 与像素中心缩放同步更新；左右主点一致。已校正输入显式 `already_rectified: true` 跳过 remap。`recorded_npy` 必须为 true，且图像、预测和标定的校正网格必须一致；该开关只是调用方声明，不能自动证明数据来源。

预测视差 resize 至 640×544 后乘 `640/预测宽度`，因此始终是目标画幅的横向像素单位。NaN、Inf、非正值与原无效区不会通过插值凭空变成有效视差。Q8 使用 `floor(d×256+0.5)`，无效/下溢/超过 uint16 范围均置 0，并计数；不会溢出回绕。

SDK 参数为 `bxf=fx×baseline_m×1000`（pixel·mm），重投影深度为 `Z=(bxf/1000)/(Q8/256)` 米。v8 原始立体点云从实际发布的同一 Q8 生成，便于与 SDK 输入一致核验。默认 PLY 和 ROS `/stereo/points` 坐标为**校正后的左目 optical：X右、Y下、Z前，单位米**。HTML 仅在显示时翻转 Y/Z，文件坐标不变。没有相机安装外参和位姿时，只能得到逐帧相机点云，不能据此声称车体/全局融合地图正确。

几何定义参考 [OpenCV 标定与重投影文档](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)。实际代码和测试固定记录所用 OpenCV 版本；不同版本的 stereoRectify 可能产生不同 Q/P，不以旧 PLY 逐字节相等作为跨版本精度判据。

## Windows：先独立生成点云

当前电脑已经有 Windows Python/OpenCV，可直接运行，不依赖 ROS：

```powershell
cd D:\projects\python_pro\gpt_pro\reconstract_code
python local_env/x86_ros_v8/prepare_local.py --platform windows
python perception-r36-ros-so-call-demo-stereo-v8/stereo/stereo_source.py `
  --config local_env/x86_ros_v8/config/windows/stereo_recorded.yaml `
  --output local_env/x86_ros_v8/results/my_recorded_run_01 --limit 2
```

`prepare_local.py` 是本机辅助脚本，使用已发现的 E 盘数据与候选标定；在别的机器上编辑源码包的 portable YAML 即可。它保留 calibration_confirmed/rectification_confirmed=false，待采集信息确认后再改。

要重新计算 CPU 视差，把 config 换成 `stereo_sgbm.yaml`。必须根据输入图是否已校正设置 `already_rectified`；重复 remap 会产生错误对应。Windows `x5_runner` 不会自动降级成 SGBM。

## WSL：编译与 ROS 联测

在 Windows PowerShell 中进入 `wsl -d Ubuntu-22.04`。以下命令在 WSL 中执行：

```bash
cd /mnt/d/projects/python_pro/gpt_pro/reconstract_code/local_env/x86_ros_v8
python3 -m venv --system-site-packages venv
venv/bin/python -m pip install -r ../../perception-r36-ros-so-call-demo-stereo-v8/stereo/requirements.txt
venv/bin/python prepare_local.py --platform wsl
bash build.sh
```

本机构建默认链接已有 `local_env/x86_ros_v6/install/sdk` CPU SDK；该 SDK 的 mapping/recharge 已关闭，因此本机该路径验证 inference + stereo point cloud + ROS 标定通信。它不是 SDK mapping 验收。要链接启用 mapping 的真实 SDK，设置 `SDK_ROOT=/actual/native/sdk` 后重新运行 `build.sh`，并使用对应完整 SDK JSON/地图参数。

在其它 Linux/WSL 机器解压源码包后，安装已有 ROS2、CMake、Ninja 和匹配架构的真实 r36 SDK，使用包内脚本构建；不依赖本机 `local_env` 辅助目录：

```bash
cd /path/to/perception-r36-ros-so-call-demo-stereo-v8
export ROS_SETUP=/opt/ros/humble/setup.bash
export SDK_ROOT=/path/to/native/perception_full_shared_sdk
bash tools/build_native.sh
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r stereo/requirements.txt
export STEREO_PYTHON="$PWD/.venv/bin/python"
# 编辑 config/input_stereo_ros.yaml 中 sdk_config，以及双目 YAML 中数据/标定/输出路径。
bash tools/run_input.sh config/input_stereo_ros.yaml
# 同 ROS_DOMAIN_ID 的另一终端：
bash tools/run_stereo.sh config/stereo_recorded.yaml --ros
```

终端一启动原算法消费者（同一个 ROS_DOMAIN_ID）：

```bash
cd /mnt/d/projects/python_pro/gpt_pro/reconstract_code/local_env/x86_ros_v8
source env.sh
bash ../../perception-r36-ros-so-call-demo-stereo-v8/tools/run_input.sh config/wsl/input_stereo_ros.yaml
```

终端二启动双目发布者：

```bash
cd /mnt/d/projects/python_pro/gpt_pro/reconstract_code/local_env/x86_ros_v8
source env.sh
bash ../../perception-r36-ros-so-call-demo-stereo-v8/tools/run_stereo.sh \
  config/wsl/stereo_recorded.yaml --ros \
  --output "$PWD/results/my_ros_stereo_run_01" --limit 2
```

本机辅助配置关闭 EOF keep_alive；便携样例则开启 keep_alive，便于在 EOF 后查看最后一帧。有限离线输入请先启动消费者，或启用 `ros.wait_for_subscribers: true`。默认等待 left/left_info/right_info；map 模式还应将 disparity 加入 `required_subscribers`。等待有明确 timeout，Ctrl+C/SIGTERM 都会清理本进程创建的 ROS 资源，runner 子进程也会被回收。

## ROS 数据流与可视化

```text
left + right + calibration
       ↓ 校正、统一缩放、视差/Q8
stereo_source ── left NV12 + Q8 + left/right CameraInfo（同 stamp）
       │                    ↓
       │              demo: input_mode=ros
       │              use_camera_info=true
       │                    ↓ r36 SO
       │              result / mask / maps / SDK clouds
       └─ /stereo/points + PLY + HTML（原始立体彩色点云）
```

| 话题 | 类型/内容 |
|---|---|
| `/camera/rgbd_left_nv12` | Image，640×544，NV12，step=640 |
| `/camera/disparity_q8` | Image，16UC1 little-endian，step=1280，Q8.8 |
| `/stereo/left/camera_info`、`/stereo/right/camera_info` | CameraInfo，校正且缩放后的 K/P；右 P[3]=-fx×baseline_m |
| `/stereo/points` | PointCloud2，XYZ float32 米，RGB packed bits，左 rectified optical frame |
| `/stereo/pair` | String JSON，源左右路径/hash、原预测、backend、标定、Q8统计和回放时间 |
| `/perception_ros_so_demo/result` | 原结果 JSON，新增 calibration_source 和本帧 intrinsics |
| `/perception_ros_so_demo/mask` | 原分割 mono8 类别图 |
| `/perception_ros_so_demo/clouds/*` | 可选 SDK camera/robot/ground/obstacle/semantic PointCloud2 |

算法启用 `use_camera_info` 后，左右 info 必须与左图/Q8 exact stamp 配齐；不会用上一帧标定冒充本帧。P 非法、不同尺寸/主点/基线符号都会明确报错。Image 与 Q8 保持相同 frame_id；右 CameraInfo 使用右 optical frame。时间为离线发布时生成的主机时间，明确不是原始采集时间。

`/stereo/points` 与 pair 元数据采用 reliable/transient-local depth1。双目离线样例的 `ros.input_reliability: reliable` 和算法 `input_reliability: reliable` 让 Image/CameraInfo 使用可靠传输、depth4，减少大消息或发现时序导致的丢帧；两侧应配置一致。既有算法 ROS 模式和不填此项的 source 仍默认 best_effort，适用于原相机数据流。Reliable 不代表应用层处理完成确认，消费者仍须先启动，队列/帧率须与处理速度匹配。需要看最后一帧时保持发布者运行，RViz 点云订阅设置 Transient Local。安装了 RViz2 的机器可执行：

```bash
rviz2 -d /mnt/d/projects/python_pro/gpt_pro/reconstract_code/perception-r36-ros-so-call-demo-stereo-v8/config/stereo.rviz
```

无需 RViz 时直接双击每帧的 `cloud.html`，页面自包含，可旋转、平移、滚轮缩放、调节点大小、复位，并下载同目录 PLY。浏览器仅为显示采样，完整 PLY 不按 viewer_max_points 截断；PLY 自身的采样由 cloud_stride 决定。

## X5 编译与运行

本轮没有重编译原 RAFT runner，也没有在 X5 执行它；使用用户已提供的真实 AArch64 程序和模型作为外部依赖。ROS 节点仍按既有交叉编译流程构建：

```bash
cd /home/st/ini_code/perception-r36-ros-so-call-demo-stereo-v8
sudo bash tools/build_in_container.sh \
  --container obs-v004-x5-build \
  --sysroot /work/obs_v004_x5/sysroot_docker \
  --sdk-root /actual/container/install/perception_full_shared_sdk \
  --out /home/st/ini_code/r36_ros_demo_release_v8
```

生成 `perception-r36-ros-so-demo-stereo-v8-x5.tar.gz` 与 `.sha256`。运行包不捆绑外部 SDK、原 stereo runner、模型和数据。先验证包的 SHA256 清单，再编辑 YAML。Python 双目工具依赖 numpy/opencv/PyYAML，可用系统已安装版本，或创建带 system-site-packages 的 venv 并按 `stereo/requirements.txt` 安装。该 venv 必须能看到 ROS2 rclpy，启动前 source ROS setup。

板上配置原 runner/model/左右图/标定实际路径，选择 `config/stereo_x5.yaml`。它一次处理选中的左右对，保存 runner.log/runner.json，检查每对预测均存在；失败时不偷偷回放旧视差或换其它算法。

```bash
export SDK_ROOT=/userdata/ai_infer_r12_v36/perception-core-x5-r12-v36/install/perception_full_shared_sdk
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# 先运行算法消费者，sdk_config须改为板上实际SDK JSON。
bash tools/run_input.sh config/input_stereo_ros.yaml
```

另一终端同 ROS 环境执行：

```bash
bash tools/run_stereo.sh config/stereo_x5.yaml --ros \
  --output /userdata/stereo_results/x5_run_02 --limit 2
```

`already_rectified` 必须按采集事实填写，未校正的原始图 false，已校正 true。模型适配输入/输出由原 runner 执行，其完整内部张量契约因源码不可读未重审；板端需验证具体选用模型与 runner 匹配。

## 接入 SDK mapping 与 SDK PLY

原始立体点云本身不依赖 r36 mapping，infer 模式也会生成 `/stereo/points` 和 cloud.ply。要让 r36 用 Q8 跑地图和处理后的点云：

1. 外部 SDK 必须有 mapping 能力，SDK JSON 的 mapping.enabled=true，配置真实地图参数与该设备安装外参；不要直接套用旧相机车体外参。
2. 算法 YAML `mode: map`、`use_camera_info: true`、`publish_clouds: true`、`save_clouds_ply: true`、`save_every_n: 1`、填写 save_root。
3. SDK JSON 的 `mapping.retain_clouds: true`；否则没有 SDK clouds，不能凭空导出。

SDK cloud 输出按其实际坐标系发布：camera 为本帧左 optical，其余为配置的 robot_frame_id（默认base_link）。camera cloud 原实现通常尚未着色；v8 保留其真实颜色，不把全黑误称彩色。源立体 `/stereo/points` 和 `cloud.ply` 从同一校正左图取 RGB；SDK robot/semantic clouds 颜色按真实SDK输出。

## 结果目录

```text
output.directory/
  run.json / effective_stereo.yaml / complete.json
  runner.log / runner.json / runner_disparity/   # x5_runner 才有
  frame-000001-图像stem/
    cloud.ply / cloud.ply.json                  # 彩色点云，米
    cloud.html                                  # 离线交互查看器
    left.png / right.png / disparity_combo.png
    left.nv12 / disparity.npy / disparity_q8.u16
    frame.json                                  # 源文件、标定和尺度统计
```

算法落盘位置独立，由算法 YAML 的 save_root/save_every_n 控制，含原 result/mask/overlay/source 文件，以及开启时的 SDK `camera.ply/robot.ply/.../clouds.json`。overlay.png 仍需 SDK JSON 的 output.save_overlay=true；它与点云HTML是不同输出。

## 验证边界与复现

产品源码单元测试：`python validation/stereo_core_test.py`、`python validation/viewer_test.py`；C++ helper 测试通过 `-DR36_BUILD_STEREO_HELPER_TESTS=ON` 构建后运行 ctest。它们分别覆盖真实几何/单位/Q8/PLY/SGBM、离线查看器、CameraInfo拒绝/PointCloud2端序/SDK PLY。

本轮实际通过：核心 12 项、查看器 9 项、原生 C++ helper CTest 1 项、打包回归 6 项。Chrome 通过 file:// 离线打开真实样例，WebGL 与强制 Canvas2D 均通过旋转/平移/缩放/复位/点大小操作，无网络请求。原生 WSL 使用真实 SDK + 真实检测/分割 ONNX：reliable 输入模式记录回放 3/3 帧、SGBM 2/2 帧全部完成同 timestamp 端到端核对，PLY 与 PointCloud2 的 XYZ/RGB 一致，5 个 CameraInfo 时序/坏标定负例也通过。此前 best_effort 对照出现首帧丢失，历史失败报告保留，不将它计为无损离线回放成功。

独立测试 task 在共享工作区 `local_env/stereo_v8_test`，负责真实样本回放、CPU双目、ROS/真实ONNX通信和异常路径。最终报告以该目录和随包 `validation/native_v8_logs` / `validation/stereo_v8_logs` 为准。继承的 v7/v7p1 历史 logs 不是 v8 端到端证据。源码包不包含私有数据/模型或测试用仿制 ARM 程序；X5硬件、原BPU模型实时执行和实体相机稳定性必须按实际设备验收。
