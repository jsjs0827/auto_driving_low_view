# V1 宿主接入 V2 SDK Q8 地图及回充

V1 保留原 NV12、GDC 共享内存回调，以及原地图、回充和待机命令。V2 执行地图和回充计算，V1 负责把结果转成原 ROS 消息。地图使用相机 SDK Q8 视差，Full/Crop 关闭。V1 暗场的原曝光增益双窗口仍由原相机回调驱动。

## 目录

| 路径 | 内容 |
|---|---|
| `project/point-cloud-processing-main` | 修改后的 V1 全量工程，包含所需模型和 SDK |
| `project/point-cloud-processing-main/perception_v2_sdk_x5` | AArch64 V2 SDK 头文件、动态库和回充桥接库 |
| `dependencies/common_interface-main` | V1 既有生产消息和相机共享内存结构 |
| `runtime-x5/lib/ai_node` | AArch64 感知模块和相关算法动态库 |
| `validation` | X5 逐帧对比、复现入口及摘要 |

编译和运行继续使用产品现有 `base_framework`、ROS 2 Humble 和 X5 系统库。包内的回充 SDK 和基准 `charing-main.zip` 中选定的原始 SO 字节相同。

## 配置和模式

编辑 `config/start_config.json` 中的 `v2_backend`，部署后对应 `/app/daye/config/ai_node/start_config.json`。

| map_enabled | recharge_enabled | 计算路径 |
|---|---|---|
| false | false | 原 V1 地图和回充 |
| true | false | V2 SDK Q8 地图，原 V1 回充 |
| true | true | V2 SDK Q8 地图和 V2 GDC 回充 |

启用 V2 时使用在线输入，并设置 `offline.enable=false`、`stereo.enabled=false`、`auto_cali=false`。地图配置在 `config/v2_bridge/map_q8.json`，回充配置在 `config/v2_bridge/recharge_gdc.json`。生产配置的默认根目录为 `/app/daye/config/ai_node`。隔离验证可通过环境变量 `PERCEP_CONFIG_DIR` 指向复制出来的一组配置。保留安装后 `v2_bridge` 与 `start_config.json`、`extrinsics.json`、`recharge_mark.json` 的相对位置。地图模型默认取 `/app/daye/model`。

NV12 和 SDK Q8 必须为同次捕获，保留两路原帧号和时间戳。地图标定应与左目、Q8 的像素网格一致，`bxf` 按 px·mm 传给 SDK。回充读取 1280×1088 左目 GDC，沿用 `recharge_mark.json` 的内部标定和发布开关。当前 V2 回充配置要求 `publish_overlay=false`。启用 V2 地图并开启保存时，地图和中间结果位于 V1 的 `output_dir/v2`，交由原保存线程写盘。

## 运行调用链

`onCameraFrame → algThread → makeV2MapInput → psdk_full_process → GridMapInfo/三张俯视图`

`onGdcFrame → algThread → makeV2RechargeInput → psdk_full_process → DockPose`

`/to_percep_cmd` 使用原来 1、2、21。切换模式等待正在执行的帧完成，清除待处理队列，再响应切换。地图继续发布到 `/percep_demo/grid_map`，回充继续发布到 `/percep_demo/recharge_mark_pose`。回充无有效姿态时三个值均为 `999.f`。

## X5 编译和验证

本机 Docker SOP 工作区为 `/home/st/ini_code/pcl_Ini/work/robot-docker-sop-20261120/workspace/ai_ws`。重编时在独立的同布局工作区中放入本包的 `point-cloud-processing-main` 和 `common_interface-main`，沿用该工作区的 `src/thirdparty`、`robot_dev_config` 和 `sysroot_docker`。在 Docker 容器内执行 `./robot_dev_config/build.sh -p X5 -s percep_main`。SDK 头文件及动态库、`config/v2_bridge` 会随工程使用。部署时 `libpercep_main.so`、V2 SDK、回充桥接库与原回充库放在同一个 `lib/ai_node` 目录。

本次构建使用本机 AArch64 GCC 11、之前 Docker 工作区的 sysroot 与匹配板端依赖。回放清单每行六列，依次为 `sample_id group_id frame_id timestamp_ns left disparity`。回充最后一列填 `-`。每个独立地图场景组建立一个新引擎，和原版基准保持相同初始化边界。位姿坐标为米、米、弧度。

```bash
./v2_q8_recharge_replay map cfg/v2_bridge/map_q8.json map.tsv intrinsics.txt out-map
./v2_q8_recharge_replay original-recharge cfg/recharge_mark.json recharge.tsv intrinsics.txt out-base
./v2_q8_recharge_replay recharge cfg/v2_bridge/recharge_gdc.json recharge.tsv intrinsics.txt out-v2
```

回放命令需要预先建立输出目录，并让 `LD_LIBRARY_PATH` 包含本包算法库及 X5 ROS 运行库。回放配置可覆盖模型的路径。地图内参文本按 `width height fx fy cx cy bxf` 排列。原版回充和 V2 回充分别启动进程，避免共享的原 SDK 会话影响结果。
