# Ubuntu / X5 交付包 v4

沿用历史 Docker 环境编译请使用 **BUILD_DOCKER_zh.md**：`obs-v004-x5-build` + `/work/obs_v004_x5/sysroot_docker/usr_x5`。新的 `tools/build_in_container.sh` 同时编译业务 SDK、DAG 程序和六组测试 X5 程序，并取回运行包；Ubuntu ROS 工具保留包内预编译版本。

本包包含完整项目源码、DAG 配置、模型、已编译的 X5 ARM 程序和 Ubuntu 22.04 x86_64 / ROS2 Humble 配套程序。C++ 算法和 DAG 实现沿用 v3；本版调整 Ubuntu 构建/打包/运行入口，集成旧版 ADB 返回码兼容，并使用板端相机平台库。交叉编译指令见 BUILD_UBUNTU_zh.md，X5 运行指令及历史默认路径见 RUN_X5_zh.md。

预编译程序来自已成功交叉编译并审计的 v3，原证据在 validation/prior_wsl_build。v4 的主机验证在 validation/ubuntu_v4_checks.json 和 validation/adb_compat_v4.json。DELIVERY_REPORT.json 记录本次实际打包依赖检查。没有 v4 板端相机、模型精度或性能实测结果；之前的相机初始化失败仍需上板复测。

## 在 Ubuntu 解压

```bash
sha256sum -c perception-dag-v4-ubuntu-docker-ready-20261019.tar.gz.sha256
tar -xzf perception-dag-v4-ubuntu-docker-ready-20261019.tar.gz
cd perception-stereo-cpp-v8-dag-v4-ubuntu
sha256sum -c SHA256SUMS
```

已有 ROS2 Humble、ADB、rosbag2 SQLite3 插件即可使用已编译测试工具。需要重新编译时，再准备匹配的 ARM 开发 sysroot；本包不包含 sysroot。

## 六组测试

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=67
adb devices -l
# 多设备时填写实际序列号；只有一台可省略。
export ADB_SERIAL=实际X5序列号

bash tools/run_benchmark6_adb.sh benchmark6_smoke.json
```

短测包含六组，每组性能/质量各跑一遍：20 帧性能统计、3 帧预热、5 帧质量保存。最后完成组数应为 6，12 次运行退出码应全部为 0。

短测通过后执行正式测试：

```bash
bash tools/run_benchmark6_adb.sh benchmark6.json
```

正式测试默认每组性能 200 帧、预热 20 帧、质量 30 帧。入口已集成 ADB 兼容，无需单独覆盖脚本；会先验证远程 exit 7 能恢复为主机退出码 7。默认部署目录 `/userdata/perception_benchmark6_v4`，结果父目录 `/userdata/perception_benchmark6_v4_results`。

首次自动部署；已有同版程序时，比较清单并验证全部文件后自动复用，不需要 B6_REUSE_RUNTIME。目录内容不同或部署不完整时保留现场并停止，应修改配置 board_root 为新目录。每次结果目录必须全新，也可显式指定：

```bash
OUT="$PWD/results/formal_$(date +%Y%m%d_%H%M%S)_$$"
bash tools/run_benchmark6_adb.sh benchmark6.json "$OUT"
```

脚本自动采集同帧左右图、原始 Q8 和内参，随后执行 camera/offline/ros 与 original_q8/stereo_crop 的六组组合。ROS 测试使用实际 Ubuntu DDS 发布采集数据，经 TCP 桥接进入 X5；板端 CPU/RSS 不包括 Ubuntu 发布/桥接进程。

## 结果和可视化

结果默认在 `results/benchmark6_日期时间`，包括 index.html、summary.csv/json、REPORT_zh.md、run_status.tsv、logs、检测/分割/视差 PNG、Q8、PLY、RViz 配置和 rosbag。可视化生成在性能采样结束后执行。

```bash
cat "$OUT/run_status.tsv"
cat "$OUT/visualization_status.txt"
build/benchmark6/r36_benchmark6 --value "$OUT/summary.json" complete_groups
```

有图形界面的 Ubuntu 上启动 RViz，然后在另一个设置了同样 OUT 和 ROS_DOMAIN_ID 的终端播放：

```bash
rviz2 -d "$OUT/runs/offline/stereo_crop/quality/view.rviz"
# 另一个终端：
ros2 bag play "$OUT/visualization/bag" --loop
```

无独立真值时 AP/mIoU/EPE/D1 为 NA，不把原始 Q8 当真值。CPU 100% 表示一个核，BPU 为板端整体使用率，无法读取时为空。原 Q8 导入耗时不等于原算法生成视差的耗时。详细标注格式及指标口径沿用 BENCHMARK6_zh.md。
