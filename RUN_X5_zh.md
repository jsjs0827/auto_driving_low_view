# X5 运行指令

X5 端运行 ARM 程序；Ubuntu 的 x86_64 程序留在 Ubuntu 上执行。相机 SDK、厂商传感器库及内核平台库使用板端原安装版本。

## 历史默认路径

| 内容 | 默认路径 |
|---|---|
| 相机 SDK | `/app/data/metoak/lib/libMoGeneralSDK.so` |
| 相机配置 | `/app/data/metoak/demo/config/x5_camera.cfg` |
| GDC 文件 | `/app/data/metoak/gdc/`，由厂商配置读取 |
| SDK 第三方库 | `/app/data/metoak/3rdparty_libs/opencv_libs`、`/app/data/metoak/3rdparty_libs/metoak_libs` |
| 平台库 | `/usr/hobot/lib`，另搜索 `/usr/lib/hbbpu` 和系统默认库目录 |
| 本项目 SDK | 解压目录下 `install/backend_sdk_x5` |
| 本项目业务程序 | 解压目录下 `install/perception_ros_so_demo_x5` |

不再打包或推送 sysroot 中的 `libcam.so*`、`libvpf.so*`、`libhbmem.so*`、`libgdcbin.so*`。这些库必须由板端系统提供。通用库及模型仍在包内。`X5_VENDOR_LIBS`、`X5_PLATFORM_LIBS` 可覆盖业务入口的厂商/平台搜索目录；SDK_ROOT、INSTALL_ROOT 可覆盖本项目安装目录。

默认相机路径位于 camera.json 和 benchmark6.json。更改相机 SDK/配置路径时应同步修改这两处；GDC、MIPI、sensor 设置继续由厂商 x5_camera.cfg 管理，不由本包自动改写。

## 部署与检查

在连接 X5 的 Ubuntu 上：

```bash
adb devices -l
export ADB_SERIAL=实际X5序列号
adb -s "$ADB_SERIAL" push perception-dag-v4-ubuntu-20261019.tar.gz /userdata/
adb -s "$ADB_SERIAL" shell
```

进入 X5 后执行以下命令。解压位置应为空闲的新位置，避免覆盖正在使用的版本：

```bash
cd /userdata
tar -xzf perception-dag-v4-ubuntu-20261019.tar.gz
cd perception-stereo-cpp-v8-dag-v4-ubuntu
bash tools/run_x5.sh check
```

要求 AArch64、glibc >=2.35。check 只检查依赖和文件，不打开相机。输出若有 not found，需要先补齐与当前板端镜像匹配的库。Ubuntu 配套工具不在 X5 上运行。

## 运行 DAG 业务流程

仅准备配置并检查引用文件：

```bash
bash tools/run_x5.sh prepare-original
bash tools/run_x5.sh prepare-stereo
```

检测、分割、原始 Q8 视差：

```bash
bash tools/run_x5.sh original
```

上一任务退出后，再运行检测、分割、新 crop 视差：

```bash
bash tools/run_x5.sh stereo
```

结果默认位于 `/userdata/perception_dag_v4/` 下每次新建的运行目录。可以显式指定新目录：

```bash
bash tools/run_x5.sh stereo /userdata/perception_dag_v4/my_stereo_001
```

流程仍使用 `tasks` 与 `dag.sequence/parallel`，配置示例是 camera_original_q8.json 和 camera_stereo.json。修改 DAG 后运行自定义配置：

```bash
bash tools/run_business_x5.sh run --config /绝对路径/my_pipeline.json \
    --run-dir /userdata/perception_dag_v4/my_custom_001
```

## 六组测试与相机状态

三输入源乘两种视差的完整测试由 Ubuntu 发起，包含 Ubuntu ROS2 DDS、ADB TCP 桥接和 X5 推理，入口见 README_UBUNTU_zh.md。上述业务 DAG 运行命令本身不会生成六组统计报告。

之前板端出现 hbn_camera_create=-1、moOpenCamera=-3；本版去掉了已确认的包内/板端相机平台库混用，但相机打开是否恢复仍需实际验证。若仍失败，应对照厂商 /app/data/metoak/demo/run.sh 和 s316_basic_sample 的运行结果，检查 sensor 库、厂商配置和设备状态；不能仅从 cam-service 进程存在判断占用。
