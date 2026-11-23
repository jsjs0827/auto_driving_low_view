# Ubuntu 交叉编译

**沿用历史 Docker 环境时，优先按 BUILD_DOCKER_zh.md 执行 `bash tools/build_in_container.sh`。** 默认容器 `obs-v004-x5-build`，sysroot `/work/obs_v004_x5/sysroot_docker`，ARM 依赖前缀为 `usr_x5`。下面是直接在 Ubuntu 主机编译的备用方式。

推荐 Ubuntu 22.04 x86_64。源码、模型和之前编译成功的 X5/Ubuntu 程序均已包含；可以先使用预编译程序。重新编译需要本机工具链和与当前 X5 匹配的 ARM 开发 sysroot，sysroot 不在本包内。

安装主机编译工具（在 Ubuntu 执行）：

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake ninja-build python3 adb \
    gcc-aarch64-linux-gnu g++-aarch64-linux-gnu binutils-aarch64-linux-gnu libopencv-dev
```

Ubuntu 的 ROS 桥接、录包和 RViz 另需已安装 ROS2 Humble，且 `/opt/ros/humble/setup.bash` 存在。在已配置 ROS 软件源的 Ubuntu 上补充插件：

```bash
sudo apt-get install -y ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-default-plugins ros-humble-rviz2
```

Humble 安装说明：https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

## 准备 ARM 开发 sysroot

已有匹配的 sysroot 时直接设置 X5_SYSROOT。历史默认搜索位置仍为项目同级的 `local_env/x86_ros_v8p1/build/runner_x5/sysroot`，可以用环境变量覆盖。

如果当前 X5 本身装有开发头文件和 CMake 包，可以在连接它的 Ubuntu 上同步到全新的主机目录：

```bash
adb devices -l
# 多设备时填写实际序列号；只有一台设备可省略此项。
export ADB_SERIAL=实际X5序列号
bash tools/pull_x5_sysroot.sh "$HOME/x5_sysroot_v4"
export X5_SYSROOT="$HOME/x5_sysroot_v4"
```

该入口只读取板端 `/usr/include`、`/usr/lib`、`/usr/hobot` 和 `/lib` 对应库，写入新的 Ubuntu 目录，不安装或替换板端文件。同步量可能较大。若板端只有运行库，没有 PCL/OpenCV/Horizon 开发包，脚本会明确报缺项；此时应使用匹配的开发 sysroot，不能用 Ubuntu x86_64 库代替。已有的不完整同步目录会保留，重试请选新目录。

sysroot 应至少包含：

```text
usr/include/dnn/hb_dnn.h
usr/lib/aarch64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake
usr/lib/aarch64-linux-gnu/cmake/pcl/PCLConfig.cmake
usr/lib/aarch64-linux-gnu/cmake/Boost-1.74.0/
usr/share/eigen3/cmake/
lib/aarch64-linux-gnu/libc.so.6
```

## 编译命令

在本项目根目录执行；首次使用与本次目标 sysroot 一致的新构建目录。本包没有携带旧 CMakeCache。

```bash
export X5_SYSROOT=/你的匹配X5_sysroot绝对路径
export BUILD_JOBS=2

# 仅交叉编译 X5 业务程序和测试程序，不依赖主机 ROS。
bash tools/build_ubuntu_x5.sh --x5-only

# 编译 Ubuntu 汇总、ROS 桥接、结果回放工具。
source /opt/ros/humble/setup.bash
bash tools/build_ubuntu_x5.sh --host-only

# 也可一次编译以上全部内容：
# bash tools/build_ubuntu_x5.sh --all
```

输出位置：`install/backend_sdk_x5`、`install/perception_ros_so_demo_x5`、`features/benchmark6/runtime/install/x5`、`features/benchmark6/runtime/install/ubuntu_ros`、`build/benchmark6/r36_benchmark6`。

按需覆盖历史目录变量：`TARGET_SYSTEM_PREFIX`、`STEREO_HORIZON_ROOT`、`TARGET_OPENCV_DIR`、`TARGET_PCL_DIR`、`X5_CXX_COMPILER`；默认均由 X5_SYSROOT 推导。

编译完成后重新打交付包（包含源码、已编译程序、模型和通用依赖）：

```bash
python3 tools/package_ubuntu_x5.py --sysroot "$X5_SYSROOT" \
    --output "$PWD/dist/perception-dag-v4-ubuntu-rebuilt.tar.gz"
```

打包器检查 AArch64、动态库依赖和版本符号；相机平台库由实际 X5 提供。X5 上运行见 RUN_X5_zh.md，六组测试见 README_UBUNTU_zh.md。
