# 本机 WSL 交叉编译记录与复现方式

使用 WSL2 的 Ubuntu-22.04、GNU aarch64-linux-gnu 11.4 和 CMake 3.22.1，目标为 Linux AArch64。依赖来自工作区已有的 X5 sysroot，未下载或替换板端 SDK。

为了减少 `/mnt/d` 文件访问开销，源码与头文件放在本任务独立的 WSL Linux 目录中；ARM 动态库仍通过只读路径引用现有 sysroot。未修改共享依赖。Release 数值选项沿用项目原值。

复现时在完整源码目录中操作，选择全新的缓存目录。

Windows PowerShell 中先打包头文件：

```powershell
Set-Location D:\projects\python_pro\gpt_pro\reconstract_code\perception-stereo-cpp-v8-dag-v3-benchmark6
New-Item -ItemType Directory -Force build\wsl_cache | Out-Null
tar.exe -cf build\wsl_cache\x5_headers.tar -C D:\projects\python_pro\gpt_pro\reconstract_code\gecao\sysroot_docker\usr_x5 include
```

在 WSL Ubuntu-22.04 内执行：

```bash
cd /mnt/d/projects/python_pro/gpt_pro/reconstract_code/perception-stereo-cpp-v8-dag-v3-benchmark6
CACHE="$HOME/perception_dag_v3_build_$(date +%Y%m%d_%H%M%S)"
bash tools/prepare_wsl_native_cache.sh "$CACHE"
export X5_SYSROOT="$CACHE/sysroot"
export TARGET_SYSTEM_PREFIX="$X5_SYSROOT/usr"
export STEREO_HORIZON_ROOT="$TARGET_SYSTEM_PREFIX"
export TARGET_OPENCV_DIR="$TARGET_SYSTEM_PREFIX/lib/aarch64-linux-gnu/cmake/opencv4"
export TARGET_PCL_DIR="$TARGET_SYSTEM_PREFIX/lib/aarch64-linux-gnu/cmake/pcl"
export CMAKE_GENERATOR=Ninja BUILD_JOBS=4
bash "$CACHE/source/tools/build_release_wsl_benchmark6.sh"
```

生成业务 SDK、业务工具、六组测试的 X5 程序，以及 Ubuntu 主机的汇总、ROS 桥接、结果回放程序。构建日志在 `$CACHE/source/build/wsl_release_logs`。安装树位于 `$CACHE/source/install` 和 `$CACHE/source/features/benchmark6/runtime/install`。

最终交付包另有 ARM ELF、动态库依赖及版本符号检查。板端相机 SDK 和 glibc 由 X5 自身提供。具体已完成的检查以交付包的 `BUILD_REPORT.json` 为准，不能用本机合成数据检查替代真实模型精度或板端性能测试。
