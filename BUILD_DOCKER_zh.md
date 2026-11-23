# 使用历史 Docker 环境交叉编译 X5

**当前 v6 三种输入六组推理测试**请使用 [`tools/build_inference_matrix_in_container.sh` 的编译与部署教程](docs/V6_三种输入六组推理测试.md)。它直接构建当前 SDK 和 ARM ROS 程序，生成 `perception-v6-matrix-x5.tar.gz`，无需先执行 `--host-only`。若旧 `tools/build_in_container.sh` 报 `Missing Ubuntu host tool: build/benchmark6/r36_benchmark6`，这是旧业务与 benchmark6 打包链的主机工具前置检查；当前六组测试应切换到专用入口。

以下保留 v4 历史打包流程，供需要旧 benchmark6 主机工具的环境参考。

在连接 X5 的远程 Ubuntu 主机执行，复用此前已存在的容器。

| 项目 | 历史默认值 |
| --- | --- |
| 容器 | `obs-v004-x5-build` |
| 容器 sysroot | `/work/obs_v004_x5/sysroot_docker` |
| ARM 依赖前缀 | `/work/obs_v004_x5/sysroot_docker/usr_x5` |
| 容器交叉编译器 | `/usr/bin/aarch64-linux-gnu-g++` |
| Ubuntu 项目父目录 | `/home/st/ini_code` |

无需重新拉镜像。可用 `docker inspect --format '{{.Config.Image}}' obs-v004-x5-build` 查询已有容器的实际镜像。

## 一键编译与打包

将本次 Docker 构建版交付包放在 `/home/st/ini_code` 后执行：

```bash
cd /home/st/ini_code
sha256sum -c perception-dag-v4-ubuntu-docker-ready-20261019.tar.gz.sha256
WORK=/home/st/ini_code/perception_docker_v4_$(date +%Y%m%d_%H%M%S)_$$
mkdir "$WORK"
tar -xzf perception-dag-v4-ubuntu-docker-ready-20261019.tar.gz -C "$WORK"
cd "$WORK/perception-stereo-cpp-v8-dag-v4-ubuntu"

bash tools/build_in_container.sh \
  --container obs-v004-x5-build \
  --sysroot /work/obs_v004_x5/sysroot_docker \
  --jobs 2 \
  --out "/home/st/ini_code/perception_dag_v4_docker_$(date +%Y%m%d_%H%M%S)_$$"
```

请解压到全新的项目目录，避免覆盖已有修改。当前用户需要有 Docker 使用权限；历史环境要求 sudo 时，在最后一条命令的 `bash` 前加 `sudo`。

脚本会启动已停止的指定容器，在容器全新的 `/tmp/perception-dag-v4-docker-*` 目录复制源码，排除旧构建缓存，依次编译本次业务 SDK、DAG 程序和六组测试的 X5 程序，再审计 ARM 依赖、打包并取回 Ubuntu。终端最后打印实际输出目录，包含：

```text
perception-dag-v4-ubuntu-docker.tar.gz
perception-dag-v4-ubuntu-docker.tar.gz.sha256
build.log
environment.txt
```

镜像需已有 CMake、Ninja、Python3、readelf、file、AArch64 GCC/G++，以及历史 sysroot 中的 ARM Horizon、OpenCV、PCL、Eigen、Boost 等开发文件。入口不会安装依赖或修改共享 sysroot。失败时保留构建目录和日志。

`usr_x5` 前缀同时用于编译与打包。自定义依赖位置可增加 `--dependency-prefix /实际/ARM前缀`、`--stereo-horizon-root /实际/Horizon前缀`。

## Ubuntu 配套工具

Docker 入口保留包内已编译的 Ubuntu 22.04 x86_64 / ROS2 Humble 汇总、ROS 桥接和回放程序，旧容器无需安装 ROS。需要重编主机工具时，先在 **Ubuntu 主机** 执行，再运行 Docker 入口：

```bash
source /opt/ros/humble/setup.bash
bash tools/build_ubuntu_x5.sh --host-only
```

## 历史 sysroot 的 usr 软链接

旧文档曾记录解包后 `usr` 软链接失效的问题。可以只读检查：

```bash
docker exec obs-v004-x5-build bash -lc '
  ls -ld /work/obs_v004_x5/sysroot_docker/usr
  test -d /work/obs_v004_x5/sysroot_docker/usr/include
'
```

若检查失败，入口会明确停止。应先确认这是历史解包产生的断链，再按原环境的 sysroot 维护方式修复。不要覆盖一个已有正常 `usr` 目录。

## X5 部署与运行

将 `OUT` 改为终端实际打印的 Ubuntu 输出目录：

```bash
OUT=/home/st/ini_code/实际输出目录
cd "$OUT"
sha256sum -c perception-dag-v4-ubuntu-docker.tar.gz.sha256
adb push perception-dag-v4-ubuntu-docker.tar.gz /userdata/
adb shell
```

进入 **X5 shell** 后执行，使用新的部署目录：

```bash
DEPLOY=/userdata/perception_dag_v4_docker_$(date +%Y%m%d_%H%M%S)_$$
mkdir "$DEPLOY"
tar -xzf /userdata/perception-dag-v4-ubuntu-docker.tar.gz -C "$DEPLOY"
cd "$DEPLOY/perception-stereo-cpp-v8-dag-v4-ubuntu"
bash tools/run_x5.sh check
bash tools/run_x5.sh original
bash tools/run_x5.sh stereo
```

原始 Q8 和 crop 新视差两条流程依次运行。相机 SDK、配置、GDC 和原生平台库沿用 RUN_X5_zh.md 的历史默认路径。编译成功不代表相机初始化问题已解决。

六组测试在 **Ubuntu 主机** 解压新编译的运行包后执行：

```bash
mkdir "$OUT/benchmark-run"
tar -xzf "$OUT/perception-dag-v4-ubuntu-docker.tar.gz" -C "$OUT/benchmark-run"
cd "$OUT/benchmark-run/perception-stereo-cpp-v8-dag-v4-ubuntu"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=67
bash tools/run_benchmark6_adb.sh benchmark6_smoke.json
# 短测通过后：
bash tools/run_benchmark6_adb.sh benchmark6.json
```

结果与 RViz/rosbag 保存说明见 README_UBUNTU_zh.md 和 BENCHMARK6_zh.md。本次提供 Docker 构建入口；当前本机 Docker 引擎未运行、无法访问远程 Ubuntu 容器，尚未执行该容器中的真实编译。成功执行后，运行包会生成真实的 `validation/docker_build.json` 构建记录。
