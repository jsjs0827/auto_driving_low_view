#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mode=${1:---all}
case "$mode" in
  --help|-h) echo 'Usage: build_ubuntu_x5.sh [--all|--x5-only|--host-only]; set X5_SYSROOT to a matching AArch64 sysroot'; exit 0;;
  --all|--x5-only|--host-only) ;;
  *) echo "Unknown option: $mode" >&2; exit 2;;
esac
[[ $# -le 1 ]] || { echo 'Unexpected arguments' >&2; exit 2; }
export BUILD_JOBS=${BUILD_JOBS:-2}
if [[ $mode != --host-only ]]; then
  export X5_SYSROOT=${X5_SYSROOT:-$(dirname -- "$root")/local_env/x86_ros_v8p1/build/runner_x5/sysroot}
  export X5_CXX_COMPILER=${X5_CXX_COMPILER:-/usr/bin/aarch64-linux-gnu-g++}
  export TARGET_SYSTEM_PREFIX=${TARGET_SYSTEM_PREFIX:-$X5_SYSROOT/usr}
  export STEREO_HORIZON_ROOT=${STEREO_HORIZON_ROOT:-$TARGET_SYSTEM_PREFIX}
  export TARGET_OPENCV_DIR=${TARGET_OPENCV_DIR:-$TARGET_SYSTEM_PREFIX/lib/aarch64-linux-gnu/cmake/opencv4}
  export TARGET_PCL_DIR=${TARGET_PCL_DIR:-$TARGET_SYSTEM_PREFIX/lib/aarch64-linux-gnu/cmake/pcl}
  for path in "$STEREO_HORIZON_ROOT/include/dnn/hb_dnn.h" "$TARGET_OPENCV_DIR/OpenCVConfig.cmake" "$TARGET_PCL_DIR/PCLConfig.cmake"; do
    [[ -f $path ]] || { echo "Missing target development dependency: $path" >&2; echo 'Use a matching complete X5 development sysroot; see BUILD_UBUNTU_zh.md.' >&2; exit 2; }
  done
  machine=$("$X5_CXX_COMPILER" -dumpmachine)
  [[ $machine == aarch64*-linux-* ]] || { echo "Wrong compiler target: $machine" >&2; exit 2; }
  bash "$root/tools/build_x5_v8.sh"
  bash "$root/features/benchmark6/runtime/tools/build_x5.sh"
fi
if [[ $mode != --x5-only ]]; then
  bash "$root/tools/build_benchmark6_host.sh"
fi
echo "Build completed: $mode"
echo 'Target camera/platform libraries are supplied by the X5, not replaced during deployment.'
