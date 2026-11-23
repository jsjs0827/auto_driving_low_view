#!/usr/bin/env bash
# Reproducible build using the existing local WSL X5 sysroot; no ADB/device access.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
bash "$root/tools/check_no_local_payload.sh" "$root"
workspace=$(dirname -- "$root")
export X5_SYSROOT=${X5_SYSROOT:-$workspace/local_env/x86_ros_v8p1/build/runner_x5/sysroot}
export TARGET_SYSTEM_PREFIX=${TARGET_SYSTEM_PREFIX:-$X5_SYSROOT/usr}
export STEREO_HORIZON_ROOT=${STEREO_HORIZON_ROOT:-$TARGET_SYSTEM_PREFIX}
export TARGET_OPENCV_DIR=${TARGET_OPENCV_DIR:-$TARGET_SYSTEM_PREFIX/lib/aarch64-linux-gnu/cmake/opencv4}
export TARGET_PCL_DIR=${TARGET_PCL_DIR:-$TARGET_SYSTEM_PREFIX/lib/aarch64-linux-gnu/cmake/pcl}
export X5_CXX_COMPILER=${X5_CXX_COMPILER:-/usr/bin/aarch64-linux-gnu-g++}
export BUILD_JOBS=${BUILD_JOBS:-3}
logs="$root/build/wsl_release_logs"
mkdir -p "$logs"
printf 'X5_SYSROOT=%s\n' "$X5_SYSROOT"
test -f "$STEREO_HORIZON_ROOT/include/dnn/hb_dnn.h"
test -f "$TARGET_OPENCV_DIR/OpenCVConfig.cmake"
test -f "$TARGET_PCL_DIR/PCLConfig.cmake"
"$X5_CXX_COMPILER" -dumpmachine
cmake --version | head -1
runtime="$root/features/benchmark6/runtime"
bash "$runtime/tools/build_x5.sh" >"$logs/benchmark_x5.log" 2>&1 & benchmark_pid=$!
bash "$root/tools/build_x5_v8.sh" >"$logs/business_x5.log" 2>&1 & business_pid=$!
bash "$root/tools/build_benchmark6_host.sh" >"$logs/ubuntu_host.log" 2>&1 & host_pid=$!
status=0
for job in "$benchmark_pid" "$business_pid" "$host_pid"; do wait "$job" || status=1;done
printf 'Build logs: %s\n' "$logs"
for name in benchmark_x5 business_x5 ubuntu_host; do printf '\n%s\n' "$name";tail -n 8 "$logs/$name.log";done
exit "$status"
