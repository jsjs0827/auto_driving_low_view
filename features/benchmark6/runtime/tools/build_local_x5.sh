#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
workspace=$(dirname -- "$root")
export X5_SYSROOT="$workspace/local_env/x86_ros_v8p1/build/runner_x5/sysroot"
export STEREO_HORIZON_ROOT="$X5_SYSROOT/usr"
export TARGET_SYSTEM_PREFIX="$X5_SYSROOT/usr"
export BUILD_JOBS=2
mkdir -p "$root/build/evidence"
bash "$root/tools/build_x5.sh" "$@" 2>&1 | tee "$root/build/evidence/x5_build.log"
