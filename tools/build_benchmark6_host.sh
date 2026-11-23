#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
set +u
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
set -u
cmake -S "$root/features/benchmark6" -B "$root/build/benchmark6" -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build "$root/build/benchmark6" --target r36_benchmark6 --parallel "${BUILD_JOBS:-2}"
runtime="$root/features/benchmark6/runtime"
cmake -S "$runtime" -B "$runtime/build/host_ros" -G Ninja -DAB_WITH_ROS=ON -DAB_WITH_X5=OFF -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release "-DCMAKE_INSTALL_PREFIX=$runtime/install/ubuntu_ros"
cmake --build "$runtime/build/host_ros" --target ab_benchmark ab_report ab_config_check ab_result_replay --parallel "${BUILD_JOBS:-2}"
cmake --install "$runtime/build/host_ros" --component ABRuntime
printf 'Host tools built. No device execution or tests were run by this build script.\n'
