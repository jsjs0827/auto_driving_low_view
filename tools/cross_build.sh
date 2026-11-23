#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
: "${X5_SYSROOT:?Set X5_SYSROOT to the matching ARM sysroot}"
PSDK_ROOT=${PSDK_ROOT:-${SDK_ROOT:-}}
: "${PSDK_ROOT:?Set SDK_ROOT or PSDK_ROOT to the matching v8 ARM SDK}"
: "${STEREO_HORIZON_ROOT:?Set STEREO_HORIZON_ROOT to matching X5 headers/libraries}"
if [[ -f ${ROS_SETUP:-/opt/ros/humble/setup.bash} ]]; then
  set +u; source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"; set -u
fi
unset CMAKE_PREFIX_PATH AMENT_PREFIX_PATH
source "$ROOT/tools/target_ros_env.sh"
BUILD=${ROS_BUILD_ROOT:-$ROOT/build/ros_x5}
INSTALL=${INSTALL_ROOT:-$ROOT/install/perception_ros_so_demo}
cmake -S "$ROOT/src/perception_ros_so_demo" -B "$BUILD" -G Ninja -C "$target_cache" \
  "-DCMAKE_TOOLCHAIN_FILE=$ROOT/cmake/x5-toolchain.cmake" "-DX5_SYSROOT=$X5_SYSROOT" \
  "-DX5_CXX_COMPILER=${X5_CXX_COMPILER:-aarch64-linux-gnu-g++}" \
  "-Dperception_full_shared_sdk_DIR=$PSDK_ROOT/lib/cmake/perception_full_shared_sdk" \
  "-DOpenCV_DIR=${TARGET_OPENCV_DIR:-$X5_SYSROOT/usr/lib/aarch64-linux-gnu/cmake/opencv4}" \
  "-DHORIZON_ROOT=$STEREO_HORIZON_ROOT" -DR36_BUILD_HORIZON_STEREO_RUNNER=ON \
  -DSTEREO_BUILD_CODEC_TESTS=OFF -DR36_BUILD_ONLINE_STEREO_TESTS=OFF \
  -DR36_COMPARE_BUILD_TESTS=OFF -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  "-DCMAKE_INSTALL_PREFIX=$INSTALL" "$@"
cmake --build "$BUILD" --parallel "${BUILD_JOBS:-4}"
cmake --install "$BUILD"
printf 'X5 ROS/native applications installed: %s\n' "$INSTALL"
