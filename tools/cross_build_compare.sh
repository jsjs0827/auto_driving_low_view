#!/usr/bin/env bash
# All standalone native tools; no target ROS packages required.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
: "${X5_SYSROOT:?Set X5_SYSROOT to matching ARM sysroot}"
PSDK_ROOT=${PSDK_ROOT:-${SDK_ROOT:-}}
: "${PSDK_ROOT:?Set SDK_ROOT or PSDK_ROOT to matching v8 ARM SDK}"
: "${STEREO_HORIZON_ROOT:?Set STEREO_HORIZON_ROOT to matching ARM Horizon prefix}"
[[ -f $STEREO_HORIZON_ROOT/include/dnn/hb_dnn.h ]] || { echo 'Missing Horizon development headers' >&2; exit 2; }
BUILD=${COMPARE_BUILD_ROOT:-$ROOT/build/compare_x5}
INSTALL=${INSTALL_ROOT:-$ROOT/install/perception_ros_so_demo}
LINK_DIRS="$PSDK_ROOT/lib;$STEREO_HORIZON_ROOT/lib;$STEREO_HORIZON_ROOT/lib/hbbpu;$STEREO_HORIZON_ROOT/hobot/lib;$X5_SYSROOT/usr/lib;$X5_SYSROOT/usr/lib/aarch64-linux-gnu;$X5_SYSROOT/lib/aarch64-linux-gnu;$X5_SYSROOT/usr/hobot/lib"
flags=${LDFLAGS:-}
IFS=';' read -r -a dirs <<< "$LINK_DIRS"
for directory in "${dirs[@]}"; do [[ ! -d $directory ]] || flags+=" -Wl,-rpath-link,$directory"; done
cmake -S "$ROOT" -B "$BUILD" -G Ninja -DPERCEPTION_BUILD_SDK=OFF \
  "-DCMAKE_TOOLCHAIN_FILE=$ROOT/cmake/x5-toolchain.cmake" "-DX5_SYSROOT=$X5_SYSROOT" \
  "-DX5_CXX_COMPILER=${X5_CXX_COMPILER:-aarch64-linux-gnu-g++}" -DCMAKE_BUILD_TYPE=Release \
  "-DCMAKE_INSTALL_PREFIX=$INSTALL" "-DCMAKE_EXE_LINKER_FLAGS=$flags" -DCMAKE_SKIP_RPATH=ON \
  "-Dperception_full_shared_sdk_DIR=$PSDK_ROOT/lib/cmake/perception_full_shared_sdk" \
  "-DOpenCV_DIR=${TARGET_OPENCV_DIR:-$X5_SYSROOT/usr/lib/aarch64-linux-gnu/cmake/opencv4}" \
  "-DHORIZON_ROOT=$STEREO_HORIZON_ROOT" "-DPSDK_LINK_DEPENDENCY_DIRS=$LINK_DIRS" \
  -DR36_COMPARE_WITH_HORIZON=ON -DSTEREO_BUILD_CODEC_TESTS=OFF \
  -DR36_COMPARE_BUILD_TESTS=OFF -DBUILD_TESTING=OFF "$@"
cmake --build "$BUILD" --parallel "${BUILD_JOBS:-4}"
cmake --install "$BUILD"
printf 'X5 native tools installed: %s\n' "$INSTALL"
