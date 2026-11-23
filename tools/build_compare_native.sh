#!/usr/bin/env bash
# Native config, reports, SDK worker and recorder; no ROS dependency.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
: "${SDK_ROOT:?Set SDK_ROOT to the native matching native SDK}"
BUILD=${COMPARE_BUILD_ROOT:-$ROOT/build/native_tools}
INSTALL=${INSTALL_ROOT:-$ROOT/install/perception_ros_so_demo}
args=(-DR36_COMPARE_WITH_HORIZON=OFF)
if [[ -n ${STEREO_HORIZON_ROOT:-} ]]; then args=(-DR36_COMPARE_WITH_HORIZON=ON "-DHORIZON_ROOT=$STEREO_HORIZON_ROOT"); fi
cmake -S "$ROOT" -B "$BUILD" -G Ninja -DPERCEPTION_BUILD_SDK=OFF \
  -DCMAKE_BUILD_TYPE=Release "-DCMAKE_INSTALL_PREFIX=$INSTALL" \
  "-Dperception_full_shared_sdk_DIR=$SDK_ROOT/lib/cmake/perception_full_shared_sdk" "${args[@]}" "$@"
cmake --build "$BUILD" --parallel "${BUILD_JOBS:-4}"
ctest --test-dir "$BUILD" --output-on-failure
cmake --install "$BUILD"
