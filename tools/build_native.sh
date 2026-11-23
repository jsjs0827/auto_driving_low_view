#!/usr/bin/env bash
# Build the caller against an existing native Linux ROS2 + r36 shared SDK.
set -euo pipefail
native_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
native_fail() { printf 'build_native.sh: %s\n' "$*" >&2; exit 2; }
native_setup=${ROS_SETUP:-/opt/ros/humble/setup.bash}
[[ -f $native_setup ]] || native_fail "Set ROS_SETUP to the existing ROS setup.bash."
[[ -n ${SDK_ROOT:-} ]] || native_fail "Set SDK_ROOT to the native installed perception_full_shared_sdk prefix."
native_sdk=$(cd -- "$SDK_ROOT" && pwd)
[[ -f $native_sdk/lib/cmake/perception_full_shared_sdk/perception_full_shared_sdkConfig.cmake ]] ||
  native_fail "Missing SDK CMake package under $native_sdk/lib/cmake/perception_full_shared_sdk."
native_build=${NATIVE_BUILD_DIR:-$native_root/build/native}
native_install=${INSTALL_ROOT:-$native_root/install/perception_ros_so_demo}
set +u
source "$native_setup"
set -u
stereo_args=()
if [[ -n ${STEREO_HORIZON_ROOT:-} ]]; then
  [[ -f $STEREO_HORIZON_ROOT/include/dnn/hb_dnn.h && -f $STEREO_HORIZON_ROOT/lib/libdnn.so ]] ||
    native_fail "STEREO_HORIZON_ROOT must contain matching include/dnn/hb_dnn.h and lib/libdnn.so."
  stereo_args=(-DR36_BUILD_HORIZON_STEREO_RUNNER=ON
    "-DHORIZON_INCLUDE_DIR=$STEREO_HORIZON_ROOT/include" "-DHORIZON_DNN_LIBRARY=$STEREO_HORIZON_ROOT/lib/libdnn.so")
fi
cmake -S "$native_root/src/perception_ros_so_demo" -B "$native_build" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$native_install" \
  -Dperception_full_shared_sdk_DIR="$native_sdk/lib/cmake/perception_full_shared_sdk" \
  -DR36_BUILD_STEREO_HELPER_TESTS=ON -DBUILD_TESTING=ON "${stereo_args[@]}" "$@"
cmake --build "$native_build" --parallel "${BUILD_JOBS:-4}"
ctest --test-dir "$native_build" --output-on-failure
cmake --install "$native_build"
printf 'Native package installed: %s\n' "$native_install"
