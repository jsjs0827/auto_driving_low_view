#!/usr/bin/env bash
# Build the feature SDK, recorder, model inspector and benchmark. ROS is optional.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
WORKSPACE=$(dirname -- "$ROOT")
with_ros=0
case "${1:-}" in
  --with-ros) with_ros=1; shift;;
  --help|-h) echo 'Set X5_SYSROOT (real AArch64 sysroot); optionally TARGET_ROS_PREFIX, then bash tools/build_x5_v7.sh [--with-ros]'; exit 0;;
  '') ;;
  *) echo "Unknown option: $1" >&2; exit 2;;
esac
[[ $# == 0 ]] || { echo 'Unexpected arguments' >&2; exit 2; }
export X5_SYSROOT=${X5_SYSROOT:-$WORKSPACE/local_env/x86_ros_v8p1/build/runner_x5/sysroot}
export X5_CXX_COMPILER=${X5_CXX_COMPILER:-/usr/bin/aarch64-linux-gnu-g++}
export STEREO_HORIZON_ROOT=${STEREO_HORIZON_ROOT:-$X5_SYSROOT/usr}
export TARGET_SYSTEM_PREFIX=${TARGET_SYSTEM_PREFIX:-$STEREO_HORIZON_ROOT}
export TARGET_OPENCV_DIR=${TARGET_OPENCV_DIR:-$TARGET_SYSTEM_PREFIX/lib/aarch64-linux-gnu/cmake/opencv4}
export PSDK_ROOT=${PSDK_ROOT:-$ROOT/install/backend_sdk_x5}
export INSTALL_ROOT=${INSTALL_ROOT:-$ROOT/install/perception_ros_so_demo_x5}
export BUILD_JOBS=${BUILD_JOBS:-2}
[[ -d $X5_SYSROOT ]] || { echo "Missing AArch64 sysroot: $X5_SYSROOT; set X5_SYSROOT." >&2; exit 2; }
if ((with_ros)); then
  : "${TARGET_ROS_PREFIX:?--with-ros requires a real AArch64 ROS2 prefix, not the host x86 ROS}"
fi
LINK_DIRS="$STEREO_HORIZON_ROOT/lib;$TARGET_SYSTEM_PREFIX/lib;$TARGET_SYSTEM_PREFIX/lib/aarch64-linux-gnu;$TARGET_SYSTEM_PREFIX/hobot/lib;$X5_SYSROOT/usr/lib;$X5_SYSROOT/usr/lib/aarch64-linux-gnu;$X5_SYSROOT/usr/hobot/lib"
bash "$ROOT/tools/build_backend_sdk.sh" --backend bpu --horizon-root "$STEREO_HORIZON_ROOT" \
  --toolchain "$ROOT/cmake/x5-toolchain.cmake" \
  --build-dir "${SDK_BUILD_ROOT:-$ROOT/build/backend_x5}" --install-dir "$PSDK_ROOT" -- \
  "-DX5_SYSROOT=$X5_SYSROOT" "-DX5_CXX_COMPILER=$X5_CXX_COMPILER" \
  -DCMAKE_LIBRARY_ARCHITECTURE=aarch64-linux-gnu \
  "-DFLANN_LIBRARY_SHARED=${TARGET_FLANN_LIBRARY:-$TARGET_SYSTEM_PREFIX/lib/aarch64-linux-gnu/libflann_cpp.so}" \
  "-DPCL_DIR=${TARGET_PCL_DIR:-$TARGET_SYSTEM_PREFIX/lib/aarch64-linux-gnu/cmake/pcl}" \
  "-DOpenCV_DIR=$TARGET_OPENCV_DIR" \
  "-DEigen3_DIR=${TARGET_EIGEN_DIR:-$TARGET_SYSTEM_PREFIX/share/eigen3/cmake}" \
  "-DBoost_DIR=${TARGET_BOOST_DIR:-$TARGET_SYSTEM_PREFIX/lib/aarch64-linux-gnu/cmake/Boost-1.74.0}" \
  "-DR33_RAPIDJSON_INCLUDE_DIR=${TARGET_RAPIDJSON_INCLUDE_DIR:-$TARGET_SYSTEM_PREFIX/include}" \
  "-DR33_TARGET_LZ4_LIBRARY=${TARGET_LZ4_LIBRARY:-$TARGET_SYSTEM_PREFIX/lib/aarch64-linux-gnu/liblz4.so}" \
  -DCMAKE_DISABLE_FIND_PACKAGE_VTK=ON -DCMAKE_SKIP_RPATH=ON \
  "-DPERCEPTION_LINK_DEPENDENCY_DIRS=$LINK_DIRS"
bash "$ROOT/tools/cross_build_compare.sh"
if ((with_ros)); then
  bash "$ROOT/tools/cross_build.sh"
fi
printf 'SDK_ROOT=%s\nINSTALL_ROOT=%s\n' "$PSDK_ROOT" "$INSTALL_ROOT"
