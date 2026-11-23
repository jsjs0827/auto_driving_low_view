#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
project_root=$(cd -- "$root/../../.." && pwd)
: "${X5_SYSROOT:?Set matching real AArch64 sysroot}"
prefix=${TARGET_SYSTEM_PREFIX:-$X5_SYSROOT/usr}
horizon=${STEREO_HORIZON_ROOT:-$prefix}
build=${AB_BUILD_DIR:-$root/build/x5}
install=${AB_INSTALL_DIR:-$root/install/x5}
ros=OFF
extra=()
if [[ ${1:-} == --with-ros ]]; then
  ros=ON
  : "${TARGET_ROS_PREFIX:?Set real AArch64 ROS2 Humble prefix}"
  [[ -f $TARGET_ROS_PREFIX/lib/librclcpp.so ]] || { echo 'Missing ARM librclcpp.so in TARGET_ROS_PREFIX' >&2; exit 2; }
  ros_header=$(readelf -h "$TARGET_ROS_PREFIX/lib/librclcpp.so")
  [[ $ros_header == *AArch64* ]] || { echo 'TARGET_ROS_PREFIX contains host ROS; real AArch64 ROS libraries are required' >&2; exit 2; }
  extra+=("-DCMAKE_PREFIX_PATH=$TARGET_ROS_PREFIX;$prefix" "-DR36_DEMO_EXTRA_ROOTS=$TARGET_ROS_PREFIX" "-DPython3_EXECUTABLE=${HOST_PYTHON_FOR_CMAKE:-/usr/bin/python3}")
elif [[ $# != 0 ]]; then echo 'Usage: build_x5.sh [--with-ros]' >&2; exit 2; fi
dirs="$prefix/lib;$prefix/lib/aarch64-linux-gnu;$prefix/hobot/lib;$prefix/lib/hbbpu;$X5_SYSROOT/lib/aarch64-linux-gnu"
flags=""
IFS=';' read -r -a parts <<< "$dirs"
for d in "${parts[@]}"; do flags+=" -Wl,-rpath-link,$d"; done
if [[ $ros == ON ]]; then flags+=" -Wl,-rpath-link,$TARGET_ROS_PREFIX/lib"; fi
cmake -S "$root" -B "$build" -G Ninja \
  "-DCMAKE_TOOLCHAIN_FILE=$project_root/cmake/x5-toolchain.cmake" \
  "-DX5_SYSROOT=$X5_SYSROOT" "-DX5_CXX_COMPILER=${X5_CXX_COMPILER:-/usr/bin/aarch64-linux-gnu-g++}" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_LIBRARY_ARCHITECTURE=aarch64-linux-gnu \
  -DAB_WITH_X5=ON "-DAB_WITH_ROS=$ros" -DBUILD_TESTING=OFF \
  "-DCMAKE_INSTALL_PREFIX=$install" "-DHORIZON_ROOT=$horizon" \
  "-DOpenCV_DIR=${TARGET_OPENCV_DIR:-$prefix/lib/aarch64-linux-gnu/cmake/opencv4}" \
  "-DPCL_DIR=${TARGET_PCL_DIR:-$prefix/lib/aarch64-linux-gnu/cmake/pcl}" \
  "-DEigen3_DIR=$prefix/share/eigen3/cmake" "-DBoost_DIR=$prefix/lib/aarch64-linux-gnu/cmake/Boost-1.74.0" \
  "-DR33_RAPIDJSON_INCLUDE_DIR=$prefix/include" "-DR33_TARGET_LZ4_LIBRARY=$prefix/lib/aarch64-linux-gnu/liblz4.so" \
  -DCMAKE_DISABLE_FIND_PACKAGE_VTK=ON -DCMAKE_SKIP_RPATH=ON \
  "-DPERCEPTION_LINK_DEPENDENCY_DIRS=$dirs" "-DPSDK_LINK_DEPENDENCY_DIRS=$dirs" \
  "-DCMAKE_EXE_LINKER_FLAGS=$flags" "${extra[@]}"
cmake --build "$build" --target ab_benchmark ab_report ab_config_check --parallel "${BUILD_JOBS:-2}"
cmake --install "$build" --component ABRuntime
file "$install/bin/ab_benchmark"
(cd "$install" && find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS)
printf 'X5 runtime: %s\n' "$install"
