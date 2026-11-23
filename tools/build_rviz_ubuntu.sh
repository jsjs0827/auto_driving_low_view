#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
setup=${ROS_SETUP:-/opt/ros/humble/setup.bash}
[[ -f $setup ]] || { echo "Set ROS_SETUP to an actual Ubuntu ROS2 setup.bash" >&2; exit 2; }
set +u
source "$setup"
set -u
cmake -S "$ROOT/features/rviz" -B "${RVIZ_BUILD_ROOT:-$ROOT/build/rviz_ubuntu}" \
  -DCMAKE_BUILD_TYPE=Release -DR36_RVIZ_BUILD_RECEIVER=ON \
  -DR36_RVIZ_BUILD_PUBLISHER=OFF -DR36_RVIZ_BUILD_SDK_OUTPUT=OFF \
  -DCMAKE_INSTALL_PREFIX="${RVIZ_INSTALL_ROOT:-$ROOT/install/rviz_ubuntu}" "$@"
cmake --build "${RVIZ_BUILD_ROOT:-$ROOT/build/rviz_ubuntu}" --parallel "${BUILD_JOBS:-4}"
cmake --install "${RVIZ_BUILD_ROOT:-$ROOT/build/rviz_ubuntu}"
