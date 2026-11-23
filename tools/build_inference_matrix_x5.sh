#!/usr/bin/env bash
# Build the current project SDK and ROS application for the inference matrix.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

usage() {
  cat <<'HELP'
Usage: bash tools/build_inference_matrix_x5.sh

Required environment:
  X5_SYSROOT        Existing matching AArch64 X5 development sysroot.
  TARGET_ROS_PREFIX Existing AArch64 ROS2 prefix; multiple prefixes use semicolons.

Optional environment is passed to tools/build_x5_v8.sh, including:
  TARGET_SYSTEM_PREFIX, STEREO_HORIZON_ROOT, X5_CXX_COMPILER, BUILD_JOBS,
  PSDK_ROOT, INSTALL_ROOT, SDK_BUILD_ROOT, COMPARE_BUILD_ROOT, ROS_BUILD_ROOT.

This entry builds the current SDK and applications with --with-ros. It does not
start Docker, install dependencies, modify the shared sysroot, or run X5 tests.
The ROS CMake build also requires the target nav_msgs package.
HELP
}

fail() { printf 'build_inference_matrix_x5.sh: %s\n' "$*" >&2; exit 2; }
if (($#)); then
  if [[ $# == 1 && ( $1 == --help || $1 == -h ) ]]; then usage; exit 0; fi
  usage >&2
  fail 'Unexpected arguments; configure the build through environment variables.'
fi

[[ -n ${X5_SYSROOT:-} ]] || fail 'Set X5_SYSROOT to the matching AArch64 development sysroot.'
[[ -n ${TARGET_ROS_PREFIX:-} ]] || fail 'Set TARGET_ROS_PREFIX to a real AArch64 ROS2 prefix.'
[[ $X5_SYSROOT == /* && -d $X5_SYSROOT ]] || fail "X5_SYSROOT must be an existing absolute directory: $X5_SYSROOT"
[[ $TARGET_ROS_PREFIX != *$'\n'* && $TARGET_ROS_PREFIX != *$'\r'* ]] ||
  fail 'TARGET_ROS_PREFIX must be one semicolon-separated line.'
[[ $TARGET_ROS_PREFIX != \;* && $TARGET_ROS_PREFIX != *\; && $TARGET_ROS_PREFIX != *\;\;* ]] ||
  fail 'TARGET_ROS_PREFIX contains an empty prefix.'
IFS=';' read -r -a matrix_ros_prefixes <<< "$TARGET_ROS_PREFIX"
for matrix_ros_prefix in "${matrix_ros_prefixes[@]}"; do
  [[ $matrix_ros_prefix == /* && -d $matrix_ros_prefix ]] ||
    fail "Each TARGET_ROS_PREFIX must be an existing absolute directory: $matrix_ros_prefix"
done

export X5_SYSROOT TARGET_ROS_PREFIX
export PSDK_ROOT=${PSDK_ROOT:-$ROOT/install/backend_sdk_x5}
export INSTALL_ROOT=${INSTALL_ROOT:-$ROOT/install/perception_ros_so_demo_x5}
bash "$ROOT/tools/build_x5_v8.sh" --with-ros

printf '\nSDK_ROOT=%s\nINSTALL_ROOT=%s\n' "$PSDK_ROOT" "$INSTALL_ROOT"
cat <<'NEXT'
Build complete. The six input/method combinations still need runtime evidence:
  camera/original_q8: vendor camera SDK + config, same-capture left + genuine Q8.
  camera/stereo_crop: vendor camera SDK + config, rectified left/right + crop model.
  ros/original_q8: target ROS2, publisher, exact-stamp left/Q8 and calibration.
  ros/stereo_crop: target ROS2, publisher, exact-stamp left/right and calibration.
  offline/original_q8: recorded genuine Q8 + corresponding left + calibration;
                       pictures alone cannot reproduce the camera SDK disparity.
  offline/stereo_crop: rectified paired pictures + calibration + crop model.
All combinations require the detection/segmentation models and matching board
runtime libraries. Verify ldd for libperception_sdk.so and the selected runner;
the target ROS build resolves nav_msgs and all remaining ROS dependencies.
NEXT
