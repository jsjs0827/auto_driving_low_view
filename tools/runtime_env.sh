#!/usr/bin/env bash
# Shared environment and native executable lookup; source this file.
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export PERCEPTION_PROJECT_ROOT=${PERCEPTION_PROJECT_ROOT:-$ROOT}
# Standalone native experiments do not need ROS setup or DDS libraries.
load_native_runtime_environment() {
  if [[ -n ${SDK_ROOT:-} ]]; then
    export LD_LIBRARY_PATH="$SDK_ROOT/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  fi
}
load_runtime_environment() {
  local setup=${ROS_SETUP:-} prefix=${INSTALL_ROOT:-$ROOT/install/perception_ros_so_demo}
  if [[ -n $setup && ! -f $setup ]]; then
    echo "Missing ROS_SETUP: $setup" >&2; return 2
  fi
  if [[ -z $setup ]]; then
    for setup in /app/humble/setup.bash /opt/tros/humble/setup.bash /opt/ros/humble/setup.bash; do
      [[ ! -f $setup ]] || break
    done
  fi
  set +u
  if [[ -f $setup ]]; then source "$setup"; fi
  if [[ -f $prefix/share/perception_ros_so_demo/local_setup.bash ]]; then
    source "$prefix/share/perception_ros_so_demo/local_setup.bash"
  fi
  set -u
  if [[ -n ${SDK_ROOT:-} ]]; then export LD_LIBRARY_PATH="$SDK_ROOT/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"; fi
}
native_executable() {
  local name=$1 candidate prefix=${INSTALL_ROOT:-$ROOT/install/perception_ros_so_demo}
  if [[ $name == perception_config && -n ${PERCEPTION_CONFIG:-} ]]; then
    [[ -x $PERCEPTION_CONFIG ]] || { echo "Not executable: $PERCEPTION_CONFIG" >&2; return 2; }
    printf '%s\n' "$PERCEPTION_CONFIG"; return
  fi
  for candidate in "$prefix/bin/$name" "$prefix/lib/perception_ros_so_demo/$name" \
    "$ROOT/build/native_tools/scheduling/config/$name" "$ROOT/build/native_tools/compare/native/$name" \
    "$ROOT/build/native_tools/compare/camera/$name" "$ROOT/build/native_tools/features/slip_app/$name" \
    "$ROOT/build/compare_x5/features/slip_app/$name" "$ROOT/build/compare_x5/features/rviz/$name" \
    "$ROOT/build/native_tools/features/rviz/$name"; do
    if [[ -x $candidate ]]; then printf '%s\n' "$candidate"; return; fi
  done
  echo "Missing native executable $name. Build/install v8, or set INSTALL_ROOT (PERCEPTION_CONFIG may override the launcher)." >&2
  return 2
}
