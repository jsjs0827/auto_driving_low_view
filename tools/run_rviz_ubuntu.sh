#!/usr/bin/env bash
set -euo pipefail
rviz_project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
rviz_launcher=${PERCEPTION_RVIZ_LAUNCHER:-}
rviz_prefix=${RVIZ_INSTALL_ROOT:-$rviz_project_root/install/rviz_ubuntu}
if [[ -z $rviz_launcher ]]; then
  for rviz_candidate in \
    "$rviz_prefix/bin/r36_rviz_launcher" \
    "$rviz_prefix/lib/perception_ros_so_demo/r36_rviz_launcher" \
    "$rviz_project_root/build/rviz_ubuntu/r36_rviz_launcher" \
    "$rviz_project_root/build/rviz_ubuntu/features/rviz/r36_rviz_launcher" \
    "$rviz_project_root/build/native_tools/features/rviz/r36_rviz_launcher" \
    "$rviz_project_root/build/rviz/features/rviz/r36_rviz_launcher"; do
    if [[ -x $rviz_candidate ]]; then rviz_launcher=$rviz_candidate; break; fi
  done
fi
if [[ -z $rviz_launcher || ! -x $rviz_launcher ]]; then
  echo "Missing r36_rviz_launcher. Build/install the Ubuntu RViz targets, or set PERCEPTION_RVIZ_LAUNCHER." >&2
  exit 2
fi
# Preparation renders JSON/YAML without ADB, ROS, a camera, or a display server.
rviz_prepare_only=false
for rviz_argument in "$@"; do
  if [[ $rviz_argument == --prepare-only || $rviz_argument == --help || $rviz_argument == -h ]]; then rviz_prepare_only=true; fi
done
if [[ $rviz_prepare_only == false ]]; then
  rviz_setup=${ROS_SETUP:-/opt/ros/humble/setup.bash}
  if [[ ! -f $rviz_setup ]]; then echo "Missing ROS setup: $rviz_setup. Set ROS_SETUP to your Ubuntu ROS 2 installation." >&2; exit 2; fi
  set +u
  source "$rviz_setup"
  set -u
fi
exec "$rviz_launcher" "$@"
