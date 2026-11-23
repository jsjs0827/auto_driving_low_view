#!/usr/bin/env bash
# Optional in-tree camera publisher for the ROS-source cases only.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
method=${1:---help}
case "$method" in
  --help|-h) echo 'Usage: bash tools/run_inference_matrix_publisher.sh original|crop [CONFIG_DIR]'; exit 0;;
  original) mode=map;; crop) mode=infer;;
  *) echo 'Expected original or crop' >&2; exit 2;;
esac
[[ $# -le 2 ]] || { echo 'Unexpected arguments' >&2; exit 2; }
configs=${2:-$root/config/inference_matrix}
source "$root/tools/x5_runtime_env.sh"
ros_setup=${ROS_SETUP:-/opt/ros/humble/setup.bash}
if [[ -r $ros_setup ]]; then set +u; source "$ros_setup"; set -u; fi
[[ -x $INSTALL_ROOT/lib/perception_ros_so_demo/raw_camera ]] || {
  echo 'Build the current ARM ROS raw_camera executable first.' >&2; exit 2;
}
exec "$INSTALL_ROOT/bin/perception_config" camera --config "$configs/ros_${method}.json" --mode "$mode"
