#!/usr/bin/env bash
# Ubuntu: actual ROS2 DDS -> C++ frame bridge -> ADB TCP -> X5 benchmark.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
board_root=${1:-/userdata/perception-ab-benchmark-v9}
board_output=${2:?Usage: run_ros_adb.sh BOARD_PROJECT NEW_BOARD_OUTPUT [HOST_ROS_CONFIG]}
config=${3:-$root/config/ros.json}
host=${AB_HOST_BIN:-$root/bin/ab_benchmark}
[[ -x $host ]] || host="$root/install/ubuntu_ros/bin/ab_benchmark"
[[ -x $host ]] || host="$root/build/host_ros/ab_benchmark"
checker=$(dirname -- "$host")/ab_config_check
export LD_LIBRARY_PATH="$root/install/ubuntu_ros/lib:$root/build/host_ros/canonical_sdk/shared${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
set +u
source /opt/ros/humble/setup.bash
set -u
[[ $board_root =~ ^/[a-zA-Z0-9_./-]+$ && $board_output =~ ^/[a-zA-Z0-9_./-]+$ ]] || { echo 'Board paths must be absolute and contain only letters, digits, _, ., /, -' >&2; exit 2; }
port=$("$checker" "$config" --bridge-port)
adb forward "tcp:$port" "tcp:$port"
adb shell "test ! -e $board_output && mkdir -p $board_output && uname -a > $board_output/system.txt && sha256sum $board_root/models/x5/*.bin > $board_output/model_sha256.txt"
publisher="";worker=""
cleanup(){ [[ -z $publisher ]] || { kill -TERM "$publisher" 2>/dev/null || true; wait "$publisher" 2>/dev/null || true; publisher=""; }; }
trap cleanup EXIT INT TERM
while read -r suite method; do
  adb shell "cd $board_root && bash tools/run_one.sh --config config/ros.json --source ros --method $method --suite $suite --output $board_output/$suite/$method" &
  worker=$!
  "$host" --config "$config" --method "$method" --publish --publish-source offline &
  publisher=$!
  "$host" --config "$config" --method "$method" --bridge-forward
  wait "$worker"
  cleanup
done < <("$checker" "$config" --matrix)
adb shell "cd $board_root && export LD_LIBRARY_PATH=$board_root/lib:$board_root/install/x5/lib && if [ -x bin/ab_report ]; then bin/ab_report $board_output; else install/x5/bin/ab_report $board_output; fi"
printf 'Board report: %s/index.html\n' "$board_output"
