#!/usr/bin/env bash
# Postprocess saved quality results; never run concurrently with performance passes.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
output=${1:?Usage: save_benchmark6_visuals.sh BENCHMARK_OUTPUT}
output=$(cd -- "$output" && pwd)
ctl=${B6_CONTROL_BIN:-$root/build/benchmark6/r36_benchmark6}
replay=${B6_REPLAY_BIN:-$root/features/benchmark6/runtime/install/ubuntu_ros/bin/ab_result_replay}
[[ -x $replay && -x $ctl ]] || { echo 'Build the Ubuntu tools first' >&2; exit 2; }
set +u
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-67}
value(){ "$ctl" --value "$output/plan.json" "visualization.$1"; }
hz=$(value replay_hz);stride=$(value cloud_stride);maximum=$(value disparity_max_px)
save_bag=$(value save_rosbag)
mkdir -p "$output/visualization"
bagpid=''
finish_bag(){ if [[ -n $bagpid ]]; then kill -INT "$bagpid" 2>/dev/null || true; wait "$bagpid" 2>/dev/null || true;bagpid='';fi; }
trap finish_bag EXIT
trap 'exit 130' INT TERM
topics=()
for source in camera offline ros; do for method in original_q8 stereo_crop; do
  for stream in left detection segmentation segmentation_overlay disparity_color disparity_q8 cloud result; do topics+=("/benchmark6/$source/$method/$stream");done
done;done
if [[ $save_bag == true ]]; then
  [[ ! -e $output/visualization/bag ]] || { echo 'Bag already exists; choose a new output or move the existing bag before rerunning' >&2; exit 2; }
  ros2 bag record --storage sqlite3 --output "$output/visualization/bag" "${topics[@]}" >"$output/visualization/rosbag-record.log" 2>&1 & bagpid=$!
  sleep 2
  kill -0 "$bagpid" 2>/dev/null || { echo 'rosbag recorder failed; see visualization/rosbag-record.log' >&2; exit 1; }
fi
status=0
for source in camera offline ros; do for method in original_q8 stereo_crop; do
  dir="$output/runs/$source/$method/quality"
  [[ -f $dir/predictions.jsonl ]] || { printf 'Missing quality predictions: %s\n' "$dir" >&2;status=1;continue; }
  prefix="/benchmark6/$source/$method"
  {
    cat <<'YAML'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Global Options:
    Fixed Frame: camera_optical_frame
    Background Color: 25; 29; 36
    Frame Rate: 15
  Displays:
YAML
    for stream in detection segmentation_overlay disparity_color; do
      cat <<YAML
    - Class: rviz_default_plugins/Image
      Name: $source $method $stream
      Enabled: true
      Topic:
        Value: $prefix/$stream
        Reliability Policy: Reliable
        Durability Policy: Volatile
        History Policy: Keep Last
        Depth: 4
      Transport Hint: raw
YAML
    done
    cat <<YAML
    - Class: rviz_default_plugins/PointCloud2
      Name: Q8 projected cloud in meters
      Enabled: true
      Topic:
        Value: $prefix/cloud
        Reliability Policy: Reliable
        Durability Policy: Volatile
        History Policy: Keep Last
        Depth: 4
      Style: Points
      Size (Pixels): 2
      Color Transformer: RGB8
      Position Transformer: XYZ
      Decay Time: 0
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 5
      Focal Point: {X: 0, Y: 0, Z: 2}
Window Geometry:
  Width: 1280
  Height: 850
YAML
  } >"$dir/view.rviz"
  args=("$dir" "$prefix" "$hz" "$stride" "$maximum")
  if [[ $save_bag == true ]]; then args+=(--record);else args+=(--derive-only);fi
  "$replay" "${args[@]}" >"$output/visualization/${source}_${method}.log" 2>&1 || status=1
done;done
finish_bag
if [[ $save_bag == true && ! -f $output/visualization/bag/metadata.yaml ]]; then status=1;echo 'Bag did not finalize: check recorder log' >&2;fi
"$ctl" --report "$output" >"$output/visualization/report.log"
printf 'Saved PNG + Q8 + PLY + RViz configurations: %s/runs\n' "$output"
[[ $save_bag != true ]] || printf 'Replay bag: %s/visualization/bag\n' "$output"
exit "$status"
