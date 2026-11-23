#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source=${1:?Usage: run_matrix.sh offline|camera|ros NEW_OUTPUT [CONFIG] [ROS_PUBLISH_SOURCE=offline|camera|external]}
out=${2:?New output directory required}
config=${3:-$root/config/$source.json}
publish_source=${4:-offline}
[[ $source == offline || $source == camera || $source == ros ]] || exit 2
[[ ! -e $out ]] || { echo 'Output already exists' >&2; exit 2; }
mkdir -p -- "$out"
bin=${AB_BIN:-$root/bin/ab_benchmark}
[[ -x $bin ]] || bin="$root/install/x5/bin/ab_benchmark"
report=$(dirname -- "$bin")/ab_report
export LD_LIBRARY_PATH="$root/lib:$root/install/x5/lib:/app/data/metoak/lib:/usr/hobot/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
publisher=""
cleanup(){ if [[ -n $publisher ]]; then kill -TERM "$publisher" 2>/dev/null || true; wait "$publisher" 2>/dev/null || true; publisher=""; fi; }
trap cleanup EXIT INT TERM
"$(dirname -- "$bin")/ab_config_check" "$config"
if [[ $source == ros && $publish_source != external ]]; then
  transport=$("$(dirname -- "$bin")/ab_config_check" "$config" --transport)
  [[ $transport == dds ]] || { echo 'TCP ROS uses Ubuntu tools/run_ros_adb.sh, or external forwarding. Native same-machine publishing uses config/ros_dds.json.' >&2; exit 2; }
fi
uname -a > "$out/system.txt"
mapfile -t models < <("$(dirname -- "$bin")/ab_config_check" "$config" --models)
sha256sum "${models[@]}" > "$out/model_sha256.txt"
while read -r suite method; do
    args=(--config "$config" --source "$source" --suite "$suite" --method "$method" --output "$out/$suite/$method")
    mkdir -p -- "$out/$suite"
    if [[ $source == ros && $publish_source != external ]]; then
      "$bin" --config "$config" --method "$method" --publish --publish-source "$publish_source" > "$out/$suite/$method.publisher.log" 2>&1 &
      publisher=$!
      args+=(--publisher-pid "$publisher")
    fi
    "$bin" "${args[@]}" > "$out/$suite/$method.log" 2>&1
    cleanup
done < <("$(dirname -- "$bin")/ab_config_check" "$config" --matrix)
"$report" "$out"
printf 'Report: %s/index.html\n' "$out"
