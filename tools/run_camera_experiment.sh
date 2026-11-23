#!/usr/bin/env bash
set -euo pipefail
args=("$@")
method=
for ((i=0;i<${#args[@]};++i)); do
  if [[ ${args[i]} == --method ]]; then
    [[ -z $method && $((i+1)) -lt ${#args[@]} ]] || { echo 'Use exactly one --method detseg_only|original|stereo' >&2; exit 2; }
    method=${args[i+1]}
  fi
done
case "$method" in
  detseg_only|original|stereo) ;;
  *) echo 'Usage: bash tools/run_camera_experiment.sh --method detseg_only|original|stereo --output NEW_DIRECTORY' >&2; exit 2;;
esac
source "$(dirname -- "${BASH_SOURCE[0]}")/runtime_env.sh"
load_native_runtime_environment
launcher=$(native_executable r36_camera_compare)
exec "$launcher" --config "$ROOT/camera_benchmark.json" "${args[@]}"
