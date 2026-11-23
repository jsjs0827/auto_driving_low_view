#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/runtime_env.sh"
load_native_runtime_environment
launcher=$(native_executable r36_camera_compare)
# Separate live acquisitions: camera SDK -> SDK DAG, without a ROS transport node.
# Required: --output NEW_DIRECTORY. Optional: --method detseg_only|original|stereo.
exec "$launcher" --config "$ROOT/camera_benchmark.json" "$@"
