#!/usr/bin/env bash
# Same-ROI disparity/PLY comparison. Pure performance uses run_benchmark.sh.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/runtime_env.sh"
load_runtime_environment
launcher=$(native_executable perception_config)
exec "$launcher" run --config "$ROOT/crop_compare.json" "$@"
