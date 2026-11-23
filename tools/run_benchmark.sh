#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/runtime_env.sh"
load_runtime_environment
launcher=$(native_executable perception_config)
# benchmark.json selects three_way; passing --both is unnecessary and rejected.
exec "$launcher" run --config "$ROOT/benchmark.json" "$@"
