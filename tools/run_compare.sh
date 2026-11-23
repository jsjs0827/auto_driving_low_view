#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/runtime_env.sh"
load_runtime_environment
launcher=$(native_executable perception_config)
exec "$launcher" run --both "$@"
