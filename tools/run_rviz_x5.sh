#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/runtime_env.sh"
load_native_runtime_environment
export LD_LIBRARY_PATH="${CAMERA_SDK_ROOT:-/app/data/metoak/lib}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
launcher=$(native_executable r36_rviz_publisher)
exec "$launcher" "$@"
