#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export SDK_ROOT=${SDK_ROOT:-$root/install/backend_sdk_x5}
export INSTALL_ROOT=${INSTALL_ROOT:-$root/install/perception_ros_so_demo_x5}
source "$root/tools/x5_runtime_env.sh"
exec "$INSTALL_ROOT/bin/perception_config" "$@"
