#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/runtime_env.sh"
load_native_runtime_environment
# Linux vendor dependencies must be visible to the ELF loader before startup.
if [[ -n ${CAMERA_SDK_ROOT:-} ]]; then
  export LD_LIBRARY_PATH="$CAMERA_SDK_ROOT${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
launcher=$(native_executable r36_slip_runner)
# Supply --config "$ROOT/slip_camera.json" or "$ROOT/slip_recording.json"
# and --output a NEW directory. No ROS setup is required.
exec "$launcher" "$@"
