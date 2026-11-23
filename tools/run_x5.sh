#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
action=${1:---help}
case "$action" in --help|-h) echo 'Usage on X5: run_x5.sh check | original [NEW_RUN_DIR] | stereo [NEW_RUN_DIR] | prepare-original [NEW_RUN_DIR] | prepare-stereo [NEW_RUN_DIR]'; exit 0;; esac
[[ $(uname -m) == aarch64 ]] || { echo 'Run this entry on the AArch64 X5, not the x86_64 Ubuntu host.' >&2; exit 2; }
source "$root/tools/x5_runtime_env.sh"
case "$action" in
  check)
    getconf GNU_LIBC_VERSION
    for path in /app/data/metoak/lib/libMoGeneralSDK.so /app/data/metoak/demo/config/x5_camera.cfg; do
      [[ -r $path ]] || { echo "Missing board camera file: $path" >&2; exit 2; }
    done
    log=$(ldd "$root/features/benchmark6/runtime/install/x5/bin/ab_benchmark" 2>&1)
    printf '%s\n' "$log"
    if [[ $log == *'not found'* ]]; then echo 'Resolve missing native/packaged libraries before running.' >&2; exit 2; fi
    ldd /app/data/metoak/lib/libMoGeneralSDK.so
    ;;
  original|stereo|prepare-original|prepare-stereo)
    [[ $# -le 2 ]] || { echo 'Unexpected arguments' >&2; exit 2; }
    method=${action#prepare-}
    config=camera_original_q8.json
    [[ $method != stereo ]] || config=camera_stereo.json
    run_dir=${2:-/userdata/perception_dag_v4/${action}_$(date +%Y%m%d_%H%M%S)_$$}
    args=(run --config "$root/$config" --run-dir "$run_dir")
    [[ $action != prepare-* ]] || args+=(--prepare-only)
    exec "$INSTALL_ROOT/bin/perception_config" "${args[@]}"
    ;;
  *) echo "Unknown action: $action" >&2; exit 2;;
esac
