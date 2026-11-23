#!/usr/bin/env bash
# Ubuntu HOST entry point. Invoke with bash; do not source into your terminal.
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  printf '%s\n' 'Run: bash tools/build_original_ros_in_container.sh [options]. Do not source this script.' >&2
  return 2
fi
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
printf '%s\n' \
  'Original ROS product plugin build: supply matching readable dependency headers and AArch64 ROS packages; a prior original-project build is not required.' \
  'Protected/binary headers in the Windows snapshot cannot compile; this command does not decode or replace their ABI.' \
  'Pass container paths with --deps-root, --ros-setup, --workspace-setup and --cross-prefix as needed. Use --without-recharge to omit the recharge SDK.' >&2
exec bash "$ROOT/tools/build_in_container.sh" --product "$@"
