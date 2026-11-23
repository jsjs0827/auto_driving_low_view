#!/usr/bin/env bash
# r33 keeps the familiar build entry point. Run with bash, never source.
set -euo pipefail
TASK_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec bash "$TASK_ROOT/tools/build_shared_x5.sh" "$@"
