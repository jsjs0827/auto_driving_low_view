#!/usr/bin/env bash
# Historical command name; v4 builds business programs AND benchmark6.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec bash "$ROOT/tools/build_docker_x5.sh" "$@"
