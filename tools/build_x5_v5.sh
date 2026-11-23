#!/usr/bin/env bash
# Compatibility entrypoint. v6 implementation lives in build_x5_v6.sh.
set -euo pipefail
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/build_x5_v6.sh" "$@"
