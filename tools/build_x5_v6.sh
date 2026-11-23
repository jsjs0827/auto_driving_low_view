#!/usr/bin/env bash
# Compatibility name; the v7 entrypoint builds native camera experiments too.
set -euo pipefail
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/build_x5_v7.sh" "$@"
