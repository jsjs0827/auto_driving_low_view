#!/usr/bin/env bash
# v3 compatibility name; input and graph are selected by pipeline.json.
set -euo pipefail
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/run_pipeline.sh" "$@"
