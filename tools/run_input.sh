#!/usr/bin/env bash
# v3 accepts the root JSON, rather than an independently maintained ROS YAML.
set -euo pipefail
if [[ ${1:-} != --* && -n ${1:-} ]]; then
  config=$1; shift
  exec bash "$(dirname -- "${BASH_SOURCE[0]}")/run_pipeline.sh" --config "$config" "$@"
fi
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/run_pipeline.sh" "$@"
