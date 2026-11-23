#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/runtime_env.sh"
launcher=$(native_executable perception_config)
# Ask the native parser; do not duplicate the JSON/deployment rules in shell.
deployment_args=()
arguments=("$@")
for ((index=0; index<${#arguments[@]}; ++index)); do
  case ${arguments[index]} in
    --config|--deployment)
      ((index+1<${#arguments[@]})) || { echo "Missing value for ${arguments[index]}" >&2; exit 2; }
      deployment_args+=("${arguments[index]}" "${arguments[index+1]}")
      ((index+=1))
      ;;
  esac
done
selected_deployment=$("$launcher" deployment "${deployment_args[@]}")
if [[ $selected_deployment == v0_host ]]; then
  load_native_runtime_environment
else
  load_runtime_environment
fi
exec "$launcher" run "$@"
