#!/usr/bin/env bash
# Optional board-only environment helper; no configuration generation.
source "$(dirname -- "${BASH_SOURCE[0]}")/runtime_env.sh"
[[ $(uname -m) == aarch64 ]] || { echo 'This helper requires X5/AArch64.' >&2; return 2 2>/dev/null || exit 2; }
load_runtime_environment
