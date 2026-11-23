#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 1 ]]; then
    echo "usage: $0 OUTPUT_EXECUTABLE" >&2
    exit 2
fi
project=$(cd "$(dirname "$0")/.." && pwd)
sdk="$project/perception_v2_sdk_x5"
common="${COMMON_INTERFACE_ROOT:-$project/../../dependencies/common_interface-main}"
compiler="${X5_CXX:-aarch64-linux-gnu-g++}"
"$compiler" -std=c++17 -O2 \
    -I"$project/include" -I"$sdk/include" -I"$common/include" -I"$project/recharge_mark/include" \
    "$project/tests/v2_q8_recharge_replay.cpp" "$project/src/v2_sdk_runtime.cpp" \
    -L"$sdk/lib" -Wl,--allow-shlib-undefined -l:libperception_sdk.so.1 -ldl \
    -o "$1"
