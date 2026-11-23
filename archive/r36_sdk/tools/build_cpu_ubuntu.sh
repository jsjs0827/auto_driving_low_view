#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ $# -ne 1 ]]; then
  echo "Usage: bash tools/build_cpu_ubuntu.sh /absolute/path/onnxruntime-linux-x64-<version>" >&2
  exit 1
fi
if [[ $(uname -m) != x86_64 ]]; then
  echo "This entry point builds a native x86_64 CPU runtime." >&2
  exit 1
fi
ORT_ROOT=$(cd -- "$1" && pwd)
if [[ ! -f "$ORT_ROOT/include/onnxruntime_cxx_api.h" || ! -f "$ORT_ROOT/lib/libonnxruntime.so" ]]; then
  echo "ORT_ROOT must contain include/onnxruntime_cxx_api.h and lib/libonnxruntime.so." >&2
  exit 1
fi
BUILD_DIR=${CPU_BUILD_DIR:-"$ROOT/build/cpu-ubuntu"}
INSTALL_DIR=${CPU_INSTALL_DIR:-"$ROOT/cpu_runtime"}
cmake -S "$ROOT" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release \
  -DR20_WITH_BPU=OFF -DR29_WITH_ORT=ON -DORT_ROOT="$ORT_ROOT" \
  -DPERCEPTION_BUILD_PRODUCT=OFF -DBUILD_TESTING=OFF
cmake --build "$BUILD_DIR" --target dag_flow_worker visualize_saved_frame perception_sdk perception_sdk_demo --parallel "${CPU_BUILD_JOBS:-4}"
cmake --install "$BUILD_DIR" --prefix "$INSTALL_DIR"
printf 'CPU runtime ready: %s\n' "$INSTALL_DIR"
printf '%s\n' 'Configure explicit ONNX models and preprocessing in a copy of profiles/cpu_exact.json.'
