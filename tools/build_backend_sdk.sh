#!/usr/bin/env bash
# Build the real backend sources owned by the feature folders.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
backend=; ort_root=; horizon_root=; toolchain=
build_dir="$ROOT/build/backend"; install_dir="$ROOT/install/backend_sdk"
usage() {
  echo 'Usage: bash tools/build_backend_sdk.sh --backend cpu|bpu [--ort-root PATH] [--horizon-root PATH] [--toolchain FILE] [--build-dir DIR] [--install-dir DIR] [-- additional CMake dependency flags]'
}
while (($#)); do
  case "$1" in
    -h|--help) usage; exit 0;;
    --) shift; break;;
    --backend|--ort-root|--horizon-root|--toolchain|--build-dir|--install-dir)
      (($# >= 2)) || { usage >&2; exit 2; }
      case "$1" in
        --backend) backend=$2;; --ort-root) ort_root=$2;; --horizon-root) horizon_root=$2;;
        --toolchain) toolchain=$2;; --build-dir) build_dir=$2;; --install-dir) install_dir=$2;;
      esac
      shift 2;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2;;
  esac
done
cmake_args=(-DCMAKE_BUILD_TYPE=Release -DR33_WITH_MAPPING=ON -DR33_WITH_RECHARGE=OFF -DBUILD_TESTING=OFF)
case "$backend" in
  cpu)
    [[ -d $ort_root ]] || { echo 'CPU backend requires an actual --ort-root C++ SDK.' >&2; exit 2; }
    cmake_args+=(-DR20_WITH_BPU=OFF -DR29_WITH_ORT=ON "-DORT_ROOT=$ort_root");;
  bpu)
    [[ -d $horizon_root ]] || { echo 'BPU backend requires actual --horizon-root headers/libraries.' >&2; exit 2; }
    if [[ $(uname -m) != aarch64 && -z $toolchain ]]; then
      echo 'Cross-building BPU on x86 requires --toolchain for actual AArch64 dependencies.' >&2; exit 2
    fi
    cmake_args+=(-DR20_WITH_BPU=ON -DR29_WITH_ORT=OFF "-DHORIZON_ROOT=$horizon_root");;
  *) usage >&2; exit 2;;
esac
if [[ -n $toolchain ]]; then
  [[ -f $toolchain ]] || { echo "Missing toolchain: $toolchain" >&2; exit 2; }
  cmake_args+=("-DCMAKE_TOOLCHAIN_FILE=$toolchain")
fi
cmake -S "$ROOT/sdk" -B "$build_dir" "${cmake_args[@]}" "-DCMAKE_INSTALL_PREFIX=$install_dir" "$@"
cmake --build "$build_dir" --target perception_sdk perception_sdk_demo perception_full_demo perception_temporal_demo r36_temporal_unit r36_temporal_contract --parallel "${BUILD_JOBS:-2}"
cmake --install "$build_dir" --component SharedSDK
printf 'Backend installed: %s\nSet sdk_root to this directory to run these rebuilt feature implementations.\n' "$install_dir"
