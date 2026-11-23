#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
kind=${1:---contract}
if [[ $kind == --contract ]]; then
  build="$root/build/host_contract"
  cmake -S "$root" -B "$build" -G Ninja -DAB_CONTRACT_ONLY=ON -DCMAKE_BUILD_TYPE=Release
  cmake --build "$build" --parallel 2
  ctest --test-dir "$build" --output-on-failure
  "$build/ab_config_check" "$root/config/offline.json"
elif [[ $kind == --ros ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  build="$root/build/host_ros"
  cmake -S "$root" -B "$build" -G Ninja -DCMAKE_BUILD_TYPE=Release -DAB_WITH_ROS=ON -DAB_WITH_X5=OFF -DBUILD_TESTING=ON "-DCMAKE_INSTALL_PREFIX=$root/install/ubuntu_ros"
  cmake --build "$build" --target ab_benchmark ab_report ab_input_tests ab_contract_tests ab_pairing_tests ab_camera_tests ab_config_check --parallel 2
  ctest --test-dir "$build" --output-on-failure
  cmake --install "$build" --component ABRuntime
else echo 'build_host.sh --contract|--ros' >&2; exit 2; fi
