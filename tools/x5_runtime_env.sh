#!/usr/bin/env bash
# Source on X5; the current board owns camera and kernel-facing runtime libraries.
x5_env_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export SDK_ROOT=${SDK_ROOT:-$x5_env_root/install/backend_sdk_x5}
export INSTALL_ROOT=${INSTALL_ROOT:-$x5_env_root/install/perception_ros_so_demo_x5}
x5_vendor_libs=${X5_VENDOR_LIBS:-/app/data/metoak/lib:/app/data/metoak/3rdparty_libs/opencv_libs:/app/data/metoak/3rdparty_libs/metoak_libs}
x5_platform_libs=${X5_PLATFORM_LIBS:-/usr/hobot/lib:/usr/lib/hbbpu}
export LD_LIBRARY_PATH="$x5_vendor_libs:$x5_platform_libs:$SDK_ROOT/lib:$INSTALL_ROOT/lib:$x5_env_root/features/benchmark6/runtime/install/x5/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
