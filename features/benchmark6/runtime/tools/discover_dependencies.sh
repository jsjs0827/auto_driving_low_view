#!/usr/bin/env bash
set -euo pipefail
printf 'Existing ARM ROS CMake metadata under /var/tmp and /opt:\n'
find /var/tmp /opt -maxdepth 7 \( -name rclcppConfig.cmake -o -name PCLConfig.cmake \) -print 2>/dev/null | head -30
printf 'Docker availability:\n'
if command -v docker >/dev/null; then docker ps --format '{{.Names}} {{.Image}}' || true; fi
