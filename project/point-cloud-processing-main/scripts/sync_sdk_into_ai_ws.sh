#!/usr/bin/env bash
# 把 submodule（charging.git main：头 + so）同步到 ai_ws 感知包目录。
# 不会覆盖 ai_ws/src/recharge_mark（那是 master 源码编译树）。
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
AI_WS="${AI_WS:-/home/st/code/ai_root/ai_ws}"
SRC="${ROOT}/recharge_mark"
DST="${AI_WS}/src/ai_node/point-cloud-processing/recharge_mark"
HDR_DST="${AI_WS}/src/ai_node/point-cloud-processing/include/percep_main"

if [[ ! -f "${SRC}/lib/aarch64/librecharge_mark.so" ]]; then
  echo "[sync_sdk] missing ${SRC}/lib/aarch64/librecharge_mark.so" >&2
  echo "  先: git submodule update --init --recursive" >&2
  exit 1
fi

mkdir -p "${DST}/include/percep_main" "${DST}/lib/aarch64" "${HDR_DST}"
# 编译树里若残留过源码，清掉，避免 colcon 误编
rm -rf "${DST}/src" "${DST}/scripts"
rm -f "${DST}/CMakeLists.txt" "${DST}/package.xml"
cp -a "${SRC}/include/percep_main/api_recharge_mark.h" "${DST}/include/percep_main/"
cp -a "${SRC}/lib/aarch64/librecharge_mark.so" "${DST}/lib/aarch64/"
cp -a "${SRC}/VERSION" "${DST}/VERSION" 2>/dev/null || true
cp -a "${SRC}/COLCON_IGNORE" "${DST}/COLCON_IGNORE" 2>/dev/null || true
cp -a "${SRC}/include/percep_main/api_recharge_mark.h" "${HDR_DST}/"
cp -a "${ROOT}/CMakeLists.txt" "${AI_WS}/src/ai_node/point-cloud-processing/CMakeLists.txt"
cp -a "${ROOT}/package.xml" "${AI_WS}/src/ai_node/point-cloud-processing/package.xml"

echo "[sync_sdk] ${SRC} -> ${DST}"
echo "[next] 容器内只编 percep_main: bash /home/st/code/ai_root/build_percep_demo.sh"
