#!/usr/bin/env bash
# 把交叉编译产物（对外头文件 + librecharge_mark.so）导入感知仓 SDK 目录。
# 感知仓不再包含 cpp，只链接本脚本写入的头和库。
#
# 用法（宿主机，编完 so 之后）:
#   bash scripts/export_sdk.sh
#   PERCEP_ROOT=/path/to/point-cloud-processing-0.0.4recharge bash scripts/export_sdk.sh
#
# 环境变量:
#   PERCEP_ROOT  感知仓根目录（默认 0.0.4recharge）
#   AI_WS        交叉工作空间（默认 ai_root/ai_ws）
#   SO           指定 so 路径（默认 AI_WS/install/lib/ai_node/librecharge_mark.so）
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PERCEP_ROOT="${PERCEP_ROOT:-/home/st/code/point-cloud-processing-v005}"
AI_WS="${AI_WS:-/home/st/code/ai_root/ai_ws}"
SO="${SO:-${AI_WS}/install/lib/librecharge_mark.so}"
if [[ ! -f "${SO}" ]]; then
  SO="${AI_WS}/install/lib/ai_node/librecharge_mark.so"
fi
HDR="${ROOT}/include/percep_demo/api_recharge_mark.h"
SDK="${PERCEP_ROOT}/recharge_mark"
AI_NESTED="${AI_WS}/src/ai_node/point-cloud-processing/recharge_mark"

if [[ ! -f "${HDR}" ]]; then
  echo "[export_sdk] missing header: ${HDR}" >&2
  exit 1
fi
if [[ ! -f "${SO}" ]]; then
  echo "[export_sdk] missing so: ${SO}" >&2
  echo "  先交叉编译: bash ${ROOT}/scripts/build_and_export.sh" >&2
  exit 1
fi
if [[ ! -d "${PERCEP_ROOT}" ]]; then
  echo "[export_sdk] PERCEP_ROOT not found: ${PERCEP_ROOT}" >&2
  exit 1
fi

install_sdk_tree() {
  local dest="$1"
  mkdir -p "${dest}/include/percep_demo" "${dest}/lib/aarch64" "${dest}/config"
  cp -f "${HDR}" "${dest}/include/percep_demo/api_recharge_mark.h"
  # 头文件已合并为单头（API + 宿主），旧拆分版 recharge_mark_host.h 不再发布
  rm -f "${dest}/include/percep_demo/recharge_mark_host.h"
  if [[ -f "${ROOT}/config/recharge_mark.json" ]]; then
    cp -f "${ROOT}/config/recharge_mark.json" "${dest}/config/recharge_mark.json"
  fi
  cp -f "${SO}" "${dest}/lib/aarch64/librecharge_mark.so"
  chmod 644 "${dest}/lib/aarch64/librecharge_mark.so" || true
  # 防止 colcon 把 SDK 目录当成源码包再编一遍
  if [[ -e "${dest}/COLCON_IGNORE" && ! -w "${dest}/COLCON_IGNORE" ]]; then
    chmod u+w "${dest}/COLCON_IGNORE" 2>/dev/null || true
  fi
  printf '\n' > "${dest}/COLCON_IGNORE"
}

COMMIT="$(git -C "${ROOT}" rev-parse HEAD 2>/dev/null || echo unknown)"
BRANCH="$(git -C "${ROOT}" rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
DIRTY=0
if git -C "${ROOT}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  if ! git -C "${ROOT}" diff --quiet HEAD -- 2>/dev/null; then
    DIRTY=1
  fi
fi
MD5="$(md5sum "${SO}" | awk '{print $1}')"
DATE="$(date -Iseconds)"

write_version() {
  local dest="$1"
  ROOT="${ROOT}" bash "${ROOT}/scripts/write_sdk_version.sh" "${dest}" "${SO}" "${HDR}"
}

echo "[export_sdk] so=${SO}"
echo "[export_sdk] commit=${COMMIT} dirty=${DIRTY}"
install_sdk_tree "${SDK}"
write_version "${SDK}"

# 头文件只随 recharge_mark submodule 提供（感知仓 CMake 经 IMPORTED 目标
# INTERFACE_INCLUDE_DIRECTORIES 指向 SDK/include），不再拷贝本地副本。
# 配置 json 感知仓仍需安装到 config/ai_node，继续拷贝。
mkdir -p "${PERCEP_ROOT}/config"
if [[ -f "${ROOT}/config/recharge_mark.json" ]]; then
  cp -f "${ROOT}/config/recharge_mark.json" "${PERCEP_ROOT}/config/recharge_mark.json"
fi

# 编译树：ai_ws 内 percep 包同目录也要有 SDK，否则 colcon 链不到
if [[ -d "${AI_WS}/src/ai_node/point-cloud-processing" ]]; then
  install_sdk_tree "${AI_NESTED}"
  write_version "${AI_NESTED}"
  echo "[export_sdk] also -> ${AI_NESTED}"
fi

# so+头写入 nocam SDK；不覆盖感知仓自己的 recharge_mark.json（YOLO/DockPose 在 percep 侧）
if [[ -d /home/st/code/point-cloud-processing-nocam/recharge_mark ]]; then
  NOCAM=/home/st/code/point-cloud-processing-nocam
  mkdir -p "${NOCAM}/recharge_mark/include/percep_demo" \
           "${NOCAM}/recharge_mark/lib/aarch64"
  cp -f "${HDR}" "${NOCAM}/recharge_mark/include/percep_demo/api_recharge_mark.h"
  cp -f "${SO}" "${NOCAM}/recharge_mark/lib/aarch64/librecharge_mark.so"
  chmod 644 "${NOCAM}/recharge_mark/lib/aarch64/librecharge_mark.so" || true
  write_version "${NOCAM}/recharge_mark"
  echo "[export_sdk] also -> ${NOCAM}/recharge_mark (so+header, json untouched)"
fi

echo "[export_sdk] -> ${SDK}"
echo "    header  include/percep_demo/api_recharge_mark.h"
echo "    lib     lib/aarch64/librecharge_mark.so"
echo "    md5     ${MD5}"
echo "[next] 感知仓对照 VERSION 后更新 submodule 指针:"
echo "  md5sum ${SDK}/lib/aarch64/librecharge_mark.so"
echo "  grep '^so_md5=' ${SDK}/VERSION"
echo "  cd ${PERCEP_ROOT} && git add recharge_mark && git commit -m \"chore: bump recharge_mark \${MD5:0:12}\""
