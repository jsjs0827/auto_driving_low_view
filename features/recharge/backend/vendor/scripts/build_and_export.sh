#!/usr/bin/env bash
# 一键：提交源码说明 → 同步 ai_ws → Docker 交叉编译 → 导出 SDK → 推 main。
# 在宿主机执行（需要 docker）。单独交叉编译可自行 cp 后 bash ../build_recharge_mark.sh。
#
# 用法:
#   bash scripts/build_and_export.sh "子仓库代码管理整合"
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
AI_ROOT="${AI_ROOT:-/home/st/code/ai_root}"
AI_WS="${AI_WS:-${AI_ROOT}/ai_ws}"
IMAGE="${DOCKER_IMAGE:-pc_tros_ubuntu22.04:v1.0.0}"
CODE_ROOT="${CODE_ROOT:-/home/st/code}"
SO="${SO:-${AI_WS}/install/lib/librecharge_mark.so}"
MSG="${1:-}"

if [[ -z "${MSG}" ]]; then
  echo "[build_and_export] 需要提交说明" >&2
  echo "  用法: bash scripts/build_and_export.sh \"本次改动说明\"" >&2
  exit 1
fi

git -C "${ROOT}" add -u
git -C "${ROOT}" add \
  include src scripts config CMakeLists.txt package.xml VERSION VERSION.md \
  2>/dev/null || true
if git -C "${ROOT}" diff --cached --quiet; then
  echo "[build_and_export] 源码无改动，沿用 HEAD $(git -C "${ROOT}" rev-parse --short HEAD)"
else
  git -C "${ROOT}" commit -m "$(cat <<EOF
${MSG}

EOF
)"
  echo "[build_and_export] 源码已提交 $(git -C "${ROOT}" rev-parse --abbrev-ref HEAD) $(git -C "${ROOT}" rev-parse --short HEAD)"
fi

bash "${ROOT}/scripts/sync_to_ai_ws.sh"

echo "[build_and_export] docker build recharge_mark ..."
docker run --rm \
  -e EXPORT_SDK=0 \
  -e SRC_REPO="${ROOT}" \
  -v "${CODE_ROOT}:${CODE_ROOT}" \
  -w "${AI_ROOT}" \
  "${IMAGE}" \
  bash /home/st/code/ai_root/build_recharge_mark.sh

if [[ ! -f "${SO}" ]]; then
  echo "[build_and_export] missing ${SO}, fallback ai_node copy" >&2
  SO="${AI_WS}/install/lib/ai_node/librecharge_mark.so"
fi
export SO

bash "${ROOT}/scripts/export_sdk.sh"
PUBLISH_MSG="${MSG}" bash "${ROOT}/scripts/publish_main.sh"
echo "[build_and_export] done"
