#!/usr/bin/env bash
# 把本仓库同步到 ai_ws，供容器内 ./robot_dev_config/build.sh 编译。
# Docker 默认只挂载 ai_root，因此宿主机上的本仓需 rsync 进 ai_ws。
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
AI_WS="${AI_WS:-/home/st/code/ai_root/ai_ws}"
DST="${AI_WS}/src/recharge_mark"

mkdir -p "${AI_WS}/src"
rsync -a --delete \
  --exclude '.git/' \
  --exclude 'build/' \
  --exclude 'install/' \
  --exclude 'log/' \
  "${ROOT}/" "${DST}/"

echo "[sync] ${ROOT} -> ${DST}"
echo "[next] 容器内："
echo "  cd ${AI_WS}"
echo "  ./robot_dev_config/build.sh -p X5 -s recharge_mark"
echo "  # 或整仓：./robot_dev_config/build.sh X5"
