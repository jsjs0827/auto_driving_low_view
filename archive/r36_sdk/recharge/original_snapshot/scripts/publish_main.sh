#!/usr/bin/env bash
# 把交叉编译产物推到 charging.git 的 main（只含头 + so，不含 cpp）。
# master 保持源码；main 给感知仓 submodule。
#
# 用法（宿主机，so 已编好）:
#   bash scripts/publish_main.sh
#   PUBLISH_MSG="子仓库代码管理整合" bash scripts/publish_main.sh
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
AI_WS="${AI_WS:-/home/st/code/ai_root/ai_ws}"
SO="${SO:-${AI_WS}/install/lib/librecharge_mark.so}"
if [[ ! -f "${SO}" ]]; then
  SO="${AI_WS}/install/lib/ai_node/librecharge_mark.so"
fi
HDR="${ROOT}/include/percep_demo/api_recharge_mark.h"
WT="${SDK_MAIN_WT:-/home/st/code/recharge_mark_sdk_main}"
PUSH="${PUSH:-1}"
REMOTE="${REMOTE:-origin}"
BRANCH_MAIN="${BRANCH_MAIN:-main}"
PUBLISH_MSG="${PUBLISH_MSG:-}"

if [[ ! -f "${HDR}" ]]; then
  echo "[publish_main] missing header: ${HDR}" >&2
  exit 1
fi
if [[ ! -f "${SO}" ]]; then
  echo "[publish_main] missing so: ${SO}" >&2
  echo "  先: bash ${ROOT}/scripts/build_and_export.sh" >&2
  exit 1
fi

SRC_COMMIT="$(git -C "${ROOT}" rev-parse HEAD)"
SRC_BRANCH="$(git -C "${ROOT}" rev-parse --abbrev-ref HEAD)"
DIRTY=0
if ! git -C "${ROOT}" diff --quiet HEAD --; then
  DIRTY=1
fi
SHORT="${SRC_COMMIT:0:12}"

git -C "${ROOT}" fetch "${REMOTE}" "${BRANCH_MAIN}"

if git -C "${ROOT}" worktree list | grep -q " ${WT} "; then
  git -C "${WT}" checkout "${BRANCH_MAIN}"
  git -C "${WT}" reset --hard "${REMOTE}/${BRANCH_MAIN}"
else
  mkdir -p "$(dirname "${WT}")"
  if [[ -e "${WT}" && ! -d "${WT}/.git" && ! -f "${WT}/.git" ]]; then
    echo "[publish_main] ${WT} exists but is not a git worktree" >&2
    exit 1
  fi
  if [[ ! -e "${WT}" ]]; then
    git -C "${ROOT}" worktree add -B "${BRANCH_MAIN}" "${WT}" "${REMOTE}/${BRANCH_MAIN}"
  fi
fi

PREV_LOG=""
if [[ -f "${WT}/CHANGELOG.md" ]]; then
  PREV_LOG="$(cat "${WT}/CHANGELOG.md")"
fi

# 清空 main 工作区，只留下 SDK
git -C "${WT}" rm -rf --ignore-unmatch . >/dev/null 2>&1 || true
find "${WT}" -mindepth 1 -maxdepth 1 ! -name '.git' -exec rm -rf {} +

mkdir -p "${WT}/include/percep_demo" "${WT}/lib/aarch64" "${WT}/config"
cp -f "${HDR}" "${WT}/include/percep_demo/api_recharge_mark.h"
# 头文件已合并为单头（API + 宿主），旧拆分版 recharge_mark_host.h 不再发布
rm -f "${WT}/include/percep_demo/recharge_mark_host.h"
if [[ -f "${ROOT}/config/recharge_mark.json" ]]; then
  cp -f "${ROOT}/config/recharge_mark.json" "${WT}/config/recharge_mark.json"
fi
cp -f "${SO}" "${WT}/lib/aarch64/librecharge_mark.so"
chmod 644 "${WT}/lib/aarch64/librecharge_mark.so"
printf '\n' > "${WT}/COLCON_IGNORE"

ROOT="${ROOT}" bash "${ROOT}/scripts/write_sdk_version.sh" "${WT}" "${SO}" "${HDR}"
MD5="$(grep '^so_md5=' "${WT}/VERSION" | cut -d= -f2)"
DATE="$(grep '^published_at=' "${WT}/VERSION" | cut -d= -f2)"

{
  echo "# changelog"
  echo
  echo "## ${DATE}  ${SHORT}  md5=${MD5}"
  echo
  if [[ -n "${PUBLISH_MSG}" ]]; then
    echo "- ${PUBLISH_MSG}"
  fi
  echo "- source=${SRC_BRANCH} ${SRC_COMMIT} dirty=${DIRTY}"
  echo "- header=${HDR}"
  echo
  if [[ -n "${PREV_LOG}" ]]; then
    echo "${PREV_LOG}" | sed '1{/^# changelog$/d;}' | sed '/^$/d' | head -n 80
  fi
} > "${WT}/CHANGELOG.md"

cat > "${WT}/README.md" << EOF
# recharge_mark SDK (\`main\`)

本分支**只含**交叉编译产物，给感知仓 submodule 使用。

当前库版本见 **[VERSION.md](VERSION.md)**（so md5 + 编译时间）。机器可读副本：\`VERSION\`。

- \`include/percep_demo/api_recharge_mark.h\`（单头：API + 宿主）
- \`lib/aarch64/librecharge_mark.so\`
- \`config/recharge_mark.json\`（回充参数；板端装到 config/ai_node/）

源码在 \`master\`。不要把 cpp / CMakeLists / package.xml 合进本分支。

核对：

\`\`\`bash
md5sum lib/aarch64/librecharge_mark.so
grep '^so_md5=' VERSION
\`\`\`
EOF

cat > "${WT}/.gitignore" << 'EOF'
src/
CMakeLists.txt
package.xml
scripts/
*.cpp
*.o
EOF

git -C "${WT}" add -A
if git -C "${WT}" diff --cached --quiet; then
  echo "[publish_main] main 无变化 (md5=${MD5})"
else
  local_title="sdk: bump librecharge_mark.so from ${SRC_BRANCH} ${SHORT}"
  if [[ -n "${PUBLISH_MSG}" ]]; then
    local_title="sdk: ${PUBLISH_MSG}"
  fi
  git -C "${WT}" commit -m "$(cat <<EOF
${local_title}

source=${SRC_BRANCH} ${SRC_COMMIT} dirty=${DIRTY} so_md5=${MD5}
EOF
)"
fi

echo "[publish_main] ${WT} commit=$(git -C "${WT}" rev-parse --short HEAD) md5=${MD5}"

if [[ "${PUSH}" == "1" ]]; then
  git -C "${WT}" push -u "${REMOTE}" "${BRANCH_MAIN}"
  echo "[publish_main] pushed ${REMOTE}/${BRANCH_MAIN}"
else
  echo "[publish_main] SKIP push (PUSH=0)"
fi
