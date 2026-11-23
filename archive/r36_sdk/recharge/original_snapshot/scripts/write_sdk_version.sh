#!/usr/bin/env bash
# 写入 SDK 版本说明（VERSION + VERSION.md），供 publish_main / export_sdk 共用。
# 用法: write_sdk_version.sh DEST_DIR SO HDR
set -euo pipefail

DEST="${1:?dest dir}"
SO="${2:?so path}"
HDR="${3:?header path}"
ROOT="${ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

mkdir -p "${DEST}"
SO_MD5="$(md5sum "${SO}" | awk '{print $1}')"
HDR_MD5="$(md5sum "${HDR}" | awk '{print $1}')"
SO_SIZE="$(stat -c%s "${SO}")"
COMPILED_AT="$(date -d "@$(stat -c %Y "${SO}")" -Iseconds)"
PUBLISHED_AT="$(date -Iseconds)"
SRC_COMMIT="$(git -C "${ROOT}" rev-parse HEAD 2>/dev/null || echo unknown)"
SRC_BRANCH="$(git -C "${ROOT}" rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
SRC_SHORT="${SRC_COMMIT:0:12}"
DIRTY=0
if git -C "${ROOT}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  if ! git -C "${ROOT}" diff --quiet HEAD -- 2>/dev/null; then
    DIRTY=1
  fi
fi

cat > "${DEST}/VERSION" << EOF
so_md5=${SO_MD5}
so_size=${SO_SIZE}
compiled_at=${COMPILED_AT}
published_at=${PUBLISHED_AT}
source_commit=${SRC_COMMIT}
source_branch=${SRC_BRANCH}
source_dirty=${DIRTY}
header_md5=${HDR_MD5}
so_path=lib/aarch64/librecharge_mark.so
header=include/percep_demo/api_recharge_mark.h
EOF

cat > "${DEST}/VERSION.md" << EOF
# librecharge_mark.so 版本说明

升库是否成功：**对比本文件 / \`VERSION\` 里的 \`so_md5\` 与实际 so 的 md5**。一致即本版本；不一致说明 submodule 未更新，或 so 被换过但版本说明没跟上。

| 项 | 值 |
|---|---|
| so md5 | \`${SO_MD5}\` |
| so 大小 | ${SO_SIZE} 字节 |
| 编译时间（so mtime） | ${COMPILED_AT} |
| 写入 main 时间 | ${PUBLISHED_AT} |
| 源码分支 | \`${SRC_BRANCH}\` |
| 源码 commit | \`${SRC_COMMIT}\` |
| 源码工作区 dirty | ${DIRTY}（1=未提交改动也编进了 so） |
| 头文件 md5 | \`${HDR_MD5}\` |

## 核对命令

在 submodule 或 main 工作区执行：

\`\`\`bash
md5sum lib/aarch64/librecharge_mark.so
grep '^so_md5=' VERSION
# 两行 hash 相同 → 当前库与版本说明一致
\`\`\`

感知仓对照：

\`\`\`bash
md5sum recharge_mark/lib/aarch64/librecharge_mark.so
grep '^so_md5=' recharge_mark/VERSION
\`\`\`
EOF
