#!/usr/bin/env bash
# Package source and, when present, the installed native binaries.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
bash "$ROOT/tools/check_no_local_payload.sh" "$ROOT"
out=${1:-$ROOT/../releases}
mkdir -p -- "$out"
out=$(cd -- "$out" && pwd)
case "$out/" in "$ROOT/"*) echo 'Release output must be outside the source tree.' >&2; exit 2;; esac
archive="$out/perception-stereo-cpp-v8-dag-v2.tar.gz"
[[ ! -e $archive && ! -e $archive.sha256 ]] || { echo "Refusing to overwrite existing release: $archive" >&2; exit 2; }
if find "$ROOT" -type f \( -name '*.py' -o -name '*.pyc' \) -print -quit | grep -q .; then
  echo 'Project contains prohibited Python program files.' >&2; exit 2
fi
tar -czf "$archive" -C "$(dirname -- "$ROOT")" \
  --exclude='*/build' --exclude='*/log' --exclude='*/releases' --exclude='*/.git' \
  --exclude='*/__pycache__' --exclude='*.py' --exclude='*.pyc' "$(basename -- "$ROOT")"
(cd -- "$out" && sha256sum "$(basename -- "$archive")" > "$(basename -- "$archive").sha256")
printf 'Release: %s\n' "$archive"
