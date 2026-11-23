#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
bash "$root/tools/check_no_local_payload.sh" "$root"
workspace=$(dirname -- "$root")
native=${1:?Usage: prepare_wsl_native_cache.sh NEW_NATIVE_DIRECTORY [WINDOWS_CREATED_HEADER_TAR]}
headers=${2:-$root/build/wsl_cache/x5_headers.tar}
[[ $native == /home/* && $native != *..* && ! -e $native ]] || { echo 'Choose a NEW absolute /home directory' >&2;exit 2; }
[[ -f $headers ]] || { echo 'Create x5_headers.tar with Windows tar.exe first; see WSL_BUILD_zh.md' >&2;exit 2; }
mkdir -p "$native/source"
tar -C "$root" --exclude='./build' --exclude='./install' --exclude='./dist' --exclude='*/build' --exclude='*/install' --exclude='*/__pycache__' -cf - . | tar -C "$native/source" -xf -
python3 - "$workspace/local_env/x86_ros_v8p1/build/runner_x5/sysroot" "$native/sysroot" <<'PY'
from pathlib import Path
import shutil,sys
source,target=map(Path,sys.argv[1:])
target.mkdir();(target/'usr').mkdir()
for item in source.iterdir():
    if item.name!='usr':(target/item.name).symlink_to(item,target_is_directory=item.is_dir())
for item in (source/'usr').iterdir():
    if item.name!='include':(target/'usr'/item.name).symlink_to(item,target_is_directory=item.is_dir())
PY
tar -C "$native/sysroot/usr" -xf "$headers"
printf '%s\n' "$native" > "$root/build/wsl_release_logs/native_cache.txt"
printf 'Native source and headers ready: %s\n' "$native"
