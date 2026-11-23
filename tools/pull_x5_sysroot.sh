#!/usr/bin/env bash
# Read-only board access; all writes go into a NEW Ubuntu host directory.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
case "${1:-}" in --help|-h) echo 'Usage: pull_x5_sysroot.sh NEW_ABSOLUTE_HOST_DIRECTORY'; exit 0;; esac
destination=${1:?Usage: pull_x5_sysroot.sh NEW_ABSOLUTE_HOST_DIRECTORY}
[[ $# == 1 && $destination == /* && $destination != / && $destination != *..* && ! -e $destination ]] || { echo 'Choose a NEW absolute host directory' >&2; exit 2; }
export B6_REAL_ADB=${B6_REAL_ADB:-$(command -v adb)}
adb_cmd=(python3 "$root/tools/adb_legacy_compat/adb")
[[ -z ${ADB_SERIAL:-} ]] || adb_cmd+=(-s "$ADB_SERIAL")
[[ $("${adb_cmd[@]}" get-state) == device ]] || { echo 'Select one online X5 using ADB_SERIAL' >&2; exit 2; }
for path in /usr/include /usr/lib /usr/hobot /lib/aarch64-linux-gnu /lib/ld-linux-aarch64.so.1; do
  "${adb_cmd[@]}" shell "test -e $path" || { echo "Missing board dependency directory: $path" >&2; exit 2; }
done
mkdir -p "$destination/usr" "$destination/lib"
"${adb_cmd[@]}" pull /usr/include "$destination/usr/"
"${adb_cmd[@]}" pull /usr/lib "$destination/usr/"
"${adb_cmd[@]}" pull /usr/hobot "$destination/usr/"
"${adb_cmd[@]}" pull /lib/aarch64-linux-gnu "$destination/lib/"
"${adb_cmd[@]}" pull /lib/ld-linux-aarch64.so.1 "$destination/lib/ld-linux-aarch64.so.1"
python3 - "$destination" <<'PY'
from pathlib import Path
import os,sys
root=Path(sys.argv[1]).resolve()
for p in root.rglob('*'):
    if not p.is_symlink(): continue
    target=os.readlink(p)
    if target.startswith('/'):
        p.unlink()
        p.symlink_to(os.path.relpath(root/target.lstrip('/'),p.parent))
checks=('usr/include/dnn/hb_dnn.h','usr/lib/aarch64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake','usr/lib/aarch64-linux-gnu/cmake/pcl/PCLConfig.cmake')
missing=[x for x in checks if not (root/x).is_file()]
if missing:
    print('Board runtime files copied, but development files are missing: '+', '.join(missing),file=sys.stderr)
    print('Use a matching development sysroot, or install matching target development packages before copying again into a NEW directory.',file=sys.stderr)
    sys.exit(2)
print('Development sysroot ready: '+str(root))
PY
