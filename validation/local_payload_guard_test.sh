#!/usr/bin/env bash
set -euo pipefail
root=${1:?Pass source project root}
check="$root/tools/check_no_local_payload.sh"
tmp=$(mktemp -d /tmp/perception-release-guard-XXXXXXXX)
trap 'rm -rf -- "$tmp"' EXIT
mkdir -p "$tmp/tree/src"
printf 'int ordinary(void) { return 0; }\n' >"$tmp/tree/src/ordinary.c"
bash "$check" "$tmp/tree" >/dev/null
mkdir "$tmp/tree/local_only"
if bash "$check" "$tmp/tree" >"$tmp/rejected.log" 2>&1; then echo 'Private directory escaped release guard' >&2; exit 1; fi
rmdir "$tmp/tree/local_only"
printf '%s%s%s\n' 'LOCAL_ONLY_' 'NV12_SHM_' 'DO_NOT_DISTRIBUTE' >"$tmp/tree/src/renamed.bin"
if bash "$check" "$tmp/tree" >"$tmp/rejected.log" 2>&1; then echo 'Renamed private content escaped release guard' >&2; exit 1; fi
rm "$tmp/tree/src/renamed.bin"
touch "$tmp/tree/.do-not-package"
if bash "$check" "$tmp/tree" >"$tmp/rejected.log" 2>&1; then echo 'Sentinel escaped release guard' >&2; exit 1; fi
rm "$tmp/tree/.do-not-package"
ln -s "$tmp/rejected.log" "$tmp/tree/outside"
if bash "$check" "$tmp/tree" >"$tmp/rejected.log" 2>&1; then echo 'Escaping link escaped release guard' >&2; exit 1; fi
# The current developer tree contains a private feature, so the real packaging
# preflight must fail. This test only checks it; no archive is ever created.
if [[ -d $root/local_only ]]; then
 if bash "$check" "$root" >"$tmp/rejected.log" 2>&1; then echo 'Developer tree escaped release guard' >&2; exit 1; fi
fi
echo 'PASS release guard: clean, private directory/content/sentinel, link and developer tree checks'
