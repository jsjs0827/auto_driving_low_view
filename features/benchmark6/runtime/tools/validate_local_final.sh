#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mkdir -p "$root/build/evidence"
bash "$root/tools/build_host.sh" --ros > "$root/build/evidence/host_final.log" 2>&1
bash "$root/tools/validate_host_transport.sh" | tee "$root/build/evidence/transport_final.log"
printf 'Final Ubuntu build, four CTests and DDS/TCP transfer checks passed.\n'
