#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
bin=${AB_BIN:-$root/bin/ab_benchmark}
[[ -x $bin ]] || bin="$root/install/x5/bin/ab_benchmark"
export LD_LIBRARY_PATH="$root/lib:$root/install/x5/lib:/app/data/metoak/lib:/usr/hobot/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$bin" "$@"
