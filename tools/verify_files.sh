#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd -- "${1:-$ROOT}"
[[ -f SHA256SUMS ]] || { echo 'No SHA256SUMS: run this check on a finalized release.' >&2; exit 2; }
sha256sum -c SHA256SUMS
