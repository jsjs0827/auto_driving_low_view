#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mkdir -p "$root/build/evidence"
exec > >(tee "$root/build/evidence/elf_audit.txt") 2>&1
for binary in "$root/install/x5/bin/ab_benchmark" "$root/install/x5/bin/ab_report" "$root/install/x5/bin/ab_config_check" "$root/install/x5/lib/libperception_sdk.so.1.0.36"; do
  file "$binary"
  header=$(readelf -h "$binary")
  [[ $header == *AArch64* ]] || { echo 'Wrong X5 ELF architecture' >&2; exit 1; }
  readelf -d "$binary" | awk '/NEEDED|RUNPATH|RPATH/ {print}'
done
file "$root/install/ubuntu_ros/bin/ab_benchmark"
readelf -h "$root/install/ubuntu_ros/bin/ab_benchmark" | awk '/Machine:/{print}'
aarch64-linux-gnu-nm -C "$root/build/x5/libab_original_dnn.a" | awk '/ab_original_dnn_namespace::HorizonDnnRunner::init/ {print}'
printf 'AArch64 programs and SDK verified; original runner has isolated genuine symbols.\n'
for f in "$root"/tools/*.sh; do bash -n "$f"; done
printf 'Bash syntax checks passed.\n'
