#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
workspace=$(dirname -- "$root")
release="$workspace/releases/perception-ab-benchmark-v9-20261015"
[[ ! -e $release ]] || { echo 'Release directory already exists; do not overwrite a release' >&2; exit 2; }
mkdir -p "$release/staging/source/perception-ab-benchmark-v9" "$release/staging/x5" "$release/staging/ubuntu"
source_dir="$release/staging/source/perception-ab-benchmark-v9"
for name in CMakeLists.txt .clang-format README_zh.md VALIDATION.md ELF_DEPENDENCIES.txt SOURCE_PROVENANCE.json config include src tools tests data models vendor; do
  cp -a -- "$root/$name" "$source_dir/"
done
for install in "$root/install/x5" "$root/install/ubuntu_ros"; do
  cp -- "$root/README_zh.md" "$root/VALIDATION.md" "$root/ELF_DEPENDENCIES.txt" "$root/SOURCE_PROVENANCE.json" "$install/"
  (cd "$install" && find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS)
  (cd "$install" && sha256sum -c SHA256SUMS > /dev/null)
done
cp -a -- "$root/install/x5" "$release/staging/x5/perception-ab-benchmark-v9"
cp -a -- "$root/install/ubuntu_ros" "$release/staging/ubuntu/perception-ab-benchmark-v9-ubuntu"
(cd "$release/staging/source" && cmake -E tar cf "$release/perception-ab-benchmark-v9-source.zip" --format=zip perception-ab-benchmark-v9)
tar -czf "$release/perception-ab-benchmark-v9-x5.tar.gz" -C "$release/staging/x5" perception-ab-benchmark-v9
tar -czf "$release/perception-ab-benchmark-v9-ubuntu-ros.tar.gz" -C "$release/staging/ubuntu" perception-ab-benchmark-v9-ubuntu
mkdir -p "$release/validation"
cp -- "$root/build/evidence/elf_audit.txt" "$root/build/evidence/host_final.log" "$root/build/evidence/x5_build.log" "$root/build/evidence/transport_final.log" "$root/VALIDATION.md" "$release/validation/"
(cd "$release" && sha256sum ./*.zip ./*.tar.gz > SHA256SUMS)
printf 'Release files:\n'
ls -lh "$release"/*.zip "$release"/*.tar.gz "$release/SHA256SUMS"
