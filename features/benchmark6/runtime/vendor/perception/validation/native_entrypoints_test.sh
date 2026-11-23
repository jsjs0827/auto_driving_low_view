#!/usr/bin/env bash
set -euo pipefail
project=${1:?Pass project root}
count=0
while IFS= read -r -d '' script; do
  bash -n "$script"
  count=$((count+1))
done < <(find "$project/tools" "$project/archive/r36_sdk/tools" -name '*.sh' -print0)
printf 'Bash syntax PASS: %s scripts\n' "$count"
scratch=$(mktemp -d /tmp/perception-v3-entrypoints.XXXXXXXX)
trap 'rm -rf -- "$scratch"' EXIT
cat > "$scratch/launcher" <<'SH'
#!/usr/bin/env bash
printf '%s\n' "$@" > "$TEST_ARGV"
SH
chmod +x "$scratch/launcher"
export PERCEPTION_CONFIG="$scratch/launcher" TEST_ARGV="$scratch/args"
bash "$project/tools/run_pipeline.sh" --config 'config with spaces.json' --prepare-only
printf '%s\n' run --config 'config with spaces.json' --prepare-only > "$scratch/expected"
cmp "$scratch/expected" "$scratch/args"
bash "$project/tools/run_crop_compare.sh" --run-dir 'quality output with spaces'
printf '%s\n' run --config "$project/crop_compare.json" --run-dir 'quality output with spaces' > "$scratch/expected"
cmp "$scratch/expected" "$scratch/args"
bash "$project/tools/run_benchmark.sh" --run-dir 'performance output with spaces'
printf '%s\n' run --config "$project/benchmark.json" --run-dir 'performance output with spaces' > "$scratch/expected"
cmp "$scratch/expected" "$scratch/args"
bash "$project/tools/run_compare.sh" --config compare.json --run-dir 'output with spaces'
printf '%s\n' run --both --config compare.json --run-dir 'output with spaces' > "$scratch/expected"
cmp "$scratch/expected" "$scratch/args"
bash "$project/tools/capture_stereo.sh" --config capture.json --frames 2 --output 'capture output'
printf '%s\n' capture --config capture.json --frames 2 --output 'capture output' > "$scratch/expected"
cmp "$scratch/expected" "$scratch/args"
bash "$project/tools/run_camera_x5.sh" --mode both --config camera.json
printf '%s\n' camera --mode both --config camera.json > "$scratch/expected"
cmp "$scratch/expected" "$scratch/args"
bash "$project/tools/run_input.sh" 'input config.json' --prepare-only
printf '%s\n' run --config 'input config.json' --prepare-only > "$scratch/expected"
cmp "$scratch/expected" "$scratch/args"
if bash "$project/tools/run_stereo.sh" > "$scratch/retired.log" 2>&1; then
  echo 'Retired script unexpectedly succeeded' >&2; exit 1
fi
printf 'Runtime wrapper argv PASS: 7 modes + retired entrypoint\n'
mkdir -p "$scratch/package_source/tools" "$scratch/package_source/build" "$scratch/package_source/install/bin"
cp "$project/tools/package_release.sh" "$scratch/package_source/tools/"
printf 'source fixture\n' > "$scratch/package_source/CMakeLists.txt"
printf 'excluded build fixture\n' > "$scratch/package_source/build/stale.txt"
printf 'installed binary fixture\n' > "$scratch/package_source/install/bin/fixture"
bash "$scratch/package_source/tools/package_release.sh" "$scratch/package_output" > "$scratch/package.log"
(cd "$scratch/package_output" && sha256sum -c perception-stereo-cpp-v8.tar.gz.sha256)
tar -tzf "$scratch/package_output/perception-stereo-cpp-v8.tar.gz" > "$scratch/package_files"
grep -q 'package_source/install/bin/fixture' "$scratch/package_files"
if grep -q 'build/stale.txt' "$scratch/package_files"; then
  echo 'Build artifacts leaked into release' >&2; exit 1
fi
cp "$scratch/package_output/perception-stereo-cpp-v8.tar.gz.sha256" "$scratch/before.sha256"
if bash "$scratch/package_source/tools/package_release.sh" "$scratch/package_output" > /dev/null 2>&1; then
  echo 'Existing release was overwritten' >&2; exit 1
fi
cmp "$scratch/before.sha256" "$scratch/package_output/perception-stereo-cpp-v8.tar.gz.sha256"
# A prohibited extension fixture validates rejection; no interpreter is run.
printf 'prohibited extension fixture\n' > "$scratch/package_source/unwanted.py"
if bash "$scratch/package_source/tools/package_release.sh" "$scratch/package_rejected" > /dev/null 2>&1; then
  echo 'Prohibited program extension was packaged' >&2; exit 1
fi
printf 'Release packaging PASS: hash, installed binary, exclusions, no-overwrite and pure-source rejection\n'
