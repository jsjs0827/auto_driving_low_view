#!/usr/bin/env bash
set -euo pipefail
[[ $# == 1 ]] || { echo 'camera_benchmark_entrypoints_test.sh PROJECT_ROOT' >&2; exit 2; }
project=$(cd -- "$1" && pwd)
temporary=$(mktemp -d /tmp/camera-benchmark-entrypoints.XXXXXX)
trap 'rm -rf -- "$temporary"' EXIT
mkdir -p "$temporary/install/bin"
: > "$temporary/ros_setup.bash"
cat > "$temporary/install/bin/r36_camera_compare" <<'MOCK'
#!/usr/bin/env bash
set -euo pipefail
printf '%s\n' "$@" > "${CAMERA_WRAPPER_ARGS:?}"
MOCK
chmod +x "$temporary/install/bin/r36_camera_compare"
export INSTALL_ROOT="$temporary/install" ROS_SETUP="$temporary/ros_setup.bash"
export CAMERA_WRAPPER_ARGS="$temporary/arguments.txt"
unset SDK_ROOT
cd -- "$temporary"
bash -n "$project/tools/run_camera_benchmark.sh"
bash -n "$project/tools/run_camera_experiment.sh"
bash "$project/tools/run_camera_benchmark.sh" --output "$temporary/output with spaces" --method stereo
printf '%s\n' --config "$project/camera_benchmark.json" --output "$temporary/output with spaces" --method stereo > "$temporary/expected.txt"
cmp "$temporary/arguments.txt" "$temporary/expected.txt"
bash "$project/tools/run_camera_experiment.sh" --output "$temporary/single output" --method original
printf '%s\n' --config "$project/camera_benchmark.json" --output "$temporary/single output" --method original > "$temporary/expected.txt"
cmp "$temporary/arguments.txt" "$temporary/expected.txt"
for method in detseg_only original stereo; do
  bash "$project/tools/run_camera_experiment.sh" --method "$method" --output "$temporary/$method"
done
for mode in missing invalid duplicate empty; do
  args=(--output "$temporary/bad")
  case "$mode" in
    invalid) args+=(--method all);;
    duplicate) args+=(--method original --method stereo);;
    empty) args+=(--method);;
  esac
  status=0
  bash "$project/tools/run_camera_experiment.sh" "${args[@]}" > /dev/null 2>&1 || status=$?
  [[ $status == 2 ]] || { echo "Invalid experiment selection was not rejected: $mode status=$status" >&2; exit 1; }
done
printf '%s\n' 'camera benchmark entrypoints: syntax, native controller lookup, source-relative config, quoted output and strict single-method selection passed'
