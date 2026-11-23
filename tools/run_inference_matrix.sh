#!/usr/bin/env bash
# Run one current-v6 test case; ROS, direct SDK camera, or offline files.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
usage() {
  cat <<'HELP'
Usage: bash tools/run_inference_matrix.sh --source camera|ros|offline --method original|crop
  [--config-dir DIR] [--run-dir NEW] [--duration SECONDS] [--min-frames N] [--prepare-only]
Default configs: config/inference_matrix. Default duration: 30 wall seconds.
Set ROS_SETUP to the actual board ROS setup.bash when it is not /opt/ros/humble/setup.bash.
ROS input requires a separate publisher. Camera input opens the SDK directly.
All three sources use the current ROS-linked demo. Python 3 checks the evidence.
Offline runs must complete every configured file once before the duration ends.
HELP
}
source_name=; method=; configs="$root/config/inference_matrix"; run=; duration=30; prepare=0; min_frames=10
while (($#)); do
  case "$1" in
    --help|-h) usage; exit 0;;
    --prepare-only) prepare=1; shift; continue;;
  esac
  (($# >= 2)) || { echo "Missing value for $1" >&2; exit 2; }
  case "$1" in
    --source) source_name=$2;; --method) method=$2;; --config-dir) configs=$2;;
    --run-dir) run=$2;; --duration) duration=$2;; --min-frames) min_frames=$2;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2;;
  esac
  shift 2
done
case "$source_name" in camera|ros|offline) ;; *) usage >&2; exit 2;; esac
case "$method" in original|crop) ;; *) usage >&2; exit 2;; esac
[[ $duration =~ ^[1-9][0-9]*$ ]] || { echo '--duration must be positive whole seconds' >&2; exit 2; }
[[ $min_frames =~ ^[1-9][0-9]*$ ]] || { echo '--min-frames must be a positive integer' >&2; exit 2; }
config="$configs/${source_name}_${method}.json"
[[ -f $config ]] || { echo "Missing configuration: $config" >&2; exit 2; }
run=${run:-$root/results/inference_matrix/${source_name}_${method}_$(date +%Y%m%d_%H%M%S)_$$}
[[ ! -e $run && ! -e $run.console.log ]] || { echo "Choose a NEW run directory: $run" >&2; exit 2; }
source "$root/tools/x5_runtime_env.sh"
ros_setup=${ROS_SETUP:-/opt/ros/humble/setup.bash}
if [[ -r $ros_setup ]]; then
  set +u
  source "$ros_setup"
  set -u
fi
[[ -x $INSTALL_ROOT/bin/perception_config ]] || { echo 'Build the current configuration tool first.' >&2; exit 2; }
args=(run --config "$config" --run-dir "$run")
if ((prepare)); then
  exec "$INSTALL_ROOT/bin/perception_config" "${args[@]}" --prepare-only
fi
[[ -x $INSTALL_ROOT/lib/perception_ros_so_demo/demo ]] || {
  echo 'Missing current demo. Build with tools/build_inference_matrix_x5.sh (ARM ROS development prefix required).' >&2; exit 2;
}
command -v timeout >/dev/null || { echo 'GNU timeout is required for bounded test runs.' >&2; exit 2; }
command -v python3 >/dev/null || { echo 'Python 3 is required to check frames and saved results.' >&2; exit 2; }
mkdir -p -- "$(dirname -- "$run")"
set +e
timeout --signal=INT --kill-after=10s "${duration}s" \
  "$INSTALL_ROOT/bin/perception_config" "${args[@]}" 2>&1 | tee "$run.console.log"
pipeline_status=("${PIPESTATUS[@]}")
set -e
status=${pipeline_status[0]}
if [[ -d $run ]]; then
  mv -- "$run.console.log" "$run/console.log"
  [[ ${pipeline_status[1]} == 0 ]] || { echo 'Failed to write console log.' >&2; exit 2; }
  exec python3 "$root/tools/check_inference_matrix_run.py" "$run" \
    --source "$source_name" --method "$method" --process-code "$status" --min-frames "$min_frames"
fi
echo "Test process failed with exit code $status; inspect $run or $run.console.log" >&2
[[ $status != 0 ]] || exit 3
exit "$status"
