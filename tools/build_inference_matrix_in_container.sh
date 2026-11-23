#!/usr/bin/env bash
# Ubuntu-host entry for the current v6 SDK and ARM ROS application.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
container=obs-v004-x5-build
sysroot=/work/obs_v004_x5/sysroot_docker
prefix=; horizon=; ros_prefix=${TARGET_ROS_PREFIX:-}; eigen_dir=${TARGET_EIGEN_DIR:-}
jobs=${BUILD_JOBS:-2}; out=
usage() {
  cat <<'HELP'
Usage on the Ubuntu host:
  bash tools/build_inference_matrix_in_container.sh [options]
  --container NAME              default: obs-v004-x5-build
  --sysroot ABS                 default: /work/obs_v004_x5/sysroot_docker
  --dependency-prefix ABS       default: SYSROOT/usr_x5
  --stereo-horizon-root ABS     default: dependency prefix
  --target-ros-prefix ABS[;ABS]  default: discover the unique ARM ROS prefix
  --eigen-dir ABS               default: discover Eigen3Config.cmake
  --jobs N                      default: 2
  --out NEW_DIR                 host output directory; must not exist

Builds the current v6 SDK, native tools and ARM ROS demo. No Ubuntu benchmark6
executable or host ROS installation is required. Target ARM ROS development
packages are still required inside the container. Sources/builds use a new
container work directory; the shared sysroot is read only to this workflow.
Output: perception-v6-matrix-x5.tar.gz, .sha256, build.log, environment.txt.
The release uses the X5's matching ROS/Horizon/camera/system runtime libraries.
HELP
}
fail() { printf 'build_inference_matrix_in_container: %s\n' "$*" >&2; exit 2; }
while (($#)); do
  case "$1" in --help|-h) usage; exit 0;; esac
  (($# >= 2)) || fail "Missing value for $1"
  case "$1" in
    --container) container=$2;; --sysroot) sysroot=$2;;
    --dependency-prefix) prefix=$2;; --stereo-horizon-root) horizon=$2;;
    --target-ros-prefix) ros_prefix=$2;; --eigen-dir) eigen_dir=$2;;
    --jobs) jobs=$2;; --out) out=$2;;
    *) fail "Unknown argument: $1";;
  esac
  shift 2
done
bash "$ROOT/tools/check_no_local_payload.sh" "$ROOT"
prefix=${prefix:-$sysroot/usr_x5}
horizon=${horizon:-$prefix}
[[ $ros_prefix != auto ]] || ros_prefix=
[[ $eigen_dir != auto ]] || eigen_dir=
[[ $container =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || fail 'Invalid container name'
[[ $jobs =~ ^[1-9][0-9]*$ ]] || fail '--jobs must be a positive integer'
for path in "$sysroot" "$prefix" "$horizon"; do
  [[ $path == /* && $path != *$'\n'* && $path != *$'\r'* ]] || fail 'Container dependency paths must be absolute and single-line'
done
for path in "$ros_prefix" "$eigen_dir"; do
  [[ $path != *$'\n'* && $path != *$'\r'* ]] || fail 'ROS/Eigen paths must be single-line'
done
out=${out:-$(dirname -- "$ROOT")/v6_matrix_docker_$(date +%Y%m%d_%H%M%S)_$$}
[[ ! -e $out ]] || fail "Choose a NEW output directory: $out"
for program in docker tar sha256sum tee; do
  command -v "$program" >/dev/null || fail "Missing host command: $program"
done
running=$(docker inspect --format '{{.State.Running}}' "$container")
if [[ $running != true ]]; then docker start "$container" >/dev/null; fi
image=$(docker inspect --format '{{.Config.Image}}' "$container")
mkdir -p -- "$out"
out=$(cd -- "$out" && pwd)
remote_parent=$(docker exec "$container" mktemp -d /tmp/perception-v6-matrix-XXXXXXXX)
[[ $remote_parent == /tmp/perception-v6-matrix-* && $remote_parent != *$'\n'* && $remote_parent != *$'\r'* ]] ||
  fail 'Unexpected container work directory'
project=$(basename -- "$ROOT")
remote="$remote_parent/$project"
trap 'status=$?; if ((status)); then printf "Build failed (%s). Logs: %s\nContainer work directory: %s:%s\n" "$status" "$out" "$container" "$remote_parent" >&2; fi' EXIT
printf 'Container: %s\nImage: %s\nSysroot: %s\nDependency prefix: %s\nRequested ARM ROS: %s\nRequested Eigen: %s\nWork directory: %s\n' \
  "$container" "$image" "$sysroot" "$prefix" "${ros_prefix:-auto}" "${eigen_dir:-auto}" "$remote" | tee "$out/environment.txt"
excludes=(--exclude=build --exclude=install --exclude=dist --exclude=results --exclude=tmp
  --exclude=.git --exclude=__pycache__ --exclude=.idea --exclude=.vscode
  --exclude=./windows/deps --exclude=./validation/inference_matrix_docker_build.json)
# Avoid recursing into the release stage when --out is inside the source tree.
if [[ $out == "$ROOT/"* ]]; then excludes+=("--exclude=./${out#"$ROOT/"}"); fi
source_tar() { tar -C "$ROOT" "${excludes[@]}" -cf - .; }
docker exec "$container" mkdir -p "$remote"
source_tar | docker exec -i "$container" tar -xf - -C "$remote"
environment=(-e "X5_SYSROOT=$sysroot" -e "TARGET_SYSTEM_PREFIX=$prefix"
  -e "STEREO_HORIZON_ROOT=$horizon" -e "TARGET_ROS_PREFIX=$ros_prefix" -e "TARGET_EIGEN_DIR=$eigen_dir"
  -e "X5_CXX_COMPILER=/usr/bin/aarch64-linux-gnu-g++" -e "BUILD_JOBS=$jobs"
  -e "TARGET_OPENCV_DIR=$prefix/lib/aarch64-linux-gnu/cmake/opencv4"
  -e "TARGET_PCL_DIR=$prefix/lib/aarch64-linux-gnu/cmake/pcl"
  -e "PSDK_ROOT=$remote/install/backend_sdk_x5" -e "INSTALL_ROOT=$remote/install/perception_ros_so_demo_x5"
  -e "SDK_BUILD_ROOT=$remote/build/backend_x5" -e "COMPARE_BUILD_ROOT=$remote/build/compare_x5"
  -e "ROS_BUILD_ROOT=$remote/build/ros_x5"
  -e "MATRIX_DOCKER_CONTAINER=$container" -e "MATRIX_DOCKER_IMAGE=$image")
docker exec -i --workdir "$remote" "${environment[@]}" "$container" bash -s <<'BUILD' 2>&1 | tee "$out/build.log"
set -euo pipefail
unset CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH
source tools/prepare_inference_matrix_env.sh
bash tools/build_inference_matrix_x5.sh
for artifact in "$PSDK_ROOT/lib/libperception_sdk.so" \
  "$INSTALL_ROOT/bin/perception_config" \
  "$INSTALL_ROOT/lib/perception_ros_so_demo/demo" \
  "$INSTALL_ROOT/lib/perception_ros_so_demo/raw_camera"; do
  [[ -f $artifact ]] || { echo "Missing built artifact: $artifact" >&2; exit 2; }
  LC_ALL=C readelf -h "$artifact" | grep -q 'Machine:.*AArch64' || {
    echo "Built artifact is not AArch64: $artifact" >&2; exit 2;
  }
done
python3 - <<'PY'
from pathlib import Path
from datetime import datetime, timezone
import json, os, subprocess
record = {
    'status': 'cross_compilation_passed',
    'completed_at_utc': datetime.now(timezone.utc).isoformat(),
    'container': os.environ['MATRIX_DOCKER_CONTAINER'],
    'image': os.environ['MATRIX_DOCKER_IMAGE'],
    'sysroot': os.environ['X5_SYSROOT'],
    'dependency_prefix': os.environ['TARGET_SYSTEM_PREFIX'],
    'target_ros_prefix': os.environ['TARGET_ROS_PREFIX'],
    'eigen_dir': os.environ['TARGET_EIGEN_DIR'],
    'compiler': subprocess.check_output([os.environ['X5_CXX_COMPILER'], '--version'], text=True).splitlines()[0],
    'components': ['current v6 SDK', 'native tools', 'ARM ROS demo and raw_camera'],
    'four_entry_artifacts_aarch64': True,
    'runtime_dependency_bundling': False,
    'board_tests': 'not_run_by_this_script',
}
Path('validation').mkdir(exist_ok=True)
Path('validation/inference_matrix_docker_build.json').write_text(json.dumps(record, indent=2) + '\n', encoding='utf-8')
PY
BUILD
stage="$out/stage/$project"
mkdir -p -- "$stage/install" "$stage/validation"
source_tar | tar -xf - -C "$stage"
docker cp "$container:$remote/install/backend_sdk_x5" "$stage/install/"
docker cp "$container:$remote/install/perception_ros_so_demo_x5" "$stage/install/"
docker cp "$container:$remote/validation/inference_matrix_docker_build.json" "$out/docker_build.json"
cp -- "$out/docker_build.json" "$stage/validation/inference_matrix_docker_build.json"
for path in install/backend_sdk_x5/lib/libperception_sdk.so \
  install/perception_ros_so_demo_x5/bin/perception_config \
  install/perception_ros_so_demo_x5/lib/perception_ros_so_demo/demo \
  install/perception_ros_so_demo_x5/lib/perception_ros_so_demo/raw_camera; do
  [[ -s $stage/$path ]] || fail "Missing copied artifact: $path"
done
archive=perception-v6-matrix-x5.tar.gz
tar -C "$out/stage" -czf "$out/$archive" "$project"
(cd -- "$out" && sha256sum "$archive" > "$archive.sha256" && sha256sum -c "$archive.sha256")
printf 'Release: %s/%s\nBuild log: %s/build.log\nBuild evidence: %s/docker_build.json\n' "$out" "$archive" "$out" "$out"
