#!/usr/bin/env bash
# Run on the Ubuntu host using the existing historical X5 build container.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
container=obs-v004-x5-build
sysroot=/work/obs_v004_x5/sysroot_docker
prefix=; horizon=; jobs=${BUILD_JOBS:-2}; out=
usage() {
  echo 'Usage: build_in_container.sh [--container NAME] [--sysroot PATH] [--dependency-prefix PATH] [--stereo-horizon-root PATH] [--jobs N] [--out NEW_DIR]'
  echo 'Defaults: obs-v004-x5-build; /work/obs_v004_x5/sysroot_docker; dependencies in SYSROOT/usr_x5.'
}
while (($#)); do
  case "$1" in --help|-h) usage; exit 0;; esac
  (($# >= 2)) || { echo "Missing value: $1" >&2; exit 2; }
  case "$1" in
    --container) container=$2;; --sysroot) sysroot=$2;;
    --dependency-prefix) prefix=$2;; --stereo-horizon-root) horizon=$2;;
    --jobs) jobs=$2;; --out) out=$2;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2;;
  esac
  shift 2
done
bash "$ROOT/tools/check_no_local_payload.sh" "$ROOT"
prefix=${prefix:-$sysroot/usr_x5}
horizon=${horizon:-$prefix}
out=${out:-$ROOT/../perception_dag_v4_docker_$(date +%Y%m%d_%H%M%S)_$$}
[[ $sysroot == /* && $prefix == /* && $horizon == /* ]] || { echo 'Container dependency paths must be absolute.' >&2; exit 2; }
[[ $jobs =~ ^[1-9][0-9]*$ ]] || { echo '--jobs must be a positive integer.' >&2; exit 2; }
for command in docker tar sha256sum tee; do command -v "$command" >/dev/null; done
for path in build/benchmark6/r36_benchmark6 features/benchmark6/runtime/install/ubuntu_ros/bin/ab_result_replay; do
  [[ -f $ROOT/$path ]] || { echo "Missing Ubuntu host tool: $path; run build_ubuntu_x5.sh --host-only on Ubuntu first." >&2; exit 2; }
done
[[ ! -e $out ]] || { echo "Choose a NEW output directory: $out" >&2; exit 2; }
running=$(docker inspect --format '{{.State.Running}}' "$container")
if [[ $running != true ]]; then docker start "$container" >/dev/null; fi
image=$(docker inspect --format '{{.Config.Image}}' "$container")
mkdir -p -- "$out"
out=$(cd -- "$out" && pwd)
remote_parent=$(docker exec "$container" mktemp -d /tmp/perception-dag-v4-docker-XXXXXXXX)
[[ $remote_parent == /tmp/perception-dag-v4-docker-* && $remote_parent != *$'\n'* ]] || { echo 'Unexpected container work directory.' >&2; exit 2; }
remote="$remote_parent/$(basename -- "$ROOT")"
trap 'status=$?; if ((status)); then printf "Docker build failed (%s). Logs: %s\nContainer work directory: %s:%s\n" "$status" "$out" "$container" "$remote_parent" >&2; fi' EXIT
printf 'Container: %s\nImage: %s\nSysroot: %s\nDependency prefix: %s\nWork directory: %s\n' "$container" "$image" "$sysroot" "$prefix" "$remote" | tee "$out/environment.txt"
docker exec "$container" mkdir -p "$remote/build/benchmark6"
# Preserve Ubuntu executables but exclude all old CMake caches.
tar -C "$ROOT" --exclude=build --exclude=dist --exclude=results --exclude=.git \
  --exclude=__pycache__ --exclude=.idea --exclude=.vscode --exclude='./windows/deps' \
  --exclude='./install/backend_sdk_x5' --exclude='./install/perception_ros_so_demo_x5' \
  --exclude='./features/benchmark6/runtime/install/x5' --exclude='./validation/docker_build.json' -cf - . |
  docker exec -i "$container" tar -xf - -C "$remote"
docker cp "$ROOT/build/benchmark6/r36_benchmark6" "$container:$remote/build/benchmark6/r36_benchmark6"
environment=(-e "X5_SYSROOT=$sysroot" -e "TARGET_SYSTEM_PREFIX=$prefix"
  -e "STEREO_HORIZON_ROOT=$horizon" -e "X5_CXX_COMPILER=/usr/bin/aarch64-linux-gnu-g++"
  -e "TARGET_OPENCV_DIR=$prefix/lib/aarch64-linux-gnu/cmake/opencv4"
  -e "TARGET_PCL_DIR=$prefix/lib/aarch64-linux-gnu/cmake/pcl"
  -e "TARGET_EIGEN_DIR=$prefix/share/eigen3/cmake"
  -e "TARGET_BOOST_DIR=$prefix/lib/aarch64-linux-gnu/cmake/Boost-1.74.0"
  -e "PSDK_ROOT=$remote/install/backend_sdk_x5" -e "INSTALL_ROOT=$remote/install/perception_ros_so_demo_x5"
  -e "SDK_BUILD_ROOT=$remote/build/backend_x5" -e "COMPARE_BUILD_ROOT=$remote/build/compare_x5"
  -e "AB_BUILD_DIR=$remote/features/benchmark6/runtime/build/x5"
  -e "AB_INSTALL_DIR=$remote/features/benchmark6/runtime/install/x5"
  -e "BUILD_JOBS=$jobs" -e "X5_DOCKER_CONTAINER=$container" -e "X5_DOCKER_IMAGE=$image")
docker exec -i --workdir "$remote" "${environment[@]}" "$container" bash -s <<'BUILD' 2>&1 | tee "$out/build.log"
set -euo pipefail
unset CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH
for command in cmake ninja python3 readelf file aarch64-linux-gnu-g++; do command -v "$command"; done
[[ -d $X5_SYSROOT/usr/include ]] || {
  echo "Sysroot usr/include is missing or its historical usr link is broken: $X5_SYSROOT/usr" >&2
  echo 'See BUILD_DOCKER_zh.md; the launcher does not modify the shared sysroot.' >&2
  exit 2
}
bash tools/build_ubuntu_x5.sh --x5-only
mkdir -p validation
python3 - <<'PY'
from pathlib import Path
from datetime import datetime,timezone
import json,os,subprocess
record={
    'status':'cross_compilation_passed',
    'completed_at_utc':datetime.now(timezone.utc).isoformat(),
    'container':os.environ['X5_DOCKER_CONTAINER'],
    'image':os.environ['X5_DOCKER_IMAGE'],
    'sysroot':os.environ['X5_SYSROOT'],
    'target_dependency_prefix':os.environ['TARGET_SYSTEM_PREFIX'],
    'compiler':subprocess.check_output([os.environ['X5_CXX_COMPILER'],'--version'],text=True).splitlines()[0],
    'target_components':['business SDK','DAG business tools','benchmark6 X5 runtime'],
    'ubuntu_host_tools':'included prebuilt Ubuntu 22.04 x86_64 / ROS2 Humble tools; not rebuilt in this container',
    'X5_board_tests':'not_run_by_this_build_script',
}
Path('validation/docker_build.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
PY
python3 tools/package_ubuntu_x5.py --sysroot "$X5_SYSROOT" \
  --target-prefix "$TARGET_SYSTEM_PREFIX" --output "$PWD/dist/perception-dag-v4-ubuntu-docker.tar.gz"
BUILD
docker cp "$container:$remote/dist/perception-dag-v4-ubuntu-docker.tar.gz" "$out/"
docker cp "$container:$remote/dist/perception-dag-v4-ubuntu-docker.tar.gz.sha256" "$out/"
(cd -- "$out" && sha256sum -c perception-dag-v4-ubuntu-docker.tar.gz.sha256)
printf 'Release: %s/perception-dag-v4-ubuntu-docker.tar.gz\nBuild log: %s/build.log\n' "$out" "$out"
