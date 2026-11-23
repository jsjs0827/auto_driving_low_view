#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
bin="$root/build/host_ros"
set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID=189 ROS_LOCALHOST_ONLY=1
mkdir -p "$root/build/evidence"
out=$(mktemp -d "$root/build/evidence/transport-XXXXXXXX")
"$bin/ab_input_tests" "$out/fixture" "$root/config/offline.json"
publisher=""
cleanup(){ [[ -z $publisher ]] || { kill -TERM "$publisher" 2>/dev/null || true; wait "$publisher" 2>/dev/null || true; publisher=""; }; }
trap cleanup EXIT INT TERM
"$bin/ab_benchmark" --config "$out/fixture/dds.json" --source offline --method original --validate-input --output "$out/offline" > "$out/offline.log" 2>&1
"$bin/ab_benchmark" --config "$out/fixture/dds.json" --source ros --method original --validate-input --output "$out/dds" > "$out/dds.log" 2>&1 &
worker=$!
"$bin/ab_benchmark" --config "$out/fixture/dds.json" --method original --publish --publish-source offline > "$out/dds_publisher.log" 2>&1 &
publisher=$!
wait "$worker"
cleanup
"$bin/ab_benchmark" --config "$out/fixture/tcp.json" --source ros --method original --validate-input --output "$out/tcp" > "$out/tcp.log" 2>&1 &
worker=$!
"$bin/ab_benchmark" --config "$out/fixture/dds.json" --method original --publish --publish-source offline > "$out/tcp_publisher.log" 2>&1 &
publisher=$!
"$bin/ab_benchmark" --config "$out/fixture/tcp.json" --method original --bridge-forward > "$out/bridge.log" 2>&1
wait "$worker"
cleanup
if "$bin/ab_benchmark" --config "$out/fixture/dds.json" --source offline --method original --validate-input --output "$out/offline" > "$out/existing_dir.log" 2>&1; then echo 'Existing output accepted unexpectedly' >&2; exit 1; fi
[[ ! -f "$out/offline/failure.json" ]]
grep -F '1200000000' "$out/dds.log" >/dev/null || grep -F '1080000000' "$out/dds.log" >/dev/null
grep -F '1080000000' "$out/tcp.log" >/dev/null
printf 'Offline + real ROS DDS + ROS->TCP normalized frame transfer passed; synthetic transport fixture only. Evidence=%s\n' "$out"
