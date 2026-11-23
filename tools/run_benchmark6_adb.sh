#!/usr/bin/env bash
# Run on the Ubuntu host that owns the X5 ADB connection.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
config=${1:-$root/benchmark6.json}
output=${2:-$root/results/benchmark6_$(date +%Y%m%d_%H%M%S)}
ctl=${B6_CONTROL_BIN:-$root/build/benchmark6/r36_benchmark6}
runtime="$root/features/benchmark6/runtime"
host=${B6_HOST_BIN:-$runtime/install/ubuntu_ros/bin/ab_benchmark}
install=${B6_X5_INSTALL:-$runtime/install/x5}
for tool in adb timeout sha256sum python3; do command -v "$tool" >/dev/null; done
[[ -x $ctl && -x $host && -x $install/bin/ab_benchmark ]] || { echo 'Build host and X5 tools first; see BENCHMARK6_zh.md' >&2; exit 2; }
export B6_REAL_ADB=${B6_REAL_ADB:-$(command -v adb)}
adb_cmd=(python3 "$root/tools/adb_legacy_compat/adb")
[[ -z ${ADB_SERIAL:-} ]] || adb_cmd+=(-s "$ADB_SERIAL")
[[ $("${adb_cmd[@]}" get-state) == device ]] || { echo 'Select exactly one online X5 with ADB_SERIAL' >&2; exit 2; }
if "${adb_cmd[@]}" shell 'exit 7'; then probe_status=0; else probe_status=$?; fi
[[ $probe_status == 7 ]] || { echo "ADB exit-code check failed: $probe_status" >&2; exit 125; }
echo 'ADB compatibility check passed: remote exit code 7 recovered.'
set +u
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-67}
"$ctl" --prepare "$config" "$output"
output=$(cd -- "$output" && pwd)
value(){ "$ctl" --value "$output/plan.json" "$1"; }
board_root=$(value board_root); board=$(value board_output); port=$(value ros.bridge_port)
safe_board(){ [[ $1 =~ ^/[a-zA-Z0-9_./-]+$ && $1 != *..* && $1 != / ]]; }
safe_board "$board_root" && safe_board "$board" || { echo 'Unsafe board path' >&2; exit 2; }
export LD_LIBRARY_PATH="$runtime/install/ubuntu_ros/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
publisher=''; worker=''; current_pid=''
cleanup(){
  if [[ -n $publisher ]]; then kill -TERM "$publisher" 2>/dev/null || true; wait "$publisher" 2>/dev/null || true; publisher=''; fi
  if [[ -n $current_pid ]]; then
    "${adb_cmd[@]}" shell "sh -s $board_root $current_pid" >/dev/null 2>&1 <<'REMOTE' || true
binary=$(readlink -f "$1/bin/ab_benchmark")
pid=$(cat "$2" 2>/dev/null) || exit 0
case "$pid" in ''|*[!0-9]*) exit 0;; esac
owned(){ test "$(readlink -f "/proc/$pid/exe" 2>/dev/null)" = "$binary"; }
owned || exit 0
kill -TERM "$pid" 2>/dev/null || exit 0
for i in 1 2 3 4 5 6 7 8 9 10; do owned || exit 0; sleep 1; done
owned && kill -KILL "$pid" 2>/dev/null
REMOTE
    current_pid=''
  fi
  if [[ -n $worker ]]; then wait "$worker" 2>/dev/null || true; worker=''; fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
[[ -f $install/SHA256SUMS ]] || { echo "Missing runtime manifest: $install/SHA256SUMS" >&2; exit 2; }
runtime_digest=$(sha256sum "$install/SHA256SUMS"); runtime_digest=${runtime_digest%% *}
if "${adb_cmd[@]}" shell "test -e $board_root"; then
  if ! deployed_digest=$("${adb_cmd[@]}" shell "sha256sum $board_root/SHA256SUMS" | awk 'length($1)==64 && !seen {print $1; seen=1}'); then
    echo "Incomplete runtime at $board_root; choose a new board_root. Existing files were preserved." >&2; exit 2
  fi
  [[ $deployed_digest == "$runtime_digest" ]] || { echo "Different/incomplete runtime at $board_root. Choose a new board_root; existing files were preserved." >&2; exit 2; }
  echo "Verifying and reusing matching runtime: $board_root"
else
  "${adb_cmd[@]}" shell "mkdir -p $board_root"
  "${adb_cmd[@]}" push "$install/." "$board_root/" >"$output/logs/deploy.log" 2>&1
fi
"${adb_cmd[@]}" shell "cd $board_root && sha256sum -c SHA256SUMS" >"$output/logs/runtime-verify.log" 2>&1
"${adb_cmd[@]}" shell "test ! -e $board && mkdir -p $board && chmod +x $board_root/bin/ab_benchmark"
"${adb_cmd[@]}" push "$output/board-config" "$board/" >"$output/logs/config-push.log" 2>&1
lib="/app/data/metoak/lib:/app/data/metoak/3rdparty_libs/opencv_libs:/app/data/metoak/3rdparty_libs/metoak_libs:/usr/hobot/lib:/usr/lib/hbbpu:$board_root/lib"
"${adb_cmd[@]}" shell "export LD_LIBRARY_PATH=$lib; ldd $board_root/bin/ab_benchmark" >"$output/logs/board-ldd.txt" 2>&1
if grep -q 'not found' "$output/logs/board-ldd.txt"; then echo "Missing ARM libraries: see $output/logs/board-ldd.txt and X5_EXTRA_LIB_DIR in the guide" >&2; exit 2; fi
"${adb_cmd[@]}" shell "uname -a; cat /proc/cpuinfo; sha256sum $board_root/models/x5/*.bin" >"$output/board-environment.txt"
printf 'source\tmethod\tpass\texit_code\n' >"$output/run_status.tsv"
board_job(){
  local cfg=$1 method=$2 source=$3 target=$4
  current_pid="$board/job.pid"
  timeout --foreground --kill-after=15 "${B6_RUN_TIMEOUT:-3600}" "${adb_cmd[@]}" shell "echo \$\$ > $current_pid; cd $board_root; export LD_LIBRARY_PATH=$lib; exec bin/ab_benchmark --config $cfg --method $method --suite matched_roi --source $source --output $target"
}
current_pid="$board/job.pid"
if timeout --foreground --kill-after=15 "${B6_RUN_TIMEOUT:-3600}" "${adb_cmd[@]}" shell "echo \$\$ > $current_pid; cd $board_root; export LD_LIBRARY_PATH=$lib; exec bin/ab_benchmark --config $board/board-config/capture.json --source camera --record --output $board/capture" >"$output/logs/capture.log" 2>&1; then current_pid=''; else echo "Capture failed; see $output/logs/capture.log" >&2; exit 1; fi
"${adb_cmd[@]}" pull "$board/capture" "$output/" >"$output/logs/capture-pull.log" 2>&1
for source in camera offline ros; do
  for method in original_q8 stereo_crop; do
    algorithm=optimized_q8; [[ $method != stereo_crop ]] || algorithm=optimized_stereo
    "${adb_cmd[@]}" shell "mkdir -p $board/runs/$source/$method"
    for pass in performance quality; do
      cfg="$board/board-config/${source}_${pass}.json"
      target="$board/runs/$source/$method/$pass"
      log="$output/logs/${source}_${method}_${pass}"
      status=0
      if [[ $source == ros ]]; then
        "${adb_cmd[@]}" forward "tcp:$port" "tcp:$port"
        # Separate host processes use actual ROS DDS; only the bridge talks ADB TCP.
        current_pid="$board/job.pid"
        board_job "$cfg" "$algorithm" "$source" "$target" >"$log.worker.log" 2>&1 & worker=$!
        "$host" --config "$output/host-config/ros_$pass.json" --method "$algorithm" --publish --publish-source offline >"$log.publisher.log" 2>&1 & publisher=$!
        if timeout --foreground --kill-after=15 "${B6_RUN_TIMEOUT:-3600}" "$host" --config "$output/host-config/ros_$pass.json" --method "$algorithm" --bridge-forward >"$log.bridge.log" 2>&1; then
          if wait "$worker"; then worker='';current_pid=''; else status=$?;worker='';current_pid=''; fi
        else status=$?; fi
        cleanup
      else
        if board_job "$cfg" "$algorithm" "$source" "$target" >"$log.worker.log" 2>&1; then current_pid=''; else status=$?;cleanup; fi
      fi
      printf '%s\t%s\t%s\t%s\n' "$source" "$method" "$pass" "$status" >>"$output/run_status.tsv"
      printf '%s / %s / %s -> %s\n' "$source" "$method" "$pass" "$status"
    done
  done
done
"${adb_cmd[@]}" pull "$board/runs" "$output/" >"$output/logs/results-pull.log" 2>&1
# Visualization is generated on Ubuntu after all measured processes have exited.
visual_status=0
bash "$root/tools/save_benchmark6_visuals.sh" "$output" || visual_status=$?
printf 'visualization_exit_code=%s\n' "$visual_status" >"$output/visualization_status.txt"
"$ctl" --report "$output" >"$output/logs/report.log"
printf 'Results: %s/index.html\nRaw run statuses: %s/run_status.tsv\n' "$output" "$output"
[[ $("$ctl" --value "$output/summary.json" complete_groups) == 6 && $visual_status == 0 ]]
