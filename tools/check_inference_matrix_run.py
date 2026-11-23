#!/usr/bin/env python3
"""Validate a bounded matrix run from completed-frame and saved-result evidence."""
import argparse
import json
from pathlib import Path
import re


def check(run, source, method, process_code, min_frames=10):
    failures = []
    rows = []
    expected = None
    saved = 0
    if process_code != 124:
        failures.append(f"Node exited before the requested duration or failed (exit {process_code}; expected GNU timeout 124)")
    try:
        resolved = json.loads((run / "resolved.json").read_text(encoding="utf-8"))
        params = resolved["parameters"]
        rows = [json.loads(line) for line in (run / "metrics.jsonl").read_text(encoding="utf-8").splitlines() if line.strip()]
        disparity = "original_q8" if method == "original" else "raft_stereo"
        if source == "offline":
            expected = len(params["offline_files"])
            cap = params.get("offline_max_frames", 0)
            if cap:
                expected = min(expected, cap)
            if params.get("offline_loop", False):
                failures.append("Full offline verification requires offline_loop=false")
            if len(rows) != expected:
                failures.append(f"Offline input did not complete exactly once: expected {expected}, completed {len(rows)}; increase duration or inspect errors")
        minimum = min(min_frames, expected) if expected is not None else min_frames
        if len(rows) < max(1, minimum):
            failures.append(f"Too few completed frames: {len(rows)}")
        identities = set()
        for row in rows:
            if row["input_mode"] != source or row["disparity_source"] != disparity:
                failures.append("Metrics source/method differs from requested case")
                break
            identity = (row["frame_id"], row["timestamp_ns"])
            if identity in identities:
                failures.append("Duplicate completed frame identity")
                break
            identities.add(identity)
            tasks = {task["task"]: task["state"] for task in row["task_trace"]["tasks"]}
            if any(tasks.get(task) != "succeeded" for task in ("det", "seg", "stereo")):
                failures.append(f"Detection, segmentation and disparity were not all successful in frame {identity}")
                break
        save_every = params.get("save_every_n", 0)
        if save_every:
            save_root = Path(params["save_root"])
            for index, row in enumerate(rows):
                if index % save_every:
                    continue
                frame = save_root / f'frame-{row["frame_id"]}-{row["timestamp_ns"]}'
                for name in ("result.json", "mask.bin", "disparity_q8.u16", "task_trace.json", "complete.json"):
                    if not (frame / name).is_file():
                        failures.append(f"Missing saved result: {frame / name}")
                if (frame / "mask.bin").is_file() and (frame / "mask.bin").stat().st_size != 640 * 544:
                    failures.append(f"Unexpected segmentation mask size: {frame}")
                if (frame / "disparity_q8.u16").is_file() and (frame / "disparity_q8.u16").stat().st_size != 640 * 544 * 2:
                    failures.append(f"Unexpected disparity size: {frame}")
                saved += 1
        console = (run / "console.log").read_text(encoding="utf-8", errors="replace")
        if re.search(r"\[(?:ERROR|FATAL)\]|Input shutdown:", console):
            failures.append("Runtime error reported in console.log")
    except (OSError, ValueError, KeyError, TypeError) as error:
        failures.append(f"Missing or invalid run evidence: {error}")
    result = {
        "schema_version": 1, "source": source, "method": method,
        "scope": "all_offline_files_once" if source == "offline" else "bounded_live_run",
        "process_exit_code": process_code, "ended_by_duration": process_code == 124,
        "expected_offline_frames": expected, "observed_metric_frames": len(rows),
        "checked_saved_frames": saved, "passed": not failures, "failures": failures,
        "accuracy_evaluated": False,
    }
    with (run / "runner_status.json").open("x", encoding="utf-8") as output:
        output.write(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--source", choices=("camera", "ros", "offline"), required=True)
    parser.add_argument("--method", choices=("original", "crop"), required=True)
    parser.add_argument("--process-code", type=int, required=True)
    parser.add_argument("--min-frames", type=int, default=10)
    args = parser.parse_args()
    if args.min_frames < 1:
        parser.error("--min-frames must be positive")
    try:
        result = check(args.run, args.source, args.method, args.process_code, args.min_frames)
    except OSError as error:
        parser.error(str(error))
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["passed"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
