#!/usr/bin/env python3
"""Summarize completed v6 matrix runs without inventing missing measurements."""
import argparse
import json
import math
import statistics
from pathlib import Path

FIELDS = (
    "load_or_acquire_ms", "queue_wait_ms", "compute_ms", "graph_ms", "stereo_ms",
    "save_ms", "worker_frame_ms", "process_cpu_percent_over_worker_frame", "rss_kib",
    "det_pre_ms", "det_infer_ms", "det_post_ms", "seg_pre_ms", "seg_infer_ms", "seg_post_ms",
)


def summarize(directory, warmup):
    path = directory / "metrics.jsonl"
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not rows:
        raise ValueError(f"No completed frames in {path}")
    cases = {(row["input_mode"], row["disparity_source"]) for row in rows}
    if len(cases) != 1:
        raise ValueError(f"Mixed source/method rows in {path}")
    source, method = next(iter(cases))
    measured = rows[warmup:]
    metrics = {}
    for name in FIELDS:
        values = [row[name] for row in measured if isinstance(row.get(name), (int, float))
                  and not isinstance(row[name], bool) and math.isfinite(row[name])]
        ordered = sorted(values)
        metrics[name] = None if not ordered else {
            "samples": len(values), "mean": statistics.mean(values),
            "p50": statistics.median(values), "p95": ordered[max(0, math.ceil(len(values) * .95) - 1)],
        }
    runner_path = directory / "runner_status.json"
    runner = json.loads(runner_path.read_text(encoding="utf-8")) if runner_path.exists() else None
    return {
        "run_directory": str(directory.resolve()), "source": source, "disparity_source": method,
        "completed_metric_frames": len(rows), "warmup_rows_excluded": min(warmup, len(rows)),
        "measurement_rows": len(measured), "metrics": metrics,
        "completed_fps_since_ready": rows[-1].get("completed_fps_since_ready"),
        "dropped_pending_frames": rows[-1].get("dropped_pending_frames"),
        "dropped_unmatched_captures": rows[-1].get("dropped_unmatched_captures"),
        "runner_status": runner,
        "stereo_timing_scope": "new model provider including preprocessing and postprocessing" if method == "raft_stereo"
            else "vendor Q8 import in this graph; upstream original disparity algorithm is not timed separately",
        "cpu_scope": "consumer process only; a separate ROS publisher is excluded" if source == "ros"
            else "demo process, including SDK camera threads for camera input",
        "comparison_scope": "independent runs; no ground truth accuracy claim",
        "bpu_percent": None,
        "note": "BPU utilization is not measured by this ROS metrics stream. Missing timing values stay null."
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--warmup", type=int, default=10)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.warmup < 0:
        parser.error("--warmup must be nonnegative")
    try:
        result = {"schema_version": 1, "runs": [summarize(path, args.warmup) for path in args.runs]}
        value = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        if args.output:
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(value)
            print(args.output)
        else:
            print(value, end="")
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
