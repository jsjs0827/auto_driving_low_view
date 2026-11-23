#!/usr/bin/env python3
"""Prepare the current v6 det/seg/disparity six-case configuration set."""
import argparse
import json
import math
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FEATURES = (
    "ros", "offline", "stereo", "inference", "detection", "segmentation",
    "pointcloud", "fusion", "metrics", "temporal", "recharge",
    "visual_motion", "motion_reference", "slip_detection",
)


def reference(path, base):
    try:
        return Path(os.path.relpath(path, base)).as_posix()
    except ValueError:  # Different Windows drives.
        return Path(path).as_posix()


def pictures(directory, suffixes):
    if directory is None:
        return {}
    directory = directory.resolve()
    if not directory.is_dir():
        raise ValueError(f"Input directory does not exist: {directory}")
    result = {}
    for path in sorted(directory.iterdir()):
        if not path.is_file() or path.suffix.lower() not in suffixes:
            continue
        if path.stem in result:
            raise ValueError(f"Duplicate frame stem {path.stem!r} in {directory}")
        result[path.stem] = path.resolve()
    if not result:
        raise ValueError(f"No supported input files in {directory}")
    return result


def generate(args):
    output = args.output.resolve()
    if output.exists():
        raise ValueError(f"Choose a NEW configuration directory: {output}")
    if not math.isfinite(args.fps) or not 0 < args.fps <= 120:
        raise ValueError("--fps must be finite and in (0,120]")
    if args.save_every < 0:
        raise ValueError("--save-every must be nonnegative")
    image_suffixes = {".png", ".jpg", ".jpeg", ".bmp", ".ppm", ".pgm", ".pnm"}
    left = pictures(args.left_dir, image_suffixes)
    right = pictures(args.right_dir, image_suffixes)
    q8 = pictures(args.q8_dir, {".png", ".raw", ".q8", ".u16"})
    if (right or q8) and not left:
        raise ValueError("--right-dir/--q8-dir requires --left-dir")
    for name, frames in (("right", right), ("Q8", q8)):
        if frames and set(frames) != set(left):
            raise ValueError(f"{name} filenames must have exactly the same frame stems as left images")
    if left and (not args.rectified or args.intrinsics is None):
        raise ValueError("Offline geometry requires --rectified and --intrinsics from the actual 640x544 rectified camera")
    if args.intrinsics is not None and not args.intrinsics.is_file():
        raise ValueError(f"Intrinsics file does not exist: {args.intrinsics}")
    if args.rectified and not left:
        raise ValueError("--rectified requires actual --left-dir input")
    save_root = args.save_root.resolve() if args.save_root else ROOT / "results/inference_matrix_frames"
    files = {}
    models = {
        "detection": args.det_model or ROOT / "models/x5/mmyolo0828.bin",
        "segmentation": args.seg_model or ROOT / "models/x5/pidnet_0825_3classes.bin",
        "stereo": args.stereo_model or ROOT / "models/x5/r34_crop_stage2_i12_cheb5_max1_tail_int16_352x640_20261012.bin",
    }
    files["inference.json"] = {"backend": "bpu", "original_profile": None, "cpu_profile": None}
    files["detection.json"] = {"enabled": True, "model": reference(models["detection"].resolve(), output),
                               "score_threshold": None, "nms_threshold": None, "nms_top_k": None}
    files["segmentation.json"] = {"enabled": True, "model": reference(models["segmentation"].resolve(), output),
                                  "align_corners": None, "argmax_mode": None}
    files["stereo.json"] = {"model": reference(models["stereo"].resolve(), output),
                            "image_transform": "crop", "input_width": 640, "input_height": 352}
    files["metrics.json"] = {"enabled": True, "write_jsonl": True, "every_n": 1}
    files["ros.json"] = json.loads((ROOT / "features/ros/config.json").read_text(encoding="utf-8-sig"))
    files["camera.json"] = json.loads((ROOT / "camera.json").read_text(encoding="utf-8-sig"))
    if args.camera_sdk:
        files["camera.json"]["sdk_library"] = str(args.camera_sdk.resolve())
    if args.camera_config:
        files["camera.json"]["vendor_config"] = str(args.camera_config.resolve())
    for method in ("original", "crop"):
        keys = sorted(left)
        offline = {
            "left_files": [str(left[k]) for k in keys],
            "right_files": [str(right[k]) for k in keys] if method == "crop" and right else [],
            "q8_files": [str(q8[k]) for k in keys] if method == "original" and q8 else [],
            "format": "nv12", "fps": args.fps, "loop": False, "max_frames": 0,
            "wait_for_subscribers": False,
            "intrinsics_file": str(args.intrinsics.resolve()) if args.intrinsics else None,
            "rectified": bool(left and args.rectified), "calibration_confirmed": bool(left and args.rectified),
            "frame_id": "camera_optical_frame",
            "_help": "Lists pair by index and must describe the same capture. Original needs real 16-bit Q8.8 disparity, not depth or an 8-bit preview. Images must already be rectified 640x544."
        }
        files[f"offline_{method}_input.json"] = offline
        for source in ("camera", "ros", "offline"):
            features = {name: reference(ROOT / "features" / name / "config.json", output) for name in FEATURES}
            for name in ("inference", "detection", "segmentation", "stereo", "metrics", "ros"):
                features[name] = name + ".json"
            features["offline"] = f"offline_{method}_input.json"
            tasks = {
                "det": {"feature": "inference", "algorithm": "det", "enabled": True},
                "seg": {"feature": "inference", "algorithm": "seg", "enabled": True},
                "stereo": {"feature": "inference", "algorithm": "stereo", "enabled": True},
                "publish": {"feature": "publish", "enabled": True},
                "record": {"feature": "record", "enabled": args.save_every > 0},
            }
            sequence = [{"parallel": ["det", "seg", "stereo"]}, "publish"]
            if args.save_every > 0:
                sequence.append("record")
            files[f"{source}_{method}.json"] = {
                "schema_version": 5,
                "input": {"mode": source, "disparity_source": "original_q8" if method == "original" else "raft_stereo"},
                "sdk": {"root": "${SDK_ROOT}", "base_config": None},
                "camera_config": "camera.json",
                "visualization_config": reference(ROOT / "visualization.json", output),
                "features": features, "engine": {"worker_threads": 3}, "tasks": tasks,
                "dag": {"sequence": sequence},
                "output": {"save_root": reference(save_root, output), "save_every_n": args.save_every,
                           "save_ply": False, "publish_every_n": 1, "publish_clouds": False,
                           "publish_maps": False, "publish_mask": True, "publish_disparity": True,
                           "publish_input": False, "render_maps": False},
                "_help": "Current v6 det+seg+disparity only; no mapping/recharge/history. Original imports genuine vendor Q8; crop runs the configured model. Camera/offline use the ROS-linked demo with SDK/file input, not ROS subscriptions."
            }
    output.mkdir(parents=True)
    for name, value in files.items():
        (output / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Configuration directory: {output}")
    print("Cases: camera_original camera_crop ros_original ros_crop offline_original offline_crop")
    if not left:
        print("Offline templates have empty inputs. Supply real images, Q8 and calibration before preparing offline cases.")
    elif not right or not q8:
        print("Only offline cases with a populated matching right/Q8 list can run.")


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--output", type=Path, required=True, help="NEW configuration directory")
    for name in ("left-dir", "right-dir", "q8-dir", "intrinsics", "det-model", "seg-model", "stereo-model",
                 "camera-sdk", "camera-config", "save-root"):
        result.add_argument("--" + name, type=Path)
    result.add_argument("--rectified", action="store_true", help="Confirm the supplied images are rectified 640x544 with matching calibration")
    result.add_argument("--fps", type=float, default=2.0)
    result.add_argument("--save-every", type=int, default=10, help="Save every N completed frames; 0 disables recording")
    return result


if __name__ == "__main__":
    arguments = parser()
    try:
        generate(arguments.parse_args())
    except (OSError, ValueError) as error:
        arguments.error(str(error))
