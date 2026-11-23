"""Configuration/data-contract tests; no board, camera or trained model execution."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def module(name, file):
    spec = importlib.util.spec_from_file_location(name, file)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


GEN = module("matrix_config", ROOT / "tools/configure_inference_matrix.py")
SUMMARY = module("matrix_summary", ROOT / "tools/summarize_inference_matrix.py")
CHECK = module("matrix_check", ROOT / "tools/check_inference_matrix_run.py")


class MatrixTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="v6 matrix ")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def generate(self, *extra):
        output = self.root / ("configs " + str(len(list(self.root.glob("configs*")))))
        args = GEN.parser().parse_args(["--output", str(output), *map(str, extra)])
        GEN.generate(args)
        return output

    def dataset(self):
        for name in ("left", "right", "q8"):
            (self.root / name).mkdir()
        ppm = b"P6\n640 544\n255\n" + bytes((64, 128, 192)) * (640 * 544)
        for eye in ("left", "right"):
            (self.root / eye / "frame000001.ppm").write_bytes(ppm)
        raw = self.root / "q8/frame000001.u16"
        raw.write_bytes(b"\x00\x01" * (640 * 544))  # Synthetic unit-test Q8, never benchmark evidence.
        (Path(str(raw) + ".json")).write_text(json.dumps({"width": 640, "height": 544,
            "encoding": "16UC1", "byte_order": "little", "value_format": "q8.8"}))
        intrinsics = self.root / "intrinsics.json"
        intrinsics.write_text(json.dumps({"width": 640, "height": 544,
            "fx": 500, "fy": 500, "cx": 320, "cy": 272, "bxf": 50000}))
        return ["--left-dir", self.root / "left", "--right-dir", self.root / "right",
                "--q8-dir", self.root / "q8", "--intrinsics", intrinsics, "--rectified"]

    def test_all_six_profiles_keep_det_seg_and_only_requested_disparity(self):
        output = self.generate()
        for source in ("camera", "ros", "offline"):
            for method in ("original", "crop"):
                config = json.loads((output / f"{source}_{method}.json").read_text(encoding="utf-8"))
                self.assertEqual(config["input"], {"mode": source, "disparity_source":
                    "original_q8" if method == "original" else "raft_stereo"})
                self.assertEqual(config["dag"]["sequence"][0], {"parallel": ["det", "seg", "stereo"]})
                self.assertEqual(set(config["tasks"]), {"det", "seg", "stereo", "publish", "record"})
                self.assertEqual(config["schema_version"], 5)
                self.assertFalse(config["output"]["publish_clouds"])

    def test_offline_pairing_and_camera_file_overrides(self):
        sdk = self.root / "camera fixture.so"
        sdk.write_bytes(b"prepare-only fixture; not a camera library")
        output = self.generate(*self.dataset(), "--camera-sdk", sdk)
        original = json.loads((output / "offline_original_input.json").read_text(encoding="utf-8"))
        crop = json.loads((output / "offline_crop_input.json").read_text(encoding="utf-8"))
        self.assertEqual(original["right_files"], [])
        self.assertEqual(crop["q8_files"], [])
        self.assertEqual(len(original["q8_files"]), 1)
        self.assertEqual(len(crop["right_files"]), 1)
        self.assertTrue(original["calibration_confirmed"])
        self.assertEqual(json.loads((output / "camera.json").read_text(encoding="utf-8"))["sdk_library"], str(sdk.resolve()))

    def test_mismatched_stems_and_unconfirmed_rectification_are_rejected(self):
        arguments = self.dataset()
        with self.assertRaisesRegex(ValueError, "rectified"):
            self.generate(*arguments[:-1])
        (self.root / "right/frame000001.ppm").rename(self.root / "right/other.ppm")
        with self.assertRaisesRegex(ValueError, "same frame stems"):
            self.generate(*arguments)

    def test_no_overwrite_and_invalid_sampling_options(self):
        output = self.generate()
        with self.assertRaisesRegex(ValueError, "NEW"):
            GEN.generate(GEN.parser().parse_args(["--output", str(output)]))
        with self.assertRaisesRegex(ValueError, "fps"):
            self.generate("--fps", "nan")
        with self.assertRaisesRegex(ValueError, "nonnegative"):
            self.generate("--save-every", -1)

    def test_summary_preserves_missing_old_stereo_timing_and_warmup(self):
        rows = [{"input_mode": "ros", "disparity_source": "original_q8",
                 "compute_ms": n, "stereo_ms": None, "completed_fps_since_ready": 10}
                for n in (100, 10, 20)]
        (self.root / "metrics.jsonl").write_text("\n".join(map(json.dumps, rows)))
        result = SUMMARY.summarize(self.root, 1)
        self.assertEqual(result["metrics"]["compute_ms"]["mean"], 15)
        self.assertIsNone(result["metrics"]["stereo_ms"])
        self.assertIsNone(result["bpu_percent"])
        self.assertEqual(result["measurement_rows"], 2)
        self.assertIn("excluded", result["cpu_scope"])

    def evidence(self, count=10, source="camera", expected=None):
        run = self.root / ("run " + str(len(list(self.root.glob("run*")))))
        run.mkdir()
        params = {"save_every_n": 0}
        if expected is not None:
            params.update(offline_files=[f"frame{i}.png" for i in range(expected)], offline_loop=False)
        (run / "resolved.json").write_text(json.dumps({"parameters": params}), encoding="utf-8")
        rows = [{"frame_id": i + 1, "timestamp_ns": (i + 1) * 1000000,
                 "input_mode": source, "disparity_source": "original_q8",
                 "task_trace": {"tasks": [{"task": task, "state": "succeeded"}
                                          for task in ("det", "seg", "stereo")]}}
                for i in range(count)]
        (run / "metrics.jsonl").write_text("\n".join(map(json.dumps, rows)), encoding="utf-8")
        (run / "console.log").write_text("SO ready\n", encoding="utf-8")
        return run

    def test_checker_rejects_early_exit_empty_or_partial_offline_runs(self):
        run = self.evidence()
        self.assertFalse(CHECK.check(run, "camera", "original", 0)["passed"])
        run = self.evidence(count=0)
        self.assertFalse(CHECK.check(run, "camera", "original", 124)["passed"])
        run = self.evidence(count=1, source="offline", expected=2)
        self.assertFalse(CHECK.check(run, "offline", "original", 124)["passed"])

    def test_checker_requires_successful_branches_and_no_runtime_errors(self):
        run = self.evidence()
        rows = (run / "metrics.jsonl").read_text(encoding="utf-8").replace('"succeeded"', '"skipped"', 1)
        (run / "metrics.jsonl").write_text(rows, encoding="utf-8")
        self.assertFalse(CHECK.check(run, "camera", "original", 124)["passed"])
        run = self.evidence()
        (run / "console.log").write_text("[ERROR] failed to save\n", encoding="utf-8")
        self.assertFalse(CHECK.check(run, "camera", "original", 124)["passed"])

    def test_checker_accepts_completed_live_and_all_offline_evidence(self):
        run = self.evidence()
        self.assertTrue(CHECK.check(run, "camera", "original", 124)["passed"])
        run = self.evidence(count=2, source="offline", expected=2)
        result = CHECK.check(run, "offline", "original", 124)
        self.assertTrue(result["passed"])
        self.assertEqual(result["expected_offline_frames"], 2)

    def test_checker_rejects_missing_saved_payloads(self):
        run = self.evidence()
        (run / "resolved.json").write_text(json.dumps({"parameters": {
            "save_every_n": 1, "save_root": str(run / "missing frames")}}), encoding="utf-8")
        self.assertFalse(CHECK.check(run, "camera", "original", 124)["passed"])

    @unittest.skipUnless(os.environ.get("MATRIX_CONFIG_EXE"), "Set MATRIX_CONFIG_EXE to test the real C++ configuration tool")
    def test_six_profiles_prepare_through_real_cpp_configurator(self):
        sdk = self.root / "camera fixture.so"
        vendor = self.root / "camera fixture.cfg"
        sdk.write_text("prepare fixture only")
        vendor.write_text("prepare fixture only")
        output = self.generate(*self.dataset(), "--camera-sdk", sdk, "--camera-config", vendor)
        env = {key.upper(): value for key, value in os.environ.items()} if os.name == "nt" else os.environ.copy()
        for source in ("camera", "ros", "offline"):
            for method in ("original", "crop"):
                run = self.root / f"prepared_{source}_{method}"
                process = subprocess.run([os.environ["MATRIX_CONFIG_EXE"], "run", "--config",
                    str(output / f"{source}_{method}.json"), "--run-dir", str(run), "--prepare-only"],
                    env=env, capture_output=True, text=True, encoding="utf-8", errors="replace")
                self.assertEqual(process.returncode, 0, process.stdout + process.stderr)
                resolved = json.loads((run / "resolved.json").read_text(encoding="utf-8"))
                self.assertFalse(json.loads((run / "effective_full.json").read_text(encoding="utf-8"))["mapping"]["enabled"])
                self.assertEqual(resolved["parameters"]["input_mode"], source)
                params = resolved["parameters"]
                self.assertEqual(params["disparity_source"], "original_q8" if method == "original" else "raft_stereo")
                if method == "crop":
                    self.assertEqual(params["stereo_input_mode"], "crop")
                    self.assertEqual(params["stereo_input_height"], 352)
                if source == "offline" and method == "original":
                    self.assertEqual(len(params["offline_q8_files"]), 1)


if __name__ == "__main__":
    unittest.main()
