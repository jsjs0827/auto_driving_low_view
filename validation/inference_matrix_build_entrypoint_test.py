"""Test the matrix build wrapper without compiling or touching a sysroot."""
from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


PROJECT = Path(__file__).resolve().parents[1]
SCRIPT = PROJECT / "tools" / "build_inference_matrix_x5.sh"


def find_bash() -> str | None:
    candidates = [shutil.which("bash")]
    if os.name == "nt":
        candidates.extend([
            r"C:\Program Files\Git\bin\bash.exe",
            r"C:\Program Files\Git\usr\bin\bash.exe",
        ])
    for candidate in candidates:
        if not candidate or not Path(candidate).is_file():
            continue
        try:
            probe = subprocess.run(
                [candidate, "-c", "printf ready"], capture_output=True,
                text=True, timeout=10,
            )
        except (OSError, subprocess.TimeoutExpired):
            continue
        if probe.returncode == 0 and probe.stdout == "ready":
            return candidate
    return None


BASH = find_bash()


@unittest.skipUnless(BASH, "No working Bash available; run this test on Ubuntu or Git Bash")
class MatrixBuildEntrypointTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory(prefix="v6-matrix-build-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        tools = self.root / "tools"
        tools.mkdir()
        shutil.copyfile(SCRIPT, tools / SCRIPT.name)
        (tools / "build_x5_v8.sh").write_text(
            '#!/usr/bin/env bash\n'
            'printf "%s\\n" "$@" > "$MATRIX_TEST_ARGV"\n'
            'printf "%s\\n" "$PSDK_ROOT" "$INSTALL_ROOT" > "$MATRIX_TEST_ROOTS"\n'
            'exit "${MATRIX_TEST_BUILD_EXIT:-0}"\n', encoding="utf-8",
        )
        self.env = os.environ.copy()
        for name in ("X5_SYSROOT", "TARGET_ROS_PREFIX", "PSDK_ROOT", "INSTALL_ROOT"):
            self.env.pop(name, None)
        self.env["MATRIX_TEST_ARGV"] = (self.root / "argv.txt").as_posix()
        self.env["MATRIX_TEST_ROOTS"] = (self.root / "roots.txt").as_posix()

    def shell_path(self, path: Path) -> str:
        # Git Bash accepts native drive paths as CLI arguments, while an env var
        # must use its POSIX form for the wrapper's absolute-path check.
        value = path.as_posix()
        if os.name == "nt":
            value = f"/{value[0].lower()}{value[2:]}"
        return value

    def invoke(self, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [BASH, (self.root / "tools" / SCRIPT.name).as_posix(), *args],
            env=self.env, capture_output=True, text=True, timeout=15,
        )

    def valid_environment(self) -> None:
        for name, directory in (("X5_SYSROOT", "sysroot with spaces"),
                                ("TARGET_ROS_PREFIX", "target ros")):
            path = self.root / directory
            path.mkdir()
            self.env[name] = self.shell_path(path)

    def assert_build_not_called(self) -> None:
        self.assertFalse((self.root / "argv.txt").exists())

    def test_help_without_dependencies(self) -> None:
        result = self.invoke("--help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("TARGET_ROS_PREFIX", result.stdout)
        self.assertIn("--with-ros", result.stdout)
        self.assert_build_not_called()

    def test_unknown_option_is_rejected(self) -> None:
        self.assertEqual(self.invoke("--with-ros").returncode, 2)
        self.assert_build_not_called()

    def test_missing_and_invalid_dependencies_are_rejected(self) -> None:
        result = self.invoke()
        self.assertEqual(result.returncode, 2)
        self.assertIn("X5_SYSROOT", result.stderr)
        self.valid_environment()
        self.env["TARGET_ROS_PREFIX"] += ";/missing/target/ros"
        self.assertEqual(self.invoke().returncode, 2)
        self.env["TARGET_ROS_PREFIX"] = ";/missing"
        self.assertEqual(self.invoke().returncode, 2)
        self.assert_build_not_called()

    def test_build_is_current_entrypoint_with_ros_and_prints_dependencies(self) -> None:
        self.valid_environment()
        second_ros = self.root / "second ros prefix"
        second_ros.mkdir()
        self.env["TARGET_ROS_PREFIX"] += ";" + self.shell_path(second_ros)
        before = set(self.root.iterdir())
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual((self.root / "argv.txt").read_text().splitlines(), ["--with-ros"])
        self.assertIn("SDK_ROOT=", result.stdout)
        self.assertIn("INSTALL_ROOT=", result.stdout)
        for source in ("camera", "ros", "offline"):
            for method in ("original_q8", "stereo_crop"):
                self.assertIn(f"{source}/{method}", result.stdout)
        for path in before:
            if path.is_dir() and path.name != "tools":
                self.assertEqual(list(path.iterdir()), [], "Wrapper modified a dependency prefix")

    def test_custom_output_roots_and_build_failure_are_preserved(self) -> None:
        self.valid_environment()
        self.env["PSDK_ROOT"] = self.shell_path(self.root / "custom sdk")
        self.env["INSTALL_ROOT"] = self.shell_path(self.root / "custom apps")
        self.env["MATRIX_TEST_BUILD_EXIT"] = "7"
        result = self.invoke()
        self.assertEqual(result.returncode, 7, result.stderr)
        self.assertEqual((self.root / "roots.txt").read_text().splitlines(),
                         [self.env["PSDK_ROOT"], self.env["INSTALL_ROOT"]])
        self.assertNotIn("Build complete", result.stdout)


if __name__ == "__main__":
    unittest.main()
