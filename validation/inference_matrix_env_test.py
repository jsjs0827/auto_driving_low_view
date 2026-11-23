"""Exercise read-only container environment discovery with mock ARM tools."""
from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools" / "prepare_inference_matrix_env.sh"
SPEC = importlib.util.spec_from_file_location(
    "matrix_entrypoint_test", ROOT / "validation" / "inference_matrix_build_entrypoint_test.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
BASH = MODULE.find_bash()


@unittest.skipUnless(BASH, "No working Bash available; run on Ubuntu or Git Bash")
class MatrixEnvironmentTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="matrix env ")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.sysroot = self.root / "sysroot"
        self.prefix = self.sysroot / "usr_x5"
        self.tools = self.root / "mock bin"
        self.tools.mkdir()
        (self.sysroot / "usr/include").mkdir(parents=True)
        self.touch(self.prefix / "include/dnn/hb_dnn.h")
        self.touch(self.prefix / "lib/libdnn.so")
        self.touch(self.prefix / "lib/aarch64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake")
        self.touch(self.prefix / "lib/aarch64-linux-gnu/cmake/pcl/PCLConfig.cmake")
        self.eigen = self.prefix / "share/eigen3/cmake"
        self.touch(self.eigen / "Eigen3Config.cmake")
        self.ros = self.sysroot / "opt/ros/humble"
        self.make_ros(self.ros)
        self.command("readelf", 'case "$*" in *"x86"*) printf "Machine: Advanced Micro Devices X86-64\\n";; *) printf "Machine: AArch64\\n";; esac\n')
        self.command("cross-c++", 'printf "aarch64-linux-gnu\\n"\n')
        self.env = os.environ.copy()
        for name in ("TARGET_ROS_PREFIX", "TARGET_EIGEN_DIR", "TARGET_OPENCV_DIR", "TARGET_PCL_DIR"):
            self.env.pop(name, None)
        self.env.update({
            "X5_SYSROOT": self.shell_path(self.sysroot),
            "TARGET_SYSTEM_PREFIX": self.shell_path(self.prefix),
            "STEREO_HORIZON_ROOT": self.shell_path(self.prefix),
            "X5_CXX_COMPILER": self.shell_path(self.tools / "cross-c++"),
            "PATH": self.shell_path(self.tools) + os.pathsep + self.env.get("PATH", ""),
        })
        # Prevent unrelated host installations from affecting auto-discovery.
        # The mock find forwards sysroot scans and returns nothing for /opt,/app.
        self.env["MATRIX_MOCK_SYSROOT"] = self.shell_path(self.sysroot)
        real_find = shutil.which("find")
        if not real_find:
            self.skipTest("A find executable is required for the filesystem fixtures")
        self.env["MATRIX_REAL_FIND"] = self.shell_path(Path(real_find))
        self.command("find", 'case "$2" in "$MATRIX_MOCK_SYSROOT") exec "$MATRIX_REAL_FIND" "$@";; *) exit 0;; esac\n')

    @staticmethod
    def touch(path):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("fixture\n", encoding="utf-8")

    def command(self, name, body):
        path = self.tools / name
        path.write_text("#!/usr/bin/env bash\n" + body, encoding="utf-8")
        path.chmod(0o755)

    def shell_path(self, path):
        value = path.resolve().as_posix()
        return f"/{value[0].lower()}{value[2:]}" if os.name == "nt" else value

    def make_ros(self, prefix, packages=("rclcpp", "nav_msgs", "sensor_msgs", "std_msgs", "ament_cmake")):
        for name in packages:
            self.touch(prefix / f"share/{name}/cmake/{name}Config.cmake")
        if "rclcpp" in packages:
            self.touch(prefix / "lib/librclcpp.so")

    def snapshot(self):
        return {p.relative_to(self.sysroot).as_posix(): p.read_bytes()
                for p in self.sysroot.rglob("*") if p.is_file()}

    def invoke(self):
        # Source without errexit: even a failed precheck must return to the host.
        return subprocess.run(
            [BASH, "-c", 'source "$1"; s=$?; printf "HOST_ALIVE=%s\\n" "$s"; '
             'if [ "$s" = 0 ]; then printf "EXPORTED=%s|%s\\n" "$TARGET_ROS_PREFIX" "$TARGET_EIGEN_DIR"; fi; exit "$s"',
             "matrix-env-test", SCRIPT.as_posix()],
            env=self.env, capture_output=True, encoding="utf-8", errors="replace", timeout=20)

    def test_auto_unique_prefix_and_default_eigen_leave_sysroot_unchanged(self):
        before = self.snapshot()
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(self.shell_path(self.ros), result.stdout)
        self.assertIn(self.shell_path(self.eigen), result.stdout)
        self.assertIn("HOST_ALIVE=0", result.stdout)
        self.assertEqual(self.snapshot(), before)

    def test_missing_ros_returns_to_host(self):
        self.env["TARGET_ROS_PREFIX"] = self.shell_path(self.root / "missing ros")
        result = self.invoke()
        self.assertEqual(result.returncode, 2)
        self.assertIn("HOST_ALIVE=2", result.stdout)
        self.assertIn("Invalid explicit", result.stderr)

    def test_no_arm_ros_has_clear_error(self):
        (self.ros / "lib/librclcpp.so").unlink()
        result = self.invoke()
        self.assertEqual(result.returncode, 2)
        self.assertIn("No AArch64 ROS2", result.stderr)

    def test_auto_discovery_ignores_x86_and_deduplicates_physical_prefix(self):
        self.make_ros(self.sysroot / "x86 host ros")
        try:
            (self.sysroot / "ros alias").symlink_to(self.ros, target_is_directory=True)
        except OSError as error:
            self.skipTest(f"Directory symlinks unavailable: {error}")
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("TARGET_ROS_PREFIX=" + self.shell_path(self.ros), result.stdout)

    def test_multiple_arm_prefixes_require_explicit_selection(self):
        other = self.sysroot / "second ros"
        self.make_ros(other)
        result = self.invoke()
        self.assertEqual(result.returncode, 2)
        self.assertIn("Multiple physical", result.stderr)
        self.assertIn(self.shell_path(other), result.stderr)
        self.env["TARGET_ROS_PREFIX"] = self.shell_path(self.ros)
        self.assertEqual(self.invoke().returncode, 0)

    def test_explicit_multiple_prefixes_can_supply_split_packages(self):
        (self.ros / "share/nav_msgs/cmake/nav_msgsConfig.cmake").unlink()
        extra = self.sysroot / "messages"
        self.make_ros(extra, ("nav_msgs",))
        self.env["TARGET_ROS_PREFIX"] = self.shell_path(self.ros) + ";" + self.shell_path(extra)
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(self.env["TARGET_ROS_PREFIX"], result.stdout)

    def test_explicit_x86_ros_is_rejected(self):
        host_ros = self.sysroot / "x86 ros"
        self.make_ros(host_ros)
        self.env["TARGET_ROS_PREFIX"] = self.shell_path(host_ros)
        result = self.invoke()
        self.assertEqual(result.returncode, 2)
        self.assertIn("no valid AArch64", result.stderr)

    def test_missing_nav_msgs_reports_package(self):
        (self.ros / "share/nav_msgs/cmake/nav_msgsConfig.cmake").unlink()
        result = self.invoke()
        self.assertEqual(result.returncode, 2)
        self.assertIn("Missing target nav_msgs", result.stderr)

    def test_eigen_fallback_and_ambiguity(self):
        (self.eigen / "Eigen3Config.cmake").unlink()
        other = self.sysroot / "other eigen/cmake"
        self.touch(other / "Eigen3Config.cmake")
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(self.shell_path(other), result.stdout)
        second = self.sysroot / "second eigen/cmake"
        self.touch(second / "Eigen3Config.cmake")
        result = self.invoke()
        self.assertEqual(result.returncode, 2)
        self.assertIn("Multiple physical Eigen", result.stderr)
        self.env["TARGET_EIGEN_DIR"] = self.shell_path(other)
        self.assertEqual(self.invoke().returncode, 0)

    def test_missing_eigen_reports_explicit_override(self):
        (self.eigen / "Eigen3Config.cmake").unlink()
        result = self.invoke()
        self.assertEqual(result.returncode, 2)
        self.assertIn("Set TARGET_EIGEN_DIR", result.stderr)

    def test_invalid_explicit_eigen_does_not_silently_fall_back(self):
        self.env["TARGET_EIGEN_DIR"] = self.shell_path(self.root / "missing eigen")
        result = self.invoke()
        self.assertEqual(result.returncode, 2)
        self.assertIn("Invalid TARGET_EIGEN_DIR", result.stderr)

    def test_invalid_compiler_and_base_dependencies_fail_early(self):
        self.command("cross-c++", 'printf "x86_64-linux-gnu\\n"\n')
        result = self.invoke()
        self.assertEqual(result.returncode, 2)
        self.assertIn("Compiler must target", result.stderr)
        self.command("cross-c++", 'printf "aarch64-linux-gnu\\n"\n')
        (self.prefix / "include/dnn/hb_dnn.h").unlink()
        result = self.invoke()
        self.assertEqual(result.returncode, 2)
        self.assertIn("Missing Horizon header", result.stderr)


if __name__ == "__main__":
    unittest.main()
