"""Exercise Docker packaging orchestration without a daemon or target libraries.

The exported Bash function replaces only the Docker CLI. Real host tar/sha256
still run so the tests check archive contents and source-cache exclusions.
"""
from __future__ import annotations

import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import unittest


PROJECT = Path(__file__).resolve().parents[1]
SCRIPT = PROJECT / "tools" / "build_inference_matrix_in_container.sh"


def find_bash() -> str | None:
    candidates = [os.environ.get("MATRIX_TEST_BASH"), shutil.which("bash")]
    if os.name == "nt":
        candidates += [r"C:\Program Files\Git\bin\bash.exe",
                       r"C:\Program Files\Git\usr\bin\bash.exe"]
    for candidate in candidates:
        if not candidate or not Path(candidate).is_file():
            continue
        try:
            probe = subprocess.run([candidate, "-c", "printf ready"],
                                   capture_output=True, text=True, timeout=10)
        except (OSError, subprocess.TimeoutExpired):
            continue
        if probe.returncode == 0 and probe.stdout == "ready":
            return candidate
    return None


BASH = find_bash()


def shell_path(path: Path) -> str:
    value = path.resolve().as_posix()
    if os.name == "nt":
        return f"/{value[0].lower()}{value[2:]}"
    return value


MOCK_DOCKER = r'''
# Git Bash launched from Windows does not always prepend its own coreutils.
export PATH="/usr/bin:$PATH"
docker() {
  printf '%s\037' "$@" >> "$MATRIX_TEST_CALLS"
  printf '\n' >> "$MATRIX_TEST_CALLS"
  case "${1:-}" in
    inspect)
      [[ ${MATRIX_TEST_INSPECT_EXIT:-0} == 0 ]] || return "$MATRIX_TEST_INSPECT_EXIT"
      if [[ "$*" == *State.Running* ]]; then
        printf '%s\n' "${MATRIX_TEST_RUNNING:-true}"
      else
        printf '[]\n'
      fi
      ;;
    start) printf '%s\n' "${@: -1}" ;;
    exec)
      if [[ "$*" == *mktemp* ]]; then
        printf '%s\n' "$MATRIX_TEST_REMOTE"
      elif [[ "$*" == *'bash -s'* ]]; then
        cat > "$MATRIX_TEST_BUILD_SCRIPT"
        printf 'mock target build, no inference or hardware execution\n'
        return "${MATRIX_TEST_BUILD_EXIT:-0}"
      elif [[ "$*" == *tar* ]]; then
        cat > "$MATRIX_TEST_UPLOAD_TAR"
        tar -tf "$MATRIX_TEST_UPLOAD_TAR" > "$MATRIX_TEST_UPLOAD_LIST"
      fi
      ;;
    cp)
      local remote_path=${2#*:} destination=$3 item
      item=${remote_path##*/}
      case "$item" in
        backend_sdk_x5|perception_ros_so_demo_x5)
          mkdir -p -- "$destination"
          cp -R -- "$MATRIX_TEST_ASSETS/$item" "$destination/"
          ;;
        inference_matrix_docker_build.json)
          if [[ -d $destination || $destination == */ ]]; then
            mkdir -p -- "$destination"
            cp -- "$MATRIX_TEST_ASSETS/inference_matrix_docker_build.json" "$destination/$item"
          else
            mkdir -p -- "$(dirname -- "$destination")"
            cp -- "$MATRIX_TEST_ASSETS/inference_matrix_docker_build.json" "$destination"
          fi
          ;;
        *) printf 'Unsupported mock docker cp source: %s\n' "$remote_path" >&2; return 91 ;;
      esac
      ;;
    *) printf 'Unsupported mock docker command: %s\n' "$*" >&2; return 92 ;;
  esac
}
'''


@unittest.skipUnless(BASH, "No working Bash; run on Ubuntu or Git Bash")
class MatrixDockerEntrypointTest(unittest.TestCase):
    def setUp(self) -> None:
        # Keep Bash-created fixtures inside the writable workspace; Windows
        # TEMP may use a short-name alias that MSYS canonicalizes differently.
        temp_parent = PROJECT / "tmp"
        temp_parent.mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(prefix="matrix docker contract ", dir=temp_parent)
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.root = self.base / "current project with spaces"
        (self.root / "tools").mkdir(parents=True)
        self.assertTrue(SCRIPT.is_file(), "The Docker build wrapper must exist")
        shutil.copyfile(SCRIPT, self.root / "tools" / SCRIPT.name)
        self.write("tools/prepare_inference_matrix_env.sh", "# inert source fixture\n")
        self.write("tools/build_inference_matrix_x5.sh", "# inert build fixture\n")
        self.write("sdk/include/contract.h", "/* packaged current source */\n")
        self.write("validation/keep.txt", "current validation source\n")
        # No features/benchmark6/runtime/install directory exists in this fixture.
        self.excluded = ["build/cache.obj", "install/stale.so", "dist/old.zip",
                         "results/old.json", "tmp/private.txt", ".git/config",
                         "windows/deps/onnxruntime/stale.dll"]
        for name in self.excluded:
            self.write(name, "must not be uploaded or delivered\n")
        self.assets = self.base / "mock install assets"
        binaries = ["backend_sdk_x5/lib/libperception_sdk.so",
                    "perception_ros_so_demo_x5/bin/perception_config",
                    "perception_ros_so_demo_x5/lib/perception_ros_so_demo/demo",
                    "perception_ros_so_demo_x5/lib/perception_ros_so_demo/raw_camera"]
        for filename in binaries:
            target = self.assets / filename
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text("mock ARM artifact; not executable test evidence\n", encoding="utf-8")
        (self.assets / "inference_matrix_docker_build.json").write_text(
            '{"mock":true,"arm_binaries_verified":4}\n', encoding="utf-8")
        self.bootstrap = self.base / "mock docker bash env.sh"
        self.bootstrap.write_text(MOCK_DOCKER, encoding="utf-8")
        self.calls_file = self.base / "docker calls.txt"
        self.out = self.base / "new package output"
        self.env = dict(os.environ)
        if os.name == "nt":
            self.env = {name.upper(): value for name, value in self.env.items()}
        self.env.update({
            "BASH_ENV": shell_path(self.bootstrap),
            "MATRIX_TEST_CALLS": shell_path(self.calls_file),
            "MATRIX_TEST_ASSETS": shell_path(self.assets),
            "MATRIX_TEST_BUILD_SCRIPT": shell_path(self.base / "remote build.sh"),
            "MATRIX_TEST_UPLOAD_TAR": shell_path(self.base / "source upload.tar"),
            "MATRIX_TEST_UPLOAD_LIST": shell_path(self.base / "source upload.txt"),
            "MATRIX_TEST_REMOTE": "/tmp/perception-v6-matrix-CONTRACT01",
            # Prevent MSYS from changing Docker container paths into drive paths.
            "MSYS_NO_PATHCONV": "1", "MSYS2_ARG_CONV_EXCL": "*",
        })
        for name in ("X5_SYSROOT", "TARGET_ROS_PREFIX", "PSDK_ROOT", "INSTALL_ROOT",
                     "SDK_BUILD_ROOT", "ROS_BUILD_ROOT", "COMPARE_BUILD_ROOT"):
            self.env.pop(name, None)

    def write(self, filename: str, value: str) -> None:
        target = self.root / filename
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(value, encoding="utf-8")

    def invoke(self, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [BASH, shell_path(self.root / "tools" / SCRIPT.name), *args],
            cwd=self.root, env=self.env, capture_output=True, text=True,
            encoding="utf-8", errors="replace", timeout=30,
        )

    def calls(self) -> list[list[str]]:
        if not self.calls_file.exists():
            return []
        return [line.rstrip("\x1f").split("\x1f")
                for line in self.calls_file.read_text(encoding="utf-8").splitlines()]

    def test_help_does_not_call_docker_or_require_target_dependencies(self) -> None:
        result = self.invoke("--help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--out", result.stdout)
        self.assertEqual(self.calls(), [])

    def test_invalid_arguments_fail_before_docker(self) -> None:
        for args in (("--unknown",), ("--out",),
                     ("--jobs", "0", "--out", shell_path(self.out)),
                     ("--jobs", "bad", "--out", shell_path(self.out)),
                     ("--sysroot", "relative/sysroot", "--out", shell_path(self.out))):
            with self.subTest(args=args):
                result = self.invoke(*args)
                self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertEqual(self.calls(), [])
                self.assertFalse(self.out.exists())

    def test_new_output_is_required_and_existing_files_survive(self) -> None:
        self.out.mkdir()
        marker = self.out / "keep.txt"
        marker.write_text("preserve", encoding="utf-8")
        result = self.invoke("--out", shell_path(self.out))
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(marker.read_text(), "preserve")
        self.assertEqual(self.calls(), [])

    def test_docker_inspection_failure_creates_no_output_or_archive(self) -> None:
        self.env["MATRIX_TEST_INSPECT_EXIT"] = "9"
        result = self.invoke("--out", shell_path(self.out))
        self.assertEqual(result.returncode, 9, result.stdout + result.stderr)
        self.assertFalse(self.out.exists())
        self.assertEqual(len(self.calls()), 1)
        self.assertEqual(self.calls()[0][0], "inspect")

    def test_success_dispatches_current_source_and_packages_without_host_benchmark(self) -> None:
        self.env["MATRIX_TEST_RUNNING"] = "false"
        result = self.invoke(
            "--container", "mock-existing-container",
            "--sysroot", "/sysroot_docker/usr_x5",
            "--dependency-prefix", "/target dependencies/usr",
            "--stereo-horizon-root", "/target horizon/usr",
            "--target-ros-prefix", "/target ros;/other ros",
            "--eigen-dir", "/target eigen/share/eigen3/cmake",
            "--jobs", "3", "--out", shell_path(self.out),
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        calls = self.calls()
        self.assertTrue(any(call[0] == "inspect" for call in calls))
        self.assertTrue(any(call[0] == "start" for call in calls))
        build = self.base / "remote build.sh"
        self.assertTrue(build.is_file(), calls)
        script = build.read_text(encoding="utf-8")
        self.assertIn("prepare_inference_matrix_env.sh", script)
        self.assertIn("build_inference_matrix_x5.sh", script)
        joined_calls = "\n".join(" ".join(call) for call in calls)
        self.assertIn("/sysroot_docker/usr_x5", joined_calls + script)
        self.assertIn("/target dependencies/usr", joined_calls + script)
        self.assertIn("/target ros;/other ros", joined_calls + script)
        self.assertFalse((self.root / "features/benchmark6/runtime/install").exists())
        with tarfile.open(self.base / "source upload.tar") as uploaded:
            uploaded_names = uploaded.getnames()
        for filename in self.excluded:
            self.assertFalse(any(name.endswith("/" + filename) or name == filename
                                 or name == "./" + filename for name in uploaded_names), filename)
        self.assertTrue(any(name.endswith("sdk/include/contract.h") for name in uploaded_names))
        archive = self.out / "perception-v6-matrix-x5.tar.gz"
        self.assertTrue(archive.is_file(), result.stdout + result.stderr)
        digest = hashlib.sha256(archive.read_bytes()).hexdigest()
        self.assertIn(digest, (self.out / (archive.name + ".sha256")).read_text())
        self.assertTrue((self.out / "environment.txt").is_file())
        self.assertTrue((self.out / "build.log").is_file())
        self.assertTrue((self.out / "docker_build.json").is_file())
        with tarfile.open(archive) as delivered:
            names = delivered.getnames()
        for filename in ("sdk/include/contract.h",
                         "install/backend_sdk_x5/lib/libperception_sdk.so",
                         "install/perception_ros_so_demo_x5/bin/perception_config",
                         "install/perception_ros_so_demo_x5/lib/perception_ros_so_demo/demo",
                         "install/perception_ros_so_demo_x5/lib/perception_ros_so_demo/raw_camera"):
            self.assertTrue(any(name.endswith("/" + filename) for name in names), filename)
        for filename in self.excluded:
            self.assertFalse(any(name.endswith("/" + filename) for name in names), filename)

    def test_container_build_failure_preserves_status_logs_and_no_success_archive(self) -> None:
        self.env["MATRIX_TEST_BUILD_EXIT"] = "7"
        result = self.invoke("--out", shell_path(self.out))
        self.assertEqual(result.returncode, 7, result.stdout + result.stderr)
        self.assertTrue((self.out / "build.log").is_file())
        self.assertTrue((self.out / "environment.txt").is_file())
        self.assertFalse((self.out / "perception-v6-matrix-x5.tar.gz").exists())
        self.assertFalse((self.out / "perception-v6-matrix-x5.tar.gz.sha256").exists())
        self.assertFalse(any(call[0] == "cp" for call in self.calls()), self.calls())
        self.assertIn(self.env["MATRIX_TEST_REMOTE"], result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
