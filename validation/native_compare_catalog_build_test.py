"""Compile real graph/catalog consumers using the native targets' CMake usage.

Only OpenCV package discovery is stubbed: this does not compile the POSIX/OpenCV
comparison implementation or an ARM/ROS application. It compiles and links the
real catalog and configuration tool, and compiles product_graph.hpp using the
comparison implementation/contract targets' resolved include directories.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
PROBE = r'''
#include "product_graph.hpp"
#include <iostream>
int main() {
  const auto& catalog = perception::algorithms::catalog();
  if (catalog.empty()) return 1;
  for (const auto& entry : catalog) {
    if (!perception::algorithms::find(entry.first)) return 2;
    if (r33::full::productStageKey(entry.first) != entry.first) return 3;
    if (r33::full::productFeatureName(entry.first) != entry.second.feature) return 4;
  }
  std::cout << "Real graph/catalog compile, link and lookup passed\n";
}
'''


def host_environment():
    environment = dict(os.environ)
    if os.name != "nt":
        return environment
    environment = {key.upper(): value for key, value in environment.items()}
    installation = Path(environment.get("VSINSTALLDIR", r"C:\Program Files\Microsoft Visual Studio\2022\Community"))
    vcvars = installation / "VC/Auxiliary/Build/vcvars64.bat"
    setup = subprocess.run(f'cmd.exe /d /s /c ""{vcvars}" >nul && set"',
                           env=environment, capture_output=True, text=True, encoding="mbcs")
    if setup.returncode:
        raise RuntimeError("MSVC environment setup failed: " + setup.stderr)
    for line in setup.stdout.splitlines():
        key, separator, value = line.partition("=")
        if separator and key:
            environment[key.upper()] = value
    environment["VSLANG"] = "1033"
    return environment


def run(command, log, environment):
    log.write("\n$ " + subprocess.list2cmdline([str(v) for v in command]) + "\n")
    log.flush()
    result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, text=True, env=environment)
    if result.returncode:
        raise RuntimeError(f"Command failed ({result.returncode}); see {log.name}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="NEW evidence directory")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Choose a NEW evidence directory")
    cmake = os.environ.get("MATRIX_CMAKE") or shutil.which("cmake")
    if not cmake:
        parser.error("CMake is required; set MATRIX_CMAKE when it is outside PATH")
    environment = host_environment()
    args.output.mkdir(parents=True)
    output = args.output.resolve()
    log_path = output / "build.log"
    # MSBuild's generated paths can exceed MAX_PATH beneath a long project path.
    with tempfile.TemporaryDirectory(prefix="catalog-") as temporary:
        work = Path(temporary)
        opencv = work / "opencv-discovery-stub"
        opencv.mkdir()
        (opencv / "OpenCVConfig.cmake").write_text(
            'set(OpenCV_FOUND TRUE)\nset(OpenCV_LIBS "")\nset(OpenCV_INCLUDE_DIRS "")\n', encoding="utf-8")
        with log_path.open("w", encoding="utf-8") as log:
            for first in ("config-only", "compare-only", "combined"):
                source, build = work / first, work / (first + "-build")
                source.mkdir()
                (source / "probe.cpp").write_text(PROBE, encoding="utf-8")
                config = f'add_subdirectory("{ROOT.as_posix()}/scheduling/config" native-config)'
                compare = f'add_subdirectory("{ROOT.as_posix()}/compare/native" native-compare)'
                order = [config] if first == "config-only" else [compare] if first == "compare-only" else [config, compare]
                cmakelists = [
                    'cmake_minimum_required(VERSION 3.20)',
                    'project(native_catalog_regression LANGUAGES CXX)',
                    'set(CMAKE_CXX_STANDARD 17)',
                    'set(BUILD_TESTING ON CACHE BOOL "" FORCE)',
                    'set(R36_NATIVE_COMPARE_TESTS ON CACHE BOOL "" FORCE)',
                    'include(CTest)',
                    'if(MSVC)',
                    '  add_compile_options(/utf-8 /EHsc)',
                    'endif()',
                    f'add_subdirectory("{ROOT.as_posix()}/features/visual_motion" motion)',
                    *order,
                ]
                if first != "config-only":
                    cmakelists += [
                    'add_executable(native_catalog_probe probe.cpp)',
                    'target_include_directories(native_catalog_probe PRIVATE "$<TARGET_PROPERTY:perception_compare_native,INCLUDE_DIRECTORIES>")',
                    'target_link_libraries(native_catalog_probe PRIVATE "$<TARGET_PROPERTY:perception_compare_native,LINK_LIBRARIES>")',
                    'add_library(native_contract_header_probe OBJECT probe.cpp)',
                    'target_include_directories(native_contract_header_probe PRIVATE "$<TARGET_PROPERTY:native_compare_contract_test,INCLUDE_DIRECTORIES>")',
                    ]
                (source / "CMakeLists.txt").write_text("\n".join(cmakelists) + "\n", encoding="utf-8")
                configure = [cmake, "-S", str(source), "-B", str(build), f"-DOpenCV_DIR={opencv.as_posix()}"]
                if os.name == "nt":
                    ninja = shutil.which("ninja", path=environment.get("PATH"))
                    if not ninja:
                        installation = Path(environment["VSINSTALLDIR"])
                        ninja = str(installation / "Common7/IDE/CommonExtensions/Microsoft/CMake/Ninja/ninja.exe")
                    configure += ["-G", "Ninja", f"-DCMAKE_MAKE_PROGRAM={ninja}", "-DCMAKE_BUILD_TYPE=Release"]
                run(configure, log, environment)
                targets = [] if first == "config-only" else ["native_catalog_probe", "native_contract_header_probe"]
                if first != "compare-only":
                    targets.append("perception_config")
                run([cmake, "--build", str(build), "--config", "Release", "--parallel", "2",
                     "--target", *targets], log, environment)
                if first != "config-only":
                    binary = build / ("native_catalog_probe.exe" if os.name == "nt" else "native_catalog_probe")
                    run([str(binary)], log, environment)
                if first != "compare-only":
                    launcher = build / "native-config" / ("perception_config.exe" if os.name == "nt" else "perception_config")
                    run([str(launcher), "--help"], log, environment)
                print(f"PASS: {first} CMake entry; targets: {', '.join(targets)}")
    print(f"Evidence: {log_path}")
    print("OpenCV discovery mocked; full native/ARM/ROS compilation not performed.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError) as error:
        raise SystemExit(str(error))
