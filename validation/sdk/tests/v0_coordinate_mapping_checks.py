#!/usr/bin/env python3
"""Compile the real mapping function and check saved V0 FP32 output goldens.

No BPU, model, ROS or private input backend is needed. Build artifacts and JSON
results must be written outside the source tree. A target X5 run is still
required to validate the entire model/decode path.
"""
import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import struct
import subprocess


def sha(data):
    return hashlib.sha256(data).hexdigest()


def function(text):
    start = text.index("RawDetection MapDetectionToOriginal(")
    opened = text.index("{", start)
    depth = 1
    end = opened + 1
    while depth:
        if text[end] == "{":
            depth += 1
        elif text[end] == "}":
            depth -= 1
        end += 1
    return text[start:end]


def bits(value):
    return struct.unpack("<I", struct.pack("<f", value))[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    project = args.project.resolve(strict=True)
    out = args.out.resolve()
    if out.is_relative_to(project):
        parser.error("--out must be outside the source tree")
    out.mkdir(parents=True, exist_ok=False)
    source = project / "features/inference/backend/src/runner.cpp"
    # Protected C/C++ paths may have an encrypted backing representation.
    content = subprocess.check_output(["cat", str(source)])
    body = function(content.decode("utf-8"))
    golden_file = Path(__file__).with_name("v0_coordinate_mapping_golden.json")
    golden = json.loads(golden_file.read_text())
    cases = list(golden["cases"])
    cases += [
        {"name": "equal-size-no-letterbox", "raw_xywh": [100, 20, 300, 400],
         "meta": [0, 0, 1, 1], "source_size": [640, 544], "expected_xyxy": [100, 20, 400, 420]},
        {"name": "direct-resize-anisotropic-clipping", "raw_xywh": [-10, 10, 30, 40],
         "meta": [0, 0, .5, 2], "source_size": [640, 544], "expected_xyxy": [0, 5, 40, 25]},
        {"name": "letterbox-unclipped-origin-before-right", "raw_xywh": [140, -10, 40, 30],
         "meta": [150, 0, .5, .5], "source_size": [640, 544], "expected_xyxy": [0, 0, 60, 40]},
        {"name": "right-bottom-clipping", "raw_xywh": [300, 500, 100, 100],
         "meta": [0, 0, .5, .5], "source_size": [640, 544], "expected_xyxy": [600, 544, 640, 544]},
    ]
    wrapper = '''#include <algorithm>
struct Box {float x,y,w,h;};
struct RawDetection {Box box; int class_id; float confidence;};
struct LetterboxMeta {float x_shift,y_shift,x_scale,y_scale;};
''' + body + '''
extern "C" void map_coordinates(const float* raw, const float* meta,
                                  int w,int h,float* xyxy) {
    RawDetection det{{raw[0],raw[1],raw[2],raw[3]},7,.75f};
    LetterboxMeta m{meta[0],meta[1],meta[2],meta[3]};
    const auto out=MapDetectionToOriginal(det,m,w,h);
    xyxy[0]=out.box.x;xyxy[1]=out.box.y;
    xyxy[2]=out.box.x+out.box.w;xyxy[3]=out.box.y+out.box.h;
    xyxy[4]=static_cast<float>(out.class_id);xyxy[5]=out.confidence;
}
'''
    cpp = out / "extracted_coordinate_mapping.cpp"
    cpp.write_text(wrapper)
    builds = []
    for name, extra in (("generic", []), ("native", ["-march=native"])):
        lib = out / (name + ".so")
        argv = ["g++", "-std=c++17", "-O3", "-fno-fast-math", "-ffp-contract=fast",
                "-Wall", "-Wextra", "-pedantic", "-shared", "-fPIC", *extra,
                str(cpp), "-o", str(lib)]
        built = subprocess.run(argv, text=True, capture_output=True, check=True)
        loaded = ctypes.CDLL(str(lib))
        call = loaded.map_coordinates
        call.argtypes = [ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
                         ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
        call.restype = None
        rows = []
        for case in cases:
            raw = (ctypes.c_float * 4)(*case["raw_xywh"])
            meta = (ctypes.c_float * 4)(*case["meta"])
            value = (ctypes.c_float * 6)()
            call(raw, meta, *case["source_size"], value)
            actual = list(value[:4])
            expected = case["expected_xyxy"]
            equal = [bits(a) == bits(b) for a, b in zip(actual, expected)]
            rows.append({"name": case["name"], "golden_x5_case": "sample_id" in case,
                         "actual": actual, "expected": expected,
                         "exact_fp32_bits_equal": all(equal),
                         "ulp_errors": [abs(bits(a)-bits(b)) for a,b in zip(actual,expected)],
                         "class_score_preserved": value[4] == 7 and value[5] == .75})
        builds.append({"name": name, "compiler_argv": argv, "compiler_stdout": built.stdout,
                       "compiler_stderr": built.stderr, "library_sha256": sha(lib.read_bytes()),
                       "rows": rows, "all_equal": all(r["exact_fp32_bits_equal"] and r["class_score_preserved"] for r in rows)})
    result = {"schema": "detseg.v0-coordinate-regression.v1", "source_path": str(source),
              "source_plaintext_sha256": sha(content), "function_sha256": sha(body.encode()),
              "golden_file_sha256": sha(golden_file.read_bytes()),
              "scope": "extracted real source function, saved V0 X5 goldens plus exact integer geometry cases; host only, no new timing claim",
              "golden_x5_case_count": len(golden["cases"]), "analytical_case_count": len(cases)-len(golden["cases"]),
              "builds": builds, "passed": all(b["all_equal"] for b in builds)}
    (out / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    print(json.dumps({k:v for k,v in result.items() if k != "builds"}, ensure_ascii=False))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
