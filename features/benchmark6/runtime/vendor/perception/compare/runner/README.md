# Comparison runner contract

This is an offline, synchronous Linux consumer of the **real r36 shared SDK**.
It loads the SDK and optional Horizon stereo model once per process. Run methods
in separate processes, sequentially, to avoid CPU/BPU competition and isolate RSS.

```
r36_compare_runner --manifest pairs.tsv --sdk-config sdk_full.json \
  --method original --output NEW_METHOD_DIRECTORY \
  --width 640 --height 544 --fx FX --fy FY --cx CX --cy CY \
  --bxf FX_TIMES_BASELINE_MM --warmup 5 --repeat 3 --save-every 10
```

`--method stereo` additionally requires `--model MODEL.bin`. It invokes the actual
`HorizonStereoModel(left, right)` in the SDK disparity provider callback, so its
execution is included in the SDK `disparity` Task. Unsupported models, missing
Horizon support, or missing SDK mapping fail explicitly. There is no fallback.
`--det-model` and `--seg-model` optionally override the SDK configuration paths.

The headerless UTF-8/LF TSV manifest has exactly five columns:

```
stem<TAB>left_nv12<TAB>left_bgr_png<TAB>right_bgr_png<TAB>original_q8_raw
```

Input paths may be absolute or relative to the manifest. The prepared rectified
NV12 is tight `width*height*3/2` bytes. Original disparity is little-endian tight
uint16 Q8.8, `width*height*2` bytes, belonging to that exact left image. Both stereo
images must decode as BGR8 at the source manifest dimensions. The default
`--stereo-input-mode strict` requires them to equal the actual model dimensions.
For the NCHW `[1,3,352,640]` model and original 640x544 pairs, explicitly select
`--stereo-input-mode resize --stereo-input-width 640 --stereo-input-height 352`.
Both views are resized with INTER_LINEAR inside the disparity provider, so that
cost is included in stereo/compute timings. Width/height assert real metadata;
they never rewrite it. Tensor layout and image type come from Horizon metadata:
NCHW with three channels does not imply RGB instead of NV12.
Model output disparity is normalized using the v8p1
finite-positive interpolation and width scaling contract, then rounded half-up
to source-resolution Q8; invalid/overflow values are zero. For 640x544 to
640x352, horizontal disparity scale is 640/640=1, never 544/352. Original NV12
and source calibration remain the inputs to detection/segmentation/mapping.
`run.json` preserves requested assertions, source/model/output/restored sizes,
actual tensor metadata and scaling factors in `stereo_input_adaptation`.

The SDK JSON must enable detection, segmentation, original mapping, cloud
retention, and command 1. Use `output.every_n=0` to keep disk recording outside
computation. Use `dnn.run_every_n=1` in the referenced original mapping config.
The SDK graph retains its real concurrency: the sum of Task durations is not a
wall-clock measurement.

`warmup=N` performs N real calls cycling the dataset, followed by `repeat=N`
complete measured passes. Every call receives unique increasing identity and
monotonic timestamp. Warmup rows carry `warmup=true` and must be excluded from
benchmark summaries. Saving uses measured ordinals `0,N,2N,...` in both methods;
`save-every=0` disables explicit result saving. Every saved frame includes the
SDK result/maps/trace/disparity and **actual SDK** `camera`, `robot`, `ground`,
`obstacle`, and `semantic` PLY views, including empty clouds. PLY coordinates are
meters and RGB/confidence/labels are copied unchanged from SDK views. SDK cloud
generation, filtering, transforms, Patchwork and semantic fusion are preserved.

`frames.jsonl` lives in the method output directory; its `clouds` paths and
`output_dir` are relative to that directory. `task_trace` is the SDK's JSON object.
`load_ms` is disk/image decoding, `compute_wall_ms` covers frame packing plus the
synchronous SDK call and view, `save_ms` is explicit SDK/PLY writing, and
`total_wall_ms` covers these operations but excludes measurement JSON logging.
`inference_ms` contains six SDK detection/segmentation stage times.

`cpu_ms` / `cpu_wall_ms` cover the computation interval using Linux process
`getrusage(RUSAGE_SELF)` user+system time, including its worker threads. A single
fully busy core is 100% in `cpu_percent`; `cpu_percent_machine` divides by
`online_cpus`. `rss_kib` is `/proc/self/status` VmRSS after computation;
`peak_rss_kib` is process-lifetime `ru_maxrss`, including initialization and
warmup, **not** a per-frame peak. `cpu_total_ms` covers the load/compute/save scope.
Aggregate FPS must be measured frame count divided by the sum of corresponding
wall times; do not average per-frame FPS.

`complete.json` additionally reports `measured_wall_ms`, `measured_cpu_ms`, and
`observed_replay_fps`. This is measured around the entire measured repeat loop,
including loading, computation, saving, JSON/progress logging, buffer cleanup,
and result destruction. It excludes model initialization and warmup. This is
actual offline replay throughput, whereas `total_fps` describes the narrower
staged per-frame scope above. Neither establishes live camera/ROS transport FPS.

Original input disparity is already recorded; its generation cost is unknown.
`stereo_ms=null` for original and `original_disparity_generation_timed=false` in
run metadata make this explicit. The stereo method includes genuine model
generation, conversion and mapping in its computation. Initialization is
reported separately in `run.json`; failed/partial runs have no `complete.json`.

Standalone build (actual SDK and OpenCV development packages required):

```
cmake -S compare/runner -B build/compare -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$SDK_ROOT" \
  -DR36_COMPARE_WITH_HORIZON=ON -DHORIZON_ROOT="$HORIZON_ROOT"
cmake --build build/compare --parallel 4
ctest --test-dir build/compare --output-on-failure
```

For x86 original-only validation set `R36_COMPARE_WITH_HORIZON=OFF`. Cross builds
use the existing project's target toolchain and real AArch64 SDK; never execute
target CTest binaries on the x86 host. `PSDK_LINK_DEPENDENCY_DIRS` may provide
semicolon-separated target DSO directories for the linker's `-rpath-link`.
