#!/usr/bin/env bash
set -euo pipefail
printf '%s\n' 'The historical stereo_source entrypoint is retired in v3. Configure input in pipeline.json and select the stereo disparity feature in pipeline.json graph, then use tools/run_pipeline.sh. Camera triplet recording uses tools/capture_stereo.sh.' >&2
exit 2
