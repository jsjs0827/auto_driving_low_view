#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
for script in run_benchmark6_adb.sh build_ubuntu_x5.sh pull_x5_sysroot.sh run_x5.sh x5_runtime_env.sh run_business_x5.sh; do
  bash -n "$root/tools/$script"
done
python3 "$root/tools/validate_adb_compat.py"
scratch=$(mktemp -d "$root/build/ubuntu_v4_validation_XXXXXX")
ctl="$root/build/benchmark6/r36_benchmark6"
checker="$root/features/benchmark6/runtime/install/ubuntu_ros/bin/ab_config_check"
for kind in benchmark6 benchmark6_smoke; do
  "$ctl" --prepare "$root/$kind.json" "$scratch/$kind" >/dev/null
  for config in "$scratch/$kind"/board-config/*.json "$scratch/$kind"/host-config/*.json; do
    "$checker" "$config" >/dev/null
  done
done
"$ctl" --report "$scratch/benchmark6" >/dev/null
"$ctl" --report "$scratch/benchmark6" >/dev/null
bash "$root/tools/build_ubuntu_x5.sh" --help >/dev/null
bash "$root/tools/pull_x5_sysroot.sh" --help >/dev/null
bash "$root/tools/run_x5.sh" --help >/dev/null
python3 - "$root" "$scratch" <<'PY'
from pathlib import Path
import ast,json,sys
root,scratch=map(Path,sys.argv[1:])
for p in (root/'tools').glob('*.py'): ast.parse(p.read_text(encoding='utf-8'))
ast.parse((root/'tools/adb_legacy_compat/adb').read_text())
for p in root.glob('*.json'):json.loads(p.read_text())
components=('install/backend_sdk_x5','install/perception_ros_so_demo_x5','features/benchmark6/runtime/install/x5')
for name in components:
    for p in (root/name/'lib').iterdir():
        assert not any(p.name.startswith(s) for s in ('libcam.so','libvpf.so','libhbmem.so','libgdcbin.so')),p
report=json.loads((scratch/'benchmark6/summary.json').read_text())
assert report['complete_groups']==0 and len(report['groups'])==6
result={'status':'passed','shell_scripts_parsed':6,'generated_runtime_configs_checked':18,
        'report_repeat_write':'passed','absent_groups_remain_unavailable':True,
        'native_camera_platform_libraries_not_bundled':True,
        'X5_camera_models_performance_executed':False,
        'evidence_directory':str(scratch)}
(root/'validation/ubuntu_v4_checks.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
PY
