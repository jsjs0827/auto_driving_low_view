#!/usr/bin/env bash
# Fail closed before producing a release; do not silently remove local code.
set -euo pipefail
[[ $# == 1 ]] || { echo 'Usage: check_no_local_payload.sh SOURCE_OR_INSTALL_TREE' >&2; exit 2; }
command -v python3 >/dev/null || { echo 'Release check requires python3.' >&2; exit 2; }
python3 - "$1" <<'PY'
from pathlib import Path
import os,subprocess,sys
root=Path(sys.argv[1]).resolve(strict=True)
if not root.is_dir(): raise SystemExit('RELEASE BLOCKED: expected a source/install directory')
# Keep the checker itself free of a contiguous private payload signature.
markers=[b'LOCAL_ONLY_'+b'NV12_SHM_'+b'DO_NOT_DISTRIBUTE', b'local_'+b'nv12', b'nv12_' + b'shm_channel', b'local_' + b'camera_shm']
denied={'local_only','.do-not-package','nv12_'+'shm_channel'}
overlap=max(map(len,markers))-1
try:
 for parent,dirs,names in os.walk(root,followlinks=False):
  dirs.sort();names.sort()
  for name in dirs+names:
   p=Path(parent)/name;rel=p.relative_to(root)
   if any(part in denied for part in rel.parts):
    raise RuntimeError('private path present: '+str(rel))
   if p.is_symlink():
    target=p.resolve(strict=True)
    if any(part in denied for part in target.parts):raise RuntimeError('link targets private payload: '+str(rel))
    try:target.relative_to(root)
    except ValueError:raise RuntimeError('link escapes release root: '+str(rel))
    continue
   if p.is_dir():continue
   if not p.is_file():raise RuntimeError('special file in release: '+str(rel))
   # cat returns plaintext for protected C/C++ source files in this workspace.
   process=subprocess.Popen(['cat','--',str(p)],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
   try:
    tail=b'';bad=False
    while True:
     chunk=process.stdout.read(1024*1024)
     if not chunk:break
     data=tail+chunk
     if any(marker in data for marker in markers):bad=True
     tail=data[-overlap:]
    error=process.stderr.read()
    if process.wait()!=0:raise RuntimeError('cannot inspect '+str(rel)+': '+error.decode(errors='replace'))
    if bad:raise RuntimeError('private source/binary content present: '+str(rel))
   finally:
    if process.poll() is None:process.terminate();process.wait()
    process.stdout.close();process.stderr.close()
 print('Release local-payload check passed: '+str(root))
except (OSError,RuntimeError) as error:
 print('RELEASE BLOCKED: '+str(error),file=sys.stderr);raise SystemExit(2)
PY
