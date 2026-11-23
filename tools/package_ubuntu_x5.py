#!/usr/bin/env python3
"""Package Ubuntu sources + built host/X5 tools; native camera ABI stays on X5."""
from pathlib import Path
import subprocess
import argparse
import hashlib
import importlib.util
import json
import os
import shutil
import tarfile
import tempfile
from datetime import datetime,timezone

ROOT=Path(__file__).resolve().parent.parent
if subprocess.run(['bash',str(ROOT/'tools/check_no_local_payload.sh'),str(ROOT)]).returncode: raise SystemExit(2)
SPEC=importlib.util.spec_from_file_location('arm_audit',ROOT/'tools/package_wsl_benchmark6.py')
AUDIT=importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)
PLATFORM=('libcam.so','libvpf.so','libhbmem.so','libgdcbin.so')
COMPONENTS=('install/backend_sdk_x5','install/perception_ros_so_demo_x5','features/benchmark6/runtime/install/x5')

def manifest(directory):
    p=directory/'SHA256SUMS'
    entries=[AUDIT.sha(f)+'  '+f.relative_to(directory).as_posix()
             for f in sorted(directory.rglob('*'))
             if f.is_file() and not f.is_symlink() and f!=p]
    p.write_text('\n'.join(entries)+'\n',encoding='utf-8')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sysroot',required=True)
    parser.add_argument('--target-prefix',help='ARM dependency prefix; historical Docker uses SYSROOT/usr_x5')
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    sysroot=Path(args.sysroot).resolve()
    prefix=Path(args.target_prefix or os.environ.get('TARGET_SYSTEM_PREFIX') or sysroot/'usr').resolve()
    output=Path(args.output).absolute()
    if not sysroot.is_dir(): raise RuntimeError('Missing matching X5 sysroot: '+str(sysroot))
    if not prefix.is_dir(): raise RuntimeError('Missing ARM dependency prefix: '+str(prefix))
    if output.exists(): raise RuntimeError('Choose a NEW output archive: '+str(output))
    required=('build/benchmark6/r36_benchmark6','features/benchmark6/runtime/install/ubuntu_ros/bin/ab_result_replay',
              'features/benchmark6/runtime/install/x5/bin/ab_benchmark','install/backend_sdk_x5/lib/libperception_sdk.so.1',
              'install/perception_ros_so_demo_x5/bin/perception_config')
    for name in required:
        if not (ROOT/name).is_file(): raise RuntimeError('Build --all first; missing '+name)
    output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='perception_ubuntu_v4_') as temporary:
        stage=Path(temporary)/ROOT.name
        print('Staging Ubuntu source and prebuilt components...',flush=True)
        ignore=shutil.ignore_patterns('build','dist','results','.git','__pycache__','.idea','.vscode')
        shutil.copytree(ROOT,stage,symlinks=True,ignore=ignore)
        windows_deps=stage/'windows/deps'
        if windows_deps.is_dir(): shutil.rmtree(windows_deps)
        for name in ('SHA256SUMS','SOURCE_SHA256SUMS'):
            p=stage/name
            if p.exists(): p.unlink()
        (stage/'build/benchmark6').mkdir(parents=True)
        shutil.copy2(ROOT/'build/benchmark6/r36_benchmark6',stage/'build/benchmark6/r36_benchmark6')
        evidence={}; native={}
        for component in COMPONENTS:
            print('Auditing ARM component: '+component,flush=True)
            folder=stage/component
            for p in (folder/'lib').iterdir():
                if any(p.name.startswith(stem) for stem in PLATFORM): p.unlink()
            records={}
            extra=[stage/'install/backend_sdk_x5/lib'] if component.endswith('perception_ros_so_demo_x5') else []
            extra += [prefix/'lib/aarch64-linux-gnu',prefix/'hobot/lib',prefix/'lib/hbbpu',prefix/'lib']
            native[component]=AUDIT.bundle_libraries(folder,sysroot,extra,records)
            evidence[component]=records
            manifest(folder)
            shutil.copy2(folder/'SHA256SUMS',ROOT/component/'SHA256SUMS')
        manifest(stage/'features/benchmark6/runtime/install/ubuntu_ros')
        glibc={v for group in evidence.values() for item in group.values() for v in item['glibc_requirements']}
        maximum_glibc=max(glibc,key=lambda v:tuple(map(int,v[6:].split('.')))) if glibc else None
        report={
            'packaged_at_utc':datetime.now(timezone.utc).isoformat(),
            'kind':'Ubuntu source + prebuilt X5/Ubuntu tools',
            'host_runtime':'Ubuntu 22.04 x86_64 + ROS2 Humble',
            'target_runtime':'Linux AArch64; matching board-native camera/platform libraries; glibc requirement recorded below',
            'maximum_glibc_requirement':maximum_glibc,
            'architecture_dependency_and_symbol_checks':'passed_against_selected_sysroot',
            'sysroot_used_for_dependency_check':str(sysroot),
            'target_dependency_prefix':str(prefix),
            'docker_build_evidence':json.loads((ROOT/'validation/docker_build.json').read_text(encoding='utf-8'))
                if (ROOT/'validation/docker_build.json').is_file() else None,
            'board_libraries_not_bundled':list(PLATFORM),
            'native_dependencies_required':native,
            'X5_board_validation':'not_performed_for_v4',
            'prior_board_failure':'hbn_camera_create=-1 / moOpenCamera=-3; native library policy requires board retest, not a confirmed camera fix',
            'prior_binary_build_evidence':'validation/prior_wsl_build/BUILD_REPORT.json',
            'source_change_scope':'Ubuntu packaging/build/run scripts and default paths; C++ inference/DAG sources unchanged from v3',
            'elf_audit':evidence,
        }
        (stage/'DELIVERY_REPORT.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        (ROOT/'DELIVERY_REPORT.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        for p in stage.rglob('*'):
            if not p.is_file() or p.is_symlink(): continue
            with p.open('rb') as f: head=f.read(4)
            if p.suffix=='.sh' or p.name=='adb' or head==b'\x7fELF': p.chmod(0o755)
        # Dedup model/runtime copies without changing the archive's directory layout.
        known={}
        for p in sorted(stage.rglob('*')):
            if not p.is_file() or p.is_symlink(): continue
            key=(p.stat().st_size,AUDIT.sha(p),p.stat().st_mode & 0o777)
            if key in known:
                p.unlink(); os.link(known[key],p)
            else: known[key]=p
        manifest(stage)
        print('Creating archive: '+str(output),flush=True)
        with tarfile.open(output,'w:gz',compresslevel=6) as tar:
            tar.add(stage,arcname=stage.name)
    digest=AUDIT.sha(output)
    output.with_suffix(output.suffix+'.sha256').write_text(digest+'  '+output.name+'\n',encoding='ascii')
    print(json.dumps({'archive':str(output),'bytes':output.stat().st_size,'sha256':digest}),flush=True)

if __name__=='__main__': main()
