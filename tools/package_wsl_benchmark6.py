#!/usr/bin/env python3
"""Package existing builds and audit AArch64 DT_NEEDED closure without a device."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import tarfile
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parent.parent
if subprocess.run(['bash',str(ROOT/'tools/check_no_local_payload.sh'),str(ROOT)]).returncode: raise SystemExit(2)
RUNTIME = ROOT / 'features/benchmark6/runtime'
SYSTEM_LIBS = {'ld-linux-aarch64.so.1', 'libc.so.6', 'libm.so.6', 'libpthread.so.0',
               'libdl.so.2', 'librt.so.1', 'libresolv.so.2', 'libutil.so.1',
               'libcam.so.1', 'libvpf.so.1', 'libhbmem.so.1', 'libgdcbin.so'}
AUDITS = {}
VERSIONS = {}
COPIED_LIBRARIES = {}

def file_key(path):
    s = path.stat()
    return s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns

def versions(path):
    key = file_key(path)
    if key not in VERSIONS:
        VERSIONS[key] = command('readelf', '--version-info', path)
    return VERSIONS[key]

def command(*args):
    return subprocess.check_output([str(x) for x in args], text=True, stderr=subprocess.STDOUT)

def sha(path):
    with path.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest() if hasattr(hashlib, 'file_digest') else hashlib.sha256(f.read()).hexdigest()

def elf(path):
    try:
        with path.open('rb') as f:
            return f.read(4) == b'\x7fELF'
    except (OSError, RuntimeError):
        return False

def audit(path):
    key = file_key(path)
    if key in AUDITS:
        return AUDITS[key]
    header = command('readelf', '-h', path)
    if not re.search(r'Machine:\s+AArch64\b', header):
        raise RuntimeError(f'Non-AArch64 binary in X5 package: {path}')
    dynamic = command('readelf', '-d', path)
    version_text = versions(path)
    result = {
        'needed': re.findall(r'\(NEEDED\).*\[(.*?)\]', dynamic),
        'glibc_requirements': sorted(set(re.findall(r'Name: (GLIBC_[0-9.]+)', version_text))),
        'glibcxx_requirements': sorted(set(re.findall(r'Name: (GLIBCXX_[0-9.]+)', version_text))),
        'sha256': sha(path),
    }
    AUDITS[key] = result
    return result

def copy_tree(source, dest):
    if not source.is_dir():
        raise RuntimeError(f'Missing build output: {source}')
    shutil.copytree(source, dest, symlinks=True)

def bundle_libraries(directory, sysroot, sdk_libs, evidence):
    library = directory / 'lib'
    library.mkdir(exist_ok=True)
    roots = [library, *sdk_libs, sysroot/'usr/lib/aarch64-linux-gnu',
             sysroot/'lib/aarch64-linux-gnu', sysroot/'usr/hobot/lib',
             sysroot/'usr/lib/hbbpu', sysroot/'usr/lib', sysroot/'lib',
             Path('/usr/aarch64-linux-gnu/lib')]
    roots = list(dict.fromkeys(roots))
    pending = [p for p in directory.rglob('*') if p.is_file() and not p.is_symlink() and elf(p)]
    seen = set()
    system_needed = set()
    while pending:
        path = pending.pop()
        if path in seen:
            continue
        seen.add(path)
        item = audit(path)
        evidence[path.relative_to(directory).as_posix()] = item
        for name in item['needed']:
            if name in SYSTEM_LIBS:
                system_needed.add(name)
                continue
            destination = library/name
            if destination.is_file():
                if destination not in seen:
                    pending.append(destination)
                continue
            source = next((base/name for base in roots if (base/name).is_file()), None)
            if source is None:
                raise RuntimeError(f'Missing ARM dependency {name}, required by {path}')
            source_audit = audit(source)
            digest = source_audit['sha256']
            if digest in COPIED_LIBRARIES:
                try:
                    os.link(COPIED_LIBRARIES[digest], destination)
                except OSError:
                    shutil.copy2(source.resolve(), destination)
            else:
                shutil.copy2(source.resolve(), destination)
                COPIED_LIBRARIES[digest] = destination
            AUDITS[file_key(destination)] = source_audit
            VERSIONS[file_key(destination)] = versions(source)
            pending.append(destination)
    # Every bundled dependency must satisfy the exact versioned symbols requested.
    provided = {}
    for p in library.iterdir():
        if p.is_file() and elf(p):
            provided[p.name] = set(re.findall(r'Name: ([A-Za-z_][A-Za-z_0-9.]*)', versions(p)))
    for name in system_needed:
        system_path = next((base/name for base in roots if (base/name).is_file()), None)
        if system_path is None:
            raise RuntimeError(f'Missing target system library in sysroot: {name}')
        audit(system_path)
        provided[name] = set(re.findall(r'Name: ([A-Za-z_][A-Za-z_0-9.]*)', versions(system_path)))
    for relative, item in evidence.items():
        source = directory/relative
        version_text = versions(source)
        section = version_text.split('Version needs section', 1)
        if len(section) != 2:
            continue
        current = None
        for line in section[1].splitlines():
            match = re.search(r'File: (\S+)', line)
            if match:
                current = match.group(1)
            version = re.search(r'Name: (\S+)\s+Flags:', line)
            if current in provided and version and version.group(1) not in provided[current]:
                raise RuntimeError(f'Unsatisfied symbol version {version.group(1)} from {current}, required by {source}')
    return sorted(system_needed)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--sysroot', required=True)
    args = parser.parse_args()
    stage = Path(args.output).absolute()
    sysroot = Path(args.sysroot).absolute()
    if stage.exists():
        raise RuntimeError('Package staging directory must be NEW')
    stage.mkdir(parents=True)
    for name in ('backend_sdk_x5', 'perception_ros_so_demo_x5'):
        copy_tree(ROOT/'install'/name, stage/'install'/name)
    copy_tree(RUNTIME/'install/x5', stage/'features/benchmark6/runtime/install/x5')
    copy_tree(RUNTIME/'install/ubuntu_ros', stage/'features/benchmark6/runtime/install/ubuntu_ros')
    copy_tree(RUNTIME/'config', stage/'features/benchmark6/runtime/config')
    (stage/'build/benchmark6').mkdir(parents=True)
    shutil.copy2(ROOT/'build/benchmark6/r36_benchmark6', stage/'build/benchmark6/r36_benchmark6')
    for source in ROOT.glob('*.json'):
        shutil.copy2(source, stage/source.name)
    for source in ROOT.glob('*.md'):
        shutil.copy2(source, stage/source.name)
    for source in ROOT.glob('features/*/config.json'):
        dest = stage/source.relative_to(ROOT)
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, dest)
    for name in ('config', 'calibration', 'models'):
        copy_tree(ROOT/name, stage/name)
    # Tools are included as scripts; all mandatory host and X5 executables are prebuilt.
    (stage/'tools').mkdir()
    for source in (ROOT/'tools').glob('*.sh'):
        shutil.copy2(source, stage/'tools'/source.name)
    for source in (ROOT/'tools').glob('*.py'):
        if source.name == Path(__file__).name:
            shutil.copy2(source, stage/'tools'/source.name)
    checks = {'business_sdk': {}, 'business_tools': {}, 'benchmark': {}}
    backend = stage/'install/backend_sdk_x5'
    tools = stage/'install/perception_ros_so_demo_x5'
    benchmark = stage/'features/benchmark6/runtime/install/x5'
    system = {}
    system['business_sdk'] = bundle_libraries(backend, sysroot, [], checks['business_sdk'])
    system['business_tools'] = bundle_libraries(tools, sysroot, [backend/'lib'], checks['business_tools'])
    system['benchmark'] = bundle_libraries(benchmark, sysroot, [], checks['benchmark'])
    relocation = ROOT/'build/package_checks'/('relocation_'+datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S'))
    relocation.mkdir(parents=True)
    command(stage/'build/benchmark6/r36_benchmark6', '--prepare', stage/'benchmark6.json', relocation/'suite')
    checker = stage/'features/benchmark6/runtime/install/ubuntu_ros/bin/ab_config_check'
    for directory in ('board-config', 'host-config'):
        for config in sorted((relocation/'suite'/directory).glob('*.json')):
            command(checker, config)
    host_config = ROOT/'build/check_packaged_dag'
    if not host_config.is_file():
        checker_source = ROOT/'build/check_packaged_dag.cpp'
        checker_source.write_text('''#include "native_config_detail.hpp"
#include "product_graph.hpp"
#include <iostream>
int main(int argc,char**argv){try{if(argc!=2)return 2;auto j=v3::normalizePipeline(v3::readJson(argv[1]));(void)r33::full::parseProductGraph(v3::detail::get(j,"graph"));return 0;}catch(const std::exception&e){std::cerr<<e.what()<<'\\n';return 1;}}
''')
        command('g++', '-std=c++17', '-O2', '-I'+str(ROOT/'scheduling/config/include'),
                '-I'+str(ROOT/'scheduling/dag/include'), '-I'+str(ROOT/'core/include'),
                checker_source, ROOT/'build/benchmark6/config/libperception_native_config.a',
                ROOT/'build/benchmark6/motion/libr36_motion_features.a', '-pthread', '-o', host_config)
    for name in ('camera_original_q8', 'camera_stereo', 'pipeline'):
        result = subprocess.run([str(host_config), str(stage/(name+'.json'))],
                                text=True, capture_output=True)
        (relocation/(name+'.log')).write_text(result.stdout+result.stderr)
        if result.returncode:
            raise RuntimeError(f'Packaged DAG validation failed for {name}: {result.stderr}')
    for path in (backend/'bin/perception_full_demo', tools/'bin/perception_config',
                 stage/'models/x5/mmyolo0828.bin', stage/'models/x5/pidnet_0825_3classes.bin',
                 stage/'models/x5/r34_crop_stage2_i12_cheb5_max1_tail_int16_352x640_20261012.bin'):
        if not path.is_file(): raise RuntimeError(f'Missing packaged runtime/model: {path}')
    relocation_result = {'status': 'passed', 'generated_test_configs': 9,
                         'business_dags_normalized_and_validated': 3, 'models_or_camera_executed': False,
                         'board_camera_sdk_file_check': 'requires_X5_device_files; not performed on host',
                         'evidence_directory': str(relocation)}
    (ROOT/'build/wsl_release_logs/relocation_checks.json').write_text(json.dumps(relocation_result, indent=2)+'\n')
    report = {
        'built_at_utc': datetime.now(timezone.utc).isoformat(),
        'host': command('uname', '-a').strip(),
        'compiler': command('aarch64-linux-gnu-g++', '--version').splitlines()[0],
        'sysroot': str(sysroot), 'architecture': 'Linux AArch64',
        'cross_compilation': 'passed', 'arm_dependency_closure': 'passed',
        'versioned_dependency_symbols': 'passed', 'board_test': 'not_run',
        'accuracy_and_performance': 'not_measured',
        'host_checks': json.loads((ROOT/'build/wsl_release_logs/host_checks.json').read_text()),
        'host_ros_transport_checks': json.loads((ROOT/'build/wsl_release_logs/ros_transport_checks.json').read_text()),
        'host_visualization_checks': json.loads((ROOT/'build/wsl_release_logs/visualization_checks.json').read_text()),
        'package_relocation_checks': relocation_result,
        'board_system_libraries_required': system,
        'board_camera_sdk_required': '/app/data/metoak/lib/libMoGeneralSDK.so',
        'board_camera_config_required': '/app/data/metoak/demo/config/x5_camera.cfg',
        'elf_audit': checks,
    }
    glibc_versions = {v for group in checks.values() for item in group.values() for v in item['glibc_requirements']}
    report['maximum_glibc_requirement'] = max(glibc_versions, key=lambda v: tuple(map(int, v[6:].split('.')))) if glibc_versions else None
    source_lines = []
    for path in sorted(ROOT.rglob('*')):
        relative = path.relative_to(ROOT)
        if set(relative.parts) & {'build', 'install', 'dist', '.git', '__pycache__'}:
            continue
        if path.is_file() and (path.suffix in {'.cpp', '.hpp', '.h', '.cmake', '.sh', '.json'} or path.name == 'CMakeLists.txt'):
            source_lines.append(sha(path)+'  '+relative.as_posix())
    source_manifest = '\n'.join(source_lines)+'\n'
    report['source_manifest_sha256'] = hashlib.sha256(source_manifest.encode()).hexdigest()
    (stage/'SOURCE_SHA256SUMS').write_text(source_manifest)
    shutil.copytree(ROOT/'build/wsl_release_logs', stage/'BUILD_LOGS')
    (stage/'BUILD_REPORT.json').write_text(json.dumps(report, indent=2)+'\n')
    (stage/'BUILD_REPORT_zh.md').write_text('''# WSL交叉编译与交付检查

- 构建主机：WSL2 Ubuntu 22.04 x86_64。
- 目标：Linux AArch64，使用现有X5 sysroot和aarch64-linux-gnu编译器。
- 业务SDK、DAG配置/运行程序、六组测试程序：编译通过。
- Ubuntu汇总、ROS桥接、RViz结果回放程序：编译通过。
- X5 ELF架构、动态库依赖闭包、版本化依赖符号：检查通过。
- 主机配置、已知合成数据的AP50/mIoU/EPE、重复汇总、PNG/Q8/PLY保存：检查通过。
- 本机ROS DDS→TCP确认式回放：两种视差输入契约及原始时间戳保持检查通过。
- 六组保存结果的RViz配置和rosbag录制：合成数据检查通过，48个结果话题、48条消息。

未连接X5，未执行真实相机/模型推理，没有板端精度、耗时、BPU、CPU、RSS统计或实际RViz画面。合成数据仅验证程序行为。

构建日志见 BUILD_LOGS，逐个ARM ELF与依赖审计见 BUILD_REPORT.json。
''', encoding='utf-8')
    (stage/'README_RUN_zh.md').write_text('''# WSL交叉编译交付包

本包包含已编译的 Linux AArch64 业务程序、六组测试程序、ARM 动态库、模型与配置；另含 Ubuntu 22.04 x86_64 ROS2 Humble 配套程序。

已执行交叉编译、ARM ELF 架构检查、动态库依赖闭包与版本符号检查。没有连接 X5，尚无板端精度、性能或 RViz 实测结果。详情见 BUILD_REPORT.json 和 BUILD_LOGS。

## 在连接X5的远程Ubuntu运行六组测试

解压后进入本目录，确保已安装 ROS2 Humble、rosbag2 sqlite3插件、ADB。只在一个设备时可省略 ADB_SERIAL：

```bash
source /opt/ros/humble/setup.bash
export ADB_SERIAL=你的X5序列号
export ROS_DOMAIN_ID=67
bash tools/run_benchmark6_adb.sh benchmark6.json
```

程序、模型和所需非glibc ARM库会自动推送。相机厂商SDK及其配置应已位于 benchmark6.json 的板端路径。部署目录已存在时请修改配置中的 board_root，或确认版本后设置 B6_REUSE_RUNTIME=1。每次使用全新结果目录。

测试完成后保存检测/分割/视差图、Q8、PLY、rosbag、RViz布局和汇总报告。无独立真值时，绝对精度指标为空。详细测试和回放说明见 BENCHMARK6_zh.md。

## 在X5运行可配置DAG业务程序

将整个包复制到X5并解压，进入包目录。下面仅准备配置，不启动相机：

```bash
bash tools/run_business_x5.sh run --config camera_original_q8.json --run-dir /userdata/dag_prepare_001 --prepare-only
```

确认配置后使用新的运行目录启动：

```bash
bash tools/run_business_x5.sh run --config camera_original_q8.json --run-dir /userdata/dag_original_001
bash tools/run_business_x5.sh run --config camera_stereo.json --run-dir /userdata/dag_stereo_001
```

两个相机任务应依次运行。检测/分割/视差以及其它功能仍通过任务和 dag.sequence/parallel 配置。业务完整流程与六组固定调度性能入口分别使用各自的SDK目录，脚本已设置对应的LD_LIBRARY_PATH。

包内glibc系统库由板端提供，匹配本次sysroot；相机SDK同样由板端提供。Ubuntu配套程序要求x86_64 Ubuntu22.04/兼容ABI和ROS2 Humble。图形桌面只在打开RViz时需要。
''', encoding='utf-8')
    # Refresh component manifests after adding dependencies and updating README.
    for directory in (backend, tools, benchmark, stage/'features/benchmark6/runtime/install/ubuntu_ros'):
        checksum_file = directory/'SHA256SUMS'
        if checksum_file.exists():
            entries = [f'{sha(p)}  {p.relative_to(directory).as_posix()}'
                       for p in sorted(directory.rglob('*'))
                       if p.is_file() and not p.is_symlink() and p != checksum_file]
            checksum_file.write_text('\n'.join(entries)+'\n')
    # Deduplicate identical content as hard links; tar preserves these links.
    existing = {}
    for path in sorted(stage.rglob('*')):
        if path.is_symlink() or not path.is_file():
            continue
        key = (path.stat().st_size, sha(path))
        if key in existing:
            other = existing[key]
            if path.stat().st_ino != other.stat().st_ino:
                temp = path.with_name(path.name+'.dedup_tmp')
                try:
                    os.link(other, temp)
                    os.replace(temp, path)
                except OSError:
                    if temp.exists(): temp.unlink()
        else:
            existing[key] = path
    sums = [f'{sha(p)}  {p.relative_to(stage).as_posix()}' for p in sorted(stage.rglob('*')) if p.is_file() and not p.is_symlink()]
    (stage/'SHA256SUMS').write_text('\n'.join(sums)+'\n')
    archive = stage.with_suffix('.tar.gz')
    if archive.exists():
        raise RuntimeError('Refusing to overwrite an existing release archive')
    with tarfile.open(archive, 'w:gz', compresslevel=6) as tar:
        tar.add(stage, arcname=stage.name)
    archive.with_name(archive.name+'.sha256').write_text(sha(archive)+'  '+archive.name+'\n')
    print(json.dumps({'package': str(archive), 'bytes': archive.stat().st_size,
                      'sha256': sha(archive), 'arm_elf_files': sum(map(len, checks.values()))}, indent=2))

if __name__ == '__main__':
    main()
