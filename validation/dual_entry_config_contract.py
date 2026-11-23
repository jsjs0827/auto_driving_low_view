#!/usr/bin/env python3
"""Configuration-only contracts; model/image bytes are explicit non-executed fixtures."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--launcher', type=Path, required=True)
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--work-dir', type=Path, required=True)
    args = parser.parse_args()
    root, fixture = args.project_root.resolve(), args.work_dir.resolve()
    fixture.mkdir(parents=True, exist_ok=False)
    (fixture / 'scheduling').mkdir()
    for feature in (root / 'features').iterdir():
        config = feature / 'config.json'
        if config.is_file():
            destination = fixture / 'features' / feature.name
            destination.mkdir(parents=True)
            shutil.copyfile(config, destination / 'config.json')
    for name in ['camera.json', 'visualization.json', 'pipeline.json']:
        shutil.copyfile(root / name, fixture / name)
    shutil.copytree(root / 'scheduling/config/sdk_templates', fixture / 'sdk/config')
    shutil.copytree(root / 'scheduling/config/profiles', fixture / 'sdk/profiles')

    def read(path):
        return json.loads((fixture / path).read_text())

    def write(path, value):
        (fixture / path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')

    for feature in ['detection', 'segmentation', 'stereo']:
        model = fixture / read(f'features/{feature}/config.json')['model']
        model.parent.mkdir(parents=True, exist_ok=True)
        model.write_text('configuration fixture only; never execute as a model\n')
    for name in ['left.nv12', 'right.nv12']:
        (fixture / name).write_text('configuration fixture only; never decode\n')
    intrinsics = dict(width=640, height=544, fx=401, fy=402, cx=319, cy=271, bxf=20050)
    write('intrinsics.json', intrinsics)
    offline = read('features/offline/config.json')
    offline.update(left_files=['left.nv12'], right_files=['right.nv12'],
                   intrinsics_file='intrinsics.json', rectified=True, calibration_confirmed=True)
    write('features/offline/config.json', offline)
    request = read('pipeline.json')
    request['input']['mode'] = 'offline'
    # A single source file contains every feature/task/DAG for both deployments.
    request['deployment'] = dict(mode='standalone', v0_host=dict(
        plugin_library='plugin path/libpercep_main.so', launch_argv=[]))
    write('pipeline.json', request)
    environment = dict(os.environ, PERCEPTION_PROJECT_ROOT=str(fixture), SDK_ROOT=str(fixture / 'sdk'))
    counter = 0

    def prepare(mode=None, config='pipeline.json', success=True, extra=(), run=False):
        nonlocal counter
        counter += 1
        output = fixture / f'run-{counter}'
        command = [str(args.launcher.resolve()), 'run', '--config', str(fixture / config),
                   '--run-dir', str(output)]
        if not run:
            command.append('--prepare-only')
        if mode:
            command.extend(['--deployment', mode])
        command.extend(extra)
        result = subprocess.run(command, env=environment, text=True, capture_output=True)
        assert (result.returncode == 0) == success, result.stdout + result.stderr
        return output, result

    standalone, _ = prepare('standalone')
    board, _ = prepare('v0_host')
    load = lambda folder, name: json.loads((folder / name).read_text())
    sdk_standalone, sdk_board = load(standalone, 'effective_full.json'), load(board, 'effective_full.json')
    assert sdk_standalone['graph'] == sdk_board['graph']
    assert sdk_board['mode'] == 'stopped'
    assert (standalone / 'algorithm_contract.json').read_bytes() == (board / 'algorithm_contract.json').read_bytes()
    expected = hashlib.sha256((board / 'algorithm_contract.json').read_bytes()).hexdigest()
    assert load(board, 'resolved.json')['algorithm_contract_sha256'] == expected
    host = load(board, 'v0_host.json')
    assert host['sdk_config'] == str(board / 'effective_full.json')
    assert host['algorithm_contract_sha256'] == expected
    assert host['initial_command'] == 21
    assert host['stereo']['model'].endswith('.bin')
    resolved = load(board, 'resolved.json')
    assert resolved['arguments'] == [] and resolved['input_mode'] == 'v0_host'
    forbidden = {'camera_sdk', 'camera_config', 'camera_shm_name', 'left_topic', 'right_topic',
                 'q8_topic', 'offline_files', 'offline_right_files', 'offline_q8_files'}
    assert not forbidden.intersection(resolved['parameters'])
    assert resolved['plugin_library'].endswith('plugin path/libpercep_main.so')
    _, failed = prepare('v0_host', success=False, run=True)
    assert 'requires deployment.v0_host.launch_argv' in failed.stderr
    _, failed = prepare('v0_host', success=False, extra=['--both'])
    assert '--both is unavailable' in failed.stderr
    _, failed = prepare('unsupported', success=False)
    assert 'deployment.mode' in failed.stderr

    # A template camera config and offline source files are unnecessary for the host.
    (fixture / 'camera.json').unlink()
    offline['left_files'] = ['${NO_OFFLINE_DATA_REQUIRED_ON_HOST}/left.nv12']
    write('features/offline/config.json', offline)
    (fixture / 'left.nv12').unlink()
    (fixture / 'right.nv12').unlink()
    board_without_data, _ = prepare('v0_host')
    assert load(board_without_data, 'resolved.json')['algorithm_contract_sha256'] == expected

    for change in ['threshold', 'intrinsics', 'fusion', 'model', 'task']:
        saved = {}
        if change in ['threshold', 'model']:
            path = 'features/detection/config.json'
            value = read(path)
            saved[path] = value.copy()
            if change == 'threshold':
                value['score_threshold'] = 0.37
            else:
                value['model'] = 'alternate-det.bin'
                (fixture / value['model']).write_text('configuration fixture only')
            write(path, value)
        elif change == 'intrinsics':
            saved['intrinsics.json'] = intrinsics
            write('intrinsics.json', dict(intrinsics, fx=410))
        elif change == 'fusion':
            path = 'features/fusion/config.json'
            value = read(path)
            saved[path] = json.loads(json.dumps(value))
            value['map']['resolution_m'] = 0.13
            write(path, value)
        else:
            saved['pipeline.json'] = request
            value = json.loads(json.dumps(request))
            value['engine']['worker_threads'] = 2
            write('pipeline.json', value)
        changed, _ = prepare('v0_host')
        assert load(changed, 'resolved.json')['algorithm_contract_sha256'] != expected, change
        for path, value in saved.items():
            write(path, value)

    # Output adapter configuration is forwarded without changing shared algorithms.
    adapter = json.loads(json.dumps(request))
    output_settings = dict(dock_pose_topic='/actual/interface/pose', grid_header_clock='capture_ns',
                           parent_frame='actual_parent', factory_reset_directory='/tmp/factory reset output')
    adapter['deployment']['v0_host'].update(output_settings)
    write('adapter.json', adapter)
    adapted, _ = prepare('v0_host', config='adapter.json')
    adapted_host = load(adapted, 'v0_host.json')
    for key, value in output_settings.items():
        assert adapted_host[key] == value
    assert load(adapted, 'resolved.json')['algorithm_contract_sha256'] == expected
    for key in output_settings:
        assert key not in host, key
    adapter['deployment']['v0_host']['factory_reset_directory'] = ''
    adapter['deployment']['v0_host']['grid_header_clock'] = 'legacy_steady_ms'
    write('adapter-empty-reset-directory.json', adapter)
    adapted_empty, _ = prepare('v0_host', config='adapter-empty-reset-directory.json')
    assert load(adapted_empty, 'v0_host.json')['factory_reset_directory'] == ''
    assert load(adapted_empty, 'resolved.json')['algorithm_contract_sha256'] == expected
    invalid_outputs = [('dock_pose_topic', ''), ('dock_pose_topic', 123),
                       ('grid_header_clock', 'wall_clock'), ('grid_header_clock', True),
                       ('parent_frame', 123), ('parent_frame', ''), ('factory_reset_directory', [])]
    for key, value in invalid_outputs:
        invalid = json.loads(json.dumps(adapter))
        invalid['deployment']['v0_host'][key] = value
        write('invalid-output.json', invalid)
        _, failed = prepare('v0_host', config='invalid-output.json', success=False)
        assert 'deployment.v0_host.' + key in failed.stderr

    # SDK initial command, recording and publication are excluded from algorithm identity.
    publication = json.loads(json.dumps(request))
    publication['tasks']['publish']['enabled'] = False
    write('publication.json', publication)
    changed, _ = prepare('v0_host', config='publication.json')
    assert load(changed, 'resolved.json')['algorithm_contract_sha256'] == expected

    # execv receives exact argv strings; no shell interpretation or implicit ROS launch.
    recorder = fixture / 'record_argv.py'
    recorder.write_text('import json,pathlib,sys\npathlib.Path(sys.argv[1]).write_text(json.dumps(sys.argv[2:]))\n')
    recorded = fixture / 'recorded_argv.json'
    launched = json.loads(json.dumps(request))
    launched['deployment']['v0_host']['launch_argv'] = [
        os.sys.executable, str(recorder), str(recorded), '{host_config}', '{plugin_library}',
        '{run_dir}', 'argument with spaces', '$(no_shell_execution)']
    write('launched.json', launched)
    launched_dir, _ = prepare('v0_host', config='launched.json', run=True)
    actual = json.loads(recorded.read_text())
    assert actual == [str(launched_dir / 'v0_host.json'), str(fixture / 'plugin path/libpercep_main.so'),
                      str(launched_dir), 'argument with spaces', '$(no_shell_execution)']
    launched['deployment']['v0_host']['launch_argv'][-1] = 'v2_host_config={host_config}'
    write('embedded.json', launched)
    _, failed = prepare('v0_host', config='embedded.json', success=False)
    assert 'whole argument' in failed.stderr

    for subcommand in ['camera', 'capture']:
        selected = json.loads(json.dumps(request))
        selected['deployment']['mode'] = 'v0_host'
        write('host-selected.json', selected)
        command = [str(args.launcher.resolve()), subcommand, '--config', str(fixture / 'host-selected.json'),
                   '--prepare-only']
        if subcommand == 'capture':
            command.extend(['--output', str(fixture / 'must-not-capture')])
        result = subprocess.run(command, env=environment, text=True, capture_output=True)
        assert result.returncode == 2 and 'unavailable' in result.stderr
    report = dict(status='passed', evidence='configuration_contract_only',
                  algorithm_contract_sha256=expected, source_config=str(fixture / 'pipeline.json'),
                  standalone_run=str(standalone), host_run=str(board),
                  verified=['same_algorithm_graph', 'same_algorithm_hash', 'sha256_matches_hashlib',
                            'initially_stopped', 'no_camera_ros_offline_input_for_host',
                            'no_unused_offline_data_environment_required', 'threshold_model_calibration_fusion_dag_hash_changes',
                            'publish_excluded_from_algorithm_hash', 'missing_host_argv_rejected',
                            'exact_exec_argv', 'camera_capture_rejected_for_host',
                            'output_adapter_passthrough_validation_and_hash_independence'])
    (fixture / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
