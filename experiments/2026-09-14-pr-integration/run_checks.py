#!/usr/bin/env python3
"""Recheck committed evidence and nine fixed integration regression cases."""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import time
import zipfile

ROOT = Path(__file__).resolve().parents[2]


def read(path):
    return json.loads(path.read_text())


def write(path, data):
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--python', type=Path, required=True)
    parser.add_argument('--mjpython', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT).strip():
        raise RuntimeError('commit protocol and source before running')
    source = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    python, mjpython = str(args.python.absolute()), str(args.mjpython.absolute())
    environment = {**os.environ, 'PYTHONPATH': str(ROOT)}
    commands = []

    def execute(name, command):
        started = time.monotonic()
        command = list(map(str, command))
        with (out / (name + '.log')).open('w') as log:
            proc = subprocess.run(command, cwd=ROOT, env=environment, stdout=log, stderr=subprocess.STDOUT)
        commands.append({'name': name, 'argv': command, 'exit_code': proc.returncode,
                         'wall_s': time.monotonic() - started})
        write(out / 'commands.json', commands)
        print(f'{name}: exit {proc.returncode}', flush=True)
        return proc.returncode

    write(out / 'environment.json', {'source_sha': source, 'platform': platform.platform(),
        'python': platform.python_version(), 'python_executable': python, 'mjpython': mjpython,
        'packages': {d.metadata['Name']: d.version for d in importlib.metadata.distributions()
                     if any(n in d.metadata['Name'].lower() for n in ('mujoco', 'numpy', 'opencv', 'pillow'))}})
    model_root = ROOT / 'experiments/2026-09-10-rgb-varied-start'
    manifest = read(model_root / 'models-manifest.json')
    archive = model_root / 'models.zip'
    assert hashlib.sha256(archive.read_bytes()).hexdigest() == manifest['archive_sha256']
    with zipfile.ZipFile(archive) as zipped:
        for rec in manifest['files']:
            data = zipped.read(rec['archive_path'])
            assert hashlib.sha256(data).hexdigest() == rec['sha256']
            dest = (out / rec['archive_path']).resolve()
            assert dest.is_relative_to(out)
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(data)
    grasp, straight, varied = [out / 'models' / n for n in ('grasp', 'straight', 'varied')]
    transport = ROOT / 'experiments/2026-09-13-rgb-short-transport/models'
    pair = ROOT / 'experiments/2026-09-13-pair-carry-sync'
    restore = execute('restore', [python, pair / 'restore_evidence.py', out / 'restored'])
    if restore:
        raise RuntimeError('evidence restoration failed')
    saved_audit = execute('saved-audit', [python, pair / 'audit_restored.py',
        '--evidence-dir', out / 'restored', '--grasp-model-dir', grasp, '--out', out / 'saved-audit.json'])
    results = []

    def trial(name, module, flags, audit_module, audit_flags, expected):
        run_dir, audit_path = out / name, out / (name + '-audit.json')
        rc = execute(name, [mjpython, '-m', 'scripts.' + module, *flags, '--out-dir', run_dir])
        if audit_module == 'audit_known_map_navigation':
            arc = execute(name + '-audit', [python, '-m', 'scripts.' + audit_module, run_dir])
            if arc == 0:
                audit_path.write_text((out / (name + '-audit.log')).read_text())
        else:
            arc = execute(name + '-audit', [python, '-m', 'scripts.' + audit_module,
                '--run-dir', run_dir, *audit_flags, '--out', audit_path])
        result = read(run_dir / 'result.json') if (run_dir / 'result.json').exists() else {}
        audit = read(audit_path) if audit_path.exists() else {}
        key = 'actor_status' if isinstance(expected, str) else 'success'
        audit_ok = any(audit.get(k) is True for k in ('ok', 'audit_pass', 'success'))
        passed = result.get(key) == expected and arc == 0 and audit_ok
        passed = passed and (rc == 0 or (expected is False and 'policy stopped:' in str(result.get('error'))))
        results.append({'case': name, 'expected': expected, 'actual': result.get(key),
            'run_exit': rc, 'audit_exit': arc, 'audit_pass': audit_ok, 'regression_pass': passed,
            'result_path': str(run_dir / 'result.json'), 'audit_path': str(audit_path)})
        write(out / 'progress.json', results)

    trial('straight-heldout-01', 'run_camera_approach_student',
        ['--grasp-model-dir', grasp, '--approach-model-dir', straight, '--condition', 'visual', '--distance', '.2062', '.2062'],
        'audit_camera_approach_student', ['--grasp-model-dir', grasp, '--approach-model-dir', straight], True)
    varied_case = out / 'varied-case.json'
    write(varied_case, read(model_root / 'heldout-cases.json')['cases'][0])
    flags = ['--grasp-model-dir', grasp, '--stage-model-dir', varied, '--straight-model-dir', straight]
    trial('varied-heldout-01', 'run_camera_varied_start_student', [*flags, '--condition', 'visual', '--case-json', varied_case],
        'audit_camera_varied_start_student', flags, True)
    cases = {c['case_id']: c for c in read(pair / 'final-cases.json')['cases']}
    flags = ['--grasp-model-dir', grasp, '--transport-model-dir', transport]
    for case_id, condition, expected in [('fixed-01', 'sync', True),
            ('departure-r1-100', 'baseline', False), ('departure-r1-100', 'sync', True),
            ('midcarry-r3-075', 'sync', True), ('report-r3-120', 'sync', True)]:
        path = out / (case_id + '.json')
        write(path, cases[case_id])
        trial(case_id + '-' + condition, 'run_pair_carry_sync',
            [*flags, '--case-json', path, '--condition', condition], 'audit_pair_carry_sync', flags, expected)
    nav_case = json.dumps({'case_id': 'heldout_r1', 'robot_id': 'r1', 'start_xy_m': [-.48, -2.50], 'start_yaw_deg': 12})
    for map_id, expected in [('slalom', 'arrived'), ('narrow', 'no_map_route')]:
        trial(map_id + '-heading', 'run_known_map_navigation',
            ['--map-file', ROOT / 'maps/navigation' / (map_id + '.json'), '--case-json', nav_case,
             '--condition', 'map', '--motion-style', 'heading', '--max-steps', '400'],
            'audit_known_map_navigation', [], expected)
    replay = read(out / 'saved-audit.json')
    passed = saved_audit == 0 and replay['audits_passed'] == 55 and replay['physical_successes_recomputed'] == 49
    passed = passed and all(r['regression_pass'] for r in results)
    assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == source
    assert not subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT).strip()
    summary = {'source_sha': source, 'saved_audits_passed': replay['audits_passed'],
        'saved_physical_successes': replay['physical_successes_recomputed'], 'cases': results,
        'pass': passed, 'scope': 'Nine previously evaluated regression cases; no new generalization cohort or LLM calls.'}
    write(out / 'summary.json', summary)
    print(json.dumps(summary), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
