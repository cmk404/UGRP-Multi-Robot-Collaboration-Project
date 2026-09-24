"""Run the three preregistered south RGB teachers through the standard workflow.

This launcher owns only its child process group. It never retries, changes a
case, or writes to the frozen v28 checkout. Run after source/input freeze and
only while no other physics or training owner is active.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
PROTOCOL = ROOT/'experiments/2026-09-24-action-act/route-coverage-protocol.json'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git(repo, *args):
    return subprocess.check_output(['git', '-C', str(repo), *args], text=True).strip()


def assert_source(repo, expected):
    actual = git(repo, 'rev-parse', 'HEAD')
    if actual != expected or git(repo, 'status', '--porcelain'):
        raise ValueError(f'frozen teacher source changed or dirty: {actual}')


def write_new(path, data):
    with path.open('x') as stream:
        json.dump(data, stream, indent=2, allow_nan=False)
        stream.write('\n')


def save(path, data):
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def group_exists(pgid):
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


def effective_wall_cap(remaining, maximum, *, cleanup_reserve=10):
    """Keep the fixed aggregate edge while still allowing a shorter last case."""
    if maximum <= 0 or cleanup_reserve < 0:
        raise ValueError('invalid teacher wall budget')
    return max(0., min(float(maximum), float(remaining)-cleanup_reserve))


def stop_own_group(process):
    if not group_exists(process.pid):
        return False
    for how, seconds in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        try:
            os.killpg(process.pid, how)
        except ProcessLookupError:
            return True
        try:
            process.wait(timeout=seconds)
        except subprocess.TimeoutExpired:
            pass
        if not group_exists(process.pid):
            return True
    if group_exists(process.pid):
        raise RuntimeError(f'owned workflow process group {process.pid} survived cleanup')
    return True


def run_owned(command, *, cwd, environment, log, timeout):
    started = time.monotonic()
    with log.open('x') as stream:
        process = subprocess.Popen(command, cwd=cwd, env=environment,
                                   stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        pid = process.pid
        timed_out = False
        try:
            code = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
            stop_own_group(process)
            code = process.poll()
        finally:
            if group_exists(process.pid):
                stop_own_group(process)
        if process.poll() is None:
            raise RuntimeError(f'owned workflow process {process.pid} survived cleanup')
    return {'pid': pid, 'exit_code': code, 'timed_out': timed_out,
            'wall_s': time.monotonic()-started, 'console_log': str(log),
            'console_sha256': sha(log)}


def input_identity(protocol):
    teacher = protocol['teacher_collection']
    path = Path(teacher['plan_replay'])
    if sha(path) != teacher['plan_replay_sha256']:
        raise ValueError('saved south plan changed')
    checkout = Path(protocol['source_boundaries']['teacher_checkout'])
    reference_path = checkout/'tests/fixtures/camera_goal_transport/reference-top.jpg'
    if sha(reference_path) != teacher['reference_top_sha256']:
        raise ValueError('reference TOP image changed')
    paths = [path, reference_path]
    cli = protocol['teacher_managed_cli']['fixed_trial_args']
    for flag in ('--grasp-model-dir', '--stage-model-dir'):
        directory = Path(cli[cli.index(flag)+1])
        if not directory.is_dir():
            raise ValueError(f'missing {flag} input')
        paths += sorted(p for p in directory.rglob('*') if p.is_file())
    return {str(p.resolve()): sha(p) for p in paths}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record-root', type=Path, required=True,
                        help='fresh root for launcher/manager/raw records')
    args = parser.parse_args()
    protocol = json.loads(PROTOCOL.read_text())
    if protocol['schema'] != 'ugrp.action_act_route_coverage_protocol.v1':
        raise ValueError('unexpected teacher protocol')
    teacher = protocol['teacher_collection']
    source = protocol['source_boundaries']
    checkout = Path(source['teacher_checkout'])
    launcher_source_sha = git(ROOT, 'rev-parse', 'HEAD')
    assert_source(ROOT, launcher_source_sha)
    assert_source(checkout, source['teacher_and_physical_inference_source_sha'])
    before_inputs = input_identity(protocol)
    root = args.record_root.resolve()
    root.mkdir(parents=True, exist_ok=False)
    (root/'raw').mkdir()
    (root/'managed').mkdir()
    (root/'plans').mkdir()
    (root/'logs').mkdir()
    started = time.monotonic()
    record = {
        'schema': 'ugrp.act.route_teacher_launcher.v1',
        'status': 'running',
        'protocol_path': str(PROTOCOL), 'protocol_sha256': sha(PROTOCOL),
        'launcher_source_sha': launcher_source_sha,
        'teacher_source_sha': source['teacher_and_physical_inference_source_sha'],
        'input_sha256_before': before_inputs,
        'aggregate_manager_hard_wall_s': teacher['aggregate_manager_hard_wall_s'],
        'cases': [],
    }
    save(root/'launcher.json', record)
    environment = dict(os.environ)
    environment['UGRP_SIM_PYTHON'] = '/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python'
    environment['OMP_NUM_THREADS'] = '2'
    environment['MKL_NUM_THREADS'] = '2'
    fixed = protocol['teacher_managed_cli']['fixed_trial_args']
    exception = None
    try:
        for case in teacher['cases']:
            remaining = teacher['aggregate_manager_hard_wall_s']-(time.monotonic()-started)
            # Leave ten seconds for owned-child cleanup at the aggregate edge.
            if effective_wall_cap(remaining, teacher['manager_timeout_per_case_s']) <= 0:
                record['cases'].append({'id': case['id'], 'status': 'unstarted_aggregate_cap'})
                continue
            assert_source(checkout, source['teacher_and_physical_inference_source_sha'])
            if input_identity(protocol) != before_inputs:
                raise ValueError('teacher input changed before case')
            raw = root/'raw'/case['id']
            managed = root/'managed'/case['id']
            if raw.exists() or managed.exists():
                raise ValueError('teacher case output already exists')
            trial = fixed + ['--spawn-offset', *[str(v) for v in case['spawn_offset_dx_dy_m_yaw_deg']],
                             '--output', str(raw)]
            plan_command = ['bash', 'scripts/open_simulation.command', 'workflow', 'plan',
                            'dispatch-skills', '--', *trial]
            case_record = {'id': case['id'], 'split': case['split'],
                           'offset': case['spawn_offset_dx_dy_m_yaw_deg'],
                           'raw': str(raw), 'managed': str(managed),
                           'plan_command': plan_command}
            record['cases'].append(case_record)
            save(root/'launcher.json', record)
            print(json.dumps({'starting': case['id'], 'record': str(managed),
                              'raw': str(raw)}), flush=True)
            planned = run_owned(plan_command, cwd=checkout, environment=environment,
                                log=root/'logs'/(case['id']+'-plan.log'),
                                timeout=effective_wall_cap(remaining, 30))
            case_record['plan'] = planned
            if planned['exit_code'] != 0 or planned['timed_out']:
                raise ValueError(f"workflow plan failed for {case['id']}")
            plan_log = Path(planned['console_log'])
            write_new(root/'plans'/(case['id']+'.json'), {
                'command': plan_command, 'console_path': str(plan_log),
                'console_sha256': sha(plan_log)})
            save(root/'launcher.json', record)
            remaining = teacher['aggregate_manager_hard_wall_s']-(time.monotonic()-started)
            effective_cap = effective_wall_cap(remaining, teacher['manager_timeout_per_case_s'])
            if effective_cap <= 0:
                case_record['status'] = 'unstarted_after_plan_aggregate_cap'
                save(root/'launcher.json', record)
                continue
            run_command = ['bash', 'scripts/open_simulation.command', 'workflow', 'run',
                           'dispatch-skills', '--record', str(managed), '--timeout',
                           str(effective_cap), '--', *trial]
            case_record['run_command'] = run_command
            case_record['effective_manager_wall_cap_s'] = effective_cap
            save(root/'launcher.json', record)
            result = run_owned(run_command, cwd=checkout, environment=environment,
                               log=root/'logs'/(case['id']+'-run.log'),
                               timeout=effective_cap)
            case_record['run'] = result
            result_path = raw/'result.json'
            if result_path.is_file():
                case_record['raw_result_sha256'] = sha(result_path)
                outcome = json.loads(result_path.read_text())
                case_record['physical_success'] = bool(outcome.get('physical_success'))
                case_record['protocol_complete'] = bool(outcome.get('protocol_complete'))
            case_record['status'] = ('teacher_success' if result['exit_code'] == 0 and
                                     case_record.get('physical_success') and
                                     case_record.get('protocol_complete') else 'teacher_failed')
            save(root/'launcher.json', record)
            print(json.dumps({'finished': case['id'], 'status': case_record['status'],
                              'exit_code': result['exit_code'], 'wall_s': result['wall_s']}), flush=True)
            assert_source(checkout, source['teacher_and_physical_inference_source_sha'])
            if input_identity(protocol) != before_inputs:
                raise ValueError('teacher input changed during case')
    except BaseException as error:
        exception = {'type': type(error).__name__, 'message': str(error)}
        raise
    finally:
        record['elapsed_wall_s'] = time.monotonic()-started
        try:
            record['input_sha256_after'] = input_identity(protocol)
        except Exception as error:
            record['input_identity_after_error'] = {'type': type(error).__name__, 'message': str(error)}
        try:
            record['source_sha_after'] = git(checkout, 'rev-parse', 'HEAD')
        except Exception as error:
            record['source_identity_after_error'] = {'type': type(error).__name__, 'message': str(error)}
        record['error'] = exception
        record['status'] = ('collection_complete' if exception is None and
                            'input_identity_after_error' not in record and
                            'source_identity_after_error' not in record and
                            record['input_sha256_after'] == before_inputs and
                            record['source_sha_after'] == source['teacher_and_physical_inference_source_sha'] and
                            len(record['cases']) == len(teacher['cases']) and
                            all(c.get('status') == 'teacher_success' for c in record['cases'])
                            else 'collection_incomplete')
        save(root/'launcher.json', record)


if __name__ == '__main__':
    main()
