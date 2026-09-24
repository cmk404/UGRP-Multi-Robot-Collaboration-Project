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
import re
import shlex
import signal
import subprocess
import time
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[1]
PROTOCOL = ROOT/'experiments/2026-09-24-action-act/route-coverage-protocol.json'
SECRET_FLAG = re.compile(r'(token|secret|password|credential|api[-_]?key)', re.I)


def safe_command(command):
    try:
        words = shlex.split(command)
    except ValueError:
        words = command.split()
    redacted, hide_next = [], False
    for word in words:
        if hide_next:
            redacted.append('[REDACTED]')
            hide_next = False
        elif word.startswith('--') and '=' in word and SECRET_FLAG.search(word.split('=', 1)[0]):
            redacted.append(word.split('=', 1)[0]+'=[REDACTED]')
        elif word.startswith('--') and SECRET_FLAG.search(word):
            redacted.append(word)
            hide_next = True
        elif '://' in word:
            redacted.append('[URL_REDACTED]')
        else:
            redacted.append(word)
    return shlex.join(redacted)


def _entrypoint(command):
    """Return a process's actual entrypoint and its arguments, not arg text."""
    try:
        words = shlex.split(command)
    except ValueError:
        words = command.split()
    if not words:
        return '', []
    executable = Path(words[0]).name.lower()
    if executable == 'mjpython':
        return 'mjpython', words[1:]
    if executable.startswith('python') or executable in ('python.app',):
        if '-m' in words:
            index = words.index('-m')
            return (words[index+1], words[index+2:]) if index+1 < len(words) else ('', [])
        for index, word in enumerate(words[1:6], start=1):
            if word.endswith('.py'):
                return word, words[index+1:]
        return executable, words[1:]
    if executable in ('bash', 'sh', 'zsh') and len(words) > 1:
        return words[1], words[2:]
    return words[0], words[1:]


def _foreign_reason(command):
    entry, args = _entrypoint(command)
    name = Path(entry).name.lower()
    module = entry.lower()
    if name == 'mjpython':
        return 'native_mjpython'
    if name in ('run_ab.py', 'run_ab') or module.endswith('.run_ab'):
        return 'foreign_cohort_owner'
    if name in ('run_dispatch_e2e.py', 'run_dispatch_skills.py', 'sim_dispatch.py',
                'run_research_dispatch.py', 'dispatch_native_process.py',
                'dispatch_native_view.py') or module in (
                    'scripts.run_dispatch_e2e', 'scripts.run_dispatch_skills',
                    'scripts.sim_dispatch', 'scripts.run_research_dispatch',
                    'scripts.dispatch_native_process', 'scripts.dispatch_native_view'):
        return 'foreign_dispatch_or_observer'
    if name == 'open_simulation.command':
        return 'foreign_simulation_launcher'
    if name == 'ugrp_session.py' and args[:1] == ['run']:
        return 'foreign_ugrp_session'
    if name == 'sim_cli.py' or module == 'scripts.sim_cli':
        if args[:1] in (['dispatch'], ['run'], ['start'], ['console']) or args[:2] == ['workflow', 'run']:
            return 'foreign_sim_cli_execution'
    if (name in ('train_carry_act.py', 'train_carry_input_act.py',
                 'train_dispatch_transfer.py') or
            module in ('scripts.train_carry_act', 'scripts.train_carry_input_act',
                       'scripts.train_dispatch_transfer') or
            name.startswith('train_act_') and name.endswith('.py')):
        return 'foreign_training'
    if name in ('pytest', 'py.test', 'torchrun') or module in ('pytest', 'torch.distributed.run'):
        return 'foreign_test_or_torch_job'
    return None


def parse_process_table(table, own_pid, owned_pgids=()):
    """Identify foreign owners by ancestry and entrypoint, including idle owners."""
    processes = {}
    for line in table.splitlines():
        parts = line.strip().split(None, 3)
        if (len(parts) != 4 or not parts[0].isdigit() or
                not parts[1].isdigit() or not parts[2].isdigit()):
            continue
        pid, ppid, pgid = int(parts[0]), int(parts[1]), int(parts[2])
        processes[pid] = {'pid': pid, 'ppid': ppid, 'pgid': pgid,
                          'command': safe_command(parts[3]),
                          '_raw_command': parts[3]}

    def owned(pid):
        seen = set()
        while pid and pid in processes and pid not in seen:
            if pid == own_pid:
                return True
            seen.add(pid)
            pid = processes[pid]['ppid']
        return False

    owned_groups = set(owned_pgids)
    own_main_group = processes.get(own_pid, {}).get('pgid')
    owned_groups.update(row['pgid'] for pid, row in processes.items()
                        if pid != own_pid and owned(pid) and row['pgid'] != own_main_group)
    return [{key: value for key, value in row.items() if key != '_raw_command'} | {'reason': reason}
            for pid, row in sorted(processes.items())
            if not owned(pid) and row['pgid'] not in owned_groups and
            (reason := _foreign_reason(row['_raw_command']))]


def foreign_processes(owned_pgids=()):
    table = subprocess.check_output(
        ['ps', '-ww', '-axo', 'pid=,ppid=,pgid=,command='], text=True)
    return parse_process_table(table, os.getpid(), owned_pgids)


def foreign_snapshot(owned_pgids=()):
    blockers = foreign_processes(owned_pgids)
    if not blockers:
        return None
    for blocker in blockers:
        try:
            text = subprocess.check_output(
                ['lsof', '-a', '-p', str(blocker['pid']), '-d', 'cwd', '-Fn'],
                text=True, stderr=subprocess.DEVNULL, timeout=.25)
            blocker['cwd'] = next((line[1:] for line in text.splitlines() if line.startswith('n')), None)
        except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
            blocker['cwd'] = None
    return {'observed_utc': datetime.now(timezone.utc).isoformat(),
            'processes': blockers}


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


def managed_caps(remaining, maximum, *, aggregate_cleanup_reserve=10,
                 manager_finalization_reserve=10):
    outer = effective_wall_cap(remaining, maximum,
                               cleanup_reserve=aggregate_cleanup_reserve)
    if manager_finalization_reserve <= 0:
        raise ValueError('positive manager finalization reserve required')
    inner = outer-manager_finalization_reserve
    return outer, max(0., inner)


def managed_run_command(record, trial, inner_timeout):
    if inner_timeout <= 0:
        raise ValueError('cannot start teacher with nonpositive manager timeout')
    return ['bash', 'scripts/open_simulation.command', 'workflow', 'run',
            'dispatch-skills', '--record', str(record), '--timeout',
            str(inner_timeout), '--', *trial]


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


def run_owned(command, *, cwd, environment, log, timeout, poll_interval=1.0):
    if timeout <= 0 or poll_interval <= 0:
        raise ValueError('positive owned workflow timeout and poll interval required')
    started = time.monotonic()
    with log.open('x') as stream:
        process = subprocess.Popen(command, cwd=cwd, env=environment,
                                   stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        pid = process.pid
        timed_out = False
        interference = None
        try:
            while True:
                # Poll only for the lifetime of this bounded collection. An
                # idle foreign cohort owner remains visible between children.
                interference = foreign_snapshot((process.pid,))
                if interference is not None:
                    stop_own_group(process)
                    code = process.poll()
                    break
                left = timeout-(time.monotonic()-started)
                if left <= 0:
                    timed_out = True
                    stop_own_group(process)
                    code = process.poll()
                    break
                try:
                    code = process.wait(timeout=min(poll_interval, left))
                    break
                except subprocess.TimeoutExpired:
                    pass
        finally:
            if group_exists(process.pid):
                stop_own_group(process)
        if process.poll() is None:
            raise RuntimeError(f'owned workflow process {process.pid} survived cleanup')
    return {'pid': pid, 'exit_code': code, 'timed_out': timed_out,
            'wall_s': time.monotonic()-started, 'console_log': str(log),
            'console_sha256': sha(log), 'foreign_interference': interference}


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
    preflight = foreign_snapshot()
    if preflight is not None:
        # No new record root exists yet. Keep the independent owner's process
        # evidence in this launcher's console; never signal foreign PIDs.
        raise RuntimeError('foreign owner blocks teacher preflight: '+json.dumps(preflight))
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
        for case_index, case in enumerate(teacher['cases']):
            external = foreign_snapshot()
            if external is not None:
                record['foreign_interference'] = {'phase': 'before_case', 'case': case['id'],
                                                  **external}
                record['cases'].extend({'id': next_case['id'], 'status': 'unstarted_foreign_interference'}
                                       for next_case in teacher['cases'][case_index:])
                save(root/'launcher.json', record)
                break
            remaining = teacher['aggregate_manager_hard_wall_s']-(time.monotonic()-started)
            # Reserve ten seconds for our cleanup and another ten for the
            # workflow manager's child/tee/manifest finalization.
            outer_cap, inner_cap = managed_caps(remaining, teacher['manager_timeout_per_case_s'])
            if inner_cap <= 0:
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
            if planned['foreign_interference'] is not None:
                case_record['status'] = 'aborted_foreign_interference_during_plan'
                record['foreign_interference'] = {'phase': 'during_plan', 'case': case['id'],
                                                  **planned['foreign_interference']}
                record['cases'].extend({'id': next_case['id'], 'status': 'unstarted_foreign_interference'}
                                       for next_case in teacher['cases'][case_index+1:])
                save(root/'launcher.json', record)
                break
            if planned['exit_code'] != 0 or planned['timed_out']:
                raise ValueError(f"workflow plan failed for {case['id']}")
            plan_log = Path(planned['console_log'])
            write_new(root/'plans'/(case['id']+'.json'), {
                'command': plan_command, 'console_path': str(plan_log),
                'console_sha256': sha(plan_log)})
            save(root/'launcher.json', record)
            external = foreign_snapshot()
            if external is not None:
                case_record['status'] = 'unstarted_after_plan_foreign_interference'
                record['foreign_interference'] = {'phase': 'before_run', 'case': case['id'],
                                                  **external}
                record['cases'].extend({'id': next_case['id'], 'status': 'unstarted_foreign_interference'}
                                       for next_case in teacher['cases'][case_index+1:])
                save(root/'launcher.json', record)
                break
            remaining = teacher['aggregate_manager_hard_wall_s']-(time.monotonic()-started)
            outer_cap, inner_cap = managed_caps(remaining, teacher['manager_timeout_per_case_s'])
            if inner_cap <= 0:
                case_record['status'] = 'unstarted_after_plan_aggregate_cap'
                case_record['effective_outer_wall_cap_s'] = outer_cap
                case_record['effective_manager_wall_cap_s'] = inner_cap
                save(root/'launcher.json', record)
                continue
            run_command = managed_run_command(managed, trial, inner_cap)
            case_record['run_command'] = run_command
            case_record['effective_outer_wall_cap_s'] = outer_cap
            case_record['effective_manager_wall_cap_s'] = inner_cap
            case_record['aggregate_cleanup_reserve_s'] = 10
            case_record['manager_finalization_reserve_s'] = 10
            save(root/'launcher.json', record)
            result = run_owned(run_command, cwd=checkout, environment=environment,
                               log=root/'logs'/(case['id']+'-run.log'),
                               timeout=outer_cap)
            case_record['run'] = result
            result_path = raw/'result.json'
            if result_path.is_file():
                case_record['raw_result_sha256'] = sha(result_path)
                outcome = json.loads(result_path.read_text())
                case_record['physical_success'] = bool(outcome.get('physical_success'))
                case_record['protocol_complete'] = bool(outcome.get('protocol_complete'))
            if result['foreign_interference'] is not None:
                case_record['status'] = 'aborted_foreign_interference_during_run'
                record['foreign_interference'] = {'phase': 'during_run', 'case': case['id'],
                                                  **result['foreign_interference']}
                record['cases'].extend({'id': next_case['id'], 'status': 'unstarted_foreign_interference'}
                                       for next_case in teacher['cases'][case_index+1:])
                save(root/'launcher.json', record)
                break
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
        record['status'] = ('foreign_interference_abort' if 'foreign_interference' in record else
                            'collection_complete' if exception is None and
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
