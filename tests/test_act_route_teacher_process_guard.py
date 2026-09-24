"""Foreign owner detection and own-process cleanup for bounded teachers."""
import os
import json
import subprocess
import sys

import pytest

from scripts import run_act_route_teachers as launcher


def _stock_pytest(tmp_path):
    stock = tmp_path/'stock'
    (stock/'tests').mkdir(parents=True)
    (stock/'tests'/'test_bot.py').write_text('def test_bot(): pass\n')
    subprocess.run(['git', 'init', '-q', str(stock)], check=True)
    subprocess.run(['git', '-C', str(stock), 'remote', 'add', 'origin',
                    'https://github.com/example/stock.git'], check=True)
    row = launcher.parse_process_table(
        '8433 1 8433 python3 -m pytest tests/test_bot.py -q', own_pid=99)[0]
    return stock, {'observed_utc': '2026-09-24T00:00:00+00:00',
                   'processes': [{**row, 'cwd': str(stock)}]}


def test_actual_entrypoints_block_but_shell_literals_and_own_children_do_not():
    table = """\
100 1 100 python3 experiments/2026-09-24-fine-gain-schedule/run_ab.py
101 100 101 /opt/homebrew/bin/Python -m scripts.sim_cli dispatch --variant open
102 1 102 python -m scripts.sim_cli workflow run dispatch-skills
103 1 103 python -m scripts.run_dispatch_e2e --variant open
104 1 104 /usr/local/bin/mjpython -m scripts.sim_cli run
105 1 105 python -m scripts.train_carry_input_act --steps 8000
106 1 106 python -m pytest -q tests
200 1 200 python scripts/run_act_route_teachers.py --record-root /tmp/record
201 200 201 python -m scripts.sim_cli workflow run dispatch-skills
202 201 201 /usr/local/bin/mjpython -m scripts.sim_cli dispatch
203 1 201 python -m scripts.sim_cli dispatch
300 1 300 /bin/zsh -lc rg 'run_ab.py|scripts.sim_cli dispatch'
301 300 300 rg run_ab.py
302 1 302 python -m scripts.sim_cli workflow plan dispatch-skills
"""
    found = launcher.parse_process_table(table, own_pid=200)
    assert {row['pid'] for row in found} == {100, 101, 102, 103, 104, 105, 106}
    assert next(row for row in found if row['pid'] == 100)['reason'] == 'foreign_cohort_owner'
    assert next(row for row in found if row['pid'] == 101)['pgid'] == 101
    assert not launcher.parse_process_table('203 1 201 python -m scripts.sim_cli dispatch',
                                            own_pid=200, owned_pgids=(201,))
    assert 'private-key' not in launcher.safe_command(
        'python -m scripts.sim_cli dispatch --api-key private-key')


def test_foreign_owner_between_children_blocks_before_new_record(tmp_path, monkeypatch):
    fresh = tmp_path/'uncreated'
    sample = {'observed_utc': '2026-09-24T00:00:00+00:00',
              'processes': [{'pid': 83073, 'ppid': 1, 'pgid': 83073,
                             'command': 'python experiments/run_ab.py',
                             'reason': 'foreign_cohort_owner', 'cwd': '/tmp/foreign'}]}
    monkeypatch.setattr(launcher, 'git', lambda *_: 'clean-source-sha')
    monkeypatch.setattr(launcher, 'assert_source', lambda *_: None)
    monkeypatch.setattr(launcher, 'input_identity', lambda *_: {})
    monkeypatch.setattr(launcher, 'foreign_snapshot', lambda *_: sample)
    monkeypatch.setattr(sys, 'argv', ['run_act_route_teachers.py', '--record-root', str(fresh)])
    with pytest.raises(RuntimeError, match='foreign owner blocks teacher preflight'):
        launcher.main()
    assert not fresh.exists()


def test_midrun_foreign_owner_aborts_only_own_group(tmp_path, monkeypatch):
    foreign = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'],
                               start_new_session=True)
    sample = {'observed_utc': '2026-09-24T00:00:00+00:00',
              'processes': [{'pid': foreign.pid, 'ppid': 1, 'pgid': foreign.pid,
                             'command': 'python experiments/run_ab.py',
                             'reason': 'foreign_cohort_owner', 'cwd': None}]}
    polls = iter((None, sample))
    monkeypatch.setattr(launcher, 'foreign_snapshot', lambda *_: next(polls, sample))
    try:
        result = launcher.run_owned(
            [sys.executable, '-c', 'import time; time.sleep(30)'],
            cwd=tmp_path, environment=dict(os.environ), log=tmp_path/'owned.log',
            timeout=5, poll_interval=.05)
        assert result['foreign_interference'] == sample
        assert 'observed_external_test_events' not in result
        assert not result['timed_out']
        assert result['pid'] != foreign.pid
        assert foreign.poll() is None
        assert result['exit_code'] != 0
    finally:
        if foreign.poll() is None:
            os.killpg(foreign.pid, 15)
        foreign.wait(timeout=5)


def test_teacher_data_records_verified_stock_pytest_but_strict_still_blocks(tmp_path):
    stock, snapshot = _stock_pytest(tmp_path)
    strict, strict_events = launcher.classify_host_load(snapshot)
    assert strict == snapshot and strict_events == []
    blockers, events = launcher.classify_host_load(snapshot, policy='teacher-data')
    assert blockers is None
    assert len(events) == 1
    assert events[0]['pid'] == 8433 and events[0]['cwd'] == str(stock)
    assert events[0]['classification'] == 'verified_non_ugrp_pytest'
    assert events[0]['timing_eligible'] is False


@pytest.mark.parametrize('change', [
    {'cwd': None},
    {'cwd': '/missing/external-project'},
    {'command': 'pytest tests/test_bot.py -q'},
    {'command': 'python3 -m torch.distributed.run train.py'},
    {'command': 'python3 -m pytest /Users/changmin/projects/ugrp/tests/test_harness.py -q'},
    {'command': 'python3 -m pytest --pyargs ugrp'},
])
def test_teacher_data_fails_closed_for_unverified_or_ugrp_tests(tmp_path, change):
    _, snapshot = _stock_pytest(tmp_path)
    snapshot['processes'][0].update(change)
    blocker, events = launcher.classify_host_load(snapshot, policy='teacher-data')
    assert blocker == snapshot and events == []


def test_teacher_data_mixed_snapshot_keeps_foreign_physics_blocker(tmp_path):
    _, snapshot = _stock_pytest(tmp_path)
    physics = launcher.parse_process_table(
        '9000 1 9000 python3 experiments/2026-09-24-fine-gain-schedule/run_ab.py',
        own_pid=99)[0]
    snapshot['processes'].append({**physics, 'cwd': None})
    blocker, events = launcher.classify_host_load(snapshot, policy='teacher-data')
    assert [row['pid'] for row in blocker['processes']] == [9000]
    assert [row['pid'] for row in events] == [8433]


def test_teacher_data_rejects_ugrp_checkout_and_unknown_repository(tmp_path):
    _, snapshot = _stock_pytest(tmp_path)
    for name, origin in [('ugrp', 'https://github.com/example/stock.git'),
                         ('other', 'https://github.com/example/ugrp.git')]:
        repo = tmp_path/name
        (repo/'tests').mkdir(parents=True)
        (repo/'tests'/'test_bot.py').write_text('def test_bot(): pass\n')
        subprocess.run(['git', 'init', '-q', str(repo)], check=True)
        subprocess.run(['git', '-C', str(repo), 'remote', 'add', 'origin', origin], check=True)
        snapshot['processes'][0]['cwd'] = str(repo)
        blocker, events = launcher.classify_host_load(snapshot, policy='teacher-data')
        assert blocker == snapshot and events == []
    no_origin = tmp_path/'plain'
    (no_origin/'tests').mkdir(parents=True)
    (no_origin/'tests'/'test_bot.py').write_text('def test_bot(): pass\n')
    snapshot['processes'][0]['cwd'] = str(no_origin)
    blocker, events = launcher.classify_host_load(snapshot, policy='teacher-data')
    assert blocker == snapshot and events == []


def test_teacher_data_owned_job_continues_and_records_every_observed_test(tmp_path, monkeypatch):
    _, snapshot = _stock_pytest(tmp_path)
    monkeypatch.setattr(launcher, 'foreign_snapshot', lambda *_: snapshot)
    result = launcher.run_owned(
        [sys.executable, '-c', 'import time; time.sleep(.15)'],
        cwd=tmp_path, environment=dict(os.environ), log=tmp_path/'owned.log',
        timeout=2, poll_interval=.05, host_load_policy='teacher-data')
    assert result['exit_code'] == 0 and result['foreign_interference'] is None
    assert len(result['observed_external_test_events']) >= 1
    assert all(row['pid'] == 8433 for row in result['observed_external_test_events'])


def test_teacher_data_cli_records_policy_and_preflight_event(tmp_path, monkeypatch):
    _, snapshot = _stock_pytest(tmp_path)
    checkout = tmp_path/'teacher_checkout'
    checkout.mkdir()
    protocol = tmp_path/'protocol.json'
    protocol.write_text(json.dumps({
        'schema': 'ugrp.action_act_route_coverage_protocol.v1',
        'teacher_collection': {'aggregate_manager_hard_wall_s': 30, 'cases': []},
        'teacher_managed_cli': {'fixed_trial_args': []},
        'source_boundaries': {'teacher_checkout': str(checkout),
                              'teacher_and_physical_inference_source_sha': 'source'},
    }))
    monkeypatch.setattr(launcher, 'PROTOCOL', protocol)
    monkeypatch.setattr(launcher, 'git', lambda *_: 'source')
    monkeypatch.setattr(launcher, 'assert_source', lambda *_: None)
    monkeypatch.setattr(launcher, 'input_identity', lambda *_: {})
    monkeypatch.setattr(launcher, 'foreign_snapshot', lambda *_: snapshot)
    output = tmp_path/'record'
    monkeypatch.setattr(sys, 'argv', ['run_act_route_teachers.py', '--record-root',
                                     str(output), '--host-load-policy', 'teacher-data'])
    launcher.main()
    record = json.loads((output/'launcher.json').read_text())
    assert record['runtime_policy']['host_load_policy'] == 'teacher-data'
    assert record['not_speedbenchmark'] is True
    assert record['timing_eligible'] is False
    assert record['observed_external_test_events'][0]['phase'] == 'preflight'
    assert record['observed_external_test_events'][0]['pid'] == 8433
