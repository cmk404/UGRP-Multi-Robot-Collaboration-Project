"""Foreign owner detection and own-process cleanup for bounded teachers."""
import os
import subprocess
import sys

import pytest

from scripts import run_act_route_teachers as launcher


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
        assert not result['timed_out']
        assert result['pid'] != foreign.pid
        assert foreign.poll() is None
        assert result['exit_code'] != 0
    finally:
        if foreign.poll() is None:
            os.killpg(foreign.pid, 15)
        foreign.wait(timeout=5)
