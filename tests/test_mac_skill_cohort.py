import json
import sys
from pathlib import Path

import pytest

from scripts.cloud_collection import write_json
from scripts.cloud_progress import run_logged
from scripts.run_mac_skill_cohort import evaluate


def protocol():
    return {'source_sha': 'frozen', 'phase': 'holdout', 'budget_s': 100,
            'limits': {'max_wall_s': 20},
            'jobs': [{'trial_id': p, 'policy': p, 'case': 'fixture', 'repeat': 0, 'arm': 'full'}
                     for p in ('rule', 'jev', 'gemini')]}


def test_task_failure_and_crashed_worker_continue_without_retry(tmp_path):
    p = protocol()
    calls = []
    def launch(command, log_path, **kwargs):
        job = p['jobs'][int(command[-1])]
        calls.append(job['policy'])
        assert 'private-key' not in str(command)
        assert kwargs['stdin_text'] == (json.dumps({'key': 'private-key'}) if job['policy'] == 'jev' else None)
        if job['policy'] == 'jev':
            return {'exit_code': -9, 'timed_out': True, 'wall_s': 20}
        trial = tmp_path/job['trial_id']
        trial.mkdir()
        write_json(trial/'result.json', {'source_sha': 'frozen', **job,
                   'success': job['policy'] == 'gemini', 'error': None, 'stop_reason': 'sim_budget'})
        return {'exit_code': 0, 'timed_out': False, 'wall_s': 1}
    result = evaluate(tmp_path, p, 'private-key', 'fixture', launch=launch, frozen=lambda: 'frozen')
    assert calls == ['rule', 'jev', 'gemini']
    assert result['complete'] and result['pending'] == 0
    assert result['by_policy']['rule']['task_failures'] == 1
    assert result['by_policy']['jev']['infrastructure_failures'] == 1
    assert result['by_policy']['gemini']['successes'] == 1
    assert len(list((tmp_path/'checkpoints').glob('*.zip'))) == 3
    with pytest.raises(FileExistsError):
        evaluate(tmp_path, p, 'private-key', 'fixture', launch=launch, frozen=lambda: 'frozen')


def test_deadline_and_source_change_leave_unattempted_trials_pending(tmp_path):
    ticks = iter([0, 101])
    result = evaluate(tmp_path, protocol(), None, 'fixture', clock=lambda: next(ticks),
                      launch=lambda *a, **k: pytest.fail('deadline reached'))
    assert result['pending'] == 3 and result['attempted'] == 0
    assert result['stop_reason'] == 'cohort_deadline' and not result['complete']
    result = evaluate(tmp_path, protocol(), None, 'fixture', frozen=lambda: 'changed',
                      launch=lambda *a, **k: pytest.fail('source changed'))
    assert result['pending'] == 3 and result['stop_reason'] == 'source_changed'


def test_secret_goes_only_through_stdin(tmp_path, capsys):
    code = 'import json,sys;v=json.load(sys.stdin);assert len(v["key"])==11;print("received")'
    result = run_logged([sys.executable, '-c', code], tmp_path/'log', name='fixture',
                        timeout=3, stdin_text=json.dumps({'key': 'private-key'}))
    assert result['exit_code'] == 0
    assert 'private-key' not in (tmp_path/'log').read_text()+capsys.readouterr().out


def test_bad_result_preserved_and_marked_as_infrastructure_failure(tmp_path):
    p = protocol()
    p['jobs'] = p['jobs'][:1]
    def launch(*a, **k):
        trial = tmp_path/'rule'
        trial.mkdir()
        (trial/'result.json').write_text('{broken')
        return {'exit_code': 0, 'timed_out': False}
    result = evaluate(tmp_path, p, None, 'fixture', launch=launch, frozen=lambda: 'frozen')
    assert result['complete'] and result['by_policy']['rule']['infrastructure_failures'] == 1
    assert (tmp_path/'rule/invalid-result.raw').read_text() == '{broken'
