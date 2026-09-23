import json
from pathlib import Path
from types import SimpleNamespace
import pytest
from scripts.carry_failure_metrics import schedule, outcome, aggregate


def test_planned_repeats_are_unique_and_pending_is_not_a_failure(tmp_path):
    jobs = schedule({'test': [{'id': 'open'}], 'evaluation': {'repeats': 3}}, ['teacher', 'act'])
    assert len({j['trial_id'] for j in jobs}) == 6
    failed = outcome(tmp_path, 1)
    row = {**jobs[0], 'outcome': failed}
    summary = aggregate(jobs, [row])
    assert not summary['complete']
    group = summary['conditions'][row['condition']]
    assert (group['planned'], group['attempted'], group['pending'], group['failures']) == (3, 1, 2, 1)
    assert group['carry_failure_fraction'] is None
    assert group['carry_entry_unknown'] == 1
    with pytest.raises(ValueError, match='duplicate'):
        aggregate(jobs, [row, row])


def test_approach_failure_is_counted_without_inventing_act_failure(tmp_path):
    (tmp_path/'result.json').write_text(json.dumps({'physical_success': False, 'phase': 'APPROACH',
        'protocol_complete': False, 'error': 'fine RGB alignment outside saved skill support'}))
    (tmp_path/'pair-decisions.json').write_text('[]')
    row = outcome(tmp_path, 1)
    assert not row['whole_success'] and not row['carry_entered'] and not row['act_carry_entered']
    assert row['failure_phase'] == 'APPROACH'
    assert row['robot_result_available']


def test_carry_conditional_denominator_excludes_prerequisite_failures(tmp_path):
    jobs = schedule({'test': [{'id': 'open'}], 'evaluation': {'repeats': 3}}, ['act'])
    data = [dict(whole_success=False, robot_result_available=True, failure_kind='execution_error',
                 carry_entered=False, act_carry_entered=False, beam_success=False),
            dict(whole_success=False, robot_result_available=True, failure_kind='task_incomplete',
                 carry_entered=True, act_carry_entered=True, beam_success=False),
            dict(whole_success=True, robot_result_available=True, failure_kind=None,
                 carry_entered=True, act_carry_entered=True, beam_success=True)]
    group = aggregate(jobs, [{**j, 'outcome': r} for j, r in zip(jobs, data)])['conditions']['act']
    assert group['failure_fraction_attempted'] == 2/3
    assert group['carry_failure_fraction'] == 1/2


def test_entire_cohort_finishes_when_every_robot_trial_fails(tmp_path, monkeypatch):
    """Exercise the runner, including failed child processes and all repeat slots."""
    import scripts.run_carry_input_ablation as runner
    import scripts.colab_carry_bundle as bundles
    training = tmp_path/'training/model-r128-h1-s1'
    (training/'act').mkdir(parents=True)
    (training/'act/model.safetensors').write_bytes(b'fixture')
    episode = tmp_path/'F1';episode.mkdir()
    dataset = tmp_path/'dataset.json'
    dataset.write_text(json.dumps({'train': [{'root': str(episode)}]}))
    protocol = {'dataset_sha256': 'data', 'training': {'seeds': [1]},
                'arms': [{'id': 'r128-h1', 'size': 128, 'history': 1}],
                'test': [{'id': 'open', 'teacher_case': 'F1', 'variant': 'open', 'offset': [0,0,0]}],
                'evaluation': {'repeats': 3}, 'controls': {'max_wall_s': 1, 'max_carry_steps': 2, 'workers': 2}}
    plan = tmp_path/'protocol.json';plan.write_text(json.dumps(protocol))
    monkeypatch.setattr(bundles, 'verify_dataset', lambda _: {'dataset_sha256': 'data'})
    monkeypatch.setattr(runner, 'validate_training', lambda *a: {'source_sha': 'training-source', 'wall_s': 1, 'development_metrics': {}})
    monkeypatch.setattr(runner.subprocess, 'check_output', lambda cmd, **kw: '' if cmd[1] == 'status' else 'frozen-source')
    calls = []
    def failed_child(cmd,log,**kw):
        calls.append(cmd)
        output = Path(cmd[cmd.index('--output')+1]);output.mkdir(parents=True)
        (output/'result.json').write_text(json.dumps({'physical_success': False, 'protocol_complete': False,
            'phase': 'APPROACH', 'error': 'fixture approach failure'}))
        (output/'pair-decisions.json').write_text('[]')
        return {'exit_code':1,'timed_out':False}
    monkeypatch.setattr(runner, 'run_logged', failed_child)
    output = tmp_path/'out'
    monkeypatch.setattr(runner.sys, 'argv', ['cohort', '--out', str(output), '--act-python', 'python',
        '--mjpython', 'python', '--grasp', str(tmp_path), '--stages', str(tmp_path),
        '--reuse-training', str(training.parent), '--dataset', str(dataset), '--protocol', str(plan)])
    assert runner.main() == 0
    report = json.loads((output/'report.json').read_text())
    assert report['complete'] and len(calls) == len(report['runs']) == 6
    assert all(c['failures'] == 3 and c['carry_failure_fraction'] is None
               for c in report['failure_estimates']['conditions'].values())
