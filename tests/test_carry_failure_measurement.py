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


@pytest.mark.parametrize('low_disk', [False, True])
def test_cohort_keeps_failures_but_does_not_count_disk_skips(tmp_path, monkeypatch, low_disk):
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
                'evaluation': {'repeats': 3}, 'controls': {'max_wall_s': 1, 'max_carry_steps': 2, 'workers': 2,
                                                       'min_free_gib': 8}}
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
    monkeypatch.setattr(runner.shutil, 'disk_usage', lambda _: SimpleNamespace(free=(7 if low_disk else 9)*2**30))
    output = tmp_path/'out'
    monkeypatch.setattr(runner.sys, 'argv', ['cohort', '--out', str(output), '--act-python', 'python',
        '--mjpython', 'python', '--grasp', str(tmp_path), '--stages', str(tmp_path),
        '--reuse-training', str(training.parent), '--dataset', str(dataset), '--protocol', str(plan)])
    assert runner.main() == (2 if low_disk else 0)
    report = json.loads((output/'report.json').read_text())
    if low_disk:
        assert not report['complete'] and not calls and not report['runs']
        assert len(report['not_attempted']) == 6
        assert all(c['pending'] == 3 and c['failures'] == 0
                   for c in report['failure_estimates']['conditions'].values())
        return
    assert report['complete'] and len(calls) == len(report['runs']) == 6
    assert all(cmd[cmd.index('--carry-max-steps')+1]==2 for cmd in calls)
    assert all(c['failures'] == 3 and c['carry_failure_fraction'] is None
               for c in report['failure_estimates']['conditions'].values())


@pytest.mark.parametrize('reference_ok',[True,False])
def test_revalidation_requires_reference_success_before_candidate(tmp_path,monkeypatch,reference_ok):
    import scripts.run_carry_revalidation as runner
    case={'id':'open-minus','variant':'open','offset':[0,0,0],'teacher_case':'F1'}
    jobs=[{'case':'open-minus','condition':name,'calibration':'original'}
          for name in ('RGB-original','ACT-original-seed18','RGB-turn')]
    regression={**case,'case':'open-plus','condition':'candidate','calibration':'native'}
    protocol={'controls':{'min_free_gib':8,'workers':2,'max_wall_s':2400,
                         'max_carry_steps':900,'video_fps':4,'process_timeout_s':2460},
              'original_act_model':str(tmp_path),'original_act_model_sha256':'hash',
              'candidate_act_model':str(tmp_path),'candidate_act_model_sha256':'hash',
              'original_calibration':str(tmp_path),'original_calibration_sha256':{},
              'native_calibration':str(tmp_path),'teacher_roots':{'F1':str(tmp_path)},
              'cases':[case],'initial_jobs':jobs,'release_regression':regression}
    plan=tmp_path/'protocol.json';plan.write_text(json.dumps(protocol));out=tmp_path/'out'
    monkeypatch.setattr(runner,'sha',lambda p:'hash')
    monkeypatch.setattr(runner.subprocess,'check_output',lambda cmd,**kw:'' if cmd[1]=='status' else 'source')
    monkeypatch.setattr(runner.shutil,'disk_usage',lambda p:SimpleNamespace(free=9*2**30))
    commands=[]
    def child(cmd,log,**kw):
        commands.append(cmd);output=Path(cmd[cmd.index('--output')+1]);output.mkdir(parents=True)
        ok=reference_ok and output.parent.name!='candidate'
        (output/'result.json').write_text(json.dumps({'physical_success':ok,'protocol_complete':ok,
                                                     'phase':'FINISHED','error':None}))
        return {'exit_code':0,'timed_out':False,'wall_s':1}
    monkeypatch.setattr(runner,'run_logged',child)
    monkeypatch.setattr(runner.sys,'argv',['runner','--out',str(out),'--protocol',str(plan),
                                        '--act-python','python','--mjpython','python'])
    assert runner.main()==(0 if reference_ok else 1)
    report=json.loads((out/'report.json').read_text())
    assert report['complete']==reference_ok
    assert len(commands)==(4 if reference_ok else 2)
    assert all(c[c.index('--max-wall-s')+1]==2400 and c[c.index('--carry-max-steps')+1]==900 for c in commands)


@pytest.mark.parametrize('error', ['RuntimeError: skill wall budget exhausted',
                                  'RuntimeError: ACT carry decision budget exhausted'])
def test_controller_budget_failure_is_classified_as_timeout(tmp_path, error):
    (tmp_path/'result.json').write_text(json.dumps({'physical_success':False,
        'protocol_complete':False,'phase':'TRANSIT','error':error}))
    (tmp_path/'pair-decisions.json').write_text(json.dumps([{'kind':'act_carry'}]))
    row=outcome(tmp_path,1)
    assert row['failure_kind']=='timeout' and row['act_carry_entered']
    assert row['error']==error
