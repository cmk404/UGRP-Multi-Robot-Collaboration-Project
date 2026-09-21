import json
import subprocess
import sys
from types import SimpleNamespace
import pytest
from harness.model_http import post_with_recovery
from scripts.cloud_progress import PREFIX,parse_line,run_logged,summarize_log
from scripts.cloud_environment_preflight import probe_python,require_ready


def test_temporary_rejection_recovers_without_erasing_first_failure():
    now=[0];calls=[]
    def call(*args):
        calls.append(args)
        return {'status':'http_error','http_status':503,'raw_response':'no healthy upstream'} if len(calls)==1 else {'status':'ok','body':{'ready':True}}
    r=post_with_recovery({},'fixture',timeout=5,call=call,clock=lambda:now[0],sleep=lambda s:now.__setitem__(0,now[0]+s))
    assert r['recovered'] and r['first_attempt_failed'] and r['retry_count']==1
    assert r['http_attempts'][0]['http_status']==503 and len(calls)==2
    assert calls[1][-1]==4.5


@pytest.mark.parametrize('response',[{'status':'transport_error','error_type':'TimeoutError'},
 {'status':'http_error','http_status':401},{'status':'invalid_json','http_status':200}])
def test_uncertain_or_permanent_errors_are_not_replayed(response):
    calls=[]
    r=post_with_recovery({},'fixture',call=lambda *a:calls.append(a) or response)
    assert len(calls)==1 and r['http_attempt_count']==1 and not r['recovered']


def test_recovery_respects_retry_after_deadline_and_global_call_budget():
    calls=[]
    r=post_with_recovery({},'fixture',timeout=1,call=lambda *a:calls.append(a) or
        {'status':'http_error','http_status':529,'retry_after_s':10},sleep=lambda _:pytest.fail('would exceed deadline'))
    assert len(calls)==1
    r=post_with_recovery({},'fixture',admit=lambda:False,call=lambda *a:pytest.fail('budget exhausted'))
    assert r['http_attempt_count']==0


def test_live_progress_crosses_child_log_without_forwarding_raw_text(tmp_path,capsys):
    code='import json;print("private raw log");print('+repr(PREFIX)+'+json.dumps({"schema":"ugrp.progress.v1","unix":10,"event":"evaluation_progress","planned":2,"attempted":1,"failures":1,"pending":1}),flush=True)'
    result=run_logged([sys.executable,'-c',code],tmp_path/'raw.log',name='fixture',timeout=3)
    stdout=capsys.readouterr().out
    assert result['exit_code']==0 and 'private raw log' not in stdout
    assert 'private raw log' in (tmp_path/'raw.log').read_text()
    summary=summarize_log(stdout)
    assert summary['attempted']==1 and summary['failures']==1 and summary['pending']==1
    assert parse_line(PREFIX+'{"schema":"ugrp.progress.v1","unix":10,"secret":"x"}') is None


def test_timeout_is_bounded_and_recorded(tmp_path):
    r=run_logged([sys.executable,'-c','import time;time.sleep(30)'],tmp_path/'timeout.log',name='fixture',timeout=.05)
    assert r['timed_out'] and r['exit_code']!=0


def test_missing_startup_dependency_blocks_even_with_zero_exit(monkeypatch):
    monkeypatch.setattr(subprocess,'run',lambda *a,**k:SimpleNamespace(returncode=0,stdout='{}',stderr="Error in sitecustomize; ModuleNotFoundError: wrapt"))
    report=probe_python('fixture',['numpy'])
    assert not report['ready'] and report['startup_error']
    with pytest.raises(RuntimeError,match='simulation not started'):require_ready(report)


def test_progress_distinguishes_uninstrumented_and_stale():
    assert summarize_log('running')['attempted'] is None
    line=PREFIX+json.dumps({'schema':'ugrp.progress.v1','unix':10,'event':'phase'})
    assert summarize_log(line,now=200)['visibility']=='stale'


def test_collector_keeps_trying_after_three_transient_errors(tmp_path,monkeypatch):
    import scripts.collect_kaggle_job as collector
    now=[0];attempts=[]
    def status(_):
        attempts.append(1)
        if len(attempts)<=3:raise OSError('temporary transport failure')
        return 'complete'
    def collect(_):
        (tmp_path/'verified-result.json').write_text('{}');return 0
    monkeypatch.setattr(collector,'status',status);monkeypatch.setattr(collector,'collect',collect)
    monkeypatch.setattr(collector.time,'monotonic',lambda:now[0])
    monkeypatch.setattr(collector.time,'sleep',lambda s:now.__setitem__(0,now[0]+s))
    monkeypatch.setattr(collector.sys,'argv',['collect','--output',str(tmp_path),'--seconds','120'])
    assert collector.main()==0 and len(attempts)==4
    assert json.loads((tmp_path/'collection-status.json').read_text())['collection_degraded'] is False


def test_failed_environment_prevents_calibration_or_robot_launch(tmp_path,monkeypatch):
    import scripts.run_kaggle_carry_recovery as runner
    monkeypatch.setattr(runner,'inspect_environment',lambda **k:{'ready':False})
    monkeypatch.setattr(runner,'configure',lambda *a:pytest.fail('renderer must not start'))
    monkeypatch.setattr(runner.sys,'argv',['run','--output',str(tmp_path/'out'),'--data-sha','x','--assets-sha','y'])
    with pytest.raises(RuntimeError,match='simulation not started'):runner.main()


def test_gpu_preflight_cannot_be_used_to_launch_simulation(tmp_path):
    from scripts.kaggle_simulation_cli import prepare
    with pytest.raises(ValueError,match='never a simulation'):
        prepare(tmp_path/'out','test',gpu_preflight=True,module='scripts.sim_quickstart')


def test_model_gate_requires_fresh_matching_providers_and_all_six_samples():
    from scripts.model_connectivity_preflight import MODELS,validate_report
    report={'ready':True,'models':MODELS,'logical_requests':6,'finished_unix':100,
            'rows':[{'provider':provider,'valid':True} for provider in MODELS for _ in range(3)]}
    assert validate_report(report,now=400)
    assert not validate_report(report,now=401)
    assert not validate_report(report,now=99)
    report['rows'][0]['provider']='unknown'
    assert not validate_report(report,now=101)
