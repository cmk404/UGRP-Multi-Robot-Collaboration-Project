import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from scripts.colab_live_contents import LiveContentsClient, RuntimeAssignmentMissing
from scripts.colab_job_lifecycle import AssignmentLoss,cleanup_ready
from scripts.run_frozen_skill_resume import remaining_protocol


class Session(SimpleNamespace):
    def model_copy(self, update):
        return Session(**{**vars(self), **update})


def assignment(endpoint='owned', token='fresh', ttl=20):
    return SimpleNamespace(endpoint=endpoint, runtime_proxy_info=SimpleNamespace(
        token=token, url='https://runtime.example', token_expires_in_seconds=ttl))


def test_contents_refreshes_same_runtime_before_expiry_without_reallocating():
    now=[0];created=[]
    def factory(session):
        client=Mock();client.download.return_value=session.token;created.append(session);return client
    assignments=Mock(side_effect=[[assignment(token='one')],[assignment(token='two')]])
    c=LiveContentsClient(Session(endpoint='owned',token='old',url='https://runtime.example'),
                         assignments=assignments,factory=factory,clock=lambda:now[0])
    assert c.download('/state.json','local')=='one'
    now[0]=9;assert c.download('/state.json','local')=='one'
    now[0]=11;assert c.download('/state.json','local')=='two'
    assert assignments.call_count==2
    assert all(s.endpoint=='owned' for s in created)
    assert c.last_operation=={'method':'download','path':'/state.json'}


def test_missing_owned_assignment_never_switches_to_another_runtime():
    c=LiveContentsClient(Session(endpoint='owned',token='old'),
        assignments=lambda:[assignment(endpoint='someone-else')],factory=Mock())
    with pytest.raises(RuntimeAssignmentMissing):c.list_dir('/content')


def test_stale_proxy_404_retries_once_with_fresh_token():
    now=[0];bad=Mock();bad.download.side_effect=FileNotFoundError('missing');good=Mock()
    factory=Mock(side_effect=[Mock(),bad,good])
    c=LiveContentsClient(Session(endpoint='owned',token='old'),assignments=lambda:[assignment(ttl=100)],
        factory=factory,clock=lambda:now[0])
    c.refresh();now[0]=6;c.download('/state.json','local')
    assert c.refresh_count==2;assert good.download.call_count==1


def test_resume_keeps_failed_completed_trials_and_every_unfinished_job():
    protocol={'jobs':[{'trial_id':s} for s in ['success','failed','unfinished']], 'source_sha':'frozen'}
    result=remaining_protocol(protocol,['success','failed'])
    assert result['jobs']==[{'trial_id':'unfinished'}]
    assert len(protocol['jobs'])==3
    assert result['source_sha']=='frozen'
    with pytest.raises(ValueError):remaining_protocol(protocol,['unknown'])
    with pytest.raises(ValueError):remaining_protocol(protocol,['success','success'])


def test_collector_survives_three_missing_files_then_collects(monkeypatch,tmp_path):
    import sys
    import scripts.collect_colab_cohort as collector
    import scripts.colab_live_contents as live
    client=Mock();client.refresh_count=1;client.last_operation={'method':'download','path':'/state-model-v4.json'}
    attempts=[0]
    def download(remote,local):
        if remote.endswith('state-model-v4.json'):
            attempts[0]+=1
            if attempts[0]<=3:raise FileNotFoundError('transient missing runtime state')
        from pathlib import Path
        Path(local).write_text(json.dumps({'source_sha':'sha','complete':True,'phase':'holdout','exports':{}}))
    client.download.side_effect=download;client.list_dir.side_effect=FileNotFoundError()
    monkeypatch.setattr(live,'LiveContentsClient',lambda _:client)
    monkeypatch.setitem(sys.modules,'requests',SimpleNamespace(request=Mock()))
    monkeypatch.setitem(sys.modules,'colab_cli.state',SimpleNamespace(StateStore=lambda:SimpleNamespace(get=lambda _:None)))
    monkeypatch.setattr(sys,'argv',['collector','--session','test','--remote','/content/test',
        '--source-sha','sha','--output',str(tmp_path/'collected'),'--seconds','100'])
    now=[0]
    monkeypatch.setattr(collector.time,'monotonic',lambda:now[0])
    def sleep(seconds):
        state=json.loads((tmp_path/'collected/collection-status.json').read_text())
        assert not cleanup_ready(deadline_reached=False,remote_complete=False,collection_complete=state.get('complete',False))
        now[0]+=seconds
    monkeypatch.setattr(collector.time,'sleep',sleep)
    assert collector.main()==0
    state=json.loads((tmp_path/'collected/collection-status.json').read_text())
    assert attempts[0]==4 and state['collection_complete']
    assert state['collection_degraded'] is False
    assert not state['cohort_complete']


def test_cleanup_requires_both_completion_checks_or_explicit_deadline():
    assert not cleanup_ready(deadline_reached=False,remote_complete=True,collection_complete=False)
    assert not cleanup_ready(deadline_reached=False,remote_complete=False,collection_complete=True)
    assert cleanup_ready(deadline_reached=False,remote_complete=True,collection_complete=True)
    assert cleanup_ready(deadline_reached=True,remote_complete=False,collection_complete=False)


def test_assignment_loss_requires_repeated_authoritative_absence():
    loss=AssignmentLoss()
    assert not loss.observe(False,0)
    assert not loss.observe(False,30)
    assert loss.observe(False,60)
    assert not loss.observe(True,70)
    assert not loss.observe(False,80)
    assert not loss.observe(None,110)  # A network error is not server confirmation.
    assert not loss.observe(False,120)
    assert not loss.observe(False,150)
    assert loss.observe(False,180)
