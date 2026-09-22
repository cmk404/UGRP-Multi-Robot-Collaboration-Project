import json
from pathlib import Path
from types import SimpleNamespace
import pytest
from scripts.run_matched_carry_cohort import command


def protocol():
    return {'conditions':{'RGB':{},'ACT':{'model':'weights'}},'grasp':'same-grasp','stages':'same-stages',
            'asset_sha256':{},'controls':{'max_wall_s':2400,'max_carry_steps':900,'video_fps':4,
                                        'efficient_capture':True,'route_overlap':True,'min_free_gib':8},
            'test':[{'id':str(i),'offset':[i*.01,0,0],'variant':'open','seed':11,'dock':'dock_a','plan':'same-plan'} for i in range(2)]}


def test_matched_commands_change_only_declared_model():
    p=protocol();job={'case':p['test'][0],'condition':'RGB'}
    rgb=command(p,job,'out','mjpython','act-python')
    act=command(p,{**job,'condition':'ACT'},'out','mjpython','act-python')
    assert act==rgb+['--carry-act-model','weights','--carry-act-python','act-python']
    assert '--route-overlap' in rgb and '--efficient-capture' in rgb


def test_hybrid_is_an_explicit_condition_and_cannot_be_applied_to_rgb():
    p=protocol();p['conditions']['ACT']['stop_mode']='rgb_refined'
    cmd=command(p,{'case':p['test'][0],'condition':'ACT'},'out','mjpython','act-python')
    assert cmd[-2:]==['--carry-act-stop-mode','rgb_refined']
    p['conditions']['RGB']['stop_mode']='rgb_refined'
    with pytest.raises(ValueError,match='requires ACT'):
        command(p,{'case':p['test'][0],'condition':'RGB'},'out','mjpython','act-python')


@pytest.mark.parametrize('low_disk',[False,True])
def test_failure_is_recorded_and_remaining_cases_run(tmp_path,monkeypatch,low_disk):
    import scripts.run_matched_carry_cohort as runner
    p=tmp_path/'protocol.json';p.write_text(json.dumps(protocol()))
    monkeypatch.setattr(runner.subprocess,'check_output',lambda cmd,**kw:'' if cmd[1]=='status' else 'source')
    monkeypatch.setattr(runner.shutil,'disk_usage',lambda _:SimpleNamespace(free=(7 if low_disk else 12)*2**30))
    calls=[]
    def child(cmd,log,**kw):
        calls.append(cmd);out=Path(cmd[cmd.index('--output')+1]);out.mkdir(parents=True)
        (out/'result.json').write_text(json.dumps({'physical_success':False,'protocol_complete':False,
                                                'phase':'GRASP','error':'bounded failed grasp'}))
        (out/'pair-decisions.json').write_text('[]')
        return {'exit_code':1,'timed_out':False}
    monkeypatch.setattr(runner,'run_logged',child)
    monkeypatch.setattr(runner.sys,'argv',['runner','--protocol',str(p),'--out',str(tmp_path/'out'),
                                        '--mjpython','mjpython','--act-python','act-python'])
    assert runner.main()==(2 if low_disk else 0)
    report=json.loads((tmp_path/'out/report.json').read_text())
    assert len(calls)==(0 if low_disk else 4)
    assert len(report['not_attempted'])==(4 if low_disk else 0)
    for group in report['failure_estimates']['conditions'].values():
        assert group['failures']==(0 if low_disk else 2)
        assert group['pending']==(2 if low_disk else 0)
