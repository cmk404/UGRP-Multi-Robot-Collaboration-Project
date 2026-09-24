"""RGB evidence and bounded admission for the live open-map pair handoff."""
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from scripts.dispatch_pair_skill import BoundPairSkill


ROBOTS = ('r1', 'r3')
AXES = ('yaw', 'lateral', 'forward')


def decision(*, gap=.06, ok=True, ready=False):
    return dict(ok=ok, ready=ready, forward=0., left=0., turn=0.,
                image_error=[gap, .0031])


def heading_unresolved(*, gap=.06, reason='own_wheel_heading_unresolved'):
    return {**decision(gap=gap,ok=False), 'reason':reason}


def frame(index):
    return {r:dict(frame_id=index, observed_at_s=float(index),
                   raw_top_bytes=str(index).encode(), own_bytes=f'{r}-{index}'.encode(),
                   top_bytes=f'top-{index}'.encode(), own_rgb={'path':f'{r}-{index}.jpg'},
                   shared_top_rgb={'path':f'top-{index}.jpg'}) for r in ROBOTS}


class Coarse:
    def __init__(self, rows):
        self.rows = rows

    def decide(self, raw, slot):
        return self.rows[int(raw)][slot]


def pair_for(rows, frames, *, realtime=True, obstacles=(), terrain=()):
    pair = BoundPairSkill.__new__(BoundPairSkill)
    pair.io = SimpleNamespace(realtime_control=realtime, time=lambda:0.,
                             ports={r:SimpleNamespace(_motor_commands=(0.,)*4) for r in ROBOTS})
    pair.bindings = SimpleNamespace(pair={r:r for r in ROBOTS}, cluttered=False, static_map={
        'map_id':'dispatch_open', 'obstacles':list(obstacles), 'terrain':list(terrain)})
    pair.coarse = Coarse(rows)
    pair.stage_models = {r:{axis:dict(slot=r, axis=axis) for axis in AXES} for r in ROBOTS}
    pair.reference = b'reference'
    pair.calls = []
    pair.stop_dwell = Mock()
    pair.drive_mecanum = Mock()
    pair._hold_pair = Mock()
    captured = iter(frames)
    pair.observe_and_compute = Mock(side_effect=lambda tag, predict:
                                    (batch := next(captured), predict(batch)))
    return pair


def supported_prediction(model, own, top):
    return dict(ok=True, precision='fine', ready=False,
                command=.01, image_derived_error=.02)


def test_near_supported_handoff_requires_second_stationary_batch(monkeypatch):
    rows={1:{r:decision() for r in ROBOTS},
          2:{r:decision(ready=True) for r in ROBOTS}}
    pair=pair_for(rows,[frame(1),frame(2)])
    predicted=[]
    def predict(model, own, top):
        predicted.append((model['slot'],model['axis'],own,top))
        return supported_prediction(model,own,top)
    monkeypatch.setattr('scripts.dispatch_pair_skill.predict_stage',predict)
    with patch('scripts.dispatch_pair_skill.run_approach',return_value={'approach_ok':False}) as fine:
        with pytest.raises(RuntimeError,match='fine RGB alignment'):
            pair.approach()
    assert pair.observe_and_compute.call_args_list[1].args[0]=='coarse-fine-handoff-stationary'
    assert len(predicted)==12
    assert {p[2].decode().split('-')[1] for p in predicted}=={'1','2'}
    assert pair.stop_dwell.call_count==1
    pair.drive_mecanum.assert_not_called()
    assert fine.call_args.kwargs==dict(reacquire_on_settle=True,
                                      final_refinement_steps=40,invalid_reobserve_budget=1,
                                      fine_gain_schedule=False)
    assert fine.call_args.args[1] is pair.stage_models
    evidence=pair.calls[-2]  # handoff precedes learned approach audit
    assert evidence['kind']=='coarse_fine_handoff'
    assert evidence['reason']=='fine_supported_handoff'
    assert evidence['initial']['frame_ids']=={'r1':1,'r3':1}
    assert evidence['stationary']['frame_ids']=={'r1':2,'r3':2}
    assert evidence['stationary']['fresh'] is True
    assert all(p['precision']=='fine' for axes in evidence['stationary']['fine_predictions'].values()
               for p in axes.values())


@pytest.mark.parametrize('first_unresolved', [False, True])
def test_exact_heading_dropout_uses_only_stopped_fine_handoff(monkeypatch,first_unresolved):
    initial={r:decision() for r in ROBOTS}
    if first_unresolved:initial['r3']=heading_unresolved()
    stationary={'r1':decision(ready=True),'r3':heading_unresolved(gap=.052)}
    pair=pair_for({1:initial,2:stationary},[frame(1),frame(2)])
    monkeypatch.setattr('scripts.dispatch_pair_skill.predict_stage',supported_prediction)
    with patch('scripts.dispatch_pair_skill.run_approach',return_value={'approach_ok':False}) as fine:
        with pytest.raises(RuntimeError,match='fine RGB alignment'):
            pair.approach()
    fine.assert_called_once()
    pair.drive_mecanum.assert_not_called()
    pair._hold_pair.assert_not_called()
    pair.stop_dwell.assert_called_once()
    handoff=next(c for c in pair.calls if c['kind']=='coarse_fine_handoff')
    assert handoff['reason']=='fine_supported_heading_unresolved_handoff'
    assert handoff['stationary']['unresolved_heading_slots']==['r3']
    assert handoff['stationary']['forward_gaps']['r3']==pytest.approx(.052)
    assert handoff['stationary']['fine_predictions']['r3']['yaw']['ok'] is True
    if first_unresolved:
        coarse=next(c for c in pair.calls if c['kind']=='coarse')
        assert coarse['coordination']['unresolved_heading']==['r3']
        assert all(not any(action.values()) for action in coarse['commands'].values())


def test_initial_heading_dropout_without_six_fine_support_holds_and_fails(monkeypatch):
    rows={1:{'r1':decision(),'r3':heading_unresolved()}}
    pair=pair_for(rows,[frame(1)])
    monkeypatch.setattr('scripts.dispatch_pair_skill.predict_stage',
                        lambda model,own,top:dict(ok=False,precision='fine',ready=False))
    with patch('scripts.dispatch_pair_skill.run_approach') as fine:
        with pytest.raises(RuntimeError,match='coarse RGB model convention unresolved'):
            pair.approach()
    fine.assert_not_called()
    pair._hold_pair.assert_called_once()
    pair.stop_dwell.assert_not_called()
    pair.drive_mecanum.assert_not_called()


@pytest.mark.parametrize('failure', ['support','far_gap','other_reason','stale'])
def test_heading_dropout_stationary_failure_holds_and_fails(monkeypatch,failure):
    initial={r:decision() for r in ROBOTS}
    stationary={'r1':decision(),'r3':heading_unresolved()}
    if failure=='far_gap':stationary['r3']=heading_unresolved(gap=.066)
    if failure=='other_reason':stationary['r3']=heading_unresolved(reason='payload_or_reference_unresolved')
    second=frame(2)
    if failure=='stale':
        for r in ROBOTS:second[r]['frame_id']=1
    pair=pair_for({1:initial,2:stationary},[frame(1),second])
    def predict(model,own,top):
        result=supported_prediction(model,own,top)
        if failure=='support' and top==b'top-2':result['ok']=False
        return result
    monkeypatch.setattr('scripts.dispatch_pair_skill.predict_stage',predict)
    with patch('scripts.dispatch_pair_skill.run_approach') as fine:
        with pytest.raises(RuntimeError,match='coarse RGB model convention unresolved'):
            pair.approach()
    fine.assert_not_called()
    pair.stop_dwell.assert_called_once()
    pair._hold_pair.assert_called_once()
    pair.drive_mecanum.assert_not_called()
    assert next(c for c in pair.calls if c['kind']=='coarse_fine_handoff')['reason'] != 'fine_supported_heading_unresolved_handoff'


@pytest.mark.parametrize('reason,rows,prediction,settings',[
    ('far',{1:{r:decision(gap=.066) for r in ROBOTS}},supported_prediction,{}),
    ('missing',{1:{r:decision() for r in ROBOTS}},supported_prediction,
     {'mutate':lambda d:d.pop('image_error')}),
    ('nonfinite',{1:{r:decision(gap=float('nan')) for r in ROBOTS}},supported_prediction,{}),
    ('coarse_signature',{1:{r:decision(ok=False) for r in ROBOTS}},supported_prediction,{}),
    ('support_false',{1:{r:decision() for r in ROBOTS}},
     lambda model,own,top:dict(ok=False,precision='fine',ready=False),{}),
    ('precision_coarse',{1:{r:decision() for r in ROBOTS}},
     lambda model,own,top:dict(ok=True,precision='coarse',ready=False),{}),
])
def test_unqualified_rgb_cannot_admit_handoff(monkeypatch,reason,rows,prediction,settings):
    if 'mutate' in settings:
        for d in rows[1].values():settings['mutate'](d)
    pair=pair_for({i:rows[1] for i in range(1,121)},[frame(i) for i in range(1,121)])
    monkeypatch.setattr('scripts.dispatch_pair_skill.predict_stage',prediction)
    with patch('scripts.dispatch_pair_skill.run_approach') as fine:
        with pytest.raises((RuntimeError,ValueError)):
            pair.approach()
    fine.assert_not_called()
    pair.stop_dwell.assert_not_called()
    assert not any(c['kind']=='coarse_fine_handoff' for c in pair.calls)


def test_failed_stationary_support_resumes_original_coarse_path_once(monkeypatch):
    rows={1:{r:decision() for r in ROBOTS},
          2:{r:decision() for r in ROBOTS},
          3:{r:decision(ready=True) for r in ROBOTS}}
    pair=pair_for(rows,[frame(1),frame(2),frame(3)])
    def predict(model,own,top):
        result=supported_prediction(model,own,top)
        if top==b'top-2':result['ok']=False
        return result
    monkeypatch.setattr('scripts.dispatch_pair_skill.predict_stage',predict)
    with patch('scripts.dispatch_pair_skill.run_approach',return_value={'approach_ok':False}) as fine:
        with pytest.raises(RuntimeError,match='fine RGB alignment'):
            pair.approach()
    fine.assert_called_once()
    assert pair.observe_and_compute.call_count==3
    assert pair.stop_dwell.call_count==2  # failed probe, then normal coarse ready
    assert pair.drive_mecanum.call_count==1
    handoffs=[c for c in pair.calls if c['kind']=='coarse_fine_handoff']
    assert len(handoffs)==1 and handoffs[0]['reason']=='stationary_fine_support_missing'
    assert len([c for c in pair.calls if c['kind']=='coarse'])==2


def test_stationary_forward_gap_can_revoke_supported_handoff(monkeypatch):
    rows={1:{r:decision() for r in ROBOTS},
          2:{'r1':decision(gap=.066),'r3':decision()},
          3:{r:decision(ready=True) for r in ROBOTS}}
    pair=pair_for(rows,[frame(1),frame(2),frame(3)])
    monkeypatch.setattr('scripts.dispatch_pair_skill.predict_stage',supported_prediction)
    with patch('scripts.dispatch_pair_skill.run_approach',return_value={'approach_ok':False}):
        with pytest.raises(RuntimeError,match='fine RGB alignment'):
            pair.approach()
    evidence=[c for c in pair.calls if c['kind']=='coarse_fine_handoff']
    assert len(evidence)==1
    assert evidence[0]['reason']=='stationary_coarse_gap_or_signature_outside_band'
    assert pair.stop_dwell.call_count==2


def test_handoff_is_allowed_at_original_last_coarse_index(monkeypatch):
    far={r:decision(gap=.066) for r in ROBOTS}
    near={r:decision() for r in ROBOTS}
    rows={i:far for i in range(1,120)} | {120:near,121:near}
    pair=pair_for(rows,[frame(i) for i in range(1,122)])
    monkeypatch.setattr('scripts.dispatch_pair_skill.predict_stage',supported_prediction)
    with patch('scripts.dispatch_pair_skill.run_approach',return_value={'approach_ok':False}) as fine:
        with pytest.raises(RuntimeError,match='fine RGB alignment'):
            pair.approach()
    fine.assert_called_once()
    evidence=[c for c in pair.calls if c['kind']=='coarse_fine_handoff']
    assert len(evidence)==1 and evidence[0]['coarse_index']==119
    assert evidence[0]['reason']=='fine_supported_handoff'


@pytest.mark.parametrize('new_id,new_time',[(1,2.),(2,1.),(2,float('nan'))])
def test_handoff_rejects_reused_or_nonmonotonic_stationary_rgb(monkeypatch,new_id,new_time):
    rows={1:{r:decision() for r in ROBOTS},
          2:{r:decision() for r in ROBOTS}}
    first=frame(1);second=frame(2)
    for r in ROBOTS:
        second[r]['frame_id']=new_id
        second[r]['observed_at_s']=new_time
    pair=pair_for(rows,[first,second]+[frame(2) for _ in range(119)])
    monkeypatch.setattr('scripts.dispatch_pair_skill.predict_stage',supported_prediction)
    with patch('scripts.dispatch_pair_skill.run_approach') as fine:
        with pytest.raises(RuntimeError,match='coarse RGB approach budget exhausted'):
            pair.approach()
    fine.assert_not_called()
    assert pair.stop_dwell.call_count==1
    handoffs=[c for c in pair.calls if c['kind']=='coarse_fine_handoff']
    assert len(handoffs)==1 and handoffs[0]['reason']=='stationary_rgb_not_fresh'


@pytest.mark.parametrize('realtime,obstacles,terrain',[
    (False,(),()),
    (True,({'id':'service_island'},),()),
    (True,(),({'id':'slope'},)),
])
def test_handoff_only_on_live_unmodified_open_map(monkeypatch,realtime,obstacles,terrain):
    rows={i:{r:decision() for r in ROBOTS} for i in range(1,121)}
    pair=pair_for(rows,[frame(i) for i in range(1,121)],
                  realtime=realtime,obstacles=obstacles,terrain=terrain)
    monkeypatch.setattr('scripts.dispatch_pair_skill.predict_stage',
                        Mock(side_effect=AssertionError('no fine preflight outside live open map')))
    with patch('scripts.dispatch_pair_skill.run_approach') as fine:
        with pytest.raises(RuntimeError,match='coarse RGB approach budget exhausted'):
            pair.approach()
    fine.assert_not_called()
    assert not any(c['kind']=='coarse_fine_handoff' for c in pair.calls)
