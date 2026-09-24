"""Owner-side cadence for the existing RGB box-carry action contract."""
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from scripts.run_dispatch_skills import SkillScene


def _scene():
    clock=[1.]
    port=SimpleNamespace(hold=Mock(),validate_bounded=Mock(),apply_bounded=Mock())
    scene=SkillScene.__new__(SkillScene)
    scene.time=lambda:clock[0]
    scene.authorize=Mock()
    scene.bindings=SimpleNamespace(
        solo='r2',tasks={'box':{'id':'box-id'},'beam':{'id':'beam-id'}},
        finished=set(),committed={'plan_hash':'test-plan'})
    scene.ports={'r2':port};scene.command_history={'r2':[]}
    scene.solo_executor=SimpleNamespace(tick=Mock(),idle=True,submit=Mock())
    scene.solo=SimpleNamespace(phase='carry',box=SimpleNamespace(last_attachment={}),
                               navigator=None,steps=0,done=False,reason=None)
    scene.solo_rows=[];scene._solo_pending=None
    scene._solo_retry_at=0.;scene.solo_lease=0.
    scene._solo_motion_accept_after_s=0.
    scene.realtime_stats={'solo_decisions':0,'solo_stale_rgb':0,
                          'max_decision_age_s':0.}
    scene._solo_phase_label='carry';scene.video=None
    scene._solo_budget_tick=Mock();scene._solo_set_resource_wait=Mock()
    scene._commit_permissions=Mock(return_value=True)
    permission=Mock(return_value=True)
    scene._permission_snapshot=Mock(return_value=SimpleNamespace(permission=permission))
    scene._adopt_waiting_carry_visual=Mock(return_value=False)
    return scene,port,clock,permission


def _pending(scene,index,observed,*,moving=True,phase='carry'):
    candidate=SimpleNamespace(phase=phase,box=SimpleNamespace(last_attachment={}),
                              navigator=None,steps=index+1,done=False,reason=None)
    action={'kind':'mecanum','forward':.06 if moving else 0.,'left':0.,
            'turn':0.,'duration_s':.2}
    future=Future();future.set_result({
        'candidate':candidate,'action':action,'evidence':{},'before':'carry',
        'permission_requests':(),
        'observation':{'frame_id':index,'sim_time':observed,'image':'rgb'},
        'input_transform':{'method':'test'},'images':{'own':'own','top':'top'},
        'observed_at_s':observed,'frame_id':index})
    scene._solo_pending={'future':future,'index':index,'stage':'TRANSIT'}
    return candidate


def _first_motion(scene,clock):
    candidate=_pending(scene,0,.9)
    scene._solo_tick_realtime()
    assert scene.solo is candidate
    assert scene._solo_motion_accept_after_s==pytest.approx(1.2)


def test_early_rgb_result_waits_for_nominal_carry_period_without_losing_overlap():
    scene,port,clock,_=_scene()
    _first_motion(scene,clock)
    candidate=_pending(scene,1,1.02)
    clock[0]=1.15
    scene._solo_tick_realtime()
    assert scene._solo_pending is not None  # completed result remains bounded to one
    assert scene.solo is not candidate
    assert scene.solo.steps==1
    assert scene.solo_lease==pytest.approx(1.02)  # capture was already eligible
    assert port.apply_bounded.call_count==1
    assert scene._commit_permissions.call_count==1  # no early live lock commit
    clock[0]=1.2
    scene._solo_tick_realtime()
    assert scene._solo_pending is None and scene.solo is candidate
    assert [row['issued_at_s'] for row in scene.command_history['r2']]==pytest.approx([1.,1.2])
    assert all(row['valid_until_s']-row['issued_at_s']==pytest.approx(.25)
               for row in scene.command_history['r2'])
    assert scene.solo_rows[-1]['nominal_motion_cadence_s']==pytest.approx(.2)
    assert scene.solo_rows[-1]['cadence_wait_s']==pytest.approx(.05)
    assert port.apply_bounded.call_count==2


def test_ttl_clipped_lease_shortens_admission_wait_to_avoid_expired_motor_gap():
    scene,port,clock,_=_scene()
    _pending(scene,0,.53)  # Only 0.13 SIM seconds remain under the original TTL.
    scene._solo_tick_realtime()
    assert scene.solo_rows[-1]['effective_lease_s']==pytest.approx(.13)
    assert scene.solo_rows[-1]['nominal_motion_cadence_s']==pytest.approx(.2)
    assert scene._solo_motion_accept_after_s==pytest.approx(1.13)
    candidate=_pending(scene,1,1.02)
    clock[0]=1.1;scene._solo_tick_realtime()
    assert scene.solo is not candidate and port.apply_bounded.call_count==1
    clock[0]=1.13;scene._solo_tick_realtime()
    assert scene.solo is candidate and port.apply_bounded.call_count==2
    assert scene.command_history['r2'][0]['valid_until_s']==pytest.approx(1.13)
    assert scene.command_history['r2'][1]['issued_at_s']==pytest.approx(1.13)


def test_stale_result_is_held_before_delayed_commit():
    scene,port,clock,_=_scene()
    _first_motion(scene,clock)
    candidate=_pending(scene,1,1.02)
    clock[0]=1.15;scene._solo_tick_realtime()
    clock[0]=1.7  # owner reaches the next poll after the original RGB TTL
    scene._solo_tick_realtime()
    assert scene.solo is not candidate
    assert scene._solo_pending is None
    assert scene.solo_rows[-1]['dropped']=='stale_rgb'
    assert scene._solo_motion_accept_after_s==0.
    port.hold.assert_called_once_with(1.7)
    assert port.apply_bounded.call_count==1


def test_denied_resource_holds_immediately_even_before_nominal_cadence():
    scene,port,clock,permission=_scene()
    _first_motion(scene,clock)
    candidate=_pending(scene,1,1.02)
    clock[0]=1.15;scene._solo_tick_realtime()
    assert scene._solo_pending is not None
    permission.return_value=False  # gate revokes while the result waits
    clock[0]=1.16;scene._solo_tick_realtime()
    assert scene._solo_pending is None and scene.solo is not candidate
    assert scene.solo_rows[-1]['dropped']=='resource_permission'
    assert scene._commit_permissions.call_count==1
    port.hold.assert_called_once_with(1.16)
    assert scene._solo_motion_accept_after_s==0.


def test_zero_or_phase_change_is_not_delayed_by_previous_motion():
    for phase in ('carry','release'):
        scene,port,clock,_=_scene()
        scene.raw=Mock()
        _first_motion(scene,clock)
        candidate=_pending(scene,1,1.02,moving=False,phase=phase)
        clock[0]=1.15;scene._solo_tick_realtime()
        assert scene._solo_pending is None and scene.solo is candidate
        scene.raw.assert_called_once()
        assert scene._solo_motion_accept_after_s==0.
        assert port.apply_bounded.call_count==1


def test_release_wait_transitions_immediately_despite_earlier_carry_motion():
    scene,port,clock,_=_scene()
    _first_motion(scene,clock)
    candidate=_pending(scene,1,1.02,moving=False,phase='release')
    candidate_result=scene._solo_pending['future'].result()
    candidate_result['action']={'kind':'wait','duration':.1}
    clock[0]=1.15;scene._solo_tick_realtime()
    assert scene._solo_pending is None and scene.solo is candidate
    scene.solo_executor.submit.assert_called_once()
    assert scene._solo_motion_accept_after_s==0.
    assert port.apply_bounded.call_count==1


def test_moving_phase_transition_is_not_delayed_by_carry_cadence():
    scene,port,clock,_=_scene()
    _first_motion(scene,clock)
    candidate=_pending(scene,1,1.02,moving=True,phase='release')
    clock[0]=1.15;scene._solo_tick_realtime()
    assert scene._solo_pending is None and scene.solo is candidate
    assert scene.command_history['r2'][-1]['issued_at_s']==pytest.approx(1.15)
    assert scene._solo_motion_accept_after_s==0.
    assert port.apply_bounded.call_count==2
