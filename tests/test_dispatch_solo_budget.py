"""Realtime solo SIM budget charges only confirmed resource denial intervals."""
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from scripts.run_dispatch_skills import SkillScene, SOLO_ACTIVE_SIM_BUDGET_S


def _budget_scene():
    scene=SkillScene.__new__(SkillScene)
    scene.solo_started=0.
    scene.realtime_stats={'solo_stale_rgb':0}
    scene._solo_budget_last_s=None
    scene._solo_budget_active_s=scene._solo_budget_wait_s=0.
    scene._solo_resource_waiting=False
    scene._solo_resource_wait_reason=None
    scene._solo_budget_ended=False
    return scene


def _owner_scene():
    scene=_budget_scene();clock=[0.]
    scene.time=lambda:clock[0]
    scene.solo=SimpleNamespace(phase='carry',
        box=SimpleNamespace(_last_sim_time=0.,last_attachment=None),
        navigator=SimpleNamespace(index=0),steps=3)
    scene.authorize=Mock()
    scene.solo_executor=SimpleNamespace(tick=Mock())
    scene.bindings=SimpleNamespace(
        tasks={'box':{'id':'box-job'},'beam':{'id':'beam-job'}},
        finished=set(),solo='r2')
    scene.ports={'r2':SimpleNamespace(hold=Mock())}
    scene._solo_pending=None;scene._solo_retry_at=0.
    scene.solo_rows=[]
    scene._adopt_waiting_carry_visual=Mock(return_value=False)
    return scene,clock


def _pending_result(scene,observed,*,frame_id,worker_wait=True):
    result={'candidate':scene.solo,'action':{'kind':'mecanum',
        'forward':0.,'left':0.,'turn':0.,'duration_s':.2},
        'evidence':{'waiting_for_resource':worker_wait},
        'before':'carry','permission_requests':(('box','UNLOAD'),),
        'observation':{'sim_time':observed,'frame_id':frame_id,'image':'own-rgb'},
        'images':{'own':{'path':'rgb/own.jpg'},'top':{'path':'rgb/top.jpg'}},
        'observed_at_s':observed,'frame_id':frame_id}
    future=Future();future.set_result(result)
    scene._solo_pending={'future':future,'index':frame_id,'stage':'TRANSIT'}


def test_repeated_denials_pause_only_active_sim_and_resume_on_owner_permission():
    scene,clock=_owner_scene();allowed=[False]
    scene._permission_snapshot=lambda:SimpleNamespace(
        permission=lambda _obj,_stage:allowed[0])
    clock[0]=.1;_pending_result(scene,0.,frame_id=1)
    scene._solo_tick_realtime()
    assert scene.realtime_stats['solo_resource_wait_intervals']==1
    assert scene.solo_rows[-1]['owner_permission_denied'] is True
    assert scene.solo.navigator.index==0

    clock[0]=40.;_pending_result(scene,39.9,frame_id=2)
    scene._solo_tick_realtime()
    assert scene.realtime_stats['solo_resource_wait_intervals']==1
    assert scene.realtime_stats['solo_resource_wait_sim_s']==pytest.approx(39.9)

    allowed[0]=True
    clock[0]=100.2;_pending_result(scene,100.,frame_id=3)
    scene._solo_tick_realtime()
    assert scene.solo_rows[-1]['owner_permission_denied'] is False
    assert scene.realtime_stats['solo_resource_waiting'] is False
    assert scene.realtime_stats['solo_resource_wait_sim_s']==pytest.approx(100.1)
    assert scene.realtime_stats['solo_active_sim_s']==pytest.approx(.1)
    assert scene.solo.navigator.index==0

    clock[0]=101.;scene._solo_pending={'future':Future(),'index':4}
    scene._solo_tick_realtime()
    assert scene.realtime_stats['solo_total_sim_s']==pytest.approx(101.)
    assert scene.realtime_stats['solo_active_sim_s']==pytest.approx(.9)
    assert scene.realtime_stats['solo_resource_wait_sim_s']==pytest.approx(100.1)
    scene.ports['r2'].hold.assert_called()


def test_stale_rgb_ends_resource_wait_and_later_time_is_active():
    scene,clock=_owner_scene()
    scene._solo_set_resource_wait(0.,True,'candidate')
    clock[0]=10.;_pending_result(scene,0.,frame_id=1)
    scene._solo_tick_realtime()
    assert scene.solo_rows[-1]['dropped']=='stale_rgb'
    assert scene.realtime_stats['solo_resource_wait_sim_s']==pytest.approx(10.)
    assert scene.realtime_stats['solo_resource_waiting'] is False
    clock[0]=11.;scene._solo_pending={'future':Future(),'index':2}
    scene._solo_tick_realtime()
    assert scene.realtime_stats['solo_active_sim_s']==pytest.approx(1.)
    assert scene.realtime_stats['solo_total_sim_s']==pytest.approx(11.)


def test_active_budget_fails_while_rgb_pending_but_total_can_exceed_limit():
    scene,clock=_owner_scene()
    scene._solo_budget_tick(40.)
    scene._solo_set_resource_wait(40.,True,'candidate')
    scene._solo_budget_tick(140.)
    scene._solo_set_resource_wait(140.,False)
    scene._solo_budget_tick(390.)
    assert scene.realtime_stats['solo_total_sim_s']==pytest.approx(390.)
    assert scene.realtime_stats['solo_active_sim_s']==pytest.approx(290.)
    assert scene.realtime_stats['solo_resource_wait_sim_s']==pytest.approx(100.)
    assert scene.realtime_stats['solo_active_budget_s']==SOLO_ACTIVE_SIM_BUDGET_S
    scene._solo_pending={'future':Future(),'index':7}
    clock[0]=401.
    with pytest.raises(RuntimeError,match='solo active SIM budget exhausted'):
        scene._solo_tick_realtime()
    assert scene.realtime_stats['solo_total_sim_s']==pytest.approx(401.)
    assert scene.realtime_stats['solo_active_sim_s']==pytest.approx(301.)


def test_budget_freezes_after_box_finish_and_rejects_reverse_sim_clock():
    scene=_budget_scene()
    scene._solo_budget_tick(12.)
    with pytest.raises(RuntimeError,match='moved backwards'):
        scene._solo_budget_tick(11.)
    assert scene.realtime_stats['solo_total_sim_s']==pytest.approx(12.)
    scene._solo_finish_budget(20.)
    scene._solo_budget_tick(200.)
    assert scene.realtime_stats['solo_total_sim_s']==pytest.approx(20.)
    assert scene.realtime_stats['solo_active_sim_s']==pytest.approx(20.)
