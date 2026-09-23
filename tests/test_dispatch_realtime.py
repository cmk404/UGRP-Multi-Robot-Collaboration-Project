"""Bounded dispatch RGB snapshots and paired command admission."""
from concurrent.futures import Future, ThreadPoolExecutor
import copy
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from scripts.research_dispatch_scene import DispatchScene
from scripts.run_dispatch_skills import SkillScene
from harness.dispatch_skill_binding import SkillBindings
from scripts.dispatch_pair_skill import BoundPairSkill
from scripts.dispatch_native_view import HeadlessPacer
from scripts.camera_approach_scene import ApproachScene
from scripts.run_camera_varied_start_student import run_approach


class _PermissionProbeSolo:
    """Worker candidate that advances its route only after an RGB gate check."""
    def __init__(self, binding):
        self.navigator=SimpleNamespace(_permission=binding.permission,index=0)
        self.box=SimpleNamespace(phase='carry',last_attachment=None)
        self.steps=0;self.done=False

    @property
    def phase(self):return self.box.phase

    def decide(self,_own,_top):
        self.steps+=1
        allowed=self.navigator._permission('box','UNLOAD')
        if allowed:self.navigator.index+=1
        return ({'kind':'mecanum','forward':.05 if allowed else 0.,
                 'left':0.,'turn':0.,'duration_s':.2},
                {'waiting_for_resource':not allowed})


def _realtime_solo_scene(tmp_path,monkeypatch):
    binding=SkillBindings.__new__(SkillBindings)
    binding.solo='r2';binding.revoked=False;binding.route_overlap=True
    binding.overlap_start='grasp';binding.grasp_started={'beam'}
    binding.transit_started=set();binding.finished={'beam-job'}
    binding.tasks={'box':{'id':'box-job','route':'south'},
                   'beam':{'id':'beam-job','route':'north'}}
    binding.static_map={'routes':{'south':{'resource':'south-lane'},
                                  'north':{'resource':'north-lane'}}}
    binding.locks={};binding.resource_events=[]
    binding.committed={'plan_hash':'test-plan'}
    clock=[0.]
    port=SimpleNamespace(_actuator_state=Mock(return_value={}),hold=Mock(),
        validate_bounded=Mock(),apply_bounded=Mock(),validate_action=Mock(),
        apply=Mock())
    scene=SkillScene.__new__(SkillScene)
    scene.bindings=binding;scene.solo=_PermissionProbeSolo(binding)
    scene.authorize=Mock();scene.time=lambda:clock[0]
    scene.ports={'r2':port};scene.command_history={'r2':[]}
    scene.solo_executor=SimpleNamespace(tick=Mock(),idle=True,submit=Mock())
    scene._solo_pending=None;scene._solo_retry_at=0.;scene.solo_lease=0.
    scene.solo_started=0.;scene.solo_rows=[];scene._solo_phase_label='carry'
    scene.realtime_stats={'solo_backpressure':0,'solo_stale_rgb':0,
                          'solo_decisions':0,'max_decision_age_s':0.}
    scene._decision_workers=ThreadPoolExecutor(max_workers=1)
    scene.out=tmp_path;scene.video=None
    frames=Future();frames.set_result({'r2':{'frame_id':11,
        'observed_at_s':0.,'own_bytes':b'own','top_bytes':b'top',
        'own_rgb':'raw-own','shared_top_rgb':'raw-top'}})
    scene.capture_async=Mock(return_value=frames)
    monkeypatch.setattr('scripts.run_dispatch_skills.normalize_own_rgb',
                        lambda obs:(obs,{'method':'test'}))
    monkeypatch.setattr('scripts.run_dispatch_skills.image_record',
                        lambda *_args:'normalized-own')
    return scene,binding,port,clock


@pytest.mark.parametrize('commit_time,deny_unload,expected',[
    (.1,False,'committed'),(.1,True,'denied'),(.7,False,'stale')])
def test_solo_worker_permission_is_private_until_fresh_owner_commit(
        tmp_path,monkeypatch,commit_time,deny_unload,expected):
    scene,binding,port,clock=_realtime_solo_scene(tmp_path,monkeypatch)
    try:
        scene._solo_tick_realtime()
        scene._solo_pending['future'].result(timeout=2)
        assert binding.locks=={} and binding.resource_events==[]
        assert scene.solo.navigator.index==0
        if deny_unload:binding.finished.clear()
        clock[0]=commit_time
        scene._solo_tick_realtime()
        if expected=='committed':
            assert scene.solo.navigator.index==1
            assert scene.solo.navigator._permission.__self__ is binding
            assert binding.locks=={'south-lane':'box-job','dispatch_apron':'box-job'}
            assert {event['resource'] for event in binding.resource_events}=={
                'south-lane','dispatch_apron'}
            port.apply_bounded.assert_called_once()
            assert scene.command_history['r2'][-1]['valid_until_s']==pytest.approx(.35)
            assert scene.solo_lease==pytest.approx(.12)
            assert scene.solo_rows[-1]['requested_duration_s']==pytest.approx(.2)
            assert scene.solo_rows[-1]['effective_lease_s']==pytest.approx(.25)
            # The next cloned worker must not inherit a live mutating callback.
            clone=copy.deepcopy(scene.solo)
            assert clone.navigator._permission.__self__ is not binding
        else:
            assert scene.solo.navigator.index==0
            assert binding.locks=={} and binding.resource_events==[]
            assert scene.solo_rows[-1]['dropped']==(
                'resource_permission' if expected=='denied' else 'stale_rgb')
            port.apply_bounded.assert_not_called()
            port.hold.assert_called_once_with(commit_time)
    finally:scene._decision_workers.shutdown(wait=True)


def test_solo_carry_lease_ends_at_original_rgb_ttl(tmp_path,monkeypatch):
    scene,binding,port,clock=_realtime_solo_scene(tmp_path,monkeypatch)
    try:
        scene._solo_tick_realtime()
        scene._solo_pending['future'].result(timeout=2)
        clock[0]=.5
        scene._solo_tick_realtime()
        assert port.apply_bounded.call_args.args[2]==pytest.approx(.1)
        assert scene.command_history['r2'][-1]['valid_until_s']==pytest.approx(.6)
        assert scene.solo_lease==pytest.approx(.52)
    finally:scene._decision_workers.shutdown(wait=True)


def test_denied_worker_route_gate_never_acquires_live_resources(tmp_path,monkeypatch):
    scene,binding,port,clock=_realtime_solo_scene(tmp_path,monkeypatch)
    binding.finished.clear()  # Beam has not released the shared unload apron.
    try:
        scene._solo_tick_realtime()
        result=scene._solo_pending['future'].result(timeout=2)
        assert result['evidence']['waiting_for_resource'] is True
        assert result['candidate'].navigator.index==0
        assert binding.locks=={} and binding.resource_events==[]
        clock[0]=.1
        scene._solo_tick_realtime()
        assert scene.solo.navigator.index==0
        assert binding.locks=={} and binding.resource_events==[]
        assert scene.solo_rows[-1]['dropped']=='resource_permission'
        port.apply_bounded.assert_not_called()
    finally:scene._decision_workers.shutdown(wait=True)


@pytest.mark.parametrize('forward,expected_lease,expected_capture_wait',[
    (.05,.15,.02),(0.,.2,.2)])
def test_yield_moving_renews_with_rgb_ttl_but_zero_keeps_dwell(
        forward,expected_lease,expected_capture_wait):
    action={'kind':'mecanum','forward':forward,'left':0.,'turn':0.,
            'duration_s':.2}
    result={'observed_at_s':.5,'frame_id':15,'action':action,'evidence':{},
            'policy':SimpleNamespace(done=False),'observation':{'image':'encoded'},
            'images':{}}
    completed=Future();completed.set_result(result)
    scene=SkillScene.__new__(SkillScene)
    scene.bindings=SimpleNamespace(solo='r2',tasks={'beam':{'id':'beam-job'},
        'box':{'id':'box-job'}},finished=set(),committed={'plan_hash':'test-plan'})
    port=SimpleNamespace(hold=Mock(),validate_bounded=Mock(),apply_bounded=Mock())
    scene.ports={'r2':port};scene.authorize=Mock();scene.raw=Mock()
    scene.command_history={'r2':[]};scene._yield_pending={'future':completed,'index':0}
    scene.yield_folded=True;scene.yield_policy=None;scene.yield_rows=[]
    scene.realtime_stats={'solo_stale_rgb':0,'solo_decisions':0,
                          'max_decision_age_s':0.}
    scene.solo_executor=SimpleNamespace(submit=Mock())
    scene.step=Mock(side_effect=AssertionError('recursive physics'))
    scene._yield_tick_realtime(.95)
    assert scene.solo_lease==pytest.approx(.95+expected_capture_wait)
    if forward:
        assert port.apply_bounded.call_args.args[2]==pytest.approx(expected_lease)
        assert scene.command_history['r2'][-1]['valid_until_s']==pytest.approx(1.1)
        scene.raw.assert_not_called()
    else:
        port.apply_bounded.assert_not_called()
        scene.raw.assert_called_once_with('r2',action,'YIELD')
    assert scene.yield_rows[-1]['requested_duration_s']==pytest.approx(.2)
    assert scene.yield_rows[-1]['effective_lease_s']==pytest.approx(expected_lease)
    scene.step.assert_not_called()


def test_async_capture_binds_all_actor_cameras_to_one_snapshot(tmp_path):
    batch=SimpleNamespace(frame_id=19,sim_time=3.125,
        rgb={(None,'cctv_top'):np.zeros((8,8,3),dtype=np.uint8),
             ('r1','robot_cam'):np.ones((8,8,3),dtype=np.uint8)*50,
             ('r3','robot_cam'):np.ones((8,8,3),dtype=np.uint8)*100})
    rendered=Future();rendered.set_result(batch)
    world=SimpleNamespace(render_snapshot_async=Mock(return_value=rendered))
    scene=DispatchScene.__new__(DispatchScene)
    scene.world=world;scene.out=tmp_path;scene.sequence=0;scene._capture_workers=None
    (tmp_path/'rgb').mkdir()
    try:
        frames=scene.capture_async('sample',own_robots=('r1','r3')).result(timeout=2)
        assert world.render_snapshot_async.call_args.args[0] == [
            (None,'cctv_top'),('r1','robot_cam'),('r3','robot_cam')]
        assert {frames[r]['frame_id'] for r in ('r1','r2','r3')} == {19}
        assert {frames[r]['observed_at_s'] for r in ('r1','r2','r3')} == {3.125}
        assert frames['r1']['top_bytes']==frames['r3']['top_bytes']
        assert 'own_bytes' not in frames['r2']
    finally:
        scene._capture_workers.shutdown(wait=True)


def test_pair_batch_preflight_prevents_half_issued_command():
    scene=SkillScene.__new__(SkillScene)
    scene.authorize=Mock();scene.time=lambda:2.
    scene.pair_phase='SETUP';scene.command_history={'r1':[],'r2':[],'r3':[]}
    scene.bindings=SimpleNamespace(pair={'r1':'r1','r3':'r3'},
        committed={'plan_hash':'abc'},note_transit_command=Mock())
    first=SimpleNamespace(validate_bounded=Mock(),apply_bounded=Mock())
    second=SimpleNamespace(validate_bounded=Mock(side_effect=ValueError('bad')),
        apply_bounded=Mock())
    scene.ports={'r1':first,'r3':second}
    commands={r:dict(kind='mecanum',forward=.05,left=0.,turn=0.,duration_s=.2)
              for r in ('r1','r3')}
    with pytest.raises(ValueError,match='bad'):
        scene.pair_issue_bounded(commands,.2,'TRANSIT')
    first.apply_bounded.assert_not_called()
    second.apply_bounded.assert_not_called()

    second.validate_bounded.side_effect=None
    scene.pair_issue_bounded(commands,.2,'TRANSIT')
    first.apply_bounded.assert_called_once_with(commands['r1'],2.,.2)
    second.apply_bounded.assert_called_once_with(commands['r3'],2.,.2)
    assert {scene.command_history[r][0]['valid_until_s'] for r in ('r1','r3')}=={2.2}


def test_pending_solo_rgb_does_not_block_physics_owner():
    pending=Future()
    scene=SkillScene.__new__(SkillScene)
    scene.solo=object();scene.authorize=Mock();scene.time=lambda:1.
    scene.solo_executor=SimpleNamespace(tick=Mock())
    scene._solo_pending={'future':pending,'index':0}
    scene.realtime_control=True;scene.native_view=None
    scene.ports={};scene.original_step=Mock()
    scene.physics_steps=scene.weld_steps=scene.obstacle_contact_steps=0
    scene._contact_audit=SimpleNamespace(has_penetrating_contact=lambda _:False)
    scene.world=SimpleNamespace(data=SimpleNamespace(eq_active=np.array([False])))
    scene.video=scene.referee=None
    scene._physics(None)
    scene.original_step.assert_called_once()
    assert scene.physics_steps==1
    assert not pending.done()


def test_delayed_pair_rgb_advances_independent_physics_then_holds_both(monkeypatch):
    """An injected slow RGB result cannot be relabelled with decision time."""
    clock=[0.];solo_ticks=[0];pending=Future();issued=[]
    frames={r:{'frame_id':2,'observed_at_s':0.,'raw_top_bytes':b'top',
               'own_bytes':b'own'} for r in ('r1','r3')}
    def step(seconds):
        clock[0]+=seconds;solo_ticks[0]+=1
        if clock[0]>=.7 and not pending.done():pending.set_result(frames)
    ports={r:SimpleNamespace(hold=Mock()) for r in ('r1','r3')}
    io=SimpleNamespace(time=lambda:clock[0],step=step,ports=ports,
        realtime_control=True,
        capture_async=Mock(return_value=pending),
        pair_issue_bounded=lambda commands,duration,stage:issued.append(commands),
        realtime_stats={'pair_backpressure':0,'pair_decisions':0,'max_decision_age_s':0.})
    binding=SimpleNamespace(cluttered=False,committed={'plan_hash':'0123456789abcdef'},
        pair={'r1':'r1','r3':'r3'},reserve_beam_apron=Mock())
    pair=BoundPairSkill.__new__(BoundPairSkill)
    pair.io=io;pair.bindings=binding;pair.phase='SETUP';pair.transport_started=False
    pair.count=0;pair.calls=[];pair.carried_beam=SimpleNamespace(previous=None)
    pair.capture=lambda _:frames
    pair._bind_capture=lambda captured,count:captured
    monkeypatch.setattr('scripts.dispatch_pair_skill.own_payload',lambda *_args,**_kw:(1.,0.,0.))
    monkeypatch.setattr('harness.dispatch_translation_skew.translation_skew',
                        lambda *_args:(0.,{}))
    navigator=SimpleNamespace(observe=lambda _raw:({'forward':.08,'left':0.},
                                                    {'done':False}))
    with pytest.raises(RuntimeError,match='decision budget exhausted'):
        pair.carry_realtime(navigator,max_steps=1)
    assert pair.realtime_open_capture is True
    assert solo_ticks[0]>0  # shared physics continued during pair RGB delay
    assert io.realtime_stats['pair_decisions']==1
    assert next(row for row in pair.calls if row['kind']=='carry')['decision_age_s']>.6
    assert pair.calls[-1]['kind']=='bounded_pair_stale_rgb'
    assert not issued  # Even a zero lease is replaced by an immediate pair hold.
    assert all(port.hold.call_count>=1 for port in ports.values())


@pytest.mark.parametrize('mode,forwards,observed_at_s,requested_s,effective_s,after_issue_step_s',[
    ('CRUISE',{'r1':.08,'r3':.06},1.,.25,.25,.02),
    ('CRUISE',{'r1':.08,'r3':.06},.55,.25,.15,.02),
    ('ALIGN',{'r1':.04,'r3':0.},1.,.25,.25,.02),
    ('SETTLE',{'r1':0.,'r3':0.},1.,.2,.2,.05),
])
def test_open_carry_renews_only_moving_pair_lease(
        monkeypatch,mode,forwards,observed_at_s,requested_s,effective_s,after_issue_step_s):
    frames={r:{'frame_id':20,'observed_at_s':observed_at_s,'raw_top_bytes':b'top',
               'own_bytes':b'own'} for r in ('r1','r3')}
    completed=Future();completed.set_result(frames)
    ports={r:SimpleNamespace(hold=Mock()) for r in ('r1','r3')}
    io=SimpleNamespace(realtime_control=True,time=lambda:1.,ports=ports,
        capture_async=Mock(return_value=completed),pair_issue_bounded=Mock(),
        realtime_stats={'pair_backpressure':0,'pair_decisions':0,
                        'max_decision_age_s':0.})
    binding=SimpleNamespace(cluttered=False,committed={'plan_hash':'0123456789abcdef'},
        pair={'r1':'r3','r3':'r1'},reserve_beam_apron=Mock())
    pair=BoundPairSkill.__new__(BoundPairSkill)
    pair.io=io;pair.bindings=binding;pair.phase='SETUP';pair.transport_started=False
    pair.count=0;pair.calls=[];pair.carried_beam=SimpleNamespace(previous=None)
    pair.capture=lambda _tag:frames
    pair._bind_capture=lambda captured,_count:captured
    pair.tick=Mock()
    control={'mode':mode,'valid':True,'forwards':forwards,
             'duration_s':.2,'abort':False,'done':False}
    monkeypatch.setattr('scripts.dispatch_pair_skill.PairCarryPolicy',
        lambda _task:SimpleNamespace(step=lambda *_args,**_kwargs:control))
    monkeypatch.setattr('scripts.dispatch_pair_skill.own_payload',
        lambda *_args,**_kwargs:(1.,0.,0.))
    monkeypatch.setattr('harness.dispatch_translation_skew.translation_skew',
        lambda *_args:(0.,{}))
    navigator=SimpleNamespace(observe=lambda _raw:({'forward':.08,'left':.01},
                                                    {'done':False}))

    with pytest.raises(RuntimeError,match='decision budget exhausted'):
        pair.carry_realtime(navigator,max_steps=1)

    io.pair_issue_bounded.assert_called_once()
    actions,duration,stage=io.pair_issue_bounded.call_args.args
    assert set(actions)=={'r1','r3'} and stage=='TRANSIT'
    assert actions['r3']['forward']==pytest.approx(forwards['r1'])
    assert actions['r1']['forward']==pytest.approx(forwards['r3'])
    assert all(action['duration_s']==pytest.approx(effective_s)
               for action in actions.values())
    assert duration==pytest.approx(effective_s)
    assert 1.+duration<=observed_at_s+.6+1e-9
    assert pair.tick.call_args.args==(after_issue_step_s,)
    carry_row=next(row for row in pair.calls if row['kind']=='carry')
    assert carry_row['control']['duration_s']==.2
    assert carry_row['requested_lease_s']==pytest.approx(requested_s)
    assert carry_row['effective_lease_s']==pytest.approx(effective_s)
    assert all(port.hold.call_count==1 for port in ports.values())


def test_moving_open_approach_overlaps_rgb_with_bounded_lease_and_caps_ttl():
    clock=[.5];issued=[]
    ports={r:SimpleNamespace(hold=Mock()) for r in ('r1','r3')}
    def step(seconds):clock[0]+=seconds
    io=SimpleNamespace(realtime_control=True,time=lambda:clock[0],step=step,
        ports=ports,pair_drive=Mock(),
        pair_issue_bounded=lambda commands,duration,stage:issued.append(
            (commands,duration,stage,clock[0])))
    pair=BoundPairSkill.__new__(BoundPairSkill)
    pair.io=io;pair.bindings=SimpleNamespace(cluttered=False,
        pair={'r1':'r1','r3':'r3'},reserve_beam_apron=Mock())
    pair.phase='APPROACH';pair.transport_started=False;pair.calls=[]
    pair.carried_beam=SimpleNamespace(previous=None)
    pair.last_capture={'r1':{'observed_at_s':0.}}
    moving={'r1':{'forward':0.,'left':-.03,'turn':0.},
            'r3':{'forward':0.,'left':0.,'turn':0.}}
    pair.drive_mecanum(moving,.2)
    commands,duration,stage,issued_at=issued[0]
    assert stage=='APPROACH' and duration==pytest.approx(.1)
    assert issued_at+duration==pytest.approx(.6)
    assert all(action['duration_s']==duration for action in commands.values())
    assert clock[0]==pytest.approx(.52)  # The rest of the lease can overlap RGB.
    io.pair_drive.assert_not_called()

    pair.drive_mecanum({r:{'forward':0.,'left':0.,'turn':0.} for r in ('r1','r3')},.25)
    io.pair_drive.assert_called_once()  # Stationary confirmation keeps full dwell.
    clock[0]=.61
    pair.drive_mecanum(moving,.2)
    assert len(issued)==1
    assert all(port.hold.called for port in ports.values())


def test_open_approach_renews_for_quarter_second_in_coarse_and_fine():
    clock=[.05];issued=[]
    io=SimpleNamespace(realtime_control=True,time=lambda:clock[0],
        step=lambda seconds:clock.__setitem__(0,clock[0]+seconds),
        pair_issue_bounded=lambda commands,duration,stage:issued.append(
            (commands,duration,stage,clock[0])))
    pair=BoundPairSkill.__new__(BoundPairSkill)
    pair.io=io;pair.bindings=SimpleNamespace(cluttered=False,pair={'r1':'r1','r3':'r3'})
    pair.phase='APPROACH';pair.transport_started=False
    pair.carried_beam=SimpleNamespace(previous=None)
    pair.last_capture={'r1':{'observed_at_s':0.}}
    moving={'r1':{'forward':0.,'left':-.03,'turn':0.},
            'r3':{'forward':0.,'left':0.,'turn':0.}}

    pair.drive_mecanum(moving)
    assert issued[0][1]==pytest.approx(.25)
    assert all(action['duration_s']==pytest.approx(.25)
               for action in issued[0][0].values())
    assert clock[0]==pytest.approx(.07)

    pair.last_capture={'r1':{'observed_at_s':clock[0]}}
    pair.drive_mecanum(moving,.2)
    assert issued[1][1]==pytest.approx(.25)


def test_cluttered_approach_keeps_full_synchronous_drive():
    io=SimpleNamespace(realtime_control=True,pair_drive=Mock())
    pair=BoundPairSkill.__new__(BoundPairSkill)
    pair.io=io;pair.bindings=SimpleNamespace(cluttered=True,pair={'r1':'r1','r3':'r3'})
    pair.transport_started=False;pair.phase='APPROACH'
    pair.carried_beam=SimpleNamespace(previous=None)
    moving={r:{'forward':.1,'left':0.,'turn':0.} for r in ('r1','r3')}
    pair.drive_mecanum(moving,.2)
    io.pair_drive.assert_called_once()


def test_open_carry_release_and_confirmation_keep_async_capture(monkeypatch):
    frames={r:{'frame_id':7,'observed_at_s':0.,'raw_top_bytes':b'top'}
            for r in ('r1','r3')}
    async_tags=[]
    def capture_async(tag,**_kwargs):
        async_tags.append(tag)
        completed=Future();completed.set_result(frames);return completed
    io=SimpleNamespace(realtime_control=True,time=lambda:0.,
        capture=Mock(side_effect=AssertionError('synchronous capture')),
        capture_async=capture_async,await_visual=lambda future:future.result(),
        compute_visual=lambda fn:fn(),
        realtime_stats={'pair_backpressure':0,'pair_stage_stale_rgb':0,
                        'pair_stage_samples':0})
    pair=BoundPairSkill.__new__(BoundPairSkill)
    pair.io=io;pair.bindings=SimpleNamespace(
        pair={'r1':'r1','r3':'r3'},plan={'dock':'north'},
        static_map={'docks':{'north':{'slots':{'beam':{
            'center_m':[1.,1.],'half_extents_m':[1.,1.]}}}}})
    pair.transport_started=True;pair.realtime_open_capture=True
    pair.phase='TRANSIT';pair.count=0;pair.calls=[]
    pair._bind_capture=lambda captured,_count:captured
    pair.commands={r:{3:1500,4:1500,5:1500} for r in ('r1','r3')}
    pair.grasp_report={'preclose_issued_commands':pair.commands}
    pair.replay=Mock();pair.tick=Mock()
    pair.carried_beam=SimpleNamespace(previous={'center':[0.,0.],'image_size':[8,8]})
    monkeypatch.setattr('scripts.dispatch_pair_skill.released_beam_envelope',
        lambda *_args:{'corners_px':[[1.,1.]]*4})
    monkeypatch.setattr('scripts.dispatch_pair_skill.pixel_from_map',
        lambda xy,*_args,**_kwargs:np.array(xy))

    pair.capture('carry-anchor')
    pair.place()
    pair.verify_placement()

    assert len(async_tags)==6
    assert async_tags[0].endswith('carry-anchor')
    assert [tag.rsplit('-',1)[-1] for tag in async_tags[1:4]]==[
        'lowered','opened','final']
    assert all(tag.endswith('placement-confirmation') for tag in async_tags[4:])
    assert io.realtime_stats['pair_stage_samples']==2
    assert pair.calls[-1]['kind']=='visual_placement'
    assert pair.calls[-1]['stable'] is True
    io.capture.assert_not_called()


def test_transport_capture_remains_sync_without_open_realtime_capability():
    frames={r:{'frame_id':3} for r in ('r1','r3')}
    io=SimpleNamespace(realtime_control=True,capture=Mock(return_value=frames),
                       capture_async=Mock(side_effect=AssertionError('async capture')))
    pair=BoundPairSkill.__new__(BoundPairSkill)
    pair.io=io;pair.bindings=SimpleNamespace(pair={'r1':'r1','r3':'r3'})
    pair.transport_started=True;pair.realtime_open_capture=False;pair.count=0
    pair._bind_capture=lambda captured,_count:captured
    assert pair.capture('rotate-carry') is frames
    io.capture.assert_called_once()
    io.capture_async.assert_not_called()


def test_cluttered_realtime_carry_keeps_rotation_capture_contract():
    pair=BoundPairSkill.__new__(BoundPairSkill)
    pair.bindings=SimpleNamespace(cluttered=True)
    pair.realtime_open_capture=False
    pair.carry_with_rotation=Mock(return_value='legacy-rotation')
    assert pair.carry_realtime(None,max_steps=2)=='legacy-rotation'
    assert pair.realtime_open_capture is False
    pair.carry_with_rotation.assert_called_once_with(2)


def test_delayed_yield_rgb_cannot_issue_fold_or_drive():
    completed=Future();completed.set_result({'observed_at_s':0.,'frame_id':7})
    scene=SkillScene.__new__(SkillScene)
    scene.bindings=SimpleNamespace(solo='r2',tasks={'beam':{'id':'beam-job'}},finished=set())
    scene.ports={'r2':SimpleNamespace(hold=Mock())}
    scene._yield_pending={'future':completed,'index':0}
    scene.yield_rows=[];scene.realtime_stats={'solo_stale_rgb':0}
    scene.solo_executor=SimpleNamespace(submit=Mock())
    scene._yield_tick_realtime(1.)
    scene.ports['r2'].hold.assert_called_once_with(1.)
    scene.solo_executor.submit.assert_not_called()
    assert scene.yield_rows[0]['dropped']=='stale_rgb'


def test_headless_pacer_has_no_viewer_dependency():
    scene=SimpleNamespace(time=lambda:0.,deadline=None)
    pacer=HeadlessPacer(scene,realtime_factor=1.)
    pacer.tick();pacer.close()
    with pytest.raises(ValueError,match='positive finite'):
        HeadlessPacer(scene,realtime_factor=0.)


def test_pair_stage_reobserves_after_delayed_rgb_without_issuing_motion():
    clock=[0.];owner_ticks=[0];calls=[0]
    first=Future()
    def frames(frame_id,observed):
        return {r:{'frame_id':frame_id,'observed_at_s':observed}
                for r in ('r1','r3')}
    def capture_async(*_args,**_kwargs):
        calls[0]+=1
        if calls[0]==1:return first
        fresh=Future();fresh.set_result(frames(2,clock[0]));return fresh
    def await_visual(future):
        while not future.done():
            clock[0]+=.02;owner_ticks[0]+=1
            if clock[0]>=.7:first.set_result(frames(1,0.))
        return future.result()
    ports={r:SimpleNamespace(hold=Mock()) for r in ('r1','r3')}
    io=SimpleNamespace(realtime_control=True,capture_async=capture_async,
        await_visual=await_visual,compute_visual=lambda fn:fn(),
        realtime_stats={'pair_backpressure':0,'pair_stage_stale_rgb':0,'pair_stage_samples':0},
        ports=ports,time=lambda:clock[0])
    pair=BoundPairSkill.__new__(BoundPairSkill)
    pair.io=io;pair.bindings=SimpleNamespace(pair={'r1':'r1','r3':'r3'})
    pair.transport_started=False;pair.phase='APPROACH';pair.count=0;pair.calls=[]
    pair._bind_capture=lambda captured,_count:captured
    observed,decision=pair.observe_and_compute('coarse',lambda frame:frame['r1']['frame_id'])
    assert owner_ticks[0]>0 and calls[0]==2
    assert decision==observed['r1']['frame_id']==2
    assert io.realtime_stats['pair_stage_stale_rgb']==1
    assert all(port.hold.call_count==1 for port in ports.values())


def test_visual_compute_pumps_owner_and_rejects_recursive_wait():
    gate=threading.Event();progress=[0]
    scene=SkillScene.__new__(SkillScene)
    scene.realtime_control=True;scene._in_physics=False
    scene.step=lambda _seconds:(progress.__setitem__(0,progress[0]+1),gate.set())
    with ThreadPoolExecutor(max_workers=1) as pool:
        scene._decision_workers=pool
        assert scene.compute_visual(lambda:(gate.wait(2),'ready')[1])=='ready'
    assert progress[0]>0
    scene._in_physics=True
    done=Future();done.set_result('ready')
    with pytest.raises(RuntimeError,match='inside physics callback'):
        scene.await_visual(done)


def test_stage_runner_uses_optional_observation_compute_hook(monkeypatch):
    monkeypatch.setattr('harness.camera_varied_start_student.predict_stage',
        lambda *_args:dict(ok=True,ready=True,stationary_ready=True,
                           precision='fine',command=0.))
    frames={r:{'own_bytes':b'own','top_bytes':b'top','frame_id':1,
               'own_rgb':'own.jpg','shared_top_rgb':'top.jpg'} for r in ('r1','r3')}
    labels=[]
    scene=SimpleNamespace(time=lambda:0.,capture=Mock(side_effect=AssertionError('sync capture')),
        drive_mecanum=Mock(),stop_dwell=Mock(),tick=Mock(),
        evaluation_snapshot=lambda:{'source':'referee_only'})
    scene.observe_and_compute=lambda tag,predict:(labels.append(tag) or frames,predict(frames))
    models={r:{stage:{} for stage in ('yaw','lateral','forward')} for r in ('r1','r3')}
    report=run_approach(scene,models)
    assert report['approach_ok']
    assert any(tag.startswith('phase-') for tag in labels)
    assert labels[-1]=='final-alignment-1'
    scene.capture.assert_not_called()


def test_inherited_grasp_uses_optional_observation_compute_hook(tmp_path,monkeypatch):
    (tmp_path/'student-skill.json').write_text('{}')
    monkeypatch.setattr('scripts.run_camera_pair_transport.evaluate_grasp_samples',
                        lambda _samples:{'grasp_success':False})
    frames={r:{'own_bytes':b'own','top_bytes':b'top',
               'own_rgb':'own.jpg','shared_top_rgb':'top.jpg'} for r in ('r1','r3')}
    labels=[]
    fake=SimpleNamespace(
        skill={'initialization_replay':[{}],'models':{r:{'sha256':'sha'} for r in ('r1','r3')},
               'close_pulses':{'r1':1500,'r3':1500},'close_duration_s':.5,
               'close_settle_s':.1,'lift_delta_pulses':{r:{} for r in ('r1','r3')},
               'lift_duration_s':.4,'lift_settle_s':.1,'hold_s':.1},
        models_root=tmp_path,commands={r:{1:1500,3:1500,4:1500,5:1500}
                                       for r in ('r1','r3')},
        evaluation_samples=[],capture=Mock(side_effect=AssertionError('sync capture')),
        replay=Mock(),tick=Mock(),phase='GRASP')
    fake.observe_and_compute=lambda tag,predict:(labels.append(tag) or frames,predict(frames))
    models={r:{'channels':[1]} for r in ('r1','r3')}
    decisions=ApproachScene.finish_grasp(fake,
        lambda *_args,**_kwargs:{'delta_pulses':[0]},models,rounds=1)
    assert len(decisions)==2
    assert labels==['grasp-000','grasp-post-recovery']
    fake.capture.assert_not_called()
