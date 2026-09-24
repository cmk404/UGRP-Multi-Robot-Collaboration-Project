"""Fine RGB bridge owns one short extension of an already issued lease."""
from concurrent.futures import Future
from threading import get_ident
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from scripts.dispatch_pair_skill import BoundPairSkill


def _rig(*, tag='phase-1-000', error=.018, schema='ugrp.rgb_varied_start_pose.v1'):
    owner=get_ident();clock=[.05];allowed=[True];authorized=[True]
    history={rid:[] for rid in ('r1','r3')}
    ports={}
    for rid in history:
        port=SimpleNamespace(_command_expires_at=None,_drive_expires_at=None,
                             _motor_commands=(0.,)*4,hold=Mock())
        def hold(now,port=port):
            port._command_expires_at=port._drive_expires_at=None
            port._motor_commands=(0.,)*4
        port.hold.side_effect=hold
        ports[rid]=port

    def issue(actions,duration,stage):
        assert get_ident()==owner and stage=='APPROACH'
        assert 0.<duration<=.25
        for rid,action in actions.items():
            port=ports[rid];f,l,t=(action[k] for k in ('forward','left','turn'))
            port._motor_commands=(f-l-t,f+l+t,f+l-t,f-l+t)
            port._command_expires_at=port._drive_expires_at=clock[0]+duration
            history[rid].append({'stage':stage,'issued_at_s':clock[0],
                'valid_until_s':clock[0]+duration,'action':dict(action)})

    def step(seconds):
        assert get_ident()==owner
        clock[0]+=seconds
        for port in ports.values():
            if port._command_expires_at is not None and clock[0]>=port._command_expires_at:
                port.hold(clock[0])

    io=SimpleNamespace(realtime_control=True,time=lambda:clock[0],step=step,
        ports=ports,pair_issue_bounded=issue,pair_drive=Mock(),
        await_visual=Mock(side_effect=AssertionError('pending bridge must pump owner')),
        compute_visual=lambda fn:fn(),realtime_stats={'pair_stage_samples':0},
        command_history=history,
        authorize=lambda:authorized[0] or (_ for _ in ()).throw(RuntimeError('plan revoked')))
    class ReadyWorker:
        def submit(self,fn):
            result=Future();result.set_result(fn());return result
    io._decision_workers=ReadyWorker()
    binding=SimpleNamespace(static_map={'map_id':'dispatch_open','terrain':[],
        'obstacles':[]},cluttered=False,pair={'r1':'r1','r3':'r3'},
        committed={'plan_hash':'plan-a'},
        permission=lambda obj,stage:obj=='beam' and stage=='APPROACH' and allowed[0])
    pair=BoundPairSkill.__new__(BoundPairSkill)
    pair.io=io;pair.bindings=binding;pair.phase='APPROACH'
    pair.transport_started=False;pair.carried_beam=SimpleNamespace(previous=None)
    pair.calls=[];pair._approach_pending_lease=None;pair._fine_pending_lease=None
    pair._fine_candidate=None
    frames={rid:{'frame_id':7,'observed_at_s':0.} for rid in ('r1','r3')}
    pair.capture=lambda _tag:frames
    pair.last_capture=frames
    pair.stage_models={rid:{axis:{'schema':schema,'robot_id':rid,'stage':axis,
                                    'settings':{'tolerance':.0025 if axis=='lateral' else .004}}
                            for axis in ('lateral','forward')} for rid in frames}
    axis='lateral' if tag.startswith(('phase-1-','phase-5-')) else 'forward'
    command=.01 if axis=='lateral' else .03
    decisions={'r1':{'ok':True,'precision':'fine','ready':False,'command':command,
                     'diagnostics':{'image_derived_error':error}},
               'r3':{'ok':True,'precision':'fine','ready':True,'command':0.,
                     'diagnostics':{'image_derived_error':.0008}}}
    commands={rid:dict(forward=0.,left=0.,turn=0.) for rid in frames}
    commands['r1']['left' if axis=='lateral' else 'forward']=command
    return pair,clock,allowed,authorized,ports,history,frames,decisions,commands,tag


def _issue(rig):
    pair,_,_,_,_,history,_,decisions,commands,tag=rig
    pair.observe_and_compute(tag,lambda _frames:decisions)
    pair.drive_mecanum(commands)
    assert len(history['r1'])==len(history['r3'])==1
    return pair


def test_fine_bridge_extends_original_lease_once_during_next_rgb_wait():
    rig=_rig();pair,clock,_,_,ports,history,*_=rig
    _issue(rig)
    assert pair._fine_pending_lease is not None
    future=Future();original_step=pair.io.step
    def step(seconds):
        original_step(seconds)
        if clock[0]>=.33 and not future.done():future.set_result('next-rgb')
    pair.io.step=step
    assert pair._await_approach_visual(future)=='next-rgb'
    receipts=[r for r in pair.calls if r['kind']=='approach_fine_pending_bridge']
    assert len(receipts)==1
    receipt=receipts[0]
    assert receipt['axis']=='lateral' and receipt['phase_tag']=='phase-1-000'
    assert receipt['source_frame_ids']=={'r1':7,'r3':7}
    assert receipt['source_observed_at_s']==0.
    assert receipt['source_ready']=={'r1':False,'r3':True}
    assert receipt['far_error_m']=={'r1':.018}
    assert receipt['outer_tolerance_m']==.0025
    assert receipt['permission_verified_at_s']==receipt['issued_at_s']
    assert receipt['issued_at_s']<receipt['previous_valid_until_s']==.30
    assert receipt['valid_until_s']<=receipt['original_valid_until_s']+.05+1e-9
    assert receipt['valid_until_s']<=receipt['original_rgb_deadline_s']+.0
    assert receipt['total_extension_s']<=.05+1e-9
    assert 0.<receipt['duration_s']<=.25
    assert len(history['r1'])==len(history['r3'])==2
    for rid in ports:
        assert history[rid][0]['fine_bridge_eligible']
        assert history[rid][0]['phase_tag']=='phase-1-000'
        assert history[rid][1]['pending_renewal']
        assert all(history[rid][1]['action'][key]==history[rid][0]['action'][key]
                   for key in ('forward','left','turn'))


def test_fine_bridge_never_repeats_or_revives_an_expired_motor_lease():
    rig=_rig();pair,clock,_,_,ports,history,*_=rig
    _issue(rig)
    for _ in range(16):pair._advance_fine_pending()
    assert len(history['r1'])==len(history['r3'])==2
    assert pair._fine_pending_lease is None
    assert all(port._motor_commands==(0.,)*4 for port in ports.values())

    rig=_rig();pair,clock,_,_,ports,history,*_=rig
    _issue(rig)
    clock[0]=.31  # The original .30 motor lease expired before the owner tick.
    pair._advance_fine_pending()
    assert pair._fine_pending_lease is None
    assert len(history['r1'])==len(history['r3'])==1
    assert all(port._motor_commands==(0.,)*4 for port in ports.values())


@pytest.mark.parametrize('failure',['permission','authorization','plan','port_stop','port_replace','phase'])
def test_fine_bridge_revocation_replacement_and_phase_exit_hold(failure):
    rig=_rig();pair,clock,allowed,authorized,ports,history,*_=rig
    _issue(rig);clock[0]=.27
    if failure=='permission':allowed[0]=False
    elif failure=='authorization':authorized[0]=False
    elif failure=='plan':pair.bindings.committed['plan_hash']='plan-b'
    elif failure=='port_stop':ports['r1'].hold(clock[0])
    elif failure=='port_replace':ports['r1']._motor_commands=(.5,)*4
    else:pair.phase='grasp_initialization'
    if failure in ('permission','authorization','plan'):
        with pytest.raises(RuntimeError):pair._advance_fine_pending()
    else:pair._advance_fine_pending()
    assert pair._fine_pending_lease is None
    assert all(port._motor_commands==(0.,)*4 for port in ports.values())
    assert len(history['r1'])==len(history['r3'])==1


def test_stale_fine_decision_cannot_reuse_an_older_history_row():
    rig=_rig();pair,clock,_,_,_,history,_,decisions,commands,tag=rig
    pair.observe_and_compute(tag,lambda _frames:decisions)
    clock[0]=.61  # The source RGB expired before original motor issue.
    pair.drive_mecanum(commands)
    assert pair._fine_candidate is None and pair._fine_pending_lease is None
    assert not history['r1'] and not history['r3']


@pytest.mark.parametrize('tag,error,schema',[
    ('phase-1-000',.004,'ugrp.rgb_varied_start_pose.v1'),
    ('phase-0-000',.018,'ugrp.rgb_varied_start_pose.v1'),
    ('final-refinement-000',.018,'ugrp.rgb_varied_start_pose.v1'),
    ('dock',.018,'ugrp.rgb_varied_start_pose.v1'),
    ('phase-1-000',.018,'unvalidated-model'),
])
def test_fine_bridge_excludes_near_ready_yaw_final_dock_and_unsupported(tag,error,schema):
    rig=_rig(tag=tag,error=error,schema=schema)
    pair=_issue(rig)
    assert pair._fine_pending_lease is None
    assert not any(row['kind']=='approach_fine_pending_bridge' for row in pair.calls)


def test_new_stationary_fine_decision_and_zero_command_remove_old_authority():
    rig=_rig();pair,_,_,_,ports,history,_,decisions,_,_=rig
    _issue(rig)
    ready={rid:{**row,'ready':True,'command':0.} for rid,row in decisions.items()}
    pair.observe_and_compute('phase-1-001',lambda _frames:ready)
    assert pair._fine_pending_lease is None
    assert all(port._motor_commands==(0.,)*4 for port in ports.values())
    assert len(history['r1'])==1

    rig=_rig();pair,_,_,_,ports,history,*_=rig
    _issue(rig)
    pair.drive_mecanum({rid:dict(forward=0.,left=0.,turn=0.) for rid in ports},.25)
    assert pair._fine_pending_lease is None
    assert all(port._motor_commands==(0.,)*4 for port in ports.values())
    assert len(history['r1'])==1


@pytest.mark.parametrize('next_tag',['phase-2-000','final-refinement-000','dock'])
def test_next_phase_holds_old_fine_lease_before_rgb_capture(next_tag):
    rig=_rig();pair,_,_,_,ports,history,frames,*_=rig
    _issue(rig)
    def capture(tag):
        assert tag==next_tag
        assert pair._fine_pending_lease is None
        assert all(port._motor_commands==(0.,)*4 for port in ports.values())
        return frames
    pair.capture=capture
    pair.observe_and_compute(next_tag,lambda _frames:{})
    assert len(history['r1'])==len(history['r3'])==1
