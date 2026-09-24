"""An accepted open coarse command may bridge only its next RGB wait."""
from concurrent.futures import Future
from threading import get_ident
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from scripts.dispatch_pair_skill import BoundPairSkill, coordinated_coarse_commands


def _rig(*, map_id='dispatch_open', terrain=(), obstacles=()):
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
            port=ports[rid]
            f,l,t=(action[k] for k in ('forward','left','turn'))
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
        await_visual=Mock(side_effect=AssertionError('owner renew path bypassed')),
        command_history=history,
        authorize=lambda:authorized[0] or (_ for _ in ()).throw(RuntimeError('plan revoked')))
    binding=SimpleNamespace(static_map={'map_id':map_id,'terrain':list(terrain),
        'obstacles':list(obstacles)},cluttered=False,pair={'r1':'r1','r3':'r3'},
        committed={'plan_hash':'plan-a'},
        permission=lambda obj,stage:obj=='beam' and stage=='APPROACH' and allowed[0])
    pair=BoundPairSkill.__new__(BoundPairSkill)
    pair.io=io;pair.bindings=binding;pair.phase='APPROACH'
    pair.transport_started=False;pair.carried_beam=SimpleNamespace(previous=None)
    pair.calls=[];pair.last_capture={'r1':{'observed_at_s':0.}}
    decisions={rid:{'ok':True,'ready':False,'forward':.08,'left':0.,'turn':0.,
                    'image_error':[.2,0.]} for rid in ('r1','r3')}
    commands,_=coordinated_coarse_commands(decisions)
    frames={rid:{'frame_id':7,'observed_at_s':0.} for rid in ('r1','r3')}
    return pair,clock,allowed,authorized,ports,history,frames,decisions,commands


def _accepted(rig):
    pair,clock,_,_,_,history,frames,decisions,commands=rig
    pair.drive_mecanum(commands)
    pair._accept_coarse_pending(frames,decisions,commands)
    assert len(history['r1'])==1
    return pair


def test_pending_rgb_renews_same_pair_command_and_preserves_original_deadline():
    rig=_rig();pair,clock,_,_,ports,history,*_=rig
    _accepted(rig)
    future=Future();original_step=pair.io.step
    def step(seconds):
        original_step(seconds)
        if clock[0]>=.55 and not future.done():future.set_result('fresh-rgb')
    pair.io.step=step
    assert pair._await_approach_visual(future)=='fresh-rgb'
    receipts=[row for row in pair.calls if row['kind']=='approach_coarse_pending_renewal']
    assert len(receipts)>=2
    assert all(row['source_frame_ids']=={'r1':7,'r3':7}
               and row['source_observed_at_s']==0.
               and row['permission_verified_at_s']==row['issued_at_s']
               and row['issued_at_s']<row['previous_valid_until_s']
               and row['valid_until_s']<=.6+1e-9
               and 0.<row['duration_s']<=.25 for row in receipts)
    assert len(history['r1'])==len(history['r3'])==1+len(receipts)
    assert all(history[rid][0]['observed_at_s']==0. for rid in ports)
    for rid in ports:
        assert all(row['action']['forward']==.08 for row in history[rid])


def test_expired_prior_port_lease_cannot_reanimate_inside_rgb_ttl():
    rig=_rig();pair,clock,_,_,ports,history,*_=rig
    _accepted(rig)
    clock[0]=.31  # Original .30 lease has elapsed, RGB TTL remains .60.
    pair._advance_approach_pending()
    assert pair._approach_pending_lease is None
    assert all(port.hold.call_count==1 for port in ports.values())
    assert len(history['r1'])==len(history['r3'])==1


def test_original_rgb_deadline_stops_pending_motion_without_new_frame():
    rig=_rig();pair,clock,_,_,ports,history,*_=rig
    _accepted(rig)
    for _ in range(30):
        pair._advance_approach_pending()
    assert clock[0]>=.6
    pair._advance_approach_pending()
    assert pair._approach_pending_lease is None
    assert all(port._motor_commands==(0.,)*4 for port in ports.values())
    assert all(row['valid_until_s']<=.6+1e-9 for row in history['r1'])
    assert len(history['r1'])==len(history['r3'])


def test_completed_new_rgb_and_cancelled_worker_both_clear_old_authority():
    rig=_rig();pair,clock,_,_,ports,history,frames,*_=rig
    _accepted(rig)
    pair.capture=Mock(return_value=frames)
    pair.io.realtime_stats={'pair_stage_samples':0}
    class ReadyWorker:
        def submit(self,fn):
            done=Future();done.set_result(fn());return done
    pair.io._decision_workers=ReadyWorker()
    returned,value=pair.observe_and_compute('coarse',lambda _frames:'new-decision')
    assert returned is frames and value=='new-decision'
    assert pair._approach_pending_lease is None
    assert all(port.hold.call_count==1 for port in ports.values())

    rig=_rig();pair,clock,_,_,ports,history,*_=rig
    _accepted(rig)
    cancelled=Future();cancelled.cancel()
    from concurrent.futures import CancelledError
    with pytest.raises(CancelledError):pair._await_approach_visual(cancelled)
    assert pair._approach_pending_lease is None
    assert all(port.hold.call_count==1 for port in ports.values())


@pytest.mark.parametrize('failure', ['permission','plan','port_stop','phase'])
def test_revocation_stop_and_phase_exit_hold_without_renewal(failure):
    rig=_rig();pair,clock,allowed,authorized,ports,history,*_=rig
    _accepted(rig)
    clock[0]=.27
    if failure=='permission':allowed[0]=False
    elif failure=='plan':pair.bindings.committed['plan_hash']='plan-b'
    elif failure=='port_stop':ports['r1'].hold(clock[0])
    else:pair.phase='grasp_initialization'
    if failure in ('permission','plan'):
        with pytest.raises(RuntimeError):pair._advance_approach_pending()
    else:pair._advance_approach_pending()
    assert pair._approach_pending_lease is None
    assert all(port._motor_commands==(0.,)*4 for port in ports.values())
    assert len(history['r1'])==len(history['r3'])==1


def test_zero_and_near_handoff_cannot_create_renewal_authority():
    rig=_rig();pair,clock,_,_,ports,history,frames,decisions,commands=rig
    _accepted(rig)
    zero={rid:{'forward':0.,'left':0.,'turn':0.} for rid in ('r1','r3')}
    pair.drive_mecanum(zero,.25)
    assert pair._approach_pending_lease is None
    assert all(port.hold.call_count==1 for port in ports.values())
    assert pair.io.pair_drive.call_count==1
    decisions={rid:{**value,'image_error':[.06,0.]} for rid,value in decisions.items()}
    pair._accept_coarse_pending(frames,decisions,commands)
    assert pair._approach_pending_lease is None


@pytest.mark.parametrize('scope', [
    {'map_id':'dispatch_shared_crossing'},
    {'terrain':({'id':'ridge'},)},
    {'obstacles':({'id':'interior'},)},
])
def test_unsupported_map_scope_never_renews(scope):
    rig=_rig(**scope);pair,clock,_,_,_,history,*_=rig
    _accepted(rig)
    assert pair._approach_pending_lease is None
    assert not any(row['kind']=='approach_coarse_pending_renewal' for row in pair.calls)
