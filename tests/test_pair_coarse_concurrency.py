"""Recorded coarse failure and fail-closed concurrent RGB admission."""
from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.dispatch_pair_skill import (BoundPairSkill, coordinated_coarse_commands,
    _coarse_concurrent_issue_age)
from scripts.run_dispatch_e2e import main
from scripts.run_dispatch_skills import coarse_concurrency_status
from sim.research_dispatch_arena import authored_map, digest


FIXTURE = (Path(__file__).parent / 'fixtures/pair_coarse_concurrency'
           / 'south-train-a-first-last.json')
ROBOTS = ('r1', 'r3')


def frames(*, observed=1., now=1.1):
    return ({r:{'frame_id':7,'observed_at_s':observed,
                'raw_top_bytes':b'same saved TOP capture'} for r in ROBOTS}, now)


def recorded_rows():
    return json.loads(FIXTURE.read_text())['rows']


@pytest.mark.parametrize('row', recorded_rows())
def test_saved_teacher_forward_is_held_by_default_but_can_move_during_peer_alignment(row):
    decisions=row['decisions']
    old, old_evidence=coordinated_coarse_commands(decisions)
    assert old==row['recorded_commands']
    assert old_evidence['held_forward']==['r1']
    capture, now=frames()
    new, evidence=coordinated_coarse_commands(
        decisions, concurrent_alignment=True, frames=capture, now_s=now)
    assert new['r1']=={'forward':.12,'left':0.,'turn':0.}
    assert new['r3']=={k:decisions['r3'][k] for k in ('forward','left','turn')}
    assert evidence['concurrent_forward_with_peer_alignment']==['r1']
    assert evidence['held_forward']==[]
    assert evidence['common_top_capture']['frame_id']==7
    assert evidence['forward_gap_px']['r1']-evidence['forward_gap_px']['r3']<16


def test_leading_robot_stays_held_at_16_pixel_limit():
    decisions=deepcopy(recorded_rows()[0]['decisions'])
    decisions['r1']['image_error']=[.10,0.]
    decisions['r3'].update(image_error=[.14,0.],forward=.12,left=0.,turn=0.,
                           heading={'angle_deg':0.})
    capture,now=frames()
    commands,evidence=coordinated_coarse_commands(
        decisions,concurrent_alignment=True,frames=capture,now_s=now)
    assert commands['r1']['forward']==0.
    assert commands['r3']['forward']==.12
    assert evidence['held_forward']==['r1']


@pytest.mark.parametrize('damage',[
    lambda d,f: d['r1'].update(image_error=[float('nan'),0.]),
    lambda d,f: d['r1'].update(forward=float('inf')),
    lambda d,f: d['r1'].update(heading={'angle_deg':float('nan')}),
    lambda d,f: d['r1'].pop('ready'),
    lambda d,f: d['r1'].update(ready=True),
    lambda d,f: d['r1'].update(left=.01),
    lambda d,f: d['r1'].update(image_error=[.17,.02]),
    lambda d,f: d['r1'].update(heading={'angle_deg':2.}),
    lambda d,f: f['r3'].update(frame_id=8),
    lambda d,f: f['r3'].update(raw_top_bytes=b'other TOP'),
    lambda d,f: f['r3'].update(observed_at_s=1.01),
    lambda d,f: f['r1'].update(raw_top_bytes=b''),
])
def test_missing_or_unsafe_rgb_inputs_hold_both(damage):
    decisions=deepcopy(recorded_rows()[0]['decisions'])
    capture,now=frames()
    damage(decisions,capture)
    commands,evidence=coordinated_coarse_commands(
        decisions,concurrent_alignment=True,frames=capture,now_s=now)
    assert commands=={r:{'forward':0.,'left':0.,'turn':0.} for r in ROBOTS}
    assert evidence['admitted'] is False


@pytest.mark.parametrize('now', [.9,1.41,1.6,float('nan'),None])
def test_future_stale_or_unknown_capture_holds_both(now):
    capture,_=frames()
    commands,evidence=coordinated_coarse_commands(
        deepcopy(recorded_rows()[0]['decisions']),concurrent_alignment=True,
        frames=capture,now_s=now)
    assert all(not any(command.values()) for command in commands.values())
    assert evidence['admitted'] is False


@pytest.mark.parametrize('observed,issued,duration,accepted',[
    (1.,1.1,.2,True), (1.,1.4,.2,True),
    (1.,1.400001,.2,False), (1.,1.61,.2,False),
    (1.,.99,.2,False), (1.,1.1,float('nan'),False),
    (float('nan'),1.1,.2,False),
])
def test_issue_guard_includes_decision_to_actual_command_delay(observed,issued,duration,accepted):
    age=_coarse_concurrent_issue_age(observed,issued,duration)
    assert (age is not None) is accepted


def test_cluttered_runner_rechecks_ttl_at_actuator_issue():
    issued=[];held=[]
    pair=BoundPairSkill.__new__(BoundPairSkill)
    pair.io=SimpleNamespace(realtime_control=True,time=lambda:1.41,
        pair_drive=lambda *args:issued.append(args),
        ports={r:SimpleNamespace(hold=lambda now:held.append(now)) for r in ROBOTS})
    pair.bindings=SimpleNamespace(cluttered=True,pair={r:r for r in ROBOTS})
    pair.phase='APPROACH';pair.transport_started=False;pair.calls=[]
    pair._approach_pending_lease=None;pair._fine_pending_lease=None;pair._fine_candidate=None
    commands={r:dict(forward=.12,left=0.,turn=0.) for r in ROBOTS}
    assert pair.drive_mecanum(commands,coarse_observed_at_s=1.) is False
    assert issued==[] and len(held)==2
    assert pair.calls[-1]['kind']=='coarse_concurrent_stale_issue'
    pair.io.time=lambda:1.39
    assert pair.drive_mecanum(commands,coarse_observed_at_s=1.) is None
    assert len(issued)==1
    assert pair.calls[-1]['issue_age_s']==pytest.approx(.39)


@pytest.mark.parametrize('variant,expected',[
    ('shared_crossing',True), ('open',True),
    ('north_blocked',False), ('narrow_south',False), ('rough_south',False),
])
def test_runtime_scope_only_allows_exact_authored_maps(variant,expected):
    pair=BoundPairSkill.__new__(BoundPairSkill)
    static=authored_map(variant)
    pair.io=SimpleNamespace(realtime_control=True,config={
        'variant':variant,'static_map_sha256':digest(static)})
    pair.phase='APPROACH';pair.transport_started=False
    pair.bindings=SimpleNamespace(static_map=static)
    assert pair._concurrent_coarse_scope() is expected
    pair.coarse=object()
    assert coarse_concurrency_status(pair,requested=True)['applied'] is expected
    assert coarse_concurrency_status(pair,requested=False)=={
        'applied':False,'reason':'disabled'}
    pair.coarse=None
    assert coarse_concurrency_status(pair,requested=True)=={
        'applied':False,'reason':'own_motion_identity_unresolved'}
    pair.io.realtime_control=False
    assert pair._concurrent_coarse_scope() is False


def test_modified_or_unhashed_map_and_later_phase_are_not_applied():
    pair=BoundPairSkill.__new__(BoundPairSkill)
    static=authored_map('shared_crossing')
    pair.io=SimpleNamespace(realtime_control=True,config={
        'variant':'shared_crossing','static_map_sha256':digest(static)})
    pair.phase='APPROACH';pair.transport_started=False
    pair.bindings=SimpleNamespace(static_map=static)
    pair.coarse=object()
    static['obstacles'][0]['height_m']+=.01
    assert coarse_concurrency_status(pair,requested=True)['applied'] is False
    static['obstacles'][0]['height_m']-=.01
    pair.phase='GRASP'
    assert coarse_concurrency_status(pair,requested=True)['applied'] is False


def test_cli_requires_realtime_skills_and_carries_opt_in_to_runner(tmp_path,monkeypatch):
    command=['--output',str(tmp_path/'new'),'--grasp-model-dir',str(tmp_path/'grasp'),
             '--stage-model-dir',str(tmp_path/'stage'),'--coarse-concurrent-alignment']
    with pytest.raises(SystemExit):
        main(command)
    with pytest.raises(SystemExit):
        main([*command,'--realtime-control','--executor','raw'])
    received=[]
    monkeypatch.setattr('scripts.run_dispatch_skills.run',
                        lambda args: received.append(args.coarse_concurrent_alignment) or 0)
    assert main([*command,'--realtime-control'])==0
    assert received==[True]
