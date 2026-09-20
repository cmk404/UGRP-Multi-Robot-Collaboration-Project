"""Real gate/port logic with explicitly synthetic visual verdicts, not success evidence."""
import copy
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from harness.multi_object_execution import MultiObjectExecution
from harness.multi_object_plan import validate_plan
from harness.multi_object_tracking import CargoTracker,detections
from harness.task_stage_sync import READY_CHECKS,DONE_CHECKS
from harness.three_robot_plan import TeamAgreement,ROBOTS
from sim.camera_robot_port import CameraRobotPort
from sim.multi_object_scene import configuration,static_task
from sim.multi_object_suite import load_pilot,fixture_plan
from tests.test_task_stage_execution import Robot,Clock


CASES=load_pilot()[1]


def jpeg(boxes=((420,570),(420,360)),beams=((260,480),)):
    frame=np.zeros((720,960,3),np.uint8)
    for x,y in boxes:cv2.rectangle(frame,(x-6,y-7),(x+6,y+7),(200,200,30),-1)
    for x,y in beams:cv2.rectangle(frame,(x-6,y-58),(x+6,y+58),(20,140,240),-1)
    return cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,95])[1].tobytes()


def mission(name='staging_three'):
    return next(c for c in CASES if c['id']==name)['mission']


def agreement(m):
    plan=fixture_plan(m)
    # Serial fixed assignment for a repeat-use regression; never a live LLM plan.
    for t in plan['tasks']:
        kind=next(o['kind'] for o in m['objects'] if o['object_id']==next(j['object_id'] for j in m['tasks'] if j['task_id']==t['task_id']))
        t['participants']={'r1':'end_a','r3':'end_b'} if kind=='beam' else {'r2':'solo'}
    a=TeamAgreement('synthetic',plan_validator=lambda p:validate_plan(p,m))
    for turn in (0,1):
        p=a.pending
        a.receive({r:dict(request_id=f'synthetic-{r}-plan-{turn}',proposal_id=p['proposal_id'] if p else None,
            plan_hash=p['plan_hash'] if p else None,accept=True,plan=plan,reason='fixture',message='') for r in ROBOTS},turn)
    return a


class World:
    def __init__(self):
        self.data=Clock();self.robots={r:Robot() for r in ROBOTS}
    def robot(self,r):return self.robots[r]


def setup(tmp_path):
    task=static_task(configuration(next(c for c in CASES if c['id']=='staging_three')))
    w=World();ports={r:CameraRobotPort(w,r,allow_reverse=True,allow_mecanum=True) for r in ROBOTS}
    e=MultiObjectExecution(task,agreement(task['mission']),ports,tmp_path/'runtime')
    return e,w


def observe(e,index,now,image=None):
    image=image or jpeg()
    return e.observe({r:{'own_bytes':image,'top_bytes':image,'frame_id':index,
                        'own_rgb':{'path':f'{r}-{index}-own.jpg'},'shared_top_rgb':{'path':f'{index}-top.jpg'}} for r in ROBOTS},now_s=now)


def reply(req,status='READY'):
    stage='CARRY' if req['stage']=='TRANSIT' else req['stage']
    checks=({'at_grasp_pose','stopped'} if stage=='APPROACH' else
            (DONE_CHECKS if status=='DONE' else READY_CHECKS)[stage])
    action={'kind':'wait'}
    if status=='READY':
        action=({'kind':'mecanum','forward':.03,'left':0.,'turn':0.,'duration_s':.2}
                if stage in ('CARRY','APPROACH') else {'kind':'arm','servo_id':1,'pulse':1800})
    return {'request_id':req['request_id'],'plan_hash':req['plan_hash'],'stage':req['stage'],
            'status':status,'confidence':1.,'checks':sorted(checks),'action':action,
            'command_id':req['last_command']['command_id'] if status=='DONE' and req['last_command'] else None,
            'reason':'synthetic visual verdict','message':''}


def test_five_tasks_require_all_stages_then_reuse_solo_and_preserve_slot_occupancy(tmp_path):
    e,w=setup(tmp_path);i=0;now=0.;seen=[]
    while len(e.protocol.completed)<5 and i<80:
        i+=1;reqs=observe(e,i,now)
        assert reqs
        for r,req in reqs.items():
            assert req['object_id']==e.protocol.jobs[req['task_id']]['object_id']
            assert req['plan_version']==1
        tid=next(iter(reqs.values()))['task_id']
        if not seen or seen[-1]!=tid:seen.append(tid)
        done=all(req['last_command'] and req['last_command']['stage']==req['stage'] for req in reqs.values())
        e.batch({r:reply(req,'DONE' if done else 'READY') for r,req in reqs.items()},now_s=now)
        now=round(now+.21,8);e.tick(now)
    assert len(e.protocol.completed)==5
    assert seen==['stage_02','stage_03','deliver_01','deliver_02','deliver_03']
    assert len(e.protocol.residents)==3
    assert not e.protocol.locks and not e.active
    assert e.protocol.summary()['final_object_claims']==3
    assert any(row['action']['kind']=='arm' for row in e.history['r2'])
    assert all({row['stage'] for row in e.history[r]}>={'GRASP','LIFT','TRANSIT','LOWER','RELEASE'} for r in ROBOTS)
    e.close(now_s=now)


def test_missing_visual_identity_holds_live_port_and_retains_lease(tmp_path):
    e,w=setup(tmp_path);reqs=observe(e,1,0.)
    e.batch({r:reply(q) for r,q in reqs.items()},now_s=0.)
    assert any(w.robots['r2'].motor_calls[-1])
    assert not observe(e,2,.1,jpeg(boxes=()))
    assert w.robots['r2'].motor_calls[-1]==[0.,0.,0.,0.]
    assert e.protocol.locks and not e.protocol.completed


def test_expiry_stops_without_new_capture(tmp_path):
    e,w=setup(tmp_path);q=observe(e,1,0.)
    e.batch({r:reply(v) for r,v in q.items()},now_s=0.)
    e.tick(.201)
    assert w.robots['r2'].motor_calls[-1]==[0.,0.,0.,0.]
    e.tick(.9)
    assert not e.protocol.completed


@pytest.mark.parametrize('mutation',['stale','wrong_task','wrong_plan','long','done_without_command'])
def test_invalid_or_premature_reply_never_advances(tmp_path,mutation):
    e,w=setup(tmp_path);q=observe(e,1,0.);r=next(iter(q));v=reply(q[r])
    if mutation=='stale':v['request_id']='old'
    if mutation=='wrong_task':v['request_id']='deliver_01-r2-999'
    if mutation=='wrong_plan':v['plan_hash']='bad'
    if mutation=='long':v['action']['duration_s']=.4
    if mutation=='done_without_command':v=reply(q[r],'DONE')
    e.batch({r:v},now_s=0.)
    assert not e.history[r] and not e.protocol.completed
    assert e.active['stage_02']['phase']=='APPROACH'


def test_unanimous_plan_revocation_cancels_current_actuation(tmp_path):
    e,w=setup(tmp_path);q=observe(e,1,0.);e.batch({r:reply(v) for r,v in q.items()},now_s=0.)
    e.protocol.authority.invalidate('test')
    with pytest.raises(ValueError):e.tick(.1)
    assert w.robots['r2'].motor_calls[-1]==[0.,0.,0.,0.]
    assert e.protocol.locks


def test_identity_tracks_small_motion_and_recovers_brief_visibility_loss():
    t=CargoTracker(mission())
    a=t.observe(jpeg(),frame_id=1,now_s=0.)
    assert a['valid'] and a['tracks']['object_02']['center'][1]>.7
    b=t.observe(jpeg(boxes=((424,570),(420,360))),frame_id=2,now_s=.2)
    assert b['valid'] and b['tracks']['object_02']['center'][0]>a['tracks']['object_02']['center'][0]
    assert not t.observe(jpeg(boxes=((424,570),)),frame_id=3,now_s=.3)['valid']
    assert t.observe(jpeg(boxes=((424,570),(420,360))),frame_id=4,now_s=.4)['valid']


def test_ambiguous_crossing_latches_instead_of_swapping_ids():
    t=CargoTracker(mission(),slack=.04)
    assert t.observe(jpeg(boxes=((400,360),(445,360))),frame_id=1,now_s=0.)['valid']
    assert not t.observe(jpeg(boxes=((418,360),(438,360))),frame_id=2,now_s=.2)['valid']
    assert t.latched=='ambiguous_identity'
    assert not t.observe(jpeg(boxes=((400,360),(445,360))),frame_id=3,now_s=.3)['valid']


def test_stale_frame_and_long_gap_do_not_rebind():
    t=CargoTracker(mission());t.observe(jpeg(),frame_id=1,now_s=0.)
    with pytest.raises(ValueError):t.observe(jpeg(),frame_id=1,now_s=.1)
    assert not t.observe(jpeg(),frame_id=2,now_s=1.)['valid']
    assert t.latched=='identity_gap_requires_new_mission_binding'


def test_blue_wall_is_not_a_cyan_box():
    image=cv2.imdecode(np.frombuffer(jpeg(),np.uint8),cv2.IMREAD_COLOR)
    hsv=np.uint8([[[104,80,90]]]);color=cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)[0,0].tolist()
    cv2.rectangle(image,(30,80),(45,95),color,-1)
    assert len([d for d in detections(cv2.imencode('.jpg',image)[1].tobytes()) if d['kind']=='box'])==2


def test_decode_error_stops_active_motion(tmp_path):
    e,w=setup(tmp_path);q=observe(e,1,0.);e.batch({r:reply(v) for r,v in q.items()},now_s=0.)
    with pytest.raises(ValueError):observe(e,2,.1,b'not a JPEG')
    assert not e.identity
    assert w.robots['r2'].motor_calls[-1]==[0.,0.,0.,0.]


def test_old_stage_replies_cannot_start_new_stage(tmp_path):
    e,w=setup(tmp_path);q=observe(e,1,0.)
    e.batch({r:reply(v) for r,v in q.items()},now_s=0.)
    q=observe(e,2,.21);old={r:reply(v,'DONE') for r,v in q.items()}
    e.batch(old,now_s=.21)
    assert e.active['stage_02']['phase']=='STAGES'
    observe(e,3,.42);before=len(e.history['r2']);e.batch(old,now_s=.42)
    assert len(e.history['r2'])==before
    assert e.active['stage_02']['stage'].sync.stage=='GRASP'


def test_initial_proposer_instruction_matches_agreement_wire_contract():
    from scripts.run_multi_object_execution import plan_request
    task=static_task(configuration(next(c for c in CASES if c['id']=='staging_three')))
    request=plan_request('r1',task=task,identity={'r1':{'claim':{},'images':[]}},
        request_id='test',own_rgb=jpeg(),top_rgb=jpeg(),agreement={},inbox=[],own_history=[])
    assert 'plan with accept=true' in request['messages'][0]['content']
