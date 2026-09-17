"""RGB-only raw-action pilot and fresh, plan-bound dispatch execution gate.

No world, pose, joints, contacts, evaluator or trained fixed-lane macro enters
this module. Claims remain fallible; protocol completion is not cargo success.
"""
from __future__ import annotations

import copy
import json
import math
import re

from harness.dispatch_plan import STAGES, compile_programs
from harness.task_stage_sync import READY_CHECKS, DONE_CHECKS, CHECKS
from harness.three_robot_plan import ROBOTS, digest, images


def checks(stage, done=False):
    if stage == 'APPROACH':
        return {'at_grasp_pose', 'stopped'} if done else set()
    return set((DONE_CHECKS if done else READY_CHECKS)['CARRY' if stage == 'TRANSIT' else stage])


def validate_action(action, stage):
    if not isinstance(action, dict):
        raise ValueError('action object required')
    kind = action.get('kind')
    fields = {'drive': {'kind','forward','turn','duration_s'},
              'mecanum': {'kind','forward','left','turn','duration_s'},
              'arm': {'kind','servo_id','pulse'}, 'look': {'kind','pan_pulse'}, 'wait': {'kind'}}
    if kind not in fields or set(action) != fields[kind]:
        raise ValueError('invalid action fields')
    allowed = {'drive','mecanum','arm','look','wait'} if stage == 'APPROACH' else (
        {'drive','mecanum','wait'} if stage == 'TRANSIT' else {'arm','wait'})
    if kind not in allowed:
        raise ValueError('action is not allowed in this stage')
    def bound(key, lo, hi, integer=False):
        x = action[key]
        if (isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x)
                or not lo <= x <= hi or (integer and not isinstance(x,int))):
            raise ValueError('invalid '+key)
    if kind in ('drive','mecanum'):
        bound('forward',-.05,.15);bound('turn',-.15,.15)
        if kind == 'mecanum':bound('left',-.1,.1)
        bound('duration_s',.2,1. if stage == 'APPROACH' else .2)
    if kind == 'arm':
        bound('servo_id',1,5,True);bound('pulse',500,2500,True)
        if action['servo_id'] not in (1,3,4,5):raise ValueError('invalid servo')
        if stage == 'APPROACH' and action['servo_id'] == 1 and action['pulse'] != 2000:
            raise ValueError('approach may only open jaws')
    if kind == 'look':bound('pan_pulse',500,2500,True)
    return copy.deepcopy(action)


def validate_reply(raw, request_id, *, plan_hash, stage, robot_id=None):
    text = raw.strip()
    fence = re.fullmatch(r'```(?:json)?\s*\n([\s\S]*?)\n```',text,re.I)
    value = json.loads(fence.group(1) if fence else text)
    fields = {'request_id','plan_hash','stage','status','confidence','checks','command_id','action','reason','message'}
    if not isinstance(value,dict):raise ValueError('reply object required')
    # Some providers echo robot_id from context. Accept only this known optional
    # field, and bind it to the trusted endpoint; never discard arbitrary extras.
    if 'robot_id' in value:
        if robot_id is None or value['robot_id'] != robot_id:raise ValueError('robot endpoint mismatch')
        fields = fields | {'robot_id'}
    if set(value) != fields:raise ValueError('invalid reply fields')
    if (value['request_id'],value['plan_hash'],value['stage']) != (request_id,plan_hash,stage):
        raise ValueError('stale request, plan or stage')
    if value['status'] not in ('READY','WORKING','DONE','UNCERTAIN','BLOCKED'):raise ValueError('invalid status')
    c = value['confidence']
    if isinstance(c,bool) or not isinstance(c,(int,float)) or not math.isfinite(c) or not 0 <= c <= 1:
        raise ValueError('invalid confidence')
    if (not isinstance(value['checks'],list) or any(not isinstance(c,str) or c not in CHECKS for c in value['checks'])
            or len(set(value['checks'])) != len(value['checks'])):raise ValueError('invalid checks')
    for key in ('reason','message'):
        if not isinstance(value[key],str) or len(value[key]) > 800:raise ValueError('invalid text')
    if value['command_id'] is not None and (not isinstance(value['command_id'],str) or not value['command_id']):
        raise ValueError('invalid command reference')
    value['action'] = validate_action(value['action'],stage)
    if value['status'] in ('DONE','UNCERTAIN','BLOCKED') and value['action'] != {'kind':'wait'}:
        raise ValueError('DONE, UNCERTAIN and BLOCKED require wait')
    return value


def build_request(rid, *, request_id, committed, program, task, frame, previous,
                  own_history, inbox, identity, feedback):
    stage = program['stage']
    context = {'robot_id':rid,'request_id':request_id,'plan_hash':committed['plan_hash'],
        'stage':stage,'committed_plan':copy.deepcopy(committed['plan']),
        'own_program':copy.deepcopy(program),'mission':copy.deepcopy(task),
        'own_issued_commands':copy.deepcopy(own_history[-20:]),
        'peer_claims':copy.deepcopy(inbox[-8:]),'own_initial_motion_identity':copy.deepcopy(identity),
        'execution_feedback':copy.deepcopy(feedback),
        'ready_checks':sorted(checks(stage)), 'done_checks':sorted(checks(stage,True))}
    prompt = '''You are an independent robot executing ONE unanimously agreed dispatch plan.
Use only your OWN camera, the shared TOP camera, their previous images, authored
static map, YOUR issued commands and peer claims. No measured state is supplied.
Your ID does not identify a screen position. The initial motion identity is only
a fallible historical pixel anchor; compare current images, do not treat it as a
current pose. Never silently change cargo/role/route/dock. BLOCKED requests a safe
stop and replan; it does not release occupied resources. end_a is the upper beam
end in TOP at pickup, end_b the lower end. The solo carrier grips the cyan box.
Commands are raw, untrained pilot actions. No successful grasp macro is supplied.
APPROACH: move toward your cargo and align OPEN jaws. Only visually straddling
the correct cargo with open jaws, stopped, supports DONE; proximity is not enough.
Preparation can overlap across jobs. Job dependencies gate GRASP onward.
GRASP closes; LIFT raises; TRANSIT follows the agreed passage to the chosen slot;
LOWER establishes support; RELEASE opens. Never drag cargo as if it were lifted.
Use READY with all ready_checks only if images establish them. Use WORKING to
adjust during APPROACH. Else use UNCERTAIN and wait. DONE needs all done_checks,
confidence >=0.8 and a new image after your last command in THIS stage; echo that
command_id. A command target does NOT establish its measured state or success.
Both beam carriers need fresh READY for joint motion. Wait for your partner;
do not switch roles when temporarily held. Unknown visual checks must be omitted.
Fixed hardware command documentation (not sensor measurements):
drive {kind,forward:-0.05..0.15,turn:-0.15..0.15,duration_s:0.2..1.0}.
mecanum {kind,forward:-0.05..0.15,left:-0.1..0.1,turn:-0.15..0.15,duration_s:0.2..1.0}.
Positive forward goes toward your front; positive left strafes to your left;
positive turn turns left. Use up to 1 second only with clear visible space during
APPROACH; shorten near cargo/peers. TRANSIT duration_s must be exactly 0.2.
arm {kind,servo_id:1|3|4|5,pulse:500..2500}. 1 gripper:2000 opens,1500 closes.
Increasing 5 lowers shoulder; increasing 4 bends elbow more; increasing 3 raises
wrist pitch. Arm commands have 0.2-second leases, max 2000 pulse units/second;
a distant target may not finish, so reobserve and repeat if needed.
look {kind,pan_pulse:500..2500}: rotates whole arm/camera,1500 forward,2500 left,
500 right. APPROACH only; does not rotate the base. wait {kind:"wait"}.
Only drive/mecanum/wait in TRANSIT; only arm/wait in other joint stages.
Return JSON only, exactly {request_id,plan_hash,stage,status,confidence,checks,
command_id,action,reason,message}. Copy current IDs/stage exactly. status is
READY|WORKING|DONE|UNCERTAIN|BLOCKED. checks is an array of names from ready_checks
and done_checks; command_id null until reporting DONE. DONE/UNCERTAIN/BLOCKED use
wait. Keep reason/message short. Explicitly describe what is unresolved if unsure.
'''
    rgb = images(frame['own_bytes'],frame['top_bytes'])
    if previous:
        rgb += [{'label':'PREVIOUS_'+i['label'],'image':i['image']}
                for i in images(previous['own_bytes'],previous['top_bytes'])]
    return {'request_id':request_id,'messages':[{'role':'system','content':prompt},
        {'role':'user','content':json.dumps(context,sort_keys=True)}],'images':rgb}


class DispatchExecution:
    """Plan/version/frame-bound RGB gate. Returns commands, never reads physics.

    Every batch is fresh, and each permission expires with its command. Peer
    readiness is not cached across batches. Resources remain owned on failures.
    """
    def __init__(self,committed,static_map):
        if not committed or committed.get('plan_hash') != digest(committed.get('plan')):
            raise ValueError('committed plan hash mismatch')
        self.committed = copy.deepcopy(committed)
        self.programs = compile_programs(committed['plan'],static_map)
        for rows in self.programs.values():
            for row in rows:row['execution_adapter'] = 'experimental_rgb_raw_actions_v1'
        self.index = {r:0 for r in ROBOTS}
        self.last_command = {r:None for r in ROBOTS}
        self.last_frame = {r:-1 for r in ROBOTS}
        self.locks = {};self.events = [];self.revoked = False
        self.feedback = {r:{} for r in ROBOTS}

    def current(self,rid):
        return copy.deepcopy(self.programs[rid][self.index[rid]]) if self.index[rid]<len(STAGES) else None

    @property
    def complete(self):return all(self.current(r) is None for r in ROBOTS)

    def revoke(self,reason):
        self.revoked = True
        self.events.append({'event':'REPLAN_REQUIRED','reason':reason,'retained_resources':copy.deepcopy(self.locks)})

    def batch(self,replies,*,frame_id,now_s):
        if self.revoked:return {}
        valid = {};programs = {r:self.current(r) for r in ROBOTS}
        for rid,row in programs.items():
            if not row:continue
            reply = replies.get(rid)
            self.feedback[rid] = {'reason':'no_valid_fresh_reply'}
            if (not reply or frame_id <= self.last_frame[rid]
                    or reply.get('stage') != row['stage'] or reply.get('plan_hash') != self.committed['plan_hash']):continue
            self.last_frame[rid] = frame_id
            if reply['status'] == 'BLOCKED':
                self.revoke(rid+': '+reply['reason']);return {}
            valid[rid] = reply
        finished = set()
        for rid,reply in valid.items():
            row = programs[rid];last = self.last_command[rid]
            if (reply['status']=='DONE' and reply['confidence'] >= .8
                    and checks(row['stage'],True) <= set(reply['checks'])
                    and last and last['stage']==row['stage'] and last['command_id']==reply['command_id']
                    and frame_id > last['frame_id']):finished.add(rid)
        # Advance atomically from the frozen batch; no old-stage action can run
        # after a transition. APPROACH is independent; joint stages need all peers.
        advanced = set()
        for rid in finished:
            row = programs[rid];peers = row['participants'] if row['barrier'] else [rid]
            if all(p in finished and programs[p]['stage']==row['stage'] for p in peers):advanced.update(peers)
        for rid in advanced:
            self.index[rid] += 1
            self.feedback[rid] = {'reason':'visual_claim_advanced','from':programs[rid]['stage']}
            self.events.append({'event':'STAGE_ADVANCED','robot':rid,'stage':programs[rid]['stage'],
                                'frame_id':frame_id,'at_s':now_s})
        for task in self.committed['plan']['tasks']:
            if all(self.current(r) is None for r in task['participants']):
                self.locks = {k:v for k,v in self.locks.items() if v!=task['id']}
        commands = {}
        for rid,reply in valid.items():
            row = programs[rid]
            if rid in advanced:continue
            reason = None
            dependency_blocked = False
            if row['stage'] != 'APPROACH':
                for dep in row['after']:
                    if any(self.current(r) is not None for r in ROBOTS if self.programs[r][0]['task_id']==dep):
                        dependency_blocked = True
                peers = row['participants'] if row['barrier'] else [rid]
                for peer in peers:
                    q = valid.get(peer)
                    if (not q or programs[peer]['stage'] != row['stage'] or q['status']!='READY'
                            or q['confidence'] < .8 or not checks(row['stage']) <= set(q['checks'])):
                        reason = 'waiting_for_fresh_visual_readiness'
                # A completed gripper may hold while the other one finishes.
                if row['stage']=='GRASP' and rid not in finished:
                    if all(p in finished or (p in valid and programs[p]['stage']==row['stage']
                        and valid[p]['status']=='READY' and valid[p]['confidence']>=.8
                        and checks(row['stage'])<=set(valid[p]['checks'])) for p in peers):
                        if reason == 'waiting_for_fresh_visual_readiness':reason = None
                if dependency_blocked:reason = 'waiting_for_job_dependency'
            elif reply['status'] not in ('WORKING','READY'):
                reason = 'approach_unresolved_or_waiting_for_valid_done'
            if rid in finished:reason = 'holding_visual_done_for_partner'
            if not reason and any(self.locks.get(x,row['task_id'])!=row['task_id'] for x in row['resources']):
                reason = 'waiting_for_shared_resource'
            if reason:
                self.feedback[rid] = {'reason':reason};continue
            for resource in row['resources']:
                if resource not in self.locks:
                    self.locks[resource] = row['task_id']
                    self.events.append({'event':'RESOURCE_GRANTED','resource':resource,'task_id':row['task_id'],'at_s':now_s})
            action = reply['action']
            duration = action.get('duration_s',.2)
            command = {'command_id':f'{rid}-f{frame_id}-{row["stage"]}',
                       'action':copy.deepcopy(action),'duration_s':duration,
                       'stage':row['stage'],'frame_id':frame_id,'issued_at_s':now_s}
            self.last_command[rid] = copy.deepcopy(command)
            commands[rid] = command
            self.feedback[rid] = {'reason':'bounded_permission','command_id':command['command_id']}
        self.events.append({'event':'BATCH','frame_id':frame_id,'at_s':now_s,
            'commands':copy.deepcopy(commands),'feedback':copy.deepcopy(self.feedback),'resources':copy.deepcopy(self.locks)})
        return commands
