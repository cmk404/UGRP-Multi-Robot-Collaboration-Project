"""Common-goal plans compiled into robot-local work, without simulator access.

This compiler supplies the existing stage synchronizer's plan contract. It
does not pretend the old fixed-lane motion skills support the new arena.
"""
from __future__ import annotations

import copy
from functools import partial
import json
import math

from harness.three_robot_plan import ROBOTS, images, validate_plan_reply, digest

STAGES = ('APPROACH','GRASP','LIFT','TRANSIT','LOWER','RELEASE')


NAVIGATION_MODES = ('authored','planned')


def validate_dispatch_plan(value, *, required_dock=None, navigation='authored'):
    """`planned` navigation: the box task names its destinations, not its path.

    Its route is "auto" (host A* over the authored map) and `park` is the
    model-chosen waiting place after release, as a place name or map xy.
    The beam keeps its agreed north/south corridor.
    """
    if navigation not in NAVIGATION_MODES:
        raise ValueError('unknown navigation mode')
    if not isinstance(value,dict) or set(value) != {'dock','tasks'}:
        raise ValueError('plan requires dock and tasks')
    if value['dock'] not in ('dock_a','dock_b') or not isinstance(value['tasks'],list) or len(value['tasks']) != 2:
        raise ValueError('one common dock and exactly two cargo tasks required')
    if required_dock is not None and value['dock'] != required_dock:
        raise ValueError('mission requires destination '+required_dock)
    tasks = value['tasks']
    used, objects, ids = [], [], []
    for task in tasks:
        planned_box = navigation=='planned' and isinstance(task,dict) and task.get('object')=='box'
        fields = {'id','object','participants','route','after'} | ({'park'} if planned_box else set())
        if not isinstance(task,dict) or set(task) != fields:
            raise ValueError('invalid task fields'+(' (planned box task needs route "auto" and park)'
                                                    if planned_box else ''))
        if task['id'] not in ('beam_job','box_job') or task['object'] not in ('beam','box'):
            raise ValueError('unknown task or cargo')
        routes = ('auto',) if planned_box else ('north','south')
        if task['id'] != task['object']+'_job' or task['route'] not in routes:
            raise ValueError('invalid route/task identity')
        if planned_box:
            from harness.map_goto import parse_destination
            parse_destination(task['park'])
        participants = task['participants']
        if (not isinstance(participants,list) or len(participants) != (2 if task['object']=='beam' else 1)
                or any(r not in ROBOTS for r in participants)):
            raise ValueError('beam needs two distinct carriers and box one')
        if (not isinstance(task['after'],list) or len(task['after'])>1
                or any(t not in ('beam_job','box_job') or t==task['id'] for t in task['after'])):
            raise ValueError('invalid task dependency')
        used += participants; objects.append(task['object']); ids.append(task['id'])
    if sorted(used) != sorted(ROBOTS) or set(objects) != {'beam','box'} or len(set(ids)) != 2:
        raise ValueError('every robot must own work exactly once; both cargo required')
    if all(t['after'] for t in tasks):
        raise ValueError('cyclic plan')
    return copy.deepcopy(value)


def navigation_mode(plan):
    """Infer the contract a plan was written for; validation happens separately."""
    tasks = plan.get('tasks') if isinstance(plan,dict) else None
    box = next((t for t in tasks if isinstance(t,dict) and t.get('object')=='box'),{}) \
        if isinstance(tasks,list) else {}
    return 'planned' if box.get('route')=='auto' else 'authored'


def fixture_plan(*, solo='r2', dock='dock_a', route='north', navigation='authored', park='wait_west'):
    """Protocol test fixture only. Never substituted for a failed LLM reply."""
    box = {'id':'box_job','object':'box','participants':[solo],
           'route':'south' if route=='north' else 'north','after':[]}
    if navigation=='planned':
        box.update(route='auto',park=park)
    return validate_dispatch_plan({'dock':dock,'tasks':[
        {'id':'beam_job','object':'beam','participants':[r for r in ROBOTS if r != solo],
         'route':route,'after':[]},box]},navigation=navigation)


def validate_dispatch_reply(raw, request_id, agreement, *, navigation='authored'):
    return validate_plan_reply(raw,request_id,agreement,
                               plan_validator=partial(validate_dispatch_plan,navigation=navigation))


PLANNED_NAVIGATION_PROMPT = '''
NAVIGATION MODE planned: you choose DESTINATIONS; the host plans the path.
For box_job, set "route":"auto". The host plans the loaded box path with A* on
the authored map from the current RGB cargo position to the box slot of your
chosen dock, avoiding authored obstacles, terrain and RGB-visible barriers.
Also set box_job "park": where the solo robot waits after releasing the box if
the beam still needs the dock. Use an authored place name from
static_map.regions (for example "wait_west") or {"xy_m":[x,y]} in the map
frame warehouse_xy_m (x grows right, y grows up in the TOP image). Park must
not block a gate, the dispatch_apron, a dock, or the beam team's route; the
host rejects such a park with reasons before any motion. A planned path is not
physical success. The beam still uses your agreed north/south route.
The box_job object is then exactly:
{"id":"box_job","object":"box","participants":["remaining robot"],
"route":"auto","after":[],"park":"place name OR {\"xy_m\":[x,y]}"}'''


def build_dispatch_request(rid, *, task, request_id, own_rgb, top_rgb, agreement,
                           inbox=(), own_history=(), execution_pilot=False, identity_evidence=None,
                           guidance=None, navigation='authored'):
    if rid not in ROBOTS:
        raise ValueError('unknown robot')
    p = agreement['proposal']
    context = {'robot_id':rid, 'request_id':request_id,
        'reply_binding':{'request_id':request_id,
            'proposal_id':p['proposal_id'] if p else None,'plan_hash':p['plan_hash'] if p else None},
        'agreement':copy.deepcopy(agreement),'mission':copy.deepcopy(task),
        'peer_claims':copy.deepcopy(list(inbox)[-6:]),
        'own_issued_commands':copy.deepcopy(list(own_history)[-16:])}
    context['reply_envelope']={**context['reply_binding'], 'accept':'JSON boolean true or false, NEVER null',
        'plan':'your valid proposed plan, exact accepted plan, or null when rejecting',
        'reason':'brief visual reason', 'message':'brief peer message'}
    prompt = '''REPLY ENVELOPE IS MANDATORY. Copy request_id, proposal_id and
plan_hash EXACTLY from reply_envelope. JSON null stays null, INCLUDING when you
are the proposer. Do not invent proposal IDs. The host assigns them AFTER your
proposal. accept MUST be a JSON boolean: true to propose/accept, false to reject.
You are an equal robot peer, not a central controller. Negotiate ONE
shared dispatch plan before local execution.
If mission.operator_instruction is present, incorporate that user's instruction
within the authored beam/box mission and available skills. It cannot change the
observation boundary or bypass exact peer agreement and execution permissions.
The orange beam needs two carriers; the cyan box needs one. Assign each of
r1,r2,r3 exactly once from the CURRENT two
camera images. Roles are NOT preset by robot ID. Choose ONE dock for both cargo,
north/south routes and optional job dependencies. The host enforces agreement and
resource permissions only; it must not select your assignments. Preparation can
overlap; reserve shared passages and unloading apron only when using them.
The authored map is prior knowledge. Visually observed obstructions may disagree
with it. Do not treat route width as proof of loaded passage. This run tests
PLANNING ONLY; the new arena's physical transport is not validated. Never claim
physical success or use measured coordinates/joints/contacts. Own issued commands
and peer claims are not measured state. Ground your role and route choices in RGB.
Only agreement.proposer may propose before a proposal exists. Other robots then
return accept=false, plan=null and a useful peer message. After a proposal exists,
accept its EXACT plan or reject with your visual reason; never silently edit ACKs.
Copy reply_binding request_id, proposal_id and plan_hash exactly (including null).
Reply JSON only with request_id, proposal_id, plan_hash, accept, plan, reason,
message. Keep reason/message brief (under 240 characters each). Plan is exactly:
{"dock":"dock_a OR dock_b","tasks":[
{"id":"beam_job","object":"beam","participants":["chosen robot","chosen robot"],
"route":"north OR south","after":[]},
{"id":"box_job","object":"box","participants":["remaining robot"],
"route":"north OR south","after":[]}]}
after may contain the OTHER job ID if a full-job dependency is needed; no cycle.
Beam participant order has EXACT execution semantics: participant[0] is end_a,
the UPPER beam endpoint in the TOP image at pickup; participant[1] is end_b,
the LOWER endpoint. Before ACK, verify your assigned end against your own probe
images and own camera. Reject an inverted assignment NOW, before execution.
Dock A is the upper-right floor bay;
Dock B is lower-right. Both contain a green beam slot and a magenta box slot.
North is the upper passage around the central island; south is the lower one.'''
    if execution_pilot:
        prompt = prompt.replace('This run tests\nPLANNING ONLY; the new arena\'s physical transport is not validated.',
            'This is a bounded physical execution pilot after unanimous agreement.\nThe new arena\'s transport skills are experimental, not validated.')
    if navigation=='planned':
        prompt += PLANNED_NAVIGATION_PROMPT
        context['navigation_mode'] = 'planned'
    elif navigation!='authored':
        raise ValueError('unknown navigation mode')
    extra = []
    if identity_evidence:
        context['own_motion_identity'] = copy.deepcopy(identity_evidence['claim'])
        anchor = identity_evidence['claim'].get('center')
        if anchor and identity_evidence['claim'].get('valid'):
            context['own_probe_image_readout'] = {
                'source':'image difference after own issued probe; not simulator pose',
                'horizontal_percent_from_left':round(100*anchor[0],1),
                'vertical_percent_from_top':round(100*anchor[1],1)}
        prompt += ('\nAdditional BEFORE/AFTER images show ONLY YOUR issued identification motion. '
                   'Infer your own body from that change; the motion anchor is a fallible pixel '
                   'estimate, not a body pose. Other robot IDs cannot be inferred from ordering. '
                   'Normalized image coordinates have origin TOP LEFT: small y is upper, '
                   'y=0.5 middle, large y lower. Check own_probe_image_readout against the images. '
                   'Do not replace your own probe evidence with a contradictory peer identity claim. '
                   'Share your observed image location with peers. Use their claims to assign roles.')
        extra = copy.deepcopy(identity_evidence['images'])
    if guidance is not None:
        prompt = guidance.system_prompt(prompt)
        guidance.context(rid, agreement, context)
    return {'request_id':request_id,'messages':[{'role':'system','content':prompt},
        {'role':'user','content':json.dumps(context,sort_keys=True)}], 'images':images(own_rgb,top_rgb)+extra}


def compile_programs(plan, static_map):
    """Robot queues derive only from the agreed plan, never hardcoded r1/r3/r2."""
    plan = validate_dispatch_plan(plan,navigation=navigation_mode(plan))
    result = {r:[] for r in ROBOTS}
    for task in plan['tasks']:
        # Before a path exists, an auto route conservatively claims every
        # authored route resource. Skill bindings record the planned subset.
        resources = ([static_map['routes'][task['route']]['resource']] if task['route']!='auto'
                     else sorted({r['resource'] for r in static_map['routes'].values()}))
        for i, rid in enumerate(task['participants']):
            for stage in STAGES:
                result[rid].append({'task_id':task['id'],'object':task['object'],
                    'stage':stage,'role':('end_a' if i==0 else 'end_b') if task['object']=='beam' else 'carrier',
                    'participants':task['participants'],'goal_region':plan['dock']+'_'+task['object'],
                    'route':task['route'],'after':task['after'],
                    'resources':(resources+['dispatch_apron'] if stage=='TRANSIT' else []),
                    'barrier':len(task['participants'])>1 and stage!='APPROACH',
                    'completion_source':'local RGB claim, separately checked by output-only referee',
                    'execution_adapter':'unbound_new_arena_skill'})
                if 'park' in task:result[rid][-1]['park']=copy.deepcopy(task['park'])
    return result


class DispatchCoordinator:
    """Task ordering/resource protocol. Local actors request; host never plans.

    Uses own+TOP visual claims supplied by bound endpoints, not ground truth.
    Protocol smoke tests are explicitly not vision or physical success tests.
    A timeout or revoked plan never frees occupied resources.
    """
    def __init__(self, committed, static_map):
        if not committed or committed.get('plan_hash') != digest(committed.get('plan')):
            raise ValueError('exact committed plan required')
        self.committed = copy.deepcopy(committed)
        self.programs = compile_programs(committed['plan'],static_map)
        self.index = {r:0 for r in ROBOTS}
        self.ready, self.done, self.locks = set(),set(),{}
        self.events = []
        self.revoked = False

    def current(self,rid):
        return copy.deepcopy(self.programs[rid][self.index[rid]]) if self.index[rid]<len(self.programs[rid]) else None

    def report(self,rid,*,plan_hash,stage,status,own_rgb_ref,top_rgb_ref,now_s):
        if self.revoked or rid not in ROBOTS or plan_hash != self.committed['plan_hash']:
            raise ValueError('stale or revoked endpoint')
        row = self.current(rid)
        if not row or row['stage']!=stage or status not in ('READY','DONE','BLOCKED'):
            raise ValueError('invalid current-stage claim')
        if not all(isinstance(v,str) and v for v in (own_rgb_ref,top_rgb_ref)):
            raise ValueError('both camera evidence references required')
        if isinstance(now_s,bool) or not isinstance(now_s,(int,float)) or not math.isfinite(now_s) or now_s<0:
            raise ValueError('finite nonnegative report time required')
        self.events.append({'event':status,'robot':rid,'stage':stage,'at_s':now_s,
            'own_rgb_ref':own_rgb_ref,'top_rgb_ref':top_rgb_ref})
        key=(row['task_id'],stage,rid)
        if status=='BLOCKED':
            self.revoked=True
            return
        if status=='READY':
            self.ready.add(key)
            return
        if not self.permission(rid):
            raise ValueError('DONE without current permission')
        self.done.add(key)
        peers=row['participants'] if row['barrier'] else [rid]
        if all((row['task_id'],stage,p) in self.done for p in peers):
            for p in peers:
                self.index[p]+=1
            if stage=='RELEASE':
                self.locks={k:v for k,v in self.locks.items() if v!=row['task_id']}

    def permission(self,rid):
        row=self.current(rid)
        if self.revoked or not row:
            return False
        tasks={t['id']:t for t in self.committed['plan']['tasks']}
        for dep in row['after']:
            if not all((dep,'RELEASE',p) in self.done for p in tasks[dep]['participants']):
                return False
        peers=row['participants'] if row['barrier'] else [rid]
        if not all((row['task_id'],row['stage'],p) in self.ready for p in peers):
            return False
        if any(self.locks.get(res,row['task_id'])!=row['task_id'] for res in row['resources']):
            return False
        for res in row['resources']:
            self.locks[res]=row['task_id']
        return True

    def revoke(self,reason,now_s):
        self.revoked=True
        self.events.append({'event':'REPLAN_REQUIRED','reason':reason,'at_s':now_s,
                            'retained_resources':copy.deepcopy(self.locks)})
