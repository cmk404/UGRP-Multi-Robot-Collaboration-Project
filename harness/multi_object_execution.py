"""RGB-bound mission leases connected to actual bounded robot-local ports.

All five manipulation stages use TaskStageExecution, including solo jobs.
APPROACH is independently bounded under the same object/robot/slot lease.
Visual producer claims are never physical success labels. Revocation and loss
of identity retain all occupied resources and stop the affected actuators.
"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

from harness.dispatch_execution import validate_reply
from harness.multi_object_plan import MissionProtocol
from harness.multi_object_tracking import CargoTracker
from harness.task_stage_execution import TaskStageExecution
from harness.task_stage_sync import TaskPlan
from harness.three_robot_plan import ROBOTS


def write(path, value):
    Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


class MultiObjectExecution:
    def __init__(self, task, agreement, ports, output, *, now_s=0.):
        if set(ports) != set(ROBOTS) or any(p.robot_id != r for r,p in ports.items()):
            raise ValueError('three bound robot ports required')
        self.task = copy.deepcopy(task)
        self.protocol = MissionProtocol(task['mission'],agreement,max_age_s=.8)
        self.ports, self.output = dict(ports), Path(output)
        self.output.mkdir(parents=True,exist_ok=False)
        self.map_path = self.output/'static-map.json'
        write(self.map_path,task['static_map'])
        self.trackers = {r:CargoTracker(task['mission']) for r in ports}
        self.identity, self.frames, self.active = {}, {}, {}
        self.history = {r:[] for r in ports}
        self.events, self.pending = [], {}
        self.sequence = 0
        self.now = now_s

    def _meta(self, tid):
        return {'task_id':tid,'object_id':self.protocol.jobs[tid]['object_id'],
                'plan_version':self.protocol.authority.version,
                'plan_hash':self.protocol.committed['plan_hash']}

    def _report(self, tid, rid, now_s):
        f = self.frames[rid]
        return dict(proposal_id=self.protocol.committed['proposal_id'],
                    plan_hash=self.protocol.committed['plan_hash'], object_id=self.protocol.jobs[tid]['object_id'],
                    sequence=self.protocol.sequences.get(rid,-1)+1,
                    observed_at_s=self.identity[rid]['observed_at_s'],now_s=now_s,
                    own_rgb_ref=f['own_rgb']['path'], top_rgb_ref=f['shared_top_rgb']['path'])

    def _visible(self, tid, now_s):
        return all(self.identity.get(r,{}).get('valid') and
                   0 <= now_s-self.identity[r]['observed_at_s'] <= .8
                   for r in self.protocol.tasks[tid]['participants'])

    def tick(self, now_s):
        self.now = now_s
        try:
            self.protocol._check(now_s)
        except ValueError:
            for p in self.ports.values(): p.hold(now_s)
            for job in self.active.values(): job['stage'].hold('mission_revoked',now_s=now_s)
            raise
        for tid,job in self.active.items():
            if not self._visible(tid,now_s):
                job['stage'].hold('rgb_identity_unresolved_or_stale',now_s=now_s)
            if job['phase'] != 'APPROACH': job['stage'].tick(now_s)
            elif not self._visible(tid,now_s):
                for r in self.protocol.tasks[tid]['participants']: self.ports[r].hold(now_s)
        for port in self.ports.values(): port.tick(now_s)

    def observe(self, frames, *, now_s):
        self.tick(now_s)
        if set(frames) != set(self.ports): raise ValueError('fresh frames from every bound endpoint required')
        self.frames = frames
        try:
            for r,f in frames.items():
                self.identity[r] = self.trackers[r].observe(f['top_bytes'],frame_id=f['frame_id'],now_s=now_s)
        except (ValueError,TypeError,KeyError):
            self.identity={}
            for p in self.ports.values():p.hold(now_s)
            self.tick(now_s)
            raise
        self.events.append({'event':'RGB_IDENTITY','at_s':now_s,'observations':copy.deepcopy(self.identity)})
        self.tick(now_s)
        for tid,t in self.protocol.tasks.items():
            if tid in self.active or tid in self.protocol.completed or not set(t['after']) <= self.protocol.completed:
                continue
            if self._visible(tid,now_s):
                for r in t['participants']: self.protocol.report_ready(tid,r,**self._report(tid,r,now_s))
        for ticket in self.protocol.grant_ready(now_s):
            tid=ticket.task_id; t=self.protocol.tasks[tid]; static=self.task['static_map']
            plan=TaskPlan(tid,self.protocol.authority.version,ticket.object_id,tuple(t['participants'].items()),
                          self.protocol.jobs[tid]['destination_id'],static['map_id'],str(static['version']),
                          hashlib.sha256(self.map_path.read_bytes()).hexdigest())
            stage=TaskStageExecution(plan,{r:self.ports[r] for r in t['participants']},
                                     map_path=self.map_path,output=self.output/f'{tid}-{ticket.serial}',
                                     now_s=now_s,report_ttl_s=.6)
            self.active[tid]={'ticket':ticket,'stage':stage,'phase':'APPROACH','last':{},'done':{}}
            self.events.append({'event':'TASK_STARTED',**self._meta(tid),'at_s':now_s})
        return self.requests(now_s)

    def requests(self, now_s):
        self.pending={}
        for tid,job in self.active.items():
            if not self._visible(tid,now_s): continue
            for rid in self.protocol.tasks[tid]['participants']:
                f=self.frames[rid]; self.sequence+=1
                reqid=f'{tid}-{rid}-{self.sequence:06}'
                if job['phase']=='APPROACH':
                    camera=None; stage='APPROACH'
                else:
                    camera=job['stage'].capture(rid,own_rgb=f['own_bytes'],top_rgb=f['top_bytes'],now_s=now_s)
                    stage='TRANSIT' if camera['stage']=='CARRY' else camera['stage']
                request={'request_id':reqid,**self._meta(tid),'robot_id':rid,'stage':stage,
                         'observed_at_s':now_s,'frame_id':f['frame_id'],'camera_request':camera,
                         'target':copy.deepcopy(self.identity[rid]['tracks'][self.protocol.jobs[tid]['object_id']]),
                         'visual_inventory':copy.deepcopy(self.identity[rid]),
                         'own_issued_commands':copy.deepcopy(self.history[rid][-20:]),
                         'last_command':copy.deepcopy(job['last'].get(rid)),
                         'own_rgb':copy.deepcopy(f['own_rgb']),'top_rgb':copy.deepcopy(f['shared_top_rgb'])}
                self.pending[rid]=request
                write(self.output/(reqid+'.json'),request)
        return copy.deepcopy(self.pending)

    def batch(self, replies, *, now_s):
        self.tick(now_s)
        accepted={}
        for rid,req in self.pending.items():
            tid=req['task_id']; job=self.active[tid]
            try:
                if not self._visible(tid,now_s) or not 0<=now_s-req['observed_at_s']<=.6:
                    raise ValueError('stale identity or request')
                reply=validate_reply(json.dumps(replies.get(rid)),req['request_id'],
                    plan_hash=req['plan_hash'],stage=req['stage'],robot_id=rid)
                if reply['action'].get('duration_s',.2) > .2:
                    raise ValueError('multi-object pilot commands limited to .2 seconds')
                accepted[rid]=reply
            except (ValueError,TypeError):
                job['stage'].hold('missing_or_invalid_reply',now_s=now_s)
                for r in self.protocol.tasks[tid]['participants']: self.ports[r].hold(now_s)
        pending=self.pending; self.pending={}
        for tid,job in list(self.active.items()):
            people=self.protocol.tasks[tid]['participants']
            if not set(people)<=accepted.keys(): continue
            if any(accepted[r]['status']=='BLOCKED' for r in people):
                job['stage'].hold('producer_requests_replan',now_s=now_s)
                continue
            if job['phase']=='APPROACH':
                complete=True
                for r in people:
                    reply=accepted[r]; req=pending[r]; last=job['last'].get(r)
                    done=bool(reply['status']=='DONE' and reply['confidence']>=.8 and
                              {'at_grasp_pose','stopped'}<=set(reply['checks']) and last and
                              reply['command_id']==last['command_id'] and req['frame_id']>last['frame_id'] and
                              req['observed_at_s']>=last['issued_at_s']+.2)
                    complete &= done
                    job['done'][r]=done
                    if not done and reply['status'] in ('WORKING','READY') and reply['confidence']>=.8:
                        self._issue_approach(tid,r,reply['action'],req,now_s)
                    else: self.ports[r].hold(now_s)
                if complete:
                    job['phase']='STAGES'
                    for r in people: self.ports[r].hold(now_s)
                    self.events.append({'event':'APPROACH_CLAIMED',**self._meta(tid),'at_s':now_s})
                continue
            ex=job['stage']
            # Validate all producer replies before submitting any coupled command.
            for r in people:
                reply=accepted[r]; req=pending[r]; camera=req['camera_request']
                status=reply['status'] if reply['status'] in ('READY','DONE','UNCERTAIN') else 'UNCERTAIN'
                ex.receive(r,{'request_id':camera['request_id'],'status':status,
                    'confidence':reply['confidence'],'checks':reply['checks'],'command_id':reply['command_id'],
                    'reason':reply['reason'],'decided_at_s':now_s},now_s=now_s)
            if ex.advance(now_s=now_s):
                self.events.append({'event':'STAGE_ADVANCED',**self._meta(tid),'at_s':now_s,
                                    'status':ex.sync.status(now_s=now_s)})
                if ex.sync.status(now_s=now_s)['phase']=='FINISH':
                    for r in people: self.protocol.report_done(job['ticket'],r,**self._report(tid,r,now_s))
                    ex.close(now_s=now_s); del self.active[tid]
                continue
            permit=ex.authorize(now_s=now_s)['permission']
            participants=ex.sync.command_participants(now_s=now_s)
            if permit and participants:
                commands={r:self._command(tid,r,accepted[r]['action'],pending[r],now_s) for r in participants}
                if ex.dispatch_pair(permit,commands,now_s=now_s):
                    for r,c in commands.items(): self._record(tid,r,c,pending[r],now_s)
        self.tick(now_s)

    def _command(self,tid,r,action,req,now_s):
        return {'command_id':req['request_id']+'-command','action':copy.deepcopy(action),'duration_s':.2}

    def _record(self,tid,r,c,req,now_s):
        row={**copy.deepcopy(c),**self._meta(tid),'stage':req['stage'],'frame_id':req['frame_id'],
             'issued_at_s':now_s,'own_rgb':req['own_rgb'],'top_rgb':req['top_rgb'],
             'meaning':'issued command, not measured state or execution success'}
        self.history[r].append(row); self.active[tid]['last'][r]=row
        self.events.append({'event':'LOCAL_COMMAND','robot_id':r,**row})

    def _issue_approach(self,tid,r,action,req,now_s):
        c=self._command(tid,r,action,req,now_s)
        self.ports[r].validate_bounded(action,.2)
        self.ports[r].apply_bounded(action,now_s,.2)
        self._record(tid,r,c,req,now_s)

    def close(self, *, now_s):
        try:
            for p in self.ports.values(): p.hold(now_s)
            for job in self.active.values(): job['stage'].close(now_s=now_s)
        finally:
            write(self.output/'events.json',self.events)
            write(self.output/'mission-events.json',self.protocol.events)
            write(self.output/'issued-commands.json',self.history)
