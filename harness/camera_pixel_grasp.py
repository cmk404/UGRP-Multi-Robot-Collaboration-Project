"""Experimental low-level RGB servo using active gripper identification.

This is not a language-model policy. All measurements originate in two camera
JPEGs; commanded pulses are remembered as commands, never measured joint state.
"""
from __future__ import annotations

import copy
import math

from harness.camera_beam_features import extract_beams, select_beam
from harness.camera_gripper_motion import GripperMotionTracker
from harness.camera_grasp_controller import STARTUP_COMMANDS


def _wrap_axis(angle):
    return (angle + math.pi / 2) % math.pi - math.pi / 2


def _own_beam_aim(own_beam, top_width):
    if own_beam is None:
        return None
    own_width, own_height = own_beam['image_size']
    if own_beam['width_px'] <= 0:
        return None
    if own_beam['length_px'] / own_beam['width_px'] < 1.5:
        aim = own_beam['center']
    else:
        aim = min(
            own_beam['endpoints'],
            key=lambda p: ((p[0] - .5) * own_width) ** 2 + ((p[1] - .5) * own_height) ** 2,
        )
    return (aim[0] - .5) * (.2 * top_width)


def alignment_features(gripper, beam, endpoint=None, own_beam=None):
    if not gripper.get('valid') or beam is None:
        return None
    w, h = beam['image_size']
    center = gripper.get('center')
    axis = gripper.get('opening_axis')
    if center is None or axis is None:
        return None
    ends = beam['endpoints']
    reference = endpoint if endpoint is not None else center
    end = min(ends, key=lambda p: ((p[0]-reference[0])*w)**2 + ((p[1]-reference[1])*h)**2)
    other = max(ends, key=lambda p: ((p[0]-end[0])*w)**2 + ((p[1]-end[1])*h)**2)
    dx, dy = (other[0]-end[0])*w, (other[1]-end[1])*h
    length = math.hypot(dx, dy)
    if length < 4 or beam['width_px'] < 2:
        return None
    tangent = [dx/length, dy/length]
    inset = min(beam['width_px']*.6, length*.1)
    target = [end[0]+tangent[0]*inset/w, end[1]+tangent[1]*inset/h]
    offset = [(target[0]-center[0])*w, (target[1]-center[1])*h]
    desired_axis = [-tangent[1], tangent[0]]
    angle = _wrap_axis(math.atan2(desired_axis[1], desired_axis[0])-math.atan2(axis[1], axis[0]))
    own_aim_error = _own_beam_aim(own_beam, w)
    # Experimental priority: penalize an opening axis parallel to the beam
    # strongly enough that top-view position improvement cannot dominate it.
    angular_scale = max(8., beam['width_px']*10.)
    cost_terms = [*offset, angular_scale * angle]
    if own_aim_error is not None:
        cost_terms.append(own_aim_error)
    return {'offset_px': offset, 'axis_error_rad': angle, 'endpoint': list(end),
            'target': target, 'distance_px': math.hypot(*offset),
            'own_aim_error_px': own_aim_error,
            'cost': math.hypot(*cost_terms),
            'width_px': beam['width_px']}


class PixelGraspController:
    """Bounded image pattern search, then a visually checked trial closure/lift.

    A gripper motion region is deliberately not called an exact pair of tips.
    No evaluation/contact channel exists in this class's interface.
    """
    def __init__(self, robot_id, startup_commands=STARTUP_COMMANDS):
        if robot_id not in ('r1', 'r3'):
            raise ValueError('unsupported robot')
        self.robot_id = robot_id
        self.history = []
        self.pulses = {}
        for action in startup_commands:
            a = copy.deepcopy(action)
            self.history.append(a)
            self.pulses[6 if a['kind']=='look' else a['servo_id']] = a.get('pan_pulse', a.get('pulse'))
        self.tracker = GripperMotionTracker(initial_gripper_pulse=self.pulses.get(1, 2000))
        self.last_action = None
        self.last_observation = {}
        self.last_decision = {}
        self.beam_center = None
        self.endpoint = None
        self.stage = 'identify'
        self.pending = None
        self.rollback = []
        self.refresh_required = False
        self.measurement_rounds = 0
        self.tried = set()
        self.repeat = None
        self.search_anchor = None
        self.unseen = 0
        self.aligned = 0
        self.attempts = 0
        self.lift_steps = 0
        self.lift_start = None
        self.lift_pulse = None
        self.height_lock = False
        self.recovery = []
        self.steps = 0

    def _issue(self, action, reason, obs, *, test=False, label=None):
        action = copy.deepcopy(action)
        undo = []
        if action['kind'] in ('arm', 'look'):
            ch = 6 if action['kind']=='look' else action['servo_id']
            field = 'pan_pulse' if ch == 6 else 'pulse'
            old = self.pulses.get(ch, 1500)
            if ch != 1:
                action[field] = int(max(500, min(2500, max(old-100, min(old+100, action[field])))))
            if action[field] != old:
                back = dict(action);back[field] = old;undo = [back]
            self.pulses[ch] = action[field]
        elif action['kind'] == 'drive':
            f,t,d = action['forward'],action['turn'],action['duration_s']
            if not -.05 <= f <= .15 or not -.1 <= t <= .1 or not 0 <= d <= 1:
                raise ValueError('drive out of bounds')
            # Equal opposite command impulse is a rollback hypothesis, not a
            # position reset. Observe again after it; wheel slip is possible.
            pieces = max(1, math.ceil(abs(f)/.05))
            undo = [{'kind':'drive','forward':-f/pieces,'turn':-t/pieces,'duration_s':d} for _ in range(pieces)]
        if test and obs.get('alignment') is not None:
            self.measurement_rounds=0
            self.pending = {'before':copy.deepcopy(obs['alignment']), 'action':action,
                            'undo':undo, 'label':label, 'step':self.steps,
                            'own_beam_visible':obs.get('own_beam') is not None}
        self.last_action = action
        self.history.append(action);self.history=self.history[-32:]
        self.last_decision = {'stage':self.stage,'reason':reason,'attempts':self.attempts,
                              'candidate':label,'pending_test':bool(test),
                              'tried':sorted(self.tried),'cost':None if obs.get('alignment') is None else obs['alignment']['cost']}
        return action

    def _joint(self, channel, delta):
        pulse = int(max(500,min(2500,self.pulses.get(channel,1500)+delta)))
        return {'kind':'look','pan_pulse':pulse} if channel==6 else {'kind':'arm','servo_id':channel,'pulse':pulse}

    def _candidates(self, alignment):
        far = alignment['distance_px'] > 30
        step = 100 if far else 50
        drive = .15 if far else .05
        duration = .8 if far else .4
        candidates = [('forward',{'kind':'drive','forward':drive,'turn':0.,'duration_s':duration}),
                      ('look+',self._joint(6,step)),('look-',self._joint(6,-step)),
                      ('left',{'kind':'drive','forward':0.,'turn':.1,'duration_s':.6}),
                      ('right',{'kind':'drive','forward':0.,'turn':-.1,'duration_s':.6}),
                      ('elbow-',self._joint(4,-step)),('elbow+',self._joint(4,step)),
                      ('wrist+',self._joint(3,step)),('wrist-',self._joint(3,-step)),
                      ('back',{'kind':'drive','forward':-.05,'turn':0.,'duration_s':.4})]
        if not self.height_lock:
            candidates.extend([('shoulder+',self._joint(5,step)),('shoulder-',self._joint(5,-step))])
        return candidates

    def _visual_lift(self, obs):
        start = self.lift_start
        if start is None or obs.get('beam') is None or start.get('beam') is None or obs.get('own_beam') is None or start.get('own_beam') is None:
            return False
        own0, own = start['own_beam'], obs['own_beam']
        top0, top = start['beam'], obs['beam']
        w,h = top['image_size'];ow,oh=own['image_size']
        top_move = math.hypot((top['center'][0]-top0['center'][0])*w,(top['center'][1]-top0['center'][1])*h)
        own_move = math.hypot((own['center'][0]-own0['center'][0])*ow,(own['center'][1]-own0['center'][1])*oh)
        size_change = abs(math.log(max(1,own['area_px'])/max(1,own0['area_px'])))
        # A tentative image hypothesis only; independent physical evaluation
        # is the success authority and is unavailable to this controller.
        return top_move > 1.0 and own_move < 8.0 and size_change < .15

    def step(self, own_jpeg, top_jpeg, active=True):
        self.steps += 1
        gripper = self.tracker.update(top_jpeg,self.last_action)
        beam = select_beam(extract_beams(top_jpeg),gripper.get('center') if gripper.get('valid') else None,self.beam_center)
        own_candidates = extract_beams(own_jpeg)
        own = max(own_candidates,key=lambda c:c['area_px']) if own_candidates else None
        if beam is not None:
            self.beam_center=list(beam['center'])
        alignment = alignment_features(gripper,beam,self.endpoint,own)
        if alignment is not None:
            self.endpoint=list(alignment['endpoint'])
        obs={'gripper':gripper,'beam':beam,'own_beam':own,'alignment':alignment}
        self.last_observation = copy.deepcopy(obs)
        if not active:
            return self._issue({'kind':'wait'},'peer owns this command slice; observe only',obs)
        if self.stage in ('hold','blocked'):
            return self._issue({'kind':'wait'},'fixed-budget observation; no physical success input',obs)
        if self.stage == 'lift':
            if self.lift_steps >= 3:
                if self._visual_lift(obs):
                    self.stage='hold'
                    return self._issue({'kind':'wait'},'tentative two-view object-follow evidence; referee alone decides success',obs)
                self.stage='recover';self.pending=None;self.repeat=None;self.tried.clear();self.aligned=0
                self.recovery=[{'kind':'arm','servo_id':1,'pulse':2000}]
                # Restore own pre-lift command in bounded steps, then vary one
                # height command. This is exploration, not measured height.
                current=self.pulses[5]
                while current < self.lift_pulse:
                    current=min(current+100,self.lift_pulse)
                    self.recovery.append({'kind':'arm','servo_id':5,'pulse':current})
                self.recovery.append({'kind':'arm','servo_id':5,'pulse':min(2500,self.lift_pulse+50)})
                self.height_lock=True
            else:
                self.lift_steps += 1
                return self._issue(self._joint(5,-50),'small commanded test lift; inspect object motion in both views',obs)
        if self.recovery:
            action=self.recovery.pop(0)
            if not self.recovery:self.stage='align'
            return self._issue(action,'restore commands after unconfirmed capture and explore next height',obs)
        if self.rollback:
            action=self.rollback.pop(0);self.pending=None;self.refresh_required=True
            return self._issue(action,'reverse rejected command hypothesis, then reobserve actual pixels',obs)
        if self.pending is not None or self.refresh_required:
            fresh_open = (gripper.get('valid') and gripper.get('source') == 'isolated_gripper_motion'
                          and self.pulses.get(1) == 2000)
            if not fresh_open:
                self.measurement_rounds += 1
                if self.measurement_rounds >= 6 and self.pending is not None:
                    rejected=self.pending;self.pending=None
                    self.repeat=None;self.tried.add(rejected['label'])
                    self.rollback=list(rejected['undo'])
                    if self.pulses.get(1)!=2000:
                        self.rollback.insert(0,{'kind':'arm','servo_id':1,'pulse':2000})
                    self.refresh_required=True;self.measurement_rounds=0
                    if self.rollback:
                        return self._issue(self.rollback.pop(0),'candidate lost measurable gripper geometry; undo before trying another direction',obs)
                elif self.measurement_rounds >= 12:
                    self.stage='blocked'
                    return self._issue({'kind':'wait'},'bounded fresh-measurement recovery failed',obs)
                self.stage='measure'
                self.unseen = self.unseen+1 if not gripper.get('valid') else 0
                if self.unseen > 12:
                    self.stage='blocked'
                    return self._issue({'kind':'wait'},'cannot remeasure the candidate gripper motion',obs)
                pulse=1500 if self.pulses.get(1,2000)==2000 else 2000
                return self._issue({'kind':'arm','servo_id':1,'pulse':pulse},
                                   'fresh open/close measurement before accepting a candidate; do not score flow drift',obs)
            self.refresh_required=False;self.measurement_rounds=0
        if alignment is None:
            self.unseen+=1
            if self.unseen>12:
                self.stage='blocked'
                return self._issue({'kind':'wait'},'active gripper identification remained unobservable',obs)
            self.stage='identify'
            # Identification by isolated finger motion, including at start.
            # This command is explicitly a probe, not a grasp success claim.
            pulse=1500 if self.pulses.get(1,2000)==2000 else 2000
            return self._issue({'kind':'arm','servo_id':1,'pulse':pulse},'isolate finger motion to identify the gripper region',obs)
        self.unseen=0
        if self.pulses.get(1)!=2000:
            return self._issue({'kind':'arm','servo_id':1,'pulse':2000},'return to an open-command observation before geometric servo',obs)
        self.stage='align'
        if self.pending:
            pending=self.pending;self.pending=None
            before=pending['before']
            # A changed endpoint would invalidate this comparison.
            same_endpoint=math.dist(before['endpoint'],alignment['endpoint'])<.04
            own_preserved = not pending['own_beam_visible'] or own is not None
            if own_preserved and same_endpoint and alignment['cost'] < before['cost'] - .15:
                self.repeat=pending['label'];self.tried.clear()
            else:
                self.repeat=None;self.tried.add(pending['label']);self.rollback=pending['undo']
                if self.rollback:
                    self.refresh_required=True
                    reason = ('candidate removed a previously visible own-camera beam; reverse it'
                              if not own_preserved else
                              'observed candidate failed to reduce combined position/direction error')
                    return self._issue(self.rollback.pop(0),reason,obs)
        close_distance=max(3.,alignment['width_px']*.5)
        aligned=alignment['distance_px']<close_distance and abs(alignment['axis_error_rad'])<.22 and own is not None
        self.aligned=self.aligned+1 if aligned else 0
        if self.aligned>=2:
            if self.attempts>=6:
                self.stage='blocked'
                return self._issue({'kind':'wait'},'bounded capture trials exhausted',obs)
            self.attempts+=1;self.stage='lift';self.lift_steps=0
            self.lift_start=copy.deepcopy(obs);self.lift_pulse=self.pulses[5]
            return self._issue({'kind':'arm','servo_id':1,'pulse':1500},'two open-view end/axis alignments; trial closure, never assumed capture',obs)
        if aligned:
            return self._issue({'kind':'wait'},'require a second observed alignment before closure',obs)
        candidates=self._candidates(alignment)
        selected=next(((label,action) for label,action in candidates if label==self.repeat),None)
        if selected is None:
            selected=next(((label,action) for label,action in candidates if label not in self.tried),None)
        if selected is None:
            self.stage='blocked'
            return self._issue({'kind':'wait'},'all bounded directions failed at this observed state',obs)
        label,action=selected
        return self._issue(action,'test one raw motion against visible beam-end and opening-axis error',obs,test=True,label=label)
