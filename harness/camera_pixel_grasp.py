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
from harness.camera_pixel_jacobian import LocalPixelJacobian, axis_residual
from harness.camera_sweep_trial import evaluate_sweep_topology


SEARCH_SCALES = (1.0, .5, .25)
BASIN_ACTIVE_BUDGET = 160
BASIN_OFFSETS = (50, -50, 100, -100, 150, -150, 200, -200)
# One trial per 50-pulse step can cover the documented command interval.
# This is a command budget, not a measured height or a target grasp pose.
MAX_CAPTURE_TRIALS = (2500 - 500) // 50 + 1


def _wrap_axis(angle):
    return (angle + math.pi / 2) % math.pi - math.pi / 2


def _own_beam_endpoint(own_beam, previous_endpoint=None):
    if own_beam is None:
        return None
    own_width, own_height = own_beam['image_size']
    reference = previous_endpoint if previous_endpoint is not None else [.5, .5]
    if own_beam['width_px'] <= 0:
        return None
    if own_beam['length_px'] / own_beam['width_px'] < 2.0:
        # Foreshortened/end-on rectangles do not identify a physical beam end.
        return None
    else:
        aim = min(
            own_beam['endpoints'],
            key=lambda p: ((p[0] - reference[0]) * own_width) ** 2 + ((p[1] - reference[1]) * own_height) ** 2,
        )
    if not all(0.0 <= value <= 1.0 for value in aim):
        # A fitted rectangle may extrapolate beyond a clipped image contour.
        return None
    return list(aim)


def alignment_features(gripper, beam, endpoint=None, own_beam=None, own_endpoint=None):
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
    own_aim = _own_beam_endpoint(own_beam, own_endpoint)
    own_aim_error = None if own_aim is None else (own_aim[0] - .5) * (.2 * w)
    # Preserve a safe inner cone where raster angle jitter does not outweigh
    # translation; closure still applies its separate absolute .22 rad gate.
    angular_scale = max(8., beam['width_px']*10.)
    angular_error = angular_scale * axis_residual(angle)
    cost_terms = [*offset, angular_error]
    if own_aim_error is not None:
        cost_terms.append(own_aim_error)
    return {'offset_px': offset, 'axis_error_rad': angle, 'endpoint': list(end),
            'target': target, 'distance_px': math.hypot(*offset),
            'own_aim_error_px': own_aim_error, 'own_endpoint': own_aim,
            'top_cost': math.hypot(*offset, angular_error),
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
        self.own_endpoint = None
        self.stage = 'identify'
        self.pending = None
        self.rollback = []
        self.refresh_required = False
        self.measurement_rounds = 0
        self.view_repair_origin = None
        self.view_repair_index = 0
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
        self.capture_height_exhausted = False
        self.search_level = 0
        self.recovery = []
        self.composite_queue = []
        self.jacobian = LocalPixelJacobian()
        self.last_learned_outcome = None
        self.basin_origin = None
        self.basin_targets = []
        self.basin_index = 0
        self.basin_active_calls = 0
        self.basin_motion = []
        self.basin_target = None
        self.sweep_phase = None
        self.sweep_passes = 0
        self.sweep_pass_steps = []
        self.steps = 0

    def _reset_sweep_confirmation(self):
        self.sweep_phase = None
        self.sweep_passes = 0
        self.sweep_pass_steps = []

    @staticmethod
    def _primitive_name(action):
        if action.get('kind')=='look':
            return 'look'
        if action.get('kind')=='arm':
            return {3:'wrist',4:'elbow',5:'shoulder'}.get(action.get('servo_id'))
        if action.get('kind')=='drive':
            if action.get('turn')==0 and action.get('forward')!=0:
                return 'forward'
            if action.get('forward')==0 and action.get('turn')!=0:
                return 'turn'
        return None

    def _issue(self, action, reason, obs, *, test=False, label=None, learn=False):
        action = copy.deepcopy(action)
        is_jaw = (action.get('kind') == 'arm' and action.get('servo_id') == 1)
        if not is_jaw:
            self._reset_sweep_confirmation()
        pulses_before = copy.deepcopy(self.pulses)
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
                            'pulses_before':pulses_before,
                            'learn':bool(learn and obs.get('gripper',{}).get('source')=='isolated_gripper_motion'
                                         and self.pulses.get(1)==2000),
                            'own_beam_visible':obs.get('own_beam') is not None}
        self.last_action = action
        self.history.append(action);self.history=self.history[-32:]
        self.last_decision = {'stage':self.stage,'reason':reason,'attempts':self.attempts,
                              'candidate':label,'pending_test':bool(test),
                              'tried':sorted(self.tried),'cost':None if obs.get('alignment') is None else obs['alignment']['cost'],
                              'search_level':self.search_level,
                              'search_scale':SEARCH_SCALES[self.search_level],
                              'basin_origin':self.basin_origin,
                              'basin_index':self.basin_index,
                              'basin_active_calls':self.basin_active_calls,
                              'basin_target':self.basin_target,
                              'sweep_phase':self.sweep_phase,
                              'sweep_passes':self.sweep_passes}
        diagnostics=getattr(self.jacobian,'diagnostics',None)
        if callable(diagnostics):
            self.last_decision['jacobian']=diagnostics()
        if self.last_learned_outcome is not None:
            self.last_decision['learned_outcome']=copy.deepcopy(self.last_learned_outcome)
        return action

    def _repair_view(self, obs):
        """Change an unobservable view with bounded own commands, then measure."""
        if self.view_repair_origin is None:
            self.view_repair_origin=self.pulses.get(6,1500)
        offsets=(50,-50)
        self.repeat=None;self.tried.clear();self.measurement_rounds=0;self.unseen=0
        self.refresh_required=True
        if self.view_repair_index < len(offsets):
            target=self.view_repair_origin+offsets[self.view_repair_index]
            self.view_repair_index+=1;self.stage='reobserve'
            return self._issue({'kind':'look','pan_pulse':target},
                               'isolated motion remained ambiguous; change view slightly and remeasure',obs)
        target=self.view_repair_origin;self.stage='blocked'
        action=({'kind':'look','pan_pulse':target} if self.pulses.get(6)!=target else {'kind':'wait'})
        return self._issue(action,'bounded alternate views exhausted; restore original look command',obs)

    def _joint(self, channel, delta):
        pulse = int(max(500,min(2500,self.pulses.get(channel,1500)+delta)))
        return {'kind':'look','pan_pulse':pulse} if channel==6 else {'kind':'arm','servo_id':channel,'pulse':pulse}

    def _next_basin(self, obs):
        """Move to a bounded command-space height basin, then remeasure pixels."""
        if self.basin_origin is None:
            self.basin_origin = self.pulses.get(5, 1500)
            self.basin_targets = [
                self.basin_origin + offset for offset in BASIN_OFFSETS
                if 500 <= self.basin_origin + offset <= 2500
            ]
        current = self.pulses.get(5, 1500)
        while (self.basin_index < len(self.basin_targets)
               and self.basin_targets[self.basin_index] == current):
            self.basin_index += 1
        if self.basin_index >= len(self.basin_targets):
            self.basin_target = None
            self.stage = 'blocked'
            return self._issue(
                {'kind':'wait'},
                'bounded height-basin exploration exhausted without visual closure', obs,
            )
        target = self.basin_targets[self.basin_index]
        self.basin_index += 1
        self.basin_target = target
        path = []
        while current != target:
            current += max(-100, min(100, target - current))
            path.append({'kind':'arm','servo_id':5,'pulse':current})
        self.basin_motion = path[1:]
        self.pending = None
        self.rollback.clear()
        self.composite_queue.clear()
        self.repeat = None
        self.tried.clear()
        self.search_anchor = None
        self.search_level = 0
        self.jacobian = LocalPixelJacobian()
        self.last_learned_outcome = None
        self.aligned = 0
        self.measurement_rounds = 0
        self.basin_active_calls = 0
        self.height_lock = True
        self.refresh_required = not self.basin_motion
        return self._issue(
            path[0],
            'move to next bounded height-command basin; remeasure before local search', obs,
        )

    def _candidates(self, alignment):
        far = alignment['distance_px'] > 30
        scale = SEARCH_SCALES[self.search_level]
        step = max(1, int(round((100 if far else 50) * scale)))
        drive = (.15 if far else .05) * scale
        turn = .1 * scale
        duration = .8 if far else .4
        candidates = [('forward',{'kind':'drive','forward':drive,'turn':0.,'duration_s':duration}),
                      ('look+',self._joint(6,step)),('look-',self._joint(6,-step)),
                      ('left',{'kind':'drive','forward':0.,'turn':turn,'duration_s':.6}),
                      ('right',{'kind':'drive','forward':0.,'turn':-turn,'duration_s':.6}),
                      ('elbow-',self._joint(4,-step)),('elbow+',self._joint(4,step)),
                      ('wrist+',self._joint(3,step)),('wrist-',self._joint(3,-step)),
                      ('back',{'kind':'drive','forward':-.05*scale,'turn':0.,'duration_s':.4})]
        if not self.height_lock:
            candidates.extend([('shoulder+',self._joint(5,step)),('shoulder-',self._joint(5,-step))])
        return candidates

    @staticmethod
    def _composite_candidates():
        """Bounded turn-drive-counterturn probes for lateral displacement."""
        return [
            ('lateral-left', [
                {'kind':'drive','forward':0.,'turn':.1,'duration_s':1.},
                {'kind':'drive','forward':.05,'turn':0.,'duration_s':.8},
                {'kind':'drive','forward':0.,'turn':-.1,'duration_s':1.},
            ]),
            ('lateral-right', [
                {'kind':'drive','forward':0.,'turn':-.1,'duration_s':1.},
                {'kind':'drive','forward':.05,'turn':0.,'duration_s':.8},
                {'kind':'drive','forward':0.,'turn':.1,'duration_s':1.},
            ]),
        ]

    @staticmethod
    def _action_undo(action, pulses_before):
        """Build an inverse command from the actual pre-command pulse state."""
        if action['kind'] in ('arm','look'):
            ch=6 if action['kind']=='look' else action['servo_id']
            field='pan_pulse' if ch==6 else 'pulse'
            old=pulses_before.get(ch)
            if old is None or action[field]==old:
                return []
            back=copy.deepcopy(action);back[field]=old
            return [back]
        if action['kind']!='drive':
            return []
        f,t,d = action['forward'],action['turn'],action['duration_s']
        pieces = max(1, math.ceil(abs(f)/.05))
        return [
            {'kind':'drive','forward':-f/pieces,'turn':-t/pieces,'duration_s':d}
            for _ in range(pieces)
        ]

    def _start_composite(self, label, actions, obs):
        """Issue the first raw action while retaining one before observation."""
        first, *remaining = copy.deepcopy(actions)
        issued = self._issue(
            first, 'begin bounded lateral probe; score only its measured endpoint',
            obs, test=True, label=label,
        )
        self.pending['composite'] = True
        self.composite_queue = remaining
        return issued

    def _start_learned(self, actions, obs):
        """Run a learned coupled correction through the measured sequence gate."""
        first, *remaining = copy.deepcopy(actions)
        self.last_learned_outcome=None
        issued=self._issue(
            first, 'begin learned image-Jacobian correction; score only its measured endpoint',
            obs, test=True, label='learned',
        )
        self.pending['composite']=True
        diagnostics=getattr(self.jacobian,'diagnostics',None)
        if callable(diagnostics):
            self.pending['prediction']=diagnostics()
        self.composite_queue=remaining
        return issued

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
        top_size_change = abs(math.log(max(1,top['area_px'])/max(1,top0['area_px'])))
        # A tentative image hypothesis only; independent physical evaluation
        # is the success authority and is unavailable to this controller.
        # Vertical motion near the top camera optical axis may change scale
        # while leaving the centroid fixed; do not require lateral object motion.
        top_changed = top_move > 1.0 or top_size_change > .04
        return top_changed and own_move < 8.0 and size_change < .15

    def step(self, own_jpeg, top_jpeg, active=True):
        self.steps += 1
        previous_beam = copy.deepcopy(self.last_observation.get('beam'))
        gripper = self.tracker.update(top_jpeg,self.last_action)
        beam = select_beam(extract_beams(top_jpeg,robust_shaft=True),gripper.get('center') if gripper.get('valid') else None,self.beam_center)
        own_candidates = extract_beams(own_jpeg)
        own = max(own_candidates,key=lambda c:c['area_px']) if own_candidates else None
        if beam is not None:
            self.beam_center=list(beam['center'])
        alignment = alignment_features(gripper,beam,self.endpoint,own,self.own_endpoint)
        if alignment is not None:
            self.endpoint=list(alignment['endpoint'])
            # Keep the same visible end through a camera pan. Closed fingers
            # can introduce orange edge components, so update on open views.
            if self.pulses.get(1) == 2000 and alignment['own_endpoint'] is not None:
                self.own_endpoint=list(alignment['own_endpoint'])
        obs={'gripper':gripper,'beam':beam,'own_beam':own,'alignment':alignment}
        self.last_observation = copy.deepcopy(obs)
        if not active:
            return self._issue({'kind':'wait'},'peer owns this command slice; observe only',obs)
        self.basin_active_calls += 1
        if self.basin_motion:
            action = self.basin_motion.pop(0)
            if not self.basin_motion:
                self.refresh_required = True
            return self._issue(
                action,
                'continue bounded height-basin move; remeasure only after its endpoint', obs,
            )
        if self.composite_queue:
            action=self.composite_queue.pop(0)
            pulses_before=copy.deepcopy(self.pulses)
            issued=self._issue(
                action, 'continue coupled correction; defer scoring until its endpoint', obs
            )
            # Each newly issued inverse belongs before earlier inverses so a
            # failed composite is unwound in exact reverse action order.
            self.pending['undo'] = self._action_undo(issued,pulses_before) + self.pending['undo']
            return issued
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
                self.capture_height_exhausted = self.lift_pulse >= 2500
                if not self.capture_height_exhausted:
                    self.recovery.append({'kind':'arm','servo_id':5,'pulse':min(2500,self.lift_pulse+50)})
                self.height_lock=True
                self.search_level=0
                # A failed capture moves to a new commanded height.  Do not
                # spend its local-search budget on earlier trials or reuse a
                # Jacobian/escape anchor learned before this height change.
                self.basin_active_calls=0
                self.basin_origin=None;self.basin_targets=[];self.basin_index=0
                self.basin_target=None
                self.search_anchor=None
                self.jacobian=LocalPixelJacobian();self.last_learned_outcome=None
                self.measurement_rounds=0
            else:
                self.lift_steps += 1
                return self._issue(self._joint(5,-50),'small commanded test lift; inspect object motion in both views',obs)
        if self.recovery:
            action=self.recovery.pop(0)
            if not self.recovery:
                self.stage='blocked' if self.capture_height_exhausted else 'align'
                self.refresh_required=not self.capture_height_exhausted
            reason = ('open and restore after unconfirmed capture; height-command range exhausted'
                      if self.capture_height_exhausted else
                      'restore commands after unconfirmed capture and explore next height')
            return self._issue(action,reason,obs)
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
                    if rejected.get('label')=='learned':
                        self.last_learned_outcome={
                            'before_top_cost':rejected['before'].get('top_cost'),
                            'after_top_cost':None,
                            'accepted':False,
                            'measurement_unavailable':True,
                        }
                        if rejected.get('prediction') is not None:
                            self.last_learned_outcome['proposal']=rejected['prediction']
                    self.repeat=None;self.tried.add(rejected['label'])
                    self.rollback=list(rejected['undo'])
                    if self.pulses.get(1)!=2000:
                        self.rollback.insert(0,{'kind':'arm','servo_id':1,'pulse':2000})
                    self.refresh_required=True;self.measurement_rounds=0
                    if self.rollback:
                        return self._issue(self.rollback.pop(0),'candidate lost measurable gripper geometry; undo before trying another direction',obs)
                elif self.measurement_rounds >= 4 and self.pending is None:
                    return self._repair_view(obs)
                self.stage='measure'
                self.unseen = self.unseen+1 if not gripper.get('valid') else 0
                if self.unseen > 12:
                    self.stage='blocked'
                    return self._issue({'kind':'wait'},'cannot remeasure the candidate gripper motion',obs)
                pulse=1500 if self.pulses.get(1,2000)==2000 else 2000
                return self._issue({'kind':'arm','servo_id':1,'pulse':pulse},
                                   'fresh open/close measurement before accepting a candidate; do not score flow drift',obs)
            self.refresh_required=False;self.measurement_rounds=0
            self.view_repair_origin=None;self.view_repair_index=0
        if self.sweep_phase == 'await_close_frame':
            self.sweep_phase = 'await_open_frame'
            return self._issue(
                {'kind':'arm','servo_id':1,'pulse':2000},
                'open after exploratory close to obtain a fresh jaw-sweep image pair', obs,
            )
        if self.sweep_phase == 'await_open_frame':
            topology = evaluate_sweep_topology(
                self.tracker.calibration_candidates,
                self.tracker.calibration_transition,
                previous_beam, beam,
                None if alignment is None else alignment.get('endpoint'),
            )
            if (alignment is None or abs(alignment['axis_error_rad']) >= .22
                    or own is None):
                topology['passed'] = False
                topology['reason'] = (
                    'current axis/own-camera visibility preconditions failed'
                )
            topology['cycle'] = self.sweep_passes + 1
            obs['sweep_topology'] = topology
            self.last_observation['sweep_topology'] = copy.deepcopy(topology)
            if topology['passed'] and self.steps not in self.sweep_pass_steps:
                self.sweep_passes += 1
                self.sweep_pass_steps.append(self.steps)
                if self.sweep_passes >= 2:
                    if self.attempts >= MAX_CAPTURE_TRIALS:
                        self._reset_sweep_confirmation()
                        self.stage='blocked'
                        return self._issue(
                            {'kind':'wait'},'bounded capture trials exhausted',obs
                        )
                    self.attempts += 1
                    self.stage='lift';self.lift_steps=0
                    self.lift_start=copy.deepcopy(obs);self.lift_pulse=self.pulses[5]
                    self._reset_sweep_confirmation()
                    return self._issue(
                        {'kind':'arm','servo_id':1,'pulse':1500},
                        'two independent image-only jaw-sweep topology confirmations; '
                        'begin trial closure, never assume contact or success', obs,
                    )
                self.sweep_phase = 'await_close_frame'
                return self._issue(
                    {'kind':'arm','servo_id':1,'pulse':1500},
                    'first image-only jaw-sweep topology pass; request an independent cycle', obs,
                )
            self._reset_sweep_confirmation()
            sweep_evaluated_this_step = True
        else:
            sweep_evaluated_this_step = False
        if alignment is None:
            self.unseen+=1
            if self.unseen>=4:
                return self._repair_view(obs)
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
            # Compare identical terms if an endpoint is not observable in one
            # own frame; appearance of a new term is not geometric regression.
            both_own_ends = before.get('own_aim_error_px') is not None and alignment.get('own_aim_error_px') is not None
            score = 'cost' if both_own_ends else 'top_cost'
            before_score=before.get(score,before['cost'])
            after_score=alignment.get(score,alignment['cost'])
            if pending.get('learn') and own_preserved and same_endpoint:
                primitive=self._primitive_name(pending['action'])
                self.jacobian.add_sample(
                    before, alignment, pending['action'], pending['pulses_before'],
                    self.pulses, fresh=True, same_endpoint=True, primitive=primitive,
                )
            accepted=own_preserved and same_endpoint and after_score < before_score - .15
            if pending.get('label')=='learned':
                self.last_learned_outcome={
                    'before_top_cost':before.get('top_cost'),
                    'after_top_cost':alignment.get('top_cost'),
                    'accepted':bool(accepted),
                }
                if pending.get('prediction') is not None:
                    self.last_learned_outcome['proposal']=pending['prediction']
            if accepted:
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
            if self.attempts>=MAX_CAPTURE_TRIALS:
                self.stage='blocked'
                return self._issue({'kind':'wait'},'bounded capture trials exhausted',obs)
            self.attempts+=1;self.stage='lift';self.lift_steps=0
            self.lift_start=copy.deepcopy(obs);self.lift_pulse=self.pulses[5]
            return self._issue({'kind':'arm','servo_id':1,'pulse':1500},'two open-view end/axis alignments; trial closure, never assumed capture',obs)
        if aligned:
            return self._issue({'kind':'wait'},'require a second observed alignment before closure',obs)
        sweep_eligible = (
            not sweep_evaluated_this_step
            and abs(alignment['axis_error_rad']) < .22 and own is not None
            and gripper.get('valid')
            and gripper.get('source') == 'isolated_gripper_motion'
            and self.pulses.get(1) == 2000
        )
        if sweep_eligible:
            topology = evaluate_sweep_topology(
                self.tracker.calibration_candidates,
                self.tracker.calibration_transition,
                previous_beam, beam, alignment.get('endpoint'),
            )
            topology['cycle'] = 1
            obs['sweep_topology'] = topology
            self.last_observation['sweep_topology'] = copy.deepcopy(topology)
            if topology['passed']:
                self.sweep_passes = 1
                self.sweep_pass_steps = [self.steps]
                self.sweep_phase = 'await_close_frame'
                return self._issue(
                    {'kind':'arm','servo_id':1,'pulse':1500},
                    'first image-only jaw-sweep topology pass; request an independent cycle', obs,
                )
        fresh_open = (gripper.get('valid') and gripper.get('source') == 'isolated_gripper_motion'
                      and self.pulses.get(1) == 2000 and own is not None)
        if self.basin_active_calls >= BASIN_ACTIVE_BUDGET:
            if not fresh_open:
                self.refresh_required=True
                pulse=1500 if self.pulses.get(1,2000)==2000 else 2000
                return self._issue(
                    {'kind':'arm','servo_id':1,'pulse':pulse},
                    'remeasure before leaving the bounded local search basin', obs,
                )
            return self._next_basin(obs)
        candidates=self._candidates(alignment)
        if self.repeat=='learned':
            learned=self.jacobian.propose(alignment,self.pulses)
            if learned:
                return self._start_learned(learned,obs)
            self.repeat=None
        if self.repeat is not None:
            repeated=next(
                ((label,actions) for label,actions in self._composite_candidates()
                 if label==self.repeat), None
            )
            if repeated is not None:
                return self._start_composite(*repeated,obs)
        selected=next(((label,action) for label,action in candidates if label==self.repeat),None)
        if selected is None:
            selected=next(((label,action) for label,action in candidates if label not in self.tried),None)
        if selected is None:
            if self.search_level == 0 and 'learned' not in self.tried:
                learned=self.jacobian.propose(alignment,self.pulses)
                if learned:
                    return self._start_learned(learned,obs)
            if self.search_level == 0:
                composite=next(
                    ((label,actions) for label,actions in self._composite_candidates()
                     if label not in self.tried), None
                )
                if composite is not None:
                    return self._start_composite(*composite,obs)
            if self.search_level + 1 < len(SEARCH_SCALES):
                fresh_open = (gripper.get('valid') and gripper.get('source') == 'isolated_gripper_motion'
                              and self.pulses.get(1) == 2000)
                if not fresh_open:
                    self.refresh_required=True
                    pulse=1500 if self.pulses.get(1,2000)==2000 else 2000
                    return self._issue({'kind':'arm','servo_id':1,'pulse':pulse},
                                       'remeasure before reducing bounded search scale',obs)
                self.search_level += 1
                self.tried.clear();self.repeat=None
                candidates=self._candidates(alignment)
                selected=candidates[0]
            else:
                fresh_open = (gripper.get('valid') and gripper.get('source') == 'isolated_gripper_motion'
                              and self.pulses.get(1) == 2000 and own is not None)
                if not fresh_open:
                    self.refresh_required=True
                    pulse=1500 if self.pulses.get(1,2000)==2000 else 2000
                    return self._issue(
                        {'kind':'arm','servo_id':1,'pulse':pulse},
                        'remeasure before leaving the exhausted local search basin', obs,
                    )
                return self._next_basin(obs)
        label,action=selected
        return self._issue(action,'test one raw motion against visible beam-end and opening-axis error',obs,test=True,label=label,learn=True)
