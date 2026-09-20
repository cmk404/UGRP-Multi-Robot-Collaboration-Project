"""RGB-only short skills and typed decisions; no simulator or referee imports.

The observer estimates geometry from saved camera pixels. Command values are
issued units, never measured velocities. All thresholds are frozen per cohort.
"""
from __future__ import annotations

import itertools
import json
import math
from collections import deque

import cv2
import numpy as np

from harness.camera_goal_transport import decode
from harness.dispatch_pair_navigation import PairVision
from harness.jev_motion import GOAL, OPTIONS, action, detect_box, wrap

DT = .25
ROBOT_RADIUS = .14
WHEEL_PLANE = .058  # nominal visible wheel surface from the unchanged geometry
MAX_HOLD_TICKS = 6
MODEL_NAMES = {'jev': 'jev-1.13.0', 'gemini': 'gemini-3.8-flash'}


def project(pixel, shape, camera, plane):
    if list(camera['quaternion_wxyz']) != [1, 0, 0, 0]:
        raise ValueError('only fixed nadir calibration is supported')
    h, w = shape[:2]
    cx, cy, z = camera['position_m']
    scale = 2 * (z-plane) * math.tan(math.radians(camera['fov_y_deg'])/2) / h
    return np.array([cx+(pixel[0]-(w-1)/2)*scale,
                     cy-(pixel[1]-(h-1)/2)*scale])


def fit_wheels(frame, anchor, prior_heading):
    """Fit the known four wheel centres to >=3 visible RGB components.

    Unlike a whole-colour bounding rectangle, internal yellow arm fragments do
    not define the heading. Association is local and orientation is continuous.
    """
    mask = PairVision._mask(None, frame)
    yy, xx = np.indices(mask.shape)
    mask[(abs(xx-anchor[0]) > 60) | (abs(yy-anchor[1]) > 60)] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, _, stats, centers = cv2.connectedComponentsWithStats(mask)
    points = np.array([centers[i] for i in range(1, n) if 45 <= stats[i, 4] <= 240])
    if len(points) < 3 or len(points) > 16:
        raise ValueError('wheel_components_unresolved')
    # Same fixed camera and production wheelbase/track; no measured pose.
    scale = 2*(2.5-WHEEL_PLANE)*math.tan(math.radians(55)/2)/frame.shape[0]
    corners = np.array(list(itertools.product((-.060, .060), (-.0655, .0655))))/scale
    best = None
    for i, j in itertools.permutations(range(len(points)), 2):
        delta = points[j]-points[i]
        for a, b in ((0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)):
            ref = corners[b]-corners[a]
            if abs(np.linalg.norm(delta)-np.linalg.norm(ref)) > 5:
                continue
            angle = wrap(math.atan2(delta[1], delta[0])-math.atan2(ref[1], ref[0]))
            if abs(wrap(-angle-prior_heading)) > math.radians(32):
                continue
            rot = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
            center = (points[i]+points[j]-(corners[a]+corners[b])@rot.T)/2
            expected = corners@rot.T+center
            distances = np.linalg.norm(expected[:, None, :]-points[None, :, :], axis=2)
            matches = distances.argmin(axis=1)
            errors = distances[np.arange(4), matches]
            use = errors < 4.5
            if sum(use) < 3 or len(set(matches[use])) != sum(use):
                continue
            # One wheel may be occluded/fragmented while an arm fragment sits
            # near its expected corner. Use the best three correspondences;
            # a fourth noisy point must not rotate an otherwise tight fit.
            order = np.argsort(errors)
            use = np.zeros(4, dtype=bool)
            use[order[:3]] = True
            score = float(np.mean(errors[order[:3]]**2)+.03*min(errors[order[3]]**2,49))
            if best is None or score < best[0]:
                best = (score, corners[use], points[matches[use]], len(matches[use]))
    if best is None:
        raise ValueError('wheel_geometry_unresolved')
    _, reference, measured, count = best
    a, b = reference-reference.mean(axis=0), measured-measured.mean(axis=0)
    u, _, vt = np.linalg.svd(a.T@b)
    rotation = u@vt
    if np.linalg.det(rotation) < 0:
        u[:, -1] *= -1
        rotation = u@vt
    center = measured.mean(axis=0)-reference.mean(axis=0)@rotation
    angle = math.atan2(rotation[0, 1], rotation[0, 0])
    residual = float(np.sqrt(np.mean(np.sum((reference@rotation+center-measured)**2, axis=1))))
    if residual > 3.5 or np.linalg.norm(center-anchor) > 35:
        raise ValueError('wheel_fit_uncertain')
    return center, wrap(-angle), {'matched_wheels':count, 'rms_px':residual,
                                  'method':'RGB component rigid fit', 'feature_plane_m':WHEEL_PLANE}


class SkillObserver:
    def __init__(self, static_map, before_top, after_top, motion_anchor):
        self.camera = dict(static_map['top_camera'])
        before, after = decode(before_top), decode(after_top)
        self.center = np.array(motion_anchor, dtype=float)
        # Translation of the compact mask resolves direction before rigid fitting.
        def centroid(frame):
            mask = PairVision._mask(None, frame)
            yy, xx = np.indices(mask.shape)
            mask[(abs(xx-self.center[0])>65)|(abs(yy-self.center[1])>65)] = 0
            y, x = np.nonzero(mask)
            if len(x) < 80: raise ValueError('identity_wheels_missing')
            return np.array([float(np.mean(x)), float(np.mean(y))])
        shift = centroid(after)-centroid(before)
        if np.linalg.norm(shift) < 5: raise ValueError('identity_probe_not_resolved')
        heading = math.atan2(-shift[1], shift[0])
        self.center, self.heading, self.fit = fit_wheels(after, self.center, heading)
        self.recent = deque(maxlen=3)
        self.previous = None
        self.last_target = None
        self.probe_evidence = {'source':'own issued forward probe and consecutive RGB',
                               'displacement_px':shift.tolist(), 'fit':self.fit}

    def observe(self, own_jpeg, top_jpeg):
        decode(own_jpeg)
        frame = decode(top_jpeg)
        center, heading, fit = fit_wheels(frame, self.center, self.heading)
        self.center, self.heading = center, heading
        target_px, area = detect_box(frame)
        if self.last_target is not None and np.linalg.norm(target_px-self.last_target)>15:
            raise ValueError('target_identity_jump')
        self.last_target = target_px.copy()
        xy = project(center, frame.shape, self.camera, WHEEL_PLANE)
        target = project(target_px, frame.shape, self.camera, .032)
        delta = target-xy
        raw_range = float(np.linalg.norm(delta))
        raw_bearing = math.degrees(wrap(math.atan2(delta[1], delta[0])-heading))
        self.recent.append((raw_range, raw_bearing))
        distance, bearing = np.median(np.array(self.recent), axis=0)
        trend = 'unknown'
        if self.previous is not None:
            change = distance-self.previous
            trend = 'closing' if change<-.002 else 'opening' if change>.002 else 'steady'
        self.previous = float(distance)
        # Orange barriers are physically present RGB evidence, never setup labels.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        barrier = cv2.inRange(hsv, np.array([5, 140, 65],np.uint8), np.array([15,255,255],np.uint8))
        count, _, stats, centers = cv2.connectedComponentsWithStats(barrier)
        obstacles = []
        for i in range(1, count):
            if stats[i, 4] < 150: continue
            x, y, w, h, pixels = stats[i]
            lo = project([x,y+h],frame.shape,self.camera,.16)
            hi = project([x+w,y],frame.shape,self.camera,.16)
            obstacles.append({'center_m':((lo+hi)/2).tolist(),
                              'half_extents_m':(abs(hi-lo)/2).tolist(),'source':'current orange RGB component'})
        return {'valid':True, 'range_m':round(float(distance),4), 'bearing_deg':round(float(bearing),2),
                'raw_range_m':raw_range, 'raw_bearing_deg':raw_bearing,
                'xy_m':xy.tolist(), 'target_xy_m':target.tolist(), 'heading_rad':heading,
                'range_trend':trend, 'quality':'clear' if fit['rms_px']<2 else 'bounded_fit',
                'range_spread_m':float(np.ptp(np.array(self.recent)[:,0])),
                'bearing_spread_deg':float(np.ptp(np.array(self.recent)[:,1])),
                'obstacles':obstacles, 'evidence':{'center_px':center.tolist(), 'target_px':target_px.tolist(),
                'target_area_px':area, 'wheel_fit':fit}, 'source':'RGB estimates only'}


def point_clear(point, obstacles, bounds, margin=ROBOT_RADIUS):
    x, y = point
    if not (bounds[0]+margin <= x <= bounds[1]-margin and bounds[2]+margin <= y <= bounds[3]-margin):
        return False
    return not any(abs(x-o['center_m'][0]) <= o['half_extents_m'][0]+margin and
                   abs(y-o['center_m'][1]) <= o['half_extents_m'][1]+margin for o in obstacles)


def segment_clear(a, b, obstacles, bounds):
    return all(point_clear((1-t)*np.array(a)+t*np.array(b),obstacles,bounds)
               for t in np.linspace(0,1,max(2,int(math.dist(a,b)/.02)+1)))


def routes(observation, static_map):
    """Shortlist geometric route alternatives from RGB and the authored map."""
    xy = np.array(observation['xy_m']); target = np.array(observation['target_xy_m'])
    # All trials approach from the west; a fixed task convention, not actual pose.
    goal = target-np.array([.275, 0.])
    obstacles = [dict(o) for o in static_map['obstacles'] if o['kind']!='wall']+observation['obstacles']
    bounds = static_map['bounds_m']
    candidates = {}
    paths = {'direct':[goal.tolist()]}
    # A bounded family of waypoints, not a ground-truth path oracle.
    for side, sign in [('north',1),('south',-1)]:
        for offset in (.24,.38,.52):
            y = xy[1]+sign*offset
            paths[f'{side}_{int(offset*100)}']=[[float(xy[0]),float(y)],[float(goal[0]),float(y)],goal.tolist()]
    for name,path in paths.items():
        points=[xy.tolist()]+path
        if all(segment_clear(a,b,obstacles,bounds) for a,b in zip(points,points[1:])):
            length=sum(math.dist(a,b) for a,b in zip(points,points[1:]))
            side='direct' if name=='direct' else name.split('_')[0]
            if side not in candidates or length<candidates[side]['length_m']:
                candidates[side]={'waypoints':path,'length_m':length,'relation':side,
                    'effect':'approach directly' if side=='direct' else f'pass on the {side} side before approaching',
                    'source':'RGB + prior map geometric proposal'}
    return candidates


class CompletionGate:
    """Strict unchanged goal on filtered RGB plus stationary image evidence."""
    def __init__(self): self.count=0
    def update(self,o):
        inside=(o.get('valid') is True and .27<=o['range_m']<=.28 and abs(o['bearing_deg'])<=3)
        stable=inside and o.get('range_spread_m',1)<=.004 and o.get('bearing_spread_deg',99)<=2
        self.count=self.count+1 if stable else 0
        return self.count>=3


def semantic_state(o, candidate_descriptions, history, active, age, stalls):
    """Explicit allowlist excludes referee data and environment event schedule."""
    if not o.get('valid'): raise ValueError('invalid observation')
    distance=o['range_m']; angle=o['bearing_deg']
    return {'task':'Reach a stable pose 0.27–0.28 m from the cyan target, face it, without contact. No grasp.',
            'observation':{'distance':'too_close' if distance<.27 else 'inside' if distance<=.28 else 'near' if distance<.36 else 'far',
            'target_side':'left' if angle>0 else 'right' if angle<0 else 'ahead',
            'alignment':'aligned' if abs(angle)<=3 else 'small_error' if abs(angle)<=8 else 'large_error',
            'range_trend':o['range_trend'],'quality':o['quality'],
            'goal_distance_edge':min(abs(distance-.27),abs(distance-.28))<.003,
            'goal_alignment_edge':distance<.36 and abs(abs(angle)-3)<1,
            'stationary_evidence':o['range_spread_m']<.003 and o['bearing_spread_deg']<1.5},
            'active_skill':active,'skill_age_ticks':age,'no_progress_ticks':stalls,
            'persistent_stall':stalls>=4,
            'detour_note':'A detour can temporarily increase straight-line target distance while following a clear route.',
            'recent_issued':[{'skill':h['skill'],'progress':h['progress']} for h in history[-3:]],
            'candidates':candidate_descriptions,
            'input_boundary':'RGB, fixed calibration/prior map, and own issued commands. Commands are not measured motion.'}


def questions(candidates, decomposed=True):
    result={'action':{'type':'choice','instructions':
        'Choose ONE available short skill making useful progress toward the task. '
        'Use its stated effect, current observation and progress. '
        'A skill is interrupted by fresh RGB and has a bounded duration. Holding is not task completion. '
        'Prefer the shortest clear route when starting. Avoid restarting a detour or reversing a productive '
        'choice merely because another observation arrived. All listed skills meet their execution preconditions. '
        'If feasible routes tie, either is acceptable: pick one. Their local feedback handles heading errors. '
        'Do not hold solely because routes tie or the heading is near its tolerance. '
        'After repeated holds with usable RGB and no motion, make progress with a feasible moving skill.',
        'criteria':candidates}}
    if decomposed:
        result['evidence']={'type':'choice','instructions':'Judge only whether the provided RGB observation is usable for a short skill choice.',
            'criteria':{'usable':'Current RGB quality is clear or bounded_fit; ordinary small boundary variation is acceptable. Bounded fits use shorter commitments.',
                        'observe_again':'The supplied evidence conflicts or is insufficient even for a short bounded skill; obtain stopped fresh RGB.'}}
        result['progress']={'type':'choice','instructions':'Judge only whether the recent issued skill is making visual progress.',
            'criteria':{'continue':'Image motion is visible or the skill has just begun; detours may increase target distance.',
                        'reconsider':'Persistent stall evidence shows that the issued skill is not producing useful image motion.'}}
    return result


def request(state, policy, decomposed=True):
    qs=questions(state['candidates'],decomposed)
    if policy=='jev': return {'model':MODEL_NAMES[policy],'state':state,'questions':qs}
    if policy!='gemini': raise ValueError('unknown model')
    return {'model':MODEL_NAMES[policy], 'temperature':0., 'messages':[
        {'role':'system','content':'Evaluate each question against the same state. Return JSON with one key per question, each value the selected option key. '+json.dumps(qs)},
        {'role':'user','content':json.dumps(state,sort_keys=True)}]}


def parse_answer(body, policy, state, decomposed=True):
    qs=questions(state['candidates'],decomposed)
    answers={}; confidence=None
    if policy=='gemini':
        raw=body['choices'][0]['message']['content'].strip()
        if raw.startswith('```'): raw=raw.removeprefix('```json').removeprefix('```').removesuffix('```').strip()
        values=json.loads(raw)
        if set(values)!=set(qs): raise ValueError('invalid Gemini question keys')
        for name,value in values.items():
            if value not in qs[name]['criteria']: raise ValueError('invalid Gemini choice')
            answers[name]=value
    else:
        if set(body['answers'])!=set(qs): raise ValueError('invalid Jev question keys')
        for name,q in qs.items():
            a=body['answers'][name];p=a['probabilities']
            if a['type']!='choice' or a['choice'] not in q['criteria'] or set(p)!=set(q['criteria']):
                raise ValueError('invalid Jev choice')
            if any(type(v) not in (int,float) or not math.isfinite(v) or not 0<=v<=1 for v in [*p.values(),a['confidence']]):
                raise ValueError('invalid Jev probabilities')
            if abs(sum(p.values())-1)>.005*len(p)+1e-9: raise ValueError('invalid Jev distribution')
            answers[name]=a['choice']
            if name=='action':confidence=a['confidence']
    return answers,confidence


def bounded_command(forward=0.,left=0.,turn=0.):
    return {'kind':'mecanum','forward':float(np.clip(forward,-.10,.10)),
            'left':float(np.clip(left,-.08,.08)),'turn':float(np.clip(turn,-.09,.09)),'duration_s':.2}


class SkillController:
    """Model chooses a skill; this object executes only that skill using RGB.

    No model choice is silently replaced with the reference policy. Visibility,
    completion and collision-envelope holds are separately logged interventions.
    """
    def __init__(self, static_map, *, always_query=False, primitive=False):
        self.map=static_map
        self.always_query=always_query
        self.primitive=primitive
        self.active=None; self.path=[]; self.age=0; self.stalls=0
        self.history=[]; self.last_xy=None; self.last_signature=None
        self.last_heading=None
        self.gate=CompletionGate(); self.done=False; self.low_confidence_holds=0

    def describe(self,o):
        if self.primitive:
            self.proposals={}
            return dict(OPTIONS)
        near=o['range_m']<.36
        self.proposals=routes(o,self.map) if not near else {}
        if self.path:
            obstacles=[x for x in self.map['obstacles'] if x['kind']!='wall']+o['obstacles']
            points=[o['xy_m']]+self.path
            if all(segment_clear(a,b,obstacles,self.map['bounds_m']) for a,b in zip(points,points[1:])):
                self.proposals['continue_route']={'waypoints':list(self.path),
                    'length_m':sum(math.dist(a,b) for a,b in zip(points,points[1:])),
                    'effect':'continue the selected route without restarting its detour','relation':'continue'}
        candidates={}
        shortest=min((v['length_m'] for v in self.proposals.values()),default=0.)
        if not near:
            for key,value in self.proposals.items():
                candidates[key]=(value['effect']+'. RGB-estimated remaining route '+
                                 ('short' if value['length_m']<.45 else 'medium' if value['length_m']<.9 else 'long')+
                                 (', tied shortest available' if abs(value['length_m']-shortest)<.03 else ', longer than another available route')+
                                 '. Locally align and follow these waypoints; stop if RGB becomes unreliable or the route is blocked.')
        else:
            if abs(o['bearing_deg'])>3:
                candidates['align_target']='Use RGB feedback to face the target with small signed corrections; stop at heading tolerance.'
            elif o['range_m']>.28:
                candidates['approach_fine']='Small forward approach toward the distance band; stop on reaching the band.'
            elif o['range_m']<.27:
                candidates['backoff_fine']='Small backward movement to increase target distance; stop on reaching the band.'
        candidates['hold_and_observe']='Brake briefly to resolve conflicting evidence or a temporary blockage. Repeated holding with usable evidence and a feasible route does not make task progress. This does not claim completion.'
        return candidates

    def state(self,o):
        candidates=self.describe(o)
        return semantic_state(o,candidates,self.history,self.active,self.age,self.stalls)

    def observe_progress(self,o):
        xy=np.array(o['xy_m'])
        if self.last_xy is not None:
            moved=np.linalg.norm(xy-self.last_xy)>=.001
            turned=abs(wrap(o['heading_rad']-self.last_heading))>=math.radians(.15)
            if self.history:
                self.history[-1]['progress']='moving' if moved else 'turning' if turned else 'no_motion'
                self.stalls=0 if moved or turned else self.stalls+1
        self.last_xy=xy
        self.last_heading=o['heading_rad']
        self.done=self.gate.update(o)

    def need_query(self,state):
        obs=state['observation']
        signature=(obs['distance'],obs['alignment'],obs['quality'],
                   tuple(k for k in state['candidates'] if k!='continue_route'),self.stalls>=4)
        reason=None
        if self.active is None:reason='initial_or_interrupted'
        elif self.always_query or self.primitive:reason='every_observation_ablation'
        elif self.active not in state['candidates'] and not (self.path and 'continue_route' in state['candidates']):reason='skill_finished_or_blocked'
        elif self.age>=(2 if getattr(self,'cautious',False) else MAX_HOLD_TICKS):reason='decision_expired'
        elif signature!=self.last_signature:reason='semantic_state_changed'
        elif self.active=='hold_and_observe' and self.age>=2:reason='fresh_observation_after_hold'
        if reason:self.last_signature=signature
        return reason

    def select(self,name,o,confidence=None):
        if name not in self.describe(o):raise ValueError('unavailable skill')
        self.history.append({'skill':name,'progress':'pending_visual_observation'})
        self.age=0;self.active=name
        if name in self.proposals:self.path=list(self.proposals[name]['waypoints'])
        elif name!='hold_and_observe':self.path=[]
        # A provisional, predeclared engineering threshold, not calibrated safety.
        # Ambiguous choice between feasible routes is not evidence of danger.
        # Shorten the commitment; do not endlessly hold or substitute a rule.
        self.cautious=(confidence is not None and confidence<.45) or o['quality']=='bounded_fit'

    def command(self,o):
        self.age+=1
        if self.done:return bounded_command(),'RGB_completion'
        if self.active in (None,'hold_and_observe'):
            return bounded_command(),'selected_hold'
        if .27<=o['range_m']<=.28 and abs(o['bearing_deg'])<=3:
            return bounded_command(),'RGB_goal_settling'
        if self.primitive:
            return action(self.active),'selected_primitive'
        cautious=1.  # Confidence controls duration, not uncalibrated motor torque.
        distance=o['range_m'];angle=o['bearing_deg']
        if self.active=='align_target':
            if abs(angle)<=2.0:
                self.active=None
                return bounded_command(),'alignment_reached'
            sign=1 if angle>0 else -1
            return bounded_command(turn=sign*(.065 if abs(angle)>8 else .025)*cautious),'selected_alignment'
        if self.active=='approach_fine':
            if distance<=.276:
                self.active=None
                return bounded_command(),'distance_reached'
            speed=.045 if distance>.30 else .022 if distance>.285 else .012
            cmd=bounded_command(forward=speed*cautious)
        elif self.active=='backoff_fine':
            if distance>=.274:
                self.active=None
                return bounded_command(),'distance_reached'
            cmd=bounded_command(forward=-.018*cautious)
        elif self.path:
            xy=np.array(o['xy_m'])
            obstacles=[x for x in self.map['obstacles'] if x['kind']!='wall']+o['obstacles']
            while len(self.path)>1 and math.dist(xy,self.path[0])<.035:
                # Proximity alone can cut the inflated corner and invalidate
                # the route on the next frame. Complete the turn only when
                # its next segment is clear from the current RGB estimate.
                if not segment_clear(xy,self.path[1],obstacles,self.map['bounds_m']):break
                self.path.pop(0)
            delta=np.array(self.path[0])-xy
            if len(self.path)==1 and np.linalg.norm(delta)<.03:
                self.path=[];self.active=None
                return bounded_command(),'waypoint_reached'
            heading=o['heading_rad']
            if abs(heading)>math.radians(8):
                cmd=bounded_command(turn=-np.sign(heading)*.065*cautious)
            else:
                unit=delta/max(np.linalg.norm(delta),1e-9)
                f=float(unit@[math.cos(heading),math.sin(heading)])
                l=float(unit@[-math.sin(heading),math.cos(heading)])
                gain=.10 if np.linalg.norm(delta)>.09 else .045
                cmd=bounded_command(forward=gain*f*cautious,left=gain*l*cautious,
                    turn=float(np.clip(-heading*.3,-.025,.025)))
        else:
            self.active=None
            return bounded_command(),'skill_finished'
        # Predeclared conservative RGB swept envelope, not contact feedback.
        heading=o['heading_rad']
        step=np.array([math.cos(heading)*cmd['forward']-math.sin(heading)*cmd['left'],
                       math.sin(heading)*cmd['forward']+math.cos(heading)*cmd['left']])*.20
        obstacles=[x for x in self.map['obstacles'] if x['kind']!='wall']+o['obstacles']
        if not segment_clear(o['xy_m'],np.array(o['xy_m'])+step,obstacles,self.map['bounds_m']):
            self.path=[];self.active=None
            return bounded_command(),'RGB_route_blocked'
        return cmd,'selected_skill_feedback'


def reference_choice(state):
    """Explicit baseline using exactly the same candidates, with no world data."""
    o=state['observation'];c=state['candidates']
    if 'forward' in c:
        if o['alignment']!='aligned':return 'turn_left' if o['target_side']=='left' else 'turn_right'
        if o['distance'] in ('near','far'):return 'forward'
        if o['distance']=='too_close':return 'backward'
        return 'stop'
    if o['quality'] not in ('clear','bounded_fit'):return 'hold_and_observe'
    if 'continue_route' in c:return 'continue_route'
    for key in ('direct','north','south'):
        if key in c:return key
    if 'align_target' in c:return 'align_target'
    if o['distance'] in ('near','far') and 'approach_fine' in c:return 'approach_fine'
    if o['distance']=='too_close' and 'backoff_fine' in c:return 'backoff_fine'
    return 'hold_and_observe'


def request_compatible(old,new):
    """Whether a delayed choice still has the same bounded decision context."""
    a,b=old['observation'],new['observation']
    keys=('distance','alignment','quality')
    if any(a[k]!=b[k] for k in keys):return False
    if a['alignment']!='aligned' and a['target_side']!=b['target_side']:return False
    return set(old['candidates'])==set(new['candidates'])
