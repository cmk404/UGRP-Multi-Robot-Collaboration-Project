"""Discrete motion policies over RGB-derived estimates only; no world access."""
from __future__ import annotations
import json
import math
import time
import urllib.request
import urllib.error

import cv2
import numpy as np
from harness.camera_goal_transport import decode
from harness.dispatch_pair_navigation import PairVision
from harness.known_map_navigation import pixel_to_world

DURATION = .2
GOAL = {'range_m': [.27, .28], 'absolute_bearing_deg_max': 3.,
        'description': 'Approach the cyan box, face its center, stop without touching it. No grasp.'}
# Values are issued motor command units, not measured speed.
VECTORS = {'forward': (.10, 0., 0.), 'backward': (-.05, 0., 0.),
           'left': (0., .06, 0.), 'right': (0., -.06, 0.),
           'turn_left': (0., 0., .06), 'turn_right': (0., 0., -.06),
           'stop': (0., 0., 0.)}
OPTIONS = {
 'forward': 'Move forward along your current heading for 0.2 seconds.',
 'backward': 'Move backward along your current heading for 0.2 seconds.',
 'left': 'Translate to your left without changing heading for 0.2 seconds.',
 'right': 'Translate to your right without changing heading for 0.2 seconds.',
 'turn_left': 'Rotate counterclockwise toward your left, no intended translation, for 0.2 seconds.',
 'turn_right': 'Rotate clockwise toward your right, no intended translation, for 0.2 seconds.',
 'stop': 'Stop and obtain another observation; also use when evidence is missing or the goal is met.'}
INSTRUCTIONS = ('Select the next short motor action to reach the goal efficiently without touching the box. '
 'Range is robot center to box center. Bearing is relative to your current forward heading: '
 'positive means target on your LEFT, negative on your RIGHT. All geometry is estimated from RGB, '
 'not simulator truth. History contains observations and issued commands, not successful motions. '
 'Use the newest state and goal; do not invent positions or motion outcomes. '
 'This is a single-robot approach task on an open floor, not grasping or transport.')


def action(name):
    f, l, t = VECTORS[name]
    return dict(kind='mecanum', forward=f, left=l, turn=t, duration_s=DURATION)


def wrap(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


def detect_box(top):
    hsv = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array((80, 125, 35), np.uint8), np.array((100, 255, 255), np.uint8))
    n, _, stats, centers = cv2.connectedComponentsWithStats(mask)
    found = [i for i in range(1, n) if 25 <= stats[i, 4] <= 600
             and .35 <= stats[i, 2] / max(1, stats[i, 3]) <= 2.8]
    if len(found) != 1:
        raise ValueError('cyan_target_missing_or_ambiguous')
    return centers[found[0]], int(stats[found[0], 4])


def box_pixel_xy(pixel, shape, camera):
    # Fixed nominal object top plane from the unchanged box specification.
    h, w = shape[:2]; cx, cy, z = camera['position_m']
    span = 2 * (z - .032) * math.tan(math.radians(camera['fov_y_deg']) / 2)
    return np.array([cx + (pixel[0] - (w - 1) / 2) * span / h,
                     cy - (pixel[1] - (h - 1) / 2) * span / h])


def wheel_envelope(mask):
    """Existing four-corner RGB shape gate with a two-pixel measurement interval."""
    h,w=mask.shape;y,x=np.nonzero(mask)
    if not 80 <= len(x) <= 700:return None
    (cx,cy),(a,b),angle=cv2.minAreaRect(np.column_stack((x,y)).astype(np.float32))
    long,short=max(a,b),min(a,b)
    if a < b:angle+=90
    angle=(angle+90)%180-90
    if not (.05*w-2 <= long <= .075*w+2 and .045*h-2 <= short <= .07*h+2
            and (long+2)/max(short-2,1) >= 1.12 and (long-2)/(short+2) <= 1.65
            and abs(angle) <= 20):return None
    theta=math.radians(angle);dx,dy=x-cx,y-cy
    u=dx*math.cos(theta)+dy*math.sin(theta);v=-dx*math.sin(theta)+dy*math.cos(theta)
    corners=[int(np.sum((u*s>.15*long)&(v*t>.15*short))) for s in (-1,1) for t in (-1,1)]
    if min(corners)<5:return None
    return {'angle_deg':float(angle),'wheel_pixels':len(x),'corner_pixels':corners,
            'shape_measurement_tolerance_px':2,'nominal_heading_domain_deg':18,'heading_pixel_tolerance_deg':2}


class RGBObserver:
    """Track own wheels after a visible own-motion probe, and one cyan box."""
    def __init__(self, static_map, before_top, after_top, motion_anchor):
        self.camera = dict(static_map['top_camera'])
        self.center = np.array(motion_anchor, dtype=float)
        first = self._wheel_observation(before_top)
        last = self._wheel_observation(after_top)
        displacement = np.array(last['xy_m']) - np.array(first['xy_m'])
        if np.linalg.norm(displacement) < .018:
            raise ValueError('own_forward_probe_not_visually_resolved')
        measured = math.atan2(displacement[1], displacement[0])
        # Motion resolves front/back only; its small displacement is too noisy for yaw calibration.
        self.heading_bias = math.pi if abs(wrap(measured-last['heading_rad'])) > math.pi/2 else 0.
        if abs(wrap(measured-last['heading_rad']-self.heading_bias)) > math.radians(20):
            raise ValueError('visual_heading_and_motion_disagree')
        self.probe_evidence = {'source': 'consecutive RGB and own issued forward probe',
                               'observed_displacement_m': displacement.tolist(),
                               'wheel_heading_bias_rad': self.heading_bias}
        self.last_box = None

    def _wheel_observation(self, jpeg):
        frame = decode(jpeg)
        mask = PairVision._mask(None, frame)
        yy, xx = np.indices(mask.shape)
        mask[(abs(xx-self.center[0]) > 43) | (abs(yy-self.center[1]) > 40)] = 0
        evidence = wheel_envelope(mask)
        if evidence is None:
            raise ValueError('four_wheel_envelope_unresolved')
        y, x = np.nonzero(mask)
        center = np.array(cv2.minAreaRect(np.column_stack((x,y)).astype(np.float32))[0])
        heading = -math.radians(evidence['angle_deg'])
        if np.linalg.norm(center - self.center) > 30:
            raise ValueError('own_robot_identity_jump')
        self.center = center
        return {'center_px': center.tolist(),
                'xy_m': list(pixel_to_world(center, frame.shape, self.camera)),
                'heading_rad': wrap(heading + getattr(self, 'heading_bias', 0.)),
                'tracking': evidence, 'source': 'current four-wheel RGB geometry'}

    def observe(self, own_jpeg, top_jpeg):
        own, top = decode(own_jpeg), decode(top_jpeg)
        robot = self._wheel_observation(top_jpeg)
        center, area = detect_box(top)
        if self.last_box is not None and np.linalg.norm(center - self.last_box) > 12:
            raise ValueError('target_identity_jump')
        self.last_box = center.copy()
        target = box_pixel_xy(center, top.shape, self.camera)
        delta = target - np.array(robot['xy_m'])
        heading = robot['heading_rad']
        bearing = wrap(math.atan2(delta[1], delta[0]) - heading)
        hsv = cv2.cvtColor(own, cv2.COLOR_BGR2HSV)
        cyan = cv2.inRange(hsv, np.array((80,125,35),np.uint8), np.array((100,255,255),np.uint8))
        return {'valid': True, 'source': 'RGB estimates with fixed camera calibration',
                'range_m': round(float(np.linalg.norm(delta)), 4),
                'bearing_deg': round(math.degrees(bearing), 2),
                'target_forward_m': round(float(delta @ [math.cos(heading), math.sin(heading)]), 4),
                'target_left_m': round(float(delta @ [-math.sin(heading), math.cos(heading)]), 4),
                'own_view_cyan_fraction': round(float(np.mean(cyan > 0)), 6),
                'top_target_area_px': area,
                'evidence': {'robot': robot, 'target_center_px': center.tolist()}}


def policy_state(observation, history):
    # Explicit allowlist: exclude world/reference data and raw filesystem paths.
    keys = ('valid', 'range_m', 'bearing_deg', 'target_forward_m', 'target_left_m',
            'own_view_cyan_fraction', 'top_target_area_px')
    if observation.get('valid') is not True:
        raise ValueError('invalid RGB observation')
    current = {k: observation[k] for k in keys}
    if any(type(current[k]) not in (int,float) or not math.isfinite(current[k]) for k in keys if k != 'valid'):
        raise ValueError('invalid observation number')
    return {'goal': GOAL, 'observation': current,
            'recent_history': [{k: row[k] for k in ('action','range_m','bearing_deg')} for row in history[-4:]]}


def at_goal(observation):
    return bool(observation.get('valid') is True and GOAL['range_m'][0] <= observation['range_m'] <= GOAL['range_m'][1]
                and abs(observation['bearing_deg']) <= GOAL['absolute_bearing_deg_max'])


def rule(state):
    o = state['observation']
    if abs(o['bearing_deg']) > GOAL['absolute_bearing_deg_max']:
        return 'turn_left' if o['bearing_deg'] > 0 else 'turn_right'
    if o['range_m'] > GOAL['range_m'][1]: return 'forward'
    if o['range_m'] < GOAL['range_m'][0]: return 'backward'
    return 'stop'


def jev_request(state, model='jev-1.13.0'):
    return {'model':model, 'state':state, 'questions':{'action':{
        'type':'choice','instructions':INSTRUCTIONS,'criteria':OPTIONS}}}


def validate_jev(body):
    a = body['answers']['action']; p = a['probabilities']
    if a['type'] != 'choice' or a['choice'] not in OPTIONS or set(p) != set(OPTIONS):
        raise ValueError('invalid Jev options')
    if any(type(v) not in (int,float) or not math.isfinite(v) or not 0 <= v <= 1 for v in [*p.values(),a['confidence']]):
        raise ValueError('invalid Jev probability')
    # The live API serializes probabilities rounded to two decimal places.
    rounded = all(abs(v*100-round(v*100)) < 1e-8 for v in p.values())
    sum_tolerance = .005*len(p)+1e-9 if rounded else .001
    if abs(sum(p.values())-1) > sum_tolerance:
        raise ValueError('invalid Jev distribution')
    return a['choice']


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs): return None


def post(body, url, key=None, timeout=30):
    headers = {'Content-Type':'application/json'}
    if key: headers['Authorization'] = 'Bearer ' + key
    req = urllib.request.Request(url, json.dumps(body, allow_nan=False).encode(), headers)
    start = time.perf_counter()
    try:
        with urllib.request.build_opener(NoRedirect).open(req, timeout=timeout) as response:
            status, raw = response.status, response.read(2_000_000).decode()
    except urllib.error.HTTPError as exc:
        status, raw = exc.code, exc.read(2_000_000).decode(errors='replace')
    except (OSError, urllib.error.URLError) as exc:
        return {'status':'transport_error','error_type':type(exc).__name__, 'latency_s':time.perf_counter()-start}
    if key: raw = raw.replace(key,'[REDACTED]')
    result = {'http_status':status,'raw_response':raw,'latency_s':time.perf_counter()-start,'status':'http_error'}
    if status == 200:
        try: result['body'] = json.loads(raw); result['status'] = 'ok'
        except ValueError: result['status'] = 'invalid_json'
    return result


def gemini_request(state, model):
    return {'model':model, 'messages':[{'role':'system','content':INSTRUCTIONS + '\nReturn only a JSON object with one key action and one of these choices: ' + json.dumps(OPTIONS)},
        {'role':'user','content':json.dumps(state,sort_keys=True)}], 'temperature':0.,
        'max_tokens':128, 'reasoning_effort':'none', 'response_format':{'type':'json_object'}}


def validate_gemini(body):
    raw = body['choices'][0]['message']['content'].strip()
    if raw.startswith('```'): raw = raw.split('\n',1)[1].rsplit('```',1)[0]
    a = json.loads(raw)
    if set(a) != {'action'} or a['action'] not in OPTIONS: raise ValueError('invalid Gemini action')
    return a['action']
