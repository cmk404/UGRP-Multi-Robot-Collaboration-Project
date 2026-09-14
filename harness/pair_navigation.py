"""RGB-only paired load navigation on an authored static map.

No simulator imports or measured state enter this module. The color/template
observer is a classical fixed-scene student, not an LLM or a grasp sensor.
"""
from __future__ import annotations

import hashlib
import heapq
import json
import math
from pathlib import Path

import cv2
import numpy as np

from harness.known_map_navigation import _decode_jpeg, pixel_to_world
from harness.pair_carry_sync import PairCarrySync

ROBOTS = ('r1', 'r3')
CAMERA = {'name': 'cctv_top', 'position_m': [.55, -2., 2.5],
          'quaternion_wxyz': [1., 0., 0., 0.], 'fov_y_deg': 55.}


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def rotate(point, angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([c * point[0] - s * point[1], s * point[0] + c * point[1]])


def validate_map(data):
    required = {'schema', 'map_id', 'top_camera', 'bounds_m', 'start_zone',
                'goal', 'obstacles', 'footprint', 'grid_m'}
    if set(data) != required or data['schema'] != 'ugrp.pair_navigation_map.v1':
        raise ValueError('invalid pair map schema')
    if data['top_camera'] != CAMERA:
        raise ValueError('the approved actor camera must remain fixed')
    if not isinstance(data['map_id'], str) or not data['map_id']:
        raise ValueError('map_id required')
    def vector(values, count):
        if not isinstance(values, list) or len(values) != count or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
            raise ValueError('finite map coordinates required')
        return values
    xmin, xmax, ymin, ymax = vector(data['bounds_m'], 4)
    if not xmin < xmax or not ymin < ymax:
        raise ValueError('empty map bounds')
    if set(data['start_zone']) != {'center_m', 'radius_m'}:
        raise ValueError('invalid start zone')
    vector(data['start_zone']['center_m'], 2)
    if not .1 <= data['start_zone']['radius_m'] <= .8:
        raise ValueError('invalid start radius')
    if set(data['goal']) != {'center_m', 'relative_yaw_deg'}:
        raise ValueError('invalid goal')
    vector(data['goal']['center_m'], 2)
    vector([data['goal']['relative_yaw_deg']], 1)
    if abs(data['goal']['relative_yaw_deg']) > 180:
        raise ValueError('goal yaw out of range')
    if set(data['footprint']) != {'half_forward_m', 'half_lateral_m', 'margin_m'}:
        raise ValueError('invalid loaded footprint')
    f = data['footprint']
    vector(list(f.values()), 3)
    if not (.18 <= f['half_forward_m'] <= .4 and .49 <= f['half_lateral_m'] <= .7
            and .02 <= f['margin_m'] <= .1):
        raise ValueError('footprint must include both chassis, arms and beam')
    if isinstance(data['grid_m'], bool) or not .03 <= data['grid_m'] <= .1:
        raise ValueError('invalid grid size')
    ids = set()
    for box in data['obstacles']:
        if set(box) != {'id', 'center_m', 'half_extents_m', 'height_m'}:
            raise ValueError('invalid static wall')
        if not isinstance(box['id'], str) or not box['id'].replace('_', '').isalnum() or box['id'] in ids:
            raise ValueError('invalid/duplicate wall id')
        ids.add(box['id'])
        x, y = vector(box['center_m'], 2)
        hx, hy = vector(box['half_extents_m'], 2)
        vector([box['height_m']], 1)
        if min(hx, hy, box['height_m']) <= 0 or not (xmin <= x-hx < x+hx <= xmax and ymin <= y-hy < y+hy <= ymax):
            raise ValueError('wall outside map')
    return data


def footprint_clear(pose, data):
    """Separating-axis test for an oriented complete-load rectangle vs walls."""
    x, y, angle = pose
    f = data['footprint']
    hx, hy = f['half_forward_m'] + f['margin_m'], f['half_lateral_m'] + f['margin_m']
    c, s = math.cos(angle), math.sin(angle)
    ex, ey = abs(c)*hx + abs(s)*hy, abs(s)*hx + abs(c)*hy
    xmin, xmax, ymin, ymax = data['bounds_m']
    if not (xmin <= x-ex and x+ex <= xmax and ymin <= y-ey and y+ey <= ymax):
        return False
    for box in data['obstacles']:
        dx, dy = box['center_m'][0]-x, box['center_m'][1]-y
        bx, by = box['half_extents_m']
        if (abs(dx) > ex+bx or abs(dy) > ey+by or
            abs(c*dx+s*dy) > hx+abs(c)*bx+abs(s)*by or
            abs(-s*dx+c*dy) > hy+abs(s)*bx+abs(c)*by):
            continue
        return False
    return True


def swept_clear(start, end, data):
    distance = math.dist(start[:2], end[:2])
    delta = wrap(end[2]-start[2])
    count = max(1, math.ceil(distance/.012), math.ceil(abs(delta)/math.radians(2)))
    # The margin also bounds the sub-sample swept corner motion (< 0.012 m).
    return all(footprint_clear((start[0]+(end[0]-start[0])*t/count,
                               start[1]+(end[1]-start[1])*t/count,
                               start[2]+delta*t/count), data) for t in range(count+1))


def plan_route(start, goal, data):
    """SE(2) lattice search. Every translation/rotation edge checks swept load."""
    validate_map(data)
    start, goal = tuple(start), tuple(goal)
    if not footprint_clear(start, data) or not footprint_clear(goal, data):
        return None
    if swept_clear(start, goal, data):
        return [list(start), list(goal)]
    grid, bins = data['grid_m'], 8
    origin = np.array(start[:2])
    def pose(node):
        return (origin[0]+node[0]*grid, origin[1]+node[1]*grid, start[2]+node[2]*2*math.pi/bins)
    def heuristic(p):
        return math.dist(p[:2], goal[:2]) + .18*abs(wrap(p[2]-goal[2]))
    initial = (0, 0, 0)
    frontier, costs, parent = [(heuristic(start), 0., initial)], {initial: 0.}, {}
    moves = [(dx, dy, 0) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy] + [(0, 0, 1), (0, 0, -1)]
    closed = set()
    while frontier:
        _, cost, node = heapq.heappop(frontier)
        if node in closed:
            continue
        closed.add(node)
        p = pose(node)
        if math.dist(p[:2], goal[:2]) <= grid*1.5 and abs(wrap(p[2]-goal[2])) < .05 and swept_clear(p, goal, data):
            nodes = [node]
            while nodes[-1] != initial:
                nodes.append(parent[nodes[-1]])
            route = [list(pose(n)) for n in reversed(nodes)] + [list(goal)]
            # Greedy shortcuts retain the same complete swept-footprint test.
            smooth = [route[0]]
            index = 0
            while index < len(route)-1:
                last = len(route)-1
                while last > index+1 and not swept_clear(route[index], route[last], data):
                    last -= 1
                smooth.append(route[last]); index = last
            return smooth
        for dx, dy, da in moves:
            nxt = (node[0]+dx, node[1]+dy, (node[2]+da) % bins)
            if nxt in closed:
                continue
            q = pose(nxt)
            step = math.hypot(dx, dy)*grid + abs(da)*.18*2*math.pi/bins
            new = cost+step
            if new >= costs.get(nxt, math.inf) or not swept_clear(p, q, data):
                continue
            costs[nxt], parent[nxt] = new, node
            heapq.heappush(frontier, (new+heuristic(q), new, nxt))
    return None


class PairVision:
    """Track unchanged wheel color templates; headings are image rotations.

    At initialization the long orange component identifies the beam. Robot
    roles are assigned by the declared lower/upper start sides, not live poses.
    A shared forward probe subsequently resolves the initial heading sign.
    """
    def __init__(self, data):
        self.map = validate_map(data)
        self.templates = {}
        self.centers = {}
        self.angles = {r: 0. for r in ROBOTS}
        self.payload = None
        self.payload_origin_angle = None
        self.payload_angle = None
        root = Path(__file__).parent/'assets'/'pair_navigation'
        metadata = json.loads((root/'manifest.json').read_text())
        self.initial_templates = {}
        for rid in ROBOTS:
            rec = metadata['templates'][rid]
            raw = (root/rec['path']).read_bytes()
            if hashlib.sha256(raw).hexdigest() != rec['sha256']:
                raise ValueError('wheel appearance model hash mismatch')
            self.initial_templates[rid] = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_GRAYSCALE)

    def _payload(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([10, 100, 70], np.uint8), np.array([16, 255, 255], np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        count, labels, stats, centers = cv2.connectedComponentsWithStats(mask, 8)
        reference = self.map['start_zone']['center_m'] if self.payload is None else self.payload['xy_m']
        candidates = []
        for i in range(1, count):
            if stats[i, cv2.CC_STAT_AREA] < 400: continue
            yy, xx = np.where(labels == i)
            eig, vectors = np.linalg.eigh(np.cov(np.column_stack([xx, yy]).T))
            if eig[1] < 150 or eig[1]/max(eig[0], 1) < 10: continue
            point = pixel_to_world(centers[i], frame.shape, self.map['top_camera'])
            if math.dist(point, reference) > (.25 if self.payload is None else .10): continue
            axis = vectors[:, -1]
            angle = math.atan2(-axis[1], axis[0])
            candidates.append((point, angle, int(stats[i, cv2.CC_STAT_AREA])))
        if len(candidates) != 1:
            raise ValueError('orange payload missing/ambiguous')
        point, raw, area = candidates[0]
        if self.payload_origin_angle is None:
            self.payload_origin_angle = self.payload_angle = raw
        else:
            self.payload_angle += (raw-self.payload_angle+math.pi/2) % math.pi-math.pi/2
        self.payload = {'xy_m': list(point), 'relative_yaw_rad': self.payload_angle-self.payload_origin_angle,
                        'feature_area_px': area, 'feature_plane_height_m': .09}

    def _mask(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        return cv2.inRange(hsv, np.array([16, 80, 55], np.uint8), np.array([38, 255, 255], np.uint8))

    def observe(self, own_rgb, top_rgb):
        # Own RGB is validated and archived, but this initial classical motion
        # observer uses the common camera; it does not claim own-camera grasp QA.
        _decode_jpeg(own_rgb, 'own_rgb')
        frame = _decode_jpeg(top_rgb, 'shared_top_rgb')
        self._payload(frame)
        mask = self._mask(frame)
        h, w = mask.shape
        if not self.templates:
            scale = 2*(2.5-.09)*math.tan(math.radians(55)/2)/h
            point = self.payload['xy_m']
            center = np.array([(point[0]-.55)/scale+(w-1)/2, (-2-point[1])/scale+(h-1)/2])
            for rid, sign in (('r1', 1), ('r3', -1)):
                expected = center+np.array([0, sign*.30/scale])
                x, y = np.round(expected).astype(int)
                extent = 70
                if min(x-extent, y-extent) < 0 or x+extent >= w or y+extent >= h:
                    raise ValueError('start-side search outside fixed camera')
                crop = cv2.GaussianBlur(mask[y-extent:y+extent+1, x-extent:x+extent+1], (3,3), .7)
                best = None
                for angle in range(-15, 16):
                    matrix = cv2.getRotationMatrix2D((30,30), angle, 1.)
                    template = cv2.warpAffine(self.initial_templates[rid], matrix, (61,61))
                    score = cv2.matchTemplate(crop, template, cv2.TM_CCOEFF_NORMED)
                    _, peak, _, loc = cv2.minMaxLoc(score)
                    if best is None or peak > best[0]: best = (peak, loc)
                peak, (u,v) = best
                if peak < .55: raise ValueError('initial wheel appearance not recognized')
                px, py = x-extent+u+30, y-extent+v+30
                template = mask[py-30:py+31, px-30:px+31].copy()
                if template.shape != (61, 61):
                    raise ValueError('robot template outside fixed camera')
                self.templates[rid] = cv2.GaussianBlur(template, (3, 3), .7)
                self.centers[rid] = (float(px), float(py))
        observations = {}
        for rid in ROBOTS:
            px, py = self.centers[rid]
            x, y = int(round(px)), int(round(py))
            extent = 52
            if x-extent < 0 or y-extent < 0 or x+extent >= w or y+extent >= h:
                raise ValueError('robot outside fixed camera tracking margin')
            crop = cv2.GaussianBlur(mask[y-extent:y+extent+1, x-extent:x+extent+1], (3, 3), .7)
            best = None
            for angle in np.arange(self.angles[rid]-5, self.angles[rid]+5.01, .5):
                matrix = cv2.getRotationMatrix2D((30, 30), angle, 1.)
                template = cv2.warpAffine(self.templates[rid], matrix, (61, 61))
                score = cv2.matchTemplate(crop, template, cv2.TM_CCOEFF_NORMED)
                _, peak, _, loc = cv2.minMaxLoc(score)
                if best is None or peak > best[0]:
                    best = (peak, angle, loc)
            peak, angle, (u, v) = best
            if peak < .48:
                raise ValueError(f'{rid} wheel template lost ({peak:.3f})')
            center = (float(x-extent+u+30), float(y-extent+v+30))
            if math.dist(center, self.centers[rid]) > 17:
                raise ValueError('implausible visual tracking jump')
            self.centers[rid], self.angles[rid] = center, float(angle)
            point = pixel_to_world(center, frame.shape, self.map['top_camera'])
            observations[rid] = {'xy_m': list(point), 'relative_yaw_rad': math.radians(angle),
                                 'confidence': float(peak), 'center_uv': list(center),
                                 'feature_plane_height_m': .09}
        return observations


class PairNavigator:
    """A common geometric plan with independent own-command/RGB instances."""
    def __init__(self, data, robot_id, task_id='pair-navigation'):
        self.map, self.rid = validate_map(data), robot_id
        if robot_id not in ROBOTS: raise ValueError('invalid robot')
        self.vision = PairVision(data)
        self.sequence = 0
        self.phase = 'probe'
        self.probe_origin = None
        self.probe_steps = 0
        self.heading = None
        self.route = None
        self.reference = None
        self.segment = 1
        self.confirmations = 0
        self.plan_hash = None

    def decide(self, own_rgb, top_rgb):
        self.sequence += 1
        zero = {'kind': 'mecanum', 'forward': 0., 'left': 0., 'turn': 0., 'duration_s': .2}
        try:
            obs = self.vision.observe(own_rgb, top_rgb)
        except ValueError as error:
            return {'action': zero, 'status': 'vision_stop', 'error': str(error), 'ready': False, 'done': False}
        positions = {r: np.array(obs[r]['xy_m']) for r in ROBOTS}
        center = (positions['r1']+positions['r3'])/2
        if self.probe_origin is None:
            self.probe_origin = center.copy()
        action = dict(zero)
        status = self.phase
        if self.phase == 'probe':
            displacement = center-self.probe_origin
            if np.linalg.norm(displacement) >= .03:
                self.heading = math.atan2(displacement[1], displacement[0])
                self.phase = 'probe_settle'
            elif self.probe_steps < 20:
                action['forward'] = .045
                self.probe_steps += 1
            else:
                return {'action': zero, 'status': 'probe_failed', 'ready': False, 'done': False, 'observations': obs}
        elif self.phase == 'probe_settle':
            self.confirmations += 1
            if self.confirmations >= 3:
                self.offsets = {r: positions[r]-center for r in ROBOTS}
                self.anchor_yaws = {r: obs[r]['relative_yaw_rad'] for r in ROBOTS}
                self.anchor_payload = dict(self.vision.payload)
                self.reference = np.array([*center, 0.])
                goal = [*self.map['goal']['center_m'], math.radians(self.map['goal']['relative_yaw_deg'])]
                # Footprint orientation follows initial visual heading plus turn.
                start_pose = [*center, self.heading]
                goal_pose = [*goal[:2], self.heading+goal[2]]
                absolute = plan_route(start_pose, goal_pose, self.map)
                if absolute is None:
                    self.phase = 'no_route'
                else:
                    self.route = [[p[0], p[1], wrap(p[2]-self.heading)] for p in absolute]
                    self.plan_hash = digest({'map_sha256': digest(self.map), 'route': self.route,
                                             'offsets': {r: self.offsets[r].tolist() for r in ROBOTS}})
                    self.phase = 'track'
                self.confirmations = 0
        elif self.phase == 'track':
            payload = self.vision.payload
            payload_angle = payload['relative_yaw_rad']-self.anchor_payload['relative_yaw_rad']
            observed_turn = sum(obs[r]['relative_yaw_rad']-self.anchor_yaws[r] for r in ROBOTS)/2
            if abs(wrap(payload_angle-observed_turn)) > .12 or math.dist(payload['xy_m'], center) > .04:
                return {'action': zero, 'status': 'payload_decoupled', 'ready': False, 'done': False,
                        'observations': obs, 'payload': dict(payload)}
            target = np.array(self.route[self.segment])
            error_xy = target[:2]-self.reference[:2]
            error_yaw = wrap(target[2]-self.reference[2])
            delta = max(np.linalg.norm(error_xy)/.04, abs(error_yaw)/.055, .2)
            velocity, omega = error_xy/delta, error_yaw/delta
            formation_errors = {}
            for r in ROBOTS:
                expected = self.reference[:2]+rotate(self.offsets[r], self.reference[2])
                yaw = obs[r]['relative_yaw_rad']-self.anchor_yaws[r]
                formation_errors[r] = (float(np.linalg.norm(expected-positions[r])), abs(wrap(self.reference[2]-yaw)))
            if max(e[0] for e in formation_errors.values()) > .055 or max(e[1] for e in formation_errors.values()) > .30:
                return {'action': zero, 'status': 'formation_abort', 'ready': False, 'done': False,
                        'observations': obs, 'formation_errors': formation_errors}
            # Both local participants see the same RGB and hold the common
            # reference while either member catches up; commands are not poses.
            if max(e[0] for e in formation_errors.values()) > .020 or max(e[1] for e in formation_errors.values()) > .10:
                velocity, omega = np.zeros(2), 0.
            self.reference[:2] += velocity*.2
            self.reference[2] += omega*.2
            offset = rotate(self.offsets[self.rid], self.reference[2])
            desired = self.reference[:2]+offset
            observed_yaw = obs[self.rid]['relative_yaw_rad']-self.anchor_yaws[self.rid]
            world_velocity = velocity+omega*np.array([-offset[1], offset[0]]) + .9*(desired-positions[self.rid])
            local = rotate(world_velocity, -(self.heading+observed_yaw))
            angular = omega+.9*wrap(self.reference[2]-observed_yaw)
            # Nominal fixed gains, with actual progress corrected by RGB. These
            # are controller tuning constants, never reads from a live body.
            action.update(forward=float(np.clip(local[0]/1.57, -.05, .08)),
                          left=float(np.clip(local[1]/1.18, -.08, .08)),
                          turn=float(np.clip(angular/1.5, -.10, .10)))
            if np.linalg.norm(target[:2]-self.reference[:2]) < .002 and abs(wrap(target[2]-self.reference[2])) < .005:
                reached = (max(e[0] for e in formation_errors.values()) < .012
                           and max(e[1] for e in formation_errors.values()) < .045
                           and abs(wrap(payload_angle-target[2])) < .045)
                self.confirmations = self.confirmations+1 if reached else 0
                if self.confirmations >= 3:
                    self.confirmations = 0
                    if self.segment == len(self.route)-1:
                        self.phase = 'done'
                        action = zero
                    else:
                        self.segment += 1
        result = {'action': action, 'status': self.phase, 'ready': self.phase not in ('no_route',),
                  'done': self.phase == 'done', 'observations': obs, 'plan_hash': self.plan_hash,
                  'route': self.route, 'reference': None if self.reference is None else self.reference.tolist(),
                  'heading_rad': self.heading, 'segment': self.segment}
        result['payload'] = dict(self.vision.payload)
        return result


def authorize_pair(sync, reports, frames, index):
    """Renew GO only for two fresh reports accepting exactly the same route."""
    now = index*.2
    hashes = [reports[r].get('plan_hash') for r in ROBOTS]
    agree = hashes[0] == hashes[1]
    for rid in ROBOTS:
        sync.report(rid, plan_version=sync.plan_version, epoch=sync.epoch,
                    sequence=index+1, ready=bool(reports[rid]['ready'] and agree),
                    observed_at_s=now, received_at_s=now, frame_id=str(frames[rid]),
                    reason='' if agree else 'plan_mismatch')
    return sync.authorize(now_s=now)
