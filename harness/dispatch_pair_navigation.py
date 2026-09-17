"""RGB-only paired load navigation on an authored static map.

Adapted from pair-loaded-navigation 85fb046 (RGB wheel tracking and SE2 control).
Dispatch uses its own map adapter and current shaft segmentation.
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


def payload_coupled(payload, robot_center, robot_turn, anchor_turn):
    """Occluded shaft pixels identify a line and a center interval, not a point."""
    angle_error = abs(wrap(payload['relative_yaw_rad']-anchor_turn-robot_turn))
    axis = np.array(payload['axis_xy'])
    normal = np.array([-axis[1], axis[0]])
    lateral = abs(float(np.dot(np.array(payload['xy_m'])-robot_center, normal)))
    along = float(np.dot(robot_center, axis))
    lower, upper = payload['center_projection_interval_m']
    return angle_error <= .12 and lateral <= .04 and lower-.04 <= along <= upper+.04


def validate_map(data):
    if data.get('schema')!='ugrp.dispatch_pair_navigation.v1' or data['top_camera']!=CAMERA:
        raise ValueError('invalid dispatch navigation map or changed camera')
    f=data['footprint']
    if f != {'half_forward_m':.20,'half_lateral_m':.445,'margin_m':.025}:
        raise ValueError('loaded footprint contract changed')
    if data['grid_m']!=.06:raise ValueError('unsupported search grid')
    values=list(data['bounds_m'])+list(data['goal']['center_m'])
    for o in data['obstacles']:
        if len(o['center_m'])!=2 or len(o['half_extents_m'])!=2 or min(o['half_extents_m'])<=0:
            raise ValueError('invalid obstacle')
        values+=list(o['center_m'])+list(o['half_extents_m'])
    if any(isinstance(v,bool) or not math.isfinite(v) for v in values):
        raise ValueError('finite map coordinates required')
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


def track_wheel_motion(previous,current,center,angle,template):
    """Bidirectional RGB feature motion of the observed rigid wheel envelope.

    Paint can merge with yellow wheels; it cannot define this motion estimate.
    Features are seeded on the prior observed body silhouette. An inlier fit
    must span multiple wheel quadrants and agree in forward/backward images.
    """
    old=cv2.cvtColor(previous,cv2.COLOR_BGR2GRAY);new=cv2.cvtColor(current,cv2.COLOR_BGR2GRAY)
    matrix=cv2.getRotationMatrix2D((30,30),angle,1.)
    matrix[:,2]+=np.asarray(center)-30
    mask=cv2.warpAffine((template>20).astype(np.uint8)*255,matrix,(old.shape[1],old.shape[0]))
    mask=cv2.dilate(mask,np.ones((5,5),np.uint8))
    points=cv2.goodFeaturesToTrack(old,80,.01,3,mask=mask)
    if points is None or len(points)<8:raise ValueError('wheel RGB features missing')
    forward,ok,errors=cv2.calcOpticalFlowPyrLK(old,new,points,None,winSize=(15,15),maxLevel=2)
    backward,back_ok,_=cv2.calcOpticalFlowPyrLK(new,old,forward,None,winSize=(15,15),maxLevel=2)
    cycle=np.linalg.norm(backward-points,axis=2).ravel()
    good=(ok.ravel()>0)&(back_ok.ravel()>0)&(cycle<.7)&(errors.ravel()<30)
    a=points.reshape(-1,2)[good];b=forward.reshape(-1,2)[good]
    if len(a)<8:raise ValueError('wheel bidirectional RGB motion unresolved')
    transform,inliers=cv2.estimateAffinePartial2D(a,b,method=cv2.RANSAC,ransacReprojThreshold=1.,maxIters=1000,confidence=.99)
    if transform is None:raise ValueError('wheel rigid RGB motion unresolved')
    keep=inliers.ravel().astype(bool);fraction=float(keep.mean())
    if keep.sum()<8 or fraction<.65 or min(np.ptp(a[keep],axis=0))<25:
        raise ValueError('wheel rigid feature coverage lost')
    scale=float(np.hypot(transform[0,0],transform[0,1]))
    turn=math.degrees(math.atan2(transform[0,1],transform[0,0]))
    next_center=transform[:,:2]@np.asarray(center)+transform[:,2]
    if not .97<=scale<=1.03 or abs(turn)>5 or np.linalg.norm(next_center-center)>17:
        raise ValueError('implausible wheel RGB motion')
    return tuple(map(float,next_center)),angle+turn,{'method':'bidirectional RGB rigid feature motion',
        'inlier_fraction':fraction,'features':int(len(a)),'inliers':int(keep.sum()),
        'scale':scale,'turn_deg':turn,'max_cycle_error_px':float(cycle[good][keep].max())}


def plan_placement_route(start,goal,data):
    """Choose the nearest collision-free placement inside the authored slot.

    The allowed offset leaves a 0.45m shaft and clearance inside the slot.
    Final image-based slot inclusion and independent release QA still apply.
    """
    limits=data['goal'].get('placement_tolerance_m',[0.,0.])
    candidates=[(x,y) for x in (0.,-.02,.02,-.04,.04,-.06,.06)
        for y in (0.,-.015,.015) if abs(x)<=limits[0]+1e-9 and abs(y)<=limits[1]+1e-9]
    for dx,dy in sorted(candidates,key=lambda d:math.hypot(*d)):
        target=[goal[0]+dx,goal[1]+dy,goal[2]]
        route=plan_route(start,target,data)
        if route is not None:return route
    return None


def reanchor_wheels(mask,template,center,angle):
    """Bound optical-flow drift using independently visible current wheel pixels.

    Rolling tread and arm motion are not chassis displacement. Only a supported
    four-corner appearance may correct the estimate; paint/occlusion keeps the
    bidirectional motion estimate, never a command-integrated fallback.
    """
    yy,xx=np.indices(template.shape)
    wheels=template.copy();wheels[(abs(xx-30)<10)|(abs(yy-30)<10)]=0
    x,y=np.rint(center).astype(int);extent=54
    if min(x-extent,y-extent)<0 or x+extent>=mask.shape[1] or y+extent>=mask.shape[0]:
        return center,angle,{'accepted':False,'reason':'search outside camera'}
    crop=cv2.GaussianBlur(mask[y-extent:y+extent+1,x-extent:x+extent+1],(3,3),.7)
    padded=np.pad(wheels,12);best=None
    for candidate in np.arange(angle-4,angle+4.01,1.):
        matrix=cv2.getRotationMatrix2D((42,42),float(candidate),1.)
        target=cv2.warpAffine(padded,matrix,(85,85))
        scores=cv2.matchTemplate(crop,target,cv2.TM_CCOEFF_NORMED)
        _,score,_,location=cv2.minMaxLoc(scores)
        if best is None or score>best[0]:best=(score,location,float(candidate),target)
    score,(u,v),candidate,target=best
    if score<.55:return center,angle,{'accepted':False,'score':float(score)}
    patch=crop[v:v+85,u:u+85]
    inverse=cv2.getRotationMatrix2D((42,42),-candidate,1.)
    overlap=cv2.warpAffine(((patch>50)&(cv2.dilate(target,np.ones((3,3),np.uint8))>50)).astype(np.uint8),inverse,(85,85))
    counts=[int(overlap[sy,sx].sum()) for sy in (slice(0,32),slice(52,85)) for sx in (slice(0,32),slice(52,85))]
    if min(counts)<8:return center,angle,{'accepted':False,'score':float(score),'corner_pixels':counts}
    observed=np.array([x-extent+u+42,y-extent+v+42],dtype=float)
    delta=observed-np.asarray(center)
    correction=delta*min(.25,1./max(np.linalg.norm(delta),1e-9))
    return tuple(np.asarray(center)+correction),angle+.15*(candidate-angle),{
        'accepted':True,'score':float(score),'corner_pixels':counts,'observed_center_px':observed.tolist(),
        'correction_px':correction.tolist(),'heading_correction_deg':.15*(candidate-angle)}


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
        from harness.dispatch_beam_tracker import CarriedBeamTracker
        self.shaft_tracker=CarriedBeamTracker()
        self.payload_origin_angle = None
        self.payload_angle = None
        root = Path(__file__).parent/'assets'/'dispatch_pair_navigation'
        metadata = json.loads((root/'manifest.json').read_text())
        self.payload_length_m = float(metadata['payload_length_m'])
        self.initial_templates = {}
        for rid in ROBOTS:
            rec = metadata['templates'][rid]
            raw = (root/rec['path']).read_bytes()
            if hashlib.sha256(raw).hexdigest() != rec['sha256']:
                raise ValueError('wheel appearance model hash mismatch')
            self.initial_templates[rid] = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_GRAYSCALE)

    def _payload(self, frame, raw):
        from harness.dispatch_skill_binding import beam_feature
        b=self.shaft_tracker.observe(raw)
        endpoints=np.array(b['endpoints'])*b['image_size']
        points=np.array([pixel_to_world(p,frame.shape,self.map['top_camera']) for p in endpoints])
        axis=points[1]-points[0];axis/=np.linalg.norm(axis)
        point=np.mean(points,axis=0);angle=math.atan2(axis[1],axis[0])
        projections=points@axis
        if self.payload_origin_angle is None:self.payload_origin_angle=self.payload_angle=angle
        else:self.payload_angle+=(angle-self.payload_angle+math.pi/2)%math.pi-math.pi/2
        self.payload={'xy_m':point.tolist(),'relative_yaw_rad':self.payload_angle-self.payload_origin_angle,
            'feature_area_px':b['area_px'],'feature_plane_height_m':.09,
            'xy_is_visible_fragment_centroid':True,'axis_xy':axis.tolist(),
            'center_projection_interval_m':[float(projections.max()-.225),float(projections.min()+.225)]}

    def _mask(self,image):
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(hsv,np.array([16,80,55],np.uint8),np.array([40,255,255],np.uint8))
        n,labels,stats,_=cv2.connectedComponentsWithStats(mask)
        for i in range(1,n):
            if stats[i,4]>300:mask[labels==i]=0
        return mask

    def observe(self, own_rgb, top_rgb):
        # Own RGB is validated and archived, but this initial classical motion
        # observer uses the common camera; it does not claim own-camera grasp QA.
        _decode_jpeg(own_rgb, 'own_rgb')
        frame = _decode_jpeg(top_rgb, 'shared_top_rgb')
        if frame.shape != (720, 960, 3):
            raise ValueError('wheel appearance requires the calibrated 960x720 top camera')
        self._payload(frame,top_rgb)
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
                if template.shape != (61, 61) or np.count_nonzero(template)<80:
                    raise ValueError('robot template outside fixed camera')
                self.templates[rid] = cv2.GaussianBlur(template, (3, 3), .7)
                self.centers[rid] = (float(px), float(py))
        observations = {}
        previous=getattr(self,'previous_frame',None)
        updated={}
        for rid in ROBOTS:
            center,angle=self.centers[rid],self.angles[rid]
            tracking={'method':'initial RGB wheel template'}
            if previous is not None:
                center,angle,tracking=track_wheel_motion(previous,frame,center,angle,self.templates[rid])
                center,angle,tracking['current_wheel_reanchor']=reanchor_wheels(mask,self.templates[rid],center,angle)
            point=pixel_to_world(center,frame.shape,self.map['top_camera'])
            observations[rid]={'xy_m':list(point),'relative_yaw_rad':math.radians(angle),
                'confidence':tracking.get('inlier_fraction',1.),'center_uv':list(center),
                'feature_plane_height_m':.09,'tracking':tracking}
            updated[rid]=(center,angle)
        for rid,(center,angle) in updated.items():self.centers[rid],self.angles[rid]=center,angle
        self.previous_frame=frame.copy()

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
            # The fixed-start appearance recognizer only accepts headings near
            # its canonical orientation. Check that entire bounded probe envelope
            # before issuing the small movement that resolves heading direction.
            for angle in np.linspace(-math.pi/12, math.pi/12, 7):
                target = center+rotate((.065, 0.), angle)
                if not swept_clear((*center, angle), (*target, angle), self.map):
                    return {'action': zero, 'status': 'no_route', 'ready': False, 'done': False,
                            'observations': obs, 'reason': 'probe_envelope_blocked'}
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
                goal_xy=np.array(self.map['goal']['center_m'])-(np.array(self.anchor_payload['xy_m'])-center)
                goal = [*goal_xy, math.radians(self.map['goal']['relative_yaw_deg'])]
                # Footprint orientation follows initial visual heading plus turn.
                start_pose = [*center, self.heading]
                goal_pose = [*goal[:2], self.heading+goal[2]]
                absolute = plan_placement_route(start_pose, goal_pose, self.map)
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
            if not payload_coupled(payload, center, observed_turn, self.anchor_payload['relative_yaw_rad']):
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
