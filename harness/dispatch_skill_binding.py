"""Bind a committed allocation to existing camera skills, never to simulator IDs.

The legacy pair names are model slots (bottom r1 / top r3), not live robots.
Static capabilities and image transforms are explicit and auditable. No fixture
poses are read here, and no failed model prediction is replaced by a raw LLM action.
"""
from __future__ import annotations
import copy
import hashlib
import math
import cv2
import numpy as np
from harness.dispatch_plan import validate_dispatch_plan, compile_programs
from harness.three_robot_plan import digest
from harness.camera_beam_features import extract_beams
from harness.camera_goal_transport import decode


def beam_feature(jpeg):
    candidates = [b for b in extract_beams(jpeg) if not b['touches_border']
                  and b['length_px'] / b['width_px'] >= 5
                  and 65 <= b['length_px'] <= 180 and b['width_px'] <= 25]
    if len(candidates) != 1:
        raise ValueError('dispatch beam unresolved or ambiguous in RGB')
    return candidates[0]


def canonical_pair_top(jpeg, reference):
    """Translate observed pixels to the saved beam-centred image convention.

    This is image preprocessing, not a changed camera or world reset. Retain
    all pixels, with black out-of-frame padding. Novel background stays novel
    for the unchanged learned support tests. No synthetic reference pixels.
    """
    frame, ref = decode(jpeg), decode(reference)
    if frame.shape != ref.shape:
        raise ValueError('pair reference and live TOP dimensions differ')
    current, anchor = beam_feature(jpeg), beam_feature(reference)
    h, w = frame.shape[:2]
    shift = (np.array(anchor['center']) - current['center']) * [w, h]
    transformed = cv2.warpAffine(frame, np.float32([[1,0,shift[0]],[0,1,shift[1]]]),
                                 (w,h), flags=cv2.INTER_LINEAR)
    data = cv2.imencode('.jpg', transformed, [cv2.IMWRITE_JPEG_QUALITY,95])[1].tobytes()
    return data, {'source_sha256':hashlib.sha256(jpeg).hexdigest(),
        'reference_sha256':hashlib.sha256(reference).hexdigest(),
        'translation_px':shift.tolist(),'observed_beam':current,
        'method':'RGB translation only; black padding; unchanged own RGB'}


class SkillBindings:
    def __init__(self, committed, static_map):
        if not committed or committed.get('plan_hash') != digest(committed.get('plan')):
            raise ValueError('exact committed plan required')
        self.committed = copy.deepcopy(committed)
        self.plan = validate_dispatch_plan(committed['plan'])
        self.static_map = copy.deepcopy(static_map)
        self.tasks = {t['object']:t for t in self.plan['tasks']}
        upper, lower = self.tasks['beam']['participants']
        self.pair = {'r1':lower, 'r3':upper}
        self.solo = self.tasks['box']['participants'][0]
        self.programs = compile_programs(self.plan, static_map)
        for rid, rows in self.programs.items():
            for row in rows:
                row['execution_adapter'] = ('existing_rgb_pair_v1' if row['object']=='beam'
                                            else 'existing_visual_box_v1')
                row['model_slot'] = next((slot for slot,r in self.pair.items() if r==rid),None)
        self.finished = set()
        self.locks = {}
        self.revoked = False

    def authorize(self, committed):
        if self.revoked or committed != self.committed:
            raise RuntimeError('skill plan revoked or replaced')

    def permission(self, obj, stage):
        if self.revoked:return False
        task = self.tasks[obj]
        if stage != 'APPROACH' and any(dep not in self.finished for dep in task['after']):
            return False
        if stage == 'TRANSIT':
            resources = [self.static_map['routes'][task['route']]['resource'], 'dispatch_apron']
            if any(self.locks.get(r,task['id']) != task['id'] for r in resources):return False
            for r in resources:self.locks[r] = task['id']
        return True

    def finish(self,obj):
        task_id = self.tasks[obj]['id']
        self.finished.add(task_id)
        self.locks = {r:t for r,t in self.locks.items() if t!=task_id}

    def capabilities(self):
        # Conservative authored envelope of the demonstrated parallel formation:
        # 0.65 m centre separation plus 0.24 m chassis envelope and 0.04 m margin.
        # A capability bound, not a runtime pose or claimed passage measurement.
        width = .65+.24+.04
        narrow = []
        if any(o['id']=='service_island' for o in self.static_map['obstacles']):
            for name, route in self.static_map['routes'].items():
                if route['declared_min_width_m'] < width:narrow.append(name)
        return {'pair_model_slots':self.pair, 'solo_robot':self.solo,
            'parallel_pair_envelope_m':width,'unsupported_parallel_pair_routes':narrow,
            'pair_rotation_skill_available':False,
            'route_execution':'RGB waypoint translation; loaded lateral motion experimental',
            'model_support':'unchanged learned support thresholds; fail closed on novelty'}

    def check_route(self):
        route = self.tasks['beam']['route']
        if route in self.capabilities()['unsupported_parallel_pair_routes']:
            raise RuntimeError('PAIR_ROUTE_TOO_NARROW: agreed '+route+
                ' route needs a rotation/regrasp skill; parallel formation is unsupported')


def pixel_from_map(xy, static_map, shape, *, height=0.):
    """Only authored goals are projected. No dynamic world positions accepted."""
    cam=static_map['top_camera']; h,w=shape[:2]
    if cam['quaternion_wxyz'] != [1,0,0,0]:raise ValueError('unsupported static camera')
    cx,cy,cz=cam['position_m']
    scale=h/(2*(cz-height)*math.tan(math.radians(cam['fov_y_deg'])/2))
    return np.array([(w-1)/2+(xy[0]-cx)*scale,(h-1)/2-(xy[1]-cy)*scale])


class ImageRoute:
    """Plan-selected authored waypoints with current cargo position from RGB."""
    def __init__(self, bindings, obj):
        self.map=bindings.static_map;self.obj=obj
        self.task=bindings.tasks[obj];self.dock=bindings.plan['dock']
        self.points=None;self.index=0;self.confirmations=0

    def observe(self,jpeg):
        frame=decode(jpeg);h,w=frame.shape[:2]
        if self.obj=='beam':
            feature=beam_feature(jpeg)
            center=np.array(feature['center'])*[w,h]
            bounds=np.array(feature['corners4'])*[w,h]
        else:
            hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
            mask=cv2.inRange(hsv,np.array((80,70,35),np.uint8),np.array((105,255,255),np.uint8))
            n,_,stats,centers=cv2.connectedComponentsWithStats(mask)
            choices=[i for i in range(1,n) if 25 <= stats[i,4] <= 600
                     and max(stats[i,2:4]) < 40]
            if len(choices)!=1:raise RuntimeError('dispatch box unresolved or ambiguous in TOP RGB')
            i=choices[0];center=centers[i];x,y,bw,bh=stats[i,:4]
            bounds=np.array([[x,y],[x+bw,y+bh]])
        if self.points is None:
            # The open arena permits the original parallel formation. Avoid
            # shifting it into a wall merely to hit a narrow symbolic gate.
            gate=self.map['regions'][self.task['route']+'_gate']['center_m'][:]
            margin=.48 if self.obj=='beam' else .18
            ymin,ymax=self.map['bounds_m'][2:]
            gate[1]=max(ymin+margin,min(ymax-margin,gate[1]))
            destination=self.map['docks'][self.dock]['slots'][self.obj]['center_m']
            gate_px=pixel_from_map(gate,self.map,frame.shape)
            east_px=pixel_from_map([1.12,gate[1]],self.map,frame.shape)
            goal_px=pixel_from_map(destination,self.map,frame.shape)
            self.points=[np.array([center[0],gate_px[1]]),east_px,
                         np.array([east_px[0],goal_px[1]]),goal_px]
        error=self.points[self.index]-center
        tolerance=4 if self.index==len(self.points)-1 else 6
        ready=float(np.linalg.norm(error)) <= tolerance
        self.confirmations=self.confirmations+1 if ready else 0
        done=self.index==len(self.points)-1 and self.confirmations>=2
        evidence={'source':'TOP RGB + authored map', 'cargo_center_px':center.tolist(),
                  'cargo_bounds_px':bounds.tolist(),'waypoint_index':self.index,
                  'waypoints_px':[p.tolist() for p in self.points],
                  'error_px':error.tolist(),'ready':ready,'done':done}
        if ready and self.confirmations>=2 and not done:
            self.index+=1;self.confirmations=0
        control=np.clip(error*.002,-.08,.08)
        if ready:control[:]=0
        else:
            for i in range(2):
                if abs(error[i])>tolerance:control[i]=math.copysign(max(.025,abs(control[i])),control[i])
                else:control[i]=0
        action={'kind':'mecanum','forward':float(control[0]),'left':float(-control[1]),
                'turn':0.,'duration_s':.2}
        return action,evidence


class PairCoarsePixels:
    """Role-bound RGB wheel selection; rejects painted floor/beam components.

    The old HSV lane mask confused new yellow paint and brighter beam pixels
    with wheels. Identity comes from the robot's own isolated probe. Heading
    keeps the original four-corner gate; model support checks are untouched.
    """
    def __init__(self, identity, bindings, reference):
        self.centers={}
        self.reference=reference
        for slot,rid in bindings.pair.items():
            claim=identity[rid]['claim']
            if not claim.get('valid') or not claim.get('center'):
                raise ValueError('valid own motion identity required for '+rid)
            self.centers[slot]=np.array(claim['center'])*[960,720]

    def decide(self, raw_top, slot):
        from harness.camera_goal_transport import lane_features, wheel_heading
        frame=decode(raw_top);h,w=frame.shape[:2]
        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        yellow=cv2.inRange(hsv,np.array((20,70,50),np.uint8),np.array((40,255,255),np.uint8))
        n,labels,stats,_=cv2.connectedComponentsWithStats(yellow)
        clean=np.zeros_like(yellow)
        for i in range(1,n):
            if stats[i,4]<=300:clean[labels==i]=255
        cx,cy=self.centers[slot]
        yy,xx=np.indices(clean.shape)
        clean[(abs(xx-cx)>55)|(abs(yy-cy)>48)]=0
        ys,xs=np.nonzero(clean)
        heading=wheel_heading(clean,pixel_tolerance=1.)
        if heading is None:
            return dict(ok=False,ready=False,forward=0.,left=0.,turn=0.,reason='own_wheel_heading_unresolved',pixel_count=len(xs))
        center=np.array([xs.mean(),ys.mean()]);self.centers[slot]=center
        beam=beam_feature(raw_top);ref=lane_features(self.reference,slot)
        gap=(np.array(beam['center'])-center/[w,h])-np.array([ref['beam_x']-ref['robot_x'],ref['beam_y']-ref['robot_y']])
        angle=heading['angle_deg'];angle_ready=abs(angle)<=1.5
        lateral_ready=abs(gap[1])<=.003
        ready=gap[0]<=.065 and angle_ready and lateral_ready
        return dict(ok=True,ready=bool(ready),
            forward=min(.12,max(.03,float(gap[0]))) if angle_ready and lateral_ready and not ready else 0.,
            left=(-math.copysign(min(.05,max(.01,abs(float(gap[1])))),float(gap[1])) if angle_ready and not lateral_ready else 0.),
            turn=0. if angle_ready else math.copysign(min(.10,max(.01,.5*abs(math.radians(angle)))),angle),
            reason='RGB role-relative coarse approach',wheel_center_px=center.tolist(),image_error=gap.tolist(),heading=heading)
