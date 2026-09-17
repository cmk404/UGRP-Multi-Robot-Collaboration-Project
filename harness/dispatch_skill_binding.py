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


def beam_feature(jpeg, *, hue_upper=24):
    # The original mask remains first, preserving the learned image convention.
    # Yellow floor paint can merge with the carried beam. Its lower saturation
    # permits a second segmentation, still subject to every shaft shape gate.
    def candidates(saturation):
        return [b for b in extract_beams(jpeg, hue_upper=hue_upper,min_saturation=saturation)
                      if not b['touches_border'] and b['length_px'] / b['width_px'] >= 3.5
                      and 65 <= b['length_px'] <= 180 and b['width_px'] <= 25]
    initial=candidates(105)
    if len(initial)==1:return initial[0]
    # A stable shaft must survive several thresholds, not one lucky cut through
    # a painted floor region. Prefer the widest supported silhouette to retain
    # the same image convention while its low-saturation surroundings vanish.
    levels=[candidates(s) for s in range(130,191,10)]
    stable=[]
    for i,level in enumerate(levels):
        for b in level:
            support=sum(any(math.dist(b['center'],c['center'])*960<4
                and abs(c['length_px']/b['length_px']-1)<.15 for c in other)
                for other in levels[i:])
            if support>=3:stable.append(b)
    if stable:
        first=stable[0]
        if all(math.dist(first['center'],b['center'])*960<6 for b in stable):return first
    raise ValueError('dispatch beam unresolved or ambiguous in RGB')


class BeamContinuity:
    """Reject switching to a different colored object while carrying a beam."""
    def __init__(self):self.previous=None
    def observe(self,feature):
        if self.previous:
            a,b=self.previous,feature
            movement=np.linalg.norm((np.array(a['center'])-b['center'])*b['image_size'])
            def angle(f):
                v=(np.array(f['endpoints'][1])-f['endpoints'][0])*f['image_size']
                return math.atan2(v[1],v[0])
            turn=abs((angle(b)-angle(a)+math.pi/2)%math.pi-math.pi/2)
            if movement>24 or turn>math.radians(15):
                raise ValueError('carried beam visual continuity lost')
        self.previous=copy.deepcopy(feature)
        return feature


def canonical_pair_top(jpeg, reference, *, translation_px=None, hue_upper=24):
    """Translate observed pixels to the saved beam-centred image convention.

    This is image preprocessing, not a changed camera or world reset. Retain
    all pixels, with black out-of-frame padding. Novel background stays novel
    for the unchanged learned support tests. No synthetic reference pixels.
    """
    frame, ref = decode(jpeg), decode(reference)
    if frame.shape != ref.shape:
        raise ValueError('pair reference and live TOP dimensions differ')
    current, anchor = beam_feature(jpeg,hue_upper=hue_upper), beam_feature(reference)
    h, w = frame.shape[:2]
    shift = (np.array(anchor['center']) - current['center']) * [w, h] if translation_px is None else np.asarray(translation_px,dtype=float)
    if shift.shape!=(2,) or not np.isfinite(shift).all():raise ValueError('invalid image translation')
    transformed = cv2.warpAffine(frame, np.float32([[1,0,shift[0]],[0,1,shift[1]]]),
                                 (w,h), flags=cv2.INTER_LINEAR)
    data = cv2.imencode('.jpg', transformed, [cv2.IMWRITE_JPEG_QUALITY,95])[1].tobytes()
    return data, {'source_sha256':hashlib.sha256(jpeg).hexdigest(),
        'reference_sha256':hashlib.sha256(reference).hexdigest(),
        'translation_px':shift.tolist(),'observed_beam':current,
        'fixed_from_prior_rgb':translation_px is not None,'hue_upper':hue_upper,
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
        if stage in ('GRASP','TRANSIT'):
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
        self.box_center=None;self.box_delta=np.zeros(2);self.box_previous=None

    def observe(self,jpeg):
        frame=decode(jpeg);h,w=frame.shape[:2]
        if self.obj=='beam':
            feature=beam_feature(jpeg,hue_upper=35)
            center=np.array(feature['center'])*[w,h]
            bounds=np.array(feature['corners4'])*[w,h]
        else:
            hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
            mask=cv2.inRange(hsv,np.array((80,125 if self.box_center is None else 70,35),np.uint8),np.array((102,255,255),np.uint8))
            if self.box_center is None:
                mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
            n,_,stats,centers=cv2.connectedComponentsWithStats(mask)
            choices=[i for i in range(1,n) if 25 <= stats[i,4] <= 600
                     and max(stats[i,2:4]) < 40]
            if self.box_center is not None:
                predicted=self.box_center+np.clip(self.box_delta,-15,15)
                choices=sorted((i for i in choices if np.linalg.norm(centers[i]-predicted)<=25),
                               key=lambda i:np.linalg.norm(centers[i]-predicted))
                if len(choices)>1 and np.linalg.norm(centers[choices[1]]-predicted)-np.linalg.norm(centers[choices[0]]-predicted)>5:
                    choices=choices[:1]
            tracking={'method':'cyan component'}
            if len(choices)==1:
                i=choices[0];center=centers[i];x,y,bw,bh=stats[i,:4]
                bounds=np.array([[x,y],[x+bw,y+bh]])
            elif self.box_previous is not None:
                # Follow actual prior RGB appearance when same-colour floor
                # merges the component. Own-camera attachment must independently
                # pass before this navigator is called. No pose/state fallback.
                px,py=np.rint(self.box_center).astype(int)
                gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
                old=cv2.cvtColor(self.box_previous,cv2.COLOR_BGR2GRAY)
                feature_mask=np.zeros_like(old)
                feature_mask[max(0,py-18):py+19,max(0,px-18):px+19]=255
                feature_mask[old>90]=0
                points=cv2.goodFeaturesToTrack(old,30,.01,3,mask=feature_mask)
                if points is None or len(points)<3:
                    raise RuntimeError('dispatch box appearance unresolved in TOP RGB')
                forward,ok,errors=cv2.calcOpticalFlowPyrLK(old,gray,points,None,winSize=(15,15),maxLevel=2)
                backward,back_ok,_=cv2.calcOpticalFlowPyrLK(gray,old,forward,None,winSize=(15,15),maxLevel=2)
                cycle=np.linalg.norm(backward-points,axis=2).ravel()
                delta=(forward-points).reshape(-1,2)
                valid=(ok.ravel()>0)&(back_ok.ravel()>0)&(cycle<1)&(np.linalg.norm(delta,axis=1)<20)&(errors.ravel()<30)
                delta=delta[valid];cycle=cycle[valid]
                if len(delta)<3:raise RuntimeError('dispatch box unresolved or ambiguous in TOP RGB')
                # A painted edge may supply stationary corners; require a
                # majority of independently tracked corners to share motion.
                groups=np.linalg.norm(delta[:,None,:]-delta[None,:,:],axis=2)<1.5
                inliers=groups[np.argmax(groups.sum(axis=1))]
                if inliers.sum()<max(3,.6*len(delta)):
                    raise RuntimeError('dispatch box motion ambiguous in TOP RGB')
                motion=np.median(delta[inliers],axis=0)
                center=self.box_center+motion
                bounds=np.array([center-12,center+12])
                tracking={'method':'bidirectional RGB feature motion; own attachment independently required',
                          'feature_count':int(len(delta)),'consistent_features':int(inliers.sum()),
                          'max_cycle_error_px':float(cycle[inliers].max()),'motion_px':motion.tolist()}
            else:raise RuntimeError('dispatch box unresolved or ambiguous in TOP RGB')
            self.box_delta=np.zeros(2) if self.box_center is None else center-self.box_center
            self.box_center=center.copy();self.box_previous=frame.copy()

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
            if self.obj=='box':
                # Cross the shared apron between docks, then approach the box
                # slot from the east. A straight west entry crosses the beam
                # slot and its parked carriers after the beam job releases.
                apron_y=self.map['regions']['dispatch_apron']['center_m'][1]
                east_clear=self.map['bounds_m'][1]-.12
                goal_px=pixel_from_map([destination[0]+.04,destination[1]],self.map,frame.shape)
                self.points=[self.points[0],east_px,
                    pixel_from_map([1.12,apron_y],self.map,frame.shape),
                    pixel_from_map([east_clear,apron_y],self.map,frame.shape),
                    pixel_from_map([east_clear,destination[1]],self.map,frame.shape),goal_px]
        error=self.points[self.index]-center
        tolerance=4 if self.index==len(self.points)-1 else 6
        ready=float(np.max(np.abs(error))) <= tolerance
        self.confirmations=self.confirmations+1 if ready else 0
        done=self.index==len(self.points)-1 and self.confirmations>=2
        evidence={'source':'TOP RGB + authored map', 'cargo_center_px':center.tolist(),
                  'cargo_bounds_px':bounds.tolist(),'waypoint_index':self.index,
                  'waypoints_px':[p.tolist() for p in self.points],
                  'error_px':error.tolist(),'ready':ready,'done':done}
        if self.obj=='box':evidence['tracking']=tracking
        if ready and self.confirmations>=2 and not done:
            self.index+=1;self.confirmations=0
        control=np.clip(error*.002,-.08,.08)
        control[0]=max(-.05,control[0])
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
        heading=wheel_heading(clean,pixel_tolerance=2.)
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
