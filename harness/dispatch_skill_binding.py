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
    candidates = [b for b in extract_beams(jpeg, hue_upper=hue_upper) if not b['touches_border']
                  and b['length_px'] / b['width_px'] >= 3.5
                  and 65 <= b['length_px'] <= 180 and b['width_px'] <= 25]
    if len(candidates) != 1:
        raise ValueError('dispatch beam unresolved or ambiguous in RGB')
    return candidates[0]


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
                template=old[py-12:py+13,px-12:px+13]
                search=gray[py-30:py+31,px-30:px+31]
                if template.shape!=(25,25) or search.shape!=(61,61) or template.std()<15:
                    raise RuntimeError('dispatch box appearance unresolved in TOP RGB')
                matches=cv2.matchTemplate(search,template,cv2.TM_CCOEFF_NORMED)
                _,score,_,location=cv2.minMaxLoc(matches)
                # Preserve subpixel displacement; rounding every .2 s slice
                # otherwise accumulates a several-pixel lead over the cargo.
                offset=np.zeros(2)
                lx,ly=location
                for axis,(before,peak,after) in enumerate((
                    (matches[ly,max(0,lx-1)],matches[ly,lx],matches[ly,min(matches.shape[1]-1,lx+1)]),
                    (matches[max(0,ly-1),lx],matches[ly,lx],matches[min(matches.shape[0]-1,ly+1),lx]))):
                    denominator=float(before)-2*float(peak)+float(after)
                    if abs(denominator)>1e-8:offset[axis]=np.clip(.5*(float(before)-float(after))/denominator,-.5,.5)
                center=self.box_center+np.array(location)-18+offset
                cx,cy=np.rint(center).astype(int)
                local_cyan=cv2.inRange(hsv[cy-12:cy+13,cx-12:cx+13],np.array((80,70,25),np.uint8),np.array((102,255,255),np.uint8))
                reverse_template=gray[cy-12:cy+13,cx-12:cx+13]
                reverse_search=old[cy-30:cy+31,cx-30:cx+31]
                reverse=cv2.matchTemplate(reverse_search,reverse_template,cv2.TM_CCOEFF_NORMED)
                _,reverse_score,_,back=cv2.minMaxLoc(reverse)
                cycle_error=float(np.linalg.norm(np.array([cx-18+back[0],cy-18+back[1]])-[px,py]))
                if score<.85 or reverse_score<.85 or cycle_error>2 or np.linalg.norm(center-self.box_center)>20:
                    raise RuntimeError('dispatch box unresolved or ambiguous in TOP RGB')
                bounds=np.array([center-12,center+12])
                tracking={'method':'bidirectional prior RGB appearance; own attachment independently required','score':score,
                          'reverse_score':reverse_score,'cycle_error_px':cycle_error,
                          'cyan_pixels':int(np.count_nonzero(local_cyan))}
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
