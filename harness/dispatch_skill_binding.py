"""Bind a committed allocation to existing camera skills, never to simulator IDs.

The legacy pair names are model slots (bottom r1 / top r3), not live robots.
Static capabilities and image transforms are explicit and auditable. No fixture
poses are read here, and no failed model prediction is replaced by a raw LLM action.
"""
from __future__ import annotations
import base64
import binascii
import copy
from functools import lru_cache
import hashlib
import json
import math
import cv2
import numpy as np
from harness.dispatch_plan import validate_dispatch_plan, compile_programs
from harness.three_robot_plan import digest
from harness.camera_beam_features import extract_beams
from harness.camera_goal_transport import decode


# The separate opt-in may bridge one short TOP occlusion, never make an
# indefinite carrier-only delivery. Both saved failure boundaries retain >20
# independent carrier corners after three proxy frames; the remaining authored
# northbound waypoint is about 70-80 image pixels away. These fixed ceilings
# cover that corridor once, with a small image-motion margin, and cannot be
# refreshed by detecting new robot corners. Twelve is a total of the original
# three proxy observations plus at most nine new observations, not twelve extra.
_CARRIER_RELINK_MAX_OCCLUDED_FRAMES=12
_CARRIER_RELINK_MAX_ELAPSED_S=3.2
_CARRIER_RELINK_MAX_MOTION_PX=105.


def beam_feature(jpeg, *, hue_upper=24):
    # The original mask remains first, preserving the learned image convention.
    # Yellow floor paint can merge with the carried beam. Its lower saturation
    # permits a second segmentation, still subject to every shaft shape gate.
    def candidates(saturation):
        beams=extract_beams(jpeg,hue_upper=hue_upper,min_saturation=saturation)
        refined=None
        selected=[]
        for b in beams:
            # A 0.45m beam projects to at most about 130 pixels at the fixed
            # nominal carry plane. A longer thin component includes a gripper
            # or wheel bridge; fit its stable-width core instead of accepting
            # an impossible visible shaft. Ordinary silhouettes stay unchanged.
            if hue_upper==35 and 130<b['length_px']<=180 and b['width_px']<=25:
                if refined is None:refined=extract_beams(jpeg,robust_shaft=True,hue_upper=hue_upper,min_saturation=saturation)
                near=[r for r in refined if np.linalg.norm((np.array(r['center'])-b['center'])*b['image_size'])<.4*b['length_px']
                      and 65<=r['length_px']<=130 and r['width_px']<=25]
                if len(near)!=1:continue
                b=near[0]
            selected.append(b)
        return [b for b in selected
                      if not b['touches_border'] and b['length_px'] / b['width_px'] >= 3.5
                      and 65 <= b['length_px'] <= 180 and b['width_px'] <= 25]
    initial=candidates(105)
    if len(initial)==1:return initial[0]
    # A stable shaft must survive several thresholds, not one lucky cut through
    # a painted floor region. Prefer the widest supported silhouette to retain
    # the same image convention while its low-saturation surroundings vanish.
    def same_shaft(a,b):
        size=np.array(a['image_size'])
        axis=(np.array(a['endpoints'][1])-a['endpoints'][0])*size
        axis=axis/np.linalg.norm(axis)
        other=(np.array(b['endpoints'][1])-b['endpoints'][0])*size
        other=other/np.linalg.norm(other)
        delta=(np.array(b['center'])-a['center'])*size
        return (abs(float(axis@other))>math.cos(math.radians(5))
            and abs(float(delta@np.array([-axis[1],axis[0]])))<4
            and abs(float(delta@axis))<.2*min(a['length_px'],b['length_px']))
    levels=[candidates(s) for s in range(130,191,5)]
    stable=[]
    for i,level in enumerate(levels):
        for b in level:
            support=sum(any(same_shaft(b,c)
                and abs(c['length_px']/b['length_px']-1)<.15 for c in other)
                for other in levels[i:])
            if support>=3:stable.append(b)
    if stable:
        first=stable[0]
        if all(same_shaft(first,b) for b in stable):return first
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


def released_beam_envelope(jpeg,prior,static_map):
    """Reacquire the full released silhouette, not a trimmed tracking core.

    The unchanged authored beam is .45m long, .05m wide and .04m high. Its
    nominal top-face plane supplies scale, never a measured object height.
    Missing visible length is uncertainty at either end, not invented pixels.
    """
    cam=static_map['top_camera'];w,h=prior['image_size']
    scale=h/(2*(cam['position_m'][2]-.04)*math.tan(math.radians(cam['fov_y_deg'])/2))
    length,width=.45*scale,.05*scale
    axis=np.diff(np.array(prior['endpoints']),axis=0)[0]*[w,h];axis/=np.linalg.norm(axis)
    origin=np.array(prior['center'])*[w,h];candidates=[]
    for saturation in range(105,191,5):
        for b in extract_beams(jpeg,hue_upper=35,min_saturation=saturation,include_contour=True):
            v=np.diff(np.array(b['endpoints']),axis=0)[0]*[w,h];v/=np.linalg.norm(v)
            if (not b['touches_border'] and .9*length<=b['length_px']<=1.1*length
                    and b['width_px']<=2*width and b['length_px']/b['width_px']>=3.5
                    and np.linalg.norm(np.array(b['center'])*[w,h]-origin)<=24
                    and abs(float(v@axis))>=math.cos(math.radians(15))):
                candidates.append((saturation,b,v))
    levels={s for s,_,_ in candidates}
    if len(levels)<3 or max(levels)-min(levels)<20:
        raise ValueError('released full beam silhouette lacks RGB support')
    # Minimum-area rectangles include empty corners when a same-colored
    # gripper joins the beam. Bound actual supported contour pixels instead.
    # Higher saturation masks are eroded subsets; their missing pixels must
    # not independently enlarge the union beyond the already visible extent.
    pixels=np.concatenate([np.array(b['contour_px']) for _,b,_ in candidates])
    normal=np.array([-axis[1],axis[0]])
    missing_length=max(0.,length-float(np.ptp(pixels@axis)))
    missing_width=max(0.,width-float(np.ptp(pixels@normal)))
    padding=np.abs(axis)*missing_length+np.abs(normal)*missing_width+1.
    lower,upper=pixels.min(axis=0)-padding,pixels.max(axis=0)+padding
    corners=[[x,y] for x in (lower[0],upper[0]) for y in (lower[1],upper[1])]
    return {'corners_px':corners,'nominal_feature_height_m':.04,
            'expected_length_px':length,'threshold_support':len(levels),
            'missing_length_px':missing_length,'missing_width_px':missing_width,
            'visible_lengths_px':[b['length_px'] for _,b,_ in candidates],
            'source':'union of fresh untrimmed RGB contour pixels, prior RGB identity and axis, authored fixed dimensions; missing-span uncertainty and one-pixel quantization margin'}


def canonical_pair_top(jpeg, reference, *, translation_px=None, hue_upper=24, observed_beam=None):
    """Translate observed pixels to the saved beam-centred image convention.

    This is image preprocessing, not a changed camera or world reset. Retain
    all pixels, with black out-of-frame padding. Novel background stays novel
    for the unchanged learned support tests. No synthetic reference pixels.
    """
    frame, ref = decode(jpeg), decode(reference)
    if frame.shape != ref.shape:
        raise ValueError('pair reference and live TOP dimensions differ')
    current, anchor = (observed_beam if observed_beam is not None else beam_feature(jpeg,hue_upper=hue_upper)), beam_feature(reference)
    h, w = frame.shape[:2]
    shift = (np.array(anchor['center']) - current['center']) * [w, h] if translation_px is None else np.asarray(translation_px,dtype=float)
    if shift.shape!=(2,) or not np.isfinite(shift).all():raise ValueError('invalid image translation')
    measured_shift=shift.copy()
    # Half-pixel contour changes must not blend wheel colours and manufacture
    # an out-of-support pose. Translate whole source pixels; retain the raw
    # estimate and <=0.50005px quantization error for inspection. Rounding the
    # minAreaRect float noise makes exact half-pixel ties deterministic.
    shift=np.rint(np.round(shift,4))
    transformed = cv2.warpAffine(frame, np.float32([[1,0,shift[0]],[0,1,shift[1]]]),
                                 (w,h), flags=cv2.INTER_NEAREST)
    data = cv2.imencode('.jpg', transformed, [cv2.IMWRITE_JPEG_QUALITY,95])[1].tobytes()
    return data, {'source_sha256':hashlib.sha256(jpeg).hexdigest(),
        'reference_sha256':hashlib.sha256(reference).hexdigest(),
        'translation_px':shift.tolist(),'observed_beam':current,
        'unquantized_translation_px':measured_shift.tolist(),
        'translation_quantization_error_px':(shift-measured_shift).tolist(),
        'fixed_from_prior_rgb':translation_px is not None,'hue_upper':hue_upper,
        'tracked_carried_shaft':observed_beam is not None,
        'method':'integer-pixel RGB translation only; black padding; unchanged own RGB'}


class SkillBindings:
    def __init__(self, committed, static_map, *, route_overlap=False, auto_route_overlap=False, overlap_start='transit'):
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
        # Jobs the team agreed to give up mid-run (dynamic coordination only).
        # They gate peers like finished jobs but are never reported delivered.
        self.dropped = set()
        self.locks = {}
        self.revoked = False
        self.cluttered=any(o['id']=='service_island' for o in static_map['obstacles'])
        if overlap_start not in ('grasp','transit'):raise ValueError('unknown overlap start')
        self.overlap_start=overlap_start
        self.grasp_started=set()
        self.transit_started=set()
        self.resource_events=[]
        beam_route=self.tasks['beam']['route'];box_route=self.tasks['box']['route']
        routes=static_map.get('routes',{})
        beam_resource=routes.get(beam_route,{}).get('resource')
        box_resource=routes.get(box_route,{}).get('resource')
        reason=None;error=None
        if self.cluttered:
            reason='service_island_requires_serial'
            error='route overlap requires open-map translation; moving-obstacle rotation is not validated'
        elif any(t['after'] for t in self.tasks.values()):
            reason='agreed_dependencies_require_serial'
            error='route overlap requires an explicitly agreed independent plan; dependencies are never removed'
        elif beam_route==box_route:
            reason='same_route_requires_serial'
            error='route overlap requires distinct routes'
        elif not beam_resource or not box_resource:
            reason='unknown_route_resource_requires_serial'
            error='route overlap requires known route resources'
        elif beam_resource==box_resource:
            reason='shared_route_resource_requires_serial'
            error='route overlap requires distinct route resources'
        if route_overlap and error:
            raise ValueError(error)
        if auto_route_overlap and not route_overlap and reason is None:
            boundary={'wall_north','wall_south','wall_west','wall_east'}
            if static_map.get('map_id')!='dispatch_open' or any(
                    obstacle.get('id') not in boundary for obstacle in static_map.get('obstacles',[])) \
                    or static_map.get('terrain'):
                reason='open_map_capability_not_established'
        self.route_overlap=bool(route_overlap or (auto_route_overlap and reason is None))
        mode='explicit' if route_overlap else 'auto' if auto_route_overlap else 'serial'
        self.overlap_selection={'mode':mode,'enabled':self.route_overlap,
            'reason':('explicit_overlap_requested' if route_overlap else
                      'independent_open_routes' if self.route_overlap else
                      reason if auto_route_overlap else 'serial_runner_default'),
            'plan_hash':committed['plan_hash'],'beam_route':beam_route,'box_route':box_route,
            'beam_resource':beam_resource,'box_resource':box_resource,
            'overlap_start':overlap_start}

    def note_transit_command(self,obj):
        """Peer stage claim from issued commands, never measured completion."""
        self.transit_started.add(obj)

    def note_grasp_command(self,obj):
        self.grasp_started.add(obj)

    def reserve_beam_apron(self,feature):
        """Reserve before the RGB-observed formation reaches the common bay."""
        if not self.route_overlap:return
        region=self.static_map['regions']['dispatch_apron']
        west=region['center_m'][0]-region['half_extents_m'][0]
        w,h=feature['image_size']
        edge=pixel_from_map([west-.30,region['center_m'][1]],self.static_map,(h,w),height=.09)[0]
        if feature['center'][0]*w>=edge and not self.permission('beam','UNLOAD'):
            raise RuntimeError('shared unload resource unavailable before RGB boundary')

    def authorize(self, committed):
        if self.revoked or committed != self.committed:
            raise RuntimeError('skill plan revoked or replaced')

    def permission(self, obj, stage):
        if self.revoked:return False
        task = self.tasks[obj]
        if self.route_overlap:
            # The pair leaves pickup first. The box may lift and use its own
            # route while the beam is moving, but queues before the shared bay.
            released=self.grasp_started if self.overlap_start=='grasp' else self.transit_started
            if obj=='box' and stage=='GRASP' and 'beam' not in released:return False
            if stage=='UNLOAD':
                if obj=='box' and not self.settled('beam'):return False
                resources=['dispatch_apron']
            elif stage in ('GRASP','TRANSIT'):
                resources=[self.static_map['routes'][task['route']]['resource']]
            else:resources=[]
            if any(self.locks.get(r,task['id'])!=task['id'] for r in resources):return False
            for r in resources:
                if r not in self.locks:self.resource_events.append({'event':'acquire','object':obj,'resource':r,'stage':stage})
                self.locks[r]=task['id']
            return True
        if (stage != 'APPROACH' or self.cluttered) and any(
                dep not in self.finished and dep not in getattr(self,'dropped',()) for dep in task['after']):
            return False
        if stage in ('GRASP','TRANSIT') or (stage=='APPROACH' and self.cluttered):
            resources = [self.static_map['routes'][task['route']]['resource'], 'dispatch_apron']
            if self.cluttered:resources.append('pickup_maneuver')
            if any(self.locks.get(r,task['id']) != task['id'] for r in resources):return False
            for r in resources:self.locks[r] = task['id']
        return True

    def finish(self,obj):
        task_id = self.tasks[obj]['id']
        self.finished.add(task_id)
        if self.route_overlap:self.resource_events.append({'event':'finish','object':obj,'released':[r for r,t in self.locks.items() if t==task_id]})
        self.locks = {r:t for r,t in self.locks.items() if t!=task_id}

    def settled(self,obj):
        task_id=self.tasks[obj]['id']
        return task_id in self.finished or task_id in getattr(self,'dropped',())

    def drop(self,obj):
        """Team decision only: give up this job and release its resources."""
        task_id=self.tasks[obj]['id']
        if task_id in self.finished:raise ValueError('finished job cannot be dropped')
        self.dropped=getattr(self,'dropped',set());self.dropped.add(task_id)
        self.resource_events.append({'event':'drop','object':obj,
            'released':[r for r,t in self.locks.items() if t==task_id]})
        self.locks={r:t for r,t in self.locks.items() if t!=task_id}

    def capabilities(self):
        # Conservative authored envelope of the demonstrated parallel formation:
        # 0.70 m maximum observed centre span plus 0.24 m chassis and 0.05 m margin.
        # A capability bound, not a runtime pose or claimed passage measurement.
        width = .70+.24+.05
        narrow = []
        if any(o['id']=='service_island' for o in self.static_map['obstacles']):
            for name, route in self.static_map['routes'].items():
                if route['declared_min_width_m'] < .45:narrow.append(name)
        return {'pair_model_slots':self.pair, 'solo_robot':self.solo,
            'route_overlap':self.route_overlap,'overlap_start':self.overlap_start,
            'overlap_selection':self.overlap_selection,
            'parallel_pair_envelope_m':width,'unsupported_pair_routes':narrow,
            'pair_rotation_skill_available':True,'rotated_pair_envelope_m':.45,
            'route_execution':'RGB wheel/shaft tracking and swept full-load SE2 search when cluttered; experimental',
            'model_support':'unchanged learned support thresholds; fail closed on novelty'}

    def check_route(self):
        route = self.tasks['beam']['route']
        if route in self.capabilities()['unsupported_pair_routes']:
            raise RuntimeError('PAIR_ROUTE_TOO_NARROW: agreed '+route+
                ' route is narrower than the rotated loaded footprint')


def pixel_from_map(xy, static_map, shape, *, height=0.):
    """Only authored goals are projected. No dynamic world positions accepted."""
    cam=static_map['top_camera']; h,w=shape[:2]
    if cam['quaternion_wxyz'] != [1,0,0,0]:raise ValueError('unsupported static camera')
    cx,cy,cz=cam['position_m']
    scale=h/(2*(cz-height)*math.tan(math.radians(cam['fov_y_deg'])/2))
    return np.array([(w-1)/2+(xy[0]-cx)*scale,(h-1)/2-(xy[1]-cy)*scale])


class ImageRoute:
    """Plan-selected authored waypoints with current cargo position from RGB."""
    def __init__(self, bindings, obj, *, time_aware_box_reacquisition=False,
                 bounded_carrier_relink=False):
        if not isinstance(time_aware_box_reacquisition, bool):
            raise ValueError('time_aware_box_reacquisition must be a bool')
        if not isinstance(bounded_carrier_relink, bool):
            raise ValueError('bounded_carrier_relink must be a bool')
        if bounded_carrier_relink and not time_aware_box_reacquisition:
            raise ValueError('bounded_carrier_relink requires time-aware direct box reacquisition')
        self.time_aware_box_reacquisition=time_aware_box_reacquisition
        self.bounded_carrier_relink=bounded_carrier_relink
        self.route_overlap=bool(getattr(bindings,'route_overlap',False))
        self._permission=getattr(bindings,'permission',None)
        if self.route_overlap and not callable(self._permission):
            raise ValueError('overlap route requires live permission check')
        self.map=bindings.static_map;self.obj=obj
        self.task=bindings.tasks[obj];self.dock=bindings.plan['dock']
        self.points=None;self.index=0;self.confirmations=0
        self.box_center=None;self.box_delta=np.zeros(2);self.box_previous=None
        self.box_previous_top_sha256=None
        self.box_background=None;self.box_origin=None;self.box_background_sha=None
        self.box_pan_frames={}
        # TOP RGB robot features are linked to cargo only after both have
        # independently shown the same motion while the cargo is visible.
        self.box_carrier_points=None
        self.box_carrier_occluded_frames=0
        self.box_carrier_occlusion_started_at_s=None
        self.box_carrier_occlusion_last_at_s=None
        self.box_carrier_occlusion_last_frame_id=None
        self.box_carrier_cumulative_motion_px=0.
        self.box_carrier_relink_audit=None
        self.box_carrier_link_source=None
        self.box_direct_recovery_pending=None
        self.box_direct_final_hold_pending=False
        self.box_direct_history=[]
        from harness.dispatch_beam_tracker import CarriedBeamTracker
        self.beam_tracker=CarriedBeamTracker()

    def record_attachment_top(self,phase,jpeg):
        from harness.dispatch_box_identity import PAN_PHASES
        if self.obj!='box' or phase not in PAN_PHASES:return
        if phase=='verify_lift':self.box_pan_frames={}
        self.box_pan_frames[phase]=jpeg

    @staticmethod
    def _tracked_points(old,new,points):
        if points is None or len(points)==0:
            return np.empty((0,2)),np.empty(0),np.empty((0,2)),np.empty((0,2))
        forward,ok,errors=cv2.calcOpticalFlowPyrLK(old,new,points,None,winSize=(15,15),maxLevel=2)
        if forward is None:
            return np.empty((0,2)),np.empty(0),np.empty((0,2)),np.empty((0,2))
        backward,back_ok,_=cv2.calcOpticalFlowPyrLK(new,old,forward,None,winSize=(15,15),maxLevel=2)
        if backward is None:
            return np.empty((0,2)),np.empty(0),np.empty((0,2)),np.empty((0,2))
        cycle=np.linalg.norm(backward-points,axis=2).ravel()
        delta=(forward-points).reshape(-1,2)
        valid=(ok.ravel()>0)&(back_ok.ravel()>0)&(cycle<1)&(np.linalg.norm(delta,axis=1)<20)&(errors.ravel()<30)
        return delta[valid],cycle[valid],points.reshape(-1,2)[valid],forward.reshape(-1,2)[valid]

    @staticmethod
    def _consistent_group(delta,*,minimum):
        if len(delta)<minimum:return None
        groups=np.linalg.norm(delta[:,None,:]-delta[None,:,:],axis=2)<1.5
        inliers=groups[np.argmax(groups.sum(axis=1))]
        return inliers if inliers.sum()>=max(minimum,.6*len(delta)) else None

    @staticmethod
    def _spread_supported(points):
        # Distributed corners on the chassis, not a tiny painted patch.
        return len(points)>=8 and np.all(np.ptp(points,axis=0)>=[20,15])

    def _link_carrier_features(self,frame,center):
        """Associate a visible cargo with the neighbouring robot by RGB comotion."""
        self.box_carrier_points=None
        if self.box_previous is None or self.box_center is None or self.box_background is None:
            return
        cargo_motion=center-self.box_center
        if not 2<=np.linalg.norm(cargo_motion)<20:return
        old=self.box_previous
        h,w=old.shape[:2]
        px,py=self.box_center
        x0=max(0,math.floor(px-120));x1=min(w,math.ceil(px+120)+1)
        y0=max(0,math.floor(py-120));y1=min(h,math.ceil(py+120)+1)
        old_roi=old[y0:y1,x0:x1]
        old_hsv=cv2.cvtColor(old_roi,cv2.COLOR_BGR2HSV)
        # This robot's orange chassis trim has independent texture. Restrict
        # candidates to the moving foreground around the previously observed
        # cargo; cyan and the static yellow apron cannot establish identity.
        orange=cv2.inRange(old_hsv,np.array((4,95,40),np.uint8),np.array((25,255,255),np.uint8))>0
        foreground=cv2.absdiff(old_roi,self.box_background[y0:y1,x0:x1]).max(axis=2)>25
        yy,xx=np.ogrid[y0:y1,x0:x1]
        radius=np.hypot(xx-px,yy-py)
        mask=np.uint8(orange&foreground&(radius>=25)&(radius<=120))*255
        old_gray=cv2.cvtColor(old,cv2.COLOR_BGR2GRAY)
        new_gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        points=cv2.goodFeaturesToTrack(old_gray[y0:y1,x0:x1],100,.01,4,mask=mask)
        if points is not None:points+=np.float32([x0,y0])
        delta,_,_,current=self._tracked_points(old_gray,new_gray,points)
        if len(delta)<8:return
        # The visual cargo motion, not the issued command, selects the robot.
        agreeing=np.linalg.norm(delta-cargo_motion,axis=1)<1.5
        if agreeing.sum()<max(8,.6*len(delta)) or not self._spread_supported(current[agreeing]):return
        self.box_carrier_points=current[agreeing].astype(np.float32).reshape(-1,1,2)

    def _fresh_linked_carrier_cloud(self,old_frame,frame,old_points,linked_motion):
        """Re-detect this linked chassis in current TOP pixels, never by command."""
        old_cloud=old_points.reshape(-1,2)
        x0=max(0,int(np.floor(old_cloud[:,0].min()-10)))
        x1=min(frame.shape[1],int(np.ceil(old_cloud[:,0].max()+10))+1)
        y0=max(0,int(np.floor(old_cloud[:,1].min()-10)))
        y1=min(frame.shape[0],int(np.ceil(old_cloud[:,1].max()+10))+1)
        old_roi=old_frame[y0:y1,x0:x1]
        old_hsv=cv2.cvtColor(old_roi,cv2.COLOR_BGR2HSV)
        orange=cv2.inRange(old_hsv,np.array((4,95,40),np.uint8),
                           np.array((25,255,255),np.uint8))>0
        foreground=cv2.absdiff(old_roi,self.box_background[y0:y1,x0:x1]).max(axis=2)>25
        old_gray=cv2.cvtColor(old_frame,cv2.COLOR_BGR2GRAY)
        new_gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        fresh=cv2.goodFeaturesToTrack(old_gray[y0:y1,x0:x1],100,.01,4,
                                      mask=np.uint8(orange&foreground)*255)
        if fresh is None:return None,{'reason':'no_fresh_carrier_corners'}
        fresh+=np.float32([x0,y0])
        delta,cycle,previous,current=self._tracked_points(old_gray,new_gray,fresh)
        inliers=self._consistent_group(delta,minimum=8)
        if inliers is None or not self._spread_supported(current[inliers]):
            return None,{'reason':'fresh_carrier_motion_ambiguous',
                         'fresh_corner_count':int(len(fresh)),
                         'fresh_valid_flow_count':int(len(delta))}
        selected_old=previous[inliers]
        nearest=np.linalg.norm(selected_old[:,None,:]-old_cloud[None,:,:],axis=2).min(axis=1)
        overlap=float(np.mean(nearest<=12.))
        fresh_motion=np.median(delta[inliers],axis=0)
        agreement=float(np.linalg.norm(fresh_motion-linked_motion))
        current_hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        current_orange=cv2.inRange(current_hsv,np.array((4,95,40),np.uint8),
                                   np.array((25,255,255),np.uint8))>0
        current_foreground=cv2.absdiff(frame,self.box_background).max(axis=2)>25
        pixels=np.rint(current[inliers]).astype(int)
        inside=(pixels[:,0]>=0)&(pixels[:,0]<frame.shape[1])&(pixels[:,1]>=0)&(pixels[:,1]<frame.shape[0])
        appearance=(float(np.mean(current_orange[pixels[:,1],pixels[:,0]]
                                  &current_foreground[pixels[:,1],pixels[:,0]]))
                    if np.all(inside) else 0.)
        metrics={'fresh_corner_count':int(len(fresh)),
                 'fresh_valid_flow_count':int(len(delta)),
                 'fresh_consistent_count':int(inliers.sum()),
                 'fresh_spread_px':np.ptp(current[inliers],axis=0).tolist(),
                 'fresh_max_cycle_error_px':float(cycle[inliers].max()),
                 'fresh_motion_px':fresh_motion.tolist(),
                 'linked_motion_px':linked_motion.tolist(),
                 'motion_disagreement_px':agreement,
                 'old_cloud_overlap_fraction':overlap,
                 'current_orange_foreground_fraction':appearance}
        # The fresh corners must still sit on the originally linked chassis,
        # move with its old cloud, and remain orange moving foreground now.
        if overlap<.6 or agreement>1.5 or appearance<.6:
            return None,{**metrics,'reason':'fresh_carrier_identity_mismatch'}
        return current[inliers].astype(np.float32).reshape(-1,1,2),metrics

    def _carrier_relink_stop(self,reason,context):
        self.box_carrier_relink_audit={**context,'stop_reason':reason,
                                      'motion_is_visual_not_issued_command':True}
        raise RuntimeError('dispatch bounded carrier relink '+reason+': '+
                           json.dumps(self.box_carrier_relink_audit,sort_keys=True,separators=(',',':')))

    def _bounded_direct_reappearance(self,candidate,proxy_center,carrier_motion,
                                     tracking,*,observed_at_s,frame_id,top_sha256):
        """Confirm two new cyan views against one bounded, visually linked carrier."""
        anchor=self.box_direct_history[-1] if self.box_direct_history else None
        pending=self.box_direct_recovery_pending
        self.box_direct_recovery_pending=None
        if candidate is None:
            if pending is not None:
                tracking['bounded_direct_reappearance']={'status':'provisional_lost',
                    'previous_frame_id':pending['frame_id']}
            return None
        rejection=None
        if anchor is None or self.box_carrier_link_source is None:
            rejection='missing_direct_appearance_or_recent_cargo_carrier_link'
        else:
            anchor_bounds=anchor['bounds_px']
            anchor_aspect=anchor_bounds[2]/max(1,anchor_bounds[3])
            area_ratio=candidate['area_px']/max(1,anchor['area_px'])
            aspect_ratio=candidate['aspect_ratio']/anchor_aspect
            if (abs(candidate['hue_median']-anchor['hue_median'])>6
                    or not .5<=area_ratio<=10.
                    or not .25<=aspect_ratio<=4.):
                rejection='cargo_appearance_anchor_mismatch'
            elif candidate['foreground_fraction']<.8:
                rejection='cyan_not_current_foreground'
            elif np.linalg.norm(np.asarray(candidate['center_px'])-proxy_center)>8.:
                rejection='cyan_outside_current_carrier_proxy'
        audit={'status':'rejected' if rejection else 'provisional',
               'reason':rejection,'current_candidate':candidate,
               'semantic_direct_anchor':({k:anchor.get(k) for k in (
                   'area_px','hue_median','bounds_px','top_sha256','frame_id','observed_at_s')}
                   if anchor is not None else None),
               'semantic_anchor_age_s':(
                   float(observed_at_s)-anchor['observed_at_s']
                   if anchor is not None else None),
               'recent_cargo_carrier_visual_link':self.box_carrier_link_source,
               'current_carrier_motion_px':carrier_motion.tolist(),
               'proxy_center_px':proxy_center.tolist(),
               'occluded_frames':self.box_carrier_occluded_frames,
               'cumulative_visual_motion_px':self.box_carrier_cumulative_motion_px,
               'same_capture_own_attachment_validated':True}
        if rejection:
            self._carrier_relink_stop('cyan_candidate_identity_mismatch',audit)
        if pending is not None:
            dt=float(observed_at_s)-pending['observed_at_s']
            predicted=np.asarray(pending['center_px'])+carrier_motion
            residual=float(np.linalg.norm(np.asarray(candidate['center_px'])-predicted))
            hue_delta=abs(candidate['hue_median']-pending['hue_median'])
            area_ratio=candidate['area_px']/max(1,pending['area_px'])
            aspect_ratio=candidate['aspect_ratio']/pending['aspect_ratio']
            stationary_final_hold=(self.box_direct_final_hold_pending
                                   and self.index==len(self.points)-1
                                   and np.linalg.norm(carrier_motion)<1.)
            repeated=(0<dt<=.6 and frame_id>pending['frame_id']
                      and (top_sha256!=pending['top_sha256']
                           or stationary_final_hold)
                      and residual<=3. and hue_delta<=4.
                      and .6<=area_ratio<=1.7 and .55<=aspect_ratio<=1.8)
            audit.update(previous_provisional=pending,repeat_gap_s=dt,
                         stationary_final_hold=bool(stationary_final_hold),
                         carrier_motion_residual_px=residual,
                         consecutive_hue_delta=hue_delta,
                         consecutive_area_ratio=area_ratio,
                         consecutive_aspect_ratio=aspect_ratio)
            if repeated:
                audit['status']='confirmed_two_view_direct_cargo'
                tracking={'method':'bounded two-view direct cyan recovery from TOP RGB',
                          'visible_component_area_px':candidate['area_px'],
                          'visible_component_bounds_px':candidate['bounds_px'],
                          'bounded_direct_reappearance':audit}
                return (np.asarray(candidate['center_px']),
                        np.asarray(candidate['bounds_px']),tracking,
                        candidate['area_px'],candidate['hue_median'])
            audit['previous_provisional_rejected']='not_consistent_with_current_carrier_motion_or_appearance'
            self._carrier_relink_stop('cyan_candidate_two_view_mismatch',audit)
        self.box_direct_recovery_pending={**candidate,
            'observed_at_s':float(observed_at_s),'frame_id':frame_id,
            'top_sha256':top_sha256}
        tracking['bounded_direct_reappearance']=audit
        return None

    def verify_release_visual(self,jpeg,own_attachment,*,observed_at_s,
                              frame_id,own_sha256):
        """Recheck held cargo and slot silhouette before the first release pose."""
        if not self.bounded_carrier_relink or self.obj!='box':
            raise ValueError('release visual gate belongs to opt-in box navigation')
        latest=self.box_direct_history[-1] if self.box_direct_history else None
        if (self.points is None or self.index!=len(self.points)-1
                or latest is None or latest['top_sha256']!=self.box_previous_top_sha256
                or isinstance(observed_at_s,bool)
                or not isinstance(observed_at_s,(int,float))
                or not math.isfinite(float(observed_at_s))
                or not 0<float(observed_at_s)-latest['observed_at_s']<=.6
                or isinstance(frame_id,bool) or not isinstance(frame_id,int)
                or frame_id<=latest['frame_id']):
            raise RuntimeError('dispatch release requires recent final direct TOP cargo')
        if (not isinstance(own_attachment,tuple) or len(own_attachment)!=3):
            raise RuntimeError('dispatch release requires current own RGB attachment')
        own_evidence,current_own,validated_own=own_attachment
        if (not isinstance(own_evidence,dict)
                or own_evidence.get('evidence')!='visual_attachment'
                or own_evidence.get('attached') is not True
                or own_evidence.get('camera_pan_delta_pwm')!=0
                or not isinstance(current_own,str) or current_own!=validated_own):
            raise RuntimeError('dispatch release requires current own RGB attachment')
        try:own_hash=hashlib.sha256(base64.b64decode(current_own,validate=True)).hexdigest()
        except (binascii.Error,ValueError,TypeError):own_hash=None
        if own_hash!=own_sha256 or not isinstance(own_sha256,str):
            raise RuntimeError('dispatch release own RGB capture SHA mismatch')
        current_top_sha=hashlib.sha256(jpeg).hexdigest()
        # A stationary cargo may produce byte-identical RGB in a genuinely
        # newer paired capture. Frame/time freshness, not pixel change, gates
        # this release recheck; moving-carrier flow still rejects stale pixels.
        frame=decode(jpeg);hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(hsv,np.array((80,125,35),np.uint8),
                         np.array((102,255,255),np.uint8))
        if self.box_background is None:
            raise RuntimeError('dispatch release TOP background unavailable')
        foreground=cv2.absdiff(frame,self.box_background).max(axis=2)>20
        mask[~foreground]=0
        count,labels,stats,centers=cv2.connectedComponentsWithStats(mask)
        previous=np.asarray(latest['center_px'])
        candidates=[i for i in range(1,count)
                    if 25<=stats[i,4]<=600 and max(stats[i,2:4])<40
                    and np.linalg.norm(centers[i]-previous)<=10]
        if len(candidates)!=1:
            raise RuntimeError('dispatch release current direct TOP cargo unresolved or ambiguous')
        i=candidates[0];x,y,bw,bh=(int(v) for v in stats[i,:4])
        slot=self.map['docks'][self.dock]['slots']['box']
        a=pixel_from_map(np.array(slot['center_m'])-slot['half_extents_m'],
                         self.map,frame.shape)
        b=pixel_from_map(np.array(slot['center_m'])+slot['half_extents_m'],
                         self.map,frame.shape)
        lo=np.minimum(a,b)+2.;hi=np.maximum(a,b)-2.
        if not (np.all(np.array([x,y])-4.>=lo)
                and np.all(np.array([x+bw,y+bh])+4.<=hi)):
            raise RuntimeError('dispatch release direct TOP cargo outside destination slot')
        self._remember_direct_box(centers[i],(x,y,bw,bh),stats[i,4],
            np.median(hsv[:,:,0][labels==i]),current_top_sha,observed_at_s,frame_id,125)
        self.box_previous_top_sha256=current_top_sha
        self.box_previous=frame.copy()
        return {'support_source':'current direct TOP cyan silhouette and current own RGB attachment',
                'top_sha256':current_top_sha,'own_sha256':own_hash,
                'frame_id':frame_id,'observed_at_s':float(observed_at_s),
                'previous_direct_top_sha256':latest['top_sha256'],
                'previous_direct_frame_id':latest['frame_id'],
                'component_area_px':int(stats[i,4]),
                'component_bounds_px':[x,y,bw,bh],
                'slot_interior_px':[lo.tolist(),hi.tolist()],
                'own_attachment':{k:own_evidence.get(k) for k in (
                    'attached','mask_iou','centroid_delta_px','area_ratio','camera_pan_delta_pwm')}}

    def _track_carrier_under_occlusion(self,frame,own_attachment,*,
                                       observed_at_s=None,frame_id=None,
                                       own_sha256=None,top_sha256=None):
        """Use a previously linked TOP robot only while fresh own RGB holds cargo."""
        if (self.box_carrier_points is None or self.box_previous is None
                or (self.box_carrier_occluded_frames>=3 and not self.bounded_carrier_relink)):
            raise RuntimeError('dispatch box unresolved or ambiguous in TOP RGB')
        relink=self.bounded_carrier_relink and self.box_carrier_occluded_frames>=3
        old_cloud=self.box_carrier_points.reshape(-1,2)
        context={'support_source':'original TOP cargo-specific RGB motion correlated with orange-carrier TOP flow',
                 'last_cargo_carrier_visual_link':self.box_carrier_link_source,
                 'previous_linked_corner_count':int(len(old_cloud)),
                 'previous_linked_bounds_px':[
                     old_cloud.min(axis=0).tolist(),old_cloud.max(axis=0).tolist()],
                 'previous_top_sha256':self.box_previous_top_sha256,
                 'current_top_sha256':top_sha256,'current_own_sha256':own_sha256,
                 'current_frame_id':frame_id,'observed_at_s':observed_at_s,
                 'occluded_frames_before':self.box_carrier_occluded_frames,
                 'cumulative_visual_motion_px_before':self.box_carrier_cumulative_motion_px,
                 'max_occluded_frames':_CARRIER_RELINK_MAX_OCCLUDED_FRAMES,
                 'max_elapsed_s':_CARRIER_RELINK_MAX_ELAPSED_S,
                 'max_visual_motion_px':_CARRIER_RELINK_MAX_MOTION_PX}
        if (not isinstance(own_attachment,tuple) or len(own_attachment)!=3):
            if self.bounded_carrier_relink:self._carrier_relink_stop('missing_current_own_attachment',context)
            raise RuntimeError('dispatch box occlusion requires current own RGB attachment evidence')
        evidence,current_own,validated_own=own_attachment
        if (not isinstance(evidence,dict) or evidence.get('evidence')!='visual_attachment'
                or evidence.get('attached') is not True
                or evidence.get('camera_pan_delta_pwm')!=0
                or not isinstance(current_own,str) or current_own!=validated_own):
            if self.bounded_carrier_relink:self._carrier_relink_stop('invalid_current_own_attachment',context)
            raise RuntimeError('dispatch box occlusion requires fresh validated own RGB attachment')
        if self.bounded_carrier_relink:
            try:own_hash=hashlib.sha256(base64.b64decode(current_own,validate=True)).hexdigest()
            except (binascii.Error,ValueError,TypeError):own_hash=None
            if own_hash!=own_sha256 or not isinstance(own_sha256,str):
                self._carrier_relink_stop('own_capture_sha_mismatch',context)
            context['current_own_attachment']={k:evidence.get(k) for k in (
                'attached','reason','mask_iou','centroid_delta_px','area_ratio',
                'camera_pan_delta_pwm')}
            if (isinstance(observed_at_s,bool) or not isinstance(observed_at_s,(int,float))
                    or not math.isfinite(float(observed_at_s))
                    or isinstance(frame_id,bool) or not isinstance(frame_id,int)
                    or not isinstance(top_sha256,str) or len(top_sha256)!=64):
                self._carrier_relink_stop('invalid_capture_identity_or_time',context)
            if self.box_carrier_occluded_frames:
                last_at=self.box_carrier_occlusion_last_at_s
                last_frame=self.box_carrier_occlusion_last_frame_id
                if (last_at is None or last_frame is None
                        or not 0<float(observed_at_s)-last_at<=.6
                        or frame_id<=last_frame):
                    self._carrier_relink_stop('stale_or_reversed_carrier_capture',context)
            else:
                link=self.box_carrier_link_source
                if (not isinstance(link,dict)
                        or isinstance(link.get('observed_at_s'),bool)
                        or not isinstance(link.get('observed_at_s'),(int,float))
                        or not math.isfinite(float(link['observed_at_s']))
                        or isinstance(link.get('frame_id'),bool)
                        or not isinstance(link.get('frame_id'),int)
                        or not 0<float(observed_at_s)-float(link['observed_at_s'])<=.6
                        or frame_id<=link['frame_id']):
                    self._carrier_relink_stop('stale_first_carrier_capture_after_cargo_link',context)
        stationary_final_hold=(self.bounded_carrier_relink
            and self.box_direct_final_hold_pending
            and self.box_direct_recovery_pending is not None
            and self.points is not None and self.index==len(self.points)-1)
        if np.array_equal(frame,self.box_previous) and not stationary_final_hold:
            if self.bounded_carrier_relink:self._carrier_relink_stop('repeated_top_rgb',context)
            raise RuntimeError('dispatch box occlusion has stale TOP RGB')
        if relink and (self.box_carrier_occluded_frames+1>_CARRIER_RELINK_MAX_OCCLUDED_FRAMES
                       or self.box_carrier_occlusion_started_at_s is None
                       or float(observed_at_s)-self.box_carrier_occlusion_started_at_s>
                            _CARRIER_RELINK_MAX_ELAPSED_S):
            self._carrier_relink_stop('occlusion_frame_or_time_budget_exhausted',context)
        old_gray=cv2.cvtColor(self.box_previous,cv2.COLOR_BGR2GRAY)
        new_gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        delta,cycle,_,current=self._tracked_points(old_gray,new_gray,self.box_carrier_points)
        inliers=self._consistent_group(delta,minimum=8)
        if inliers is None or not self._spread_supported(current[inliers]):
            if relink:self._carrier_relink_stop('linked_motion_ambiguous',context)
            raise RuntimeError('dispatch linked carrier motion ambiguous in TOP RGB')
        # Preserve the linked robot's colour and foreground identity in the
        # new image; tracking onto a stationary floor or a peer fails closed.
        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        orange=cv2.inRange(hsv,np.array((4,95,40),np.uint8),np.array((25,255,255),np.uint8))>0
        foreground=cv2.absdiff(frame,self.box_background).max(axis=2)>25
        pixels=np.rint(current[inliers]).astype(int)
        inside=(pixels[:,0]>=0)&(pixels[:,0]<frame.shape[1])&(pixels[:,1]>=0)&(pixels[:,1]<frame.shape[0])
        if not np.all(inside) or np.mean(orange[pixels[:,1],pixels[:,0]]&foreground[pixels[:,1],pixels[:,0]])<.6:
            if relink:self._carrier_relink_stop('linked_appearance_unresolved',context)
            raise RuntimeError('dispatch linked carrier appearance unresolved in TOP RGB')
        motion=np.median(delta[inliers],axis=0)
        if np.linalg.norm(motion)<1 and not stationary_final_hold:
            if relink:self._carrier_relink_stop('linked_carrier_no_fresh_motion',context)
            raise RuntimeError('dispatch linked carrier has no fresh TOP RGB motion')
        relink_metrics=None
        if relink:
            refreshed,relink_metrics=self._fresh_linked_carrier_cloud(
                self.box_previous,frame,self.box_carrier_points,motion)
            if refreshed is None:
                self._carrier_relink_stop(relink_metrics['reason'],{**context,**relink_metrics})
        cumulative=self.box_carrier_cumulative_motion_px+float(np.linalg.norm(motion))
        if self.bounded_carrier_relink and cumulative>_CARRIER_RELINK_MAX_MOTION_PX:
            self._carrier_relink_stop('cumulative_visual_motion_budget_exhausted',
                                      {**context,'next_cumulative_visual_motion_px':cumulative})
        self.box_carrier_points=(refreshed if relink else
                                 current[inliers].astype(np.float32).reshape(-1,1,2))
        self.box_carrier_occluded_frames+=1
        if self.bounded_carrier_relink:
            if self.box_carrier_occlusion_started_at_s is None:
                self.box_carrier_occlusion_started_at_s=float(observed_at_s)
            self.box_carrier_occlusion_last_at_s=float(observed_at_s)
            self.box_carrier_occlusion_last_frame_id=frame_id
            self.box_carrier_cumulative_motion_px=cumulative
        tracking={
            'method':'linked TOP RGB carrier motion during cargo occlusion',
            'linked_feature_count':int(len(delta)),
            'consistent_features':int(inliers.sum()),
            'max_cycle_error_px':float(cycle[inliers].max()),
            'motion_px':motion.tolist(),
            'occluded_frames':self.box_carrier_occluded_frames,
            'own_attachment':'fresh validated own RGB required',
            'uncertainty':'cargo silhouette hidden; proxy center cannot confirm delivery',
        }
        if relink:
            audit={**context,**relink_metrics,'status':'accepted',
                   'occluded_frames_after':self.box_carrier_occluded_frames,
                   'elapsed_occlusion_s':float(observed_at_s)-self.box_carrier_occlusion_started_at_s,
                   'cumulative_visual_motion_px_after':cumulative,
                   'fresh_linked_corner_count':int(len(self.box_carrier_points)),
                   'same_capture_own_attachment_validated':True,
                   'motion_is_visual_not_issued_command':True}
            self.box_carrier_relink_audit=audit
            tracking['bounded_carrier_relink']=audit
        return self.box_center+motion,tracking

    def _remember_direct_box(self,center,bounds,area,hue,top_sha,
                             observed_at_s,frame_id,saturation):
        """Only the strong, directly segmented TOP cargo may seed a visual speed."""
        if (not self.time_aware_box_reacquisition or saturation<125
                or isinstance(observed_at_s,bool)
                or not isinstance(observed_at_s,(int,float))
                or not math.isfinite(float(observed_at_s))
                or isinstance(frame_id,bool) or not isinstance(frame_id,int)):
            return
        if self.box_direct_history and (observed_at_s<=self.box_direct_history[-1]['observed_at_s']
                or frame_id<=self.box_direct_history[-1]['frame_id']):
            return
        self.box_direct_history.append({
            'center_px':tuple(float(v) for v in center),
            'bounds_px':tuple(int(v) for v in bounds),
            'area_px':int(area),'hue_median':float(hue),
            'top_sha256':top_sha,'observed_at_s':float(observed_at_s),
            'frame_id':frame_id})
        self.box_direct_history=self.box_direct_history[-2:]

    def _reacquire_direct_box(self,frame,hsv,foreground,jpeg,own_attachment,
                              observed_at_s,frame_id,own_sha256):
        """Recover one visible cargo after a missed narrow gate, not a proxy."""
        if not self.time_aware_box_reacquisition or len(self.box_direct_history)<2:
            return None
        if (not isinstance(own_attachment,tuple) or len(own_attachment)!=3
                or not isinstance(own_sha256,str)):
            return None
        evidence,current_own,validated_own=own_attachment
        if (not isinstance(evidence,dict) or evidence.get('evidence')!='visual_attachment'
                or evidence.get('attached') is not True
                or evidence.get('camera_pan_delta_pwm')!=0
                or not isinstance(current_own,str) or current_own!=validated_own):
            return None
        try:
            if hashlib.sha256(base64.b64decode(current_own,validate=True)).hexdigest()!=own_sha256:
                return None
        except (ValueError,TypeError):
            return None
        if (isinstance(observed_at_s,bool) or not isinstance(observed_at_s,(int,float))
                or not math.isfinite(float(observed_at_s))
                or isinstance(frame_id,bool) or not isinstance(frame_id,int)):
            return None
        earlier,last=self.box_direct_history
        dt=float(observed_at_s)-last['observed_at_s']
        history_dt=last['observed_at_s']-earlier['observed_at_s']
        top_sha=hashlib.sha256(jpeg).hexdigest()
        if (not 0<dt<=.6 or not 0<history_dt<=.8
                or frame_id<=last['frame_id'] or top_sha==last['top_sha256']
                or np.array_equal(frame,self.box_previous)):
            return None
        velocity=(np.asarray(last['center_px'])-np.asarray(earlier['center_px']))/history_dt
        speed=float(np.linalg.norm(velocity))
        if not 3<=speed<=100:
            return None
        direction=velocity/speed
        predicted=np.asarray(last['center_px'])+velocity*dt
        longitudinal_limit=min(18.,4.+.5*speed*dt)
        lateral_limit=8.
        displacement_limit=min(36.,2.+1.75*speed*dt)
        # The stronger original cyan cut is required for reacquisition. A
        # lower-saturation fragment remains eligible only for legacy tracking.
        mask=cv2.inRange(hsv,np.array((80,125,35),np.uint8),
                         np.array((102,255,255),np.uint8))
        mask[~foreground]=0
        count,labels,stats,centers=cv2.connectedComponentsWithStats(mask)
        plausible=[]
        corridor_count=0
        for i in range(1,count):
            x,y,width,height,area=(int(v) for v in stats[i,:5])
            if not (8<=area<=600 and max(width,height)<40
                    and x>0 and y>0 and x+width<frame.shape[1]
                    and y+height<frame.shape[0]):
                continue
            shift=centers[i]-np.asarray(last['center_px'])
            residual=centers[i]-predicted
            along=float(residual@direction)
            across=float(abs(np.linalg.det(np.stack((direction,residual)))))
            if (abs(along)>longitudinal_limit or across>lateral_limit
                    or np.linalg.norm(shift)>displacement_limit
                    or float(shift@direction)<2.):
                continue
            corridor_count+=1
            old_x,old_y,old_width,old_height=last['bounds_px']
            if not (.5<=area/last['area_px']<=2.
                    and .55<=width/old_width<=1.9
                    and .55<=height/old_height<=1.9):
                continue
            hue=float(np.median(hsv[:,:,0][labels==i]))
            if abs(hue-last['hue_median'])>8:
                continue
            plausible.append((i,centers[i],(x,y,width,height),area,hue,along,across))
        # A second cyan fragment in the motion corridor is an identity
        # ambiguity even if its size or hue fit is weaker than the first.
        if corridor_count!=1 or len(plausible)!=1:
            return None
        _,center,bbox,area,hue,along,across=plausible[0]
        x,y,width,height=bbox
        bounds=np.array([[x,y],[x+width,y+height]])
        tracking={
            'method':'time-aware direct cyan reacquisition from TOP RGB',
            'source':'unique current foreground cyan silhouette, two prior direct TOP RGB silhouettes, current own RGB attachment',
            'previous_direct_frame_ids':[earlier['frame_id'],last['frame_id']],
            'current_frame_id':frame_id,
            'previous_direct_top_sha256':last['top_sha256'],
            'current_top_sha256':top_sha,
            'direct_observation_gap_s':dt,
            'visual_velocity_px_s':velocity.tolist(),
            'predicted_center_px':predicted.tolist(),
            'longitudinal_residual_px':along,
            'lateral_residual_px':across,
            'longitudinal_limit_px':longitudinal_limit,
            'lateral_limit_px':lateral_limit,
            'displacement_limit_px':displacement_limit,
            'candidate_count_in_corridor':corridor_count,
            'visible_component_area_px':area,
            'visible_component_hue_median':hue,
            'own_attachment':'fresh validated same-capture own RGB required',
            'uncertainty':'identity inferred from bounded image continuity; no measured contact or cargo pose',
        }
        return center,bounds,tracking,area,hue

    def observe(self,jpeg, *, own_attachment=None, observed_at_s=None,
                frame_id=None, own_sha256=None):
        frame=decode(jpeg);h,w=frame.shape[:2]
        if self.obj=='beam':
            feature=self.beam_tracker.observe(jpeg)
            center=np.array(feature['center'])*[w,h]
            bounds=np.array(feature['corners4'])*[w,h]
        else:
            hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
            binding=None
            if self.box_center is None and self.box_pan_frames:
                from harness.dispatch_box_identity import bind_attachment_pan
                bound_center,binding=bind_attachment_pan(self.box_pan_frames)
            # Preserve the strongest usable cargo colour first. Only an already
            # acquired target may use dimmer/smaller visible fragments, and each
            # candidate must match its prior RGB motion within eight pixels.
            # Otherwise gripper flow at a different height accumulates drift.
            if self.box_background is None:
                self.box_background=frame.copy()
                self.box_background_sha=hashlib.sha256(jpeg).hexdigest()
                self.box_origin=None if self.box_center is None else self.box_center.copy()
            # The fixed TOP camera provides observed static-floor memory. Do
            # not suppress the initially occupied patch, whose floor was hidden.
            suppress_background=(self.box_center is not None and self.box_origin is not None
                                 and np.linalg.norm(self.box_center-self.box_origin)>30)
            foreground=cv2.absdiff(frame,self.box_background).max(axis=2)>20
            levels=[125] if self.box_center is None else [125,105,90,70]
            competing_strong_cyan=0
            for saturation in levels:
                mask=cv2.inRange(hsv,np.array((80,saturation,35),np.uint8),np.array((102,255,255),np.uint8))
                if suppress_background:mask[~foreground]=0
                if self.box_center is None:
                    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
                n,labels,stats,centers=cv2.connectedComponentsWithStats(mask)
                choices=[i for i in range(1,n) if (25 if self.box_center is None else 8) <= stats[i,4] <= 600
                         and max(stats[i,2:4]) < 40]
                if binding is not None:
                    choices=[i for i in choices if np.linalg.norm(centers[i]-bound_center)<1.]
                if self.box_center is not None:
                    predicted=self.box_center+np.clip(self.box_delta,-15,15)
                    choices=sorted((i for i in choices if np.linalg.norm(centers[i]-predicted)<=8),
                                   key=lambda i:np.linalg.norm(centers[i]-predicted))
                    if saturation==125 and len(choices)>1:
                        # The legacy nearest-candidate shortcut is not an
                        # identity proof after cargo occlusion. Remember all
                        # strong near-proxy components before that reduction.
                        competing_strong_cyan=len(choices)
                    if len(choices)>1 and np.linalg.norm(centers[choices[1]]-predicted)-np.linalg.norm(centers[choices[0]]-predicted)>5:
                        choices=choices[:1]
                # A strong mask can split one partially occluded box into two
                # nearby fragments. Keep the same geometry and motion gates,
                # but allow an existing lower-saturation mask to join them.
                # A unique strong candidate still wins immediately.
                if len(choices)==1:break
            weak_occluded_component=occluded_candidate=None
            if self.bounded_carrier_relink and self.box_carrier_occluded_frames:
                if competing_strong_cyan:
                    self._carrier_relink_stop('competing_current_strong_cyan',{
                        'current_top_sha256':hashlib.sha256(jpeg).hexdigest(),
                        'current_own_sha256':own_sha256,
                        'frame_id':frame_id,'observed_at_s':observed_at_s,
                        'strong_candidate_count':competing_strong_cyan,
                        'occluded_frames':self.box_carrier_occluded_frames,
                        'cumulative_visual_motion_px':self.box_carrier_cumulative_motion_px})
                if len(choices)==1:
                    i=choices[0];x,y,bw,bh=(int(v) for v in stats[i,:4])
                    if saturation==125 and stats[i,4]>=25:
                        occluded_candidate={
                            'center_px':centers[i].tolist(),
                            'bounds_px':[[x,y],[x+bw,y+bh]],
                            'area_px':int(stats[i,4]),
                            'hue_median':float(np.median(hsv[:,:,0][labels==i])),
                            'aspect_ratio':float(bw/max(1,bh)),
                            'foreground_fraction':float(np.mean(foreground[labels==i]))}
                    else:
                        weak_occluded_component={'saturation':int(saturation),
                            'area_px':int(stats[i,4]),
                            'center_px':centers[i].tolist()}
                # Even a full-size strong cyan component is provisional while
                # the cargo was hidden. The linked carrier and current own
                # attachment must pass first; no single frame resets budgets.
                choices=[]
            tracking={'method':'cyan component','min_saturation':saturation}
            if binding is not None:tracking['initial_identity']=binding
            recovered=None
            def reacquire_direct():
                # The original eight-pixel colour/KLT/carrier routes have
                # priority. Only a missing narrow-gate candidate can enter.
                if self.bounded_carrier_relink and self.box_carrier_occluded_frames:
                    # The opt-in's already-hidden cargo cannot regain identity
                    # through one direct frame after carrier support fails.
                    return None
                if choices:return None
                return self._reacquire_direct_box(
                    frame,hsv,foreground,jpeg,own_attachment,
                    observed_at_s,frame_id,own_sha256)
            if len(choices)==1:
                i=choices[0];center=centers[i];x,y,bw,bh=stats[i,:4]
                bounds=np.array([[x,y],[x+bw,y+bh]])
                if self.bounded_carrier_relink:
                    tracking['visible_component_area_px']=int(stats[i,4])
                    tracking['visible_component_bounds_px']=bounds.tolist()
            elif self.box_previous is not None and self.box_carrier_occluded_frames:
                # Once the silhouette was lost, unrelated corners near the
                # proxy must not silently re-establish cargo identity. Only a
                # fresh cyan component can end the bounded occlusion window.
                try:center,tracking=self._track_carrier_under_occlusion(
                    frame,own_attachment,observed_at_s=observed_at_s,
                    frame_id=frame_id,own_sha256=own_sha256,
                    top_sha256=hashlib.sha256(jpeg).hexdigest())
                except RuntimeError as exc:
                    if not (str(exc).startswith('dispatch bounded carrier relink ')
                            or str(exc) in {
                            'dispatch box unresolved or ambiguous in TOP RGB',
                            'dispatch linked carrier motion ambiguous in TOP RGB',
                            'dispatch linked carrier appearance unresolved in TOP RGB',
                            'dispatch linked carrier has no fresh TOP RGB motion'}):
                        raise
                    recovered=reacquire_direct()
                    if recovered is None:raise
                if recovered is None:
                    bounds=np.array([center-12,center+12])
                    if self.bounded_carrier_relink:
                        recovered=self._bounded_direct_reappearance(
                            occluded_candidate,center,
                            np.asarray(tracking['motion_px']),tracking,
                            observed_at_s=observed_at_s,frame_id=frame_id,
                            top_sha256=hashlib.sha256(jpeg).hexdigest())
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
                    if self.box_carrier_points is None:
                        recovered=reacquire_direct()
                        if recovered is None:
                            raise RuntimeError('dispatch box appearance unresolved in TOP RGB')
                    delta,cycle=np.empty((0,2)),np.empty(0)
                else:
                    delta,cycle,_,_=self._tracked_points(old,gray,points)
                if recovered is None:
                    if len(delta)<3 and self.box_carrier_points is None:
                        recovered=reacquire_direct()
                        if recovered is None:
                            raise RuntimeError('dispatch box unresolved or ambiguous in TOP RGB')
                    else:
                        # A painted edge may supply stationary corners; require
                        # a majority of independent features to share motion.
                        inliers=self._consistent_group(delta,minimum=3)
                        if inliers is not None:
                            motion=np.median(delta[inliers],axis=0)
                            center=self.box_center+motion
                            bounds=np.array([center-12,center+12])
                            tracking={'method':'bidirectional RGB feature motion; own attachment independently required',
                                      'feature_count':int(len(delta)),'consistent_features':int(inliers.sum()),
                                      'max_cycle_error_px':float(cycle[inliers].max()),'motion_px':motion.tolist()}
                        else:
                            try:center,tracking=self._track_carrier_under_occlusion(
                                frame,own_attachment,observed_at_s=observed_at_s,
                                frame_id=frame_id,own_sha256=own_sha256,
                                top_sha256=hashlib.sha256(jpeg).hexdigest())
                            except RuntimeError as exc:
                                if not (str(exc).startswith('dispatch bounded carrier relink ')
                                        or str(exc) in {
                                        'dispatch box unresolved or ambiguous in TOP RGB',
                                        'dispatch linked carrier motion ambiguous in TOP RGB',
                                        'dispatch linked carrier appearance unresolved in TOP RGB',
                                        'dispatch linked carrier has no fresh TOP RGB motion'}):
                                    raise
                                recovered=reacquire_direct()
                                if recovered is None:raise
                            if recovered is None:bounds=np.array([center-12,center+12])
            else:raise RuntimeError('dispatch box unresolved or ambiguous in TOP RGB')
            if recovered is not None:
                center,bounds,tracking,recovered_area,recovered_hue=recovered
                if (self.box_carrier_relink_audit is not None
                        and 'stop_reason' in self.box_carrier_relink_audit):
                    tracking['bounded_carrier_relink_aborted_before_direct_reacquisition']=(
                        self.box_carrier_relink_audit)
                elif self.box_carrier_relink_audit is not None:
                    tracking['bounded_carrier_relink_prior_support']=self.box_carrier_relink_audit
                # A newly reacquired image is directly visible, but a failed
                # carrier cloud is not reusable as evidence on the next frame.
                self.box_carrier_points=None
                self.box_carrier_occluded_frames=0
                self.box_carrier_occlusion_started_at_s=None
                self.box_carrier_occlusion_last_at_s=None
                self.box_carrier_occlusion_last_frame_id=None
                self.box_carrier_cumulative_motion_px=0.
                self.box_carrier_relink_audit=None
                self.box_carrier_link_source=None
                self.box_direct_recovery_pending=None
                self.box_direct_final_hold_pending=False
                self.confirmations=0
            if weak_occluded_component is not None:
                tracking['weak_cyan_fragment_not_direct_identity']=weak_occluded_component
            if self.box_origin is None:self.box_origin=center.copy()
            tracking['static_background']={'reference_sha256':self.box_background_sha,
                'active':bool(suppress_background),'rgb_difference_threshold':20,
                'initial_occupied_radius_px':30,'origin_px':self.box_origin.tolist()}
            if tracking['method']=='cyan component':
                self._remember_direct_box(center,(int(x),int(y),int(bw),int(bh)),
                    stats[i,4],np.median(hsv[:,:,0][labels==i]),
                    hashlib.sha256(jpeg).hexdigest(),observed_at_s,frame_id,saturation)
            elif recovered is not None:
                x,y,bw,bh=(int(v) for v in (bounds[0,0],bounds[0,1],
                                             bounds[1,0]-bounds[0,0],bounds[1,1]-bounds[0,1]))
                self._remember_direct_box(center,(x,y,bw,bh),recovered_area,
                    recovered_hue,hashlib.sha256(jpeg).hexdigest(),
                    observed_at_s,frame_id,125)
            if (tracking['method']!='linked TOP RGB carrier motion during cargo occlusion'
                    and recovered is None):
                self._link_carrier_features(frame,center)
                self.box_carrier_link_source=(
                    {'cargo_support_method':tracking['method'],
                     'frame_id':frame_id,'observed_at_s':observed_at_s,
                     'top_sha256':hashlib.sha256(jpeg).hexdigest(),
                     'linked_corner_count':int(len(self.box_carrier_points))}
                    if self.box_carrier_points is not None else None)
                self.box_carrier_occluded_frames=0
                self.box_carrier_occlusion_started_at_s=None
                self.box_carrier_occlusion_last_at_s=None
                self.box_carrier_occlusion_last_frame_id=None
                self.box_carrier_cumulative_motion_px=0.
                self.box_carrier_relink_audit=None
                self.box_direct_recovery_pending=None
                self.box_direct_final_hold_pending=False
            self.box_delta=np.zeros(2) if self.box_center is None else center-self.box_center
            self.box_center=center.copy();self.box_previous=frame.copy()
            self.box_previous_top_sha256=hashlib.sha256(jpeg).hexdigest()

        if self.points is None:
            # The open arena permits the original parallel formation. Avoid
            # shifting it into a wall merely to hit a narrow symbolic gate.
            gate=self.map['regions'][self.task['route']+'_gate']['center_m'][:]
            if self.obj=='box':
                from harness.dispatch_navigation_map import solo_gate
                gate=solo_gate(self.map,self.task['route'],jpeg)
            margin=.48 if self.obj=='beam' else .18
            ymin,ymax=self.map['bounds_m'][2:]
            gate[1]=max(ymin+margin,min(ymax-margin,gate[1]))
            slots=self.map['docks'][self.dock]['slots']
            destination=list(slots[self.obj]['center_m'])
            if self.obj=='beam':
                # Match the rotated navigator's clearance preference even in
                # the open arena. Leave space for the adjacent box carrier.
                direction=math.copysign(1.,slots['box']['center_m'][0]-destination[0])
                destination[0]-=direction*.06
            gate_px=pixel_from_map(gate,self.map,frame.shape)
            east_px=pixel_from_map([1.12,gate[1]],self.map,frame.shape)
            # A lifted shaft is viewed above the floor. Use the same authored
            # 9cm nominal feature plane as the map navigator, never live height.
            goal_px=pixel_from_map(destination,self.map,frame.shape,
                                   height=.09 if self.obj=='beam' else 0.)
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
                if self.route_overlap:
                    # A separate south/north staging point stays west of the
                    # shared apron, including the authored chassis margin.
                    west=self.map['regions']['dispatch_apron']['center_m'][0]-self.map['regions']['dispatch_apron']['half_extents_m'][0]
                    self.points.insert(1,pixel_from_map([west-.20,gate[1]],self.map,frame.shape))
        error=self.points[self.index]-center
        tolerance=4 if self.index==len(self.points)-1 else 6
        ready=float(np.max(np.abs(error))) <= tolerance
        slot_evidence=None
        if (self.obj=='box' and self.index==len(self.points)-1
                and self.bounded_carrier_relink
                and self.box_direct_recovery_pending is not None
                and tracking['method']=='linked TOP RGB carrier motion during cargo occlusion'):
            # A first reappearing cyan view is only provisional. Stop motion
            # and request the second fresh frame; do not consume a final-slot
            # confirmation or use the proxy to authorize release.
            self.box_direct_final_hold_pending=True
            return ({'kind':'mecanum','forward':0.,'left':0.,'turn':0.,'duration_s':.1},
                    {'source':'current TOP carrier and own RGB; pending second direct cargo view',
                     'cargo_center_px':center.tolist(),'cargo_bounds_px':bounds.tolist(),
                     'waypoint_index':self.index,
                     'waypoints_px':[p.tolist() for p in self.points],
                     'error_px':error.tolist(),'ready':False,'done':False,
                     'waiting_for_second_direct_cargo':True,'tracking':tracking})
        if self.obj=='box' and self.index==len(self.points)-1 and self.bounded_carrier_relink:
            direct_strong=(tracking['method'] in {
                'cyan component','time-aware direct cyan reacquisition from TOP RGB',
                'bounded two-view direct cyan recovery from TOP RGB'}
                and (tracking['method']!='cyan component'
                     or tracking.get('min_saturation')==125)
                and tracking.get('visible_component_area_px',0)>=25)
            if not direct_strong:
                raise RuntimeError('dispatch box delivery requires strong current direct TOP cargo silhouette')
        if (self.obj=='box' and self.index==len(self.points)-1
                and tracking['method']=='linked TOP RGB carrier motion during cargo occlusion'):
            # The proxy can guide travel, but hidden cargo cannot establish
            # final slot containment or authorize release.
            raise RuntimeError('dispatch box delivery requires fresh cargo silhouette in TOP RGB')
        if self.obj=='box' and self.index==len(self.points)-1:
            # Delivery is containment in an authored floor region. Continuing
            # toward its exact centre can push the carrier into released cargo.
            # Pad the observed silhouette for occluded edges and segmentation
            # error; own attachment and post-release visual QA remain required.
            slot=self.map['docks'][self.dock]['slots']['box']
            a=pixel_from_map(np.array(slot['center_m'])-slot['half_extents_m'],self.map,frame.shape)
            b=pixel_from_map(np.array(slot['center_m'])+slot['half_extents_m'],self.map,frame.shape)
            lo=np.minimum(a,b)+2.;hi=np.maximum(a,b)-2.
            lower=bounds.min(axis=0)-4.;upper=bounds.max(axis=0)+4.
            ready=bool(np.all(lower>=lo) and np.all(upper<=hi))
            minimum_center=lo-(lower-center);maximum_center=hi-(upper-center)
            if np.any(minimum_center>maximum_center):
                raise RuntimeError('observed box envelope does not fit destination region')
            guided=np.clip(self.points[self.index],minimum_center,maximum_center)
            error=guided-center;tolerance=.5
            slot_evidence={'source':'current RGB silhouette plus authored slot; not measured cargo pose',
                'slot_interior_px':[lo.tolist(),hi.tolist()],
                'padded_cargo_bounds_px':[lower.tolist(),upper.tolist()],
                'occlusion_padding_px':4.,'floor_margin_px':2.,'inside':ready,'guided_center_px':guided.tolist()}
        self.confirmations=self.confirmations+1 if ready else 0
        done=self.index==len(self.points)-1 and self.confirmations>=2
        evidence={'source':'TOP RGB + authored map', 'cargo_center_px':center.tolist(),
                  'cargo_bounds_px':bounds.tolist(),'waypoint_index':self.index,
                  'waypoints_px':[p.tolist() for p in self.points],
                  'error_px':error.tolist(),'ready':ready,'done':done}
        if (self.obj=='box' and self.route_overlap and self.index==1 and ready
                and not self._permission('box','UNLOAD')):
            self.confirmations=0
            evidence.update(waiting_for_resource=True,resource='dispatch_apron',done=False)
            return {'kind':'mecanum','forward':0.,'left':0.,'turn':0.,'duration_s':.2},evidence
        if self.obj=='box':evidence['tracking']=tracking
        if slot_evidence is not None:evidence['destination_region']=slot_evidence
        if ready and self.confirmations>=2 and not done:
            self.index+=1;self.confirmations=0
        # More authority only on known independent open-map cargo routes.
        # The same proportional gain decelerates near every waypoint; final
        # beam alignment and box containment keep their original caps.
        cruise=(self.obj in ('beam','box') and self.route_overlap
                and self.map.get('map_id')=='dispatch_open'
                and not self.map.get('terrain')
                and all(o.get('id') in {'wall_north','wall_south','wall_west','wall_east'}
                        for o in self.map.get('obstacles',[]))
                and evidence['waypoint_index']<len(self.points)-1)
        limits=np.array([.12,.10]) if cruise else np.array([.08,.08])
        control=np.clip(error*.002,-limits,limits)
        control[0]=max(-.05,control[0])
        if cruise:evidence['cruise_command_limits']={'forward':.12,'left':.10,'reverse':.05}
        if ready:control[:]=0
        else:
            for i in range(2):
                if abs(error[i])>tolerance:control[i]=math.copysign(max(.025,abs(control[i])),control[i])
                else:control[i]=0
        action={'kind':'mecanum','forward':float(control[0]),'left':float(-control[1]),
                'turn':0.,'duration_s':.2}
        return action,evidence


def _coarse_small_components(yellow):
    """Keep the original <=300-pixel components with one label lookup."""
    n, labels, stats, _ = cv2.connectedComponentsWithStats(yellow)
    keep = np.zeros(n, dtype=np.uint8)
    keep[1:] = (stats[1:, 4] <= 300).astype(np.uint8) * 255
    clean = keep[labels]
    return clean, int(np.count_nonzero(keep[1:])), int(np.count_nonzero(clean))


@lru_cache(maxsize=4)
def _cached_coarse_raw_mask(raw_top):
    """State-free TOP segmentation, shared only by calls with identical JPEGs."""
    frame = decode(raw_top)
    h, w = frame.shape[:2]
    if (w, h) != (960, 720):
        raise ValueError('pair coarse TOP requires calibrated 960x720 pixels')
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, np.array((20, 70, 50), np.uint8),
                         np.array((40, 255, 255), np.uint8))
    clean, count, pixels = _coarse_small_components(yellow)
    clean.setflags(write=False)
    return clean, int(np.count_nonzero(yellow)), count, pixels


def _coarse_raw_mask(raw_top):
    clean, yellow_pixels, count, pixels = _cached_coarse_raw_mask(raw_top)
    return clean.copy(), yellow_pixels, count, pixels


@lru_cache(maxsize=4)
def _cached_coarse_beam(raw_top):
    return beam_feature(raw_top)


def _coarse_beam(raw_top):
    """Pure beam candidates are reused; callers receive their own copy."""
    return copy.deepcopy(_cached_coarse_beam(raw_top))


@lru_cache(maxsize=8)
def _cached_coarse_reference_lane(reference, slot):
    from harness.camera_goal_transport import lane_features
    return lane_features(reference, slot)


def _coarse_reference_lane(reference, slot):
    return copy.deepcopy(_cached_coarse_reference_lane(reference, slot))


class PairCoarsePixels:
    """Role-bound RGB wheel selection; rejects painted floor/beam components.

    The old HSV lane mask confused new yellow paint and brighter beam pixels
    with wheels. Identity comes from the robot's own isolated probe. Heading
    keeps the original four-corner gate; model support checks are untouched.
    """
    def __init__(self, identity, bindings, reference):
        self.centers={}
        self.reference=reference
        height,width=decode(reference).shape[:2]
        if (width,height)!=(960,720):
            raise ValueError('pair coarse reference requires calibrated 960x720 TOP')
        for slot,rid in bindings.pair.items():
            claim=identity[rid]['claim']
            if not claim.get('valid') or not claim.get('center'):
                raise ValueError('valid own motion identity required for '+rid)
            self.centers[slot]=np.array(claim['center'])*[width,height]

    def decide(self, raw_top, slot):
        from harness.camera_goal_transport import wheel_heading
        if not isinstance(raw_top, bytes) or not raw_top:
            raise ValueError('nonempty JPEG required')
        clean, raw_yellow_pixels, selected_components, component_pixels = _coarse_raw_mask(raw_top)
        h,w=clean.shape
        cx,cy=self.centers[slot]
        yy,xx=np.indices(clean.shape)
        clean[(abs(xx-cx)>55)|(abs(yy-cy)>48)]=0
        ys,xs=np.nonzero(clean)
        mask={'raw_yellow_pixels':raw_yellow_pixels,
              'small_component_pixels':component_pixels,
              'small_component_count':selected_components,
              'local_wheel_pixels':int(len(xs)),
              'crop_center_px':[float(cx),float(cy)],
              'crop_half_size_px':[55,48], 'heading_tolerance_px':2.}
        heading=wheel_heading(clean,pixel_tolerance=2.)
        if not 80 <= len(xs) <= 700:
            return dict(ok=False,ready=False,forward=0.,left=0.,turn=0.,
                        reason='own_wheel_heading_unresolved',mask=mask)
        center=np.array([xs.mean(),ys.mean()])
        if heading is not None:self.centers[slot]=center
        try:
            beam=_coarse_beam(raw_top)
            ref=_coarse_reference_lane(self.reference,slot)
        except ValueError as error:
            return dict(ok=False,ready=False,forward=0.,left=0.,turn=0.,
                        reason='payload_or_reference_unresolved',detail=str(error),
                        wheel_center_px=center.tolist(),heading=heading,mask=mask)
        if ref is None:
            return dict(ok=False,ready=False,forward=0.,left=0.,turn=0.,
                        reason='reference_lane_unresolved',wheel_center_px=center.tolist(),
                        heading=heading,mask=mask)
        gap=(np.array(beam['center'])-center/[w,h])-np.array([ref['beam_x']-ref['robot_x'],ref['beam_y']-ref['robot_y']])
        if heading is None:
            # Translation remains observable even when the four-corner yaw
            # estimator has no support. This is never a coarse motion permit;
            # only the separate six-model, stopped-RGB handoff may use it.
            return dict(ok=False,ready=False,forward=0.,left=0.,turn=0.,
                        reason='own_wheel_heading_unresolved',mask=mask,
                        wheel_center_px=center.tolist(),image_error=gap.tolist())
        angle=heading['angle_deg'];angle_ready=abs(angle)<=1.5
        lateral_ready=abs(gap[1])<=.003
        ready=gap[0]<=.065 and angle_ready and lateral_ready
        return dict(ok=True,ready=bool(ready),
            forward=min(.12,max(.03,float(gap[0]))) if angle_ready and lateral_ready and not ready else 0.,
            left=(-math.copysign(min(.05,max(.01,abs(float(gap[1])))),float(gap[1])) if angle_ready and not lateral_ready else 0.),
            turn=0. if angle_ready else math.copysign(min(.10,max(.01,.5*abs(math.radians(angle)))),angle),
            reason='RGB role-relative coarse approach',wheel_center_px=center.tolist(),
            image_error=gap.tolist(),heading=heading,mask=mask)
