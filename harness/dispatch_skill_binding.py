"""Bind a committed allocation to existing camera skills, never to simulator IDs.

The legacy pair names are model slots (bottom r1 / top r3), not live robots.
Static capabilities and image transforms are explicit and auditable. No fixture
poses are read here, and no failed model prediction is replaced by a raw LLM action.
"""
from __future__ import annotations
import copy
from functools import lru_cache
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
                if obj=='box' and self.tasks['beam']['id'] not in self.finished:return False
                resources=['dispatch_apron']
            elif stage in ('GRASP','TRANSIT'):
                resources=[self.static_map['routes'][task['route']]['resource']]
            else:resources=[]
            if any(self.locks.get(r,task['id'])!=task['id'] for r in resources):return False
            for r in resources:
                if r not in self.locks:self.resource_events.append({'event':'acquire','object':obj,'resource':r,'stage':stage})
                self.locks[r]=task['id']
            return True
        if (stage != 'APPROACH' or self.cluttered) and any(dep not in self.finished for dep in task['after']):
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
    def __init__(self, bindings, obj):
        self.route_overlap=bool(getattr(bindings,'route_overlap',False))
        self._permission=getattr(bindings,'permission',None)
        if self.route_overlap and not callable(self._permission):
            raise ValueError('overlap route requires live permission check')
        self.map=bindings.static_map;self.obj=obj
        self.task=bindings.tasks[obj];self.dock=bindings.plan['dock']
        self.points=None;self.index=0;self.confirmations=0
        self.box_center=None;self.box_delta=np.zeros(2);self.box_previous=None
        self.box_background=None;self.box_origin=None;self.box_background_sha=None
        self.box_pan_frames={}
        from harness.dispatch_beam_tracker import CarriedBeamTracker
        self.beam_tracker=CarriedBeamTracker()

    def record_attachment_top(self,phase,jpeg):
        from harness.dispatch_box_identity import PAN_PHASES
        if self.obj!='box' or phase not in PAN_PHASES:return
        if phase=='verify_lift':self.box_pan_frames={}
        self.box_pan_frames[phase]=jpeg

    def observe(self,jpeg):
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
            for saturation in levels:
                mask=cv2.inRange(hsv,np.array((80,saturation,35),np.uint8),np.array((102,255,255),np.uint8))
                if suppress_background:mask[~foreground]=0
                if self.box_center is None:
                    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
                n,_,stats,centers=cv2.connectedComponentsWithStats(mask)
                choices=[i for i in range(1,n) if (25 if self.box_center is None else 8) <= stats[i,4] <= 600
                         and max(stats[i,2:4]) < 40]
                if binding is not None:
                    choices=[i for i in choices if np.linalg.norm(centers[i]-bound_center)<1.]
                if self.box_center is not None:
                    predicted=self.box_center+np.clip(self.box_delta,-15,15)
                    choices=sorted((i for i in choices if np.linalg.norm(centers[i]-predicted)<=8),
                                   key=lambda i:np.linalg.norm(centers[i]-predicted))
                    if len(choices)>1 and np.linalg.norm(centers[choices[1]]-predicted)-np.linalg.norm(centers[choices[0]]-predicted)>5:
                        choices=choices[:1]
                if choices:break
            tracking={'method':'cyan component','min_saturation':saturation}
            if binding is not None:tracking['initial_identity']=binding
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
            if self.box_origin is None:self.box_origin=center.copy()
            tracking['static_background']={'reference_sha256':self.box_background_sha,
                'active':bool(suppress_background),'rgb_difference_threshold':20,
                'initial_occupied_radius_px':30,'origin_px':self.box_origin.tolist()}
            self.box_delta=np.zeros(2) if self.box_center is None else center-self.box_center
            self.box_center=center.copy();self.box_previous=frame.copy()

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
        if heading is None:
            return dict(ok=False,ready=False,forward=0.,left=0.,turn=0.,
                        reason='own_wheel_heading_unresolved',mask=mask)
        center=np.array([xs.mean(),ys.mean()]);self.centers[slot]=center
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
        angle=heading['angle_deg'];angle_ready=abs(angle)<=1.5
        lateral_ready=abs(gap[1])<=.003
        ready=gap[0]<=.065 and angle_ready and lateral_ready
        return dict(ok=True,ready=bool(ready),
            forward=min(.12,max(.03,float(gap[0]))) if angle_ready and lateral_ready and not ready else 0.,
            left=(-math.copysign(min(.05,max(.01,abs(float(gap[1])))),float(gap[1])) if angle_ready and not lateral_ready else 0.),
            turn=0. if angle_ready else math.copysign(min(.10,max(.01,.5*abs(math.radians(angle)))),angle),
            reason='RGB role-relative coarse approach',wheel_center_px=center.tolist(),
            image_error=gap.tolist(),heading=heading,mask=mask)
