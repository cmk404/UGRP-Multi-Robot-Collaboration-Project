"""Dispatch adapters for the loaded-pair RGB navigator.

Authored geometry + RGB estimates only. No episode setup or referee input.
"""
import copy,hashlib,math
import cv2
import numpy as np
from harness.camera_goal_transport import decode
from harness.dispatch_skill_binding import beam_feature,pixel_from_map
from harness.known_map_navigation import pixel_to_world

FOOTPRINT={'half_forward_m':.20,'half_lateral_m':.47,'margin_m':.025}

def visual_barriers(jpeg,static):
    frame=decode(jpeg);hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.array([3,170,45],np.uint8),np.array([35,255,255],np.uint8))
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    n,_,stats,_=cv2.connectedComponentsWithStats(mask)
    result=[]
    camera=static['top_camera'];cx,cy,cz=camera['position_m'];h,w=frame.shape[:2]
    for i in range(1,n):
        x,y,bw,bh,area=map(int,stats[i])
        if area<4000 or min(bw,bh)<30:continue
        corners=[]
        # Unknown elevated surfaces: union projection over the authored sensor
        # model's conservative 0..0.25m range, never a measured object height.
        for z in (0.,.25):
            scale=2*(cz-z)*math.tan(math.radians(camera['fov_y_deg'])/2)/h
            corners.extend([[cx+(u-(w-1)/2)*scale,cy-(v-(h-1)/2)*scale]
                for u in (x,x+bw) for v in (y,y+bh)])
        lo=np.min(corners,axis=0)-.02;hi=np.max(corners,axis=0)+.02
        result.append({'id':'rgb_barrier_'+str(len(result)),
            'center_m':((lo+hi)/2).tolist(),'half_extents_m':((hi-lo)/2).tolist(),
            'height_m':.25,'source':'conservative TOP RGB projection; unknown actual height',
            'bbox_px':[x,y,bw,bh],'image_sha256':hashlib.sha256(jpeg).hexdigest()})
    return result

def navigation_map(bindings,jpeg,*,other_robot_center_px=None,planned_box_completion=False):
    static=bindings.static_map
    data={'schema':'ugrp.dispatch_pair_navigation.v1','top_camera':copy.deepcopy(static['top_camera']),
        'bounds_m':copy.deepcopy(static['bounds_m']),'footprint':dict(FOOTPRINT),
        'obstacles':copy.deepcopy(static['obstacles']),'grid_m':.06,
        'goal':{'center_m':copy.deepcopy(static['docks'][bindings.plan['dock']]['slots']['beam']['center_m']),
                'relative_yaw_deg':0.,'placement_tolerance_m':[.06,.015]},'route_name':bindings.tasks['beam']['route'],
        'source_map_sha256':hashlib.sha256(__import__('json').dumps(static,sort_keys=True).encode()).hexdigest()}
    # Retain wall boxes: the bounds describe wall centres, not free interior.
    data['obstacles']+=visual_barriers(jpeg,static)
    if planned_box_completion:
        slot_pixel=pixel_from_map(static['docks'][bindings.plan['dock']]['slots']['box']['center_m'],static,decode(jpeg).shape)
        data['obstacles'].append({'id':'planned_box_slot','center_m':list(pixel_to_world(slot_pixel,decode(jpeg).shape,static['top_camera'])),
            'half_extents_m':[.05,.06],'height_m':.10,'source':'conditional authored destination; fresh RGB validation required after box job'})
        yield_xy=[static['bounds_m'][1]-.22,static['regions']['dispatch_apron']['center_m'][1]]
        source='conditional yield goal; not an observed position'
        park=bindings.tasks['box'].get('park')
        if park is not None:
            # Planned navigation: the models chose where the box robot waits.
            from harness.map_goto import resolve_destination
            try:
                yield_xy=resolve_destination(static,park)['xy_m']
                source='conditional model-chosen park place; not an observed position'
            except ValueError:
                pass  # the separate park check rejects an unresolvable place
        data['obstacles'].append({'id':'planned_yield_pose','center_m':yield_xy,
            'half_extents_m':[.14,.14],'height_m':.35,'source':source})
    else:
        data['obstacles']+=visual_boxes(jpeg,static)
        if other_robot_center_px is not None:data['obstacles'].append(occupied_robot(other_robot_center_px,static))
    for terrain in static['terrain']:
        data['obstacles'].append({**copy.deepcopy(terrain),'id':'unvalidated_'+terrain['id']})
    if any(o['id']=='service_island' for o in static['obstacles']):
        other='south' if data['route_name']=='north' else 'north'
        gate=static['regions'][other+'_gate']
        data['obstacles'].append({'id':'unselected_'+other,'center_m':gate['center_m'][:],
            'half_extents_m':[.18,.35],'height_m':.25,'source':'agreed route constraint'})
    # Floor slot as seen in fixed RGB; carrier observer uses a nominal feature
    # plane. This is a calibration transform, not a live measured height.
    pixel=pixel_from_map(data['goal']['center_m'],static,decode(jpeg).shape)
    data['goal']['center_m']=list(pixel_to_world(pixel,decode(jpeg).shape,static['top_camera']))
    # Use the permitted placement region to leave room for the adjacent solo
    # carrier. Exact-centre placement can obstruct that carrier's rear wheels.
    slots=static['docks'][bindings.plan['dock']]['slots']
    direction=math.copysign(1.,slots['box']['center_m'][0]-slots['beam']['center_m'][0])
    data['goal']['preferred_offset_m']=[-direction*data['goal']['placement_tolerance_m'][0],0.]
    data['goal']['placement_preference']='leave clearance toward adjacent authored box slot'
    return data

def solo_gate(static,route,jpeg):
    """Fit the fixed-heading solo envelope into the selected corridor.

    Uses authored geometry and conservative image-only obstacle projections.
    A ridge is treated as unvalidated terrain, never silently assumed passable.
    """
    gate=static['regions'][route+'_gate']['center_m'][:]
    island=next((o for o in static['obstacles'] if o['id']=='service_island'),None)
    if island is None:return gate
    ymin,ymax=static['bounds_m'][2:]
    cy=island['center_m'][1];hy=island['half_extents_m'][1]
    intervals=[(cy+hy,ymax-.025)] if route=='north' else [(ymin+.025,cy-hy)]
    blockers=[o for o in static['obstacles'] if not o['id'].startswith('wall') and o['id']!='service_island']
    blockers+=static['terrain']+visual_barriers(jpeg,static)
    for o in blockers:
        if abs(o['center_m'][0]-gate[0])>o['half_extents_m'][0]+.32:continue
        a=o['center_m'][1]-o['half_extents_m'][1];b=o['center_m'][1]+o['half_extents_m'][1]
        remaining=[]
        for lo,hi in intervals:
            if b<=lo or a>=hi:remaining.append((lo,hi))
            else:
                if lo<a:remaining.append((lo,a))
                if b<hi:remaining.append((b,hi))
        intervals=remaining
    intervals=[(lo,hi) for lo,hi in intervals if hi-lo>=.32]
    if not intervals:raise RuntimeError('BOX_ROUTE_UNSUPPORTED: '+route+' lacks 0.32m conservative clearance or crosses unvalidated terrain')
    lo,hi=max(intervals,key=lambda x:x[1]-x[0]);gate[1]=max(lo+.16,min(hi-.16,gate[1]))
    return gate


def visual_boxes(jpeg,static):
    frame=decode(jpeg);hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.array([80,125,35],np.uint8),np.array([102,255,255],np.uint8))
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    n,_,stats,centers=cv2.connectedComponentsWithStats(mask)
    objects=[]
    for i in range(1,n):
        x,y,w,h,area=map(int,stats[i])
        if not (50<=area<=600 and 5<=min(w,h) and max(w,h)<40 and .4<=w/h<=2.5):continue
        xy=pixel_to_world(centers[i],frame.shape,static['top_camera'])
        # Bound a small visible cargo plus nominal-plane projection uncertainty.
        objects.append({'id':'rgb_box_'+str(len(objects)),'center_m':list(xy),
            'half_extents_m':[.05,.06],'height_m':.10,'source':'current cyan cargo RGB with conservative projection bound'})
    return objects


def occupied_robot(center_px,static,shape=(720,960,3)):
    return {'id':'rgb_other_robot','center_m':list(pixel_to_world(center_px,shape,static['top_camera'])),
        'half_extents_m':[.14,.14],'height_m':.35,'source':'current wheel-envelope RGB observation'}
