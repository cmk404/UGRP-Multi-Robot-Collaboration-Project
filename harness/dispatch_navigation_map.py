"""Dispatch adapters for the loaded-pair RGB navigator.

Authored geometry + RGB estimates only. No episode setup or referee input.
"""
import copy,hashlib,math
import cv2
import numpy as np
from harness.camera_goal_transport import decode
from harness.dispatch_skill_binding import beam_feature,pixel_from_map
from harness.known_map_navigation import pixel_to_world

FOOTPRINT={'half_forward_m':.20,'half_lateral_m':.445,'margin_m':.025}

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

def navigation_map(bindings,jpeg):
    static=bindings.static_map
    data={'schema':'ugrp.dispatch_pair_navigation.v1','top_camera':copy.deepcopy(static['top_camera']),
        'bounds_m':copy.deepcopy(static['bounds_m']),'footprint':dict(FOOTPRINT),
        'obstacles':copy.deepcopy(static['obstacles']),'grid_m':.06,
        'goal':{'center_m':copy.deepcopy(static['docks'][bindings.plan['dock']]['slots']['beam']['center_m']),
                'relative_yaw_deg':0.},'route_name':bindings.tasks['beam']['route'],
        'source_map_sha256':hashlib.sha256(__import__('json').dumps(static,sort_keys=True).encode()).hexdigest()}
    # Retain wall boxes: the bounds describe wall centres, not free interior.
    data['obstacles']+=visual_barriers(jpeg,static)
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
    return data
