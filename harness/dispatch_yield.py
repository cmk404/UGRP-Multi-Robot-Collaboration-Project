"""RGB-only observation and post-release yielding of a plan-selected robot."""
import math
import cv2
import numpy as np
from harness.camera_goal_transport import decode,wheel_heading
from harness.dispatch_pair_navigation import PairVision,track_wheel_motion,rotate,reanchor_wheel_geometry
from harness.known_map_navigation import pixel_to_world

def acquire_wheel_geometry(mask,hint):
    """Resolve a nearby chassis from four current wheel corners, not cargo pose.

    A released cargo fragment is only a rough search hint: its height and
    occlusion can move its apparent centre. Arm pixels must not define the
    chassis rectangle. Keep the existing strict corner geometry gates and
    reject a search that supports multiple spatially distinct chassis.
    """
    fits=[]
    for dx in range(-30,31,6):
        for dy in range(-30,31,6):
            for angle in range(-18,19,6):
                _,_,e=reanchor_wheel_geometry(mask,np.asarray(hint)+[dx,dy],angle)
                if e.get('accepted'):
                    center=np.array(e['observed_center_px']);refined=angle+2*e['heading_correction_deg']
                    for _ in range(6):
                        _,_,e=reanchor_wheel_geometry(mask,center,refined)
                        if not e.get('accepted'):break
                        center=np.array(e['observed_center_px']);refined+=2*e['heading_correction_deg']
                    if e.get('accepted'):fits.append([*center,refined])
    if len(fits)<4:raise ValueError('yield robot wheel geometry unresolved in RGB')
    fits=np.asarray(fits);observed=np.median(fits,axis=0)
    if np.linalg.norm(fits[:,:2]-observed[:2],axis=1).max()>8 or abs(fits[:,2]-observed[2]).max()>9:
        raise ValueError('yield robot wheel geometry ambiguous in RGB')
    center=observed[:2];angle=observed[2]
    for _ in range(6):
        _,_,e=reanchor_wheel_geometry(mask,center,angle)
        if not e.get('accepted'):raise ValueError('yield robot wheel geometry refinement unresolved in RGB')
        center=np.array(e['observed_center_px']);angle+=2*e['heading_correction_deg']
    return center,math.radians(angle),{'method':'current four-wheel corner consensus around last cargo RGB hint',
        'search_hint_px':list(map(float,hint)),'candidate_count':len(fits),'corner_geometry':e}

class WheelObserver:
    def __init__(self,static_map,hint_px):
        self.map=static_map;self.hint=np.array(hint_px,dtype=float)
        self.previous=None;self.center=None;self.angle=0.;self.initial_heading=None
    def observe(self,jpeg):
        frame=decode(jpeg)
        if self.previous is None:
            mask=PairVision._mask(None,frame)
            yy,xx=np.indices(mask.shape);heading=None
            for extent in (32,40,48):
                selected=mask.copy()
                selected[(abs(xx-self.hint[0])>extent)|(abs(yy-self.hint[1])>35)]=0
                heading=wheel_heading(selected,pixel_tolerance=2.)
                if heading is not None:break
            if heading is None:
                self.center,self.initial_heading,evidence=acquire_wheel_geometry(mask,self.hint)
            else:
                ys,xs=np.nonzero(selected)
                (x,y),_,_=cv2.minAreaRect(np.column_stack((xs,ys)).astype(np.float32))
                self.center=np.array([x,y]);self.initial_heading=-math.radians(heading['angle_deg'])
                evidence={'method':'own-motion/last-cargo RGB hint plus current four-wheel geometry'}
            px,py=np.rint(self.center).astype(int)
            crop=mask[py-30:py+31,px-30:px+31]
            if crop.shape!=(61,61):raise ValueError('yield wheel envelope outside camera')
            self.template=cv2.GaussianBlur(crop,(3,3),.7)
        else:
            center,self.angle,evidence=track_wheel_motion(self.previous,frame,self.center,self.angle,self.template)
            self.center=np.array(center)
        self.previous=frame.copy()
        return {'center_px':self.center.tolist(),'xy_m':list(pixel_to_world(self.center,frame.shape,self.map['top_camera'])),
            'heading_rad':self.initial_heading+math.radians(self.angle),'tracking':evidence,
            'source':'current RGB; nominal fixed camera feature plane, not simulator pose'}

class SoloYield:
    """Back away from a released box, clear the bay, then yield its resources."""
    def __init__(self,static_map,cargo_center_px):
        self.map=static_map
        self.vision=WheelObserver(static_map,np.array(cargo_center_px)-[50,0])
        self.points=None;self.index=0;self.confirmations=0;self.done=False;self.steps=0
    def decide(self,jpeg):
        obs=self.vision.observe(jpeg);xy=np.array(obs['xy_m']);self.steps+=1
        if self.steps>300:raise RuntimeError('RGB yield decision budget exhausted')
        if self.points is None:
            # Authored staging coordinates, verified by subsequent actual RGB.
            # Beam has not been delivered yet; this clears its destination bay.
            x=self.map['docks']['dock_a']['slots']['beam']['center_m'][0]-.10
            y=self.map['regions']['dispatch_apron']['center_m'][1]
            east=self.map['bounds_m'][1]-.22
            self.points=[[x,float(xy[1])],[x,y],[east,y]]
        error=np.array(self.points[self.index])-xy
        ready=np.linalg.norm(error)<.012
        self.confirmations=self.confirmations+1 if ready else 0
        if self.confirmations>=3:
            if self.index==len(self.points)-1:self.done=True
            else:self.index+=1;self.confirmations=0
        velocity=np.zeros(2) if ready else error/max(np.linalg.norm(error)/.06,.2)
        local=rotate(velocity,-obs['heading_rad'])
        action={'kind':'mecanum','forward':float(np.clip(local[0]/1.57,-.05,.06)),
            'left':float(np.clip(local[1]/1.18,-.06,.06)),'turn':0.,'duration_s':.2}
        return action,{'observation':obs,'waypoints_m':self.points,'index':self.index,
            'error_m':error.tolist(),'done':self.done,'phase':'released cargo; clearing the unload bay'}
