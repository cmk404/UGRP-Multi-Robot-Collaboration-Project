"""Obstacle-map geometry and provider contract for physical MasterPi."""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Any, Protocol
from .real_geometry import CAMERA_HFOV_DEG,CAMERA_VFOV_DEG,BASE_CENTER,PULSE_PER_DEGREE,forward_kinematics,CAMERA_Z_OFFSET_CM

class MetricDepthProvider(Protocol):
    name:str
    def infer_metric_depth(self,jpeg:bytes)->Any: ...

@dataclass(frozen=True)
class ObstacleProviderStatus:
    status:str='unconfigured'; backend:str|None=None; metric_depth_available:bool=False; validated:bool=False; reason:str='no metric depth backend configured'
    def public(self)->dict[str,Any]: return {'status':self.status,'backend':self.backend,'metric_depth_available':self.metric_depth_available,'validated':self.validated,'reason':self.reason}

def unavailable_status(reason:str='no metric depth backend configured')->dict[str,Any]: return ObstacleProviderStatus(reason=reason).public()

def _camera_origin_and_basis(pose:dict[int,int]):
    arm=forward_kinematics(pose)
    if arm is None or 6 not in pose: return None
    yaw=math.radians((pose[6]-BASE_CENTER)/PULSE_PER_DEGREE)
    pitch=math.radians(arm.pitch_deg)
    origin=(arm.radius_cm/100*math.cos(yaw),arm.radius_cm/100*math.sin(yaw),(arm.height_cm+CAMERA_Z_OFFSET_CM)/100)
    forward=(math.cos(pitch)*math.cos(yaw),math.cos(pitch)*math.sin(yaw),math.sin(pitch))
    left=(-math.sin(yaw),math.cos(yaw),0.0)
    up=(-math.sin(pitch)*math.cos(yaw),-math.sin(pitch)*math.sin(yaw),math.cos(pitch))
    return origin,forward,left,up

def depth_pixel_to_base(pose:dict[int,int], *, nx:float, ny:float, depth_z_m:float):
    """Project optical-axis metric z-depth to +x forward,+y left,+z up."""
    basis=_camera_origin_and_basis(pose)
    if basis is None or not math.isfinite(depth_z_m) or depth_z_m<=0: return None
    origin,fwd,left,up=basis
    tan_h=(0.5-nx)*2*math.tan(math.radians(CAMERA_HFOV_DEG/2))
    tan_v=(0.5-ny)*2*math.tan(math.radians(CAMERA_VFOV_DEG/2))
    d=depth_z_m; l=tan_h*d; u=tan_v*d
    return tuple(origin[i]+fwd[i]*d+left[i]*l+up[i]*u for i in range(3))

def depth_to_obstacle_cells(depth, pose:dict[int,int], *, stride:int=16, cell_size_m:float=.08, min_height_m:float=.045, max_height_m:float=.55, min_range_m:float=.10, max_range_m:float=1.50, min_samples:int=2)->list[dict[str,Any]]:
    """Convert a validated metric depth map into coarse base-frame occupancy cells."""
    try: h,w=depth.shape[:2]
    except Exception: return []
    bins:dict[tuple[int,int],list[tuple[float,float,float]]]={}
    for y in range(stride//2,h,stride):
        ny=y/max(1,h-1)
        for x in range(stride//2,w,stride):
            nx=x/max(1,w-1)
            try: zdepth=float(depth[y,x])
            except Exception: continue
            p=depth_pixel_to_base(pose,nx=nx,ny=ny,depth_z_m=zdepth)
            if p is None: continue
            px,py,pz=p; rng=math.hypot(px,py)
            if not(min_range_m<=rng<=max_range_m and min_height_m<=pz<=max_height_m): continue
            key=(round(px/cell_size_m),round(py/cell_size_m)); bins.setdefault(key,[]).append(p)
    out=[]
    for key,pts in bins.items():
        if len(pts)<min_samples: continue
        n=len(pts); mx=sum(p[0] for p in pts)/n; my=sum(p[1] for p in pts)/n; mz=sum(p[2] for p in pts)/n
        out.append({'position_xy':[mx,my],'height_m':mz,'size_m':cell_size_m,'confidence':min(.95,.35+.08*n),'samples':n,'reference_frame':'robot_base_at_observation','source':'metric_monocular_depth'})
    out.sort(key=lambda o:math.hypot(*o['position_xy']))
    return out
