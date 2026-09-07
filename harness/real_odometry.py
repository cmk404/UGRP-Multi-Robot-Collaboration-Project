"""Odometry contract for the physical MasterPi semantic map.

The current ugrp1 chassis has no calibrated encoder/visual odometry source in
UGRP.  This module deliberately represents that fact instead of integrating
motor command duration into fake world coordinates.  A future provider can be
plugged in without changing the semantic-map schema.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Protocol

class OdometryProvider(Protocol):
    name: str
    def pose(self) -> tuple[float,float,float] | None: ...

@dataclass(frozen=True)
class OdometryStatus:
    available: bool = False
    calibrated: bool = False
    provider: str | None = None
    frame: str = 'robot_base_at_observation'
    reason: str = 'no calibrated chassis odometry provider configured'
    def public(self)->dict[str,Any]:
        return {
            'available':self.available,'calibrated':self.calibrated,
            'provider':self.provider,'frame':self.frame,'reason':self.reason,
        }

def unavailable_status(reason:str='no calibrated chassis odometry provider configured')->dict[str,Any]:
    return OdometryStatus(reason=reason).public()


def transform_base_point_to_world(
    xy:tuple[float,float]|list[float],
    base_pose:tuple[float,float,float]|None,
)->list[float]|None:
    """Transform +x-forward,+y-left point using a calibrated SE(2) base pose."""
    if base_pose is None or len(xy)!=2: return None
    import math
    x,y=float(xy[0]),float(xy[1]); bx,by,yaw=map(float,base_pose)
    c,s=math.cos(yaw),math.sin(yaw)
    return [bx+c*x-s*y, by+s*x+c*y]
