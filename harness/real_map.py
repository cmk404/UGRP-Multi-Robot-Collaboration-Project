"""Semantic-map bookkeeping for physical MasterPi observations.

Until calibrated chassis odometry is available, metric observations are stored
as snapshots in the robot-base frame at observation time.  This module makes
that limitation explicit and provides a future odometry transform seam.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any

@dataclass
class BaseFrameSnapshot:
    observation_id: int
    objects: dict[str, dict[str, Any]] = field(default_factory=dict)
    obstacle_cells: list[dict[str, Any]] = field(default_factory=list)
    frame: str = 'robot_base_at_observation'
    odometry_available: bool = False

    def public(self) -> dict[str, Any]:
        return {
            'observation_id':self.observation_id,
            'frame':self.frame,
            'odometry_available':self.odometry_available,
            'persistent_world_map':False,
            'objects':self.objects,
            'obstacle_cells':self.obstacle_cells,
        }


def snapshot_from_memory(observation_id:int,memory:dict[str,dict[str,Any]]) -> BaseFrameSnapshot:
    objects={}
    for color,entry in memory.items():
        if not isinstance(entry,dict) or entry.get('metric_position_available') is not True:
            continue
        if entry.get('reference_frame')!='robot_base_at_observation':
            continue
        objects[color]={k:entry.get(k) for k in (
            'position_xy','height_m','distance_m','bearing_deg','confidence','relation',
            'calibration_id','reference_frame') if k in entry}
    return BaseFrameSnapshot(observation_id=observation_id,objects=objects)
