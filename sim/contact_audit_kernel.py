"""Fast read-only contact flag for the dispatch evaluation sink.

The geom membership table is fixed at scene construction.  MuJoCo still
computes every contact at every physics step; this helper only replaces the
Python generator used to count penetrating robot/obstacle contact ticks.
"""
from __future__ import annotations

from collections.abc import Collection

import numpy as np


class ContactAuditKernel:
    def __init__(self, ngeom: int, robot_ids: Collection[int], obstacle_ids: Collection[int]) -> None:
        self._pairs = np.zeros((ngeom, ngeom), dtype=np.bool_)
        if len(robot_ids) and len(obstacle_ids):
            robots = np.fromiter(robot_ids, dtype=np.intp)
            obstacles = np.fromiter(obstacle_ids, dtype=np.intp)
            self._pairs[np.ix_(robots, obstacles)] = True
            self._pairs[np.ix_(obstacles, robots)] = True

    def has_penetrating_contact(self, data) -> bool:
        if data.ncon == 0:
            return False
        contacts = data.contact
        return bool(np.any(self._pairs[contacts.geom1, contacts.geom2] & (contacts.dist < 0.0)))
