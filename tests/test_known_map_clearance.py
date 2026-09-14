from pathlib import Path

import numpy as np

from harness.known_map_navigation import plan_grid_path
from sim.authored_navigation_map import load_map


def test_slalom_prefers_clearance_instead_of_the_shortest_wall_edge():
    data=load_map(Path(__file__).resolve().parents[1]/'maps/navigation/slalom.json')
    path=plan_grid_path(data,(-.5,-2.55),(1.55,-2.55))
    assert path is not None
    points=[]
    for a,b in zip(path,path[1:]):
        points.extend(np.linspace(a,b,100))
    # Across the first wall's width, stay at least 5 cm beyond the hard
    # inflated top edge (-1.78 m), addressing the observed edge-hugging failure.
    crossing=[y for x,y in points if .05 <= x <= .25]
    assert crossing and min(crossing) >= -1.73


def test_clearance_preference_does_not_open_a_physically_narrow_passage():
    data=load_map(Path(__file__).resolve().parents[1]/'maps/navigation/narrow.json')
    assert plan_grid_path(data,(-.5,-2.55),(1.55,-2.55)) is None
