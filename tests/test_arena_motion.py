from dataclasses import replace
import math
import unittest

import numpy as np

from sim.solo_cargo_task import _navigation_bounds as solo_bounds, _transport_heading
from sim.warehouse_crew import _navigation_bounds as crew_bounds, _transport_frame, stage_crew
from sim.warehouse_mission import CargoSpec, ZoneSpec


class _Robot:
    def __init__(self, xy): self.xy=xy
    def base_xyz(self): return (*self.xy, 0.0)


class _World:
    warehouse_layout="arena"
    robot_ids=("r1", "r2")
    warehouse_zones={
        "A": ZoneSpec("A", "a", "blue", (-2.0, 1.0), (.8, .8), "0 0 1 1"),
        "B": ZoneSpec("B", "b", "green", (1.5, 3.8), (.9, .9), "0 1 0 1"),
        "C": ZoneSpec("C", "c", "yellow", (2.8, -.9), (.75, .75), "1 1 0 1"),
    }
    def robot(self, rid): return _Robot({"r1":(-2.5, 1.5), "r2":(3.2, -.4)}[rid])


def _spec(start=(-2., 1., .02), goal=(1.5, 3.8, .02)):
    return CargoSpec("box", "small_box", "red", "body", "joint", (.04,.04,.04), .03,
                     start, goal, .02, required_carriers=1, carriers=("r1",))


class ArenaMotionTests(unittest.TestCase):
    def test_transport_heading_uses_both_coordinates(self):
        direction, yaw = _transport_heading(_spec())
        expected = np.asarray((3.5, 2.8)); expected /= np.linalg.norm(expected)
        np.testing.assert_allclose(direction, expected)
        self.assertAlmostEqual(yaw, math.atan2(2.8, 3.5))
        forward, lateral = _transport_frame(_spec())
        np.testing.assert_allclose(forward, expected)
        self.assertAlmostEqual(float(np.dot(forward, lateral)), 0.0, places=7)

    def test_backward_and_lateral_destination_faces_naturally(self):
        spec = _spec(start=(2., 1., .02), goal=(-1., -3., .02))
        direction, yaw = _transport_heading(spec)
        self.assertLess(direction[0], 0)
        self.assertLess(direction[1], 0)
        self.assertAlmostEqual(yaw, math.atan2(-4., -3.))
        approach = np.asarray(spec.goal_xyz[:2])-direction*.165
        self.assertAlmostEqual(float(np.dot(approach-spec.goal_xyz[:2], direction)), -.165)

    def test_dynamic_bounds_include_square_zones_robots_and_target(self):
        world = _World(); target=(4.4, 4.6)
        for bounds in (crew_bounds(world,target), solo_bounds(world,target)):
            xmin,xmax,ymin,ymax=bounds
            self.assertLess(xmin, -2.8); self.assertGreater(xmax, 4.4)
            self.assertLess(ymin, -1.65); self.assertGreater(ymax, 4.7)

    def test_joint_staging_consumes_formation_chassis_yaw(self):
        class Crew:
            def __init__(self): self.yaws=[]
            def navigate(self,rid,target,activity,final_yaw=None):
                self.yaws.append((rid,final_yaw)); return (target,)
            def start_scout(self): pass
            def wait_for(self,robot_ids): pass
        crew=Crew()
        class World:
            _warehouse_crew=crew; _mixed_engine=object(); robot_ids=("r1","r2","r3")
            def _warehouse_inward_formation(self,spec,reach):
                return {rid:{"x":index,"y":index+.5,"chassis_yaw":yaw}
                        for index,(rid,yaw) in enumerate((("r1",1.2),("r2",-1.4)))}
            def _record_warehouse_phase(self,*args,**kwargs): pass
        spec=replace(_spec(),required_carriers=2,carriers=("r1","r2"),scout="r3")
        stage_crew(World(),spec,.2)
        self.assertEqual(crew.yaws,[("r1",1.2),("r2",-1.4)])


if __name__ == "__main__":
    unittest.main()
