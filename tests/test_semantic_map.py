from __future__ import annotations
import unittest
from sim.masterpi_physics import MasterPiPhysicsWorld

class SemanticMapTests(unittest.TestCase):
    def test_camera_depth_builds_obstacles_and_exposes_zones(self):
        w=MasterPiPhysicsWorld(seed=11,width=320,height=240); w.set_speed_multiplier(3)
        try:
            for _ in range(3): w.observe_scene()
            m=w.semantic_map_public()
            self.assertGreater(len(m['obstacles']),5)
            self.assertEqual(len(m['drop_zones']),2)
            self.assertTrue(all(o['source']=='robot_camera_depth' for o in m['obstacles']))
            self.assertEqual(m['scan']['source'],'robot_camera')
        finally: w.close()
    def test_reset_clears_perceived_obstacles(self):
        w=MasterPiPhysicsWorld(seed=11,width=320,height=240); w.set_speed_multiplier(3)
        try:
            for _ in range(3): w.observe_scene()
            self.assertTrue(w.obstacle_cells)
            w.reset(12)
            self.assertFalse(w.obstacle_cells)
        finally: w.close()
    def test_same_seed_reset_reproduces_initial_layout(self):
        w=MasterPiPhysicsWorld(seed=123,width=320,height=240)
        try:
            first=w.state()["red_xyz"]
            w.move_base_relative(forward=0.05, frames=1)
            w.reset(123)
            second=w.state()["red_xyz"]
            self.assertEqual(first, second)
            self.assertEqual(w.state()["seed"], 123)
            w.reset(124)
            third=w.state()["red_xyz"]
            self.assertNotEqual(first[:2], third[:2])
            self.assertEqual(w.state()["seed"], 124)
        finally:
            w.close()

