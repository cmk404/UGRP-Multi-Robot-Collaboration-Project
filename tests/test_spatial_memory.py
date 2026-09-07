from __future__ import annotations
import math
import unittest
import numpy as np
from sim.masterpi_physics import MasterPiPhysicsWorld

class SpatialMemoryTests(unittest.TestCase):
    def setUp(self):
        self.w = MasterPiPhysicsWorld(seed=11)
        self.w.set_speed_multiplier(3)
    def tearDown(self):
        self.w.close()
    def test_one_view_localizes_multiple_objects(self):
        self.w.observe_scene()
        mem = self.w.spatial_memory_public()
        self.assertIn('blue', mem)
        self.assertIn('yellow', mem)
        for color, truth_name in [('blue','blue_block'),('yellow','yellow_block')]:
            est=np.array(mem[color]['position_xy'],dtype=float)
            truth=self.w._body_pos(truth_name)[:2]
            self.assertLess(float(np.linalg.norm(est-truth)), .018)
    def test_head_first_search_avoids_chassis_yaw_when_target_is_reachable(self):
        self.w.reset(block_xy=(.45,.38))
        before=self.w.base_yaw
        result=self.w.act('search')
        self.assertTrue(result.ok)
        self.assertLess(abs(self.w.base_yaw-before), math.radians(1.0))
        self.assertGreater(abs(self.w._joint_qpos('arm_yaw')), math.radians(10.0))
    def test_memory_survives_target_leaving_view(self):
        self.w.observe_scene()
        before=dict(self.w.spatial_memory_public()['blue'])
        self.w._set_search_head_yaw(1.2, 80)
        self.w.observe_scene()
        after=self.w.spatial_memory_public()['blue']
        self.assertEqual(before['position_xy'], after['position_xy'])
        self.assertGreaterEqual(after['seen_count'], before['seen_count'])

if __name__ == '__main__': unittest.main()
