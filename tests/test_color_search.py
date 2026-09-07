from __future__ import annotations
import math, unittest
from sim.masterpi_physics import MasterPiPhysicsWorld

class ColorSearchTests(unittest.TestCase):
    def test_each_color_searches_its_own_detector(self):
        for action,color in [('search_red','red'),('search_blue','blue'),('search_yellow','yellow')]:
            with self.subTest(action=action):
                w=MasterPiPhysicsWorld(seed=11); w.set_speed_multiplier(3)
                before=w.base_yaw; r=w.act(action); mem=w.spatial_memory_public()
                self.assertTrue(r.ok, r.reason)
                self.assertIn(color, mem)
                self.assertTrue(mem[color]['visible'])
                self.assertLess(abs(w.base_yaw-before), math.radians(1.0))
                w.close()
