import unittest
from harness.real_map import snapshot_from_memory
class RealMapTests(unittest.TestCase):
    def test_only_metric_base_frame_objects_enter_snapshot(self):
        m={
            'red':{'metric_position_available':True,'reference_frame':'robot_base_at_observation','position_xy':[.3,.1]},
            'blue':{'metric_position_available':False,'image_xy':[.5,.5]},
        }
        s=snapshot_from_memory(3,m).public()
        self.assertIn('red',s['objects']); self.assertNotIn('blue',s['objects'])
        self.assertFalse(s['persistent_world_map']); self.assertFalse(s['odometry_available'])
if __name__=='__main__': unittest.main()
