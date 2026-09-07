import math,unittest
from harness.real_odometry import transform_base_point_to_world, unavailable_status
class RealOdometryTests(unittest.TestCase):
    def test_unavailable_is_explicit(self):
        s=unavailable_status(); self.assertFalse(s['available']); self.assertFalse(s['calibrated'])
    def test_se2_transform(self):
        p=transform_base_point_to_world([1,0],(2,3,math.pi/2))
        self.assertAlmostEqual(p[0],2,places=8); self.assertAlmostEqual(p[1],4,places=8)
    def test_no_pose_means_no_world_coordinate(self):
        self.assertIsNone(transform_base_point_to_world([.2,.1],None))
if __name__=='__main__':unittest.main()
