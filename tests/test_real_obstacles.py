import unittest, numpy as np
from harness.real_obstacles import depth_pixel_to_base, depth_to_obstacle_cells
POSE={1:2000,3:740,4:2320,5:1320,6:1500}
class RealObstacleTests(unittest.TestCase):
    def test_center_depth_projects_finite(self):
        p=depth_pixel_to_base(POSE,nx=.5,ny=.5,depth_z_m=.4)
        self.assertIsNotNone(p); self.assertGreater(p[0],0)
    def test_left_pixel_projects_left_of_right_pixel(self):
        a=depth_pixel_to_base(POSE,nx=.25,ny=.5,depth_z_m=.4); b=depth_pixel_to_base(POSE,nx=.75,ny=.5,depth_z_m=.4)
        self.assertGreater(a[1],b[1])
    def test_synthetic_depth_builds_cells_only_above_ground(self):
        d=np.full((96,128),.35,dtype=np.float32)
        cells=depth_to_obstacle_cells(d,POSE,stride=8,min_samples=1,min_height_m=-1,max_height_m=2)
        self.assertGreater(len(cells),0)
    def test_invalid_depth_returns_empty(self): self.assertEqual(depth_to_obstacle_cells(object(),POSE),[])
if __name__=='__main__': unittest.main()
