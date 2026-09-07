import math
import unittest

from harness.real_geometry import (
    CAMERA_HFOV_DEG, CALIBRATION_ID, forward_kinematics,
    metric_memory_entry, project_detection, project_floor_pixel,
)

SEARCH = {1:2000,3:740,4:2320,5:1320,6:1500}

class RealGeometryTests(unittest.TestCase):
    def test_forward_kinematics_is_finite(self):
        p=forward_kinematics(SEARCH)
        self.assertIsNotNone(p)
        self.assertTrue(math.isfinite(p.radius_cm))
        self.assertTrue(math.isfinite(p.height_cm))
        self.assertLess(p.pitch_deg,0)
    def test_center_floor_pixel_projects_forward(self):
        p=project_floor_pixel(SEARCH,nx=.5,ny=.8)
        self.assertIsNotNone(p)
        self.assertGreater(p.x_forward_m,0)
        self.assertAlmostEqual(p.y_left_m,0,places=5)
        self.assertEqual(p.calibration_id,CALIBRATION_ID)
    def test_pan_left_makes_positive_left_coordinate(self):
        left=dict(SEARCH); left[6]=1700
        p=project_floor_pixel(left,nx=.5,ny=.8)
        self.assertIsNotNone(p)
        self.assertGreater(p.y_left_m,0)
        self.assertGreater(p.bearing_deg,0)
    def test_image_left_makes_positive_left_coordinate(self):
        p=project_floor_pixel(SEARCH,nx=.25,ny=.8)
        self.assertIsNotNone(p)
        self.assertGreater(p.bearing_deg,0)
        self.assertAlmostEqual(p.bearing_deg,CAMERA_HFOV_DEG*.25,places=5)
    def test_bbox_bottom_is_used(self):
        d={'visible':True,'cx':.5,'cy':.5,'bbox':[.45,.4,.55,.8]}
        a=project_detection(SEARCH,d)
        b=project_floor_pixel(SEARCH,nx=.5,ny=.8)
        self.assertIsNotNone(a); self.assertIsNotNone(b)
        self.assertAlmostEqual(a.range_m,b.range_m,places=9)
    def test_metric_memory_requires_stable_pose(self):
        d={'visible':True,'cx':.5,'cy':.5,'bbox':[.45,.4,.55,.8]}
        self.assertIsNone(metric_memory_entry(SEARCH,d,pose_age_s=.3,pose_stable=False))
        m=metric_memory_entry(SEARCH,d,pose_age_s=.3,pose_stable=True)
        self.assertTrue(m['metric_position_available'])
        self.assertEqual(m['reference_frame'],'robot_base_at_observation')
    def test_invalid_upward_ray_is_rejected(self):
        weird={1:2000,3:2200,4:800,5:600,6:1500}
        self.assertIsNone(project_floor_pixel(weird,nx=.5,ny=.1))

if __name__=='__main__': unittest.main()
