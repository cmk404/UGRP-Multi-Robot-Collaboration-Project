import unittest
from harness.state import StateEstimator
class RealSemanticMapTests(unittest.TestCase):
    def test_metric_blue_yellow_become_stack_zones(self):
        e=StateEstimator()
        scene={
          'red':{'visible':False,'pixels':0},
          'blue':{'visible':True,'cx':.5,'cy':.5,'area_ratio':.01},
          'yellow':{'visible':True,'cx':.6,'cy':.5,'area_ratio':.01},
        }
        metric={
          'blue':{'metric_position_available':True,'position_xy':[.4,-.1],'calibrated':True,'reference_frame':'robot_base_at_observation'},
          'yellow':{'metric_position_available':True,'position_xy':[.5,.2],'calibrated':True,'reference_frame':'robot_base_at_observation'},
        }
        e.update_scene_memory(scene,metric_estimates=metric)
        sm=e.state.semantic_map
        self.assertFalse(sm['persistent_world_map'])
        self.assertFalse(sm['odometry_available'])
        self.assertEqual(len(sm['drop_zones']),2)
        self.assertEqual(sm['obstacle_provider']['status'],'unconfigured')
        self.assertEqual(sm['obstacles'],[])
    def test_image_only_landmark_does_not_create_fake_zone(self):
        e=StateEstimator()
        scene={'red':{'visible':False,'pixels':0},'blue':{'visible':True,'cx':.5,'cy':.5,'area_ratio':.01},'yellow':{'visible':False,'pixels':0}}
        e.update_scene_memory(scene)
        self.assertEqual(e.state.semantic_map['drop_zones'],[])
if __name__=='__main__': unittest.main()
