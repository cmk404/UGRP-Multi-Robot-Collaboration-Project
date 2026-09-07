import io
import unittest

import numpy as np
from PIL import Image

from harness.perception import detect_scene_bytes
from harness.state import StateEstimator


def jpeg_with_blocks(colors):
    a=np.zeros((240,320,3),dtype=np.uint8)+30
    for (x0,y0,x1,y1), rgb in colors:
        a[y0:y1,x0:x1]=rgb
    out=io.BytesIO(); Image.fromarray(a).save(out,format='JPEG',quality=95); return out.getvalue()


class RealSceneMemoryTests(unittest.TestCase):
    def test_detects_three_colors_from_one_frame(self):
        j=jpeg_with_blocks([
            ((35,120,75,165),(230,15,15)),
            ((135,115,175,155),(235,185,20)),
            ((235,125,275,165),(15,35,230)),
        ])
        d=detect_scene_bytes(j)
        self.assertIsNotNone(d)
        self.assertTrue(all(d[c]['visible'] for c in ('red','yellow','blue')))

    def test_real_memory_is_explicitly_non_metric_and_persists(self):
        est=StateEstimator()
        est.update_scene_memory({
            'red': {'visible':True,'cx':.3,'cy':.6,'area_ratio':.02},
            'yellow': {'visible':False,'pixels':0},
            'blue': {'visible':False,'pixels':0},
        })
        mem=est.state.spatial_memory['red']
        self.assertFalse(mem['metric_position_available'])
        self.assertFalse(mem['calibrated'])
        self.assertEqual(mem['source'],'robot_camera_rgb')
        est.update_scene_memory({c:{'visible':False,'pixels':0} for c in ('red','yellow','blue')})
        self.assertFalse(est.state.spatial_memory['red']['visible'])
        self.assertGreaterEqual(est.state.spatial_memory['red']['age_observations'],1)


if __name__=='__main__':
    unittest.main()

class RealMetricSceneMemoryTests(unittest.TestCase):
    def test_metric_patch_promotes_camera_memory(self):
        from harness.state import StateEstimator
        est=StateEstimator()
        scene={
            'red':{'visible':True,'cx':.5,'cy':.6,'area_ratio':.01},
            'yellow':{'visible':False,'pixels':0},
            'blue':{'visible':False,'pixels':0},
        }
        metric={'red':{
            'position_xy':[.42,.03], 'distance_m':.421,
            'bearing_deg':4.1, 'metric_position_available':True,
            'calibrated':True,'reference_frame':'robot_base_at_observation',
            'calibration_id':'test','pose_stable':True,
        }}
        est.update_scene_memory(scene,metric_estimates=metric)
        red=est.state.spatial_memory['red']
        self.assertTrue(red['metric_position_available'])
        self.assertEqual(red['position_xy'],[.42,.03])
        self.assertEqual(red['reference_frame'],'robot_base_at_observation')
        self.assertEqual(red['source'],'robot_camera_rgb')

class RealObserveToolResultTests(unittest.TestCase):
    def test_observe_tool_result_updates_metric_memory(self):
        from harness.state import StateEstimator
        e=StateEstimator()
        e.update_tool_result({
          'skill':'observe_scene','outcome_status':'ACHIEVED',
          'detections':{
            'red':{'visible':True,'cx':.5,'cy':.6,'area_ratio':.01},
            'blue':{'visible':False,'pixels':0},'yellow':{'visible':False,'pixels':0}},
          'metric_estimates':{'red':{
            'metric_position_available':True,'position_xy':[.31,.02],
            'distance_m':.311,'bearing_deg':3.0,'calibrated':True,
            'reference_frame':'robot_base_at_observation','calibration_id':'test'}}})
        self.assertEqual(e.state.spatial_memory['red']['position_xy'],[.31,.02])
        self.assertEqual(e.state.last_action.name,'observe_scene')
