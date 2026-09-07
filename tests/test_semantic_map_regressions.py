from __future__ import annotations
import math,random,unittest
import numpy as np
from sim.masterpi_physics import MasterPiPhysicsWorld
from scripts.stress_semantic_map import set_scenario, static_obstacle_boxes, obstacle_error_to_static_boxes

class SemanticMapRegressionTests(unittest.TestCase):
    def _world(self,seed):
        w=MasterPiPhysicsWorld(seed=seed,width=320,height=240);w.set_speed_multiplier(3);set_scenario(w,random.Random(seed*1000003+17));return w
    def test_same_color_delivery_regions_are_not_blocks(self):
        # These seeds previously selected the large blue/yellow delivery/crate regions.
        for seed in (1001,1002,1004,1005):
            with self.subTest(seed=seed):
                w=self._world(seed)
                try:
                    r=w.act('scan_world');self.assertTrue(r.ok)
                    mem=w.spatial_memory_public()
                    for c in ('red','blue','yellow'):
                        self.assertIn(c,mem)
                        truth=w._body_pos(c+'_block')[:2]
                        self.assertLess(np.linalg.norm(np.asarray(mem[c]['position_xy'])-truth),.055)
                finally:w.close()
    def test_edge_sighting_is_refined_without_chassis_rotation(self):
        # Seed 3003 sees yellow at cx~0.02 in the coarse survey; refinement must re-center it.
        w=self._world(3003)
        try:
            y0=w.base_yaw;r=w.act('scan_world');self.assertTrue(r.ok)
            dy=(w.base_yaw-y0+math.pi)%(2*math.pi)-math.pi
            self.assertLess(abs(dy),math.radians(1.0))
            mem=w.spatial_memory_public();self.assertIn('yellow',mem)
            self.assertLess(np.linalg.norm(np.asarray(mem['yellow']['position_xy'])-w._body_pos('yellow_block')[:2]),.055)
        finally:w.close()
    def test_depth_occupancy_lands_on_static_obstacles(self):
        w=self._world(1000)
        try:
            w.act('scan_world');obs=w.semantic_map_public()['obstacles'];self.assertGreater(len(obs),20)
            boxes=static_obstacle_boxes(w);errs=[obstacle_error_to_static_boxes(o['position_xy'],boxes) for o in obs]
            self.assertLess(float(np.quantile(errs,.95)),.055)
        finally:w.close()
    def test_fresh_red_memory_reacquires_without_chassis_spin(self):
        # 8103 previously had an accurate red memory after map_yellow but ignored
        # it, rotated the chassis ~155 deg, and then failed search.
        w=self._world(8103)
        try:
            self.assertTrue(w.act('map_yellow').ok)
            self.assertIn('red',w.spatial_memory_public())
            y0=w.base_yaw
            r=w.act('search');self.assertTrue(r.ok,r.reason)
            dy=(w.base_yaw-y0+math.pi)%(2*math.pi)-math.pi
            self.assertLess(abs(dy),math.radians(1.0))
        finally:w.close()

    def test_mid_pitch_search_handles_servo_settling_margin(self):
        # 8302/8304 are ~0.64 m red targets. Exact SEARCH pose could see them,
        # but realistic joint settling moved the blob onto the top image edge.
        for seed in (8302,8304):
            with self.subTest(seed=seed):
                w=self._world(seed)
                try:
                    y0=w.base_yaw
                    r=w.act('search');self.assertTrue(r.ok,r.reason)
                    dy=(w.base_yaw-y0+math.pi)%(2*math.pi)-math.pi
                    self.assertLess(abs(dy),math.radians(1.0))
                finally:w.close()

    def test_close_blue_mapping_uses_vertical_pitch_coverage(self):
        # 8406 sees blue at cy~0.92 in pure survey pose: visible but too low for
        # a stable ground-ray update. The second pitch band must map it head-only.
        w=self._world(8406)
        try:
            y0=w.base_yaw
            r=w.act('map_blue');self.assertTrue(r.ok,r.reason)
            dy=(w.base_yaw-y0+math.pi)%(2*math.pi)-math.pi
            self.assertLess(abs(dy),math.radians(1.0))
            mem=w.spatial_memory_public();self.assertIn('blue',mem)
            self.assertLess(np.linalg.norm(np.asarray(mem['blue']['position_xy'])-w._body_pos('blue_block')[:2]),.055)
        finally:w.close()

    def test_tracking_to_metric_approach_survives_pose_transition(self):
        # 8001 originally lost red during survey/search -> tracking pose change.
        w=self._world(8001)
        try:
            for action in ('map_yellow','search','track','approach'):
                r=w.act(action);self.assertTrue(r.ok,f'{action}: {r.reason}')
            mem=w.spatial_memory_public();self.assertIn('red',mem)
            d=np.linalg.norm(np.asarray(mem['red']['position_xy'])-w.odom_xy)
            self.assertLess(d,.39)
        finally:w.close()
    def test_verified_place_relation_survives_later_rgb_observation(self):
        w=self._world(8801)
        try:
            w.spatial_memory['red']={
                'xy':[0.70,-0.20],'z':0.075,'confidence':0.97,'visible':False,
                'relation':'ON_BLUE','last_seen_observation':w._memory_observation_id,
                'source':'verified_place_postcondition+target_memory','seen_count':4,
            }
            w._update_spatial_memory({
                'red': {'visible':True,'cx':0.5,'cy':0.55,'pixels':300},
                'blue': {'visible':False},'yellow': {'visible':False},
            })
            self.assertEqual(w.spatial_memory['red']['relation'],'ON_BLUE')
            self.assertAlmostEqual(w.spatial_memory['red']['z'],0.075,places=6)
        finally:w.close()

