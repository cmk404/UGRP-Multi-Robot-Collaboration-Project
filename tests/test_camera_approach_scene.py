import unittest
from scripts.camera_approach_scene import normalize_replay, ready_after_two_fresh_stationary, payload_contact_other_geoms

class ApproachContractTests(unittest.TestCase):
 def test_replay_keys_are_ints(self):
  self.assertEqual(normalize_replay({'targets':{'r1':{'3':740}}}),{'r1':{3:740}})
 def test_two_fresh_stationary_ready_frames_required(self):
  base={'stationary':True,'ok':True,'ready':True}
  self.assertFalse(ready_after_two_fresh_stationary([{**base,'frame_id':1}]))
  self.assertFalse(ready_after_two_fresh_stationary([{**base,'frame_id':1},{**base,'frame_id':1}]))
  self.assertTrue(ready_after_two_fresh_stationary([{**base,'frame_id':1},{**base,'frame_id':2}]))
 def test_ood_blocks_ready(self):
  self.assertFalse(ready_after_two_fresh_stationary([{'frame_id':1,'stationary':True,'ok':False,'ready':True},{'frame_id':2,'stationary':True,'ok':False,'ready':True}]))

class CollisionAndStopTests(unittest.TestCase):
 def test_body_and_finger_contact_count_but_ground_contact_does_not(self):
  self.assertEqual(payload_contact_other_geoms([(1,7),(8,1),(2,7),(1,9)],1,{7,8}),[7,8])
  self.assertEqual(payload_contact_other_geoms([(2,7),(1,9)],1,{7,8}),[])
 def test_new_frame_must_follow_previous_frame(self):
  base={'stationary':True,'ok':True,'ready':True}
  self.assertFalse(ready_after_two_fresh_stationary([{**base,'frame_id':2},{**base,'frame_id':1}]))
 def test_blocked_peer_stops_both_robots_and_cannot_authorize_handoff(self):
  from scripts.run_camera_approach_student import choose_actions
  d={'r1':{'ok':False,'ready':True,'forward':.15},'r3':{'ok':True,'ready':False,'forward':.1}}
  c=choose_actions(d,'visual','cruise',0,1)
  self.assertTrue(c['blocked']);self.assertFalse(c['enter_confirmation'])
  self.assertEqual([a['forward'] for a in c['actions'].values()],[0.,0.])
 def test_confirmation_failure_stops_without_resuming_approach(self):
  from scripts.run_camera_approach_student import choose_actions
  d={'r1':{'ok':True,'ready':False,'forward':.1},'r3':{'ok':True,'ready':True,'forward':0.}}
  c=choose_actions(d,'visual','confirmation-1',9,1)
  self.assertTrue(c['blocked']);self.assertEqual(c['actions']['r1']['forward'],0.)
