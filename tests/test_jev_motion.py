import copy
import json
import math
import unittest
from harness.jev_motion import (policy_state, action, at_goal, rule, OPTIONS,
                               validate_jev, validate_gemini, detect_box)
import cv2
import numpy as np

class TestJevMotion(unittest.TestCase):
    def observation(self):
        return dict(valid=True,range_m=.4,bearing_deg=10.,target_forward_m=.39,target_left_m=.07,
                    own_view_cyan_fraction=.01,top_target_area_px=150,truth={'qpos':[1]},evidence={'private':'excluded'})
    def test_input_allowlist(self):
        state=policy_state(self.observation(),[dict(action='forward',range_m=.45,bearing_deg=10,contacts=True)])
        self.assertNotIn('qpos',json.dumps(state));self.assertNotIn('private',json.dumps(state));self.assertNotIn('contacts',json.dumps(state))
        for value in (math.nan,math.inf,True):
            o=self.observation();o['range_m']=value
            with self.assertRaises(ValueError):policy_state(o,[])
    def test_raw_actions_are_bounded(self):
        from sim.camera_robot_port import CameraRobotPort
        p=object.__new__(CameraRobotPort);p._allow_reverse=True;p._allow_mecanum=True
        for name in OPTIONS:p.validate_bounded(action(name),.2)
        with self.assertRaises(KeyError):action('teleport')
    def test_readiness_and_rule(self):
        o=self.observation();self.assertEqual(rule(policy_state(o,[])),'turn_left')
        o['bearing_deg']=-10;self.assertEqual(rule(policy_state(o,[])),'turn_right')
        o.update(range_m=.28,bearing_deg=0);self.assertTrue(at_goal(o));self.assertEqual(rule(policy_state(o,[])),'stop')
        o['range_m']=.19;self.assertFalse(at_goal(o));self.assertEqual(rule(policy_state(o,[])),'backward')
    def test_provider_rejects_invalid_decisions(self):
        p={k:float(k=='forward') for k in OPTIONS}
        body={'answers':{'action':{'type':'choice','choice':'forward','probabilities':p,'confidence':.4}}}
        self.assertEqual(validate_jev(body),'forward')
        for mutate in (lambda a:a.update(choice='stop'),lambda a:a['probabilities'].update(forward=float('nan')),lambda a:a['probabilities'].update(extra=0)):
            b=copy.deepcopy(body);mutate(b['answers']['action'])
            with self.assertRaises(ValueError):validate_jev(b)
        with self.assertRaises(ValueError):validate_gemini({'choices':[{'message':{'content':'{"action":"forward","duration":10}'}}]})
    def test_target_ambiguity_holds(self):
        frame=np.zeros((200,300,3),np.uint8)
        cv2.rectangle(frame,(40,50),(52,62),(255,255,0),-1)
        xy,_=detect_box(frame);self.assertLess(np.linalg.norm(xy-[46,56]),1)
        cv2.rectangle(frame,(100,50),(112,62),(255,255,0),-1)
        with self.assertRaises(ValueError):detect_box(frame)
