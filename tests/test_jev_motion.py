import copy
import json
import math
import unittest
from pathlib import Path
from harness.jev_motion import (policy_state, action, at_goal, rule, OPTIONS,
                               validate_jev, validate_gemini, detect_box)
import cv2
import numpy as np

class TestJevMotion(unittest.TestCase):
    def test_output_evaluation_accepts_actual_clock_roundoff(self):
        from scripts.run_jev_motion import physical_goal_confirmed
        # Actual saved new09-r0/r2 Jev semantic timestamps; 350ms became just less.
        times=[17.70000000000107,17.75200000000104,17.80400000000101,
               17.856000000000982,17.908000000000953,17.960000000000925,
               18.012000000000896,18.050000000000875]
        rows=[dict(sim_s=t,range_m=.2885879199,bearing_deg=4.39682415) for t in times]
        self.assertLess(times[-1]-times[0],.35)
        self.assertTrue(physical_goal_confirmed(rows))

    def test_output_evaluation_keeps_physical_limits(self):
        from scripts.run_jev_motion import physical_goal_confirmed
        rows=[dict(sim_s=t,range_m=.28,bearing_deg=0.) for t in (0.,.348)]
        self.assertFalse(physical_goal_confirmed(rows))
        rows[-1]['sim_s']=.35
        self.assertTrue(physical_goal_confirmed(rows))
        for key,value in [('range_m',.300001),('range_m',.259999),('bearing_deg',6.000001)]:
            invalid=copy.deepcopy(rows);invalid[0][key]=value
            self.assertFalse(physical_goal_confirmed(invalid))
        self.assertFalse(physical_goal_confirmed([]))

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
        for mutate in (lambda a:a.update(choice='teleport'),lambda a:a['probabilities'].update(forward=float('nan')),lambda a:a['probabilities'].update(extra=0)):
            b=copy.deepcopy(body);mutate(b['answers']['action'])
            with self.assertRaises(ValueError):validate_jev(b)
        with self.assertRaises(ValueError):validate_gemini({'choices':[{'message':{'content':'{"action":"forward","duration":10}'}}]})
    def test_target_ambiguity_holds(self):
        frame=np.zeros((200,300,3),np.uint8)
        cv2.rectangle(frame,(40,50),(52,62),(255,255,0),-1)
        xy,_=detect_box(frame);self.assertLess(np.linalg.norm(xy-[46,56]),1)
        cv2.rectangle(frame,(100,50),(112,62),(255,255,0),-1)
        with self.assertRaises(ValueError):detect_box(frame)

    def test_live_rounded_probabilities(self):
        probabilities=dict(zip(('turn_left','turn_right','backward','stop','left','forward','right'),(.13,.08,.09,.04,.05,.55,.05)))
        body={'answers':{'action':{'type':'choice','choice':'forward','probabilities':probabilities,'confidence':.48}}}
        self.assertEqual(validate_jev(body),'forward')
        body['answers']['action']['probabilities']['forward']=.85
        with self.assertRaises(ValueError):validate_jev(body)

    def test_real_rgb_observer_without_truth_or_command_integration(self):
        from harness.jev_motion import RGBObserver
        from sim.research_dispatch_arena import FIXED_TOP
        root=Path(__file__).parent/'fixtures/jev_motion'
        identity=json.loads((root/'identity.json').read_text())
        observer=RGBObserver({'top_camera':FIXED_TOP},(root/'probe-before-top.jpg').read_bytes(),
            (root/'probe-after-top.jpg').read_bytes(),np.array(identity['claim']['center'])*[959,719])
        own=(root/'000-r2.jpg').read_bytes();top=(root/'000-top.jpg').read_bytes()
        a=observer.observe(own,top);b=observer.observe(own,top)
        self.assertGreater(a['range_m'],.5);self.assertLess(a['range_m'],.6)
        self.assertLess(abs(a['bearing_deg']),10)
        self.assertAlmostEqual(a['range_m'],b['range_m'],places=4)
        ok,blank=cv2.imencode('.jpg',np.zeros((720,960,3),np.uint8))
        with self.assertRaises(ValueError):observer.observe(own,blank.tobytes())

    def test_nonmaximal_valid_choice_is_preserved_not_replaced(self):
        p={k:0. for k in OPTIONS};p.update(forward=.4,stop=.6)
        body={'answers':{'action':{'type':'choice','choice':'forward','probabilities':p,'confidence':.2}}}
        self.assertEqual(validate_jev(body),'forward')

    def test_wheel_pixel_uncertainty_regression(self):
        from harness.jev_motion import wheel_envelope
        from harness.dispatch_pair_navigation import PairVision
        from harness.camera_goal_transport import decode
        root=Path(__file__).parent/'fixtures/jev_motion'
        im=decode((root/'negative-yaw-boundary.jpg').read_bytes());mask=PairVision._mask(None,im)
        yy,xx=np.indices(mask.shape);mask[(abs(xx-179)>43)|(abs(yy-530)>40)]=0
        result=wheel_envelope(mask)
        self.assertIsNotNone(result);self.assertLess(abs(result['angle_deg']-11.3),1)
        mask[yy<530]=0
        self.assertIsNone(wheel_envelope(mask))

    def test_representation_diagnostic_preserves_actions_and_excludes_truth(self):
        from scripts.probe_jev_motion_representation import request, semantic, VARIANTS
        state=policy_state(self.observation(),[]); original=copy.deepcopy(state)
        for variant in VARIANTS:
            body=request(state,variant)
            self.assertEqual(set(body['questions']['action']['criteria']),set(OPTIONS))
            self.assertNotIn('qpos',json.dumps(body));self.assertNotIn('private',json.dumps(body))
        self.assertEqual(state,original)
        # Original observed failures: numeric goal boundary and negative bearing.
        a=semantic(dict(range_m=.2832,bearing_deg=-1.93))
        self.assertEqual(a['distance_to_goal_band'],'too_far');self.assertFalse(a['goal_satisfied'])
        b=semantic(dict(range_m=.6148,bearing_deg=-3.59))
        self.assertEqual(b['target_side'],'right');self.assertEqual(b['heading_alignment'],'outside_tolerance')
        for distance in (.27,.28):
            self.assertTrue(semantic(dict(range_m=distance,bearing_deg=3.))['goal_satisfied'])

    def test_live_variants_share_the_exact_semantic_contract(self):
        from scripts.run_jev_motion import model_request
        from scripts.probe_jev_motion_representation import request
        from scripts.run_jev_semantic_cohort import plan,NEW_CASES,CASES
        state=policy_state(self.observation(),[])
        j=model_request(state,'jev','semantic','jev-1.13.0')
        self.assertEqual(j,request(state,'semantic_state'))
        g=model_request(state,'gemini','semantic','gemini-3.8-flash')
        self.assertEqual(json.loads(g['messages'][1]['content']),j['state'])
        self.assertTrue(g['messages'][0]['content'].startswith(j['questions']['action']['instructions']))
        self.assertEqual(j['questions']['action']['criteria'],OPTIONS)
        jobs=plan();self.assertEqual(len(jobs),195);self.assertEqual(len({x['trial_id'] for x in jobs}),195)
        self.assertEqual(jobs,plan());self.assertTrue(all(v not in CASES.values() for v in NEW_CASES.values()))
