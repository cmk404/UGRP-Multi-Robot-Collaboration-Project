import copy
import json
import unittest
from pathlib import Path

import numpy as np

from harness.jev_skill_motion import (SkillObserver,CompletionGate,SkillController,
    semantic_state,request,parse_answer,routes,bounded_command,reference_choice)
from sim.research_dispatch_arena import episode


class TestSkillMotion(unittest.TestCase):
    def test_arm_fragment_does_not_force_permanent_uncertainty(self):
        import cv2
        from harness.jev_skill_motion import fit_wheels
        root=Path(__file__).parent/'fixtures/jev_skill_motion'
        meta=json.loads((root/'provenance.json').read_text())
        image=cv2.imread(str(root/'arm-fragment-top.jpg'))
        _,_,fit=fit_wheels(image,np.array(meta['rgb_anchor']),meta['rgb_prior_heading_rad'])
        self.assertLess(fit['rms_px'],1.)

    def observation(self,**changes):
        o={'valid':True,'range_m':.5,'bearing_deg':0.,'raw_range_m':.5,'raw_bearing_deg':0.,
           'xy_m':[-.7,-2.65],'target_xy_m':[-.2,-2.65],'heading_rad':0.,
           'range_trend':'closing','quality':'clear','range_spread_m':.002,
           'bearing_spread_deg':.2,'obstacles':[],'evidence':{'truth':'not allowed'}}
        o.update(changes);return o

    def test_rgb_wheel_fit_and_repeated_frame(self):
        root=Path(__file__).parent/'fixtures/jev_motion'
        identity=json.loads((root/'identity.json').read_text())
        observer=SkillObserver(episode('open')['static_map'],(root/'probe-before-top.jpg').read_bytes(),
            (root/'probe-after-top.jpg').read_bytes(),np.array(identity['claim']['center'])*[959,719])
        own=(root/'000-r2.jpg').read_bytes();top=(root/'000-top.jpg').read_bytes()
        a=observer.observe(own,top);b=observer.observe(own,top)
        self.assertAlmostEqual(a['range_m'],b['range_m'])
        self.assertAlmostEqual(a['bearing_deg'],b['bearing_deg'])
        self.assertGreater(a['evidence']['wheel_fit']['matched_wheels'],2)
        with self.assertRaises(ValueError):observer.observe(own,b'not a JPEG')

    def test_goal_requires_strict_bounds_stationarity_and_consecutive_frames(self):
        gate=CompletionGate();inside=self.observation(range_m=.275,bearing_deg=2.9)
        self.assertFalse(gate.update(inside));self.assertFalse(gate.update(inside))
        self.assertFalse(gate.update(self.observation(range_m=.275,bearing_deg=3.08)))
        self.assertFalse(gate.update(inside));self.assertFalse(gate.update(inside));self.assertTrue(gate.update(inside))
        self.assertFalse(gate.update(self.observation(range_m=.275,bearing_deg=0,range_spread_m=.009)))
        self.assertFalse(gate.update(self.observation(range_m=.28001,bearing_deg=0)))

    def test_request_allowlist_and_same_semantic_questions(self):
        o=self.observation();o['referee']={'qpos':'secret'}
        state=semantic_state(o,{'direct':'direct','hold_and_observe':'wait'},[],None,0,0)
        self.assertNotIn('qpos',json.dumps(state));self.assertNotIn('evidence',state)
        j=request(state,'jev');g=request(state,'gemini')
        self.assertEqual(j['state'],json.loads(g['messages'][1]['content']))
        self.assertIn(json.dumps(j['questions']),g['messages'][0]['content'])

    def test_typed_answer_validation_and_canonical_choice(self):
        state=semantic_state(self.observation(),{'direct':'direct','hold_and_observe':'wait'},[],None,0,0)
        questions=request(state,'jev')['questions']
        answers={}
        for name,q in questions.items():
            keys=list(q['criteria'])
            answers[name]={'type':'choice','choice':keys[0],'confidence':.2,
                'probabilities':{keys[0]:.4,keys[1]:.6}}
        a,confidence=parse_answer({'answers':answers},'jev',state)
        self.assertEqual(a['action'],'direct');self.assertEqual(confidence,.2)
        answers['action']['probabilities']['direct']=float('nan')
        with self.assertRaises(ValueError):parse_answer({'answers':answers},'jev',state)
        with self.assertRaises(ValueError):parse_answer({'choices':[{'message':{'content':'{"action":"teleport"}'}}]},'gemini',state)

    def test_route_candidates_use_rgb_barrier_and_map(self):
        m=episode('open')['static_map']
        o=self.observation(xy_m=[-.8,-2.65],target_xy_m=[.1,-2.65],range_m=.9)
        self.assertIn('direct',routes(o,m))
        o['obstacles']=[{'center_m':[-.42,-2.65],'half_extents_m':[.06,.10]}]
        candidates=routes(o,m)
        self.assertNotIn('direct',candidates)
        self.assertTrue(set(candidates)&{'north','south'})

    def test_event_hold_has_expiry_and_change_interrupt(self):
        c=SkillController(episode('open')['static_map']);o=self.observation()
        s=c.state(o);self.assertEqual(c.need_query(s),'initial_or_interrupted')
        c.select('direct',o)
        for _ in range(5):
            c.command(o);self.assertIsNone(c.need_query(c.state(o)))
        c.command(o);self.assertEqual(c.need_query(c.state(o)),'decision_expired')
        c.select('direct',o);o['range_m']=.33
        self.assertIsNotNone(c.need_query(c.state(o)))

    def test_low_confidence_shortens_commitment_without_rule_substitution(self):
        c=SkillController(episode('open')['static_map']);o=self.observation()
        c.state(o);c.select('direct',o,.1)
        cmd,source=c.command(o)
        self.assertEqual(source,'selected_skill_feedback')
        self.assertGreater(cmd['forward'],0);self.assertEqual(c.active,'direct')
        c.command(o);self.assertEqual(c.need_query(c.state(o)),'decision_expired')

    def test_fine_action_smaller_than_coarse_and_port_valid(self):
        from sim.camera_robot_port import CameraRobotPort
        c=SkillController(episode('open')['static_map']);o=self.observation(range_m=.283)
        c.state(o);c.select('approach_fine',o)
        cmd,_=c.command(o);self.assertLess(cmd['forward'],.1)
        p=object.__new__(CameraRobotPort);p._allow_reverse=True;p._allow_mecanum=True
        p.validate_bounded(cmd,.2)
        for key,obs in [('align_target',self.observation(range_m=.3,bearing_deg=8.)),
                        ('backoff_fine',self.observation(range_m=.265)),('hold_and_observe',o)]:
            c.select(key,obs);p.validate_bounded(c.command(obs)[0],.2)

    def test_already_satisfied_skills_are_not_reoffered(self):
        c=SkillController(episode('open')['static_map'])
        candidates=c.describe(self.observation())
        self.assertNotIn('align_target',candidates)
        self.assertNotIn('align_left',candidates)
        candidates=c.describe(self.observation(range_m=.275,bearing_deg=0.))
        self.assertEqual(set(candidates),{'hold_and_observe'})

    def test_stale_decision_rejected_on_goal_or_route_change(self):
        from harness.jev_skill_motion import request_compatible
        c=SkillController(episode('open')['static_map'])
        old=c.state(self.observation());new=copy.deepcopy(old)
        self.assertTrue(request_compatible(old,new))
        new['observation']['distance']='near'
        self.assertFalse(request_compatible(old,new))
        new=copy.deepcopy(old);new['candidates'].pop('direct')
        self.assertFalse(request_compatible(old,new))

    def test_frozen_cohorts_are_unique_and_repeatable(self):
        from scripts.run_jev_skill_cohort import plan
        for phase,count in [('holdout',108),('regression',36),('ablation',68),('continuous',16)]:
            jobs,_=plan(phase)
            self.assertEqual(len(jobs),count)
            self.assertEqual(len({j['trial_id'] for j in jobs}),count)
            self.assertEqual(jobs,plan(phase)[0])

    def test_primitive_ablation_really_uses_fixed_motor_action(self):
        c=SkillController(episode('open')['static_map'],primitive=True)
        o=self.observation(range_m=.283);state=c.state(o)
        self.assertEqual(reference_choice(state),'forward')
        c.select('forward',o);cmd,source=c.command(o)
        self.assertEqual(cmd['forward'],.1);self.assertEqual(source,'selected_primitive')

    def test_case_future_event_does_not_enter_static_prior(self):
        from scripts.run_jev_skill_motion import configuration,HOLDOUT
        c=configuration(HOLDOUT['temporary01'])
        self.assertNotIn('barrier_window',json.dumps(c['static_map']))
        self.assertNotIn('motion_barrier',json.dumps(c['static_map']))
        self.assertIn('motion_barrier',json.dumps(c['setup_only']))

    def test_repeated_hold_reports_no_progress_and_far_heading_is_not_goal_edge(self):
        c=SkillController(episode('open')['static_map']);o=self.observation(bearing_deg=2.8)
        c.observe_progress(o);c.state(o);c.select('hold_and_observe',o)
        for _ in range(4):c.observe_progress(o)
        state=c.state(o)
        self.assertTrue(state['persistent_stall'])
        self.assertFalse(state['observation']['goal_alignment_edge'])
        self.assertFalse(state['observation']['goal_distance_edge'])
        self.assertEqual(state['recent_issued'][-1]['progress'],'no_motion')
