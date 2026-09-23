import unittest
from unittest.mock import patch

from scripts.run_camera_varied_start_student import choose_stage_actions, run_approach


class FakeScene:
    def __init__(self):
        self.sim_time = 1.0
        self.drives = []

    def time(self):
        return self.sim_time

    def capture(self, label):
        return {r: {'frame_id': f'{label}-{r}', 'own_bytes': b'own', 'top_bytes': b'top',
                    'own_rgb': f'{label}-{r}-own.png', 'shared_top_rgb': f'{label}-top.png'}
                for r in ('r1', 'r3')}

    def drive_mecanum(self, commands, duration_s):
        self.drives.append((commands, duration_s))
        self.sim_time += duration_s

    def stop_dwell(self):
        self.sim_time += .25

    def tick(self, seconds):
        self.sim_time += seconds

    def evaluation_snapshot(self):
        return {'sim_time': self.sim_time}

class VariedRunnerTests(unittest.TestCase):
    def test_ready_peer_stops_while_other_corrects_only_requested_axis(self):
        decisions = {'r1': {'ok': True, 'ready': True, 'command': .03},
                     'r3': {'ok': True, 'ready': False, 'command': -.04}}
        c = choose_stage_actions(decisions, 'lateral')
        self.assertEqual(c['commands']['r1'], dict(forward=0., left=0., turn=0.))
        self.assertEqual(c['commands']['r3'], dict(forward=0., left=-.04, turn=0.))
        self.assertEqual(c['duration_s'], .2)

    def test_failed_stationary_confirmation_cannot_move(self):
        d = {r: dict(ok=True, ready=False, command=.06) for r in ('r1', 'r3')}
        c = choose_stage_actions(d, 'yaw', confirming=True)
        self.assertFalse(c['enter_confirmation'])
        self.assertTrue(all(v == 0 for r in c['commands'].values() for v in r.values()))
        self.assertEqual(c['duration_s'], .25)

    def test_unsupported_peer_stops_both(self):
        d = {'r1': dict(ok=False, ready=False, command=.06),
             'r3': dict(ok=True, ready=False, command=-.06)}
        c = choose_stage_actions(d, 'yaw')
        self.assertFalse(c['valid'])
        self.assertTrue(all(v == 0 for r in c['commands'].values() for v in r.values()))

    def test_hold_band_accepts_small_settling_change_without_moving(self):
        d = {r: dict(ok=True, ready=False, stationary_ready=True, command=.01)
             for r in ('r1', 'r3')}
        moving = choose_stage_actions(d, 'yaw')
        self.assertFalse(moving['enter_confirmation'])
        self.assertEqual(moving['commands']['r1']['turn'], .01)
        holding = choose_stage_actions(d, 'yaw', confirming=True)
        self.assertTrue(all(holding['ready'].values()))
        self.assertTrue(all(v == 0 for r in holding['commands'].values() for v in r.values()))

    @patch('harness.camera_approach_student.predict_approach')
    def test_settling_error_reacquires_from_fresh_rgb_before_confirming(self, predict):
        ready = dict(ok=True, ready=True, stationary_ready=True, forward=0.)
        drift = dict(ok=True, ready=False, stationary_ready=False, forward=.01)
        predict.side_effect = [d for d in (ready, drift, drift, ready, ready, ready) for _ in range(2)]
        scene = FakeScene()
        result = run_approach(scene, {}, condition='straight',
                              straight_models={'r1': {}, 'r3': {}}, reacquire_on_settle=True)
        self.assertTrue(result['approach_ok'])
        self.assertEqual(result['stage_results'][0]['reacquisitions'], 1)
        self.assertEqual(scene.drives[1][0]['r1']['forward'], 0.)
        self.assertEqual(scene.drives[2][0]['r1']['forward'], .01)
        confirmations = result['stage_results'][0]['confirmations']
        self.assertEqual([all(c['ready'].values()) for c in confirmations], [False, True, True])
        self.assertEqual(len(result['approach_calls']), 12)

    @patch('harness.camera_approach_student.predict_approach')
    def test_reacquisition_keeps_total_confirmation_budget(self, predict):
        ready = dict(ok=True, ready=True, forward=0.)
        drift = dict(ok=True, ready=False, forward=.01)
        predict.side_effect = [d for d in (ready, drift)*4 for _ in range(2)]
        result = run_approach(FakeScene(), {}, condition='straight',
                              straight_models={'r1': {}, 'r3': {}}, reacquire_on_settle=True)
        self.assertFalse(result['approach_ok'])
        self.assertEqual(len(result['stage_results'][0]['confirmations']), 4)
        self.assertEqual(result['stage_results'][0]['reason'], 'stationary RGB confirmation budget exhausted')

    @patch('harness.camera_approach_student.predict_approach')
    def test_run_approach_returns_auditable_straight_confirmation(self, predict):
        predict.return_value = {'ok': True, 'ready': True, 'forward': .04}
        scene = FakeScene()

        result = run_approach(scene, {}, condition='straight',
                              straight_models={'r1': {}, 'r3': {}})

        self.assertTrue(result['approach_ok'])
        self.assertEqual(len(result['stage_results']), 1)
        self.assertEqual(result['stage_results'][0]['confirmations'][-1]['stationary'], True)
        self.assertEqual(len(result['approach_calls']), 6)
        self.assertEqual(result['approach_calls'][0]['action']['forward'], 0.)
        self.assertEqual(result['approach_calls'][-1]['own_command_history'],
                         [result['approach_calls'][1]['action'], result['approach_calls'][3]['action']])
        self.assertNotIn('final_alignment_checks', result)
        self.assertAlmostEqual(result['approach_elapsed_sim_s'], .75)
        self.assertEqual(result['approach_end_state'], {'sim_time': 1.75})

class TaggedScene(FakeScene):
    def capture(self, label):
        frames = super().capture(label)
        for row in frames.values():
            row['own_bytes'] = label.encode()
        return frames


class InvalidRGBRecoveryTests(unittest.TestCase):
    good = dict(ok=True, ready=True, stationary_ready=True, precision='fine', command=0.)
    invalid = dict(ok=False, ready=False, command=0., reason='rgb_pose_outside_support')

    def models(self):
        return {r: {axis: {'robot': r} for axis in ('yaw', 'lateral', 'forward')}
                for r in ('r1', 'r3')}

    @patch('scripts.run_camera_varied_start_student.PHASES', ('forward',))
    @patch('harness.camera_varied_start_student.predict_stage')
    def test_opt_in_reobserves_after_stationary_hold_and_two_supported_frames(self, predict):
        predict.side_effect = lambda model, own, top: (
            self.invalid if own == b'phase-0-000' and model['robot'] == 'r3' else self.good)
        scene = TaggedScene()
        result = run_approach(scene, self.models(), invalid_reobserve_budget=1)
        self.assertTrue(result['approach_ok'])
        stage = result['stage_results'][0]
        self.assertEqual(stage['recovery_trigger']['stationary_command_s'], .25)
        self.assertEqual(stage['recovery_trigger']['additional_hold_s'], 1.3)
        self.assertEqual(len(stage['recovery_confirmations']), 2)
        self.assertTrue(all(all(c['supported'].values()) and c['stationary']
                            for c in stage['recovery_confirmations']))
        self.assertNotEqual(stage['recovery_confirmations'][0]['frame_ids'],
                            stage['recovery_confirmations'][1]['frame_ids'])
        self.assertTrue(all(v == 0 for commands, _ in scene.drives[:3]
                            for command in commands.values() for v in command.values()))
        self.assertEqual([row['action']['duration_s'] for row in result['approach_calls'][:2]], [.25, .25])
        self.assertAlmostEqual(result['approach_elapsed_sim_s'], 1.3 + 8*.25)

    @patch('scripts.run_camera_varied_start_student.PHASES', ('forward',))
    @patch('harness.camera_varied_start_student.predict_stage')
    def test_persistent_invalid_rgb_stops_after_one_bounded_attempt(self, predict):
        predict.side_effect = lambda model, own, top: (
            self.invalid if model['robot'] == 'r3' else self.good)
        scene = TaggedScene()
        result = run_approach(scene, self.models(), invalid_reobserve_budget=1)
        self.assertFalse(result['approach_ok'])
        self.assertEqual(len(scene.drives), 2)
        self.assertEqual(result['stage_results'][0]['reason'],
                         'RGB outside learned stage support after bounded re-observation')
        self.assertEqual(len(result['stage_results'][0]['recovery_confirmations']), 1)
        self.assertTrue(all(v == 0 for commands, _ in scene.drives
                            for command in commands.values() for v in command.values()))

    @patch('scripts.run_camera_varied_start_student.PHASES', ('forward',))
    @patch('harness.camera_varied_start_student.predict_stage')
    def test_default_invalid_rgb_still_fails_immediately(self, predict):
        predict.side_effect = lambda model, own, top: (
            self.invalid if model['robot'] == 'r3' else self.good)
        scene = TaggedScene()
        result = run_approach(scene, self.models())
        self.assertFalse(result['approach_ok'])
        self.assertEqual(len(scene.drives), 1)
        self.assertEqual(scene.drives[0][1], .25)
        self.assertNotIn('recovery_trigger', result['stage_results'][0])


class FinalAlignmentRefinementTests(unittest.TestCase):
    good = dict(ok=True, ready=True, stationary_ready=True, precision='fine', command=0.)
    drift = dict(ok=True, ready=False, stationary_ready=False, precision='fine', command=.01)

    def models(self):
        return {r: {s: dict(robot=r, axis=s) for s in ('yaw', 'lateral', 'forward')}
                for r in ('r1', 'r3')}

    @patch('harness.camera_varied_start_student.predict_stage')
    def test_final_forward_coupling_is_corrected_and_all_axes_reconfirmed(self, predict):
        def decide(model, own, top):
            tag = own.decode()
            return self.drift if model['robot']=='r1' and model['axis']=='lateral' and (
                tag.startswith('final-alignment-') or tag=='alignment-refine-000') else self.good
        predict.side_effect = decide
        result = run_approach(TaggedScene(), self.models(), final_refinement_steps=40)
        self.assertTrue(result['approach_ok'])
        calls = result['alignment_refinement_calls']
        self.assertEqual(len(calls), 4)
        self.assertEqual(calls[0]['control']['axis'], 'lateral')
        self.assertEqual(calls[0]['control']['commands']['r1']['left'], .01)
        self.assertEqual(calls[0]['control']['commands']['r3']['left'], 0.)
        for c in calls[-2:]:
            self.assertTrue(c['stationary'] and all(c['control']['ready'].values()))
            self.assertTrue(all(v==0 for d in c['control']['commands'].values() for v in d.values()))
        self.assertNotEqual(calls[-1]['frame_ids'], calls[-2]['frame_ids'])

    @patch('harness.camera_varied_start_student.predict_stage')
    def test_persistent_joint_error_has_a_hard_budget(self, predict):
        predict.side_effect = lambda model, own, top: self.drift if own.decode().startswith(
            ('final-alignment-', 'alignment-refine-')) else self.good
        result = run_approach(TaggedScene(), self.models(), final_refinement_steps=40)
        self.assertFalse(result['approach_ok'])
        self.assertEqual(len(result['alignment_refinement_calls']), 40)
        self.assertEqual(result['alignment_refinement_reason'], 'bounded refinement budget exhausted')

    @patch('harness.camera_varied_start_student.predict_stage')
    def test_unsupported_refinement_stops_both(self, predict):
        def decide(model, own, top):
            if own.decode().startswith('alignment-refine-'):
                return {**self.drift, 'ok':False}
            return self.drift if own.decode().startswith('final-alignment-') else self.good
        predict.side_effect = decide
        result = run_approach(TaggedScene(), self.models(), final_refinement_steps=40)
        self.assertFalse(result['approach_ok'])
        calls = result['alignment_refinement_calls']
        self.assertEqual(len(calls), 1)
        self.assertTrue(all(v==0 for d in calls[0]['control']['commands'].values() for v in d.values()))

if __name__ == '__main__':
    unittest.main()
