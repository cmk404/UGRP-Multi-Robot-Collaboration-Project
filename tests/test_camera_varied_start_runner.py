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

if __name__ == '__main__':
    unittest.main()
