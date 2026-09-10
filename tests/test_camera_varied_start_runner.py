import unittest
from scripts.run_camera_varied_start_student import choose_stage_actions

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

if __name__ == '__main__':
    unittest.main()
