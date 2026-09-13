import unittest
from harness.pair_carry_policy import PairCarryPolicy


def decisions(ready=False):
    return {r: {'ok': True, 'held_estimate': True, 'ready': ready,
                'forward': 0. if ready else .1} for r in ('r1', 'r3')}


class PairCarryPolicyTests(unittest.TestCase):
    def step(self, p, t, skew=0., ready=False, delivered=('r1','r3')):
        return p.step(decisions(ready), skew, {r: str(t) for r in ('r1','r3')}, t, delivered)

    def test_missing_peer_then_fresh_pair_starts(self):
        p = PairCarryPolicy()
        self.assertEqual(self.step(p, 0, delivered=('r1',))['forwards'], {'r1':0.,'r3':0.})
        self.assertEqual(self.step(p, .2)['permission']['phase'], 'GO')

    def test_skew_stop_catch_up_and_fresh_rejoin_both_directions(self):
        for sign, lagging in ((1,'r3'),(-1,'r1')):
            with self.subTest(sign=sign):
                p = PairCarryPolicy()
                self.step(p, 0)
                held = self.step(p, .2, skew=sign*4)
                self.assertEqual(held['mode'], 'SETTLE')
                self.assertFalse(any(held['forwards'].values()))
                aligning = self.step(p, .6, skew=sign*4)
                self.assertEqual(aligning['mode'], 'ALIGN')
                self.assertEqual(aligning['forwards'][lagging], .04)
                self.assertEqual(sum(aligning['forwards'].values()), .04)
                rejoin = self.step(p, .8, skew=0)
                self.assertEqual(rejoin['mode'], 'REJOIN')
                self.assertFalse(any(rejoin['forwards'].values()))
                self.assertEqual(self.step(p, 1.2)['mode'], 'CRUISE')

    def test_missing_images_never_advance_and_eventually_abort(self):
        p = PairCarryPolicy()
        self.step(p, 0)
        for i in range(1,6):
            result = self.step(p, i*.2, skew=None)
            self.assertFalse(any(result['forwards'].values()))
        self.assertTrue(result['abort'])
        self.assertTrue(self.step(p, 1.2)['abort'])

    def test_goal_requires_three_fresh_observations_and_two_stopped_intervals(self):
        p = PairCarryPolicy()
        self.assertFalse(self.step(p, 0, ready=True)['done'])
        self.assertFalse(self.step(p, .25, ready=True)['done'])
        self.assertTrue(self.step(p, .5, ready=True)['done'])

    def test_expired_peer_halts_bounded_lease(self):
        p = PairCarryPolicy()
        self.step(p, 0)
        self.step(p, .2, delivered=('r1',))
        row = self.step(p, .8, delivered=('r1',))
        self.assertEqual(row['permission']['phase'], 'HOLD')
        self.assertFalse(any(row['forwards'].values()))


if __name__ == '__main__':
    unittest.main()
