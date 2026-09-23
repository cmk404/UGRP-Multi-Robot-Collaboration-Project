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

    def test_delayed_rgb_keeps_capture_time_and_holds_both(self):
        p=PairCarryPolicy()
        initial=p.step(decisions(),0.,{'r1':'frame-1','r3':'frame-1'},0.,
                       observed_at_s=0.)
        self.assertEqual(initial['permission']['phase'],'GO')
        delayed=p.step(decisions(),0.,{'r1':'frame-2','r3':'frame-2'},.7,
                       observed_at_s=.05)
        self.assertEqual(delayed['permission']['phase'],'HOLD')
        self.assertFalse(any(delayed['forwards'].values()))
        self.assertFalse(delayed['valid'])
        fresh=p.step(decisions(),0.,{'r1':'frame-3','r3':'frame-3'},.8,
                     observed_at_s=.75)
        self.assertEqual(fresh['permission']['phase'],'GO')
        self.assertEqual(fresh['forwards'],{'r1':.1,'r3':.1})

    def test_continuously_late_rgb_uses_separate_bounded_timeout(self):
        p=PairCarryPolicy(stale_budget_s=1.)
        for index,now in enumerate((.7,.9,1.1,1.3,1.5),1):
            row=p.step(decisions(),0.,{'r1':str(index),'r3':str(index)},now,
                       observed_at_s=now-.7)
            self.assertTrue(row['stale_rgb'])
            self.assertFalse(any(row['forwards'].values()))
            self.assertEqual(p.invalid_count,0)
            self.assertFalse(row['abort'])
        expired=p.step(decisions(),0.,{'r1':'6','r3':'6'},1.7,
                       observed_at_s=1.)
        self.assertTrue(expired['abort'])
        self.assertEqual(expired['permission']['reason'],'stale_pair_rgb_budget_exhausted')
        self.assertEqual(p.invalid_count,0)

    def test_stale_timer_resets_only_after_fresh_complete_pair(self):
        p=PairCarryPolicy(stale_budget_s=2.)
        p.step(decisions(),0.,{'r1':'1','r3':'1'},.7,observed_at_s=0.)
        self.assertEqual(p.stale_since_s,.7)
        p.step(decisions(),0.,{'r1':'2','r3':'2'},.9,('r1',),observed_at_s=.8)
        self.assertEqual(p.stale_since_s,.7)
        fresh=p.step(decisions(),0.,{'r1':'3','r3':'3'},1.,observed_at_s=.95)
        self.assertEqual(fresh['permission']['phase'],'GO')
        self.assertIsNone(p.stale_since_s)

    def test_undelivered_ready_cannot_complete_or_change_policy(self):
        p, q = PairCarryPolicy(), PairCarryPolicy()
        self.step(p, 0, ready=True)
        self.step(q, 0, ready=True)
        a, b = decisions(True), decisions(False)
        a['r1'] = b['r1']
        for i in range(1,4):
            frames = {'r1': str(i), 'r3': str(i)}
            left = p.step(a, 0, frames, i*.2, ('r1',))
            right = q.step(b, 0, frames, i*.2, ('r1',))
            self.assertEqual(left,right)
            self.assertFalse(left['done'])
            self.assertFalse(any(left['forwards'].values()))

    def test_rejected_duplicate_evidence_does_not_count_as_confirmation(self):
        p = PairCarryPolicy()
        self.step(p, 0, ready=True)
        for t in (.2,.4,.6):
            row = p.step(decisions(True),0,{'r1':'0','r3':'0'},t)
            self.assertFalse(row['done'])
            self.assertFalse(any(row['forwards'].values()))
        self.assertFalse(self.step(p,.8,ready=True)['done'])


if __name__ == '__main__':
    unittest.main()
