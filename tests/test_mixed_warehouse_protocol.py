import unittest
from harness.mixed_warehouse_protocol import MixedWarehouseReferee

class MixedProtocolTests(unittest.TestCase):
    def setup_referee(self):
        r=MixedWarehouseReferee({"plank":2,"box1":1,"box2":1},revision=1)
        for rid in ("r1","r2","r3"):
            r.observe({"robot_id":rid,"cargo_ids":["plank","box1","box2"],"revision":1,"observation_id":rid+"obs"})
        return r
    def claim(self,rid,cargo,participants):
        return dict(robot_id=rid,cargo_id=cargo,participants=participants,destination="B",revision=1,observation_id=rid+"obs")
    def test_solo_does_not_wait_for_joint_consent(self):
        r=self.setup_referee()
        self.assertFalse(r.submit([self.claim("r1","plank",["r1","r2"])]).accepted)
        solo=r.submit([self.claim("r3","box1",["r3"])]).accepted
        self.assertEqual(len(solo),1)
        pair=r.submit([self.claim("r2","plank",["r1","r2"])]).accepted
        self.assertEqual(len(pair),1)
        self.assertEqual(len(r.active_assignments),2)
    def test_solo_finisher_can_claim_next_box_while_pair_reserved(self):
        r=self.setup_referee()
        r.submit([self.claim("r1","plank",["r1","r2"]),self.claim("r2","plank",["r1","r2"])])
        a=r.submit([self.claim("r3","box1",["r3"])]).accepted[0]
        r.complete(a.assignment_id,"r3",1);r.release(a.assignment_id,1)
        self.assertTrue(r.submit([self.claim("r3","box2",["r3"])]).accepted)
        self.assertEqual(len(r.active_assignments),2)
    def test_heavy_solo_and_invented_peer_report_rejected(self):
        r=self.setup_referee()
        self.assertFalse(r.submit([self.claim("r1","plank",["r1"])]).accepted)
        r.observe({"robot_id":"r2","cargo_ids":[],"revision":1,"observation_id":"new"})
        with self.assertRaisesRegex(ValueError,"UNSUPPORTED_PEER_REPORT"):
            r.report_peer_observation("r2","box1")
    def test_busy_robot_and_stale_claim_rejected(self):
        r=self.setup_referee();r.submit([self.claim("r3","box1",["r3"])])
        busy=r.submit([self.claim("r3","box2",["r3"])])
        self.assertFalse(busy.accepted)
        self.assertEqual(busy.rejected[0].reason,"RESERVED")
        p=self.claim("r1","box2",["r1"]);p["revision"]=0
        self.assertFalse(r.submit([p]).accepted)

    def test_same_reserved_cargo_is_rejected_immediately(self):
        r=self.setup_referee()
        r.submit([self.claim("r1","plank",["r1","r2"]),self.claim("r2","plank",["r1","r2"])])
        partial=self.claim("r3","plank",["r2","r3"])
        verdict=r.submit([partial])
        self.assertFalse(verdict.accepted)
        self.assertEqual([(x.proposal.robot_id,x.reason) for x in verdict.rejected],[("r3","RESERVED")])
        self.assertNotIn("r3",r.pending)
        replay=r.submit([partial])
        self.assertEqual(replay.rejected[0].reason,"REPLAYED_PROPOSAL")

    def test_future_joint_consent_survives_busy_partner_then_accepts(self):
        r=self.setup_referee()
        solo=r.submit([self.claim("r2","box1",["r2"])]).accepted[0]
        self.assertFalse(r.submit([self.claim("r1","plank",["r1","r2"])]).accepted)
        self.assertIn("r1",r.pending)
        r.complete(solo.assignment_id,"r2",1);r.release(solo.assignment_id,1)
        verdict=r.submit([self.claim("r2","plank",["r1","r2"])])
        self.assertEqual(len(verdict.accepted),1)
        self.assertFalse(verdict.rejected)
        self.assertNotIn("r1",r.pending)

    def test_new_acceptance_does_not_prune_future_partner_consent(self):
        r=self.setup_referee()
        self.assertFalse(r.submit([self.claim("r1","plank",["r1","r2"])]).accepted)
        verdict=r.submit([self.claim("r2","box1",["r2"])])
        self.assertEqual(len(verdict.accepted),1)
        self.assertFalse(verdict.rejected)
        self.assertIn("r1",r.pending)

if __name__=="__main__":unittest.main()
