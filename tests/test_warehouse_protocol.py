import unittest

from harness.warehouse_protocol import LocalObservation, Proposal, WarehouseProtocol


def observations(revision=0):
    return {rid: LocalObservation(rid, ("crate-1",), ("green",), revision,
                                   f"obs-{rid}-{revision}")
            for rid in ("r1", "r2", "r3")}


def submit_all(protocol, round_id, revision=0, cargo="crate-1", destination="green"):
    assignments = {"carrier_0": "r1", "carrier_1": "r2", "scout": "r3"}
    for rid, role in zip(("r1", "r2", "r3"), ("carrier_0", "carrier_1", "scout")):
        protocol.submit_proposal(round_id, Proposal(rid, role, cargo, destination, revision,
                                                    f"I propose {role} for {cargo} to {destination}",
                                                    assignments=assignments,
                                                    observation_id=f"obs-{rid}-{revision}"))


class WarehouseProtocolTests(unittest.TestCase):
    def test_unanimous_distinct_roles_are_accepted_and_logged(self):
        p = WarehouseProtocol()
        r = p.start_round(observations(), "crate-1", "green")
        submit_all(p, r)
        result = p.evaluate(r)
        self.assertTrue(result.accepted)
        self.assertEqual(result.assignments["carrier_0"], "r1")
        self.assertEqual(p.decision_log(r)[0].raw_message, "I propose carrier_0 for crate-1 to green")

    def test_no_comm_keeps_inboxes_isolated(self):
        p = WarehouseProtocol()
        r = p.start_round(observations(), "crate-1", "green", peer_comm=False)
        p.publish_message(r, "r1", "private observation")
        self.assertEqual(p.inbox(r, "r1"), ())
        self.assertEqual(p.inbox(r, "r2"), ())

    def test_missing_or_invalid_proposals_fail_closed(self):
        p = WarehouseProtocol()
        r = p.start_round(observations(), "crate-1", "green")
        plan = {"carrier_0": "r1", "carrier_1": "r2", "scout": "r3"}
        p.submit_proposal(r, Proposal("r1", "carrier_0", "crate-1", "green", 0, "ok", assignments=plan, observation_id="obs-r1-0"))
        p.submit_proposal(r, Proposal("r2", "carrier_0", "crate-1", "green", 0, "duplicate role", assignments=plan, observation_id="obs-r2-0"))
        result = p.evaluate(r)
        self.assertFalse(result.accepted)
        self.assertIn("MISSING_PROPOSAL", result.reason_codes)
        self.assertIn("ROLE_COVERAGE", result.reason_codes)

    def test_observation_and_revision_are_authoritative(self):
        p = WarehouseProtocol()
        r = p.start_round(observations(2), "crate-1", "green", revision=2)
        submit_all(p, r, revision=1)
        result = p.evaluate(r)
        self.assertFalse(result.accepted)
        self.assertIn("STALE_REVISION", result.reason_codes)

    def test_goal_revision_recovers_with_a_fresh_round(self):
        p = WarehouseProtocol()
        old = p.start_round(observations(), "crate-1", "green", revision=0)
        submit_all(p, old)
        self.assertTrue(p.evaluate(old).accepted)
        new_observations = {rid: LocalObservation(rid, ("crate-1",), ("yellow",), 1,
                                                   f"obs-{rid}-1")
                            for rid in ("r1", "r2", "r3")}
        new = p.start_round(new_observations, "crate-1", "yellow", revision=1)
        submit_all(p, new, revision=1, destination="yellow")
        self.assertTrue(p.evaluate(new).accepted)
        with self.assertRaises(ValueError):
            p.start_round(observations(), "crate-1", "green", revision=0)

    def test_agents_can_agree_on_cargo_and_peer_report_extends_observation(self):
        local = {rid: LocalObservation(rid, ("crate-1",), ("green",), 0, f"obs-{rid}-0")
                 for rid in ("r1", "r2", "r3")}
        local["r2"] = LocalObservation("r2", (), ("green",), 0, "obs-r2-0")
        p = WarehouseProtocol()
        r = p.start_round(local, None, "green")
        p.publish_observation_report(r, "r1", "crate-1", "r1 sees crate-1")
        submit_all(p, r)
        result = p.evaluate(r)
        self.assertTrue(result.accepted)
        self.assertEqual(result.cargo_id, "crate-1")

    def test_evaluated_round_is_immutable(self):
        p = WarehouseProtocol()
        r = p.start_round(observations(), "crate-1", "green")
        submit_all(p, r)
        self.assertTrue(p.evaluate(r).accepted)
        with self.assertRaises(ValueError):
            p.publish_message(r, "r1", "late")


if __name__ == "__main__":
    unittest.main()
