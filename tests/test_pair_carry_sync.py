import json
import math
import unittest

from harness.pair_carry_sync import PairCarrySync, SharedResourceLedger


def ready(sync, robot, sequence, now, *, epoch=None, version=None, frame=None, value=True):
    return sync.report(
        robot,
        plan_version=sync.plan_version if version is None else version,
        epoch=sync.epoch if epoch is None else epoch,
        sequence=sequence,
        ready=value,
        observed_at_s=now - 0.01,
        received_at_s=now,
        frame_id=frame or f"{robot}-{sequence}",
    )


class PairCarrySyncTests(unittest.TestCase):
    def test_two_fresh_reports_authorize_go(self):
        sync = PairCarrySync("carry-1")
        self.assertEqual("WAIT", sync.authorize(1.0)["phase"])
        self.assertTrue(ready(sync, "r1", 1, 1.0))
        self.assertTrue(ready(sync, "r3", 1, 1.1))
        self.assertEqual({"phase": "GO", "epoch": 0, "reason": "all_participants_ready"},
                         sync.authorize(1.1))
        json.dumps(sync.events)

    def test_hold_revokes_old_permission_and_requires_both_fresh_reports(self):
        sync = PairCarrySync("carry-1")
        ready(sync, "r1", 1, 1.0)
        ready(sync, "r3", 1, 1.0)
        self.assertEqual("GO", sync.authorize(1.0)["phase"])

        self.assertEqual(1, sync.hold("visual_disagreement", 1.1)["epoch"])
        self.assertEqual(1, sync.hold("duplicate_hold", 1.2)["epoch"])
        self.assertFalse(ready(sync, "r1", 2, 1.3, epoch=0))
        self.assertTrue(ready(sync, "r1", 2, 1.3))
        self.assertEqual("HOLD", sync.authorize(1.3)["phase"])
        self.assertTrue(ready(sync, "r3", 2, 1.4))
        decision = sync.authorize(1.4)
        self.assertEqual("GO", decision["phase"])
        self.assertEqual("all_participants_ready_to_resume", decision["reason"])

    def test_negative_or_expired_report_revokes_go_with_new_epoch(self):
        sync = PairCarrySync("carry-1", report_ttl_s=0.5)
        ready(sync, "r1", 1, 2.0)
        ready(sync, "r3", 1, 2.0)
        sync.authorize(2.0)
        self.assertTrue(ready(sync, "r1", 2, 2.1, value=False))
        self.assertEqual({"phase": "HOLD", "epoch": 1, "reason": "not_ready:r1"},
                         sync.authorize(2.1))

        ready(sync, "r1", 3, 3.0)
        ready(sync, "r3", 2, 3.0)
        self.assertEqual("GO", sync.authorize(3.0)["phase"])
        expired = sync.authorize(3.51)
        self.assertEqual("HOLD", expired["phase"])
        self.assertEqual(2, expired["epoch"])
        self.assertIn("report_expired", expired["reason"])

    def test_rejects_wrong_identity_version_epoch_order_frame_and_time(self):
        sync = PairCarrySync("carry-1")
        self.assertFalse(ready(sync, "r2", 1, 1.0))
        self.assertFalse(ready(sync, "r1", 1, 1.0, version=2))
        self.assertFalse(ready(sync, "r1", 1, 1.0, epoch=3))
        self.assertTrue(ready(sync, "r1", 2, 2.0, frame="camera-a"))
        self.assertFalse(ready(sync, "r1", 2, 2.1))
        self.assertFalse(ready(sync, "r1", 3, 2.2, frame="camera-a"))
        self.assertFalse(sync.report("r1", plan_version=1, epoch=0, sequence=4,
                                     ready=True, observed_at_s=2.0, received_at_s=1.9,
                                     frame_id="camera-b"))
        self.assertFalse(sync.report("r1", plan_version=1, epoch=0, sequence=4,
                                     ready=True, observed_at_s=math.nan, received_at_s=2.3,
                                     frame_id="camera-c"))
        self.assertFalse(sync.report("r1", plan_version=True, epoch=0, sequence=4,
                                     ready=True, observed_at_s=2.2, received_at_s=2.3,
                                     frame_id="camera-d"))
        self.assertFalse(sync.report("r1", plan_version=1, epoch=False, sequence=4,
                                     ready=True, observed_at_s=2.2, received_at_s=2.3,
                                     frame_id="camera-e"))

    def test_old_observation_cannot_be_replayed_with_a_fresh_receipt(self):
        sync = PairCarrySync("carry-1", report_ttl_s=0.5)
        self.assertFalse(sync.report("r1", plan_version=1, epoch=0, sequence=1,
                                     ready=True, observed_at_s=1.0, received_at_s=2.0,
                                     frame_id="old-frame"))
        ready(sync, "r1", 2, 2.0)
        ready(sync, "r3", 1, 2.0)
        self.assertEqual("GO", sync.authorize(2.0)["phase"])
        with self.assertRaisesRegex(ValueError, "backwards"):
            sync.authorize(1.9)

    def test_plan_update_invalidates_old_readiness_and_abort_is_terminal(self):
        sync = PairCarrySync("carry-1")
        ready(sync, "r1", 1, 1.0)
        ready(sync, "r3", 1, 1.0)
        sync.authorize(1.0)
        self.assertEqual("HOLD", sync.update_plan(2, 1.1)["phase"])
        self.assertEqual(1, sync.epoch)
        self.assertFalse(ready(sync, "r1", 2, 1.2, version=1))
        self.assertEqual("ABORT", sync.abort("operator_stop", 1.3)["phase"])
        self.assertFalse(ready(sync, "r1", 3, 1.4))
        self.assertEqual("ABORT", sync.hold("ignored", 1.5)["phase"])


class SharedResourceLedgerTests(unittest.TestCase):
    def _reserve_pair(self, ledger, resource="aisle-a", task="carry-1", version=1, now=1.0):
        self.assertTrue(ledger.reserve(resource, task_id=task, plan_version=version,
                                       participant="r1", now_s=now))
        self.assertTrue(ledger.reserve(resource, task_id=task, plan_version=version,
                                       participant="r3", now_s=now))

    def test_occupied_resource_never_expires_and_cannot_be_stolen(self):
        ledger = SharedResourceLedger(reservation_ttl_s=0.5)
        self._reserve_pair(ledger)
        self.assertTrue(ledger.occupy("aisle-a", task_id="carry-1", plan_version=1, now_s=1.1))
        self.assertTrue(ledger.state("aisle-a", 1000.0)["occupied"])
        self.assertFalse(ledger.reserve("aisle-a", task_id="carry-2", plan_version=1,
                                        participant="r1", now_s=1000.0))

    def test_release_requires_exact_owner_version_and_all_acknowledgments(self):
        ledger = SharedResourceLedger()
        self._reserve_pair(ledger)
        ledger.occupy("aisle-a", task_id="carry-1", plan_version=1, now_s=1.1)
        self.assertFalse(ledger.release("aisle-a", task_id="carry-1", plan_version=2,
                                        participant="r1", now_s=1.2))
        self.assertFalse(ledger.release("aisle-a", task_id="carry-2", plan_version=1,
                                        participant="r1", now_s=1.2))
        self.assertFalse(ledger.release("aisle-a", task_id="carry-1", plan_version=1,
                                        participant="r1", now_s=1.2))
        self.assertIsNotNone(ledger.state("aisle-a", 1.3))
        self.assertTrue(ledger.release("aisle-a", task_id="carry-1", plan_version=1,
                                       participant="r3", now_s=1.3))
        self.assertIsNone(ledger.state("aisle-a", 1.3))

    def test_unoccupied_reservation_expires(self):
        ledger = SharedResourceLedger(reservation_ttl_s=0.5)
        self.assertTrue(ledger.reserve("aisle-a", task_id="old", plan_version=1,
                                       participant="r1", now_s=1.0))
        self.assertTrue(ledger.reserve("aisle-a", task_id="new", plan_version=1,
                                       participant="r1", now_s=1.51))
        self.assertEqual("new", ledger.state("aisle-a", 1.51)["task_id"])


if __name__ == "__main__":
    unittest.main()
