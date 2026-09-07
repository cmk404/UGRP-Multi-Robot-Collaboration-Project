import copy
import unittest
from unittest.mock import patch

from harness.coela_modules import Communication, Memory, Planner, PlanningError, actor_context
from sim.mixed_warehouse import MixedEngine
from sim.multi_masterpi_production import MultiMasterPiProductionV2


class CommunicationTests(unittest.TestCase):
    def test_intent_expiry_uses_receiver_clock_and_preserves_identity_evidence(self):
        now = [10.]
        memory = Memory("r3", clock=lambda: now[0])
        event = Communication("natural").send("r2", {"recipients": ["r3"],
            "content": "small_box_01을 제안합니다", "observed_ids": ["small_box_01"]}, 7.71)
        memory.receive(event)
        self.assertEqual(len(memory.active_reports()), 1)
        now[0] = 23.  # Physics may remain at7.71 while every robot waits.
        self.assertEqual(memory.active_reports(), [])
        snapshot = memory.snapshot()
        self.assertEqual(snapshot["expired_peer_intents"][0]["status"], "EXPIRED_UNCONFIRMED_INTENT")
        self.assertEqual(snapshot["peer_observations"][0]["cargo_id"], "small_box_01")

    def test_confirmed_completion_report_is_not_an_expiring_intent(self):
        now = [0.]
        memory = Memory("r3", clock=lambda: now[0])
        event = Communication("structured").send("r2", {"recipients": ["r3"],
            "content": {"kind": "completed", "cargo_id": "small_box_01"}}, 0.)
        memory.receive(event);now[0] = 100.
        self.assertEqual(len(memory.active_reports()), 1)
    def test_natural_negative_mention_does_not_make_observation_receipt(self):
        bus = Communication("natural")
        e = bus.send("r1", {"recipients": ["r2"], "content": "I have not observed small_box_01"}, 0.)
        self.assertEqual(e["observed_ids"], [])
        for mode, content in (("natural", "small_box_01을 관측했습니다"),
                              ("structured", {"kind": "observation"})):
            e = Communication(mode).send("r1", {"recipients": ["r2"], "content": content, "observed_ids": ["small_box_01"]}, 0.)
            self.assertEqual(e["observed_ids"], ["small_box_01"])
    def test_none_cannot_send_but_allows_silence(self):
        bus = Communication("none")
        self.assertIsNone(bus.send("r1", None, 0))
        with self.assertRaises(ValueError):
            bus.send("r1", {"recipients": ["r2"], "content": "hello"}, 0)
        self.assertEqual(bus.receive("r2", 1), [])
        self.assertEqual(bus.sent, [])

    def test_recipients_and_budget_are_enforced(self):
        bus = Communication("natural", max_messages=1)
        sent = bus.send("r1", {"recipients": ["r2"], "content": "R2만 받는 메시지"}, 1)
        self.assertEqual(bus.receive("r3", 2), [])
        received = bus.receive("r2", 2)
        self.assertEqual(received[0]["message_id"], sent["message_id"])
        self.assertEqual(received[0]["received_sim_time"], 2)
        self.assertEqual(bus.receive("r2", 3), [])
        with self.assertRaises(ValueError): bus.send("r1", {"recipients": ["r2"], "content": "again"}, 3)

    def test_structured_has_no_free_text_backdoor(self):
        bus = Communication("structured")
        for content in ({"kind": "intent", "text": "hidden free text"},
                        {"kind": "intent", "cargo_id": "hidden free text"},
                        {"kind": "recovery", "reason_code": "arbitrary story"}):
            with self.assertRaises(ValueError): bus.send("r1", {"recipients": ["r2"], "content": content}, 1)
        self.assertIsNotNone(bus.send("r1", {"recipients": ["r2"], "content": {"kind": "recovery", "reason_code": "obstacle_changed", "action": "replan"}}, 1))

    def test_reports_do_not_overwrite_direct_measurements(self):
        memory = Memory("r1")
        memory.observe({"observation_id": "o1", "cargo": [{"cargo_id": "x", "current_view": True, "range_m": 1.}]}, 2.)
        memory.receive({"sender_id": "r2", "content": "x is 100 metres away"})
        self.assertEqual(memory.snapshot()["observations"][0]["range_m"], 1.)
        self.assertEqual(len(memory.snapshot()["peer_reports"]), 1)

    def test_preflight_delay_does_not_make_old_observation_fresh(self):
        memory = Memory("r1")
        memory.observe({"observation_id": "o1", "sim_time": 2., "cargo": [{"cargo_id": "x", "current_view": True}]}, 8.)
        self.assertEqual(memory.snapshot()["observations"][0]["last_seen_sim_time"], 2.)

    def test_invalid_llm_reply_keeps_raw_evidence_and_usage(self):
        class Completer:
            model_name = "fixture"
            last_usage = {"completion_tokens": 8}
            def complete(self, context): return '{"action":'
        with self.assertRaises(PlanningError) as caught: Planner("r1", Completer()).decide({})
        self.assertEqual(caught.exception.raw, '{"action":')
        self.assertEqual(caught.exception.usage["completion_tokens"], 8)


class LocalInformationTests(unittest.TestCase):
    def test_retiring_planner_clears_old_consent_and_cannot_join_new_task(self):
        e = self.engine
        p = {"robot_id": "r3", "cargo_id": "oak_plank_01", "participants": ["r1", "r3"], "destination": "B", "revision": 1, "observation_id": "r3"}
        e.referee.observe({"robot_id": "r3", "cargo_ids": ["oak_plank_01"], "revision": 1, "observation_id": "r3"})
        e.referee.submit([p])
        self.assertIn("r3", e.referee.pending)
        e.retire("r3")
        self.assertNotIn("r3", e.referee.pending)
        with self.assertRaisesRegex(ValueError, "PARTICIPANT_UNAVAILABLE"):
            e.submit([{**p, "robot_id": "r1", "observation_id": "r1"}])
        self.assertFalse(e.referee.active_assignments)
    def setUp(self):
        self.world = MultiMasterPiProductionV2(warehouse_layout="mixed", render=False)
        self.engine = MixedEngine(self.world, information_mode="strict_local")

    def tearDown(self): self.world.close()

    def test_hidden_completion_changes_neither_local_status_nor_memory(self):
        engine = self.engine
        engine.memories["r1"]["small_box_01"] = {"cargo_id": "small_box_01"}
        engine.scan_initialized.add("r1")
        before = engine.local_status("r1")
        with patch("sim.warehouse_observation.observe_robot", return_value={"cargo": []}):
            first = engine.observe("r1")
            engine.completed.add("small_box_01")
            engine.history.append({"assignment_id": "foreign", "event": "completed", "cargo_id": "small_box_01", "participants": ["r3"]})
            second = engine.observe("r1")
        self.assertEqual(before, engine.local_status("r1"))
        self.assertEqual(first["cargo"], second["cargo"])
        self.assertEqual(second["cargo"][0]["cargo_id"], "small_box_01")

    def test_unrelated_robot_reservation_does_not_change_actor_context(self):
        e = self.engine
        local = e.local_status("r1")
        memory = Memory("r1")
        obs = {"cargo": [], "sim_time": local["sim_time"]}
        before = actor_context("r1", e.manifest, "B", "none", obs, memory, local)
        e.referee.observe({"robot_id": "r3", "cargo_ids": ["small_box_01"], "revision": 1, "observation_id": "o3"})
        e.referee.submit([{"robot_id": "r3", "cargo_id": "small_box_01", "participants": ["r3"], "destination": "B", "revision": 1, "observation_id": "o3"}])
        after = actor_context("r1", e.manifest, "B", "none", obs, memory, e.local_status("r1"))
        self.assertEqual(before, after)
        self.assertNotIn("r3", e.status()["idle_robots"])

    def test_private_recovery_evidence_only_reaches_observer(self):
        e = self.engine
        proposals = []
        for rid in ("r1", "r2"):
            e.referee.observe({"robot_id": rid, "cargo_ids": ["oak_plank_01"], "revision": 1, "observation_id": rid})
            proposals.append({"robot_id": rid, "cargo_id": "oak_plank_01", "participants": ["r1", "r2"], "destination": "B", "revision": 1, "observation_id": rid})
        a = e.referee.submit(proposals).accepted[0]
        e.pause_assignment(a.assignment_id, "obstacle_changed", {"known_to": ["r1"], "position_xy": [.72, -.64]})
        self.assertEqual(e.local_status("r1")["recovery_events"][0]["reason"], "obstacle_changed")
        hidden = e.local_status("r2")["recovery_events"][0]
        self.assertEqual(hidden["reason"], "cause_unknown")
        self.assertNotIn("position_xy", hidden["evidence"])
        self.assertEqual(e.local_status("r3")["recovery_events"], [])


if __name__ == "__main__": unittest.main()
