"""Contract fixtures only; these do not count as LLM or physics evidence."""
import copy
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from harness.coela_runtime import run_coela_episode
from harness.coela_modules import Execution, Memory as CoelaMemory


class FixtureEnvironment:
    def __init__(self):
        self.done = set()
        self.retired = set()
        self.lock = threading.Lock()
        self.requests = []

    def request(self, rid, request):
        with self.lock:
            self.requests.append((rid, copy.deepcopy(request)))
            op = request["operation"]
            if op == "mixed_begin":
                return {"episode_id": "test", "manifest": [{"cargo_id": "small_box_01", "required_carriers": 1}]}
            if op == "mixed_observe":
                return {"robot_id": rid, "observation_id": rid, "revision": 1, "sim_time": 0.,
                        "cargo": [{"cargo_id": "small_box_01", "current_view": True}], "private_debug": "must be stripped"}
            if op == "mixed_local_status":
                return {"ok": True, "robot_id": rid, "episode_id": "test", "revision": 1, "sim_time": 0.,
                        "active": None, "pending": None, "history": [], "recovery_events": []}
            if op in ("mixed_status", "mixed_stop"):
                return {"ok": True, "complete": len(self.done) == 3, "active": [],
                        "available_ids": ["SECRET_GLOBAL"], "idle_robots": ["SECRET_GLOBAL"],
                        "history": [{"secret": len(self.requests)}]}
            if op == "mixed_local_submit":
                self.done.add(rid)
                return {"ok": True, "accepted": [], "rejected": []}
            if op == "mixed_retire":
                self.retired.add(rid)
                return {"ok": True, "retired": rid}
            raise AssertionError(op)


class FixturePlanner:
    model_name = "test_fixture"
    evidence_kind = "fixture"
    def __init__(self, rid, mode, timings):
        self.rid, self.mode, self.timings, self.calls = rid, mode, timings, 0

    def decide(self, context):
        if self.rid == "r1": time.sleep(.12)
        self.timings[self.rid] = time.monotonic()
        self.calls += 1
        content = "small_box_01을 확인했습니다" if self.mode == "natural" else {"kind": "observation", "observed_ids": ["small_box_01"]}
        message = None if self.mode == "none" or self.calls > 1 else {"recipients": [r for r in ("r1", "r2", "r3") if r != self.rid], "content": content, "observed_ids": ["small_box_01"]}
        proposed = any(d.get("execution_result", {}).get("proposed")
                       for d in context["memory"]["own_decisions"])
        kind = "claim" if proposed else "propose"
        return {"decision": {"action": {"kind": kind, "cargo_id": "small_box_01", "participants": [self.rid]}, "message": message}, "usage": None}


class RuntimeTests(unittest.TestCase):
    def test_remote_supervisor_changes_do_not_wake_idle_actors(self):
        class WaitingPlanner:
            model_name = "waiting_fixture"
            evidence_kind = "fixture"
            def decide(self, context):
                return {"decision": {"action": {"kind": "wait"}, "message": None}, "usage": None}
        env = FixtureEnvironment()
        result = run_coela_episode(env, {r: WaitingPlanner() for r in ("r1", "r2", "r3")},
            mode="none", timeout_s=.35, decision_period_s=10., max_calls_per_robot=3)
        self.assertEqual(result["calls"], {"r1": 1, "r2": 1, "r3": 1})
        self.assertFalse(result["success"])

    def test_all_channels_use_local_context_and_do_not_wait_for_slow_peer(self):
        for mode in ("none", "structured", "natural"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                env, timings = FixtureEnvironment(), {}
                path = Path(directory)/"episode.jsonl"
                result = run_coela_episode(env, {r: FixturePlanner(r, mode, timings) for r in ("r1", "r2", "r3")},
                    mode=mode, timeout_s=3., max_calls_per_robot=6, journal_path=path)
                self.assertTrue(result["success"])
                self.assertLess(timings["r2"], timings["r1"])
                self.assertLess(timings["r3"], timings["r1"])
                self.assertEqual(result["messages"] == 0, mode == "none")
                rows = [json.loads(line) for line in path.read_text().splitlines()]
                for row in rows:
                    if row["event"] == "actor_input":
                        text = json.dumps(row["context"])
                        self.assertNotIn("SECRET_GLOBAL", text)
                        self.assertNotIn("private_debug", text)
                        self.assertNotIn("available_ids", row["context"])
                actor_ops = [req["operation"] for _, req in env.requests]
                self.assertIn("mixed_local_submit", actor_ops)
                self.assertNotIn("mixed_submit", actor_ops)
                preflight = [(rid, req["operation"]) for rid, req in env.requests[:7]]
                self.assertEqual(preflight, [("r1", "mixed_begin"),
                    ("r1", "mixed_observe"), ("r2", "mixed_observe"), ("r3", "mixed_observe"),
                    ("r1", "mixed_observe"), ("r2", "mixed_observe"), ("r3", "mixed_observe")])

    def test_slow_reply_after_deadline_has_no_message_or_action(self):
        class ClosingEnvironment(FixtureEnvironment):
            def __init__(self):
                super().__init__()
                self.closed = False
                self.post_close_requests = []
            def request(self, rid, request):
                if self.closed:
                    self.post_close_requests.append(request["operation"])
                    raise AssertionError("WORLD_ALREADY_CLOSED")
                result = super().request(rid, request)
                if request["operation"] == "mixed_stop":
                    self.closed = True
                return result
        class SlowPlanner:
            model_name = "slow_fixture"
            evidence_kind = "fixture"
            def decide(self, context):
                time.sleep(.2)
                return {"decision": {"action": {"kind": "claim", "cargo_id": "small_box_01",
                    "participants": [context["robot_id"]]}, "message": {"recipients": [
                    r for r in ("r1", "r2", "r3") if r != context["robot_id"]],
                    "content": "late small_box_01"}}, "raw": "late", "usage": {"tokens": 7}}
        env = ClosingEnvironment()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"deadline.jsonl"
            result = run_coela_episode(env, {r: SlowPlanner() for r in ("r1", "r2", "r3")},
                mode="natural", timeout_s=.08, journal_path=path)
            self.assertLess(result["elapsed_s"], .18)
            frozen = copy.deepcopy(result)
            self.assertEqual(result["pending_model_replies"], ["r1", "r2", "r3"])
            self.assertEqual(set(result["pending_usage"]), {"r1", "r2", "r3"})
            time.sleep(.18)
            self.assertFalse(any(req["operation"] == "mixed_local_submit" for _, req in env.requests))
            self.assertEqual(result["messages"], 0)
            self.assertEqual(result, frozen)
            self.assertEqual(env.post_close_requests, [])
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertTrue(any(row["event"] == "late_reply" and row.get("raw") == "late" for row in rows))

    def test_reply_is_discarded_when_own_state_changes(self):
        class StateChangingEnvironment(FixtureEnvironment):
            def __init__(self):
                super().__init__()
                self.status_reads = {r: 0 for r in ("r1", "r2", "r3")}
            def request(self, rid, request):
                if request["operation"] == "mixed_local_status":
                    self.status_reads[rid] += 1
                    active = None if self.status_reads[rid] < 3 else {"assignment_id": "peer-started"}
                    return {"ok": True, "robot_id": rid, "episode_id": "test", "revision": 1,
                            "sim_time": 0., "active": active, "pending": None, "history": [],
                            "recovery_events": []}
                return super().request(rid, request)
        class Planner:
            model_name = "state_fixture"
            evidence_kind = "fixture"
            def decide(self, context):
                time.sleep(.02)
                return {"decision": {"action": {"kind": "claim", "cargo_id": "small_box_01",
                    "participants": [context["robot_id"]]}, "message": None}, "raw": "stale", "usage": None}
        env = StateChangingEnvironment()
        result = run_coela_episode(env, {r: Planner() for r in ("r1", "r2", "r3")},
            mode="none", timeout_s=.12, max_calls_per_robot=1)
        self.assertFalse(any(req["operation"] == "mixed_local_submit" for _, req in env.requests))
        self.assertEqual(result["messages"], 0)
        self.assertEqual(env.retired, {"r1", "r2", "r3"})

    def test_recovery_reply_is_discarded_after_event_resolves(self):
        class ResolvedEnvironment(FixtureEnvironment):
            def __init__(self):
                super().__init__()
                self.reads = {r: 0 for r in ("r1", "r2", "r3")}
                self.recover_calls = 0
            def request(self, rid, request):
                if request["operation"] == "mixed_local_status":
                    self.reads[rid] += 1
                    recovering = self.reads[rid] < 3
                    return {"ok": True, "robot_id": rid, "episode_id": "test", "revision": 1,
                            "sim_time": 0., "active": {"assignment_id": "a"} if recovering else None,
                            "pending": None, "history": [], "recovery_events": [
                                {"event_id": "e"*32}] if recovering else []}
                if request["operation"] == "mixed_recover":
                    self.recover_calls += 1
                    return {"ok": True}
                return super().request(rid, request)
        class RecoveryPlanner:
            model_name = "resolved_fixture"
            evidence_kind = "fixture"
            def decide(self, context):
                time.sleep(.02)
                if not context["own_recovery_events"]:
                    return {"decision": {"action": {"kind": "wait"}, "message": None},
                            "raw": "current wait", "usage": None}
                return {"decision": {"action": {"kind": "recover", "event_id": "e"*32,
                    "recovery_action": "replan"}, "message": None}, "raw": "stale recovery",
                    "usage": None}
        env = ResolvedEnvironment()
        run_coela_episode(env, {r: RecoveryPlanner() for r in ("r1", "r2", "r3")},
            mode="none", timeout_s=.12, max_calls_per_robot=2)
        self.assertEqual(env.recover_calls, 0)

    def test_reply_is_discarded_when_own_history_changes(self):
        class CompletedEnvironment(FixtureEnvironment):
            def __init__(self):
                super().__init__()
                self.reads = {r: 0 for r in ("r1", "r2", "r3")}
            def request(self, rid, request):
                if request["operation"] == "mixed_local_status":
                    self.reads[rid] += 1
                    history = [] if self.reads[rid] < 3 else [{"assignment_id": "a", "event": "completed"}]
                    return {"ok": True, "robot_id": rid, "episode_id": "test", "revision": 1,
                            "sim_time": 0., "active": None, "pending": None,
                            "history": history, "recovery_events": []}
                return super().request(rid, request)
        class Planner:
            model_name = "history_fixture"
            evidence_kind = "fixture"
            def decide(self, context):
                time.sleep(.02)
                return {"decision": {"action": {"kind": "claim", "cargo_id": "small_box_01",
                    "participants": [context["robot_id"]]}, "message": None}, "usage": None}
        env = CompletedEnvironment()
        run_coela_episode(env, {r: Planner() for r in ("r1", "r2", "r3")},
            mode="none", timeout_s=.1, max_calls_per_robot=1)
        self.assertFalse(any(req["operation"] == "mixed_local_submit" for _, req in env.requests))

    def test_budget_reserves_calls_for_recovery(self):
        class RecoveryEnvironment(FixtureEnvironment):
            def __init__(self):
                super().__init__()
                self.recovery = False
                self.recovery_calls = 0
            def request(self, rid, request):
                if request["operation"] == "mixed_local_status":
                    event = [{"event_id": "e"*32}] if self.recovery else []
                    return {"ok": True, "robot_id": rid, "episode_id": "test", "revision": 1,
                            "sim_time": 0., "active": {"assignment_id": "a"} if self.recovery else None,
                            "pending": None,
                            "history": [], "recovery_events": event}
                if request["operation"] == "mixed_recover":
                    self.recovery_calls += 1
                    return {"ok": True, "awaiting": ["partner"]}
                return super().request(rid, request)
        class RecoveryPlanner:
            model_name = "recovery_fixture"
            evidence_kind = "fixture"
            def __init__(self, env): self.env = env
            def decide(self, context):
                if context["own_recovery_events"]:
                    return {"decision": {"action": {"kind": "recover", "event_id": "e"*32,
                        "recovery_action": "replan"}, "message": None}, "usage": None}
                self.env.recovery = True
                return {"decision": {"action": {"kind": "wait"}, "message": None}, "usage": None}
        env = RecoveryEnvironment()
        result = run_coela_episode(env, {r: RecoveryPlanner(env) for r in ("r1", "r2", "r3")},
            mode="none", timeout_s=.2, max_calls_per_robot=4, decision_period_s=.01)
        self.assertEqual(result["recovery_call_reserve"], 1)
        self.assertGreater(env.recovery_calls, 0)
        self.assertTrue(all(count <= 4 for count in result["calls"].values()))

    def test_pending_intent_is_retired_when_call_budget_ends(self):
        class PendingEnvironment(FixtureEnvironment):
            def request(self, rid, request):
                if request["operation"] == "mixed_local_status":
                    return {"ok": True, "robot_id": rid, "episode_id": "test", "revision": 1,
                            "sim_time": 0., "active": None,
                            "pending": {"cargo_id": "oak_plank_01", "participants": [rid, "r3"]},
                            "history": [], "recovery_events": []}
                return super().request(rid, request)
        class WaitingPlanner:
            model_name = "pending_fixture"
            evidence_kind = "fixture"
            def decide(self, context):
                return {"decision": {"action": {"kind": "wait"}, "message": None}, "usage": None}
        env = PendingEnvironment()
        run_coela_episode(env, {r: WaitingPlanner() for r in ("r1", "r2", "r3")},
            mode="none", timeout_s=.2, max_calls_per_robot=1)
        self.assertEqual(env.retired, {"r1", "r2", "r3"})

    def test_new_accepted_history_wakes_busy_actor_once_and_blocks_claim(self):
        class AcceptedEnvironment(FixtureEnvironment):
            def request(self, rid, request):
                if request["operation"] == "mixed_local_status":
                    return {"ok": True, "robot_id": rid, "episode_id": "test", "revision": 1,
                            "sim_time": 1., "active": {"assignment_id": rid+"-active"},
                            "pending": None, "history": [{"event": "accepted",
                                "assignment_id": rid+"-active", "participants": [rid]}],
                            "recovery_events": []}
                return super().request(rid, request)
        class BusyClaimPlanner:
            model_name = "busy_notice_fixture"
            evidence_kind = "fixture"
            def decide(self, context):
                return {"decision": {"action": {"kind": "claim", "cargo_id": "small_box_01",
                    "participants": [context["robot_id"]]}, "message": None}, "usage": None}
        env = AcceptedEnvironment()
        result = run_coela_episode(env, {r: BusyClaimPlanner() for r in ("r1", "r2", "r3")},
            mode="none", timeout_s=.15, max_calls_per_robot=3, decision_period_s=10.)
        self.assertEqual(result["calls"], {"r1": 1, "r2": 1, "r3": 1})
        self.assertFalse(any(req["operation"] == "mixed_local_submit" for _, req in env.requests))

    def test_identical_pending_claim_is_idempotent(self):
        env = FixtureEnvironment()
        original = env.request
        def request(rid, req):
            if req["operation"] == "mixed_local_status":
                return {"ok": True, "pending": {"cargo_id": "oak_plank_01", "participants": ["r1", "r2"]}}
            return original(rid, req)
        env.request = request
        result = Execution(env, "r1", "test", "B").execute(
            {"kind": "claim", "cargo_id": "oak_plank_01", "participants": ["r2", "r1"]},
            {"revision": 1, "observation_id": "r1"})
        self.assertTrue(result["ok"])
        self.assertEqual(env.done, set())


    def test_claim_consent_survives_unrelated_intent_expiry_during_model_call(self):
        clocks = {rid: [0.] for rid in ("r1", "r2", "r3")}

        class UnmatchedEnvironment(FixtureEnvironment):
            def request(self, rid, request):
                if request["operation"] == "mixed_local_submit":
                    with self.lock:
                        self.requests.append((rid, copy.deepcopy(request)))
                    return {"ok": True, "accepted": [], "rejected": []}
                return super().request(rid, request)

        class ExpiringMemory(CoelaMemory):
            def __init__(self, rid):
                super().__init__(rid, clock=lambda rid=rid: clocks[rid][0])
                self.receive({"message_id": "intent-"+rid, "sender_id": "peer",
                    "recipients": [rid], "content": {"kind": "intent"},
                    "kind": "intent", "observed_ids": [], "sent_sim_time": 0.,
                    "received_sim_time": 0., "intent_ttl_s": .1})

        class ExpiringPlanner:
            model_name = "expiring_intent_fixture"
            evidence_kind = "fixture"
            def __init__(self, rid): self.rid, self.calls = rid, 0
            def decide(self, context):
                self.calls += 1
                partner = {"r1": "r2", "r2": "r3", "r3": "r1"}[self.rid]
                participants = sorted([self.rid, partner])
                if self.calls == 1:
                    action = {"kind": "propose", "cargo_id": "oak_plank_01",
                              "participants": participants}
                else:
                    self.assert_context = [m["message_id"] for m in
                        context["memory"]["peer_reports"]]
                    clocks[self.rid][0] = 1.
                    action = {"kind": "claim", "cargo_id": "oak_plank_01",
                              "participants": participants}
                return {"decision": {"action": action, "message": None}, "usage": None}

        env = UnmatchedEnvironment()
        planners = {r: ExpiringPlanner(r) for r in ("r1", "r2", "r3")}
        with tempfile.TemporaryDirectory() as directory, patch(
                "harness.coela_runtime.Memory", ExpiringMemory):
            path = Path(directory)/"expiry.jsonl"
            run_coela_episode(env, planners, mode="none", timeout_s=1.5,
                max_calls_per_robot=2, decision_period_s=.01, journal_path=path)
            rows = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertFalse(any(row["event"] == "actor_decision" and
            row.get("discard_reason") == "PEER_REPORT_EXPIRED" for row in rows))
        self.assertTrue(any(req["operation"] == "mixed_local_submit" for _, req in env.requests))
        self.assertEqual(env.done, set())

    def test_recovery_is_replanned_when_peer_intent_expires_during_model_call(self):
        clocks = {rid: [0.] for rid in ("r1", "r2", "r3")}

        class ExpiringMemory(CoelaMemory):
            def __init__(self, rid):
                super().__init__(rid, clock=lambda rid=rid: clocks[rid][0])
                self.receive({"message_id": "intent-"+rid, "sender_id": "peer",
                    "recipients": [rid], "content": {"kind": "intent"},
                    "kind": "intent", "observed_ids": [], "sent_sim_time": 0.,
                    "received_sim_time": 0., "intent_ttl_s": .1})

        class RecoveryEnvironment(FixtureEnvironment):
            def __init__(self):
                super().__init__()
                self.recover_calls = 0
            def request(self, rid, request):
                if request["operation"] == "mixed_local_status":
                    return {"ok": True, "robot_id": rid, "episode_id": "test", "revision": 1,
                            "sim_time": 0., "active": {"assignment_id": "a"}, "pending": None,
                            "history": [], "recovery_events": [{"event_id": "e"*32}]}
                if request["operation"] == "mixed_recover":
                    self.recover_calls += 1
                    return {"ok": True}
                return super().request(rid, request)

        class RecoveryPlanner:
            model_name = "expiring_recovery_fixture"
            evidence_kind = "fixture"
            def __init__(self, rid): self.rid = rid
            def decide(self, context):
                clocks[self.rid][0] = 1.
                return {"decision": {"action": {"kind": "recover", "event_id": "e"*32,
                    "recovery_action": "replan"}, "message": None}, "usage": None}

        env = RecoveryEnvironment()
        with tempfile.TemporaryDirectory() as directory, patch(
                "harness.coela_runtime.Memory", ExpiringMemory):
            path = Path(directory)/"recovery-expiry.jsonl"
            run_coela_episode(env, {r: RecoveryPlanner(r) for r in ("r1", "r2", "r3")},
                mode="none", timeout_s=.2, max_calls_per_robot=1, journal_path=path)
            rows = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertTrue(any(row["event"] == "actor_decision" and
            row.get("discard_reason") == "PEER_REPORT_EXPIRED" for row in rows))
        self.assertEqual(env.recover_calls, 0)


if __name__ == "__main__": unittest.main()
