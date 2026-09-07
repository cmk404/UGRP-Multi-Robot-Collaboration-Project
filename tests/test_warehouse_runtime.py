import json
import threading
import time
import unittest

from harness.warehouse_runtime import EpisodeBudget, RulePolicy, parse_reply, run_episode
from harness.warehouse_protocol import LocalObservation, Proposal, WarehouseProtocol


class FakeEnvironment:
    def __init__(self, *, complete=True):
        self.complete = complete
        self.execute_calls = []
        self.global_secret = "must never enter actor context"

    def request(self, robot_id, request):
        op = request["operation"]
        if op == "begin":
            return {"episode_id": "ep-1", "world_seed": request["sensor_seed"],
                    "revision": 1, "destination": "B", "complete": False,
                    "delivered_ids": [], "remaining_ids": ["crate-1"],
                    "attempts": 0, "manifest": [{"cargo_id": "crate-1"}]}
        if op == "observe":
            return {"robot_id": robot_id, "observation_id": f"obs-{robot_id}",
                    "revision": 1, "destination": "B",
                    "cargo": [{"cargo_id": "crate-1"}],
                    "capabilities": {"can_carry": True}}
        if op == "execute":
            self.execute_calls.append(request)
            plans = request["proposals"]
            self.last_assignments = plans["r1"]["assignments"]
            return {"ok": self.complete, "reason": "CARGO_DELIVERED" if self.complete else "AGREEMENT_REJECTED",
                    "complete": self.complete, "delivered_ids": ["crate-1"] if self.complete else [],
                    "remaining_ids": [] if self.complete else ["crate-1"], "executed_cargo_id": "crate-1" if self.complete else None}
        if op == "status":
            return {"complete": self.complete, "delivered_ids": ["crate-1"] if self.complete else [],
                    "remaining_ids": [] if self.complete else ["crate-1"]}
        raise AssertionError(op)


class RecordingCompleter:
    model_name = "test-completer"
    def __init__(self, robot_id, *, barrier=None, malformed=False, log=None):
        self.robot_id, self.barrier, self.malformed, self.log = robot_id, barrier, malformed, log

    def decide(self, context):
        self.log.append((self.robot_id, context)) if self.log is not None else None
        if self.barrier is not None:
            self.barrier.wait(timeout=2)
        if self.malformed:
            return {}
        assignments = {"carrier_0": "r1", "carrier_1": "r2", "scout": "r3"}
        return {"robot_id": self.robot_id, "role": next(k for k, v in assignments.items() if v == self.robot_id),
                           "cargo_id": "crate-1", "destination": "B", "revision": 1,
                           "observation_id": f"obs-{self.robot_id}", "assignments": assignments,
                           "message": f"{self.robot_id} sees crate-1 and proposes its role"}


class WarehouseRuntimeTests(unittest.TestCase):
    def budget(self, rounds=1):
        return EpisodeBudget(max_rounds=rounds, max_actions=1, response_tokens=128, call_timeout_s=3)

    def test_no_comm_actor_context_has_only_local_observation(self):
        env = FakeEnvironment()
        seen = []
        policies = {rid: RecordingCompleter(rid, log=seen) for rid in ("r1", "r2", "r3")}
        result = run_episode(env, policies, condition="llm_no_comm", budget=self.budget())
        self.assertTrue(result["success"])
        self.assertEqual(len(env.execute_calls), 1)
        for _, context in seen:
            self.assertEqual(context["inbox"], [])
            self.assertNotIn("global_secret", context)
            self.assertIn("observation", context)

    def test_three_completers_really_overlap_and_authored_roster_is_used(self):
        env = FakeEnvironment()
        barrier = threading.Barrier(3)
        seen = []
        policies = {rid: RecordingCompleter(rid, barrier=barrier, log=seen) for rid in ("r1", "r2", "r3")}
        result = run_episode(env, policies, condition="llm_peer_comm", budget=self.budget())
        self.assertTrue(result["success"])
        self.assertEqual(env.last_assignments, {"carrier_0": "r1", "carrier_1": "r2", "scout": "r3"})
        self.assertEqual(len(seen), 3)

    def test_missing_decision_never_reaches_environment_execute(self):
        env = FakeEnvironment()
        policies = {"r1": RecordingCompleter("r1", malformed=True),
                    "r2": RecordingCompleter("r2"), "r3": RecordingCompleter("r3")}
        result = run_episode(env, policies, condition="llm_peer_comm", budget=self.budget())
        self.assertFalse(result["success"])
        self.assertEqual(env.execute_calls, [])
        self.assertEqual(result["invalid_decisions"], 1)

    def test_reply_envelope_stamps_exact_observation_metadata(self):
        raw = {"robot_id": "r1", "role": "carrier_0", "cargo_id": "crate-1",
               "destination": "B", "revision": 999, "observation_id": "forged",
               "assignments": {"carrier_0": "r1", "carrier_1": "r2", "scout": "r3"},
               "message": "local evidence"}
        _, parsed, _ = parse_reply(raw, "r1",
                                   {"revision": 4, "observation_id": "obs-exact"})
        self.assertEqual(parsed["revision"], 4)
        self.assertEqual(parsed["observation_id"], "obs-exact")

    def test_invalid_r0_roster_is_rejected_before_environment_execute(self):
        env = FakeEnvironment()
        class BadRoster(RecordingCompleter):
            def decide(self, context):
                if self.robot_id == "r1":
                    return {"robot_id": "r1", "role": "carrier_0", "cargo_id": "crate-1",
                            "destination": "B", "assignments": {"carrier_0": "r0", "carrier_1": "r2", "scout": "r3"},
                            "message": "invalid roster"}
                return super().decide(context)
        policies = {rid: BadRoster(rid) for rid in ("r1", "r2", "r3")}
        result = run_episode(env, policies, condition="llm_peer_comm", budget=self.budget())
        self.assertFalse(result["success"])
        self.assertEqual(env.execute_calls, [])

    def test_llm_peer_inbox_contains_only_opaque_natural_language_messages(self):
        env = FakeEnvironment()
        seen = []
        policies = {rid: RecordingCompleter(rid, log=seen) for rid in ("r1", "r2", "r3")}
        env.complete = False
        result = run_episode(env, policies, condition="llm_peer_comm",
                             budget=EpisodeBudget(max_rounds=2, max_actions=2,
                                                  response_tokens=128, call_timeout_s=3))
        self.assertFalse(result["success"])
        peer_contexts = [context for _, context in seen if context["inbox"]]
        self.assertTrue(peer_contexts)
        for context in peer_contexts:
            for message in context["inbox"]:
                self.assertEqual(set(message), {"sender_id", "message"})

    def test_mixed_observation_revisions_fail_before_transport(self):
        protocol = WarehouseProtocol()
        local = {rid: LocalObservation(rid, ("crate-1",), ("green",), 2, f"obs-{rid}-2")
                 for rid in ("r1", "r2", "r3")}
        round_id = protocol.start_round(local, "crate-1", "green", revision=2)
        roster = {"carrier_0": "r1", "carrier_1": "r2", "scout": "r3"}
        for rid, role in zip(("r1", "r2", "r3"), ("carrier_0", "carrier_1", "scout")):
            protocol.submit_proposal(round_id, Proposal(rid, role, "crate-1", "green", 1,
                                                         "stale", assignments=roster,
                                                         observation_id=f"obs-{rid}-2"))
        result = protocol.evaluate(round_id)
        self.assertFalse(result.accepted)
        self.assertIn("STALE_REVISION", result.reason_codes)

    def test_invalid_reply_usage_is_counted_and_overbudget_reply_never_executes(self):
        env = FakeEnvironment()
        class OverBudget(RecordingCompleter):
            def decide(self, context):
                return {"raw": "{}", "usage": {"total_tokens": 9, "completion_tokens": 129}}
        policies = {"r1": OverBudget("r1"), "r2": RecordingCompleter("r2"), "r3": RecordingCompleter("r3")}
        result = run_episode(env, policies, condition="llm_peer_comm",
                             budget=EpisodeBudget(max_rounds=1, max_actions=1, response_tokens=128, call_timeout_s=3))
        self.assertFalse(result["success"])
        self.assertEqual(env.execute_calls, [])
        self.assertEqual(result["known_tokens_total"], 9)

    def test_single_cargo_goal_revision_executes_second_action(self):
        class RevisionEnvironment(FakeEnvironment):
            def __init__(self):
                super().__init__(complete=False)
                self.phase = 0
            def request(self, robot_id, request):
                if request["operation"] == "revise":
                    self.phase = 1
                    return {"episode_id": "ep-1", "revision": 2, "destination": "C", "complete": False,
                            "delivered_ids": [], "remaining_ids": ["crate-1"]}
                if request["operation"] == "observe":
                    value = super().request(robot_id, request)
                    value["destination"] = "C" if self.phase else "B"
                    value["revision"] = 2 if self.phase else 1
                    value["observation_id"] = f"obs-{robot_id}-{self.phase}"
                    return value
                if request["operation"] == "execute":
                    self.execute_calls.append(request)
                    self.phase += 1
                    done = self.phase >= 2
                    return {"ok": done, "reason": "CARGO_DELIVERED" if done else "REFEREE_FAILED",
                            "complete": done, "delivered_ids": ["crate-1"] if done else [],
                            "remaining_ids": [] if done else ["crate-1"],
                            "executed_cargo_id": "crate-1"}
                return super().request(robot_id, request)
        class RevisionPolicy(RecordingCompleter):
            def decide(self, context):
                destination = context["observation"]["destination"]
                revision = context["observation"]["revision"]
                value = super().decide(context)
                value["destination"], value["revision"] = destination, revision
                value["observation_id"] = context["observation"]["observation_id"]
                return value
        env = RevisionEnvironment()
        policies = {rid: RevisionPolicy(rid) for rid in ("r1", "r2", "r3")}
        result = run_episode(env, policies, condition="llm_peer_comm", scenario="goal_revision",
                             budget=EpisodeBudget(max_rounds=3, max_actions=2, response_tokens=128, call_timeout_s=3))
        self.assertTrue(result["success"])
        self.assertEqual(len(env.execute_calls), 2)


if __name__ == "__main__":
    unittest.main()
