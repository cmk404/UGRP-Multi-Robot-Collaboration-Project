"""Offline fixture evidence for the independent RGB communication runtime."""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from harness.rgb_communication_runtime import RuntimeLimits, run_rgb_communication


ROOT = Path(__file__).resolve().parents[1]
FIXTURE = json.loads(
    (ROOT / "tests/fixtures/rgb_communication/scenario.json").read_text(encoding="utf-8")
)
ROBOTS = tuple(FIXTURE["robot_ids"])


class MutableClock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class FixturePort:
    clock_domain = "sim"

    def __init__(self, *, leak_field: str | None = None, tick_secret: str = "SECRET") -> None:
        self.now_s = 0.0
        self.leak_field = leak_field
        self.tick_secret = tick_secret
        self.submissions: list[tuple[str, dict]] = []
        self.submission_history = {robot_id: [] for robot_id in ROBOTS}
        self.closed_at_s: float | None = None

    def tick(self, now_s: float) -> dict:
        self.now_s = now_s
        # Deliberate supervisor-only values; the runtime ignores this result.
        return {"timestamp_s": now_s, "completed_ids": [self.tick_secret], "terminal": False}

    def observe(self, robot_id: str) -> dict:
        result = {
            "schema": "ugrp.rgb_actor_observation.v1",
            "robot_id": robot_id,
            "observation_id": f"{robot_id}-obs-{int(self.now_s)}",
            "observed_at_s": self.now_s,
            "clock_domain": self.clock_domain,
            "images": {
                name: {
                    "ref": f"memory://{robot_id}/{int(self.now_s)}/{name}.jpg",
                    "sha256": FIXTURE["jpeg_sha256"],
                    "jpeg_base64": FIXTURE["jpeg_base64"],
                }
                for name in ("own_rgb", "top_rgb")
            },
            "static_context": copy.deepcopy(FIXTURE["static_context"]),
            "static_context_sha256": FIXTURE["static_context_sha256"],
            "own_issued_commands": [],
        }
        if self.leak_field:
            result[self.leak_field] = {"measured_joint": 1.0, "success": True}
        return result

    def local_status(self, robot_id: str) -> dict:
        return {
            "schema": "ugrp.rgb_actor_status.v1",
            "robot_id": robot_id,
            "clock_domain": self.clock_domain,
            "timestamp_s": self.now_s,
            "active": [],
            "pending": [],
            "own_submission_history": copy.deepcopy(self.submission_history[robot_id]),
        }

    def submit(self, robot_id: str, action: dict) -> dict:
        self.submissions.append((robot_id, copy.deepcopy(action)))
        return {
            "request_id": action["request_id"],
            "status": "PENDING",
            "reason": "fixture_only",
            "clock_domain": self.clock_domain,
            "timestamp_s": self.now_s,
            "task_id": action["task_id"],
        }

    def close(self, now_s: float) -> None:
        self.closed_at_s = now_s


class FixturePlanner:
    model_name = "deterministic-fixture"
    evidence_kind = "fixture"

    def __init__(self, robot_id: str, condition: str, *, ttl_s: float = 2.0) -> None:
        self.robot_id = robot_id
        self.condition = condition
        self.ttl_s = ttl_s
        self.requests: list[dict] = []

    def decide(self, request: dict) -> dict:
        self.requests.append(copy.deepcopy(request))
        call = len(self.requests)
        now_s = request["local_status"]["timestamp_s"]
        action = {
            "kind": "task_request",
            "task_id": f"{self.robot_id}-task-{call}",
            "object_id": "box-1",
            "skill": "carry",
            "participants": [self.robot_id],
            "resources": [f"cargo:box-1:{self.robot_id}"],
            "stage": "carry",
            "own_role": "carrier",
            "expires_at_s": now_s + 5.0,
        }
        message = None
        if self.robot_id == "r1" and call == 1 and self.condition != "none":
            content = ({"message_type": "intent", "task_id": action["task_id"],
                        "object_id": "box-1", "participants": ["r1"]}
                       if self.condition == "structured"
                       else "I intend to carry box-1 alone.")
            message = {"recipients": ["r2"], "content": content, "ttl_s": self.ttl_s}
        raw = json.dumps({"action": action, "message": message}, sort_keys=True)
        return {"action": action, "message": message, "raw_text": raw}


def planners(condition: str, *, ttl_s: float = 2.0) -> dict[str, FixturePlanner]:
    return {robot_id: FixturePlanner(robot_id, condition, ttl_s=ttl_s) for robot_id in ROBOTS}


class RGBCommunicationRuntimeTests(unittest.TestCase):
    def run_fixture(self, condition: str, *, ttl_s: float = 2.0, ticks: int = 2):
        port = FixturePort()
        owned_planners = planners(condition, ttl_s=ttl_s)
        clock = MutableClock()
        directory = tempfile.TemporaryDirectory()
        root = Path(directory.name)
        result = run_rgb_communication(
            port,
            owned_planners,
            condition=condition,
            common_task=FIXTURE["static_context"]["task"],
            run_id=f"fixture-{condition}",
            limits=RuntimeLimits(max_ticks=ticks, max_calls_per_robot=ticks,
                                 max_actions_per_robot=ticks, tick_period_s=1.0),
            trace_path=root / "trace.jsonl",
            artifact_dir=root / "artifacts",
            wall_clock=clock,
        )
        return directory, port, owned_planners, result, root

    def test_matched_conditions_keep_independent_planners_actions_and_budgets(self):
        results = {}
        directories = []
        try:
            for condition in ("none", "structured", "natural"):
                directory, port, owned_planners, result, _ = self.run_fixture(condition)
                directories.append(directory)
                results[condition] = result
                self.assertEqual(len({id(planner) for planner in owned_planners.values()}), 3)
                self.assertEqual(result["calls"], {robot_id: 2 for robot_id in ROBOTS})
                self.assertEqual(result["actions"], {robot_id: 2 for robot_id in ROBOTS})
                self.assertEqual(len(port.submissions), 6)
                self.assertEqual(port.closed_at_s, 1.0)
            self.assertEqual(results["none"]["messages"], {robot_id: 0 for robot_id in ROBOTS})
            self.assertEqual(results["structured"]["messages"]["r1"], 1)
            self.assertEqual(results["natural"]["messages"]["r1"], 1)
        finally:
            for directory in directories:
                directory.cleanup()

    def test_recipient_isolation_and_expired_claim_memory(self):
        directory, _, owned_planners, result, _ = self.run_fixture("natural", ttl_s=2.0)
        try:
            r2_memory = owned_planners["r2"].requests[1]["memory"]
            r3_memory = owned_planners["r3"].requests[1]["memory"]
            self.assertEqual(len(r2_memory["received_messages"]), 1)
            self.assertEqual(r2_memory["received_messages"][0]["sender_id"], "r1")
            self.assertEqual(r3_memory["received_messages"], [])
            self.assertFalse(any(event["event_type"] == "message_received"
                                 and event["robot_id"] == "r3" for event in result["events"]))
        finally:
            directory.cleanup()

    def test_none_condition_can_submit_independent_joint_requests(self):
        class JointPlanner(FixturePlanner):
            def decide(self, request: dict) -> dict:
                reply = super().decide(request)
                if self.robot_id in {"r1", "r2"}:
                    reply["action"].update({
                        "task_id": "joint-plank",
                        "object_id": "plank-1",
                        "participants": ["r1", "r2"],
                        "resources": ["cargo:plank-1"],
                        "own_role": "left" if self.robot_id == "r1" else "right",
                    })
                    reply["raw_text"] = json.dumps(reply["action"], sort_keys=True)
                reply["message"] = None
                return reply

        port = FixturePort()
        owned = {robot_id: JointPlanner(robot_id, "none") for robot_id in ROBOTS}
        result = run_rgb_communication(
            port, owned, condition="none", common_task=FIXTURE["static_context"]["task"],
            run_id="fixture-none-joint",
            limits=RuntimeLimits(max_ticks=1, max_calls_per_robot=1,
                                 max_actions_per_robot=1),
        )
        joint = [action for robot_id, action in port.submissions if robot_id in {"r1", "r2"}]
        self.assertEqual(len(joint), 2)
        self.assertEqual({action["task_id"] for action in joint}, {"joint-plank"})
        self.assertEqual({tuple(action["participants"]) for action in joint}, {("r1", "r2")})
        self.assertEqual(len({action["request_id"] for action in joint}), 2)
        self.assertEqual(result["messages"], {robot_id: 0 for robot_id in ROBOTS})

    def test_supervisor_truth_does_not_change_actor_inputs_or_actions(self):
        copies = []
        for secret in ("all-complete", "none-complete"):
            port = FixturePort(tick_secret=secret)
            owned = planners("none")
            result = run_rgb_communication(
                port, owned, condition="none",
                common_task=FIXTURE["static_context"]["task"],
                run_id="fixture-noninterference",
                limits=RuntimeLimits(max_ticks=1, max_calls_per_robot=1,
                                     max_actions_per_robot=1),
                wall_clock=MutableClock(),
            )
            copies.append((owned, port, result))
        self.assertEqual(
            {rid: copies[0][0][rid].requests for rid in ROBOTS},
            {rid: copies[1][0][rid].requests for rid in ROBOTS},
        )
        self.assertEqual(copies[0][1].submissions, copies[1][1].submissions)
        self.assertEqual(copies[0][2]["calls"], copies[1][2]["calls"])

    def test_own_state_change_during_planning_discards_reply(self):
        port = FixturePort()

        class StateChangingPlanner(FixturePlanner):
            def decide(self, request: dict) -> dict:
                reply = super().decide(request)
                if self.robot_id == "r1":
                    port.submission_history["r1"].append({
                        "request_id": "external-r1",
                        "status": "TERMINATED",
                        "reason": "lease_expired",
                        "clock_domain": "sim",
                        "timestamp_s": port.now_s,
                        "task_id": "old-r1",
                        "lease_id": "lease-r1",
                    })
                return reply

        owned = {robot_id: StateChangingPlanner(robot_id, "none") for robot_id in ROBOTS}
        result = run_rgb_communication(
            port, owned, condition="none", common_task=FIXTURE["static_context"]["task"],
            run_id="fixture-own-state-change",
            limits=RuntimeLimits(max_ticks=1, max_calls_per_robot=1,
                                 max_actions_per_robot=1),
        )
        self.assertFalse(any(robot_id == "r1" for robot_id, _ in port.submissions))
        self.assertTrue(any(event["event_type"] == "planner_response_rejected"
                            and event["robot_id"] == "r1"
                            and event["payload"]["reason"] == "OWN_STATE_CHANGED"
                            for event in result["events"]))

        directory, _, owned_planners, result, _ = self.run_fixture(
            "structured", ttl_s=1.5, ticks=3)
        try:
            memory = owned_planners["r2"].requests[2]["memory"]
            self.assertEqual(memory["received_messages"], [])
            self.assertEqual(memory["expired_peer_claims"][0]["status"], "EXPIRED_PEER_CLAIM")
            self.assertTrue(any(event["event_type"] == "message_expired"
                                and event["payload"]["reason"] == "TTL_EXPIRED_AFTER_RECEIPT"
                                for event in result["events"]))
        finally:
            directory.cleanup()

        directory, _, owned_planners, result, _ = self.run_fixture("natural", ttl_s=.5)
        try:
            memory = owned_planners["r2"].requests[1]["memory"]
            self.assertEqual(memory["received_messages"], [])
            self.assertEqual(memory["expired_peer_claims"][0]["status"], "EXPIRED_PEER_CLAIM")
            self.assertTrue(any(event["event_type"] == "message_expired"
                                and event["robot_id"] == "r2" for event in result["events"]))
        finally:
            directory.cleanup()

    def test_trace_schema_links_request_decision_message_and_action(self):
        directory, _, _, result, root = self.run_fixture("structured")
        try:
            trace_text = (root / "trace.jsonl").read_text()
            rows = [json.loads(line) for line in trace_text.splitlines()]
            self.assertEqual(sum(row["event_type"] == "run_started" for row in rows), 1)
            self.assertEqual(sum(row["event_type"] == "run_finished" for row in rows), 1)
            self.assertEqual(rows[-1]["payload"]["outcome"], "aborted")
            required = {"schema_version", "event_id", "run_id", "condition", "robot_id",
                        "event_type", "sim_time_s", "wall_time_s", "related_ids", "payload"}
            self.assertTrue(all(set(row) == required for row in rows))
            self.assertTrue(all(row["schema_version"] == "rgb-communication-event.v1" for row in rows))
            action = next(row for row in rows if row["event_type"] == "action_submitted")
            response = next(row for row in rows if row["event_type"] == "planner_responded"
                            and row["related_ids"]["decision_id"] == action["related_ids"]["decision_id"])
            request = next(row for row in rows if row["event_type"] == "planner_requested"
                           and row["related_ids"]["request_id"] == response["related_ids"]["request_id"])
            self.assertEqual(action["related_ids"]["observation_id"],
                             request["related_ids"]["observation_id"])
            self.assertIn("response_latency_s", response["payload"])
            self.assertNotIn("commands", rows[0]["payload"]["measured_metrics"])
            self.assertIn("high_level_actions", rows[0]["payload"]["measured_metrics"])
            artifact_paths = [image["path"] for image in request["payload"]["observation"]["images"].values()]
            self.assertTrue(all(Path(path).is_file() for path in artifact_paths))
            self.assertNotIn("jpeg_base64", trace_text)
        finally:
            directory.cleanup()

    def test_committed_trace_fixture_preserves_raw_artifact_hashes(self):
        trace_path = ROOT / "tests/fixtures/rgb_communication/trace.jsonl"
        rows = [json.loads(line) for line in trace_path.read_text().splitlines()]
        self.assertEqual([rows[0]["event_type"], rows[-1]["event_type"]],
                         ["run_started", "run_finished"])
        self.assertEqual(len({row["event_id"] for row in rows}), len(rows))
        response = next(row for row in rows if row["event_type"] == "planner_responded")
        self.assertEqual(
            hashlib.sha256(response["payload"]["raw_text"].encode()).hexdigest(),
            response["payload"]["raw_text_sha256"],
        )
        request = next(row for row in rows if row["event_type"] == "planner_requested")
        for image in request["payload"]["observation"]["images"].values():
            path = ROOT / image["path"]
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), image["sha256"])

    def test_late_reply_is_rejected_without_message_or_action(self):
        class LatePlanner(FixturePlanner):
            def __init__(self, robot_id: str, clock: MutableClock) -> None:
                super().__init__(robot_id, "natural")
                self.clock = clock

            def decide(self, request: dict) -> dict:
                reply = super().decide(request)
                self.clock.advance(2.0)
                return reply

        clock = MutableClock()
        port = FixturePort()
        owned = {robot_id: LatePlanner(robot_id, clock) for robot_id in ROBOTS}
        result = run_rgb_communication(
            port, owned, condition="natural", common_task=FIXTURE["static_context"]["task"],
            run_id="fixture-late",
            limits=RuntimeLimits(max_ticks=1, max_calls_per_robot=1,
                                 decision_timeout_s=1.0, wall_timeout_s=20.0),
            wall_clock=clock,
        )
        self.assertEqual(port.submissions, [])
        self.assertEqual(result["messages"], {robot_id: 0 for robot_id in ROBOTS})
        self.assertEqual(sum(event["event_type"] == "planner_response_rejected"
                             and event["payload"]["reason"] == "LATE_REPLY"
                             for event in result["events"]), 3)

    def test_unknown_truth_field_fails_closed_before_planner(self):
        port = FixturePort(leak_field="ground_truth")
        owned = planners("none")
        result = run_rgb_communication(
            port, owned, condition="none", common_task=FIXTURE["static_context"]["task"],
            run_id="fixture-leak", limits=RuntimeLimits(max_ticks=1, max_calls_per_robot=1),
        )
        self.assertEqual(result["outcome"], "failure")
        self.assertEqual(result["termination_reason"], "RUNTIME_ERROR")
        self.assertEqual(result["error_type"], "ValueError")
        self.assertTrue(all(planner.requests == [] for planner in owned.values()))
        self.assertEqual(port.closed_at_s, 0.0)
        self.assertEqual(sum(event["event_type"] == "run_finished"
                             for event in result["events"]), 1)


if __name__ == "__main__":
    unittest.main()
