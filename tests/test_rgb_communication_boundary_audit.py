"""Offline audit of the current RGB communication boundary.

These tests do not declare the research pilot runnable.  They preserve current
boundary evidence and make known adapter gaps machine-readable for R2/R3.
"""
from __future__ import annotations

import base64
from concurrent.futures import Future
import copy
import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from harness.camera_policy import (
    CameraPlanner,
    FORBIDDEN_SPATIAL_FIELDS,
    OBSERVATION_FIELDS,
)
from harness.camera_runtime import run_camera_episode
from harness.coela_modules import Communication, Memory, MODES, Perception, PlanningError, ROBOTS
from harness.dispatch_plan import build_dispatch_request, fixture_plan, validate_dispatch_plan
from harness.mixed_warehouse_protocol import MixedWarehouseReferee
from harness.three_robot_plan import TeamAgreement, digest
from sim.camera_robot_port import CameraRobotPort


FIXTURES = Path(__file__).parent / "fixtures" / "rgb_communication_audit"
JPEG = b"\xff\xd8rgb-boundary-audit\xff\xd9"


def load_fixture(name):
    return json.loads((FIXTURES / name).read_text())


def observation(robot_id="r1"):
    return {
        "robot_id": robot_id,
        "frame_id": f"{robot_id}-frame",
        "sim_time": 1.0,
        "image": base64.b64encode(JPEG).decode(),
        "sha256": hashlib.sha256(JPEG).hexdigest(),
        "camera": "robot_cam",
        "actuator_state": {"motor_commands": [0.0] * 4, "servo_pulses": {}},
    }


class CapturingCompleter:
    model_name = "audit-fixture"
    last_usage = None

    def __init__(self, reply=None):
        self.reply = reply or {
            "action": {"kind": "wait"},
            "private_reason": "현재 RGB 입력만 사용",
            "message": None,
        }
        self.calls = []

    def complete(self, messages, *, image):
        self.calls.append((messages, image))
        return json.dumps(self.reply, ensure_ascii=False)


class SimClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def step(self):
        self.now = round(self.now + 0.1, 6)


class ImmediateExecutor:
    """Deterministic scheduling only; concurrency is covered by existing tests."""

    def __init__(self, **_kwargs):
        pass

    def submit(self, fn, *args, **kwargs):
        future = Future()
        try:
            future.set_result(fn(*args, **kwargs))
        except Exception as exc:
            future.set_exception(exc)
        return future

    def shutdown(self, **_kwargs):
        pass


class Port:
    def __init__(self, rid, clock, *, command_delay=0.0):
        self.rid = rid
        self.clock = clock
        self.frames = 0
        self.command_delay = command_delay

    def tick(self, _now):
        return None

    def capture(self):
        self.frames += 1
        value = observation(self.rid)
        value["frame_id"] = f"{self.rid}-{self.frames}"
        value["sim_time"] = self.clock()
        return value

    def apply(self, _action, now):
        return {"ok": True, "robot_id": self.rid, "busy_until": now + self.command_delay}

    def stop(self):
        return None


class RecordingPlanner:
    def __init__(self, rid, *, private_reason="private", send=True):
        self.rid = rid
        self.inputs = []
        self.private_reason = private_reason
        self.send = send

    def decide(self, obs, memory, messages, **_options):
        self.inputs.append({"observation": obs, "memory": memory, "messages": messages})
        first = len(self.inputs) == 1
        message = None
        if self.rid == "r1" and first and self.send:
            message = {
                "recipients": ["r2"],
                "content": "small_box_01을 보았다는 발신자 주장",
                "kind": "intent",
                "observed_ids": ["small_box_01"],
            }
        return {"action": {"kind": "wait"}, "private_reason": self.private_reason, "message": message}


class RGBCommunicationBoundaryAuditTests(unittest.TestCase):
    def test_fixture_matches_current_rgb_allowlist_and_modes(self):
        contract = load_fixture("boundary_contract.json")
        self.assertEqual(set(contract["current_rgb_actor_observation_fields"]), OBSERVATION_FIELDS)
        self.assertEqual(set(contract["required_modes"]), set(MODES))
        self.assertEqual(set(contract["allowed_robot_ids"]), set(ROBOTS))
        self.assertTrue(set(contract["forbidden_actor_fields"]) <= FORBIDDEN_SPATIAL_FIELDS)

    def test_private_peer_state_cannot_enter_rgb_planner_context(self):
        completer = CapturingCompleter()
        planner = CameraPlanner("r1", completer)
        planner.decide(observation(), {"own_note": "unchanged"}, [], mode="none")
        baseline = copy.deepcopy(completer.calls)
        with self.assertRaisesRegex(ValueError, "SPATIAL_ORACLE_FORBIDDEN"):
            planner.decide(observation(), {"own_note": "unchanged", "peer_positions": {"r2": [1, 2]}}, [], mode="none")
        self.assertEqual(completer.calls, baseline)

    def test_real_rgb_port_and_model_input_ignore_hidden_peer_and_evaluator_changes(self):
        class Robot:
            servo_command_pulses = {1: 2000, 3: 600}

            def __getattr__(self, name):
                raise AssertionError(f"measured robot state accessed: {name}")

        class World:
            def __init__(self, hidden):
                self.data = SimpleNamespace(time=1.0)
                self.hidden = hidden

            def robot(self, rid):
                self.asserted_robot = rid
                return Robot()

            def render_jpeg(self, *, robot_id, camera):
                if (robot_id, camera) != ("r1", "robot_cam"):
                    raise AssertionError("foreign camera access")
                return JPEG

            def __getattr__(self, name):
                raise AssertionError(f"privileged world accessor: {name}")

        calls = []
        for hidden in ({"r2_intent": "box", "success": False}, {"r2_intent": "beam", "success": True}):
            world = World(hidden)
            port = CameraRobotPort(world, "r1")
            completer = CapturingCompleter()
            CameraPlanner("r1", completer).decide(port.capture(), [], [], mode="none")
            self.assertEqual(world.asserted_robot, "r1")
            calls.append(completer.calls)
        self.assertEqual(calls[0], calls[1])

    def test_current_planner_blacklist_does_not_reject_semantic_oracle_fields(self):
        # A-02: an injected adapter payload reaches the model. The actual port
        # above does not generate these values. This is a defense gap, not a
        # claim that a physical run leaked them.
        for field, value in (("success", True), ("contact_count", 2), ("measured_joints", [1.2])):
            with self.subTest(field=field):
                completer = CapturingCompleter()
                obs = observation()
                obs["actuator_state"][field] = value
                CameraPlanner("r1", completer).decide(obs, [], [], mode="none")
                payload = json.loads(completer.calls[0][0][-1]["content"])
                self.assertEqual(payload["actuator_state"][field], value)

    def test_recipient_isolation_and_none_silence_are_transport_enforced(self):
        bus = Communication("natural")
        sent = bus.send("r1", {"recipients": ["r2"], "content": "r2 전용 의도"}, 1.0)
        self.assertEqual([item["message_id"] for item in bus.receive("r2", 1.1)], [sent["message_id"]])
        self.assertEqual(bus.receive("r3", 1.1), [])
        with self.assertRaisesRegex(ValueError, "COMMUNICATION_DISABLED"):
            Communication("none").send("r1", {"recipients": ["r2"], "content": "금지"}, 1.0)

    def test_rgb_action_contract_reproduces_joint_adapter_no_go(self):
        reply = {
            "action": {"kind": "request_joint", "cargo_id": "oak_plank_01", "participants": ["r1", "r2"]},
            "private_reason": "공동 이송을 요청",
            "message": None,
        }
        with self.assertRaises(PlanningError) as caught:
            CameraPlanner("r1", CapturingCompleter(reply)).decide(observation(), {}, [], mode="none")
        self.assertIn("INVALID_ACTION_FIELDS", str(caught.exception))

    def test_observed_ids_remain_unverified_sender_claims(self):
        event = Communication("natural").send("r1", {
            "recipients": ["r2"],
            "content": "사진에서 small_box_01을 보았다고 주장",
            "kind": "observation",
            "observed_ids": ["small_box_01"],
        }, 1.0)
        self.assertEqual(event["observed_ids"], ["small_box_01"])
        self.assertNotIn("visual_verification", event)
        self.assertNotIn("source_frame_sha256", event)

    def test_rgb_runtime_reproduces_missing_message_expiry(self):
        clock = SimClock()
        ports = {rid: Port(rid, clock, command_delay=13.0) for rid in ROBOTS}
        planners = {rid: RecordingPlanner(rid) for rid in ROBOTS}
        with tempfile.TemporaryDirectory() as output, patch(
            "harness.camera_runtime.ThreadPoolExecutor", ImmediateExecutor
        ), patch(
            "harness.camera_runtime.time.monotonic", clock
        ):
            run_camera_episode(
                ports,
                planners,
                clock=clock,
                step=clock.step,
                output=output,
                destination="B",
                mode="natural",
                timeout_s=42.0,
                max_calls=3,
                realtime=False,
            )
        r2_inputs = planners["r2"].inputs
        received_index = next(i for i, item in enumerate(r2_inputs) if item["messages"])
        later = r2_inputs[received_index + 1]
        self.assertEqual(later["messages"], [])
        retained = next(item["received_message"] for item in later["memory"] if "received_message" in item)
        age = later["observation"]["sim_time"] - retained["received_sim_time"]
        self.assertGreater(age, retained["intent_ttl_s"])
        self.assertNotIn("expired", retained)
        self.assertNotIn("status", retained)
        # The CoELA memory does expire the exact same report. The RGB runtime
        # does not use it, so its TTL field alone is not enforcement.
        now = [0.0]
        memory = Memory("r2", clock=lambda: now[0])
        memory.receive(retained)
        now[0] = age
        self.assertEqual(memory.active_reports(), [])

    def test_hidden_peer_reason_does_not_change_none_actor_input_or_schedule(self):
        runs = []
        for secret in ("r1 private box intent", "r1 private beam recovery"):
            clock = SimClock()
            ports = {rid: Port(rid, clock) for rid in ROBOTS}
            planners = {rid: RecordingPlanner(rid, send=False) for rid in ROBOTS}
            planners["r1"].private_reason = secret
            with tempfile.TemporaryDirectory() as output, patch(
                "harness.camera_runtime.ThreadPoolExecutor", ImmediateExecutor
            ):
                result = run_camera_episode(ports, planners, clock=clock, step=clock.step,
                    output=output, destination="B", mode="none", timeout_s=3.0,
                    max_calls=3, realtime=False)
                self.assertEqual(result["messages"], 0)
                self.assertEqual((Path(output) / "dialogue.jsonl").read_text(), "")
            self.assertEqual(result["calls"], {rid: 3 for rid in ROBOTS})
            runs.append({rid: planners[rid].inputs for rid in ("r2", "r3")})
        self.assertEqual(runs[0], runs[1])

    def test_directed_rgb_delivery_and_saved_image_hashes_are_auditable(self):
        clock = SimClock()
        ports = {rid: Port(rid, clock) for rid in ROBOTS}
        planners = {rid: RecordingPlanner(rid) for rid in ROBOTS}
        with tempfile.TemporaryDirectory() as output, patch(
            "harness.camera_runtime.ThreadPoolExecutor", ImmediateExecutor
        ):
            run_camera_episode(ports, planners, clock=clock, step=clock.step,
                output=output, destination="B", mode="natural", timeout_s=3.0,
                max_calls=3, realtime=False)
            rows = [json.loads(line) for line in (Path(output) / "episode.jsonl").read_text().splitlines()]
            inputs = [row for row in rows if row["event"] == "planner_input"]
            self.assertEqual(len(inputs), 9)
            for row in inputs:
                image = (Path(output) / row["image_path"]).read_bytes()
                self.assertEqual(hashlib.sha256(image).hexdigest(), row["observation"]["sha256"])
            self.assertTrue(any(item["messages"] for item in planners["r2"].inputs))
            self.assertTrue(all(not item["messages"] for item in planners["r3"].inputs))
            self.assertNotIn("발신자 주장", json.dumps(planners["r3"].inputs, ensure_ascii=False))

    def test_none_can_submit_matching_joint_consent_in_old_protocol_without_messages(self):
        referee = MixedWarehouseReferee({"oak_plank_01": 2, "small_box_01": 1}, revision=1)
        claims = []
        for rid in ("r1", "r2"):
            referee.observe({"robot_id": rid, "cargo_ids": ["oak_plank_01"],
                "revision": 1, "observation_id": rid})
            claims.append({"robot_id": rid, "cargo_id": "oak_plank_01",
                "participants": ["r1", "r2"], "destination": "B", "revision": 1,
                "observation_id": rid})
        first = referee.submit(claims[:1])
        self.assertEqual(first.accepted, ())
        second = referee.submit(claims[1:])
        self.assertEqual(len(second.accepted), 1)
        self.assertEqual(second.accepted[0].participants, ("r1", "r2"))
        self.assertTrue(all(not values for values in referee.peer_reports.values()))

    def test_legacy_perception_preserves_ideal_sim_contract_and_numeric_sensor_fields(self):
        original = {"robot_id": "r1", "observation_id": "o1", "revision": 1,
            "sim_time": 1.0, "controller_contract": "ideal_sim_geometry",
            "cargo": [{"cargo_id": "oak_plank_01", "range_m": 1.2,
                "bearing_rad": 0.4, "source": "rgbd_aruco"}], "private_debug": "hidden"}
        environment = SimpleNamespace(request=lambda *_args: original)
        observed = Perception(environment, "r1", "fixture").read()
        self.assertEqual(observed["controller_contract"], "ideal_sim_geometry")
        self.assertEqual(observed["cargo"], original["cargo"])
        self.assertNotIn("private_debug", observed)

    def test_empty_dispatch_inbox_still_receives_peer_authored_whole_plan(self):
        agreement = TeamAgreement("audit", plan_validator=validate_dispatch_plan)
        plan = fixture_plan(solo="r2")
        agreement.receive({"r1": {"request_id": "audit-r1-plan-0", "proposal_id": None,
            "plan_hash": None, "accept": True, "plan": plan, "reason": "fixture", "message": ""}}, 0)
        request = build_dispatch_request("r3", task={}, request_id="audit-r3-plan-1",
            own_rgb=JPEG, top_rgb=JPEG, agreement=agreement.context(), inbox=[])
        context = json.loads(request["messages"][-1]["content"])
        self.assertEqual(context["peer_claims"], [])
        self.assertEqual(context["agreement"]["proposal"]["plan"], plan)
        self.assertEqual(context["reply_binding"]["plan_hash"], digest(plan))
        self.assertIsNone(agreement.committed)

    def test_pilot_candidate_is_exactly_six_bounded_unexecuted_trials(self):
        pilot = load_fixture("pilot_protocol.json")
        self.assertEqual(pilot["status"], "candidate_not_executed")
        self.assertEqual(pilot["linked_trials"], 6)
        self.assertEqual(pilot["linked_trials"], len(pilot["modes"]) * len(pilot["scenarios"]))
        budgets = pilot["budgets"]
        self.assertEqual(budgets["max_cohort_model_calls"],
                         6 * 3 * budgets["max_high_level_calls_per_robot"])
        self.assertEqual(budgets["max_cohort_output_tokens"],
                         budgets["max_cohort_model_calls"] * budgets["max_output_tokens_per_call"])
        self.assertEqual(budgets["max_cohort_input_tokens"], 6 * budgets["max_trial_input_tokens"])
        self.assertEqual(budgets["max_cohort_command_records"], 6 * budgets["max_trial_command_records"])
        self.assertIsNone(pilot["integrated_source_sha"])
        self.assertIsNone(pilot["setup_hashes"])
        self.assertFalse(pilot["mission"]["weld"])
        self.assertFalse(pilot["mission"]["camera_fov_change"])
        self.assertTrue(all(value > 0 for value in pilot["budgets"].values()))
        self.assertIn("integration", pilot["statistical_claim"])


if __name__ == "__main__":
    unittest.main()
