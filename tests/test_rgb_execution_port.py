import copy
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path

import pytest

from harness.rgb_execution_contract import RGBExecutionPortProtocol, SkillCapability
from harness.rgb_execution_port import RGBExecutionPort


FIXTURE = json.loads((Path(__file__).parent / "fixtures/rgb_execution_port/basic.json").read_text())
JPEG = b"\xff\xd8fixture-jpeg\xff\xd9"
TOP = b"\xff\xd8common-top\xff\xd9"


class Endpoint:
    def __init__(self, robot_id, *, fail_apply=False, fail_validate=False):
        self.robot_id = robot_id
        self.fail_apply = fail_apply
        self.fail_validate = fail_validate
        self.validated = []
        self.applied = []
        self.holds = []
        self.ticks = []

    def validate_bounded(self, action, duration_s):
        if self.fail_validate:
            raise OSError("fixture validator disconnected")
        if set(action) != {"kind", "value"}:
            raise ValueError("fixture action shape")
        self.validated.append((copy.deepcopy(action), duration_s))

    def apply_bounded(self, action, now_s, duration_s):
        if self.fail_apply:
            raise OSError("fixture endpoint disconnected")
        self.applied.append((copy.deepcopy(action), now_s, duration_s))

    def hold(self, now_s):
        self.holds.append(now_s)

    def tick(self, now_s):
        self.ticks.append(now_s)


def capabilities():
    return [
        SkillCapability("pair_transport", frozenset({"CARRY"}), frozenset({2}),
                        frozenset({"drive"}), .5, distinct_roles=True),
        SkillCapability("solo_transport", frozenset({"CARRY"}), frozenset({1}),
                        frozenset({"drive"}), .5),
    ]


def build_port(*, fail_robot=None, fail_validate_robot=None, evaluator=None):
    endpoints = {rid: Endpoint(rid, fail_apply=rid == fail_robot,
                               fail_validate=rid == fail_validate_robot)
                 for rid in ("r1", "r2", "r3")}
    port = RGBExecutionPort(
        endpoints,
        frame_source=lambda rid, now: {"own_rgb": JPEG + rid.encode() + b"\xff\xd9",
                                       "top_rgb": TOP},
        static_context=FIXTURE["static_context"],
        capabilities=capabilities(),
        clock_domain="sim",
        evaluation_source=evaluator,
    )
    assert isinstance(port, RGBExecutionPortProtocol)
    return port, endpoints


def task_request(robot_id, task_id, *, participants=None, resource=None,
                 role="solo", skill="solo_transport", expires=3., request_id=None):
    participants = participants or [robot_id]
    return {"kind": "task_request", "request_id": request_id or f"{robot_id}-{task_id}",
            "task_id": task_id, "object_id": task_id + "-object", "skill": skill,
            "participants": participants, "resources": [resource or task_id],
            "stage": "CARRY", "own_role": role,
            "observation_id": f"{robot_id}-observation", "decision_id": f"{robot_id}-decision",
            "requested_at_s": 0., "expires_at_s": expires}


def command(robot_id, task_id, lease_id, *, request_id=None, command_id=None):
    return {"kind": "command", "request_id": request_id or f"{robot_id}-{task_id}-submit",
            "task_id": task_id, "lease_id": lease_id,
            "command_id": command_id or f"{robot_id}-{task_id}-command", "stage": "CARRY",
            "action": {"kind": "drive", "value": robot_id}, "duration_s": .2,
            "observation_id": f"{robot_id}-observation", "decision_id": f"{robot_id}-command-decision",
            "requested_at_s": 0.}


def pair(port):
    first = port.submit("r1", FIXTURE["pair_requests"]["r1"])
    second = port.submit("r2", FIXTURE["pair_requests"]["r2"])
    assert first["status"] == "PENDING"
    assert second["status"] == "ACCEPTED"
    accepted = [row for row in port.local_status("r1")["own_submission_history"]
                if row["status"] == "ACCEPTED"]
    assert len(accepted) == 1
    return accepted[0]["lease_id"]


def test_actor_observation_is_exact_allowlist_and_evaluator_is_separate():
    truth = {"mission_complete": False, "secret_contact": [1, 2, 3],
             "objects": {}, "recovery": {"required": False, "reached": False,
                                           "succeeded": False}}
    port, _ = build_port(evaluator=lambda: truth)
    observation = port.observe("r1")
    assert set(observation) == {"schema", "robot_id", "observation_id", "observed_at_s",
                                "clock_domain", "images", "static_context",
                                "static_context_sha256", "own_issued_commands"}
    assert set(observation["images"]) == {"own_rgb", "top_rgb"}
    assert all(set(image) == {"ref", "sha256", "jpeg_base64"}
               for image in observation["images"].values())
    assert "secret_contact" not in json.dumps(observation)
    assert port.local_status("r1")["active"] == []
    snapshot = port.evaluation_snapshot()
    assert snapshot["external_evaluation"]["secret_contact"] == [1, 2, 3]
    snapshot["external_evaluation"]["secret_contact"].append(4)
    assert truth["secret_contact"] == [1, 2, 3]


def test_evaluator_truth_change_does_not_change_actor_status_or_wakeup_inputs():
    truth = {"mission_complete": False, "objects": {},
             "recovery": {"required": False, "reached": False, "succeeded": False}}
    port, _ = build_port(evaluator=lambda: truth)
    before_status = port.local_status("r1")
    before_observation = port.observe("r1")
    truth["mission_complete"] = True
    truth["objects"] = {"beam_01": {"stages": {"CARRY": True}}}
    after_status = port.local_status("r1")
    after_observation = port.observe("r1")
    assert before_status == after_status
    for observation in (before_observation, after_observation):
        observation.pop("observation_id")
        for image in observation["images"].values():
            image.pop("ref")
    assert before_observation == after_observation


def test_none_condition_can_form_joint_lease_without_peer_messages_or_shared_request_id():
    port, _ = build_port()
    lease_id = pair(port)
    status = port.local_status("r1")
    assert status["pending"] == []
    assert status["active"][0]["lease_id"] == lease_id
    assert status["active"][0]["own_role"] == "end_a"
    serialized = json.dumps(status)
    assert "end_b" not in serialized
    assert "r2-beam" not in serialized
    assert "peer" not in serialized


def test_concurrent_independent_consents_are_serialized_without_lost_request():
    port, _ = build_port()
    with ThreadPoolExecutor(max_workers=2) as pool:
        futures = [pool.submit(port.submit, rid, FIXTURE["pair_requests"][rid])
                   for rid in ("r1", "r2")]
    assert {future.result()["status"] for future in futures} == {"PENDING", "ACCEPTED"}
    for rid in ("r1", "r2"):
        status = port.local_status(rid)
        assert status["pending"] == []
        assert len(status["active"]) == 1


def test_joint_commands_wait_for_both_but_unrelated_solo_progresses():
    port, endpoints = build_port()
    lease_id = pair(port)
    solo = port.submit("r3", task_request("r3", "box-task"))
    assert solo["status"] == "ACCEPTED"
    assert port.submit("r1", command("r1", "beam-task", lease_id))["status"] == "QUEUED"
    assert port.submit("r3", command("r3", "box-task", solo["lease_id"]))["status"] == "QUEUED"
    first = port.tick(.1)
    assert [row["task_id"] for row in first["events"]] == ["box-task"]
    assert not endpoints["r1"].applied and not endpoints["r2"].applied
    assert len(endpoints["r3"].applied) == 1
    port.submit("r2", command("r2", "beam-task", lease_id))
    second = port.tick(.2)
    assert second["events"] == [{"task_id": "beam-task", "status": "DISPATCHED",
                                  "participants": ["r1", "r2"]}]
    assert len(endpoints["r1"].applied) == len(endpoints["r2"].applied) == 1
    assert port.observe("r1")["own_issued_commands"][0]["command_id"] == "r1-beam-task-command"
    assert port.observe("r1")["own_issued_commands"][0]["meaning"] == (
        "issued command, not measured state or success")


def test_role_mismatch_resource_conflict_and_unsupported_capability_fail_closed():
    port, _ = build_port()
    first = task_request("r1", "bad-pair", participants=["r1", "r2"], role="same",
                         skill="pair_transport")
    second = task_request("r2", "bad-pair", participants=["r1", "r2"], role="same",
                          skill="pair_transport")
    assert port.submit("r1", first)["status"] == "PENDING"
    assert port.submit("r2", second)["reason"] == "joint roles must be independently distinct"
    assert any(row["status"] == "REJECTED"
               for row in port.local_status("r1")["own_submission_history"])

    active = port.submit("r1", task_request("r1", "solo-a", resource="shared"))
    assert active["status"] == "ACCEPTED"
    conflict = port.submit("r3", task_request("r3", "solo-b", resource="shared"))
    assert conflict == {"request_id": "r3-solo-b", "status": "REJECTED",
                        "reason": "resource conflict", "clock_domain": "sim", "timestamp_s": 0.0}
    unsupported = task_request("r2", "unknown")
    unsupported["skill"] = "unwired_real_skill"
    assert port.submit("r2", unsupported)["reason"] == "unsupported skill"


def test_mismatched_joint_request_does_not_modify_first_actor_request():
    port, _ = build_port()
    first = task_request("r1", "pair-x", participants=["r1", "r2"], role="end_a",
                         skill="pair_transport", resource="beam")
    second = task_request("r2", "pair-x", participants=["r1", "r2"], role="end_b",
                          skill="pair_transport", resource="different")
    assert port.submit("r1", first)["status"] == "PENDING"
    assert port.submit("r2", second)["reason"] == "joint request core mismatch"
    pending = port.local_status("r1")["pending"]
    assert pending[0]["task_id"] == "pair-x" and pending[0]["own_role"] == "end_a"


def test_expiry_interrupt_and_invalid_time_hold_only_affected_participants():
    port, endpoints = build_port()
    pending = task_request("r1", "expiring", participants=["r1", "r2"], role="end_a",
                           skill="pair_transport", expires=.5)
    port.submit("r1", pending)
    assert port.tick(.5)["events"][0]["reason"] == "task request expired"
    assert not endpoints["r3"].holds

    port, endpoints = build_port()
    lease_id = pair(port)
    interrupted = port.submit("r1", {"kind": "interrupt", "request_id": "stop-beam",
        "task_id": "beam-task", "lease_id": lease_id, "reason": "own_rgb_uncertain",
        "observation_id": "r1-2", "decision_id": "r1-stop", "requested_at_s": 0.})
    assert interrupted["status"] == "ACCEPTED"
    assert endpoints["r1"].holds == endpoints["r2"].holds == [0.]
    assert endpoints["r3"].holds == []
    partner_history = port.local_status("r2")["own_submission_history"]
    assert partner_history[-1]["status"] == "TERMINATED"
    assert partner_history[-1]["reason"] == "task_interrupted"
    assert "r1" not in json.dumps(partner_history[-1])
    with pytest.raises(ValueError, match="monotonic"):
        port.tick(-1.)


def test_future_stale_invalid_and_duplicate_requests_are_rejected():
    port, _ = build_port()
    future = task_request("r1", "future")
    future["requested_at_s"] = 1.
    assert port.submit("r1", future)["reason"] == "future actor request"
    port.tick(6.)
    stale = task_request("r2", "stale")
    assert port.submit("r2", stale)["reason"] == "stale actor request"
    malformed = {"kind": "task_request", "request_id": "x", "contacts": True}
    assert port.submit("r3", malformed)["status"] == "REJECTED"


def test_batch_failure_stops_its_participants_but_does_not_block_other_task():
    port, endpoints = build_port(fail_robot="r2")
    lease_id = pair(port)
    solo = port.submit("r3", task_request("r3", "solo-ok"))
    port.submit("r1", command("r1", "beam-task", lease_id))
    port.submit("r2", command("r2", "beam-task", lease_id))
    port.submit("r3", command("r3", "solo-ok", solo["lease_id"]))
    result = port.tick(.1)
    assert [row["status"] for row in result["events"]] == ["FAILED", "DISPATCHED"]
    assert endpoints["r1"].holds == endpoints["r2"].holds == [.1]
    assert endpoints["r3"].holds == [] and len(endpoints["r3"].applied) == 1


def test_frame_and_validation_failures_hold_without_leaking_backend_details():
    port, endpoints = build_port(fail_validate_robot="r2")
    lease_id = pair(port)
    rejected = port.submit("r2", command("r2", "beam-task", lease_id))
    assert rejected["status"] == "REJECTED"
    assert rejected["reason"] == "command validation failed: OSError"
    assert endpoints["r1"].holds == endpoints["r2"].holds == [0.]
    assert port.local_status("r1")["own_submission_history"][-1]["reason"] == "execution_unavailable"

    port, endpoints = build_port()
    port._frame_source = lambda rid, now: (_ for _ in ()).throw(OSError("camera secret"))
    with pytest.raises(RuntimeError, match="rgb source unavailable"):
        port.observe("r3")
    assert endpoints["r3"].holds == [0.]
    assert "camera secret" not in str(port.local_status("r3"))


def test_close_holds_without_dispatching_a_ready_queue():
    port, endpoints = build_port()
    solo = port.submit("r3", task_request("r3", "queued"))
    port.submit("r3", command("r3", "queued", solo["lease_id"]))
    port.close(.1)
    assert endpoints["r3"].applied == []
    assert endpoints["r3"].holds == [.1]
    with pytest.raises(RuntimeError, match="closed"):
        port.observe("r3")


def test_actor_release_is_not_reported_as_measured_completion():
    truth = {"mission_complete": False, "objects": {"box": {"stages": {"CARRY": False}}},
             "recovery": {"required": False, "reached": False, "succeeded": False}}
    port, endpoints = build_port(evaluator=lambda: truth)
    accepted = port.submit("r3", task_request("r3", "release-me"))
    released = port.submit("r3", {"kind": "release", "request_id": "release-request",
        "task_id": "release-me", "lease_id": accepted["lease_id"], "reason": "own_rgb_claim",
        "observation_id": "r3-release-view", "decision_id": "r3-release-decision",
        "requested_at_s": 0.})
    assert released["reason"] == "actor_released_without_measured_success"
    status = port.local_status("r3")
    assert status["active"] == []
    assert any(row["reason"] == "task_released_unverified" for row in status["own_submission_history"])
    assert "completed" not in json.dumps(status).lower()
    assert port.evaluation_snapshot()["external_evaluation"]["mission_complete"] is False
    assert endpoints["r3"].holds == [0.]


def test_static_context_and_frame_source_reject_dynamic_or_extra_fields():
    bad = copy.deepcopy(FIXTURE["static_context"])
    bad["live_contacts"] = []
    endpoints = {rid: Endpoint(rid) for rid in ("r1", "r2", "r3")}
    with pytest.raises(ValueError, match="static context"):
        RGBExecutionPort(endpoints, frame_source=lambda rid, now: {}, static_context=bad,
                         capabilities=capabilities(), clock_domain="sim")
    port, endpoints = build_port()
    port._frame_source = lambda rid, now: {"own_rgb": JPEG, "top_rgb": TOP,
                                           "measured_joint": 1.}
    with pytest.raises(ValueError, match="only own_rgb and top_rgb"):
        port.observe("r1")
    assert endpoints["r1"].holds == [0.]
