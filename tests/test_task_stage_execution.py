"""Actuator integration tests. Visual verdicts here are explicitly fixtures."""
import base64
from dataclasses import replace
import hashlib
import json
from pathlib import Path

import pytest

from harness.task_stage_execution import TaskStageExecution, uncertain_reply
from harness.task_stage_sync import TaskPlan, READY_CHECKS, DONE_CHECKS, STAGES
from sim.camera_robot_port import CameraRobotPort


class Robot:
    servo_command_pulses = {1: 2000, 3: 600, 4: 2200, 5: 1900, 6: 1500}

    def __init__(self):
        self.motor_calls, self.servo_calls = [], []

    def set_motor_commands(self, command):
        self.motor_calls.append(list(command))

    def set_servo_pulses(self, command):
        self.servo_calls.append(dict(command))

    def __getattr__(self, name):
        raise AssertionError(f"measured robot state read: {name}")


class Clock:
    time = 0.

    def __getattr__(self, name):
        raise AssertionError(f"privileged state read: {name}")


class World:
    def __init__(self):
        self.data = Clock()
        self.robots = {r: Robot() for r in ("r1", "r3")}

    def robot(self, rid):
        return self.robots[rid]


@pytest.fixture
def setup(tmp_path):
    world = World()
    ports = {r: CameraRobotPort(world, r) for r in world.robots}
    map_path = tmp_path / "map.json"
    map_path.write_text('{"authored_static_map":true}')
    plan = TaskPlan("test", 1, "beam", (("r1", "left"), ("r3", "right")), "goal",
                    "test-map", "1", hashlib.sha256(map_path.read_bytes()).hexdigest())
    execution = TaskStageExecution(plan, ports, map_path=map_path, output=tmp_path / "evidence")
    return execution, world, map_path


def capture(execution, rid, now):
    return execution.capture(rid, own_rgb=b"\xff\xd8own\xff\xd9",
                             top_rgb=b"\xff\xd8top\xff\xd9", now_s=now)


def reply(request, status="READY", command_id=None):
    checks = DONE_CHECKS if status == "DONE" else READY_CHECKS
    return {"request_id": request["request_id"], "status": status, "confidence": 1.,
            "checks": sorted(checks[request["stage"]]), "reason": "synthetic test verdict",
            "command_id": command_id, "decided_at_s": request["observed_at_s"]}


def ready(execution, now):
    for rid in execution.ports:
        assert execution.receive(rid, reply(capture(execution, rid, now)), now_s=now)
    return execution.authorize(now_s=now)["permission"]


def batch(execution, index=0):
    kind = {"kind": "drive", "forward": .1, "turn": 0., "duration_s": .2} if execution.sync.stage == "CARRY" else {
        "kind": "arm", "servo_id": 5, "pulse": 500}
    return {r: {"command_id": f"{r}-{index}", "duration_s": .2, "action": kind} for r in execution.ports}


def test_absent_visual_producer_never_starts_or_finishes(setup):
    ex, world, _ = setup
    for rid in ex.ports:
        assert ex.receive(rid, uncertain_reply(capture(ex, rid, 0.)), now_s=0.)
    assert not ex.dispatch_pair(None, batch(ex), now_s=0.)
    ex.tick(3.)
    assert not ex.advance(now_s=3.)
    assert ex.sync.stage == "GRASP"
    assert all(not r.servo_calls for r in world.robots.values())


def test_expiry_cancels_arm_even_without_coordinator_polling(setup):
    ex, world, _ = setup
    assert ex.dispatch_pair(ready(ex, 0.), batch(ex), now_s=0.)
    for port in ex.ports.values():
        port.tick(.05)
        port.tick(.5)  # Late tick may interpolate only as far as the .2 lease.
    for robot in world.robots.values():
        assert robot.servo_calls[-1] == {5: 1500}
    for port in ex.ports.values():
        port.tick(3.)
    assert all(r.servo_calls[-1] == {5: 1500} for r in world.robots.values())


def test_hold_cancels_pending_motion_and_resume_needs_both_new_reports(setup):
    ex, world, _ = setup
    old_permission = ready(ex, 0.)
    assert ex.dispatch_pair(old_permission, batch(ex), now_s=0.)
    ex.tick(.05)
    ex.hold("camera_lost", now_s=.05)
    ex.tick(.3)
    assert all(r.servo_calls[-1] == {5: 1800} for r in world.robots.values())
    first = capture(ex, "r1", .3)
    assert ex.receive("r1", reply(first), now_s=.3)
    assert ex.authorize(now_s=.3)["permission"] is None
    second = capture(ex, "r3", .3)
    assert ex.receive("r3", reply(second), now_s=.3)
    new_permission = ex.authorize(now_s=.3)["permission"]
    assert new_permission["epoch"] > old_permission["epoch"]
    assert ex.dispatch_pair(new_permission, batch(ex, 1), now_s=.3)
    ex.tick(.35)
    assert all(r.servo_calls[-1][5] < 1800 for r in world.robots.values())


def test_missing_peer_and_expired_camera_are_not_readiness(setup):
    ex, _, _ = setup
    r1 = capture(ex, "r1", 0.)
    r3 = capture(ex, "r3", 0.)
    assert ex.receive("r1", reply(r1), now_s=0.)
    assert ex.authorize(now_s=0.)["permission"] is None
    ex.tick(.7)
    assert not ex.receive("r3", reply(r3), now_s=.7)
    assert ex.authorize(now_s=.7)["permission"] is None


def test_partial_submission_failure_stops_first_and_second(setup, monkeypatch):
    ex, world, _ = setup
    permission = ready(ex, 0.)
    def fail(*args):
        raise OSError("port disconnected")
    monkeypatch.setattr(ex.ports["r3"], "apply_bounded", fail)
    with pytest.raises(OSError):
        ex.dispatch_pair(permission, batch(ex), now_s=0.)
    ex.tick(.2)
    assert all(not r.servo_calls for r in world.robots.values())
    assert ex.authorize(now_s=.2)["phase"] == "HOLD"
    assert any(e["event"] == "COMMAND_ERROR" for e in ex.sync.events)


@pytest.mark.parametrize("fault", ["extra", "long", "mismatch", "wrong_stage", "missing_robot"])
def test_preflight_rejects_bad_second_command_before_first_moves(setup, fault):
    ex, world, _ = setup
    permission = ready(ex, 0.)
    commands = batch(ex)
    if fault == "extra": commands["r3"]["action"]["qpos"] = 1
    elif fault == "long": commands["r3"]["duration_s"] = .3
    elif fault == "mismatch": commands["r3"]["action"] = {"kind":"drive", "forward":.1, "turn":0., "duration_s":.8}
    elif fault == "wrong_stage": commands["r3"]["action"] = {"kind":"drive", "forward":.1, "turn":0., "duration_s":.2}
    else: commands.pop("r3")
    with pytest.raises(ValueError):
        ex.dispatch_pair(permission, commands, now_s=0.)
    assert not any(e["event"] == "COMMAND_ISSUED" for e in ex.sync.events)
    assert all(not r.servo_calls for r in world.robots.values())


def test_all_stages_use_actual_ports_but_verdicts_are_synthetic(setup):
    ex, world, _ = setup
    now = 0.
    for index, stage in enumerate(STAGES):
        assert ex.sync.stage == stage
        permission = ready(ex, now)
        commands = batch(ex, index)
        assert ex.dispatch_pair(permission, commands, now_s=now)
        now += .2
        ex.tick(now)
        assert not ex.advance(now_s=now)  # Sending a command is never completion.
        for rid in ex.ports:
            assert ex.receive(rid, reply(capture(ex, rid, now), "DONE", commands[rid]["command_id"]), now_s=now)
        assert ex.advance(now_s=now)
    assert ex.authorize(now_s=now)["phase"] == "FINISH"
    assert all(any(any(x != 0 for x in c) for c in r.motor_calls) for r in world.robots.values())
    assert all(r.motor_calls[-1] == [0.] * 4 for r in world.robots.values())
    ex.close(now_s=now)
    saved = json.loads((ex.output / "events.json").read_text())
    assert len([e for e in saved["sync"] if e["event"] == "STAGE_REPORTED_DONE"]) == 5


@pytest.mark.parametrize("fault", ["reused", "extra", "future", "superseded", "wrong_robot"])
def test_reply_binding_and_untrusted_fields(setup, fault):
    ex, _, _ = setup
    request = capture(ex, "r1", 0.)
    answer = reply(request)
    if fault == "reused": assert ex.receive("r1", answer, now_s=0.)
    elif fault == "extra": answer["contacts"] = True
    elif fault == "future": answer["decided_at_s"] = 1.
    elif fault == "superseded": capture(ex, "r1", 0.)
    assert not ex.receive("r3" if fault == "wrong_robot" else "r1", answer, now_s=0.)
    assert ex.authorize(now_s=0.)["phase"] == "HOLD"


def test_producer_receives_exact_rgb_and_only_own_history(setup):
    ex, _, _ = setup
    assert ex.dispatch_pair(ready(ex, 0.), batch(ex), now_s=0.)
    ex.tick(.2)
    request = capture(ex, "r1", .2)
    assert set(request) == {"schema", "request_id", "robot_id", "task_id", "run_id", "plan_version",
                            "stage", "epoch", "observed_at_s", "images", "plan", "own_issued_commands"}
    assert request["own_issued_commands"][0]["command_id"] == "r1-0"
    for image in request["images"].values():
        raw = base64.b64decode(image["jpeg_base64"], validate=True)
        assert raw == (ex.output / image["ref"]).read_bytes()
        assert hashlib.sha256(raw).hexdigest() == image["sha256"]
    request["stage"] = "RELEASE"  # Mutable producer input cannot forge the bound capture.
    request["images"]["own_rgb"]["ref"] = "fake"
    assert ex.receive("r1", reply({**request, "stage":"GRASP"}), now_s=.2)
    record = [e for e in ex.sync.events if e["event"] == "REPORT"][-1]["report"]
    assert record["stage"] == "GRASP" and record["own_rgb_ref"] != "fake"


def test_plan_update_revokes_and_map_hash_is_checked(setup):
    ex, world, map_path = setup
    assert ex.dispatch_pair(ready(ex, 0.), batch(ex), now_s=0.)
    ex.tick(.05)
    new_plan = replace(ex.sync.plan, plan_version=2)
    ex.update_plan(new_plan, map_path=map_path, now_s=.05)
    ex.tick(.2)
    assert all(r.servo_calls[-1] == {5:1800} for r in world.robots.values())
    assert ex.authorize(now_s=.2)["permission"] is None
    map_path.write_text("changed")
    with pytest.raises(ValueError):
        ex.update_plan(replace(new_plan, plan_version=3), map_path=map_path, now_s=.2)


def test_old_permission_after_hold_cannot_move(setup):
    ex, world, _ = setup
    old = ready(ex, 0.)
    ex.hold("pause", now_s=0.)
    ready(ex, .1)
    assert not ex.dispatch_pair(old, batch(ex), now_s=.1)
    ex.tick(.3)
    assert all(not r.servo_calls for r in world.robots.values())


def test_stop_failure_still_attempts_peer_and_aborts(setup, monkeypatch):
    ex, world, _ = setup
    assert ex.dispatch_pair(ready(ex, 0.), batch(ex), now_s=0.)
    def fail(*args): raise OSError("stop port failed")
    monkeypatch.setattr(ex.ports["r1"], "hold", fail)
    with pytest.raises(RuntimeError): ex.hold("pause", now_s=0.)
    ex.ports["r3"].tick(.3)
    assert world.robots["r3"].servo_calls == []
    assert ex.sync.status(now_s=.3)["phase"] == "ABORT"
