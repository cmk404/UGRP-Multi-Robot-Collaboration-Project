"""Offline actuator/clock tests; predictions and physics clock are fixtures."""
import base64
import copy
import hashlib
import json
from pathlib import Path
import threading

import pytest

from harness.rgb_execution_contract import SkillCapability
from harness.rgb_execution_port import RGBExecutionPort
from harness.rgb_skill_execution import (INITIAL_COMMANDS, SKILLS, MacroQueue, PairActorSkill,
    RGBSkillExecutionPort, _ActorFacade, _RelativeEndpoint, backend_descriptor, map_support_matrix,
    public_static_context, SoloActorSkill)
from sim.camera_robot_port import CameraRobotPort

JPEG = b"\xff\xd8test-only\xff\xd9"


class Robot:
    def __init__(self):
        self.servo_command_pulses = {int(k): v for k, v in INITIAL_COMMANDS.items()}
        self.motor_calls, self.servo_calls = [], []

    def set_motor_commands(self, command):
        self.motor_calls.append(list(command))

    def set_servo_pulses(self, command):
        self.servo_calls.append(dict(command))

    def __getattr__(self, name):
        raise AssertionError("truth read: " + name)


class Clock:
    time = 3.

    def __getattr__(self, name):
        raise AssertionError("truth read: " + name)


class World:
    def __init__(self):
        self.data = Clock()
        self.robots = {r: Robot() for r in ("r1", "r2", "r3")}
        self.steps = []

    def robot(self, rid):
        return self.robots[rid]


def static_context():
    return {"schema": "ugrp.rgb_static_context.v1", "task": {
        "description": "offline fixture only", "object_ids": ["box", "beam"], "allowed_skills": list(SKILLS)},
        "static_map": {"id": "test", "version": "1", "sha256": "a"*64, "description": "static"},
        "camera_calibration": {"id": "test", "version": "1", "sha256": "b"*64, "description": "static"}}


class Controller:
    def __init__(self, *, gate=None, action=None):
        self.gate, self.inputs, self.advances, self.resumes = gate, [], 0, 0
        self.action = action or {"kind": "drive", "fwd": .1, "turn": 0., "duration": .25}

    def decide(self, own, top):
        self.inputs.append((copy.deepcopy(own), top))
        if self.gate is not None:
            assert self.gate.wait(1.)
        return {"phase": "carry", "ready": False, "action": copy.deepcopy(self.action), "evidence": {}}

    def advance(self):
        self.advances += 1

    def resume(self):
        self.resumes += 1


def build(controllers=None, *, max_commands=10000):
    world = World()
    origin = world.data.time
    endpoints = {rid: CameraRobotPort(world, rid, allow_reverse=True, allow_mecanum=True) for rid in world.robots}
    controllers = controllers or {rid: Controller() for rid in world.robots}
    def advance(delta):
        world.steps.append(delta)
        # This fixture advances ONLY a command clock, not physical simulation.
        for _ in range(round(delta/.005)):
            world.data.time += .005
            for endpoint in endpoints.values():
                endpoint.tick(world.data.time)
    port = RGBSkillExecutionPort({rid: _RelativeEndpoint(p, origin) for rid, p in endpoints.items()},
        frame_source=lambda rid, now: {"own_rgb": JPEG, "top_rgb": JPEG},
        static_context=static_context(), skill_factory=lambda rid, role, skill: controllers[rid],
        advance_physics=advance, command_states={rid: dict(INITIAL_COMMANDS) for rid in endpoints},
        max_commands=max_commands)
    return port, world, endpoints, controllers


def submit(port, rid, *, skill="solo_transport_A", task="box-task", participants=None, role="solo", **changes):
    observed = port.observe(rid)
    spec = SKILLS[skill]
    payload = {"kind": "task_request", "request_id": f"{rid}-{task}-{observed['observation_id']}",
        "task_id": task, "object_id": spec["object_id"], "skill": skill,
        "participants": participants or [rid], "resources": spec["resources"], "stage": "RUN",
        "own_role": role, "observation_id": observed["observation_id"], "decision_id": rid+"-decision",
        "requested_at_s": observed["observed_at_s"], "expires_at_s": 10.,
        "expected_revision": observed["own_revision"], **changes}
    return port.submit(rid, payload)


def lifecycle(port, rid, kind, *, task=None, **changes):
    observed = port.observe(rid)
    active = port.local_status(rid)["active"]
    payload = {"kind": kind, "request_id": kind+observed["observation_id"],
        "task_id": task or active[0]["task_id"], "reason": "actor decision",
        "observation_id": observed["observation_id"], "decision_id": "life-"+observed["observation_id"],
        "requested_at_s": observed["observed_at_s"], "expected_revision": observed["own_revision"]}
    if kind != "cancel_pending":
        payload["lease_id"] = active[0]["lease_id"]
    return port.submit(rid, {**payload, **changes})


def settle_workers(port):
    for runner in list(port._runners.values()):
        if runner.future is not None:
            runner.future.result(timeout=1.)
    port.tick(port._now_s)


def step(port, count=1):
    for _ in range(count):
        port.tick(round(port._now_s+.05, 10))


def test_actual_camera_port_receives_skill_commands_without_world_in_controller():
    port, world, endpoints, controllers = build()
    try:
        assert submit(port, "r2")["status"] == "ACCEPTED"
        port.tick(0.)
        settle_workers(port)
        step(port, 4)
        assert any(any(c) for c in world.robots["r2"].motor_calls)
        assert world.steps == pytest.approx([.05]*4)
        assert not world.robots["r1"].motor_calls
        own, top = controllers["r2"].inputs[0]
        assert set(own) == {"robot_id", "frame_id", "sim_time", "camera", "image", "sha256", "actuator_state"}
        assert own["robot_id"] == "r2" and top == JPEG
        issued = port.observe("r2")["own_issued_commands"]
        assert issued and all(r["duration_s"] <= .1 for r in issued)
        assert all(r["decision_id"] == "r2-decision" and r["observation_id"] for r in issued)
        assert all(r["skill_decision_id"] for r in issued)
        assert not hasattr(_ActorFacade(port), "evaluation_snapshot")
    finally:
        port.close(port._now_s)


def test_slow_pair_image_worker_does_not_block_solo_or_clock():
    gate = threading.Event()
    controllers = {"r1": Controller(gate=gate), "r2": Controller(), "r3": Controller()}
    port, world, _, _ = build(controllers)
    try:
        for rid, role in (("r1", "lower"), ("r3", "upper")):
            submit(port, rid, task="beam-task", skill="pair_transport_A", participants=["r1", "r3"], role=role)
        submit(port, "r2")
        port.tick(0.)
        port._runners["r2"].future.result(timeout=1.)
        step(port, 2)
        assert any(any(c) for c in world.robots["r2"].motor_calls)
        assert not world.robots["r1"].motor_calls
        assert len(world.steps) == 2
        lifecycle(port, "r1", "interrupt")
        gate.set()
        step(port, 2)
        assert not any(any(c) for c in world.robots["r1"].motor_calls)
        assert port.local_status("r1")["active"] == []
    finally:
        gate.set()
        port.close(port._now_s)


def test_none_independent_pair_consent_and_pause_resume_cleanup():
    port, world, _, controllers = build()
    try:
        result = submit(port, "r1", skill="pair_transport_A", task="pair", participants=["r1", "r3"], role="lower")
        assert result["status"] == "PENDING"
        port.tick(0.)
        assert not port._runners
        assert submit(port, "r3", skill="pair_transport_A", task="pair", participants=["r1", "r3"], role="upper")["status"] == "ACCEPTED"
        port.tick(0.)
        settle_workers(port)
        step(port)
        assert lifecycle(port, "r1", "pause")["status"] == "ACCEPTED"
        for rid in ("r1", "r3"):
            assert world.robots[rid].motor_calls[-1] == [0.]*4
            assert port._runners[rid].macro.events == []
        step(port, 4)
        assert lifecycle(port, "r1", "resume")["reason"] == "awaiting_independent_resume"
        assert port.local_status("r1")["active"][0]["paused"] is True
        assert lifecycle(port, "r3", "resume")["reason"] == "task_resumed"
        port.tick(port._now_s)
        settle_workers(port)
        assert controllers["r1"].resumes == controllers["r3"].resumes == 1
    finally:
        port.close(port._now_s)


def test_revision_retains_old_observation_but_revoked_intent_cannot_return():
    port, _, _, _ = build()
    try:
        observed = port.observe("r1")
        port.observe("r1")
        assert submit(port, "r1", observation_id=observed["observation_id"], expected_revision=observed["own_revision"])["status"] == "ACCEPTED"
        assert lifecycle(port, "r1", "interrupt")["status"] == "ACCEPTED"
        assert submit(port, "r1", task="new", observation_id=observed["observation_id"], expected_revision=observed["own_revision"])["status"] == "REJECTED"
        assert submit(port, "r1", task="new")["status"] == "ACCEPTED"
    finally:
        port.close(port._now_s)


def test_pending_cancel_releases_identity_but_old_task_cannot_reappear():
    port, _, _, _ = build()
    try:
        assert submit(port, "r1", skill="pair_transport_A", task="pair", participants=["r1", "r3"], role="lower")["status"] == "PENDING"
        assert lifecycle(port, "r1", "cancel_pending", task="pair")["status"] == "ACCEPTED"
        assert submit(port, "r3", skill="pair_transport_A", task="pair", participants=["r1", "r3"], role="upper")["status"] == "REJECTED"
        assert submit(port, "r1", task="solo-new")["status"] == "ACCEPTED"
    finally:
        port.close(port._now_s)


@pytest.mark.parametrize("kind", ["interrupt", "release", "pause", "expiry", "close"])
def test_cancelled_pose_never_replays_after_lifecycle_change(kind):
    controllers = {rid: Controller(action={"kind": "pose", "pulses": {5: 2000}}) for rid in ("r1", "r2", "r3")}
    port, world, _, _ = build(controllers)
    try:
        submit(port, "r2", expires_at_s=.15 if kind == "expiry" else 10.)
        port.tick(0.)
        settle_workers(port)
        step(port)
        if kind in {"interrupt", "release", "pause"}:
            assert lifecycle(port, "r2", kind)["status"] == "ACCEPTED"
        elif kind == "close":
            port.close(port._now_s)
        else:
            step(port, 2)
        calls = len(world.robots["r2"].servo_calls)
        if kind != "close":
            step(port, 4)
        assert len(world.robots["r2"].servo_calls) == calls
    finally:
        port.close(port._now_s)


def test_clock_jump_close_and_raw_budget_fail_closed():
    port, world, _, _ = build(max_commands=1)
    try:
        with pytest.raises(ValueError):
            port.tick(.1)
        assert world.steps == []
        submit(port, "r2")
        port.tick(0.)
        settle_workers(port)
        step(port, 3)
        assert port.local_status("r2")["own_skill_status"]["state"] == "STOPPED"
        assert port._raw_count == 1
        port.close(port._now_s)
        count = len(world.steps)
        with pytest.raises(RuntimeError):
            port.tick(port._now_s+.05)
        assert len(world.steps) == count
    finally:
        port.close(port._now_s)


def test_finish_claim_does_not_read_or_produce_evaluator_success():
    controllers = {rid: Controller(action={"kind": "finish", "reason": "SYNTHETIC_RGB_CLAIM"}) for rid in ("r1", "r2", "r3")}
    port, world, _, _ = build(controllers)
    try:
        submit(port, "r2")
        port.tick(0.)
        settle_workers(port)
        status = port.local_status("r2")
        assert status["own_skill_status"]["state"] == "FINISHED_UNVERIFIED"
        assert "mission_complete" not in json.dumps(status)
        assert "physical_success" not in json.dumps(status)
        assert status["active"] == []
    finally:
        port.close(port._now_s)


def test_macro_does_not_infer_actual_pose_and_bounds_every_drive():
    commands = dict(INITIAL_COMMANDS)
    issued = []
    macro = MacroQueue(lambda action, duration: issued.append((action, duration)), commands)
    macro.submit({"kind": "drive", "fwd": .1, "turn": 0., "duration": .25}, 0.)
    for t in (0., .05, .1, .15, .2, .25):
        macro.tick(t)
    assert len(issued) == 3 and max(d for _, d in issued) <= .1
    macro.submit({"kind": "pose", "pulses": {5: 1380}}, .3)
    macro.tick(.35)
    macro.cancel(.35)
    count = len(issued)
    macro.tick(5.)
    assert len(issued) == count


def test_all_suite_maps_are_explicitly_unsupported_and_never_remapped():
    matrix = map_support_matrix()
    assert len(matrix) == 23
    assert any(row.get("split") == "dev" for row in matrix)
    assert all(not row["skill_supported"] for row in matrix[1:])
    for row in matrix[1:]:
        with pytest.raises(ValueError, match="unsupported map"):
            backend_descriptor({"schema": "ugrp.rgb_skill_backend.v1", "map_id": row["map_id"],
                "seed": 11, "output_dir": "never-created", "max_sim_s": 180, "max_commands": 10000,
                "grasp_model_dir": "not-read", "stage_model_dir": "not-read", "reference_top": "not-read"})


def test_pair_controller_resumes_only_rgb_reobservable_phases():
    actor = PairActorSkill.__new__(PairActorSkill)
    actor.phases, actor.index, actor.confirmations = ["carry"], 0, 2
    actor.resume()
    assert actor.confirmations == 0
    actor.phases = ["close"]
    with pytest.raises(ValueError, match="explicit recovery"):
        actor.resume()


def manual_port():
    world = World()
    endpoints = {rid: _RelativeEndpoint(CameraRobotPort(world, rid), 3.) for rid in world.robots}
    caps = [SkillCapability(name, frozenset({"RUN"}), frozenset({spec["team_size"]}),
                            frozenset({"drive"}), .1, distinct_roles=True) for name, spec in SKILLS.items()]
    port = RGBExecutionPort(endpoints, frame_source=lambda rid, now: {"own_rgb": JPEG, "top_rgb": JPEG},
        static_context=static_context(), capabilities=caps, clock_domain="sim", strict_revisions=True)
    return port, world


def manual_command(port, rid):
    observed = port.observe(rid)
    active = port.local_status(rid)["active"][0]
    return {"kind": "command", "request_id": "request-"+observed["observation_id"],
        "task_id": active["task_id"], "lease_id": active["lease_id"],
        "command_id": "command-"+observed["observation_id"], "stage": "RUN",
        "action": {"kind": "drive", "forward": .1, "turn": 0., "duration_s": .1},
        "duration_s": .1, "observation_id": observed["observation_id"],
        "decision_id": "decision-"+observed["observation_id"], "requested_at_s": observed["observed_at_s"],
        "expected_revision": observed["own_revision"]}


def activate_pair(port):
    for rid, role in (("r1", "lower"), ("r3", "upper")):
        submit(port, rid, skill="pair_transport_A", task="pair", participants=["r1", "r3"], role=role)


def test_old_queued_command_is_rechecked_when_late_partner_arrives():
    port, world = manual_port()
    activate_pair(port)
    assert port.submit("r1", manual_command(port, "r1"))["status"] == "QUEUED"
    port.tick(6.)
    assert port.submit("r3", manual_command(port, "r3"))["status"] == "QUEUED"
    port.tick(6.)
    assert not any(any(c) for c in world.robots["r1"].motor_calls)
    assert not any(any(c) for c in world.robots["r3"].motor_calls)
    assert port.local_status("r1")["active"] == []
    port.close(6.)


def test_sensor_failure_revokes_queued_joint_commands_not_just_own_wheels():
    port, world = manual_port()
    activate_pair(port)
    for rid in ("r1", "r3"):
        port.submit(rid, manual_command(port, rid))
    port._frame_source = lambda rid, now: {"own_rgb": b"invalid", "top_rgb": JPEG}
    with pytest.raises(ValueError):
        port.observe("r1")
    port.tick(.1)
    assert port._active == {}
    assert not any(any(c) for r in world.robots.values() for c in r.motor_calls)
    port.close(.1)


def test_failed_hold_quarantines_resource_from_reallocation(monkeypatch):
    port, _ = manual_port()
    activate_pair(port)
    monkeypatch.setattr(port._endpoints["r1"], "hold", lambda now: (_ for _ in ()).throw(OSError("lost endpoint")))
    assert lifecycle(port, "r1", "interrupt")["hold_failed"]
    assert port._resource_owners["beam"] == "pair"
    assert submit(port, "r2", task="new", skill="pair_transport_A", participants=["r2", "r3"], role="lower")["status"] == "REJECTED"
    port.close(0.)


def write_assets(tmp_path):
    grasp, staged = tmp_path / "grasp", tmp_path / "stages"
    grasp.mkdir(); staged.mkdir()
    g, s = {}, {}
    for rid in ("r1", "r3"):
        content = json.dumps({"schema": "ugrp.rgb_recovery_kernel.v1", "channels": [3, 4, 5]})
        (grasp / (rid+".json")).write_text(content)
        g[rid] = {"path": rid+".json", "sha256": hashlib.sha256(content.encode()).hexdigest()}
        s[rid] = {}
        for stage in ("yaw", "lateral", "forward"):
            content = json.dumps({"robot_id": rid, "stage": stage})
            name = rid+stage+".json"
            (staged / name).write_text(content)
            s[rid][stage] = {"path": name, "sha256": hashlib.sha256(content.encode()).hexdigest()}
    (grasp / "student-skill.json").write_text(json.dumps({"models": g}))
    (staged / "varied-start-skill.json").write_text(json.dumps({"models": s}))
    reference = tmp_path / "reference.jpg"
    reference.write_bytes(JPEG)
    return {"schema": "ugrp.rgb_skill_backend.v1", "map_id": "dispatch_open", "seed": 11,
            "output_dir": str(tmp_path / "not-created"), "max_sim_s": 180, "max_commands": 10000,
            "grasp_model_dir": str(grasp), "stage_model_dir": str(staged), "reference_top": str(reference)}


def test_readonly_descriptor_binds_models_map_camera_goal_and_reset(tmp_path):
    config = write_assets(tmp_path)
    a = backend_descriptor(config)
    b = backend_descriptor({**config, "seed": 29})
    assert a["map_instance_sha256"] == b["map_instance_sha256"]
    assert a["map_group_sha256"] == b["map_group_sha256"]
    assert a["reset_sha256"] != b["reset_sha256"]
    assert a["camera_sha256"] == b["camera_sha256"]
    assert a["goal_frame"] == "warehouse_xy_m"
    assert not Path(config["output_dir"]).exists()
    public = public_static_context(config)
    assert public == public_static_context({**config, "seed": 29})
    for forbidden in ("setup_only", "split", "nominal_start", "expected_outcome", "seed"):
        assert forbidden not in json.dumps(public)
    path = Path(config["grasp_model_dir"]) / "r1.json"
    path.write_text("{}")
    with pytest.raises(ValueError, match="hash mismatch"):
        backend_descriptor(config)


def test_existing_solo_rgb_policy_is_called_without_full_plan_or_world():
    import cv2
    import numpy as np
    from sim.research_dispatch_arena import authored_map
    jpeg = cv2.imencode(".jpg", np.zeros((480, 640, 3), np.uint8))[1].tobytes()
    own = {"robot_id": "r2", "frame_id": 1, "sim_time": 0., "camera": "robot_cam",
        "image": base64.b64encode(jpeg).decode(), "sha256": hashlib.sha256(jpeg).hexdigest(),
        "actuator_state": {"motor_commands": [0.]*4, "servo_pulses": dict(INITIAL_COMMANDS)}}
    actor = SoloActorSkill("r2", authored_map("open"), SKILLS["solo_transport_A"])
    assert actor.decide(own, jpeg)["action"]["kind"] == "pose"
    decision = actor.decide({**own, "frame_id": 2, "sim_time": 1.}, jpeg)
    assert decision["action"]["kind"] in {"pose", "drive", "wait", "finish"}
    assert actor.skill.steps == 2
    assert not hasattr(actor, "world")
    assert not hasattr(actor.skill.navigator, "bindings")


def test_existing_pair_rgb_coarse_policy_and_phase_barrier_are_incremental():
    from sim.research_dispatch_arena import authored_map
    reference = (Path(__file__).parent / "fixtures/camera_goal_transport/reference-top.jpg").read_bytes()
    own = {"image": base64.b64encode(reference).decode(),
           "actuator_state": {"servo_pulses": dict(INITIAL_COMMANDS)}}
    saved = {"initialization_replay": [{"targets": {}}]}
    actor = PairActorSkill("r2", "lower", authored_map("open"), SKILLS["pair_transport_A"],
                          (saved, {"r1": {}}, {"r1": {}}, reference))
    first, second = actor.decide(own, reference), actor.decide(own, reference)
    assert first["phase"] == second["phase"] == "coarse"
    assert not first["ready"] and second["ready"]
    assert second["action"]["kind"] == "wait"
    assert actor.phase == "coarse"  # No inferred peer readiness.
    actor.advance()
    assert actor.phase == "yaw"
    assert actor.slot == "r1" and actor.robot_id == "r2"
