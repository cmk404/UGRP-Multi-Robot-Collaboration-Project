"""Offline actuator/clock tests; predictions and physics clock are fixtures."""
import base64
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import pytest

from harness.rgb_execution_contract import SkillCapability
from harness.rgb_execution_port import RGBExecutionPort
from harness.rgb_skill_execution import (INITIAL_COMMANDS, SKILLS, MacroQueue, PairActorSkill,
    RGBSkillExecutionPort, _ActorFacade, _RelativeEndpoint, _SimulationClock, backend_descriptor, map_support_matrix,
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


def build_dispatch_float_clock():
    """The recovered bfe478f clock, using DispatchScene.step but NO physics."""
    from scripts.research_dispatch_scene import DispatchScene
    world = World()
    world.data.time = 1.300000000000001
    world.data.eq_active = SimpleNamespace(any=lambda: False)
    world.data.ncon = 0
    world.data.contact = []
    world.model = SimpleNamespace(opt=SimpleNamespace(timestep=.002))
    world.controllers = world.robots
    def add_timestep(_):
        world.data.time += world.model.opt.timestep
    world._physics_step_for = add_timestep
    scene = DispatchScene({}, "not-created-float-clock-fixture")
    scene.world = world
    scene.ports = {rid: CameraRobotPort(world, rid) for rid in world.robots}
    scene.robot_ids = scene.obstacle_ids = set()
    clock = _SimulationClock(lambda: world.data.time, origin=world.data.time, timestep=.002, max_sim_s=180.)
    port = RGBSkillExecutionPort({rid: _RelativeEndpoint(p, world.data.time, clock=clock)
        for rid, p in scene.ports.items()}, frame_source=lambda rid, now: {"own_rgb": JPEG, "top_rgb": JPEG},
        static_context=static_context(), skill_factory=lambda rid, role, skill: Controller(),
        advance_physics=scene.step, command_states={rid: dict(INITIAL_COMMANDS) for rid in world.robots},
        simulation_clock=clock)
    return port, scene


def test_recovered_dispatch_clock_survives_exact_point_eight_boundary():
    port, scene = build_dispatch_float_clock()
    try:
        for i in range(16):
            port.tick(i*.05)
        assert scene.world.data.time == 2.049999999999996
        port.tick(.8)
        assert port._now_s == .8
        assert all(p._servo_tick_time == scene.world.data.time for p in scene.ports.values())
    finally:
        port.close(port._now_s)


def test_accumulated_clock_through_full_180_second_budget_without_physics():
    port, scene = build_dispatch_float_clock()
    try:
        for i in range(3601):
            port.tick(i*.05)
        actual = scene.world.data.time-port._simulation_clock.origin
        assert port.clock_snapshot() == {"schema": "ugrp.execution_clock.v1", "clock_domain": "sim",
            "last_acknowledged_time_s": actual, "actual_time_s": actual}
        assert abs(actual-180.) <= port._simulation_clock.roundoff_bound(180.)
        assert all(p._servo_tick_time == scene.world.data.time for p in scene.ports.values())
        with pytest.raises(ValueError):
            port.tick(180.05)
        # C supplies the verified raw actual value, including accumulation
        # roundoff above the requested 180-second cap. Close never steps.
        steps_before_close = scene.physics_steps
        port.close(actual)
        assert port._closed
        assert scene.physics_steps == steps_before_close
    finally:
        port.close(None)


@pytest.mark.parametrize("requested", [-.01, math.nan, math.inf, -math.inf, True, .1])
def test_invalid_requested_clock_never_steps(requested):
    port, scene = build_dispatch_float_clock()
    try:
        with pytest.raises(ValueError):
            port.tick(requested)
        assert scene.physics_steps == 0
    finally:
        port.close(None)


@pytest.mark.parametrize("partial", [0., .02])
def test_failed_tick_preserves_partial_actual_and_prior_ack_and_holds_all(partial):
    port, scene = build_dispatch_float_clock()
    for i in range(16):
        port.tick(i*.05)
    submit(port, "r2")
    port.tick(.75)
    settle_workers(port)
    acknowledged = port.clock_snapshot()["actual_time_s"]
    def fail_after_partial(_):
        scene.step(partial)
        raise OSError("fixture physics owner lost")
    port._advance_physics = fail_after_partial
    with pytest.raises(RuntimeError, match="physics owner unavailable") as raised:
        port.tick(.8)
    assert isinstance(raised.value.__cause__, OSError)
    assert raised.value.error_code == "PHYSICS_OWNER_UNAVAILABLE"
    snapshot = port.clock_snapshot()
    assert snapshot["last_acknowledged_time_s"] == acknowledged
    assert snapshot["actual_time_s"] == scene.world.data.time-port._simulation_clock.origin
    assert snapshot["actual_time_s"] == pytest.approx(.75+partial)
    assert port._now_s == .75 and port._closed
    assert all(r.motor_calls[-1] == [0.]*4 for r in scene.world.robots.values())
    assert all(not p._servo_targets for p in scene.ports.values())
    assert not port._active and not port._resource_owners
    steps = scene.physics_steps
    port.close(None)
    assert port.clock_snapshot() == snapshot and scene.physics_steps == steps


@pytest.mark.parametrize("bad_clock", [math.nan, math.inf, -1., "one_ulp_back", "meaningful_back"])
def test_invalid_actual_clock_stops_commands_and_never_claims_last_ack_as_actual(bad_clock):
    port, scene = build_dispatch_float_clock()
    port.tick(0.)
    submit(port, "r2")
    port.tick(0.)
    settle_workers(port)
    port.tick(.05)
    acknowledged = port.clock_snapshot()["last_acknowledged_time_s"]
    current = scene.world.data.time
    scene.world.data.time = (math.nextafter(current, -math.inf) if bad_clock == "one_ulp_back"
        else current-.01 if bad_clock == "meaningful_back" else bad_clock)
    # Isolate the boundary from the arithmetic world fixture, not a real step.
    port._advance_physics = lambda delta: None
    with pytest.raises(RuntimeError):
        port.tick(.1)
    assert port.clock_snapshot()["actual_time_s"] is None
    assert port.clock_snapshot()["last_acknowledged_time_s"] == acknowledged
    assert all(r.motor_calls[-1] == [0.]*4 for r in scene.world.robots.values())
    assert all(not p._servo_targets for p in scene.ports.values())
    assert port._closed


def test_meaningful_grid_mismatch_is_not_hidden_by_roundoff_bound():
    port, scene = build_dispatch_float_clock()
    port.tick(0.)
    port._advance_physics = lambda delta: scene.step(.048)
    with pytest.raises(RuntimeError) as raised:
        port.tick(.05)
    assert raised.value.__cause__.error_code == "SIM_CLOCK_MISMATCH"
    assert port.clock_snapshot()["last_acknowledged_time_s"] == 0.
    assert port.clock_snapshot()["actual_time_s"] == pytest.approx(.048)


@pytest.mark.parametrize("claimed", [-1., math.nan, math.inf, .01, .2, 180.05])
def test_close_rejects_invalid_or_unverified_clock_without_advancing(claimed):
    port, scene = build_dispatch_float_clock()
    port.tick(.05)
    before = port.clock_snapshot()
    steps = scene.physics_steps
    try:
        with pytest.raises(ValueError):
            port.close(claimed)
        assert not port._closed
        assert port.clock_snapshot() == before and scene.physics_steps == steps
    finally:
        port.close(None)


@pytest.mark.parametrize("kind", ["pause", "interrupt", "release", "expiry", "close"])
def test_actual_clock_cancels_arm_interpolation_without_replay(kind):
    port, scene = build_dispatch_float_clock()
    port._skill_factory = lambda *args: Controller(action={"kind": "pose", "pulses": {5: 2000}})
    try:
        submit(port, "r2", expires_at_s=.15 if kind == "expiry" else 10.)
        port.tick(0.)
        settle_workers(port)
        port.tick(.05)
        if kind in {"pause", "interrupt", "release"}:
            assert lifecycle(port, "r2", kind)["status"] == "ACCEPTED"
        elif kind == "close":
            port.close(None)
        else:
            port.tick(.1)
            port.tick(.15)
        calls = len(scene.world.robots["r2"].servo_calls)
        # Continue just the clock owner after shutdown to test no queued replay.
        scene.step(.2)
        assert len(scene.world.robots["r2"].servo_calls) == calls
        assert not scene.ports["r2"]._servo_targets
    finally:
        port.close(None)


def test_supervisor_clock_is_readonly_separate_and_available_after_shutdown(monkeypatch):
    port, scene = build_dispatch_float_clock()
    port.tick(0.)
    def prohibited(*args):
        raise AssertionError("clock getter must not observe, render, evaluate or step")
    monkeypatch.setattr(port, "_frame_source", prohibited)
    monkeypatch.setattr(port, "_advance_physics", prohibited)
    monkeypatch.setattr(port, "_evaluation_source", prohibited)
    assert not hasattr(_ActorFacade(port), "clock_snapshot")
    snapshot = port.clock_snapshot()
    assert set(snapshot) == {"schema", "clock_domain", "last_acknowledged_time_s", "actual_time_s"}
    port.close(None)
    port._simulation_clock.freeze()
    monkeypatch.setattr(port._simulation_clock, "_read_absolute", prohibited)
    assert port.clock_snapshot() == snapshot
    assert scene.physics_steps == 0


@pytest.mark.parametrize("wall_age,sim_age,rejected", [(2., 1., False), (2.01, .05, True), (.01, 1.01, True)])
def test_worker_guard_logs_actual_duration_separately_from_late_poll(monkeypatch, wall_age, sim_age, rejected):
    import harness.rgb_skill_execution as skills
    port, world, _, _ = build()
    rows = []
    port._record_supervisor = rows.append
    try:
        submit(port, "r2")
        port.tick(0.)
        runner = port._runners["r2"]
        runner.future.result(timeout=1.)
        # A completed image-only fixture, then only the supervisor polling
        # clock/image-age changes. Neither cap uses worker completion time.
        runner.observation["observed_at_s"] = -sim_age
        monkeypatch.setattr(skills, "time", SimpleNamespace(monotonic=lambda: runner.started_wall+wall_age))
        port.tick(0.)
        denials = [r for r in rows if r["event"] == "RGB_WORKER_REJECTED"]
        assert bool(denials) is rejected
        if rejected:
            row = denials[0]
            assert row["wall_cap_exceeded"] is (wall_age > 2.)
            assert row["sim_age_exceeded"] is (sim_age > 1.)
            assert row["image_age_s"] == sim_age
            assert row["submission_to_poll_wall_s"] == pytest.approx(wall_age)
            assert row["worker_wall_duration_s"] >= 0 and row["future_done"]
            assert row["wall_cap_s"] == 2. and row["sim_age_cap_s"] == 1.
            assert not any(any(c) for c in world.robots["r2"].motor_calls)
        assert any(r["event"] == "RGB_WORKER_COMPLETED" for r in rows)
        assert "worker_wall_duration_s" not in json.dumps(port.local_status("r2"))
    finally:
        port.close(None)


def test_in_flight_wall_timeout_logs_unknown_duration_then_late_completion(monkeypatch):
    import harness.rgb_skill_execution as skills
    entered, release = threading.Event(), threading.Event()
    wall = [100.]
    class WaitingController(Controller):
        def decide(self, own, top):
            entered.set()
            assert release.wait(1.)
            return super().decide(own, top)
    monkeypatch.setattr(skills, "time", SimpleNamespace(monotonic=lambda: wall[0]))
    port, world, _, _ = build({"r2": WaitingController()})
    rows = []
    port._record_supervisor = rows.append
    try:
        submit(port, "r2")
        port.tick(0.)
        assert entered.wait(1.)
        future = port._runners["r2"].future
        wall[0] = 102.01
        port.tick(.05)
        denial = next(r for r in rows if r["event"] == "RGB_WORKER_REJECTED")
        assert denial["guard"] == "in_flight_wall"
        assert denial["wall_cap_exceeded"] and not denial["sim_age_exceeded"]
        assert denial["worker_wall_duration_s"] is None and not denial["future_done"]
        wall[0] = 103.01
        release.set()
        future.result(timeout=1.)
        completion = next(r for r in rows if r["event"] == "RGB_WORKER_COMPLETED")
        assert completion["worker_wall_duration_s"] == pytest.approx(3.01)
        port.tick(.1)
        assert not any(any(c) for c in world.robots["r2"].motor_calls)
    finally:
        release.set()
        port.close(None)


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


def test_completed_worker_result_still_requires_fresh_sim_observation():
    port, world, _, _ = build()
    try:
        submit(port, "r2")
        port.tick(0.)
        port._runners["r2"].future.result(timeout=1.)
        port._runners["r2"].observation["observed_at_s"] = -1.01
        port.tick(0.)
        assert port.local_status("r2")["own_skill_status"]["state"] == "STOPPED"
        assert not any(any(c) for c in world.robots["r2"].motor_calls)
    finally:
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
    assert a["supervisor_clock_schema"] == "ugrp.execution_clock.v1"
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


def test_real_factory_wires_clock_only_callback_and_raw_snapshot_with_fixture_scene(tmp_path, monkeypatch):
    import harness.rgb_skill_execution as skills
    import scripts.research_dispatch_scene as dispatch
    import scripts.run_dispatch_e2e as evaluation
    fixture_port, fixture_scene = build_dispatch_float_clock()
    fixture_port.close(None)
    events = []
    class FixtureScene(dispatch.DispatchScene):
        def open(self):
            self.world, self.ports = fixture_scene.world, fixture_scene.ports
            self.robot_ids = self.obstacle_ids = set()
            self.manifest = {"scene_xml_sha256": "a"*64}
            self.initial_invariants = {"weld_active": False}
            self.out.mkdir()
            (self.out / "rgb").mkdir()
        def invariants(self):
            return {"weld_active": False}
        def close(self):
            self.world = None
    class Referee:
        def __init__(self, scene):
            self.file = SimpleNamespace(close=lambda: events.append("referee_close"))
        def sample(self):
            events.append("sample")
        def finish(self, plan):
            return {"cargo": {}}
    class Video:
        def __init__(self, *args):
            pass
        def capture(self, **kwargs):
            events.append("capture")
        def close(self):
            events.append("video_close")
    monkeypatch.setattr(dispatch, "DispatchScene", FixtureScene)
    monkeypatch.setattr(evaluation, "Referee", Referee)
    # Do not import the real Video module: its import-time MuJoCo dependency
    # is deliberately absent in offline CI. Also deny it on rich local envs.
    monkeypatch.setitem(sys.modules, "mujoco", None)
    monkeypatch.setitem(sys.modules, "scripts.probe_dual_grasp_sync", SimpleNamespace(Video=Video))
    bundle = skills.build_rgb_skill_backend(write_assets(tmp_path))
    try:
        for i in range(17):
            bundle.actor_port.tick(i*.05)
        before = list(events)
        clock = bundle.clock_snapshot()
        assert clock["actual_time_s"] == fixture_scene.world.data.time-1.300000000000001
        assert events == before  # clock-only callback never samples or captures
        bundle.actor_port.close(clock["actual_time_s"])
        assert bundle.evaluation_snapshot()["timestamp_s"] == clock["actual_time_s"]
        bundle.close()
        assert bundle.clock_snapshot() == clock
        assert not hasattr(bundle.actor_port, "clock_snapshot")
    finally:
        bundle.close()


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
