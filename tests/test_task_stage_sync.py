from dataclasses import replace
import json
import math
import random

import pytest

from harness.task_stage_sync import DONE_CHECKS, READY_CHECKS, STAGES, StageReport, TaskPlan, TaskStageSync


def plan():
    return TaskPlan("beam-1", 1, "beam", (("r1", "left"), ("r3", "right")),
                    "drop-zone", "room", "1", "a" * 64)


class Client:
    def __init__(self, sync):
        self.sync = sync
        self.sequences = {r: 0 for r in sync.plan.participants}
        self.issued = []

    def report(self, robot="r1", now=1.0, status="READY", **changes):
        self.sequences[robot] += 1
        sequence = self.sequences[robot]
        checks = DONE_CHECKS[self.sync.stage] if status == "DONE" else READY_CHECKS[self.sync.stage]
        report = StageReport(self.sync.plan.task_id, self.sync.run_id, self.sync.plan.plan_version,
                             self.sync.stage, self.sync.epoch, robot, sequence, status,
                             now, now, now, f"{robot}-{sequence}", f"own/{robot}/{sequence}.png",
                             f"top/{sequence}.png", .95, tuple(sorted(checks)), None, "test fixture")
        return replace(report, **changes).to_dict()

    def send(self, robot="r1", now=1.0, status="READY", **changes):
        return self.sync.receive(robot, self.report(robot, now, status, **changes), now_s=now)

    def ready(self, now):
        for robot in self.sync.plan.participants:
            assert self.send(robot, now)
        return self.sync.authorize(now_s=now)["permission"]

    def command(self, robot, permit, now, suffix="", duration=.1):
        command_id = f"{self.sync.stage}-{robot}-{suffix}"
        allowed = self.sync.dispatch(robot, permit, command_id, now_s=now, duration_s=duration,
                                     submit=lambda: self.issued.append(command_id))
        return allowed, command_id

    def complete(self, now):
        permission = self.ready(now)
        ids = {r: self.command(r, permission, now)[1] for r in self.sync.plan.participants}
        for r in ids:
            assert self.send(r, now + .11, "DONE", command_id=ids[r])
        assert self.sync.advance(now_s=now + .11)


def test_plan_and_report_strict_json_roundtrip():
    original = plan()
    assert TaskPlan.from_dict(json.loads(json.dumps(original.to_dict()))) == original
    sync = TaskStageSync(original)
    wire = Client(sync).report()
    assert StageReport.from_dict(json.loads(json.dumps(wire))).to_dict() == wire
    for extra in ("qpos", "contact", "robot_xyz", "success"):
        with pytest.raises(ValueError):
            TaskPlan.from_dict({**original.to_dict(), extra: 1})
        assert not sync.receive("r1", {**wire, extra: 1}, now_s=1)


@pytest.mark.parametrize("field,value", [("plan_version", True), ("epoch", False),
    ("sequence", -1), ("confidence", math.nan), ("confidence", 2), ("own_rgb_ref", ""),
    ("top_rgb_ref", None), ("observed_at_s", math.inf), ("stage", []),
    ("status", "ACK"), ("checks", ["contact_true"]), ("evidence_source", "simulator")])
def test_malformed_reports_are_rejected(field, value):
    sync = TaskStageSync(plan())
    wire = Client(sync).report()
    wire[field] = value
    assert not sync.receive("r1", wire, now_s=1)
    assert sync.authorize(now_s=1)["phase"] == "WAIT"


@pytest.mark.parametrize("field,value", [("plan_version", True), ("map_sha256", "invalid"),
    ("roles", (("r1", "left"), ("r1", "right"))), ("task_id", "")])
def test_invalid_plans_rejected(field, value):
    with pytest.raises(ValueError):
        replace(plan(), **{field: value})


def test_five_stages_need_both_ready_and_both_post_command_done():
    sync = TaskStageSync(plan())
    client = Client(sync)
    for index, stage in enumerate(STAGES):
        now = 1.0 + index
        assert sync.stage == stage
        assert client.send("r1", now)
        assert sync.authorize(now_s=now)["permission"] is None
        assert not sync.advance(now_s=now)
        assert client.send("r3", now)
        permit = sync.authorize(now_s=now)["permission"]
        commands = {r: client.command(r, permit, now)[1] for r in plan().participants}
        # Issuing both commands is not evidence of physical completion.
        assert not sync.advance(now_s=now + .05)
        assert client.send("r1", now + .11, "DONE", command_id=commands["r1"])
        assert not sync.advance(now_s=now + .11)
        assert not client.command("r1", permit, now + .11, "after-done")[0]
        assert client.send("r3", now + .12, "DONE", command_id=commands["r3"])
        assert sync.advance(now_s=now + .12)
        assert not client.command("r3", permit, now + .12, "old-stage")[0]
    assert sync.authorize(now_s=6)["phase"] == "FINISH"
    assert len(client.issued) == 10
    assert not client.send("r1", 6)
    assert next(e for e in sync.events if e["event"] == "FINISH")["physical_success_evaluated"] is False
    json.dumps(sync.events, allow_nan=False)


@pytest.mark.parametrize("stage", STAGES)
def test_each_stage_rejects_missing_visual_preconditions(stage):
    sync = TaskStageSync(plan())
    client = Client(sync)
    for i in range(STAGES.index(stage)):
        client.complete(1.0+i)
    now = 7.0
    assert client.send("r1", now)
    assert client.send("r3", now, checks=())
    assert sync.authorize(now_s=now)["phase"] == "HOLD"
    # The peer's readiness from before the negative report cannot be reused.
    assert client.send("r3", now + .01)
    assert sync.authorize(now_s=now + .01)["permission"] is None
    assert client.send("r1", now + .02)
    assert sync.authorize(now_s=now + .02)["permission"] is not None


def test_negative_report_while_already_held_clears_partial_readiness():
    sync = TaskStageSync(plan())
    client = Client(sync)
    sync.hold("reobserve", now_s=1)
    assert client.send("r1", 1.01)
    epoch = sync.epoch
    assert client.send("r3", 1.02, "UNCERTAIN")
    assert sync.epoch == epoch + 1
    assert client.send("r3", 1.03)
    assert sync.authorize(now_s=1.03)["phase"] == "HOLD"
    assert client.send("r1", 1.04)
    assert sync.authorize(now_s=1.04)["phase"] == "GO"


@pytest.mark.parametrize("stage", STAGES)
def test_missing_done_evidence_blocks_transition_and_needs_fresh_pair(stage):
    sync = TaskStageSync(plan())
    client = Client(sync)
    for i in range(STAGES.index(stage)):
        client.complete(1.0+i)
    permission = client.ready(7)
    ids = {r: client.command(r, permission, 7)[1] for r in plan().participants}
    assert client.send("r1", 7.11, "DONE", command_id=ids["r1"])
    assert client.send("r3", 7.12, "DONE", command_id=ids["r3"], checks=("stopped",))
    assert not sync.advance(now_s=7.12)
    assert sync.stage == stage
    assert client.send("r3", 7.13, "DONE", command_id=ids["r3"])
    assert not sync.advance(now_s=7.13)
    assert client.send("r1", 7.14, "DONE", command_id=ids["r1"])
    assert sync.advance(now_s=7.14)


@pytest.mark.parametrize("kind", ["hold", "uncertain", "failed", "low_confidence", "timeout", "plan"])
def test_revocation_requires_new_pair_and_invalidates_old_permission(kind):
    sync = TaskStageSync(plan())
    client = Client(sync)
    permission = client.ready(1)
    now = 1.1
    if kind == "hold":
        sync.hold("test", now_s=now)
    elif kind == "plan":
        sync.update_plan(replace(plan(), plan_version=2), now_s=now)
    elif kind == "timeout":
        now = 1.61
        assert sync.authorize(now_s=now)["phase"] == "HOLD"
    elif kind == "low_confidence":
        assert client.send("r1", now, confidence=.5)
    else:
        assert client.send("r1", now, kind.upper())
    assert not client.command("r1", permission, now)[0]
    assert client.send("r1", now + .01)
    assert sync.authorize(now_s=now + .01)["permission"] is None
    assert client.send("r3", now + .02)
    fresh = sync.authorize(now_s=now + .02)["permission"]
    assert fresh is not None
    assert not client.command("r1", permission, now + .02)[0]
    assert client.command("r1", fresh, now + .02)[0]


@pytest.mark.parametrize("field,value", [("robot_id", "r3"), ("task_id", "other"),
    ("run_id", "old-run"), ("plan_version", 99), ("stage", "RELEASE"), ("epoch", 99),
    ("sent_at_s", 2.0)])
def test_report_identity_context_and_future_rejected(field, value):
    sync = TaskStageSync(plan())
    client = Client(sync)
    assert not sync.receive("r1", client.report(**{field: value}), now_s=1)


def test_duplicate_reordered_stale_and_pre_hold_observations_rejected():
    sync = TaskStageSync(plan())
    client = Client(sync)
    old = client.report(now=1)
    assert sync.receive("r1", old, now_s=1)
    assert not sync.receive("r1", old, now_s=1)
    assert not client.send("r1", 1.01, sequence=0)
    assert not client.send("r1", 1.02, own_rgb_ref=old["own_rgb_ref"], top_rgb_ref=old["top_rgb_ref"])
    sync.hold("pause", now_s=1.2)
    assert not client.send("r1", 1.21, observed_at_s=1.19)
    assert not client.send("r1", 2.0, observed_at_s=1.3)


def test_done_requires_latest_own_command_and_a_later_frame_after_bounded_action():
    sync = TaskStageSync(plan())
    client = Client(sync)
    permit = client.ready(1)
    assert not client.send("r1", 1, "DONE", command_id="invented")
    okay, command = client.command("r1", permit, 1)
    assert okay
    assert not client.send("r1", 1.01, "DONE", command_id=command)
    assert not client.send("r3", 1.11, "DONE", command_id=command)
    assert client.command("r1", permit, 1.11, "next")[0]
    assert not client.send("r1", 1.22, "DONE", command_id=command)
    assert client.send("r1", 1.23, "DONE", command_id="GRASP-r1-next")


def test_observed_motion_is_not_done_and_completion_cannot_be_silently_revoked():
    sync = TaskStageSync(plan())
    client = Client(sync)
    permission = client.ready(1)
    _, command = client.command("r1", permission, 1)
    assert client.send("r1", 1.05, "OBSERVED_MOTION", command_id=command, checks=("motion_observed",))
    assert not sync.advance(now_s=1.05)
    assert client.send("r1", 1.11, "DONE", command_id=command)
    assert client.send("r1", 1.12)
    assert sync.authorize(now_s=1.12)["phase"] == "HOLD"


def test_hold_rechecks_done_without_repeating_nonidempotent_action():
    sync = TaskStageSync(plan())
    client = Client(sync)
    permission = client.ready(1)
    ids = {r: client.command(r, permission, 1)[1] for r in plan().participants}
    sync.hold("uncertain", now_s=1.11)
    for robot, command in ids.items():
        assert client.send(robot, 1.12, "DONE", command_id=command)
    assert sync.advance(now_s=1.12)
    assert len(client.issued) == 2


def test_permission_expiry_tampering_overlap_duplicate_and_restart():
    sync = TaskStageSync(plan())
    client = Client(sync)
    permission = client.ready(1)
    assert not client.command("r1", {**permission, "expires_at_s": 50}, 1)[0]
    assert not client.command("r1", permission, 1, duration=.3)[0]
    assert client.command("r1", permission, 1)[0]
    assert not client.command("r1", permission, 1.01, "overlap")[0]
    assert not client.command("r1", permission, 1.11)[0]
    assert not client.command("r3", permission, 1.2, duration=.1)[0]
    assert not client.command("r3", permission, 1.25, duration=.001)[0]
    other = TaskStageSync(plan())
    other_client = Client(other)
    other_client.ready(1)
    assert not other_client.command("r1", permission, 1)[0]


def test_submit_failure_holds_peer_and_cannot_be_used_as_done():
    sync = TaskStageSync(plan())
    client = Client(sync)
    permission = client.ready(1)
    def broken():
        raise RuntimeError("disconnected")
    with pytest.raises(RuntimeError):
        sync.dispatch("r1", permission, "failure", now_s=1, duration_s=.1, submit=broken)
    assert not client.command("r3", permission, 1)[0]
    assert not client.send("r1", 1.2, "DONE", command_id="failure")


def test_terminal_abort_and_plan_identity_and_clock_restrictions():
    sync = TaskStageSync(plan())
    with pytest.raises(ValueError):
        sync.update_plan(replace(plan(), object_id="different", plan_version=2), now_s=1)
    with pytest.raises(ValueError):
        sync.update_plan(plan(), now_s=1)
    for now in (math.nan, -1, .9, True):
        with pytest.raises(ValueError):
            sync.authorize(now_s=now)
    sync.abort("operator", now_s=1)
    assert sync.authorize(now_s=2)["phase"] == "ABORT"
    assert not Client(sync).send("r1", 2)
    with pytest.raises(ValueError):
        sync.update_plan(replace(plan(), plan_version=2), now_s=2)


def test_delays_are_logged_in_one_domain_and_wall_time_separately():
    sync = TaskStageSync(plan(), clock_domain="sim")
    client = Client(sync)
    report = client.report(now=1, decided_at_s=1.1, sent_at_s=1.2)
    assert sync.receive("r1", report, now_s=1.3)
    event = sync.events[-1]
    assert event["observation_to_decision_s"] == pytest.approx(.1)
    assert event["sender_queue_s"] == pytest.approx(.1)
    assert event["delivery_s"] == pytest.approx(.1)
    assert event["clock_domain"] == "sim"
    assert event["wall_elapsed_s"] >= 0


def test_seeded_report_order_and_revocation_interleavings():
    # Different report order and hold point in every stage: no early transition.
    for seed in range(25):
        rng = random.Random(seed)
        sync = TaskStageSync(plan())
        client = Client(sync)
        for index in range(len(STAGES)):
            now = index + 1.0
            order = list(plan().participants)
            rng.shuffle(order)
            assert client.send(order[0], now)
            assert sync.authorize(now_s=now)["permission"] is None
            assert client.send(order[1], now + .01)
            permission = sync.authorize(now_s=now + .01)["permission"]
            if rng.choice((True, False)):
                sync.hold("injected", now_s=now + .02)
                assert not client.command(order[0], permission, now + .02)[0]
                permission = client.ready(now + .03)
            ids = {r: client.command(r, permission, now + .04)[1] for r in order}
            assert client.send(order[0], now + .15, "DONE", command_id=ids[order[0]])
            assert not sync.advance(now_s=now + .15)
            assert client.send(order[1], now + .16, "DONE", command_id=ids[order[1]])
            assert sync.advance(now_s=now + .16)
        assert sync.authorize(now_s=7)["phase"] == "FINISH"


@pytest.mark.parametrize("case", ["normal", "delayed_ready", "delayed_done", "hold_reobserve",
    "report_gap", "missing_support", "command_only", "old_stage_permission", "plan_change"])
def test_example_ports_exercise_the_public_json_contract(case):
    from scripts.demo_task_stage_sync import run_case
    result = run_case(case, plan())
    assert result["passed"], result["error"]
    assert result["physical_success_evaluated"] is False
    if case == "missing_support":
        assert result["stage"] == "RELEASE"
        assert result["command_count"] == 8
    elif case == "command_only":
        assert result["stage"] == "GRASP"
        assert result["command_count"] == 2
    else:
        assert result["phase"] == "FINISH"
        assert result["command_count"] == 10
