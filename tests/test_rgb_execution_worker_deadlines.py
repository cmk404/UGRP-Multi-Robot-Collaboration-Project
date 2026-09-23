"""B3: deterministic image workers/monotonic clock, never physics or rendering."""
from concurrent.futures import Future
import hashlib
import json
from types import SimpleNamespace

import pytest

from test_rgb_execution_skills import Controller, JPEG, build, lifecycle, submit
import harness.rgb_skill_execution as skills


class WallClock:
    def __init__(self):
        self.now = 0.

    def advance(self, seconds):
        self.now += seconds


class ManualPool:
    """Run exact submitted callables only when the test releases them."""
    def __init__(self):
        self.jobs = []

    def submit(self, fn, *args):
        future = Future()
        self.jobs.append((future, fn, args))
        return future

    def run(self, index=0):
        future, fn, args = self.jobs.pop(index)
        if future.set_running_or_notify_cancel():
            try:
                future.set_result(fn(*args))
            except BaseException as error:
                future.set_exception(error)
        return future

    def shutdown(self, *, wait, cancel_futures):
        assert wait is False
        if cancel_futures:
            for future, _, _ in self.jobs:
                future.cancel()

    def run_robot(self, rid):
        return self.run(next(i for i, (_, _, args) in enumerate(self.jobs) if args[1]["robot_id"] == rid))


@pytest.fixture
def rig(monkeypatch):
    clock, pool, rows = WallClock(), ManualPool(), []
    monkeypatch.setattr(skills, "time", SimpleNamespace(monotonic=lambda: clock.now))
    class TimedController(Controller):
        compute_s = .01

        def decide(self, own, top):
            clock.advance(self.compute_s)
            return super().decide(own, top)
    controllers = {rid: TimedController() for rid in ("r1", "r2", "r3")}
    port, world, _, _ = build(controllers)
    port._pool.shutdown(wait=False, cancel_futures=True)
    port._pool, port._record_supervisor = pool, rows.append
    yield SimpleNamespace(port=port, world=world, controllers=controllers, clock=clock, pool=pool, rows=rows)
    port.close(None)


def activate_pair_then_solo(rig):
    for rid, role in (("r1", "lower"), ("r3", "upper")):
        submit(rig.port, rid, skill="pair_transport_A", task="pair", participants=["r1", "r3"], role=role)
    submit(rig.port, "r2")


@pytest.mark.parametrize("case", ["fast_completion_late_poll", "unfinished_late_poll", "late_completion", "cancel_after_completion"])
def test_existing_total_wall_guard_is_not_redefined_as_completion_deadline(rig, case):
    """All four old-policy rejections stay rejections, not an accepted fallback."""
    submit(rig.port, "r2")
    rig.port.tick(0.)
    if case == "late_completion":
        rig.controllers["r2"].compute_s = 2.01
    if case != "unfinished_late_poll":
        rig.pool.run()
    if case == "cancel_after_completion":
        assert lifecycle(rig.port, "r2", "interrupt")["status"] == "ACCEPTED"
    else:
        rig.clock.now = 2.02
    rig.port.tick(.05)
    assert not any(any(command) for command in rig.world.robots["r2"].motor_calls)
    assert rig.port.local_status("r2")["active"] == []
    if case != "cancel_after_completion":
        rejection = next(row for row in rig.rows if row["event"] == "RGB_WORKER_REJECTED")
        assert rejection["wall_cap_exceeded"] and not rejection["sim_age_exceeded"]
        assert rejection["wall_cap_s"] == 2. and rejection["sim_age_cap_s"] == 1.
    else:
        revoked = next(row for row in rig.rows if row["event"] == "RGB_WORKER_REVOKED")
        assert revoked["worker_completed_wall_s"] <= revoked["revoked_wall_s"]
        assert revoked["future_done"] and not revoked["future_cancelled"]
        assert not any(row["event"] == "RGB_WORKER_CONSUMED" for row in rig.rows)


def test_completed_solo_is_polled_before_new_pair_input_blocks_owner(rig):
    """New capture must not needlessly age an already-ready unrelated result."""
    rig.controllers["r1"].action = rig.controllers["r3"].action = {"kind": "wait", "duration": .001}
    activate_pair_then_solo(rig)
    rig.port.tick(0.)
    rig.pool.run_robot("r1")
    rig.pool.run_robot("r3")  # solo is still pending
    rig.port.tick(0.)  # consume pair, leave solo pending
    rig.pool.run()  # solo complete well inside the unchanged wall budget
    def slow_new_pair_frame(rid, now):
        if rid in {"r1", "r3"}:
            rig.clock.advance(2.1)
        return {"own_rgb": JPEG, "top_rgb": JPEG}
    rig.port._frame_source = slow_new_pair_frame
    rig.port.tick(.05)
    assert rig.port.local_status("r2")["own_skill_status"]["state"] == "RUNNING"
    assert any(any(command) for command in rig.world.robots["r2"].motor_calls)


def test_prepare_pair_images_before_starting_either_submission_clock(rig):
    for rid, role in (("r1", "lower"), ("r3", "upper")):
        submit(rig.port, rid, skill="pair_transport_A", task="pair", participants=["r1", "r3"], role=role)
    def slow_frame(rid, now):
        rig.clock.advance(2.1)
        return {"own_rgb": JPEG, "top_rgb": JPEG}
    rig.port._frame_source = slow_frame
    rig.port.tick(0.)
    first, second = (rig.port._runners[rid] for rid in ("r1", "r3"))
    assert first.started_wall == second.started_wall == rig.clock.now
    rig.pool.run()
    rig.pool.run()
    rig.port.tick(0.)
    assert rig.port.local_status("r1")["own_skill_status"]["state"] == "RUNNING"


def test_new_controller_preparation_does_not_block_ready_unrelated_worker(rig):
    submit(rig.port, "r2")
    rig.port.tick(0.)
    rig.pool.run()
    for rid, role in (("r1", "lower"), ("r3", "upper")):
        submit(rig.port, rid, skill="pair_transport_A", task="pair", participants=["r1", "r3"], role=role)
    def slow_factory(rid, role, skill):
        rig.clock.advance(2.1)
        return rig.controllers[rid]
    rig.port._skill_factory = slow_factory
    rig.port.tick(.05)
    assert rig.port.local_status("r2")["own_skill_status"]["state"] == "RUNNING"


def test_input_capture_and_submission_use_fixed_robot_order(rig):
    activate_pair_then_solo(rig)
    captured = []
    def frames(rid, now):
        captured.append(rid)
        return {"own_rgb": JPEG, "top_rgb": JPEG}
    rig.port._frame_source = frames
    rig.port.tick(0.)
    assert captured == ["r1", "r2", "r3"]
    assert [args[1]["robot_id"] for _, _, args in rig.pool.jobs] == captured
    assert len(rig.port._all_futures) == 3


def test_failed_pair_preparation_discards_staged_partner_but_not_unrelated_task(rig):
    activate_pair_then_solo(rig)
    def frames(rid, now):
        if rid == "r3":
            raise ValueError("offline broken frame")
        return {"own_rgb": JPEG, "top_rgb": JPEG}
    rig.port._frame_source = frames
    rig.port.tick(0.)
    assert [args[1]["robot_id"] for _, _, args in rig.pool.jobs] == ["r2"]
    assert rig.port.local_status("r1")["active"] == []
    rig.pool.run()
    rig.port.tick(.05)
    assert rig.port.local_status("r2")["own_skill_status"]["state"] == "RUNNING"


def test_revoked_running_workers_keep_capacity_until_really_done(rig):
    activate_pair_then_solo(rig)
    rig.port.tick(0.)
    running = [future for future, _, _ in rig.pool.jobs]
    for future in running:
        assert future.set_running_or_notify_cancel()
    lifecycle(rig.port, "r1", "interrupt", task="pair")
    lifecycle(rig.port, "r2", "interrupt")
    revoked = [row for row in rig.rows if row["event"] == "RGB_WORKER_REVOKED"]
    assert len(revoked) == 3
    assert all(not row["future_cancelled"] and not row["future_done"] for row in revoked)
    submit(rig.port, "r2", task="replacement")
    rig.port.tick(.05)
    assert len(rig.pool.jobs) == 3  # no fourth queued worker behind revoked work
    assert rig.port.local_status("r2")["active"] == []
    for future in running:
        future.set_result({"action": {"kind": "drive"}})  # late result has no motion authority
    rig.port.tick(.1)
    assert not any(any(command) for robot in rig.world.robots.values() for command in robot.motor_calls)


def test_apply_and_hold_invalidate_same_time_frame_bytes(rig):
    endpoint, calls = rig.port._endpoints["r2"], []
    endpoint.invalidate_frames = lambda: calls.append("invalidated")
    endpoint.apply_bounded({"kind": "drive", "forward": .1, "turn": 0., "duration_s": .1}, 0., .1)
    endpoint.hold(0.)
    assert calls == ["invalidated", "invalidated"]


def test_completion_timestamp_cannot_be_overwritten():
    timing = skills._WorkerTiming()
    timing.start(1.)
    timing.complete(1.25)
    with pytest.raises(ValueError):
        timing.complete(1.1)
    with pytest.raises(ValueError):
        timing.start(0.)
    assert timing.snapshot(0.)["worker_completed_wall_s"] == 1.25


@pytest.mark.parametrize("reason", ["revision", "cancel", "pause", "expiry", "missing_timing", "stale_sim"])
def test_prepared_result_cannot_bypass_consumption_gates(rig, reason):
    submit(rig.port, "r2", expires_at_s=.05 if reason == "expiry" else 10.)
    rig.port.tick(0.)
    rig.pool.run()
    if reason == "revision":
        rig.port._own_revision["r2"] += 1
    elif reason == "cancel":
        lifecycle(rig.port, "r2", "interrupt")
    elif reason == "pause":
        lifecycle(rig.port, "r2", "pause")
    elif reason == "missing_timing":
        rig.port._runners["r2"].timing = None
    elif reason == "stale_sim":
        rig.port._runners["r2"].observation["observed_at_s"] = -1.
    rig.port.tick(.05)
    assert not any(any(command) for command in rig.world.robots["r2"].motor_calls)


def test_queue_compute_and_owner_poll_are_separate_but_total_cap_remains(rig):
    submit(rig.port, "r2")
    rig.port.tick(0.)
    rig.clock.advance(.4)
    rig.pool.run()
    rig.clock.advance(.2)
    rig.port.tick(.05)
    row = next(row for row in rig.rows if row["event"] == "RGB_WORKER_CONSUMED")
    assert row["worker_queue_wall_s"] == .4
    assert row["worker_wall_duration_s"] == pytest.approx(.01)
    assert row["completion_to_poll_wall_s"] == pytest.approx(.2)
    assert row["submission_to_poll_wall_s"] == pytest.approx(.61)
    assert row["wall_scope"] == "submission_to_consumption"
    assert "completion_to_poll" not in json.dumps(rig.port.local_status("r2"))


def test_frame_cache_preserves_original_bytes_hash_times_and_robot_identity(monkeypatch):
    clock, calls, rows = WallClock(), [], []
    monkeypatch.setattr(skills, "time", SimpleNamespace(monotonic=lambda: clock.now))
    def render(rid):
        calls.append(rid)
        clock.advance(.1)
        return b"\xff\xd8"+rid.encode()+b"\xff\xd9"
    cache = skills._RGBFrameCache(render, lambda: render("top"), record=rows.append)
    args = dict(sim_time_s=.5, absolute_time_s=1.8, episode="episode1",
                camera_identity=("robot_cam", "cctv_top", 960, 720, "calibrationA"))
    first = cache.read("r1", observation_id="r1-1", **args)
    clock.advance(.6)
    again = cache.read("r1", observation_id="r1-2", **args)
    peer = cache.read("r2", observation_id="r2-1", **args)
    assert calls == ["top", "r1", "r2"]
    assert first == again and peer["own_rgb"] != first["own_rgb"]
    assert rows[1]["captures"] == rows[0]["captures"]
    assert rows[1]["captures"]["own_rgb"]["sha256"] == hashlib.sha256(first["own_rgb"]).hexdigest()
    assert rows[1]["read_wall_s"] > rows[1]["captures"]["own_rgb"]["completed_wall_s"]
    assert rows[1]["captures"]["own_rgb"]["sim_time_s"] == .5
    assert rows[1]["cache_hit"] == {"own_rgb": True, "top_rgb": True}


@pytest.mark.parametrize("change", ["sim_step", "episode_reset", "camera", "resolution", "calibration", "same_time_actuation"])
def test_frame_cache_invalidates_every_frame_producing_identity_change(change):
    calls = []
    def render(rid):
        calls.append(rid)
        return JPEG
    cache = skills._RGBFrameCache(render, lambda: render("top"), record=lambda row: None)
    args = dict(sim_time_s=.5, absolute_time_s=1.8, episode="one", observation_id="one",
                camera_identity=("robot_cam", 960, 720, "calibrationA"))
    cache.read("r1", **args)
    if change == "sim_step":
        args.update(sim_time_s=.55, absolute_time_s=1.85)
    elif change == "episode_reset":
        cache.invalidate()
        args.update(sim_time_s=0., absolute_time_s=0., episode="two")
    elif change == "camera":
        args["camera_identity"] = ("other_cam", 960, 720, "calibrationA")
    elif change == "resolution":
        args["camera_identity"] = ("robot_cam", 1280, 720, "calibrationA")
    elif change == "calibration":
        args["camera_identity"] = ("robot_cam", 960, 720, "calibrationB")
    else:
        cache.invalidate()
    cache.read("r1", **args)
    assert calls == ["top", "r1", "top", "r1"]


def test_frame_cache_clock_reversal_fails_without_reusing_old_image():
    rows = []
    cache = skills._RGBFrameCache(lambda rid: JPEG, lambda: JPEG, record=rows.append)
    args = dict(episode="same", camera_identity=("fixed",), observation_id="one")
    cache.read("r1", sim_time_s=1., absolute_time_s=2., **args)
    with pytest.raises(ValueError, match="backwards"):
        cache.read("r1", sim_time_s=.9, absolute_time_s=1.9, **args)
    assert len(rows) == 1 and not cache._frames
