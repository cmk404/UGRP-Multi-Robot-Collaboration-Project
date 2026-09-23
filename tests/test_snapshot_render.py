"""Frozen RGB capture must not hold or read the advancing physics world."""
from __future__ import annotations

import threading
import time

import mujoco
import numpy as np
import pytest

from sim.multi_masterpi_production import MultiMasterPiProductionV2
from sim.snapshot_render import SnapshotBackpressure


@pytest.fixture
def world():
    scene = MultiMasterPiProductionV2(
        seed=11, width=80, height=60, render=True, use_calibration_manifest=False)
    try:
        yield scene
    finally:
        scene.close()


@pytest.mark.parametrize('render_ready', [False, True])
def test_actor_and_top_pixels_share_frozen_instant_while_physics_advances(world, monkeypatch, render_ready):
    own = world.render_rgb(robot_id="r1", camera="robot_cam")
    top = world._render_executor.submit(world._render_team_rgb_direct, "cctv_top").result()
    before_time = float(world.data.time)
    before_qpos = world.data.qpos.copy()
    before_cam_fovy = world.model.cam_fovy.copy()

    # Initialize the opt-in broker before blocking its actual render operation.
    warm = world.render_snapshot_async([("r1", "robot_cam"), (None, "cctv_top")]).result()
    assert np.array_equal(warm.rgb[("r1", "robot_cam")], own)
    assert np.array_equal(warm.rgb[(None, "cctv_top")], top)
    assert float(world.data.time) == before_time
    assert np.array_equal(world.data.qpos, before_qpos)
    assert np.array_equal(world.model.cam_fovy, before_cam_fovy)

    broker = world._snapshot_broker
    original_render = broker.render
    entered, resume = threading.Event(), threading.Event()

    def blocked_render(snapshot):
        entered.set()
        assert resume.wait(5), "test render gate was not released"
        return original_render(snapshot)

    monkeypatch.setattr(broker, "render", blocked_render)
    future = world.render_snapshot_async([("r1", "robot_cam"), (None, "cctv_top")],
                                         capture_on_render_ready=render_ready)
    try:
        assert entered.wait(2)
        assert world.physics_lock.acquire(timeout=.5), "render retained physics_lock"
        try:
            # Changing live physics and visual options after capture must not
            # change either image in this already-frozen batch.
            mujoco.mj_step(world.model, world.data)
            world.model.cam_fovy[world.robot("r1").robot_cam_cid] *= 1.5
            world.robot("r1")._robot_sensor_scene_option.geomgroup[0] = 0
        finally:
            world.physics_lock.release()
    finally:
        resume.set()
    batch = future.result(timeout=5)
    assert batch.frame_id == warm.frame_id + 1
    assert batch.sim_time == before_time
    assert float(world.data.time) > batch.sim_time
    assert np.array_equal(batch.rgb[("r1", "robot_cam")], own)
    assert np.array_equal(batch.rgb[(None, "cctv_top")], top)


@pytest.mark.parametrize('render_ready', [False, True])
def test_snapshot_pressure_is_bounded_nonblocking_and_recovers(world, monkeypatch, render_ready):
    world.render_snapshot_async([(None, "cctv_top")]).result()
    broker = world._snapshot_broker
    original_render = broker.render
    entered, resume = threading.Event(), threading.Event()

    def blocked_render(snapshot):
        entered.set()
        assert resume.wait(5), "test render gate was not released"
        return original_render(snapshot)

    monkeypatch.setattr(broker, "render", blocked_render)
    def capture():
        return world.render_snapshot_async([("r1", "robot_cam")],
                                            capture_on_render_ready=render_ready)
    futures = [capture()]
    try:
        assert entered.wait(2)
        futures.extend(capture() for _ in range(2))
        started = time.monotonic()
        with pytest.raises(SnapshotBackpressure):
            capture()
        assert time.monotonic() - started < .5
        assert broker.slots.qsize() == 0
        # A queued request can be cancelled without leaking its slot.
        assert futures[1].cancel()
        assert broker.slots.qsize() == 1
        replacement = capture()
    finally:
        resume.set()
    assert all(f.result(timeout=5).rgb[("r1", "robot_cam")].shape == (60, 80, 3)
               for f in (futures[0], futures[2], replacement))
    assert broker.slots.qsize() == 3


@pytest.mark.parametrize('render_ready', [False, True])
def test_snapshot_failure_returns_slot_without_corrupting_next_capture(world, monkeypatch, render_ready):
    world.render_snapshot_async([(None, "cctv_top")]).result()
    broker = world._snapshot_broker
    original_render = broker.render

    def fail(_snapshot):
        raise RuntimeError("render failed")

    monkeypatch.setattr(broker, "render", fail)
    with pytest.raises(RuntimeError, match="render failed"):
        world.render_snapshot_async([(None, "cctv_top")],
                                    capture_on_render_ready=render_ready).result(timeout=5)
    assert broker.slots.qsize() == 3
    monkeypatch.setattr(broker, "render", original_render)
    assert world.render_snapshot_async([(None, "cctv_top")]).result(timeout=5).rgb


def test_render_ready_captures_after_queue_wait_with_true_timestamp(world):
    world.render_snapshot_async([(None, 'cctv_top')]).result(timeout=5)
    entered, resume = threading.Event(), threading.Event()
    def queue_gate():
        entered.set()
        assert resume.wait(5)
    gate = world._render_executor.submit(queue_gate)
    assert entered.wait(2)
    before = float(world.data.time)
    try:
        # A recording still freezes the requested instant. The actor request
        # reserves capacity, but does not freeze pixels behind the queue gate.
        immediate = world.render_snapshot_async([(None, 'cctv_top')])
        cameras = [(None, 'cctv_top')]
        fresh = world.render_snapshot_async(cameras, capture_on_render_ready=True)
        cameras.clear()  # callers cannot mutate a queued camera selection
        with world.physics_lock:
            for _ in range(4):
                mujoco.mj_step(world.model, world.data)
        captured_at = float(world.data.time)
        assert captured_at > before
    finally:
        resume.set()
    gate.result(timeout=5)
    old_batch, new_batch = immediate.result(timeout=5), fresh.result(timeout=5)
    assert old_batch.sim_time == before
    assert new_batch.sim_time == captured_at
    assert new_batch.frame_id > old_batch.frame_id
    expected = world.render_snapshot_async([(None, 'cctv_top')]).result(timeout=5)
    assert np.array_equal(new_batch.rgb[(None, 'cctv_top')], expected.rgb[(None, 'cctv_top')])
    assert world._snapshot_broker.slots.qsize() == 3


def test_failed_deferred_capture_returns_reserved_slot(world):
    world.render_snapshot_async([(None, 'cctv_top')]).result(timeout=5)
    with pytest.raises(ValueError, match='unknown camera'):
        world.render_snapshot_async([(None, 'missing_camera')],
                                    capture_on_render_ready=True).result(timeout=5)
    assert world._snapshot_broker.slots.qsize() == 3
    assert world.render_snapshot_async([(None, 'cctv_top')],
                                       capture_on_render_ready=True).result(timeout=5).rgb


def test_deferred_copy_waits_for_physics_lock_and_close_drains_it(world, monkeypatch):
    world.render_snapshot_async([(None, 'cctv_top')]).result(timeout=5)
    broker = world._snapshot_broker
    entered = threading.Event()
    original_capture = broker.capture
    def capture(*args):
        entered.set()
        return original_capture(*args)
    monkeypatch.setattr(broker, 'capture', capture)
    with world.physics_lock:
        future = world.render_snapshot_async([(None, 'cctv_top')],
                                             capture_on_render_ready=True)
        assert entered.wait(2)
        assert not future.done()
        mujoco.mj_step(world.model, world.data)
        expected_time = float(world.data.time)
    world.close()
    assert future.result(timeout=5).sim_time == expected_time
    assert broker.slots.qsize() == 3
    with pytest.raises(RuntimeError):
        world.render_snapshot_async([(None, 'cctv_top')], capture_on_render_ready=True)


def test_navigation_camera_uses_same_actor_pixels_on_camera_team_map():
    world = MultiMasterPiProductionV2(
        seed=41, width=80, height=60, render=True,
        use_calibration_manifest=False, warehouse_layout="camera_team")
    try:
        expected = world.robot("r1").render_rgb("nav_cam")
        fovy = world.model.cam_fovy.copy()
        got = world.render_snapshot_async([("r1", "nav_cam")]).result(timeout=5)
        assert np.array_equal(got.rgb[("r1", "nav_cam")], expected)
        assert np.array_equal(world.model.cam_fovy, fovy)
    finally:
        world.close()
