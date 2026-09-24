"""Frozen RGB capture must not hold or read the advancing physics world."""
from __future__ import annotations

import threading
import time
from dataclasses import replace
from concurrent.futures import CancelledError

import mujoco
import numpy as np
import pytest

from sim.multi_masterpi_production import MultiMasterPiProductionV2
from sim.snapshot_render import SnapshotBackpressure


PAIR_CAMERAS = [(None, 'cctv_top'), ('r1', 'robot_cam'), ('r3', 'robot_cam')]


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


def test_partial_top_precedes_own_from_same_frozen_slot_and_pixels(world, monkeypatch):
    baseline = world.render_snapshot_async(PAIR_CAMERAS,
        capture_on_render_ready=True).result(timeout=5)
    broker = world._snapshot_broker
    from sim.snapshot_render import TopRenderLatch
    original_publish = TopRenderLatch.publish
    top_signaled, resume = threading.Event(), threading.Event()

    def pause_after_top(self, value):
        original_publish(self, value)
        top_signaled.set()
        assert resume.wait(5), 'test TOP gate was not released'

    monkeypatch.setattr(TopRenderLatch, 'publish', pause_after_top)
    latch, full = world.render_pair_snapshot_with_top_async(PAIR_CAMERAS)
    try:
        assert top_signaled.wait(2)
        top = latch.result(timeout=1)
        assert not full.done()
        assert broker.slots.qsize() == 2  # partial release is forbidden
        assert world.physics_lock.acquire(timeout=.5)
        try:
            mujoco.mj_step(world.model, world.data)
        finally:
            world.physics_lock.release()
    finally:
        resume.set()
    batch = full.result(timeout=5)
    assert batch.frame_id == top.frame_id == baseline.frame_id + 1
    assert batch.sim_time == top.sim_time == baseline.sim_time
    assert top.rgb is batch.rgb[(None, 'cctv_top')]
    assert top.render_completed_wall_s < batch.render_completed_wall_s
    assert all(np.array_equal(batch.rgb[key], baseline.rgb[key]) for key in PAIR_CAMERAS)
    assert broker.slots.qsize() == 3


def test_queued_partial_cancel_wakes_top_and_returns_only_its_slot(world):
    world.render_snapshot_async([(None, 'cctv_top')]).result(timeout=5)
    broker = world._snapshot_broker
    entered, resume = threading.Event(), threading.Event()
    def queue_gate():
        entered.set()
        assert resume.wait(5)
    gate = world._render_executor.submit(queue_gate)
    assert entered.wait(2)
    try:
        latch, full = world.render_pair_snapshot_with_top_async(PAIR_CAMERAS)
        assert broker.slots.qsize() == 2
        assert full.cancel()
        with pytest.raises(CancelledError):
            latch.result(timeout=1)
        assert broker.slots.qsize() == 3
    finally:
        resume.set()
    gate.result(timeout=5)


def test_partial_top_success_then_own_failure_aborts_and_releases_slot(world, monkeypatch):
    world.render_snapshot_async([(None, 'cctv_top')]).result(timeout=5)
    broker = world._snapshot_broker
    original_render = broker.render
    def fail_after_top(snapshot, *, top_latch=None):
        original_render(replace(snapshot, requests=snapshot.requests[:1]),
                        top_latch=top_latch)
        raise RuntimeError('own render failed')
    monkeypatch.setattr(broker, 'render', fail_after_top)
    latch, full = world.render_pair_snapshot_with_top_async(PAIR_CAMERAS)
    assert latch.result(timeout=5).frame_id > 0
    with pytest.raises(RuntimeError, match='own render failed'):
        full.result(timeout=5)
    assert broker.slots.qsize() == 3
    monkeypatch.setattr(broker, 'render', original_render)
    assert world.render_snapshot_async(PAIR_CAMERAS).result(timeout=5).rgb


def test_partial_capture_failure_before_top_wakes_latch_and_releases_slot(world, monkeypatch):
    world.render_snapshot_async([(None, 'cctv_top')]).result(timeout=5)
    broker = world._snapshot_broker
    monkeypatch.setattr(broker, 'capture', lambda *_args: (_ for _ in ()).throw(
        RuntimeError('copy failed')))
    latch, full = world.render_pair_snapshot_with_top_async(PAIR_CAMERAS)
    with pytest.raises(RuntimeError, match='copy failed'):
        latch.result(timeout=5)
    with pytest.raises(RuntimeError, match='copy failed'):
        full.result(timeout=5)
    assert broker.slots.qsize() == 3
