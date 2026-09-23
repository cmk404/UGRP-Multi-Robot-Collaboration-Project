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


def test_actor_and_top_pixels_share_frozen_instant_while_physics_advances(world, monkeypatch):
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
    future = world.render_snapshot_async([("r1", "robot_cam"), (None, "cctv_top")])
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


def test_snapshot_pressure_is_bounded_nonblocking_and_recovers(world, monkeypatch):
    world.render_snapshot_async([(None, "cctv_top")]).result()
    broker = world._snapshot_broker
    original_render = broker.render
    entered, resume = threading.Event(), threading.Event()

    def blocked_render(snapshot):
        entered.set()
        assert resume.wait(5), "test render gate was not released"
        return original_render(snapshot)

    monkeypatch.setattr(broker, "render", blocked_render)
    futures = [world.render_snapshot_async([("r1", "robot_cam")])]
    try:
        assert entered.wait(2)
        futures.extend(world.render_snapshot_async([("r1", "robot_cam")]) for _ in range(2))
        started = time.monotonic()
        with pytest.raises(SnapshotBackpressure):
            world.render_snapshot_async([("r1", "robot_cam")])
        assert time.monotonic() - started < .5
        assert broker.slots.qsize() == 0
        # A queued request can be cancelled without leaking its slot.
        assert futures[1].cancel()
        assert broker.slots.qsize() == 1
        replacement = world.render_snapshot_async([("r1", "robot_cam")])
    finally:
        resume.set()
    assert all(f.result(timeout=5).rgb[("r1", "robot_cam")].shape == (60, 80, 3)
               for f in (futures[0], futures[2], replacement))
    assert broker.slots.qsize() == 3


def test_snapshot_failure_returns_slot_without_corrupting_next_capture(world, monkeypatch):
    world.render_snapshot_async([(None, "cctv_top")]).result()
    broker = world._snapshot_broker
    original_render = broker.render

    def fail(_snapshot):
        raise RuntimeError("render failed")

    monkeypatch.setattr(broker, "render", fail)
    with pytest.raises(RuntimeError, match="render failed"):
        world.render_snapshot_async([(None, "cctv_top")]).result(timeout=5)
    assert broker.slots.qsize() == 3
    monkeypatch.setattr(broker, "render", original_render)
    assert world.render_snapshot_async([(None, "cctv_top")]).result(timeout=5).rgb


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
