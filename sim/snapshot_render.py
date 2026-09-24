"""Bounded, coherent MuJoCo RGB snapshots rendered on one GL owner thread.

Only capture touches the live world, under its physics lock.  MuJoCo forward,
OpenGL rendering, readback and fisheye remapping use a separate model/data copy.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass
import queue
import threading
import time
from typing import Sequence

import mujoco
import numpy as np

from sim.masterpi_camera_profile import CAMERA_LOCAL_POS_M, CAMERA_LOCAL_QUAT_WXYZ
from sim.snapshot_contract import SnapshotBackpressure

CameraKey = tuple[str | None, str]


@dataclass(frozen=True)
class RenderBatch:
    frame_id: int
    sim_time: float
    rgb: dict[CameraKey, np.ndarray]
    render_completed_wall_s: float | None = None


@dataclass(frozen=True)
class RenderedTop:
    frame_id: int
    sim_time: float
    rgb: np.ndarray
    render_completed_wall_s: float


class TopRenderLatch:
    """A callback-free signal; GL never runs image encoding or consumer code."""
    def __init__(self):
        self._event = threading.Event()
        self._value: RenderedTop | None = None
        self._error: BaseException | None = None
        self._lock = threading.Lock()

    def publish(self, value: RenderedTop) -> None:
        with self._lock:
            if self._event.is_set():
                return
            self._value = value
            self._event.set()

    def fail(self, error: BaseException) -> None:
        with self._lock:
            if self._event.is_set():
                return
            self._error = error
            self._event.set()

    def done(self) -> bool:
        return self._event.is_set()

    def result(self, timeout: float | None = None) -> RenderedTop:
        if not self._event.wait(timeout):
            raise TimeoutError('TOP render did not complete')
        if self._error is not None:
            raise self._error
        assert self._value is not None
        return self._value


@dataclass(frozen=True)
class _Request:
    key: CameraKey
    actual_camera: str
    actor: bool
    option: mujoco.MjvOption | None
    fisheye_map: tuple[np.ndarray, np.ndarray] | None


@dataclass(frozen=True)
class FrozenRenderSnapshot:
    frame_id: int
    sim_time: float
    data: mujoco.MjData
    model_arrays: dict[str, np.ndarray]
    requests: tuple[_Request, ...]
    needs_forward: bool


def _front_follow_pose(data: mujoco.MjData, body_id: int) -> tuple[np.ndarray, np.ndarray]:
    """Match the existing presentation-only front camera without live writes."""
    robot_pos = np.asarray(data.xpos[body_id], dtype=float)
    robot_rot = np.asarray(data.xmat[body_id], dtype=float).reshape(3, 3)
    forward = np.asarray(robot_rot[:, 0], dtype=float)
    forward[2] = 0.0
    norm = float(np.linalg.norm(forward))
    forward = np.asarray([1.0, 0.0, 0.0]) if norm < 1e-9 else forward / norm
    camera_pos = robot_pos + 0.90 * forward + np.asarray([0.0, 0.0, 0.38])
    target = robot_pos + np.asarray([0.0, 0.0, 0.13])
    view = target - camera_pos
    view /= max(1e-9, float(np.linalg.norm(view)))
    right = np.cross(view, [0.0, 0.0, 1.0])
    right /= max(1e-9, float(np.linalg.norm(right)))
    image_up = np.cross(right, view)
    quat = np.zeros(4, dtype=float)
    mujoco.mju_mat2Quat(quat, np.column_stack((right, image_up, -view)).reshape(-1))
    return camera_pos, quat


class SnapshotRenderBroker:
    """Own three reusable frozen-data slots and separate GL renderers."""

    def __init__(self, world):
        self.owner_thread_id = threading.get_ident()
        self.model = copy.copy(world.model)
        self.data = mujoco.MjData(self.model)
        self.slots: queue.Queue[mujoco.MjData] = queue.Queue(maxsize=3)
        for _ in range(3):
            self.slots.put_nowait(mujoco.MjData(world.model))
        self.renderer = mujoco.Renderer(self.model, height=world.height, width=world.width)
        try:
            self.observer = mujoco.Renderer(
                self.model, height=world.observer_height, width=world.observer_width)
        except Exception:
            self.renderer.close()
            raise

    def acquire(self) -> mujoco.MjData:
        try:
            return self.slots.get_nowait()
        except queue.Empty as exc:
            raise SnapshotBackpressure("three render snapshots already in flight") from exc

    def release(self, snapshot: FrozenRenderSnapshot) -> None:
        self.slots.put_nowait(snapshot.data)

    def capture(self, world, cameras: Sequence[CameraKey], slot: mujoco.MjData) -> FrozenRenderSnapshot:
        if not cameras:
            raise ValueError("render snapshot needs at least one camera")
        if len(cameras) > 8 or len(set(cameras)) != len(cameras):
            raise ValueError("render snapshot cameras must be unique and at most eight")
        with world.physics_lock:
            requests = []
            needs_forward = False
            arrays = {
                name: getattr(world.model, name).copy()
                for name in dir(world.model) if name.startswith("cam_")
                and isinstance(getattr(world.model, name), np.ndarray)
            }
            for name in ("geom_group", "geom_rgba", "mat_rgba"):
                arrays[name] = getattr(world.model, name).copy()
            for robot_id, camera in cameras:
                if not isinstance(camera, str) or not camera:
                    raise ValueError("camera name must be nonempty")
                robot = world.robot(robot_id) if robot_id is not None else None
                if camera in {"robot_cam", "nav_cam"} and robot is None:
                    raise ValueError(f"{camera} requires a robot id")
                if camera == "nav_cam" and world.warehouse_layout != "camera_team":
                    raise ValueError("nav_cam is available only in camera_team layout")
                actor = camera in {"robot_cam", "nav_cam"}
                actual = robot._n(camera) if actor else camera
                if mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_CAMERA, actual) < 0:
                    raise ValueError(f"unknown camera: {actual}")
                if actor:
                    option = copy.copy(robot._robot_sensor_scene_option)
                elif robot is None:
                    # Legacy team RGB includes all geom groups, unlike the
                    # default MjvOption created by Renderer.update_scene.
                    option = mujoco.MjvOption()
                    option.geomgroup[:] = 1
                else:
                    option = None
                fisheye = robot._robot_fisheye_map if camera == "robot_cam" else None
                if camera == "robot_cam":
                    needs_forward |= (not np.array_equal(arrays["cam_pos"][robot.robot_cam_cid], CAMERA_LOCAL_POS_M)
                                      or not np.array_equal(arrays["cam_quat"][robot.robot_cam_cid], CAMERA_LOCAL_QUAT_WXYZ))
                    arrays["cam_pos"][robot.robot_cam_cid] = CAMERA_LOCAL_POS_M
                    arrays["cam_quat"][robot.robot_cam_cid] = CAMERA_LOCAL_QUAT_WXYZ
                elif camera == "cctv_front_left" and robot is not None:
                    pos, quat = _front_follow_pose(world.data, robot.robot_bid)
                    needs_forward |= (not np.array_equal(arrays["cam_pos"][robot.front_cam_cid], pos)
                                      or not np.array_equal(arrays["cam_quat"][robot.front_cam_cid], quat))
                    arrays["cam_pos"][robot.front_cam_cid] = pos
                    arrays["cam_quat"][robot.front_cam_cid] = quat
                requests.append(_Request((robot_id, camera), actual, actor, option, fisheye))
            mujoco.mj_copyData(slot, world.model, world.data)
            world._snapshot_frame_id += 1
            return FrozenRenderSnapshot(world._snapshot_frame_id, float(world.data.time),
                                        slot, arrays, tuple(requests), needs_forward)

    def render(self, snapshot: FrozenRenderSnapshot, *,
               top_latch: TopRenderLatch | None = None) -> RenderBatch:
        if threading.get_ident() != self.owner_thread_id:
            raise RuntimeError("MuJoCo snapshot renderer accessed outside its owner thread")
        for name, source in snapshot.model_arrays.items():
            np.copyto(getattr(self.model, name), source)
        mujoco.mj_copyData(self.data, self.model, snapshot.data)
        if snapshot.needs_forward:
            mujoco.mj_forward(self.model, self.data)
        images = {}
        for req in snapshot.requests:
            renderer = self.renderer if req.actor else self.observer
            renderer.update_scene(self.data, camera=req.actual_camera, scene_option=req.option)
            rgb = renderer.render().copy()
            if req.fisheye_map is not None:
                import cv2
                rgb = cv2.remap(rgb, *req.fisheye_map, cv2.INTER_LINEAR,
                                borderMode=cv2.BORDER_CONSTANT)
            images[req.key] = rgb
            if top_latch is not None and req.key == (None, 'cctv_top'):
                top_latch.publish(RenderedTop(snapshot.frame_id, snapshot.sim_time,
                                              rgb, time.monotonic()))
        return RenderBatch(snapshot.frame_id, snapshot.sim_time, images,
                           time.monotonic())

    def close(self) -> None:
        if threading.get_ident() != self.owner_thread_id:
            raise RuntimeError("MuJoCo snapshot renderer closed outside its owner thread")
        try:
            self.renderer.close()
        finally:
            self.observer.close()
