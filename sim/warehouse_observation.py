"""Actor visible warehouse observations.

This module is deliberately a sensor boundary.  It consumes only a robot's
camera frame (and an optional depth frame supplied by the renderer) and never
reads ``warehouse_state`` or any cargo/peer pose from the simulator.  The
adapter is usable with the production render broker through ``render_rgb`` and
with an RGB-D renderer by supplying ``render_depth`` or ``render_sensor_frame``.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
import threading
from typing import Any, Callable, Mapping

import numpy as np
import cv2


@dataclass(frozen=True)
class WarehouseSensorConfig:
    """Explicit limits for the simulated RGB-D sensor."""

    width: int = 640
    height: int = 480
    fx_px: float = 520.0
    fy_px: float = 520.0
    cx_px: float | None = None
    cy_px: float | None = None
    min_depth_m: float = 0.08
    max_depth_m: float = 4.0
    min_pixels: int = 6
    color_tolerance: float = 0.24
    frame_source: str = "mujoco:robot_cam:rgbd"
    sensor_seed: int = 0
    depth_noise_std_m: float = 0.002

    def principal_point(self, shape: tuple[int, int]) -> tuple[float, float]:
        height, width = shape
        return (float(self.cx_px if self.cx_px is not None else (width - 1) / 2),
                float(self.cy_px if self.cy_px is not None else (height - 1) / 2))


# The labels are intentionally the visible warehouse markers.  They identify
# cargo type without using geom or body ids: red label=oak, blue band=pipe,
# yellow label=crate.  Values are RGB and tolerate MuJoCo lighting/shading.
_CARGO_TAG_IDS = {11: "oak_plank_01", 12: "steel_pipe_01", 13: "wood_crate_01", 14: "small_box_01", 15: "small_box_02"}


def camera_intrinsics_from_vertical_fovy(width: int, height: int, fovy_deg: float) -> dict[str, float]:
    """Calibrate a pinhole image using MuJoCo's vertical camera FOV."""
    focal = float(height) / (2.0 * math.tan(math.radians(float(fovy_deg)) / 2.0))
    return {"fx_px": focal, "fy_px": focal, "cx_px": (int(width) - 1) / 2.0,
            "cy_px": (int(height) - 1) / 2.0}


def _as_rgb(frame: Any) -> np.ndarray:
    image = np.asarray(frame)
    if image.ndim != 3 or image.shape[2] < 3:
        raise ValueError("RGB sensor frame must have shape (height, width, 3)")
    image = image[..., :3]
    if image.dtype.kind in "ui":
        image = image.astype(np.float32) / 255.0
    else:
        image = image.astype(np.float32)
        if float(np.nanmax(image, initial=0.0)) > 1.5:
            image /= 255.0
    return np.clip(image, 0.0, 1.0)


def _mask_for_color(rgb: np.ndarray, target: tuple[float, float, float], tolerance: float) -> np.ndarray:
    # Euclidean distance is stable under brightness changes and avoids broad
    # hue ranges that would confuse blue/green zone paint with cargo labels.
    distance = np.linalg.norm(rgb - np.asarray(target, dtype=np.float32), axis=2)
    brightness = rgb.mean(axis=2)
    return (distance <= float(tolerance)) & (brightness >= 0.07)


def _component_masks(mask: np.ndarray, minimum: int) -> list[np.ndarray]:
    """Return connected color components without importing a segmentation oracle."""
    height, width = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    components = []
    for y, x in zip(*np.nonzero(mask)):
        if seen[y, x]:
            continue
        stack = [(int(y), int(x))]
        seen[y, x] = True
        points = []
        while stack:
            py, px = stack.pop(); points.append((py, px))
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    ny, nx = py + dy, px + dx
                    if 0 <= ny < height and 0 <= nx < width and mask[ny, nx] and not seen[ny, nx]:
                        seen[ny, nx] = True; stack.append((ny, nx))
        if len(points) >= minimum:
            item = np.zeros_like(mask, dtype=bool)
            yy, xx = zip(*points); item[yy, xx] = True
            components.append(item)
    return components


def _depth_metric(depth: Any, config: WarehouseSensorConfig) -> np.ndarray:
    values = np.asarray(depth).copy()
    if values.ndim == 3 and values.shape[-1] == 1:
        values = values[..., 0]
    if values.ndim != 2:
        raise ValueError("depth sensor frame must have shape (height, width)")
    values = values.astype(np.float32)
    # Renderer depth is allowed to be metric already.  Normalized MuJoCo depth
    # needs near/far, which a caller can convert before passing this boundary;
    # values in [0,1] are therefore intentionally treated as invalid rather
    # than guessed into fake metric geometry.
    values[~np.isfinite(values)] = np.nan
    values[(values < config.min_depth_m) | (values > config.max_depth_m)] = np.nan
    return values


def detect_cargo_pixels(rgb: Any, *, config: WarehouseSensorConfig | None = None) -> dict[str, dict[str, Any]]:
    """Detect only decoded cargo ArUco labels from an actor RGB frame."""
    cfg = config or WarehouseSensorConfig()
    image = _as_rgb(rgb)
    result: dict[str, dict[str, Any]] = {}
    gray = cv2.cvtColor((image * 255.0).astype(np.uint8), cv2.COLOR_RGB2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(dictionary)
    corners, ids, _ = detector.detectMarkers(gray)
    decoded: dict[str, tuple[np.ndarray, int]] = {}
    if ids is not None:
        for points, marker_id in zip(corners, ids.reshape(-1)):
            cargo_id = _CARGO_TAG_IDS.get(int(marker_id))
            if cargo_id is None:
                continue
            polygon = np.round(points.reshape(4, 2)).astype(int)
            marker_mask = np.zeros(gray.shape, dtype=np.uint8)
            cv2.fillConvexPoly(marker_mask, polygon, 1)
            marker_mask = marker_mask.astype(bool)
            if int(marker_mask.sum()) >= cfg.min_pixels:
                decoded[cargo_id] = (marker_mask, int(marker_mask.sum()))
    for cargo_id in _CARGO_TAG_IDS.values():
        found = decoded.get(cargo_id)
        if found is None:
            result[cargo_id] = {"visible": False, "reason": "ARUCO_NOT_DECODED", "provenance": "robot_camera_rgb_aruco"}
            continue
        mask, marker_pixels = found
        ys, xs = np.nonzero(mask)
        result[cargo_id] = {
            "visible": True,
            "pixels": marker_pixels,
            "bbox_px": [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())],
            "centroid_px": [float(xs.mean()), float(ys.mean())],
            "provenance": "robot_camera_rgb_aruco",
            "_component_mask": mask,
        }
    return result


class WarehouseObservationAdapter:
    """Build one robot's observable warehouse dictionary.

    ``render_depth`` is an optional callable ``(robot_id, camera) -> ndarray``.
    ``render_sensor_frame`` may instead return ``{"rgb": ..., "depth": ...}``.
    The latter takes precedence and is the requested world rendering hook for
    an RGB-D implementation; both hooks are intentionally narrow and public.
    """

    def __init__(self, config: WarehouseSensorConfig | None = None, *, render_depth: Callable[..., Any] | None = None,
                 render_sensor_frame: Callable[..., Mapping[str, Any]] | None = None):
        self.config = config or WarehouseSensorConfig()
        self.render_depth = render_depth
        self.render_sensor_frame = render_sensor_frame

    def _frames(self, world: Any, robot_id: str) -> tuple[Any, Any | None, str, Mapping[str, Any]]:
        if self.render_sensor_frame is not None:
            sample = self.render_sensor_frame(robot_id=robot_id, camera="robot_cam")
            return sample["rgb"], sample.get("depth"), str(sample.get("source", self.config.frame_source)), sample.get("intrinsics", {})
        rgb = world.render_rgb(robot_id=robot_id, camera="robot_cam")
        if self.render_depth is not None:
            depth = self.render_depth(robot_id=robot_id, camera="robot_cam")
            return rgb, depth, self.config.frame_source, {}
        try:
            sample = render_rgbd_sensor(world, robot_id, config=self.config)
            return sample["rgb"], sample["depth"], str(sample.get("source", self.config.frame_source)), sample.get("intrinsics", {})
        except (RuntimeError, AttributeError, KeyError):
            # Render=False worlds and lightweight unit fakes still have a valid
            # RGB actor path; preserve that path while reporting missing depth.
            return rgb, None, "mujoco:robot_cam:rgb", {}

    def observe(self, world: Any, robot_id: str = "r1") -> dict[str, Any]:
        rgb, depth, source, intrinsics = self._frames(world, str(robot_id))
        config = replace(self.config, **{key: float(value) for key, value in intrinsics.items()
                                         if key in {"fx_px", "fy_px", "cx_px", "cy_px"}})
        image = _as_rgb(rgb)
        detections = detect_cargo_pixels(image, config=config)
        metric_depth = _depth_metric(depth, config) if depth is not None else None
        height, width = image.shape[:2]
        cx, cy = config.principal_point((height, width))
        cargos: dict[str, dict[str, Any]] = {}
        detection_status: dict[str, dict[str, Any]] = {}
        for cargo_id, item in detections.items():
            item = dict(item)
            component_mask = item.pop("_component_mask", None)
            if not item["visible"]:
                detection_status[cargo_id] = item
                continue
            px, py = item["centroid_px"]
            if metric_depth is None:
                item.update({"geometry": None, "confidence": 0.35, "reason": "DEPTH_UNAVAILABLE"})
                detection_status[cargo_id] = item
                continue
            sampled = metric_depth[component_mask] if component_mask is not None else np.asarray((), dtype=np.float32)
            valid = sampled[np.isfinite(sampled)]
            if valid.size < max(3, self.config.min_pixels // 2):
                item.update({"geometry": None, "confidence": 0.25, "reason": "DEPTH_OCCLUDED_OR_INVALID"})
                detection_status[cargo_id] = item
                continue
            z = float(np.median(valid))
            depth_std = float(max(config.depth_noise_std_m, 1.4826*np.median(np.abs(valid-z))))
            # Camera optical coordinates: x right, y down, z forward.  This is
            # a camera local measurement, never a world/global cargo position.
            geometry = {"frame": "robot_camera", "position_m": [(px - cx) * z / config.fx_px,
                         (py - cy) * z / config.fy_px, z],
                        "depth_m": z}
            item.update({"geometry": geometry, "range_std_m": depth_std,
                         "confidence": float(min(0.99, 0.55 + 0.4 * min(1.0, valid.size / 30.0))*max(0.,1-depth_std/max(.01,z))),
                         "depth_samples": int(valid.size), "provenance": f"{source}+depth"})
            cargos[cargo_id] = item
        return {"robot_id": str(robot_id), "camera": "robot_cam", "cargos": cargos,
                "detection_status": detection_status,
                "peers": [], "provenance": {"source": source, "geometry": "camera_rgbd_only",
                "privileged_state_used": False}, "limits": {"max_depth_m": self.config.max_depth_m,
                "min_depth_m": config.min_depth_m}}


def observe_warehouse(world: Any, robot_id: str = "r1", *, config: WarehouseSensorConfig | None = None,
                      render_depth: Callable[..., Any] | None = None,
                      render_sensor_frame: Callable[..., Mapping[str, Any]] | None = None) -> dict[str, Any]:
    """Convenience adapter entry point for actor planners."""
    return WarehouseObservationAdapter(config, render_depth=render_depth,
                                       render_sensor_frame=render_sensor_frame).observe(world, robot_id)


def render_rgbd_sensor(world: Any, robot_id: str = "r1", *, config: WarehouseSensorConfig | None = None) -> dict[str, Any]:
    """Render an RGB-D sample through the world's existing render broker.

    The helper intentionally returns pixels and metric depth only.  It does
    not expose ``MjData``, model ids, object transforms, or any other
    simulator metadata.  MuJoCo's depth buffer is converted from its
    normalized representation using the renderer camera clip range on the
    broker thread; this is sensor calibration, not a privileged pose query.
    """
    cfg = config or WarehouseSensorConfig()
    owner = getattr(world, "_owner", world)
    executor = getattr(owner, "_render_executor", None)
    renderer = getattr(owner, "renderer", None)
    if executor is None or renderer is None:
        raise RuntimeError("world has no active MuJoCo render broker")

    def direct() -> dict[str, Any]:
        robot = world.robot(robot_id)
        physics_lock = getattr(owner, "physics_lock", None)
        render_lock = getattr(owner, "render_lock", None)
        locks = [lock for lock in (physics_lock, render_lock) if lock is not None]
        for lock in locks:
            lock.acquire()
        try:
            if hasattr(robot, "_sync_real_camera_mount"):
                robot._sync_real_camera_mount()
            renderer.update_scene(
                owner.data, camera=robot._n("robot_cam"),
                scene_option=getattr(robot, "_robot_sensor_scene_option", None),
            )
            rgb = renderer.render().copy()
            renderer.enable_depth_rendering()
            try:
                normalized = renderer.render().copy()
            finally:
                renderer.disable_depth_rendering()
        finally:
            for lock in reversed(locks):
                lock.release()
        # MuJoCo 3.12 converts the depth buffer to metric camera optical-axis
        # distance inside Renderer.render(); do not apply a second conversion.
        depth = normalized
        camera_id = __import__("mujoco").mj_name2id(
            owner.model, __import__("mujoco").mjtObj.mjOBJ_CAMERA, robot._n("robot_cam")
        )
        if camera_id < 0:
            raise RuntimeError("robot camera is unavailable")
        fovy = float(owner.model.cam_fovy[camera_id])
        height, width = rgb.shape[:2]
        if cfg.depth_noise_std_m:
            # Stable per pixel/seed noise makes evaluation repeatable for a
            # fixed sensor seed while still representing a bounded sensor.
            rng = np.random.default_rng(int(cfg.sensor_seed))
            depth = depth + rng.normal(0.0, float(cfg.depth_noise_std_m), depth.shape)
        # Return calibrated intrinsics for this exact raw RGB-D frame.  The
        # observation adapter uses these instead of the generic fallback.
        return {"rgb": rgb, "depth": depth.astype(np.float32), "source": cfg.frame_source,
                "intrinsics": camera_intrinsics_from_vertical_fovy(width, height, fovy)}

    if threading.get_ident() == getattr(owner, "_render_thread_id", None):
        return direct()
    return executor.submit(direct).result(timeout=30.0)


def observe_robot(world: Any, robot_id: str = "r1", *, observation_id: str | None = None,
                  sensor_seed: int = 0, render_depth: Callable[..., Any] | None = None,
                  render_sensor_frame: Callable[..., Mapping[str, Any]] | None = None) -> dict[str, Any]:
    """Planner facing observation with camera local range and bearing."""
    cfg = replace(WarehouseSensorConfig(), sensor_seed=int(sensor_seed))
    result = observe_warehouse(world, robot_id, config=cfg, render_depth=render_depth,
                               render_sensor_frame=render_sensor_frame)
    labels = {"oak_plank_01": "red", "steel_pipe_01": "blue", "wood_crate_01": "yellow"}
    cargo = []
    for cargo_id, item in result["cargos"].items():
        record = {"cargo_id": cargo_id, "label_color": labels.get(cargo_id, "cyan"), **item}
        geometry = record.get("geometry")
        if geometry is not None:
            x, _y, z = (float(v) for v in geometry["position_m"])
            record["range_m"] = float(math.hypot(x, z))
            record["bearing_rad"] = float(math.atan2(x, z))
        cargo.append(record)
    result["cargo"] = cargo
    result["cargo_observations"] = cargo
    result["sensor_source"] = result["provenance"]["source"]
    result["observation_id"] = observation_id
    result["sensor_seed"] = int(sensor_seed)
    return result


def observe_robot_scan(world: Any, robot_id: str = "r1", *, pending_ids: set[str] | None = None,
                       observation_id: str | None = None, sensor_seed: int = 0,
                       pans: tuple[int, ...] = (1500, 1300, 1700, 1100, 1900)) -> dict[str, Any]:
    """Perform a bounded camera pan sweep using physical servo interpolation."""
    try:
        from sim.masterpi_production_v2 import SEARCH_POSE
    except ModuleNotFoundError as exc:  # lightweight sensor tests without MuJoCo
        if exc.name != "mujoco":
            raise
        SEARCH_POSE = {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}
    rid = str(robot_id)
    wanted = set(pending_ids or ())
    results: list[dict[str, Any]] = []
    try:
        for pan in pans:
            world._team_joint_move_servos({rid: {**SEARCH_POSE, 6: int(pan)}}, 0.35, settle_s=0.08)
            sample = observe_robot(world, rid, observation_id=observation_id, sensor_seed=sensor_seed)
            for item in sample["cargo"]:
                record = dict(item); record["scan_pan"] = int(pan); results.append(record)
            seen = {item["cargo_id"] for item in results}
            if wanted and wanted.issubset(seen):
                break
    finally:
        world._team_joint_move_servos({rid: SEARCH_POSE}, 0.35, settle_s=0.08)
    union: dict[str, dict[str, Any]] = {}
    for item in results:
        union.setdefault(item["cargo_id"], item)
    return {"robot_id": rid, "camera": "robot_cam", "cargo": list(union.values()),
            "scan_pans": [int(p) for p in pans], "observation_id": observation_id,
            "sensor_seed": int(sensor_seed), "sensor_source": "mujoco:robot_cam:rgbd_pan_scan",
            "provenance": {"source": "mujoco:robot_cam:rgbd_pan_scan", "privileged_state_used": False}}


def observe_robots_scan(
    world: Any,
    robot_ids: tuple[str, ...] | list[str],
    pending_ids: Mapping[str, set[str]] | set[str] | None = None,
    observation_ids: Mapping[str, str | None] | list[str | None] | tuple[str | None, ...] | None = None,
    sensor_seed: int = 0,
    pans: tuple[int, ...] = (1500, 1300, 1700, 1100, 1900),
) -> dict[str, dict[str, Any]]:
    """Scan several robot cameras while their pan servos share one timeline.

    Servo motion is issued as one mapping per pan stage.  RGB-D rendering is
    intentionally performed one robot at a time through the existing render
    broker, so every frame observes the same settled simulation time without
    touching another robot's sensor or memory.  Robots leave the active set as
    soon as their own requested cargo IDs have been seen.
    """
    from sim.masterpi_production_v2 import SEARCH_POSE

    ids = tuple(str(rid) for rid in robot_ids)
    if len(set(ids)) != len(ids):
        raise ValueError("robot_ids must be unique")
    if not ids:
        return {}
    if isinstance(pending_ids, Mapping):
        wanted_by_robot = {rid: set(pending_ids.get(rid, set())) for rid in ids}
    else:
        wanted = set(pending_ids or ())
        wanted_by_robot = {rid: set(wanted) for rid in ids}
    if observation_ids is None:
        oid_by_robot = {rid: None for rid in ids}
    elif isinstance(observation_ids, Mapping):
        oid_by_robot = {rid: observation_ids.get(rid) for rid in ids}
    else:
        if len(observation_ids) != len(ids):
            raise ValueError("observation_ids must match robot_ids")
        oid_by_robot = dict(zip(ids, observation_ids))
    results: dict[str, list[dict[str, Any]]] = {rid: [] for rid in ids}
    scan_pans: dict[str, list[int]] = {rid: [] for rid in ids}
    active = set(ids)
    try:
        for pan in pans:
            if not active:
                break
            world._team_joint_move_servos(
                {rid: {**SEARCH_POSE, 6: int(pan)} for rid in sorted(active)},
                0.35,
                settle_s=0.08,
            )
            # The broker serializes image reads, but all active heads were
            # already moved together and therefore share this sim-time view.
            for rid in ids:
                if rid not in active:
                    continue
                sample = observe_robot(
                    world,
                    rid,
                    observation_id=oid_by_robot[rid],
                    sensor_seed=int(sensor_seed),
                )
                scan_pans[rid].append(int(pan))
                results[rid].extend(
                    {**dict(item), "scan_pan": int(pan),
                     **({"observed_sim_time": float(world.data.time)} if getattr(world, "data", None) is not None else {})}
                    for item in sample.get("cargo", ())
                )
                seen = {item.get("cargo_id") for item in results[rid]}
                wanted = wanted_by_robot[rid]
                if wanted and wanted.issubset(seen):
                    active.remove(rid)
    finally:
        # Even a renderer or detector failure leaves every participating head
        # in the calibrated search pose, using one shared servo timeline.
        world._team_joint_move_servos(
            {rid: dict(SEARCH_POSE) for rid in ids},
            0.35,
            settle_s=0.08,
        )

    source = "mujoco:robot_cam:rgbd_pan_scan"
    output: dict[str, dict[str, Any]] = {}
    for rid in ids:
        union: dict[str, dict[str, Any]] = {}
        for item in results[rid]:
            cargo_id = item.get("cargo_id")
            if cargo_id is not None:
                union.setdefault(str(cargo_id), item)
        output[rid] = {
            "robot_id": rid,
            "camera": "robot_cam",
            "cargo": list(union.values()),
            "scan_pans": scan_pans[rid],
            "observation_id": oid_by_robot[rid],
            "sensor_seed": int(sensor_seed),
            "sensor_source": source,
            "provenance": {"source": source, "privileged_state_used": False},
        }
    return output


__all__ = ["WarehouseSensorConfig", "WarehouseObservationAdapter", "detect_cargo_pixels", "observe_warehouse", "observe_robot", "observe_robot_scan", "observe_robots_scan", "render_rgbd_sensor", "camera_intrinsics_from_vertical_fovy"]
