"""Validated, immutable authored maps for camera-localized navigation."""

from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any, Mapping
import xml.etree.ElementTree as ET


SCHEMA = "ugrp.authored_navigation_map.v1"
FRAME = "warehouse_xy_m"
_ID = re.compile(r"^[a-z][a-z0-9_]*$")
_MAP_KEYS = {
    "schema", "map_id", "version", "frame", "bounds_m", "grid_resolution_m",
    "top_camera", "footprint", "zones", "obstacles",
}
_CAMERA_KEYS = {"name", "position_m", "quaternion_wxyz", "fov_y_deg"}
_FOOTPRINT_KEYS = {"unloaded_radius_m", "safety_margin_m"}
_ZONE_KEYS = {"center_m", "radius_m"}
_OBSTACLE_KEYS = {
    "id", "kind", "center_m", "half_extents_m", "height_m", "traversable",
    "cost_multiplier",
}


def _object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _mapping(value: Any, name: str, keys: set[str]) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be an object")
    if set(value) != keys:
        raise ValueError(f"{name} has invalid fields")
    return value


def _number(value: Any, name: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    if positive and result <= 0.0:
        raise ValueError(f"{name} must be positive")
    return result


def _vector(value: Any, name: str, length: int, *, positive: bool = False) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name} must contain {length} numbers")
    return [_number(item, f"{name}[{index}]", positive=positive)
            for index, item in enumerate(value)]


def _identifier(value: Any, name: str) -> str:
    if not isinstance(value, str) or not _ID.fullmatch(value):
        raise ValueError(f"{name} must be a lowercase identifier")
    return value


def _inside_bounds(center: list[float], extent: list[float], bounds: list[float], name: str) -> None:
    if (center[0] - extent[0] < bounds[0] or center[0] + extent[0] > bounds[1]
            or center[1] - extent[1] < bounds[2] or center[1] + extent[1] > bounds[3]):
        raise ValueError(f"{name} lies outside bounds_m")


def validate_map(map_value: Mapping[str, Any]) -> dict[str, Any]:
    """Validate and return a detached plain-dict copy of an authored map."""
    value = _mapping(map_value, "map", _MAP_KEYS)
    if (value["schema"] != SCHEMA or isinstance(value["version"], bool)
            or not isinstance(value["version"], int) or value["version"] != 1
            or value["frame"] != FRAME):
        raise ValueError("unsupported authored navigation map contract")
    _identifier(value["map_id"], "map_id")
    bounds = _vector(value["bounds_m"], "bounds_m", 4)
    if not bounds[0] < bounds[1] or not bounds[2] < bounds[3]:
        raise ValueError("bounds_m must be [min_x,max_x,min_y,max_y]")
    _number(value["grid_resolution_m"], "grid_resolution_m", positive=True)

    camera = _mapping(value["top_camera"], "top_camera", _CAMERA_KEYS)
    if camera["name"] != "cctv_top":
        raise ValueError("top_camera.name must be cctv_top")
    _vector(camera["position_m"], "top_camera.position_m", 3)
    quaternion = _vector(camera["quaternion_wxyz"], "top_camera.quaternion_wxyz", 4)
    if not math.isclose(sum(item * item for item in quaternion), 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError("top_camera.quaternion_wxyz must be a unit quaternion")
    fov = _number(camera["fov_y_deg"], "top_camera.fov_y_deg", positive=True)
    if fov >= 180.0:
        raise ValueError("top_camera.fov_y_deg must be below 180")

    footprint = _mapping(value["footprint"], "footprint", _FOOTPRINT_KEYS)
    _number(footprint["unloaded_radius_m"], "footprint.unloaded_radius_m", positive=True)
    _number(footprint["safety_margin_m"], "footprint.safety_margin_m", positive=True)

    zones = _mapping(value["zones"], "zones", {"start", "goal"})
    for zone_name in ("start", "goal"):
        zone = _mapping(zones[zone_name], f"zones.{zone_name}", _ZONE_KEYS)
        center = _vector(zone["center_m"], f"zones.{zone_name}.center_m", 2)
        radius = _number(zone["radius_m"], f"zones.{zone_name}.radius_m", positive=True)
        _inside_bounds(center, [radius, radius], bounds, f"zones.{zone_name}")

    obstacles = value["obstacles"]
    if not isinstance(obstacles, list):
        raise ValueError("obstacles must be a list")
    seen: set[str] = set()
    for index, raw in enumerate(obstacles):
        obstacle = _mapping(raw, f"obstacles[{index}]", _OBSTACLE_KEYS)
        obstacle_id = _identifier(obstacle["id"], f"obstacles[{index}].id")
        if obstacle_id in seen:
            raise ValueError(f"duplicate obstacle id: {obstacle_id}")
        seen.add(obstacle_id)
        _identifier(obstacle["kind"], f"obstacles[{index}].kind")
        center = _vector(obstacle["center_m"], f"obstacles[{index}].center_m", 2)
        half = _vector(obstacle["half_extents_m"], f"obstacles[{index}].half_extents_m", 2,
                       positive=True)
        _number(obstacle["height_m"], f"obstacles[{index}].height_m", positive=True)
        if obstacle["traversable"] is not False:
            raise ValueError(f"obstacles[{index}].traversable must be false")
        _number(obstacle["cost_multiplier"], f"obstacles[{index}].cost_multiplier",
                positive=True)
        _inside_bounds(center, half, bounds, f"obstacles[{index}]")
        for zone_name in ("start", "goal"):
            zone = zones[zone_name]
            zx, zy = (float(item) for item in zone["center_m"])
            nearest_x = max(center[0] - half[0], min(zx, center[0] + half[0]))
            nearest_y = max(center[1] - half[1], min(zy, center[1] + half[1]))
            if math.hypot(zx - nearest_x, zy - nearest_y) <= float(zone["radius_m"]):
                raise ValueError(f"obstacles[{index}] overlaps zones.{zone_name}")
    return copy.deepcopy(dict(value))


def load_map(path: str | Path) -> dict[str, Any]:
    """Load a UTF-8 JSON map, rejecting duplicate keys and invalid data."""
    source = Path(path)
    try:
        value = json.loads(source.read_text(encoding="utf-8"), object_pairs_hook=_object)
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot load authored navigation map: {source}") from exc
    return validate_map(value)


def map_sha256(map_value: Mapping[str, Any]) -> str:
    """Return SHA-256 of the validated canonical JSON representation."""
    value = validate_map(map_value)
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
                         allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def augment_map_xml(xml: str, map_value: Mapping[str, Any]) -> str:
    """Add map obstacle collision boxes and non-colliding zone paint to MuJoCo XML."""
    value = validate_map(map_value)
    try:
        root = ET.fromstring(xml)
    except ET.ParseError as exc:
        raise ValueError("invalid MuJoCo XML") from exc
    world = root.find("worldbody")
    if world is None:
        raise ValueError("MuJoCo XML has no worldbody")
    names = {f"known_map_{item['id']}" for item in value["obstacles"]}
    names.update({"known_map_zone_start", "known_map_zone_goal"})
    existing = {node.get("name") for node in world.iter() if node.get("name")}
    overlap = names & existing
    if overlap:
        raise ValueError(f"MuJoCo XML already contains known-map names: {sorted(overlap)}")
    for item in value["obstacles"]:
        x, y = item["center_m"]
        hx, hy = item["half_extents_m"]
        height = item["height_m"]
        ET.SubElement(world, "geom", {
            "name": f"known_map_{item['id']}", "type": "box",
            "pos": f"{x:.9g} {y:.9g} {height / 2.0:.9g}",
            "size": f"{hx:.9g} {hy:.9g} {height / 2.0:.9g}",
            "rgba": ".32 .35 .40 1", "contype": "1", "conaffinity": "3",
            "mass": "0", "group": "0",
        })
    for zone_name, rgba in (("start", ".15 .35 .95 .32"), ("goal", ".12 .82 .30 .32")):
        zone = value["zones"][zone_name]
        x, y = zone["center_m"]
        ET.SubElement(world, "geom", {
            "name": f"known_map_zone_{zone_name}", "type": "cylinder",
            "pos": f"{x:.9g} {y:.9g} .001", "size": f"{zone['radius_m']:.9g} .001",
            "rgba": rgba, "contype": "0", "conaffinity": "0", "mass": "0", "group": "0",
        })
    return ET.tostring(root, encoding="unicode")


__all__ = ["SCHEMA", "FRAME", "augment_map_xml", "load_map", "map_sha256", "validate_map"]
