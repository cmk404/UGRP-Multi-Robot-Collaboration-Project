"""MuJoCo geometry and geometry-only checks for cooperative beam transport.

The multi-robot production world is deliberately kept unchanged.  Call
``add_cooperative_payload_xml`` on the parsed XML root returned by
``build_multi_robot_xml`` to add one dynamic payload, two visual mission
zones, and inactive endpoint welds for the two carrier robots.  The welds are
activated by the controller only after both grippers have been physically
verified at their endpoints.

The pose and zone functions in this module do not mutate MuJoCo state.  This
keeps mission scoring usable from a referee, a test, or a replay process.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
import copy
import math
import xml.etree.ElementTree as ET
from typing import Mapping, Sequence


# ---------------------------------------------------------------------------
# Payload and mission fixture constants

BEAM_BODY_NAME = "team_beam"
BEAM_FREEJOINT_NAME = "team_beam_free"
BEAM_GEOM_NAME = "team_beam_geom"

# A 45 cm beam is long enough that one MasterPi cannot span both endpoints.
# Its mass is a provisional SIM stress value; it is intentionally not a claim
# about the physical carrying capacity of the real platform.
BEAM_LENGTH_M = 0.45
BEAM_WIDTH_M = 0.05
BEAM_HEIGHT_M = 0.04
BEAM_MASS_KG = 0.18
BEAM_HALF_LENGTH_M = BEAM_LENGTH_M / 2.0
BEAM_HALF_WIDTH_M = BEAM_WIDTH_M / 2.0
BEAM_HALF_HEIGHT_M = BEAM_HEIGHT_M / 2.0

# Leave a small end margin for a gripper pad.  The endpoint child bodies are
# what the weld constraints bind to; the sites make the intended grasp points
# visible to a renderer and available to an observer.
BEAM_ENDPOINT_OFFSET_M = BEAM_HALF_LENGTH_M - 0.025
BEAM_CARRIER_IDS = ("r1", "r3")
BEAM_ENDPOINT_OFFSETS_M = {
    "r1": (0.0, -BEAM_ENDPOINT_OFFSET_M, 0.0),
    "r3": (0.0, BEAM_ENDPOINT_OFFSET_M, 0.0),
}

# The lane is below the existing workbench and delivery fixtures.  Positions
# are beam-centre poses; the z coordinate puts the beam on the floor.
# Legacy single-beam fixture is parked outside the primary warehouse camera.
# The user-facing mission uses the natural oak plank/pipe/crate set instead.
BEAM_START = (0.58, -2.0, BEAM_HALF_HEIGHT_M)
BEAM_GOAL = (1.08, -2.0, BEAM_HALF_HEIGHT_M)
BEAM_START_XY = BEAM_START[:2]
BEAM_GOAL_XY = BEAM_GOAL[:2]
BEAM_START_YAW_RAD = 0.0
BEAM_GOAL_YAW_RAD = 0.0

# Zone half-extents include both the payload centre and its endpoint markers.
# They are visual-only geoms and do not create a hidden physical target.
BEAM_ZONE_HALF_EXTENTS_M = (0.16, 0.30)
BEAM_ZONE_HEIGHT_M = 0.004
BEAM_ZONE_CENTER_Z_M = 0.002
BEAM_POSITION_TOLERANCE_M = 0.035
BEAM_YAW_TOLERANCE_RAD = math.radians(15.0)
BEAM_LEVEL_TOLERANCE_RAD = math.radians(10.0)


@dataclass(frozen=True)
class BeamZone:
    """Immutable rectangular mission zone in the world XY plane."""

    center_xy: tuple[float, float]
    half_extents_xy: tuple[float, float] = BEAM_ZONE_HALF_EXTENTS_M
    yaw_rad: float = 0.0


BEAM_START_ZONE = BeamZone(BEAM_START_XY, BEAM_ZONE_HALF_EXTENTS_M, BEAM_START_YAW_RAD)
BEAM_GOAL_ZONE = BeamZone(BEAM_GOAL_XY, BEAM_ZONE_HALF_EXTENTS_M, BEAM_GOAL_YAW_RAD)


@dataclass(frozen=True)
class BeamPose:
    """Pure, serializable pose representation used by the mission referee."""

    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
    yaw_rad: float
    endpoint_positions: tuple[tuple[float, float, float], ...]

    def as_dict(self) -> dict[str, object]:
        """Return a plain mapping convenient for JSON/event traces."""
        return {
            "position": self.position,
            "center": self.position,
            "quaternion": self.quaternion,
            "quat": self.quaternion,
            "yaw_rad": self.yaw_rad,
            "yaw": self.yaw_rad,
            "endpoint_positions": self.endpoint_positions,
        }


def _vector(values: Sequence[float], length: int, name: str) -> tuple[float, ...]:
    try:
        result = tuple(float(v) for v in values)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain {length} finite numbers") from exc
    if len(result) != length or not all(math.isfinite(v) for v in result):
        raise ValueError(f"{name} must contain {length} finite numbers")
    return result


def _normalise_quaternion(quaternion: Sequence[float]) -> tuple[float, float, float, float]:
    q = _vector(quaternion, 4, "quaternion")
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= 1e-12:
        raise ValueError("quaternion norm must be non-zero")
    return tuple(v / norm for v in q)  # type: ignore[return-value]


def _quat_to_yaw(quaternion: Sequence[float]) -> float:
    w, x, y, z = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _rotate_vector(quaternion: Sequence[float], vector: Sequence[float]) -> tuple[float, float, float]:
    """Rotate a 3-vector by a wxyz quaternion without numpy."""
    w, x, y, z = quaternion
    vx, vy, vz = vector
    # R(q) in row-major form.  MuJoCo exposes quaternions in wxyz order.
    return (
        (1 - 2 * (y * y + z * z)) * vx + 2 * (x * y - z * w) * vy + 2 * (x * z + y * w) * vz,
        2 * (x * y + z * w) * vx + (1 - 2 * (x * x + z * z)) * vy + 2 * (y * z - x * w) * vz,
        2 * (x * z - y * w) * vx + 2 * (y * z + x * w) * vy + (1 - 2 * (x * x + y * y)) * vz,
    )


def _angle_error(a: float, b: float) -> float:
    return abs((float(a) - float(b) + math.pi) % (2.0 * math.pi) - math.pi)


def payload_endpoint_positions(
    position: Sequence[float],
    quaternion: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
) -> tuple[tuple[float, float, float], ...]:
    """Return world positions of the two beam endpoint grasp markers."""
    p = _vector(position, 3, "position")
    q = _normalise_quaternion(quaternion)
    result = []
    for offset in (BEAM_ENDPOINT_OFFSETS_M["r1"], BEAM_ENDPOINT_OFFSETS_M["r3"]):
        rotated = _rotate_vector(q, offset)
        result.append(tuple(p[i] + rotated[i] for i in range(3)))
    return tuple(result)


def payload_pose(
    position: Sequence[float],
    quaternion: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
) -> BeamPose:
    """Build a pure ``BeamPose`` from a centre position and wxyz quaternion."""
    p = _vector(position, 3, "position")
    q = _normalise_quaternion(quaternion)
    return BeamPose(
        position=p,  # type: ignore[arg-type]
        quaternion=q,
        yaw_rad=_quat_to_yaw(q),
        endpoint_positions=payload_endpoint_positions(p, q),
    )


def _pose_values(pose: BeamPose | Mapping[str, object] | Sequence[float]) -> BeamPose:
    if isinstance(pose, BeamPose):
        return pose
    if isinstance(pose, Mapping):
        position = pose.get("position", pose.get("center", pose.get("xyz")))
        quaternion = pose.get("quaternion", pose.get("quat", (1.0, 0.0, 0.0, 0.0)))
        if position is None:
            raise ValueError("pose mapping must contain position or center")
        return payload_pose(position, quaternion)  # type: ignore[arg-type]
    values = tuple(float(v) for v in pose)
    if len(values) == 7:
        return payload_pose(values[:3], values[3:])
    if len(values) == 3:
        return payload_pose(values)
    raise ValueError("pose must be BeamPose, mapping, xyz, or xyz+wxyz")


def point_in_zone(
    point_xy: Sequence[float],
    center_xy: Sequence[float],
    half_extents_xy: Sequence[float] = BEAM_ZONE_HALF_EXTENTS_M,
    *,
    margin_m: float = 0.0,
) -> bool:
    """Pure inclusive rectangle test for a point in an XY mission zone."""
    point = _vector(point_xy, 2, "point_xy")
    center = _vector(center_xy, 2, "center_xy")
    extents = _vector(half_extents_xy, 2, "half_extents_xy")
    margin = float(margin_m)
    if not math.isfinite(margin) or margin < 0.0:
        raise ValueError("margin_m must be a finite non-negative number")
    return all(abs(point[i] - center[i]) <= extents[i] + margin for i in range(2))


def beam_in_zone(
    pose: BeamPose | Mapping[str, object] | Sequence[float],
    zone: BeamZone = BEAM_GOAL_ZONE,
    *,
    margin_m: float = 0.0,
    require_endpoints: bool = True,
) -> bool:
    """Return whether the beam centre (and optionally both endpoints) is in ``zone``."""
    value = _pose_values(pose)
    centre_ok = point_in_zone(value.position[:2], zone.center_xy, zone.half_extents_xy, margin_m=margin_m)
    if not centre_ok:
        return False
    if not require_endpoints:
        return True
    return all(
        point_in_zone(endpoint[:2], zone.center_xy, zone.half_extents_xy, margin_m=margin_m)
        for endpoint in value.endpoint_positions
    )


def evaluate_beam_mission(
    pose: BeamPose | Mapping[str, object] | Sequence[float],
    *,
    zone: BeamZone = BEAM_GOAL_ZONE,
    position_tolerance_m: float = BEAM_POSITION_TOLERANCE_M,
    yaw_tolerance_rad: float = BEAM_YAW_TOLERANCE_RAD,
    level_tolerance_rad: float = BEAM_LEVEL_TOLERANCE_RAD,
) -> dict[str, object]:
    """Score a beam pose against a destination zone without mutating state.

    ``success`` requires centre and endpoint containment, target yaw, and a
    level beam (roll/pitch).  The individual booleans and errors are returned
    as evidence so a failed mission is diagnosable rather than just false.
    """
    value = _pose_values(pose)
    z = float(value.position[2])
    target_z = BEAM_HALF_HEIGHT_M
    position_error = math.hypot(
        value.position[0] - zone.center_xy[0],
        value.position[1] - zone.center_xy[1],
    )
    centre_in_zone = point_in_zone(value.position[:2], zone.center_xy, zone.half_extents_xy)
    endpoints_in_zone = all(
        point_in_zone(endpoint[:2], zone.center_xy, zone.half_extents_xy)
        for endpoint in value.endpoint_positions
    )
    yaw_error = _angle_error(value.yaw_rad, zone.yaw_rad)
    # q is wxyz; derive roll/pitch from the normalised quaternion for a level
    # check that catches a beam resting on an edge even when yaw is correct.
    w, x, y, zz = value.quaternion
    roll = math.atan2(2.0 * (w * x + y * zz), 1.0 - 2.0 * (x * x + y * y))
    pitch_sin = 2.0 * (w * y - zz * x)
    pitch = math.asin(max(-1.0, min(1.0, pitch_sin)))
    level_error = max(abs(roll), abs(pitch))
    height_error = abs(z - target_z)
    in_zone = centre_in_zone and endpoints_in_zone
    position_ok = position_error <= float(position_tolerance_m)
    orientation_ok = yaw_error <= float(yaw_tolerance_rad)
    level_ok = level_error <= float(level_tolerance_rad)
    height_ok = height_error <= float(position_tolerance_m)
    return {
        "success": bool(in_zone and position_ok and orientation_ok and level_ok and height_ok),
        "in_zone": bool(in_zone),
        "position_ok": bool(position_ok),
        "center_in_zone": bool(centre_in_zone),
        "endpoints_in_zone": bool(endpoints_in_zone),
        "orientation_ok": bool(orientation_ok),
        "level_ok": bool(level_ok),
        "height_ok": bool(height_ok),
        "position": value.position,
        "quaternion": value.quaternion,
        "endpoint_positions": value.endpoint_positions,
        "position_error_m": position_error,
        "height_error_m": height_error,
        "yaw_error_rad": yaw_error,
        "level_error_rad": level_error,
        "zone_center_xy": zone.center_xy,
    }


def beam_pose(data: object, model: object, body_name: str = BEAM_BODY_NAME) -> dict[str, object]:
    """Read the current MuJoCo body pose and return a plain mapping.

    This is the only helper that touches MuJoCo objects.  It is read-only from
    the caller's perspective; ``payload_pose`` and all scoring helpers remain
    usable without importing MuJoCo.
    """
    try:
        import mujoco

        body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name))
    except Exception as exc:
        raise KeyError(f"MuJoCo body not found: {body_name}") from exc
    if body_id < 0:
        raise KeyError(f"MuJoCo body not found: {body_name}")
    pose = payload_pose(data.xpos[body_id], data.xquat[body_id])  # type: ignore[attr-defined]
    result = pose.as_dict()
    result["body_id"] = body_id
    result["body_name"] = body_name
    return result


def _xyz(value: Sequence[float], name: str) -> tuple[float, float, float]:
    values = _vector(value, 3, name)
    return values  # type: ignore[return-value]


def _set_attr(element: ET.Element, name: str, values: Sequence[float]) -> None:
    element.set(name, " ".join(f"{float(v):.6f}" for v in values))


def add_cooperative_payload_xml(
    root: ET.Element,
    *,
    beam_position: Sequence[float] = BEAM_START,
    beam_quaternion: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
    carrier_ids: Sequence[str] = BEAM_CARRIER_IDS,
    start_xy: Sequence[float] = BEAM_START_XY,
    goal_xy: Sequence[float] = BEAM_GOAL_XY,
) -> ET.Element:
    """Add the cooperative beam fixture to a parsed MuJoCo XML root.

    The root is mutated in place and returned for convenient chaining.  The
    operation is idempotent for this fixture: calling it twice replaces only
    nodes owned by this module, not unrelated world geometry.
    """
    if not isinstance(root, ET.Element):
        raise TypeError("root must be an xml.etree.ElementTree.Element")
    ids = tuple(str(robot_id).strip().lower() for robot_id in carrier_ids)
    if len(ids) != 2 or len(set(ids)) != 2:
        raise ValueError("carrier_ids must contain two unique robot ids")
    position = _xyz(beam_position, "beam_position")
    quaternion = _normalise_quaternion(beam_quaternion)
    start = _vector(start_xy, 2, "start_xy")
    goal = _vector(goal_xy, 2, "goal_xy")
    world = root.find("worldbody")
    if world is None:
        raise ValueError("MuJoCo XML root has no worldbody")

    owned_world_names = {
        BEAM_BODY_NAME,
        "team_beam_start_zone",
        "team_beam_goal_zone",
    }
    for child in list(world):
        if child.get("name") in owned_world_names:
            world.remove(child)
    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")
    owned_equality_names = {f"{rid}__beam_grasp" for rid in ids}
    for child in list(equality):
        if child.get("name") in owned_equality_names:
            equality.remove(child)

    beam = ET.Element("body", {"name": BEAM_BODY_NAME})
    _set_attr(beam, "pos", position)
    _set_attr(beam, "quat", quaternion)
    ET.SubElement(beam, "freejoint", {"name": BEAM_FREEJOINT_NAME})
    ET.SubElement(
        beam,
        "geom",
        {
            "name": BEAM_GEOM_NAME,
            "type": "box",
            # The central bar stops before the two narrow grasp handles.  Each
            # handle is a distinct rigid child geom so the referee can prove
            # that the two carriers contacted opposite endpoints.
            "size": f"{BEAM_HALF_WIDTH_M:.6f} {BEAM_ENDPOINT_OFFSET_M - 0.025:.6f} {BEAM_HALF_HEIGHT_M:.6f}",
            "mass": f"{BEAM_MASS_KG:.6f}",
            "rgba": ".90 .46 .08 1",
            "friction": "1.2 .02 .002",
            "contype": "1",
            "conaffinity": "3",
            "solref": ".008 1",
            "solimp": ".92 .98 .002",
        },
    )
    endpoint_names = []
    for rid, offset in zip(ids, (BEAM_ENDPOINT_OFFSETS_M["r1"], BEAM_ENDPOINT_OFFSETS_M["r3"])):
        endpoint_name = f"{BEAM_BODY_NAME}_{rid}_endpoint"
        endpoint_names.append(endpoint_name)
        endpoint = ET.SubElement(beam, "body", {"name": endpoint_name})
        _set_attr(endpoint, "pos", offset)
        ET.SubElement(
            endpoint,
            "geom",
            {
                "name": f"{endpoint_name}_geom",
                "type": "box",
                "pos": "0 0 .015",
                "size": f"{BEAM_HALF_WIDTH_M:.6f} .015000 {BEAM_HALF_HEIGHT_M:.6f}",
                "mass": ".008",
                "rgba": ".10 .80 .95 1",
                "friction": "2.0 .03 .002",
                "contype": "1",
                "conaffinity": "3",
                "solref": ".006 1",
                "solimp": ".92 .98 .002",
            },
        )
        ET.SubElement(
            endpoint,
            "site",
            {
                "name": f"{endpoint_name}_site",
                "type": "sphere",
                "size": ".012",
                "rgba": ".15 .85 .95 .85",
                "group": "3",
            },
        )
    world.append(beam)

    for name, xy, colour in (
        ("team_beam_start_zone", start, ".10 .40 .95 .30"),
        ("team_beam_goal_zone", goal, ".15 .85 .35 .30"),
    ):
        zone = ET.Element(
            "geom",
            {
                "name": name,
                "type": "box",
                "size": f"{BEAM_ZONE_HALF_EXTENTS_M[0]:.6f} {BEAM_ZONE_HALF_EXTENTS_M[1]:.6f} {BEAM_ZONE_HEIGHT_M:.6f}",
                "rgba": colour,
                "contype": "0",
                "conaffinity": "0",
                "mass": "0",
                "group": "4",
            },
        )
        _set_attr(zone, "pos", (xy[0], xy[1], BEAM_ZONE_CENTER_Z_M))
        world.append(zone)

    for rid, endpoint_name in zip(ids, endpoint_names):
        ET.SubElement(
            equality,
            "weld",
            {
                "name": f"{rid}__beam_grasp",
                "body1": f"{rid}__gripper",
                "body2": endpoint_name,
                "active": "false",
                "solref": ".010 1",
                "solimp": ".90 .95 .001",
            },
        )
    return root


# Catalog entries beyond ``beam`` define the next mission fixtures without
# pretending their physical controllers are implemented yet.  They give the
# mission UI/referee stable, varied shapes to enumerate while Beam Transport is
# the executable MVP.
PAYLOAD_CATALOG: dict[str, dict[str, object]] = {
    "beam": {
        "payload": {
            "dimensions": (BEAM_WIDTH_M, BEAM_LENGTH_M, BEAM_HEIGHT_M),
            "mass": BEAM_MASS_KG,
            "required_carriers": 2,
        },
        "start_zone": {
            "center": BEAM_START,
            "half_extents": BEAM_ZONE_HALF_EXTENTS_M,
            "yaw": BEAM_START_YAW_RAD,
        },
        "goal_zone": {
            "center": BEAM_GOAL,
            "half_extents": BEAM_ZONE_HALF_EXTENTS_M,
            "yaw": BEAM_GOAL_YAW_RAD,
            "yaw_tolerance": BEAM_YAW_TOLERANCE_RAD,
        },
        "required_carriers": 2,
        "yaw_tolerance": BEAM_YAW_TOLERANCE_RAD,
    },
    "panel": {
        "payload": {"dimensions": (0.04, 0.38, 0.26), "mass": 0.15, "required_carriers": 2},
        "start_zone": {"center": BEAM_START, "yaw": 0.0},
        "goal_zone": {"center": BEAM_GOAL, "yaw": math.pi / 2.0},
        "required_carriers": 2,
    },
    "pipe": {
        "payload": {"dimensions": (0.05, 0.42, 0.05), "mass": 0.12, "required_carriers": 2},
        "start_zone": {"center": BEAM_START, "yaw": 0.0},
        "goal_zone": {"center": BEAM_GOAL, "yaw": 0.0},
        "required_carriers": 2,
    },
    "tray": {
        "payload": {"dimensions": (0.24, 0.36, 0.025), "mass": 0.10, "required_carriers": 2},
        "start_zone": {"center": BEAM_START, "yaw": 0.0},
        "goal_zone": {"center": BEAM_GOAL, "yaw": 0.0},
        "required_carriers": 2,
    },
}


class CooperativePayloadMission:
    """Pure mission contract used by tests, UI summaries, and the SIM referee."""

    def __init__(self, payload_type: str = "beam"):
        kind = str(payload_type or "beam").strip().lower()
        if kind not in PAYLOAD_CATALOG:
            raise ValueError(f"unsupported cooperative payload type: {kind}")
        self.payload_type = kind
        self.spec = copy.deepcopy(PAYLOAD_CATALOG[kind])

    @staticmethod
    def _contacts(state: Mapping[str, object]) -> set[str]:
        contacts = state.get("contacts") or []
        out = set()
        if isinstance(contacts, Sequence) and not isinstance(contacts, (str, bytes)):
            for item in contacts:
                if not isinstance(item, Mapping) or not bool(item.get("grasp")):
                    continue
                rid = str(item.get("robot_id") or "").strip().lower()
                if rid:
                    out.add(rid)
        return out

    def evaluate_final_state(self, state: Mapping[str, object]) -> dict[str, object]:
        contacts = self._contacts(state)
        if len(contacts) < int(self.spec.get("required_carriers") or 2):
            return {
                "success": False,
                "failure_code": "CONTACT_REQUIRED_NO_TELEPORT",
                "reason": "cooperative delivery requires two physical carrier contacts",
                "state": dict(state),
            }
        pose_obj = state.get("object_pose") or state.get("payload_pose") or {}
        if not isinstance(pose_obj, Mapping):
            return {"success": False, "failure_code": "PAYLOAD_POSE_MISSING", "state": dict(state)}
        position = pose_obj.get("position") or pose_obj.get("center") or pose_obj.get("xyz")
        if position is None:
            return {"success": False, "failure_code": "PAYLOAD_POSE_MISSING", "state": dict(state)}
        if "quaternion" in pose_obj or "quat" in pose_obj:
            quaternion = pose_obj.get("quaternion") or pose_obj.get("quat")
        else:
            yaw = float(pose_obj.get("yaw") or pose_obj.get("yaw_rad") or 0.0)
            quaternion = (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))
        evidence = evaluate_beam_mission({"position": position, "quaternion": quaternion})
        stable = bool(state.get("stable")) and bool(state.get("settled"))
        success = bool(evidence["success"] and stable)
        return {
            **evidence,
            "success": success,
            "stable": stable,
            "contacts": sorted(contacts),
            "failure_code": None if success else (
                "PAYLOAD_NOT_STABLE" if not stable else "DESTINATION_POSE_INVALID"
            ),
            "state": dict(state),
        }

    evaluate_delivery = evaluate_final_state
    evaluate = evaluate_final_state

    def attempt_transport(
        self,
        carriers: Sequence[str],
        state: Mapping[str, object] | None = None,
    ) -> dict[str, object]:
        snapshot = dict(state or {})
        unique = tuple(dict.fromkeys(str(r).strip().lower() for r in carriers if str(r).strip()))
        required = int(self.spec.get("required_carriers") or 2)
        if len(unique) < required:
            return {
                "success": False,
                "failure_code": "SINGLE_CARRIER_REFUSED",
                "reason": f"cooperative payload requires at least {required} carriers",
                "state": snapshot,
            }
        contacts = self._contacts(snapshot)
        if not set(unique).issubset(contacts):
            return {
                "success": False,
                "failure_code": "CARRIER_CONTACT_MISSING",
                "reason": "pose-only transport is forbidden without carrier grasp evidence",
                "state": snapshot,
            }
        return self.evaluate_final_state(snapshot)

    transport = attempt_transport
    run_transport = attempt_transport


def build_mission(payload_type: str = "beam") -> CooperativePayloadMission:
    return CooperativePayloadMission(payload_type)


create_mission = build_mission


def build_cooperative_payload_xml(*args: object, **kwargs: object) -> str:
    """Convenience builder around ``build_multi_robot_xml`` plus the fixture."""
    from sim.multi_masterpi_production import build_multi_robot_xml

    root_xml = build_multi_robot_xml(*args, **kwargs)
    root = ET.fromstring(root_xml)
    add_cooperative_payload_xml(root)
    return ET.tostring(root, encoding="unicode")


# Friendly aliases used by callers that describe the catalog as a collection.
PAYLOADS = PAYLOAD_CATALOG
payload_catalog = PAYLOAD_CATALOG
BEAM_MISSION_SPEC = PAYLOAD_CATALOG["beam"]
build_cooperative_payload_mission = build_mission
cooperative_payload_mission = build_mission


__all__ = [
    "BEAM_BODY_NAME", "BEAM_FREEJOINT_NAME", "BEAM_GEOM_NAME",
    "BEAM_LENGTH_M", "BEAM_WIDTH_M", "BEAM_HEIGHT_M", "BEAM_MASS_KG",
    "BEAM_HALF_LENGTH_M", "BEAM_HALF_WIDTH_M", "BEAM_HALF_HEIGHT_M",
    "BEAM_ENDPOINT_OFFSET_M", "BEAM_ENDPOINT_OFFSETS_M", "BEAM_CARRIER_IDS",
    "BEAM_START", "BEAM_GOAL", "BEAM_START_XY", "BEAM_GOAL_XY",
    "BEAM_START_YAW_RAD", "BEAM_GOAL_YAW_RAD",
    "BEAM_ZONE_HALF_EXTENTS_M", "BEAM_START_ZONE", "BEAM_GOAL_ZONE",
    "BeamZone", "BeamPose", "payload_endpoint_positions", "payload_pose",
    "beam_pose", "point_in_zone", "beam_in_zone", "evaluate_beam_mission",
    "add_cooperative_payload_xml", "build_cooperative_payload_xml",
    "PAYLOAD_CATALOG", "PAYLOADS", "payload_catalog", "BEAM_MISSION_SPEC",
    "CooperativePayloadMission", "build_mission", "create_mission",
    "build_cooperative_payload_mission", "cooperative_payload_mission",
]
