"""Multi-zone, multi-cargo warehouse mission fixtures and referee helpers."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Mapping


@dataclass(frozen=True)
class ZoneSpec:
    zone_id: str
    name: str
    color: str
    center_xy: tuple[float, float]
    half_extents_xy: tuple[float, float]
    rgba: str


@dataclass(frozen=True)
class CargoSpec:
    cargo_id: str
    cargo_type: str
    label_color: str
    body_name: str
    joint_name: str
    dimensions_m: tuple[float, float, float]
    mass_kg: float
    start_xyz: tuple[float, float, float]
    goal_xyz: tuple[float, float, float]
    half_length_m: float
    # Assigned by the runtime auction. Scene fixtures intentionally carry no
    # robot identity so a new object cannot inherit a memorized pair.
    carriers: tuple[str, str] = ()
    scout: str = ""
    start_yaw_rad: float = 0.0
    yaw_tolerance_rad: float = math.radians(25.0)
    position_tolerance_m: float = 0.22
    required_carriers: int = 2
    goal_yaw_rad: float = 0.0


@dataclass(frozen=True)
class TerrainSpec:
    terrain_id: str
    kind: str
    center_xy: tuple[float, float]
    half_extents_xy: tuple[float, float]
    height_m: float
    traversable: bool = True
    cost_multiplier: float = 1.0


WAREHOUSE_ZONES: dict[str, ZoneSpec] = {
    "A": ZoneSpec("A", "checkpoint_a", "blue", (1.05, 0.0), (0.28, 1.45), ".10 .35 .95 .28"),
    "B": ZoneSpec("B", "checkpoint_b", "green", (1.85, 0.0), (0.28, 1.45), ".12 .82 .30 .28"),
    "C": ZoneSpec("C", "checkpoint_c", "yellow", (2.65, 0.0), (0.28, 1.45), ".95 .72 .08 .30"),
}
WAREHOUSE_ZONE_ORDER = ("A", "B", "C")

CARGO_SPECS: tuple[CargoSpec, ...] = (
    CargoSpec(
        "oak_plank_01", "plank", "red", "warehouse_oak_plank", "warehouse_oak_plank_free",
        (0.06, 0.43, 0.036), 0.18, (0.75, -1.00, 0.018), (1.55, -1.00, 0.018),
        0.215,
    ),
    CargoSpec(
        "steel_pipe_01", "pipe", "blue", "warehouse_steel_pipe", "warehouse_steel_pipe_free",
        (0.052, 0.42, 0.044), 0.20, (0.75, 0.0, 0.018), (1.55, 0.0, 0.018),
        0.210, yaw_tolerance_rad=math.radians(25.0),
        position_tolerance_m=0.22,
    ),
    CargoSpec(
        "wood_crate_01", "crate", "yellow", "warehouse_wood_crate", "warehouse_wood_crate_free",
        (0.055, 0.34, 0.04), 0.26, (0.75, 1.00, 0.020), (1.55, 1.00, 0.020),
        0.170, position_tolerance_m=0.22,
    ),
)
CARGO_BY_ID = {spec.cargo_id: spec for spec in CARGO_SPECS}


def mixed_cargo_specs_for_seed(
    seed: int,
    source_zone: str = "A",
    destination_zone: str = "B",
) -> tuple[CargoSpec, ...]:
    """Return one joint long plank and two independently carryable small boxes.

    This is opt-in so the historical three-cargo fixture remains byte-for-byte
    compatible for callers that use :data:`CARGO_SPECS`.
    """
    source = str(source_zone).upper()
    destination = str(destination_zone).upper()
    warehouse_route_zones(source, destination)
    base = CARGO_SPECS[0]
    boxes = (
        CargoSpec(
            "small_box_01", "small_box", "small_box", "warehouse_small_box_01",
            "warehouse_small_box_01_free", (0.034, 0.040, 0.032), 0.03,
            (0.75, -0.46, 0.016), (1.55, -0.46, 0.016), 0.020,
            position_tolerance_m=0.12, required_carriers=1,
        ),
        CargoSpec(
            "small_box_02", "small_box", "small_box", "warehouse_small_box_02",
            "warehouse_small_box_02_free", (0.036, 0.038, 0.034), 0.03,
            (0.75, 0.46, 0.017), (1.55, 0.46, 0.017), 0.019,
            position_tolerance_m=0.12, required_carriers=1,
        ),
    )
    specs = (base, *boxes)
    positions = mixed_cargo_zone_positions_for_seed(seed, specs)
    return tuple(replace(
        spec,
        start_xyz=positions[spec.cargo_id][source],
        goal_xyz=positions[spec.cargo_id][destination],
        carriers=(), scout="",
    ) for spec in specs)


def warehouse_route_zones(source_zone: str, destination_zone: str) -> tuple[str, ...]:
    source = str(source_zone).upper()
    destination = str(destination_zone).upper()
    if source not in WAREHOUSE_ZONES or destination not in WAREHOUSE_ZONES:
        raise ValueError("unknown warehouse zone")
    if source == destination:
        raise ValueError("source and destination zones must differ")
    start = WAREHOUSE_ZONE_ORDER.index(source)
    end = WAREHOUSE_ZONE_ORDER.index(destination)
    step = 1 if end > start else -1
    return tuple(WAREHOUSE_ZONE_ORDER[index] for index in range(start, end + step, step))


def terrain_specs_for_seed(seed: int) -> tuple[TerrainSpec, ...]:
    """Generate a bounded multi-feature obstacle course for an unseen seed."""
    rng = random.Random((int(seed) ^ 0x54455252) & 0xFFFFFFFF)
    edge_index = rng.randrange(len(WAREHOUSE_ZONE_ORDER) - 1)
    left = WAREHOUSE_ZONES[WAREHOUSE_ZONE_ORDER[edge_index]].center_xy[0]
    right = WAREHOUSE_ZONES[WAREHOUSE_ZONE_ORDER[edge_index + 1]].center_xy[0]
    barrier_left = WAREHOUSE_ZONES["A"].center_xy[0]
    barrier_right = WAREHOUSE_ZONES["B"].center_xy[0]
    gate_x = (WAREHOUSE_ZONES["B"].center_xy[0] + WAREHOUSE_ZONES["C"].center_xy[0]) / 2.0
    # The passage is constrained but does not force metre-scale strafing from
    # the outer cargo lanes. Loaded teams still need a short lane correction.
    gap_half = rng.uniform(1.22, 1.28)
    post_half_y = rng.uniform(0.20, 0.24)
    post_center_y = gap_half + post_half_y
    return (
        TerrainSpec(
            terrain_id="procedural_mound_01",
            kind="mound",
            center_xy=(
                (left + right) / 2.0 + rng.uniform(-0.035, 0.035),
                rng.uniform(-0.82, 0.82),
            ),
            half_extents_xy=(rng.uniform(0.15, 0.20), rng.uniform(0.12, 0.18)),
            height_m=rng.uniform(0.006, 0.010),
            traversable=True,
            cost_multiplier=rng.uniform(1.8, 2.5),
        ),
        TerrainSpec(
            terrain_id="procedural_barrier_01",
            kind="barrier",
            center_xy=(
                (barrier_left + barrier_right) / 2.0 + rng.uniform(-0.05, 0.05),
                rng.choice((-1.0, 1.0)) * rng.uniform(0.58, 0.82),
            ),
            half_extents_xy=(rng.uniform(0.055, 0.085), rng.uniform(0.08, 0.12)),
            height_m=rng.uniform(0.08, 0.13),
            traversable=False,
            cost_multiplier=math.inf,
        ),
        TerrainSpec(
            terrain_id="procedural_gate_north",
            kind="gate_post",
            center_xy=(gate_x + rng.uniform(-0.025, 0.025), post_center_y),
            half_extents_xy=(rng.uniform(0.055, 0.075), post_half_y),
            height_m=rng.uniform(0.10, 0.16),
            traversable=False,
            cost_multiplier=math.inf,
        ),
        TerrainSpec(
            terrain_id="procedural_gate_south",
            kind="gate_post",
            center_xy=(gate_x + rng.uniform(-0.025, 0.025), -post_center_y),
            half_extents_xy=(rng.uniform(0.055, 0.075), post_half_y),
            height_m=rng.uniform(0.10, 0.16),
            traversable=False,
            cost_multiplier=math.inf,
        ),
    )


def cargo_zone_positions_for_seed(
    seed: int,
    specs: tuple[CargoSpec, ...] = CARGO_SPECS,
) -> dict[str, dict[str, tuple[float, float, float]]]:
    rng = random.Random((int(seed) ^ 0x55475250) & 0xFFFFFFFF)
    lane_y = {
        "oak_plank_01": -1.0,
        "steel_pipe_01": 0.0,
        "wood_crate_01": 1.0,
    }
    positions: dict[str, dict[str, tuple[float, float, float]]] = {}
    for spec in specs:
        positions[spec.cargo_id] = {}
        seeded_lane_y = lane_y.get(spec.cargo_id, spec.start_xyz[1]) + rng.uniform(-0.030, 0.030)
        for zone_id in WAREHOUSE_ZONE_ORDER:
            zone = WAREHOUSE_ZONES[zone_id]
            positions[spec.cargo_id][zone_id] = (
                float(zone.center_xy[0] + rng.uniform(-0.035, 0.035)),
                float(seeded_lane_y + rng.uniform(-0.006, 0.006)),
                float(spec.start_xyz[2]),
            )
    return positions


def mixed_cargo_zone_positions_for_seed(
    seed: int,
    specs: tuple[CargoSpec, ...] | None = None,
) -> dict[str, dict[str, tuple[float, float, float]]]:
    """Place mixed-fixture cargo in separated, collision-safe A/B lanes."""
    selected = specs
    # Keep this helper independently callable without recursively constructing
    # shaped specs; callers normally pass mixed_cargo_specs_for_seed's specs.
    if not selected:
        raise ValueError("mixed fixture requires cargo specs")
    rng = random.Random((int(seed) ^ 0x4D495845) & 0xFFFFFFFF)
    lanes = {"oak_plank_01": -1.0, "small_box_01": 0.45, "small_box_02": 1.10}
    positions: dict[str, dict[str, tuple[float, float, float]]] = {}
    for spec in selected:
        y = lanes.get(spec.cargo_id, spec.start_xyz[1]) + rng.uniform(-0.018, 0.018)
        positions[spec.cargo_id] = {
            zone_id: (
                float(WAREHOUSE_ZONES[zone_id].center_xy[0] + rng.uniform(-0.025, 0.025)),
                float(y + rng.uniform(-0.004, 0.004)),
                float(spec.dimensions_m[2] / 2.0),
            )
            for zone_id in WAREHOUSE_ZONE_ORDER
        }
    return positions


def cargo_specs_for_seed(
    seed: int,
    source_zone: str = "A",
    destination_zone: str = "B",
) -> tuple[CargoSpec, ...]:
    """Return deterministic procedural cargo for one unseen-layout seed.

    Safety lanes and collision identities remain stable for instrumentation;
    metric geometry, mass, yaw and coordinates vary, and roles remain empty
    until the runtime auction observes the scene.
    """
    source = str(source_zone).upper()
    destination = str(destination_zone).upper()
    warehouse_route_zones(source, destination)
    shape_rng = random.Random((int(seed) ^ 0x4F424A53) & 0xFFFFFFFF)
    shaped: list[CargoSpec] = []
    for spec in CARGO_SPECS:
        width = float(spec.dimensions_m[0]) * shape_rng.uniform(0.90, 1.10)
        length = float(spec.dimensions_m[1]) * shape_rng.uniform(0.88, 1.12)
        height = float(spec.dimensions_m[2]) * shape_rng.uniform(0.90, 1.10)
        shaped.append(replace(
            spec,
            dimensions_m=(width, length, height),
            mass_kg=float(spec.mass_kg) * shape_rng.uniform(0.80, 1.20),
            half_length_m=length / 2.0,
            start_xyz=(spec.start_xyz[0], spec.start_xyz[1], height / 2.0),
            goal_xyz=(spec.goal_xyz[0], spec.goal_xyz[1], height / 2.0),
            start_yaw_rad=shape_rng.uniform(-math.radians(7.0), math.radians(7.0)),
            carriers=(),
            scout="",
        ))
    shaped_specs = tuple(shaped)
    positions = cargo_zone_positions_for_seed(seed, shaped_specs)
    randomized: list[CargoSpec] = []
    for spec in shaped_specs:
        randomized.append(replace(
            spec,
            start_xyz=positions[spec.cargo_id][source],
            goal_xyz=positions[spec.cargo_id][destination],
        ))
    return tuple(randomized)


def _attrs(values) -> str:
    return " ".join(f"{float(v):.6f}" for v in values)


def _zone_geom(world: ET.Element, zone: ZoneSpec) -> None:
    ET.SubElement(
        world,
        "geom",
        {
            "name": f"warehouse_zone_{zone.zone_id.lower()}",
            "type": "box",
            "pos": _attrs((zone.center_xy[0], zone.center_xy[1], 0.002)),
            "size": _attrs((zone.half_extents_xy[0], zone.half_extents_xy[1], 0.004)),
            "rgba": zone.rgba,
            "contype": "0",
            "conaffinity": "0",
            "mass": "0",
            "group": "4",
        },
    )


def _cargo_body(spec: CargoSpec) -> ET.Element:
    body = ET.Element("body", {"name": spec.body_name, "pos": _attrs(spec.start_xyz)})
    ET.SubElement(body, "joint", {
        "name": spec.joint_name, "type": "free", "damping": ".08",
    })
    width, length, height = (float(v) for v in spec.dimensions_m)
    half_width, half_length, half_height = width / 2.0, length / 2.0, height / 2.0
    if spec.cargo_type == "plank":
        ET.SubElement(body, "geom", {
            "name": f"{spec.body_name}_geom", "type": "box",
            "size": _attrs((half_width, half_length, half_height)), "mass": f"{spec.mass_kg:.4f}",
            "rgba": ".56 .29 .09 1", "friction": "1.2 .20 .010",
            "contype": "1", "conaffinity": "3",
        })
        for x in (-0.020, 0.020):
            ET.SubElement(body, "geom", {
                "name": f"{spec.body_name}_grain_{'l' if x < 0 else 'r'}",
                "type": "box", "pos": _attrs((x, 0, .019)),
                "size": f".003 {half_length:.3f} .0015",
                "mass": "0", "rgba": ".78 .51 .20 1", "contype": "0", "conaffinity": "0",
            })
        ET.SubElement(body, "geom", {
            "name": f"{spec.body_name}_red_label", "type": "box", "pos": "0 0 .0205",
            "size": ".031 .025 .0015", "mass": "0", "rgba": ".85 .08 .06 1",
            "contype": "0", "conaffinity": "0",
        })
    elif spec.cargo_type == "pipe":
        ET.SubElement(body, "geom", {
            # A shallow box is the stable physical support footprint; the
            # massless cylinder below is the visible steel pipe. This prevents
            # an unattended pipe from rolling out of Zone A before selection.
            "name": f"{spec.body_name}_geom", "type": "box",
            "size": _attrs((half_width, half_length, half_height)), "mass": f"{spec.mass_kg:.4f}",
            "rgba": "0 0 0 0", "friction": "1.3 .20 .010",
            "contype": "1", "conaffinity": "3",
        })
        ET.SubElement(body, "geom", {
            "name": f"{spec.body_name}_visual", "type": "cylinder", "euler": "1.570796 0 0",
            "size": f"{min(half_width, half_height):.4f} {half_length:.3f}", "mass": "0", "rgba": ".55 .59 .63 1",
            "contype": "0", "conaffinity": "0",
        })
        ET.SubElement(body, "geom", {
            "name": f"{spec.body_name}_blue_band", "type": "cylinder", "euler": "1.570796 0 0",
            "size": ".028 .025", "mass": "0", "rgba": ".06 .24 .90 1",
            "contype": "0", "conaffinity": "0",
        })
    elif spec.cargo_type == "small_box":
        ET.SubElement(body, "geom", {
            "name": f"{spec.body_name}_geom", "type": "box",
            "size": _attrs((half_width, half_length, half_height)),
            "mass": f"{spec.mass_kg:.4f}", "rgba": ".20 .65 .70 1",
            "friction": "1.2 .20 .010", "contype": "1", "conaffinity": "3",
        })
    else:
        ET.SubElement(body, "geom", {
            "name": f"{spec.body_name}_geom", "type": "box",
            "size": _attrs((half_width, half_length, half_height)),
            "mass": f"{spec.mass_kg:.4f}", "rgba": ".45 .24 .08 1",
            "friction": "1.2 .20 .010", "contype": "1", "conaffinity": "3",
        })
        for y in (-.135, 0.0, .135):
            ET.SubElement(body, "geom", {
                "name": f"{spec.body_name}_slat_{str(y).replace('.', '_').replace('-', 'm')}",
                "type": "box", "pos": _attrs((.0285, y, 0)), "size": ".002 .025 .018",
                "mass": "0", "rgba": ".72 .43 .15 1", "contype": "0", "conaffinity": "0",
            })
        ET.SubElement(body, "geom", {
            "name": f"{spec.body_name}_yellow_label", "type": "box", "pos": ".030 0 .006",
            "size": ".0025 .032 .010", "mass": "0", "rgba": ".95 .72 .05 1",
            "contype": "0", "conaffinity": "0",
        })
    # Actor-visible identity fiducials.  These are visual-only plates on both
    # local +/-X faces, so a robot can decode cargo identity from its own RGB
    # camera without geom/body ids or privileged state.
    for side in (-1, 1):
        ET.SubElement(body, "geom", {
            "name": f"{spec.body_name}_tag_{'neg' if side < 0 else 'pos'}x",
            "type": "box", "pos": _attrs((side * (half_width + .002), 0, .060)),
            "size": ".050 .050 .0015", "euler": f"0 {side * math.pi / 2.0:.6f} 0",
            "mass": "0", "material": f"warehouse_tag_mat_{spec.cargo_id}",
            "contype": "0", "conaffinity": "0",
        })
    return body


def cargo_grasp_offset_y(spec: CargoSpec, side: str) -> float:
    """Return the actual cargo end position; no artificial handle exists."""
    return -spec.half_length_m if side == "left" else spec.half_length_m


def add_warehouse_mission_xml(
    root: ET.Element,
    *,
    specs: tuple[CargoSpec, ...] = CARGO_SPECS,
    carrier_ids: tuple[str, ...] = ("r1", "r2", "r3"),
    terrain: tuple[TerrainSpec, ...] = (),
    zones: Mapping[str, ZoneSpec] | None = None,
) -> ET.Element:
    zones = WAREHOUSE_ZONES if zones is None else zones
    world = root.find("worldbody")
    if world is None:
        raise ValueError("MuJoCo XML has no worldbody")
    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    tag_files = {
        "oak_plank_01": str(Path(__file__).resolve().parent / "assets/warehouse_tags/oak_plank_01.png"),
        "steel_pipe_01": str(Path(__file__).resolve().parent / "assets/warehouse_tags/steel_pipe_01.png"),
        "wood_crate_01": str(Path(__file__).resolve().parent / "assets/warehouse_tags/wood_crate_01.png"),
        "small_box_01": str(Path(__file__).resolve().parent / "assets/warehouse_tags/small_box_01.png"),
        "small_box_02": str(Path(__file__).resolve().parent / "assets/warehouse_tags/small_box_02.png"),
        "small_box_03": str(Path(__file__).resolve().parent / "assets/warehouse_tags/small_box_03.png"),
    }
    for spec in specs:
        texture_name = f"warehouse_tag_{spec.cargo_id}"
        material_name = f"warehouse_tag_mat_{spec.cargo_id}"
        if not any(node.get("name") == texture_name for node in asset):
            ET.SubElement(asset, "texture", {"name": texture_name, "type": "2d", "file": tag_files[spec.cargo_id]})
        if not any(node.get("name") == material_name for node in asset):
            ET.SubElement(asset, "material", {"name": material_name, "texture": texture_name,
                                                "texrepeat": "1 1", "texuniform": "false", "reflectance": "0"})
    owned = {spec.body_name for spec in specs} | {
        f"warehouse_zone_{z.lower()}" for z in zones
    } | {"cctv_warehouse"} | {item.terrain_id for item in terrain}
    for node in list(world):
        if node.get("name") in owned:
            world.remove(node)
    weld_names = {
        f"{rid}__{spec.cargo_id}_grasp"
        for spec in specs for rid in carrier_ids
    }
    for node in list(equality):
        if node.get("name") in weld_names:
            equality.remove(node)
    for zone in zones.values():
        _zone_geom(world, zone)
    ET.SubElement(world,"geom",{
        "name":"mixed_event_barrier","type":"box","pos":"-9 -9 .06",
        "size":".06 .09 .06","rgba":".9 .15 .1 1","contype":"1","conaffinity":"3"})
    center = tuple(sum(z.center_xy[i] for z in zones.values())/len(zones) for i in (0,1))
    camera_pos = "1.85 0 4.00" if zones is WAREHOUSE_ZONES else f"{center[0]} {center[1]} 6.0"
    ET.SubElement(world, "camera", {
        "name": "cctv_warehouse", "pos": camera_pos,
        "xyaxes": "1 0 0 0 1 0", "fovy": "58",
    })
    for item in terrain:
        is_mound = item.kind == "mound"
        ET.SubElement(world, "geom", {
            "name": item.terrain_id,
            "type": "ellipsoid" if is_mound else "box",
            "pos": _attrs((
                item.center_xy[0], item.center_xy[1],
                0.0 if is_mound else item.height_m / 2.0,
            )),
            "size": _attrs((
                item.half_extents_xy[0], item.half_extents_xy[1],
                item.height_m if is_mound else item.height_m / 2.0,
            )),
            "rgba": (
                ".34 .25 .12 1" if is_mound
                else (".93 .48 .06 1" if item.kind == "barrier" else ".20 .22 .26 1")
            ),
            "friction": "1.1 .08 .006",
            "contype": "1", "conaffinity": "3", "mass": "0", "group": "0",
        })
    for spec in specs:
        world.append(_cargo_body(spec))
        for rid in carrier_ids:
            ET.SubElement(equality, "weld", {
                "name": f"{rid}__{spec.cargo_id}_grasp",
                "body1": f"{rid}__gripper",
                "body2": spec.body_name,
                "active": "false", "solref": ".010 1", "solimp": ".90 .95 .001",
            })
    return root


def cargo_pose(data, model, spec: CargoSpec) -> dict[str, object]:
    import mujoco

    bid = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, spec.body_name))
    p = tuple(float(v) for v in data.xpos[bid])
    q = tuple(float(v) for v in data.xquat[bid])
    w, x, y, z = q
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return {"position": p, "quaternion": q, "yaw": yaw, "body_id": bid}


def evaluate_cargo_delivery(pose: Mapping[str, object], spec: CargoSpec) -> dict[str, object]:
    position = tuple(float(v) for v in pose["position"])
    yaw = float(pose.get("yaw") or 0.0)
    xy_error = math.hypot(position[0] - spec.goal_xyz[0], position[1] - spec.goal_xyz[1])
    raw_yaw_error = abs((yaw - spec.goal_yaw_rad + math.pi) % (2 * math.pi) - math.pi)
    # All current cargo is 180-degree symmetric around vertical; an end
    # swap does not change its destination orientation.
    yaw_error = min(raw_yaw_error, abs(math.pi - raw_yaw_error))
    height_error = abs(position[2] - spec.goal_xyz[2])
    success = (
        xy_error <= spec.position_tolerance_m
        and yaw_error <= spec.yaw_tolerance_rad
        and height_error <= 0.025
    )
    return {
        "success": bool(success), "position_error_m": xy_error,
        "yaw_error_rad": yaw_error, "height_error_m": height_error,
        "goal_xyz": spec.goal_xyz,
    }


def warehouse_manifest(
    specs: tuple[CargoSpec, ...] = CARGO_SPECS,
    zone_positions: Mapping[str, Mapping[str, tuple[float, float, float]]] | None = None,
    zones: Mapping[str, ZoneSpec] | None = None,
) -> dict[str, object]:
    return {
        "zones": {
            key: {
                "id": z.zone_id, "name": z.name, "color": z.color,
                "center_xy": z.center_xy, "half_extents_xy": z.half_extents_xy,
            }
            for key, z in (WAREHOUSE_ZONES if zones is None else zones).items()
        },
        "cargo": [
            {
                "cargo_id": s.cargo_id, "type": s.cargo_type,
                "label_color": s.label_color, "dimensions_m": s.dimensions_m,
                "mass_kg": s.mass_kg, "required_carriers": s.required_carriers,
                "start_xyz": s.start_xyz, "goal_xyz": s.goal_xyz,
                "zone_positions": dict(zone_positions.get(s.cargo_id, {})) if zone_positions else None,
                "carriers": s.carriers, "scout": s.scout,
                "observation_required": not bool(s.carriers),
            }
            for s in specs
        ],
    }


__all__ = [
    "ZoneSpec", "CargoSpec", "TerrainSpec", "WAREHOUSE_ZONES", "WAREHOUSE_ZONE_ORDER", "CARGO_SPECS", "CARGO_BY_ID",
    "add_warehouse_mission_xml", "cargo_pose", "evaluate_cargo_delivery",
    "warehouse_manifest", "warehouse_route_zones", "cargo_zone_positions_for_seed",
    "cargo_grasp_offset_y", "cargo_specs_for_seed", "terrain_specs_for_seed",
]
