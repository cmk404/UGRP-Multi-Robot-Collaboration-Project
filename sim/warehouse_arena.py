"""Deterministic square-zone arena fixtures for warehouse transfers.

Unlike the legacy checkpoint lanes, these fixtures are compact 2-D arenas:
three separated square rooms surround open, navigable gaps containing static
barriers.  Layout randomness is entirely derived from the supplied seed.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
import random
from typing import Mapping

from sim.warehouse_mission import CargoSpec, TerrainSpec, ZoneSpec, mixed_cargo_specs_for_seed


_ZONE_CENTERS = {
    "A": (-1.15, -0.75),
    "B": (1.15, -0.75),
    "C": (0.0, 1.35),
}
_ZONE_COLORS = {
    "A": ("blue", ".10 .35 .95 .28"),
    "B": ("green", ".12 .82 .30 .28"),
    "C": ("yellow", ".95 .72 .08 .30"),
}
_ZONE_HALF_EXTENT = 0.41


@dataclass(frozen=True)
class ArenaSpec:
    seed: int
    zones: Mapping[str, ZoneSpec]
    cargo_specs: tuple[CargoSpec, ...]
    zone_positions: Mapping[str, Mapping[str, tuple[float, float, float]]]
    terrain: tuple[TerrainSpec, ...]
    source_zone: str
    destination_zone: str
    robot_start_poses: Mapping[str, tuple[float, float, float]]


def _zones() -> dict[str, ZoneSpec]:
    return {
        zone_id: ZoneSpec(
            zone_id, f"arena_zone_{zone_id.lower()}", _ZONE_COLORS[zone_id][0],
            center, (_ZONE_HALF_EXTENT, _ZONE_HALF_EXTENT),
            _ZONE_COLORS[zone_id][1],
        )
        for zone_id, center in _ZONE_CENTERS.items()
    }


def _zone_positions(
    rng: random.Random,
    cargo: tuple[CargoSpec, ...],
    source: str,
    destination: str,
) -> dict[str, dict[str, tuple[float, float, float]]]:
    source_center = _ZONE_CENTERS[source]
    destination_center = _ZONE_CENTERS[destination]
    heading = math.atan2(
        destination_center[1] - source_center[1],
        destination_center[0] - source_center[0],
    )
    direction = (math.cos(heading), math.sin(heading))
    normal = (-direction[1], direction[0])
    # The plank stays central. Boxes occupy the two rear corners relative to
    # travel, preserving both cargo separation and distinct final approaches
    # inside an 0.82 m square without shrinking any physical body.
    docks = (
        (0.0, 0.0),
        (-0.25 * direction[0] + 0.23 * normal[0],
         -0.25 * direction[1] + 0.23 * normal[1]),
        (-0.25 * direction[0] - 0.23 * normal[0],
         -0.25 * direction[1] - 0.23 * normal[1]),
    )
    result: dict[str, dict[str, tuple[float, float, float]]] = {}
    for spec, (dx, dy) in zip(cargo, docks):
        result[spec.cargo_id] = {}
        for zone_id, (cx, cy) in _ZONE_CENTERS.items():
            result[spec.cargo_id][zone_id] = (
                cx + dx,
                cy + dy,
                spec.dimensions_m[2] / 2.0,
            )
    return result


def _barriers(rng: random.Random, blocked_edge: tuple[str, str]) -> tuple[TerrainSpec, ...]:
    edges = (("A", "B"), ("A", "C"), ("B", "C"))
    other_edges = [edge for edge in edges if set(edge) != set(blocked_edge)]
    selected = (blocked_edge, rng.choice(other_edges))
    barriers = []
    for index, (left, right) in enumerate(selected, 1):
        ax, ay = _ZONE_CENTERS[left]
        bx, by = _ZONE_CENTERS[right]
        # Axis-aligned compact blocks fit wholly in the inter-zone gaps.  A
        # loaded 0.7 m-wide team cannot take the centre line, yet has over a
        # metre of open arena on either side for a smooth detour.
        half_x = rng.uniform(0.070, 0.085)
        half_y = rng.uniform(0.105, 0.130)
        barriers.append(TerrainSpec(
            terrain_id=f"arena_barrier_{index}_{left.lower()}{right.lower()}",
            kind="barrier",
            center_xy=(
                (ax + bx) / 2.0 + rng.uniform(-0.045, 0.045),
                (ay + by) / 2.0 + rng.uniform(-0.035, 0.035),
            ),
            half_extents_xy=(half_x, half_y),
            height_m=rng.uniform(0.11, 0.16),
            traversable=False,
            cost_multiplier=math.inf,
        ))
    return tuple(barriers)


def arena_for_seed(seed: int) -> ArenaSpec:
    """Build one deterministic, compact three-zone transfer arena."""
    value = int(seed)
    rng = random.Random((value ^ 0x4152454E) & 0xFFFFFFFF)
    source, destination = rng.sample(("A", "B", "C"), 2)
    base_cargo = mixed_cargo_specs_for_seed(value, "A", "B")
    positions = _zone_positions(rng, base_cargo, source, destination)
    cargo_items = []
    for spec in base_cargo:
        start = positions[spec.cargo_id][source]
        goal = positions[spec.cargo_id][destination]
        # A plank's local long axis is Y.  Align local X with the actual
        # seeded dock-to-dock route so its two grasp endpoints remain on
        # opposite sides of travel and both arms can approach inward.
        route_yaw = (
            math.atan2(goal[1] - start[1], goal[0] - start[0])
            if spec.required_carriers == 2 else 0.0
        )
        cargo_items.append(replace(
            spec,
            start_xyz=start,
            goal_xyz=goal,
            start_yaw_rad=route_yaw,
            goal_yaw_rad=route_yaw,
            carriers=(),
            scout="",
        ))
    cargo = tuple(cargo_items)

    cx, cy = _ZONE_CENTERS[source]
    cargo_centroid = tuple(
        sum(positions[spec.cargo_id][source][axis] for spec in cargo) / len(cargo)
        for axis in (0, 1)
    )
    starts = {}
    for rid, dx in zip(("r1", "r2", "r3"), (-0.60, 0.0, 0.60)):
        x, y = cx + dx, cy - 0.85
        # Every camera begins pointed toward the cargo dock centroid.
        starts[rid] = (
            x, y,
            math.atan2(cargo_centroid[1] - y, cargo_centroid[0] - x),
        )

    return ArenaSpec(
        seed=value,
        zones=_zones(),
        cargo_specs=cargo,
        zone_positions=positions,
        terrain=_barriers(rng, (source, destination)),
        source_zone=source,
        destination_zone=destination,
        robot_start_poses=starts,
    )


def camera_team_arena_for_seed(seed: int) -> ArenaSpec:
    """Three independent robots, boxes, and cyclic zone transfers for RGB navigation."""
    value = int(seed)
    rng = random.Random((value ^ 0x43414D54) & 0xFFFFFFFF)
    zones = _zones()
    zone_order = ("A", "B", "C")
    # Seed changes which physical box begins in each zone without changing the
    # cyclic A->B->C->A task or any object's physical dimensions.
    offset = rng.randrange(3)
    ids = ("small_box_01", "small_box_02", "small_box_03")
    source_by_id = {cargo_id: zone_order[(index + offset) % 3]
                    for index, cargo_id in enumerate(ids)}
    next_zone = {"A": "B", "B": "C", "C": "A"}
    dimensions_by_id = {
        "small_box_01": (0.034, 0.040, 0.032),
        "small_box_02": (0.036, 0.038, 0.034),
        "small_box_03": (0.034, 0.040, 0.032),
    }
    specs = []
    positions: dict[str, dict[str, tuple[float, float, float]]] = {}
    starts: dict[str, tuple[float, float, float]] = {}
    for rid, cargo_id in zip(("r1", "r2", "r3"), ids):
        source = source_by_id[cargo_id]
        destination = next_zone[source]
        dimensions = dimensions_by_id[cargo_id]
        sx, sy = _ZONE_CENTERS[source]
        gx, gy = _ZONE_CENTERS[destination]
        # Small seeded offsets preserve a clear marker face and remain well
        # inside the 0.82 m square.
        tangent = (-math.sin(math.atan2(gy - sy, gx - sx)),
                   math.cos(math.atan2(gy - sy, gx - sx)))
        jitter = rng.uniform(-0.055, 0.055)
        start = (sx + tangent[0] * jitter, sy + tangent[1] * jitter, dimensions[2] / 2.0)
        goal = (gx + tangent[0] * jitter, gy + tangent[1] * jitter, dimensions[2] / 2.0)
        positions[cargo_id] = {
            zone_id: ((start if zone_id == source else goal) if zone_id in {source, destination}
                      else (_ZONE_CENTERS[zone_id][0], _ZONE_CENTERS[zone_id][1], dimensions[2] / 2.0))
            for zone_id in zone_order
        }
        specs.append(CargoSpec(
            cargo_id, "small_box", "cyan", f"warehouse_{cargo_id}",
            f"warehouse_{cargo_id}_free", dimensions, 0.03, start, goal,
            dimensions[1] / 2.0, position_tolerance_m=0.12, required_carriers=1,
            carriers=(), scout="",
        ))
        heading = math.atan2(gy - sy, gx - sx)
        # Each robot starts 0.48 m behind its own box with the tagged +/-X face
        # squarely visible. The three zones make pairwise chassis separation >1 m.
        starts[rid] = (start[0] - 0.48 * math.cos(heading),
                       start[1] - 0.48 * math.sin(heading), heading)

    # Reuse the established seeded, immovable inter-zone barrier generator.
    blocked = (zone_order[offset], next_zone[zone_order[offset]])
    return ArenaSpec(
        seed=value, zones=zones, cargo_specs=tuple(specs), zone_positions=positions,
        terrain=_barriers(rng, blocked), source_zone="A", destination_zone="B",
        robot_start_poses=starts,
    )


__all__ = ["ArenaSpec", "arena_for_seed", "camera_team_arena_for_seed"]
