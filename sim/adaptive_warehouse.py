"""Runtime observation and planning for unfamiliar warehouse missions.

The planner deliberately consumes measured scene state rather than ``CargoSpec``
role hints.  Geometry limits and safety margins remain deterministic; object
order, carrier assignment, route choice and goal revisions do not.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import itertools
import math
from typing import Mapping, Sequence


@dataclass(frozen=True)
class RobotObservation:
    robot_id: str
    position_xy: tuple[float, float]


@dataclass(frozen=True)
class CargoObservation:
    cargo_id: str
    position_xy: tuple[float, float]
    yaw_rad: float
    dimensions_m: tuple[float, float, float]
    mass_kg: float
    required_carriers: int = 2

    @property
    def half_length_m(self) -> float:
        return max(self.dimensions_m) / 2.0


@dataclass(frozen=True)
class TerrainObservation:
    terrain_id: str
    kind: str
    center_xy: tuple[float, float]
    half_extents_xy: tuple[float, float]
    height_m: float
    traversable: bool
    cost_multiplier: float


@dataclass(frozen=True)
class RolePlan:
    carriers: tuple[str, str]
    scout: str
    endpoint_xy: Mapping[str, tuple[float, float]]
    bids: Mapping[str, Mapping[str, float]]
    total_cost: float


def cargo_endpoints(cargo: CargoObservation) -> tuple[tuple[float, float], tuple[float, float]]:
    """Infer opposing grasp points from the observed major axis."""
    axis = (-math.sin(cargo.yaw_rad), math.cos(cargo.yaw_rad))
    half = cargo.half_length_m
    center = cargo.position_xy
    return (
        (center[0] - axis[0] * half, center[1] - axis[1] * half),
        (center[0] + axis[0] * half, center[1] + axis[1] * half),
    )


def auction_roles(
    robots: Sequence[RobotObservation],
    cargo: CargoObservation,
    *,
    prior_carrier_jobs: Mapping[str, int] | None = None,
) -> RolePlan:
    """Award both endpoints to the lowest-cost distinct robots.

    Each robot bids independently for both measured endpoints.  A small load
    term prevents one memorized pair from winning every object when travel
    distances are similar.
    """
    if cargo.required_carriers != 2 or len(robots) < 3:
        raise ValueError("adaptive warehouse currently requires two carriers and one scout")
    jobs = {r.robot_id: int((prior_carrier_jobs or {}).get(r.robot_id, 0)) for r in robots}
    endpoints = cargo_endpoints(cargo)
    bids: dict[str, dict[str, float]] = {}
    for robot in robots:
        bids[robot.robot_id] = {
            "end_0": math.dist(robot.position_xy, endpoints[0]) + jobs[robot.robot_id] * 0.08,
            "end_1": math.dist(robot.position_xy, endpoints[1]) + jobs[robot.robot_id] * 0.08,
            "scout": math.dist(robot.position_xy, cargo.position_xy) * 0.25,
        }
    candidates: list[tuple[float, str, str, str]] = []
    ids = tuple(robot.robot_id for robot in robots)
    for left, right in itertools.permutations(ids, 2):
        scout = next(rid for rid in ids if rid not in {left, right})
        cost = bids[left]["end_0"] + bids[right]["end_1"] + bids[scout]["scout"]
        candidates.append((cost, left, right, scout))
    cost, left, right, scout = min(candidates)
    return RolePlan(
        carriers=(left, right),
        scout=scout,
        endpoint_xy={left: endpoints[0], right: endpoints[1]},
        bids=bids,
        total_cost=float(cost),
    )


def shortest_zone_route(
    source: str,
    destination: str,
    graph: Mapping[str, Mapping[str, float]],
) -> tuple[str, ...]:
    """Dijkstra route over the currently observed traversable zone graph."""
    source = str(source).upper()
    destination = str(destination).upper()
    if source == destination:
        return (source,)
    if source not in graph or destination not in graph:
        raise ValueError("unknown warehouse zone")
    queue: list[tuple[float, tuple[str, ...], str]] = [(0.0, (source,), source)]
    best = {source: 0.0}
    while queue:
        cost, path, node = heapq.heappop(queue)
        if node == destination:
            return path
        if cost > best.get(node, math.inf):
            continue
        for neighbor, edge_cost in graph[node].items():
            value = float(edge_cost)
            if not math.isfinite(value) or value < 0.0:
                continue
            new_cost = cost + value
            if new_cost + 1e-12 < best.get(neighbor, math.inf):
                best[neighbor] = new_cost
                heapq.heappush(queue, (new_cost, path + (neighbor,), neighbor))
    raise ValueError(f"no traversable route from {source} to {destination}")


def segment_terrain(
    start_xy: Sequence[float],
    end_xy: Sequence[float],
    terrain: Sequence[TerrainObservation],
    *,
    margin_m: float = 0.05,
) -> tuple[TerrainObservation, ...]:
    """Return terrain features intersecting an axis-aligned carry corridor."""
    x0, x1 = sorted((float(start_xy[0]), float(end_xy[0])))
    y0, y1 = sorted((float(start_xy[1]), float(end_xy[1])))
    hits = []
    for item in terrain:
        cx, cy = item.center_xy
        hx, hy = item.half_extents_xy
        if (
            x1 + margin_m >= cx - hx
            and x0 - margin_m <= cx + hx
            and y1 + margin_m >= cy - hy
            and y0 - margin_m <= cy + hy
        ):
            hits.append(item)
    return tuple(hits)


def plan_local_path(
    start_xy: Sequence[float],
    goal_xy: Sequence[float],
    terrain: Sequence[TerrainObservation],
    *,
    footprint_xy: tuple[float, float],
    axis_costs: tuple[float, float] = (1.0, 1.0),
    resolution_m: float = 0.08,
    bounds: tuple[float, float, float, float] = (0.42, 2.68, -1.82, 1.82),
) -> tuple[tuple[float, float], ...]:
    """Plan an axis-aligned collision-free payload path on an inflated grid.

    Non-traversable features are hard obstacles. Traversable terrain remains
    available with its measured cost multiplier, so a short gentle mound can
    be crossed while a large one is naturally bypassed.
    """
    resolution = max(0.04, float(resolution_m))
    x_cost, y_cost = (max(0.01, float(v)) for v in axis_costs)
    min_x, max_x, min_y, max_y = (float(v) for v in bounds)
    start = (float(start_xy[0]), float(start_xy[1]))
    goal = (float(goal_xy[0]), float(goal_xy[1]))

    def cell(point: Sequence[float]) -> tuple[int, int]:
        return (
            int(round((float(point[0]) - min_x) / resolution)),
            int(round((float(point[1]) - min_y) / resolution)),
        )

    def point(index: tuple[int, int]) -> tuple[float, float]:
        return (min_x + index[0] * resolution, min_y + index[1] * resolution)

    max_ix = int(math.floor((max_x - min_x) / resolution))
    max_iy = int(math.floor((max_y - min_y) / resolution))
    start_cell, goal_cell = cell(start), cell(goal)
    if not (0 <= start_cell[0] <= max_ix and 0 <= start_cell[1] <= max_iy):
        raise ValueError("path start is outside warehouse planning bounds")
    if not (0 <= goal_cell[0] <= max_ix and 0 <= goal_cell[1] <= max_iy):
        raise ValueError("path goal is outside warehouse planning bounds")

    inflate_x, inflate_y = (max(0.0, float(v)) for v in footprint_xy)

    def terrain_cost(index: tuple[int, int]) -> float:
        if index in {start_cell, goal_cell}:
            return 1.0
        px, py = point(index)
        multiplier = 1.0
        for item in terrain:
            cx, cy = item.center_xy
            hx, hy = item.half_extents_xy
            inside = (
                abs(px - cx) <= hx + inflate_x
                and abs(py - cy) <= hy + inflate_y
            )
            if not inside:
                continue
            if not item.traversable:
                expanded_x = max(1e-9, hx + inflate_x)
                expanded_y = max(1e-9, hy + inflate_y)
                start_depth = max(
                    abs(start[0] - cx) / expanded_x,
                    abs(start[1] - cy) / expanded_y,
                )
                point_depth = max(
                    abs(px - cx) / expanded_x,
                    abs(py - cy) / expanded_y,
                )
                # A live robot may begin inside a conservative inflation halo
                # after releasing cargo. Permit only cells that monotonically
                # leave that halo; never let the path move deeper through it.
                if start_depth <= 1.0 and point_depth + 1e-9 >= start_depth:
                    continue
                goal_depth = max(
                    abs(goal[0] - cx) / expanded_x,
                    abs(goal[1] - cy) / expanded_y,
                )
                # Likewise, a conservative halo may cover the measured goal
                # beside a small movable object. Allow an approach from the
                # halo boundary down to, but never deeper than, that goal.
                if goal_depth <= 1.0 and point_depth + 1e-9 >= goal_depth:
                    continue
                return math.inf
            multiplier = max(multiplier, float(item.cost_multiplier))
        return multiplier

    frontier: list[tuple[float, float, tuple[int, int]]] = [(0.0, 0.0, start_cell)]
    previous: dict[tuple[int, int], tuple[int, int] | None] = {start_cell: None}
    best: dict[tuple[int, int], float] = {start_cell: 0.0}
    while frontier:
        _priority, cost, current = heapq.heappop(frontier)
        if current == goal_cell:
            break
        if cost > best.get(current, math.inf):
            continue
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbor = (current[0] + dx, current[1] + dy)
            if not (0 <= neighbor[0] <= max_ix and 0 <= neighbor[1] <= max_iy):
                continue
            multiplier = terrain_cost(neighbor)
            if not math.isfinite(multiplier):
                continue
            axis_cost = x_cost if dx else y_cost
            new_cost = cost + resolution * multiplier * axis_cost
            if new_cost + 1e-12 >= best.get(neighbor, math.inf):
                continue
            best[neighbor] = new_cost
            previous[neighbor] = current
            heuristic = resolution * (
                abs(neighbor[0] - goal_cell[0]) * x_cost
                + abs(neighbor[1] - goal_cell[1]) * y_cost
            )
            heapq.heappush(frontier, (new_cost + heuristic, new_cost, neighbor))
    if goal_cell not in previous:
        raise ValueError("no collision-free payload path")

    cells = []
    cursor: tuple[int, int] | None = goal_cell
    while cursor is not None:
        cells.append(cursor)
        cursor = previous[cursor]
    cells.reverse()
    raw = [point(index) for index in cells]
    raw[0] = start
    raw[-1] = goal
    compressed = [raw[0]]
    previous_direction = None
    for index in range(1, len(raw)):
        dx = raw[index][0] - raw[index - 1][0]
        dy = raw[index][1] - raw[index - 1][1]
        direction = (0 if abs(dx) < 1e-9 else (1 if dx > 0 else -1),
                     0 if abs(dy) < 1e-9 else (1 if dy > 0 else -1))
        if previous_direction is not None and direction != previous_direction:
            compressed.append(raw[index - 1])
        previous_direction = direction
    compressed.append(raw[-1])
    return tuple(compressed)


def plan_monotone_visibility_path(
    start_xy: Sequence[float], goal_xy: Sequence[float],
    terrain: Sequence[TerrainObservation], *,
    footprint_xy: tuple[float, float],
    progress_axis: tuple[float, float],
    bounds: tuple[float, float, float, float],
    clearance_m: float = 0.012,
) -> tuple[tuple[float, float], ...]:
    """Plan forward/lateral segments around inflated rectangular obstacles."""
    start = (float(start_xy[0]), float(start_xy[1]))
    goal = (float(goal_xy[0]), float(goal_xy[1]))
    min_x, max_x, min_y, max_y = (float(value) for value in bounds)
    if not (min_x <= start[0] <= max_x and min_y <= start[1] <= max_y):
        raise ValueError("path start is outside warehouse planning bounds")
    if not (min_x <= goal[0] <= max_x and min_y <= goal[1] <= max_y):
        raise ValueError("path goal is outside warehouse planning bounds")
    length = math.hypot(float(progress_axis[0]), float(progress_axis[1]))
    if length < 1e-9:
        raise ValueError("progress_axis must be nonzero")
    progress = (float(progress_axis[0]) / length, float(progress_axis[1]) / length)
    if ((goal[0] - start[0]) * progress[0]
            + (goal[1] - start[1]) * progress[1] < -1e-9):
        raise ValueError("goal is behind monotone progress axis")

    inflate_x, inflate_y = (max(0.0, float(value)) for value in footprint_xy)
    rectangles = []
    for item in terrain:
        if item.traversable:
            continue
        cx, cy = item.center_xy
        hx, hy = item.half_extents_xy
        rectangles.append((
            cx - hx - inflate_x, cx + hx + inflate_x,
            cy - hy - inflate_y, cy + hy + inflate_y,
        ))

    nodes = [start, goal]
    pad = max(0.001, float(clearance_m))
    for x0, x1, y0, y1 in rectangles:
        nodes.extend(((x0-pad, y0-pad), (x0-pad, y1+pad),
                      (x1+pad, y0-pad), (x1+pad, y1+pad)))
    nodes = nodes[:2] + [
        point for point in nodes[2:]
        if min_x <= point[0] <= max_x and min_y <= point[1] <= max_y
    ]

    def segment_hits(a, b, rectangle):
        x0, x1, y0, y1 = rectangle
        dx, dy = b[0]-a[0], b[1]-a[1]
        low, high = 0.0, 1.0
        for origin, direction, lower, upper in (
            (a[0], dx, x0, x1), (a[1], dy, y0, y1),
        ):
            if abs(direction) < 1e-12:
                if not lower <= origin <= upper:
                    return False
                continue
            enter, leave = (lower-origin)/direction, (upper-origin)/direction
            if enter > leave:
                enter, leave = leave, enter
            low, high = max(low, enter), min(high, leave)
            if low > high:
                return False
        return low <= high

    def visible(first, last):
        projected = ((last[0]-first[0])*progress[0]
                     + (last[1]-first[1])*progress[1])
        return projected >= -1e-9 and not any(
            segment_hits(first, last, rectangle) for rectangle in rectangles
        )

    frontier = [(0.0, 0)]
    costs, previous = {0: 0.0}, {0: None}
    while frontier:
        cost, current = heapq.heappop(frontier)
        if cost > costs.get(current, math.inf):
            continue
        if current == 1:
            break
        for neighbor in range(len(nodes)):
            if neighbor == current or not visible(nodes[current], nodes[neighbor]):
                continue
            new_cost = cost + math.dist(nodes[current], nodes[neighbor])
            if new_cost + 1e-12 >= costs.get(neighbor, math.inf):
                continue
            costs[neighbor], previous[neighbor] = new_cost, current
            heapq.heappush(frontier, (new_cost, neighbor))
    if 1 not in previous:
        raise ValueError("no monotone collision-free payload path")
    path, cursor = [], 1
    while cursor is not None:
        path.append(nodes[cursor])
        cursor = previous[cursor]
    return tuple(reversed(path))


__all__ = [
    "CargoObservation", "RobotObservation", "RolePlan", "TerrainObservation",
    "auction_roles", "cargo_endpoints", "plan_local_path", "plan_monotone_visibility_path",
    "segment_terrain", "shortest_zone_route",
]
