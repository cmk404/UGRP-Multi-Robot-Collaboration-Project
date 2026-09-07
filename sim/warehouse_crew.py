"""Robot-local motion tasks advanced together on ONE shared physics clock.

The role assignment comes from the agents. This executor never changes it.
Each empty robot follows its own forward-facing path; the scout keeps working
while the two carriers synchronize only their coupled manipulation phases.
"""
from __future__ import annotations

import math
from dataclasses import replace

import numpy as np

from sim.adaptive_warehouse import plan_local_path
from sim.crew_navigation import ForwardPathController
from sim.crew_motion_metrics import CrewMotionRecorder
from sim.masterpi_dynamics_v2 import FORWARD_PATTERN
from sim.masterpi_production_v2 import SEARCH_POSE
from sim.warehouse_mission import WAREHOUSE_ZONES, cargo_pose

STOP = np.zeros(4, dtype=float)


def _zones(world):
    """Use the scene's metric zone registry when it supplies one."""
    return getattr(world, "warehouse_zones", None) or WAREHOUSE_ZONES


def _transport_frame(spec):
    delta = np.asarray(spec.goal_xyz[:2], dtype=float)-np.asarray(spec.start_xyz[:2], dtype=float)
    distance = float(np.linalg.norm(delta))
    if distance < 1e-6:
        raise ValueError("WAREHOUSE_ROUTE_HAS_ZERO_LENGTH")
    forward = delta/distance
    return forward, np.array((-forward[1], forward[0]))


def _navigation_bounds(world, target, margin=.65):
    if getattr(world, "warehouse_layout", "standard") != "arena":
        return (-.60, 3.40, -2.30, 2.30)
    points = [np.asarray(target, dtype=float)]
    for zone in _zones(world).values():
        center, half = np.asarray(zone.center_xy), np.asarray(zone.half_extents_xy)
        points.extend((center-half, center+half))
    points.extend(np.asarray(world.robot(rid).base_xyz()[:2], dtype=float)
                  for rid in world.robot_ids)
    points = np.asarray(points)
    return (float(points[:, 0].min()-margin), float(points[:, 0].max()+margin),
            float(points[:, 1].min()-margin), float(points[:, 1].max()+margin))


class WarehouseCrew:
    def __init__(self, world, spec, route):
        self.world, self.spec, self.route = world, spec, tuple(route)
        self.tasks = {}
        self.activities = {rid: "ready" for rid in world.robot_ids}
        self.started = float(world.data.time)
        self.last_sample = self.started - 1.0
        self.pending_views = []
        self.scout_phase = "outbound"
        self.scout_scan_started = None
        self.scout_scan_index = -1
        self.last_escort_target = None
        self.last_frame = self.started
        self.blocked_since = {}
        self.last_replan = {}
        self.failed = {}
        self.forward, self.lateral = _transport_frame(spec)
        if getattr(world, "warehouse_layout", "standard") == "arena":
            scout_offset = (np.asarray(world.robot(spec.scout).base_xyz()[:2])-np.asarray(spec.start_xyz[:2])
                            if spec.scout else self.lateral)
            self.scout_side = 1.0 if float(np.dot(scout_offset, self.lateral)) >= 0 else -1.0
        else:
            self.scout_side = 1.0 if not spec.scout or world.robot(spec.scout).base_xyz()[1] >= 0 else -1.0
        if getattr(world, "_crew_motion_recorder", None) is None:
            world._crew_motion_recorder = CrewMotionRecorder()
        self.recorder = world._crew_motion_recorder
        self.sample()

    def path(self, rid, target, *, include_peers=False):
        world = self.world
        # Initial formation paths account for parked/nonparticipant robots but
        # do not treat the fellow carrier's current pose as its final anchor.
        # A reactive replan includes every peer at its then-current position.
        excluded = () if include_peers else tuple(self.spec.carriers)
        obstacles = world._warehouse_navigation_observations(
            exclude_robots=excluded, exclude_robot=rid)
        # Slopes are obstacles to unloaded travel, the same conservative
        # contract used by the previous empty-return planner.
        obstacles = tuple(
            replace(o, traversable=False, cost_multiplier=math.inf)
            if o.kind == "mound" else
            # Runtime yielding forecasts a 0.34 m center separation.  Match
            # that with the .16 m planning footprint so a replan cannot return
            # a path that remains permanently blocked (.16 + .18 = .34).
            replace(o, half_extents_xy=(max(.18, o.half_extents_xy[0]),
                                        max(.18, o.half_extents_xy[1])))
            if o.kind == "peer" else o
            for o in obstacles
        )
        raw = plan_local_path(tuple(world.robot(rid).base_xyz()[:2]), target, obstacles,
                              footprint_xy=(.16, .16), resolution_m=.12,
                              bounds=_navigation_bounds(world, target))
        # Remove grid stair-steps only when the full swept chassis footprint
        # remains clear. This avoids needless quarter-turns at tiny grid edges.
        def clear(first, last):
            a, b = np.asarray(first), np.asarray(last)
            for u in np.linspace(0., 1., max(2, int(np.linalg.norm(b-a)/.025)+1)):
                point = a+(b-a)*u
                for obstacle in obstacles:
                    if obstacle.traversable:
                        continue
                    delta = np.abs(point-np.asarray(obstacle.center_xy))
                    smoothing_footprint = .16 if obstacle.kind == "peer" else .14
                    if all(delta < np.asarray(obstacle.half_extents_xy)+smoothing_footprint):
                        return False
            return True
        result = [raw[0]]
        index = 0
        while index < len(raw)-1:
            next_index = index+1
            for candidate in range(len(raw)-1, index, -1):
                if clear(raw[index], raw[candidate]):
                    next_index = candidate
                    break
            result.append(raw[next_index])
            index = next_index
        return tuple(result)

    def navigate(self, rid, target, activity, *, final_yaw=None, tolerance=.035):
        path = self.path(rid, target)
        self.tasks[rid] = ForwardPathController(path, final_yaw=final_yaw,
                                               tolerance_m=tolerance, max_speed=.55)
        self.activities[rid] = activity
        self.world._record_warehouse_phase("crew_path_started", robot_id=rid,
            cargo_id=self.spec.cargo_id, activity=activity,
            waypoints=[list(p) for p in path], direction_policy="face_then_forward",
            movement_mode="rotate_then_forward")
        return path

    def start_scout(self):
        zone = _zones(self.world)[self.route[0]]
        if getattr(self.world, "warehouse_layout", "standard") == "arena":
            lateral_clearance = max(zone.half_extents_xy)+.45
            target = (np.asarray(zone.center_xy)-self.forward*.70+
                      self.lateral*self.scout_side*lateral_clearance)
        else:
            direction = 1. if self.spec.goal_xyz[0] >= self.spec.start_xyz[0] else -1.
            target = (zone.center_xy[0]-direction*.70, self.scout_side*1.90)
        look_at = self.spec.start_xyz[:2]
        yaw = math.atan2(look_at[1] - target[1], look_at[0] - target[0])
        self.navigate(self.spec.scout, target, "inspect_pickup", final_yaw=yaw)

    def begin_handoff(self):
        self.scout_phase = "handoff"
        zones = _zones(self.world)
        zone = zones[self.route[-1]]
        if getattr(self.world, "warehouse_layout", "standard") == "arena":
            lateral_clearance = max(zone.half_extents_xy)+.45
            target = (np.asarray(zone.center_xy)-self.forward*.25+
                      self.lateral*self.scout_side*lateral_clearance)
        else:
            target = (zone.center_xy[0]-.25, self.scout_side*1.90)
        source = zones[self.route[0]].center_xy
        yaw = math.atan2(source[1]-target[1], source[0]-target[0])
        self.navigate(self.spec.scout, target, "inspect_next_cargo", final_yaw=yaw)

    def before_step(self):
        """Compute every managed command before the world's single mj_step."""
        world = self.world
        dt = float(world.model.opt.timestep)
        now = float(world.data.time)
        commands = {}
        for rid, task in tuple(self.tasks.items()):
            commands[rid] = task.update(world.robot(rid).base_xyz()[:2], world.robot(rid).base_rpy()[2], dt)
            if task.done and self.activities[rid] not in {"ready", "scanning", "observing", "watch_route"}:
                self.world._record_warehouse_phase("crew_path_complete", robot_id=rid,
                    cargo_id=self.spec.cargo_id, activity=self.activities[rid])
                if rid == self.spec.scout:
                    if self.scout_phase in {"escort", "staging_ahead"}:
                        self.scout_phase = "escort"
                        self.activities[rid] = "watch_route"
                    else:
                        self.activities[rid] = "scanning"
                        self.scout_scan_started = now
                        self.scout_scan_index = -1
                else:
                    self.activities[rid] = "ready"

        # Local yielding pauses only the conflicting navigator. A blocked
        # route is re-planned around the peer's CURRENT pose, never bypassed.
        for rid, command in tuple(commands.items()):
            speed = float(np.dot(command, FORWARD_PATTERN) / 4)
            if speed <= .01:
                continue
            robot = world.robot(rid)
            position = np.asarray(robot.base_xyz()[:2], dtype=float)
            yaw = robot.base_rpy()[2]
            velocity = np.array((math.cos(yaw), math.sin(yaw))) * speed * .34
            blocked = False
            for other_id in world.robot_ids:
                if other_id == rid:
                    continue
                other = world.robot(other_id)
                other_pos = np.asarray(other.base_xyz()[:2], dtype=float)
                other_cmd = commands.get(other_id, other.motor_command)
                other_speed = float(np.dot(other_cmd, FORWARD_PATTERN) / 4)
                other_yaw = other.base_rpy()[2]
                other_vel = np.array((math.cos(other_yaw), math.sin(other_yaw))) * other_speed * .34
                separation = other_pos-position
                relative = other_vel-velocity
                t = max(0., min(.9, -float(np.dot(separation, relative)) / max(1e-8, float(np.dot(relative, relative)))))
                predicted = float(np.linalg.norm(separation+t*relative))
                if predicted < .34 and (float(np.dot(separation, velocity)) > 0):
                    # A moving higher-priority peer goes first. A stationary
                    # peer is a physical obstacle regardless of ID.
                    if abs(other_speed) < .015 or rid > other_id or float(np.linalg.norm(separation)) < .34:
                        blocked = True
                        break
            if blocked:
                commands[rid] = STOP.copy()
                self.blocked_since.setdefault(rid, now)
                if now-self.blocked_since[rid] > .6 and now-self.last_replan.get(rid, -100) > 1.0:
                    self.last_replan[rid] = now
                    task = self.tasks[rid]
                    target = task.path[-1]
                    try:
                        path = self.path(rid, target, include_peers=True)
                        self.tasks[rid] = ForwardPathController(path, final_yaw=task.final_yaw,
                                                               tolerance_m=task.tolerance_m, max_speed=task.max_speed)
                        world._record_warehouse_phase("crew_yield_replanned", robot_id=rid,
                            cargo_id=self.spec.cargo_id, waypoints=[list(p) for p in path])
                    except ValueError:
                        pass  # Keep stopped; never ignore the obstacle.
            else:
                self.blocked_since.pop(rid, None)
        for rid, command in commands.items():
            world.robot(rid).set_motor_commands(command)

        if getattr(world, "_mixed_engine", None) is not None:
            return  # The nonparticipant owns an independent task, never a forced scout role.
        scout = self.spec.scout
        if self.activities[scout] == "scanning":
            elapsed = now - self.scout_scan_started
            index = int(elapsed / .8)
            pans = (1500, 1200, 1800, 1500)
            if index < len(pans):
                start = 1500 if index == 0 else pans[index-1]
                u = min(1.0, (elapsed - index*.8) / .55)
                pan = int(round(start + (pans[index]-start)*u))
                world.robot(scout).set_servo_pulses({**SEARCH_POSE, 6: pan})
                if u >= 1 and index != self.scout_scan_index:
                    self.scout_scan_index = index
                    self.pending_views.append((scout, self.scout_phase, index))
            elif self.scout_phase == "outbound":
                self.scout_phase = "staging_ahead"
                zone = _zones(world)[self.route[0]]
                if getattr(world, "warehouse_layout", "standard") == "arena":
                    lateral_clearance = max(zone.half_extents_xy)+.60
                    target = (np.asarray(zone.center_xy)+self.forward*.20+
                              self.lateral*self.scout_side*lateral_clearance)
                    heading = math.atan2(self.forward[1], self.forward[0])
                else:
                    direction = 1. if self.spec.goal_xyz[0] >= self.spec.start_xyz[0] else -1.
                    target = (zone.center_xy[0]+direction*.20, self.scout_side*2.05)
                    heading = 0. if direction > 0 else math.pi
                self.navigate(scout, target, "inspect_route", final_yaw=heading)
            else:
                self.activities[scout] = "observing"
        if self.scout_phase == "escort":
            carrier_state = self.activities[self.spec.carriers[0]]
            if carrier_state == "carry":
                payload = cargo_pose(world.data, world.model, self.spec)["position"]
                if math.dist(payload[:2], self.spec.goal_xyz[:2]) < .30:
                    self.begin_handoff()
                    return
                arena = getattr(world, "warehouse_layout", "standard") == "arena"
                if arena:
                    target = (np.asarray(payload[:2])+self.forward*.45+
                              self.lateral*self.scout_side*.60)
                else:
                    direction = 1. if self.spec.goal_xyz[0] >= self.spec.start_xyz[0] else -1.
                    target = (max(-.35, min(3.10, float(payload[0])+direction*.45)),
                              self.scout_side*2.05)
                if arena and self.last_escort_target is not None:
                    origin = np.asarray(self.spec.start_xyz[:2])
                    progress = max(float(np.dot(target-origin, self.forward)),
                                   float(np.dot(np.asarray(self.last_escort_target)-origin, self.forward)))
                    lateral = float(np.dot(target-origin, self.lateral))
                    target = origin+self.forward*progress+self.lateral*lateral
                elif not arena and self.last_escort_target is not None:
                    progress = max(direction*target[0], direction*self.last_escort_target[0])
                    target = (progress*direction, target[1])
                if self.last_escort_target is None or math.dist(target, self.last_escort_target) > .12:
                    try:
                        self.navigate(scout, target, "escort_route", tolerance=.045)
                        self.last_escort_target = target
                    except ValueError as exc:
                        self.failed[scout] = str(exc)
            elif carrier_state == "release":
                try:
                    self.begin_handoff()
                except ValueError as exc:
                    self.activities[scout] = "observing"
                    self.failed[scout] = str(exc)

    def after_step(self):
        self.sample()
        # Called AFTER physics_lock is released. Only the renderer's owner
        # thread executes OpenGL. Sensor inference is never in the physics lock.
        while self.pending_views:
            rid, phase, index = self.pending_views.pop(0)
            visible = []
            if self.world.renderer is not None:
                from sim.warehouse_observation import observe_robot
                oid = f"crew:{rid}:{float(self.world.data.time):.3f}"
                observation = observe_robot(self.world, rid, observation_id=oid, sensor_seed=self.world.seed)
                visible = [c["cargo_id"] for c in observation["cargo"]]
                ep = getattr(self.world, "_research_episode", None)
                if ep and not ep.get("closed"):
                    for c in observation["cargo"]:
                        ep["sensor_memory"][rid][c["cargo_id"]] = {
                            "cargo_id": c["cargo_id"], "label_color": c.get("label_color"),
                            "source_observation_id": oid,
                        }
            self.world._record_warehouse_phase("scout_camera_observed" if self.world.renderer is not None else "scout_camera_unavailable", robot_id=rid,
                cargo_id=self.spec.cargo_id, scouting_phase=phase, scan_index=index,
                visible_cargo_ids=visible, camera_available=self.world.renderer is not None,
                observation_source="robot_rgbd_fiducials")
        now = float(self.world.data.time)
        if self.world.frame_callback and now-self.last_frame >= .10:
            self.last_frame = now
            self.world.frame_callback()

    def sample(self):
        now = float(self.world.data.time)
        if now-self.last_sample < .099:
            return
        self.last_sample = now
        if getattr(self.recorder, "_last_time", None) is not None and now <= self.recorder._last_time:
            return
        self.recorder.observe(now, {rid: {
            "position": tuple(float(x) for x in self.world.robot(rid).base_xyz()),
            "yaw": float(self.world.robot(rid).base_rpy()[2]),
            "state": self.activities[rid],
        } for rid in self.world.robot_ids})

    def wait_for(self, robot_ids, max_sim_s=90.):
        deadline = float(self.world.data.time) + max_sim_s
        pause_baseline=getattr(self,"paused_seconds",0.)
        frames = 0
        while not all(self.tasks[rid].done for rid in robot_ids):
            if float(self.world.data.time) >= deadline+getattr(self,"paused_seconds",0.)-pause_baseline:
                details = {rid: {"state": self.tasks[rid].state, "target": self.tasks[rid].target,
                                 "xy": list(self.world.robot(rid).base_xyz()[:2])} for rid in robot_ids}
                raise RuntimeError(f"CREW_NAVIGATION_TIMEOUT: {details}")
            self.world._physics_step_for(self.world.robot("r1"))
            frames += 1
            if self.world.frame_callback and frames % 40 == 0:
                self.world.frame_callback()
        for rid in robot_ids:
            self.tasks.pop(rid, None)
            self.world.robot(rid).set_motor_commands(STOP)

    def finish(self, complete_scout=False):
        if complete_scout and self.scout_phase == "handoff":
            deadline = float(self.world.data.time)+20.
            while self.activities[self.spec.scout] != "observing":
                if float(self.world.data.time) >= deadline:
                    self.failed[self.spec.scout] = "SCOUT_OBSERVATION_TIMEOUT"
                    break
                self.world._physics_step_for(self.world.robot("r1"))
        for rid in self.tasks:
            self.world.robot(rid).set_motor_commands(STOP)
        self.sample()
        summary = self.recorder.summary()
        self.world._warehouse_crew_metrics = {
            "activities": dict(self.activities), "scout_phase": self.scout_phase,
            "navigation_failures": dict(self.failed),
            "robots": {rid: {k: v for k, v in row.items() if k != "moving_intervals"}
                       for rid, row in summary["robots"].items()},
            "concurrency": {k: v for k, v in summary["concurrency"].items() if k != "overlap_windows"},
            "diagnostics": summary["diagnostics"],
        }


def stage_crew(world, spec, reach_m):
    crew = world._warehouse_crew
    formation = world._warehouse_inward_formation(spec, reach_m)
    for rid in spec.carriers:
        target = (formation[rid]["x"], formation[rid]["y"])
        # Formation owns the chassis/arm geometry. Legacy layouts retain their
        # historical 0/pi frame while arena formations may supply any 2D yaw.
        path = crew.navigate(rid, target, "approach_grip",
                             final_yaw=formation[rid]["chassis_yaw"])
        world._record_warehouse_phase("warehouse_final_approach_path_planned",
            cargo_id=spec.cargo_id, robot_id=rid, waypoints=[list(p) for p in path],
            planner="independent_forward_path_shared_tick")
    if getattr(world, "_mixed_engine", None) is None:
        crew.start_scout()
    world._record_warehouse_phase("crew_parallel_started", robot_ids=list(world.robot_ids), cargo_id=spec.cargo_id)
    crew.wait_for(spec.carriers)
    return world._warehouse_inward_formation(spec, reach_m)
