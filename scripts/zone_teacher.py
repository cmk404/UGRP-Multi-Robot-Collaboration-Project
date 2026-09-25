"""Ground-truth TEACHER executor for the zone benchmark (never a student result).

Each robot job is "move box X into zone slot S". The teacher reads simulator
truth (robot base pose, box pose) to drive and to aim the calibrated arm IK,
and physically grasps with the gripper: no weld or attachment. Its outcomes are
reported as teacher-executor conditions, separate from RGB-skill success.
Robots only ever receive issued-command receipts from it, never these poses.
"""
from __future__ import annotations

import heapq
import math

import numpy as np

from harness.static_keepouts import inside_rect, keepout_rects, passage_zones, rect_distance
from harness.visual_arm import solve_grip_ik, tool_pose

FOLDED = {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}
GRASP_RADIUS_M = .155
GRASP_Z_M, HOVER_Z_M = .024, .095
OPEN, CLOSED = 2000, 1500
CONTROL_S = .1
GRID_M = .05
ROBOT_RADIUS_M = .17
CARRY_RADIUS_M = .21
BOX_CLEARANCE_M = .06
PEER_CLEARANCE_M = .14
# A drive phase (to the box, or carrying to the slot) that has not arrived
# within this SIM time ends the job as teacher_path_blocked.
DRIVE_PHASE_LIMIT_S = 120.
# Maps with interior walls: trips detour through doors and corridors and may
# queue at them (corr-G5-plan_first-s11: a carry to B through the corridor
# queued 42 s and was stopped 1.3 m short of the slot at 120 s).
ROUTE_DRIVE_PHASE_LIMIT_S = 240.
# A drive goal inside another box's keep-out can never be reached (a box was
# pushed onto the pregrasp or slot approach spot); stop after this long.
GOAL_OCCUPIED_LIMIT_S = 10.
# Two driving robots can block each other (one parked beside the other's goal,
# or head-on in a lane between box columns) and both wait forever. A robot that
# has had no path this long, while a path exists without peers, asks the peers
# on that path to step aside: an idle peer, or a blocked driving peer of lower
# priority (larger robot id). A yield lasts at most YIELD_LIMIT_S.
YIELD_AFTER_S = 2.
YIELD_LIMIT_S = 20.
DRIVE_PHASES = ('to_box', 'carry')
# Single-lane passages (maps with interior walls and declared passages only).
# Two robots that stand still inside the same passage zone for this long are
# in a standoff; the one of lower physical priority backs out of the lane.
# The rule reads only current physical state (positions, stillness, a box
# lifted off the floor next to the robot) and the static map: never a peer's
# job, goal, path or claim, so it acts identically in every coordination mode.
PASSAGE_STANDOFF_S = 2.
# If the standoff persists this long (the lower-priority robot found no spot
# to back out to), the other robot backs out instead.
PASSAGE_FALLBACK_S = 6.
# Clearance of a backed-out robot from the lane centre line and from the peer.
PASSAGE_CLEAR_M = PEER_CLEARANCE_M + CARRY_RADIUS_M + GRID_M
# A robot counts as still while it stays within this distance.
STILL_M = .02
LIFTED_Z_M = .045
# A peer in these phases holds (or is closing on) the box.
TAKEN_PHASES = ('grasp', 'lift', 'carry', 'align_slot', 'release', 'retract', 'back_off')


def _wrap(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi


class PeerDisc(tuple):
    """(x, y, r) keep-out around another robot. It is never released: when the
    robot already overlaps it, it only stops the robot getting any closer."""


def _no_closer(start, disc, radius):
    x, y, r = disc
    d = math.hypot(start[0]-x, start[1]-y)
    return (x, y, d-radius-1e-3) if d < r+radius else (x, y, r)


def _wall_limits(start, rects, radius):
    """Static rectangles with the clearance a disc robot keeps from them.

    A robot already closer than its radius (e.g. it started carrying next to a
    wall) may not get any closer, like an overlapped peer.
    """
    out = []
    for rect in rects:
        d = rect_distance(start, rect)
        out.append((rect, radius if d >= radius else max(d-1e-3, 0.)))
    return out


def plan_path(start, goal, bounds, discs, *, grid=GRID_M, radius=ROBOT_RADIUS_M, budget=40000, rects=()):
    """8-connected grid A* for a disc robot; discs are (x, y, r) keep-outs.

    rects: static (cx, cy, hx, hy, yaw) keep-outs from the map (interior walls),
    kept at the robot radius. Without rects the search is unchanged.

    Only when no path exists: a box keep-out whose clearance margin (not the
    obstacle itself) already overlaps the robot, e.g. a box it just dropped
    next to itself, walls in the start cell, so that margin is released.
    Releasing it on every replan would let the robot plough through boxes.
    Peer robots (PeerDisc) are never released; an overlapped one only forbids
    getting closer, so two robots that met head-on do not push each other.
    """
    peers = [_no_closer(start, d, radius) for d in discs if isinstance(d, PeerDisc)]
    boxes = [d for d in discs if not isinstance(d, PeerDisc)]
    walls = _wall_limits(start, rects, radius)
    path = _astar(start, goal, bounds, boxes + peers, grid, radius, budget, walls)
    overlapped = [(x, y, r) for x, y, r in boxes if math.hypot(start[0]-x, start[1]-y) < r+radius]
    if path is None and overlapped:
        # Back out of the overlapped box margin to the nearest free cell first;
        # ploughing straight through pushed a box along for minutes (ZC1 gate).
        spot = retreat_point(start, [(x, y) for x, y, _ in overlapped], bounds, boxes + peers,
                             clear=max(r for *_, r in overlapped)+radius+grid, grid=grid, radius=radius,
                             rects=rects)
        rest = spot and _astar(spot, goal, bounds, boxes + peers, grid, radius, budget,
                               _wall_limits(spot, rects, radius))
        if rest:
            path = [spot] + rest
    if path is None:
        freed = [(x, y, r) for x, y, r in boxes if not r <= math.hypot(start[0]-x, start[1]-y) < r+radius]
        if len(freed) != len(boxes):
            path = _astar(start, goal, bounds, freed + peers, grid, radius, budget, walls)
    return path


def retreat_point(start, avoid, bounds, discs, *, clear, grid=GRID_M, radius=ROBOT_RADIUS_M, budget=20000,
                  rects=()):
    """Nearest reachable grid point at least `clear` from every point of `avoid`.

    Keep-outs the robot already overlaps only forbid getting closer to them.
    """
    x0, x1, y0, y1 = bounds
    discs = [_no_closer(start, d, radius) for d in discs]
    walls = _wall_limits(start, rects, radius)
    def free(p):
        if not (x0+radius <= p[0] <= x1-radius and y0+radius <= p[1] <= y1-radius):
            return False
        return (all(math.hypot(p[0]-x, p[1]-y) >= r+radius for x, y, r in discs)
                and all(rect_distance(p, rect) >= limit for rect, limit in walls))
    def point(c):
        return (x0+c[0]*grid, y0+c[1]*grid)
    start_c = (round((start[0]-x0)/grid), round((start[1]-y0)/grid))
    frontier = [(0., start_c)]
    cost = {start_c: 0.}
    steps = 0
    while frontier and steps < budget:
        steps += 1
        d, current = heapq.heappop(frontier)
        p = point(current)
        if current != start_c and min(math.hypot(p[0]-ax, p[1]-ay) for ax, ay in avoid) >= clear:
            return p
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                nxt = (current[0]+dx, current[1]+dy)
                if (not dx and not dy) or not free(point(nxt)):
                    continue
                new = d + math.hypot(dx, dy)*grid
                if new < cost.get(nxt, math.inf):
                    cost[nxt] = new
                    heapq.heappush(frontier, (new, nxt))
    return None


def _astar(start, goal, bounds, discs, grid, radius, budget, walls=()):
    x0, x1, y0, y1 = bounds
    def free(p):
        if not (x0+radius <= p[0] <= x1-radius and y0+radius <= p[1] <= y1-radius):
            return False
        return (all(math.hypot(p[0]-x, p[1]-y) >= r+radius for x, y, r in discs)
                and all(rect_distance(p, rect) >= limit for rect, limit in walls))
    def cell(p):
        return (round((p[0]-x0)/grid), round((p[1]-y0)/grid))
    def point(c):
        return (x0+c[0]*grid, y0+c[1]*grid)
    start_c, goal_c = cell(start), cell(goal)
    goal_p = point(goal_c)
    frontier = [(0., start_c)]
    came, cost = {start_c: None}, {start_c: 0.}
    steps = 0
    while frontier and steps < budget:
        steps += 1
        _, current = heapq.heappop(frontier)
        if current == goal_c:
            break
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if not dx and not dy:
                    continue
                nxt = (current[0]+dx, current[1]+dy)
                p = point(nxt)
                if nxt != goal_c and not free(p):
                    continue
                new = cost[current] + math.hypot(dx, dy)*grid
                if new < cost.get(nxt, math.inf):
                    cost[nxt] = new
                    came[nxt] = current
                    heapq.heappush(frontier, (new + math.hypot(p[0]-goal_p[0], p[1]-goal_p[1]), nxt))
    if goal_c not in came:
        return None
    path, c = [], goal_c
    while c is not None:
        path.append(point(c)); c = came[c]
    path.reverse()
    path[-1] = tuple(goal)
    return path


class ArmSequence:
    """Interpolated issued servo targets; the teacher waits for completion."""
    def __init__(self, port, commanded):
        self.port, self.commanded = port, dict(commanded)
        self.events, self.until = [], 0.

    def queue(self, targets, now, *, duration=None, settle=.1):
        start = max(now, self.until)
        delta = max((abs(targets[s]-self.commanded[s]) for s in targets), default=0)
        duration = max(.2, delta/600.) if duration is None else duration
        count = max(4, math.ceil(duration/.05))
        begin = dict(self.commanded)
        for i in range(1, count+1):
            u = i/count; ease = u*u*(3-2*u)
            for servo, end in targets.items():
                self.events.append((start+i*duration/count, servo,
                                    round(begin[servo]+ease*(end-begin[servo]))))
        self.commanded.update(targets)
        self.until = start + duration + settle

    def tick(self, now):
        due = [e for e in self.events if e[0] <= now+1e-9]
        self.events = [e for e in self.events if e[0] > now+1e-9]
        for _, servo, pulse in due:
            self.port.apply({'kind': 'look', 'pan_pulse': pulse} if servo == 6 else
                            {'kind': 'arm', 'servo_id': servo, 'pulse': pulse}, now)
        return now >= self.until and not self.events


class TeacherRobot:
    """One robot's job: navigate, align, grasp, carry, align, place, back off."""
    def __init__(self, rid, world, port, static_map, log):
        self.rid, self.world, self.port, self.map, self.log = rid, world, port, static_map, log
        self.arm = ArmSequence(port, FOLDED)
        self.job = None
        self.phase = 'idle'
        self.path, self.path_goal = None, None
        self.replan_at = 0.
        self.phase_started = 0.
        self.attempts = 0
        self.stuck_since = None
        self.last_xy = None
        self.outcome = None
        self.blocked_since = None
        self.yield_req = None
        self.team = {self.rid: self}  # the executor links all robots
        self.goal_occupied = False
        # Static map keep-outs; empty for maps with only perimeter walls (v1).
        self.rects = keepout_rects(static_map)
        self.passages = passage_zones(static_map)
        self.passage_yield = None
        self.still_from, self.still_since = None, None
        self.wait = None

    # --- ground truth (teacher only) ---
    def pose(self):
        robot = self.world.robot(self.rid)
        xyz = robot.base_xyz()
        return float(xyz[0]), float(xyz[1]), float(robot.base_rpy()[2])

    def box_xyz(self, body):
        return np.array(self.world.data.body(body).xpos, float)

    def to_base(self, xy):
        x, y, yaw = self.pose()
        dx, dy = xy[0]-x, xy[1]-y
        return (math.cos(yaw)*dx + math.sin(yaw)*dy, -math.sin(yaw)*dx + math.cos(yaw)*dy)

    # --- job interface ---
    def assign(self, job, now):
        """job: {'job_id', 'box_body', 'slot_xy'}; issued, not a success claim."""
        if self.phase not in ('idle', 'done', 'failed'):
            raise RuntimeError(f'{self.rid} is busy')
        self.job, self.phase, self.phase_started = dict(job), 'to_box', now
        self.assigned_at = now
        self.attempts, self.outcome, self.path = 0, None, None
        self.log('assign', self.rid, now, job=job['job_id'])

    @property
    def busy(self):
        return self.phase not in ('idle', 'done', 'failed')

    def _grip(self):
        """Diagnostic injection (output-only record): the gripper stays open, so
        every grasp attempt of this job really fails and ends as
        grasp_failed_by_teacher after the normal retries."""
        return OPEN if self.job.get('inject') == 'grasp_stays_open' else CLOSED

    def _taken_by_peer(self, body):
        for other in self.team.values():
            if other is self or not other.job or other.job['box_body'] != body:
                continue
            if other.phase in TAKEN_PHASES or other.outcome == 'placed_by_teacher':
                return True
            if other.phase == 'align_box' and self.phase == 'to_box':
                return True
            # Both still driving to the same box would block each other at its
            # one pregrasp spot: the robot sent first keeps it (tie: lower id).
            if other.phase == 'to_box' and self.phase == 'to_box' and (
                    (other.assigned_at, other.rid) < (self.assigned_at, self.rid)):
                return True
        return False

    def _set(self, phase, now, **detail):
        self.phase, self.phase_started, self.path = phase, now, None
        self.blocked_since = None
        self.goal_occupied = False
        self.log('phase', self.rid, now, phase=phase, job=self.job and self.job['job_id'], **detail)

    def _finish(self, outcome, now, **detail):
        self.port.hold(now)
        self.outcome = outcome
        self._set('done' if outcome == 'placed_by_teacher' else 'failed', now, outcome=outcome, **detail)

    # --- motion primitives ---
    def radius(self, carrying):
        """Planning disc: the robot, or robot plus carried cargo. A job may
        declare a larger carried-cargo clearance (carry_radius_m)."""
        if not carrying:
            return ROBOT_RADIUS_M
        return max(CARRY_RADIUS_M, float((self.job or {}).get('carry_radius_m', 0.)))

    def _drive_to(self, goal, heading, now, discs, *, carrying, tol=.03):
        x, y, yaw = self.pose()
        dist = math.hypot(goal[0]-x, goal[1]-y)
        if dist <= tol:
            return True
        if self.path is None or now >= self.replan_at or self.path_goal != tuple(goal):
            self.path = plan_path((x, y), goal, self.map['bounds_m'], discs,
                                  radius=self.radius(carrying), rects=self.rects)
            self.path_goal, self.replan_at = tuple(goal), now + 1.
            if self.path is None:
                if self.blocked_since is None:
                    self.blocked_since = now
                self.port.hold(now)
                if (now - self.blocked_since >= GOAL_OCCUPIED_LIMIT_S and any(
                        not isinstance(d, PeerDisc) and math.hypot(goal[0]-d[0], goal[1]-d[1]) < d[2]+self.radius(carrying)
                        for d in discs)):
                    self.goal_occupied = True
                return False
            self.blocked_since = None
        while len(self.path) > 1 and math.hypot(self.path[0][0]-x, self.path[0][1]-y) < .12:
            self.path.pop(0)
        target = self.path[0] if len(self.path) > 1 else goal
        wx, wy = target[0]-x, target[1]-y
        norm = max(math.hypot(wx, wy), 1e-9)
        speed = min(1., dist/.25)
        vx, vy = wx/norm*speed, wy/norm*speed
        want = heading if dist < .6 else math.atan2(wy, wx)
        err = _wrap(want - yaw)
        fwd = math.cos(yaw)*vx + math.sin(yaw)*vy
        left = -math.sin(yaw)*vx + math.cos(yaw)*vy
        scale = 1. if abs(err) < .5 else .3
        self.port.apply({'kind': 'mecanum', 'forward': float(np.clip(.15*fwd*scale, -.05, .15)),
                         'left': float(np.clip(.10*left*scale, -.10, .10)),
                         'turn': float(np.clip(.6*err, -.15, .15)), 'duration_s': .15}, now)
        return False

    def _align(self, target_xy, now):
        """Final fine alignment: target at (GRASP_RADIUS, 0) in the base frame, facing east."""
        x, y, yaw = self.pose()
        bx, by = self.to_base(target_xy)
        ex, ey, eyaw = bx-GRASP_RADIUS_M, by, _wrap(0. - yaw)
        if abs(ex) < .006 and abs(ey) < .006 and abs(eyaw) < .03:
            self.port.hold(now)
            return True
        self.port.apply({'kind': 'mecanum', 'forward': float(np.clip(.9*ex, -.05, .08)),
                         'left': float(np.clip(.9*ey, -.06, .06)),
                         'turn': float(np.clip(.8*eyaw, -.10, .10)), 'duration_s': .12}, now)
        return False

    def _grasp_targets(self, xy):
        bx, by = self.to_base(xy)
        grasp = solve_grip_ik(bx, by, GRASP_Z_M, -90)
        pitch = tool_pose(grasp).pitch_deg
        hover = solve_grip_ik(bx, by, HOVER_Z_M, pitch)
        path = [solve_grip_ik(bx, by, float(z), pitch) for z in np.linspace(HOVER_Z_M, GRASP_Z_M, 8)[1:]]
        return hover, path

    @property
    def carrying(self):
        return self.busy and self.phase in ('lift', 'carry', 'align_slot', 'release')

    def request_yield(self, requester, avoid, now):
        self.yield_req = {'by': requester, 'job': requester.job, 'phase': requester.phase,
                          'avoid': avoid, 'until': now + YIELD_LIMIT_S, 'target': None, 'started': now}
        self.path, self.blocked_since = None, None
        self.log('yield', self.rid, now, to=requester.rid, own_phase=self.phase)

    def _yield(self, now, discs_for):
        """Step off a higher-priority peer's path; True while yielding."""
        req = self.yield_req
        other = req['by']
        if now >= req['until'] or other.job is not req['job'] or other.phase != req['phase']:
            self.yield_req, self.path, self.blocked_since = None, None, None
            self.port.hold(now)
            self.log('yield_end', self.rid, now, to=other.rid, held_s=round(now-req['started'], 2))
            return False
        carrying = self.carrying
        discs = discs_for(self, exclude=self.job['box_body'] if carrying else None, carrying=carrying)
        radius = self.radius(carrying)
        if req['target'] is None:
            clear = PEER_CLEARANCE_M + other.radius(other.phase == 'carry') + GRID_M
            req['target'] = retreat_point(self.pose()[:2], req['avoid'], self.map['bounds_m'], discs,
                                          clear=clear, radius=radius, rects=self.rects)
            self.log('yield_target', self.rid, now, to=other.rid,
                     target=req['target'] and [round(v, 3) for v in req['target']])
            if req['target'] is None:
                self.yield_req = None
                return False
        if self._drive_to(req['target'], self.pose()[2], now, discs, carrying=carrying, tol=.05):
            self.port.hold(now)
        return True

    def request_passage_yield(self, other, passage, now, *, fallback=False):
        """Back out of a single-lane passage for a peer standing still in it.
        Physical inputs only: the peer's current position and the static lane."""
        pid, core, zone = passage
        cx, cy, hx, hy, _ = core
        along_x = zone[2]-core[2] > zone[3]-core[3]
        half = zone[2] if along_x else zone[3]
        count = max(2, math.ceil(2*half/GRID_M))
        line = [(cx-half+2*half*i/count, cy) if along_x else (cx, cy-half+2*half*i/count) for i in range(count+1)]
        self.passage_yield = {'from': other, 'pid': pid, 'zone': zone, 'avoid': line, 'target': None,
                              'until': now + YIELD_LIMIT_S, 'started': now}
        self.path, self.blocked_since = None, None
        self.log('passage_yield', self.rid, now, to=other.rid, passage=pid, own_phase=self.phase,
                 fallback=fallback)

    def _passage_yield(self, now, discs_for):
        req = self.passage_yield
        other = req['from']
        reason = ('limit' if now >= req['until'] else
                  'peer_left_zone' if not inside_rect(other.pose()[:2], req['zone']) else None)
        if reason is None and req['target'] is None:
            carrying = self.carrying
            discs = discs_for(self, exclude=self.job['box_body'] if carrying and self.job else None,
                              carrying=carrying)
            req['target'] = retreat_point(self.pose()[:2], req['avoid'] + [other.pose()[:2]], self.map['bounds_m'],
                                          discs, clear=PASSAGE_CLEAR_M, radius=self.radius(carrying),
                                          rects=self.rects)
            self.log('passage_yield_target', self.rid, now, to=other.rid, passage=req['pid'],
                     target=req['target'] and [round(v, 3) for v in req['target']])
            if req['target'] is None:
                reason = 'no_spot'
        if reason:
            self.passage_yield, self.path, self.blocked_since = None, None, None
            self.port.hold(now)
            self.log('passage_yield_end', self.rid, now, to=other.rid, passage=req['pid'], reason=reason,
                     held_s=round(now-req['started'], 2))
            return False
        carrying = self.carrying
        discs = discs_for(self, exclude=self.job['box_body'] if carrying and self.job else None, carrying=carrying)
        if self._drive_to(req['target'], self.pose()[2], now, discs, carrying=carrying, tol=.05):
            self.port.hold(now)
        return True

    def tick(self, now, discs_for):
        done = self.arm.tick(now)
        if self.passage_yield and self._passage_yield(now, discs_for):
            return
        if self.yield_req and self._yield(now, discs_for):
            return
        if not self.busy:
            return
        job = self.job
        if self.phase in ('to_box', 'carry') and self.goal_occupied:
            if self.phase == 'carry':
                self.arm.queue({1: OPEN}, now, duration=.3)
                self.arm.queue(FOLDED, now)
            self._finish('teacher_path_blocked', now, goal_occupied=True)
            return
        if self.phase in ('to_box', 'carry') and now - self.phase_started > (
                ROUTE_DRIVE_PHASE_LIMIT_S if self.rects else DRIVE_PHASE_LIMIT_S):
            if self.phase == 'carry':
                self.arm.queue({1: OPEN}, now, duration=.3)
                self.arm.queue(FOLDED, now)
            self._finish('teacher_path_blocked', now)
            return
        if self.phase in ('to_box', 'align_box') and self._taken_by_peer(job['box_body']):
            # No-communication runs can send two robots to one box: the one that
            # arrives second stops (teacher truth decides; robots only get the
            # "stopped before finishing" receipt).
            self.port.hold(now)
            self._finish('box_taken_by_peer', now)
            return
        if self.phase == 'to_box':
            box = self.box_xyz(job['box_body'])
            goal = (box[0]-GRASP_RADIUS_M-.10, box[1])
            # The target box stays a keep-out: the pregrasp goal is outside its
            # margin, and a robot coming from the east must go around it.
            if self._drive_to(goal, 0., now, discs_for(self, carrying=False),
                              carrying=False, tol=.04):
                self._set('align_box', now)
        elif self.phase == 'align_box':
            if self._align(self.box_xyz(job['box_body'])[:2], now) or now-self.phase_started > 12:
                hover, path = self._grasp_targets(self.box_xyz(job['box_body'])[:2])
                self.arm.queue({**hover, 1: OPEN}, now)
                for pose in path:
                    self.arm.queue(pose, now, duration=.12, settle=0.)
                self.arm.queue({1: self._grip()}, now, duration=.5, settle=.4)
                self._set('grasp', now)
        elif self.phase == 'grasp':
            if done:
                hover, _ = self._grasp_targets(self.box_xyz(job['box_body'])[:2])
                self.arm.queue({**hover, 1: self._grip()}, now, duration=.8, settle=.6)
                self._set('lift', now)
        elif self.phase == 'lift':
            if done:
                z = self.box_xyz(job['box_body'])[2]
                if z > .045:
                    self._set('carry', now, box_z_m=round(float(z), 4))
                elif self.attempts < 2:
                    self.attempts += 1
                    self.arm.queue({1: OPEN}, now, duration=.3)
                    self.arm.queue(FOLDED, now)
                    self._set('to_box', now, retry=self.attempts, box_z_m=round(float(z), 4))
                else:
                    self.arm.queue({1: OPEN}, now, duration=.3)
                    self.arm.queue(FOLDED, now)
                    self._finish('grasp_failed_by_teacher', now)
        elif self.phase == 'carry':
            sx, sy = job['slot_xy']
            goal = (sx-GRASP_RADIUS_M-.08, sy)
            if self.box_xyz(job['box_body'])[2] < .03:
                self._finish('dropped_in_transit', now)
            elif self._drive_to(goal, 0., now, discs_for(self, exclude=job['box_body'], carrying=True),
                                carrying=True, tol=.04):
                self._set('align_slot', now)
        elif self.phase == 'align_slot':
            if self._align(job['slot_xy'], now) or now-self.phase_started > 12:
                _, path = self._grasp_targets(job['slot_xy'])
                for pose in path:
                    self.arm.queue({**pose, 1: CLOSED}, now, duration=.12, settle=0.)
                self.arm.queue({1: OPEN}, now, duration=.4, settle=.4)
                self._set('release', now)
        elif self.phase == 'release':
            if done:
                hover, _ = self._grasp_targets(job['slot_xy'])
                self.arm.queue({**hover, 1: OPEN}, now, duration=.5)
                self.arm.queue(FOLDED, now)
                self._set('retract', now)
        elif self.phase == 'retract':
            if done:
                self._set('back_off', now)
        elif self.phase == 'back_off':
            x, _, _ = self.pose()
            sx, _ = job['slot_xy']
            if x < sx - .45 or now - self.phase_started > 8:
                self._finish('placed_by_teacher', now)
            else:
                self.port.apply({'kind': 'mecanum', 'forward': -.05, 'left': 0., 'turn': 0.,
                                 'duration_s': .15}, now)


class ZoneTeacherExecutor:
    """Ticks every robot's teacher at CONTROL_S on the one physics clock."""
    def __init__(self, world, ports, static_map, objects, log):
        self.world, self.objects, self.log = world, objects, log
        self.robots = {rid: TeacherRobot(rid, world, port, static_map, log) for rid, port in ports.items()}
        for robot in self.robots.values():
            robot.team = self.robots
        self.next_tick = 0.

    def discs_for(self, robot, *, exclude=None, carrying=False, peers=True):
        discs = []
        held = {r.job['box_body'] for r in self.robots.values()
                if r.job and r.phase in ('lift', 'carry', 'align_slot', 'release')}
        for item in self.objects.values():
            body = item['body_name']
            if body == exclude or body in held:
                continue
            p = self.world.data.body(body).xpos
            discs.append((float(p[0]), float(p[1]), BOX_CLEARANCE_M))
        for other in self.robots.values():
            if other is robot or not peers:
                continue
            x, y, _ = other.pose()
            discs.append(PeerDisc((x, y, PEER_CLEARANCE_M)))
        return discs

    def lifted_box_near(self, robot):
        """Physical: a box above the floor within reach of the robot (it holds one)."""
        x, y, _ = robot.pose()
        for item in self.objects.values():
            p = self.world.data.body(item['body_name']).xpos
            if float(p[2]) > LIFTED_Z_M and math.hypot(float(p[0])-x, float(p[1])-y) < .30:
                return True
        return False

    def _zone_of(self, robot):
        xy = robot.pose()[:2]
        return next((p for p in robot.passages if inside_rect(xy, p[2])), None)

    def track_motion(self, now):
        """Stillness from the robots' own poses; waits inside passage zones are logged."""
        for robot in self.robots.values():
            xy = robot.pose()[:2]
            if robot.still_from is None or math.dist(xy, robot.still_from) > STILL_M:
                robot.still_from, robot.still_since = xy, now
            zone = self._zone_of(robot)
            waiting = zone is not None and robot.busy and (robot.blocked_since is not None
                                                          or robot.passage_yield or robot.yield_req)
            if waiting and robot.wait is None:
                robot.wait = (zone[0], now)
            elif robot.wait and (not waiting or zone[0] != robot.wait[0]):
                self.log('passage_wait', robot.rid, now, passage=robot.wait[0],
                         wait_s=round(now-robot.wait[1], 2))
                robot.wait = (zone[0], now) if waiting else None

    def resolve_passages(self, now):
        """Standoff in a single-lane passage: the robot of lower physical priority
        backs out. Priority: inside the passage opening, then holding a lifted
        box, then the lower robot id. A robot without a job of its own always
        backs out. Robots grasping or placing are fixed obstacles here."""
        for passage in (next(iter(self.robots.values())).passages if self.robots else ()):
            pid, core, zone = passage
            if any(r.passage_yield for r in self.robots.values() if inside_rect(r.pose()[:2], zone)):
                continue
            still = [r for r in sorted(self.robots.values(), key=lambda r: r.rid)
                     if r.still_since is not None and now - r.still_since >= PASSAGE_STANDOFF_S
                     and not r.yield_req and (not r.busy or r.phase in DRIVE_PHASES)
                     and inside_rect(r.pose()[:2], zone)]
            if len(still) < 2:
                continue
            def key(r):
                return (0 if inside_rect(r.pose()[:2], core) else 1, 0 if self.lifted_box_near(r) else 1, r.rid)
            order = sorted(still, key=key)
            idle = [r for r in order if not r.busy]
            fallback = False
            if idle:
                yielder = idle[-1]
                other = next(r for r in order if r is not yielder)
            elif min(now - r.still_since for r in order) >= PASSAGE_FALLBACK_S:
                # The lower-priority robot found no spot or the standoff outlived
                # its back-out: the other robot tries instead.
                yielder, other, fallback = order[0], order[-1], True
            else:
                yielder, other = order[-1], order[0]
            self.log('passage_standoff', yielder.rid, now, passage=pid, peer=other.rid, fallback=fallback,
                     keys={r.rid: list(key(r)) for r in order})
            yielder.request_passage_yield(other, passage, now, fallback=fallback)

    def _same_passage(self, a, b):
        za, zb = self._zone_of(a), self._zone_of(b)
        return za is not None and zb is not None and za[0] == zb[0]

    def resolve_blocks(self, now):
        """Ask peers standing on a blocked robot's peer-free path to step aside."""
        for robot in sorted(self.robots.values(), key=lambda r: r.rid):
            if (robot.blocked_since is None or now - robot.blocked_since < YIELD_AFTER_S
                    or robot.yield_req or robot.phase not in DRIVE_PHASES or robot.path_goal is None):
                continue
            if robot.passage_yield:
                continue
            carrying = robot.phase == 'carry'
            radius = robot.radius(carrying)
            discs = self.discs_for(robot, exclude=robot.job['box_body'] if carrying else None,
                                   carrying=carrying, peers=False)
            avoid = plan_path(robot.pose()[:2], robot.path_goal, robot.map['bounds_m'], discs, radius=radius,
                              rects=robot.rects)
            if avoid is None:
                continue
            clear = PEER_CLEARANCE_M + radius
            for other in sorted(self.robots.values(), key=lambda r: r.rid):
                if other is robot or other.yield_req:
                    continue
                if robot.passages and (other.passage_yield or self._same_passage(robot, other)):
                    # Standoffs inside one single-lane passage follow resolve_passages.
                    continue
                ox, oy, _ = other.pose()
                if min(math.hypot(px-ox, py-oy) for px, py in avoid) >= clear:
                    continue
                if not other.busy or (other.phase in DRIVE_PHASES and other.blocked_since is not None
                                      and robot.rid < other.rid):
                    other.request_yield(robot, avoid, now)

    def tick(self, now):
        if now + 1e-9 < self.next_tick:
            for robot in self.robots.values():
                robot.arm.tick(now)
            return
        self.next_tick = now + CONTROL_S
        routes = any(r.passages for r in self.robots.values())
        if routes:
            self.track_motion(now)
        for robot in self.robots.values():
            robot.tick(now, self.discs_for)
        if routes:
            self.resolve_passages(now)
        self.resolve_blocks(now)
