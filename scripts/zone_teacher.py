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


def _wrap(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi


def plan_path(start, goal, bounds, discs, *, grid=GRID_M, radius=ROBOT_RADIUS_M, budget=40000):
    """8-connected grid A* for a disc robot; discs are (x, y, r) keep-outs.

    Only when no path exists: a keep-out whose clearance margin (not the
    obstacle itself) already overlaps the robot, e.g. a box it just dropped
    next to itself, walls in the start cell, so that margin is released.
    Releasing it on every replan would let the robot plough through boxes.
    """
    path = _astar(start, goal, bounds, discs, grid, radius, budget)
    if path is None:
        freed = [(x, y, r) for x, y, r in discs if not r <= math.hypot(start[0]-x, start[1]-y) < r+radius]
        if len(freed) != len(discs):
            path = _astar(start, goal, bounds, freed, grid, radius, budget)
    return path


def _astar(start, goal, bounds, discs, grid, radius, budget):
    x0, x1, y0, y1 = bounds
    def free(p):
        if not (x0+radius <= p[0] <= x1-radius and y0+radius <= p[1] <= y1-radius):
            return False
        return all(math.hypot(p[0]-x, p[1]-y) >= r+radius for x, y, r in discs)
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
        self.attempts, self.outcome, self.path = 0, None, None
        self.log('assign', self.rid, now, job=job['job_id'])

    @property
    def busy(self):
        return self.phase not in ('idle', 'done', 'failed')

    def _set(self, phase, now, **detail):
        self.phase, self.phase_started, self.path = phase, now, None
        self.log('phase', self.rid, now, phase=phase, job=self.job and self.job['job_id'], **detail)

    def _finish(self, outcome, now, **detail):
        self.port.hold(now)
        self.outcome = outcome
        self._set('done' if outcome == 'placed_by_teacher' else 'failed', now, outcome=outcome, **detail)

    # --- motion primitives ---
    def _drive_to(self, goal, heading, now, discs, *, carrying, tol=.03):
        x, y, yaw = self.pose()
        dist = math.hypot(goal[0]-x, goal[1]-y)
        if dist <= tol:
            return True
        if self.path is None or now >= self.replan_at or self.path_goal != tuple(goal):
            self.path = plan_path((x, y), goal, self.map['bounds_m'], discs,
                                  radius=CARRY_RADIUS_M if carrying else ROBOT_RADIUS_M)
            self.path_goal, self.replan_at = tuple(goal), now + 1.
            if self.path is None:
                self.port.hold(now)
                return False
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

    def tick(self, now, discs_for):
        done = self.arm.tick(now)
        if not self.busy:
            return
        job = self.job
        if self.phase in ('to_box', 'carry') and now - self.phase_started > DRIVE_PHASE_LIMIT_S:
            if self.phase == 'carry':
                self.arm.queue({1: OPEN}, now, duration=.3)
                self.arm.queue(FOLDED, now)
            self._finish('teacher_path_blocked', now)
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
                self.arm.queue({1: CLOSED}, now, duration=.5, settle=.4)
                self._set('grasp', now)
        elif self.phase == 'grasp':
            if done:
                hover, _ = self._grasp_targets(self.box_xyz(job['box_body'])[:2])
                self.arm.queue({**hover, 1: CLOSED}, now, duration=.8, settle=.6)
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
        self.next_tick = 0.

    def discs_for(self, robot, *, exclude=None, carrying=False):
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
            if other is robot:
                continue
            x, y, _ = other.pose()
            discs.append((x, y, PEER_CLEARANCE_M))
        return discs

    def tick(self, now):
        if now + 1e-9 < self.next_tick:
            for robot in self.robots.values():
                robot.arm.tick(now)
            return
        self.next_tick = now + CONTROL_S
        for robot in self.robots.values():
            robot.tick(now, self.discs_for)
