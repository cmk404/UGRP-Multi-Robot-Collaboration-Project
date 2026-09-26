"""Team formation motion targets for zone team jobs (TEACHER geometry, pure).

Phase A1 of the zone team-carry work (2026-09-25). Given an item pose (or a
reference item trajectory) and the team's role assignment, this computes each
carrier's base target, approach command, arm IK targets and the carry command
of the virtual-structure controller. It reads no simulator state: the caller
passes poses. The teacher (A2) passes ground-truth poses, which the teacher
exception allows; a student must not receive these targets.

The control law is the one probed in PR #164
(``scripts.cargo_formation_teacher.FormationTeacher``): each carrier tracks
``reference_item_pose * approach_base(role)`` with shared feed-forward, a
bounded P correction and ONE saturation scale for the whole team; the
reference pauses while any carrier is off by more than the pause tolerance.
Constants and the reference trajectory are imported from there so the two
cannot drift apart. There is no weld and no equality constraint anywhere in
this path, and it never goes through the old beam weld activation in
``sim/multi_masterpi_production.py``.
"""
from __future__ import annotations

import math

import numpy as np

from harness.visual_arm import solve_grip_ik, tool_pose
from harness.zone_goal_v2 import fills_formation, formations
from harness.zone_team_footprint import grasps, station_offset
from scripts.cargo_formation_teacher import (APPROACH_TOL_M, APPROACH_TOL_RAD, FORWARD_GAIN, HOVER_Z_M, LEFT_GAIN,
                                             LIMITS, PAUSE_ERR_M, PAUSE_ERR_RAD, TURN_GAIN, Reference, compose, wrap)
from sim.zone_cargo import GRASP_RADIUS_M

CARRY_P_GAIN = 1.5
APPROACH_GAINS = {'forward': (.9, -.05, .08), 'left': (.9, -.06, .06), 'turn': (.8, -.10, .10)}


def to_base(robot_pose, xy):
    x, y, a = robot_pose
    dx, dy = xy[0]-x, xy[1]-y
    return math.cos(a)*dx + math.sin(a)*dy, -math.sin(a)*dx + math.cos(a)*dy


def world_velocity_command(robot_pose, vx, vy, w):
    """World-frame velocity -> mecanum command units (before saturation)."""
    a = robot_pose[2]
    return {'forward': (math.cos(a)*vx + math.sin(a)*vy)/FORWARD_GAIN,
            'left': (-math.sin(a)*vx + math.cos(a)*vy)/LEFT_GAIN,
            'turn': w/TURN_GAIN}


def common_scale(cmds):
    """One scale for the whole team so no command leaves its limit."""
    s = 1.
    for c in cmds.values():
        for k, (lo, hi) in LIMITS.items():
            v = c[k]
            if v > hi:
                s = min(s, hi/v)
            elif v < lo:
                s = min(s, lo/v)
    return s


class FormationPlan:
    """Per-robot base and arm targets for one item and one role assignment."""

    def __init__(self, kind, role_by_robot):
        roles = sorted(role_by_robot.values())
        if not fills_formation(kind, roles):
            raise ValueError(f'roles {roles} do not fill a {kind} formation {[sorted(f) for f in formations(kind)]}')
        self.kind = kind
        self.role_by_robot = dict(sorted(role_by_robot.items()))
        self.offsets = {rid: station_offset(kind, role) for rid, role in self.role_by_robot.items()}
        by_role = {g.role: g for g in grasps(kind)}
        self.grip = {rid: by_role[role].grip_xyz for rid, role in self.role_by_robot.items()}

    # --- stations --------------------------------------------------------
    def stations(self, item_pose, standoff_m=0.):
        """World base pose per robot; standoff backs each one off along its heading."""
        out = {}
        for rid, off in self.offsets.items():
            x, y, a = compose(item_pose, off)
            out[rid] = (x - standoff_m*math.cos(a), y - standoff_m*math.sin(a), a)
        return out

    def grip_world(self, rid, item_pose):
        gx, gy, gz = self.grip[rid]
        x, y, a = item_pose
        c, s = math.cos(a), math.sin(a)
        return (x + c*gx - s*gy, y + s*gx + c*gy, gz)

    def approach_command(self, rid, robot_pose, item_pose):
        """Fine alignment of the grip point to (GRASP_RADIUS, 0) and the station heading.

        Returns (command or None when aligned, errors).
        """
        g = self.grip_world(rid, item_pose)
        gx, gy = to_base(robot_pose, g[:2])
        ex, ey = gx - GRASP_RADIUS_M, gy
        ea = wrap(compose(item_pose, self.offsets[rid])[2] - robot_pose[2])
        err = {'x_m': ex, 'y_m': ey, 'yaw_rad': ea}
        if abs(ex) < APPROACH_TOL_M and abs(ey) < APPROACH_TOL_M and abs(ea) < APPROACH_TOL_RAD:
            return None, err
        cmd = {}
        for key, e in (('forward', ex), ('left', ey), ('turn', ea)):
            k, lo, hi = APPROACH_GAINS[key]
            cmd[key] = float(np.clip(k*e, lo, hi))
        return cmd, err

    # --- arm -------------------------------------------------------------
    def arm_plan(self, rid, robot_pose, item_pose, *, hover_z=HOVER_Z_M, steps=8):
        """IK joints for grasp, hover and the straight descent, from the grip point in the base frame.

        Called once when the barrier opens PREGRASP; the executor keeps the
        result for the whole job (as the PR #164 teacher does).
        """
        g = self.grip_world(rid, item_pose)
        bx, by = to_base(robot_pose, g[:2])
        gz = g[2]
        grasp = solve_grip_ik(bx, by, gz, -90)
        pitch = tool_pose(grasp).pitch_deg
        hover = solve_grip_ik(bx, by, hover_z, pitch)
        descent = [solve_grip_ik(bx, by, float(h), pitch) for h in np.linspace(hover_z, gz, steps)[1:]]
        lift = solve_grip_ik(bx, by, hover_z, pitch)
        return {'grip_in_base_m': (bx, by, gz), 'pitch_deg': pitch, 'grasp': grasp, 'hover': hover,
                'descent': descent, 'lift': lift}

    # --- carry -----------------------------------------------------------
    def targets(self, item_pose):
        return {rid: compose(item_pose, off) for rid, off in self.offsets.items()}

    def trajectory_targets(self, item_poses):
        """Per-robot base targets along an item pose trajectory."""
        return [self.targets(p) for p in item_poses]

    def carry_step(self, ref_pose, ref_velocity, robot_poses):
        """Virtual-structure command for one control tick.

        Returns targets, tracking errors, whether the reference may advance and
        the commands with one common saturation scale already applied.
        """
        vx, vy, w = ref_velocity
        errs, targets = {}, self.targets(ref_pose)
        for rid, want in targets.items():
            x, y, a = robot_poses[rid]
            errs[rid] = (math.hypot(want[0]-x, want[1]-y), abs(wrap(want[2]-a)))
        worst = max(e[0] for e in errs.values())
        worst_a = max(e[1] for e in errs.values())
        advance = worst < PAUSE_ERR_M and worst_a < PAUSE_ERR_RAD
        if not advance:
            vx = vy = w = 0.
        raw = {}
        a0 = ref_pose[2]
        for rid, want in targets.items():
            bx, by, _ = self.offsets[rid]
            ox = math.cos(a0)*bx - math.sin(a0)*by
            oy = math.sin(a0)*bx + math.cos(a0)*by
            fvx, fvy = vx - w*oy, vy + w*ox
            x, y, a = robot_poses[rid]
            raw[rid] = world_velocity_command(robot_poses[rid], fvx + CARRY_P_GAIN*(want[0]-x),
                                              fvy + CARRY_P_GAIN*(want[1]-y), w + CARRY_P_GAIN*wrap(want[2]-a))
        scale = common_scale(raw)
        cmds = {rid: {k: float(np.clip(c[k]*scale, *LIMITS[k])) for k in LIMITS} for rid, c in raw.items()}
        return {'targets': targets, 'errors': errs, 'track_err_m': worst, 'track_err_rad': worst_a,
                'advance': advance, 'scale': scale, 'commands': cmds}


def reference(start_pose, legs, **kw):
    """The PR #164 piecewise item reference (straight moves, strafes, in-place turns)."""
    return Reference(start_pose, legs, **kw)


__all__ = ['FormationPlan', 'reference', 'common_scale', 'world_velocity_command', 'to_base', 'HOVER_Z_M']
