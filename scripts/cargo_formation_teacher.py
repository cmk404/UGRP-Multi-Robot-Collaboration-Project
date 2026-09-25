"""Ground-truth formation TEACHER for zone cargo (1, 2 or 3 carriers; weld OFF).

Never a student result. The teacher reads simulator truth (robot base pose,
cargo pose, finger contact forces) to align, to aim the calibrated arm IK and
to decide phase transitions. Robots are driven only through their
``CameraRobotPort`` command channel (mecanum drive + servo pulses); the cargo is
held by finger contact and friction alone: no weld, no equality constraint, no
pose writes and no external force after the fixture setup.

Carry control is a virtual structure (proposal by the Codex read-only analysis,
2026-09-25): one reference cargo pose trajectory, each robot tracking
``reference * approach_base(role)`` with a shared feed-forward velocity, a
bounded P correction, and one common saturation scale for the whole team. The
reference pauses while any robot's tracking error is large. Arm phases are
queued for every carrier at the same SIM instant with the same durations.
"""
from __future__ import annotations

import math

import numpy as np

from harness.visual_arm import solve_grip_ik, tool_pose
from scripts.zone_teacher import FOLDED, ArmSequence
from sim.zone_cargo import GRASP_RADIUS_M, world_grasps

OPEN, CLOSED = 2000, 1500
HOVER_Z_M = .095
CONTROL_S = .1
# Steady-state speed per unit mecanum command, from the drive model:
# max force / linear damping (2.2/1.4, 1.65/1.4) and yaw torque / damping (.12/.08).
FORWARD_GAIN, LEFT_GAIN, TURN_GAIN = 2.2/1.4, 1.65/1.4, .12/.08
LIMITS = {'forward': (-.05, .15), 'left': (-.10, .10), 'turn': (-.15, .15)}
FINGER_MIN_N = .5
APPROACH_TOL_M, APPROACH_TOL_RAD = .006, .03
PAUSE_ERR_M, PAUSE_ERR_RAD = .04, .10
# The carry ends when the reference is finished and the team is close enough,
# or FINISH_SETTLE_S after the reference finished (a loaded team settles with
# small offsets; waiting longer only lets the load creep).
FINISH_ERR_M, FINISH_ERR_RAD, FINISH_SETTLE_S = .02, .05, 3.


def wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi


def compose(pose, local):
    x, y, a = pose
    lx, ly, la = local
    return (x + math.cos(a)*lx - math.sin(a)*ly, y + math.sin(a)*lx + math.cos(a)*ly, wrap(a + la))


class Reference:
    """Piecewise cargo reference: straight moves then in-place turns."""
    def __init__(self, start, legs, *, speed=.05, yaw_rate=.20):
        self.poses = [tuple(start)]
        for leg in legs:
            x, y, a = self.poses[-1]
            if leg[0] == 'move':
                d = leg[1]
                self.poses.append((x + math.cos(a)*d, y + math.sin(a)*d, a))
            elif leg[0] == 'strafe':
                d = leg[1]
                self.poses.append((x - math.sin(a)*d, y + math.cos(a)*d, a))
            elif leg[0] == 'turn':
                self.poses.append((x, y, wrap(a + leg[1])))
        self.speed, self.yaw_rate = speed, yaw_rate
        self.leg, self.u = 0, 0.
        self.pose = tuple(start)

    @property
    def finished(self):
        return self.leg >= len(self.poses) - 1

    def _leg_len(self):
        a, b = self.poses[self.leg], self.poses[self.leg+1]
        d = math.hypot(b[0]-a[0], b[1]-a[1])
        return d/self.speed if d > 1e-9 else abs(wrap(b[2]-a[2]))/self.yaw_rate

    def velocity(self):
        """World (vx, vy, w) of the reference cargo pose on the current leg."""
        if self.finished:
            return 0., 0., 0.
        a, b = self.poses[self.leg], self.poses[self.leg+1]
        T = self._leg_len()
        return (b[0]-a[0])/T, (b[1]-a[1])/T, wrap(b[2]-a[2])/T

    def advance(self, dt):
        while dt > 1e-12 and not self.finished:
            T = self._leg_len()
            step = min(dt, (1-self.u)*T)
            self.u += step/T
            dt -= step
            if self.u >= 1-1e-9:
                self.leg, self.u = self.leg+1, 0.
        if self.finished:
            self.pose = self.poses[-1]
        else:
            a, b = self.poses[self.leg], self.poses[self.leg+1]
            self.pose = (a[0]+(b[0]-a[0])*self.u, a[1]+(b[1]-a[1])*self.u, wrap(a[2]+wrap(b[2]-a[2])*self.u))

    @property
    def total_s(self):
        total, saved = 0., self.leg
        for i in range(len(self.poses)-1):
            self.leg = i
            total += self._leg_len()
        self.leg = saved
        return total


class FormationTeacher:
    """Phases: approach, grasp, verify, lift, hold, carry, lower, release, retract, back_off, done."""
    def __init__(self, world, ports, instance, roles, *, legs, log, hold_s=1.5, lift_z=HOVER_Z_M,
                 carry_if_not_clear=False):
        self.world, self.ports, self.inst, self.roles = world, ports, instance, dict(roles)
        self.legs, self.log, self.hold_s, self.lift_z = legs, log, hold_s, lift_z
        self.carry_if_not_clear = carry_if_not_clear
        self.arms = {rid: ArmSequence(port, FOLDED) for rid, port in ports.items() if rid in self.roles}
        self.phase, self.phase_t = 'approach', 0.
        self.outcome, self.detail = None, {}
        self.next_tick = 0.
        self.ref = None
        self.grasp_pose = {}
        self.clear_since = None
        self.lifted_clear = False
        self.start_pose = self.cargo_pose()
        self.phase_times = {}
        self.targets = {}

    # --- teacher-only truth ---
    def robot_pose(self, rid):
        r = self.world.robot(rid)
        xyz = r.base_xyz()
        return float(xyz[0]), float(xyz[1]), float(r.base_rpy()[2])

    def cargo_pose(self):
        d = self.world.data.body(self.inst.body)
        q = d.xquat
        yaw = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
        return float(d.xpos[0]), float(d.xpos[1]), yaw

    def to_base(self, rid, xy):
        x, y, a = self.robot_pose(rid)
        dx, dy = xy[0]-x, xy[1]-y
        return math.cos(a)*dx + math.sin(a)*dy, -math.sin(a)*dx + math.cos(a)*dy

    def grasps(self):
        return world_grasps(self.inst, self.cargo_pose())

    def _set(self, phase, now, **detail):
        self.phase_times[self.phase] = round(now - self.phase_t, 3)
        self.log('phase', now, phase=phase, **detail)
        self.phase, self.phase_t = phase, now

    def _fail(self, outcome, now, **detail):
        self.outcome = outcome
        self.detail.update(detail)
        self.log('failure', now, outcome=outcome, **detail)

    # --- motion ---
    def _command(self, rid, vx, vy, w, now, scale=1.):
        _, _, a = self.robot_pose(rid)
        fwd = (math.cos(a)*vx + math.sin(a)*vy)/FORWARD_GAIN
        left = (-math.sin(a)*vx + math.cos(a)*vy)/LEFT_GAIN
        turn = w/TURN_GAIN
        return {'forward': fwd*scale, 'left': left*scale, 'turn': turn*scale}

    @staticmethod
    def _scale(cmds):
        s = 1.
        for c in cmds.values():
            for k, (lo, hi) in LIMITS.items():
                v = c[k]
                if v > hi:
                    s = min(s, hi/v)
                elif v < lo:
                    s = min(s, lo/v)
        return s

    def _send(self, cmds, now):
        s = self._scale(cmds)
        for rid, c in cmds.items():
            self.ports[rid].apply({'kind': 'mecanum', 'duration_s': .15,
                                   **{k: float(np.clip(c[k]*s, *LIMITS[k])) for k in LIMITS}}, now)

    def _approach(self, now):
        """Each robot aligns its grip point to (GRASP_RADIUS, 0) and its role heading."""
        cmds, done = {}, True
        for rid, role in self.roles.items():
            g = self.grasps()[role]
            gx, gy = self.to_base(rid, g['grip_xyz'][:2])
            ex, ey = gx - GRASP_RADIUS_M, gy
            ea = wrap(g['base_xyyaw'][2] - self.robot_pose(rid)[2])
            if abs(ex) < APPROACH_TOL_M and abs(ey) < APPROACH_TOL_M and abs(ea) < APPROACH_TOL_RAD:
                self.ports[rid].hold(now)
                continue
            done = False
            self.ports[rid].apply({'kind': 'mecanum', 'duration_s': .12,
                                   'forward': float(np.clip(.9*ex, -.05, .08)),
                                   'left': float(np.clip(.9*ey, -.06, .06)),
                                   'turn': float(np.clip(.8*ea, -.10, .10))}, now)
        return done

    def _ik(self, rid, z):
        """Grasp, hover and descent joints for this robot's grip point (fixed at grasp time)."""
        if rid not in self.targets:
            g = self.grasps()[self.roles[rid]]
            bx, by = self.to_base(rid, g['grip_xyz'][:2])
            self.targets[rid] = (bx, by, g['grip_xyz'][2])
        bx, by, gz = self.targets[rid]
        grasp = solve_grip_ik(bx, by, gz, -90)
        pitch = tool_pose(grasp).pitch_deg
        hover = solve_grip_ik(bx, by, z, pitch)
        path = [solve_grip_ik(bx, by, float(h), pitch) for h in np.linspace(z, gz, 8)[1:]]
        return grasp, hover, path

    def finger_forces(self):
        """{rid: (left N, right N)} normal force of fingers on this cargo (truth)."""
        import mujoco
        m, d = self.world.model, self.world.data
        cargo = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, self.inst.geom(p.name))
                 for p in self.inst.spec().parts if p.collision}
        out = {rid: [0., 0.] for rid in self.roles}
        ids = {}
        for rid in self.roles:
            for k, side in enumerate(('left', 'right')):
                ids[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f'{rid}__{side}_finger')] = (rid, k)
        f = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            pair = (c.geom1, c.geom2)
            for a, b in (pair, pair[::-1]):
                if a in ids and b in cargo:
                    mujoco.mj_contactForce(m, d, i, f)
                    rid, k = ids[a]
                    out[rid][k] += abs(float(f[0]))
        return {rid: tuple(v) for rid, v in out.items()}

    def bilateral(self):
        return {rid: min(v) >= FINGER_MIN_N for rid, v in self.finger_forces().items()}

    # --- main tick ---
    def tick(self, now, metrics):
        busy = [not arm.tick(now) for arm in self.arms.values()]
        if self.outcome is not None and self.phase == 'done':
            return
        if now + 1e-9 < self.next_tick:
            return
        self.next_tick = now + CONTROL_S
        arms_done = not any(busy)
        el = now - self.phase_t
        if self.phase == 'approach':
            if self._approach(now):
                self.grasp_pose = {}
                for rid, arm in self.arms.items():
                    grasp, hover, path = self._ik(rid, HOVER_Z_M)
                    self.grasp_pose[rid] = path[-1]
                    arm.queue({**hover, 1: OPEN}, now, duration=1.0)
                for rid, arm in self.arms.items():
                    _, _, path = self._ik(rid, HOVER_Z_M)
                    for pose in path:
                        arm.queue(pose, now, duration=.12, settle=0.)
                    arm.queue({1: CLOSED}, now, duration=.5, settle=.4)
                self._set('grasp', now)
            elif el > 40:
                self._fail('approach_timeout', now)
                self._set('done', now)
        elif self.phase == 'grasp':
            if arms_done:
                self._set('verify', now)
        elif self.phase == 'verify':
            forces = self.finger_forces()
            if all(min(v) >= FINGER_MIN_N for v in forces.values()):
                self.detail['grasp_forces_n'] = {r: [round(x, 2) for x in v] for r, v in forces.items()}
                for rid, arm in self.arms.items():
                    _, hover, _ = self._ik(rid, self.lift_z)
                    arm.queue({**hover, 1: CLOSED}, now, duration=1.2, settle=.3)
                self._set('lift', now)
            elif el > 1.:
                self._fail('grasp_contact_missing', now,
                           grasp_forces_n={r: [round(x, 2) for x in v] for r, v in forces.items()})
                self._release(now)
        elif self.phase == 'lift':
            if arms_done:
                self._set('hold', now)
                self.clear_since = None
        elif self.phase == 'hold':
            clear = metrics['cargo_min_z'] > .012
            if clear and self.clear_since is None:
                self.clear_since = now
            if not clear:
                self.clear_since = None
            if el >= self.hold_s:
                self.lifted_clear = self.clear_since is not None and now - self.clear_since >= self.hold_s - .2
                self.detail['hold_min_z_m'] = round(metrics['cargo_min_z'], 4)
                if not self.lifted_clear and self.carry_if_not_clear:
                    # Diagnostic only: try to move a load that is not clear of the floor.
                    self.detail['not_lifted_clear'] = True
                    self.log('not_lifted_clear_attempting_carry', now, hold_min_z_m=round(metrics['cargo_min_z'], 4))
                if self.lifted_clear or self.carry_if_not_clear:
                    if not self.legs:
                        self._lower(now)
                    else:
                        self.ref = Reference(self.cargo_pose(), self.legs)
                        self.ref_total_s = self.ref.total_s
                        self._set('carry', now)
                else:
                    self._fail('not_lifted_clear', now, hold_min_z_m=round(metrics['cargo_min_z'], 4))
                    self._lower(now)
        elif self.phase == 'carry':
            self._carry(now, metrics)
        elif self.phase == 'lower':
            if arms_done:
                for arm in self.arms.values():
                    arm.queue({1: OPEN}, now, duration=.4, settle=.5)
                self._set('release', now)
        elif self.phase == 'release':
            if arms_done:
                for rid, arm in self.arms.items():
                    arm.queue({**self._hover_after(rid), 1: OPEN}, now, duration=.6)
                    arm.queue(FOLDED, now)
                self._set('retract', now)
        elif self.phase == 'retract':
            if arms_done:
                self._set('back_off', now)
        elif self.phase == 'back_off':
            if el < 2.5:
                for rid in self.roles:
                    self.ports[rid].apply({'kind': 'mecanum', 'forward': -.05, 'left': 0., 'turn': 0.,
                                           'duration_s': .15}, now)
            else:
                for rid in self.roles:
                    self.ports[rid].hold(now)
                self._set('settle', now)
        elif self.phase == 'settle':
            if el >= 1.5:
                if self.outcome is None:
                    self.outcome = ('carried_without_clear_lift' if self.detail.get('not_lifted_clear')
                                    else 'completed_sequence')
                self._set('done', now)

    def _hover_after(self, rid):
        """Hover joints above the (released) grip pose, recomputed from truth."""
        try:
            _, hover, _ = self._ik(rid, HOVER_Z_M)
            return hover
        except Exception:
            return {k: v for k, v in FOLDED.items() if k != 1}

    def _lower(self, now):
        for rid, arm in self.arms.items():
            arm.queue({**self.grasp_pose[rid], 1: CLOSED}, now, duration=1.2, settle=.4)
        self._set('lower', now)

    def _release(self, now):
        for arm in self.arms.values():
            arm.queue({1: OPEN}, now, duration=.4, settle=.3)
        self._set('release', now)

    def _carry(self, now, metrics):
        # drop / grip-loss checks (truth)
        if metrics['cargo_min_z'] < .004 and self.lifted_clear:
            metrics['events'].append({'event': 'cargo_touched_floor', 't': round(now, 2)})
        lost = [rid for rid, ok in self.bilateral().items() if not ok]
        if lost:
            self.lost_since = getattr(self, 'lost_since', None) or now
            if now - self.lost_since > .3:
                for p in self.roles:
                    self.ports[p].hold(now)
                self._fail('grip_lost_in_transit', now, robots=lost)
                self._lower(now)
                return
        else:
            self.lost_since = None
        errs = {}
        targets = {}
        for rid, role in self.roles.items():
            base = self.inst.spec().grasps[[g.role for g in self.inst.spec().grasps].index(role)].approach_base()
            want = compose(self.ref.pose, base)
            x, y, a = self.robot_pose(rid)
            errs[rid] = (math.hypot(want[0]-x, want[1]-y), abs(wrap(want[2]-a)))
            targets[rid] = (want, base)
        worst = max(max(e[0] for e in errs.values()), 0.)
        worst_a = max(e[1] for e in errs.values())
        metrics['track_err_m'] = worst
        if worst < PAUSE_ERR_M and worst_a < PAUSE_ERR_RAD:
            self.ref.advance(CONTROL_S)
        if self.ref.finished and getattr(self, 'ref_done_at', None) is None:
            self.ref_done_at = now
        if self.ref.finished and ((worst < FINISH_ERR_M and worst_a < FINISH_ERR_RAD)
                                  or now - self.ref_done_at >= FINISH_SETTLE_S):
            self.detail['finish_track_err_m'] = round(worst, 4)
            for rid in self.roles:
                self.ports[rid].hold(now)
            self._lower(now)
            return
        if now - self.phase_t > 3*self.ref_total_s + 30:
            self._fail('carry_timeout', now)
            self._lower(now)
            return
        vx, vy, w = self.ref.velocity() if worst < PAUSE_ERR_M and worst_a < PAUSE_ERR_RAD else (0., 0., 0.)
        cmds = {}
        for rid, (want, base) in targets.items():
            a0 = self.ref.pose[2]
            ox = math.cos(a0)*base[0] - math.sin(a0)*base[1]
            oy = math.sin(a0)*base[0] + math.cos(a0)*base[1]
            fvx, fvy = vx - w*oy, vy + w*ox
            x, y, a = self.robot_pose(rid)
            cvx = fvx + 1.5*(want[0]-x)
            cvy = fvy + 1.5*(want[1]-y)
            cw = w + 1.5*wrap(want[2]-a)
            cmds[rid] = self._command(rid, cvx, cvy, cw, now)
        self._send(cmds, now)
