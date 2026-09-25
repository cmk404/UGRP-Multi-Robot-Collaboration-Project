"""Wrist-camera-only PAIR carry of long_beam: feasibility study runner (open floor, weld OFF).

Two MasterPi robots (r1: end_neg, r2: end_pos) grasp the 0.60 m / 0.30 kg beam
at its ends, lift it together, carry it on a fixed pre-agreed route and put it
down together. Each robot decides only from:

* its own ``robot_cam`` JPEG and its own issued servo PWM
  (``harness.owncam_pair_beam``);
* the shared task sheet (roles, route legs in the beam frame, speeds, durations);
* the approved pair barrier ``harness.pair_carry_sync.PairCarrySync``: readiness
  reports carry the robot's own frame id; no world state is exchanged;
* its own clock (the barrier GO time starts identical timed schedules).

Condition ``stub_approach``: robots start 0.6-1.0 m away and a GT pose stub
(label ``gt_stub_eval_only``) drives them to a pre-station 0.30 m behind their
station, facing the beam. From there on nothing reads truth. This condition is
never a success claim for the approach. Condition ``own_only``: robots start
0.20-0.40 m behind the station with lateral/yaw offsets and never use truth.

Truth (poses, finger forces, beam pose) is written only to
``evaluation-only.jsonl`` and the result's ``evaluation_only`` block.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness import owncam_pair_beam as ob  # noqa: E402
from harness.pair_carry_sync import PairCarrySync  # noqa: E402

SCHEMA = 'ugrp.owncam_pair_beam_study.v1'
PROFILE = 'owncam_pair_beam_v1'
BASE_Z = .032355118817659255
ROLES = {'r1': 'end_neg', 'r2': 'end_pos'}
SEARCH = {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}
OPEN, CLOSED = 2000, 1500
GRASP_Z_M, HOVER_Z_M = .024, .095
CONTROL_S = .1
LOOK_EVERY_S = .4
# Task sheet (shared, static): route legs in the beam frame of r1 (axial = +r1 forward).
SPEED_M_S = .06
FORWARD_GAIN, LEFT_GAIN = 2.2 / 1.4, 1.65 / 1.4       # drive model steady state (cargo_formation_teacher)
LEGS = (('axial', .60), ('lateral', .30))
BARRIER_TTL_S = 1.0
# dev 601 calib run (7d97bab): grip-view signature r1 0.029 / r2 0.060, so 0.04 rejected a real grip.
# The grip itself is confirmed by the lift co-motion IoU (grip view vs lift view: r1 0.735, r2 0.835).
GRIP_MIN_SIGNATURE = .02
HOLD_MIN_IOU = .45                # lift co-motion only
# Carry/barrier hold: strip presence ratio (held signature fraction / lift anchor fraction). In dev 601 the
# held-strip IoU fell to 0.46 on r1 with NO slip (GT offset constant; shading of the thin strip), while
# the ratio stayed >= 0.99. A dropped beam leaves the lower frame, i.e. ratio -> 0.
HOLD_MIN_RATIO = .5
# Static drive calibration for the loaded pair carry (labelled, like a camera calibration): dev 601 GT
# travel / commanded = axial 0.463/0.60, lateral 0.209/0.30 with the teacher's unloaded gains.
CARRY_ODOM_SCALE = {'axial': .772, 'lateral': .697}
CARRY_ODOM_SOURCE = 'dev 601 calib run 7d97bab, evaluation-only GT beam travel (static calibration, not live)'
LOST_FRAMES = 2
PRESTATION_BACK_M = .30
LIMIT_S = 240.
STATE_LIMIT_S = {'align': 60., 'wait_lift': 20., 'wait_carry': 15., 'wait_lower': 20., 'wait_open': 15.}
CONTACT_PROFILES = ('cargo_noslip_v1', 'local_contact_fine')
# seed -> beam pose and per-robot start offsets (back m, lateral m, yaw rad) from the station.
SCENARIOS = {
    # development (tuning allowed, labelled dev, never reported as results)
    601: {'beam': (1.20, -0.80, 0.00), 'offsets': {'r1': (.30, .00, .00), 'r2': (.30, .00, .00)}},
    602: {'beam': (1.40, -0.50, 0.40), 'offsets': {'r1': (.25, .04, .12), 'r2': (.35, -.03, -.10)}},
}
DEV_SEEDS = (601, 602)
STUB_START_M = .80


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def sha_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class PairStudent:
    """One robot's own-view controller. Holds no reference to the world or the partner."""

    def __init__(self, rid, port, arm, sync_for, log, save=None):
        self.rid, self.port, self.arm, self.sync_for, self.log = rid, port, arm, sync_for, log
        self.save = save
        self.state, self.state_t = 'align_start', 0.
        self.next_look = 0.
        self.aligned_streak = 0
        self.last_obs = None
        self.grip_base = None
        self.anchor = None
        self.lost = 0
        self.seq = 0
        self.schedule = None
        self.claims = {}
        self.failure = None
        self.frames = 0
        self.commands = 0
        self.stub_active = False

    def set(self, state, now, **detail):
        self.log(self.rid, 'state', now, state=state, **detail)
        self.state, self.state_t = state, now

    def fail(self, reason, now):
        self.failure = reason
        self.set('failed', now, reason=reason)

    def look(self, now):
        obs = self.port.capture()
        self.frames += 1
        if self.save is not None:
            self.save(self.rid, obs, self.state)
        return obs

    def pose_of(self, obs):
        return {int(k): v for k, v in obs['actuator_state']['servo_pulses'].items()}

    def report(self, key, obs, now, ready=True, reason=''):
        sync = self.sync_for(key)
        self.seq += 1
        ok = sync.report(self.rid, plan_version=1, epoch=sync.epoch, sequence=self.seq, ready=ready,
                         observed_at_s=float(obs['sim_time']), received_at_s=now,
                         frame_id=f"{self.rid}-{obs['frame_id']}-{obs['sha256'][:12]}", reason=reason)
        self.log(self.rid, 'barrier_report', now, barrier=key, ready=ready, accepted=ok, reason=reason)

    def drive(self, cmd, now):
        self.commands += 1
        self.port.apply({'kind': 'mecanum', 'forward': cmd['forward'], 'left': cmd['left'], 'turn': cmd['turn'],
                         'duration_s': cmd['duration']}, now)

    # ---- phases -------------------------------------------------------------
    def tick(self, now):
        arm_idle = now >= self.arm.until and not self.arm.events   # arms are ticked by the loop
        handler = getattr(self, '_' + self.state, None)
        if handler is not None:
            handler(now, arm_idle)

    def _align_start(self, now, arm_idle):
        self.look_name, pose = ob.look_posture(None)
        self.arm.queue(pose, now, duration=.8)
        self.set('align', now)

    def _switch_look(self, distance, now):
        name, pose = ob.look_posture(distance)
        order = [n for n, _, _ in ob.LOOK_POSTURES]
        if order.index(name) <= order.index(self.look_name):     # only nearer on distance (hysteresis)
            return False
        self.log(self.rid, 'look_posture', now, posture=name, grip_distance_m=round(distance, 3))
        self.look_name = name
        self.aligned_streak = 0
        self.arm.queue(pose, now, duration=.6)
        return True

    def _align(self, now, arm_idle):
        if not arm_idle or now < self.next_look:
            return
        self.next_look = now + LOOK_EVERY_S
        if now - self.state_t > STATE_LIMIT_S['align']:
            return self.fail('ALIGN_TIMEOUT', now)
        obs = self.look(now)
        beam = ob.observe_beam(obs['image'], self.pose_of(obs))
        self.log(self.rid, 'beam_obs', now, **{k: v for k, v in beam.items() if k != 'provenance'})
        if not beam['visible']:
            # Own-view search: rotate slowly in place.
            self.aligned_streak = 0
            return self.drive({'forward': 0., 'left': 0., 'turn': .06, 'duration': .3}, now)
        if not beam['end_visible']:
            self.aligned_streak = 0
            # End below the view: look from the next farther posture, or back up from the farthest.
            order = [n for n, _, _ in ob.LOOK_POSTURES]
            k = order.index(self.look_name)
            if k > 0:
                self.look_name = order[k - 1]
                self.log(self.rid, 'look_posture', now, posture=self.look_name, reason='end_clipped')
                return self.arm.queue(ob.LOOK_POSTURES[k - 1][2], now, duration=.6)
            return self.drive({'forward': -.04, 'left': 0., 'turn': 0., 'duration': .3}, now)
        if self._switch_look(beam['grip_base_m'][0], now):
            return
        cmd = ob.align_command(beam)
        if cmd is None:
            self.aligned_streak += 1
            self.grip_base = beam['grip_base_m']
            if self.aligned_streak >= 2:
                self.claims['aligned'] = {'grip_base_m': self.grip_base, 'errors': ob.align_errors(beam),
                                          'sim_time': now}
                self._queue_grasp(now)
            return
        self.aligned_streak = 0
        self.drive(cmd, now)

    def _queue_grasp(self, now):
        from harness.visual_arm import solve_grip_ik, tool_pose
        bx, by = self.grip_base
        try:
            grasp = solve_grip_ik(bx, by, GRASP_Z_M, -90)
            pitch = tool_pose(grasp).pitch_deg
            self.hover = solve_grip_ik(bx, by, HOVER_Z_M, pitch)
            path = [solve_grip_ik(bx, by, float(h), pitch) for h in np.linspace(HOVER_Z_M, GRASP_Z_M, 8)[1:]]
        except Exception as exc:     # noqa: BLE001 - recorded failure mode
            return self.fail(f'IK_UNAVAILABLE:{exc}', now)
        self.grasp_pose = path[-1]
        self.arm.queue({**self.hover, 1: OPEN}, now, duration=1.0)
        for p in path:
            self.arm.queue(p, now, duration=.12, settle=0.)
        self.arm.queue({1: CLOSED}, now, duration=.5, settle=.4)
        self.set('grasp', now, grip_base_m=[round(v, 4) for v in self.grip_base])

    def _grasp(self, now, arm_idle):
        if not arm_idle:
            return
        obs = self.look(now)
        sig = ob.held_signature(obs['image'])
        frac = ob.signature_fraction(sig)
        self.log(self.rid, 'grip_view', now, signature_fraction=round(frac, 4))
        if frac < GRIP_MIN_SIGNATURE:
            return self.fail('GRIP_NOT_SEEN', now)
        self.anchor = sig
        self.claims['gripped'] = {'signature_fraction': round(frac, 4), 'sim_time': now}
        self.set('wait_lift', now)

    def _wait(self, key, nxt, now, on_go):
        if now - self.state_t > STATE_LIMIT_S['wait_' + key]:
            return self.fail(f'BARRIER_{key.upper()}_TIMEOUT', now)
        sync = self.sync_for(key)
        decision = sync.authorize(now)
        if decision['phase'] == 'GO':
            self.log(self.rid, 'barrier_go', now, barrier=key)
            on_go(now)
            return self.set(nxt, now)
        if now >= self.next_look:
            self.next_look = now + LOOK_EVERY_S
            obs = self.look(now)
            ratio = self.hold_ratio(obs)
            self.report(key, obs, now, ready=ratio >= HOLD_MIN_RATIO, reason=f'hold_ratio={ratio:.2f}')

    def hold_ratio(self, obs):
        if self.anchor is None:
            return 1.
        base = ob.signature_fraction(self.anchor)
        return ob.signature_fraction(ob.held_signature(obs['image'])) / base if base > 0 else 0.

    def _wait_lift(self, now, arm_idle):
        def go(t):
            self.arm.queue({**self.hover, 1: CLOSED}, t, duration=1.2, settle=.3)
        self._wait('lift', 'lift', now, go)

    def _lift(self, now, arm_idle):
        if not arm_idle:
            return
        obs = self.look(now)
        sig = ob.held_signature(obs['image'])
        iou = ob.signature_iou(self.anchor, sig)
        self.log(self.rid, 'lift_view', now, held_iou=round(iou, 3), signature_fraction=round(ob.signature_fraction(sig), 4))
        if iou < HOLD_MIN_IOU:
            return self.fail('LOAD_NOT_HELD_AFTER_LIFT', now)
        self.anchor = sig
        self.claims['lifted'] = {'held_iou': round(iou, 3), 'sim_time': now}
        self.set('wait_carry', now)

    def _wait_carry(self, now, arm_idle):
        def go(t):
            self.schedule = build_schedule(self.rid, t)
        self._wait('carry', 'carry', now, go)

    def _carry(self, now, arm_idle):
        cmd = None
        for start, end, c in self.schedule:
            if start <= now < end:
                cmd = c
                break
        if cmd is None and now >= self.schedule[-1][1]:
            self.port.hold(now)
            self.claims['route_done'] = {'sim_time': now}
            return self.set('wait_lower', now)
        if cmd is not None:
            self.commands += 1
            self.port.apply({'kind': 'mecanum', **cmd, 'duration_s': .15}, now)
        if now >= self.next_look:
            self.next_look = now + LOOK_EVERY_S
            obs = self.look(now)
            sig = ob.held_signature(obs['image'])
            iou = ob.signature_iou(self.anchor, sig)
            ratio = self.hold_ratio(obs)
            self.log(self.rid, 'carry_view', now, held_iou=round(iou, 3), hold_ratio=round(ratio, 3))
            self.lost = self.lost + 1 if ratio < HOLD_MIN_RATIO else 0
            if self.lost >= LOST_FRAMES:
                self.port.hold(now)
                self.sync_for('carry').hold(f'{self.rid}_load_changed', now)
                return self.fail('LOAD_CHANGED_IN_CARRY', now)

    def _wait_lower(self, now, arm_idle):
        def go(t):
            self.arm.queue({**self.grasp_pose, 1: CLOSED}, t, duration=1.2, settle=.4)
        self._wait('lower', 'lower', now, go)

    def _lower(self, now, arm_idle):
        if arm_idle:
            self.set('wait_open', now)

    def _wait_open(self, now, arm_idle):
        def go(t):
            self.arm.queue({1: OPEN}, t, duration=.4, settle=.5)
            self.arm.queue({**self.hover, 1: OPEN}, t, duration=.6)
            self.arm.queue(SEARCH, t, duration=.8)
        self._wait('open', 'released', now, go)

    def _released(self, now, arm_idle):
        if not arm_idle:
            return
        if now - self.state_t < 6.:
            self.port.apply({'kind': 'mecanum', 'forward': -.04, 'left': 0., 'turn': 0., 'duration_s': .15}, now)
            return
        self.port.hold(now)
        obs = self.look(now)
        beam = ob.observe_beam(obs['image'], self.pose_of(obs))
        self.log(self.rid, 'verify_view', now, **{k: v for k, v in beam.items() if k != 'provenance'})
        self.claims['placed'] = {'beam_visible_on_floor_plane': bool(beam['visible']), 'sim_time': now,
                                 'scope': 'own view after release; no map target in the open-floor study'}
        self.set('done', now)


def build_schedule(rid, t0):
    """Timed mecanum commands for this robot's role (mirror image for the far end)."""
    sign = 1. if ROLES[rid] == 'end_neg' else -1.
    out, t = [], t0
    for kind, dist in LEGS:
        dur = dist / (SPEED_M_S * CARRY_ODOM_SCALE[kind])
        if kind == 'axial':
            cmd = {'forward': sign * SPEED_M_S / FORWARD_GAIN, 'left': 0., 'turn': 0.}
        else:
            cmd = {'forward': 0., 'left': sign * SPEED_M_S / LEFT_GAIN, 'turn': 0.}
        out.append((t, t + dur, cmd))
        t += dur + .5            # short common pause between legs
    return out


def git(*args):
    return subprocess.run(['git', *args], cwd=ROOT, capture_output=True, text=True).stdout.strip()


def main():
    import mujoco
    from sim.camera_robot_port import CameraRobotPort
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.zone_cargo import instances, world_grasps
    from sim.zone_cargo_scene import CargoZoneScene
    from scripts.zone_teacher import FOLDED, ArmSequence
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--seed', type=int, choices=sorted(SCENARIOS), required=True)
    p.add_argument('--condition', choices=('stub_approach', 'own_only'), required=True)
    p.add_argument('--contact-profile', choices=CONTACT_PROFILES, default='cargo_noslip_v1')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--allow-dirty', action='store_true')
    a = p.parse_args()
    dirty = bool(git('status', '--porcelain'))
    if dirty and not a.allow_dirty:
        raise SystemExit('commit and freeze the source before a recorded run (or --allow-dirty)')
    out = a.output
    out.mkdir(parents=True, exist_ok=False)
    (out / 'inputs').mkdir()
    wall0, load0 = time.time(), [round(v, 2) for v in os.getloadavg()]
    sc = SCENARIOS[a.seed]
    item = {'item_id': 'beam', 'kind': 'long_beam', 'pose': list(sc['beam'])}
    scene = CargoZoneScene.from_cargo_config('zone_wide', 11, cargo=[item], goal={'A': {'cyan': 1}},
                                             contact_profile=a.contact_profile)
    world = MultiMasterPiProductionV2(seed=a.seed, width=640, height=480, render=True,
                                      warehouse_layout=scene.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=scene.transform)
    scene.setup(world)
    m, d = world.model, world.data
    inst = instances([item])[0]
    grasps = world_grasps(inst)
    stations = {}
    for rid, role in ROLES.items():
        x, y, yaw = grasps[role]['base_xyyaw']
        stations[rid] = (x, y, yaw)
        back, lat, dyaw = sc['offsets'][rid]
        if a.condition == 'stub_approach':
            back, lat, dyaw = back + STUB_START_M, lat, dyaw + .6 * (1 if rid == 'r1' else -1)
        sx = x - back * math.cos(yaw) - lat * math.sin(yaw)
        sy = y - back * math.sin(yaw) + lat * math.cos(yaw)
        world.robot(rid).set_base_pose_for_test((sx, sy, BASE_Z), yaw + dyaw)
    world.robot('r3').set_base_pose_for_test((-0.6, 1.0, BASE_Z), 0.)
    mujoco.mj_forward(m, d)
    (out / 'scene.xml').write_text(world.scene_xml)
    ports = {r: CameraRobotPort(world, r, allow_reverse=True, allow_mecanum=True) for r in ('r1', 'r2', 'r3')}
    events = []

    def log(rid, kind, now, **detail):
        events.append({'robot': rid, 'event': kind, 'sim_time_s': round(now, 3), **detail})

    syncs = {}

    def sync_for(key):
        if key not in syncs:
            syncs[key] = PairCarrySync(f'beam-{key}', participants=tuple(ROLES), report_ttl_s=BARRIER_TTL_S)
        return syncs[key]

    arms = {r: ArmSequence(ports[r], FOLDED) for r in ROLES}
    frame_index = []

    def save(rid, obs, state):
        name = f"{rid}-{obs['frame_id']:05d}.jpg"
        (out / 'inputs' / name).write_bytes(base64.b64decode(obs['image']))
        frame_index.append({'file': name, 'robot': rid, 'state': state, 'sim_time': obs['sim_time'],
                            'sha256': obs['sha256'], 'own_pose_commands': obs['actuator_state']['servo_pulses']})

    students = {r: PairStudent(r, ports[r], arms[r], sync_for, log, save) for r in ROLES}
    # ---- labelled GT stub approach (condition stub_approach only) -------------------------
    stub_log = []

    def stub_drive(rid, now):
        x, y, yaw = stations[rid]
        gx, gy = x - PRESTATION_BACK_M * math.cos(yaw), y - PRESTATION_BACK_M * math.sin(yaw)
        r = world.robot(rid)
        px, py, pa = float(r.base_xyz()[0]), float(r.base_xyz()[1]), float(r.base_rpy()[2])
        ex, ey, ea = gx - px, gy - py, wrap(yaw - pa)
        if math.hypot(ex, ey) < .03 and abs(ea) < .05:
            return True
        fwd = math.cos(pa) * ex + math.sin(pa) * ey
        left = -math.sin(pa) * ex + math.cos(pa) * ey
        ports[rid].apply({'kind': 'mecanum', 'forward': float(np.clip(1.2 * fwd, -.05, .10)),
                          'left': float(np.clip(1.2 * left, -.08, .08)), 'turn': float(np.clip(.8 * ea, -.12, .12)),
                          'duration_s': .15}, now)
        return False

    stub_done = {r: a.condition != 'stub_approach' for r in ROLES}
    # ---- evaluation-only truth ------------------------------------------------------------
    truth = (out / 'evaluation-only.jsonl').open('w')
    beam_body = d.body(inst.body)
    cargo_geoms = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, inst.geom(pp.name))
                   for pp in inst.spec().parts if pp.collision}
    finger_ids = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f'{r}__{s}_finger'): (r, k)
                  for r in ROLES for k, s in enumerate(('left', 'right'))}
    stats = {'eq_active_max': 0, 'max_beam_z_m': 0., 'max_tilt_deg_lifted': 0., 'max_pair_distance_dev_m': 0.}
    beam_start = np.array(beam_body.xpos[:2], float)
    nominal_pair_dist = math.hypot(stations['r1'][0] - stations['r2'][0], stations['r1'][1] - stations['r2'][1])

    def finger_forces():
        f6 = np.zeros(6)
        forces = {r: [0., 0.] for r in ROLES}
        for i in range(d.ncon):
            c = d.contact[i]
            for g1, g2 in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if g1 in finger_ids and g2 in cargo_geoms:
                    mujoco.mj_contactForce(m, d, i, f6)
                    r, k = finger_ids[g1]
                    forces[r][k] += abs(float(f6[0]))
        return forces

    def sample(now):
        mat = beam_body.xmat.reshape(3, 3)
        tilt = math.degrees(math.acos(max(-1., min(1., float(mat[2, 2])))))
        z = float(beam_body.xpos[2])
        rp = {r: [round(float(v), 4) for v in (*world.robot(r).base_xyz()[:2], world.robot(r).base_rpy()[2])]
              for r in ROLES}
        dist = math.hypot(rp['r1'][0] - rp['r2'][0], rp['r1'][1] - rp['r2'][1])
        stats['max_beam_z_m'] = max(stats['max_beam_z_m'], z)
        if z > .03:
            stats['max_tilt_deg_lifted'] = max(stats['max_tilt_deg_lifted'], tilt)
            stats['max_pair_distance_dev_m'] = max(stats['max_pair_distance_dev_m'], abs(dist - nominal_pair_dist))
        truth.write(json.dumps({'t': round(now, 2), 'states': {r: s.state for r, s in students.items()},
                                'beam_xyz': [round(float(v), 4) for v in beam_body.xpos], 'beam_tilt_deg': round(tilt, 2),
                                'robots': rp, 'pair_distance_m': round(dist, 4),
                                'finger_n': {r: [round(x, 2) for x in v] for r, v in finger_forces().items()}}) + '\n')

    dt = float(m.opt.timestep)
    next_ctrl, next_sample, next_arm = 0., 0., 0.
    try:
        while float(d.time) < LIMIT_S:
            now = float(d.time)
            if now >= next_ctrl:
                next_ctrl = now + CONTROL_S
                for rid, st in students.items():
                    if not stub_done[rid]:
                        st.stub_active = True
                        if stub_drive(rid, now):
                            stub_done[rid] = True
                            stub_log.append({'robot': rid, 'reached_prestation_s': round(now, 2)})
                            ports[rid].hold(now)
                        continue
                    st.tick(now)
                if any(s.state == 'failed' for s in students.values()):
                    for rid, s in students.items():
                        ports[rid].hold(now)
                    break
                if all(s.state == 'done' for s in students.values()):
                    break
            if now >= next_arm:
                next_arm = now + .05
                for arm in arms.values():
                    arm.tick(now)
            for port in ports.values():
                port.tick(now)
            world._physics_step_for(world.controllers['r1'])
            if d.eq_active.any():
                stats['eq_active_max'] = 1
                raise RuntimeError('weld/equality became active')
            if now >= next_sample:
                next_sample = now + .2
                sample(now)
    finally:
        truth.close()
    # ---- evaluation (truth) ----------------------------------------------------------------
    now = float(d.time)
    mat = beam_body.xmat.reshape(3, 3)
    tilt = math.degrees(math.acos(max(-1., min(1., float(mat[2, 2])))))
    u = np.array([math.cos(sc['beam'][2]), math.sin(sc['beam'][2])])
    v = np.array([-u[1], u[0]])
    planned = beam_start + LEGS[0][1] * u + LEGS[1][1] * v
    final = np.array(beam_body.xpos[:2], float)
    forces = finger_forces()
    reached = {r: s.state for r, s in students.items()}
    evaluation = {
        'both_gripped_gt': None, 'lifted_clear_gt': stats['max_beam_z_m'] > .06,
        'beam_final_xyz': [round(float(x), 4) for x in beam_body.xpos], 'beam_final_tilt_deg': round(tilt, 2),
        'planned_beam_xy': [round(float(x), 4) for x in planned],
        'final_error_m': round(float(np.linalg.norm(final - planned)), 4),
        'on_floor_released': bool(float(beam_body.xpos[2]) < .03 and tilt < 10. and
                                  all(max(f) < .2 for f in forces.values())),
        'max_tilt_deg_lifted': round(stats['max_tilt_deg_lifted'], 2),
        'max_pair_distance_dev_m': round(stats['max_pair_distance_dev_m'], 4),
        'weld_eq_active_max': stats['eq_active_max'], 'final_finger_n': forces,
    }
    grips = [e for e in events if e['event'] == 'grip_view']
    evaluation['completed_sequence'] = all(s == 'done' for s in reached.values())
    evaluation['success_gt'] = bool(evaluation['completed_sequence'] and evaluation['lifted_clear_gt']
                                    and evaluation['on_floor_released'] and evaluation['final_error_m'] <= .10)
    result = {
        'schema': SCHEMA, 'profile': PROFILE, 'seed': a.seed, 'condition': a.condition, 'scenario': sc,
        'contact_profile': a.contact_profile,
        'contact_profile_note': 'cargo_noslip_v1 primary, PENDING the user decision on the profile',
        'weld': 'off', 'pose_source': ('gt_stub_eval_only (approach to pre-station only)'
                                       if a.condition == 'stub_approach' else 'none'),
        'counts_as_m1': False, 'development_seed': a.seed in DEV_SEEDS,
        'controller_inputs': 'own robot_cam JPEG + own issued PWM + task sheet + PairCarrySync barrier (own frame ids)',
        'task_sheet': {'legs': LEGS, 'speed_m_s': SPEED_M_S, 'roles': ROLES},
        'carry_odometry_calibration': {'scale': CARRY_ODOM_SCALE, 'source': CARRY_ODOM_SOURCE},
        'thresholds': {'grip_min_signature': GRIP_MIN_SIGNATURE, 'lift_min_iou': HOLD_MIN_IOU,
                       'hold_min_ratio': HOLD_MIN_RATIO, 'lost_frames': LOST_FRAMES},
        'final_states': reached, 'failures': {r: s.failure for r, s in students.items()},
        'claims': {r: s.claims for r, s in students.items()},
        'frames': {r: s.frames for r, s in students.items()}, 'commands': {r: s.commands for r, s in students.items()},
        'stub_approach': stub_log, 'barrier_events': {k: s.events for k, s in syncs.items()},
        'evaluation_only': evaluation, 'grip_views': grips,
        'sim_seconds': round(now, 3), 'wall_seconds': round(time.time() - wall0, 1),
        'load_average_start': load0, 'load_average_end': [round(x, 2) for x in os.getloadavg()],
        'source_sha': git('rev-parse', 'HEAD'), 'dirty_source': dirty,
        'scene_xml_sha256': hashlib.sha256(world.scene_xml.encode()).hexdigest(),
        'contact_profile_record': scene.manifest.get('cargo_contact_profile'),
        'threads': {k: os.environ.get(k) for k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
                                                  'VECLIB_MAXIMUM_THREADS', 'MKL_NUM_THREADS')},
    }
    (out / 'inputs.jsonl').write_text(''.join(json.dumps(f) + '\n' for f in frame_index))
    (out / 'events.jsonl').write_text(''.join(json.dumps(e, default=str) + '\n' for e in events))
    (out / 'result.json').write_text(json.dumps(result, indent=1, default=str) + '\n')
    (out / 'hashes.json').write_text(json.dumps({n: sha_file(out / n) for n in
                                                 ('result.json', 'events.jsonl', 'evaluation-only.jsonl', 'scene.xml')},
                                                indent=1) + '\n')
    print(json.dumps({'seed': a.seed, 'condition': a.condition, 'states': reached,
                      'failures': result['failures'], **{k: evaluation[k] for k in
                      ('success_gt', 'lifted_clear_gt', 'final_error_m', 'on_floor_released', 'max_tilt_deg_lifted')}}))


if __name__ == '__main__':
    main()
