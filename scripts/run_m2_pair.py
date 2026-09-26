"""M2: pair long_beam carry with NO ground truth at runtime (own RGB + static tag map + order sheet).

Two MasterPi robots (r1: end_neg, r2: end_pos) start at their depot spawns on the
tagged zone map and never receive a live pose. Each robot decides only from:

* its own ``robot_cam`` JPEG frames and its own issued commands;
* the static tagged map (``maps/zones/zone_wide_door_tags_v2.json``) and the fixed
  loop-v2 localizer calibration (PR #201 e10f88d, imported read-only);
* the COARSE ORDER SHEET: the beam pose rounded to 0.10 m / 10 deg
  (``harness.pair_owncam_approach.coarse_order_sheet``), the roles, the carry legs
  and speeds (the pair study task sheet);
* the approved pair barrier (``PairCarrySync``; readiness reports carry own frame ids);
* optionally the CANDIDATE executor status channel (``--status-channel on``,
  pending user decision; ON and OFF are reported separately).

Phases per robot: ``approach`` (own-camera localizer, ``PairApproachDriver``: look
sweep -> A* drive with final heading -> arrival look at the pre-station 0.30 m behind
its order-sheet station) -> ``wait_approach`` (barrier 'approach') -> the pair study
v3 own-RGB phases unchanged (align, grasp, lift, carry, lower, release;
``scripts/study_owncam_pair_beam.PairStudent`` imported read-only from PR #200 29700fd).

``--on-failure continue`` (M2 default): the runner never stops the partner when one
robot fails; the failed robot holds and each robot keeps acting on its own inputs
until it is done, failed, or the time limit. ``halt_all`` is the pair study behaviour
(the runner holds both robots at the first failure) and is kept only as a comparator,
because the runner's halt is information no robot has.

Truth (robot poses, localizer error, finger forces, beam pose) is written only to
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

from harness import owncam_pair_beam_v2 as ob2  # noqa: E402
from harness import owncam_pair_hold_v3 as hv3  # noqa: E402
from harness import pair_owncam_approach as pa  # noqa: E402
from harness import team_carry_status as tcs  # noqa: E402
from harness.pair_carry_sync import PairCarrySync  # noqa: E402
from scripts import study_owncam_pair_beam as study  # noqa: E402

SCHEMA = 'ugrp.m2_pair_run.v1'
PROFILE = 'm2_pair_v1'
MAP = 'zone_wide_door_tags_v2'
CALIBRATION = ROOT / 'experiments' / '2026-09-26-zone-owncam-loop-v2' / 'calibration_loop_v2.json'
ROLES = study.ROLES
FRAME_S = .2                    # approach localization frames (M1 loop runner value)
LIMIT_S = 480.
DOOR_LIMIT_S = 720.
APPROACH_LIMIT_S = 200.
APPROACH_WAIT_S = 150.          # same limit in both status-channel arms
KEEPOUT_PAD_M = .06             # order-sheet grid error (<= 0.05 m) + 1 cm
PARTNER_KEEPOUT_HALF_M = .17    # partner's order-sheet station/pre-station (robot radius)
STATUS_OF = {**study.STATUS_OF, 'approach': 'aligning', 'wait_approach': 'aligning', 'pregrasp_look': 'aligning',
             'cp_open': 'put_down'}
CONTACT_PROFILES = study.CONTACT_PROFILES
# seed -> setup beam pose (x, y, yaw) and per-robot start offsets from the map spawn (dx, dy, dyaw).
# The robots receive only coarse_order_sheet(beam); the start offsets are never given to them.
SCENARIOS = {
    # stage 1 development (open floor, west pickup floor; tuning allowed, labelled dev)
    701: {'beam': (0.62, -1.23, 0.13), 'start': {'r1': (.00, .00, .00), 'r2': (.00, .00, .00)}},
    702: {'beam': (0.47, -0.78, -0.31), 'start': {'r1': (.08, -.05, .30), 'r2': (-.06, .07, -.25)}},
    703: {'beam': (0.78, -1.62, 0.38), 'start': {'r1': (-.05, .08, -.35), 'r2': (.07, .04, .20)}},
    # stage 1 test (pre-registered, experiments/2026-09-26-zone-m2-pair/README.md; gen_stage1_seeds.py, rng 20260926)
    711: {'beam': (0.64, -1.14, 0.51), 'start': {'r1': (0.08, -0.08, -0.09), 'r2': (-0.06, 0.0, 0.34)}},
    712: {'beam': (0.53, -1.0, 0.18), 'start': {'r1': (0.05, 0.04, 0.33), 'r2': (-0.1, 0.03, -0.23)}},
    713: {'beam': (0.45, -1.12, -0.04), 'start': {'r1': (0.0, -0.08, -0.29), 'r2': (-0.08, -0.09, 0.09)}},
    714: {'beam': (0.44, -1.8, -0.21), 'start': {'r1': (-0.02, -0.03, -0.19), 'r2': (0.03, 0.01, 0.21)}},
    715: {'beam': (0.43, -1.22, -0.29), 'start': {'r1': (0.05, -0.03, 0.36), 'r2': (-0.06, 0.06, -0.01)}},
    716: {'beam': (0.66, -0.95, 0.38), 'start': {'r1': (0.0, 0.09, -0.06), 'r2': (-0.09, -0.01, -0.18)}},
    717: {'beam': (0.37, -1.64, -0.25), 'start': {'r1': (0.01, 0.08, 0.37), 'r2': (0.09, -0.07, 0.13)}},
    718: {'beam': (0.57, -1.82, -0.17), 'start': {'r1': (0.03, 0.05, 0.18), 'r2': (-0.1, 0.02, -0.01)}},
    # stage 3 failure propagation (pre-registered; gen_stage3_seeds.py, rng 20260927; drop = experimenter action)
    721: {'beam': (0.4, -1.3, -0.54), 'start': {'r1': (0.08, -0.1, 0.32), 'r2': (0.05, -0.01, 0.31)}, 'drop': 'r1:3.0'},
    722: {'beam': (0.78, -1.58, 0.52), 'start': {'r1': (-0.05, 0.02, -0.19), 'r2': (0.08, -0.03, -0.33)}, 'drop': 'r2:3.0'},
    723: {'beam': (0.35, -1.3, -0.17), 'start': {'r1': (-0.05, 0.02, -0.04), 'r2': (-0.04, -0.01, -0.18)}, 'drop': 'r1:6.0'},
    724: {'beam': (0.37, -0.46, -0.43), 'start': {'r1': (0.01, -0.09, -0.18), 'r2': (-0.02, 0.08, 0.34)}, 'drop': 'r2:6.0'},
}
DEV_SEEDS = (701, 702, 703)
STAGE1_TEST_SEEDS = tuple(range(711, 719))   # pre-registered in experiments/2026-09-26-zone-m2-pair/README.md
STAGE3_TEST_SEEDS = tuple(range(721, 725))   # failure propagation (experimenter drop), pre-registered
# ---- stage 2: carry through door_1 (0.50 m) of zone_wide_door_tags_v2 -------------------------------
# Order sheet for the door task (static): door axis y, the pair's target headings, and a FIXED axial
# carry distance (the same number for both robots, so both timed schedules have the same length).
DOOR_PLAN = {'door_id': 'door_1', 'axis_y_m': .05, 'target_beam_x_m': 3.20, 'checkpoints_beam_x_m': (1.55, 2.40),
             'headings_rad': {'r1': 0., 'r2': math.pi}}
DOOR_ALIGN_S = 6.               # own lateral/heading correction onto the door axis (both robots, from GO)
DOOR_ALIGN_MAX_M = .15          # larger own offsets are clamped (logged)
DOOR_ALIGN_MAX_RAD = .20
TURN_GAIN = 1.4885              # static drive calibration (loop-v2 motion gain, turn), unloaded
PREGRASP_MAX_SWEEPS = 2         # door stage: stationary relocalization sweeps before the grasp
PREGRASP_FIX_STD_M = .06
DOOR_SCENARIOS = {
    # stage 2 development (door carry; tuning allowed, labelled dev)
    801: {'beam': (1.00, .08, .04), 'start': {'r1': (.00, .00, .00), 'r2': (.00, .00, .00)}},
    802: {'beam': (0.93, -.02, -.07), 'start': {'r1': (.06, -.04, .25), 'r2': (-.05, .06, -.20)}},
    803: {'beam': (1.07, .12, .06), 'start': {'r1': (-.04, .07, -.30), 'r2': (.07, .03, .15)}},
    # stage 2 test (pre-registered; gen_stage2_seeds.py, rng 20260928; #202 checker feasible)
    811: {'beam': (1.01, 0.01, -0.04), 'start': {'r1': (0.07, -0.1, -0.12), 'r2': (0.03, 0.03, 0.03)}},
    812: {'beam': (0.97, 0.06, 0.08), 'start': {'r1': (0.05, -0.07, 0.29), 'r2': (0.05, -0.04, 0.26)}},
    813: {'beam': (0.98, 0.08, 0.04), 'start': {'r1': (0.05, 0.09, 0.23), 'r2': (0.03, -0.03, -0.11)}},
    814: {'beam': (1.06, 0.01, -0.04), 'start': {'r1': (0.02, -0.05, 0.1), 'r2': (0.1, 0.09, -0.13)}},
    815: {'beam': (0.94, 0.08, -0.0), 'start': {'r1': (0.08, 0.06, -0.15), 'r2': (0.03, -0.1, -0.18)}},
    816: {'beam': (0.97, 0.1, 0.05), 'start': {'r1': (0.08, 0.0, 0.11), 'r2': (0.09, -0.07, -0.08)}},
}
SCENARIOS.update(DOOR_SCENARIOS)
STAGE2_TEST_SEEDS = tuple(range(811, 817))


def git(*args):
    return subprocess.run(['git', *args], cwd=ROOT, capture_output=True, text=True).stdout.strip()


def sha_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class M2Student(study.PairStudent):
    """PairStudent (read-only import) + own-camera approach and an 'approach' barrier."""

    def __init__(self, rid, port, arm, sync_for, log, save, status, hold_check, driver, eval_hook):
        super().__init__(rid, port, arm, sync_for, log, save, status, hold_check=hold_check)
        self.driver = driver
        self.eval_hook = eval_hook
        self.state, self.state_t = 'approach', 0.
        self.next_frame = 0.
        self.approach_frames = 0
        self.approach_commands = 0

    def tick(self, now):
        if self.status is not None and self.state != 'failed':
            channel, publisher = self.status
            publisher.tick(STATUS_OF[self.state], now)
            if any(v['state'] == 'abort' for v in channel.partner_view(self.rid, now).values()):
                self.port.hold(now)
                return self.fail('PARTNER_ABORT', now)
        arm_idle = now >= self.arm.until and not self.arm.events
        handler = getattr(self, '_' + self.state, None)
        if handler is not None:
            handler(now, arm_idle)

    # ---- own-camera approach ---------------------------------------------------------------
    def _approach(self, now, arm_idle):
        import cv2
        if now - self.state_t > APPROACH_LIMIT_S:
            self.port.hold(now)
            return self.fail('APPROACH_TIMEOUT', now)
        if now + 1e-9 >= self.next_frame:
            self.next_frame = now + FRAME_S
            obs = self.look(now)
            self.approach_frames += 1
            jpeg = base64.b64decode(obs['image'])
            rgb = cv2.cvtColor(cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
            est = self.driver.observe(now, rgb)       # the student sees exactly the saved JPEG
            self.eval_hook(self.rid, now, est, self.driver.state)
        for cmd in self.driver.tick(now):
            if cmd['kind'] == 'hold':
                self.port.hold(now)
            else:
                self.port.apply(cmd, now)
                self.approach_commands += 1
                self.commands += 1
        if self.driver.outcome == 'arrived':
            est = self.driver.loc.estimate()
            self.claims['at_prestation'] = {'estimate': [round(est['x'], 4), round(est['y'], 4), round(est['yaw'], 4)],
                                            'std_xy_m': round(est['std_xy_m'], 4), 'looks': self.driver.looks,
                                            'sim_time': now}
            self.set('wait_approach', now)
        elif self.driver.outcome:
            self.port.hold(now)
            self.fail('APPROACH_' + self.driver.outcome.upper(), now)

    def _wait_approach(self, now, arm_idle):
        waited = now - self.state_t
        if self.status is not None:
            verdict = tcs.wait_verdict(self.status[0].partner_view(self.rid, now), waited, APPROACH_WAIT_S)
            # Approach barrier: the same limit as the OFF arm; the channel adds only the
            # partner abort / silent early exits (not the 75 s pre-grasp align cap).
            if verdict['verdict'] == 'ABORT' and verdict['why'] != 'partner_align_wait_exceeded':
                return self.fail(f'BARRIER_APPROACH_{verdict["why"].upper()}', now)
        if waited > APPROACH_WAIT_S:
            return self.fail('BARRIER_APPROACH_TIMEOUT', now)
        sync = self.sync_for('approach')
        if sync.authorize(now)['phase'] == 'GO':
            self.log(self.rid, 'barrier_go', now, barrier='approach')
            # Hand-off: the arm sequencer continues from this robot's own issued servo pulses.
            self.arm.commanded.update({k: v for k, v in self.driver.servo.items() if k in self.arm.commanded})
            return self.set('align_start', now)
        if now >= self.next_look:
            self.next_look = now + study.LOOK_EVERY_S
            obs = self.look(now)
            self.report('approach', obs, now, ready=True, reason='at_prestation')


class M2DoorStudent(M2Student):
    """Stage 2: the localizer keeps running (own commands) until the grasp; after the lift each robot
    puts its OWN base onto the order-sheet door axis with its own heading target (the pair formation
    rotates onto the axis because both ends go there), then the fixed axial carry from the sheet."""

    def __init__(self, *args, door_plan, axial_m, sheet_beam_x, **kw):
        super().__init__(*args, **kw)
        self.door_plan, self.axial_m = door_plan, float(axial_m)
        # Segmented carry (dev 801/802 at 076cf53: open-loop formation yaw drift 0.06-0.1 rad/m over the
        # 2.2 m carry; the held view shows only the beam): checkpoints from the order sheet, identical
        # sheet distances for both robots; at each checkpoint lower, open, relocalize, re-grasp.
        stops = [float(sheet_beam_x), *door_plan['checkpoints_beam_x_m'], door_plan['target_beam_x_m']]
        self.segments = [round(b - a, 4) for a, b in zip(stops, stops[1:])]
        self.seg = 0
        base_sync = self.sync_for
        self.sync_for = lambda key: base_sync(f'{key}@{self.seg}')
        self.grasp_estimate = None
        self.pregrasp_done = False
        self.pregrasp_sweeps = 0
        self.pg_pans = []

    # Dev 801 (0bb4d6e): dead reckoning through the align phase is not usable -- the loop-v2 motion
    # model is fit for the drive posture and over-predicted the arm-lowered align pulses ~4x (r1 grasp
    # estimate 1.37 m off). The robot therefore relocalizes from scratch while standing still, after it
    # is aligned and before it grasps (fresh localizer + the M1 wide look sweep, own frames only).
    def _queue_grasp(self, now):
        if self.pregrasp_done:
            return super()._queue_grasp(now)
        from harness.owncam_drive import LOOK_P20, WIDE_LOOK_PANS
        from harness.owncam_localizer import OwnCamLocalizer
        drv = self.driver
        drv.loc = OwnCamLocalizer(drv.map, drv.loc.params, seed=int(drv.loc.rng.integers(1 << 30)))
        drv.loc.command({'t': float(now), 'kind': 'initial_servo_command', 'pulses': dict(drv.servo)})
        self.pregrasp_sweeps += 1
        self.pg_pans = list(WIDE_LOOK_PANS)
        self.arm.queue({**LOOK_P20, 6: self.pg_pans.pop(0)}, now, duration=.8, settle=.6)
        self.set('pregrasp_look', now, sweep=self.pregrasp_sweeps)

    def _pregrasp_look(self, now, arm_idle):
        import cv2
        if not arm_idle:
            return
        obs = self.look(now)
        jpeg = base64.b64decode(obs['image'])
        rgb = cv2.cvtColor(cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
        est = self.driver.observe(now, rgb)
        self.eval_hook(self.rid, now, est, 'pregrasp_look')
        if self.pg_pans:
            self.arm.queue({6: self.pg_pans.pop(0)}, now, duration=.4, settle=.6)
            return
        est = self.driver.loc.estimate()
        ok = bool(est.get('initialized')) and est['std_xy_m'] <= PREGRASP_FIX_STD_M
        self.log(self.rid, 'pregrasp_fix', now, ok=ok, std_xy_m=est.get('std_xy_m'), sweep=self.pregrasp_sweeps)
        if ok:
            self.pregrasp_done = True
            self.arm.queue(ob2.pose_of(self.look_name), now, duration=.8, settle=.3)   # back to the align view
            return super()._queue_grasp(now)
        if self.pregrasp_sweeps >= PREGRASP_MAX_SWEEPS:
            return self.fail('DOOR_POSE_NOT_LOCALIZED', now)
        self._queue_grasp(now)

    def set(self, state, now, **detail):
        if state == 'grasp' and self.grasp_estimate is None:
            loc = self.driver.loc
            loc.predict_to(now)
            est = loc.estimate()
            self.grasp_estimate = [float(est['x']), float(est['y']), float(est['yaw'])] if est.get('initialized') else None
            self.claims['grasp_pose_estimate'] = {'xyyaw': None if self.grasp_estimate is None else
                                                  [round(v, 4) for v in self.grasp_estimate],
                                                  'std_xy_m': round(float(est.get('std_xy_m', float('nan'))), 4),
                                                  'sim_time': now}
        super().set(state, now, **detail)

    def _wait_carry(self, now, arm_idle):
        def go(t):
            self.schedule = self.door_schedule(t)
        self._wait('carry', 'carry', now, go)

    def _wait_open(self, now, arm_idle):
        if self.seg + 1 >= len(self.segments):
            return super()._wait_open(now, arm_idle)

        def go(t):
            self.arm.queue({1: study.OPEN}, t, duration=.4, settle=.5)
            self.arm.queue({**self.hover, 1: study.OPEN}, t, duration=.6)
        self._wait('open', 'cp_open', now, go)

    def _cp_open(self, now, arm_idle):
        if not arm_idle:
            return
        self.log(self.rid, 'checkpoint', now, seg=self.seg)
        self.seg += 1                      # new barrier keys from here on
        self.pregrasp_done = False
        self.pregrasp_sweeps = 0
        self.grasp_estimate = None
        self.claims.pop('door_align', None)
        self._queue_grasp(now)             # relocalize (own sweep), then re-grasp at the unchanged arm pose

    def door_schedule(self, t0):
        sign = 1. if ROLES[self.rid] == 'end_neg' else -1.
        out, t = [], t0
        if self.grasp_estimate is not None:
            x, y, yaw = self.grasp_estimate
            dy = float(np.clip(self.door_plan['axis_y_m'] - y, -DOOR_ALIGN_MAX_M, DOOR_ALIGN_MAX_M))
            e_yaw = float(np.clip(study.wrap(self.door_plan['headings_rad'][self.rid] - yaw),
                                  -DOOR_ALIGN_MAX_RAD, DOOR_ALIGN_MAX_RAD))
            fwd_m, left_m = math.sin(yaw) * dy, math.cos(yaw) * dy      # world (0, dy) in the own body frame
            cmd = {'forward': fwd_m / DOOR_ALIGN_S / (study.FORWARD_GAIN * study.CARRY_ODOM_SCALE['axial']),
                   'left': left_m / DOOR_ALIGN_S / (study.LEFT_GAIN * study.CARRY_ODOM_SCALE['lateral']),
                   'turn': e_yaw / DOOR_ALIGN_S / TURN_GAIN}
            self.claims['door_align'] = {'dy_m': round(dy, 4), 'e_yaw_rad': round(e_yaw, 4),
                                         'cmd': {k: round(v, 4) for k, v in cmd.items()}}
            out.append((t, t + DOOR_ALIGN_S, cmd))
            t += DOOR_ALIGN_S + .5
        else:
            self.claims['door_align'] = {'skipped': 'no initialised estimate at grasp'}
        dist = self.segments[self.seg]
        self.claims.setdefault('segments', []).append({'seg': self.seg, 'axial_m': dist,
                                                       'door_align': self.claims.get('door_align')})
        dur = dist / (study.SPEED_M_S * study.CARRY_ODOM_SCALE['axial'])
        out.append((t, t + dur, {'forward': sign * study.SPEED_M_S / study.FORWARD_GAIN, 'left': 0., 'turn': 0.}))
        return out


def main():
    import mujoco
    from scripts.record_owncam_localization import LoggingPort
    from scripts.zone_teacher import FOLDED, ArmSequence
    from sim.camera_robot_port import CameraRobotPort
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.zone_cargo import instances, world_grasps
    from sim.zone_tagged_cargo_scene import TaggedCargoZoneScene
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--seed', type=int, choices=sorted(SCENARIOS), required=True)
    p.add_argument('--stage', choices=('open_floor', 'door'), default='open_floor')
    p.add_argument('--contact-profile', choices=CONTACT_PROFILES, default='cargo_noslip_v1')
    p.add_argument('--status-channel', choices=('on', 'off'), required=True,
                   help='CANDIDATE executor status channel (pending user decision); off = barrier only')
    p.add_argument('--on-failure', choices=('continue', 'halt_all'), default='continue',
                   help='continue: runner never stops the partner (M2); halt_all: pair study comparator')
    p.add_argument('--hold-check', choices=study.HOLD_CHECKS, default='fullframe_v3')
    p.add_argument('--approach', choices=('v1', 'v2'), default='v1',
                   help='approach driver: v1 (stage 1/3 cohorts) or v2 (turn in place first + relocalize)')
    p.add_argument('--inject-drop', default=None,
                   help='EXPERIMENTER deliberate drop "<robot>:<seconds after its carry start>" (evaluation only)')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--allow-dirty', action='store_true')
    a = p.parse_args()
    injection = None
    if a.inject_drop:
        rid_inj, secs = a.inject_drop.split(':')
        if rid_inj not in ROLES:
            raise SystemExit('--inject-drop robot must be r1 or r2')
        injection = {'robot': rid_inj, 'after_carry_start_s': float(secs), 'applied_at_s': None,
                     'action': 'gripper servo 1 -> OPEN (experimenter, not a controller command)'}
    dirty = bool(git('status', '--porcelain'))
    if dirty and not a.allow_dirty:
        raise SystemExit('commit and freeze the source before a recorded run (or --allow-dirty)')
    out = a.output
    out.mkdir(parents=True, exist_ok=False)
    (out / 'inputs').mkdir()
    wall0, load0 = time.time(), [round(v, 2) for v in os.getloadavg()]
    sc = SCENARIOS[a.seed]
    if (a.seed in DOOR_SCENARIOS) != (a.stage == 'door'):
        raise SystemExit('seed and --stage do not match')
    item = {'item_id': 'beam', 'kind': 'long_beam', 'pose': list(sc['beam'])}
    scene = TaggedCargoZoneScene.from_tagged_cargo(MAP, 11, cargo=[item], goal={'A': {'cyan': 1}},
                                                   contact_profile=a.contact_profile)
    world = MultiMasterPiProductionV2(seed=a.seed, width=640, height=480, render=True,
                                      warehouse_layout=scene.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=scene.transform)
    scene.setup(world)
    m, d = world.model, world.data
    static = scene.config['static_map']
    spawns = scene.config['setup_only']['spawns']
    inst = instances([item])[0]
    # ---- order sheet (static task information given to the robots) ------------------------
    sheet = pa.coarse_order_sheet(sc['beam'])
    sheet_grasps = world_grasps(inst, pose=sheet['beam_xyyaw'])      # catalogue geometry at the SHEET pose
    spec = inst.spec()
    bar = next(pp for pp in spec.parts if pp.name == 'bar')
    beam_len, beam_w = 2 * bar.size[0], 2 * bar.size[1]
    stations_sheet = {r: list(sheet_grasps[role]['base_xyyaw']) for r, role in ROLES.items()}
    prestations = {r: pa.prestation(stations_sheet[r], study.PRESTATION_BACK_M) for r in ROLES}
    true_grasps = world_grasps(inst)                                   # evaluation only
    stations_true = {r: list(true_grasps[role]['base_xyyaw']) for r, role in ROLES.items()}
    # ---- setup-only start poses --------------------------------------------------------------
    for rid in ROLES:
        sx, sy, _z, syaw = spawns[rid]
        dx, dy, dyaw = sc['start'][rid]
        world.robot(rid).set_base_pose_for_test((sx + dx, sy + dy, study.BASE_Z), syaw + dyaw)
    mujoco.mj_forward(m, d)
    (out / 'scene.xml').write_text(world.scene_xml)
    raw = {r: CameraRobotPort(world, r, allow_reverse=True, allow_mecanum=True) for r in ('r1', 'r2', 'r3')}
    calibration = json.loads(CALIBRATION.read_text())
    commands = {r: [] for r in ROLES}
    feed_stop = {r: False for r in ROLES}
    drivers, ports = {}, dict(raw)
    for rid in ROLES:
        partner = next(r for r in ROLES if r != rid)
        keepouts = [pa.beam_keepout(sheet['beam_xyyaw'], beam_len, beam_w, KEEPOUT_PAD_M)]
        for tag, (px, py, _pyaw) in (('station', stations_sheet[partner]), ('prestation', prestations[partner])):
            keepouts.append({'id': f'partner_{tag}', 'center_m': [px, py],
                             'half_extents_m': [PARTNER_KEEPOUT_HALF_M, PARTNER_KEEPOUT_HALF_M],
                             'source': "partner's order-sheet station (static task sheet), not a live pose"})
        initial = {int(k): int(v) for k, v in world.robot(rid).servo_command_pulses.items()}
        driver_cls = pa.PairApproachDriverV2 if a.approach == 'v2' else pa.PairApproachDriver
        drivers[rid] = driver_cls(static, calibration['params'], goal_xyyaw=prestations[rid],
                                             door_xy=None, keepouts=keepouts, initial_servo=initial, seed=a.seed)

        def sink(row, rid=rid):
            commands[rid].append(row)
            if drivers[rid].outcome is None or (a.stage == 'door' and not feed_stop[rid]):
                drivers[rid].on_command(row)       # open floor: until the pre-station; door: until the grasp
        sink({'t': 0.0, 'kind': 'initial_servo_command', 'pulses': dict(initial)})
        ports[rid] = LoggingPort(raw[rid], sink)
    events = []

    def log(rid, kind, now, **detail):
        events.append({'robot': rid, 'event': kind, 'sim_time_s': round(now, 3), **detail})

    syncs = {}

    def sync_for(key):
        if key not in syncs:
            syncs[key] = PairCarrySync(f'beam-{key}', participants=tuple(ROLES), report_ttl_s=study.BARRIER_TTL_S)
        return syncs[key]

    arms = {r: ArmSequence(ports[r], FOLDED) for r in ROLES}
    frame_index = []

    def save(rid, obs, state):
        name = f"{rid}-{obs['frame_id']:05d}.jpg"
        (out / 'inputs' / name).write_bytes(base64.b64decode(obs['image']))
        frame_index.append({'file': name, 'robot': rid, 'state': state, 'sim_time': obs['sim_time'],
                            'sha256': obs['sha256'], 'own_pose_commands': obs['actuator_state']['servo_pulses']})

    loc_eval = []

    def eval_hook(rid, now, est, driver_state):          # evaluation only: never read by a robot
        r = world.robot(rid)
        gx, gy, gyaw = float(r.base_xyz()[0]), float(r.base_xyz()[1]), float(r.base_rpy()[2])
        row = {'t': round(now, 3), 'robot': rid, 'driver_state': driver_state, 'gt': [round(gx, 4), round(gy, 4), round(gyaw, 4)],
               'initialized': bool(est.get('initialized')), 'tags': est.get('tags', [])}
        if est.get('initialized'):
            row.update(est=[round(est['x'], 4), round(est['y'], 4), round(est['yaw'], 4)],
                       pos_err_m=round(math.hypot(est['x'] - gx, est['y'] - gy), 4),
                       yaw_err_rad=round(abs(study.wrap(est['yaw'] - gyaw)), 4), std_xy_m=round(est['std_xy_m'], 4))
        loc_eval.append(row)

    channel = tcs.StatusChannel('beam-carry', tuple(ROLES)) if a.status_channel == 'on' else None
    if a.stage == 'door':
        axial_m = DOOR_PLAN['target_beam_x_m'] - sheet['beam_xyyaw'][0]
        students = {r: M2DoorStudent(r, ports[r], arms[r], sync_for, log, save,
                                     (channel, tcs.StatusPublisher(channel, r)) if channel else None,
                                     a.hold_check, drivers[r], eval_hook, door_plan=DOOR_PLAN, axial_m=axial_m,
                                     sheet_beam_x=sheet['beam_xyyaw'][0])
                    for r in ROLES}
    else:
        students = {r: M2Student(r, ports[r], arms[r], sync_for, log, save,
                                 (channel, tcs.StatusPublisher(channel, r)) if channel else None,
                                 a.hold_check, drivers[r], eval_hook) for r in ROLES}
    # ---- evaluation-only truth ------------------------------------------------------------
    truth = (out / 'evaluation-only.jsonl').open('w')
    beam_body = d.body(inst.body)
    cargo_geoms = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, inst.geom(pp.name))
                   for pp in spec.parts if pp.collision}
    finger_ids = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f'{r}__{s}_finger'): (r, k)
                  for r in ROLES for k, s in enumerate(('left', 'right'))}
    robot_geoms = {r: {g for g in range(m.ngeom) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or '').startswith(r + '__')}
                   for r in ('r1', 'r2', 'r3')}
    wall_geoms = {g for g in range(m.ngeom) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or '').startswith('zone_wall_')}
    stats = {'eq_active_max': 0, 'max_beam_z_m': 0., 'max_tilt_deg_lifted': 0., 'max_pair_distance_dev_m': 0.,
             'robot_robot_contacts': 0, 'robot_wall_contacts': 0, 'robot_beam_contacts_before_align': 0,
             'beam_disp_before_grasp_m': 0., 'beam_wall_contacts': 0}
    beam_start = np.array(beam_body.xpos[:2], float)
    nominal_pair_dist = math.hypot(stations_true['r1'][0] - stations_true['r2'][0],
                                   stations_true['r1'][1] - stations_true['r2'][1])

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

    def contacts_check():
        pairs = {(int(d.contact[i].geom1), int(d.contact[i].geom2)) for i in range(d.ncon)}
        rr = rw = rb = False
        for g1, g2 in pairs:
            for x, y in ((g1, g2), (g2, g1)):
                ra = next((r for r in ROLES if x in robot_geoms[r]), None)
                if ra is None:
                    continue
                if any(y in robot_geoms[o] for o in ('r1', 'r2', 'r3') if o != ra):
                    rr = True
                if y in wall_geoms:
                    rw = True
                if y in cargo_geoms and students[ra].state in ('approach', 'wait_approach'):
                    rb = True
        stats['robot_robot_contacts'] += int(rr)
        stats['robot_wall_contacts'] += int(rw)
        stats['beam_wall_contacts'] += int(any((g1 in cargo_geoms and g2 in wall_geoms) or (g2 in cargo_geoms and g1 in wall_geoms)
                                               for g1, g2 in pairs))
        stats['robot_beam_contacts_before_align'] += int(rb)

    def sample(now):
        mat = beam_body.xmat.reshape(3, 3)
        tilt = math.degrees(math.acos(max(-1., min(1., float(mat[2, 2])))))
        z = float(beam_body.xpos[2])
        rp = {r: [round(float(v), 4) for v in (*world.robot(r).base_xyz()[:2], world.robot(r).base_rpy()[2])]
              for r in ROLES}
        dist = math.hypot(rp['r1'][0] - rp['r2'][0], rp['r1'][1] - rp['r2'][1])
        stats['max_beam_z_m'] = max(stats['max_beam_z_m'], z)
        if not any(s.claims.get('gripped') for s in students.values()):
            stats['beam_disp_before_grasp_m'] = max(stats['beam_disp_before_grasp_m'],
                                                    float(np.linalg.norm(np.array(beam_body.xpos[:2]) - beam_start)))
        if z > .03:
            stats['max_tilt_deg_lifted'] = max(stats['max_tilt_deg_lifted'], tilt)
            stats['max_pair_distance_dev_m'] = max(stats['max_pair_distance_dev_m'], abs(dist - nominal_pair_dist))
        fingers = finger_forces()
        if z > .03 and all(min(v) > 1. for v in fingers.values()):
            stats['both_gripped_lifted_samples'] = stats.get('both_gripped_lifted_samples', 0) + 1
        truth.write(json.dumps({'t': round(now, 2), 'states': {r: s.state for r, s in students.items()},
                                'beam_xyz': [round(float(v), 4) for v in beam_body.xpos], 'beam_tilt_deg': round(tilt, 2),
                                'robots': rp, 'pair_distance_m': round(dist, 4),
                                'finger_n': {r: [round(x, 2) for x in v] for r, v in fingers.items()}}) + '\n')

    first_failure = None
    dt = float(m.opt.timestep)
    next_ctrl, next_sample, next_arm = 0., 0., 0.
    terminal = ('done', 'failed')
    try:
        while float(d.time) < (DOOR_LIMIT_S if a.stage == 'door' else LIMIT_S):
            now = float(d.time)
            if now >= next_ctrl:
                next_ctrl = now + study.CONTROL_S
                for rid, st in students.items():
                    was = st.state
                    st.tick(now)
                    if st.state == 'failed' and was != 'failed':
                        ports[rid].hold(now)                   # the failed robot's own stop
                        if first_failure is None:
                            first_failure = {'robot': rid, 'reason': st.failure, 'sim_time_s': round(now, 3),
                                             'state_before': was,
                                             'beam_z_m': round(float(beam_body.xpos[2]), 4)}
                if injection is not None and injection['applied_at_s'] is None:
                    victim = students[injection['robot']]
                    if victim.state == 'carry' and now - victim.state_t >= injection['after_carry_start_s']:
                        ports[injection['robot']].apply({'kind': 'arm', 'servo_id': 1, 'pulse': study.OPEN}, now)
                        injection['applied_at_s'] = round(now, 3)
                if first_failure is not None and a.on_failure == 'halt_all':
                    for rid in ROLES:
                        ports[rid].hold(now)
                    break
                if all(s.state in terminal for s in students.values()):
                    break
            if now >= next_arm:
                next_arm = now + .05
                for arm in arms.values():
                    arm.tick(now)
            for port in raw.values():
                port.tick(now)
            world._physics_step_for(world.controllers['r1'])
            if d.eq_active.any():
                stats['eq_active_max'] = 1
                raise RuntimeError('weld/equality became active')
            if now >= next_sample:
                next_sample = now + .2
                sample(now)
                contacts_check()
    finally:
        if not truth.closed:
            sample(float(d.time))        # final evaluation-only sample (post-failure state is always recorded)
        truth.close()
    # ---- evaluation (truth) ----------------------------------------------------------------
    now = float(d.time)
    mat = beam_body.xmat.reshape(3, 3)
    tilt = math.degrees(math.acos(max(-1., min(1., float(mat[2, 2])))))
    u = np.array([math.cos(sc['beam'][2]), math.sin(sc['beam'][2])])
    v = np.array([-u[1], u[0]])
    planned = beam_start + study.LEGS[0][1] * u + study.LEGS[1][1] * v
    if a.stage == 'door':
        planned = np.array([DOOR_PLAN['target_beam_x_m'], DOOR_PLAN['axis_y_m']])
    final = np.array(beam_body.xpos[:2], float)
    forces = finger_forces()
    reached = {r: s.state for r, s in students.items()}
    # approach accuracy at the hand-off (truth vs the robot's pre-station goal and true station)
    approach_eval = {}
    for rid, st in students.items():
        rows = [r for r in loc_eval if r['robot'] == rid]
        errs = [r['pos_err_m'] for r in rows if 'pos_err_m' in r]
        go = next((e for e in events if e['robot'] == rid and e['event'] == 'barrier_go'
                   and e.get('barrier') == 'approach'), None)
        arr = st.claims.get('at_prestation')
        true_pre = pa.prestation(stations_true[rid], study.PRESTATION_BACK_M)
        at_arrival = None
        if arr is not None:
            row = min(rows, key=lambda r: abs(r['t'] - arr['sim_time'])) if rows else None
            if row is not None:
                gx, gy, gyaw = row['gt']
                at_arrival = {'gt': row['gt'], 'pos_err_to_goal_m': round(math.hypot(gx - prestations[rid][0], gy - prestations[rid][1]), 4),
                              'pos_err_to_true_prestation_m': round(math.hypot(gx - true_pre[0], gy - true_pre[1]), 4),
                              'yaw_err_to_true_rad': round(abs(study.wrap(gyaw - true_pre[2])), 4),
                              'localizer_pos_err_m': row.get('pos_err_m')}
        approach_eval[rid] = {'frames': st.approach_frames, 'commands': st.approach_commands,
                              'looks': st.driver.looks, 'outcome': st.driver.outcome,
                              'arrived_s': None if arr is None else round(arr['sim_time'], 3),
                              'approach_go_s': None if go is None else go['sim_time_s'],
                              'localizer_pos_err_p50_m': round(float(np.median(errs)), 4) if errs else None,
                              'localizer_pos_err_max_m': round(float(np.max(errs)), 4) if errs else None,
                              'at_arrival': at_arrival}
    # failure propagation (evaluation only): what the partner did after the first failure
    propagation = None
    if first_failure is not None:
        victim = first_failure['robot']
        partner = next(r for r in ROLES if r != victim)
        t0 = first_failure['sim_time_s']
        motion_after = [c for c in commands[partner] if c['t'] > t0 and c['kind'] in ('mecanum', 'drive')]
        arm_after = [c for c in commands[partner] if c['t'] > t0 and c['kind'] in ('arm', 'look')]
        pf = next((e for e in events if e['robot'] == partner and e['event'] == 'state'
                   and e.get('state') in terminal and e['sim_time_s'] >= t0), None)
        truth_rows = [json.loads(line) for line in (out / 'evaluation-only.jsonl').read_text().splitlines()]
        after = [r for r in truth_rows if r['t'] >= t0]
        propagation = {'first_failure': first_failure, 'partner': partner,
                       'partner_terminal_state': reached[partner], 'partner_failure': students[partner].failure,
                       'partner_terminal_latency_s': None if pf is None else round(pf['sim_time_s'] - t0, 3),
                       'partner_motion_commands_after': len(motion_after),
                       'partner_arm_commands_after': len(arm_after),
                       'partner_last_motion_after_s': round(motion_after[-1]['t'] - t0, 3) if motion_after else None,
                       'beam_max_tilt_deg_after': max((r['beam_tilt_deg'] for r in after), default=None),
                       'beam_final_z_m': round(float(beam_body.xpos[2]), 4),
                       'on_failure_mode': a.on_failure}
    evaluation = {
        'both_gripped_gt': stats.get('both_gripped_lifted_samples', 0) > 0,
        'both_gripped_lifted_samples': stats.get('both_gripped_lifted_samples', 0), 'lifted_clear_gt': stats['max_beam_z_m'] > .06,
        'beam_final_xyz': [round(float(x), 4) for x in beam_body.xpos], 'beam_final_tilt_deg': round(tilt, 2),
        'planned_beam_xy': [round(float(x), 4) for x in planned],
        'final_error_m': round(float(np.linalg.norm(final - planned)), 4),
        'on_floor_released': bool(float(beam_body.xpos[2]) < .03 and tilt < 10. and
                                  all(max(f) < .2 for f in forces.values())),
        'max_tilt_deg_lifted': round(stats['max_tilt_deg_lifted'], 2),
        'max_pair_distance_dev_m': round(stats['max_pair_distance_dev_m'], 4),
        'beam_disp_before_grasp_m': round(stats['beam_disp_before_grasp_m'], 4),
        'robot_robot_contact_samples': stats['robot_robot_contacts'],
        'robot_wall_contact_samples': stats['robot_wall_contacts'],
        'beam_wall_contact_samples': stats['beam_wall_contacts'],
        'robot_beam_contact_samples_in_approach': stats['robot_beam_contacts_before_align'],
        'weld_eq_active_max': stats['eq_active_max'], 'final_finger_n': forces,
        'approach': approach_eval, 'failure_propagation': propagation,
        'order_sheet_error': {'xy_m': round(math.hypot(sheet['beam_xyyaw'][0] - sc['beam'][0],
                                                       sheet['beam_xyyaw'][1] - sc['beam'][1]), 4),
                              'yaw_rad': round(abs(study.wrap(sheet['beam_xyyaw'][2] - sc['beam'][2])), 4)},
    }
    evaluation['completed_sequence'] = all(s == 'done' for s in reached.values())
    evaluation['drop_injection'] = injection
    evaluation['success_gt'] = bool(evaluation['completed_sequence'] and evaluation['lifted_clear_gt']
                                    and evaluation['on_floor_released'] and evaluation['final_error_m'] <= .10)
    if a.stage == 'door':
        rows = [json.loads(line) for line in (out / 'evaluation-only.jsonl').read_text().splitlines()]
        cross = next((r for r in rows if r['beam_xyz'][0] >= 2.2), None)
        passed = float(beam_body.xpos[0]) >= 2.7
        evaluation['door'] = {'plan': DOOR_PLAN, 'beam_passed_x_ge_2_7': passed,
                              'beam_y_offset_at_crossing_m': None if cross is None else
                              round(cross['beam_xyz'][1] - DOOR_PLAN['axis_y_m'], 4),
                              'beam_z_at_crossing_m': None if cross is None else cross['beam_xyz'][2],
                              'crossing_t': None if cross is None else cross['t'],
                              'door_align_claims': {r: s.claims.get('door_align') for r, s in students.items()},
                              'grasp_estimates': {r: s.claims.get('grasp_pose_estimate') for r, s in students.items()},
                              'success_rule': 'completed + lifted + released on floor + beam x >= 2.7 + final error <= 0.25 m'}
        evaluation['success_gt'] = bool(evaluation['completed_sequence'] and evaluation['lifted_clear_gt']
                                        and evaluation['on_floor_released'] and passed
                                        and evaluation['final_error_m'] <= .25)
    grips = [e for e in events if e['event'] == 'grip_view']
    result = {
        'schema': SCHEMA, 'profile': PROFILE, 'seed': a.seed, 'stage': a.stage, 'map': MAP,
        'scenario_setup_only': sc, 'order_sheet': sheet,
        'order_sheet_stations': stations_sheet, 'prestations': prestations,
        'contact_profile': a.contact_profile,
        'contact_profile_note': 'cargo_noslip_v1 primary, PENDING the user decision on the profile',
        'weld': 'off', 'pose_source': 'owncam_pf_v2 localizer (own RGB + static tag map + own commands); no GT',
        'gt_at_runtime': False, 'on_failure': a.on_failure,
        'development_seed': a.seed in DEV_SEEDS, 'stage1_test_seed': a.seed in STAGE1_TEST_SEEDS,
        'stage3_test_seed': a.seed in STAGE3_TEST_SEEDS, 'stage2_test_seed': a.seed in STAGE2_TEST_SEEDS,
        'approach_version': a.approach,
        'imports': 'experiments/2026-09-26-zone-m2-pair/imports.json (byte-identical, read-only)',
        'perception': ob2.PROFILE, 'hold_check': {'selected': a.hold_check, 'profile': hv3.PROFILE},
        'approach_driver': {'schema': pa.SCHEMA, 'version': drivers['r1'].version,
                            'calibration': str(CALIBRATION.relative_to(ROOT)), 'calibration_sha256': sha_file(CALIBRATION),
                            'envelope': pa.APPROACH_ENVELOPE, 'frame_s': FRAME_S,
                            'events': {r: drivers[r].log for r in ROLES}},
        'status_channel': {'enabled': a.status_channel == 'on', 'profile': tcs.PROFILE,
                           'status': 'CANDIDATE, pending user decision; executor states only, no free text, no GT',
                           'messages': len(channel.log) if channel else 0,
                           'rejected': channel.rejected if channel else [],
                           'partner_aligning_waits': {r: s.status_waits for r, s in students.items()}},
        'controller_inputs': ('own robot_cam JPEG + own issued commands + static tag map + loop-v2 calibration + '
                              'coarse order sheet (beam pose on 0.10 m / 10 deg grid, roles, legs) + PairCarrySync '
                              'barrier (own frame ids)' + (' + partner executor status (candidate channel)' if channel else '')),
        'task_sheet': {'legs': study.LEGS, 'speed_m_s': study.SPEED_M_S, 'roles': ROLES,
                       'prestation_back_m': study.PRESTATION_BACK_M},
        'final_states': reached, 'failures': {r: s.failure for r, s in students.items()},
        'claims': {r: s.claims for r, s in students.items()},
        'frames': {r: s.frames for r, s in students.items()}, 'commands': {r: s.commands for r, s in students.items()},
        'barrier_events': {k: s.events for k, s in syncs.items()},
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
    (out / 'commands.jsonl').write_text(''.join(json.dumps({'robot': r, **c}) + '\n' for r in ROLES for c in commands[r]))
    (out / 'localizer-eval.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in loc_eval))
    (out / 'status-channel.jsonl').write_text(''.join(json.dumps(mm) + '\n' for mm in (channel.log if channel else [])))
    (out / 'events.jsonl').write_text(''.join(json.dumps(e, default=str) + '\n' for e in events))
    (out / 'result.json').write_text(json.dumps(result, indent=1, default=str) + '\n')
    (out / 'hashes.json').write_text(json.dumps({n: sha_file(out / n) for n in
                                                 ('result.json', 'events.jsonl', 'evaluation-only.jsonl', 'scene.xml',
                                                  'commands.jsonl', 'localizer-eval.jsonl', 'inputs.jsonl')},
                                                indent=1) + '\n')
    print(json.dumps({'seed': a.seed, 'status_channel': a.status_channel, 'on_failure': a.on_failure,
                      'states': reached, 'failures': result['failures'],
                      'approach': {r: {k: approach_eval[r][k] for k in ('outcome', 'arrived_s', 'looks')} for r in ROLES},
                      **{k: evaluation[k] for k in ('success_gt', 'lifted_clear_gt', 'final_error_m', 'on_floor_released',
                                                    'max_tilt_deg_lifted', 'robot_robot_contact_samples')},
                      'sim_s': round(now, 1), 'wall_s': result['wall_seconds']}))


if __name__ == '__main__':
    main()
