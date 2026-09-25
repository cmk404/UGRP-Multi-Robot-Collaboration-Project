#!/usr/bin/env python3
"""Side-effect audit of a cargo contact profile (PHYSICS A/B, GT teacher, weld OFF).

``cargo_noslip_v1`` (merged PR #167, ``sim/zone_cargo_contact.py``) is
``local_contact_fine`` plus ``noslip_iterations 10``. ``noslip_iterations`` is a
GLOBAL MuJoCo solver option, so selecting the profile for team cargo also
changes every other contact of the scene: the four wheel/floor contacts of each
robot, the arm, the walls and the resting boxes. This runner measures those
contacts under an explicitly selected profile so the two profiles can be
compared scenario by scenario.

This is a physics audit, not a robot success measurement. Commands come from a
fixed scripted tape (drive, wall, arm, idle) or from the ground-truth
``cargo_formation_teacher`` (carry scenarios). Nothing here is an RGB/student
result, no weld or equality constraint is used (asserted every physics step),
and no pose is written after the fixture setup.

Pre-registration, tolerances and results:
``experiments/2026-09-26-noslip-side-effects/README.md``.

Sim step cost / wall time is deliberately NOT a metric: the research question is
SIM-time robot behaviour. Host load averages are recorded per run as required by
AGENTS.md.

  .venv-sim/bin/python -m scripts.audit_contact_profiles --scenario drive_commands \
      --contact-profile local_contact_fine --seed 11 --output outputs/noslip-audit/d11-lcf
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import platform
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

SCHEMA = 'ugrp.contact_profile_side_effect_audit.v1'
PROFILES = ('local_contact_fine', 'cargo_noslip_v1')
VARIANT = 'zone_wide_door'
GOAL = {'B': {'cyan': 1}}
BASE_Z = .032355118817659255
SAMPLE_S = .05
ARM_SERVOS = ('arm_yaw', 'shoulder', 'elbow', 'wrist_pitch')

# --- scripted command tapes -------------------------------------------------
# Identical for both profiles by construction, so a drive/arm/wall difference is
# a physics difference and not a controller reaction. ``dwell_s`` keeps the
# wheels stopped afterwards so each segment includes its own coast to rest.


def _mecanum(forward=0., left=0., turn=0., duration_s=1.):
    return {'kind': 'mecanum', 'forward': forward, 'left': left, 'turn': turn, 'duration_s': duration_s}


def _drive_tape():
    tape = []
    for i in range(3):
        tape.append((f'straight_{i+1}', _mecanum(forward=.10), 1.2))
    for i in range(3):
        tape.append((f'strafe_{i+1}', _mecanum(left=.08), 1.2))
    for i in range(3):
        tape.append((f'rotate_{i+1}', _mecanum(turn=.12), 1.2))
    for i in range(2):
        tape.append((f'reverse_{i+1}', _mecanum(forward=-.05), 1.2))
    return tape


def _wall_tape():
    # Head-on press, then a diagonal press that would slide along the wall face,
    # then a reverse escape. noslip removes residual slip of friction rows, so
    # the along-wall slide is the segment most likely to change.
    return ([(f'approach_{i+1}', _mecanum(forward=.10), .2) for i in range(4)]
            + [(f'press_{i+1}', _mecanum(forward=.10), .2) for i in range(4)]
            + [(f'slide_{i+1}', _mecanum(forward=.10, left=.08), .2) for i in range(8)]
            + [(f'escape_{i+1}', _mecanum(forward=-.05), 1.0) for i in range(4)])


def _arm_tape():
    # From FOLDED (1:2000, 3:740, 4:2320, 5:1320) through a look pose, the v1/v2
    # carry pose, an empty-jaw close/open (finger-finger contact) and back to
    # folded. One servo per command, as the CameraRobotPort contract requires.
    poses = [(3, 1100), (4, 1900), (5, 1500),
             (3, 777), (4, 2053), (5, 1646),
             (1, 1500), (1, 2000),
             (3, 740), (4, 2320), (5, 1320)]
    return [(f'arm_{i+1}_s{servo}_{pulse}', {'kind': 'arm', 'servo_id': servo, 'pulse': pulse}, .4)
            for i, (servo, pulse) in enumerate(poses)]


SCENARIOS = {
    'drive_commands': {
        'kind': 'tape', 'tape': _drive_tape, 'robots': ('r1',), 'cargo': None,
        'start': {'r1': (3.60, -2.40, 0.)},
        'question': 'mecanum drive response to issued commands (straight/strafe/rotate/reverse)',
        'metric': 'displacement and heading change per issued command'},
    'wall_push': {
        'kind': 'tape', 'tape': _wall_tape, 'robots': ('r1',), 'cargo': None,
        'start': {'r1': (2.72, -2.30, math.pi)},
        'question': 'wall/obstacle contact behaviour (head-on, along-wall slide, escape)',
        'metric': 'penetration, normal force, along-wall slide, escape distance'},
    'idle_settle': {
        'kind': 'tape', 'tape': lambda: [], 'robots': (), 'cargo': None, 'limit_s': 20.,
        'question': 'resting scene bodies (painted boxes, beam, warehouse props)',
        'metric': 'free-body displacement with no command'},
    'arm_sweep': {
        'kind': 'tape', 'tape': _arm_tape, 'robots': ('r1',), 'cargo': None,
        'question': 'arm servo response with empty jaws',
        'metric': 'grip point and arm joint angle per commanded pose'},
    'solo_carry': {
        'kind': 'carry', 'roles': {'r1': 'west'}, 'cargo_kind': 'cal_block', 'mass_kg': .030,
        'hold_s': 120., 'legs': [('move', 1.0)], 'cargo_start': (3.30, -2.40, 0.),
        'question': 'solo box grasp-lift-hold(120 s)-carry slip in the jaws',
        'metric': 'grip slip in the cargo frame, creep rate, drop, placement error'},
    'solo_hold_load': {
        'kind': 'carry', 'roles': {'r1': 'west'}, 'cargo_kind': 'cal_block', 'mass_kg': .300,
        'hold_s': 120., 'legs': [], 'cargo_start': (3.30, -2.40, 0.),
        'question': 'same hold at 10x the load (creep-vs-load scaling)',
        'metric': 'grip slip and creep rate'},
    'pair_beam_hold': {
        'kind': 'carry', 'roles': {'r1': 'end_neg', 'r2': 'end_pos'}, 'cargo_kind': 'long_beam',
        'hold_s': 120., 'legs': [], 'cargo_start': (3.30, -2.10, 0.),
        'question': 'pair beam hold (two carriers, 0.30 kg beam)',
        'metric': 'per-robot grip slip, beam tilt, finger forces, formation drift'},
}

# --- pre-registered "no side effect" tolerances -----------------------------
# Written before the runs (see the experiment README). Justified against the
# behavioural thresholds already in the code: the formation teacher aligns to
# APPROACH_TOL_M = 6 mm, the zone slot judge allows +-60 mm, and the wrist skill
# re-seats on an 18 px anchor move. A difference below these tolerances cannot
# change those decisions.
TOLERANCES = {
    'drive_commands': {
        'per_command_disp_mm': 2.0, 'per_command_disp_frac': .01,
        'per_command_yaw_deg': .5, 'final_pose_mm': 10.0, 'final_yaw_deg': .5},
    'wall_push': {
        'max_penetration_mm': .5, 'along_wall_slide_mm': 5.0, 'along_wall_slide_frac': .10,
        'escape_mm': 2.0, 'escape_floor_mm': 20.0},
    'idle_settle': {'body_drift_mm': .5, 'body_drift_diff_mm': .5},
    'arm_sweep': {'grip_point_mm': 1.0, 'joint_deg': .2},
    'solo_carry': {'placement_mm': 5.0, 'finger_force_frac': .10},
    'solo_hold_load': {'finger_force_frac': .10},
    'pair_beam_hold': {'tilt_deg': 1.0, 'finger_force_frac': .10, 'formation_mm': 5.0},
}
# Gates that every run must satisfy regardless of profile.
HARD_GATES = {'eq_active_max': 0, 'max_qvel_abs': 50., 'nan_samples': 0}


def _tol(scenario, key):
    return TOLERANCES[scenario][key]


def compare(scenario, baseline, candidate):
    """Pre-registered per-scenario comparison: [{gate, baseline, candidate, diff, limit, ok}].

    ``baseline`` / ``candidate`` are the ``metrics`` blocks of two runs of the
    same scenario and seed. Pure function, unit-tested; it never reads a model.
    """
    gates = []

    def gate(name, base, cand, limit, diff=None):
        diff = abs(cand - base) if diff is None else diff
        gates.append({'gate': name, 'baseline': base, 'candidate': cand,
                      'diff': round(float(diff), 6), 'limit': round(float(limit), 6),
                      'ok': bool(diff <= limit + 1e-12)})

    if scenario == 'drive_commands':
        t = TOLERANCES[scenario]
        for a, b in zip(baseline['segments'], candidate['segments']):
            if a['label'] != b['label']:
                raise ValueError('segment tapes differ: the A/B commands must be identical')
            limit = max(t['per_command_disp_mm'], t['per_command_disp_frac']*abs(a['displacement_mm']))
            gate(f"disp::{a['label']}", a['displacement_mm'], b['displacement_mm'], limit)
            gate(f"yaw::{a['label']}", a['yaw_change_deg'], b['yaw_change_deg'], t['per_command_yaw_deg'])
        gate('final_xy', 0., float(np.linalg.norm(np.subtract(candidate['final_xy_m'], baseline['final_xy_m'])))*1000,
             t['final_pose_mm'])
        gate('final_yaw', baseline['final_yaw_deg'], candidate['final_yaw_deg'], t['final_yaw_deg'])
    elif scenario == 'wall_push':
        t = TOLERANCES[scenario]
        gate('max_penetration', baseline['max_penetration_mm'], candidate['max_penetration_mm'],
             t['max_penetration_mm'])
        limit = max(t['along_wall_slide_mm'], t['along_wall_slide_frac']*abs(baseline['along_wall_slide_mm']))
        gate('along_wall_slide', baseline['along_wall_slide_mm'], candidate['along_wall_slide_mm'], limit)
        gate('escape', baseline['escape_mm'], candidate['escape_mm'], t['escape_mm'])
        for name, value in (('escape_floor_baseline', baseline['escape_mm']),
                            ('escape_floor_candidate', candidate['escape_mm'])):
            gates.append({'gate': name, 'baseline': t['escape_floor_mm'], 'candidate': value,
                          'diff': None, 'limit': t['escape_floor_mm'],
                          'ok': bool(value >= t['escape_floor_mm'])})
    elif scenario == 'idle_settle':
        t = TOLERANCES[scenario]
        for body, base in sorted(baseline['body_drift_mm'].items()):
            cand = candidate['body_drift_mm'][body]
            gates.append({'gate': f'drift_abs::{body}', 'baseline': base, 'candidate': cand, 'diff': None,
                          'limit': t['body_drift_mm'],
                          'ok': bool(base <= t['body_drift_mm'] and cand <= t['body_drift_mm'])})
            gate(f'drift_diff::{body}', base, cand, t['body_drift_diff_mm'])
    elif scenario == 'arm_sweep':
        t = TOLERANCES[scenario]
        for a, b in zip(baseline['segments'], candidate['segments']):
            if a['label'] != b['label']:
                raise ValueError('segment tapes differ: the A/B commands must be identical')
            gate(f"grip::{a['label']}", 0.,
                 float(np.linalg.norm(np.subtract(b['grip_xyz_m'], a['grip_xyz_m'])))*1000,
                 t['grip_point_mm'])
            for joint in sorted(a['joints_deg']):
                gate(f"joint::{a['label']}::{joint}", a['joints_deg'][joint], b['joints_deg'][joint],
                     t['joint_deg'])
    elif scenario in ('solo_carry', 'solo_hold_load', 'pair_beam_hold'):
        t = TOLERANCES[scenario]
        for rid, base in sorted(baseline['hold_finger_total_n'].items()):
            cand = candidate['hold_finger_total_n'][rid]
            gate(f'finger_force::{rid}', base, cand, t['finger_force_frac']*max(abs(base), 1e-9))
        if 'placement_mm' in t and baseline.get('placement_err_mm') is not None:
            gate('placement', baseline['placement_err_mm'], candidate['placement_err_mm'], t['placement_mm'])
        if 'tilt_deg' in t:
            gate('cargo_tilt', baseline['max_cargo_tilt_deg'], candidate['max_cargo_tilt_deg'], t['tilt_deg'])
        if 'formation_mm' in t:
            gate('formation_spread', baseline['carrier_spread_change_mm'],
                 candidate['carrier_spread_change_mm'], t['formation_mm'])
    else:
        raise ValueError(f'unknown scenario: {scenario}')
    return gates


def hard_gates(metrics):
    return [{'gate': name, 'value': metrics.get(name), 'limit': limit,
             'ok': bool(metrics.get(name) is not None and metrics[name] <= limit)}
            for name, limit in sorted(HARD_GATES.items())]


def creep_fit(times, slips):
    """Least-squares slope (mm/min) and R^2 of slip against SIM time."""
    if len(times) < 5:
        return None, None
    t = np.asarray(times, float)
    s = np.asarray(slips, float)
    slope, intercept = np.polyfit(t, s, 1)
    pred = slope*t + intercept
    var = float(((s - s.mean())**2).sum())
    r2 = 1. - float(((s - pred)**2).sum())/var if var > 1e-18 else None
    return round(float(slope)*60., 5), (None if r2 is None else round(r2, 4))


def _git(*args):
    r = subprocess.run(['git', *args], cwd=ROOT, text=True, capture_output=True)
    return r.stdout.strip() if r.returncode == 0 else 'unavailable'


class Audit:
    """One scenario under one explicitly selected contact profile."""

    def __init__(self, scenario, profile, seed, output, *, hold_s=None, limit_s=None):
        import mujoco
        from sim.camera_robot_port import CameraRobotPort
        from sim.multi_masterpi_production import MultiMasterPiProductionV2
        from sim.zone_cargo import instances, world_grasps
        from sim.zone_cargo_contact import CARGO_PROFILES, profile_record
        from sim.zone_cargo_scene import CargoZoneScene
        if scenario not in SCENARIOS:
            raise ValueError(f'unknown scenario: {scenario}')
        if profile not in PROFILES:
            raise ValueError(f'select the contact profile explicitly: {PROFILES}')
        self.name, self.profile, self.seed = scenario, profile, int(seed)
        self.spec = dict(SCENARIOS[scenario])
        self.out = Path(output)
        self.out.mkdir(parents=True, exist_ok=False)
        self.mj = mujoco
        cargo_items = []
        self.inst = None
        if self.spec['kind'] == 'carry':
            item = {'item_id': 'audit', 'kind': self.spec['cargo_kind'],
                    'pose': list(self.spec['cargo_start'])}
            if self.spec.get('mass_kg') is not None:
                item['mass_kg'] = self.spec['mass_kg']
            cargo_items = [item]
            self.inst = instances(cargo_items)[0]
        self.scene = CargoZoneScene.from_cargo_config(VARIANT, self.seed, cargo=cargo_items, goal=GOAL,
                                                      contact_profile=profile)
        self.world = MultiMasterPiProductionV2(
            seed=self.seed, width=64, height=48, render=False, warehouse_layout=self.scene.engine_layout,
            warehouse_cargo_ids=None, xml_transform=self.scene.transform)
        self.scene.setup(self.world)
        m, d = self.world.model, self.world.data
        self.m, self.d = m, d
        (self.out/'scene.xml').write_text(self.world.scene_xml)
        self.scene_xml_sha256 = hashlib.sha256(self.world.scene_xml.encode()).hexdigest()
        self.profile_record = (profile_record(profile) if profile in CARGO_PROFILES
                               else {'name': profile, 'base': profile})
        self.applied = {'noslip_iterations': int(m.opt.noslip_iterations),
                        'timestep_s': float(m.opt.timestep), 'impratio': float(m.opt.impratio),
                        'cone': int(m.opt.cone), 'solver_iterations': int(m.opt.iterations)}
        expected = 10 if profile == 'cargo_noslip_v1' else 0
        if self.applied['noslip_iterations'] != expected:
            raise RuntimeError(f'profile {profile} expects noslip_iterations {expected}, '
                               f'model has {self.applied["noslip_iterations"]}')
        # Fixture setup only (never during the run).
        self.roles = dict(self.spec.get('roles') or {})
        if self.spec['kind'] == 'carry':
            grasps = world_grasps(self.inst)
            for rid, role in self.roles.items():
                x, y, a = grasps[role]['base_xyyaw']
                self.world.robot(rid).set_base_pose_for_test(
                    (x - .10*math.cos(a), y - .10*math.sin(a), BASE_Z), a)
        for rid, (x, y, a) in (self.spec.get('start') or {}).items():
            self.world.robot(rid).set_base_pose_for_test((x, y, BASE_Z), a)
        mujoco.mj_forward(m, d)
        self.ports = {rid: CameraRobotPort(self.world, rid, allow_reverse=True, allow_mecanum=True)
                      for rid in ('r1', 'r2', 'r3')}
        self.hold_s = float(self.spec.get('hold_s', 1.5) if hold_s is None else hold_s)
        self.limit_s = float(limit_s if limit_s is not None
                             else self.spec.get('limit_s', 600. if self.spec['kind'] == 'carry' else 120.))
        self._index_geoms()
        self.trace = []
        self.events = []
        self.stats = {'eq_active_max': 0, 'max_qvel_abs': 0., 'nan_samples': 0,
                      'wall_contact_samples': 0, 'max_penetration_mm': 0., 'max_wall_normal_n': 0.}
        self.teacher = None
        self.metrics_io = {'cargo_min_z': 0., 'events': []}
        if self.spec['kind'] == 'carry':
            from scripts.cargo_formation_teacher import FormationTeacher, Reference
            self.teacher = FormationTeacher(
                self.world, {rid: self.ports[rid] for rid in self.roles}, self.inst, self.roles,
                legs=self.spec['legs'], log=self._log, hold_s=self.hold_s)
            legs = self.spec['legs']
            self.target = (Reference(self.spec['cargo_start'], legs).poses[-1] if legs
                           else tuple(self.spec['cargo_start']))
        self.free_bodies = self._free_bodies()
        self.start_free = {name: np.array(d.xpos[bid], float).copy()
                           for name, bid in self.free_bodies.items()}

    # --- indexing -----------------------------------------------------------
    def _index_geoms(self):
        m, mj = self.m, self.mj
        self.gname = {g: (mj.mj_id2name(m, mj.mjtObj.mjOBJ_GEOM, g) or '') for g in range(m.ngeom)}
        self.wall_geoms = {g for g, n in self.gname.items() if n.startswith('zone_wall')}
        self.robot_geoms = {g: n[:2] for g, n in self.gname.items() if n[:4] in ('r1__', 'r2__', 'r3__')}
        self.fingers = {}
        for rid in ('r1', 'r2', 'r3'):
            for side in ('left', 'right'):
                self.fingers[self.mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, f'{rid}__{side}_finger')] = rid
        self.cargo_geoms = []
        if self.inst is not None:
            self.cargo_geoms = [mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, self.inst.geom(p.name))
                                for p in self.inst.spec().parts if p.collision]
        self.cargo_set = set(self.cargo_geoms)

    def _free_bodies(self):
        m, mj = self.m, self.mj
        out = {}
        for j in range(m.njnt):
            if m.jnt_type[j] != mj.mjtJoint.mjJNT_FREE:
                continue
            bid = int(m.jnt_bodyid[j])
            name = mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, bid) or f'body_{bid}'
            if name.endswith('__robot') or name.startswith(('r1__', 'r2__', 'r3__')):
                continue
            out[name] = bid
        return out

    def _log(self, kind, now, **detail):
        self.events.append({'event': kind, 'sim_time_s': round(float(now), 3), **detail})

    # --- truth read-outs (evaluation only) ----------------------------------
    def robot_pose(self, rid):
        r = self.world.robot(rid)
        xyz = r.base_xyz()
        return float(xyz[0]), float(xyz[1]), float(r.base_rpy()[2])

    def grip_point(self, rid):
        d = self.d
        return (np.array(d.geom(f'{rid}__left_finger').xpos, float)
                + np.array(d.geom(f'{rid}__right_finger').xpos, float))/2.

    def joints_deg(self, rid):
        m, d = self.m, self.d
        out = {}
        for joint in ARM_SERVOS:
            jid = self.mj.mj_name2id(m, self.mj.mjtObj.mjOBJ_JOINT, f'{rid}__{joint}')
            out[joint] = round(math.degrees(float(d.qpos[int(m.jnt_qposadr[jid])])), 4)
        return out

    def cargo_min_z(self):
        m, d, mj = self.m, self.d, self.mj
        lo = math.inf
        for g in self.cargo_geoms:
            pos, mat, size = d.geom_xpos[g], d.geom_xmat[g].reshape(3, 3), m.geom_size[g]
            if m.geom_type[g] == mj.mjtGeom.mjGEOM_BOX:
                for sx in (-1, 1):
                    for sy in (-1, 1):
                        for sz in (-1, 1):
                            lo = min(lo, float((pos + mat @ (np.array([sx, sy, sz], float)*size))[2]))
            else:
                r, h = float(size[0]), float(size[1])
                for k in range(8):
                    a = k*math.pi/4
                    for sz in (-1, 1):
                        lo = min(lo, float((pos + mat @ np.array([r*math.cos(a), r*math.sin(a), sz*h]))[2]))
        return 0. if lo is math.inf else lo

    def grip_in_cargo_mm(self, rid):
        """Grip midpoint in the cargo body frame (mm) - the PR #167 slip convention."""
        body = self.d.body(self.inst.body)
        mat = np.array(body.xmat, float).reshape(3, 3)
        return (mat.T @ (self.grip_point(rid) - np.array(body.xpos, float)))*1000.

    def wall_state(self):
        """Deepest wall penetration (mm) and total wall normal force (N) on robots."""
        m, d, mj = self.m, self.d, self.mj
        pen, force, count = 0., 0., 0
        f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            a, b = int(c.geom1), int(c.geom2)
            pair = {a, b}
            if pair & self.wall_geoms and (a in self.robot_geoms or b in self.robot_geoms):
                count += 1
                pen = max(pen, -float(c.dist)*1000.)
                mj.mj_contactForce(m, d, i, f6)
                force += abs(float(f6[0]))
        return pen, force, count

    # --- stepping -----------------------------------------------------------
    def step(self):
        self.world._physics_step_for(self.world.controllers['r1'])
        if self.d.eq_active.any():
            self.stats['eq_active_max'] = 1
            raise RuntimeError('equality constraint became active: weld assistance is forbidden')

    def sample(self):
        d = self.d
        now = float(d.time)
        qvel = np.abs(np.asarray(d.qvel, float))
        if not np.isfinite(qvel).all() or not np.isfinite(np.asarray(d.qpos, float)).all():
            self.stats['nan_samples'] += 1
        else:
            self.stats['max_qvel_abs'] = max(self.stats['max_qvel_abs'], float(qvel.max()))
        pen, force, count = self.wall_state()
        if count:
            self.stats['wall_contact_samples'] += 1
            self.stats['max_penetration_mm'] = max(self.stats['max_penetration_mm'], pen)
            self.stats['max_wall_normal_n'] = max(self.stats['max_wall_normal_n'], force)
        row = {'t': round(now, 4), 'wall_contacts': count,
               'wall_penetration_mm': round(pen, 4), 'wall_normal_n': round(force, 3)}
        for rid in (self.spec.get('start') or {}):
            x, y, a = self.robot_pose(rid)
            row[f'{rid}_pose'] = [round(x, 6), round(y, 6), round(math.degrees(a), 4)]
        if self.teacher is not None:
            row['phase'] = self.teacher.phase
            body = self.d.body(self.inst.body)
            mat = np.array(body.xmat, float).reshape(3, 3)
            row['cargo_xyz'] = [round(float(v), 6) for v in body.xpos]
            row['cargo_tilt_deg'] = round(math.degrees(math.acos(max(-1., min(1., float(mat[2, 2]))))), 4)
            row['cargo_min_z_m'] = round(self.metrics_io['cargo_min_z'], 5)
            forces = self.teacher.finger_forces()
            row['finger_n'] = {r: [round(x, 3) for x in v] for r, v in forces.items()}
            row['grip_in_cargo_mm'] = {r: [round(float(x), 3) for x in self.grip_in_cargo_mm(r)]
                                       for r in self.roles}
            row['robot_pose'] = {r: [round(v, 6) for v in self.robot_pose(r)] for r in self.roles}
        self.trace.append(row)

    def run_until(self, until_s, *, sample=True):
        dt = float(self.m.opt.timestep)
        every = max(1, round(SAMPLE_S/dt))
        while float(self.d.time) < until_s - 1e-9:
            now = float(self.d.time)
            if self.teacher is not None:
                self.teacher.tick(now, self.metrics_io)
            for port in self.ports.values():
                port.tick(now)
            self.step()
            if self.teacher is not None:
                self.metrics_io['cargo_min_z'] = self.cargo_min_z()
            self._steps += 1
            if sample and self._steps % every == 0:
                self.sample()

    # --- scenario drivers ---------------------------------------------------
    def run(self):
        self._steps = 0
        load0 = os.getloadavg()
        wall0 = time.time()
        self.metrics_io['cargo_min_z'] = self.cargo_min_z() if self.inst is not None else 0.
        self.sample()
        metrics = (self._run_tape() if self.spec['kind'] == 'tape' else self._run_carry())
        metrics.update({name: round(self.stats[name], 5) if isinstance(self.stats[name], float)
                        else self.stats[name] for name in self.stats})
        metrics['body_drift_mm'] = {
            name: round(float(np.linalg.norm(np.array(self.d.xpos[bid], float) - self.start_free[name]))*1000., 4)
            for name, bid in self.free_bodies.items()}
        return metrics, load0, time.time()-wall0

    def _run_tape(self):
        tape = self.spec['tape']()
        segments = []
        rid = (self.spec.get('robots') or ('r1',))[0]
        tracked = bool(self.spec.get('robots'))
        if not tape:
            self.run_until(float(self.d.time) + self.limit_s)
            return {'segments': [], 'commands': 0}
        for label, action, dwell in tape:
            t0 = float(self.d.time)
            pose0 = self.robot_pose(rid)
            grip0 = self.grip_point(rid)
            ack = self.ports[rid].apply(action, t0)
            end = (t0 + float(action['duration_s']) if action['kind'] == 'mecanum'
                   else float(ack['busy_until']))
            self.run_until(end)
            cmd_pose = self.robot_pose(rid)
            self.run_until(end + dwell)
            pose1 = self.robot_pose(rid)
            dx, dy = pose1[0]-pose0[0], pose1[1]-pose0[1]
            segments.append({
                'label': label, 'action': dict(action),
                't_start_s': round(t0, 4), 't_end_s': round(float(self.d.time), 4),
                'displacement_mm': round(math.hypot(dx, dy)*1000., 4),
                'delta_xy_mm': [round(dx*1000., 4), round(dy*1000., 4)],
                'yaw_change_deg': round(math.degrees(_wrap(pose1[2]-pose0[2])), 4),
                'disp_at_command_end_mm': round(math.hypot(cmd_pose[0]-pose0[0], cmd_pose[1]-pose0[1])*1000., 4),
                'pose_after': [round(pose1[0], 6), round(pose1[1], 6), round(math.degrees(pose1[2]), 4)],
                'grip_xyz_m': [round(float(v), 6) for v in self.grip_point(rid)],
                'grip_move_mm': round(float(np.linalg.norm(self.grip_point(rid)-grip0))*1000., 4),
                'joints_deg': self.joints_deg(rid),
                'wall_contacts_now': self.wall_state()[2]})
            self._log('segment', float(self.d.time), label=label,
                      displacement_mm=segments[-1]['displacement_mm'])
        out = {'segments': segments, 'commands': len(segments)}
        if tracked:
            final = self.robot_pose(rid)
            out['final_xy_m'] = [round(final[0], 6), round(final[1], 6)]
            out['final_yaw_deg'] = round(math.degrees(final[2]), 4)
        if self.name == 'wall_push':
            out.update(self._wall_metrics(segments))
        return out

    def _wall_metrics(self, segments):
        slide = [s for s in segments if s['label'].startswith('slide_')]
        escape = [s for s in segments if s['label'].startswith('escape_')]
        # Along-wall axis: the divider wall is N-S at x = 2.2, so the along-wall
        # component is y and the into-wall component is x.
        start_slide = np.array(segments[segments.index(slide[0])-1]['pose_after'][:2], float)
        end_slide = np.array(slide[-1]['pose_after'][:2], float)
        start_escape = np.array(segments[segments.index(escape[0])-1]['pose_after'][:2], float)
        end_escape = np.array(escape[-1]['pose_after'][:2], float)
        contact_first = next((s['label'] for s in segments if s['wall_contacts_now']), None)
        pressing = [s['wall_contacts_now'] for s in slide]
        return {'along_wall_slide_mm': round(float(abs(end_slide[1]-start_slide[1]))*1000., 4),
                'into_wall_during_slide_mm': round(float(start_slide[0]-end_slide[0])*1000., 4),
                'escape_mm': round(float(np.linalg.norm(end_escape-start_escape))*1000., 4),
                'first_contact_segment': contact_first,
                'slide_contacts_min': min(pressing), 'slide_contacts_max': max(pressing),
                'slide_yaw_change_deg': round(float(sum(s['yaw_change_deg'] for s in slide)), 4)}

    def _run_carry(self):
        t = self.teacher
        limit = float(self.d.time) + self.limit_s
        dt = float(self.m.opt.timestep)
        every = max(1, round(SAMPLE_S/dt))
        while t.phase != 'done' and float(self.d.time) < limit:
            now = float(self.d.time)
            t.tick(now, self.metrics_io)
            for port in self.ports.values():
                port.tick(now)
            self.step()
            self.metrics_io['cargo_min_z'] = self.cargo_min_z()
            self._steps += 1
            if self._steps % every == 0:
                self.sample()
        return self._carry_metrics()

    def _carry_metrics(self):
        from scripts.cargo_formation_teacher import wrap
        t = self.teacher
        hold = [r for r in self.trace if r.get('phase') == 'hold']
        carry = [r for r in self.trace if r.get('phase') == 'carry']
        out = {'outcome': t.outcome, 'phase': t.phase, 'phase_times_s': dict(t.phase_times),
               'lifted_clear': bool(t.lifted_clear), 'detail': dict(t.detail),
               'hold_samples': len(hold), 'carry_samples': len(carry),
               'teacher_events': [e for e in self.metrics_io['events']],
               'hold_span_s': None, 'hold_slip_mm': {}, 'hold_creep_mm_per_min': {},
               'hold_creep_r2': {}, 'carry_slip_mm': {}, 'max_slip_mm': {},
               'hold_finger_total_n': {}, 'max_cargo_tilt_deg': 0.,
               'cargo_max_min_z_m': round(max((r['cargo_min_z_m'] for r in self.trace
                                               if 'cargo_min_z_m' in r), default=0.), 5)}
        # Skip the first 2 s of hold: the lift transient is not creep.
        window = [r for r in hold if r['t'] >= hold[0]['t'] + 2.] if hold else []
        if len(window) > 5:
            out['hold_span_s'] = round(window[-1]['t'] - window[0]['t'], 3)
            for rid in self.roles:
                ref = np.array(window[0]['grip_in_cargo_mm'][rid], float)
                series = [float(np.linalg.norm(np.array(r['grip_in_cargo_mm'][rid], float) - ref))
                          for r in window]
                out['hold_slip_mm'][rid] = round(series[-1], 4)
                slope, r2 = creep_fit([r['t'] for r in window], series)
                out['hold_creep_mm_per_min'][rid] = slope
                out['hold_creep_r2'][rid] = r2
        for rid in self.roles:
            ref = np.array((hold or self.trace)[0].get('grip_in_cargo_mm', {rid: [0., 0., 0.]})[rid], float)
            rows = [r for r in self.trace if 'grip_in_cargo_mm' in r
                    and r.get('phase') in ('hold', 'carry', 'lower')]
            if rows:
                out['max_slip_mm'][rid] = round(max(
                    float(np.linalg.norm(np.array(r['grip_in_cargo_mm'][rid], float) - ref)) for r in rows), 4)
            if carry:
                a = np.array(carry[0]['grip_in_cargo_mm'][rid], float)
                b = np.array(carry[-1]['grip_in_cargo_mm'][rid], float)
                out['carry_slip_mm'][rid] = round(float(np.linalg.norm(b-a)), 4)
            values = [sum(r['finger_n'][rid]) for r in hold] if hold else []
            out['hold_finger_total_n'][rid] = round(float(np.mean(values)), 4) if values else 0.
        tilts = [r['cargo_tilt_deg'] for r in self.trace if 'cargo_tilt_deg' in r
                 and r.get('phase') in ('lift', 'hold', 'carry')]
        out['max_cargo_tilt_deg'] = round(max(tilts), 4) if tilts else 0.
        if len(self.roles) > 1 and hold:
            def spread(row):
                poses = [np.array(row['robot_pose'][r][:2], float) for r in sorted(self.roles)]
                return float(np.linalg.norm(poses[0]-poses[1]))
            out['carrier_spread_change_mm'] = round(abs(spread(hold[-1]) - spread(hold[0]))*1000., 4)
        else:
            out['carrier_spread_change_mm'] = 0.
        final = t.cargo_pose()
        pos_err = math.hypot(final[0]-self.target[0], final[1]-self.target[1])
        out['placement_err_mm'] = round(pos_err*1000., 3) if self.spec['legs'] else None
        out['final_cargo_pose'] = [round(final[0], 6), round(final[1], 6),
                                   round(math.degrees(wrap(final[2])), 4)]
        out['dropped'] = bool([e for e in self.metrics_io['events'] if e['event'] == 'cargo_touched_floor'])
        return out


def _wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--scenario', required=True, choices=sorted(SCENARIOS))
    parser.add_argument('--contact-profile', required=True, choices=PROFILES,
                        help='explicit selection; PR #169 requires it and it is recorded with its hash')
    parser.add_argument('--seed', type=int, default=11)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--hold-s', type=float, default=None)
    parser.add_argument('--limit-s', type=float, default=None)
    parser.add_argument('--allow-dirty', action='store_true',
                        help='development only; committed sources are required for recorded runs')
    args = parser.parse_args(argv)
    dirty = bool(_git('status', '--porcelain', '--untracked-files=no', '--', 'scripts', 'sim', 'harness'))
    if dirty and not args.allow_dirty:
        raise SystemExit('commit the run source first (or --allow-dirty for development)')
    audit = Audit(args.scenario, args.contact_profile, args.seed, args.output,
                  hold_s=args.hold_s, limit_s=args.limit_s)
    metrics, load0, wall_s = audit.run()
    result = {
        'schema': SCHEMA, 'scenario': args.scenario, 'question': audit.spec['question'],
        'metric': audit.spec['metric'], 'contact_profile': audit.profile_record,
        'applied_solver_options': audit.applied, 'seed': audit.seed, 'variant': VARIANT,
        'scene_xml_sha256': audit.scene_xml_sha256,
        'scene_record_sha256': hashlib.sha256(
            json.dumps(audit.scene.record(), sort_keys=True, default=str).encode()).hexdigest(),
        'weld': 'off', 'eq_active_max': metrics['eq_active_max'],
        'teacher_condition': True, 'student_result': False,
        'sim_time_s': round(float(audit.d.time), 4), 'physics_steps': audit._steps,
        'hold_s': audit.hold_s, 'limit_s': audit.limit_s,
        'hard_gates': hard_gates(metrics), 'metrics': metrics,
        'git': {'sha': _git('rev-parse', 'HEAD'), 'dirty': dirty, 'branch': _git('rev-parse', '--abbrev-ref', 'HEAD')},
        'host': {'platform': platform.platform(), 'python': platform.python_version(),
                 'load_avg_start': [round(v, 2) for v in load0],
                 'load_avg_end': [round(v, 2) for v in os.getloadavg()],
                 'wall_s_not_a_metric': round(wall_s, 2)},
    }
    (args.output/'trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in audit.trace))
    (args.output/'events.json').write_text(json.dumps(audit.events, indent=1)+'\n')
    (args.output/'result.json').write_text(json.dumps(result, indent=1, default=str)+'\n')
    (args.output/'hashes.json').write_text(json.dumps(
        {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
         for p in sorted(args.output.iterdir()) if p.name != 'hashes.json'}, indent=1)+'\n')
    failed = [g['gate'] for g in result['hard_gates'] if not g['ok']]
    print(json.dumps({'scenario': args.scenario, 'profile': args.contact_profile, 'seed': audit.seed,
                      'noslip_iterations': audit.applied['noslip_iterations'],
                      'sim_s': result['sim_time_s'], 'hard_gates_failed': failed,
                      'load_avg_end': result['host']['load_avg_end']}))
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
