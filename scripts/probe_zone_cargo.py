#!/usr/bin/env python3
"""Physical feasibility probes for the zone cargo catalogue (GT TEACHER, weld OFF).

Each probe places one catalogue item and 1-3 MasterPi robots on an open patch
of the ``zone_wide`` floor (fixture setup is the only direct pose write), then
runs ``scripts.cargo_formation_teacher`` on synchronous SIM time with the
``local_contact_fine`` contact profile. Every physics step asserts that no
equality constraint (weld) is active. Outputs per probe: ``result.json``,
``trace.jsonl`` (sampled truth metrics), ``events.json`` and an observer MP4.

These are teacher-condition physics results (ground-truth poses, IK and contact
forces), never an RGB/student success.

  .venv-sim/bin/python -m scripts.probe_zone_cargo --probe pair_crate --output outputs/zone-cargo/pair_crate
  .venv-sim/bin/python -m scripts.probe_zone_cargo --sweep cal_block --output outputs/zone-cargo/sweep
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

from sim.zone_cargo import catalogue_record, instances  # noqa: E402

START = (2.5, .2, 0.)
SOLO_LEGS = [('move', 1.0)]
TEAM_LEGS = [('move', .8), ('turn', math.pi/2), ('move', .5)]
BASE_Z = .032355118817659255
PROBES = {
    'solo_box': {'kind': 'cal_block', 'mass_kg': .030, 'roles': {'r1': 'west'}, 'legs': SOLO_LEGS,
                 'note': 'box-equivalent: production box shape, 30 g, box material and finger pairs'},
    'solo_can': {'kind': 'can', 'roles': {'r1': 'any'}, 'legs': SOLO_LEGS},
    'solo_tile': {'kind': 'tile', 'roles': {'r1': 'west'}, 'legs': SOLO_LEGS},
    'pair_beam': {'kind': 'long_beam', 'roles': {'r1': 'end_neg', 'r2': 'end_pos'}, 'legs': TEAM_LEGS},
    'pair_crate': {'kind': 'heavy_crate', 'roles': {'r1': 'west', 'r2': 'east'}, 'legs': TEAM_LEGS},
    'trio_frame': {'kind': 'tri_frame', 'roles': {'r1': 'v0', 'r2': 'v1', 'r3': 'v2'}, 'legs': TEAM_LEGS},
    # "needs N" evidence: the same teacher with one robot fewer.
    # They attempt the carry even when the load is not lifted clear, to show
    # whether it can be moved at all (dragging on the floor is not a carry).
    'solo_beam': {'kind': 'long_beam', 'roles': {'r1': 'end_neg'}, 'legs': TEAM_LEGS, 'expect': 'fail',
                  'attempt_carry': True},
    'solo_crate': {'kind': 'heavy_crate', 'roles': {'r1': 'west'}, 'legs': TEAM_LEGS, 'expect': 'fail',
                   'attempt_carry': True},
    'duo_frame': {'kind': 'tri_frame', 'roles': {'r1': 'v0', 'r2': 'v1'}, 'legs': TEAM_LEGS, 'expect': 'fail',
                  'attempt_carry': True},
}
SWEEP_MASSES = (.03, .1, .2, .3, .4, .5, .6, .7, .8, 1.0, 1.2)


def _git(*args):
    r = subprocess.run(['git', *args], cwd=ROOT, text=True, capture_output=True)
    return r.stdout.strip() if r.returncode == 0 else 'unavailable'


class Probe:
    def __init__(self, name, spec, output, *, video=True, variant='zone_wide', seed=11,
                 contact_profile='local_contact_fine', mass_kg=None, legs=None, hold_s=1.5):
        import mujoco
        from sim.camera_robot_port import CameraRobotPort
        from sim.multi_masterpi_production import MultiMasterPiProductionV2
        from sim.zone_cargo_scene import CargoZoneScene
        from scripts.cargo_formation_teacher import FormationTeacher, compose
        self.name, self.spec, self.out = name, dict(spec), Path(output)
        self.out.mkdir(parents=True, exist_ok=False)
        mass = mass_kg if mass_kg is not None else spec.get('mass_kg')
        item = {'item_id': 'probe', 'kind': spec['kind'], 'pose': list(START),
                **({'mass_kg': mass} if mass is not None else {})}
        self.scene = CargoZoneScene.from_cargo_config(variant, seed, cargo=[item], goal={'A': {'cyan': 1}},
                                                      contact_profile=contact_profile)
        self.inst = instances([item])[0]
        self.world = MultiMasterPiProductionV2(
            seed=seed, width=320, height=240, render=False, warehouse_layout=self.scene.engine_layout,
            warehouse_cargo_ids=None, xml_transform=self.scene.transform)
        self.scene.setup(self.world)
        m, d = self.world.model, self.world.data
        self.m, self.d, self.mj = m, d, mujoco
        (self.out/'scene.xml').write_text(self.world.scene_xml)
        # Fixture setup: each carrier 0.10 m behind its approach base pose.
        from sim.zone_cargo import world_grasps
        grasps = world_grasps(self.inst)
        self.roles = dict(spec['roles'])
        for rid, role in self.roles.items():
            x, y, a = grasps[role]['base_xyyaw']
            self.world.robot(rid).set_base_pose_for_test((x - .10*math.cos(a), y - .10*math.sin(a), BASE_Z), a)
        mujoco.mj_forward(m, d)
        self.ports = {r: CameraRobotPort(self.world, r, allow_reverse=True, allow_mecanum=True)
                      for r in ('r1', 'r2', 'r3')}
        self.events = []
        legs = spec['legs'] if legs is None else legs
        self.teacher = FormationTeacher(self.world, {r: self.ports[r] for r in self.roles}, self.inst,
                                        self.roles, legs=legs, log=self._log, hold_s=hold_s,
                                        carry_if_not_clear=bool(spec.get('attempt_carry')))
        self.legs = legs
        from scripts.cargo_formation_teacher import Reference
        self.target = Reference(START, legs).poses[-1] if legs else START
        # geometry for metrics
        self.cargo_geoms = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, self.inst.geom(p.name))
                            for p in self.inst.spec().parts if p.collision]
        self.cargo_set = set(self.cargo_geoms)
        self.floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
        self.fingers = {}
        for rid in ('r1', 'r2', 'r3'):
            for side in ('left', 'right'):
                self.fingers[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f'{rid}__{side}_finger')] = rid
        self.robot_geom = {}
        for g in range(m.ngeom):
            gname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ''
            if gname[:4] in ('r1__', 'r2__', 'r3__'):
                self.robot_geom[g] = gname[:2]
        self.servos = {rid: {s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f'{rid}__servo_{s}')
                             for s in ('shoulder', 'elbow', 'wrist', 'arm_yaw')} for rid in self.roles}
        self.video = video
        self.renderer = None
        if video:
            self.renderer = mujoco.Renderer(m, 480, 854)
            self.cam = mujoco.MjvCamera()
            self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            self.cam.distance = {1: .95, 2: 1.45, 3: 1.75}[len(self.roles)]
            self.cam.elevation, self.cam.azimuth = -32., 225.
            self.cam.lookat[:] = [START[0], START[1], .03]
            # H.264 through ffmpeg (as scripts/compose_recovery_comparison.py) so the file plays anywhere.
            self.writer = subprocess.Popen(
                ['ffmpeg', '-y', '-v', 'error', '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', '854x480', '-r', '20',
                 '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '30', '-preset', 'medium',
                 str(self.out/f'{self.name}.mp4')], stdin=subprocess.PIPE)
        self.trace = []
        self.body_contact_geoms = {}
        self.metrics = {'cargo_min_z': 0., 'events': []}
        self.slip_ref = {}
        self.stats = {'max_cargo_tilt_deg': {}, 'max_robot_tilt_deg': 0., 'max_slip_mm': {},
                      'finger_force_n': {}, 'floor_contact_samples_carry': 0, 'robot_body_contact_samples': 0,
                      'saturated_samples': {}, 'eq_active_max': 0, 'max_track_err_m': 0., 'max_lift_z_m': 0.}

    def _log(self, kind, now, **detail):
        self.events.append({'event': kind, 'sim_time_s': round(now, 3), **detail})

    # --- truth metrics ---
    def cargo_min_z(self):
        m, d = self.m, self.d
        lo = math.inf
        for g in self.cargo_geoms:
            pos, mat, size = d.geom_xpos[g], d.geom_xmat[g].reshape(3, 3), m.geom_size[g]
            if m.geom_type[g] == self.mj.mjtGeom.mjGEOM_BOX:
                for sx in (-1, 1):
                    for sy in (-1, 1):
                        for sz in (-1, 1):
                            lo = min(lo, float((pos + mat @ (np.array([sx, sy, sz])*size))[2]))
            else:
                r, h = size[0], size[1]
                for k in range(8):
                    a = k*math.pi/4
                    for sz in (-1, 1):
                        lo = min(lo, float((pos + mat @ np.array([r*math.cos(a), r*math.sin(a), sz*h]))[2]))
        return lo

    def sample(self, now):
        m, d, mj = self.m, self.d, self.mj
        phase = self.teacher.phase
        body = d.body(self.inst.body)
        mat = body.xmat.reshape(3, 3)
        tilt = math.degrees(math.acos(max(-1., min(1., float(mat[2, 2])))))
        minz = self.cargo_min_z()
        self.metrics['cargo_min_z'] = minz
        self.stats['max_lift_z_m'] = max(self.stats['max_lift_z_m'], minz)
        forces = self.teacher.finger_forces()
        floor = robot_body = 0
        for i in range(d.ncon):
            c = d.contact[i]
            a, b = c.geom1, c.geom2
            if a in self.cargo_set or b in self.cargo_set:
                other = b if a in self.cargo_set else a
                if other == self.floor:
                    floor += 1
                elif other in self.robot_geom and other not in self.fingers:
                    robot_body += 1
                    if phase in ('lift', 'hold', 'carry'):
                        name = mj.mj_id2name(m, mj.mjtObj.mjOBJ_GEOM, other)
                        self.body_contact_geoms[name] = self.body_contact_geoms.get(name, 0) + 1
        if phase == 'carry' and floor:
            self.stats['floor_contact_samples_carry'] += 1
        if robot_body and phase in ('lift', 'hold', 'carry'):
            self.stats['robot_body_contact_samples'] += 1
        rt = {}
        for rid in self.roles:
            rmat = d.body(f'{rid}__robot').xmat.reshape(3, 3)
            rt[rid] = math.degrees(math.acos(max(-1., min(1., float(rmat[2, 2])))))
            self.stats['max_robot_tilt_deg'] = max(self.stats['max_robot_tilt_deg'], rt[rid])
        grip = {}
        for rid in self.roles:
            p = (d.geom(f'{rid}__left_finger').xpos + d.geom(f'{rid}__right_finger').xpos)/2
            grip[rid] = mat.T @ (p - body.xpos)
        if phase in ('lift', 'hold', 'carry'):
            self.stats['max_cargo_tilt_deg'][phase] = max(self.stats['max_cargo_tilt_deg'].get(phase, 0.), tilt)
            for rid in self.roles:
                ref = self.slip_ref.setdefault(rid, grip[rid].copy())
                slip = float(np.linalg.norm(grip[rid] - ref))*1000
                self.stats['max_slip_mm'][rid] = max(self.stats['max_slip_mm'].get(rid, 0.), slip)
                ff = self.stats['finger_force_n'].setdefault(phase, {}).setdefault(rid, [math.inf, 0., 0., 0])
                lo = min(forces[rid]); tot = sum(forces[rid])
                ff[0] = min(ff[0], lo); ff[1] = max(ff[1], tot); ff[2] += tot; ff[3] += 1
                sat = [s for s, a in self.servos[rid].items()
                       if abs(float(d.actuator_force[a])) >= .98*float(m.actuator_forcerange[a][1])]
                if sat:
                    cnt = self.stats['saturated_samples'].setdefault(rid, {})
                    for s in sat:
                        cnt[s] = cnt.get(s, 0) + 1
        if phase == 'carry':
            self.stats['max_track_err_m'] = max(self.stats['max_track_err_m'], self.metrics.get('track_err_m', 0.))
        self.trace.append({'t': round(now, 3), 'phase': phase,
                           'cargo_xyz': [round(float(v), 4) for v in body.xpos],
                           'cargo_yaw_deg': round(math.degrees(self.teacher.cargo_pose()[2]), 2),
                           'cargo_tilt_deg': round(tilt, 2), 'cargo_min_z_m': round(minz, 4),
                           'finger_n': {r: [round(x, 2) for x in v] for r, v in forces.items()},
                           'robot_tilt_deg': {r: round(v, 2) for r, v in rt.items()},
                           'grip_in_cargo_mm': {r: [round(float(x)*1000, 1) for x in v] for r, v in grip.items()},
                           'cargo_floor_contacts': floor, 'cargo_robot_body_contacts': robot_body})

    def frame(self, now):
        import cv2
        c = self.d.body(self.inst.body).xpos
        self.cam.lookat[:] = .9*self.cam.lookat + .1*np.array([c[0], c[1], .04])
        self.renderer.update_scene(self.d, camera=self.cam)
        img = cv2.cvtColor(self.renderer.render(), cv2.COLOR_RGB2BGR)
        lines = [f'{self.name}  SIM {now:6.1f}s  phase: {self.teacher.phase}',
                 f'{self.inst.kind} {self.inst.spec().mass_kg:.3f} kg  robots {",".join(self.roles)}  '
                 f'weld OFF  GT teacher']
        banner = img[:8+22*len(lines)]
        banner[:] = (banner*.35).astype(np.uint8)
        for k, text in enumerate(lines):
            cv2.putText(img, text, (10, 22+22*k), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1, cv2.LINE_AA)
        self.writer.stdin.write(np.ascontiguousarray(img).tobytes())

    def run(self, limit_s=400.):
        load0 = os.getloadavg()[0]
        wall0 = time.time()
        dt = float(self.m.opt.timestep)
        sample_every = max(1, round(.02/dt))
        frame_every = max(1, round(.2/dt))
        step = 0
        while self.teacher.phase != 'done' and self.d.time < limit_s:
            now = float(self.d.time)
            self.teacher.tick(now, self.metrics)
            for port in self.ports.values():
                port.tick(now)
            self.world._physics_step_for(self.world.controllers['r1'])
            if self.d.eq_active.any():
                self.stats['eq_active_max'] = 1
                raise RuntimeError('equality constraint became active: weld assistance is forbidden')
            step += 1
            if step % sample_every == 0:
                self.sample(float(self.d.time))
            if self.video and step % frame_every == 0:
                self.frame(float(self.d.time))
        if self.video:
            self.writer.stdin.close()
            if self.writer.wait(timeout=120):
                raise RuntimeError('ffmpeg failed')
        return self.summarize(load0, time.time()-wall0)

    def summarize(self, load0, wall_s):
        from scripts.cargo_formation_teacher import wrap
        t = self.teacher
        final = t.cargo_pose()
        body = self.d.body(self.inst.body)
        tilt = math.degrees(math.acos(max(-1., min(1., float(body.xmat.reshape(3, 3)[2, 2])))))
        forces = t.finger_forces()
        pos_err = math.hypot(final[0]-self.target[0], final[1]-self.target[1])
        sym = self.inst.spec().landing_yaw_symmetry_deg
        yaw_err = abs(math.degrees(wrap(final[2]-self.target[2])))
        if sym:
            yaw_err = min(yaw_err % sym, sym - yaw_err % sym)
        drops = [e for e in self.metrics['events'] if e['event'] == 'cargo_touched_floor']
        carried = t.phase_times.get('carry') is not None and t.outcome == 'completed_sequence'
        placed = (carried and pos_err < .05 and yaw_err < 10 and tilt < 3 and self.cargo_min_z() < .004
                  and all(sum(v) < .05 for v in forces.values()))
        ff = {}
        for phase, per in self.stats['finger_force_n'].items():
            ff[phase] = {r: {'min_finger_n': round(v[0], 2), 'max_total_n': round(v[1], 2),
                             'mean_total_n': round(v[2]/max(v[3], 1), 2)} for r, v in per.items()}
        success = bool(placed and t.lifted_clear and not drops and self.stats['max_robot_tilt_deg'] < 10
                       and self.stats['eq_active_max'] == 0 and self.stats['floor_contact_samples_carry'] == 0)
        hold = [r for r in self.trace if r['phase'] == 'hold']
        creep = {}
        if len(hold) > 10:
            a, b = hold[5], hold[-1]
            for rid in self.roles:
                d = np.linalg.norm(np.subtract(b['grip_in_cargo_mm'][rid], a['grip_in_cargo_mm'][rid]))
                creep[rid] = round(float(d)/max(b['t']-a['t'], 1e-6), 4)
        mp4 = self.out/f'{self.name}.mp4'
        result = {
            'schema': 'ugrp.zone_cargo_probe.v1', 'probe': self.name, 'condition': 'GT teacher (not RGB/student)',
            'kind': self.inst.kind, 'mass_kg': self.inst.spec().mass_kg, 'robots': self.roles,
            'required_carriers_claim': self.inst.spec().required_carriers,
            'expect': self.spec.get('expect', 'success'), 'note': self.spec.get('note'),
            'attempt_carry_when_not_clear': bool(self.spec.get('attempt_carry')),
            'legs': [list(l) for l in self.legs], 'target_pose': [round(v, 4) for v in self.target],
            'weld': 'off', 'eq_active_max': self.stats['eq_active_max'],
            'contact_profile': self.scene.scene['contact_profile'], 'timestep_s': float(self.m.opt.timestep),
            'success': success, 'teacher_outcome': t.outcome, 'lifted_clear': t.lifted_clear,
            'hold_min_z_m': t.detail.get('hold_min_z_m'), 'max_lift_min_z_m': round(self.stats['max_lift_z_m'], 4),
            'placement': {'pos_err_m': round(pos_err, 4), 'yaw_err_deg': round(yaw_err, 2),
                          'final_tilt_deg': round(tilt, 2), 'final_min_z_m': round(self.cargo_min_z(), 4),
                          'released': all(sum(v) < .05 for v in forces.values())},
            'max_cargo_tilt_deg': {k: round(v, 2) for k, v in self.stats['max_cargo_tilt_deg'].items()},
            'max_robot_tilt_deg': round(self.stats['max_robot_tilt_deg'], 2),
            'max_slip_mm': {k: round(v, 1) for k, v in self.stats['max_slip_mm'].items()},
            'hold_creep_mm_per_s': creep,
            'drop_events': drops, 'floor_contact_samples_in_carry': self.stats['floor_contact_samples_carry'],
            'cargo_robot_body_contact_samples': self.stats['robot_body_contact_samples'],
            'cargo_robot_body_contact_geoms': self.body_contact_geoms,
            'finger_forces': ff, 'grasp_forces_n': t.detail.get('grasp_forces_n'),
            'servo_saturated_samples': self.stats['saturated_samples'],
            'max_track_err_m': round(self.stats['max_track_err_m'], 4),
            'phase_sim_s': t.phase_times, 'sim_time_s': round(float(self.d.time), 2),
            'wall_s': round(wall_s, 1), 'load_avg_1m': {'start': round(load0, 2), 'end': round(os.getloadavg()[0], 2)},
            'failure_detail': {k: v for k, v in t.detail.items() if k not in ('grasp_forces_n',)},
            'scene_xml_sha256': hashlib.sha256(self.world.scene_xml.encode()).hexdigest(),
            'catalogue_sha256': catalogue_record()['sha256'], 'git_sha': _git('rev-parse', 'HEAD'),
            'git_dirty': bool(_git('status', '--porcelain')), 'python': platform.python_version(),
            'video': mp4.name if mp4.exists() else None}
        (self.out/'result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
        (self.out/'events.json').write_text(json.dumps(self.events, indent=2) + '\n')
        with open(self.out/'trace.jsonl', 'w') as fh:
            for row in self.trace:
                fh.write(json.dumps(row) + '\n')
        return result


def sweep(output, masses=SWEEP_MASSES, hold_s=8.):
    rows = []
    for mass in masses:
        probe = Probe(f'cal_{int(round(mass*1000)):04d}g', {'kind': 'cal_block', 'roles': {'r1': 'west'},
                      'legs': [('move', .5)]}, Path(output)/f'cal_{int(round(mass*1000)):04d}g',
                      video=False, mass_kg=mass, hold_s=hold_s)
        r = probe.run()
        rows.append({k: r[k] for k in ('mass_kg', 'success', 'teacher_outcome', 'lifted_clear', 'hold_min_z_m',
                                       'max_robot_tilt_deg', 'max_slip_mm', 'hold_creep_mm_per_s', 'max_cargo_tilt_deg',
                                       'servo_saturated_samples', 'sim_time_s', 'load_avg_1m')})
        print(json.dumps(rows[-1]), flush=True)
    passing = [r['mass_kg'] for r in rows if r['success']]
    capacity = max(passing) if passing else None
    first_fail = min([r['mass_kg'] for r in rows if not r['success'] and r['mass_kg'] > (capacity or 0)],
                     default=None)
    summary = {'schema': 'ugrp.zone_cargo_capacity.v1', 'rows': rows,
               'single_robot_capacity_kg': capacity, 'first_failing_mass_kg': first_fail,
               'hold_s': hold_s,
               'criterion': f'lift clear >= 12 mm for the {hold_s:g} s hold, carry 0.5 m, place within 5 cm, robot tilt < 10 deg, weld off'}
    (Path(output)/'capacity.json').write_text(json.dumps(summary, indent=2) + '\n')
    return summary


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--probe', choices=sorted(PROBES))
    p.add_argument('--sweep', choices=['cal_block'])
    p.add_argument('--masses', help='comma separated kg for --sweep')
    p.add_argument('--mass-kg', type=float, help='override the catalogue mass (diagnostic)')
    p.add_argument('--hold-s', type=float, default=8., help='static hold for --sweep')
    p.add_argument('--no-video', action='store_true')
    p.add_argument('--output', required=True)
    a = p.parse_args(argv)
    if bool(a.probe) == bool(a.sweep):
        p.error('choose exactly one of --probe / --sweep')
    if a.sweep:
        masses = tuple(float(v) for v in a.masses.split(',')) if a.masses else SWEEP_MASSES
        Path(a.output).mkdir(parents=True, exist_ok=False)
        print(json.dumps(sweep(a.output, masses, a.hold_s), indent=2))
        return 0
    result = Probe(a.probe, PROBES[a.probe], a.output, video=not a.no_video, mass_kg=a.mass_kg).run()
    print(json.dumps({k: result[k] for k in ('probe', 'success', 'teacher_outcome', 'lifted_clear', 'placement',
                                             'max_cargo_tilt_deg', 'max_robot_tilt_deg', 'max_slip_mm',
                                             'sim_time_s', 'wall_s')}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
