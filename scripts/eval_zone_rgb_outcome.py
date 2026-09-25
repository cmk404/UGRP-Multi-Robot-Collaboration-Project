"""Offline evaluation of ``harness.zone_rgb_outcome`` (v2: fixed-cadence tracker).

Every subcommand writes into a NEW ``--out`` directory and aborts if it exists,
so earlier evidence is never overwritten.

``record``   Run the zone runner (``scripts/run_zone_dispatch.py``, unmodified)
             in fixture mode with the observer replay, plus an observation-only
             wrapper that logs every command each robot port issued
             (``port-commands.jsonl``, with the port's initial servo pulses).
             Optional diagnostic ``--inject-drop N``: the N-th assigned job's
             gripper is commanded open 3 s into its carry (a real drop; the
             command is in the log like any other issued command).
``track``    For every job of every run in a split: TOP frames re-rendered from
             the replay on a fixed SIM grid (every ``CADENCE_S``), fed to a
             ``JobTracker`` from the first grid tick after the job's assignment
             until it confirms or the run ends. Reference frame = the run's own
             recorded first TOP capture (the frame the labels came from).
             Commands and own RGB are used only when the run has a port log.
             No simulator pose, teacher event other than the assignment time
             (the runner's own issued job), or label is read here.
``label``    Evaluation only: the job box's simulator pose at each tracked tick.
``score``    Join decisions with labels: confusion, false delivered, time to detect.
``synth``    Render-only synthetic cases (incl. adversarial static occluders).
``validate`` Decision equivalence: ``observe`` on the frames captured during
             control vs the re-rendered frames at the same SIM times.
"""
from __future__ import annotations

import argparse
import hashlib
import io
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

from harness import zone_rgb_outcome as zro  # noqa: E402

OUTPUTS = Path('/Users/changmin/projects/ugrp/outputs')
GT_SOURCE_M = .15
GT_FLOOR_Z_M = .05
DROP_AFTER_S = 3.


def load_avg():
    return [round(v, 2) for v in os.getloadavg()]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def sha_bytes(b):
    return hashlib.sha256(b).hexdigest()


def new_dir(path):
    path = Path(path)
    if path.exists():
        raise SystemExit(f'refusing to overwrite existing output: {path}')
    path.mkdir(parents=True)
    return path


def git_sha():
    return subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()


def run_path(rel):
    p = Path(rel)
    return p if p.is_absolute() else OUTPUTS/rel


# ----------------------------------------------------------------------------- replay

class Replay:
    """Render robot-input images from a recorded run's observer replay (render only)."""
    def __init__(self, run_dir):
        import mujoco
        from sim.masterpi_camera_profile import raw_fisheye_remap
        self.mj = mujoco
        self.dir = Path(run_dir)
        self.model = mujoco.MjModel.from_binary_path(str(self.dir/'replay'/'model.mjb'))
        states = np.load(self.dir/'replay'/'states.npz')
        self.times, self.qpos = states['time'], states['qpos']
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, 720, 960)
        self.fisheye = raw_fisheye_remap(960, 720)
        self.top_opt = mujoco.MjvOption()
        self.top_opt.geomgroup[:] = 1
        self.own_opt = mujoco.MjvOption()
        self.own_opt.geomgroup[:] = 1
        self.own_opt.geomgroup[4] = 0
        self.own_opt.geomgroup[5] = 0

    def index(self, t):
        return int(max(0, min(len(self.times)-1, np.searchsorted(self.times, t, side='right')-1)))

    def set(self, t=None, qpos=None):
        self.data.qpos[:] = self.qpos[self.index(t)] if qpos is None else qpos
        self.mj.mj_forward(self.model, self.data)

    def _jpeg(self, rgb, quality):
        from PIL import Image
        buf = io.BytesIO()
        Image.fromarray(rgb).save(buf, format='JPEG', quality=quality)
        return buf.getvalue()

    def top(self, camera):
        self.renderer.update_scene(self.data, camera=camera, scene_option=self.top_opt)
        return self._jpeg(self.renderer.render().copy(), 95)

    def own(self, rid):
        import cv2
        self.renderer.update_scene(self.data, camera=f'{rid}__robot_cam', scene_option=self.own_opt)
        ideal = self.renderer.render().copy()
        mx, my = self.fisheye
        return self._jpeg(cv2.remap(ideal, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT), 90)

    def body_xyz(self, body):
        return [float(v) for v in self.data.body(body).xpos]

    def close(self):
        self.renderer.close()


def _views(static):
    from sim.zone_arena import top_views
    return top_views(static)


def _reference(run_dir, static):
    """The run's first recorded TOP capture (the frame box-labels.json came from)."""
    tops, names = {}, {}
    for cam, _, _, suffix, _ in _views(static):
        f = sorted((run_dir/'rgb').glob(f'001-*-{suffix}.jpg'))[0]
        tops[cam], names[cam] = f.read_bytes(), str(f)
    return tops, names


def _jobs(run_dir):
    """Issued jobs: (robot, job_id, own_jobs entry, assign time). Assign time is
    the runner's own issue time (teacher 'assign' log = the same tick)."""
    result = json.loads((run_dir/'result.json').read_text())
    events = json.loads((run_dir/'teacher-events.json').read_text())
    assign = {e['job']: e['sim_time_s'] for e in events if e['event'] == 'assign'}
    out = []
    for rid, jobs in result['jobs'].items():
        for i, j in enumerate(jobs):
            jid = f'{rid}-{i+1}'
            if j.get('slot') is None or jid not in assign:
                continue
            out.append({'robot': rid, 'job_id': jid, 'job': j, 'assign_t': assign[jid]})
    return result, out


def _commands(run_dir):
    path = run_dir/'port-commands.jsonl'
    if not path.is_file():
        return None
    out = {}
    for line in path.read_text().splitlines():
        c = json.loads(line)
        out.setdefault(c['robot_id'], []).append(c)
    for v in out.values():
        v.sort(key=lambda c: c['sim_time_s'])
    return out


# ----------------------------------------------------------------------------- record

def record(args):
    out = Path(args.out)
    if out.exists():
        raise SystemExit(f'refusing to overwrite existing output: {out}')
    from sim.camera_robot_port import CameraRobotPort
    import scripts.run_zone_dispatch as rzd
    import scripts.zone_teacher as zt
    log, injected, counter = [], {}, {'n': 0}
    orig_init, orig_apply = CameraRobotPort.__init__, CameraRobotPort.apply

    def init(self, *a, **k):
        orig_init(self, *a, **k)
        log.append({'robot_id': self.robot_id, 'sim_time_s': float(self._world.data.time), 'kind': 'initial',
                    'pulses': {str(s): int(p) for s, p in self._servo_pulses.items()}})

    def apply(self, action, sim_time):
        ack = orig_apply(self, action, sim_time)
        log.append({'robot_id': self.robot_id, 'sim_time_s': float(sim_time), **dict(action)})
        return ack
    CameraRobotPort.__init__, CameraRobotPort.apply = init, apply
    orig_assign, orig_tick = zt.TeacherRobot.assign, zt.TeacherRobot.tick

    def assign(self, job, now):
        counter['n'] += 1
        if args.inject_drop and counter['n'] == args.inject_drop:
            injected['job_id'] = job['job_id']
        return orig_assign(self, job, now)

    def tick(self, now, discs_for):
        if (injected.get('job_id') and self.job and self.job['job_id'] == injected['job_id']
                and self.phase == 'carry' and now - self.phase_started >= DROP_AFTER_S and 'at' not in injected):
            injected['at'] = round(now, 3)
            self.arm.queue({1: zt.OPEN}, now, duration=.2, settle=0.)
        return orig_tick(self, now, discs_for)
    if args.inject_drop:
        zt.TeacherRobot.assign, zt.TeacherRobot.tick = assign, tick
    load0 = load_avg()
    argv = ['--output', str(out), '--mode', 'fixture', '--coordination', args.coordination, '--seed', str(args.seed),
            '--variant', args.variant, '--record-replay', '--goal', args.goal, '--extra-boxes', args.extra_boxes]
    if args.inject_grasp_failure:
        argv += ['--inject-grasp-failure', str(args.inject_grasp_failure)]
    try:
        code = rzd.main(argv)
    finally:
        CameraRobotPort.__init__, CameraRobotPort.apply = orig_init, orig_apply
        zt.TeacherRobot.assign, zt.TeacherRobot.tick = orig_assign, orig_tick
    (out/'port-commands.jsonl').write_text(''.join(json.dumps(c) + '\n' for c in log))
    meta = {'schema': 'ugrp.zone_rgb_outcome.record.v1', 'runner_argv': argv, 'exit': code,
            'wrapper': 'observation-only port command log; runner/teacher sources unmodified',
            'diagnostic_drop_injection': ({'job_index': args.inject_drop, **injected} if args.inject_drop else None),
            'source_sha': git_sha(), 'load_avg_start': load0, 'load_avg_end': load_avg(),
            'port_commands_sha256': sha(out/'port-commands.jsonl')}
    (out/'record-meta.json').write_text(json.dumps(meta, indent=1) + '\n')
    print(json.dumps(meta))


# ----------------------------------------------------------------------------- track

def track(args):
    split = json.loads(Path(args.split_file).read_text())
    runs = split[args.split]
    if args.only:
        runs = [r for r in runs if args.only in r]
    if args.part:
        i, n = (int(v) for v in args.part.split('/'))
        runs = runs[i::n]
    out_root = new_dir(args.out)
    manifest = {'schema': 'ugrp.zone_rgb_outcome.track.v2', 'split': args.split, 'part': args.part, 'cadence_s': zro.CADENCE_S,
                'deadline_s': zro.DEADLINE_S, 'source_sha': git_sha(),
                'module_sha256': sha(ROOT/'harness'/'zone_rgb_outcome.py'), 'load_avg_start': load_avg(), 'runs': {}}
    started = time.monotonic()
    for rel in runs:
        run_dir = run_path(rel)
        static = json.loads((run_dir/'episode-setup-only.json').read_text())['static_map']
        labels = json.loads((run_dir/'box-labels.json').read_text())
        _, jobs = _jobs(run_dir)
        commands = _commands(run_dir)
        views = _views(static)
        reference, ref_names = _reference(run_dir, static)
        rp = Replay(run_dir)
        t_end = float(rp.times[-1])
        rdir = out_root/rel.replace('/', '__')
        (rdir/'frames').mkdir(parents=True)
        trackers = {}
        for job in jobs:
            j = job['job']
            spec = zro.job_spec(robot_ids=job['robot'], item=j['box'], kind=labels[j['box']]['kind'],
                                source_xy_m=labels[j['box']]['floor_xy_m'], zone=j['zone'],
                                target=zro.slot_target(static, j['slot']))
            rp.set(job['assign_t'])
            before, bnames = {}, {}
            for cam, _, _, suffix, _ in views:
                data = rp.top(cam)
                name = f"before-{job['job_id']}-{suffix}.jpg"
                (rdir/'frames'/name).write_bytes(data)
                before[cam], bnames[cam] = data, name
            trackers[job['job_id']] = {
                'job': job, 'spec': spec, 'history': [],
                'tracker': zro.JobTracker(spec, reference, before, static, assigned_at=job['assign_t']),
                'inputs': {'run': rel, 'job_id': job['job_id'], 'robot': job['robot'], 'spec': spec,
                           'assigned_at': job['assign_t'], 'reference_frames': ref_names, 'before_frames': bnames,
                           'commands_logged': commands is not None, 'slot': j['slot']}}
        # A job's window closes when its robot is issued its next job (the robot's
        # own job history; the issue time itself follows the executor end: L4b).
        by_robot = {}
        for job in sorted(jobs, key=lambda j: j['assign_t']):
            by_robot.setdefault(job['robot'], []).append(job)
        for seq in by_robot.values():
            for a, b in zip(seq, seq[1:]):
                trackers[a['job_id']]['close_at'] = b['assign_t']
        # Fixed SIM grid, independent of any executor end.
        k = math.floor(min(j['assign_t'] for j in jobs)/zro.CADENCE_S) + 1
        while k*zro.CADENCE_S <= t_end + 1e-9:
            t = round(k*zro.CADENCE_S, 6)
            k += 1
            for v in trackers.values():
                if v.get('close_at') is not None and v['close_at'] < t and v['tracker'].decision['status'] != 'confirmed':
                    v['tracker'].close(v['close_at'], commands=commands)
            live = [v for v in trackers.values() if v['job']['assign_t'] < t
                    and v['tracker'].decision['status'] != 'confirmed']
            if not live:
                if all(v['job']['assign_t'] < t for v in trackers.values()):
                    break
                continue
            rp.set(t)
            tops, names = {}, {}
            for cam, _, _, suffix, _ in views:
                data = rp.top(cam)
                name = f'tick-{t:07.1f}-{suffix}.jpg'
                (rdir/'frames'/name).write_bytes(data)
                tops[cam], names[cam] = data, name
            own_cache = {}
            for v in live:
                rid = v['job']['robot']
                own = None
                if commands is not None:
                    if rid not in own_cache:
                        own_cache[rid] = rp.own(rid)
                        (rdir/'frames'/f'tick-{t:07.1f}-own-{rid}.jpg').write_bytes(own_cache[rid])
                    own = own_cache[rid]
                d = v['tracker'].update(t, tops, commands=commands, own_rgb=own,
                                        names=names | ({'own': f'tick-{t:07.1f}-own-{rid}.jpg'} if own else {}))
                v['history'].append({**v['tracker'].history[-1], 'status': d['status']})
        for jid, v in trackers.items():
            jdir = rdir/jid
            jdir.mkdir()
            (jdir/'inputs.json').write_text(json.dumps(v['inputs'], indent=1) + '\n')
            (jdir/'observations.jsonl').write_text(''.join(json.dumps(h) + '\n' for h in v['history']))
            (jdir/'decision.json').write_text(json.dumps(v['tracker'].decision, indent=1) + '\n')
        rp.close()
        manifest['runs'][rel] = {'jobs': sorted(trackers), 'commands_logged': commands is not None,
                                 'replay_files_sha256': json.loads((run_dir/'replay'/'replay.json').read_text()).get('files_sha256')}
        print(f'{rel}: {len(trackers)} jobs, load {load_avg()}', flush=True)
    manifest['load_avg_end'] = load_avg()
    manifest['wall_s'] = round(time.monotonic()-started, 1)
    (out_root/'track-manifest.json').write_text(json.dumps(manifest, indent=1) + '\n')


# ----------------------------------------------------------------------------- labels / score

def _gt_class(xyz, source_xyz, zone_region):
    on_floor = xyz[2] < GT_FLOOR_Z_M
    if on_floor and zro.inside_rect(xyz[:2], zone_region['center_m'], zone_region['half_extents_m']):
        return 'delivered'
    if on_floor and math.dist(xyz[:2], source_xyz[:2]) <= GT_SOURCE_M:
        return 'still_at_source'
    return 'elsewhere'


def label(args):
    """Evaluation only. Reads simulator poses from the replay and teacher events."""
    track_dir = Path(args.track)
    out_root = new_dir(args.out)
    for inp_path in sorted(track_dir.glob('*/*/inputs.json')):
        inp = json.loads(inp_path.read_text())
        run_dir = run_path(inp['run'])
        episode = json.loads((run_dir/'episode-setup-only.json').read_text())
        objects, static = episode['setup_only']['objects'], episode['static_map']
        events = json.loads((run_dir/'teacher-events.json').read_text())
        rp = Replay(run_dir)
        rp.set(inp['assigned_at'])
        src_xy = inp['spec']['source_xy_m']
        oid = min(objects, key=lambda o: math.dist(rp.body_xyz(objects[o]['body_name'])[:2], src_xy))
        body = objects[oid]['body_name']
        source_xyz = rp.body_xyz(body)
        region = static['regions']['zone_'+inp['spec']['zone']]
        ticks = [json.loads(line)['t'] for line in (inp_path.parent/'observations.jsonl').read_text().splitlines()]
        gt = []
        for t in ticks:
            rp.set(t)
            xyz = rp.body_xyz(body)
            gt.append({'t': t, 'gt': _gt_class(xyz, source_xyz, region), 'z': round(xyz[2], 4)})
        dec = json.loads((inp_path.parent/'decision.json').read_text())
        gt_decision = None
        if dec.get('decided_at') is not None:
            rp.set(dec['decided_at'])
            gt_decision = _gt_class(rp.body_xyz(body), source_xyz, region)
        rp.set(float(rp.times[-1]))
        final = _gt_class(rp.body_xyz(body), source_xyz, region)
        end = next((e for e in events if e['event'] == 'phase' and e.get('job') == inp['job_id']
                    and e.get('phase') in ('done', 'failed')), None)
        injected = None
        meta = run_dir/'record-meta.json'
        if meta.is_file():
            m = json.loads(meta.read_text())
            if (m.get('diagnostic_drop_injection') or {}).get('job_id') == inp['job_id']:
                injected = 'drop'
        res = json.loads((run_dir/'result.json').read_text())
        if res.get('injection') and res['injection'].get('robot') == inp['robot'] and \
                res['injection'].get('box') == inp['spec']['item'] and \
                abs(res['injection']['sim_time_s'] - inp['assigned_at']) < .01:
            injected = 'grasp_stays_open'
        first_delivered = next((g['t'] for g in gt if g['gt'] == 'delivered'), None)
        d = out_root/inp_path.parent.relative_to(track_dir)
        d.mkdir(parents=True)
        (d/'eval-labels.json').write_text(json.dumps({
            'scope': 'evaluation only; never a robot input', 'object': oid, 'body': body,
            'teacher_outcome': end['outcome'] if end else 'unfinished_at_run_end',
            'teacher_end_t': end['sim_time_s'] if end else None, 'injected': injected,
            'first_gt_delivered_t': first_delivered, 'final_gt': final, 'gt_at_decision': gt_decision,
            'ticks': gt}, indent=1) + '\n')
        rp.close()
    print(f'labels written to {out_root}')


COLS = ('delivered', 'still_at_source', 'seen_elsewhere', 'not_seen', 'unconfirmed')
ROWS = ('delivered', 'still_at_source', 'elsewhere')
RIGHT = {'delivered': 'delivered', 'still_at_source': 'still_at_source', 'elsewhere': 'seen_elsewhere'}


def score(args):
    if len(args.track) != len(args.labels):
        raise SystemExit('give one --labels dir per --track dir, in the same order')
    out_root = new_dir(args.out)
    rows = []
    pairs = [(Path(t), Path(lb), p) for t, lb in zip(args.track, args.labels)
             for p in sorted(Path(t).glob('*/*/decision.json'))]
    for track_dir, label_dir, dec_path in pairs:
        rel = dec_path.parent.relative_to(track_dir)
        dec = json.loads(dec_path.read_text())
        inp = json.loads((dec_path.parent/'inputs.json').read_text())
        obs = [json.loads(line) for line in (dec_path.parent/'observations.jsonl').read_text().splitlines()]
        lab = json.loads((label_dir/rel/'eval-labels.json').read_text())
        gt_at = {g['t']: g['gt'] for g in lab['ticks']}
        if dec['status'] == 'confirmed':
            gt = lab['gt_at_decision']
        else:
            gt = lab['ticks'][-1]['gt'] if lab['ticks'] else lab['final_gt']
        raw_fd = [o['t'] for o in obs if o['rule'] == 'delivered_source_proven' and gt_at.get(o['t']) != 'delivered']
        rows.append({'run': inp['run'], 'job_id': inp['job_id'], 'kind': inp['spec']['kind'],
                     'commands_logged': inp['commands_logged'], 'teacher_outcome': lab['teacher_outcome'],
                     'injected': lab['injected'], 'status': dec['status'],
                     'outcome': dec['outcome'] if dec['status'] == 'confirmed' else 'unconfirmed',
                     'rule': dec['rule'], 'decided_at': dec['decided_at'], 'assigned_at': inp['assigned_at'],
                     'gt_at_decision': gt, 'final_gt': lab['final_gt'],
                     'first_gt_delivered_t': lab['first_gt_delivered_t'], 'teacher_end_t': lab['teacher_end_t'],
                     'ticks': len(obs), 'raw_proven_delivered_on_non_delivered_ticks': raw_fd})
    summary = summarize(rows)
    out = {'schema': 'ugrp.zone_rgb_outcome.score.v2', 'track': args.track, 'labels': args.labels,
           'source_sha': git_sha(), 'summary': summary, 'jobs': rows}
    (out_root/'score.json').write_text(json.dumps(out, indent=1) + '\n')
    print(json.dumps(summary, indent=1))


def summarize(rows):
    m = {g: {c: 0 for c in COLS} for g in ROWS}
    by_teacher = {}
    for r in rows:
        m[r['gt_at_decision']][r['outcome']] += 1
        by_teacher.setdefault(r['teacher_outcome'], {c: 0 for c in COLS})[r['outcome']] += 1
    fd = [r for r in rows if r['outcome'] == 'delivered' and r['gt_at_decision'] != 'delivered']
    fd_final = [r for r in rows if r['outcome'] == 'delivered' and r['final_gt'] != 'delivered']
    ttd = {}
    for kind, sel, ref in (('delivered', lambda r: r['outcome'] == 'delivered' and r['gt_at_decision'] == 'delivered',
                            'first_gt_delivered_t'),
                           ('non_delivered_confirmed', lambda r: r['outcome'] in ('still_at_source', 'seen_elsewhere')
                            and RIGHT[r['gt_at_decision']] == r['outcome'], 'teacher_end_t')):
        vals = sorted(round(r['decided_at'] - r[ref], 1) for r in rows if sel(r) and r[ref] is not None)
        since_assign = sorted(round(r['decided_at'] - r['assigned_at'], 1) for r in rows if sel(r))
        ttd[kind] = {'n': len(vals), 'reference': ref + ' (evaluation only)',
                     'median_s': vals[len(vals)//2] if vals else None, 'max_s': vals[-1] if vals else None,
                     'p90_s': vals[int(.9*(len(vals)-1))] if vals else None,
                     'since_assignment_median_s': since_assign[len(since_assign)//2] if since_assign else None}
    rules = {}
    for r in rows:
        if r['status'] == 'confirmed':
            rules[r['rule']] = rules.get(r['rule'], 0) + 1
    correct = sum(1 for r in rows if r['outcome'] == RIGHT[r['gt_at_decision']])
    return {'jobs': len(rows), 'matrix': m, 'correct': correct,
            'unconfirmed_at_run_end': sum(1 for r in rows if r['status'] != 'confirmed'),
            'confirmed_not_seen': sum(1 for r in rows if r['outcome'] == 'not_seen'),
            'false_delivered_at_decision': [(r['run'], r['job_id']) for r in fd],
            'false_delivered_vs_final_gt': [(r['run'], r['job_id'], r['final_gt']) for r in fd_final],
            'raw_proven_delivered_observations_on_non_delivered_ticks':
                sum(len(r['raw_proven_delivered_on_non_delivered_ticks']) for r in rows),
            'by_teacher_outcome': by_teacher, 'confirm_rules': rules, 'time_to_detect': ttd}


# ----------------------------------------------------------------------------- synthetic

SYNTH_CASES = {
    # case: (safe confirmed outcomes, description)
    'drop_mid_route_visible': (('seen_elsewhere',), 'box on the floor half way to the target, robot 0.45 m behind it'),
    'drop_under_robot': (('seen_elsewhere', 'not_seen'), 'box half way, 0.13 m in front of the stopped robot'),
    'grasp_fail_robot_at_pregrasp': (('still_at_source',), 'box at source, robot at the pregrasp pose 0.255 m west'),
    'grasp_fail_robot_over_source': (('still_at_source', 'not_seen'), 'box at source, robot 0.10 m west (over it)'),
    'placed_robot_over_target': (('delivered', 'not_seen', 'seen_elsewhere'), 'box in its slot, robot 0.10 m west of it'),
    'placed_arm_over_target': (('delivered', 'not_seen'), 'box in its slot, robot at the release pose 0.235 m west'),
    'placed_clear': (('delivered',), 'box in its slot, robot backed off 0.45 m'),
    'wrong_zone': (('seen_elsewhere',), 'box in a slot of another zone, robot backed off'),
    'wrong_kind_in_target_source_occluded': (('still_at_source', 'not_seen'),
                                             'own box at source under the robot; a different-kind box in the own slot'),
    'peer_same_kind_own_slot_source_occluded': (('still_at_source', 'not_seen'),
                                                'own box at source under the robot; a same-kind box in the own slot'),
    # Adversarial (review item 2): the occluder is already there at assignment and never moves.
    'static_occluder_from_assignment_box_stays_peer_in_target': (
        ('still_at_source', 'not_seen'),
        'robot over the source at assignment and now (unchanged ring); own box still under it; same-kind box in own slot'),
    'static_occluder_from_assignment_box_gone_peer_in_target': (
        ('still_at_source', 'not_seen', 'seen_elsewhere'),
        'robot over the source at assignment and now; own box removed from view; same-kind box in own slot'),
    'static_occluder_from_reference_peer_in_target': (
        ('still_at_source', 'not_seen', 'seen_elsewhere'),
        'reference frame, assignment and now all have a robot over the source; own box removed; same-kind box in own slot'),
    # Known residual: colour cannot tell two same-kind boxes apart.
    'identity_swap_source_empty_peer_in_target': (
        ('seen_elsewhere', 'not_seen'),
        'own box carried away out of view; source visibly empty; a same-kind box in own slot (expected to fail: identity)'),
}


def _set_free(model, q, joint, xy, yaw=0., z=None):
    adr = int(model.jnt_qposadr[model.joint(joint).id])
    q[adr:adr+2] = xy
    if z is not None:
        q[adr+2] = z
    q[adr+3:adr+7] = [math.cos(yaw/2), 0., 0., math.sin(yaw/2)]


def synth(args):
    split = json.loads(Path(args.split_file).read_text())
    bases = split[args.split]
    out_root = new_dir(args.out)
    manifest = {'schema': 'ugrp.zone_rgb_outcome.synth.v2', 'split': args.split,
                'cases': {k: {'safe': list(v[0]), 'description': v[1]} for k, v in SYNTH_CASES.items()},
                'source_sha': git_sha(), 'module_sha256': sha(ROOT/'harness'/'zone_rgb_outcome.py'),
                'load_avg_start': load_avg(),
                'method': 'render-only qpos edit + mj_forward; tracker fed the edited frame on two cadence ticks '
                          'with the deadline reached, no commands', 'rows': []}
    for rel in bases:
        run_dir = run_path(rel)
        episode = json.loads((run_dir/'episode-setup-only.json').read_text())
        static, objects = episode['static_map'], episode['setup_only']['objects']
        labels = json.loads((run_dir/'box-labels.json').read_text())
        _, jobs = _jobs(run_dir)
        events = json.loads((run_dir/'teacher-events.json').read_text())
        ends = {e['job']: e['sim_time_s'] for e in events if e['event'] == 'phase' and e.get('outcome') == 'placed_by_teacher'}
        views = _views(static)
        reference, _ = _reference(run_dir, static)
        rp = Replay(run_dir)
        placed = [j for j in jobs if j['job_id'] in ends][:args.jobs_per_run]
        for job in placed:
            j, rid = job['job'], job['robot']
            label_ = labels[j['box']]
            rp.set(job['assign_t'])
            oid = min(objects, key=lambda o: math.dist(rp.body_xyz(objects[o]['body_name'])[:2], label_['floor_xy_m']))
            body, kind = objects[oid]['body_name'], objects[oid]['kind']
            src = rp.body_xyz(body)
            assign_q = rp.qpos[rp.index(job['assign_t'])].copy()
            base_q = rp.qpos[rp.index(ends[job['job_id']] + .5)].copy()
            first_q = rp.qpos[0].copy()
            tgt = zro.slot_target(static, j['slot'])['center_m']
            mid = [(src[0]+tgt[0])/2, (src[1]+tgt[1])/2]
            other_zone = next(z for z in static['zone_slots'] if z != j['zone'])
            other_slot = static['zone_slots'][other_zone][0]['center_m']
            rj, box = f'{rid}__base_free', objects[oid]['joint_name']
            same = [o for o in objects if o != oid and objects[o]['kind'] == kind]
            diff = [o for o in objects if objects[o]['kind'] != kind]
            for case in SYNTH_CASES:
                q, before_q, ref_tops = base_q.copy(), assign_q, reference
                if case == 'drop_mid_route_visible':
                    _set_free(rp.model, q, box, mid, z=.016); _set_free(rp.model, q, rj, [mid[0]-.45, mid[1]])
                elif case == 'drop_under_robot':
                    _set_free(rp.model, q, box, mid, z=.016); _set_free(rp.model, q, rj, [mid[0]-.13, mid[1]])
                elif case == 'grasp_fail_robot_at_pregrasp':
                    _set_free(rp.model, q, box, src[:2], z=.016); _set_free(rp.model, q, rj, [src[0]-.255, src[1]])
                elif case == 'grasp_fail_robot_over_source':
                    _set_free(rp.model, q, box, src[:2], z=.016); _set_free(rp.model, q, rj, [src[0]-.10, src[1]])
                elif case == 'placed_robot_over_target':
                    _set_free(rp.model, q, box, tgt, z=.016); _set_free(rp.model, q, rj, [tgt[0]-.10, tgt[1]])
                elif case == 'placed_arm_over_target':
                    _set_free(rp.model, q, box, tgt, z=.016); _set_free(rp.model, q, rj, [tgt[0]-.235, tgt[1]])
                elif case == 'placed_clear':
                    _set_free(rp.model, q, box, tgt, z=.016); _set_free(rp.model, q, rj, [tgt[0]-.45, tgt[1]])
                elif case == 'wrong_zone':
                    _set_free(rp.model, q, box, other_slot, z=.016)
                    _set_free(rp.model, q, rj, [other_slot[0]-.45, other_slot[1]])
                elif case in ('wrong_kind_in_target_source_occluded', 'peer_same_kind_own_slot_source_occluded'):
                    pool = diff if case.startswith('wrong_kind') else same
                    if not pool:
                        continue
                    _set_free(rp.model, q, box, src[:2], z=.016); _set_free(rp.model, q, rj, [src[0]-.10, src[1]])
                    _set_free(rp.model, q, objects[pool[0]]['joint_name'], tgt, z=.016)
                elif case.startswith('static_occluder') or case.startswith('identity_swap'):
                    if not same:
                        continue
                    before_q = assign_q.copy()
                    if not case.startswith('identity_swap'):
                        _set_free(rp.model, before_q, rj, [src[0]-.10, src[1]])
                        _set_free(rp.model, q, rj, [src[0]-.10, src[1]])
                    if case.endswith('box_stays_peer_in_target'):
                        _set_free(rp.model, q, box, src[:2], z=.016)
                    else:
                        _set_free(rp.model, q, box, mid, z=-.5)   # below the floor: out of every view
                    peer_adr = objects[same[0]]['joint_name']
                    _set_free(rp.model, q, peer_adr, tgt, z=.016)
                    if case == 'static_occluder_from_reference_peer_in_target':
                        rq = first_q.copy()
                        _set_free(rp.model, rq, rj, [src[0]-.10, src[1]])
                        rp.set(qpos=rq)
                        ref_tops = {cam: rp.top(cam) for cam, *_ in views}
                    if case.startswith('identity_swap'):
                        _set_free(rp.model, q, rj, [tgt[0]-.45, tgt[1]])
                spec = zro.job_spec(robot_ids=rid, item=j['box'], kind=label_['kind'], source_xy_m=label_['floor_xy_m'],
                                    zone=j['zone'], target=zro.slot_target(static, j['slot']))
                rp.set(qpos=before_q)
                before = {cam: rp.top(cam) for cam, *_ in views}
                rp.set(qpos=q)
                cur = {cam: rp.top(cam) for cam, *_ in views}
                cdir = out_root/f"{rel.replace('/', '__')}__{job['job_id']}"/case
                (cdir/'frames').mkdir(parents=True)
                for cam, _, _, suffix, _ in views:
                    (cdir/'frames'/f'reference-{suffix}.jpg').write_bytes(ref_tops[cam])
                    (cdir/'frames'/f'before-{suffix}.jpg').write_bytes(before[cam])
                    (cdir/'frames'/f'current-{suffix}.jpg').write_bytes(cur[cam])
                t0 = 1000.
                tr = zro.JobTracker(spec, ref_tops, before, static, assigned_at=t0 - zro.DEADLINE_S, keep_evidence=True)
                tr.update(t0, cur)
                dec = tr.update(t0 + zro.CADENCE_S, cur)
                xyz = rp.body_xyz(body)
                outcome = dec['outcome'] if dec['status'] == 'confirmed' else 'unconfirmed'
                row = {'base': rel, 'job_id': job['job_id'], 'case': case, 'kind': kind,
                       'observation': {k: tr.history[-1][k] for k in ('outcome', 'rule', 'flags')},
                       'source_state': tr.observations[-1]['evidence']['source']['state'],
                       'decision': dec, 'outcome': outcome, 'safe': outcome in SYNTH_CASES[case][0],
                       'gt': _gt_class(xyz, src, static['regions']['zone_'+j['zone']])}
                (cdir/'result.json').write_text(json.dumps(row, indent=1) + '\n')
                manifest['rows'].append({k: row[k] for k in ('base', 'job_id', 'case', 'outcome', 'safe', 'gt')}
                                        | {'rule': row['observation']['rule']})
        rp.close()
        print(f'{rel}: synth done, load {load_avg()}', flush=True)
    cases = {}
    for r in manifest['rows']:
        c = cases.setdefault(r['case'], {'n': 0, 'safe': 0, 'outcomes': {}})
        c['n'] += 1
        c['safe'] += r['safe']
        c['outcomes'][r['outcome']] = c['outcomes'].get(r['outcome'], 0) + 1
    manifest['summary'] = cases
    manifest['confirmed_false_delivered'] = [r for r in manifest['rows'] if r['outcome'] == 'delivered' and r['gt'] != 'delivered']
    manifest['load_avg_end'] = load_avg()
    (out_root/'synth-manifest.json').write_text(json.dumps(manifest, indent=1) + '\n')
    print(json.dumps({'summary': cases, 'confirmed_false_delivered': manifest['confirmed_false_delivered']}, indent=1))


# ----------------------------------------------------------------------------- decision equivalence

def validate(args):
    """``observe`` on frames captured during control vs re-rendered frames at the
    same SIM times, for every job active at each recorded claim capture."""
    split = json.loads(Path(args.split_file).read_text())
    runs = [r for key in args.splits.split(',') for r in split[key]]
    out_root = new_dir(args.out)
    rows = []
    for rel in runs:
        run_dir = run_path(rel)
        static = json.loads((run_dir/'episode-setup-only.json').read_text())['static_map']
        labels = json.loads((run_dir/'box-labels.json').read_text())
        _, jobs = _jobs(run_dir)
        team = json.loads((run_dir/'team'/'team.json').read_text())
        views = _views(static)
        reference, _ = _reference(run_dir, static)
        rp = Replay(run_dir)
        suffix_of = {v[0]: v[3] for v in views}
        befores = {}
        for rnd in team['rounds']:
            t = rnd.get('sim_time_s')
            files = {cam: sorted((run_dir/'rgb').glob(f"*-{rnd['phase']}-{suffix_of[cam]}.jpg")) for cam in suffix_of}
            if t is None or not all(len(f) == 1 for f in files.values()):
                continue
            recorded = {cam: f[0].read_bytes() for cam, f in files.items()}
            rp.set(t)
            rendered = {cam: rp.top(cam) for cam in suffix_of}
            for job in jobs:
                if not job['assign_t'] < t:
                    continue
                j = job['job']
                spec = zro.job_spec(robot_ids=job['robot'], item=j['box'], kind=labels[j['box']]['kind'],
                                    source_xy_m=labels[j['box']]['floor_xy_m'], zone=j['zone'],
                                    target=zro.slot_target(static, j['slot']))
                if job['job_id'] not in befores:
                    rp.set(job['assign_t'])
                    befores[job['job_id']] = {cam: rp.top(cam) for cam in suffix_of}
                before = befores[job['job_id']]
                a = zro.observe(spec, reference, before, recorded, static)
                b = zro.observe(spec, reference, before, rendered, static)
                rows.append({'run': rel, 'phase': rnd['phase'], 't': t, 'job_id': job['job_id'],
                             'recorded': a['rule'], 'rendered': b['rule'], 'same': a['rule'] == b['rule']})
        rp.close()
    diffs = [r for r in rows if not r['same']]
    summary = {'observations': len(rows), 'same_rule': len(rows) - len(diffs), 'differences': diffs,
               'load_avg_end': load_avg()}
    (out_root/'decision-equivalence.json').write_text(json.dumps({'summary': summary, 'rows': rows}, indent=1) + '\n')
    print(json.dumps({k: v for k, v in summary.items() if k != 'differences'} | {'n_diff': len(diffs)}))


def parser():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest='cmd', required=True)
    split_file = str(ROOT/'experiments'/'2026-09-25-zone-rgb-outcome'/'split-v2.json')
    s = sub.add_parser('record')
    s.add_argument('--out', required=True)
    s.add_argument('--seed', type=int, required=True)
    s.add_argument('--variant', default='zone_wide')
    s.add_argument('--coordination', default='dynamic', choices=('dynamic', 'independent', 'plan_first'))
    s.add_argument('--inject-grasp-failure', type=int, default=0)
    s.add_argument('--inject-drop', type=int, default=0)
    s.add_argument('--goal', default=json.dumps({'A': {'red': 2}, 'B': {'cyan': 2}, 'C': {'green': 1, 'yellow': 1}}))
    s.add_argument('--extra-boxes', default=json.dumps({'red': 1, 'cyan': 1}))
    s = sub.add_parser('track')
    s.add_argument('--split-file', default=split_file)
    s.add_argument('--split', required=True)
    s.add_argument('--out', required=True)
    s.add_argument('--only')
    s.add_argument('--part', help='i/n: every n-th run starting at i')
    s = sub.add_parser('label')
    s.add_argument('--track', required=True)
    s.add_argument('--out', required=True)
    s = sub.add_parser('score')
    s.add_argument('--track', required=True, nargs='+')
    s.add_argument('--labels', required=True, nargs='+')
    s.add_argument('--out', required=True)
    s = sub.add_parser('synth')
    s.add_argument('--split-file', default=split_file)
    s.add_argument('--split', required=True)
    s.add_argument('--out', required=True)
    s.add_argument('--jobs-per-run', type=int, default=2)
    s = sub.add_parser('validate')
    s.add_argument('--split-file', default=split_file)
    s.add_argument('--splits', required=True, help='comma-separated split keys')
    s.add_argument('--out', required=True)
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    {'record': record, 'track': track, 'label': label, 'score': score, 'synth': synth,
     'validate': validate}[args.cmd](args)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
