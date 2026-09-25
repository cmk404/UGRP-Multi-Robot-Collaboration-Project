"""Offline evaluation of ``harness.zone_rgb_outcome`` on recorded zone runs.

``render``  For every job of every run in a split, re-render the robot-input
            images from the run's observer replay (``replay/model.mjb`` +
            ``replay/states.npz``, 30 SIM fps): the four TOP JPEGs at the job's
            assignment ("before") and, after the job's end, TOP + the job
            robot's own RGB at a fixed delay schedule. Robot inputs go to
            ``<job>/frames`` and ``<job>/inputs.json`` (RGB-estimated label
            position, kind, zone, target slot, own commanded arm pulses).
            Evaluation-only labels (the job box's simulator pose, the teacher
            outcome, segmentation-visible pixels) go to ``<job>/eval-labels.json``
            and are never read by ``score`` before the decisions are made.
``synth``   Render-only synthetic cases from base states: move a box / a robot
            by editing qpos and ``mj_forward`` (no stepping).
``score``   Run the outcome check on inputs only, then join the labels:
            confusion matrices, false "delivered", time-to-detect.

Replay re-render fidelity: see ``validate`` (compares re-rendered frames with
the frames each run captured during control).
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
DEFAULT_OUT = OUTPUTS/'zone-rgb-outcome-20260925'
DELAYS_S = (0., .5, 1., 2., 4., 8.)
# Own commanded arm pulses whenever the teacher has queued its FOLDED pose
# (scripts/zone_teacher.py FOLDED): after every job end and while driving to a box.
FOLDED = {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}
FOLDED_PHASES = ('to_box', 'done', 'failed', 'idle')
GT_SOURCE_M = .15
GT_FLOOR_Z_M = .05


def load_avg():
    return [round(v, 2) for v in os.getloadavg()]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


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

    def visible_px(self, cameras, body):
        """Evaluation only: segmentation pixels of a body in each TOP."""
        bid = self.model.body(body).id
        geoms = {g for g in range(self.model.ngeom) if self.model.geom_bodyid[g] == bid}
        self.renderer.enable_segmentation_rendering()
        try:
            out = {}
            for cam in cameras:
                self.renderer.update_scene(self.data, camera=cam, scene_option=self.top_opt)
                seg = self.renderer.render()
                ids, types = seg[..., 0], seg[..., 1]
                out[cam] = int(np.isin(ids, list(geoms)).__and__(types == int(self.mj.mjtObj.mjOBJ_GEOM)).sum())
            return out
        finally:
            self.renderer.disable_segmentation_rendering()

    def body_xyz(self, body):
        return [float(v) for v in self.data.body(body).xpos]

    def body_qadr(self, body):
        return int(self.model.jnt_qposadr[self.model.body(body).jntadr])

    def close(self):
        self.renderer.close()


def _jobs(run_dir):
    """(robot, job_id, own_jobs entry, assign_t, end_t, outcome) from the run's records."""
    result = json.loads((run_dir/'result.json').read_text())
    events = json.loads((run_dir/'teacher-events.json').read_text())
    assign = {e['job']: e['sim_time_s'] for e in events if e['event'] == 'assign'}
    ends = {}
    for e in events:
        if e['event'] == 'phase' and e.get('phase') in ('done', 'failed'):
            ends[e['job']] = (e['sim_time_s'], e['outcome'])
    job_end = {}
    for e in events:
        if e['event'] == 'job_end':
            job_end.setdefault(e['robot_id'], []).append(e)
    out = []
    for rid, jobs in result['jobs'].items():
        ends_r = iter(job_end.get(rid, []))
        for i, j in enumerate(jobs):
            jid = f'{rid}-{i+1}'
            if j.get('slot') is None or jid not in assign:
                continue
            end = next(ends_r, None)
            out.append({'robot': rid, 'job_id': jid, 'job': j, 'assign_t': assign[jid],
                        'end_t': end['sim_time_s'] if end else None,
                        'teacher_outcome': end['outcome'] if end else 'unfinished_at_run_end',
                        'teacher_phase_end_t': ends.get(jid, (None,))[0]})
    return result, events, out


def _phase_at(events, rid, t):
    phase = 'idle'
    for e in events:
        if e['robot_id'] != rid or e['sim_time_s'] > t:
            continue
        if e['event'] == 'assign':
            phase = 'to_box'
        elif e['event'] == 'phase':
            phase = e['phase']
    return phase


def _gt_class(xyz, source_xyz, zone_region):
    on_floor = xyz[2] < GT_FLOOR_Z_M
    if on_floor and zro.inside_rect(xyz[:2], zone_region['center_m'], zone_region['half_extents_m']):
        return 'delivered'
    if on_floor and math.dist(xyz[:2], source_xyz[:2]) <= GT_SOURCE_M:
        return 'still_at_source'
    return 'elsewhere'


def render(args):
    split = json.loads(Path(args.split_file).read_text())
    runs = split[f'{args.split}_runs']
    if args.only:
        runs = [r for r in runs if args.only in r]
    out_root = Path(args.out)/args.split
    manifest = {'schema': 'ugrp.zone_rgb_outcome.frames.v1', 'split': args.split, 'delays_s': list(DELAYS_S),
                'source_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                'load_avg_start': load_avg(), 'runs': {}}
    started = time.monotonic()
    for rel in runs:
        run_dir = OUTPUTS/rel
        episode = json.loads((run_dir/'episode-setup-only.json').read_text())
        static = episode['static_map']
        labels = json.loads((run_dir/'box-labels.json').read_text())
        result, events, jobs = _jobs(run_dir)
        from sim.zone_arena import top_views
        views = top_views(static)
        rp = Replay(run_dir)
        replay_meta = json.loads((run_dir/'replay'/'replay.json').read_text())
        t_last = float(rp.times[-1])
        run_out = out_root/rel.replace('/', '__')
        entries = []
        for job in jobs:
            jdir = run_out/job['job_id']
            (jdir/'frames').mkdir(parents=True, exist_ok=True)
            j = job['job']
            label = labels[j['box']]
            # --- robot inputs
            rp.set(job['assign_t'])
            before = {}
            for cam, _, _, suffix, _ in views:
                name = f'before-{suffix}.jpg'
                (jdir/'frames'/name).write_bytes(rp.top(cam))
                before[cam] = name
            objects = episode['setup_only']['objects']
            # Evaluation only: the physical box the teacher resolved (nearest to the label).
            oid = min(objects, key=lambda o: math.dist(rp.body_xyz(objects[o]['body_name'])[:2], label['floor_xy_m']))
            body = objects[oid]['body_name']
            source_xyz = rp.body_xyz(body)
            afters, gts = [], []
            end_t = job['end_t'] if job['end_t'] is not None else t_last
            for d in DELAYS_S:
                t = end_t + d
                if t > t_last + 1e-6:
                    break
                rp.set(t)
                tag = f'after-{d:04.1f}s'
                names = {}
                for cam, _, _, suffix, _ in views:
                    name = f'{tag}-{suffix}.jpg'
                    (jdir/'frames'/name).write_bytes(rp.top(cam))
                    names[cam] = name
                phase = _phase_at(events, job['robot'], t)
                own_name, pose = None, None
                if phase in FOLDED_PHASES:
                    own_name, pose = f'{tag}-own-{job["robot"]}.jpg', FOLDED
                    (jdir/'frames'/own_name).write_bytes(rp.own(job['robot']))
                afters.append({'delay_s': d, 'sim_time_s': round(t, 3), 'tops': names, 'own': own_name,
                               'own_commanded_arm_pulses': pose})
                xyz = rp.body_xyz(body)
                gts.append({'delay_s': d, 'box_xyz': [round(v, 4) for v in xyz],
                            'gt_class': _gt_class(xyz, source_xyz, static['regions']['zone_'+j['zone']]),
                            'in_own_slot': zro.inside_rect(xyz[:2], *zro.slot_target(static, j['slot']).values()),
                            'visible_px': rp.visible_px([v[0] for v in views], body)})
            inputs = {'schema': 'ugrp.zone_rgb_outcome.inputs.v1', 'run': rel, 'robot': job['robot'],
                      'job_id': job['job_id'], 'item': j['box'], 'kind': label['kind'],
                      'source_xy_m': label['floor_xy_m'], 'source_of_source_xy': 'box-labels.json (first TOP RGB)',
                      'zone': j['zone'], 'target': zro.slot_target(static, j['slot']), 'slot': j['slot'],
                      'static_map_file': str(run_dir/'episode-setup-only.json') + '#static_map',
                      'before': {'sim_time_s': job['assign_t'], 'tops': before}, 'after': afters,
                      'note': 'after[0] is at the job end the runner observed (executor idle); later '
                              'entries are re-checks at fixed SIM delays'}
            labels_eval = {'schema': 'ugrp.zone_rgb_outcome.eval_labels.v1', 'scope': 'evaluation only; never a robot input',
                           'teacher_outcome': job['teacher_outcome'], 'teacher_phase_end_t': job['teacher_phase_end_t'],
                           'job_end_t': job['end_t'], 'object': oid, 'body': body,
                           'source_xyz_at_assign': [round(v, 4) for v in source_xyz],
                           'injected': bool(result.get('injection') and result['injection'].get('robot') == job['robot']
                                            and result['injection'].get('box') == j['box']
                                            and abs(result['injection']['sim_time_s'] - job['assign_t']) < .01),
                           'after': gts}
            (jdir/'inputs.json').write_text(json.dumps(inputs, indent=1) + '\n')
            (jdir/'eval-labels.json').write_text(json.dumps(labels_eval, indent=1) + '\n')
            entries.append(job['job_id'])
        rp.close()
        manifest['runs'][rel] = {'jobs': entries, 'replay_files_sha256': replay_meta.get('files_sha256'),
                                 'result_sha256': sha(run_dir/'result.json'),
                                 'teacher_events_sha256': sha(run_dir/'teacher-events.json'),
                                 'box_labels_sha256': sha(run_dir/'box-labels.json')}
        print(f'{rel}: {len(entries)} jobs, load {load_avg()}', flush=True)
    manifest['load_avg_end'] = load_avg()
    manifest['wall_s'] = round(time.monotonic()-started, 1)
    out_root.mkdir(parents=True, exist_ok=True)
    (out_root/'frames-manifest.json').write_text(json.dumps(manifest, indent=1) + '\n')


def validate(args):
    """Replay re-render vs the frames captured during control (TOP and own RGB)."""
    import cv2
    split = json.loads(Path(args.split_file).read_text())
    rows = []
    for rel in split['dev_runs'] + split['test_runs']:
        run_dir = OUTPUTS/rel
        team = json.loads((run_dir/'team'/'team.json').read_text())
        rounds = [r for r in team['rounds'] if 'sim_time_s' in r]
        if not rounds:
            continue
        rnd = rounds[len(rounds)//2]
        files = sorted((run_dir/'rgb').glob(f'*-{rnd["phase"]}-top-*.jpg'))
        if not files:
            continue
        rp = Replay(run_dir)
        rp.set(rnd['sim_time_s'])
        episode = json.loads((run_dir/'episode-setup-only.json').read_text())
        from sim.zone_arena import top_views
        suffix_cam = {v[3]: v[0] for v in top_views(episode['static_map'])}
        for f in files:
            suffix = f.stem.split('-top-')[1]
            mine = cv2.imdecode(np.frombuffer(rp.top(suffix_cam['top-'+suffix]), np.uint8), 1)
            rec = cv2.imread(str(f))
            d = np.abs(rec.astype(int)-mine.astype(int))
            rows.append({'run': rel, 'frame': f.name, 'mean_abs': round(float(d.mean()), 4),
                         'frac_gt20': round(float((d.max(2) > 20).mean()), 5),
                         'replay_dt_s': round(float(rp.times[rp.index(rnd['sim_time_s'])] - rnd['sim_time_s']), 4)})
        rp.close()
    Path(args.out).mkdir(parents=True, exist_ok=True)
    summary = {'frames': len(rows), 'max_mean_abs': max(r['mean_abs'] for r in rows),
               'max_frac_gt20': max(r['frac_gt20'] for r in rows), 'rows': rows}
    (Path(args.out)/'replay-validation.json').write_text(json.dumps(summary, indent=1) + '\n')
    print(json.dumps({k: v for k, v in summary.items() if k != 'rows'}))


# ----------------------------------------------------------------------------- scoring

def decide_job(jdir, static_map, profile):
    inputs = json.loads((jdir/'inputs.json').read_text())
    job = zro.job_spec(robot_ids=inputs['robot'], item=inputs['item'], kind=inputs['kind'],
                       source_xy_m=inputs['source_xy_m'], zone=inputs['zone'], target=inputs['target'])
    read = lambda n: (jdir/'frames'/n).read_bytes()  # noqa: E731
    before = {cam: read(n) for cam, n in inputs['before']['tops'].items()}
    before_rows = zro.sightings(before, static_map, [inputs['kind']], profile=profile)
    results = []
    for a in inputs['after']:
        after = {cam: read(n) for cam, n in a['tops'].items()}
        pose = {int(k): v for k, v in a['own_commanded_arm_pulses'].items()} if a['own'] else None
        res = zro.job_outcome(job, before, after, static_map, own_rgb=read(a['own']) if a['own'] else None,
                              own_servo_pose=pose, before_names=inputs['before']['tops'], after_names=a['tops'],
                              own_name=a['own'], profile=profile, before_rows=before_rows)
        res['delay_s'] = a['delay_s']
        results.append(res)
    return inputs, results


def _matrix():
    return {g: {o: 0 for o in zro.OUTCOMES} for g in ('delivered', 'still_at_source', 'elsewhere')}


def score(args):
    split = args.split
    root = Path(args.out)/split
    rows = []
    for jdir in sorted(p.parent for p in root.glob('*/*/inputs.json')):
        inputs0 = json.loads((jdir/'inputs.json').read_text())
        run_dir = OUTPUTS/inputs0['run']
        static = json.loads((run_dir/'episode-setup-only.json').read_text())['static_map']
        inputs, results = decide_job(jdir, static, args.profile)
        # --- decisions are fixed; labels are read only now
        labels = json.loads((jdir/'eval-labels.json').read_text())
        idx, committed = zro.commit(results)
        rows.append({'run': inputs['run'], 'job_id': inputs['job_id'], 'kind': inputs['kind'],
                     'teacher_outcome': labels['teacher_outcome'], 'injected': labels['injected'],
                     'per_delay': [{'delay_s': r['delay_s'], 'outcome': r['outcome'], 'confidence': r['confidence'],
                                    'rule': r['rule'], 'flags': r['flags'],
                                    'gt': g['gt_class'], 'in_own_slot': g['in_own_slot'],
                                    'visible_px_max': max(g['visible_px'].values())}
                                   for r, g in zip(results, labels['after'])],
                     'committed_index': idx, 'committed_outcome': committed['outcome'],
                     'committed_confidence': committed['confidence'],
                     'committed_evidence_images': committed['evidence']['images']})
    summary = summarize(rows)
    out = {'schema': 'ugrp.zone_rgb_outcome.score.v1', 'split': split, 'profile': args.profile,
           'module_sha256': sha(ROOT/'harness'/'zone_rgb_outcome.py'),
           'source_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
           'summary': summary, 'jobs': rows}
    name = args.name or f'score-{split}-{args.profile}.json'
    (root/name).write_text(json.dumps(out, indent=1) + '\n')
    print(json.dumps(summary, indent=1))


def summarize(rows):
    at0, commit_m = _matrix(), _matrix()
    by_teacher = {}
    ttd = []
    false_delivered = []
    for r in rows:
        first = r['per_delay'][0]
        at0[first['gt']][first['outcome']] += 1
        c = r['per_delay'][r['committed_index']]
        commit_m[c['gt']][r['committed_outcome']] += 1
        by_teacher.setdefault(r['teacher_outcome'], {o: 0 for o in zro.OUTCOMES})[r['committed_outcome']] += 1
        for p in r['per_delay']:
            if p['outcome'] == 'delivered' and p['gt'] != 'delivered':
                false_delivered.append({'run': r['run'], 'job_id': r['job_id'], 'delay_s': p['delay_s'],
                                        'gt': p['gt'], 'rule': p['rule'], 'confidence': p['confidence']})
        want = {'delivered': 'delivered', 'still_at_source': 'still_at_source', 'elsewhere': 'seen_elsewhere'}
        hit = next((p['delay_s'] for p in r['per_delay'] if p['outcome'] == want[p['gt']]
                    and p['confidence'] >= zro.COMMIT_CONFIDENCE), None)
        ttd.append({'gt_at_end': first['gt'], 'delay_s': hit})

    def acc(m):
        n = sum(sum(v.values()) for v in m.values())
        ok = m['delivered']['delivered'] + m['still_at_source']['still_at_source'] + m['elsewhere']['seen_elsewhere']
        return {'n': n, 'correct': ok, 'not_seen': sum(v['not_seen'] for v in m.values()),
                'false_delivered': m['still_at_source']['delivered'] + m['elsewhere']['delivered']}
    ttd_summary = {}
    for g in ('delivered', 'still_at_source', 'elsewhere'):
        vals = [t['delay_s'] for t in ttd if t['gt_at_end'] == g]
        found = sorted(v for v in vals if v is not None)
        ttd_summary[g] = {'jobs': len(vals), 'detected_within_8s': len(found),
                          'at_0s': sum(1 for v in found if v == 0), 'median_s': found[len(found)//2] if found else None,
                          'max_s': found[-1] if found else None}
    return {'jobs': len(rows), 'at_job_end': {'matrix': at0, **acc(at0)},
            'committed_within_8s': {'matrix': commit_m, **acc(commit_m)},
            'committed_by_teacher_outcome': by_teacher, 'time_to_detect': ttd_summary,
            'false_delivered_any_delay': false_delivered}


def parser():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest='cmd', required=True)
    for name in ('render', 'score', 'validate'):
        s = sub.add_parser(name)
        s.add_argument('--split-file', default=str(ROOT/'experiments'/'2026-09-25-zone-rgb-outcome'/'split.json'))
        s.add_argument('--out', default=str(DEFAULT_OUT))
        if name != 'validate':
            s.add_argument('--split', choices=('dev', 'test'), required=True)
        if name == 'render':
            s.add_argument('--only', help='substring filter on run paths')
        if name == 'score':
            s.add_argument('--profile', default=zro.TOP_PROFILE)
            s.add_argument('--name')
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    {'render': render, 'score': score, 'validate': validate}[args.cmd](args)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
