"""Physical no-LLM plumbing run of the integrated zone study (issue #223).

One MuJoCo world (sync SIM), three robots, one own-camera executor each (#206),
driven by the study core (#194) through ``harness.zone_study_integration``:

    fixture actor reply -> SIM-costed release -> executor job API -> physics
    -> own executor events -> own wake triggers -> next call (own RGB inputs)

The physics owner advances in ``QUANTUM_S`` chunks; every scheduler event lands
on a chunk boundary. Robot inputs: own robot_cam frames, own issued commands,
static tagged map, order sheet from the scenario config, and (by condition) the
messages actually delivered. Simulator truth goes to ``eval_only/`` only. Weld
OFF, contact profile ``cargo_noslip_v1`` (user approval 2026-09-26).

Pose provider: chosen by ``--prereg``'s ``pose_provider`` from
``configs/zone_study_integration/pose_providers.json``; ``tags_temporary`` is the
own-camera wall-tag PF, a TEMPORARY provider: "임시, 표식 사용, 연구 결과 아님".

This is plumbing, not a study result: no model call, fixture decisions only.

    python scripts/run_zone_study_integration.py --prereg experiments/2026-09-26-zone-study-integration/prereg.json \
        --episode smoke-i700 --condition no_comm --output /Users/changmin/projects/ugrp/outputs/zone-study-integration
    python scripts/run_zone_study_integration.py --prereg ... --episode smoke-i700 --bundle   # print the run bundle only
"""
from __future__ import annotations

import argparse
import base64
import collections
import copy
import json
import math
import os
import platform
import subprocess
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness import zone_study_integration as zi  # noqa: E402
from harness import zone_study_offline as zo  # noqa: E402
from harness.zone_own_executor import OwnCamTeamHost, ROBOTS  # noqa: E402
from harness.zone_study_contract import MAIN_CONDITIONS, digest  # noqa: E402

SCHEMA = 'ugrp.zone_study_integration_run.v1'
ON_FLOOR_MAX_Z_M = .05
SLOT_HALF_M = .06
SETTLE_S = 2.0
TAP_FRAMES = 64
RUNTIME_FILES = (
    'scripts/run_zone_study_integration.py', 'harness/zone_study_integration.py', 'harness/zone_study_offline.py',
    'harness/zone_study_contract.py', 'harness/zone_study_inputs.py', 'harness/zone_study_prompts_ko.py',
    'harness/zone_study_protocol.py', 'harness/zone_sim_cost.py', 'harness/zone_event_scheduler.py',
    'harness/zone_study_eval.py', 'harness/zone_study_scenarios.py', 'harness/zone_map_schematic.py',
    'harness/team_carry_status.py', 'harness/zone_own_executor.py', 'harness/m1_owncam_delivery.py',
    'harness/m1_owncam_contract.py', 'harness/m1_contract.py', 'harness/owncam_pose_source.py',
    'harness/owncam_localizer.py', 'harness/owncam_drive.py', 'harness/owncam_drive_v2.py', 'harness/wall_tags.py',
    'harness/map_goto.py', 'harness/zone_own_perception.py', 'harness/wrist_zone_skill_v9.py',
    'harness/wrist_zone_skill_v6.py', 'sim/zone_cargo_contact.py', 'sim/zone_landmarks.py', 'sim/zone_scene.py',
    'sim/camera_robot_port.py', 'sim/multi_masterpi_production.py',
    'configs/zone_study_integration/pose_providers.json')


def git(*args):
    return subprocess.run(['git', *args], cwd=ROOT, capture_output=True, text=True).stdout.strip()


def jsonl(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(''.join(json.dumps(r, ensure_ascii=False, default=str) + '\n' for r in rows))


# ---------------------------------------------------------------------------
# One robot seen by the study layer

class HostRobotLink:
    """``zi.RobotLink`` over ONE host robot slot: own frames, own executor, own job API."""

    def __init__(self, host, rid):
        self.robot_id, self._host, self._slot = rid, host, host.robots[rid]
        self._frames = collections.deque(maxlen=TAP_FRAMES)
        self.aborts = []
        capture = self._slot.port.capture

        def tapped(camera='robot_cam'):
            obs = capture(camera)
            jpeg = base64.b64decode(obs['image'])
            if zi.hashlib.sha256(jpeg).hexdigest() != obs['sha256'] or obs['robot_id'] != rid:
                raise RuntimeError(f'{rid}: frame tap got a foreign or corrupted frame')
            self._frames.append(zi.OwnFrame(len(self._slot.frames), round(float(obs['sim_time']), 4), jpeg,
                                            obs['sha256']))
            return obs
        self._slot.port.capture = tapped

    def clock(self):
        return float(self._host.world.data.time)

    def frame_at(self, t):
        return next((f for f in reversed(self._frames) if f.t <= t + 1e-9), None)

    def belief(self):
        return self._slot.executor.belief_projection()

    def job(self):
        job = self._slot.executor.job
        return None if job is None else {'kind': job.kind, 'order_id': job.args.get('order_id'),
                                         'job_id': job.job_id}

    def call(self, api, *args):
        """The executor job API of THIS robot. An accepted abort also drops the host's
        scheduled macro commands of this robot and holds at once (#221 P1 at the host layer)."""
        slot, now = self._slot, self.clock()
        slot.executor.now = now
        ack = getattr(slot.executor, api)(*args)
        self._host.api_calls.append(ack)
        if api == 'abort' and ack['accepted']:
            dropped = sum(len(cmds) for _, cmds in slot.timeline)
            slot.timeline, slot.capture_after = [], False
            self._host._hold(self.robot_id, now)
            slot.next_decide = now
            self.aborts.append({'t': round(now, 4), 'job_id': ack['job_id'], 'dropped_macro_commands': dropped})
        return ack


class StudyTeamHost(OwnCamTeamHost):
    """#206's 3-robot host, stepped in chunks by the study clock, pose provider from config."""

    def __init__(self, spec, student, *, root, provider_spec):
        super().__init__(spec, student, root=root, study_layer=self._no_layer, frames_dir=None)
        self.provider_sources = {}
        for rid, slot in self.robots.items():
            ex = slot.executor
            provider = zi.build_pose_provider(provider_spec, ex.map, ex.params, ex.seed)
            for row in slot.commands:                # the own command log so far (initial servo command)
                provider.on_command(row)
            if ex.mode == 'm1':
                ex._require_owncam(provider.source, 'pose provider')
            ex.pose = provider
            self.provider_sources[rid] = provider.source
        self.links = {rid: HostRobotLink(self, rid) for rid in ROBOTS}

    @staticmethod
    def _no_layer(*_args):
        raise AssertionError('StudyTeamHost is stepped by advance_to(), never by run()')

    def settle(self, t0):
        """Settle to the first quantum boundary >= ``t0`` and the post-setup SIM time; one own frame each."""
        q = zi.QUANTUM_S
        start = round(q * math.ceil(max(float(t0), float(self.world.data.time)) / q - 1e-9), 6)
        self._physics_until(start)
        now = float(self.world.data.time)
        for rid, slot in self.robots.items():
            self._capture(rid, now)
            slot.next_decide = now
        return start

    def advance_to(self, t_end):
        """Decisions, macros and physics up to ``t_end``; returns the executors' events."""
        data, events = self.world.data, []
        while True:
            now = float(data.time)
            for rid in ROBOTS:
                slot = self.robots[rid]
                if slot.dead:
                    continue
                if slot.timeline or slot.capture_after:
                    self._run_timeline(rid, now)
                if not slot.timeline and not slot.capture_after and now + 1e-9 >= slot.next_decide:
                    self._decide(rid, now)
            for rid in ROBOTS:
                events.extend(self.robots[rid].executor.drain_events())
            if now >= t_end - 1e-9:
                self.event_log.extend(events)
                return events
            live = [s for s in self.robots.values() if not s.dead]
            nxt = min((s.timeline[0][0] if s.timeline else s.next_decide) for s in live) if live else t_end
            self._physics_until(min(max(nxt, now + float(self.world.model.opt.timestep)), t_end))


# ---------------------------------------------------------------------------
# Bundle, scenario, plan

def load_prereg(path):
    prereg = json.loads(Path(path).read_text())
    if prereg.get('schema') != 'ugrp.zone_study_integration_prereg.v1':
        raise SystemExit(f'{path}: not an integration prereg')
    return prereg


class _StubLink:
    def __init__(self, rid):
        self.robot_id = rid

    def __getattr__(self, name):
        raise AssertionError(f'bundle construction must not touch a robot ({name})')


def run_bundle(prereg, episode):
    """Everything that identifies this execution, hashed (docs/execution_versioning.md)."""
    from harness.zone_study_scenarios import bundle_for, validate
    from sim.zone_landmarks import tagged_map
    scenario = json.loads((ROOT / episode['scenario']).read_text())
    report = validate(scenario)
    if not report.ok:
        raise SystemExit(f'scenario {episode["scenario"]} fails validation: {report.checks}')
    map_bundle = bundle_for(scenario)
    if json.loads(Path(map_bundle['map_file']).read_text()) != tagged_map(episode['map']):
        raise SystemExit('the map file the robots read differs from the tagged map the scene builds')
    provider = zi.pose_provider_spec(prereg['pose_provider'], map_id=episode['map'])
    stub = {r: _StubLink(r) for r in ROBOTS}
    trial = zi.IntegratedTrial(scenario, condition=MAIN_CONDITIONS[0], seed=episode['trial_seed'], links=stub,
                               horizon_s=prereg['horizon_s'], map_bundle=map_bundle)
    bundle = {'execution_bundle_id': zi.EXECUTION_BUNDLE_ID, 'schema': SCHEMA,
              'runtime_files_sha256': {f: zi.file_sha256(ROOT / f) for f in RUNTIME_FILES},
              'scenario': episode['scenario'], 'scenario_sha256': zi.file_sha256(ROOT / episode['scenario']),
              'map_id': episode['map'], 'map_file_sha256': map_bundle['map_file_sha256'],
              'public_map_sha256': map_bundle['public_map_sha256'], 'tagged_map_sha256': digest(tagged_map(episode['map'])),
              'physical': {k: episode[k] for k in ('base_map', 'layout_seed', 'goal', 'extra_boxes',
                                                    'contact_profile', 'job_sim_limit_s')},
              'weld': 'off', 'sync_sim': True, 'frame_period_s': OwnCamTeamHost.FRAME_S,
              'executor_tick_s': zi.QUANTUM_S, 'student': dict(prereg['student']),
              'student_calibration_sha256': zi.file_sha256(ROOT / prereg['student']['calibration']),
              'pose_provider': zi.provider_record(provider),
              'study_invariant': zi.condition_invariant_config(trial.study_config()),
              'conditions': list(MAIN_CONDITIONS), 'horizon_s': prereg['horizon_s']}
    return bundle, scenario, map_bundle, provider


def placements_match(scenario, host):
    """The scenario's declared setup placements are the physical episode's."""
    want = {p['item_id']: [round(v, 4) for v in p['pose_m'][:2]] for p in scenario['eval']['setup']['placements']}
    got = {oid: [round(v, 4) for v in o['position_m'][:2]] for oid, o in host.objects.items()}
    if want != got:
        raise SystemExit(f'scenario placements {want} != physical episode {got}')


# ---------------------------------------------------------------------------
# Evaluation only (after the run, from simulator truth)

def referee_from_gt(gt_rows, static_map, end_s):
    """Eval-only delivery rows: a cyan box resting on the floor inside a zone for SETTLE_S."""
    zones = {z: static_map['regions'][f'zone_{z}'] for z in ('A', 'B', 'C')}

    def zone_of(xyz):
        if xyz[2] >= ON_FLOOR_MAX_Z_M:
            return None
        for z, r in zones.items():
            (cx, cy), (hx, hy) = r['center_m'], r['half_extents_m']
            if abs(xyz[0] - cx) <= hx and abs(xyz[1] - cy) <= hy:
                return z
        return None
    rows, state = [], {}
    for row in gt_rows:
        for box, xyz in row['boxes'].items():
            z = zone_of(xyz)
            cur = state.get(box)
            if cur is None or cur['zone'] != z:
                state[box] = {'zone': z, 'since': row['t'], 'emitted': False}
                continue
            if z is not None and not cur['emitted'] and row['t'] - cur['since'] >= SETTLE_S - 1e-9:
                cur['emitted'] = True
                rows.append({'item_id': box, 'kind': 'cyan', 'zone': z, 'sim_s': round(cur['since'], 3)})
    final = {b: s['zone'] for b, s in state.items()}
    return {'deliveries': rows, 'final_zone': final, 'settle_s': SETTLE_S, 'source': 'eval_only simulator truth',
            'end_sim_s': end_s}


def robot_eval(host, rid, boxes_final):
    """Eval-only per-robot audit (#206 E3/E6 style) against simulator truth."""
    slot = host.robots[rid]
    ex = slot.executor
    summ = ex.summary()
    confirmed = []
    for job in summ['jobs']:
        if job['kind'] == 'deliver' and job['confirmation'] == 'own_camera_confirmed' and job.get('slot_id'):
            s = next(s for zs in host.static['zone_slots'].values() for s in zs if s['slot_id'] == job['slot_id'])
            in_slot = [b for b, (x, y, z) in boxes_final.items() if abs(x - s['center_m'][0]) <= SLOT_HALF_M
                       and abs(y - s['center_m'][1]) <= SLOT_HALF_M and z < ON_FLOOR_MAX_Z_M]
            confirmed.append({'job_id': job['job_id'], 'slot_id': job['slot_id'], 'cyan_in_slot': in_slot,
                              'false_confirmation': not in_slot})
    ev = host.eval_only
    return {'jobs': summ['jobs'], 'events': [e['event'] for e in ex.events], 'pose_sources_seen': summ['pose_sources_seen'],
            'cameras_seen': summ['cameras_seen'], 'counts_as_m1_inputs': summ['counts_as_m1_inputs'],
            'foreign_frames_fed': sum(1 for f in slot.frames if f['robot_id'] != rid or f['camera'] != 'robot_cam'),
            'rejected_foreign_frames': summ['rejected_foreign_frames'], 'frames': len(slot.frames),
            'commands': len(slot.commands), 'exception': slot.exception, 'dead': slot.dead,
            'aborts': host.links[rid].aborts, 'confirmed_deliveries': confirmed,
            'false_confirmations': sum(c['false_confirmation'] for c in confirmed),
            'evaluation_only': {'contact_steps': ev['kind_steps'][rid], 'retention': ev['retention'][rid]}}


# ---------------------------------------------------------------------------
# One trial

def run_trial(prereg, episode, condition, out, *, horizon_s, dev=False):
    import cv2
    import mujoco
    import numpy as np
    bundle, scenario, map_bundle, provider = run_bundle(prereg, episode)
    bundle_sha = digest(bundle)
    if not dev and prereg.get('bundle_sha256') != bundle_sha:
        raise SystemExit(f'run bundle {bundle_sha[:12]} differs from the pre-registered '
                         f'{str(prereg.get("bundle_sha256"))[:12]}; commit, then refresh the prereg')
    run_id = f'{condition}-{episode["episode_id"]}'
    out = Path(out) / run_id
    out.mkdir(parents=True, exist_ok=False)                   # never overwrite a result
    started, load0 = time.time(), os.getloadavg()
    code = {'sha': git('rev-parse', 'HEAD'),
            'dirty': bool(git('status', '--porcelain', '--', 'harness', 'sim', 'scripts', 'configs', 'maps'))}
    label = dict(bundle['pose_provider']['label'])
    (out / 'attempt_started.json').write_text(json.dumps(
        {'run_id': run_id, 'condition': condition, 'episode': episode['episode_id'], 'code': code, 'dev': dev,
         'bundle_sha256': bundle_sha, 'horizon_s': horizon_s, 'pose_provider': label,
         'started_unix': round(started, 3), 'load_average_start': load0}, indent=2, ensure_ascii=False) + '\n')
    spec = {'map': episode['map'], 'seed': episode['layout_seed'], 'goal': episode['goal'],
            'extra_boxes': episode['extra_boxes'], 'contact_profile': episode['contact_profile'],
            'job_sim_limit_s': episode['job_sim_limit_s'], 'order_sheet': None}
    from harness.zone_study_inputs import OrderSheetSource
    spec['order_sheet'] = OrderSheetSource(scenario, map_bundle).sheet()   # the SAME sheet the models get
    host = trial = result = None
    failure, t = None, 0.0
    try:
        host = StudyTeamHost(spec, prereg['student'], root=ROOT, provider_spec=provider)
        placements_match(scenario, host)
        host.assigned_box = {}
        trial = zi.IntegratedTrial(scenario, condition=condition, seed=episode['trial_seed'], links=host.links,
                                   horizon_s=horizon_s, code_sha=code['sha'], map_bundle=map_bundle,
                                   pose_label=label)
        t = host.settle(float(prereg['t0_s']))
        trial.begin(t)
        stop = 'horizon'
        while t < horizon_s - 1e-9:
            t = round(t + zi.QUANTUM_S, 6)
            for event in host.advance_to(t):
                trial.on_executor_event(event, at_s=t)
            trial.step_to(t)
            if all(s.dead for s in host.robots.values()):
                stop = 'all_robots_stopped'
                break
            if trial.quiescent():
                stop = 'quiescent_budget_spent'
                break
        for rid in ROBOTS:
            host._hold(rid, float(host.world.data.time))
        result = trial.finish(t)
    except Exception as exc:                                 # noqa: BLE001 - recorded, then re-raised below
        failure = {'type': type(exc).__name__, 'message': str(exc)[:2000], 'traceback': traceback.format_exc()[-8000:],
                   'sim_s': None if host is None else round(float(host.world.data.time), 3)}
        stop = 'exception'
    finally:
        record = write_outputs(out, prereg, episode, condition, bundle, bundle_sha, host, trial, result, stop,
                               failure, code, started, load0, dev)
        if host is not None:
            host.close()
    env = {'mujoco': mujoco.__version__, 'opencv': cv2.__version__, 'numpy': np.__version__}
    manifest = json.loads((out / 'manifest.json').read_text())
    manifest['env'].update(env)
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False, default=str) + '\n')
    if failure is not None:
        raise SystemExit(f'{run_id}: {failure["type"]}: {failure["message"]}')
    return record


def trial_record_for(trial, result, referee):
    """The package I trial record, with the eval-only referee filled from simulator truth."""
    from harness import zone_study_eval as ev
    record = trial.trial_record(result)
    record['referee'] = {'deliveries': referee['deliveries']}
    state = ev.delivery_state(record)
    if state['orders_complete']:
        done_at = max(d['sim_s'] for d in state['delivered'].values())
        record['end_reason'], record['end_sim_s'] = ev.SUCCESS_END_REASON, done_at
    return record


def write_outputs(out, prereg, episode, condition, bundle, bundle_sha, host, trial, result, stop, failure, code,
                  started, load0, dev):
    """Everything that exists, also after an exception; returns the result summary."""
    summary = {'schema': SCHEMA, 'run_id': out.name, 'condition': condition, 'episode': episode['episode_id'],
               'dev': dev, 'stop': stop, 'failure': failure, 'bundle_sha256': bundle_sha,
               'pose_provider': bundle['pose_provider']['label'], 'plumbing_only': True,
               'note_ko': '배선 스모크(no-LLM fixture). 통신 효과·연구 결과가 아니다. ' + zi.TEMPORARY_NOTE_KO}
    if host is not None:
        summary['sim_s'] = round(float(host.world.data.time), 3)
        ev = host.eval_only
        boxes = {b: [float(v) for v in host.world.data.body(o['body_name']).xpos] for b, o in host.objects.items()
                 if o['kind'] == 'cyan'}
        referee = referee_from_gt(ev['gt'], host.static, summary['sim_s'])
        summary['robots'] = {rid: robot_eval(host, rid, boxes) for rid in ROBOTS}
        summary['eval_only'] = {'referee': referee, 'box_final_xyz': {b: [round(v, 4) for v in p] for b, p in boxes.items()},
                                'weld_max_eq_active': ev['max_eq_active'], 'contact_profile': host.contact_record}
        for rid, slot in host.robots.items():
            base = out / 'robots' / rid
            jsonl(base / 'inputs' / 'commands.jsonl', slot.commands)
            jsonl(base / 'inputs' / 'frames.jsonl', slot.frames)
            jsonl(base / 'executor' / 'events.jsonl', slot.executor.events)
            jsonl(base / 'executor' / 'api.jsonl', slot.executor.api_log)
            jsonl(base / 'executor' / 'judgments.jsonl', slot.executor.judgment_log)
            jsonl(base / 'executor' / 'macros.jsonl', slot.decisions)
            (base / 'executor' / 'job_summaries.json').write_text(
                json.dumps(slot.executor._summaries, indent=1, default=str) + '\n')
        jsonl(out / 'eval_only' / 'gt.jsonl', ev['gt'])
        jsonl(out / 'eval_only' / 'frames_eval.jsonl', ev['frames_eval'])
        jsonl(out / 'eval_only' / 'contacts.jsonl', ev['contacts'])
        (out / 'eval_only' / 'referee.json').write_text(json.dumps(referee, indent=1) + '\n')
        (out / 'scene.xml').write_text(host.world.scene_xml)
        summary['provider_sources'] = dict(host.provider_sources)
    if trial is not None:
        write_study(out, trial, result, summary)
    manifest = {'schema': SCHEMA, 'run_id': out.name, 'bundle': bundle, 'bundle_sha256': bundle_sha, 'code': code,
                'prereg_sha256': zi.file_sha256(ROOT / prereg['_path']) if prereg.get('_path') else None,
                'env': {'python': platform.python_version(), 'platform': platform.platform(),
                        'threads': {k: os.environ.get(k) for k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
                                                                   'VECLIB_MAXIMUM_THREADS', 'MKL_NUM_THREADS')}},
                'load_average': {'start': [round(v, 2) for v in load0], 'end': [round(v, 2) for v in os.getloadavg()]},
                'wall_s': round(time.time() - started, 1), 'pose_provider': bundle['pose_provider']['label']}
    (out / 'result.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False, default=str) + '\n')
    manifest['files'] = {str(q.relative_to(out)): zi.file_sha256(q) for q in sorted(out.rglob('*'))
                         if q.is_file() and q.name not in ('manifest.json',)}
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False, default=str) + '\n')
    return summary


def write_study(out, trial, result, summary):
    """Study-side logs: requests (text + image bytes), calls, messages, actions, dispatch, checks."""
    study = out / 'study'
    images = study / 'request_images'
    images.mkdir(parents=True, exist_ok=True)
    for sha, jpeg in trial.request_images.items():
        (images / f'{sha}.jpg').write_bytes(jpeg)
    jsonl(study / 'dispatch.jsonl', trial.dispatch_log)
    jsonl(study / 'inputs.jsonl', trial.input_log)
    jsonl(study / 'executor_events.jsonl', trial.executor_events)
    jsonl(study / 'scheduler_events.jsonl', trial.scheduler.events)
    (study / 'pair_status.json').write_text(json.dumps(trial.pair_status.record(), indent=1) + '\n')
    (study / 'study_config.json').write_text(json.dumps(trial.study_config(), indent=1, ensure_ascii=False) + '\n')
    if result is None:
        return
    referee = summary.get('eval_only', {}).get('referee', {'deliveries': []})
    record = trial_record_for(trial, result, referee)
    record['pose_provider'] = dict(summary['pose_provider'])   # A's provenance keys are closed: top level
    record['plumbing_only'] = True
    (study / 'trial_record.json').write_text(json.dumps(record, indent=1, ensure_ascii=False) + '\n')
    summary['study'] = {'calls': len(result.calls), 'messages': len(result.messages), 'actions': len(result.actions),
                        'end_reason': record['end_reason'], 'end_sim_s': record['end_sim_s'],
                        'channel': zo.channel_checks(trial, result), 'cost': zo.cost_checks(trial, result),
                        'requests': zo.request_checks(result),
                        'reopen': zo.reopen_trial_record(study / 'trial_record.json'),
                        'clock_drift_s': trial.clock_drift_s, 'wakeups': {r: trial.wakeups(r) for r in ROBOTS},
                        'dispatch': collections.Counter(f'{d["api"]}:{(d["ack"] or {}).get("accepted")}'
                                                        for d in trial.dispatch_log),
                        'pair_status_sha256': trial.pair_status.config_sha256(),
                        'study_config_sha256': digest(trial.study_config())}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--prereg', required=True)
    p.add_argument('--episode', required=True)
    p.add_argument('--condition', choices=MAIN_CONDITIONS)
    p.add_argument('--output')
    p.add_argument('--bundle', action='store_true', help='print the run bundle and its sha256, run nothing')
    p.add_argument('--dev-horizon-s', type=float, help='dev plumbing only: shorter horizon, bundle not enforced')
    args = p.parse_args(argv)
    prereg = load_prereg(args.prereg)
    prereg['_path'] = str(Path(args.prereg).resolve().relative_to(ROOT)) if Path(args.prereg).resolve().is_relative_to(ROOT) else None
    episode = next((e for e in prereg['episodes'] if e['episode_id'] == args.episode), None)
    if episode is None:
        raise SystemExit(f'unknown episode {args.episode!r}')
    if args.bundle:
        bundle = run_bundle(prereg, episode)[0]
        print(json.dumps({'bundle_sha256': digest(bundle), 'bundle': bundle}, indent=1, ensure_ascii=False))
        return 0
    if not args.condition or not args.output:
        raise SystemExit('--condition and --output are required to run')
    dev = args.dev_horizon_s is not None
    horizon = float(args.dev_horizon_s if dev else prereg['horizon_s'])
    rec = run_trial(prereg, episode, args.condition, args.output, horizon_s=horizon, dev=dev)
    study = rec.get('study', {})
    print(json.dumps({'run_id': rec['run_id'], 'stop': rec['stop'], 'sim_s': rec.get('sim_s'),
                      'end_reason': study.get('end_reason'), 'calls': study.get('calls'),
                      'messages': study.get('messages'), 'dispatch': study.get('dispatch'),
                      'deliveries': len(rec.get('eval_only', {}).get('referee', {}).get('deliveries', [])),
                      'pose_provider': rec['pose_provider']}, ensure_ascii=False, default=str), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
