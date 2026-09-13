"""Run one unloaded static-map/RGB navigation case, with a separate referee."""
from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import platform
import subprocess
import time
import traceback

import cv2

from harness.known_map_navigation import KnownMapNavigator
from sim.authored_navigation_map import load_map, map_sha256

ROOT = Path(__file__).resolve().parents[1]


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')


def validate_case(case, authored_map):
    if set(case) != {'case_id', 'robot_id', 'start_xy_m', 'start_yaw_deg'}:
        raise ValueError('case requires only case_id, robot_id, start_xy_m, start_yaw_deg')
    if case['robot_id'] not in ('r1', 'r3') or not isinstance(case['case_id'], str):
        raise ValueError('invalid case identity')
    values = [*case['start_xy_m'], case['start_yaw_deg']]
    if len(case['start_xy_m']) != 2 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise ValueError('case setup must contain finite numbers')
    start = authored_map['zones']['start']
    if math.dist(case['start_xy_m'], start['center_m']) > start['radius_m']:
        raise ValueError('private setup must be inside the declared start zone')
    return copy.deepcopy(case)


def run(map_path, case, condition, out, max_steps):
    from scripts.known_map_scene import KnownMapScene
    authored_map = load_map(map_path)
    case = validate_case(case, authored_map)
    if not 1 <= max_steps <= 400:
        raise ValueError('max_steps must be 1..400')
    dirty = subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True)
    if dirty.strip():
        raise RuntimeError('commit experiment source/config before running; working tree is dirty')
    source = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    out.mkdir(parents=True, exist_ok=False)
    (out / 'rgb').mkdir()
    digest = map_sha256(authored_map)
    dump(out / 'actor-map.json', authored_map)
    metadata = {'schema': 'ugrp.known_map_run.v1', 'source_sha': source, 'map_sha256': digest,
        'condition': condition, 'robot_id': case['robot_id'], 'private_setup': case,
        'max_steps': max_steps, 'controller': 'classical RGB localization, visual actuator calibration, static-map A*',
        'actor_inputs': ['authored static map with fixed camera calibration', 'own RGB JPEG', 'common top RGB JPEG', 'frame index', 'own issued command history'],
        'own_camera_usage': 'decoded and preserved; current localization and control use top RGB only',
        'prohibited_runtime_inputs': ['simulator pose', 'joint measurements', 'contacts', 'referee evaluation'],
        'environment': {'python': platform.python_version(), 'platform': platform.platform(),
            'opencv_imported': cv2.__version__,
            **{p: importlib.metadata.version(p) for p in ('numpy', 'mujoco')}},
        'model_calls': 0, 'model_cost_usd': 0, 'scope': 'one unloaded active robot per run; fixed cameras; weld OFF'}
    dump(out / 'run.json', metadata)
    scene = KnownMapScene(out, authored_map, case['robot_id'], case)
    actor = KnownMapNavigator(copy.deepcopy(authored_map), case['robot_id'], condition)
    started = time.monotonic()
    status, decisions, history, error = 'budget_exhausted', 0, [], None
    evaluation = None
    try:
        scene.open()
        if scene.initial_invariants['map_sha256'] != digest:
            raise ValueError('actor map and physics map hash mismatch')
        dump(out / 'invariants-before.json', scene.initial_invariants)
        with (out / 'actor-decisions.jsonl').open('w') as ledger:
            for index in range(max_steps):
                own, top, image_records = scene.capture(index)
                decision = actor.decide(own, top, index)
                # The actor receives none of the private setup or referee data.
                row = {'frame_id': index, 'map_sha256': digest, 'images': image_records,
                       'issued_history': copy.deepcopy(history), 'decision': decision}
                ledger.write(json.dumps(row) + '\n'); ledger.flush()
                scene.last_status = decision['status']
                scene.execute(decision['action'])
                history.append(decision['action']); decisions += 1
                if decision['done']:
                    status = decision['status']; break
            else:
                status = 'budget_exhausted'
        evaluation = scene.evaluate(status)
        dump(out / 'invariants-after.json', scene.invariants())
    except Exception:
        error = traceback.format_exc()
        status = 'error'
    finally:
        try:
            scene.close()
        except Exception:
            error = (error or '') + '\ncleanup: ' + traceback.format_exc()
            status = 'error'
    result = {'source_sha': source, 'map_sha256': digest, 'case_id': case['case_id'],
        'condition': condition, 'actor_status': status, 'decisions': decisions,
        'wall_elapsed_s': time.monotonic() - started, 'evaluation': evaluation, 'error': error,
        'model_calls': 0, 'model_cost_usd': 0}
    dump(out / 'result.json', result)
    files = {str(p.relative_to(out)): hashlib.sha256(p.read_bytes()).hexdigest()
             for p in sorted(out.rglob('*')) if p.is_file()}
    dump(out / 'manifest.json', files)
    print(json.dumps(result), flush=True)
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map-file', type=Path, required=True)
    p.add_argument('--case-json', required=True)
    p.add_argument('--condition', choices=['map', 'direct'], default='map')
    p.add_argument('--out-dir', type=Path, required=True)
    p.add_argument('--max-steps', type=int, default=240)
    a = p.parse_args()
    result = run(a.map_file, json.loads(a.case_json), a.condition, a.out_dir, a.max_steps)
    return 1 if result['error'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
