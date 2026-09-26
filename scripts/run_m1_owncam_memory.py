"""M1 own-camera delivery, observation memory ON vs OFF, on the frozen M1 physics runner.

Both conditions run ``scripts.run_m1_owncam.run`` unchanged (same physics owner,
contact profile, frame rate, judge, contract checks and ``eval_only/`` split);
the condition only selects the controller class the runner constructs:

* ``off``        -> ``harness.m1_owncam_delivery.M1OwnCamDelivery`` (frozen M1 student, no memory)
* ``memory_v2``  -> ``harness.m1_owncam_memory.M1OwnCamDeliveryMem`` (the same student + the
  landmark-agnostic memory v2 with the INTERIM tag provider; results are labelled
  "interim, tag provider")

``memory_v1`` (commit ad78ef2) ran only in dev-a1 and is replaced by ``memory_v2``
(prereg amendment A1-A3) before any test episode.

Guards before every episode: thread caps OMP/OPENBLAS/VECLIB/MKL = 1, at least
``MIN_FREE_GIB`` free on the output disk, the registered prereg sha256, and for
``--split test`` a clean tree whose run inputs equal ``frozen_source.json``.
Each episode gets ``memory_runner.json`` next to the runner's own files
(condition, controller class, memory source hashes, disk, load average).
Sync SIM only; weld OFF (the runner asserts no active equality).
"""
from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

SCHEMA = 'ugrp.m1_owncam_memory_run.v2'
CONDITIONS = {'off': ('harness.m1_owncam_delivery', 'M1OwnCamDelivery'),
              'memory_v2': ('harness.m1_owncam_memory', 'M1OwnCamDeliveryMem')}
RESULT_LABELS = {'off': None, 'memory_v2': 'interim, tag provider'}
MEMORY_FILES = ('harness/owncam_memory.py', 'harness/owncam_memory_kf.py', 'harness/owncam_drive_mem.py',
                'harness/owncam_landmarks.py', 'harness/owncam_landmark_tags.py',
                'harness/m1_owncam_memory.py', 'scripts/run_m1_owncam_memory.py')
THREAD_VARS = ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS', 'MKL_NUM_THREADS')
MIN_FREE_GIB = 30.
EXPERIMENT = 'experiments/2026-09-26-zone-owncam-memory'
FROZEN_PATHS = ('harness', 'scripts', 'sim', 'maps', 'configs', EXPERIMENT,
                'experiments/2026-09-26-zone-m1-owncam/calibration_m1_dev.json',
                'experiments/2026-09-26-zone-owncam-loop-v2/calibration_loop_v2.json')


def sha_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git(*args) -> str:
    return subprocess.run(['git', *args], cwd=ROOT, capture_output=True, text=True).stdout.strip()


def free_gib(path: Path) -> float:
    probe = Path(path)
    while not probe.exists():
        probe = probe.parent
    return shutil.disk_usage(probe).free/2**30


def check_threads(env=os.environ) -> dict:
    values = {k: env.get(k) for k in THREAD_VARS}
    bad = {k: v for k, v in values.items() if v != '1'}
    if bad:
        raise SystemExit(f'refused: thread caps must be 1 ({bad})')
    return values


def controller_class(condition: str):
    if condition not in CONDITIONS:
        raise SystemExit(f'unknown condition {condition!r}; one of {sorted(CONDITIONS)}')
    module, name = CONDITIONS[condition]
    return getattr(importlib.import_module(module), name)


def run_episode(spec: dict, out: Path, student: dict, condition: str, *, prereg_sha256: str, freeze=None,
                amendments_sha256: str | None = None):
    """One episode through the frozen M1 runner with the condition's controller class."""
    import harness.m1_owncam_delivery as base
    from scripts import run_m1_owncam as runner

    out = Path(out)
    gib = free_gib(out.parent)
    if gib < MIN_FREE_GIB:
        raise SystemExit(f'refused: {gib:.1f} GiB free on the output disk (< {MIN_FREE_GIB} GiB)')
    threads = check_threads()
    cls = controller_class(condition)
    started, load0 = time.time(), os.getloadavg()
    original = base.M1OwnCamDelivery
    base.M1OwnCamDelivery = cls              # the runner imports the class name at call time
    try:
        result, manifest = runner.run(spec, out, {**student, 'condition': condition})
    finally:
        base.M1OwnCamDelivery = original
    record = {'schema': SCHEMA, 'episode': spec['episode_id'], 'condition': condition,
              'result_label': RESULT_LABELS.get(condition),
              'controller_class': f'{cls.__module__}.{cls.__name__}',
              'controller_schema': result.get('controller', {}).get('schema'),
              'memory_files_sha256': {f: sha_file(ROOT/f) for f in MEMORY_FILES},
              'code_sha': git('rev-parse', 'HEAD'), 'dirty': bool(git('status', '--porcelain', '--', *FROZEN_PATHS)),
              'prereg_sha256': prereg_sha256, 'amendments_sha256': amendments_sha256, 'freeze': freeze,
              'threads': threads,
              'free_gib_before': round(gib, 2), 'free_gib_after': round(free_gib(out), 2),
              'load_average': {'start': [round(v, 2) for v in load0], 'end': [round(v, 2) for v in os.getloadavg()]},
              'wall_s': round(time.time() - started, 1),
              'runner_files_sha256': {name: sha_file(out/name) for name in ('result.json', 'manifest.json')}}
    (out/'memory_runner.json').write_text(json.dumps(record, indent=2) + '\n')
    return result, manifest, record


def check_frozen(frozen_path: Path) -> dict:
    """A test launch needs a clean tree whose run inputs equal the frozen source."""
    if not frozen_path.is_file():
        raise SystemExit(f'test refused: no frozen source file {frozen_path}')
    frozen = json.loads(frozen_path.read_text())
    dirty = git('status', '--porcelain', '--', *FROZEN_PATHS)
    if dirty:
        raise SystemExit(f'test refused: uncommitted changes under {FROZEN_PATHS}:\n{dirty}')
    sha = frozen['source_sha']
    changed = git('diff', '--name-only', sha, 'HEAD', '--', *FROZEN_PATHS).split()
    bad = [f for f in changed if f not in set(frozen.get('records_only_paths', []))]
    if bad:
        raise SystemExit(f'test refused: files changed since frozen source {sha[:9]}: {bad}')
    diff = [f for f, want in frozen['sha256'].items() if sha_file(ROOT/f) != want]
    if diff:
        raise SystemExit(f'test refused: hash mismatch vs frozen_source.json: {diff}')
    return {'frozen_source_sha': sha, 'frozen_file': str(frozen_path.relative_to(ROOT)),
            'frozen_file_sha256': sha_file(frozen_path), 'head': git('rev-parse', 'HEAD')}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--prereg', required=True)
    p.add_argument('--condition', required=True, choices=sorted(CONDITIONS))
    p.add_argument('--output', required=True)
    p.add_argument('--only', default='')
    p.add_argument('--split', choices=('dev', 'test'), default='dev')
    p.add_argument('--frozen', default='', help='frozen_source.json (required for --split test)')
    args = p.parse_args(argv)
    prereg_path = Path(args.prereg).resolve()
    prereg = json.loads(prereg_path.read_text())
    freeze = None
    if args.split == 'test':
        if not args.frozen:
            raise SystemExit('test refused: --frozen <frozen_source.json> is required')
        freeze = check_frozen(Path(args.frozen).resolve())
    only = {s for s in args.only.split(',') if s}
    episodes = [e for e in prereg['episodes'] if e['split'] == args.split]
    unknown = only - {e['episode_id'] for e in episodes}
    if unknown:
        raise SystemExit(f'--only names episodes outside the {args.split} split: {sorted(unknown)}')
    student = dict(prereg['student'])
    for spec in (e for e in episodes if not only or e['episode_id'] in only):
        spec = {**spec, 'contact_profile': student['contact_profile']}
        out = Path(args.output)/args.condition/spec['episode_id']
        amend = prereg_path.parent/'prereg_amendments.json'
        result, manifest, record = run_episode(spec, out, student, args.condition,
                                               prereg_sha256=sha_file(prereg_path), freeze=freeze,
                                               amendments_sha256=sha_file(amend) if amend.is_file() else None)
        print(json.dumps({'episode': spec['episode_id'], 'condition': args.condition,
                          'result_label': RESULT_LABELS.get(args.condition), 'outcome': result['outcome'],
                          'm1_success': result['m1_success'], 'false_success': result['false_success'],
                          'sim_s': result['sim_s'], 'looks': result['looks'], 'commands': result['commands'],
                          'wall_s': manifest['wall_s'], 'load': record['load_average'],
                          'free_gib': record['free_gib_after']}), flush=True)


if __name__ == '__main__':
    main()
