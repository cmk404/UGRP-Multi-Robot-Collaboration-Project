"""Profile/equivalence wrapper for the M2 pair runner in a detached worktree (dev tool of PR #209).

usage: python pair_prof.py <pair_worktree> <none|kernel> <out_dir> <seed> <cprofile 0|1> [--speed-root DIR]

The pair worktree supplies the unchanged runner (``scripts/run_m2_pair.py``) and its ``sim`` package; the speed
checkout (``--speed-root``, default: the checkout containing this file) supplies the injected tools
(``scripts/sim_profile.py`` counters and, in ``kernel`` mode, ``sim/exact_speedups.py``). ``profile.json`` pins
both sides (Codex review of PR #209, item 6): the pair worktree HEAD and dirty paths, every injected file with
the SHA-256 of the exact bytes that were executed plus the speed checkout HEAD and dirty paths, and the pair-side
modules the injected kernel imported (``sim.physics_drive_kernel``, ``sim.masterpi_dynamics_v2``) with their
file hashes. Records written before this change (``outputs/sim-speed-20260926/pair701-*``) carry only
``pair_worktree_head``; see the experiment README.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import resource
import subprocess
import sys
import threading
import time
import types
from pathlib import Path

SCHEMA = 'ugrp.sim_speed.pair_prof.v2'
SPEED_ROOT = Path(__file__).resolve().parents[2]
KERNEL_IMPORTS = ('sim.physics_drive_kernel', 'sim.masterpi_dynamics_v2')


def sha256_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git_state(root: Path, paths: tuple[str, ...] = ()) -> dict:
    def git(*args):
        done = subprocess.run(['git', '-C', str(root), *args], capture_output=True, text=True)
        return done.stdout.strip() if done.returncode == 0 else None
    status = git('status', '--porcelain', '--', *paths)
    return {'root': str(root), 'head': git('rev-parse', 'HEAD'),
            'dirty_paths': None if status is None else [line[3:] for line in status.splitlines()]}


def load(name: str, path: Path, injected: list[dict]) -> types.ModuleType:
    """Execute ``path`` as module ``name`` from bytes read once, and record the hash of exactly those bytes."""
    path = Path(path).resolve()
    source = path.read_bytes()
    module = types.ModuleType(name)
    module.__file__ = str(path)
    sys.modules[name] = module
    exec(compile(source, str(path), 'exec'), module.__dict__)       # noqa: S102 - our own dev tool source
    injected.append({'module': name, 'path': str(path), 'sha256': hashlib.sha256(source).hexdigest(),
                     'bytes': len(source)})
    return module


def module_files(names: tuple[str, ...]) -> dict:
    out = {}
    for name in names:
        module = sys.modules.get(name)
        path = getattr(module, '__file__', None)
        out[name] = None if path is None else {'path': str(Path(path).resolve()), 'sha256': sha256_file(path)}
    return out


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('pair_worktree', type=Path)
    p.add_argument('mode', choices=('none', 'kernel'))
    p.add_argument('out', type=Path)
    p.add_argument('seed')
    p.add_argument('cprofile', choices=('0', '1'))
    p.add_argument('--speed-root', type=Path, default=SPEED_ROOT)
    return p.parse_args(argv)


class Hooks:
    """Trajectory checkpoints on ``mj_step``, render-thread CPU at close and (kernel mode) the injected kernel."""

    def __init__(self, mode: str, speed: Path, injected: list[dict], record: dict) -> None:
        import mujoco
        self.steps, self.checkpoints, self.render_cpu, self.kernel = 0, [], None, None
        orig_step = mujoco.mj_step
        hooks = self

        def mj_step(m, d, *a, **k):
            orig_step(m, d, *a, **k)
            hooks.steps += 1
            if hooks.steps % 2000 == 0:
                hooks.checkpoints.append({'step': hooks.steps, 't': float(d.time), 'sha256': hashlib.sha256(
                    d.qpos.tobytes() + d.qvel.tobytes() + d.act.tobytes()).hexdigest()})

        mujoco.mj_step = mj_step            # before the pair sim package is imported, as the original wrapper
        from sim.multi_masterpi_production import MultiMasterPiProductionV2 as world_cls
        orig_init, orig_close = world_cls.__init__, world_cls.close

        def init(world, *a, **k):
            orig_init(world, *a, **k)
            if mode == 'kernel':
                speedups = sys.modules.get('kiro_exact_speedups') or load('kiro_exact_speedups',
                                                                           speed/'sim/exact_speedups.py', injected)
                hooks.kernel = speedups.install_drive_kernel(world)
                record['pair_modules_used_by_kernel'] = module_files(KERNEL_IMPORTS)

        def close(world):
            if world._render_executor is not None and hooks.render_cpu is None:
                hooks.render_cpu = world._render_executor.submit(time.thread_time).result(timeout=30)
            return orig_close(world)

        world_cls.__init__, world_cls.close = init, close


def write_outputs(out: Path, record: dict, hooks: Hooks, prof) -> None:
    (out/'profile.json').write_text(json.dumps(record, indent=2) + '\n')
    with (out/'qpos_checkpoints.jsonl').open('w') as fh:
        fh.writelines(json.dumps(r) + '\n' for r in hooks.checkpoints)
    if prof:
        import io
        import pstats
        prof.dump_stats(str(out/'profile.pstats'))
        for sort in ('tottime', 'cumulative'):
            buf = io.StringIO()
            pstats.Stats(prof, stream=buf).sort_stats(sort).print_stats(60)
            (out/f'profile_{sort}.txt').write_text(buf.getvalue())
    print(json.dumps({k: record[k] for k in ('mode', 'seed', 'error', 'kernel', 'cpu_s', 'mj_steps')}), flush=True)


def main(argv=None) -> int:
    args = parse_args(argv)
    wt, speed, out = args.pair_worktree.resolve(), args.speed_root.resolve(), args.out
    out.mkdir(parents=True, exist_ok=False)
    injected: list[dict] = []
    pair = git_state(wt)
    record = {'schema': SCHEMA, 'mode': args.mode, 'seed': args.seed, 'cprofile': args.cprofile == '1',
              'pair_worktree': pair, 'pair_worktree_head': pair['head'],
              'speed_source': git_state(speed, ('scripts/sim_profile.py', 'sim/exact_speedups.py',
                                                'experiments/2026-09-26-sim-speed/pair_prof.py')),
              'injected_sources': injected,
              'this_file': {'path': str(Path(__file__).resolve()), 'sha256': sha256_file(Path(__file__))}}
    sys.path.insert(0, str(wt))
    os.chdir(wt)
    prof_tools = load('kiro_sim_profile', speed/'scripts/sim_profile.py', injected)
    sys.path[:] = [x for x in sys.path if os.path.realpath(x) != os.path.realpath(speed)]   # sim_profile adds its root
    sys.path.insert(0, str(wt))
    leaked = sorted(k for k in sys.modules if k in ('harness', 'sim') or k.startswith(('harness.', 'sim.')))
    if leaked:
        raise RuntimeError(f'speed checkout modules leaked into the pair run: {leaked}')
    hooks = Hooks(args.mode, speed, injected, record)
    runner = load('run_m2_pair_under_test', wt/'scripts/run_m2_pair.py', injected)
    sys.argv = ['run_m2_pair.py', '--seed', args.seed, '--output', str(out/'run')]
    load0, c0, t0, m0 = os.getloadavg(), resource.getrusage(resource.RUSAGE_SELF), time.time(), time.thread_time()
    ctr0 = prof_tools.proc_counters()
    prof = None
    if args.cprofile == '1':
        import cProfile
        prof = cProfile.Profile(time.process_time)
        prof.enable()
    err = None
    try:
        runner.main()
    except BaseException as e:  # noqa: BLE001 - recorded in profile.json, exit code 1
        err = f'{type(e).__name__}: {e}'
    finally:
        if prof:
            prof.disable()
        main_s = time.thread_time() - m0
        c1, ctr1 = resource.getrusage(resource.RUSAGE_SELF), prof_tools.proc_counters()
        proc = (c1.ru_utime - c0.ru_utime) + (c1.ru_stime - c0.ru_stime)
        record.update({
            'error': err, 'kernel': hooks.kernel, 'threads_alive': threading.active_count(),
            'cpu_s': {'process': round(proc, 3), 'main_thread': round(main_s, 3),
                      'render_thread': None if hooks.render_cpu is None else round(hooks.render_cpu, 3)},
            'counters': prof_tools.counter_delta(ctr0, ctr1), 'wall_s_informational': round(time.time() - t0, 1),
            'load_average': {'start': [round(v, 2) for v in load0], 'end': [round(v, 2) for v in os.getloadavg()]},
            'mj_steps': hooks.steps,
            'env': {k: os.environ.get(k) for k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
                                                   'VECLIB_MAXIMUM_THREADS', 'MKL_NUM_THREADS')}})
        write_outputs(out, record, hooks, prof)
    return 1 if err else 0


if __name__ == '__main__':
    sys.exit(main())
