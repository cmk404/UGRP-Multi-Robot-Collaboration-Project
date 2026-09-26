"""CPU-time profiler and equivalence recorder for one sync-SIM episode (dev tool, no behaviour change).

Runs an existing runner unchanged in-process and records:

* CPU time from ``resource.getrusage`` (process user+sys), the main thread
  (``time.thread_time``), the MuJoCo render thread (queried on its own
  executor before ``world.close``) and the rest (OpenCV/BLAS workers);
* optional ``cProfile`` (timer = ``time.process_time``) and optional section
  timers (thread CPU per wrapped call: mj_step, drive arithmetic, port ticks,
  render, remap, JPEG encode/decode, AprilTag, PF, skill, logging);
* optional qpos/qvel/act SHA-256 checkpoints every N ``mj_step`` calls, the
  trajectory fingerprint used by ``scripts/sim_equivalence.py``.

Wrappers only time/hash; they never change arguments, return values or call
order. Wall time is informational (AGENTS.md: wall comparisons need the agent
lock); compare CPU seconds.

M1 example (dev seed, truncated SIM limit)::

    OMP_NUM_THREADS=1 ... python3 scripts/ugrp_session.py run kiro-prof -- \
      .venv-sim-worker-mac/bin/python scripts/sim_profile.py m1 --episode m1dev-s93 \
      --sim-limit 120 --output /Users/changmin/projects/ugrp/outputs/sim-speed-20260926/prof-s93 --sections
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import resource
import sys
import threading
import time
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

SCHEMA = 'ugrp.sim_profile.v1'
M1_PREREG = 'experiments/2026-09-26-zone-m1-owncam/prereg.json'


def rusage_cpu() -> dict:
    r = resource.getrusage(resource.RUSAGE_SELF)
    return {'user_s': r.ru_utime, 'sys_s': r.ru_stime}


class Recorder:
    """Thread-CPU accumulators and trajectory checkpoints (no behaviour change)."""

    def __init__(self, qpos_every: int = 0, cpu_mark_sim_s: float = 0.):
        self.main_ident = threading.get_ident()
        self.cpu_mark_sim_s = float(cpu_mark_sim_s)
        self.cpu_at_mark: dict | None = None
        self.cpu_origin: tuple | None = None
        self.cpu = defaultdict(float)
        self.calls = defaultdict(int)
        self.qpos_every = int(qpos_every)
        self.steps = 0
        self.checkpoints: list[dict] = []
        self.render_thread_cpu_s = None
        self._patched: list[tuple[object, str, object]] = []

    # ------------------------------------------------------------ patching
    def _set(self, owner, name, value):
        self._patched.append((owner, name, getattr(owner, name)))
        setattr(owner, name, value)

    def restore(self):
        for owner, name, value in reversed(self._patched):
            setattr(owner, name, value)
        self._patched.clear()

    def timed(self, owner, name, key):
        orig = getattr(owner, name)
        acc, calls, main = self.cpu, self.calls, self.main_ident
        tt = time.thread_time

        def wrapper(*args, **kwargs):
            t0 = tt()
            try:
                return orig(*args, **kwargs)
            finally:
                k = key if threading.get_ident() == main else key + '@other_thread'
                acc[k] += tt() - t0
                calls[k] += 1
        wrapper.__wrapped__ = orig
        self._set(owner, name, wrapper)

    def install_step_hook(self):
        import mujoco
        orig = mujoco.mj_step
        rec = self

        def mj_step(m, d, *args, **kwargs):
            orig(m, d, *args, **kwargs)
            rec.steps += 1
            if rec.qpos_every and rec.steps % rec.qpos_every == 0:
                h = hashlib.sha256(d.qpos.tobytes() + d.qvel.tobytes() + d.act.tobytes()).hexdigest()
                rec.checkpoints.append({'step': rec.steps, 't': float(d.time), 'sha256': h})
            if (rec.cpu_mark_sim_s and rec.cpu_at_mark is None and rec.cpu_origin is not None
                    and d.time >= rec.cpu_mark_sim_s):
                # CPU used up to a fixed SIM instant, so full and truncated runs compare like for like
                c, m0 = rusage_cpu(), rec.cpu_origin
                rec.cpu_at_mark = {'sim_t': float(d.time), 'step': rec.steps,
                                   'process_s': round(c['user_s'] - m0[0]['user_s'] + c['sys_s'] - m0[0]['sys_s'], 3),
                                   'main_thread_s': round(time.thread_time() - m0[1], 3)}
        mj_step.__wrapped__ = orig
        self._set(mujoco, 'mj_step', mj_step)

    def install_render_thread_probe(self):
        from sim.multi_masterpi_production import MultiMasterPiProductionV2
        orig = MultiMasterPiProductionV2.close
        rec = self

        def close(world):
            ex = getattr(world, '_render_executor', None)
            if ex is not None and rec.render_thread_cpu_s is None:
                try:
                    rec.render_thread_cpu_s = ex.submit(time.thread_time).result(timeout=30)
                except Exception:          # noqa: BLE001 - probe only
                    rec.render_thread_cpu_s = None
            return orig(world)
        self._set(MultiMasterPiProductionV2, 'close', close)

    def install_sections(self):
        import base64

        import cv2
        import mujoco
        from PIL import Image

        from harness import owncam_localizer, wall_tags
        from sim import camera_robot_port
        from sim.multi_masterpi_production import MultiMasterPiProductionV2
        # physics
        self.timed(MultiMasterPiProductionV2, '_physics_step_for', 'physics_step_for_incl_mj_step')
        self.timed(camera_robot_port.CameraRobotPort, 'tick', 'port_tick')
        # rendering / encoding
        self.timed(camera_robot_port.CameraRobotPort, 'capture', 'port_capture_incl_render_wait')
        self.timed(MultiMasterPiProductionV2, '_render_rgb_direct', 'render_rgb_direct')
        self.timed(mujoco.Renderer, 'update_scene', 'renderer_update_scene')
        self.timed(mujoco.Renderer, 'render', 'renderer_render')
        self.timed(cv2, 'remap', 'cv2_remap')
        self.timed(Image.Image, 'save', 'pil_jpeg_encode')
        self.timed(cv2, 'imdecode', 'cv2_imdecode')
        self.timed(base64, 'b64encode', 'b64encode')
        self.timed(base64, 'b64decode', 'b64decode')
        self.timed(Path, 'write_bytes', 'path_write_bytes')
        self.timed(Path, 'write_text', 'path_write_text')
        # perception / estimation
        self.timed(wall_tags.TagDetector, 'detect', 'apriltag_detect_pnp')
        self.timed(owncam_localizer.OwnCamLocalizer, 'update', 'pf_update_incl_predict')
        self.timed(owncam_localizer.OwnCamLocalizer, 'predict_to', 'pf_predict_to')
        self.timed(owncam_localizer.OwnCamLocalizer, 'command', 'pf_command_incl_predict')


def m1_prepare(rec: Recorder, args):
    from harness import m1_owncam_delivery
    from harness import zone_color_boxes
    from scripts import run_m1_owncam as runner
    prereg_path = (ROOT/args.prereg).resolve()
    student, episodes = runner.effective_prereg(prereg_path, json.loads(prereg_path.read_text()))
    spec = next((e for e in episodes if e['episode_id'] == args.episode), None)
    if spec is None or spec['split'] != 'dev':
        raise SystemExit(f'{args.episode!r} is not a dev episode of {args.prereg} (profiling runs dev only)')
    spec = {**spec, 'contact_profile': student.get('contact_profile', spec.get('contact_profile'))}
    if args.sim_limit:
        runner.SIM_LIMIT_S = float(args.sim_limit)    # truncation: behaviour before the limit is unchanged
    if args.sections:
        rec.timed(m1_owncam_delivery.M1OwnCamDelivery, 'on_frame', 'ctl_on_frame')
        rec.timed(m1_owncam_delivery.M1OwnCamDelivery, 'decide', 'ctl_decide')
        rec.timed(m1_owncam_delivery.M1OwnCamDelivery, 'on_command', 'ctl_on_command')
        rec.timed(zone_color_boxes, 'detect_own', 'detect_own')
        rec.timed(runner, 'jsonl', 'runner_jsonl')
        import importlib
        module, name, _kind, _kw = runner.SKILLS[student['skill']]
        cls = getattr(importlib.import_module(module), name)
        rec.timed(cls, 'decide', f'skill_{student["skill"]}_decide')
    kwargs = {}
    if args.speedups is not None:
        kwargs['speedups'] = args.speedups
    return lambda out: runner.run(spec, out, student, **kwargs), {'spec': spec, 'student': student}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('runner', choices=('m1',))
    p.add_argument('--prereg', default=M1_PREREG)
    p.add_argument('--episode', required=True)
    p.add_argument('--sim-limit', type=float, default=0., help='truncate at this SIM time (0 = runner default)')
    p.add_argument('--output', required=True, help='new directory (under the primary outputs/)')
    p.add_argument('--cprofile', action='store_true')
    p.add_argument('--sections', action='store_true')
    p.add_argument('--qpos-every', type=int, default=2000, help='mj_step calls per trajectory checkpoint (0 = off)')
    p.add_argument('--cpu-mark-sim-s', type=float, default=120.,
                   help='also record process/main-thread CPU at the first mj_step with SIM time >= this (0 = off)')
    p.add_argument('--speedups', default=None, help="passed to the runner (e.g. 'none', 'exact-v1')")
    p.add_argument('--cv-threads', type=int, default=None, help='cv2.setNumThreads before the run (default: unchanged)')
    args = p.parse_args(argv)
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    import cv2
    if args.cv_threads is not None:
        cv2.setNumThreads(int(args.cv_threads))
    rec = Recorder(args.qpos_every, args.cpu_mark_sim_s)
    rec.install_step_hook()
    rec.install_render_thread_probe()
    if args.sections:
        rec.install_sections()
    run, meta = m1_prepare(rec, args)
    load0, wall0, cpu0, main0 = os.getloadavg(), time.time(), rusage_cpu(), time.thread_time()
    rec.cpu_origin = (cpu0, main0)
    prof = None
    if args.cprofile:
        import cProfile
        prof = cProfile.Profile(time.process_time)
        prof.enable()
    error = None
    try:
        result, manifest = run(out/'run')
    except BaseException as exc:          # noqa: BLE001 - record then re-raise
        error = f'{type(exc).__name__}: {exc}'
        raise
    finally:
        if prof is not None:
            prof.disable()
        main_s = time.thread_time() - main0
        cpu1, wall1, load1 = rusage_cpu(), time.time(), os.getloadavg()
        rec.restore()
        proc_s = (cpu1['user_s'] - cpu0['user_s']) + (cpu1['sys_s'] - cpu0['sys_s'])
        render_s = rec.render_thread_cpu_s
        summary = {
            'schema': SCHEMA, 'runner': args.runner, 'episode': args.episode, 'sim_limit_s': args.sim_limit or None,
            'speedups': args.speedups, 'cv2_threads': cv2.getNumThreads(), 'error': error,
            'cpu_s': {'process': round(proc_s, 3), 'user': round(cpu1['user_s'] - cpu0['user_s'], 3),
                      'sys': round(cpu1['sys_s'] - cpu0['sys_s'], 3), 'main_thread': round(main_s, 3),
                      'render_thread': None if render_s is None else round(render_s, 3),
                      'other_threads': None if render_s is None else round(proc_s - main_s - render_s, 3)},
            'wall_s_informational': round(wall1 - wall0, 1),
            'load_average': {'start': [round(v, 2) for v in load0], 'end': [round(v, 2) for v in load1]},
            'cpu_at_sim_mark': rec.cpu_at_mark,
            'mj_steps': rec.steps, 'qpos_every': rec.qpos_every, 'checkpoints': len(rec.checkpoints),
            'sections_thread_cpu_s': {k: round(v, 4) for k, v in sorted(rec.cpu.items())},
            'sections_calls': dict(sorted(rec.calls.items())),
            'env': {k: os.environ.get(k) for k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS',
                                                   'MKL_NUM_THREADS')},
            'argv': sys.argv[1:], 'git_head': os.popen(f'git -C {ROOT} rev-parse HEAD').read().strip(),
        }
        try:
            res = json.loads((out/'run'/'result.json').read_text())
            summary.update(sim_s=res.get('sim_s'), frames=res.get('frames'), commands=res.get('commands'),
                           outcome=res.get('outcome'))
        except (OSError, ValueError):
            pass
        (out/'profile.json').write_text(json.dumps(summary, indent=2) + '\n')
        with (out/'qpos_checkpoints.jsonl').open('w') as fh:
            for row in rec.checkpoints:
                fh.write(json.dumps(row) + '\n')
        if prof is not None:
            import io
            import pstats
            prof.dump_stats(str(out/'profile.pstats'))
            for sort in ('tottime', 'cumulative'):
                buf = io.StringIO()
                pstats.Stats(prof, stream=buf).sort_stats(sort).print_stats(70)
                (out/f'profile_{sort}.txt').write_text(buf.getvalue())
        print(json.dumps({k: summary[k] for k in ('episode', 'cpu_s', 'wall_s_informational', 'mj_steps')} |
                         {'sim_s': summary.get('sim_s'), 'outcome': summary.get('outcome')}), flush=True)


if __name__ == '__main__':
    main()
