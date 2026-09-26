"""Profile/equivalence wrapper for the M2 pair runner in a detached worktree (scratch dev tool, not committed).

usage: python pair_prof.py <pair_worktree> <none|kernel> <out_dir> <seed> <cprofile 0|1>
"""
import hashlib, importlib.util, json, os, resource, sys, threading, time

WT, MODE, OUT, SEED, CPROF = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5] == '1'
SPEED_WT = '/Users/changmin/projects/ugrp-wt/kiro-sim-speed'
sys.path.insert(0, WT)
os.chdir(WT)
os.makedirs(OUT, exist_ok=False)


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


prof_tools = load('kiro_sim_profile', f'{SPEED_WT}/scripts/sim_profile.py')
sys.path[:] = [x for x in sys.path if os.path.realpath(x) != os.path.realpath(SPEED_WT)]   # sim_profile adds its root
sys.path.insert(0, WT)
assert not any(k == 'harness' or k.startswith(('harness.', 'sim.')) or k == 'sim' for k in sys.modules)
import mujoco  # noqa: E402

steps, cps = [0], []
orig_step = mujoco.mj_step


def mj_step(m, d, *a, **k):
    orig_step(m, d, *a, **k)
    steps[0] += 1
    if steps[0] % 2000 == 0:
        cps.append({'step': steps[0], 't': float(d.time),
                    'sha256': hashlib.sha256(d.qpos.tobytes() + d.qvel.tobytes() + d.act.tobytes()).hexdigest()})


mujoco.mj_step = mj_step
from sim.multi_masterpi_production import MultiMasterPiProductionV2  # noqa: E402

render_cpu, kernel_status = [None], [None]
orig_init, orig_close = MultiMasterPiProductionV2.__init__, MultiMasterPiProductionV2.close


def init(self, *a, **k):
    orig_init(self, *a, **k)
    if MODE == 'kernel':
        ex = load('kiro_exact_speedups', f'{SPEED_WT}/sim/exact_speedups.py')
        kernel_status[0] = ex.install_drive_kernel(self)


def close(self):
    if self._render_executor is not None and render_cpu[0] is None:
        render_cpu[0] = self._render_executor.submit(time.thread_time).result(timeout=30)
    return orig_close(self)


MultiMasterPiProductionV2.__init__, MultiMasterPiProductionV2.close = init, close
R = load('run_m2_pair_under_test', f'{WT}/scripts/run_m2_pair.py')
sys.argv = ['run_m2_pair.py', '--seed', SEED, '--output', f'{OUT}/run']
load0, c0, t0, m0, ctr0 = os.getloadavg(), resource.getrusage(resource.RUSAGE_SELF), time.time(), time.thread_time(), \
    prof_tools.proc_counters()
prof = None
if CPROF:
    import cProfile
    prof = cProfile.Profile(time.process_time)
    prof.enable()
err = None
try:
    R.main()
except BaseException as e:  # noqa: BLE001
    err = f'{type(e).__name__}: {e}'
finally:
    if prof:
        prof.disable()
    main_s = time.thread_time() - m0
    c1, ctr1 = resource.getrusage(resource.RUSAGE_SELF), prof_tools.proc_counters()
    proc = (c1.ru_utime - c0.ru_utime) + (c1.ru_stime - c0.ru_stime)
    summary = {'mode': MODE, 'seed': SEED, 'cprofile': CPROF, 'error': err, 'kernel': kernel_status[0],
               'cpu_s': {'process': round(proc, 3), 'main_thread': round(main_s, 3),
                         'render_thread': None if render_cpu[0] is None else round(render_cpu[0], 3)},
               'counters': prof_tools.counter_delta(ctr0, ctr1), 'wall_s_informational': round(time.time() - t0, 1),
               'load_average': {'start': [round(v, 2) for v in load0], 'end': [round(v, 2) for v in os.getloadavg()]},
               'mj_steps': steps[0], 'pair_worktree_head': os.popen(f'git -C {WT} rev-parse HEAD').read().strip(),
               'env': {k: os.environ.get(k) for k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
                                                      'VECLIB_MAXIMUM_THREADS', 'MKL_NUM_THREADS')}}
    json.dump(summary, open(f'{OUT}/profile.json', 'w'), indent=2)
    with open(f'{OUT}/qpos_checkpoints.jsonl', 'w') as fh:
        fh.writelines(json.dumps(r) + '\n' for r in cps)
    if prof:
        import io, pstats
        prof.dump_stats(f'{OUT}/profile.pstats')
        for sort in ('tottime', 'cumulative'):
            buf = io.StringIO()
            pstats.Stats(prof, stream=buf).sort_stats(sort).print_stats(60)
            open(f'{OUT}/profile_{sort}.txt', 'w').write(buf.getvalue())
    print(json.dumps(summary), flush=True)
