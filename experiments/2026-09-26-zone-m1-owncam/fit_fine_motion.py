"""Dev-only fit of the localizer's 'fine' motion profile (skill grasp phase, arm lowered).

Data: M1 dev-a1 m1dev-s91 (dev split only): own issued drive/mecanum commands
issued while the skill phase was 'grasp', and the evaluation-only GT base
trajectory (offline, never a runtime input). Model per command = the
localizer's first-order spin-up lag tau plus its stop coast tau_stop = 0.05 s:
displacement = g*u*(D - tau*(1 - exp(-D/tau)) + tau_stop*(1 - exp(-D/tau))).
Forward: grid over (g, tau). Turn: gain at tau_turn = 0.05 s.
"""
import bisect
import json
import math
import sys
from pathlib import Path

import numpy as np

RUN = Path(sys.argv[1] if len(sys.argv) > 1 else
           '/Users/changmin/projects/ugrp/outputs/m1-owncam-20260926/dev-a1/m1dev-s91')
OUT = Path(__file__).with_name('fine_motion_fit_dev.json')


def main():
    gt = [json.loads(l) for l in open(RUN/'eval_only/gt_trajectory.jsonl')]
    T = [g['t'] for g in gt]
    at = lambda t: gt[min(len(gt) - 1, bisect.bisect_left(T, t))]
    phase = {}
    for e in map(json.loads, open(RUN/'eval_only/frames_eval.jsonl')):
        phase[round(e['t'], 3)] = e.get('skill_phase')
    ft = sorted(phase)
    ph = lambda t: phase[ft[max(0, bisect.bisect_right(ft, t) - 1)]]
    fwd, turn = [], []
    for r in map(json.loads, open(RUN/'inputs/commands.jsonl')):
        if r['kind'] not in ('drive', 'mecanum') or ph(r['t']) != 'grasp':
            continue
        a, b = at(r['t']), at(r['t'] + r['duration_s'] + .35)
        c, s = math.cos(a['yaw']), math.sin(a['yaw'])
        dx, dy = b['x'] - a['x'], b['y'] - a['y']
        if abs(r['forward']) > 1e-3:
            fwd.append((r['forward'], r['duration_s'], c*dx + s*dy))
        if abs(r['turn']) > 1e-3:
            turn.append((r['turn'], r['duration_s'], b['yaw'] - a['yaw']))
    TAU_STOP = .05
    lag = lambda D, tau: D - tau*(1 - math.exp(-D/tau)) + TAU_STOP*(1 - math.exp(-D/tau))
    best = None
    for tau in np.arange(.05, 3.01, .01):
        pred = np.array([u*lag(D, tau) for u, D, _ in fwd])
        obs = np.array([d for *_, d in fwd])
        g = float(pred @ obs/(pred @ pred))
        rms = float(np.sqrt(np.mean((g*pred - obs)**2)))
        if best is None or rms < best[2]:
            best = (float(tau), g, rms)
    tp = np.array([u*lag(D, .05) for u, D, _ in turn]); to = np.array([d for *_, d in turn])
    g_turn = float(tp @ to/(tp @ tp))
    out = {'source_run': str(RUN), 'split': 'dev', 'n_forward': len(fwd), 'n_turn': len(turn),
           'forward': {'tau_s': round(best[0], 3), 'gain': round(best[1], 4), 'rms_m': round(best[2], 5)},
           'turn': {'tau_s': .05, 'gain': round(g_turn, 4),
                    'rms_rad': round(float(np.sqrt(np.mean((g_turn*tp - to)**2))), 5)},
           'tau_stop_s': .05, 'note': 'no lateral (strafe) commands in this run: lateral uses the forward values'}
    OUT.write_text(json.dumps(out, indent=2) + '\n')
    print(json.dumps(out, indent=2))


if __name__ == '__main__':
    main()
