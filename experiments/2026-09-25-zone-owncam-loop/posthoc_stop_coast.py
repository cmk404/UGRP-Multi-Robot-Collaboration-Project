"""POST HOC (after the test split): loaded motion model error including ~1.2 s after each stop.

The dev refit (fit_motion_rowscale.py) used drive-start..drive-end frames only; the
tau=1.0 s lag then keeps the estimate moving after the wheels stop. Not part of the
pre-registered analysis; the frozen test result stands as reported.
"""
import json, math
from pathlib import Path
import numpy as np
exec(Path(__file__).with_name('fit_motion_rowscale.py').read_text().split('CAL=json.load')[0])
CAL = json.loads(Path(__file__).with_name('calibration_loop.json').read_text())
O = Path('/Users/changmin/projects/ugrp/outputs/owncam-loop-20260925')


def segs_stop(d, after=6):
    cmds = [json.loads(l) for l in open(d/'inputs/commands.jsonl')]
    ev = [e for e in map(json.loads, open(d/'eval_only/frames_eval.jsonl')) if e['phase'] == 'student']
    i = 0
    while i < len(ev):
        if ev[i]['student_state'] != 'drive':
            i += 1
            continue
        j = i
        while j < len(ev) and ev[j]['student_state'] == 'drive':
            j += 1
        k = min(j + after, len(ev) - 1)
        if j - i > 5:
            a, b = ev[i], ev[k]
            g0, g1 = np.array(a['gt']), np.array(b['gt'])
            c, s = math.cos(g0[2]), math.sin(g0[2])
            dx, dy = g1[:2] - g0[:2]
            yield cmds, a['t'], b['t'], np.array([c*dx + s*dy, -s*dx + c*dy])
        i = j


G = np.array(CAL['params']['motion_loaded']['gain'])
tau = CAL['params']['motion_loaded']['tau_s']
for pat in ('dev-a4/dev-box-s3*', 'test/test-box-s4*'):
    for d in sorted(O.glob(pat)):
        S = list(segs_stop(d))
        err = np.array([integrate(c, t0, t1, G, tau)[:2] - g for c, t0, t1, g in S])
        print(d.parent.name, d.name, 'segments', len(S), 'mean fwd err %+.3f m, lat %+.3f m' % tuple(err.mean(0)))
