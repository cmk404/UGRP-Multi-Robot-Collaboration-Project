"""POST HOC (written after the single test scoring; not pre-registered): door-plane errors and failure types.

1. Door plane: frames whose GT x is within 0.15 m of door_1 (x = 2.2) while loaded, i.e. the chassis is
   inside or at the mouth of the opening. The pre-registered door zone reaches 0.6 m before the door,
   where the M1 controller still has its 0.6 m checkpoint look ahead of it.
2. Wedged-robot episodes: GT chassis displacement over the last 300 SIM s < 0.05 m while own wheel
   commands keep being issued (the teacher is stuck); every filter integrates the commands and is lost.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import vision_loc as vl  # noqa: E402
import vision_loc_cli as cli  # noqa: E402

EST = cli.PRIMARY_OUT/'test'/'estimates'
FILTERS = ('vision', 'oracle', 'boundary', 'deadreck')


def pct(a, q):
    return None if len(a) == 0 else round(float(np.percentile(a, q)), 4)


def main():
    tab = cli.episodes_table()['episodes']
    eps = [e['episode_id'] for e in tab if e['split'] == 'test']
    out = {'schema': 'ugrp.vision_loc.posthoc.v1', 'post_hoc': True, 'door_plane': {}, 'wedged': {}}
    pooled = {f: [] for f in FILTERS}
    for ep in eps:
        est = vl.read_jsonl(EST/f'{ep}.estimates.jsonl')
        ev = {r['frame']: r for r in vl.read_jsonl(cli.RENDER_ROOT/ep/'eval_only'/'frames_eval.jsonl')}
        per = {}
        for f in FILTERS:
            lat = []
            for r in est:
                gt = ev[r['frame']]['gt']
                if r['loaded'] and abs(gt[0] - cli.DOOR_X) < .15 and -.45 < gt[1] < .55 and r.get(f):
                    lat.append(abs(r[f]['xyyaw'][1] - gt[1]))
            pooled[f] += lat
            per[f] = {'n': len(lat), 'lat_abs_p90_m': pct(lat, 90), 'lat_abs_max_m': None if not lat else round(max(lat), 4)}
        out['door_plane'][ep] = per
        gt = vl.read_jsonl(cli.RENDER_ROOT/ep/'eval_only'/'gt_trajectory.jsonl')
        cmds = vl.read_jsonl(cli.RENDER_ROOT/ep/'inputs'/'commands.jsonl')
        t_end = gt[-1]['t']
        tail = [g for g in gt if g['t'] >= t_end - 300.]
        disp = max(math.hypot(g['x'] - tail[0]['x'], g['y'] - tail[0]['y']) for g in tail)
        wheel = sum(1 for c in cmds if c['t'] >= t_end - 300. and c['kind'] in ('mecanum', 'drive')
                    and any(abs(float(c.get(k, 0.))) > 1e-6 for k in ('forward', 'left', 'turn')))
        out['wedged'][ep] = {'gt_max_displacement_last_300s_m': round(disp, 4), 'own_wheel_commands_last_300s': wheel,
                             'wedged': bool(disp < .05 and wheel > 50)}
    out['door_plane']['pooled'] = {f: {'n': len(v), 'lat_abs_p90_m': pct(v, 90), 'lat_abs_p99_m': pct(v, 99),
                                       'lat_abs_max_m': None if not v else round(max(v), 4)} for f, v in pooled.items()}
    (HERE/'results'/'posthoc_door_plane.json').write_text(json.dumps(out, indent=1) + '\n')
    print(json.dumps(out['door_plane']['pooled'], indent=1))
    print(json.dumps(out['wedged'], indent=1))


if __name__ == '__main__':
    main()
