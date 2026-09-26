"""Offline check (evaluation only): the own-RGB box range bias is the camera elevation bias the PF already calibrated.

dev-a1 (memory_v1, s151) confirmed the target box 0.12 m from the truth although its
track sigma was 0.03 m. This script re-runs the robot's own detector
(``harness.zone_color_boxes.detect_own``) on recorded own frames and scores each
detection in the GT base frame (so the pose error is excluded), twice:

* raw: the detector's ``estimated_box_center_base_m`` (nominal commanded-PWM camera);
* corrected: the same detection re-projected with the unloaded camera elevation bias of
  the frozen own-camera PF calibration (``calibration_m1_dev.json``
  ``params.measurement.elevation_bias_rad``, fitted for the tag measurements; nothing is
  fitted here). This is ``harness.owncam_memory.correct_box_detection``.

GT (``eval_only/frames_eval.jsonl``: ``gt`` pose and ``box_xyz``) is used for scoring only.
Episodes: this experiment's dev split (dev-a1) and the M1 experiment's test records
(``outputs/m1-owncam-20260926/test``, a different experiment and map); never this
experiment's test seeds.

  python experiments/2026-09-26-zone-owncam-memory/box_bias_check.py <episode dir> [...] --output box_bias_check.json
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics as st
import sys
from pathlib import Path

import cv2

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))

from harness.owncam_memory import correct_box_detection  # noqa: E402
from harness.zone_color_boxes import OWN_PROFILE_ZONE, detect_own  # noqa: E402

UNLOADED_POSTURES = {740, 1072}              # servo 3 of SEARCH_POSE / LOOK_P20
SETTLED_S = .3                               # = harness.owncam_memory.SETTLED_S (the memory's box-frame filter)
BASE_STILL_S = .3                            # no own base motion command for this long
BINS = ((0., .6), (.6, .8), (.8, 1.), (1., 1.3), (1.3, 1.8), (1.8, 3.5))


def rows(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def pct(v, q):
    v = sorted(v)
    return round(v[min(len(v) - 1, int(q*(len(v) - 1) + .5))], 4) if v else None


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('episodes', nargs='+')
    p.add_argument('--calibration', default=str(ROOT/'experiments/2026-09-26-zone-m1-owncam/calibration_m1_dev.json'))
    p.add_argument('--output', default=str(HERE/'box_bias_check.json'))
    args = p.parse_args(argv)
    cal_path = Path(args.calibration)
    params = json.loads(cal_path.read_text())['params']
    found = {'near': [], 'far_coarse': []}
    per_episode = {}
    for ep in map(Path, args.episodes):
        frames = rows(ep/'inputs'/'frames.jsonl')
        cmds = rows(ep/'inputs'/'commands.jsonl')
        arm_t = sorted(float(c['t']) for c in cmds if c['kind'] in ('arm', 'look'))
        base = sorted((float(c['t']), float(c['t']) + float(c.get('duration_s', 0.) or 0.))
                      for c in cmds if c['kind'] in ('mecanum', 'drive'))
        ev = {r['frame']: r for r in rows(ep/'eval_only'/'frames_eval.jsonl')}
        result = json.loads((ep/'result.json').read_text())
        kind = (result.get('controller') or {}).get('box_kind') or 'cyan'
        n = 0
        for f in frames:
            servo = {int(k): int(v) for k, v in f['commanded_servo'].items()}
            g = ev.get(f['frame'])
            if servo.get(3) not in UNLOADED_POSTURES or not g or not g.get('box_xyz') or f['report'].get('load_state') == 'loaded':
                continue
            gx, gy, gyaw = g['gt']
            bxw, byw = g['box_xyz'][:2]
            c, s = math.cos(gyaw), math.sin(gyaw)
            tx, ty = c*(bxw - gx) + s*(byw - gy), -s*(bxw - gx) + c*(byw - gy)     # GT box in the GT base frame
            t = float(f['t'])
            last_arm = max((a for a in arm_t if a <= t), default=-1e9)
            last_base = max((b[1] for b in base if b[0] <= t), default=-1e9)
            settled = t - last_arm >= SETTLED_S
            still = t - last_base >= BASE_STILL_S
            img = cv2.imread(str(ep/f['file']))                                      # BGR, as detect_own decodes the JPEG
            for d in detect_own(img, servo, profile=OWN_PROFILE_ZONE)['detections']:
                if d['kind'] != kind:
                    continue
                bx, by = d['estimated_box_center_base_m'][:2]
                if math.hypot(bx - tx, by - ty) > .6:          # a different box of the same colour
                    continue
                cx, cy = correct_box_detection((bx, by), servo, params, loaded=False)
                r_true = math.hypot(tx, ty)
                found[d['range_class']].append({'range_m': r_true, 'raw_err_m': math.hypot(bx - tx, by - ty),
                                                'corr_err_m': math.hypot(cx - tx, cy - ty),
                                                'raw_range_bias_m': math.hypot(bx, by) - r_true,
                                                'corr_range_bias_m': math.hypot(cx, cy) - r_true,
                                                'settled': settled, 'base_still': still})
                n += 1
        per_episode[str(ep)] = {'detections': n, 'box_kind': kind,
                                'frames_eval_sha256': hashlib.sha256((ep/'eval_only'/'frames_eval.jsonl').read_bytes()).hexdigest()}
    table = []
    subsets = {'all': lambda r: True, 'arm_settled': lambda r: r['settled'],
               'arm_settled_base_still': lambda r: r['settled'] and r['base_still']}
    for (name, keep), (rc, items) in ((a, b) for a in subsets.items() for b in found.items()):
        for lo, hi in BINS:
            sel = [r for r in items if lo <= r['range_m'] < hi and keep(r)]
            if not sel:
                continue
            table.append({'frames': name, 'range_class': rc, 'range_m': [lo, hi], 'n': len(sel),
                          'raw_err_p50_m': pct([r['raw_err_m'] for r in sel], .5),
                          'raw_err_p90_m': pct([r['raw_err_m'] for r in sel], .9),
                          'raw_range_bias_p50_m': round(st.median(r['raw_range_bias_m'] for r in sel), 4),
                          'corrected_err_p50_m': pct([r['corr_err_m'] for r in sel], .5),
                          'corrected_err_p90_m': pct([r['corr_err_m'] for r in sel], .9),
                          'corrected_range_bias_p50_m': round(st.median(r['corr_range_bias_m'] for r in sel), 4)})
    out = {'schema': 'ugrp.owncam_memory_box_bias_check.v1',
           'purpose': 'offline evaluation of the own-RGB box range bias; GT for scoring only; nothing fitted',
           'calibration': str(cal_path.relative_to(ROOT)) if cal_path.is_relative_to(ROOT) else str(cal_path),
           'calibration_sha256': hashlib.sha256(cal_path.read_bytes()).hexdigest(),
           'elevation_bias_rad': params['measurement'].get('elevation_bias_rad'),
           'episodes': per_episode, 'table': table}
    Path(args.output).write_text(json.dumps(out, indent=1) + '\n')
    for r in table:
        print(f"{r['frames']:22s} {r['range_class']:10s} {r['range_m'][0]:.1f}-{r['range_m'][1]:.1f} m n={r['n']:4d} raw p50/p90 "
              f"{r['raw_err_p50_m']:.3f}/{r['raw_err_p90_m']:.3f} (range bias {r['raw_range_bias_p50_m']:+.3f}) -> corrected "
              f"{r['corrected_err_p50_m']:.3f}/{r['corrected_err_p90_m']:.3f} ({r['corrected_range_bias_p50_m']:+.3f})")


if __name__ == '__main__':
    main()
