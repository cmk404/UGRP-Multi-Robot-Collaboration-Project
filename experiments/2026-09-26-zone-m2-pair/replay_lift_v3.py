"""Offline replay: grasp -> lift co-motion v2 vs v3 on every recorded M2 lift (own frames + eval-only GT label).

For each ``lift_view`` event: the robot's own grasp frame (``grip_view`` time) and lift frame, the v2 and
v3 IoU, and the evaluation-only label at the lift frame (held = beam z > 0.03 m and both of that robot's
fingers > 1 N). Labels are used only here, offline, to score the check; the robots never see them.
Usage: python experiments/2026-09-26-zone-m2-pair/replay_lift_v3.py <raw_root> [<raw_root> ...] [--out f.json]
"""
from __future__ import annotations

import base64
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from harness import owncam_pair_beam_v2 as ob2  # noqa: E402
from harness import owncam_pair_beam as ob  # noqa: E402
from harness import owncam_pair_lift_v3 as lv3  # noqa: E402


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def frame_at(frames, rid, t):
    cand = [f for f in frames if f['robot'] == rid and abs(f['sim_time'] - t) < .06]
    return min(cand, key=lambda f: abs(f['sim_time'] - t)) if cand else None


def replay_run(run):
    events, frames, truth = rows(run / 'events.jsonl'), rows(run / 'inputs.jsonl'), rows(run / 'evaluation-only.jsonl')
    out = []
    for lv in (e for e in events if e['event'] == 'lift_view'):
        rid, t = lv['robot'], lv['sim_time_s']
        gv = [e for e in events if e['robot'] == rid and e['event'] == 'grip_view' and e['sim_time_s'] <= t]
        if not gv:
            continue
        fg, fl = frame_at(frames, rid, gv[-1]['sim_time_s']), frame_at(frames, rid, t)
        if fg is None or fl is None:
            continue
        g = base64.b64encode((run / 'inputs' / fg['file']).read_bytes()).decode()
        li = base64.b64encode((run / 'inputs' / fl['file']).read_bytes()).decode()
        gt = min(truth, key=lambda r: abs(r['t'] - t))
        held = gt['beam_xyz'][2] > .03 and min(gt['finger_n'][rid]) > 1.
        out.append({'run': str(run), 'robot': rid, 't': t, 'grasp_frame': fg['file'], 'lift_frame': fl['file'],
                    'lift_frame_sha256': hashlib.sha256((run / 'inputs' / fl['file']).read_bytes()).hexdigest(),
                    'robot_x_gt': gt['robots'][rid][0], 'held_gt': held,
                    'iou_v2_logged': lv.get('held_iou'),
                    'iou_v2': round(ob.signature_iou(ob2.co_motion_signature(g), ob2.co_motion_signature(li)), 3),
                    'iou_v3': round(lv3.lift_iou(g, li), 3)})
    return out


def main():
    args = sys.argv[1:]
    dest = None
    if '--out' in args:
        dest = Path(args[args.index('--out') + 1])
        args = args[:args.index('--out')]
    runs = sorted({p.parent for a in args for p in Path(a).rglob('events.jsonl') if (p.parent / 'inputs.jsonl').exists()})
    lifts = [x for r in runs for x in replay_run(r)]
    pos = [x for x in lifts if x['held_gt']]
    neg = [x for x in lifts if not x['held_gt']]
    summary = {'profile': lv3.PROFILE, 'limit': lv3.HOLD_MIN_IOU, 'runs': len(runs), 'lifts': len(lifts),
               'held': len(pos), 'not_held': len(neg),
               'v2_false_negatives': sum(x['iou_v2'] < lv3.HOLD_MIN_IOU for x in pos),
               'v3_false_negatives': sum(x['iou_v3'] < lv3.HOLD_MIN_IOU for x in pos),
               'v2_false_positives': sum(x['iou_v2'] >= lv3.HOLD_MIN_IOU for x in neg),
               'v3_false_positives': sum(x['iou_v3'] >= lv3.HOLD_MIN_IOU for x in neg),
               'held_iou_v2_min': min((x['iou_v2'] for x in pos), default=None),
               'held_iou_v3_min': min((x['iou_v3'] for x in pos), default=None),
               'not_held_iou_v3_max': max((x['iou_v3'] for x in neg), default=None),
               'not_held_iou_v2_max': max((x['iou_v2'] for x in neg), default=None)}
    print(json.dumps(summary))
    if dest:
        dest.write_text(json.dumps({'summary': summary, 'lifts': lifts}, indent=1) + '\n')


if __name__ == '__main__':
    main()
