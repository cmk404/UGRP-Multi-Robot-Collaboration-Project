#!/usr/bin/env python3
"""Independence check of evaluation episodes against the training (and dev) renders.

Seeds alone do not make episodes independent: round-2 test ``vl-test-s917`` drove
the same trajectory as train ``vl-train-s903`` (same spawn row, slot and cyan
pickup cell; review of PR #227). This tool compares the rendered episodes
themselves (eval-only GT and the own JPEG bytes; it is an audit, never a student
input):

* ``jpeg``: own frames whose JPEG bytes (sha256 of ``frames/*.jpg``) also occur in
  a reference episode;
* ``replay``: frames whose GT base pose is within ``replay_m`` / ``replay_deg`` of a
  reference frame with the same commanded arm/pan pulses (a re-driven trajectory);
* ``aligned``: when two episodes have the same frame count, the largest GT position
  difference between frames of equal index (0.01 mm for s917 vs s903);
* ``near``: share of frames within ``near_m`` / ``near_deg`` of any reference frame
  with the same commanded pulses (informational: every delivery crosses door_1).

``--fail`` exits 3 when any candidate has a JPEG or replay overlap, or an
aligned maximum below ``replay_m``.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

REPLAY_M, REPLAY_DEG = .001, .1
NEAR_M, NEAR_DEG = .02, 2.
DOOR_X, DOOR_Y0, DOOR_Y1 = 2.2, -.45, .55


def read_jsonl(path: Path) -> list[dict]:
    with open(path) as fh:
        return [json.loads(line) for line in fh if line.strip()]


def load_episode(ep_dir: Path) -> dict:
    ep_dir = Path(ep_dir)
    frames = read_jsonl(ep_dir/'inputs'/'frames.jsonl')
    ev = {r['frame']: r for r in read_jsonl(ep_dir/'eval_only'/'frames_eval.jsonl')}
    if len(ev) != len(frames):
        raise SystemExit(f'{ep_dir.name}: {len(frames)} frames but {len(ev)} eval rows')
    gt = np.array([ev[r['frame']]['gt'] for r in frames], float).reshape(-1, 3)
    servo = [tuple(sorted((int(k), int(v)) for k, v in r['commanded_servo'].items())) for r in frames]
    jpeg = [hashlib.sha256((ep_dir/r['file']).read_bytes()).hexdigest() for r in frames]
    door = (np.abs(gt[:, 0] - DOOR_X) < .6) & (gt[:, 1] > DOOR_Y0) & (gt[:, 1] < DOOR_Y1)
    return {'name': ep_dir.name, 'gt': gt, 'servo': servo, 'jpeg': jpeg, 'door': door}


def _buckets(gt: np.ndarray, cell: float) -> dict:
    out: dict = {}
    for i, (x, y) in enumerate(gt[:, :2]):
        out.setdefault((int(math.floor(x/cell)), int(math.floor(y/cell))), []).append(i)
    return out


def pose_overlap(cand: dict, ref: dict, max_m: float, max_deg: float) -> np.ndarray:
    """Bool per candidate frame: some reference frame within max_m and max_deg with the same commanded pulses."""
    cell = max(max_m, 1e-3)
    buckets = _buckets(ref['gt'], cell)
    hit = np.zeros(len(cand['gt']), bool)
    for i, (x, y, yaw) in enumerate(cand['gt']):
        bx, by = int(math.floor(x/cell)), int(math.floor(y/cell))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for j in buckets.get((bx + dx, by + dy), ()):
                    rx, ry, ryaw = ref['gt'][j]
                    if (math.hypot(x - rx, y - ry) <= max_m
                            and abs(math.degrees((yaw - ryaw + math.pi) % (2*math.pi) - math.pi)) <= max_deg
                            and cand['servo'][i] == ref['servo'][j]):
                        hit[i] = True
                        break
                if hit[i]:
                    break
            if hit[i]:
                break
    return hit


def compare(cand: dict, refs: list[dict], *, replay_m=REPLAY_M, replay_deg=REPLAY_DEG, near_m=NEAR_M,
            near_deg=NEAR_DEG) -> dict:
    ref_jpeg: dict = {}
    for r in refs:
        for h in r['jpeg']:
            ref_jpeg.setdefault(h, r['name'])
    jpeg_hit = np.array([h in ref_jpeg for h in cand['jpeg']])
    replay = np.zeros(len(cand['gt']), bool)
    near = np.zeros(len(cand['gt']), bool)
    per_ref = {}
    for r in refs:
        rp = pose_overlap(cand, r, replay_m, replay_deg)
        nr = pose_overlap(cand, r, near_m, near_deg)
        replay |= rp
        near |= nr
        aligned = None
        if len(r['gt']) == len(cand['gt']):
            aligned = float(np.max(np.hypot(*(cand['gt'][:, :2] - r['gt'][:, :2]).T)))
        per_ref[r['name']] = {'jpeg_frames': int(sum(h in set(r['jpeg']) for h in cand['jpeg'])),
                              'replay_frames': int(rp.sum()), 'near_frames': int(nr.sum()),
                              'aligned_max_pos_diff_m': None if aligned is None else round(aligned, 6)}
    aligned_min = min((v['aligned_max_pos_diff_m'] for v in per_ref.values()
                       if v['aligned_max_pos_diff_m'] is not None), default=None)
    fail = bool(jpeg_hit.any() or replay.any() or (aligned_min is not None and aligned_min < replay_m))
    return {'episode': cand['name'], 'frames': int(len(cand['gt'])), 'door_frames': int(cand['door'].sum()),
            'jpeg_overlap_frames': int(jpeg_hit.sum()), 'jpeg_overlap_door_frames': int((jpeg_hit & cand['door']).sum()),
            'replay_frames': int(replay.sum()), 'replay_door_frames': int((replay & cand['door']).sum()),
            'near_share': round(float(near.mean()), 4) if len(near) else None,
            'near_door_share': round(float(near[cand['door']].mean()), 4) if cand['door'].any() else None,
            'aligned_min_max_pos_diff_m': aligned_min, 'per_reference': per_ref, 'independent': not fail}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--render-root', required=True)
    ap.add_argument('--candidates', nargs='+', required=True)
    ap.add_argument('--references', nargs='+', required=True)
    ap.add_argument('--output', required=True)
    ap.add_argument('--fail', action='store_true', help='exit 3 unless every candidate is independent')
    args = ap.parse_args(argv)
    out = Path(args.output)
    if out.exists():
        raise SystemExit(f'refusing to overwrite {out}')
    root = Path(args.render_root)
    refs = [load_episode(root/r) for r in args.references]
    report = {'schema': 'ugrp.vision_loc.overlap.v1', 'references': args.references,
              'thresholds': {'replay_m': REPLAY_M, 'replay_deg': REPLAY_DEG, 'near_m': NEAR_M, 'near_deg': NEAR_DEG,
                             'same_commanded_pulses': True},
              'rule': 'independent iff no JPEG-byte overlap, no replay-pose frame and no aligned trajectory '
                      '(max equal-index GT position difference < replay_m)',
              'episodes': [compare(load_episode(root/c), [r for r in refs if r['name'] != c]) for c in args.candidates]}
    report['all_independent'] = all(e['independent'] for e in report['episodes'])
    out.write_text(json.dumps(report, indent=1) + '\n')
    for e in report['episodes']:
        print(f"{e['episode']}: jpeg {e['jpeg_overlap_frames']} (door {e['jpeg_overlap_door_frames']}), replay "
              f"{e['replay_frames']}, aligned {e['aligned_min_max_pos_diff_m']}, near {e['near_share']} -> "
              f"{'independent' if e['independent'] else 'NOT independent'}")
    if args.fail and not report['all_independent']:
        sys.exit(3)


if __name__ == '__main__':
    main()
