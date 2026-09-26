"""B5 offline check (pre-registered, README section 2): relabelling a beam set down off its pickup spot.

Reads the saved TOP JPEGs, labels and static map of PR #169 cohort 1 ``a-two-dynamic-s12`` (the beam was
lowered by hold_lower at 382 SIM s and never re-claimed) and runs ``observe_items`` on the last three
captures with ``top_cargo_v2`` and ``top_cargo_v2_track``. No physics. The referee pose (evaluation only)
is used only to score where the relabelled beam is.

  .venv-sim-worker-mac/bin/python experiments/2026-09-26-zone-teacher-fix/b5_offline.py --output <json>
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path

from harness.zone_perception_v2 import observe_items, public_labels
from sim.zone_arena import top_views

SOURCE = Path('/Users/changmin/projects/ugrp-worktrees/zone-team-a2/outputs/zone-team-a2-20260925/a-two-dynamic-s12')
CAPTURES = ('038-claim-37-0', '039-claim-38-0', '040-final')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--source', type=Path, default=SOURCE)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    setup = json.loads((args.source/'episode-setup-only.json').read_text())
    static = setup['static_map']
    labels0 = json.loads((args.source/'item-labels.json').read_text())
    result = json.loads((args.source/'result.json').read_text())
    beam_gt = result['referee_v2']['items'].get('long_beam_0') or {}
    views = top_views(static)
    out = {'source_run': str(args.source), 'captures': list(CAPTURES), 'profiles': {},
           'files_sha256': {}, 'scope': 'offline robot-input perception only (TOP RGB); referee pose for scoring'}
    for profile in ('top_cargo_v2', 'top_cargo_v2_track'):
        labels = copy.deepcopy(labels0)
        rows = []
        for cap in CAPTURES:
            tops = {}
            for camera, _, _, suffix, _ in views:
                f = args.source/'rgb'/f'{cap}-{suffix}.jpg'
                tops[camera] = f.read_bytes()
                out['files_sha256'][f.name] = hashlib.sha256(tops[camera]).hexdigest()
            view = observe_items(tops, static, labels, profile)
            rows.append({'capture': cap, 'pickup_items_still_visible': view['pickup_items_still_visible'],
                         'moved_labels': view.get('moved_labels')})
        beam = public_labels(labels)['long_beam-1']['rgb_floor_xy_m']
        out['profiles'][profile] = {'observations': rows, 'long_beam-1_rgb_xy_after': beam,
                                    'long_beam-1_claimable_at_end': 'long_beam-1' in rows[-1]['pickup_items_still_visible']}
    out['referee_long_beam_0'] = beam_gt
    if beam_gt.get('pose'):
        track = out['profiles']['top_cargo_v2_track']['long_beam-1_rgb_xy_after']
        out['track_error_vs_referee_m'] = round(math.dist(track, beam_gt['pose'][:2]), 4)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(out, indent=2) + '\n')
    print(json.dumps({k: (v['long_beam-1_claimable_at_end'] if k in ('top_cargo_v2', 'top_cargo_v2_track') else v)
                      for k, v in out['profiles'].items()} | {'err_m': out.get('track_error_vs_referee_m')}))


if __name__ == '__main__':
    main()
