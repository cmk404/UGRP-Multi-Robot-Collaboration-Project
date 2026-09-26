"""Collect every dev-split variant (metrics files, configs, notes) into dev_variants.json (record, not a result)."""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = Path('/Users/changmin/projects/ugrp/outputs/vision-loc-20260926')
KEYS = ('n', 'pos_p50_m', 'pos_p90_m', 'pos_p99_m', 'lat_abs_p99_m', 'yaw_p90_deg')
GROUPS = ('all', 'door_loaded', 'loaded', 'unloaded')

VARIANTS = [
    # (id, metrics json, filter, note)
    ('oracle_o1_eff8', 'dev-oracle-grid2/o1_eff8/metrics.json', 'oracle',
     'oracle labels, eff 8 (s910 dev + s903 train; calibration from train s901 only)'),
    ('oracle_o2_eff12', 'dev-oracle-grid2/o2_eff12/metrics.json', 'oracle', 'oracle, eff 12'),
    ('oracle_o4_eff12_disc', 'dev-oracle-grid2/o4_eff12_disc/metrics.json', 'oracle',
     'oracle, eff 12, discriminative-column tempering'),
    ('oracle_o5_eff20_disc', 'dev-oracle-grid2/o5_eff20_disc/metrics.json', 'oracle', 'oracle, eff 20, discriminative'),
    ('vision_320_s910', 'dev-vision-grid-320-aborted/s910_v_eff8_sig2.5.json', 'vision',
     'learned seg at the 320x240 training size: class boundary ~3 px low (1/8 head); aborted after s910'),
    ('w1_480_eff8', 'dev-grid-480/w1_480_eff8/metrics.json', 'vision', 'inference 480x360, no refinement, eff 8'),
    ('w2_480_eff12', 'dev-grid-480/w2_480_eff12/metrics.json', 'vision', 'inference 480x360, no refinement, eff 12'),
    ('w3_480r6_eff8', 'dev-grid-480r6/w3_480r6_eff8/metrics.json', 'vision', '480x360 + class-guided refinement 6 px, eff 8'),
    ('w4_480r6_eff12', 'dev-grid-480r6/w4_480r6_eff12/metrics.json', 'vision', '480x360 + refinement 6 px, eff 12'),
    ('w5_480r6_floor10_eff8', 'dev-s910-w5_480r6_floor10_eff8/metrics.json', 'vision',
     'w3 + floor runs < 10 px dropped (s910 + s911)'),
    ('w6_480r6_cons4_eff8', 'dev-s910-w6_480r6_cons4_eff8/metrics.json', 'vision',
     'w3 + neighbour-consistency filter 4 px (s910 + s911)'),
    ('w7_480r6_floor10_cons4_eff8', 'dev-s910-w7_480r6_floor10_cons4_eff8/metrics.json', 'vision',
     'w3 + floor 10 px + consistency 4 px (s910 + s911)'),
    ('boundary', 'dev-baselines/b_baselines/metrics.json', 'boundary', 'PR #210 detector (wall height 0.40), same PF'),
    ('deadreck', 'dev-baselines/b_baselines/metrics.json', 'deadreck', 'command integration'),
    ('oracle_b', 'dev-baselines/b_baselines/metrics.json', 'oracle', 'oracle, eff 12, train calibration'),
]


def main():
    rows = {}
    for vid, rel, filt, note in VARIANTS:
        path = OUT/rel
        if not path.exists():
            rows[vid] = {'note': note, 'status': 'missing'}
            continue
        m = json.loads(path.read_text())
        pooled = m['pooled'].get(filt, {})
        rows[vid] = {'note': note, 'filter': filt, 'metrics_file': str(path),
                     'metrics_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                     'episodes': list(m['episodes']),
                     'pooled': {g: {k: pooled.get(g, {}).get(k) for k in KEYS} for g in GROUPS},
                     'false_detections': m.get('false_detections', {}).get('pooled')}
    notes = [
        'Segmentation checkpoint seg-v1 (running BN statistics of augmented batches) read walls as floor in eval '
        'mode (dev wall IoU 0.74); retrained as seg-v2 with precise BN (dev wall IoU 0.954). seg-v1 kept, unused.',
        'Motion refit on train (scripts/eval_owncam_localization.fit_motion): dev s910 command integration p90 '
        '1.23 -> 2.12 m (yaw p90 14.2 -> 2.6 deg); rejected, the M1 motion model stays.',
        'Pan-induced chassis yaw (loaded) added to the camera model after the dev s910 oracle showed ~4 deg yaw '
        'offsets after every loaded look sweep (calibration_train.json pan_base_yaw).',
        'Line-of-sight expected rows replaced the floor-trace cast (door-jamb columns off by up to 400 px).',
        'Selection rule (written before looking at w1-w4): smallest pooled dev door_loaded lateral p99, then '
        'door_loaded p90, then all p90. Only dev s910 crosses door_1 (s909 teacher stuck at the box approach, '
        's911 stopped by the keep-out guard), so door metrics rest on 246 frames of one episode.',
        'dev s909: the teacher robot is wedged at its approach point for ~560 SIM s while it keeps issuing turn '
        'commands; every filter (oracle too) integrates the commanded turns and is lost (p90 ~2.5 m). The PF has '
        'no relocalisation; recorded as an open issue.',
        'w1-w4 on s909 were started before the 480x360 r6 observations of s909 existed (w3/w4 s909 rerun, same code).',
    ]
    out = {'schema': 'ugrp.vision_loc.dev_variants.v1', 'split': 'dev (oracle grid also train s903)', 'variants': rows,
           'notes': notes}
    (HERE/'dev_variants.json').write_text(json.dumps(out, indent=1) + '\n')
    for vid, r in rows.items():
        p = r.get('pooled', {})
        print(f"{vid:22s}", {g: (p.get(g, {}).get('pos_p90_m'), p.get(g, {}).get('lat_abs_p99_m'),
                                 p.get(g, {}).get('yaw_p90_deg')) for g in ('all', 'door_loaded')} if p else r.get('status'))


if __name__ == '__main__':
    sys.exit(main())
