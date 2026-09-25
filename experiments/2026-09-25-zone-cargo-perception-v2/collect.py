#!/usr/bin/env python3
"""Build results.json for this record (top_cargo_v1 vs top_cargo_v2 on the v2 split).

  .venv-sim/bin/python experiments/2026-09-25-zone-cargo-perception-v2/collect.py outputs/zone-cargo-perception-v2-20260925

Reads render manifests, eval labels (case parameters only) and score records.
Raw frames and labels stay local.
"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SHA = '330e856'
RUNS = {'dev': ('dev', f'dev-score-{SHA}'), 'test': (f'test-{SHA}', f'test-score-{SHA}')}
PROFILES = ('top_cargo_v1', 'top_cargo_v2', 'top_zone_v2', 'zone_perception_v1')
CLASSES = ('box_cyan', 'box_green', 'box_red', 'box_yellow', 'can', 'tile', 'long_beam', 'heavy_crate', 'tri_frame')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rate(d):
    return {'detected': d['detected'], 'positives': d['positives'], 'rate': d['rate']}


def targeted(fdir, sdir, profile):
    recs = [json.loads(line) for line in (sdir/f'records-{profile}.jsonl').read_text().splitlines()]
    gt = {(r['view_id'], r['item']): r for r in recs if r['level'] == 'merged' and r['record'] == 'gt'}
    manifest = json.loads((fdir/'manifest.json').read_text())
    pairs = {'edge_gap_lt_3cm': [0, 0], 'edge_gap_3_8cm': [0, 0], 'across_seam': [0, 0], 'same_view': [0, 0]}
    pair_rows, inside, outside = [], [0, 0], [0, 0]
    for view in manifest['views']:
        if 'skipped' in view:
            continue
        lab = json.loads((fdir/'eval-labels'/f"{view['view_id']}.json").read_text())
        cp = lab.get('case_params') or {}
        if 'beams' in cp:
            rows = [gt.get((view['view_id'], b)) for b in cp['beams']]
            ok = all(r and r['detected'] and r['xy_error_m'] < .03 for r in rows)
            for key in ('edge_gap_lt_3cm' if cp['edge_gap_m'] < .03 else 'edge_gap_3_8cm',
                        'across_seam' if cp['across_seam'] else 'same_view'):
                pairs[key][0] += ok
                pairs[key][1] += 1
            pair_rows.append({'view_id': view['view_id'], 'edge_gap_m': round(cp['edge_gap_m'], 4),
                              'along_offset_m': round(cp['along_offset_m'], 3), 'across_seam': cp['across_seam'],
                              'both_found_within_3cm': ok,
                              'xy_error_m': [round(r['xy_error_m'], 4) if r and r['detected'] else None for r in rows]})
        if 'boxes_inside' in cp:
            for b in cp['boxes_inside']:
                inside[0] += bool(gt.get((view['view_id'], b), {}).get('detected'))
                inside[1] += 1
            for b in cp['boxes_outside_bar']:
                outside[0] += bool(gt.get((view['view_id'], b), {}).get('detected'))
                outside[1] += 1
    extra = [{'view_id': r['view_id'], 'outcome': r['outcome'], 'item': r.get('item'), 'background': r.get('background'),
              'confidence': r.get('confidence')}
             for r in recs if r['level'] == 'merged' and r['record'] == 'detection' and r['class'] == 'long_beam'
             and r['outcome'] != 'tp']
    return {'parallel_beam_pairs_both_found_within_3cm': {k: {'ok': v[0], 'n': v[1]} for k, v in pairs.items()},
            'parallel_beam_pairs': pair_rows,
            'boxes_inside_frame_detected': {'ok': inside[0], 'n': inside[1]},
            'boxes_just_outside_a_bar_detected': {'ok': outside[0], 'n': outside[1]},
            'extra_beam_reports': extra}


def compact(res):
    out = {'frames': res['frames'], 'false_positives_total': res['false_positives_total'],
           'false_positives_by_background': res['false_positives_by_background'],
           'box_cargo_confusion': res['box_cargo_confusion'], 'count_error_views': res.get('count_error_views'),
           'per_class': {}}
    for cls in CLASSES:
        row = res['per_class'][cls]
        out['per_class'][cls] = {'full': rate(row['full']), 'partial_frame': rate(row['partial_frame']),
                                 'occluded': rate(row['occluded']), 'xy_error_m_all': row['xy_error_m'],
                                 'yaw_error_deg_all': row['yaw_error_deg'], 'grip_error_m_all': row.get('grip_error_m'),
                                 'duplicates': row['duplicates'], 'confused_as_this_from': row['confused_as_this_from'],
                                 'fp_background': row['fp_background'], 'full_by_case': row.get('full_by_case')}
    return out


def main(root):
    root = Path(root)
    record = json.loads((HERE/'results.json').read_text()) if (HERE/'results.json').exists() else {}
    renders, results, artifacts = {}, {}, {}
    for split, (frames, score) in RUNS.items():
        fdir, sdir = root/frames, root/score
        manifest = json.loads((fdir/'manifest.json').read_text())
        summary = json.loads((sdir/'summary.json').read_text())
        renders[split] = {'render_source_sha': manifest['source_sha'], 'render_source_dirty': manifest['source_dirty'],
                          'score_source_sha': summary['score_source_sha'], 'score_source_dirty': summary['score_source_dirty'],
                          'split_file_sha256_at_render': manifest['split_file_sha256'], 'view_plan': manifest['view_plan'],
                          'catalogue_sha256': manifest['scenes'][0]['catalogue_sha256'],
                          'views': len([v for v in manifest['views'] if 'skipped' not in v]),
                          'skipped_views': len([v for v in manifest['views'] if 'skipped' in v]),
                          'render_wall_s': manifest['wall_s'], 'render_load_average_1min': manifest['load_average_1min'],
                          'score_load_average_1min': summary['load_average_1min']}
        results[split] = {p: {'merged': compact(summary['results'][p]['merged'])} for p in PROFILES}
        for p in ('top_cargo_v1', 'top_cargo_v2'):
            results[split][p]['targeted'] = targeted(fdir, sdir, p)
        artifacts[split] = {'manifest.json': sha(fdir/'manifest.json'), 'summary.json': sha(sdir/'summary.json'),
                            **{q.name: sha(q) for q in sorted(sdir.glob('records-*.jsonl'))}}
    record.update({'renders': renders, 'results': results,
                   'artifacts': {'local_root': str(root), 'backup': 'local only (gitignored outputs/); not a remote backup',
                                 'sha256': artifacts}})
    (HERE/'results.json').write_text(json.dumps(record, indent=1, ensure_ascii=False)+'\n')
    print('written', HERE/'results.json')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else ROOT/'outputs/zone-cargo-perception-v2-20260925')
