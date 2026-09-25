#!/usr/bin/env python3
"""Build results.json (and media/) for this record from the local output root.

  .venv-sim/bin/python experiments/2026-09-25-zone-cargo-perception/collect.py outputs/zone-cargo-perception-20260925

Reads the dev/test render manifests and score summaries (records-*.jsonl for the
confidence and occlusion breakdowns). Raw frames and labels stay local.
"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RUNS = {'dev': ('dev-r1', 'dev-score-07a958d'), 'test': ('test-07a958d', 'test-score-07a958d')}
CLASSES = ('box_cyan', 'box_green', 'box_red', 'box_yellow', 'can', 'tile', 'long_beam', 'heavy_crate', 'tri_frame')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rate(d):
    return {'detected': d['detected'], 'positives': d['positives'], 'rate': d['rate']}


def compact(level_result, level):
    out = {'frames': level_result['frames'], 'false_positives_total': level_result['false_positives_total'],
           'false_positives_by_background': level_result['false_positives_by_background'],
           'false_positives_per_frame': level_result['false_positives_per_frame'],
           'box_cargo_confusion': level_result['box_cargo_confusion'],
           'confusion_matrix': level_result['confusion_matrix'], 'per_class': {}}
    for cls in CLASSES:
        row = level_result['per_class'][cls]
        keep = {'full': rate(row['full']), 'partial_frame': rate(row['partial_frame']),
                'occluded': rate(row['occluded']), 'xy_error_m_full': row['xy_error_m_full'],
                'xy_error_m_all': row['xy_error_m'], 'yaw_error_deg_full': row['yaw_error_deg_full'],
                'yaw_error_deg_all': row['yaw_error_deg'], 'duplicates': row['duplicates'],
                'confused_as_this_from': row['confused_as_this_from'], 'fp_background': row['fp_background']}
        if level == 'merged':
            keep.update({'grip_error_m_full': row.get('grip_error_m_full'), 'grip_error_m_all': row.get('grip_error_m'),
                         'full_by_case': row.get('full_by_case'), 'centre_modes': row.get('centre_modes')})
        out['per_class'][cls] = keep
    return out


def extras(score_dir, frames_dir):
    """Confidence bins (merged, top_cargo_v1) and cargo visible fractions per case."""
    recs = [json.loads(line) for line in (score_dir/'records-top_cargo_v1.jsonl').read_text().splitlines()]
    dets = [r for r in recs if r['level'] == 'merged' and r['record'] == 'detection' and not r['class'].startswith('box_')]
    bins = {}
    for lo, hi in ((0, .5), (.5, .8), (.8, 1.01)):
        sel = [r for r in dets if r.get('confidence') is not None and lo <= r['confidence'] < hi]
        bins[f'{lo}-{min(hi, 1.0)}'] = {'detections': len(sel), 'tp': sum(r['outcome'] == 'tp' for r in sel)}
    manifest = json.loads((frames_dir/'manifest.json').read_text())
    vis = {}
    for view in manifest['views']:
        if 'skipped' in view:
            continue
        lab = json.loads((frames_dir/'eval-labels'/f"{view['view_id']}.json").read_text())
        for oid, t in lab['items'].items():
            if t['kind'] == 'box':
                continue
            best = max((c['items'][oid]['visible_fraction'] for c in lab['cameras'].values()
                        if c['items'][oid]['visible_px'] >= 20 and not c['items'][oid]['partial_frame']), default=None)
            if best is not None:
                vis.setdefault(view['case'], []).append(best)
    occl = {case: {'n': len(v), 'min_best_view_visible_fraction': round(min(v), 3),
                   'below_0.9': sum(x < .9 for x in v)} for case, v in sorted(vis.items())}
    return {'merged_cargo_confidence_bins': bins, 'cargo_best_view_visible_fraction_by_case': occl}


def main(root):
    root = Path(root)
    record = json.loads((HERE/'results.json').read_text()) if (HERE/'results.json').exists() else {}
    results, artifacts, renders = {}, {}, {}
    for split, (frames, score) in RUNS.items():
        fdir, sdir = root/frames, root/score
        if not (sdir/'summary.json').exists():
            continue
        manifest = json.loads((fdir/'manifest.json').read_text())
        summary = json.loads((sdir/'summary.json').read_text())
        renders[split] = {'frames_dir': str(fdir.relative_to(ROOT)) if fdir.is_relative_to(ROOT) else str(fdir),
                          'render_source_sha': manifest['source_sha'], 'render_source_dirty': manifest['source_dirty'],
                          'score_source_sha': summary['score_source_sha'], 'score_source_dirty': summary['score_source_dirty'],
                          'split_file_sha256': manifest['split_file_sha256'],
                          'catalogue_sha256': manifest['scenes'][0]['catalogue_sha256'],
                          'views': len([v for v in manifest['views'] if 'skipped' not in v]),
                          'skipped_views': len([v for v in manifest['views'] if 'skipped' in v]),
                          'render_wall_s': manifest['wall_s'], 'render_load_average_1min': manifest['load_average_1min'],
                          'score_load_average_1min': summary['load_average_1min'],
                          'scene_xml_sha256': sorted({s['scene_xml_sha256'] for s in manifest['scenes']})}
        results[split] = {prof: {lvl: compact(res[lvl], lvl) for lvl in ('merged', 'camera')}
                          for prof, res in summary['results'].items()}
        results[split]['top_cargo_v1_extras'] = extras(sdir, fdir)
        artifacts[split] = {'manifest.json': sha(fdir/'manifest.json'), 'summary.json': sha(sdir/'summary.json'),
                            **{p.name: sha(p) for p in sorted(sdir.glob('records-*.jsonl'))}}
    record.update({'renders': renders, 'results': results,
                   'artifacts': {'local_root': str(root), 'backup': 'local only (gitignored outputs/); not a remote backup',
                                 'sha256': artifacts}})
    (HERE/'results.json').write_text(json.dumps(record, indent=1, ensure_ascii=False)+'\n')
    print('written', HERE/'results.json')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else ROOT/'outputs/zone-cargo-perception-20260925')
