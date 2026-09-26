"""Collect the probe's test metrics, run metadata and raw-file hashes into results/.

Inputs: the ``score`` output (``--metrics``), the ``localize`` output directory
(``--estimates``: per-episode ``*.estimates.jsonl`` and ``*.meta.json``) and the
wall-height visibility output (``--visibility``). Writes ``results/results.json``
and ``results/raw_index.json`` and prints the markdown tables used in README.md.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
FILTERS = [('boundary', '벽 경계(태그 없음)'), ('tag_online', '태그 PF(실행 중 기록)'),
           ('tag_replay', '태그 PF(같은 재생기)'), ('deadreck', '명령 적분(측정 없음)')]
GROUPS = [('all', '전체'), ('search_approach', '탐색·접근'), ('door_zone', '문 근처'), ('carry', '운반'),
          ('manipulate', '파지·놓기'), ('look_back', '되보기')]


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def cm(v):
    return '–' if v is None else f'{100*v:.1f}'


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--metrics', type=Path, required=True)
    ap.add_argument('--estimates', type=Path, required=True)
    ap.add_argument('--visibility', type=Path)
    ap.add_argument('--out', type=Path, default=HERE/'results')
    args = ap.parse_args(argv)
    metrics = json.loads(args.metrics.read_text())
    args.out.mkdir(exist_ok=True)
    metas = {}
    raw = {}
    for p in sorted(args.estimates.glob('*')):
        raw[p.name] = {'bytes': p.stat().st_size, 'sha256': sha(p)}
        if p.name.endswith('.meta.json'):
            m = json.loads(p.read_text())
            metas[m['episode']] = {k: m[k] for k in ('frames', 'servo_mismatch_frames', 'filters', 'seed', 'dock',
                                                    'wall_s', 'load_average', 'stats', 'probe_calibration',
                                                    'probe_module_sha256', 'runner_sha256')}
    results = {'schema': 'ugrp.markerless_probe.results.v1', 'metrics_source': str(args.metrics),
               'metrics_sha256': sha(args.metrics), 'estimates_dir': str(args.estimates),
               'pooled': metrics['pooled'], 'episodes': metrics['episodes'], 'runs': metas}
    if args.visibility:
        results['wall_height_visibility'] = json.loads(args.visibility.read_text())
    (args.out/'results.json').write_text(json.dumps(results, indent=1, ensure_ascii=False) + '\n')
    (args.out/'raw_index.json').write_text(json.dumps({'root': str(args.estimates), 'files': raw}, indent=1) + '\n')

    pooled = metrics['pooled']
    print('| 필터 | 프레임 | p50 (cm) | p90 (cm) | p99 (cm) | 최대 (cm) | 5 cm 이내 | yaw p90 (°) |')
    print('|---|---:|---:|---:|---:|---:|---:|---:|')
    for f, name in FILTERS:
        if f not in pooled:
            continue
        a = pooled[f]['all']
        print(f"| {name} | {a['n']} | {cm(a['pos_p50_m'])} | {cm(a['pos_p90_m'])} | {cm(a['pos_p99_m'])} | "
              f"{cm(a['pos_max_m'])} | {100*a['share_pos_lt_5cm']:.0f}% | {a['yaw_p90_deg']:.1f} |")
    print()
    print('| 구간 (p50 / p90, cm) | ' + ' | '.join(n for f, n in FILTERS if f in pooled) + ' |')
    print('|---|' + '---|'*sum(f in pooled for f, _ in FILTERS))
    for g, gname in GROUPS[1:]:
        cells = []
        for f, _ in FILTERS:
            if f not in pooled:
                continue
            s = pooled[f].get(g)
            cells.append('–' if not s else f"{cm(s['pos_p50_m'])} / {cm(s['pos_p90_m'])} (n={s['n']})")
        print(f'| {gname} | ' + ' | '.join(cells) + ' |')
    print()
    print('| 에피소드 | ' + ' | '.join(n for f, n in FILTERS if f in pooled) + ' |')
    print('|---|' + '---|'*sum(f in pooled for f, _ in FILTERS))
    for ep, per in metrics['episodes'].items():
        cells = []
        for f, _ in FILTERS:
            if f in pooled:
                a = per.get(f, {}).get('all')
                cells.append('–' if not a else f"{cm(a['pos_p50_m'])} / {cm(a['pos_p90_m'])} / {cm(a['pos_max_m'])}")
        print(f'| {ep.split("/")[-1]} | ' + ' | '.join(cells) + ' |')
    if args.visibility:
        vis = json.loads(args.visibility.read_text())['heights']
        print()
        print('| 벽 높이 | 구간 | 바닥선 보이는 열(평균/48) | 윗모서리 보이는 열 | 윗모서리만 | 바닥선 ≥4열 프레임 | 보이는 모서리 없음 |')
        print('|---|---|---:|---:|---:|---:|---:|')
        for h, groups in vis.items():
            for g, gname in GROUPS:
                s = groups.get(g)
                if s:
                    print(f"| {h} m | {gname} | {s['mean_bottom_cols']} | {s['mean_top_cols']} | {s['mean_top_only_cols']} | "
                          f"{100*s['share_ge4_bottom']:.0f}% | {100*s['share_no_edge']:.0f}% |")


if __name__ == '__main__':
    main()
