"""Replay saved RGB, static maps and issued-command clock; never read truth."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from harness.rgb_traffic import RGBTrafficRuntime, moving
from harness.traffic_reservations import paths_conflict


def audit(folder):
    config = json.loads((folder/'run.json').read_text())
    maps = json.loads((folder/'actor-maps.json').read_text())
    runtime = RGBTrafficRuntime(maps, coordination=config['coordinated'])
    now = 0.; count = 0; checks = 0
    for line in (folder/'actor-decisions.jsonl').read_text().splitlines():
        saved = json.loads(line)
        frames = {}
        for unit in maps:
            values = []
            for name in ('own', 'top'):
                image = saved['images'][unit][name]
                data = (folder/image['path']).read_bytes()
                if hashlib.sha256(data).hexdigest() != image['sha256']:
                    raise ValueError(f'RGB hash mismatch at {count}: {unit}/{name}')
                values.append(data)
            frames[unit] = tuple(values)
        decision = runtime.step(frames, count, now, missing=saved['missing_reports'], restart=saved['restart'])
        for key, actual in decision.items():
            if actual != saved[key]:
                raise ValueError(f'replay differs at {count}: {key}')
        if config['coordinated']:
            owners = list(decision['reservation_state']['reservations'].values())
            for i, first in enumerate(owners):
                for second in owners[i+1:]:
                    if any(paths_conflict(a, b, 2*runtime.coordinator.envelope_radius)
                           for a in first['routes'] for b in second['routes']):
                        raise ValueError('conflicting simultaneous reservations')
            for unit, action in decision['issued_actions'].items():
                if moving(action) and decision['permissions'][unit]['phase'] != 'GO':
                    raise ValueError('movement without fresh GO')
            if saved['missing_reports'] and any(moving(a) for a in decision['issued_actions'].values()):
                raise ValueError('movement during incomplete report snapshot')
            checks += 1
        now += max(a['duration_s'] for a in decision['issued_actions'].values())
        count += 1
    return {'success': True, 'source_sha': config['source_sha'], 'frames_replayed': count,
        'coordinated_rounds_checked': checks, 'reads_evaluation_truth': False,
        'scope': 'exact RGB/command/report/reservation replay; not a new physics run'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    args = parser.parse_args()
    result = audit(args.folder)
    (args.folder/'input-audit.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__': main()
