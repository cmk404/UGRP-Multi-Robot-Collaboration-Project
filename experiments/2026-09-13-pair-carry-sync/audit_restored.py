#!/usr/bin/env python3
"""Audit all packaged runs after restoring them to an independent directory."""
import argparse
import gzip
import json
from pathlib import Path
import sys

EXPERIMENT = Path(__file__).resolve().parent
ROOT = EXPERIMENT.parents[1]
sys.path.insert(0, str(ROOT))
from scripts.audit_pair_carry_sync import audit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-dir', type=Path, required=True)
    parser.add_argument('--grasp-model-dir', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    index = json.loads(gzip.decompress((EXPERIMENT / 'evidence-index.json.gz').read_bytes()))
    transport = ROOT / 'experiments/2026-09-13-rgb-short-transport/models'
    results = {}
    for name in sorted(index['runs']):
        result = audit(args.evidence_dir / name, transport, args.grasp_model_dir)
        results[name] = result
        if not result['success']:
            print(name, result['errors'], flush=True)
    summary = {
        'scope': 'Saved RGB/command/message/referee replay; no new physics simulation',
        'runs': len(results),
        'audits_passed': sum(r['success'] for r in results.values()),
        'physical_successes_recomputed': sum(r['physical_success'] for r in results.values()),
        'results': results,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n')
    print(json.dumps({k: v for k, v in summary.items() if k != 'results'}))
    return int(summary['runs'] != 55 or summary['audits_passed'] != 55)


if __name__ == '__main__':
    raise SystemExit(main())
