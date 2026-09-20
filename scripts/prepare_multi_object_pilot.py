#!/usr/bin/env python3
"""Export 12 logical mission templates and 36 synthetic protocol traces.

No physics, model calls, image interpretation or motor adapter. Commit source
before running; a clean checkout and a new output directory are mandatory.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from harness.multi_object_plan import MissionProtocol, validate_plan
from harness.three_robot_plan import ROBOTS, TeamAgreement, digest
from sim.multi_object_suite import SPEC, fixture_plan, load_pilot


def run_fixture(mission, rotation=0):
    plan = fixture_plan(mission, rotation=rotation)
    agreement = TeamAgreement('fixture', plan_validator=lambda p: validate_plan(p, mission))
    for turn in (0, 1):
        p = agreement.pending
        agreement.receive({r: {'request_id': f'fixture-{r}-plan-{turn}',
            'proposal_id': p['proposal_id'] if p else None, 'plan_hash': p['plan_hash'] if p else None,
            'accept': True, 'plan': plan, 'reason': 'synthetic fixture', 'message': ''} for r in ROBOTS}, turn)
    protocol = MissionProtocol(mission, agreement)
    tick, max_parallel = 0, 0
    while len(protocol.completed) < len(protocol.tasks):
        tick += 1
        if tick > len(protocol.tasks):
            raise ValueError('fixture deadlock or missing task')
        now = float(tick)

        def report(tid, rid):
            return dict(proposal_id=protocol.committed['proposal_id'], plan_hash=protocol.committed['plan_hash'],
                        object_id=protocol.jobs[tid]['object_id'], sequence=protocol.sequences.get(rid, -1)+1,
                        observed_at_s=now, now_s=now, own_rgb_ref=f'fixture-only://{tick}/{rid}/own',
                        top_rgb_ref=f'fixture-only://{tick}/top')

        for tid, task in protocol.tasks.items():
            if tid not in protocol.completed and set(task['after']) <= protocol.completed:
                for rid in task['participants']:
                    protocol.report_ready(tid, rid, **report(tid, rid))
        tickets = protocol.grant_ready(now)
        if not tickets:
            raise ValueError('fixture has no eligible task')
        max_parallel = max(max_parallel, len(tickets))
        for ticket in tickets:
            for rid in protocol.tasks[ticket.task_id]['participants']:
                protocol.report_done(ticket, rid, **report(ticket.task_id, rid))
    return {'summary': {**protocol.summary(), 'rotation': rotation, 'protocol_ticks': tick,
                        'max_concurrent_tasks': max_parallel, 'policy_calls': 0, 'transport_attempts': 0},
            'fixture_plan': plan, 'agreement_events': agreement.events, 'protocol_events': protocol.events}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).strip():
        parser.error('commit source/configuration first; clean checkout required')
    source_sha = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    if args.output.exists():
        parser.error('output already exists; choose a new path')
    spec, cases = load_pilot()
    started = time.monotonic()
    args.output.mkdir(parents=True)

    def write(name, value):
        path = args.output / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')

    manifest, results = [], []
    write('pilot-spec.json', spec)
    for case in cases:
        mission = case['mission']
        write(f"{case['id']}/mission.json", mission)
        manifest.append({'id': case['id'], 'configuration': case['configuration'],
                         'objects': len(mission['objects']), 'tasks': len(mission['tasks']),
                         'mission_sha256': digest(mission), 'physical_admission': 'not_run'})
        for rotation in range(3):
            trace = run_fixture(mission, rotation)
            write(f"{case['id']}/fixture-{rotation}.json", trace)
            results.append({'case_id': case['id'], **trace['summary']})
    write('manifest.json', {'schema': 'ugrp.multi_object_pilot_manifest.v1', 'source_sha': source_sha,
                            'spec_sha256': hashlib.sha256(SPEC.read_bytes()).hexdigest(), 'cases': manifest})
    write('results.json', results)
    summary = {'scope': 'synthetic planning/lease protocol fixtures only', 'source_sha': source_sha,
               'cases': len(cases), 'fixtures': len(results),
               'complete_claim_traces': sum(r['whole_mission_claimed'] for r in results),
               'object_counts': sorted({r['objects'] for r in results}),
               'physical_admission': 'not_run', 'transport_attempts': 0, 'policy_calls': 0,
               'api_cost_usd': 0, 'wall_seconds': time.monotonic()-started}
    write('summary.json', summary)
    write('environment.json', {'python': sys.version, 'platform': platform.platform(),
                               'command': sys.argv, 'source_sha': source_sha})
    write('artifact-hashes.json', {str(p.relative_to(args.output)): hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in sorted(args.output.rglob('*.json'))})
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
