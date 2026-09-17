#!/usr/bin/env python3
"""Reconstruct three-robot input/ACK boundaries and pair execution from evidence."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.audit_camera_goal_transport import audit as audit_pair, require, same
from scripts.audit_camera_grasp_student import _image
from harness.gemini_proxy import _to_gemini_multi_image_messages
from harness.three_robot_plan import (ROBOTS, PAIR, TeamAgreement, build_plan_request,
    build_inspection_request, validate_plan_reply, validate_inspection_reply,
    stage_deliveries, carry_deliveries)


def audit(root):
    root = Path(root).resolve()
    report = json.loads((root/'result.json').read_text())
    for path in ('harness/three_robot_plan.py', 'scripts/three_robot_runtime.py',
                 'scripts/run_three_robot_e2e.py'):
        raw = subprocess.check_output(['git', 'show', f'{report["source_sha"]}:{path}'], cwd=ROOT)
        require(raw == (ROOT/path).read_bytes(), f'team replay source changed: {path}')
    pair = audit_pair(root)
    saved = json.loads((root/'team/team.json').read_text())
    same(saved, report['team'], 'team report and persisted ledger')
    agreement = TeamAgreement(saved['run_id'])
    inbox = {r: [] for r in ROBOTS}
    used_requests = set()
    wire_count = 0

    def check_call(rid, request, validator):
        nonlocal wire_count
        rows = [c for c in saved['calls'] if c['request_id'] == request['request_id']]
        require(len(rows) == 1, 'missing or duplicate request record')
        row = rows[0]
        require(row['robot_id'] == rid, 'wrong client identity')
        used_requests.add(row['request_id'])
        actual = json.loads((root/'team'/row['request']).read_text())
        same(request, actual, 'allowed team request reconstruction')
        if saved['mode'] == 'llm':
            wire_path = root/'team'/rid/f'wire-{row["wire_index"]:03d}.json'
            wire = json.loads(wire_path.read_text())
            same(wire['messages'], _to_gemini_multi_image_messages(request['messages'], request['images']),
                 'exact team model wire')
            wire_count += 1
            if 'reply' in row:
                response = json.loads(wire_path.with_name(wire_path.stem+'-response.json').read_text())
                same(response['choices'][0]['message']['content'].strip(), row['raw_response'], 'team raw response')
                same(response.get('usage'), row['usage'], 'team token usage')
        else:
            require(saved['mode'] == 'fixture' and row['model'] == 'scripted-fixture-not-llm', 'fixture label')
        reply = validator(row['raw_response'], request['request_id']) if 'reply' in row else None
        same(reply, row.get('reply'), 'team reply schema')
        return reply

    trace = json.loads((root/'execution-trace.json').read_text())
    setup_history = {r: [] for r in ROBOTS}
    for row in trace:
        if row['stage'] != 'folded_setup':
            break
        for rid, targets in row['command']['targets'].items():
            setup_history[rid].append({'stage': row['stage'], 'targets': targets,
                                      'duration_s': row['command']['duration_s']})
    for row in saved['rounds']:
        context = agreement.context()
        same(context, row['agreement'], 'current agreement context')
        same(setup_history, row['own_history'], 'only own actual setup commands')
        replies = {}
        for rid in ROBOTS:
            frame = row['images'][rid]
            request = build_plan_request(rid, request_id=f'{saved["run_id"]}-{rid}-plan-{row["turn"]}',
                own_rgb=_image(root, frame['own']), top_rgb=_image(root, frame['top']),
                agreement=context, inbox=inbox[rid], own_history=setup_history[rid])
            replies[rid] = check_call(rid, request, lambda raw, req: validate_plan_reply(raw, req, context))
        same(replies, row['replies'], 'plan decisions')
        same(agreement.receive(replies, row['turn']), row['committed'], 'three-peer commit')
        for sender, reply in replies.items():
            if reply and reply['message']:
                for receiver in ROBOTS:
                    if sender != receiver:
                        inbox[receiver].append({'from_robot': sender, 'turn': row['turn'], 'message': reply['message']})
    # This success audit requires a committed plan; stopped failures retain their
    # raw ledger but must not be promoted to a successful mission by this audit.
    require(agreement.committed is not None, 'no unanimous plan commit')
    same(agreement.committed, saved['committed'], 'immutable execution plan')
    same(agreement.events, saved['agreement_events'], 'agreement transition replay')
    committed_at = saved['rounds'][-1]['sim_time_s']
    first_task = next(row for row in trace if row['stage'] != 'folded_setup')
    require(first_task['start_sim_time_s'] >= committed_at, 'task acted before agreement')
    require(saved['inspections'], 'third robot performed no inspection')
    for row in saved['inspections']:
        frame = row['images']
        request = build_inspection_request(request_id=row['request_id'],
            own_rgb=_image(root, frame['own']), top_rgb=_image(root, frame['top']),
            committed_plan=agreement.committed)
        same(check_call('r2', request, validate_inspection_reply), row['reply'], 'inspection decision')
        require(row['received_at_sim_s'] >= row['observed_at_sim_s'] >= committed_at, 'inspection clock')
    require(len(used_requests) == len(saved['calls']), 'unaccounted team calls')
    fault = report['config']['team_fault']
    require(fault in ('none', 'ready_delay', 'carry_report_loss'), 'unknown injection')
    stage_holds = 0
    if report['llm'] is not None:
        gate = json.loads((root/'llm/gate.json').read_text())
        same(gate['team_plan'], agreement.committed, 'motor clients use the committed plan')
        previous = []
        for event in gate['events']:
            delivered = stage_deliveries(fault, event['skill'], previous)
            same(event['delivered'], list(delivered), 'stage fault schedule')
            if delivered != PAIR:
                require(not event['ready'], 'incomplete readiness authorized motion')
                stage_holds += 1
            previous.append(event)
    carry_holds = 0
    for row in report['carry_calls']:
        delivered = carry_deliveries(fault, row['index'])
        same(row['delivered'], list(delivered), 'carry fault schedule')
        if delivered != PAIR:
            require(row['control']['forwards'] == {'r1': 0., 'r3': 0.}, 'peer moved during report loss')
            require(row['control']['permission']['phase'] == 'HOLD', 'report loss did not HOLD')
            carry_holds += 1
    require(stage_holds == (2 if fault == 'ready_delay' else 0), 'stage injection not exercised')
    require(carry_holds == (5 if fault == 'carry_report_loss' else 0), 'carry injection not exercised')
    return {'ok': True, 'source_sha': report['source_sha'], 'pair': pair,
            'team_wire_requests': wire_count, 'team_mode': saved['mode'],
            'plan_rounds': len(saved['rounds']), 'inspection_reports': len(saved['inspections']),
            'stage_injected_holds': stage_holds, 'carry_injected_holds': carry_holds,
            'scope': 'saved input/ACK/command replay; not OS isolation, real network faults or general planning'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    args = parser.parse_args()
    result = audit(args.run_dir)
    (args.run_dir/'audit.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'pair'}, indent=2))
