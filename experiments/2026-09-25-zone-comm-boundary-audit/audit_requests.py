"""R1 audit of the saved ZC2 zone-benchmark model requests (read-only, no SIM).

Reads the raw run directories (default: the primary checkout's
outputs/zone-communication-20260925/ZC2-*), and for every model call:
  * checks that the wire body actually sent equals the saved request
    (system text, user JSON text, image labels and image bytes);
  * records the top-level input fields per condition;
  * measures the non-message channels found by the audit (box_taken_by_peer
    stops, slot ids that encode peer jobs, global turn counters in request_id,
    shared board / peer messages).
Prints one JSON document; nothing is written into the run directories.
"""
from __future__ import annotations

import argparse
import glob
import hashlib
import json
import os
import re
from pathlib import Path

DEFAULT_ROOT = Path('/Users/changmin/projects/ugrp/outputs/zone-communication-20260925')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def wire_matches(run, rid, decision):
    """Wire body == saved request (system, user text, image labels + bytes)."""
    req = json.loads((run/'team'/decision['request']).read_text())
    wire = json.loads((run/'team'/rid/f"wire-{decision['wire_index']:03d}.json").read_text())
    sys_ok = wire['messages'][0]['content'] == req['messages'][0]['content']
    parts = wire['messages'][1]['content']
    texts = [p['text'] for p in parts if p['type'] == 'text']
    urls = [p['image_url']['url'] for p in parts if p['type'] == 'image_url']
    user_ok = texts[0] == req['messages'][1]['content']
    labels_ok = texts[1:] == [i['label'] for i in req['images']]
    images_ok = urls == [i['image'] for i in req['images']]
    return {'system': sys_ok, 'user_text': user_ok, 'image_labels': labels_ok, 'image_bytes': images_ok,
            'max_tokens': wire.get('max_tokens'), 'temperature': wire.get('temperature'),
            'model': wire.get('model')}


def phase_at(events, rid, t):
    """Teacher phase of rid at SIM time t (from logged phase transitions)."""
    phase = 'idle'
    for e in events:
        if e['robot_id'] != rid or e['sim_time_s'] > t + 1e-6:
            continue
        if e['event'] == 'assign':
            phase = 'to_box'
        elif e['event'] == 'phase':
            phase = e['phase']
    return phase


def analyse_run(run):
    result = json.loads((run/'result.json').read_text())
    events = json.loads((run/'teacher-events.json').read_text())
    cond = result['coordination']
    out = {'run': run.name, 'coordination': cond, 'seed': result['config']['seed'],
           'inject': result['config']['inject_grasp_failure'], 'source_sha': result['source_sha'],
           'referee_goal_met': result['referee']['goal_met'], 'calls': 0,
           'wire_mismatch': [], 'body_keys': set(), 'system_prompt_sha': {}, 'max_tokens': set(),
           'temperature': set(), 'models': set()}
    # --- every model call: wire vs saved request ---
    for dec_path in sorted(glob.glob(str(run/'team'/'r*'/'*-decision*.json'))):
        dec = json.loads(Path(dec_path).read_text())
        if dec.get('wire_index') is None:
            continue
        rid = dec['robot_id']
        out['calls'] += 1
        check = wire_matches(run, rid, dec)
        if not all(check[k] for k in ('system', 'user_text', 'image_labels', 'image_bytes')):
            out['wire_mismatch'].append(dec['request'])
        out['max_tokens'].add(check['max_tokens'])
        out['temperature'].add(check['temperature'])
        out['models'].add(check['model'])
        req = json.loads((run/'team'/dec['request']).read_text())
        body = json.loads(req['messages'][1]['content'])
        out['body_keys'] |= set(body)
        system = req['messages'][0]['content'].replace(f'You are robot {rid},', 'You are robot <rid>,')
        out['system_prompt_sha'].setdefault(hashlib.sha256(system.encode()).hexdigest()[:16], 0)
        out['system_prompt_sha'][hashlib.sha256(system.encode()).hexdigest()[:16]] += 1
    # --- box_taken_by_peer: delay after assignment and the peer phase then ---
    assigns = {}
    taken = []
    for e in events:
        if e['event'] == 'assign':
            assigns[e['job']] = e['sim_time_s']
        if e['event'] == 'phase' and e.get('outcome') == 'box_taken_by_peer':
            job = e['job']
            # job ids are "<rid>-<1-based index into that robot's own jobs>"
            box = result['jobs'][e['robot_id']][int(job.rsplit('-', 1)[1]) - 1]['box']
            # the peer that holds the same box label at that time
            holders = [r for r, jobs in result['jobs'].items() if r != e['robot_id']
                       for j in jobs if j['box'] == box and j['issued_at_sim_s'] <= e['sim_time_s']]
            peer = holders[-1] if holders else None
            taken.append({'robot': e['robot_id'], 'job': job, 'box': box,
                          'delay_after_assign_s': round(e['sim_time_s'] - assigns[job], 2),
                          'peer': peer, 'peer_phase': peer and phase_at(events, peer, e['sim_time_s'])})
    out['box_taken_by_peer'] = taken
    # --- slot ids that encode a peer's still-active job in the same zone ---
    ends = {}
    for e in events:
        if e['event'] == 'job_end':
            ends.setdefault(e['robot_id'], []).append(e)
    spans = []
    for rid, jobs in result['jobs'].items():
        for i, j in enumerate(jobs):
            if j.get('slot') is None:
                continue
            end = ends.get(rid, [])
            finish = [e for e in end if e['sim_time_s'] >= j['issued_at_sim_s'] and e['box'] == j['box']]
            stop = finish[0]['sim_time_s'] if finish else 1e9
            spans.append({'robot': rid, 'zone': j['zone'], 'slot': j['slot'], 'start': j['issued_at_sim_s'],
                          'end': stop, 'status': j['status']})
    reveal = 0
    for s in spans:
        peers_active = [p for p in spans if p['robot'] != s['robot'] and p['zone'] == s['zone']
                        and p['start'] < s['start'] < p['end']]
        if peers_active and int(s['slot'][1:]) > 1:
            reveal += 1
    out['own_slot_ids_encoding_active_peer_job'] = reveal
    out['own_jobs_total'] = len(spans)
    # --- global turn counter visible in request_id ---
    gaps = 0
    for rid in ('r1', 'r2', 'r3'):
        turns = sorted({int(m.group(1)) for f in glob.glob(str(run/'team'/rid/'*-request.json'))
                        if (m := re.search(r'-(?:solo|claim)-(\d+)-\d+-\d+-request\.json$', f))})
        gaps += sum(1 for a, b in zip(turns, turns[1:]) if b - a > 1)
    out['request_id_turn_gaps_revealing_peer_rounds'] = gaps
    for key in ('body_keys', 'max_tokens', 'temperature', 'models'):
        out[key] = sorted(out[key])
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--root', type=Path, default=DEFAULT_ROOT)
    args = p.parse_args()
    runs = sorted(d for d in args.root.glob('ZC2-*') if d.is_dir())
    rows = [analyse_run(r) for r in runs]
    print(json.dumps({'root': str(args.root), 'runs': rows}, indent=1, sort_keys=True))


if __name__ == '__main__':
    main()
