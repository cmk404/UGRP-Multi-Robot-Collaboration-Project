#!/usr/bin/env python3
"""OFFLINE Korean dialogue pilot on recorded ZC2 zone decision points.

No SIM, no runner change. Re-asks recorded decision points with prompt
variants V0-V3 (harness/zone_dialogue_ko.py) through the zone runner's model
client (GeminiProxyCompleter with an audited opener, as in
scripts/three_robot_runtime.py) and simulates short multi-turn windows.

  select    choose decision points -> <exp>/sample.json
  single    V0..V3 (+ V0 repeat) per point -> <exp>/calls.jsonl, raw wire in <raw>
  dialogue  multi-turn windows -> <exp>/calls.jsonl, <exp>/dialogues.json
  analyze   evaluation-only metrics -> <exp>/results.json

Every call is appended to calls.jsonl as it returns: full prompt text, image
hashes, exact HTTP response body, usage, and pointers to the local raw wire.
A hard budget (--max-calls, default 150) counts every HTTP attempt.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import io
import json
import random
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from harness import zone_dialogue_ko as zk  # noqa: E402
from harness import zone_dialogue_metrics as zm  # noqa: E402
from harness.gemini_proxy import GeminiProxyCompleter, GeminiProxyError  # noqa: E402
from harness.three_robot_plan import parse  # noqa: E402

SOURCE = Path('/Users/changmin/projects/ugrp/outputs/zone-communication-20260925')
EXP = ROOT/'experiments/2026-09-25-zone-dialogue-ko-pilot'
RAW = ROOT/'outputs/zone-dialogue-ko-pilot-20260925'
SEED = 20260925
PER_STRATUM = 5
REPEAT_PER_STRATUM = 2
WINDOW = {'max_turns': 6, 'per_robot': 2}
SCENARIOS = (
    # All three robots were asked in these recorded rounds; M1 recorded a same-box collision (cyan-1).
    {'id': 'M1', 'run': 'ZC2-s13-dynamic-graspfail', 'phase': 'claim-1-0-1', 'order': ('r1', 'r2', 'r3'),
     'variants': ('V2', 'V3')},
    {'id': 'M2', 'run': 'ZC2-s12-dynamic-nominal', 'phase': 'claim-1-0-1', 'order': ('r2', 'r3', 'r1'),
     'variants': ('V2',)},
    {'id': 'M3', 'run': 'ZC2-s14-dynamic-graspfail', 'phase': 'claim-1-0-1', 'order': ('r3', 'r1', 'r2'),
     'variants': ('V2', 'V3')},
    {'id': 'M4', 'run': 'ZC2-s13-dynamic-nominal', 'phase': 'claim-1-0-1', 'order': ('r1', 'r2', 'r3'),
     'variants': ('V2',)},
)
LOCK = threading.Lock()


def sha(data):
    return hashlib.sha256(data if isinstance(data, bytes) else data.encode()).hexdigest()


def now():
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


def load_calls(exp):
    path = exp/'calls.jsonl'
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.exists() else []


def append_call(exp, row):
    with LOCK:
        with (exp/'calls.jsonl').open('a') as fh:
            fh.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + '\n')


class Budget:
    def __init__(self, used, limit):
        self.used, self.limit = used, limit

    def take(self):
        with LOCK:
            if self.used >= self.limit:
                raise SystemExit(f'call budget reached ({self.used}/{self.limit}); stopping before exceeding it')
            self.used += 1
            return self.used


# ---------------------------------------------------------------- selection

def describe_point(path):
    req = json.loads(path.read_text())
    facts = zk.parse_recorded_system(req['messages'][0]['content'])
    user = json.loads(req['messages'][1]['content'])
    run = path.parts[-4]
    _, seed, coord, fault = run.split('-')
    decision_path = path.with_name(path.name.replace('-request.json', '-decision.json'))
    decision = json.loads(decision_path.read_text())
    wire_path = path.parent/f"wire-{decision['wire_index']:03d}.json"
    wire = json.loads(wire_path.read_text())
    wire_resp = wire_path.with_name(wire_path.stem + '-response.json')
    if wire['messages'][0]['content'] != req['messages'][0]['content']:
        raise ValueError(f'wire/request mismatch for {path}')
    if facts.mode == 'plan':
        agreement = user['agreement']
        sub = ('ack' if agreement['proposal'] else
               'proposer' if agreement['proposer'] == user['robot_id'] else 'waiting')
    else:
        sub = 'conflict' if user.get('conflict') else ('post_stop' if user['team_board']['stopped_reports']
                                                       else ('initial' if not user['peer_messages'] else 'later'))
        reply = decision.get('reply') or {}
        if sub in ('post_stop', 'later') and (reply.get('claim') or {}).get('box') is None:
            sub += '_idle'      # every goal already met or covered: the model answered null
    return {'point_id': f"{run}/{user['robot_id']}/{req['request_id'].split(user['robot_id'] + '-', 1)[1]}",
            'run': run, 'seed': seed, 'coordination': coord, 'fault': fault, 'robot_id': user['robot_id'],
            'mode': facts.mode, 'sub': sub, 'tie_break_in_original': facts.tie_break,
            'request': str(path.relative_to(SOURCE)), 'request_sha256': sha(path.read_bytes()),
            'decision': str(decision_path.relative_to(SOURCE)), 'decision_sha256': sha(decision_path.read_bytes()),
            'original_reply': decision.get('reply'), 'original_error': decision.get('error'),
            'original_usage': decision.get('usage'),
            'wire': str(wire_path.relative_to(SOURCE)), 'wire_sha256': sha(wire_path.read_bytes()),
            'wire_response_sha256': sha(wire_resp.read_bytes()) if wire_resp.exists() else None,
            'settings': {k: wire[k] for k in ('model', 'temperature', 'max_tokens', 'reasoning_effort')}}


def cmd_select(args):
    points = [describe_point(p) for p in sorted(SOURCE.glob('ZC2-*-*/team/r*/zone-*-request.json'))
              if 'independent' not in p.parts[-4]]
    rng = random.Random(SEED)
    chosen = []
    for coord in ('dynamic', 'plan_first'):
        for fault in ('nominal', 'graspfail'):
            pool = [p for p in points if p['coordination'] == coord and p['fault'] == fault]
            forced = [p for p in pool if p['sub'] == 'conflict']       # the only recorded conflicts
            groups = {}
            for p in pool:
                if p['sub'] != 'conflict':
                    groups.setdefault(p['sub'], []).append(p)
            for g in groups.values():
                rng.shuffle(g)
            picked = list(forced)
            keys = sorted(groups)
            while len(picked) < PER_STRATUM and any(groups.values()):
                for k in keys:
                    if groups[k] and len(picked) < PER_STRATUM:
                        cand = groups[k].pop()
                        # spread seeds within a stratum when possible
                        if sum(q['seed'] == cand['seed'] for q in picked) >= 2 and any(
                                sum(q['seed'] == c['seed'] for q in picked) < 2 for c in groups[k]):
                            groups[k].insert(0, cand)
                            continue
                        picked.append(cand)
            for i, p in enumerate(picked):
                p['stratum'] = f'{coord}/{fault}'
                p['v0_repeat'] = i < REPEAT_PER_STRATUM
            chosen += picked
    summary = {'n_candidates': len(points), 'n_chosen': len(chosen), 'seed': SEED,
               'tie_break_in_originals': {m: sorted({p['tie_break_in_original'] for p in points if p['mode'] == m})
                                          for m in ('claim', 'plan')},
               'settings_in_originals': sorted({json.dumps(p['settings'], sort_keys=True) for p in points}),
               'original_invalid': [p['point_id'] for p in points if not p['original_reply']],
               'source_root': str(SOURCE), 'source_record': (SOURCE/'source.txt').read_text().strip()}
    EXP.mkdir(parents=True, exist_ok=True)
    (EXP/'sample.json').write_text(json.dumps({'summary': summary, 'points': chosen}, ensure_ascii=False,
                                              indent=2) + '\n')
    print(json.dumps(summary, indent=1))
    for p in chosen:
        print(p['stratum'], p['point_id'], p['sub'], 'repeat' if p['v0_repeat'] else '')


# ---------------------------------------------------------------- calling

def call_model(request, call_id, settings, budget, meta):
    """One audited call through the zone runner's client; one retry on a retryable transport error."""
    RAW.mkdir(parents=True, exist_ok=True)
    safe = call_id.replace('/', '__').replace(':', '_')
    for attempt in range(2):
        n = budget.take()
        wire_path = RAW/f'{safe}-a{attempt}-wire.json'
        resp_path = RAW/f'{safe}-a{attempt}-response.json'
        captured = {}

        def audited_open(req, *, timeout):
            captured['body'] = req.data
            wire_path.write_bytes(req.data)
            with urlopen(req, timeout=timeout) as response:
                data = response.read()
            captured['response'] = data
            resp_path.write_bytes(data)
            return io.BytesIO(data)

        client = GeminiProxyCompleter(model=settings['model'], max_tokens=settings['max_tokens'],
                                      temperature=settings['temperature'],
                                      reasoning_effort=settings['reasoning_effort'], timeout=60.,
                                      http_open=audited_open)
        row = {'call_id': call_id, 'attempt': attempt, 'budget_index': n, 'started_utc': now(), **meta,
               'settings': settings,
               'request': {'request_id': request['request_id'],
                           'messages': request['messages'],
                           'images': zk.image_digests(request)}}
        error = None
        try:
            raw = client.complete(request['messages'], images=request['images'])
            row['raw_text'] = raw
        except GeminiProxyError as exc:
            error = exc
            row['error'] = {'kind': exc.error_kind, 'message': str(exc), 'retryable': exc.retryable,
                            'http_status': exc.http_status}
        body = captured.get('body')
        row['wire'] = {'raw_path': str(wire_path.relative_to(ROOT)) if body else None,
                       'body_sha256': sha(body) if body else None, 'bytes': len(body) if body else 0}
        resp = captured.get('response')
        row['response'] = {'raw_path': str(resp_path.relative_to(ROOT)) if resp else None,
                           'sha256': sha(resp) if resp else None,
                           'http_body': resp.decode('utf-8', 'replace') if resp else None}
        row['usage'] = client.last_usage
        row['model_reported'] = client.last_model
        row['latency_ms'] = client.last_latency_ms
        append_call(EXP, row)
        if error is None or not error.retryable or attempt == 1:
            return row
        time.sleep(min(30., error.retry_after_s or 5.))
    return row


def done_ids(calls):
    return {c['call_id'] for c in calls if 'raw_text' in c or (c.get('error') and not c['error']['retryable'])
            or c['attempt'] == 1}


def cmd_single(args):
    sample = json.loads((EXP/'sample.json').read_text())
    calls = load_calls(EXP)
    budget = Budget(len(calls), args.max_calls)
    finished = done_ids(calls)
    jobs = []
    for p in sample['points']:
        req = json.loads((SOURCE/p['request']).read_text())
        if sha((SOURCE/p['request']).read_bytes()) != p['request_sha256']:
            raise SystemExit(f"source request changed: {p['request']}")
        reps = [('V0', 1)] if p['v0_repeat'] else []
        for variant, rep in [(v, 0) for v in zk.VARIANTS] + reps:
            call_id = f"single:{p['point_id']}:{variant}:{rep}"
            if call_id in finished:
                continue
            jobs.append((zk.build_variant(req, variant), call_id, p, variant, rep))
    planned = budget.used + len(jobs)
    print(f'single: {len(jobs)} calls to make; budget after = {planned}/{args.max_calls}', flush=True)
    if planned > args.max_calls:
        raise SystemExit('planned single-turn calls exceed the budget; stopping to ask first')
    if args.dry_run:
        return

    def run(job):
        request, call_id, p, variant, rep = job
        meta = {'kind': 'single', 'point_id': p['point_id'], 'variant': variant, 'rep': rep,
                'source_request': p['request'], 'source_request_sha256': p['request_sha256']}
        row = call_model(request, call_id, p['settings'], budget, meta)
        print(call_id, 'ok' if 'raw_text' in row else row.get('error'), row.get('usage'), flush=True)

    with ThreadPoolExecutor(max_workers=2) as pool:      # at most 2 concurrent model requests
        list(pool.map(run, jobs))


# ---------------------------------------------------------------- dialogue

def parse_window_reply(raw, request_id, structured, image_labels):
    """Strict parse; returns (value or None, error). No repair."""
    try:
        value = parse(raw)
        if not isinstance(value, dict) or set(value) != {'request_id', 'claim', 'reason', 'message', 'recipients'}:
            raise ValueError('window reply needs request_id, claim, reason, message, recipients')
        if value['request_id'] != request_id:
            raise ValueError('stale request_id')
        claim = value['claim']
        if not isinstance(claim, dict) or set(claim) != {'box', 'zone'} or (claim['box'] is None) != (claim['zone'] is None):
            raise ValueError('claim needs box and zone, both set or both null')
        if not isinstance(value['reason'], str) or len(value['reason']) > 600:
            raise ValueError('reason must be a string of at most 600 characters')
        if structured:
            zk.validate_struct_message(value['message'], image_labels)
        elif not isinstance(value['message'], str) or len(value['message']) > 600:
            raise ValueError('message must be a string of at most 600 characters')
        rec = value['recipients']
        if not isinstance(rec, list) or any(r not in zk.ROBOTS for r in rec) or len(set(rec)) != len(rec):
            raise ValueError('recipients must be a list of distinct robot ids')
        return value, None
    except (ValueError, TypeError, KeyError) as exc:
        return None, f'{type(exc).__name__}: {exc}'


def run_window(scenario, variant, budget):
    requests, settings = {}, None
    for rid in zk.ROBOTS:
        path = next((SOURCE/scenario['run']/'team'/rid).glob(f"zone-*-{rid}-{scenario['phase']}-request.json"))
        requests[rid] = (path, json.loads(path.read_text()))
        dec = json.loads(path.with_name(path.name.replace('-request.json', '-decision.json')).read_text())
        wire = json.loads((path.parent/f"wire-{dec['wire_index']:03d}.json").read_text())
        s = {k: wire[k] for k in ('model', 'temperature', 'max_tokens', 'reasoning_effort')}
        if settings not in (None, s):
            raise ValueError('robots used different settings')
        settings = s
    inbox = {r: [] for r in zk.ROBOTS}
    sent = {r: [] for r in zk.ROBOTS}
    last_claim = {r: None for r in zk.ROBOTS}
    turns = []
    structured = variant == 'V3'
    for t in range(1, WINDOW['max_turns'] + 1):
        rid = scenario['order'][(t - 1) % 3]
        path, req = requests[rid]
        request = zk.dialogue_request(req, window=WINDOW, received=inbox[rid], sent=sent[rid], turn=t,
                                      variant=variant)
        labels = [i['label'] for i in req['images']]
        call_id = f"dialogue:{scenario['id']}:{variant}:t{t}:{rid}"
        meta = {'kind': 'dialogue', 'scenario': scenario['id'], 'variant': variant, 'turn': t, 'robot_id': rid,
                'source_request': str(path.relative_to(SOURCE)), 'source_request_sha256': sha(path.read_bytes()),
                'received_before': copy.deepcopy(inbox[rid])}
        row = call_model(request, call_id, settings, budget, meta)
        value, error = (parse_window_reply(row['raw_text'], request['request_id'], structured, labels)
                        if 'raw_text' in row else (None, f"transport: {row.get('error')}"))
        delivered = []
        if value is not None:
            last_claim[rid] = value['claim']
            msg = value['message']
            speaking = (msg is not None) if structured else bool(msg.strip())
            if speaking:
                utter = {'from_robot': rid, 'to': list(value['recipients']), 'turn': t, 'message': msg}
                sent[rid].append(utter)
                for other in value['recipients']:
                    if other != rid:
                        inbox[other].append(utter)
                        delivered.append(other)
        turns.append({'turn': t, 'robot_id': rid, 'call_id': call_id, 'valid': value is not None,
                      'error': error, 'reply': value, 'delivered_to': delivered,
                      'received_before': meta['received_before'], 'usage': row.get('usage')})
        print(call_id, 'valid' if value else error, 'to', delivered, flush=True)
    return {'scenario': scenario['id'], 'variant': variant, 'run': scenario['run'], 'phase': scenario['phase'],
            'order': list(scenario['order']), 'settings': settings, 'tie_break_in_window': False,
            'final_claims': last_claim, 'turns': turns}


def cmd_dialogue(args):
    calls = load_calls(EXP)
    budget = Budget(len(calls), args.max_calls)
    out_path = EXP/'dialogues.json'
    done = json.loads(out_path.read_text()) if out_path.exists() else []
    have = {(d['scenario'], d['variant']) for d in done}
    todo = [(s, v) for s in SCENARIOS for v in s['variants'] if (s['id'], v) not in have]
    planned = budget.used + WINDOW['max_turns'] * len(todo)
    print(f'dialogue: {len(todo)} windows; budget after = {planned}/{args.max_calls}', flush=True)
    if planned > args.max_calls:
        raise SystemExit('planned dialogue calls exceed the budget; stopping to ask first')
    if args.dry_run:
        return
    for s, v in todo:           # windows are sequential by construction
        done.append(run_window(s, v, budget))
        out_path.write_text(json.dumps(done, ensure_ascii=False, indent=2) + '\n')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('command', choices=('select', 'single', 'dialogue', 'analyze'))
    ap.add_argument('--max-calls', type=int, default=150)
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()
    if args.command == 'analyze':
        from scripts.pilot_korean_dialogue_analysis import analyze
        analyze(EXP, SOURCE)
        return
    {'select': cmd_select, 'single': cmd_single, 'dialogue': cmd_dialogue}[args.command](args)


if __name__ == '__main__':
    main()
