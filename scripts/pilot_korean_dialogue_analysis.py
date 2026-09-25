"""EVALUATION-ONLY analysis for the offline Korean dialogue pilot.

Reads <exp>/sample.json, calls.jsonl, dialogues.json (and manual_labels.json if
present) and writes <exp>/results.json. Validity uses the zone runner's own
validators at this checkout (validate_claim_reply / validate_plan_reply /
plan_validator / check_claims); nothing is fed back into any request.
These are language and format measurements, NOT task-efficiency evidence.
"""
from __future__ import annotations

import collections
import hashlib
import json
import statistics
from pathlib import Path

from harness import zone_coordination as zc
from harness import zone_dialogue_ko as zk
from harness import zone_dialogue_metrics as zm
from harness.three_robot_plan import digest, parse, validate_plan_reply

# Identifier-like names (underscore) are literal references; plain English words
# such as zone/box/claim written in prose count as code-switching, not literals.
KEY_WORDS = ('rgb_view', 'box_labels', 'team_board', 'peer_messages', 'pickup_boxes_still_visible',
             'zone_counts_seen', 'robot_id', 'request_id', 'proposal_id', 'plan_hash', 'dialogue_window',
             'own_jobs', 'finished_reports', 'stopped_reports', 'executor_receipt')


def sha(data):
    return hashlib.sha256(data if isinstance(data, bytes) else data.encode()).hexdigest()


def mean(xs):
    xs = [x for x in xs if x is not None]
    return round(statistics.mean(xs), 4) if xs else None


def rate(num, den):
    return None if not den else round(num / den, 4)


def final_rows(calls, kind):
    rows = {}
    for c in calls:
        if c['kind'] == kind:
            rows[c['call_id']] = c        # later attempts overwrite earlier ones
    return rows


def attempts_by_id(calls):
    out = collections.Counter(c['call_id'] for c in calls)
    return out


def language(text, labels):
    ratio = zm.hangul_ratio(text, labels, KEY_WORDS)
    return {'hangul_ratio': None if ratio is None else round(ratio, 4),
            'english_words': zm.english_words(text, labels, KEY_WORDS),
            'id_issues': zm.id_issues(text, labels),
            'chars': len(text or ''), 'utf8_bytes': len((text or '').encode())}


def evaluate_single(row, point, source):
    req = json.loads((source/point['request']).read_text())
    facts = zk.parse_recorded_system(req['messages'][0]['content'])
    user = json.loads(req['messages'][1]['content'])
    rid, request_id = user['robot_id'], req['request_id']
    labels = user['box_labels']
    goal = json.loads(facts.goal)
    variant = row['variant']
    out = {'call_id': row['call_id'], 'point_id': point['point_id'], 'stratum': point['stratum'],
           'sub': point['sub'], 'mode': facts.mode, 'variant': variant, 'rep': row['rep'],
           'usage': row.get('usage'), 'latency_ms': row.get('latency_ms'),
           'transport_error': row.get('error'), 'wire_body_sha256': row['wire']['body_sha256']}
    if 'raw_text' not in row:
        out.update(json_ok=False, schema_ok=False, decision=None)
        return out
    raw = row['raw_text']
    try:
        value = parse(raw)
        out['json_ok'] = isinstance(value, dict)
    except (ValueError, TypeError):
        value, out['json_ok'] = None, False
    schema_error = None
    message = value.get('message') if isinstance(value, dict) else None
    try:
        if not isinstance(value, dict):
            raise ValueError('reply is not a JSON object')
        checked = dict(value)
        if variant == 'V3':
            zk.validate_struct_message(value.get('message'), [i['label'] for i in req['images']])
            checked['message'] = ''
        text = json.dumps(checked)
        if facts.mode == 'claim':
            zc.validate_claim_reply(text, request_id)
        else:
            validate_plan_reply(text, request_id, user['agreement'],
                                plan_validator=zc.plan_validator(goal, labels))
    except (ValueError, TypeError, KeyError) as exc:
        schema_error = f'{type(exc).__name__}: {exc}'
    out['schema_ok'] = schema_error is None
    out['schema_error'] = schema_error
    if isinstance(value, dict):
        if facts.mode == 'claim':
            claim = value.get('claim') if isinstance(value.get('claim'), dict) else None
            out['decision'] = None if claim is None else [claim.get('box'), claim.get('zone')]
            if out['schema_ok'] and claim and claim.get('box') is not None:
                out['host_check'] = host_check({rid: claim}, user, goal, labels)
        else:
            plan = value.get('plan')
            out['decision'] = [value.get('accept'), None if plan is None else digest(plan)]
            if plan is not None:
                try:
                    zc.plan_validator(goal, labels)(plan)
                    out['plan_valid'] = True
                except (ValueError, TypeError, KeyError) as exc:
                    out['plan_valid'] = False
                    out['plan_error'] = str(exc)
    out['message'] = message
    names = list(labels)
    if variant == 'V3':
        out['message_struct'] = True
        out['acts'] = zm.struct_acts(message) if isinstance(message, (dict, type(None))) else ['invalid']
    else:
        text = message if isinstance(message, str) else ''
        out['acts'] = zm.dialogue_acts(text)
        out['message_lang'] = language(text, names)
    reason = value.get('reason') if isinstance(value, dict) else None
    out['reason'] = reason
    out['reason_lang'] = language(reason if isinstance(reason, str) else '', names)
    return out


def host_check(claims, user, goal, labels):
    board = user['team_board']
    active = {r: {'box': j['box'], 'zone': j['zone'], 'kind': labels[j['box']]['kind']}
              for r, j in board['active'].items() if r not in claims and j['box'] in labels}
    finished = [{'zone': f['zone'], 'kind': labels[f['box']]['kind']} for f in board['finished_reports']
                if f['box'] in labels]
    return zc.check_claims(claims, goal=goal, labels=labels, view=user['rgb_view'], active=active,
                           finished=finished)


def original_decision(point):
    reply = point['original_reply']
    if reply is None:
        return None
    if 'claim' in reply:
        return [reply['claim']['box'], reply['claim']['zone']]
    return [reply['accept'], None if reply['plan'] is None else digest(reply['plan'])]


def summarize(rows, points, v0_index):
    by_point = {p['point_id']: p for p in points}
    out = {}
    for variant in zk.VARIANTS:
        sel = [r for r in rows if r['variant'] == variant and r['rep'] == 0]
        n = len(sel)
        valid = [r for r in sel if r.get('schema_ok')]
        speak = [r for r in sel if r.get('message_lang') and r['message_lang']['chars'] > 0]
        ratios = [r['message_lang']['hangul_ratio'] for r in speak if r['message_lang']['hangul_ratio'] is not None]
        same_v0 = [r for r in sel if v0_index.get(r['point_id']) is not None and r.get('decision') is not None]
        usage = [r['usage'] for r in sel if r.get('usage')]
        host = [r for r in sel if r.get('host_check')]
        entry = {
            'n_calls': n, 'transport_errors': sum(1 for r in sel if r.get('transport_error')),
            'json_ok': rate(sum(1 for r in sel if r.get('json_ok')), n),
            'schema_ok': rate(len(valid), n),
            'schema_errors': [(r['point_id'], r.get('schema_error')) for r in sel if not r.get('schema_ok')],
            'decision_same_as_v0': rate(sum(1 for r in same_v0 if r['decision'] == v0_index[r['point_id']]),
                                        len(same_v0)),
            'decision_changed_vs_v0': [(r['point_id'], v0_index[r['point_id']], r['decision']) for r in same_v0
                                       if r['decision'] != v0_index[r['point_id']]],
            'decision_same_as_original': rate(
                sum(1 for r in sel if r.get('decision') is not None
                    and r['decision'] == original_decision(by_point[r['point_id']])),
                sum(1 for r in sel if original_decision(by_point[r['point_id']]) is not None)),
            'claim_host_valid': rate(sum(1 for r in host if not r['host_check']['invalid']
                                         and not r['host_check']['collisions']), len(host)),
            'claim_host_checked': len(host),
            'plan_valid_when_present': rate(sum(1 for r in sel if r.get('plan_valid')),
                                            sum(1 for r in sel if 'plan_valid' in r)),
            'tokens': {k: {'mean': mean([u.get(k) for u in usage]), 'sum': sum(u.get(k, 0) for u in usage)}
                       for k in ('prompt_tokens', 'completion_tokens', 'total_tokens')},
            'unreported_tokens_mean': mean([u['total_tokens'] - u['prompt_tokens'] - u['completion_tokens']
                                            for u in usage if {'total_tokens', 'prompt_tokens',
                                                               'completion_tokens'} <= set(u)]),
            'latency_ms_mean_not_a_speed_claim': mean([r.get('latency_ms') for r in sel]),
        }
        if variant != 'V3':
            entry['messages'] = {
                'non_empty': len(speak), 'silent': n - len(speak),
                'hangul_ratio_mean': mean(ratios),
                'korean_ge_0_9': rate(sum(1 for x in ratios if x >= 0.9), len(speak)),
                'english_fallback_lt_0_5': rate(sum(1 for r in speak if (r['message_lang']['hangul_ratio'] or 0) < 0.5),
                                                len(speak)),
                'code_switched': rate(sum(1 for r in speak if 0 < (r['message_lang']['hangul_ratio'] or 0) < 1),
                                      len(speak)),
                'code_switch_words': collections.Counter(
                    w for r in speak if 0 < (r['message_lang']['hangul_ratio'] or 0) < 1
                    for w in r['message_lang']['english_words']).most_common(15),
                'id_issues': [(r['point_id'], r['message_lang']['id_issues']) for r in speak
                              if r['message_lang']['id_issues']],
                'chars_mean': mean([r['message_lang']['chars'] for r in speak]),
                'chars_max': max([r['message_lang']['chars'] for r in speak], default=None),
                'utf8_bytes_mean': mean([r['message_lang']['utf8_bytes'] for r in speak]),
                'over_240_chars': sum(1 for r in speak if r['message_lang']['chars'] > 240),
            }
        else:
            msgs = [r.get('message') for r in sel if r.get('schema_ok')]
            entry['messages'] = {
                'struct_valid_replies': len(msgs), 'silent': sum(1 for m in msgs if m is None),
                'fields_non_null': {f: sum(1 for m in msgs if isinstance(m, dict) and m.get(f) is not None)
                                    for f in zk.STRUCT_FIELDS},
                'json_chars_mean': mean([len(json.dumps(m, ensure_ascii=False)) for m in msgs if m is not None]),
            }
        entry['reason_hangul_ratio_mean'] = mean([r['reason_lang']['hangul_ratio'] for r in sel
                                                  if r.get('reason_lang')])
        entry['acts'] = collections.Counter(a for r in sel for a in r.get('acts', [])).most_common()
        entry['by_mode'] = {mode: {'n': sum(1 for r in sel if r['mode'] == mode),
                                   'schema_ok': rate(sum(1 for r in sel if r['mode'] == mode and r.get('schema_ok')),
                                                     sum(1 for r in sel if r['mode'] == mode)),
                                   'decision_same_as_v0': rate(
                                       sum(1 for r in same_v0 if r['mode'] == mode
                                           and r['decision'] == v0_index[r['point_id']]),
                                       sum(1 for r in same_v0 if r['mode'] == mode))}
                            for mode in ('claim', 'plan')}
        out[variant] = entry
    return out


def conflict_pairs(rows, points, source):
    """The two recorded conflict robots (same box cyan-1): joint host check per variant."""
    conf = [p for p in points if p['sub'] == 'conflict']
    out = {}
    if len(conf) != 2:
        return out
    for variant in zk.VARIANTS:
        claims, user0 = {}, None
        for p in conf:
            r = next((x for x in rows if x['point_id'] == p['point_id'] and x['variant'] == variant
                      and x['rep'] == 0), None)
            if r and r.get('schema_ok') and r.get('decision'):
                claims[p['robot_id']] = {'box': r['decision'][0], 'zone': r['decision'][1]}
            user0 = json.loads(json.loads((source/p['request']).read_text())['messages'][1]['content'])
        if len(claims) == 2:
            goal = {'A': {'red': 2}, 'B': {'cyan': 2}, 'C': {'green': 1, 'yellow': 1}}
            idle = {r: c for r, c in claims.items() if c['box'] is not None}
            check = host_check(idle, user0, goal, user0['box_labels']) if idle else None
            out[variant] = {'claims': claims, 'check': check}
    return out


def evaluate_dialogues(dialogues, calls, source):
    rows = final_rows(calls, 'dialogue')
    out = []
    for d in dialogues:
        req0 = next((source/d['run']/'team'/'r1').glob(f"zone-*-r1-{d['phase']}-request.json"))
        user0 = json.loads(json.loads(req0.read_text())['messages'][1]['content'])
        labels = user0['box_labels']
        goal = {'A': {'red': 2}, 'B': {'cyan': 2}, 'C': {'green': 1, 'yellow': 1}}
        turns = []
        last_seen = {r: 0 for r in zk.ROBOTS}
        for t in d['turns']:
            row = rows[t['call_id']]
            reply = t['reply']
            msg = reply['message'] if reply else None
            new_in = [u for u in t['received_before'] if u['turn'] > last_seen[t['robot_id']]]
            last_seen[t['robot_id']] = t['turn']
            entry = {'turn': t['turn'], 'robot_id': t['robot_id'], 'valid': t['valid'], 'error': t['error'],
                     'claim': reply['claim'] if reply else None, 'recipients': reply['recipients'] if reply else None,
                     'delivered_to': t['delivered_to'], 'new_received': [(u['from_robot'], u['turn']) for u in new_in],
                     'usage': row.get('usage')}
            if d['variant'] == 'V3':
                entry['message'] = msg
                entry['acts'] = zm.struct_acts(msg) if reply else ['invalid']
                rt = msg.get('reply_to') if isinstance(msg, dict) else None
                entry['reply_to'] = rt
                entry['reply_to_matches_received'] = bool(rt) and any(
                    (u['from_robot'], u['turn']) == (rt['from_robot'], rt['turn']) for u in t['received_before'])
            else:
                text = msg if isinstance(msg, str) else ''
                entry['message'] = text
                entry['acts'] = zm.dialogue_acts(text) if reply else ['invalid']
                entry['lang'] = language(text, list(labels))
                entry['references_new_peer_utterance'] = (
                    any(zm.references(text, u['message'], u['from_robot'], labels) for u in new_in)
                    if new_in and text else None)
            entry['reason_lang'] = language(reply['reason'] if reply else '', list(labels))
            turns.append(entry)
        final = {r: c for r, c in d['final_claims'].items() if c and c.get('box') is not None}
        check = host_check(final, user0 | {'team_board': {'active': {}, 'finished_reports': [],
                                                           'stopped_reports': []}}, goal, labels) if final else None
        recorded = {}
        for rid in zk.ROBOTS:
            p = next((source/d['run']/'team'/rid).glob(f"zone-*-{rid}-{d['phase']}-decision.json"))
            rep = json.loads(p.read_text()).get('reply')
            if rep and rep['claim']['box'] is not None:
                recorded[rid] = rep['claim']
        rec_check = host_check(recorded, user0, goal, labels) if recorded else None
        spoke = [x for x in turns if x['valid'] and (x['message'] not in ('', None))]
        ratios = [x['lang']['hangul_ratio'] for x in spoke if x.get('lang') and x['lang']['hangul_ratio'] is not None]
        answered = [x for x in turns if x['new_received']]
        out.append({
            'scenario': d['scenario'], 'variant': d['variant'], 'run': d['run'], 'phase': d['phase'],
            'order': d['order'], 'turns': turns,
            'summary': {
                'valid_turns': sum(1 for x in turns if x['valid']), 'turns': len(turns),
                'utterances': len(spoke), 'silences': sum(1 for x in turns if x['valid'] and x['message'] in ('', None)),
                'robots_speaking': sorted({x['robot_id'] for x in spoke}),
                'broadcasts': sum(1 for x in spoke if len(x['recipients'] or []) == 2),
                'direct': sum(1 for x in spoke if len(x['recipients'] or []) == 1),
                'no_recipient': sum(1 for x in spoke if not x['recipients']),
                'hangul_ratio_mean': mean(ratios),
                'korean_ge_0_9': rate(sum(1 for r in ratios if r >= 0.9), len(spoke)) if d['variant'] != 'V3' else None,
                'id_issues': [x['lang']['id_issues'] for x in spoke if x.get('lang') and x['lang']['id_issues']],
                'turns_with_new_peer_input': len(answered),
                'referenced_new_peer_input': sum(1 for x in answered if x.get('references_new_peer_utterance')),
                'reply_to_matches': sum(1 for x in answered if x.get('reply_to_matches_received')),
                'claim_changes': sum(1 for a, b in zip(turns, turns[3:]) if a['claim'] != b['claim']),
                'final_claims': d['final_claims'], 'final_check': check,
                'recorded_round_claims': recorded, 'recorded_round_check': rec_check,
                'tokens': {k: sum((x['usage'] or {}).get(k, 0) for x in turns)
                           for k in ('prompt_tokens', 'completion_tokens', 'total_tokens')},
            }})
    return out


def analyze(exp: Path, source: Path):
    sample = json.loads((exp/'sample.json').read_text())
    points = sample['points']
    calls = [json.loads(l) for l in (exp/'calls.jsonl').read_text().splitlines() if l.strip()]
    by_point = {p['point_id']: p for p in points}
    single = final_rows(calls, 'single')
    rows = [evaluate_single(r, by_point[r['point_id']], source) for r in single.values()]
    v0 = {r['point_id']: r.get('decision') for r in rows if r['variant'] == 'V0' and r['rep'] == 0}
    # replay control: V0 wire body vs the recorded original wire body, V0 rep0 vs rep1
    replay = []
    for p in points:
        r0 = single.get(f"single:{p['point_id']}:V0:0")
        r1 = single.get(f"single:{p['point_id']}:V0:1")
        e0 = next((r for r in rows if r['call_id'] == f"single:{p['point_id']}:V0:0"), None)
        e1 = next((r for r in rows if r['call_id'] == f"single:{p['point_id']}:V0:1"), None)
        replay.append({
            'point_id': p['point_id'],
            'wire_body_identical_to_original': bool(r0) and r0['wire']['body_sha256'] == p['wire_sha256'],
            'decision_original': original_decision(p), 'decision_v0': e0 and e0.get('decision'),
            'v0_equals_original': bool(e0) and e0.get('decision') == original_decision(p),
            'raw_text_identical_to_original': bool(r0) and 'raw_text' in r0 and p['original_reply'] is not None
                and r0['raw_text'] == json.loads((source/p['decision']).read_text()).get('raw_response'),
            'repeat': None if not r1 else {
                'decision_v0_rep1': e1 and e1.get('decision'),
                'same_decision': bool(e1) and e0 and e1.get('decision') == e0.get('decision'),
                'same_raw_text': 'raw_text' in r0 and 'raw_text' in r1 and r0['raw_text'] == r1['raw_text']}})
    attempts = attempts_by_id(calls)
    dialogues_path = exp/'dialogues.json'
    dialogues = evaluate_dialogues(json.loads(dialogues_path.read_text()), calls, source) \
        if dialogues_path.exists() else []
    manual = json.loads((exp/'manual_labels.json').read_text()) if (exp/'manual_labels.json').exists() else None
    usage_all = [c['usage'] for c in calls if c.get('usage')]
    result = {
        'schema': 'ugrp.zone_dialogue_ko_pilot.results.v1',
        'scope': ('OFFLINE re-asks of recorded ZC2 decision points; evaluation-only language/format metrics. '
                  'NOT task-efficiency evidence: no SIM ran, no decision was executed, and changed decisions '
                  'were not tested physically.'),
        'source': sample['summary'],
        'calls': {'http_attempts': len(calls), 'by_kind': collections.Counter(c['kind'] for c in calls),
                  'retried_call_ids': [k for k, v in attempts.items() if v > 1],
                  'transport_errors': sum(1 for c in calls if c.get('error')),
                  'budget': 150,
                  'tokens_total': {k: sum(u.get(k, 0) for u in usage_all)
                                   for k in ('prompt_tokens', 'completion_tokens', 'total_tokens')},
                  'calls_jsonl_sha256': sha((exp/'calls.jsonl').read_bytes())},
        'replay_control': {
            'wire_identical': sum(1 for r in replay if r['wire_body_identical_to_original']),
            'v0_decision_equals_original': sum(1 for r in replay if r['v0_equals_original']),
            'v0_raw_equals_original': sum(1 for r in replay if r['raw_text_identical_to_original']),
            'repeats': sum(1 for r in replay if r['repeat']),
            'repeat_same_decision': sum(1 for r in replay if r['repeat'] and r['repeat']['same_decision']),
            'repeat_same_raw': sum(1 for r in replay if r['repeat'] and r['repeat']['same_raw_text']),
            'points': replay},
        'variants': summarize(rows, points, v0),
        'conflict_pair': conflict_pairs(rows, points, source),
        'single_rows': rows,
        'dialogues': dialogues,
        'manual_labels': manual,
    }
    (exp/'results.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, default=list) + '\n')
    print(json.dumps({k: result[k] for k in ('calls', 'replay_control')}, ensure_ascii=False, default=list)[:3000])
