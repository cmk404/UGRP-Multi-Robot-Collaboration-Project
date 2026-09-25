#!/usr/bin/env python3
"""EVALUATION-ONLY recheck of the dialogue act rules (v1 vs v2) on a recorded pilot.

Offline: reads a recorded results file and manual label files, relabels every
free-text message with both rule versions and writes one new artifact. It never
touches the recorded results, calls.jsonl or raw wire files, and never calls a
model.

  PYTHONPATH=. python scripts/pilot_korean_dialogue_act_recheck.py \
      --exp experiments/2026-09-25-zone-dialogue-ko-pilot \
      --out experiments/2026-09-25-zone-dialogue-ko-pilot/act_rules_v2_check.json

Rule v2 was written from the 2026-09-25 review findings (word boundaries,
negation, act vs mention), not fitted to these labels; remaining disagreements
are listed as `rule_misses` instead of being patched away. The one exception is
recorded in the artifact: the `please accept` suppression was added after the
English sample showed it, see notes in the output.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from harness import zone_dialogue_metrics as zm  # noqa: E402
from scripts.pilot_korean_dialogue import window_call_id  # noqa: E402  (id format, no model call)


def sha(data):
    return hashlib.sha256(data if isinstance(data, bytes) else data.encode()).hexdigest()


def messages(results):
    """Every free-text message in the record: single-turn rows and window turns."""
    out = {}
    for row in results.get('single_rows', []):
        if row.get('variant') != 'V3' and isinstance(row.get('message'), str) and row['message'].strip():
            out[row['call_id']] = {'message': row['message'], 'variant': row['variant'],
                                   'kind': 'single'}
    for d in results.get('dialogues', []):
        if d.get('variant') == 'V3':
            continue
        episode = d.get('episode', 1)
        for turn in d.get('turns', []):
            text = turn.get('message')
            if isinstance(text, str) and text.strip():
                call_id = turn.get('call_id') or window_call_id(d['scenario'], d['variant'], episode,
                                                               turn['turn'], turn['robot_id'])
                out[call_id] = {'message': text, 'variant': d['variant'], 'kind': 'dialogue',
                                'scenario': d['scenario'], 'episode': episode}
    return out


def compare(manual_rows, labels):
    """Manual labels vs both rule versions. Sets: label order is not evaluated."""
    rows, agree = [], Counter()
    for row in manual_rows:
        manual = set(row['manual'])
        text = row['message']
        got = {v: set(zm.dialogue_acts(text, v)) for v in zm.ACT_RULES_VERSIONS}
        known = labels.get(row['id'], {}).get('message')
        rows.append({'id': row['id'], 'message': text,
                     'message_matches_record': None if known is None else known == text,
                     'manual': sorted(manual),
                     **{f'rule_{v}': sorted(got[v]) for v in zm.ACT_RULES_VERSIONS},
                     **{f'exact_{v}': manual == got[v] for v in zm.ACT_RULES_VERSIONS},
                     'rule_v2_missed': sorted(manual - got['v2']),
                     'rule_v2_extra': sorted(got['v2'] - manual)})
        for v in zm.ACT_RULES_VERSIONS:
            agree[v] += manual == got[v]
    return rows, agree


def recheck(exp: Path, results_path: Path, manual_paths, out_path: Path):
    if out_path.exists():
        raise SystemExit(f'{out_path} already exists; pass a new --out (records are never overwritten)')
    results = json.loads(results_path.read_text())
    labels = messages(results)
    relabelled, changed = {}, []
    for call_id, item in sorted(labels.items()):
        got = {v: zm.dialogue_acts(item['message'], v) for v in zm.ACT_RULES_VERSIONS}
        relabelled[call_id] = {**item, **{f'acts_{v}': got[v] for v in zm.ACT_RULES_VERSIONS}}
        if set(got['v1']) != set(got['v2']):
            changed.append({'call_id': call_id, 'message': item['message'],
                            'v1': got['v1'], 'v2': got['v2']})
    manual = []
    for path in manual_paths:
        if not path.exists():
            continue
        data = json.loads(path.read_text())
        rows, agree = compare(data['rows'], labels)
        manual.append({'file': str(path.relative_to(ROOT)), 'sha256': sha(path.read_bytes()),
                       'labeller': data.get('labeller'), 'n': len(rows),
                       'exact_match': {v: agree[v] for v in zm.ACT_RULES_VERSIONS},
                       'rule_misses': [r for r in rows if not r['exact_v2']], 'rows': rows})
    artifact = {
        'schema': 'ugrp.zone_dialogue_ko_pilot.act_rules_check.v1',
        'scope': ('EVALUATION-ONLY relabelling of recorded messages. No model call, no SIM, and no change to '
                  'the recorded results/raw files. Language and act-format measurements only.'),
        'source': {'experiment': str(exp.relative_to(ROOT)), 'results': str(results_path.relative_to(ROOT)),
                   'results_schema': results.get('schema'), 'results_sha256': sha(results_path.read_bytes()),
                   'free_text_messages': len(labels)},
        'rules': {'reported': 'v2', 'versions': list(zm.ACT_RULES_VERSIONS),
                  'v1_sha256': sha(json.dumps(zm.ACT_RULES_V1, ensure_ascii=False, sort_keys=True)),
                  'v2_sha256': sha(json.dumps(zm.ACT_RULES_V2, ensure_ascii=False, sort_keys=True)),
                  'definitions': zm.ACT_DEFINITIONS,
                  'free_text_only_acts': list(zm.ACTS_FREE_TEXT_ONLY),
                  'structured_enum': list(zm.STRUCTURED_ACTS),
                  'design_note': ('v2 = word boundaries on Latin cues + suppression spans for negation and for '
                                  'mentions of a peer act. Written from the review findings, not fitted to the '
                                  'manual labels; the `please accept` suppression is the one rule added after '
                                  'seeing the English sample. Remaining disagreements stay as rule_misses.')},
        'label_changes': {'messages': len(labels), 'changed_v1_to_v2': len(changed), 'rows': changed,
                          'acts_v1': Counter(a for r in relabelled.values() for a in r['acts_v1']).most_common(),
                          'acts_v2': Counter(a for r in relabelled.values() for a in r['acts_v2']).most_common()},
        'manual_checks': manual,
        'relabelled': relabelled,
    }
    out_path.write_text(json.dumps(artifact, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'messages': len(labels), 'changed_v1_to_v2': len(changed),
                      'manual': [{'file': m['file'], 'n': m['n'], 'exact_match': m['exact_match']}
                                 for m in manual]}, ensure_ascii=False, indent=1))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--exp', type=Path, default=ROOT/'experiments/2026-09-25-zone-dialogue-ko-pilot')
    ap.add_argument('--results', type=Path, default=None)
    ap.add_argument('--manual', type=Path, nargs='*', default=None)
    ap.add_argument('--out', type=Path, default=None)
    args = ap.parse_args()
    exp = args.exp
    recheck(exp, args.results or exp/'results.json',
            args.manual if args.manual is not None else [exp/'manual_labels.json', exp/'manual_labels_en.json'],
            args.out or exp/'act_rules_v2_check.json')


if __name__ == '__main__':
    main()
