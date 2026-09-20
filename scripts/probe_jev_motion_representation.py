#!/usr/bin/env python3
"""Representation diagnostic over saved RGB states; no simulator or actuation."""
from __future__ import annotations
import argparse
import copy
import getpass
import hashlib
import json
from pathlib import Path
import random
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from harness.jev_motion import GOAL, OPTIONS, at_goal, jev_request, post, rule, validate_jev

# Fixed before the first request. Failure states are diagnostic, not a holdout.
SELECTION = [
    ('dev-v5', 'rule-dev', (0, 20, 33, 35), 'development'),
    ('dev-v5', 'rule-dev_yaw', (0, 4, 20, 37), 'development'),
    ('final-v2', 'jev-straight', (39, 60), 'observed_failure'),
    ('final-v2', 'jev-left_offset', (0, 10), 'observed_failure'),
]
VARIANTS = ('original', 'semantic_state', 'semantic_state_explicit_criteria')

def semantic(o):
    lo, hi = GOAL['range_m']
    return {
        'distance_to_goal_band': 'too_far' if o['range_m'] > hi else 'too_close' if o['range_m'] < lo else 'inside',
        'target_side': 'left' if o['bearing_deg'] > 0 else 'right' if o['bearing_deg'] < 0 else 'straight_ahead',
        'heading_alignment': 'outside_tolerance' if abs(o['bearing_deg']) > GOAL['absolute_bearing_deg_max'] else 'inside_tolerance',
        'goal_satisfied': at_goal({'valid': True, **o}),
    }

def request(state, variant):
    body = jev_request(copy.deepcopy(state))
    if variant == 'original': return body
    body['state'] = {
        'task': 'Approach and face the cyan box without touching it, on an open floor.',
        'state_origin': 'Named relations computed from the same RGB estimates and goal thresholds. No simulator truth.',
        'observation': semantic(state['observation']),
        'recent_history': [{'issued_action': h['action'], 'observed_before_command': semantic(h)} for h in state['recent_history']],
    }
    body['questions']['action']['instructions'] = (
        'Select the next short motor action to reach the goal efficiently without touching the box. '
        'Goal: distance inside the target band AND heading inside tolerance. '
        'Target side is relative to your own forward heading. '
        'History is issued commands and RGB observations before each command, not guaranteed movement. '
        'Use the newest observation. No grasp or transport.')
    if variant == 'semantic_state_explicit_criteria':
        conditions = {
            'forward': 'Use when heading is inside tolerance and distance is too_far.',
            'backward': 'Use when heading is inside tolerance and distance is too_close.',
            'turn_left': 'Use when heading is outside tolerance and target_side is left.',
            'turn_right': 'Use when heading is outside tolerance and target_side is right.',
            'left': 'For a lateral positioning goal. This task has no lateral positioning requirement.',
            'right': 'For a lateral positioning goal. This task has no lateral positioning requirement.',
            'stop': 'Use when goal_satisfied is true. A valid observation outside the goal is not completion.',
        }
        body['questions']['action']['criteria'] = {k: {'effect': OPTIONS[k], 'use_when': conditions[k]} for k in OPTIONS}
    return body

def write(path, body):
    path.write_text(json.dumps(body, ensure_ascii=False, indent=2, allow_nan=False)+'\n')

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--raw-root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--execute', action='store_true')
    args = p.parse_args()
    if not args.execute: p.error('--execute required for 72 paid API requests; no actuation')
    if args.output.exists(): p.error('output must be new')
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip(): p.error('commit source first')
    sha = subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    samples = []
    for cohort, trial, indices, partition in SELECTION:
        src = args.raw_root/('jev-direct-motion-20260921-'+cohort)/trial/'turns.json'
        data = src.read_bytes(); turns = json.loads(data)
        for index in indices:
            row = turns[index]
            samples.append({'id': f'{cohort}/{trial}/{index:03d}', 'partition': partition,
                'source': str(src), 'source_sha256': hashlib.sha256(data).hexdigest(),
                'images': row['images'], 'state': row['state'], 'reference_rule_action': rule(row['state'])})
    jobs = [(i,v,r) for i in range(len(samples)) for v in VARIANTS for r in range(2)]
    random.Random(21).shuffle(jobs)
    args.output.mkdir(parents=True)
    write(args.output/'protocol.json', {'source_sha': sha, 'samples': samples, 'order': jobs,
        'max_calls': len(jobs), 'max_input_tokens': 150000, 'max_wall_s': 360,
        'scope': 'diagnostic archived-state API replay; not new autonomous execution or optimality proof',
        'metric': 'agreement with existing explicit rule, not unique optimal-action correctness',
        'confounds': 'semantic_state changes representation and question wording; explicit_criteria encodes the reference controller'})
    key = getpass.getpass('Jev API key (hidden): ')
    started = time.perf_counter(); results = []; tokens = 0
    for seq,(index,variant,repeat) in enumerate(jobs):
        if tokens+5000 > 150000 or time.perf_counter()-started > 360: raise RuntimeError('diagnostic budget reached')
        sample = samples[index]; body = request(sample['state'],variant)
        write(args.output/f'{seq:03d}-request.json',body)
        response = post(body,'https://api.typesafe.ai/v1/systemone',key,30)
        write(args.output/f'{seq:03d}-response.json',response)
        if response['status'] != 'ok': raise RuntimeError('diagnostic API request failed; no retry')
        choice = validate_jev(response['body']); answer = response['body']['answers']['action']
        usage = response['body'].get('usage',{}); tokens += usage.get('input_tokens',0)
        results.append({'seq':seq,'sample':sample['id'],'partition':sample['partition'],
            'variant':variant,'repeat':repeat,'choice':choice,'reference_rule_action':sample['reference_rule_action'],
            'agrees_with_rule':choice==sample['reference_rule_action'],'answer':answer,
            'latency_s':response['latency_s'],'usage':usage,
            'choice_probability_mismatch':answer['probabilities'][choice]<max(answer['probabilities'].values())-1e-6})
        write(args.output/'results.json',results)
        if (seq+1)%12==0: print(f'{seq+1}/{len(jobs)} archived-state decisions recorded',flush=True)
    summary = {v: {'rule_agreement':sum(r['agrees_with_rule'] for r in results if r['variant']==v),
        'calls':sum(r['variant']==v for r in results)} for v in VARIANTS}
    write(args.output/'summary.json',{'source_sha':sha,'summary':summary,'input_tokens':tokens,
        'wall_s':time.perf_counter()-started,'cost_usd_billed':None})
    print(json.dumps(summary),flush=True)

if __name__ == '__main__': main()
