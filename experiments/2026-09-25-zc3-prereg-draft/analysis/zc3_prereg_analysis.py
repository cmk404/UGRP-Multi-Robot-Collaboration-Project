#!/usr/bin/env python3
"""ZC3 preregistration DRAFT: measured costs, precision/power tables, seed plan, budget tiers.

Reads only recorded raw files (no SIM, no LLM). Outputs are deterministic given the inputs:

    .venv-sim-worker-mac/bin/python experiments/2026-09-25-zc3-prereg-draft/analysis/zc3_prereg_analysis.py \
        --outputs-root /Users/changmin/projects/ugrp/outputs

Writes (next to this file, and budget.json one level up):
    measured_runs.json   per-run table + per-cell summaries + SHA-256 of every input file read
    power.json           CI half-width and power tables (exact binomial / Fisher / Beta posterior)
    seed_plan.json       spawn permutation of candidate seeds (setup RNG only) + proposed seed lists
    ../budget.json       three budget tiers (runs, tokens, wall hours) derived from the above

Statistics are pure Python (math.comb / lgamma) so no scipy is needed.
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import itertools
import json
import math
import random
import statistics
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXP = HERE.parent
REPO = EXP.parent.parent

ZC_DIR = 'zone-communication-20260925'
ZW_DIR = 'zone-wide-20260925'
Z_DIR = 'zone-dispatch-20260925'
CONDITIONS = ('independent', 'plan_first', 'dynamic')
SCENARIOS = ('nominal', 'graspfail')
GOAL_G8 = {'A': {'red': 2}, 'B': {'cyan': 2}, 'C': {'green': 1, 'yellow': 1}}
EXTRA_G8 = {'red': 1, 'cyan': 1}
USED_SEEDS = (11, 12, 13, 14)  # every seed in Z1-Z3, ZW1-ZW2, ZC1-ZC2 and their dev/gate runs


# ----------------------------------------------------------------------------- inputs

class Inputs:
    """Reads JSON files and records a SHA-256 for every file actually read."""

    def __init__(self, root: Path):
        self.root = root
        self.hashes: dict[str, str] = {}

    def read_bytes(self, rel: str) -> bytes:
        data = (self.root / rel).read_bytes()
        self.hashes[rel] = hashlib.sha256(data).hexdigest()
        return data

    def json(self, rel: str):
        return json.loads(self.read_bytes(rel))

    def jsonl(self, rel: str):
        return [json.loads(line) for line in self.read_bytes(rel).decode().splitlines() if line.strip()]


def driver_wall(loads: list[dict]) -> dict[str, float]:
    """Driver wall seconds per run from loads.jsonl start/end stamps (includes process startup)."""
    start, out = {}, {}
    for row in loads:
        if 'start_utc' in row:
            start[row['run']] = dt.datetime.fromisoformat(row['start_utc'].replace('Z', '+00:00'))
        elif 'end_utc' in row and row['run'] in start:
            end = dt.datetime.fromisoformat(row['end_utc'].replace('Z', '+00:00'))
            out[row['run']] = (end - start[row['run']]).total_seconds()
    return out


def over_under(goal: dict, counts: dict) -> tuple[int, int]:
    over = under = 0
    for zone in set(goal) | set(counts):
        want, seen = goal.get(zone, {}), counts.get(zone, {})
        for kind in set(want) | set(seen):
            diff = seen.get(kind, 0) - want.get(kind, 0)
            over += max(diff, 0)
            under += max(-diff, 0)
    return over, under


def failure_mode(r: dict, over: int, under: int) -> str:
    if r.get('error'):
        return 'error'
    if r.get('phase') != 'FINISHED':
        return 'phase:' + str(r.get('phase'))
    if r.get('success'):
        return 'success'
    if over and under:
        return 'over+under_delivery'
    if over:
        return 'over_delivery'
    if under:
        return 'under_delivery'
    return 'failed_other'


def run_row(inp: Inputs, cohort_dir: str, name: str, loads_wall: dict, *, cohort: str) -> dict:
    r = inp.json(f'{cohort_dir}/{name}/result.json')
    cfg = r['config']
    over, under = over_under(r['goal'], r['referee']['zone_counts'])
    stats = r.get('coordination_stats') or {}
    events = inp.json(f'{cohort_dir}/{name}/teacher-events.json')
    row = {
        'cohort': cohort, 'run': name, 'arena': cfg['variant'], 'seed': cfg['seed'],
        'condition': cfg['coordination'], 'mode': cfg['mode'], 'model': cfg['model'] if cfg['mode'] == 'llm' else None,
        'scenario': 'graspfail' if cfg.get('inject_grasp_failure') else 'nominal',
        'source_sha': r['source_sha'], 'phase': r['phase'], 'error': r['error'],
        'success': bool(r['success']), 'goal_met_rgb': r.get('goal_met_rgb'),
        'over': over, 'under': under, 'failure_mode': failure_mode(r, over, under),
        'makespan_sim_s': r.get('makespan_sim_s'), 'control_end_sim_s': round(r['sim_end_s'], 2),
        'llm_calls': r['llm_calls'],
        'input_tokens': (r.get('usage') or {}).get('prompt_tokens', 0),
        'output_tokens': (r.get('usage') or {}).get('completion_tokens', 0),
        'wall_s': r['wall_s'], 'driver_wall_s': loads_wall.get(name),
        'claim_rounds': stats.get('claim_rounds'), 'collisions': stats.get('collisions'),
        'invalid_claims': stats.get('invalid_claims'), 'plan_turns': stats.get('plan_turns'),
        'stopped_reports': len(r.get('stopped_reports') or []),
        'injection_robot': (r.get('injection') or {}).get('robot'),
        'teacher_event_count': len(events) if isinstance(events, list) else len(events.get('events', [])),
    }
    if cfg['mode'] == 'llm':
        team = inp.json(f'{cohort_dir}/{name}/team/team.json')
        lat = [c['latency_ms'] / 1000 for c in team['calls'] if c.get('latency_ms') is not None]
        row['model_latency_sum_s'] = round(sum(lat), 2)
        row['model_latency_mean_s'] = round(statistics.mean(lat), 2) if lat else None
        row['calls_in_team_json'] = len(team['calls'])
    return row


def collect(root: Path):
    inp = Inputs(root)
    zc_loads = driver_wall(inp.jsonl(f'{ZC_DIR}/loads.jsonl'))
    primary, gates = [], []
    for seed in (12, 13, 14):
        for scen in SCENARIOS:
            for cond in CONDITIONS:
                primary.append(run_row(inp, ZC_DIR, f'ZC2-s{seed}-{cond}-{scen}', zc_loads, cohort='ZC2'))
    for prefix, cohort in (('gate', 'ZC1-gate'), ('gate2', 'ZC2-gate')):
        for cond in CONDITIONS:
            for scen in SCENARIOS:
                gates.append(run_row(inp, ZC_DIR, f'{prefix}-{cond}-{scen}', zc_loads, cohort=cohort))
    # Supplementary cost reference (plan_first/dynamic only, 1 run per condition; different goals/cohorts).
    supp = []
    zw_loads = driver_wall(inp.jsonl(f'{ZW_DIR}/loads.jsonl'))
    for c in ('ZW1', 'ZW2'):
        for g in ('G5', 'G8'):
            for m in ('plan', 'dyn'):
                supp.append(run_row(inp, ZW_DIR, f'{c}-{g}-{m}', zw_loads, cohort=c))
    z_loads = driver_wall(inp.jsonl(f'{Z_DIR}/loads.jsonl'))
    for g in ('G5', 'G8'):
        for m in ('plan', 'dyn'):
            supp.append(run_row(inp, Z_DIR, f'Z3-{g}-{m}', z_loads, cohort='Z3'))
    # Cross-check against the committed ZC2 record.
    rec_rel = 'experiments/2026-09-25-zone-communication/results.json'
    rec_bytes = (REPO / rec_rel).read_bytes()
    record = json.loads(rec_bytes)
    by_run = {x['run']: x for x in record['runs']}
    mismatches = []
    for row in primary:
        ref = by_run[row['run']]
        for ours, theirs in (('success', 'success'), ('over', 'over'), ('under', 'under'),
                             ('makespan_sim_s', 'makespan_sim_s'), ('llm_calls', 'llm_calls'),
                             ('input_tokens', 'prompt_tokens'), ('output_tokens', 'completion_tokens')):
            if row[ours] != ref[theirs]:
                mismatches.append({'run': row['run'], 'field': ours, 'raw': row[ours], 'record': ref[theirs]})
        if row['calls_in_team_json'] != row['llm_calls']:
            mismatches.append({'run': row['run'], 'field': 'calls_in_team_json', 'raw': row['calls_in_team_json'],
                               'record': row['llm_calls']})
    hashes = {f'outputs/{k}': v for k, v in sorted(inp.hashes.items())}
    hashes[rec_rel] = hashlib.sha256(rec_bytes).hexdigest()
    return primary, gates, supp, hashes, mismatches


def describe(values):
    v = [x for x in values if x is not None]
    if not v:
        return None
    out = {'n': len(v), 'mean': round(statistics.mean(v), 2), 'min': min(v), 'max': max(v)}
    out['sd'] = round(statistics.stdev(v), 2) if len(v) > 1 else None
    return out


def cell_summary(rows):
    cells = {}
    for cond in CONDITIONS:
        for scen in SCENARIOS:
            rs = [r for r in rows if r['condition'] == cond and r['scenario'] == scen]
            if not rs:
                continue
            cells[f'{cond}/{scen}'] = {
                'success': f"{sum(r['success'] for r in rs)}/{len(rs)}",
                'failure_modes': sorted(r['failure_mode'] for r in rs),
                'control_end_sim_s': describe([r['control_end_sim_s'] for r in rs]),
                'makespan_sim_s': describe([r['makespan_sim_s'] for r in rs]),
                'llm_calls': describe([r['llm_calls'] for r in rs]),
                'input_tokens': describe([r['input_tokens'] for r in rs]),
                'output_tokens': describe([r['output_tokens'] for r in rs]),
                'wall_s': describe([r['wall_s'] for r in rs]),
                'driver_wall_s': describe([r['driver_wall_s'] for r in rs]),
                'model_latency_mean_s': describe([r.get('model_latency_mean_s') for r in rs]),
            }
    return cells


# ----------------------------------------------------------------------------- statistics

def binom_pmf(n, p):
    return [math.comb(n, k) * p**k * (1 - p)**(n - k) for k in range(n + 1)]


def _betacf(a, b, x):
    # Lentz continued fraction for the regularised incomplete beta (Numerical Recipes).
    tiny, qab, qap, qam = 1e-300, a + b, a + 1, a - 1
    c, d = 1.0, 1 - qab * x / qap
    d = 1 / (d if abs(d) > tiny else tiny)
    h = d
    for m in range(1, 400):
        m2 = 2 * m
        aa = m * (b - m) * x / ((qam + m2) * (a + m2))
        d = 1 + aa * d; d = 1 / (d if abs(d) > tiny else tiny)
        c = 1 + aa / c; c = c if abs(c) > tiny else tiny
        h *= d * c
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))
        d = 1 + aa * d; d = 1 / (d if abs(d) > tiny else tiny)
        c = 1 + aa / c; c = c if abs(c) > tiny else tiny
        de = d * c
        h *= de
        if abs(de - 1) < 1e-14:
            break
    return h


def betainc(a, b, x):
    if x <= 0:
        return 0.0
    if x >= 1:
        return 1.0
    lbt = math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b) + a * math.log(x) + b * math.log(1 - x)
    if x < (a + 1) / (a + b + 2):
        return math.exp(lbt) * _betacf(a, b, x) / a
    return 1 - math.exp(lbt) * _betacf(b, a, 1 - x) / b


def beta_ppf(q, a, b):
    lo, hi = 0.0, 1.0
    for _ in range(100):
        mid = (lo + hi) / 2
        if betainc(a, b, mid) < q:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def clopper_pearson(k, n, conf=0.95):
    a = (1 - conf) / 2
    lo = 0.0 if k == 0 else beta_ppf(a, k, n - k + 1)
    hi = 1.0 if k == n else beta_ppf(1 - a, k + 1, n - k)
    return lo, hi


def wilson(k, n, z=1.959963984540054):
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return c - h, c + h


def newcombe_diff(k1, n1, k2, n2):
    """Newcombe hybrid score CI (method 10) for p1 - p2."""
    p1, p2 = k1 / n1, k2 / n2
    l1, u1 = wilson(k1, n1)
    l2, u2 = wilson(k2, n2)
    d = p1 - p2
    lo = d - math.sqrt((p1 - l1) ** 2 + (u2 - p2) ** 2)
    hi = d + math.sqrt((u1 - p1) ** 2 + (p2 - l2) ** 2)
    return lo, hi


def single_rate_n(h, planning_ps, nmax=200):
    """Smallest n with CI half-width <= h: worst case over outcomes, and expected at planning p."""
    out = {}
    for method, ci in (('clopper_pearson', clopper_pearson), ('wilson', wilson)):
        worst = None
        for n in range(2, nmax + 1):
            if max((ci(k, n)[1] - ci(k, n)[0]) / 2 for k in range(n + 1)) <= h:
                worst = n
                break
        expected = {}
        for p in planning_ps:
            for n in range(2, nmax + 1):
                pmf = binom_pmf(n, p)
                if sum(w * (ci(k, n)[1] - ci(k, n)[0]) / 2 for k, w in enumerate(pmf)) <= h:
                    expected[str(p)] = n
                    break
        out[method] = {'worst_case_n': worst, 'expected_n_at_p': expected}
    return out


def diff_n(h, pairs, nmax=300):
    """Per-arm n with Newcombe CI half-width <= h: worst case (p1=p2=0.5 expected) and at planning pairs."""
    def exp_hw(n, p1, p2):
        a, b = binom_pmf(n, p1), binom_pmf(n, p2)
        return sum(wa * wb * (newcombe_diff(k1, n, k2, n)[1] - newcombe_diff(k1, n, k2, n)[0]) / 2
                   for k1, wa in enumerate(a) for k2, wb in enumerate(b) if wa * wb > 1e-12)
    out = {}
    for label, (p1, p2) in {'p=0.5 vs 0.5 (worst)': (0.5, 0.5), **pairs}.items():
        lo, hi = 2, nmax
        if exp_hw(hi, p1, p2) > h:
            out[label] = f'>{nmax}'
            continue
        while lo < hi:
            mid = (lo + hi) // 2
            if exp_hw(mid, p1, p2) <= h:
                hi = mid
            else:
                lo = mid + 1
        out[label] = lo
    return out


def fisher_p(k1, n1, k2, n2):
    """Two-sided Fisher exact p (sum of tables with probability <= observed)."""
    m, N = k1 + k2, n1 + n2
    lo, hi = max(0, m - n2), min(m, n1)
    denom = math.comb(N, m)
    probs = {x: math.comb(n1, x) * math.comb(n2, m - x) / denom for x in range(lo, hi + 1)}
    obs = probs[k1]
    return min(1.0, sum(p for p in probs.values() if p <= obs * (1 + 1e-7)))


def fisher_power(n, p1, p2, alpha):
    a, b = binom_pmf(n, p1), binom_pmf(n, p2)
    return sum(wa * wb for k1, wa in enumerate(a) for k2, wb in enumerate(b)
               if wa * wb > 1e-14 and fisher_p(k1, n, k2, n) <= alpha)


def prob_greater_uniform_prior(k1, n1, k2, n2):
    """P(p1 > p2 | data) with Beta(1,1) priors; closed form for integer posterior parameters."""
    a1, b1, a2, b2 = k1 + 1, n1 - k1 + 1, k2 + 1, n2 - k2 + 1

    def lbeta(x, y):
        return math.lgamma(x) + math.lgamma(y) - math.lgamma(x + y)
    # P(p1 > p2) = sum_{i=0}^{a1-1} B(a2+i, b1+b2) / ((b1+i) B(1+i, b1) B(a2, b2))
    total = 0.0
    for i in range(a1):
        total += math.exp(lbeta(a2 + i, b1 + b2) - math.log(b1 + i) - lbeta(1 + i, b1) - lbeta(a2, b2))
    return total


def bayes_assurance(n, p1, p2, threshold):
    a, b = binom_pmf(n, p1), binom_pmf(n, p2)
    return sum(wa * wb for k1, wa in enumerate(a) for k2, wb in enumerate(b)
               if wa * wb > 1e-14 and prob_greater_uniform_prior(k1, n, k2, n) >= threshold)


def min_n(fn, target, nmax=80):
    for n in range(2, nmax + 1):
        if fn(n) >= target:
            return n
    return f'>{nmax}'


def jeffreys_mean(k, n):
    return round((k + 0.5) / (n + 1), 3)


def power_tables(primary):
    counts = {}
    for cond in CONDITIONS:
        for scen in SCENARIOS:
            rs = [r for r in primary if r['condition'] == cond and r['scenario'] == scen]
            counts[(cond, scen)] = (sum(r['success'] for r in rs), len(rs))
    j = {f'{c}/{s}': jeffreys_mean(*counts[(c, s)]) for c, s in counts}
    pooled = {c: (sum(counts[(c, s)][0] for s in SCENARIOS), sum(counts[(c, s)][1] for s in SCENARIOS))
              for c in CONDITIONS}
    contrasts = {
        'P1 dynamic vs independent (nominal+graspfail, per-scenario arm n; planned stratified test)': {
            'observed_ZC2': f"dynamic {pooled['dynamic'][0]}/{pooled['dynamic'][1]} vs independent "
                            f"{pooled['independent'][0]}/{pooled['independent'][1]}",
            'planning': {
                'ZC2 shrunk (Jeffreys posterior means, pooled)': (jeffreys_mean(*pooled['dynamic']),
                                                                  jeffreys_mean(*pooled['independent'])),
                'conservative': (0.80, 0.30),
                'moderate': (0.80, 0.50),
            },
            'pooled_over_scenarios': True,
        },
        'P2 dynamic vs plan_first under grasp-fail injection': {
            'observed_ZC2': f"dynamic {counts[('dynamic', 'graspfail')][0]}/3 vs plan_first "
                            f"{counts[('plan_first', 'graspfail')][0]}/3",
            'planning': {
                'ZC2 shrunk (Jeffreys posterior means)': (j['dynamic/graspfail'], j['plan_first/graspfail']),
                'conservative': (0.80, 0.30),
                'moderate': (0.80, 0.50),
            },
            'pooled_over_scenarios': False,
        },
        'S1 dynamic vs independent, nominal only (secondary)': {
            'observed_ZC2': f"dynamic {counts[('dynamic', 'nominal')][0]}/3 vs independent "
                            f"{counts[('independent', 'nominal')][0]}/3",
            'planning': {'ZC2 shrunk (Jeffreys posterior means)': (j['dynamic/nominal'], j['independent/nominal']),
                         'conservative': (0.80, 0.30)},
            'pooled_over_scenarios': False,
        },
    }
    power = {}
    for name, spec in contrasts.items():
        entry = {'observed_ZC2': spec['observed_ZC2'], 'rows': []}
        for label, (p1, p2) in spec['planning'].items():
            row = {'planning': label, 'p_dynamic': p1, 'p_other': p2}
            # For P1 the arm size is 2n (both scenarios); fisher on the pooled arm approximates the
            # stratified exact test (slightly optimistic if scenario rates differ strongly).
            mult = 2 if spec['pooled_over_scenarios'] else 1
            for alpha in (0.05, 0.025):
                for target in (0.80, 0.90):
                    n = min_n(lambda n: fisher_power(n * mult, p1, p2, alpha), target)
                    row[f'fisher_two_sided_alpha{alpha}_power{target}_n_per_cell'] = n
            for thr in (0.95, 0.975):
                n = min_n(lambda n: bayes_assurance(n * mult, p1, p2, thr), 0.80)
                row[f'bayes_uniform_prior_P(dyn>other)>={thr}_assurance0.8_n_per_cell'] = n
            entry['rows'].append(row)
        power[name] = entry
    # Power achieved by each tier's n.
    return counts, j, contrasts, power


def tier_power(n, contrasts):
    out = {}
    for name, spec in contrasts.items():
        mult = 2 if spec['pooled_over_scenarios'] else 1
        out[name.split(' ')[0]] = {
            label: {'fisher_alpha0.025_power': round(fisher_power(n * mult, p1, p2, 0.025), 3),
                    'bayes_P>=0.975_assurance': round(bayes_assurance(n * mult, p1, p2, 0.975), 3)}
            for label, (p1, p2) in spec['planning'].items()}
    cp = max((clopper_pearson(k, n)[1] - clopper_pearson(k, n)[0]) / 2 for k in range(n + 1))
    out['per_cell_CP95_worst_half_width'] = round(cp, 3)
    out['per_cell_CP95_half_width_if_all_success'] = round((1 - clopper_pearson(n, n)[0]) / 2, 3)
    wd = max((newcombe_diff(k1, n, k2, n)[1] - newcombe_diff(k1, n, k2, n)[0]) / 2
             for k1 in range(n + 1) for k2 in range(n + 1))
    out['difference_newcombe95_worst_half_width'] = round(wd, 3)
    return out


# ----------------------------------------------------------------------------- seeds

def spawn_perm(arena, seed):
    sys.path.insert(0, str(REPO))
    from sim.zone_arena import episode  # setup RNG only; does not build or step a simulator
    e = episode(arena, seed, goal=GOAL_G8, extra_boxes=EXTRA_G8)
    sp = e['setup_only']['spawns']
    return '-'.join(sorted(sp, key=lambda r: sp[r][1]))  # robots from south to north spawn row


def balanced_seeds(arena, start, count, exclude):
    """Scan seeds upward; each consecutive round of 6 accepted seeds holds every spawn permutation once."""
    assert count % 6 == 0
    perms = ['-'.join(p) for p in itertools.permutations(('r1', 'r2', 'r3'))]
    got, tally, seed = [], {p: 0 for p in perms}, start
    for _ in range(count // 6):
        remaining = set(perms)
        while remaining:
            if seed not in exclude:
                p = spawn_perm(arena, seed)
                if p in remaining:
                    got.append(seed)
                    tally[p] += 1
                    remaining.discard(p)
            seed += 1
    return got, tally


def seed_plan(max_n_wide=24, max_n_open=12):
    used = {a: {s: spawn_perm(a, s) for s in USED_SEEDS} for a in ('zone_wide', 'zone_open')}
    dev, dev_tally = balanced_seeds('zone_wide', 101, 6, set(USED_SEEDS))
    held, held_tally = balanced_seeds('zone_wide', 1001, max_n_wide, set(USED_SEEDS))
    held_open, open_tally = balanced_seeds('zone_open', 2001, max_n_open, set(USED_SEEDS))
    dev_open, dev_open_tally = balanced_seeds('zone_open', 201, 6, set(USED_SEEDS))
    return {
        'status': 'DRAFT proposal; freeze by committing the final protocol before any ZC3 run',
        'rule': ('Spawn permutation = robot IDs ordered by spawn row (south->north) from sim.zone_arena.episode '
                 '(G8 + spare red1/cyan1). Seeds are scanned upward from a fixed start, skipping used seeds '
                 '11-14, and accepted while their permutation is below its quota so the 6 permutations appear '
                 'equally often. Only setup RNG is evaluated; no outcome is looked at.'),
        'used_seeds_spawn_perm': used,
        'development_seeds_zone_wide': {'seeds': dev, 'perm_tally': dev_tally,
                                        'use': 'gate fixtures and any pilot/debug; never analysed as ZC3 results'},
        'heldout_seeds_zone_wide_ordered': {'seeds': held, 'perm_tally': held_tally,
                                            'use': 'tier n uses the first n seeds of this list (prefix is '
                                                   'balanced at n=6,12,18,24)'},
        'heldout_seeds_zone_open_optional': {'seeds': held_open, 'perm_tally': open_tally},
        'development_seeds_zone_open_optional': {'seeds': dev_open, 'perm_tally': dev_open_tally},
        'prefix_balance_zone_wide': {str(n): _tally(held[:n], 'zone_wide') for n in (6, 12, 18, 24)
                                     if n <= len(held)},
    }


def _tally(seeds, arena):
    t = {}
    for s in seeds:
        p = spawn_perm(arena, s)
        t[p] = t.get(p, 0) + 1
    return dict(sorted(t.items()))


def run_order(seeds, order_seed, arena):
    """Blocks = seeds (block order shuffled); the 6 cells are shuffled within each block."""
    rng = random.Random(order_seed)
    blocks = list(seeds)
    rng.shuffle(blocks)
    cells = [(c, s) for c in CONDITIONS for s in SCENARIOS]
    order = []
    for seed in blocks:
        cs = cells[:]
        rng.shuffle(cs)
        order += [{'arena': arena, 'seed': seed, 'condition': c, 'scenario': s} for c, s in cs]
    return order


# ----------------------------------------------------------------------------- budget

def budget(primary, gates, cells, contrasts, seeds):
    per_cell = {}
    for key, c in cells.items():
        per_cell[key] = {'input_tokens_mean': c['input_tokens']['mean'], 'input_tokens_max': c['input_tokens']['max'],
                         'output_tokens_mean': c['output_tokens']['mean'], 'llm_calls_mean': c['llm_calls']['mean'],
                         'driver_wall_s_mean': c['driver_wall_s']['mean'], 'driver_wall_s_max': c['driver_wall_s']['max']}
    six = list(per_cell.values())
    block_in = sum(x['input_tokens_mean'] for x in six)
    block_in_max = sum(x['input_tokens_max'] for x in six)
    block_out = sum(x['output_tokens_mean'] for x in six)
    block_wall = sum(x['driver_wall_s_mean'] for x in six)
    block_wall_max = sum(x['driver_wall_s_max'] for x in six)
    gate_wall = sum(g['driver_wall_s'] for g in gates if g['cohort'] == 'ZC2-gate')
    overhead = 1.15  # lock/handoff, per-run bookkeeping, hashing; not measured

    def tier(name, n_wide, n_open, claims, cannot):
        blocks = n_wide + n_open
        runs = 6 * blocks
        gate_runs = 6 * (1 + (1 if n_open else 0))
        wall_h = (blocks * block_wall + gate_wall * (1 + (1 if n_open else 0))) * overhead / 3600
        wall_h_hi = (blocks * block_wall_max + gate_wall * (1 + (1 if n_open else 0))) * overhead / 3600
        return {
            'tier': name, 'n_per_cell_zone_wide': n_wide, 'n_per_cell_zone_open_optional': n_open,
            'live_llm_runs': runs, 'fixture_gate_runs': gate_runs,
            'input_tokens_estimate': round(blocks * block_in), 'input_tokens_upper': round(blocks * block_in_max),
            'output_tokens_estimate': round(blocks * block_out),
            'model_calls_estimate': round(blocks * sum(x['llm_calls_mean'] for x in six)),
            'wall_hours_estimate': round(wall_h, 1), 'wall_hours_upper': round(wall_h_hi, 1),
            'wall_hours_if_every_run_hits_max_wall_s_3600': float(runs),
            'zone_wide_power_and_precision': tier_power(n_wide, contrasts),
            'can_claim': claims, 'cannot_claim': cannot,
            'zone_wide_seeds': seeds['heldout_seeds_zone_wide_ordered']['seeds'][:n_wide],
            'zone_open_seeds': seeds['heldout_seeds_zone_open_optional']['seeds'][:n_open],
        }
    tiers = [
        tier('small', 6, 0,
             ['zone_wide, G8+spares, gemini-3.8-flash, teacher executor, new held-out seeds with balanced spawn '
              'permutations: P1 (dynamic vs independent, pooled scenarios) if the effect is ZC2-sized '
              '(Fisher alpha 0.025 power ~0.90)',
              'P2 only as a Bayesian statement at the ZC2-sized effect (assurance ~0.82; Fisher power ~0.55)',
              'descriptive SIM time / calls / tokens per cell with n=6'],
             ['conservative (0.8 vs 0.3) or moderate (0.8 vs 0.5) effects',
              'per-cell rate CI half-width <= 0.20 (worst case ~0.38; ~0.23 even at 6/6)',
              'any arena/goal/model generalisation; any RGB-skill or student success']),
        tier('medium', 12, 0,
             ['P1 at the conservative effect 0.8 vs 0.3 (Fisher alpha 0.025 power ~0.88)',
              'P2 at the ZC2-sized effect (power ~0.93); at 0.8 vs 0.3 only Bayesian assurance ~0.77',
              'per-cell rate CI half-width <= 0.20 only when the cell is near 0 or 1 (0.13 at 12/12; worst ~0.29)',
              'SIM-time and cost differences between conditions as descriptive paired estimates'],
             ['P2 at 0.8 vs 0.3 with 80% frequentist power; any moderate (0.8 vs 0.5) effect',
              'worst-case per-cell half-width <= 0.20; difference CI half-width <= 0.20',
              'arena/goal/model generalisation']),
        tier('full', 24, 12,
             ['P1 and P2 at the conservative effect 0.8 vs 0.3 (Fisher alpha 0.025 power ~1.0 / ~0.88)',
              'P1 at the moderate effect 0.8 vs 0.5 with Fisher power ~0.75 (Bayesian assurance ~0.88)',
              'per-cell rate CI half-width <= 0.20 in the worst case by Wilson (Clopper-Pearson worst ~0.21)',
              'optional zone_open replication (12/cell) reported as a separate table, never pooled with zone_wide'],
             ['per-cell half-width <= 0.15 in the worst case (needs 39 Wilson / 47 Clopper-Pearson per cell)',
              'P2 at the moderate effect (needs ~52/cell)', 'difference CI half-width <= 0.20 (needs ~33-44/arm)',
              'model or goal generalisation; student/RGB-skill success; real-time behaviour (synchronous SIM only)']),
    ]
    return {
        'status': 'DRAFT - not frozen. Needs the user budget decision before commit-as-preregistration.',
        'basis': {
            'source': 'ZC2 18 live runs (zone_wide, G8+spare red1/cyan1, seeds 12-14, gemini-3.8-flash) + ZC2 gate fixtures',
            'per_cell_measured': per_cell,
            'one_block_six_cells': {'input_tokens_mean_sum': round(block_in), 'input_tokens_max_sum': block_in_max,
                                    'output_tokens_mean_sum': round(block_out),
                                    'driver_wall_s_mean_sum': round(block_wall, 1),
                                    'driver_wall_s_max_sum': round(block_wall_max, 1)},
            'gate_fixture_wall_s_per_arena': round(gate_wall, 1),
            'overhead_factor_unmeasured': overhead,
            'zone_open_assumption': ('zone_open sends 3 TOP images per request instead of 5; zone_wide per-run '
                                     'tokens/wall are used as a conservative upper bound. independent mode has '
                                     'never run on zone_open, so a zone_open gate is required first.'),
            'price': {'usd_per_million_input_tokens': None, 'usd_per_million_output_tokens': None,
                      'note': 'provider billing unavailable in team.json (cost_usd null); fill in to convert'},
            'execution': 'Mac, synchronous SIM, one run at a time, inside scripts/agent_lock.py (owner claude)',
        },
        'tiers': tiers,
    }


# ----------------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--outputs-root', type=Path, default=Path('/Users/changmin/projects/ugrp/outputs'))
    args = ap.parse_args()
    primary, gates, supp, hashes, mismatches = collect(args.outputs_root)
    cells = cell_summary(primary)
    measured = {
        'status': 'DRAFT analysis input for ZC3 preregistration; ZC2 is development evidence, not ZC3 data',
        'outputs_root': str(args.outputs_root) + ' (local only, not a remote backup)',
        'cross_check_vs_committed_record': {'mismatches': mismatches, 'ok': not mismatches},
        'definitions': {
            'control_end_sim_s': 'result.json sim_end_s (SIM clock when control ended)',
            'makespan_sim_s': 'first job issue to last job end (result.json makespan_sim_s)',
            'wall_s': 'result.json wall_s (episode loop incl. model latency)',
            'driver_wall_s': 'loads.jsonl start_utc->end_utc (whole process incl. scene build/replay write)',
            'model_latency_sum_s': 'sum of team.json calls[].latency_ms',
        },
        'zc2_runs': primary,
        'zc2_cells': cells,
        'gate_fixture_runs': gates,
        'supplementary_cost_reference': {
            'note': 'plan_first/dynamic only, 1 run per condition, other goals/arenas/sources; never pooled',
            'runs': supp,
        },
        'input_sha256': hashes,
    }
    counts, jmeans, contrasts, power = power_tables(primary)
    precision = {
        'per_cell_rate': {str(h): single_rate_n(h, (0.5, 0.8, 0.9, 0.95)) for h in (0.15, 0.20)},
        'difference_newcombe_per_arm_n': {str(h): diff_n(h, {'0.875 vs 0.125': (0.875, 0.125),
                                                             '0.8 vs 0.3': (0.8, 0.3), '0.8 vs 0.5': (0.8, 0.5)})
                                          for h in (0.15, 0.20)},
    }
    zc2_now = {}
    for key, (a, b) in {'P1 pooled 6/6 vs 1/6': ((6, 6), (1, 6)), 'P2 3/3 vs 0/3': ((3, 3), (0, 3)),
                        'S1 3/3 vs 0/3': ((3, 3), (0, 3))}.items():
        zc2_now[key] = {'fisher_two_sided_p': round(fisher_p(a[0], a[1], b[0], b[1]), 4),
                        'P(p_dyn>p_other) uniform prior': round(prob_greater_uniform_prior(a[0], a[1], b[0], b[1]), 4),
                        'diff_newcombe95': [round(x, 3) for x in newcombe_diff(a[0], a[1], b[0], b[1])]}
    power_out = {
        'status': 'DRAFT',
        'assumptions': [
            'One run per (seed, condition, scenario); seeds are independent draws, so each cell outcome is '
            'Bernoulli with the seed-population-average rate (no within-cell beta-binomial overdispersion).',
            'All 6 cells share the same seed list (paired blocks). Power below ignores the pairing '
            '(unpaired Fisher / independent Beta posteriors), which is conservative when seed difficulty is '
            'shared across conditions.',
            'Two primary contrasts; family-wise alpha 0.05 via Bonferroni/Holm -> alpha 0.025 each.',
            'P1 pools nominal and grasp-fail (arm size 2n) and is planned as an exact stratified '
            '(scenario) test; the Fisher-on-pooled approximation is used for sizing.',
            'Bayesian: independent Beta(1,1) priors; decision P(p_dyn > p_other | data) >= threshold; '
            'assurance = probability of reaching that decision under the planning rates.',
            'Planning rates: ZC2 observed 3/3 and 0/3 are extreme, so they are shrunk with Jeffreys posterior '
            'means; conservative and moderate alternatives are also listed.',
        ],
        'zc2_observed_counts': {f'{c}/{s}': f'{k}/{n}' for (c, s), (k, n) in counts.items()},
        'zc2_jeffreys_means': jmeans,
        'zc2_as_if_tested_now': zc2_now,
        'precision': precision,
        'power': power,
        'secondary_descriptive': {
            'note': 'SIM time, calls, tokens: report per-cell mean/SD/min/max and paired (same seed) differences '
                    'with bootstrap CIs over seeds; no hypothesis test is primary.',
            'sd_by_cell_ZC2': {k: {'makespan_sim_s_sd': v['makespan_sim_s']['sd'],
                                   'llm_calls_sd': v['llm_calls']['sd'],
                                   'input_tokens_sd': v['input_tokens']['sd']} for k, v in cells.items()},
            'n_for_mean_makespan_half_width_10s': {
                k: math.ceil((1.96 * v['makespan_sim_s']['sd'] / 10) ** 2) if v['makespan_sim_s']['sd'] else None
                for k, v in cells.items()},
        },
    }
    seeds = seed_plan()
    seeds['proposed_run_order_seed'] = 20260925
    seeds['proposed_run_order_medium_example'] = run_order(
        seeds['heldout_seeds_zone_wide_ordered']['seeds'][:12], 20260925, 'zone_wide')
    bud = budget(primary, gates, cells, contrasts, seeds)
    (HERE / 'measured_runs.json').write_text(json.dumps(measured, indent=1, ensure_ascii=False) + '\n')
    (HERE / 'power.json').write_text(json.dumps(power_out, indent=1, ensure_ascii=False) + '\n')
    (HERE / 'seed_plan.json').write_text(json.dumps(seeds, indent=1, ensure_ascii=False) + '\n')
    (EXP / 'budget.json').write_text(json.dumps(bud, indent=1, ensure_ascii=False) + '\n')
    print('mismatches vs committed record:', mismatches or 'none')
    print('wrote measured_runs.json power.json seed_plan.json ../budget.json')


if __name__ == '__main__':
    main()
