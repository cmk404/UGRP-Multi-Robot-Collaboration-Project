"""Summarise the plan-guidance cohort from saved run outputs only.

Idle and overlap are derived from issued-command timestamps (control-side
records), not referee state. Physical success comes from the separate
referee output. A missing file is reported, never filled with a default.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import statistics
from pathlib import Path


def span(commands, rid, stages):
    ts = [c['issued_at_s'] for c in commands.get(rid, [])
          if c.get('issued_at_s') is not None and (c.get('stage') in stages or
                                                   any(c.get('stage', '').startswith(s) for s in stages if s.endswith('_')))]
    return (min(ts), max(ts)) if ts else None


def run_row(path: Path):
    row = {'run': path.name}
    result_path = path / 'result.json'
    if not result_path.exists():
        row['missing'] = 'result.json'
        return row
    result = json.loads(result_path.read_text())
    row.update(condition=(result.get('plan_guidance') or {}).get('mode'),
               error=result.get('error'), plan_committed=result.get('plan_committed'),
               protocol_complete=result.get('protocol_complete'),
               physical_success=result.get('physical_success'),
               control_end_sim_s=(result.get('timing') or {}).get('control_end_sim_s'),
               llm_calls=result.get('llm_calls'), usage=result.get('usage'),
               result_sha256=hashlib.sha256(result_path.read_bytes()).hexdigest())
    plan = result.get('plan')
    if plan:
        tasks = {t['object']: t for t in plan['tasks']}
        row['plan'] = {'dock': plan['dock'],
                       'beam': [tasks['beam']['participants'], tasks['beam']['route'], tasks['beam']['after']],
                       'box': [tasks['box']['participants'], tasks['box']['route'], tasks['box']['after']]}
        row['has_dependency'] = any(t['after'] for t in plan['tasks'])
        row['dependency'] = ('box_first' if tasks['beam']['after'] else
                             'beam_first' if tasks['box']['after'] else 'none')
    row['overlap_selection'] = result.get('overlap_selection')
    team_path = path / 'team' / 'team.json'
    if team_path.exists():
        team = json.loads(team_path.read_text())
        events = team.get('agreement_events', [])
        row['agreement_events'] = [e['event'] for e in events]
        row['negotiation_turns'] = len(team.get('rounds', []))

        def dep(plan):
            t = {x['object']: x for x in plan['tasks']}
            return 'box_first' if t['beam']['after'] else 'beam_first' if t['box']['after'] else 'none'
        row['proposed_dependencies'] = [dep(e['plan']) for e in events if e['event'] == 'PROPOSED']
    commands_path = path / 'issued-commands.json'
    if plan and commands_path.exists():
        commands = json.loads(commands_path.read_text())
        pair = tasks['beam']['participants'][0]
        solo = tasks['box']['participants'][0]
        approach = span(commands, pair, ('APPROACH',))
        grasp = span(commands, pair, ('grasp_',))
        beam_carry = span(commands, pair, ('TRANSIT',))
        box_carry = span(commands, solo, ('TRANSIT',))
        if approach and grasp:
            row['pair_idle_after_approach_sim_s'] = round(grasp[0] - approach[1], 1)
        if beam_carry and box_carry:
            row['carry_overlap_sim_s'] = round(max(0., min(beam_carry[1], box_carry[1]) - max(beam_carry[0], box_carry[0])), 1)
    previews_path = path / 'plan-previews.json'
    if previews_path.exists() and plan:
        from harness.three_robot_plan import digest
        shown = json.loads(previews_path.read_text())
        committed = [p['preview'] for p in shown if p['preview'].get('plan_hash') == digest(plan)]
        row['previews_shown'] = len(shown)
        row['preview_distinct_plans'] = len({p['preview'].get('plan_hash') for p in shown})
        if committed:
            row['committed_preview_total_sim_s'] = committed[0].get('estimated_total_sim_s')
    return row


def summarise(rows):
    out = {}
    for cond in ('legacy', 'objective', 'objective_preview'):
        group = [r for r in rows if r.get('condition') == cond]
        if not group:
            continue
        done = [r for r in group if r.get('physical_success') and r.get('protocol_complete')]
        times = [r['control_end_sim_s'] for r in done if r.get('control_end_sim_s')]
        out[cond] = {
            'runs': len(group),
            'committed': sum(bool(r.get('plan_committed')) for r in group),
            'mission_success': len(done),
            'dependency_choice': {k: sum(r.get('dependency') == k for r in group)
                                  for k in ('none', 'beam_first', 'box_first')},
            'concurrent_execution': sum(bool((r.get('overlap_selection') or {}).get('enabled')) for r in group),
            'success_sim_s_median': statistics.median(times) if times else None,
            'success_sim_s_all': times,
            'pair_idle_median': statistics.median([r['pair_idle_after_approach_sim_s'] for r in group
                                                   if 'pair_idle_after_approach_sim_s' in r] or [None]),
            'llm_calls_median': statistics.median([r['llm_calls'] for r in group if r.get('llm_calls')] or [None]),
            'prompt_tokens_median': statistics.median([(r.get('usage') or {}).get('prompt_tokens') for r in group
                                                       if (r.get('usage') or {}).get('prompt_tokens')] or [None]),
            'errors': [r['error'] for r in group if r.get('error')],
        }
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cohort', type=Path)
    parser.add_argument('--write', type=Path)
    args = parser.parse_args(argv)
    rows = [run_row(p) for p in sorted(args.cohort.iterdir())
            if p.is_dir() and not p.name.endswith('-grasp-models')]
    report = {'cohort': str(args.cohort), 'runs': rows, 'summary': summarise(rows)}
    text = json.dumps(report, indent=1, ensure_ascii=False)
    if args.write:
        args.write.write_text(text + '\n')
    print(json.dumps(report['summary'], indent=1, ensure_ascii=False))


if __name__ == '__main__':
    main()
