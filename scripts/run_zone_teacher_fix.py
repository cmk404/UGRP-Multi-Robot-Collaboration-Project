"""Zone dispatch protocol v2 with the GT TEACHER blocker fixes (zone teacher fix, 2026-09-26).

TEACHER FEASIBILITY ONLY. Every motion is the ground-truth teacher (drive, IK,
real gripper, weld OFF); its deliveries are never a robot, RGB-skill or
student success, and it is never the study executor.

Same arguments as ``scripts.run_zone_dispatch`` (protocol v2 only), plus:

- ``--teacher-fix``: JSON switch overrides for ``scripts.zone_team_teacher_fix``
  (``b3_claim_yield``, ``b4_return_setdown``; default all on);
- ``--feasibility-gate enforce|report``: before any physics, the scenario config
  is checked with ``harness.zone_teacher_gate`` (B2/B8). ``enforce`` (default)
  refuses ``infeasible`` and ``order_constrained`` scenarios (exit 3, writes
  ``<output>/feasibility-gate.json`` and nothing else); ``report`` runs anyway;
- ``--perception-profile`` also accepts ``top_cargo_v2_track`` (B5 label tracking).

PR #169's runner, dispatcher, conditions and ModeLoops are used unchanged; only
the executor class is swapped (``fix_run_class``). Example::

  OMP_NUM_THREADS=1 ... python -m scripts.run_zone_teacher_fix --output outputs/x --variant zone_wide_two_doors \\
      --coordination dynamic --mode fixture --goal '{...}' --seed 21 --contact-profile cargo_noslip_v1 \\
      --perception-profile top_cargo_v2_track --record-replay
"""
from __future__ import annotations

import json
import sys

from scripts import run_zone_dispatch as rzd

PERCEPTION_CHOICES = ('top_cargo_v1', 'top_cargo_v2', 'top_cargo_v2_track')
GATE_REFUSES = ('infeasible', 'order_constrained')


def parser():
    p = rzd.parser()
    p.description = 'zone dispatch v2 with GT TEACHER blocker fixes (teacher feasibility only)'
    for action in p._actions:
        if action.dest == 'perception_profile':
            action.choices = PERCEPTION_CHOICES
    p.add_argument('--teacher-fix', default='{}', help='JSON switch overrides for scripts.zone_team_teacher_fix')
    p.add_argument('--feasibility-gate', choices=('enforce', 'report'), default='enforce')
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    if args.variant in rzd.RETIRED_VARIANTS:
        raise SystemExit(f'{args.variant} is retired; the teacher fix runner has no retired-map path')
    protocol, goal = rzd.protocol_of(args)
    if protocol != 'v2':
        raise SystemExit('the teacher fix runner is protocol v2 only')
    if args.fixture_plan or args.inject_grasp_failure:
        raise SystemExit('--fixture-plan and --inject-grasp-failure are protocol v1 options')
    from harness.zone_mixed_episode import mixed_episode
    from harness.zone_teacher_gate import team_route_feasibility
    from scripts import zone_dispatch_v2
    from scripts.zone_team_teacher_fix import DEFAULT_SWITCHES, fix_run_class
    switches = {**DEFAULT_SWITCHES, **json.loads(args.teacher_fix)}
    config = mixed_episode(args.variant, args.seed, goal=goal, extra_boxes=json.loads(args.extra_boxes),
                           extra_cargo=json.loads(args.extra_cargo), colour_only_ok=True)
    gate = team_route_feasibility(config)
    gate['mode'] = args.feasibility_gate
    print('FEASIBILITY_GATE ' + json.dumps({'verdict': gate['verdict'], 'items': [
        {k: r.get(k) for k in ('item_id', 'zone', 'verdict', 'blocking_solo_items')} for r in gate['items']]}),
        flush=True)
    if args.feasibility_gate == 'enforce' and gate['verdict'] in GATE_REFUSES:
        args.output.mkdir(parents=True, exist_ok=False)
        rzd.write(args.output/'feasibility-gate.json', gate | {'run_started': False})
        print(f"REFUSED: scenario is {gate['verdict']} for the teacher (no physics run)", flush=True)
        return 3
    zone_dispatch_v2.ZoneRunV2 = fix_run_class(switches)
    result = zone_dispatch_v2.run_v2(args, goal)
    rzd.write(args.output/'feasibility-gate.json', gate | {'run_started': True})
    return 0 if result.get('error') is None else 1


if __name__ == '__main__':
    sys.exit(main())
