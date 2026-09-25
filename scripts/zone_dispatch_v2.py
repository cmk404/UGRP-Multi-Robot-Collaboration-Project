"""Zone dispatch, protocol v2: mixed cargo goals, role claims, team carries (zone team A2).

Entered from ``scripts.run_zone_dispatch`` when the goal names catalogue cargo
(goal v2) or ``--protocol v2`` is given; colour-only goals otherwise keep the
v1 path unchanged. What differs from v1:

- scene: ``CargoZoneScene`` (PR #164 cargo path) with the goal's cargo placed
  by ``harness.zone_mixed_episode``; contact profile ``cargo_noslip_v1`` by default;
- robot input: TOP RGB labels and view from ``top_cargo_v1`` (+ its box path),
  the static task text (kinds, carriers, roles, landing areas, team rule) that
  is identical in every mode (``harness.zone_protocol_v2``);
- claims ``{item, zone, role}`` in all three modes; the host never picks teammates;
- executor ``scripts.zone_team_teacher.ZoneTeamExecutor`` (GT TEACHER; every job a
  TeamJob; one rendezvous rule; L1 fixed);
- output: ``referee_v2`` (full footprint, evaluation only), per-item outcomes,
  formation waits, door waits, carry pauses, slip and drops, ``eq_active`` max.

Receipts are still teacher-ground-truth receipts (audit L4 unresolved).
"""
from __future__ import annotations

import copy
import json
import os
import subprocess
import time
import traceback
from pathlib import Path

from harness import zone_protocol_v2 as zp2
from harness.three_robot_plan import ROBOTS, TeamAgreement, validate_plan_reply
from harness.zone_goal_v2 import referee_v2
from harness.zone_mixed_episode import item_table, mixed_episode, scene_for
from harness.zone_perception_v2 import detect_items, goal_met, label_items, observe_items
from harness.zone_team_jobs import (RECEIPT_FINISHED, check_dynamic_claims, check_independent_claims,
                                    normalize_claim, validate_team_plan)
from scripts.run_zone_dispatch import ZoneRun, write
from sim.zone_arena import top_views

SCHEMA = 'ugrp.zone_dispatch_result.v2'
PROTOCOL = 'zone_protocol_v2'
DEFAULT_MAX_SIM_S = 1800.
STALL_TURNS = 2
SOLO_REASK_S = 10.


def _load():
    try:
        return [round(v, 2) for v in os.getloadavg()]
    except OSError:
        return None


class ZoneRunV2(ZoneRun):
    """Cargo zone scene, team executor and RGB capture on one physics clock."""
    def __init__(self, config, output, *, contact_profile, record_replay=False, inject=None):
        from sim.camera_robot_port import CameraRobotPort
        from sim.multi_masterpi_production import MultiMasterPiProductionV2
        from scripts.zone_team_teacher import ZoneTeamExecutor
        self.config, self.out = config, Path(output)
        self.out.mkdir(parents=True, exist_ok=False)
        (self.out/'rgb').mkdir()
        self.definition = scene_for(config, contact_profile)
        self.world = MultiMasterPiProductionV2(seed=config['seed'], width=960, height=720, render=True,
            warehouse_layout=self.definition.engine_layout, warehouse_cargo_ids=None,
            xml_transform=self.definition.transform)
        self.definition.setup(self.world)
        (self.out/'scene.xml').write_text(self.world.scene_xml)
        self.ports = {r: CameraRobotPort(self.world, r, allow_reverse=True, allow_mecanum=True) for r in ROBOTS}
        self.events = []
        self.items = item_table(config)
        self.executor = ZoneTeamExecutor(self.world, self.ports, config['static_map'], self.items, config['goal'],
                                         self._log, inject=inject)
        self.replay = None
        if record_replay:
            from scripts.zone_replay import ZoneReplayRecorder, arena_view
            self.replay = ZoneReplayRecorder(self.world, self.out, view=arena_view(config['static_map']['bounds_m']))
        self.count = 0
        self.eq_active_max = 0
        self.neq = int(self.world.model.neq)

    def step(self, seconds):
        dt = float(self.world.model.opt.timestep)
        for _ in range(max(1, round(seconds/dt))):
            now = self.time()
            self.executor.tick(now)
            for port in self.ports.values():
                port.tick(now)
            self.world._physics_step_for(self.world.controllers['r1'])
            if self.neq and self.world.data.eq_active.any():
                self.eq_active_max = 1
            if self.replay:
                self.replay.sample(' | '.join(f'{r}:{t.phase}' for r, t in self.executor.robots.items()))

    def referee_items(self):
        """Evaluation only: final item poses, lowest points, tilt and finger contact."""
        ex = self.executor
        return {iid: {'kind': it['kind'], 'pose': [round(v, 4) for v in ex.item_pose(iid)],
                      'min_z_m': round(ex.item_min_z(iid), 4), 'tilt_deg': round(ex.item_tilt_deg(iid), 2),
                      'held': ex.item_held(iid)}
                for iid, it in self.items.items()}


def _issue(zone, rid, claim, labels, own_jobs, active):
    zone.executor.claim(rid, claim, labels, zone.time())
    active[rid] = claim
    own_jobs[rid].append({'item': claim.item, 'zone': claim.zone, 'role': claim.role, 'status': 'issued',
                          'issued_at_sim_s': round(zone.time(), 2)})
    print(f'CLAIM {rid} {claim.item}/{claim.role} -> {claim.zone}', flush=True)


def run_v2(args, goal):
    if subprocess.check_output(['git', 'status', '--porcelain'], cwd=Path(__file__).resolve().parents[1],
                               text=True).strip():
        raise RuntimeError('commit and freeze source before a trial')
    root = Path(__file__).resolve().parents[1]
    config = mixed_episode(args.variant, args.seed, goal=goal, extra_boxes=json.loads(args.extra_boxes),
                           extra_cargo=json.loads(args.extra_cargo), colour_only_ok=True)
    goal = config['goal']
    profile = args.contact_profile or 'cargo_noslip_v1'
    config['contact_solver_profile'] = profile
    source = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip()
    switches = zp2.CONDITIONS[args.coordination]
    max_sim_s = args.max_sim_s if args.max_sim_s is not None else DEFAULT_MAX_SIM_S
    result = {'schema': SCHEMA, 'protocol': PROTOCOL, 'source_sha': source, 'coordination': args.coordination,
              'condition_switches': switches,
              'executor': ('ground-truth TEACHER team executor (scripts.zone_team_teacher; drive + calibrated IK + '
                           'real gripper, weld OFF); every job a TeamJob, one rendezvous rule'),
              'claim_scope': 'teacher-executor condition: coordination/team metrics, not RGB-skill or student success',
              'config': {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(args).items()}
              | {'max_sim_s_effective': max_sim_s, 'contact_profile_effective': profile},
              'goal': goal, 'phase': 'SETUP', 'error': None, 'goal_met_rgb': False,
              'load_avg': {'start': _load()}}
    started = time.monotonic()
    inject = None
    if args.inject_team_grasp_failure:
        inject = {'kind': None if args.inject_team_grasp_failure == 'any' else args.inject_team_grasp_failure,
                  'min_carriers': 2}
    zone = ZoneRunV2(config, args.output, contact_profile=profile, record_replay=args.record_replay, inject=inject)
    write(args.output/'episode-setup-only.json', config)
    result['scene'] = {'scene_xml_sha256': zone.definition.manifest.get('scene_xml_sha256'),
                       'cargo_contact_profile': zone.definition.manifest.get('cargo_contact_profile'),
                       'cargo': zone.definition.manifest.get('cargo'), 'neq': zone.neq}
    from scripts.three_robot_runtime import ThreeRobotRuntime
    task = zp2.actor_task_v2(config['static_map'], goal, allow_colour_only=True)
    run_id = 'zone2-' + source[:8] + '-' + str(int(time.time()))[-6:]
    labels = {}
    agreement = TeamAgreement(run_id, plan_validator=lambda plan: validate_team_plan(plan, goal, labels))
    team = ThreeRobotRuntime(args.output/'team', run_id=run_id, mode=args.mode, agreement=agreement,
                             model=args.model, max_wall_s=args.max_wall_s, request_timeout=60., max_tokens=1400)
    views = top_views(config['static_map'])
    ex = zone.executor
    active, own_jobs, finished, failed = {}, {r: [] for r in ROBOTS}, [], []
    queues = {r: [] for r in ROBOTS}
    stats = {'claim_rounds': 0, 'collisions': 0, 'invalid_claims': 0, 'plan_turns': 0, 'done_robots': [],
             'stalled_turns': 0}
    solo = {'next_ask': {r: 0. for r in ROBOTS}, 'last_ask': {r: -1. for r in ROBOTS},
            'answer': {r: None for r in ROBOTS}, 'last_end': 0., 'own_turns': {r: 0 for r in ROBOTS}}
    stalled = False
    motion_started = None
    try:
        zone.step(.5)
        frames, tops = zone.capture('start')
        labels.update(label_items(detect_items(tops, config['static_map']), config['static_map']))
        write(args.output/'item-labels.json', labels)
        result['labels'] = {k: v['kind'] for k, v in labels.items()}
        write(args.output/'task.json', task)

        def delivered():
            return [{'zone': ex.ledger.jobs[j].zone, 'kind': ex.ledger.jobs[j].kind}
                    for j in ex.ledger.delivered_items.values()]

        def board():
            if not switches['peer_board']:
                return None
            return {'active': {r: {'item': c.item, 'zone': c.zone, 'role': c.role} for r, c in active.items()},
                    'finished_reports': copy.deepcopy(finished[-12:]), 'stopped_reports': copy.deepcopy(failed[-6:])}

        def collect_done():
            for c in ex.pop_ended():
                rid = c.rid
                active.pop(rid, None)
                report = {'robot': rid, 'item': c.role_claim.item, 'zone': c.role_claim.zone,
                          'role': c.role_claim.role, 'executor_receipt': c.receipt,
                          'sim_time_s': round(zone.time(), 2)}
                own_jobs[rid][-1]['status'] = c.receipt
                (finished if c.receipt == RECEIPT_FINISHED else failed).append(report)
                if switches['wake_on_peer_job_end']:
                    stats['done_robots'] = []
                solo['last_end'] = zone.time()
                if args.coordination == 'independent' and c.receipt != RECEIPT_FINISHED:
                    solo['next_ask'][rid] = zone.time() + SOLO_REASK_S
                zone._log('claim_end', rid, zone.time(), outcome=c.outcome, item=c.item_id, receipt=c.receipt)

        if args.coordination == 'plan_first':
            result['phase'] = 'NEGOTIATE'
            view = observe_items(tops, config['static_map'], labels)
            for turn in range(args.planning_rounds):
                frames, tops = zone.capture(f'plan-{turn}')
                context = agreement.context()
                ctx = {r: zp2.context(r, labels=labels, view=view, own_jobs=own_jobs[r],
                                      inbox=team.inbox[r] if switches['peer_messages'] else None) for r in ROBOTS}
                def build(rid, request_id, ctx=ctx, context=context, frames=frames):
                    return zp2.build_request('plan_first', rid, request_id=request_id, task=task, frame=frames[rid],
                                             ctx=ctx[rid], views=views, agreement=context)
                def fixture(rid, request_id, context=context):
                    return zp2.fixture_plan_reply(request_id, context, goal, labels)
                replies = team.ask(ROBOTS, build,
                    lambda raw, rq, c=context: validate_plan_reply(raw, rq, c, plan_validator=agreement.plan_validator),
                    fixture, phase='plan', turn=turn, sim_time=zone.time(),
                    recipients=None if switches['peer_messages'] else [])
                stats['plan_turns'] += 1
                if agreement.receive(replies, turn):
                    break
            if agreement.committed is None:
                raise RuntimeError('no plan agreed within the planning rounds')
            write(args.output/'committed-plan.json', agreement.committed)
            plan = validate_team_plan(agreement.committed['plan'], goal, labels)
            for rid in ROBOTS:
                queues[rid] = [normalize_claim(rid, job, labels) for job in plan['assignments'][rid]]
            print('PLAN COMMITTED ' + agreement.committed['plan_hash'], flush=True)
        result['phase'] = 'EXECUTE'
        motion_started = zone.time()
        turn = 0
        next_progress = motion_started
        while zone.time() - motion_started < max_sim_s:
            if zone.time() >= next_progress:
                next_progress += 30.
                print('PROGRESS ' + json.dumps({
                    't': round(zone.time() - motion_started, 1),
                    'robots': {r: t.phase for r, t in ex.robots.items()},
                    'jobs': {j: c.job.state for j, c in ex.carries.items() if not c.job.terminal},
                    'delivered': ex.ledger.delivered}), flush=True)
            collect_done()
            idle = [r for r in ROBOTS if r not in active and not ex.robots[r].busy]
            if args.coordination == 'plan_first':
                for rid in idle:
                    if queues[rid]:
                        _issue(zone, rid, queues[rid].pop(0), labels, own_jobs, active)
                if not active and not any(queues.values()) and not any(r.busy for r in ex.robots.values()):
                    break
            elif args.coordination == 'independent':
                due = [r for r in idle if solo['next_ask'][r] <= zone.time()]
                if due:
                    turn += 1
                    decided = independent_round(zone, team, task, labels, goal, due, own_jobs, stats, turn,
                                                solo['own_turns'], views)
                    for rid in due:
                        solo['last_ask'][rid] = zone.time()
                        if rid in decided['accepted']:
                            _issue(zone, rid, decided['accepted'][rid], labels, own_jobs, active)
                            solo['answer'][rid] = 'job'
                        else:
                            solo['answer'][rid] = 'null' if rid in decided['idle'] else 'invalid'
                            solo['next_ask'][rid] = zone.time() + SOLO_REASK_S
                if not active and all(solo['answer'][r] == 'null' and solo['last_ask'][r] >= solo['last_end']
                                      for r in ROBOTS) and not any(r.busy for r in ex.robots.values()):
                    break
            else:
                waiting = [r for r in idle if r not in stats['done_robots']]
                if waiting:
                    turn += 1
                    decided = dynamic_round(zone, team, task, labels, goal, waiting, active, own_jobs, board,
                                            stats, turn, delivered(), switches, views)
                    for rid, claim in decided['accepted'].items():
                        _issue(zone, rid, claim, labels, own_jobs, active)
                    for rid in waiting:
                        if rid not in decided['accepted']:
                            stats['done_robots'].append(rid)
                    if not active and set(stats['done_robots']) >= set(idle):
                        if not decided['idle_all'] and stats['stalled_turns'] < STALL_TURNS:
                            stats['stalled_turns'] += 1
                            stats['done_robots'] = []
                        else:
                            if not decided['idle_all']:
                                stalled = True
                            break
                elif not active and not any(r.busy for r in ex.robots.values()):
                    break
            zone.step(.5)
        control_end = zone.time()
        result['phase'] = ('STALLED' if stalled else 'FINISHED' if control_end - motion_started < max_sim_s
                           else 'SIM_BUDGET')
        result['control_end_sim_s'] = round(control_end - motion_started, 2)
        zone.step(1.)
        _, tops = zone.capture('final')
        result['final_rgb_view'] = observe_items(tops, config['static_map'], labels)
        result['goal_met_rgb'] = goal_met(goal, result['final_rgb_view'])
        result['makespan_sim_s'] = round(control_end - motion_started, 2)
    except Exception as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
        (args.output/'exception.txt').write_text(traceback.format_exc())
    finally:
        team.close(zone.time())
        items = zone.referee_items()
        result['referee_v2'] = referee_v2(goal, config['static_map'], items)
        result['physical_success_teacher_condition'] = result['referee_v2']['goal_met']
        result['success'] = result['physical_success_teacher_condition']
        result['sim_s'] = result.get('makespan_sim_s')
        result['max_sim_s'] = max_sim_s
        result['scope'] = ('zone benchmark protocol v2, TEACHER team executor (ground truth drive/IK, real gripper, '
                           'weld off): success is the referee_v2 goal count, not an RGB-skill or student result')
        result['sim_end_s'] = round(zone.time(), 2)
        result['wall_s'] = round(time.monotonic() - started, 2)
        result['eq_active_max'] = zone.eq_active_max
        result['neq'] = zone.neq
        result['jobs'] = own_jobs
        result['finished_reports'], result['stopped_reports'] = finished, failed
        result['coordination_stats'] = stats
        result['team_executor'] = ex.record()
        result['items'] = item_outcomes(result['referee_v2'], ex, items, labels)
        result['door_waits'] = [e for e in zone.events if e['event'] == 'passage_wait']
        result['llm_calls'] = sum(1 for c in team.calls if c.get('model') != 'scripted-fixture-not-llm')
        result['usage'] = {k: sum((c.get('usage') or {}).get(k, 0) for c in team.calls)
                           for k in ('prompt_tokens', 'completion_tokens', 'total_tokens')}
        result['replay'] = zone.close()
        result['load_avg']['end'] = _load()
        write(args.output/'teacher-events.json', zone.events)
        write(args.output/'result.json', result)
        print(json.dumps({k: result.get(k) for k in ('phase', 'error', 'goal_met_rgb',
              'physical_success_teacher_condition', 'makespan_sim_s', 'llm_calls', 'eq_active_max', 'wall_s')}),
              flush=True)
    return result


def item_outcomes(referee, ex, items, labels):
    """Per physical item: kind, referee_v2 result and every job attempt (evaluation output)."""
    jobs = {}
    for rec in ex.record()['jobs']:
        jobs.setdefault(rec['item_id'], []).append({k: rec.get(k) for k in (
            'job_id', 'state', 'participants', 'role_by_robot', 'formation_wait_s', 'commit_sim_s', 'end_sim_s',
            'failures', 'pauses', 'slip_mm', 'drops', 'max_track_err_m', 'hold_min_z_m', 'inject')})
    claims = {}
    for c in ex.claim_log:
        claims.setdefault(c.item_id, []).append({'robot': c.rid, 'label': c.role_claim.item, 'outcome': c.outcome,
                                                 'zone': c.role_claim.zone})
    return {iid: {'kind': it['kind'], 'referee': referee['items'].get(iid), 'jobs': jobs.get(iid, []),
                  'claims': claims.get(iid, [])} for iid, it in items.items()}


def dynamic_round(zone, team, task, labels, goal, waiting, active, own_jobs, board, stats, turn, delivered,
                  switches, views):
    """Idle robots claim one role each; the host checks against the goal, RGB and active peer claims."""
    accepted, idle = {}, []
    askers, extra = list(waiting), {}
    retry = []
    for attempt in range(3):
        frames, tops = zone.capture(f'claim-{turn}-{attempt}', robots=askers)
        view = observe_items(tops, zone.config['static_map'], labels)
        pending = {**active, **accepted}
        brd = board()
        if brd is not None:
            brd = brd | {'active': {k: {'item': c.item, 'zone': c.zone, 'role': c.role} for k, c in pending.items()}}
        ctx = {r: zp2.context(r, labels=labels, view=view, own_jobs=own_jobs[r], board=brd,
                              inbox=team.inbox[r] if switches['peer_messages'] else None, extra=extra.get(r))
               for r in askers}
        def build(rid, request_id, ctx=ctx, frames=frames):
            return zp2.build_request('dynamic', rid, request_id=request_id, task=task, frame=frames[rid],
                                     ctx=ctx[rid], views=views)
        def fixture(rid, request_id, view=view, pending=pending, askers=tuple(askers)):
            return zp2.fixture_claim(rid, request_id, goal, labels, view, pending, askers, delivered)
        replies = team.ask(askers, build, zp2.validate_claim_reply, fixture, phase=f'claim-{turn}-{attempt}',
                           turn=turn, sim_time=zone.time(), recipients=None if switches['peer_messages'] else [])
        stats['claim_rounds'] += 1
        raw = {r: (v['claim'] if v else None) for r, v in replies.items()}
        if switches['host_arbitration']:
            checked = check_dynamic_claims(raw, goal=goal, labels=labels, view=view, active=pending,
                                           finished=delivered)
        else:
            checked = {**check_independent_claims(raw, goal=goal, labels=labels, view=view), 'collisions': []}
        accepted.update(checked['accepted'])
        idle += checked['idle']
        stats['collisions'] += len(checked['collisions'])
        stats['invalid_claims'] += len(checked['invalid'])
        team.event('CLAIMS_CHECKED', zone.time(), turn=turn, attempt=attempt,
                   accepted={r: c.record() for r, c in checked['accepted'].items()}, invalid=checked['invalid'],
                   idle=checked['idle'], collisions=checked['collisions'])
        retry = sorted({r for c in checked['collisions'] for r in c['robots'] if r in askers} | set(checked['invalid']))
        retry = [r for r in retry if r not in accepted]
        if not retry:
            break
        askers = retry
        extra = {r: {'conflict': [c for c in checked['collisions'] if r in c['robots']],
                     'invalid_reason': checked['invalid'].get(r)} for r in retry}
    return {'accepted': accepted, 'idle': idle, 'unresolved': retry,
            'idle_all': not retry and set(waiting) - set(accepted) <= set(idle)}


def independent_round(zone, team, task, labels, goal, askers, own_jobs, stats, turn, own_turns, views):
    """No communication: each claim alone against the robot's own view; one retry for invalid claims."""
    accepted, idle, reasons = {}, [], {}
    ask = list(askers)
    for rid in ask:
        own_turns[rid] = own_turns.get(rid, 0) + 1
    def own_id(rid, attempt):
        return f'{team.agreement.run_id}-{rid}-solo-{own_turns[rid]}-{attempt}'
    for attempt in range(2):
        frames, tops = zone.capture(f'solo-{turn}-{attempt}', robots=ask)
        view = observe_items(tops, zone.config['static_map'], labels)
        ctx = {r: zp2.context(r, labels=labels, view=view, own_jobs=own_jobs[r],
                              extra={'invalid_reason': reasons[r]} if r in reasons else None) for r in ask}
        def build(rid, _shared, ctx=ctx, frames=frames, attempt=attempt):
            return zp2.build_request('independent', rid, request_id=own_id(rid, attempt), task=task,
                                     frame=frames[rid], ctx=ctx[rid], views=views)
        def fixture(rid, _shared, view=view, attempt=attempt):
            return zp2.fixture_solo_claim(rid, own_id(rid, attempt), goal, labels, view)
        replies = team.ask(ask, build, zp2.validate_solo_reply, fixture, phase=f'solo-{turn}-{attempt}',
                           turn=turn, sim_time=zone.time(), recipients=[])
        stats['claim_rounds'] += 1
        checked = check_independent_claims({r: (v['claim'] if v else None) for r, v in replies.items()},
                                           goal=goal, labels=labels, view=view)
        accepted.update(checked['accepted'])
        idle += checked['idle']
        stats['invalid_claims'] += len(checked['invalid'])
        team.event('CLAIMS_CHECKED', zone.time(), turn=turn, attempt=attempt, mode='independent',
                   accepted={r: c.record() for r, c in checked['accepted'].items()}, invalid=checked['invalid'],
                   idle=checked['idle'])
        reasons = dict(checked['invalid'])
        ask = sorted(reasons)
        if not ask:
            break
    return {'accepted': accepted, 'idle': idle}


__all__ = ['run_v2', 'ZoneRunV2', 'SCHEMA', 'PROTOCOL', 'DEFAULT_MAX_SIM_S']
