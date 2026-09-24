"""Zone-goal delivery benchmark: LLM coordination over a TEACHER motion executor.

Robots (one independent LLM client each) decide who moves which box to which
zone, from own RGB + two TOP RGB images, RGB-derived labels/counts, own jobs
and peer messages. A ground-truth teacher executes the motions with the real
gripper (weld OFF). Results are teacher-executor conditions: they measure the
coordination (calls, conflicts, allocation, makespan), not RGB-skill success.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.three_robot_plan import ROBOTS, TeamAgreement  # noqa: E402
from harness import zone_coordination as zc  # noqa: E402
from harness.zone_perception import detect_all, label_pickup, observe  # noqa: E402
from sim.zone_arena import actor_task, episode, goal_counts  # noqa: E402

SCHEMA = 'ugrp.zone_dispatch_result.v1'
# Dynamic mode: re-ask a stalled team (no job running, claims unresolved) at
# most this many times before ending the run as STALLED.
STALL_TURNS = 2


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


class ZoneRun:
    """Scene, ports, teacher executor and RGB capture on one physics clock."""
    def __init__(self, config, output, *, record_replay=False):
        import mujoco
        from sim.camera_robot_port import CameraRobotPort
        from sim.multi_masterpi_production import MultiMasterPiProductionV2
        from sim.zone_scene import ZoneScene
        from scripts.zone_teacher import ZoneTeacherExecutor
        self.config, self.out = config, Path(output)
        self.out.mkdir(parents=True, exist_ok=False)
        (self.out/'rgb').mkdir()
        self.definition = ZoneScene.from_zone_config(config)
        self.world = MultiMasterPiProductionV2(seed=config['seed'], width=960, height=720, render=True,
            warehouse_layout=self.definition.engine_layout, warehouse_cargo_ids=None,
            xml_transform=self.definition.transform)
        self.definition.setup(self.world)
        (self.out/'scene.xml').write_text(self.world.scene_xml)
        self.ports = {r: CameraRobotPort(self.world, r, allow_reverse=True, allow_mecanum=True) for r in ROBOTS}
        self.events = []
        self.executor = ZoneTeacherExecutor(self.world, self.ports, config['static_map'],
                                            config['setup_only']['objects'], self._log)
        self.replay = None
        if record_replay:
            from scripts.zone_replay import ZoneReplayRecorder, arena_view
            self.replay = ZoneReplayRecorder(self.world, self.out, view=arena_view(config['static_map']['bounds_m']))
        self.robot_geoms = {i for i in range(self.world.model.ngeom)
                            if (mujoco.mj_id2name(self.world.model, mujoco.mjtObj.mjOBJ_GEOM, i) or '')
                            .startswith(tuple(r+'__' for r in ROBOTS))}
        self.robot_contact_steps = 0
        self.count = 0

    def _log(self, kind, rid, now, **detail):
        self.events.append({'event': kind, 'robot_id': rid, 'sim_time_s': round(now, 3), **detail})

    def time(self):
        return float(self.world.data.time)

    def step(self, seconds):
        dt = float(self.world.model.opt.timestep)
        for _ in range(max(1, round(seconds/dt))):
            now = self.time()
            self.executor.tick(now)
            for port in self.ports.values():
                port.tick(now)
            self.world._physics_step_for(self.world.controllers['r1'])
            if self.replay:
                self.replay.sample(' | '.join(f'{r}:{t.phase}' for r, t in self.executor.robots.items()))

    def tops(self):
        return {'cctv_top': self.world.render_team_jpeg(camera='cctv_top', quality=95),
                'cctv_top_east': self.world.render_team_jpeg(camera='cctv_top_east', quality=95)}

    def capture(self, label, robots=ROBOTS):
        self.count += 1
        tops = self.tops()
        (self.out/'rgb'/f'{self.count:03d}-{label}-top-west.jpg').write_bytes(tops['cctv_top'])
        (self.out/'rgb'/f'{self.count:03d}-{label}-top-east.jpg').write_bytes(tops['cctv_top_east'])
        frames = {}
        for rid in robots:
            own = self.world.render_jpeg(robot_id=rid, camera='robot_cam', quality=90)
            (self.out/'rgb'/f'{self.count:03d}-{label}-{rid}.jpg').write_bytes(own)
            frames[rid] = {'own': own, 'top_west': tops['cctv_top'], 'top_east': tops['cctv_top_east']}
        return frames, tops

    def box_positions(self):
        """Referee/teacher only."""
        return {oid: {'kind': item['kind'],
                      'xyz': [round(float(v), 4) for v in self.world.data.body(item['body_name']).xpos]}
                for oid, item in self.config['setup_only']['objects'].items()}

    def close(self):
        return self.replay.close() if self.replay else None


def resolve_label(run, labels, label):
    """Teacher-side: the physical box nearest to the label's RGB floor estimate."""
    import math
    xy = labels[label]['floor_xy_m']
    best = min(run.config['setup_only']['objects'].items(),
               key=lambda kv: math.dist(run.world.data.body(kv[1]['body_name']).xpos[:2], xy))
    return best[0], best[1]['body_name']


class Slots:
    """Executor-side slot bookkeeping from issued jobs (no simulator reads)."""
    def __init__(self, static_map):
        self.free = {z: [s['slot_id'] for s in slots] for z, slots in static_map['zone_slots'].items()}
        self.xy = {s['slot_id']: s['center_m'] for slots in static_map['zone_slots'].values() for s in slots}

    def take(self, zone):
        if not self.free[zone]:
            raise RuntimeError(f'zone {zone} has no free slot')
        return self.free[zone].pop(0)

    def give_back(self, slot):
        """A job the executor stopped before its release never filled its slot."""
        zone = slot[0]
        if slot in self.free[zone]:
            raise ValueError(f'slot {slot} is already free')
        self.free[zone].insert(0, slot)


def run(args):
    if subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).strip():
        raise RuntimeError('commit and freeze source before a trial')
    goal = goal_counts(json.loads(args.goal))
    config = episode(args.variant, args.seed, goal=goal, extra_boxes=json.loads(args.extra_boxes))
    config['contact_solver_profile'] = args.contact_profile
    config['extra_boxes'] = json.loads(args.extra_boxes)
    source = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    result = {'schema': SCHEMA, 'source_sha': source, 'coordination': args.coordination,
              'executor': 'ground-truth TEACHER (drive + calibrated IK + real gripper, weld OFF)',
              'claim_scope': 'teacher-executor condition: coordination metrics, not RGB-skill success',
              'config': {k: v for k, v in vars(args).items() if k != 'output'} | {'output': str(args.output)},
              'goal': goal, 'phase': 'SETUP', 'error': None, 'goal_met_rgb': False}
    started = time.monotonic()
    zone = ZoneRun(config, args.output, record_replay=args.record_replay)
    write(args.output/'episode-setup-only.json', config)
    from scripts.three_robot_runtime import ThreeRobotRuntime
    task = actor_task(config['static_map'], goal)
    run_id = 'zone-' + source[:8] + '-' + str(int(time.time()))[-6:]
    labels = {}
    agreement = TeamAgreement(run_id, plan_validator=lambda plan: zc.plan_validator(goal, labels)(plan))
    team = ThreeRobotRuntime(args.output/'team', run_id=run_id, mode=args.mode, agreement=agreement,
                             model=args.model, max_wall_s=args.max_wall_s, request_timeout=60., max_tokens=1400)
    slots = Slots(config['static_map'])
    active, own_jobs, finished, failed = {}, {r: [] for r in ROBOTS}, [], []
    queues = {r: [] for r in ROBOTS}
    stats = {'claim_rounds': 0, 'collisions': 0, 'invalid_claims': 0, 'plan_turns': 0, 'done_robots': [],
             'stalled_turns': 0}
    stalled = False
    try:
        zone.step(.5)
        frames, tops = zone.capture('start')
        labels.update(label_pickup(detect_all(tops, config['static_map']), config['static_map']))
        write(args.output/'box-labels.json', labels)
        result['labels'] = {k: v['kind'] for k, v in labels.items()}

        def board():
            return {'active': {r: {'box': j['box'], 'zone': j['zone']} for r, j in active.items()},
                    'finished_reports': copy.deepcopy(finished[-12:]), 'stopped_reports': copy.deepcopy(failed[-6:])}

        def assign(rid, job):
            oid, body = resolve_label(zone, labels, job['box'])
            slot = slots.take(job['zone'])
            full = {**job, 'kind': labels[job['box']]['kind'], 'object': oid, 'slot': slot,
                    'job_id': f"{rid}-{len(own_jobs[rid])+1}"}
            active[rid] = full
            own_jobs[rid].append({'box': job['box'], 'zone': job['zone'], 'slot': slot, 'status': 'issued',
                                  'issued_at_sim_s': round(zone.time(), 2)})
            zone.executor.robots[rid].assign({'job_id': full['job_id'], 'box_body': body,
                                              'slot_xy': slots.xy[slot]}, zone.time())
            print(f"JOB {rid} {job['box']} -> {job['zone']} ({slot})", flush=True)

        def collect_done():
            for rid, robot in zone.executor.robots.items():
                if rid in active and not robot.busy:
                    # New evidence: robots that passed or were refused may be asked again.
                    stats['done_robots'] = []
                    job = active.pop(rid)
                    report = {'robot': rid, 'box': job['box'], 'zone': job['zone'],
                              'executor_receipt': ('issued sequence finished' if robot.outcome == 'placed_by_teacher'
                                                   else 'executor stopped before finishing'),
                              'sim_time_s': round(zone.time(), 2)}
                    own_jobs[rid][-1]['status'] = report['executor_receipt']
                    (finished if robot.outcome == 'placed_by_teacher' else failed).append(report)
                    if robot.outcome in ('grasp_failed_by_teacher', 'teacher_path_blocked', 'dropped_in_transit'):
                        slots.give_back(job['slot'])
                    zone._log('job_end', rid, zone.time(), outcome=robot.outcome, box=job['box'], zone=job['zone'])

        if args.coordination == 'plan_first':
            result['phase'] = 'NEGOTIATE'
            view = observe(tops, config['static_map'], labels)
            for turn in range(args.planning_rounds):
                frames, tops = zone.capture(f'plan-{turn}')
                ctx = {r: zc.context(r, task=task, labels=labels, view=view, board=board(),
                                     own_jobs=own_jobs[r], inbox=team.inbox[r]) for r in ROBOTS}
                context = agreement.context()
                def build(rid, request_id, ctx=ctx, context=context, frames=frames):
                    return zc.build_plan_request(rid, request_id=request_id, task=task, frame=frames[rid],
                                                 agreement=context, ctx=ctx[rid])
                def fixture(rid, request_id, context=context):
                    return _fixture_plan_reply(rid, request_id, context, goal, labels, args.fixture_plan)
                from harness.three_robot_plan import validate_plan_reply
                replies = team.ask(ROBOTS, build,
                    lambda raw, rq, c=context: validate_plan_reply(raw, rq, c, plan_validator=agreement.plan_validator),
                    fixture, phase='plan', turn=turn, sim_time=zone.time())
                stats['plan_turns'] += 1
                if agreement.receive(replies, turn):
                    break
            if agreement.committed is None:
                raise RuntimeError('no plan agreed within the planning rounds')
            write(args.output/'committed-plan.json', agreement.committed)
            for rid in ROBOTS:
                queues[rid] = list(agreement.committed['plan']['assignments'][rid])
            print('PLAN COMMITTED ' + agreement.committed['plan_hash'], flush=True)
        result['phase'] = 'EXECUTE'
        motion_started = zone.time()
        turn = 0
        while zone.time() - motion_started < args.max_sim_s:
            collect_done()
            idle = [r for r in ROBOTS if r not in active and not zone.executor.robots[r].busy]
            if args.coordination == 'plan_first':
                for rid in idle:
                    if queues[rid]:
                        assign(rid, queues[rid].pop(0))
                if not active and not any(queues.values()):
                    break
            else:
                waiting = [r for r in idle if r not in stats['done_robots']]
                if waiting:
                    turn += 1
                    decided = dynamic_round(zone, team, task, labels, goal, waiting, active, own_jobs,
                                            board, stats, turn, args)
                    for rid, job in decided['accepted'].items():
                        assign(rid, job)
                    # Robots with no job (null claim, or refused after talking) wait
                    # until a peer's job ends before they are asked again.
                    for rid in waiting:
                        if rid not in decided['accepted']:
                            stats['done_robots'].append(rid)
                    if not active and set(stats['done_robots']) >= set(idle):
                        # Nobody holds a job: either the team believes the goal
                        # is met, or unresolved claims left everyone waiting.
                        # Ask again (with the stall visible) a bounded number
                        # of times before ending as STALLED.
                        if not decided['idle_all'] and stats['stalled_turns'] < STALL_TURNS:
                            stats['stalled_turns'] += 1
                            stats['done_robots'] = []
                        else:
                            if not decided['idle_all']:
                                stalled = True
                            break
                elif not active:
                    break
            zone.step(.5)
        result['phase'] = ('STALLED' if stalled else 'FINISHED' if zone.time() - motion_started < args.max_sim_s
                           else 'SIM_BUDGET')
        zone.step(1.)
        _, tops = zone.capture('final')
        result['final_rgb_view'] = observe(tops, config['static_map'], labels)
        result['goal_met_rgb'] = zc.goal_met(goal, result['final_rgb_view'])
        result['makespan_sim_s'] = round(zone.time() - motion_started, 2)
    except Exception as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
        (args.output/'exception.txt').write_text(traceback.format_exc())
    finally:
        team.close(zone.time())
        result['referee'] = zc.referee(goal, config['static_map'], zone.box_positions())
        result['physical_success_teacher_condition'] = result['referee']['goal_met']
        # Generic keys for the shared TensorBoard exporter; scope marks the condition.
        result['success'] = result['physical_success_teacher_condition']
        result['sim_s'] = result.get('makespan_sim_s')
        result['scope'] = ('zone benchmark, TEACHER motion executor (ground truth drive/IK, real gripper, '
                           'weld off): success is the referee goal count, not an RGB-skill result')
        result['sim_end_s'] = round(zone.time(), 2)
        result['wall_s'] = round(time.monotonic() - started, 2)
        result['jobs'] = own_jobs
        result['finished_reports'], result['stopped_reports'] = finished, failed
        result['coordination_stats'] = stats
        result['llm_calls'] = sum(1 for c in team.calls if c.get('model') != 'scripted-fixture-not-llm')
        result['usage'] = {k: sum((c.get('usage') or {}).get(k, 0) for c in team.calls)
                           for k in ('prompt_tokens', 'completion_tokens', 'total_tokens')}
        result['replay'] = zone.close()
        write(args.output/'teacher-events.json', zone.events)
        write(args.output/'result.json', result)
        print(json.dumps({k: result.get(k) for k in ('phase', 'error', 'goal_met_rgb',
              'physical_success_teacher_condition', 'makespan_sim_s', 'llm_calls', 'wall_s')}), flush=True)
    return result


def dynamic_round(zone, team, task, labels, goal, waiting, active, own_jobs, board, stats, turn, args):
    """Idle robots claim one job each; collide -> the colliding robots talk (<=2 rounds)."""
    accepted, idle = {}, []
    askers, extra = list(waiting), {}
    for attempt in range(3):
        frames, tops = zone.capture(f'claim-{turn}-{attempt}', robots=askers)
        view = observe(tops, zone.config['static_map'], labels)
        pending = {**active, **accepted}
        ctx = {r: zc.context(r, task=task, labels=labels, view=view, board=board() | {
                   'active': {k: {'box': j['box'], 'zone': j['zone']} for k, j in pending.items()}},
                   own_jobs=own_jobs[r], inbox=team.inbox[r], extra=extra.get(r)) for r in askers}
        def build(rid, request_id, ctx=ctx, frames=frames):
            return zc.build_claim_request(rid, request_id=request_id, task=task, frame=frames[rid], ctx=ctx[rid])
        def fixture(rid, request_id, view=view, pending=pending, askers=tuple(askers)):
            return _fixture_claim(rid, request_id, goal, labels, view, pending, askers)
        replies = team.ask(askers, build, zc.validate_claim_reply, fixture,
                           phase=f'claim-{turn}-{attempt}', turn=turn, sim_time=zone.time())
        stats['claim_rounds'] += 1
        checked = zc.check_claims({r: (v['claim'] if v else None) for r, v in replies.items()},
                                  goal=goal, labels=labels, view=view, active=pending)
        accepted.update(checked['accepted'])
        idle += checked['idle']
        stats['collisions'] += len(checked['collisions'])
        stats['invalid_claims'] += len(checked['invalid'])
        team.event('CLAIMS_CHECKED', zone.time(), turn=turn, attempt=attempt, **checked)
        retry = sorted({r for c in checked['collisions'] for r in c['robots']} | set(checked['invalid']))
        retry = [r for r in retry if r not in accepted]
        if not retry:
            break
        askers = retry
        extra = {r: {'conflict': [c for c in checked['collisions'] if r in c['robots']],
                     'invalid_reason': checked['invalid'].get(r)} for r in retry}
    # idle_all: no claim was left unresolved; every robot without a job said
    # that nothing useful remains for it.
    return {'accepted': accepted, 'idle': idle, 'unresolved': retry,
            'idle_all': not retry and set(waiting) - set(accepted) <= set(idle)}


def _fixture_plan_reply(rid, request_id, context, goal, labels, plan_file=None):
    proposal = context['proposal']
    if proposal:
        return {'request_id': request_id, 'proposal_id': proposal['proposal_id'],
                'plan_hash': proposal['plan_hash'], 'accept': True, 'plan': proposal['plan'],
                'reason': 'scripted protocol fixture, not visual reasoning', 'message': ''}
    if plan_file:
        # Diagnostic: propose a recorded committed plan to replay its execution.
        plan = json.loads(Path(plan_file).read_text())['plan']
        return {'request_id': request_id, 'proposal_id': None, 'plan_hash': None, 'accept': True,
                'plan': plan, 'reason': f'scripted fixture proposing recorded plan {plan_file}', 'message': ''}
    jobs, unused = [], sorted(labels)
    for zone, kinds in sorted(goal.items()):
        for kind, count in sorted(kinds.items()):
            for _ in range(count):
                box = next(b for b in unused if labels[b]['kind'] == kind)
                unused.remove(box)
                jobs.append({'box': box, 'zone': zone})
    plan = {'assignments': {r: jobs[i::3] for i, r in enumerate(ROBOTS)}}
    return {'request_id': request_id, 'proposal_id': None, 'plan_hash': None, 'accept': True,
            'plan': plan, 'reason': 'scripted protocol fixture, not visual reasoning', 'message': ''}


def _fixture_claim(rid, request_id, goal, labels, view, pending, askers=ROBOTS):
    """Scripted protocol fixture (not visual reasoning): the i-th asking robot
    takes the i-th open need unit, so idle robots claim different boxes."""
    need = zc.remaining_need(goal, view, pending)
    taken = {j['box'] for j in pending.values()}
    units = [(zone, kind) for zone, kinds in sorted(need.items())
             for kind in sorted(kinds) for _ in range(kinds[kind])]
    free = {kind: [b for b in view['pickup_boxes_still_visible'] if b not in taken and labels[b]['kind'] == kind]
            for kind in {k for _, k in units}}
    order = []
    for zone, kind in units:
        if free[kind]:
            order.append({'box': free[kind].pop(0), 'zone': zone})
    rank = list(askers).index(rid)
    claim = order[rank] if rank < len(order) else {'box': None, 'zone': None}
    return {'request_id': request_id, 'claim': claim,
            'reason': 'scripted protocol fixture, not visual reasoning', 'message': ''}


def parser():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--goal', default=json.dumps({'A': {'red': 2}, 'B': {'cyan': 1}, 'C': {'green': 1, 'red': 1}}))
    p.add_argument('--extra-boxes', default='{}', help='spare boxes per colour, JSON')
    p.add_argument('--variant', default='zone_open')
    p.add_argument('--seed', type=int, default=11)
    p.add_argument('--coordination', choices=('plan_first', 'dynamic'), default='dynamic')
    p.add_argument('--mode', choices=('llm', 'fixture'), default='llm')
    p.add_argument('--model', default='gemini-3.8-flash')
    p.add_argument('--planning-rounds', type=int, default=8)
    p.add_argument('--max-sim-s', type=float, default=900.)
    p.add_argument('--max-wall-s', type=float, default=3600.)
    p.add_argument('--contact-profile', default='local_contact_fine')
    p.add_argument('--record-replay', action='store_true')
    p.add_argument('--fixture-plan', help='diagnostic, fixture plan_first only: propose this recorded '
                   'committed-plan.json instead of the scripted split')
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    if args.fixture_plan and (args.mode != 'fixture' or args.coordination != 'plan_first'):
        raise SystemExit('--fixture-plan needs --mode fixture --coordination plan_first')
    result = run(args)
    return 0 if result.get('error') is None else 1


if __name__ == '__main__':
    raise SystemExit(main())
