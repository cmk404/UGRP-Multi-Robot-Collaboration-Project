"""Planning guidance for dispatch peers: stated objective and plan schedule preview.

Three explicitly selected modes, compared as separate conditions:

- ``legacy``: the original prompt and capability text, byte for byte.
- ``objective``: states the team objective (deliver safely, then minimise total
  SIM time) and the executor's consequence of each dependency choice in
  neutral terms. The dependency example that pointed at one answer is removed
  and the passage description follows the authored map.
- ``objective_preview``: ``objective`` plus a schedule preview of each frozen
  proposal. The preview runs the executor's own permission rules
  (``SkillBindings``) over historical nominal stage durations. It reads only the
  proposed plan, the authored static map and the saved priors: no live pose,
  joint, contact or simulator state. It is a rough prior, not a measurement or
  success claim, and it never edits or rejects a plan.
"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

from harness.three_robot_plan import digest

MODES = ('legacy', 'objective', 'objective_preview')
ROOT = Path(__file__).resolve().parents[1]
PRIORS_PATH = ROOT / 'configs' / 'dispatch_stage_priors.json'

LEGACY_CAPABILITY_SCOPE = (
    'RGB pair approach/grasp plus loaded rotation and complete-footprint path checking; existing '
    'VisualBoxSkill. Parallel envelope 0.99m; rotated envelope 0.45m. Loaded terrain is unvalidated '
    'and avoided. In clutter, pickup preparation is exclusive. Waiting cargo and robots remain '
    'occupied space. If you select beam.after=[box_job] and box.after=[], the box executor releases '
    'its cargo and visually clears the unloading bay before finishing box_job. Role binding, routes '
    'and task dependencies follow your plan. All skills remain experimental; no raw-action fallback.')

OBJECTIVE_CAPABILITY_SCOPE = (
    'RGB pair approach/grasp plus loaded rotation and complete-footprint path checking; existing '
    'VisualBoxSkill. Parallel envelope 0.99m; rotated envelope 0.45m. Loaded terrain is unvalidated '
    'and avoided. In clutter, pickup preparation is exclusive. Waiting cargo and robots remain '
    'occupied space. Role binding, routes and task dependencies follow your plan. All skills remain '
    'experimental; no raw-action fallback.')

LEGACY_FEEDBACK = (
    'Propose feasible participants, routes AND task dependencies and get a NEW exact unanimous '
    'agreement. When pickup_approach rejects a coarse path, change the participants or end order; '
    'relocating an occupied pickup robot is not an available skill. Where '
    'after_box_delivery_and_yield_feasible is true, you may choose beam.after=[box job id] and '
    'box.after=[]: the box executor then delivers, releases, and clears the unloading bay before '
    'completing its job. Current waiting cargo/robots are obstacles; do not assume they disappear. '
    'Assignments remain your decision. Conditional future goals must be rechecked in actual RGB after '
    'execution. A map path is not physical success.')

OBJECTIVE_FEEDBACK = (
    'Propose feasible participants, routes AND task dependencies and get a NEW exact unanimous '
    'agreement. When pickup_approach rejects a coarse path, change the participants or end order; '
    'relocating an occupied pickup robot is not an available skill. Where '
    'after_box_delivery_and_yield_feasible is true, a dependency with the box first is also '
    'executable. Current waiting cargo/robots are obstacles; do not assume they disappear. '
    'Assignments remain your decision. Conditional future goals must be rechecked in actual RGB after '
    'execution. A map path is not physical success.')

LEGACY_PASSAGES = 'North is the upper passage around the central island; south is the lower one.'


def passage_sentence(static_map):
    if any(o.get('id') == 'service_island' for o in static_map.get('obstacles', [])):
        return LEGACY_PASSAGES
    return ('This map has no central island: north is the upper half and south the lower half of '
            'the open floor between pickup and the docks, each through its own gate region.')


def objective_block(static_map):
    """Neutral consequences of the executor's dependency and route rules."""
    open_map = not any(o.get('id') == 'service_island' for o in static_map.get('obstacles', []))
    lines = [
        'TEAM OBJECTIVE: first, deliver both cargo to the chosen dock without collision, drop or',
        'unsafe passage. Second, among plans you judge safe, finish the whole mission in the',
        'least total SIM time. Idle robots cost time.',
        'HOW THE EXECUTOR APPLIES YOUR DEPENDENCIES (choose on your own judgement):',
        '- X.after=[other job]: X does not grasp or carry until the other job has finished',
        '  completely (delivered, released and, for the box, bay cleared). X may approach'
        + (' meanwhile.' if open_map else ' only after that on this cluttered map.'),
        '- Both after=[] and different routes with different gates: both jobs run concurrently'
        + (' on this open map.' if open_map else ' only on an open map; this map runs them one at a time.'),
        '  The executor then starts the beam first: the box grasps once the beam pair starts',
        '  carrying, travels its own route, and waits before the shared unloading bay until the',
        '  beam is delivered. The bay is used by one team at a time either way.',
        '- Same route or same gate: jobs take turns on that resource.',
        'Use a dependency when your plan needs one job finished first, for example when waiting',
        'cargo or a robot would block the other team. Concurrency is not proof of safe passage.',
    ]
    return '\n'.join(lines)


def load_priors(path=PRIORS_PATH):
    raw = Path(path).read_bytes()
    value = json.loads(raw)
    if value.get('schema') != 'ugrp.dispatch_stage_priors.v1':
        raise ValueError('dispatch stage priors schema mismatch')
    return value, hashlib.sha256(raw).hexdigest()


def _stage(priors, obj, stage, dock):
    table = priors['stages_sim_s'][obj][stage]
    if isinstance(table, dict):
        if dock in table:
            return float(table[dock]), 'observed'
        values = [float(v) for v in table.values()]
        return sum(values) / len(values), 'unobserved_dock_mean'
    return float(table), 'observed'


def preview_plan(plan, static_map, priors, *, route_overlap=False, auto_route_overlap=True,
                 overlap_start='transit', step_s=0.1, max_sim_s=900.):
    """Rough schedule of one plan under the executor's own permission rules.

    Reuses ``SkillBindings.permission``/``finish``/``note_*`` so that the gate
    logic cannot drift from execution. Durations are historical nominal
    priors; unobserved route/dock combinations fall back to a pooled mean and
    are marked. Nothing here is observed in the current episode.
    """
    from harness.dispatch_skill_binding import SkillBindings
    bindings = SkillBindings({'plan': copy.deepcopy(plan), 'plan_hash': digest(plan)},
                             static_map, route_overlap=route_overlap,
                             auto_route_overlap=auto_route_overlap, overlap_start=overlap_start)
    dock = plan['dock']
    basis = {}

    def dur(obj, stage):
        value, source = _stage(priors, obj, stage, dock)
        basis[f'{obj}.{stage}'] = {'sim_s': round(value, 1), 'basis': source}
        return value

    # Each program is a list of (label, permission request or None, duration, hook).
    beam = [('approach', ('beam', 'APPROACH'), dur('beam', 'approach'), None),
            ('grasp', ('beam', 'GRASP'), dur('beam', 'grasp'), lambda: bindings.note_grasp_command('beam')),
            ('carry', ('beam', 'TRANSIT'), dur('beam', 'carry'), lambda: bindings.note_transit_command('beam'))]
    if bindings.route_overlap:
        beam.append(('release', ('beam', 'UNLOAD'), dur('beam', 'release'), None))
    else:
        beam.append(('release', None, dur('beam', 'release'), None))
    carry = dur('box', 'carry')
    box = [('approach', ('box', 'APPROACH'), dur('box', 'approach'), None),
           ('grasp', ('box', 'GRASP'), dur('box', 'grasp'), None)]
    if bindings.route_overlap:
        # The box queues before the shared bay (see ImageRoute UNLOAD check).
        share = float(priors.get('box_carry_before_bay_fraction', 0.7))
        box += [('carry', ('box', 'TRANSIT'), carry * share, None),
                ('carry_into_bay', ('box', 'UNLOAD'), carry * (1 - share), None)]
    else:
        box.append(('carry', ('box', 'TRANSIT'), carry, None))
    box.append(('release', None, dur('box', 'release'), None))
    yield_s = dur('box', 'yield')

    state = {'beam': {'program': beam, 'index': 0, 'left': None, 'waiting': 0., 'wait_on': {}, 'done_at': None},
             'box': {'program': box, 'index': 0, 'left': None, 'waiting': 0., 'wait_on': {}, 'done_at': None}}
    beam_id = bindings.tasks['beam']['id']
    t = 0.
    yield_used = False
    while t < max_sim_s:
        for obj, s in state.items():
            if s['done_at'] is not None:
                continue
            if s['index'] >= len(s['program']):
                if obj == 'beam':
                    bindings.finish('beam'); s['done_at'] = t
                elif beam_id in bindings.finished or yield_used:
                    # Executor: finish on release when the beam is done, else after clearing the bay.
                    bindings.finish('box'); s['done_at'] = t
                else:
                    s['program'].append(('clear_bay', None, yield_s, None)); yield_used = True
                continue
            label, request, duration, hook = s['program'][s['index']]
            if s['left'] is None:
                if request is not None and not bindings.permission(*request):
                    s['waiting'] += step_s
                    key = f'before_{label}'
                    s['wait_on'][key] = s['wait_on'].get(key, 0.) + step_s
                    continue
                if hook:
                    hook()
                s['left'] = duration
            s['left'] -= step_s
            if s['left'] <= 1e-9:
                s['left'] = None; s['index'] += 1
        if all(s['done_at'] is not None for s in state.values()):
            break
        t += step_s
    complete = all(s['done_at'] is not None for s in state.values())
    robots = {}
    for obj, task in bindings.tasks.items():
        s = state[obj]
        for rid in task['participants']:
            robots[rid] = {'job': task['id'], 'estimated_idle_sim_s': round(s['waiting']),
                           'idle_before': {k: round(v) for k, v in s['wait_on'].items() if v >= 1.},
                           'estimated_job_finish_sim_s': None if s['done_at'] is None else round(s['done_at'])}
    concurrent = bindings.route_overlap
    return {
        'schema': 'ugrp.dispatch_plan_preview.v1',
        'plan_hash': digest(plan),
        'execution_mode': 'concurrent' if concurrent else 'one_job_at_a_time',
        'execution_mode_reason': bindings.overlap_selection['reason'],
        'estimated_total_sim_s': round(max(s['done_at'] for s in state.values())) if complete else None,
        'robots': robots,
        'bay_clearing_by_box': yield_used,
        'duration_basis': basis,
        'scope': ('ROUGH PRIOR from historical synchronous runs and the executor gate rules applied to '
                  'this plan only. Not an observation of the current scene, not a feasibility or '
                  'success check, and never a replacement for your RGB judgement. Real durations '
                  'vary widely.'),
    }


class PlanGuidance:
    """Applies one explicitly selected guidance mode to planning requests."""

    def __init__(self, mode, static_map, *, route_overlap=False, auto_route_overlap=False,
                 overlap_start='transit', priors_path=PRIORS_PATH):
        if mode not in MODES:
            raise ValueError('unknown plan guidance mode')
        self.mode = mode
        # The preview must apply the same gate settings as this run's executor.
        self.route_settings = {'route_overlap': bool(route_overlap),
                               'auto_route_overlap': bool(auto_route_overlap),
                               'overlap_start': overlap_start}
        self.static_map = copy.deepcopy(static_map)
        self.priors = self.priors_sha256 = None
        if mode == 'objective_preview':
            self.priors, self.priors_sha256 = load_priors(priors_path)
        self.previews = {}
        self.preview_log = []

    @property
    def capability_scope(self):
        return LEGACY_CAPABILITY_SCOPE if self.mode == 'legacy' else OBJECTIVE_CAPABILITY_SCOPE

    @property
    def feasibility_feedback(self):
        return LEGACY_FEEDBACK if self.mode == 'legacy' else OBJECTIVE_FEEDBACK

    def system_prompt(self, prompt):
        if self.mode == 'legacy':
            return prompt
        if LEGACY_PASSAGES not in prompt:
            raise ValueError('planning prompt changed; guidance anchor missing')
        prompt = prompt.replace(LEGACY_PASSAGES, passage_sentence(self.static_map))
        prompt += '\n' + objective_block(self.static_map)
        if self.mode == 'objective_preview':
            prompt += ('\nproposal_schedule_preview, when present, is the host\'s ROUGH timing prior for '
                       'the frozen proposal under the executor rules above, from historical nominal '
                       'stage durations. It is not current-scene evidence or a success check. Weigh it '
                       'with your own RGB judgement; the host never changes or rejects a plan for time.')
        return prompt

    def preview(self, plan):
        key = digest(plan)
        if key not in self.previews:
            try:
                self.previews[key] = preview_plan(plan, self.static_map, self.priors, **self.route_settings)
            except ValueError as error:
                # e.g. explicit --route-overlap with a dependent plan: no estimate, say why.
                self.previews[key] = {'schema': 'ugrp.dispatch_plan_preview.v1', 'plan_hash': key,
                                      'execution_mode': 'not_executable_with_run_settings',
                                      'reason': str(error), 'estimated_total_sim_s': None, 'robots': {}}
        return copy.deepcopy(self.previews[key])

    def context(self, rid, agreement, context):
        if self.mode != 'objective_preview':
            return
        proposal = agreement.get('proposal')
        if proposal:
            preview = self.preview(proposal['plan'])
            context['proposal_schedule_preview'] = preview
            self.preview_log.append({'robot_id': rid, 'request_id': context.get('request_id'),
                                     'proposal_id': proposal['proposal_id'], 'preview': preview})
        if self.previews:
            # Previews of earlier (rejected) proposals remain visible to the next proposer.
            earlier = [p for k, p in self.previews.items() if not proposal or k != proposal['plan_hash']]
            if earlier:
                context['earlier_proposal_previews'] = [
                    {k: p[k] for k in ('plan_hash', 'execution_mode', 'estimated_total_sim_s', 'robots')}
                    for p in earlier[-2:]]

    def record(self):
        return {'mode': self.mode, 'route_settings': dict(self.route_settings), 'priors_path': (str(PRIORS_PATH.relative_to(ROOT))
                                                   if self.priors is not None else None),
                'priors_sha256': self.priors_sha256,
                'capability_scope_sha256': hashlib.sha256(self.capability_scope.encode()).hexdigest(),
                'previews_shown': len(self.preview_log)}
