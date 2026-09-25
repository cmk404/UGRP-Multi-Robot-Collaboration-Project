"""Robot inputs for the Korean-dialogue zone study: order sheet, per-call payload,
log records (package A).

Every input here is a function of the SCENARIO CONFIG and the static map FILE.
No simulator object, no MuJoCo import, no live state: ``order_sheet`` runs from a
dict (or JSON file) plus ``harness.zone_map_schematic.map_bundle``, and
``OrderSheetSource`` freezes it for the whole run, so the sheet cannot drift with
what happens in the arena.

* ``initial_location`` is where an item was PUT AT SETUP (a coarse pickup bay
  slot). It is never refreshed after a move, a drop or a re-grasp, and it carries
  no simulator coordinate or body name.
* Hidden events live in the scenario's ``eval`` section. ``order_sheet`` and
  ``build_call_input`` never read it; ``eval_section``/``hidden_events`` are for
  the evaluator and the setup generator only.
* ``build_call_input`` assembles exactly the condition's allowlisted keys from the
  run's frozen ``OrderSheetSource`` and runs
  ``harness.zone_study_contract.validate_robot_payload`` before returning (no
  switch to skip it). The validator closes every robot-facing sub-schema, so a
  renamed TOP frame, pose, teacher receipt, completion flag or peer camera is
  rejected too. It checks key names and structure, not the meaning of values:
  whoever fills a belief or a command argument must still use own observations
  only.
* ``call_log_record``/``message_log_record``/``action_log_record`` write the log
  schema that package D (SIM cost) and package I (evaluation) consume.

Korean prompts keep IDs, zone letters and JSON keys literal: keys stay ASCII and
only free-text values are Korean.
"""
from __future__ import annotations

import copy
import json
import re
from collections.abc import Mapping, Sequence
from pathlib import Path

from harness.zone_map_schematic import MAP_BUNDLE_SCHEMA, digest, static_map_section
from harness.zone_study_contract import (ACTION_LOG_SCHEMA, CALL_LOG_SCHEMA, COMMANDER, ID_TOKEN,
                                         LOCAL_STATES, MESSAGE_LOG_SCHEMA, ORDER_KEYS, ORDER_SHEET_SCHEMA,
                                         PAYLOAD_SCHEMA, PROVENANCE_KEYS, ROBOTS, ROLE_NAMES, SHA256_HEX,
                                         ZONE_IDS, ContractViolation, Vocabulary, channel_section, condition,
                                         condition_manifest, forbidden_key_hits, free_text_report,
                                         leader_for_seed, registry_sha256, role_of, validate_log_record,
                                         validate_robot_payload)

SCENARIO_SCHEMA = 'ugrp.zone_scenario.v1'
# Item kinds, their physical team size and their grasp roles. Frozen here so the
# inputs stay simulator-free; tests check the table against ``harness.zone_goal_v2``.
FORMATIONS = {'cyan': ('west',), 'green': ('west',), 'red': ('west',), 'yellow': ('west',),
              'can': ('any',), 'tile': ('west',), 'long_beam': ('end_neg', 'end_pos'),
              'heavy_crate': ('west', 'east'), 'tri_frame': ('v0', 'v1', 'v2')}
REQUIRED_ROBOTS = {kind: len(roles) for kind, roles in FORMATIONS.items()}
ITEM_KINDS = tuple(FORMATIONS)
IDENTITY = ('kind_fungible', 'specific_item')
# Development start values, identical in every condition (fairness): the same
# observation history, the same command history and the same inbox depth.
INPUT_PROFILE = {'profile_id': 'zone_study_inputs.v1', 'own_rgb_frames': 2, 'command_history_entries': 12,
                 'inbox_messages': 8, 'text_token_budget': 8000, 'image_slots': 2, 'output_token_budget': 768}
ORDER_NOTE_KO = ('주문서와 initial_location은 설정 시점의 계획 정보다. 현재 위치·재고·배송 완료를 보장하지 '
                 '않는다. 현재 상황은 자기 RGB, 자기 발행 명령과 허용된 수신 메시지로만 판단한다.')
TRIGGERS = ('start', 'own_view_change', 'own_timer', 'message_received', 'idle_review', 'execution_review')


def _token(value: object, label: str) -> str:
    if not isinstance(value, str) or not ID_TOKEN.match(value):
        raise ContractViolation(f'{label} must be a literal ASCII id token, got {value!r}')
    return value


def load_scenario(path: Path | str) -> dict:
    """Read a scenario config JSON (no simulator, no map access)."""
    return json.loads(Path(path).read_text())


def eval_section(scenario: Mapping) -> dict:
    """Evaluation/setup-only part of the scenario (hidden events, notes). Never robot input."""
    return copy.deepcopy(dict(scenario.get('eval') or {}))


def hidden_events(scenario: Mapping) -> list:
    """The hidden event schedule. Evaluation and setup only; never a robot input or a call trigger."""
    return copy.deepcopy(list(eval_section(scenario).get('hidden_events', ())))


def validate_scenario(scenario: Mapping, *, map_bundle: Mapping | None = None) -> dict:
    """Normalized deep copy of a scenario config; raises on anything a robot could not be told."""
    if not isinstance(scenario, Mapping):
        raise ContractViolation('scenario config must be an object')
    if scenario.get('schema') != SCENARIO_SCHEMA:
        raise ContractViolation(f'scenario schema must be {SCENARIO_SCHEMA}, got {scenario.get("schema")!r}')
    unknown = set(scenario) - {'schema', 'scenario_id', 'map_id', 'landmark_detail', 'seeds', 'orders',
                               'eval', 'notes'}
    if unknown:
        raise ContractViolation(f'unknown scenario key(s): {sorted(unknown)}')
    scenario_id = _token(scenario.get('scenario_id'), 'scenario_id')
    map_id = _token(scenario.get('map_id'), 'map_id')
    seeds = scenario.get('seeds')
    if not isinstance(seeds, Sequence) or isinstance(seeds, str) or not seeds \
            or any(isinstance(s, bool) or not isinstance(s, int) for s in seeds):
        raise ContractViolation('seeds must be a non-empty list of ints')
    if len(set(seeds)) != len(seeds):
        raise ContractViolation('seeds must be unique')
    leaders = leader_rotation(seeds)
    if len(seeds) >= len(ROBOTS) and len(set(leaders.values())) < len(ROBOTS):
        raise ContractViolation('leader_ko rotates as robots[seed % 3]; these seeds would never make '
                                f'{sorted(set(ROBOTS) - set(leaders.values()))} the leader. '
                                'Pick seeds that cover every residue.')
    orders = scenario.get('orders')
    if not isinstance(orders, Sequence) or isinstance(orders, str) or not orders:
        raise ContractViolation('orders must be a non-empty list')
    if map_bundle is not None:
        if map_bundle.get('schema') != MAP_BUNDLE_SCHEMA:
            raise ContractViolation('map_bundle must come from harness.zone_map_schematic.map_bundle')
        if map_bundle['map_id'] != map_id:
            raise ContractViolation(f'scenario map_id {map_id} differs from the bundle {map_bundle["map_id"]}')
        declared = map_bundle['public_map']['pickup_bays']
        bays = {b['bay_id'] for b in declared}
        slots = {s['slot_id'] for b in declared for s in b['slots']}
    else:
        bays = slots = None
    seen, normalized = set(), []
    for raw in orders:
        normalized.append(_order(raw, bays=bays, slots=slots, seen=seen))
    value = {'schema': SCENARIO_SCHEMA, 'scenario_id': scenario_id, 'map_id': map_id,
             'landmark_detail': scenario.get('landmark_detail', 'full'), 'seeds': [int(s) for s in seeds],
             'orders': normalized, 'eval': eval_section(scenario)}
    if 'notes' in scenario:
        value['notes'] = scenario['notes']
    return value


def _order(raw: Mapping, *, bays, slots, seen: set) -> dict:
    if not isinstance(raw, Mapping):
        raise ContractViolation('each order must be an object')
    hits = forbidden_key_hits(raw)
    if hits:
        raise ContractViolation('an order may not carry evaluation-only data: ' + '; '.join(hits))
    unknown = set(raw) - set(ORDER_KEYS)
    if unknown:
        raise ContractViolation(f'unknown order key(s): {sorted(unknown)}')
    order_id = _token(raw.get('order_id'), 'order_id')
    if order_id in seen:
        raise ContractViolation(f'duplicate order_id: {order_id}')
    seen.add(order_id)
    kind = raw.get('kind')
    if kind not in REQUIRED_ROBOTS:
        raise ContractViolation(f'unknown item kind: {kind!r} (known: {ITEM_KINDS})')
    count = raw.get('count')
    if isinstance(count, bool) or not isinstance(count, int) or count < 1:
        raise ContractViolation('count must be a positive int')
    identity = raw.get('identity', 'kind_fungible')
    if identity not in IDENTITY:
        raise ContractViolation(f'identity must be one of {IDENTITY}')
    item_ids = raw.get('item_ids')
    if not item_ids:
        if identity == 'specific_item':
            raise ContractViolation(f'{order_id}: a specific_item order needs item_ids')
        if item_ids is not None and not isinstance(item_ids, (list, tuple)):
            raise ContractViolation('item_ids must be a list')
        item_ids = []
    else:
        if not isinstance(item_ids, Sequence) or isinstance(item_ids, str):
            raise ContractViolation('item_ids must be a list')
        item_ids = [_token(i, 'item_id') for i in item_ids]
        if len(set(item_ids)) != len(item_ids):
            raise ContractViolation(f'{order_id}: item_ids must be unique')
        if len(item_ids) != count:
            raise ContractViolation(f'{order_id}: count {count} differs from len(item_ids) {len(item_ids)}')
    required = raw.get('required_robots', REQUIRED_ROBOTS[kind])
    if required != REQUIRED_ROBOTS[kind]:
        raise ContractViolation(f'{order_id}: {kind} needs {REQUIRED_ROBOTS[kind]} robots, '
                                f'the config says {required!r}')
    if required > len(ROBOTS):
        raise ContractViolation(f'{order_id}: needs more robots than the team has')
    zone = raw.get('destination_zone')
    if zone not in ZONE_IDS:
        raise ContractViolation(f'destination_zone must be one of {ZONE_IDS}, got {zone!r}')
    location = raw.get('initial_location')
    if not isinstance(location, Mapping) or set(location) - {'pickup_bay', 'slot'} or 'pickup_bay' not in location:
        raise ContractViolation(f'{order_id}: initial_location must be {{"pickup_bay": ..., "slot": ...}}')
    bay = _token(location['pickup_bay'], 'pickup_bay')
    slot = location.get('slot')
    if slot is not None:
        slot = _token(slot, 'slot')
        if not slot.startswith(bay + '-'):
            raise ContractViolation(f'{order_id}: slot {slot} is not inside bay {bay}')
    if bays is not None:
        if bay not in bays:
            raise ContractViolation(f'{order_id}: pickup_bay {bay} is not in the map (bays: {sorted(bays)})')
        if slot is not None and slot not in slots:
            raise ContractViolation(f'{order_id}: slot {slot} is not in the map')
    return {'order_id': order_id, 'kind': kind, 'count': count, 'item_ids': item_ids,
            'required_robots': required, 'destination_zone': zone,
            'initial_location': {'pickup_bay': bay, 'slot': slot}, 'identity': identity}


def leader_rotation(seeds: Sequence[int]) -> dict:
    """{seed: leader} of the ``leader_ko`` rotation, so a cohort can check its coverage."""
    return {int(seed): leader_for_seed('leader_ko', int(seed)) for seed in seeds}


def _kind_table(orders: Sequence[Mapping]) -> dict:
    """Static team requirement of every kind the sheet names (task information, not live state)."""
    return {kind: {'required_robots': REQUIRED_ROBOTS[kind], 'roles': list(FORMATIONS[kind])}
            for kind in sorted({o['kind'] for o in orders})}


def _build_sheet(normalized: Mapping, map_ref: Mapping) -> dict:
    sheet = {'schema': ORDER_SHEET_SCHEMA, 'scenario_id': normalized['scenario_id'],
             'map_id': normalized['map_id'], 'map_file_sha256': map_ref['map_file_sha256'],
             'public_map_sha256': map_ref['public_map_sha256'],
             'orders': copy.deepcopy(list(normalized['orders'])),
             'kinds': _kind_table(normalized['orders']), 'team_size': len(ROBOTS),
             'note_ko': ORDER_NOTE_KO}
    hits = forbidden_key_hits(sheet)
    if hits:
        raise ContractViolation('order sheet leaks evaluation-only data: ' + '; '.join(hits))
    return sheet


def _map_ref(map_bundle: Mapping) -> dict:
    return {'map_id': map_bundle['map_id'], 'map_file_sha256': map_bundle['map_file_sha256'],
            'public_map_sha256': map_bundle['public_map_sha256']}


def order_sheet(scenario: Mapping, map_bundle: Mapping) -> dict:
    """The immutable robot-facing task order sheet built from the scenario config alone."""
    return _build_sheet(validate_scenario(scenario, map_bundle=map_bundle), _map_ref(map_bundle))


class OrderSheetSource:
    """A frozen order sheet: built once from the config, identical for the whole run."""

    def __init__(self, scenario: Mapping, map_bundle: Mapping):
        self._scenario = validate_scenario(scenario, map_bundle=map_bundle)
        self._bundle_ref = _map_ref(map_bundle)
        self._sheet = _build_sheet(self._scenario, self._bundle_ref)
        self.sha256 = digest(self._sheet)
        self.scenario_sha256 = digest(self._scenario)

    @property
    def scenario_id(self) -> str:
        return self._scenario['scenario_id']

    @property
    def seeds(self) -> tuple[int, ...]:
        return tuple(self._scenario['seeds'])

    def sheet(self) -> dict:
        """A fresh deep copy; mutating it cannot change the next call's input."""
        return copy.deepcopy(self._sheet)

    def hidden_events(self) -> list:
        """Evaluation/setup only. Kept here so the setup generator reads the SAME config."""
        return hidden_events(self._scenario)

    def assert_unchanged(self) -> None:
        """Raise if the frozen sheet or its source config drifted during the run."""
        if digest(self._scenario) != self.scenario_sha256:
            raise ContractViolation('the scenario config changed during the run')
        if digest(self._sheet) != self.sha256:
            raise ContractViolation('the order sheet changed during the run')
        if digest(_build_sheet(self._scenario, self._bundle_ref)) != self.sha256:
            raise ContractViolation('the order sheet no longer matches its scenario config')

    def manifest(self) -> dict:
        """What the run bundle records about the inputs (docs/execution_versioning.md)."""
        return {'scenario_schema': SCENARIO_SCHEMA, 'scenario_id': self.scenario_id,
                'scenario_sha256': self.scenario_sha256, 'order_sheet_schema': ORDER_SHEET_SCHEMA,
                'order_sheet_sha256': self.sha256, 'orders': len(self._sheet['orders']),
                'seeds': list(self.seeds), 'leader_rotation': leader_rotation(self.seeds),
                'map': dict(self._bundle_ref), 'hidden_event_count': len(self.hidden_events()),
                'input_profile': dict(INPUT_PROFILE)}


def vocabulary(sheet: Mapping, public_map: Mapping) -> Vocabulary:
    """The IDs a structured message may name, from the order sheet and the public map only."""
    items = {o['order_id'] for o in sheet['orders']} | {i for o in sheet['orders'] for i in o['item_ids']}
    items |= {o['kind'] for o in sheet['orders']}
    passages = {p['id'] for p in public_map.get('passages', ())}
    refs = {b['bay_id'] for b in public_map.get('pickup_bays', ())}
    refs |= {s['slot_id'] for b in public_map.get('pickup_bays', ()) for s in b['slots']}
    refs |= {s['slot_id'] for slots in public_map.get('zone_slots', {}).values() for s in slots}
    refs |= set(ZONE_IDS) | {'pickup'}
    return Vocabulary(items=frozenset(items), zones=frozenset(ZONE_IDS), roles=frozenset(ROLE_NAMES),
                      passages=frozenset(passages), location_refs=frozenset(refs))


# ---------------------------------------------------------------------------
# Own observations, own commands, belief

def own_rgb_ref(robot_id: str, index: int, captured_at_sim_s: float, sha256: str) -> dict:
    """A reference to one of this robot's own wrist frames; ``sha256`` binds it to the real bytes."""
    if robot_id not in ROBOTS:
        raise ContractViolation(f'unknown robot: {robot_id!r}')
    if not isinstance(sha256, str) or not SHA256_HEX.match(sha256):
        raise ContractViolation('own_rgb_ref needs the sha256 of the frame bytes (auditable input)')
    return {'ref': f'own-{robot_id}-{int(index):04d}', 'kind': 'own_wrist_rgb',
            'captured_at_sim_s': float(captured_at_sim_s), 'sha256': sha256}


def command_entry(command_id: str, issued_at_sim_s: float, kind: str, arguments: Mapping | None = None, *,
                  local_state: str = 'command_issued') -> dict:
    """One of this robot's OWN issued commands. Issuing is not moving and not success."""
    if local_state not in LOCAL_STATES:
        raise ContractViolation(f'local_state must be one of {LOCAL_STATES}')
    entry = {'command_id': _token(command_id, 'command_id'), 'issued_at_sim_s': float(issued_at_sim_s),
             'kind': kind, 'arguments': copy.deepcopy(dict(arguments or {})), 'local_state': local_state}
    hits = forbidden_key_hits(entry)
    if hits:
        raise ContractViolation('own command history may not carry evaluation-only data: ' + '; '.join(hits))
    return entry


def belief_skeleton() -> dict:
    """The unknown-by-default self belief; only own RGB and own commands may raise its confidence."""
    return {'region': 'unknown', 'last_visual_anchor': None, 'last_requested_destination': None,
            'last_visually_confirmed_region': None, 'confidence': 'low', 'sources': []}


def _trim(values: Sequence | None, limit: int) -> list:
    items = list(values or ())
    return copy.deepcopy(items[-limit:] if limit and len(items) > limit else items)


# ---------------------------------------------------------------------------
# Per-call input

def build_call_input(*, robot_id: str, condition_name: str, request_id: str, sim_time_s: float,
                     static_map: Mapping, source: 'OrderSheetSource', own_rgb_refs: Sequence[Mapping] = (),
                     own_command_history: Sequence[Mapping] = (), inbox: Sequence[Mapping] | None = None,
                     self_belief: Mapping | None = None, seed: int | None = None,
                     team_rgb_refs: Sequence[Mapping] | None = None,
                     issued_orders: Sequence[Mapping] | None = None,
                     profile: Mapping = INPUT_PROFILE) -> dict:
    """Assemble one validated per-call payload for ``robot_id`` under ``condition_name``.

    The order sheet comes from the frozen ``OrderSheetSource`` and is checked for
    drift on every call, so a tampered sheet cannot reach a model. There is no
    switch to skip validation.
    """
    spec = condition(condition_name)
    if robot_id not in spec.actors:
        raise ContractViolation(f'{robot_id!r} is not an actor of {condition_name}')
    if not isinstance(source, OrderSheetSource):
        raise ContractViolation('build_call_input needs the run\'s frozen OrderSheetSource')
    source.assert_unchanged()
    payload = {'schema': PAYLOAD_SCHEMA, 'request_id': _token(request_id, 'request_id'), 'robot_id': robot_id,
               'condition': condition_name, 'sim_time_s': float(sim_time_s),
               'static_map': copy.deepcopy(dict(static_map)), 'order_sheet': source.sheet(),
               'channel': channel_section(condition_name, robot_id, seed)}
    allow = spec.input_allowlist
    if 'own_rgb_refs' in allow:
        payload['own_rgb_refs'] = _trim(own_rgb_refs, int(profile['own_rgb_frames']))
    if 'own_command_history' in allow:
        payload['own_command_history'] = _trim(own_command_history, int(profile['command_history_entries']))
    if 'self_belief' in allow:
        payload['self_belief'] = copy.deepcopy(dict(self_belief or belief_skeleton()))
    if 'inbox' in allow:
        payload['inbox'] = _trim(inbox, int(profile['inbox_messages']))
    elif inbox:
        raise ContractViolation(f'{condition_name} delivers no messages, so an inbox is not allowed')
    if spec.leader_rotation:
        if seed is None:
            raise ContractViolation(f'{condition_name} needs the seed to place the rotating leader')
        payload['leader_id'] = leader_for_seed(condition_name, seed)
        payload['role'] = role_of(condition_name, robot_id, seed)
    if 'team_rgb_refs' in allow:
        payload['team_rgb_refs'] = _trim(team_rgb_refs, len(ROBOTS) * int(profile['own_rgb_frames']))
    if 'issued_orders' in allow:
        payload['issued_orders'] = _trim(issued_orders, int(profile['command_history_entries']))
    validate_robot_payload(payload, seed=seed)
    return payload


def payload_sha256(payload: Mapping) -> str:
    return digest(payload)


def call_input_bundle(*, map_id: str, source: OrderSheetSource, condition_name: str, seed: int,
                      code_sha: str, execution_bundle_id: str, model: str, provider: str | None = None,
                      model_settings_sha256: str | None = None, prompt_template_sha256: str | None = None,
                      cost_profile_id: str | None = None, landmark_detail: str = 'full') -> dict:
    """Run-level provenance: contract, condition, map hashes, order sheet hashes, code and model."""
    return {'contract': condition_manifest(condition_name, seed), 'registry_sha256': registry_sha256(),
            'inputs': source.manifest(), 'map_id': map_id, 'landmark_detail': landmark_detail,
            'provenance': provenance(source=source, code_sha=code_sha,
                                     execution_bundle_id=execution_bundle_id, model=model,
                                     provider=provider, model_settings_sha256=model_settings_sha256,
                                     prompt_template_sha256=prompt_template_sha256,
                                     cost_profile_id=cost_profile_id)}


def provenance(*, source: OrderSheetSource, code_sha: str, execution_bundle_id: str, model: str,
               provider: str | None = None, model_settings_sha256: str | None = None,
               prompt_template_sha256: str | None = None, cost_profile_id: str | None = None) -> dict:
    """The ``provenance`` object every call record carries (closed keys, see PROVENANCE_KEYS)."""
    value = {'registry_sha256': registry_sha256(), 'order_sheet_sha256': source.sha256,
             'map_file_sha256': source.manifest()['map']['map_file_sha256'],
             'public_map_sha256': source.manifest()['map']['public_map_sha256'],
             'code_sha': code_sha, 'execution_bundle_id': execution_bundle_id, 'model': model,
             'provider': provider, 'model_settings_sha256': model_settings_sha256,
             'prompt_template_sha256': prompt_template_sha256, 'cost_profile_id': cost_profile_id,
             'input_profile_id': INPUT_PROFILE['profile_id']}
    unknown = set(value) - set(PROVENANCE_KEYS)
    if unknown:
        raise ContractViolation(f'provenance carries unknown key(s): {sorted(unknown)}')
    return value


# ---------------------------------------------------------------------------
# Log records (package D writes the cost fields, package I reads everything)

def call_log_record(*, run_id: str, condition_name: str, seed: int, actor: str, request_id: str,
                    call_index: int, trigger: str, requested_at_sim_s: float, released_at_sim_s: float,
                    cost_terms: Mapping, input_sha256: str, input_tokens: Mapping, output_tokens: int,
                    status: str, provenance: Mapping, http_attempts: int = 1,
                    wall_latency_s: float | None = None, action_id: str | None = None,
                    message_ids: Sequence[str] = (), decision_sources: Sequence[str] = (),
                    payload_validated: bool = True) -> dict:
    """One logical model call, with the SIM cost that package D charged for it."""
    if trigger not in TRIGGERS:
        raise ContractViolation(f'trigger must be one of {TRIGGERS}')
    record = {'schema': CALL_LOG_SCHEMA, 'run_id': run_id, 'condition': condition_name, 'seed': int(seed),
              'actor': actor, 'role': role_of(condition_name, actor, seed), 'request_id': request_id,
              'call_index': int(call_index), 'trigger': trigger,
              'requested_at_sim_s': float(requested_at_sim_s), 'released_at_sim_s': float(released_at_sim_s),
              'sim_cost_s': float(released_at_sim_s) - float(requested_at_sim_s),
              'cost_terms': dict(cost_terms), 'input_sha256': input_sha256, 'input_tokens': dict(input_tokens),
              'output_tokens': int(output_tokens),
              'wall_latency_s': None if wall_latency_s is None else float(wall_latency_s),
              'http_attempts': int(http_attempts), 'status': status, 'action_id': action_id,
              'message_ids': list(message_ids), 'decision_sources': list(decision_sources),
              'payload_validated': bool(payload_validated), 'provenance': dict(provenance)}
    return dict(validate_log_record(record))


def message_log_record(*, run_id: str, condition_name: str, seed: int, envelope: Mapping,
                       deliveries: Sequence[Mapping], delivery_delay_s: float, status: str,
                       rejected_reason: str | None = None, act: str | None = None) -> dict:
    """One message: what was sent, when each recipient got it and whether it was accepted."""
    body = envelope['body']
    free = condition(condition_name).encoding == 'free_ko'
    text = body.get('text') if isinstance(body, Mapping) else None
    report = free_text_report(text) if free else None
    times = [d['delivered_at_sim_s'] for d in deliveries
             if isinstance(d, Mapping) and d.get('delivered_at_sim_s') is not None]
    record = {'schema': MESSAGE_LOG_SCHEMA, 'run_id': run_id, 'condition': condition_name, 'seed': int(seed),
              'message_id': envelope['message_id'], 'sender': envelope['sender'],
              'recipients': list(envelope['recipients']), 'encoding': envelope['encoding'],
              'reply_to': envelope.get('reply_to'), 'created_at_sim_s': float(envelope['created_at_sim_s']),
              'delivered_at_sim_s': min(times) if times else None,
              'deliveries': [dict(d) for d in deliveries], 'delivery_delay_s': float(delivery_delay_s),
              'status': status, 'rejected_reason': rejected_reason, 'body': copy.deepcopy(body),
              'body_sha256': digest(body),
              'act': act if act is not None else (body.get('act') if isinstance(body, Mapping) else None),
              'chars': len(text) if isinstance(text, str) else 0,
              'korean_ok': None if report is None else report['ok']}
    return dict(validate_log_record(record))


def action_log_record(*, run_id: str, condition_name: str, seed: int, actor: str, action_id: str,
                      request_id: str, submitted_at_sim_s: float, kind: str, arguments: Mapping,
                      accepted: bool, local_state: str = 'command_issued', order_id: str | None = None,
                      role: str | None = None, rejected_reason: str | None = None) -> dict:
    """One submitted robot action. Submitting a command is not moving and not success."""
    record = {'schema': ACTION_LOG_SCHEMA, 'run_id': run_id, 'condition': condition_name, 'seed': int(seed),
              'actor': actor, 'action_id': action_id, 'request_id': request_id,
              'submitted_at_sim_s': float(submitted_at_sim_s), 'kind': kind,
              'arguments': copy.deepcopy(dict(arguments)), 'order_id': order_id, 'role': role,
              'accepted': bool(accepted), 'rejected_reason': rejected_reason, 'local_state': local_state}
    return dict(validate_log_record(record))


def static_map_for_call(map_bundle: Mapping) -> dict:
    """The ``static_map`` value of every call of a run (same bytes on every call)."""
    return static_map_section(map_bundle)


__all__ = ['SCENARIO_SCHEMA', 'ITEM_KINDS', 'REQUIRED_ROBOTS', 'FORMATIONS', 'ROLE_NAMES', 'IDENTITY',
           'INPUT_PROFILE', 'TRIGGERS', 'COMMANDER', 'load_scenario', 'validate_scenario', 'eval_section',
           'hidden_events', 'leader_rotation', 'order_sheet', 'OrderSheetSource', 'vocabulary',
           'own_rgb_ref', 'command_entry', 'belief_skeleton', 'build_call_input', 'payload_sha256',
           'call_input_bundle', 'provenance', 'call_log_record', 'message_log_record', 'action_log_record',
           'static_map_for_call']
