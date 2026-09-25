"""EVALUATION-ONLY metrics for the Korean-dialogue multi-robot study.

Scope and boundary
------------------
Everything in this module reads evaluation logs *after* a trial finished. The
referee (ground-truth) sub-log, the TOP camera and any derived verdict must
never re-enter a robot request, memory, wake-up or executor decision. The
functions here are pure: they take a trial record and return new dictionaries,
never mutating the input, so the same record can also be hashed/archived.

Package A (``harness/zone_study_contract.py``) owns the condition registry and
the call/message/action log schema. The conditions, their Korean labels and the
message encodings here come from A. Two record shapes are accepted:

* ``ugrp.zone_study_trial.v1`` (:data:`TRIAL_SCHEMA`) — the A-aligned trial
  envelope. Its ``calls``/``messages``/``actions`` rows ARE A's log records and
  are validated by ``A.validate_log_record``; :func:`parse_trial` maps them onto
  the ``requests``/``utterances`` views the metrics below read.
* ``ugrp.zone_study_trial.provisional.v1`` (:data:`PROVISIONAL_SCHEMA`) — the
  earlier hand-written shape, kept so archived pilot logs still parse.

Unknown schemas are refused, never silently accepted.

Study decisions (user, 2026-09-25/26) that this module encodes:

* four main conditions differing only in the communication channel, plus the
  all-seeing reference ceiling ``R`` which is *not* a main condition;
* the leader condition has one robot doubling as leader, rotating across seeds,
  hub-and-spoke only;
* denominators keep failures, aborts, timeouts and budget exhaustion, so a fast
  failure can never look better than a slow success;
* talking and thinking cost SIM time.
"""
from __future__ import annotations

import collections
import copy
import json
import math
import random
import re
import statistics
from pathlib import Path

from harness import zone_dialogue_metrics as zm
from harness.zone_study_contract import (ACTION_LOG_SCHEMA, CALL_LOG_SCHEMA, CONDITIONS as A_CONDITIONS,
                                         MAIN_CONDITIONS as A_MAIN_CONDITIONS, MESSAGE_LOG_SCHEMA,
                                         forbidden_key_hits, validate_log_record)

#: A-aligned trial envelope: ``calls``/``messages``/``actions`` are A log records.
TRIAL_SCHEMA = 'ugrp.zone_study_trial.v1'
PROVISIONAL_SCHEMA = 'ugrp.zone_study_trial.provisional.v1'
#: Schemas this module knows how to read.
SUPPORTED_SCHEMAS = (TRIAL_SCHEMA, PROVISIONAL_SCHEMA)

#: Main conditions in reporting order, then the reference ceiling (package A).
MAIN_CONDITIONS = A_MAIN_CONDITIONS
REFERENCE_CONDITION = next(name for name, c in A_CONDITIONS.items() if not c.is_main)
CONDITIONS = tuple(A_CONDITIONS)

#: Report labels: package A's ``korean_label`` with the study's numbering.
_NUMBER_KO = {'no_comm': '①', 'peer_ko': '②', 'leader_ko': '③', 'structured': '④',
              REFERENCE_CONDITION: 'R'}
CONDITION_LABELS_KO = {name: f'{_NUMBER_KO.get(name, "")} {c.korean_label}'.strip()
                       for name, c in A_CONDITIONS.items()}
#: Condition spellings of the provisional records, normalised onto package A's
#: names. The value as logged is kept in ``condition_as_logged``.
CONDITION_ALIASES = {'peer_structured': 'structured', 'central_rgb_reference': REFERENCE_CONDITION}
#: Utterance field names of the provisional records, mapped onto the ones used here.
UTTERANCE_ALIASES = {'from_robot': 'sender', 'sent_at_sim_s': 'sim_s',
                     'delivered_at_sim_s': 'delivered_sim_s', 'structured': 'message'}

#: Free-text and fixed-schema channels, using package A's encoding literals.
FREE_TEXT_ENCODINGS = ('free_ko',)
STRUCTURED_ENCODINGS = ('schema',)
#: Encoding spellings of the provisional records, normalised onto A's literals.
ENCODING_ALIASES = {'ko_free': 'free_ko', 'structured': 'schema'}

SUCCESS_END_REASON = 'orders_complete'
#: Every non-success terminal reason still counts in the denominators.
FAILURE_END_REASONS = (
    'sim_horizon', 'budget_exhausted', 'deadlock', 'aborted',
    'api_failure', 'policy_failure', 'orders_incomplete',
)
END_REASONS = (SUCCESS_END_REASON,) + FAILURE_END_REASONS

#: Robot-facing input keys allowed on every model call. Package A's per-condition
#: allowlists are the source of truth; the extra names are request bookkeeping the
#: provisional records used.
ALLOWED_INPUT_KEYS = frozenset().union(*(c.input_allowlist for c in A_CONDITIONS.values())) | {
    'static_map_figure', 'own_rgb', 'own_rgb_history', 'own_commands', 'own_command_state',
    'own_belief', 'action_schema', 'condition_instruction', 'dialogue_window',
}
#: Input keys that mean evaluation data leaked back into a robot request.
FORBIDDEN_INPUT_KEYS = frozenset({
    'top_rgb', 'top_camera', 'top_labels', 'top_zone_counts', 'nav_cam',
    'referee', 'referee_deliveries', 'ground_truth', 'gt_pose', 'gt_poses',
    'object_poses', 'robot_poses', 'measured_joints', 'contact_state',
    'grasp_success', 'placed', 'teacher_receipt', 'executor_receipt',
    'peer_rgb', 'peer_commands', 'peer_jobs', 'global_task_table',
    'hidden_event_schedule', 'simulator_state', 'body_ids', 'progress_rate',
})

#: Coarse act types the study reports on. Fine labels come from PR 172.
ACT_TYPES = ('report', 'request', 'order', 'objection', 'ack')
#: PR 172 (merged) rule labels mapped onto the study's coarse act types.
#: ``harness/zone_dialogue_metrics.ACT_RULES`` stays the source of truth for the
#: fine labels; this module only adds ``order`` and the coarse mapping.
FINE_TO_COARSE = {
    'report': 'report', 'inform_obstacle': 'report', 'standby': 'report', 'claim': 'report',
    'request': 'request', 'propose': 'request', 'question': 'request',
    'agree': 'ack', 'yield': 'ack',
    'refuse': 'objection', 'correct': 'objection',
}
#: Polite-request endings PR 172's ``request`` rule does not cover (it keys on
#: ``해 주세요``, so ``가져와 주세요`` / ``옮겨 주십시오`` need this supplement).
#: Added here instead of editing the merged pilot module.
REQUEST_RULE_EXTRA = r'주세요|주십시오|주시겠|주실 ?수|주시기|주시길|주라|해 ?줘|부탁'
#: Korean imperative/assignment cues for leader orders. Applied only when no
#: polite-request cue matched, so ``가져와 주세요`` stays ``request``.
ORDER_RULE = (r'하십시오|하시오|하라(?=[.!\s]|$)|해라|가라(?=[.!\s]|$)|가십시오|맡아라|맡으십시오|'
              r'배달하라|배달하십시오|운반하라|운반하십시오|이동하라|이동하십시오|'
              r'가져오라|가져오십시오|가져와(?=[.!]|$)|대기하라|대기하십시오|'
              r'중단하라|중단하십시오|양보하라|양보하십시오|배정한다|지시한다|명령한다')

IDLE_REASONS = ('thinking', 'speaking', 'await_order', 'team_rendezvous',
                'door_wait', 'unassigned', 'reobserve', 'other')

#: Claim kinds this module can check against the referee log.
CLAIM_KINDS = ('delivered', 'holding', 'blocked', 'absent')
CLAIM_CUES = {
    'delivered': r'배달했|배달 ?완료|놓았|내려놓|두었|옮겼|배송했|배송 ?완료|delivered',
    'holding': r'들고 ?있|잡았|파지했|집었|holding',
    'blocked': r'막혀|막힌|막았|차단|blocked',
    'absent': r'없습니다|없어|비었|비어 ?있|텅 ?비|absent|empty',
}
ZONE_RE = re.compile(r'(?<![A-Za-z0-9_-])([ABC])(?![A-Za-z0-9_])')
ITEM_RE = re.compile(r'(?<![A-Za-z0-9_])([a-z][a-z_]*-\d+)(?![0-9])')
PASSAGE_RE = re.compile(r'(?<![A-Za-z0-9_])((?:door|corridor|passage)_[a-z0-9_]+)')

#: Grounds prefixes a speaker may legitimately cite.
GROUND_PREFIXES = ('own-', 'command-', 'msg-', 'm-', 'belief-', 'map-', 'order-')
FORBIDDEN_GROUND_PREFIXES = ('top-', 'referee-', 'gt-', 'teacher-', 'peer-rgb-', 'peer-command-')

#: Korean compliance threshold on the literal-stripped Hangul ratio.
KOREAN_RATIO_THRESHOLD = 0.9
#: PAR-style penalty multiplier on the SIM horizon for non-success trials.
DEFAULT_PENALTY_FACTOR = 2.0
#: Pairs below this count get an interval-only note instead of any test.
SMALL_SAMPLE_PAIRS = 10
#: Window before a decision change in which inbound utterances are associated.
DEFAULT_LOOKBACK_S = 30.0


class TrialError(ValueError):
    """Raised for a trial record this module refuses to interpret."""


# --------------------------------------------------------------------------- #
# loading
# --------------------------------------------------------------------------- #

def parse_trial(obj):
    """Validate a trial record and return an independent copy.

    For :data:`TRIAL_SCHEMA` the ``calls``/``messages``/``actions`` rows are
    package A log records: each one is validated by ``A.validate_log_record`` and
    then mapped onto the ``requests``/``utterances`` views the metrics read. The
    input mapping is never mutated; callers keep their raw archived log.
    """
    if not isinstance(obj, dict):
        raise TrialError('trial record must be a JSON object')
    schema = obj.get('schema')
    if schema not in SUPPORTED_SCHEMAS:
        raise TrialError(f'unsupported schema {schema!r}; known: {SUPPORTED_SCHEMAS}')
    trial = copy.deepcopy(obj)
    condition = trial.get('condition')
    if condition in CONDITION_ALIASES:
        trial['condition_as_logged'] = condition
        condition = trial['condition'] = CONDITION_ALIASES[condition]
    if condition not in CONDITIONS:
        raise TrialError(f'unknown condition {condition!r}; known: {CONDITIONS}'
                         f' (aliases: {sorted(CONDITION_ALIASES)})')
    if schema == TRIAL_SCHEMA:
        _adapt_contract_rows(trial)
    for utt in _rows(trial, 'utterances'):
        for old, new in UTTERANCE_ALIASES.items():
            if old in utt and new not in utt:
                utt[new] = utt.pop(old)
        if utt.get('encoding') in ENCODING_ALIASES:
            utt['encoding_as_logged'] = utt['encoding']
            utt['encoding'] = ENCODING_ALIASES[utt['encoding']]
    reason = trial.get('end_reason')
    if reason not in END_REASONS:
        raise TrialError(f'unknown end_reason {reason!r}; known: {END_REASONS}')
    for key in ('trial_id', 'scenario', 'seed'):
        if trial.get(key) in (None, ''):
            raise TrialError(f'trial record needs {key}')
    if condition == 'leader_ko' and not trial.get('leader_id'):
        raise TrialError('leader_ko needs leader_id (rotates r1/r2/r3 across seeds)')
    if condition != 'leader_ko' and trial.get('leader_id'):
        raise TrialError(f'leader_id is only valid for leader_ko, not {condition}')
    horizon = _budget(trial).get('sim_horizon_s')
    if not isinstance(horizon, (int, float)) or horizon <= 0:
        raise TrialError('budget.sim_horizon_s must be a positive number')
    if trial.get('end_sim_s') is None:
        raise TrialError('trial record needs end_sim_s')
    return trial


def _adapt_contract_rows(trial):
    """Validate package A log rows in place and build the metric views.

    ``requests`` and ``utterances`` are derived, never authored: an A-aligned
    record that also carries them by hand is refused, so there is exactly one
    source for every number.
    """
    for key in ('requests', 'utterances'):
        if key in trial:
            raise TrialError(f'{TRIAL_SCHEMA} derives {key} from the package A log rows; '
                             'do not author it as well')
    expected = {'calls': CALL_LOG_SCHEMA, 'messages': MESSAGE_LOG_SCHEMA, 'actions': ACTION_LOG_SCHEMA}
    for key, log_schema in expected.items():
        for row in _rows(trial, key):
            if row.get('schema') != log_schema:
                raise TrialError(f'{key}[] must carry schema {log_schema}, got {row.get("schema")!r}')
            validate_log_record(row)
    trial['requests'] = [_request_view(row) for row in _rows(trial, 'calls')]
    trial['utterances'] = [_utterance_view(row) for row in _rows(trial, 'messages')]


def _request_view(call):
    """One package A call record as the request row the boundary audit reads.

    A's call record carries no ``input_keys``: the payload was validated at build
    time, and ``payload_validated`` is the auditable fact. A run that also stored
    the payload keys may pass them through ``input_keys``.
    """
    return {'request_id': call['request_id'], 'robot': call['actor'], 'role': call['role'],
            'sim_s': call['requested_at_sim_s'], 'released_sim_s': call['released_at_sim_s'],
            'sim_cost_s': call['sim_cost_s'], 'trigger': call['trigger'], 'status': call['status'],
            'payload_validated': call['payload_validated'], 'input_sha256': call['input_sha256'],
            'decision_sources': list(call['decision_sources']),
            'message_ids': list(call['message_ids']),
            'http_attempts': call['http_attempts'], 'output_tokens': call['output_tokens'],
            'input_tokens': dict(call['input_tokens'])}


def _utterance_view(message):
    """One package A message record as the utterance row the dialogue metrics read.

    The SIM cost of speaking is charged on the CALL that produced the message
    (package D), so ``sim_cost_s`` stays ``None`` here and the authoritative talk
    cost is ``model.sim_cost_s``.
    """
    body = message.get('body')
    free = message.get('encoding') in FREE_TEXT_ENCODINGS
    view = {'message_id': message['message_id'], 'sender': message['sender'],
            'recipients': list(message['recipients']), 'encoding': message['encoding'],
            'sim_s': message['created_at_sim_s'], 'delivered_sim_s': message['delivered_at_sim_s'],
            'reply_to': message.get('reply_to'), 'status': message['status'],
            'act': message.get('act'), 'chars': message.get('chars'),
            'korean_ok': message.get('korean_ok'), 'sim_cost_s': None}
    if free:
        view['text'] = body.get('text') if isinstance(body, dict) else None
    else:
        view['message'] = copy.deepcopy(body)
    return view


def load_trial(path):
    return parse_trial(json.loads(Path(path).read_text()))


def load_trials(paths):
    """Load trial records from files and/or directories (``*.json``), sorted."""
    files = []
    for entry in paths:
        entry = Path(entry)
        if entry.is_dir():
            files.extend(sorted(p for p in entry.rglob('*.json') if p.name != 'metrics.json'))
        else:
            files.append(entry)
    trials, seen = [], {}
    for path in files:
        trial = load_trial(path)
        key = trial['trial_id']
        if key in seen:
            raise TrialError(f'duplicate trial_id {key!r} in {path} and {seen[key]}')
        seen[key] = path
        trial['source_path'] = str(path)
        trials.append(trial)
    return trials


def _budget(trial):
    budget = trial.get('budget')
    return budget if isinstance(budget, dict) else {}


def _referee(trial):
    """Ground-truth sub-log. EVALUATION-ONLY; never returned to a robot."""
    referee = trial.get('referee')
    return referee if isinstance(referee, dict) else {}


def _rows(container, key):
    rows = container.get(key)
    return [r for r in rows if isinstance(r, dict)] if isinstance(rows, list) else []


# --------------------------------------------------------------------------- #
# input-boundary audit
# --------------------------------------------------------------------------- #

def audit_input_boundary(trial):
    """Check that no recorded robot request carried evaluation-only inputs.

    Returns a report with one row per offending request. ``clean`` is False as
    soon as a forbidden or unknown input key, an unvalidated payload, a forbidden
    grounds citation, or a channel/topology violation appears.

    For an A-aligned record the per-call boundary was enforced when the payload
    was built, so ``payload_validated`` is the auditable fact; a run that also
    archived the payload keys may add ``input_keys`` and both are checked.
    """
    condition = trial['condition']
    leaks, unknown, unvalidated = [], [], []
    for req in _rows(trial, 'requests'):
        keys = req.get('input_keys')
        keys = list(keys) if isinstance(keys, (list, tuple)) else []
        bad = sorted(set(keys) & FORBIDDEN_INPUT_KEYS
                     | {k for k in keys if forbidden_key_hits({k: None})})
        odd = sorted(set(keys) - FORBIDDEN_INPUT_KEYS - ALLOWED_INPUT_KEYS)
        if bad:
            leaks.append({'request_id': req.get('request_id'), 'robot': req.get('robot'),
                          'sim_s': req.get('sim_s'), 'forbidden_input_keys': bad})
        if odd:
            unknown.append({'request_id': req.get('request_id'), 'robot': req.get('robot'),
                            'unknown_input_keys': odd})
        if req.get('payload_validated') is False:
            unvalidated.append({'request_id': req.get('request_id'), 'robot': req.get('robot'),
                                'status': req.get('status')})
    if condition == REFERENCE_CONDITION:
        # R is the all-seeing commander: peer RGB is expected there, so the
        # request audit is reported but does not gate the main conditions.
        leaks = [dict(row, reference_condition=True) for row in leaks]
    grounds = []
    for utt in _rows(trial, 'utterances'):
        bad = [g for g in (utt.get('grounds') or [])
               if isinstance(g, str) and g.lower().startswith(FORBIDDEN_GROUND_PREFIXES)]
        if bad:
            grounds.append({'message_id': utt.get('message_id'), 'forbidden_grounds': bad})
    channel = channel_compliance(trial)
    clean = (not leaks and not unknown and not grounds and not unvalidated
             and not channel['violations'] and condition != REFERENCE_CONDITION)
    return {'condition': condition, 'requests_checked': len(_rows(trial, 'requests')),
            'input_leaks': leaks, 'unknown_input_keys': unknown,
            'unvalidated_payloads': unvalidated,
            'forbidden_grounds': grounds, 'channel_violations': channel['violations'],
            'clean': clean,
            'note': 'R은 전지적 참조 상한이므로 주 조건 경계 판정에서 제외한다.'
                    if condition == REFERENCE_CONDITION else
                    '평가 로그·정답·TOP은 로봇 입력으로 되돌리지 않는다.'}


def channel_compliance(trial):
    """Per-condition channel rules: no_comm silence, hub-and-spoke, no free text."""
    condition, leader = trial['condition'], trial.get('leader_id')
    robots = list(trial.get('robots') or ())
    violations, sends = [], collections.Counter()
    for utt in _rows(trial, 'utterances'):
        sender = utt.get('sender')
        recipients = [r for r in (utt.get('recipients') or []) if r]
        encoding = utt.get('encoding')
        text = utt.get('text') or ''
        sends[sender] += 1
        if condition == 'no_comm':
            violations.append({'message_id': utt.get('message_id'), 'kind': 'no_comm_message'})
            continue
        if condition == 'leader_ko':
            if sender != leader and any(r != leader for r in recipients):
                violations.append({'message_id': utt.get('message_id'), 'kind': 'follower_to_follower',
                                   'sender': sender, 'recipients': recipients})
            if sender == leader and len(recipients) > 1:
                violations.append({'message_id': utt.get('message_id'), 'kind': 'leader_broadcast',
                                   'recipients': recipients})
        if condition == 'structured':
            if encoding not in STRUCTURED_ENCODINGS:
                violations.append({'message_id': utt.get('message_id'), 'kind': 'non_structured_encoding',
                                   'encoding': encoding})
            if text.strip():
                violations.append({'message_id': utt.get('message_id'), 'kind': 'free_text_in_structured',
                                   'chars': len(text)})
        elif condition in ('peer_ko', 'leader_ko') and encoding not in FREE_TEXT_ENCODINGS:
            violations.append({'message_id': utt.get('message_id'), 'kind': 'non_korean_encoding',
                               'encoding': encoding})
        for recipient in recipients:
            if recipient == sender:
                violations.append({'message_id': utt.get('message_id'), 'kind': 'self_addressed'})
            if robots and recipient not in robots and recipient != 'commander':
                violations.append({'message_id': utt.get('message_id'), 'kind': 'unknown_recipient',
                                   'recipient': recipient})
    return {'violations': violations, 'sends_by_actor': dict(sends)}


# --------------------------------------------------------------------------- #
# efficiency
# --------------------------------------------------------------------------- #

def _ordered_items(trial):
    """Item ids the order sheet asked for; falls back to per-order counts."""
    ids, count = [], 0
    for order in _rows(trial, 'orders'):
        item_ids = [i for i in (order.get('item_ids') or []) if i]
        ids.extend(item_ids)
        count += int(order.get('count') or len(item_ids) or 0)
    return ids, max(count, len(ids))


def _destinations(trial):
    dest = {}
    for order in _rows(trial, 'orders'):
        zone = order.get('destination_zone')
        for item in (order.get('item_ids') or []):
            if item:
                dest[item] = zone
    return dest


def efficiency_metrics(trial, penalty_factor=DEFAULT_PENALTY_FACTOR):
    """SIM makespan (talk cost included), deliveries, idle, conflicts, cost.

    Failures keep the denominators: ``par_makespan_sim_s`` charges every
    non-success trial ``penalty_factor × horizon``, which is strictly worse than
    any success because a success must land before the horizon.
    """
    if penalty_factor < 1:
        raise TrialError('penalty_factor must be >= 1 so failures cannot beat successes')
    referee, budget = _referee(trial), _budget(trial)
    horizon = float(budget['sim_horizon_s'])
    t0 = float(trial.get('t0_sim_s') or 0.0)
    end = float(trial['end_sim_s'])
    success = trial['end_reason'] == SUCCESS_END_REASON
    elapsed = max(end - t0, 0.0)
    if success and elapsed > horizon:
        raise TrialError(f'{trial["trial_id"]}: success at {elapsed}s exceeds horizon {horizon}s')
    charged = elapsed if success else penalty_factor * horizon

    ordered_ids, ordered_count = _ordered_items(trial)
    dest = _destinations(trial)
    delivered, misdelivered, seen = {}, {}, set()
    for row in _rows(referee, 'deliveries'):
        item = row.get('item_id')
        if not item or item in seen:
            continue            # count each item once, first referee delivery wins
        seen.add(item)
        zone = row.get('zone')
        correct = row.get('correct')
        if correct is None:
            correct = (dest.get(item) == zone) if item in dest else None
        (delivered if correct else misdelivered)[item] = {'zone': zone, 'sim_s': row.get('sim_s')}
    surplus = sorted(set(delivered) - set(ordered_ids)) if ordered_ids else []

    idle_raw = trial.get('idle') if isinstance(trial.get('idle'), dict) else {}
    idle_by_reason = collections.Counter()
    idle_by_robot = {}
    for robot, reasons in idle_raw.items():
        if not isinstance(reasons, dict):
            continue
        total = 0.0
        for reason, seconds in reasons.items():
            value = float(seconds or 0.0)
            idle_by_reason[reason if reason in IDLE_REASONS else 'other'] += value
            total += value
        idle_by_robot[robot] = round(total, 4)
    robots = len(trial.get('robots') or ()) or len(idle_by_robot) or 1
    idle_total = sum(idle_by_robot.values())

    conflicts = _rows(referee, 'conflicts')
    deadlocks = _rows(referee, 'deadlocks')
    replans = _rows(trial, 'replans')
    model = trial.get('model') if isinstance(trial.get('model'), dict) else {}
    tokens = model.get('tokens') if isinstance(model.get('tokens'), dict) else {}
    sim_cost = model.get('sim_cost_s') if isinstance(model.get('sim_cost_s'), dict) else {}
    talk_cost = float(sim_cost.get('talk') or 0.0) + float(sim_cost.get('delivery') or 0.0)
    think_cost = float(sim_cost.get('think') or 0.0)
    latencies = [float(v) for v in (model.get('wall_latency_ms') or []) if v is not None]

    return {
        'trial_id': trial['trial_id'], 'condition': trial['condition'],
        'scenario': trial['scenario'], 'seed': trial['seed'],
        'leader_id': trial.get('leader_id'),
        'end_reason': trial['end_reason'], 'success': bool(success),
        'censored': not success,
        'sim_horizon_s': round(horizon, 4),
        'makespan_sim_s': round(elapsed, 4),
        'makespan_success_only_s': round(elapsed, 4) if success else None,
        'par_makespan_sim_s': round(charged, 4),
        'penalty_factor': penalty_factor,
        'talk_sim_cost_s': round(talk_cost, 4),
        'think_sim_cost_s': round(think_cost, 4),
        'talk_share_of_makespan': _ratio(talk_cost, elapsed),
        'ordered_items': ordered_count,
        'delivered_items': len(delivered),
        'misdelivered_items': len(misdelivered),
        'surplus_items': len(surplus),
        'undelivered_items': max(ordered_count - len(delivered), 0),
        'delivery_rate': _ratio(len(delivered), ordered_count),
        'par_sim_s_per_delivered': round(charged / len(delivered), 4) if delivered else None,
        'idle_robot_s': round(idle_total, 4),
        'idle_share': _ratio(idle_total, robots * charged),
        'idle_by_reason': {k: round(v, 4) for k, v in sorted(idle_by_reason.items())},
        'idle_by_robot': idle_by_robot,
        'conflicts': len(conflicts),
        'conflicts_by_kind': dict(collections.Counter(c.get('kind', 'other') for c in conflicts)),
        'deadlocks': len(deadlocks),
        'deadlock_sim_s': round(sum(float(d.get('duration_s') or 0.0) for d in deadlocks), 4),
        'replans': len(replans),
        'replans_by_kind': dict(collections.Counter(r.get('kind', 'other') for r in replans)),
        'model_calls': int(model.get('logical_calls') or 0),
        'http_attempts': int(model.get('http_attempts') or 0),
        'tokens_input': int(tokens.get('input') or 0),
        'tokens_output': int(tokens.get('output') or 0),
        'tokens_image': int(tokens.get('image') or 0),
        'tokens_cached': int(tokens.get('cached') or 0),
        'tokens_total': int(sum(int(tokens.get(k) or 0) for k in ('input', 'output', 'image'))),
        'model_calls_per_delivered': round(int(model.get('logical_calls') or 0) / len(delivered), 4)
                                     if delivered else None,
        'wall_latency_ms_mean': round(statistics.mean(latencies), 2) if latencies else None,
        'budget_http_attempts': budget.get('http_attempts'),
        'budget_exhausted': trial['end_reason'] == 'budget_exhausted',
    }


def _ratio(num, den):
    return None if not den else round(num / den, 9)


# --------------------------------------------------------------------------- #
# dialogue
# --------------------------------------------------------------------------- #

def _labels(trial):
    """Item ids that must stay literal in Korean prose."""
    ids, _ = _ordered_items(trial)
    return tuple(ids)


#: Identifier-like study words that are literal references, not code-switching.
#: Same convention as PR 172's analysis script (underscore names are literals).
STUDY_KEY_WORDS = ('order_id', 'item_id', 'item_ids', 'destination_zone', 'required_robots',
                   'pickup_bay', 'slot', 'initial_location', 'robot_id', 'request_id',
                   'message_id', 'reply_to', 'recipients', 'observed_at_sim_s',
                   'location_ref', 'confidence', 'unknown', 'suspected', 'clear',
                   'blocked', 'present', 'absent', 'held', 'placed')


def literal_tokens(trial):
    """Literal tokens excluded from the Hangul ratio and the code-switch count.

    Sources are declarative, never the live simulator: the order sheet (order and
    item ids, kinds), the static map projection the record carries in
    ``literals``, passage names the referee log names, and the id-bearing fields
    of structured messages. An English word that is *not* declared anywhere stays
    counted as code-switching, which is the intended reading.
    """
    tokens = set(STUDY_KEY_WORDS)
    for order in _rows(trial, 'orders'):
        for key in ('order_id', 'kind'):
            if order.get(key):
                tokens.add(str(order[key]))
        tokens.update(str(i) for i in (order.get('item_ids') or []) if i)
        if order.get('initial_location') and isinstance(order['initial_location'], dict):
            tokens.update(str(v) for v in order['initial_location'].values() if v)
    declared = trial.get('literals')
    if isinstance(declared, (list, tuple)):
        tokens.update(str(t) for t in declared if t)
    for row in _rows(_referee(trial), 'blockages'):
        if row.get('passage'):
            tokens.add(str(row['passage']))
    for utt in _rows(trial, 'utterances'):
        message = utt.get('message')
        if isinstance(message, dict):
            for key in ('item', 'passage', 'location_ref', 'role', 'act', 'state'):
                if message.get(key):
                    tokens.add(str(message[key]))
        if utt.get('location_ref'):
            tokens.add(str(utt['location_ref']))
    return tuple(sorted(tokens))


def act_types(utterance):
    """Coarse study act types plus PR 172 fine labels, multi-label.

    Free text uses PR 172's ``ACT_RULES`` (merged) and adds ``order`` for the
    leader condition. Structured messages map their ``act`` enum directly.
    """
    encoding = utterance.get('encoding')
    if encoding in STRUCTURED_ENCODINGS:
        message = utterance.get('message')
        fine = zm.struct_acts(message if isinstance(message, dict) else None)
        coarse = sorted({_STRUCT_TO_COARSE.get(f, 'report') for f in fine if f != 'silence'})
        return {'fine': fine, 'coarse': coarse or (['silence'] if 'silence' in fine else ['report'])}
    text = utterance.get('text') or ''
    fine = list(zm.dialogue_acts(text))
    if fine == ['silence']:
        return {'fine': fine, 'coarse': ['silence']}
    if 'request' not in fine and re.search(REQUEST_RULE_EXTRA, text):
        fine.append('request')
    if 'other' in fine and len(fine) > 1:
        fine.remove('other')
    if 'request' not in fine and re.search(ORDER_RULE, text):
        fine.append('order')
    coarse = sorted({FINE_TO_COARSE.get(f, 'order' if f == 'order' else 'report') for f in fine})
    return {'fine': sorted(fine), 'coarse': coarse}


_STRUCT_TO_COARSE = {
    'inform': 'report', 'propose': 'request', 'request': 'request',
    'accept': 'ack', 'yield': 'ack', 'reject': 'objection',
    'correct': 'objection', 'cancel': 'order', 'order': 'order',
}


def extract_claims(utterance, labels=()):
    """Checkable propositions in an utterance.

    An explicit ``claims`` list (Package A) wins. Otherwise rule-based cues plus
    literal ids give ``delivered/holding/blocked/absent`` claims, matching the
    literal-id policy of PR 172's metrics.
    """
    given = utterance.get('claims')
    if isinstance(given, list):
        return [dict(c) for c in given if isinstance(c, dict) and c.get('type') in CLAIM_KINDS]
    message = utterance.get('message')
    if utterance.get('encoding') in STRUCTURED_ENCODINGS and isinstance(message, dict):
        return _structured_claims(message)
    text = utterance.get('text') or ''
    if not text.strip():
        return []
    return _text_claims(text, utterance, labels)


SENTENCE_SPLIT = re.compile(r'(?<=[.!?。])\s+|\n+')


def _ids(fragment, labels):
    items = [m for m in ITEM_RE.findall(fragment) if not labels or m in labels]
    return items, ZONE_RE.findall(fragment), PASSAGE_RE.findall(fragment)


def _text_claims(text, utterance, labels):
    """Claims scoped to the sentence carrying the cue.

    Sentence scoping stops ``door_narrow가 막혀 있습니다. door_wide로 우회하십시오.``
    from also asserting that ``door_wide`` is blocked. Only the *complement* of a
    claim (the zone of a delivery, the item of a hold) falls back to ids named
    elsewhere in the same utterance.
    """
    all_items, all_zones, _ = _ids(text, labels)
    claims, seen = [], set()

    def add(claim):
        key = tuple(sorted(claim.items(), key=lambda kv: kv[0]))
        if key not in seen:
            seen.add(key)
            claims.append(claim)

    for sentence in SENTENCE_SPLIT.split(text):
        if not sentence.strip():
            continue
        items, zones, passages = _ids(sentence, labels)
        if re.search(CLAIM_CUES['delivered'], sentence):
            for item in items or all_items or [None]:
                add({'type': 'delivered', 'item_id': item,
                     'zone': (zones or all_zones or [None])[0]})
        if re.search(CLAIM_CUES['holding'], sentence):
            for item in items or all_items or [None]:
                add({'type': 'holding', 'item_id': item, 'robot': utterance.get('sender')})
        if re.search(CLAIM_CUES['blocked'], sentence):
            for passage in passages or [None]:
                add({'type': 'blocked', 'passage': passage})
        if re.search(CLAIM_CUES['absent'], sentence):
            for item in items or all_items or [None]:
                add({'type': 'absent', 'item_id': item,
                     'location_ref': utterance.get('location_ref')})
    return claims


def _structured_claims(message):
    act, state = message.get('act'), message.get('state')
    item, zone = message.get('item'), message.get('zone')
    if act not in ('inform', 'correct'):
        return []
    if state == 'placed':
        return [{'type': 'delivered', 'item_id': item, 'zone': zone}]
    if state == 'held':
        return [{'type': 'holding', 'item_id': item, 'robot': message.get('sender')}]
    if state == 'blocked':
        return [{'type': 'blocked', 'passage': message.get('passage')}]
    if state == 'absent':
        return [{'type': 'absent', 'item_id': item, 'location_ref': message.get('location_ref')}]
    return []


def check_claim(claim, trial, at_sim_s):
    """Compare one claim with the referee log as of ``at_sim_s``.

    Returns ``true`` / ``false`` / ``unverifiable``. ``unverifiable`` is used
    whenever the referee log does not carry the needed sub-log; it is never
    silently turned into a failure or a success.
    """
    referee = _referee(trial)
    kind = claim.get('type')
    if kind == 'delivered':
        rows = _rows(referee, 'deliveries')
        if 'deliveries' not in referee:
            return 'unverifiable'
        item, zone = claim.get('item_id'), claim.get('zone')
        if item is None:
            return 'unverifiable'
        for row in rows:
            if row.get('item_id') != item:
                continue
            when = row.get('sim_s')
            if when is not None and at_sim_s is not None and float(when) > float(at_sim_s):
                continue
            if zone is None or row.get('zone') == zone:
                return 'true'
        return 'false'
    if kind == 'holding':
        if 'holds' not in referee:
            return 'unverifiable'
        item, robot = claim.get('item_id'), claim.get('robot')
        if item is None or at_sim_s is None:
            return 'unverifiable'
        for row in _rows(referee, 'holds'):
            if row.get('item_id') != item or (robot and row.get('robot') != robot):
                continue
            start = float(row.get('from_s') or 0.0)
            stop = row.get('to_s')
            if start <= float(at_sim_s) and (stop is None or float(at_sim_s) <= float(stop)):
                return 'true'
        return 'false'
    if kind == 'blocked':
        if 'blockages' not in referee:
            return 'unverifiable'
        passage = claim.get('passage')
        if passage is None or at_sim_s is None:
            return 'unverifiable'
        for row in _rows(referee, 'blockages'):
            if row.get('passage') != passage:
                continue
            start = float(row.get('from_s') or 0.0)
            stop = row.get('to_s')
            if start <= float(at_sim_s) and (stop is None or float(at_sim_s) <= float(stop)):
                return 'true'
        return 'false'
    if kind == 'absent':
        if 'slot_states' not in referee:
            return 'unverifiable'
        item = claim.get('item_id')
        for row in _rows(referee, 'slot_states'):
            if row.get('item_id') != item:
                continue
            when = row.get('sim_s')
            if when is not None and at_sim_s is not None and float(when) > float(at_sim_s):
                continue
            return 'true' if row.get('present') is False else 'false'
        return 'unverifiable'
    return 'unverifiable'


def grounds_verdict(utterance):
    """Did the speaker cite allowed grounds (own camera, own commands, inbox)?"""
    grounds = [g for g in (utterance.get('grounds') or []) if isinstance(g, str)]
    if not grounds:
        return 'unknown'
    if any(g.lower().startswith(FORBIDDEN_GROUND_PREFIXES) for g in grounds):
        return 'forbidden'
    if all(g.lower().startswith(GROUND_PREFIXES) for g in grounds):
        return 'grounded'
    return 'unknown'


def dialogue_metrics(trial, lookback_s=DEFAULT_LOOKBACK_S):
    """Korean compliance, truthfulness, act types, utterance→decision association."""
    labels = _labels(trial)
    extra = literal_tokens(trial)
    utterances = _rows(trial, 'utterances')
    rows, korean_ok, korean_checked, silent = [], 0, 0, 0
    code_switch, id_issue_rows = [], []
    verdicts = collections.Counter()
    grounds = collections.Counter()
    coarse_counts, fine_counts = collections.Counter(), collections.Counter()
    for utt in utterances:
        text = utt.get('text') or ''
        free = utt.get('encoding') in FREE_TEXT_ENCODINGS
        ratio = zm.hangul_ratio(text, labels, extra) if free else None
        english = zm.english_words(text, labels, extra) if free else []
        issues = zm.id_issues(text, labels) if free else []
        if free:
            if ratio is None:
                silent += 1     # silence is never counted as Korean success
            else:
                korean_checked += 1
                korean_ok += int(ratio >= KOREAN_RATIO_THRESHOLD)
            if english:
                code_switch.append({'message_id': utt.get('message_id'), 'english_words': english})
            if issues:
                id_issue_rows.append({'message_id': utt.get('message_id'), 'id_issues': issues})
        acts = act_types(utt)
        coarse_counts.update(acts['coarse'])
        fine_counts.update(acts['fine'])
        claims = extract_claims(utt, labels)
        at = utt.get('sim_s')
        claim_rows = [dict(c, verdict=check_claim(c, trial, at)) for c in claims]
        verdicts.update(r['verdict'] for r in claim_rows)
        ground = grounds_verdict(utt)
        grounds[ground] += 1
        rows.append({'message_id': utt.get('message_id'), 'sender': utt.get('sender'),
                     'recipients': list(utt.get('recipients') or ()), 'sim_s': at,
                     'encoding': utt.get('encoding'),
                     'hangul_ratio': None if ratio is None else round(ratio, 4),
                     'english_words': english, 'id_issues': issues,
                     'acts': acts, 'claims': claim_rows, 'grounds': ground,
                     'chars': len(text), 'sim_cost_s': utt.get('sim_cost_s')})
    checked = sum(verdicts.values())
    influence = decision_influence(trial, lookback_s=lookback_s)
    channel = channel_compliance(trial)
    return {
        'trial_id': trial['trial_id'], 'condition': trial['condition'],
        'scenario': trial['scenario'], 'seed': trial['seed'],
        'utterances': len(utterances),
        'delivery_edges': sum(len(u.get('recipients') or ()) for u in utterances),
        'broadcasts': sum(1 for u in utterances if len(u.get('recipients') or ()) > 1),
        'utterance_sim_cost_s': round(sum(float(u.get('sim_cost_s') or 0.0) for u in utterances), 4),
        'korean_checked': korean_checked, 'korean_ok': korean_ok,
        'korean_share': _ratio(korean_ok, korean_checked),
        'silent_messages': silent,
        'code_switch_messages': len(code_switch), 'code_switch': code_switch,
        'id_issue_messages': len(id_issue_rows), 'id_issues': id_issue_rows,
        'claims_checked': checked,
        'claims_true': verdicts['true'], 'claims_false': verdicts['false'],
        'claims_unverifiable': verdicts['unverifiable'],
        'truthful_share': _ratio(verdicts['true'], verdicts['true'] + verdicts['false']),
        'grounds_by_verdict': dict(grounds),
        'acts_coarse': dict(coarse_counts), 'acts_fine': dict(fine_counts),
        'channel_violations': len(channel['violations']),
        'decision_influence': influence,
        'messages': rows,
        'note': '사실성은 평가 로그 대조 결과이며 로봇 입력으로 되돌리지 않는다. '
                '선행 발화는 연관이고 인과가 아니다.',
    }


def decision_influence(trial, lookback_s=DEFAULT_LOOKBACK_S):
    """Which inbound utterances preceded each decision change (association only)."""
    changes = _rows(trial, 'decision_changes') or [
        dict(r, kind=r.get('kind', 'replan')) for r in _rows(trial, 'replans')]
    inbound = []
    for utt in _rows(trial, 'utterances'):
        at = utt.get('delivered_sim_s', utt.get('sim_s'))
        if at is None:
            continue
        for recipient in (utt.get('recipients') or ()):
            inbound.append((float(at), recipient, utt))
    rows, with_prior = [], 0
    act_counter = collections.Counter()
    for change in changes:
        at, robot = change.get('sim_s'), change.get('robot')
        prior = []
        if at is not None and robot:
            for when, recipient, utt in inbound:
                if recipient == robot and float(at) - lookback_s <= when <= float(at):
                    prior.append({'message_id': utt.get('message_id'), 'sender': utt.get('sender'),
                                  'delivered_sim_s': when, 'acts': act_types(utt)['coarse']})
                    act_counter.update(act_types(utt)['coarse'])
        with_prior += bool(prior)
        rows.append({'sim_s': at, 'robot': robot, 'kind': change.get('kind'),
                     'reason': change.get('reason'), 'preceding_inbound': prior})
    return {'lookback_s': lookback_s, 'decision_changes': len(rows),
            'changes_with_prior_inbound': with_prior,
            'changes_without_prior_inbound': len(rows) - with_prior,
            'prior_share': _ratio(with_prior, len(rows)),
            'preceding_acts': dict(act_counter), 'rows': rows}


def trial_metrics(trial, penalty_factor=DEFAULT_PENALTY_FACTOR, lookback_s=DEFAULT_LOOKBACK_S):
    """Efficiency + dialogue + boundary audit for one trial, without mutating it."""
    before = json.dumps(trial, sort_keys=True, ensure_ascii=False, default=str)
    out = {'efficiency': efficiency_metrics(trial, penalty_factor=penalty_factor),
           'dialogue': dialogue_metrics(trial, lookback_s=lookback_s),
           'boundary': audit_input_boundary(trial),
           'provenance': dict(trial.get('provenance') or {}),
           'source_path': trial.get('source_path')}
    after = json.dumps(trial, sort_keys=True, ensure_ascii=False, default=str)
    if before != after:
        raise TrialError('evaluation mutated the trial record; metrics must be read-only')
    return out


# --------------------------------------------------------------------------- #
# cohort summary
# --------------------------------------------------------------------------- #
#: Cohort metrics reported per condition. ``None`` values are dropped from the
#: mean but the trial still counts in ``trials``.
SUMMARY_METRICS = (
    'par_makespan_sim_s', 'makespan_sim_s', 'makespan_success_only_s',
    'talk_sim_cost_s', 'think_sim_cost_s', 'delivery_rate', 'delivered_items',
    'misdelivered_items', 'undelivered_items', 'idle_robot_s', 'idle_share',
    'conflicts', 'deadlocks', 'deadlock_sim_s', 'replans', 'model_calls',
    'http_attempts', 'tokens_total', 'tokens_input', 'tokens_output',
)


def summarise(trials, penalty_factor=DEFAULT_PENALTY_FACTOR, lookback_s=DEFAULT_LOOKBACK_S):
    """Per-condition cohort summary keeping failures in every denominator."""
    per_trial = [trial_metrics(t, penalty_factor, lookback_s) for t in trials]
    by_condition = collections.defaultdict(list)
    for row in per_trial:
        by_condition[row['efficiency']['condition']].append(row)
    conditions = {}
    for condition in CONDITIONS:
        rows = by_condition.get(condition)
        if not rows:
            continue
        eff = [r['efficiency'] for r in rows]
        dia = [r['dialogue'] for r in rows]
        charged = sum(e['par_makespan_sim_s'] for e in eff)
        delivered = sum(e['delivered_items'] for e in eff)
        ordered = sum(e['ordered_items'] for e in eff)
        conditions[condition] = {
            'label_ko': CONDITION_LABELS_KO[condition],
            'is_reference': condition == REFERENCE_CONDITION,
            'trials': len(eff),
            'successes': sum(e['success'] for e in eff),
            'success_rate': _ratio(sum(e['success'] for e in eff), len(eff)),
            'end_reasons': dict(collections.Counter(e['end_reason'] for e in eff)),
            'censored_trials': sum(e['censored'] for e in eff),
            'seeds': sorted({e['seed'] for e in eff}),
            'leader_ids': sorted({e['leader_id'] for e in eff if e['leader_id']}),
            'metrics': {name: _mean([e.get(name) for e in eff]) for name in SUMMARY_METRICS},
            'cohort_delivered_items': delivered,
            'cohort_ordered_items': ordered,
            'cohort_delivery_rate': _ratio(delivered, ordered),
            'cohort_par_sim_s_per_delivered': round(charged / delivered, 4) if delivered else None,
            'cohort_model_calls_per_delivered': round(
                sum(e['model_calls'] for e in eff) / delivered, 4) if delivered else None,
            'idle_by_reason': _sum_dicts(e['idle_by_reason'] for e in eff),
            'replans_by_kind': _sum_dicts(e['replans_by_kind'] for e in eff),
            'conflicts_by_kind': _sum_dicts(e['conflicts_by_kind'] for e in eff),
            'dialogue': {
                'utterances': sum(d['utterances'] for d in dia),
                'delivery_edges': sum(d['delivery_edges'] for d in dia),
                'korean_checked': sum(d['korean_checked'] for d in dia),
                'korean_ok': sum(d['korean_ok'] for d in dia),
                'korean_share': _ratio(sum(d['korean_ok'] for d in dia),
                                       sum(d['korean_checked'] for d in dia)),
                'silent_messages': sum(d['silent_messages'] for d in dia),
                'code_switch_messages': sum(d['code_switch_messages'] for d in dia),
                'id_issue_messages': sum(d['id_issue_messages'] for d in dia),
                'claims_true': sum(d['claims_true'] for d in dia),
                'claims_false': sum(d['claims_false'] for d in dia),
                'claims_unverifiable': sum(d['claims_unverifiable'] for d in dia),
                'truthful_share': _ratio(sum(d['claims_true'] for d in dia),
                                         sum(d['claims_true'] for d in dia)
                                         + sum(d['claims_false'] for d in dia)),
                'acts_coarse': _sum_dicts(d['acts_coarse'] for d in dia),
                'acts_fine': _sum_dicts(d['acts_fine'] for d in dia),
                'grounds_by_verdict': _sum_dicts(d['grounds_by_verdict'] for d in dia),
                'decision_changes': sum(d['decision_influence']['decision_changes'] for d in dia),
                'changes_with_prior_inbound': sum(
                    d['decision_influence']['changes_with_prior_inbound'] for d in dia),
                'preceding_acts': _sum_dicts(d['decision_influence']['preceding_acts'] for d in dia),
            },
            'boundary_clean_trials': sum(r['boundary']['clean'] for r in rows),
            'boundary_violation_trials': sum(
                bool(r['boundary']['input_leaks'] or r['boundary']['forbidden_grounds']
                     or r['boundary']['channel_violations']) for r in rows),
        }
    return {'penalty_factor': penalty_factor, 'lookback_s': lookback_s,
            'trials': len(per_trial), 'conditions': conditions, 'per_trial': per_trial,
            'scenarios': sorted({r['efficiency']['scenario'] for r in per_trial}),
            'provenance': _provenance_check(trials)}


def _mean(values):
    values = [v for v in values if v is not None]
    return round(statistics.mean(values), 4) if values else None


def _sum_dicts(dicts):
    total = collections.Counter()
    for d in dicts:
        total.update({k: v for k, v in (d or {}).items()})
    return {k: round(v, 4) if isinstance(v, float) else v for k, v in sorted(total.items())}


def _provenance_check(trials):
    """Flag mixed code/map/cost provenance; conditions must share one bundle."""
    fields = collections.defaultdict(set)
    for trial in trials:
        for key, value in (trial.get('provenance') or {}).items():
            fields[key].add(str(value))
    mixed = sorted(k for k, v in fields.items() if len(v) > 1)
    return {'values': {k: sorted(v) for k, v in sorted(fields.items())},
            'mixed_fields': mixed,
            'single_bundle': not mixed and bool(fields)}


# --------------------------------------------------------------------------- #
# paired comparison
# --------------------------------------------------------------------------- #

def paired_values(trials, metric, baseline, variant, penalty_factor=DEFAULT_PENALTY_FACTOR):
    """Per-(scenario, seed) pairs of ``metric`` for two conditions.

    Repetitions of the same (condition, scenario, seed) are averaged first so
    one pair equals one matched scenario/seed, as the paired-seed design asks.
    """
    buckets = collections.defaultdict(list)
    for trial in trials:
        eff = efficiency_metrics(trial, penalty_factor=penalty_factor)
        value = eff.get(metric)
        if value is None:
            continue
        buckets[(eff['condition'], eff['scenario'], eff['seed'])].append(float(value))
    pairs = []
    keys = sorted({(s, d) for (c, s, d) in buckets if c in (baseline, variant)},
                  key=lambda k: (str(k[0]), str(k[1])))
    for scenario, seed in keys:
        base = buckets.get((baseline, scenario, seed))
        var = buckets.get((variant, scenario, seed))
        if not base or not var:
            continue
        pairs.append({'scenario': scenario, 'seed': seed,
                      'baseline': statistics.mean(base), 'variant': statistics.mean(var),
                      'reps': (len(base), len(var))})
    return pairs


def bootstrap_ci(values, statistic=None, confidence=0.95, resamples=10000, seed=0):
    """Deterministic percentile bootstrap CI for a statistic over ``values``."""
    values = [float(v) for v in values]
    if not values:
        return {'point': None, 'low': None, 'high': None, 'n': 0,
                'resamples': 0, 'confidence': confidence}
    statistic = statistic or statistics.mean
    rng = random.Random(seed)
    n = len(values)
    point = statistic(values)
    if n == 1:
        return {'point': round(point, 6), 'low': None, 'high': None, 'n': 1,
                'resamples': 0, 'confidence': confidence,
                'note': '표본 1개로 구간을 만들지 않는다.'}
    draws = sorted(statistic([values[rng.randrange(n)] for _ in range(n)])
                   for _ in range(resamples))
    alpha = (1.0 - confidence) / 2.0
    low = draws[max(int(math.floor(alpha * resamples)), 0)]
    high = draws[min(int(math.ceil((1.0 - alpha) * resamples)) - 1, resamples - 1)]
    return {'point': round(point, 6), 'low': round(low, 6), 'high': round(high, 6),
            'n': n, 'resamples': resamples, 'confidence': confidence, 'seed': seed}


def compare_conditions(trials, metric, baseline, variant, confidence=0.95,
                       resamples=10000, seed=0, penalty_factor=DEFAULT_PENALTY_FACTOR):
    """Paired-seed comparison with effect sizes and a bootstrap interval.

    Deliberately reports no p-value: on a pilot cohort the interval and the
    per-pair table are the result. ``higher_is_better`` is left to the reader;
    the sign convention is ``variant - baseline``.
    """
    for condition in (baseline, variant):
        if condition not in CONDITIONS:
            raise TrialError(f'unknown condition {condition!r}')
    pairs = paired_values(trials, metric, baseline, variant, penalty_factor)
    diffs = [p['variant'] - p['baseline'] for p in pairs]
    ci = bootstrap_ci(diffs, confidence=confidence, resamples=resamples, seed=seed)
    stdev = statistics.stdev(diffs) if len(diffs) > 1 else None
    dz = round(statistics.mean(diffs) / stdev, 4) if stdev else None
    pos = sum(1 for d in diffs if d > 0)
    neg = sum(1 for d in diffs if d < 0)
    return {
        'metric': metric, 'baseline': baseline, 'variant': variant,
        'n_pairs': len(pairs),
        'baseline_mean': round(statistics.mean([p['baseline'] for p in pairs]), 4) if pairs else None,
        'variant_mean': round(statistics.mean([p['variant'] for p in pairs]), 4) if pairs else None,
        'mean_diff': round(statistics.mean(diffs), 4) if diffs else None,
        'median_diff': round(statistics.median(diffs), 4) if diffs else None,
        'diff_ci': ci,
        'cohens_dz': dz,
        'rank_biserial': _rank_biserial(diffs),
        'pairs_variant_higher': pos, 'pairs_variant_lower': neg,
        'pairs_tied': len(diffs) - pos - neg,
        'pairs': pairs,
        'small_sample': len(pairs) < SMALL_SAMPLE_PAIRS,
        'reporting': '유의성 검정을 하지 않는다. 구간과 짝별 표만 보고한다.',
    }


def _rank_biserial(diffs):
    """Matched-pairs rank-biserial correlation with average ranks for ties."""
    nonzero = [d for d in diffs if d != 0]
    if not nonzero:
        return None
    order = sorted(range(len(nonzero)), key=lambda i: abs(nonzero[i]))
    ranks = [0.0] * len(nonzero)
    i = 0
    while i < len(order):
        j = i
        while j + 1 < len(order) and abs(nonzero[order[j + 1]]) == abs(nonzero[order[i]]):
            j += 1
        average = (i + j + 2) / 2.0            # 1-based average rank of the tie group
        for k in range(i, j + 1):
            ranks[order[k]] = average
        i = j + 1
    total = sum(ranks)
    positive = sum(r for r, d in zip(ranks, nonzero) if d > 0)
    return round(2.0 * positive / total - 1.0, 4) if total else None


#: Metrics compared pairwise by default; the report shows all of them.
COMPARISON_METRICS = ('par_makespan_sim_s', 'success', 'delivery_rate', 'idle_robot_s',
                      'conflicts', 'deadlocks', 'replans', 'model_calls', 'tokens_total',
                      'talk_sim_cost_s')


def compare_all(trials, metrics=COMPARISON_METRICS, include_reference=False,
                confidence=0.95, resamples=10000, seed=0,
                penalty_factor=DEFAULT_PENALTY_FACTOR):
    """All condition pairs × metrics. ``no_comm`` is the default baseline order."""
    present = [c for c in CONDITIONS if any(t['condition'] == c for t in trials)]
    if not include_reference:
        present = [c for c in present if c != REFERENCE_CONDITION]
    out = []
    for i, baseline in enumerate(present):
        for variant in present[i + 1:]:
            for metric in metrics:
                row = compare_conditions(trials, metric, baseline, variant, confidence,
                                         resamples, seed, penalty_factor)
                if row['n_pairs']:
                    out.append(row)
    return out


# --------------------------------------------------------------------------- #
# TensorBoard-friendly scalar export
# --------------------------------------------------------------------------- #
#: Scalar tags pinned in the dashboard, mapped from per-trial metric names.
SCALAR_TAGS = {
    'evaluation/success': 'success',
    'result/par_makespan_sim_s': 'par_makespan_sim_s',
    'result/makespan_sim_s': 'makespan_sim_s',
    'result/talk_sim_cost_s': 'talk_sim_cost_s',
    'result/think_sim_cost_s': 'think_sim_cost_s',
    'result/delivered_items': 'delivered_items',
    'result/delivery_rate': 'delivery_rate',
    'result/idle_robot_s': 'idle_robot_s',
    'result/conflicts': 'conflicts',
    'result/deadlocks': 'deadlocks',
    'result/replans': 'replans',
    'result/model_calls': 'model_calls',
    'result/tokens_total': 'tokens_total',
    'result/wall_latency_ms_mean': 'wall_latency_ms_mean',
}
DIALOGUE_TAGS = {
    'dialogue/utterances': 'utterances',
    'dialogue/korean_share': 'korean_share',
    'dialogue/code_switch_messages': 'code_switch_messages',
    'dialogue/id_issue_messages': 'id_issue_messages',
    'dialogue/claims_true': 'claims_true',
    'dialogue/claims_false': 'claims_false',
    'dialogue/truthful_share': 'truthful_share',
    'dialogue/channel_violations': 'channel_violations',
}
#: HParams columns to show; the viewer config lives in outputs/tensorboard-view.json.
HPARAM_KEYS = ('condition', 'scenario', 'seed', 'leader_id', 'end_reason',
               'penalty_factor', 'sim_horizon_s')


def scalar_export(summary):
    """TB-friendly scalar payload: one run per trial plus per-condition runs.

    Writing events is optional (see ``scripts/zone_study_report.py --tb-events``);
    this payload alone is enough for the existing export/viewer path and needs no
    server change.
    """
    runs = []
    for index, row in enumerate(summary['per_trial']):
        eff, dia = row['efficiency'], row['dialogue']
        scalars = {tag: _num(eff.get(key)) for tag, key in SCALAR_TAGS.items()}
        scalars.update({tag: _num(dia.get(key)) for tag, key in DIALOGUE_TAGS.items()})
        runs.append({
            'run': f'{eff["condition"]}/{eff["scenario"]}-s{eff["seed"]}',
            'step': index,
            'hparams': {k: _hparam(eff.get(k, summary.get(k))) for k in HPARAM_KEYS},
            'scalars': {k: v for k, v in scalars.items() if v is not None},
            'boundary_clean': bool(row['boundary']['clean']),
        })
    for condition, row in summary['conditions'].items():
        scalars = {'cohort/success_rate': _num(row['success_rate']),
                   'cohort/trials': float(row['trials']),
                   'cohort/delivery_rate': _num(row['cohort_delivery_rate']),
                   'cohort/par_sim_s_per_delivered': _num(row['cohort_par_sim_s_per_delivered']),
                   'cohort/korean_share': _num(row['dialogue']['korean_share']),
                   'cohort/truthful_share': _num(row['dialogue']['truthful_share'])}
        for name in SUMMARY_METRICS:
            scalars[f'cohort/{name}'] = _num(row['metrics'].get(name))
        runs.append({'run': f'cohort/{condition}', 'step': 0,
                     'hparams': {'condition': condition, 'scenario': 'cohort',
                                 'seed': -1, 'leader_id': '',
                                 'end_reason': 'cohort',
                                 'penalty_factor': summary['penalty_factor'],
                                 'sim_horizon_s': 0.0},
                     'scalars': {k: v for k, v in scalars.items() if v is not None},
                     'boundary_clean': row['boundary_violation_trials'] == 0})
    return {'schema': 'ugrp.zone_study_scalars.v1',
            'note': 'EVALUATION-ONLY. 실패·중단·예산 소진을 분모에 유지한 값이다.',
            'hparam_columns': list(HPARAM_KEYS), 'runs': runs}


def _num(value):
    if isinstance(value, bool):
        return float(value)
    return float(value) if isinstance(value, (int, float)) else None


def _hparam(value):
    if value is None:
        return ''
    return value if isinstance(value, (int, float, str, bool)) else str(value)
