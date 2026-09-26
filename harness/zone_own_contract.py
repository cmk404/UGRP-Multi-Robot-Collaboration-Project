"""Package F executor: map vocabulary and the package A/D adapters (issue #221).

Contracts are IMPORTED from the versioned study-core modules on main, never copied (Codex audit
of PR #206): ``harness.zone_study_contract`` (package A: action log schema, kinds, local states,
conditions, order-line keys, validator) and ``harness.zone_event_scheduler`` (package D wake
triggers). Pickup bays/slots come from ``harness.zone_map_schematic.pickup_bays`` (package A/E rule).
"""
from __future__ import annotations

import copy
import math
from collections.abc import Mapping, Sequence

from harness import m1_owncam_contract
from harness import zone_study_contract as A
from harness.zone_event_scheduler import TRIGGERS as D_TRIGGERS
from harness.zone_map_schematic import pickup_bays

JOB_APIS = ('deliver', 'goto', 'look_around', 'hold', 'abort')
API_TO_ACTION_KIND = {'deliver': 'claim_order', 'goto': 'goto', 'look_around': 'observe', 'hold': 'wait',
                      'abort': 'abort_job'}
assert set(API_TO_ACTION_KIND.values()) <= set(A.ACTION_KINDS)
EVENTS = ('job_started', 'job_done', 'job_failed', 'blockage_seen', 'pose_uncertain')
# Package D wake trigger per executor event; None = logged, no call. D has no dedicated label for a
# pose-uncertainty entry: it is an own execution fault seen on the own camera, so 'failure' (package A
# ``own_view_change`` through ``zone_sim_cost.TRIGGER_TO_CONTRACT``). With the gate's hysteresis it
# fires once per entry, not per frame (Codex review 2 of PR #206, P1-3).
EVENT_TO_TRIGGER = {'job_started': None, 'job_done': 'idle', 'job_failed': 'failure',
                    'blockage_seen': 'blockage', 'pose_uncertain': 'failure'}
assert set(EVENT_TO_TRIGGER.values()) - {None} <= set(D_TRIGGERS)
# job_failed reasons that are a local SIM budget (own timer) rather than a view change.
TIMEOUT_REASONS = ('LOCAL_TIMEOUT', 'EPISODE_END')
ORDER_KEYS = A.ORDER_KEYS
SLOT_SEARCH_MARGIN_M = .15      # own-RGB cyan detections kept within the ordered pickup slot + this margin
PICKUP_VIEW_X_M = -.47          # = m1_owncam_delivery.SEARCH_VIEW_X_M (west of the pickup grid)
FAR_BAY_M = 1.0                 # a bay starting this far east of the west viewpoint gets lane viewpoints
LANE_OFFSET_M = .40             # mid-lane between static pickup rows (rows are 0.80 m apart)


class ExecutorContractError(m1_owncam_contract.M1ContractError):
    """An input the executor contract forbids (non own-camera pose, foreign observation, coordinates in orders)."""


def pickup_slots(static_map: Mapping) -> dict[str, dict]:
    """{'P1-1': {'bay_id', 'center_m', 'x_range_m', 'y_range_m'}, ...} from ``zone_map_schematic.pickup_bays``."""
    out = {}
    for bay in pickup_bays(static_map):
        for s in bay['slots']:
            (cx, cy), (hx, hy) = s['center_m'], s['half_extents_m']
            out[s['slot_id']] = {'bay_id': bay['bay_id'], 'center_m': [round(cx, 4), round(cy, 4)],
                                 'x_range_m': [round(cx - hx, 4), round(cx + hx, 4)],
                                 'y_range_m': [round(cy - hy, 4), round(cy + hy, 4)]}
    return out


def pickup_slot_of(static_map: Mapping, xy: Sequence[float]) -> str | None:
    """Coarse slot id of a floor point (used by scenario configs to write the order sheet)."""
    for sid, s in pickup_slots(static_map).items():
        if s['x_range_m'][0] <= xy[0] < s['x_range_m'][1] and s['y_range_m'][0] <= xy[1] < s['y_range_m'][1]:
            return sid
    return None


def zone_slot(static_map: Mapping, slot_id: str) -> dict:
    for slots in static_map['zone_slots'].values():
        for s in slots:
            if s['slot_id'] == slot_id:
                return s
    raise KeyError(f'unknown zone slot {slot_id!r}')


def validate_order_sheet(order_sheet: Mapping, static_map: Mapping) -> dict[str, dict]:
    """Order lines by id; a coordinate or unknown key anywhere in a line is refused (exact-pose smuggling)."""
    orders = order_sheet.get('orders') if isinstance(order_sheet, Mapping) else None
    if not isinstance(orders, Sequence) or isinstance(orders, str):
        raise ExecutorContractError('order_sheet.orders must be a list')
    slots = pickup_slots(static_map)
    out = {}
    for order in orders:
        if not isinstance(order, Mapping):
            raise ExecutorContractError('an order line must be an object')
        extra = sorted(set(order) - set(ORDER_KEYS))
        if extra:
            raise ExecutorContractError(f'order line keys {extra} are not order-sheet vocabulary')
        oid = order.get('order_id')
        if not isinstance(oid, str) or not oid:
            raise ExecutorContractError('order_id must be a non-empty string')
        where = order.get('initial_location') or {}
        if not isinstance(where, Mapping) or set(where) - {'pickup_bay', 'slot'}:
            raise ExecutorContractError('initial_location may only name pickup_bay / slot')
        for v in where.values():
            if not isinstance(v, str):
                raise ExecutorContractError('initial_location must be map vocabulary, never a coordinate')
        if where.get('slot') is not None and where['slot'] not in slots:
            raise ExecutorContractError(f"unknown pickup slot {where['slot']!r}")
        if order.get('destination_zone') not in static_map['zone_slots']:
            raise ExecutorContractError(f"unknown destination zone {order.get('destination_zone')!r}")
        out[oid] = copy.deepcopy(dict(order))
    return out


def lane_viewpoints(slot_rect, rows_y: Sequence[float], y_est: float) -> list[tuple[float, float]]:
    """Extra search viewpoints for a far pickup slot: its west edge, on the mid-lanes between static rows."""
    (x0, _), (y0, y1) = slot_rect
    if x0 - PICKUP_VIEW_X_M <= FAR_BAY_M:
        return []
    lanes = sorted({round(r + d, 3) for r in rows_y for d in (-LANE_OFFSET_M, LANE_OFFSET_M)
                    if y0 - .1 <= r + d <= y1 + .1}, key=lambda y: (abs(y - y_est), y))
    return [(float(x0), y) for y in lanes]


def scheduler_trigger(event: Mapping) -> str | None:
    """Package D wake trigger for one executor event (None = logged, no call)."""
    if event['event'] == 'job_failed' and str(event['detail'].get('reason', '')).startswith(TIMEOUT_REASONS):
        return 'timeout'
    return EVENT_TO_TRIGGER[event['event']]


def action_record(ack: Mapping, *, run_id: str, condition: str, seed: int, request_id: str) -> dict:
    """Package A ``ugrp.zone_study_action.v1`` record of one API call, validated by package A itself.

    ``condition`` must be a package A condition (``zone_study_contract.CONDITIONS``); the old
    ``no_llm_scripted`` label is refused by A (Codex review 2 of PR #206, P2-8).
    """
    A.condition(condition)
    record = {'schema': A.ACTION_LOG_SCHEMA, 'run_id': run_id, 'condition': condition, 'seed': int(seed),
              'actor': ack['robot_id'], 'action_id': ack['action_id'], 'request_id': request_id,
              'submitted_at_sim_s': ack['sim_s'], 'kind': API_TO_ACTION_KIND[ack['api']],
              'arguments': dict(ack['arguments']), 'order_id': ack['arguments'].get('order_id'), 'role': None,
              'accepted': bool(ack['accepted']), 'rejected_reason': ack['rejected_reason'],
              'local_state': ack['local_state']}
    return dict(A.validate_log_record(record))


def finite_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
