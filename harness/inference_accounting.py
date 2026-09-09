"""Settle outstanding usage without applying late model actions."""
from __future__ import annotations

import copy


def settle_completed(rid, item, planner, budget, *, now):
    """Return a log row only when already complete; never block or apply actions."""
    future = item['future']
    call_id = item['call_id']
    if not future.done():
        return None
    if future.cancelled():
        budget.cancel_unstarted(call_id)
        return {'event': 'llm_result', 'robot_id': rid, 'time': now,
                'call_id': call_id, 'disposition': 'cancelled_before_start'}
    decision = None
    error = None
    try:
        decision = future.result()
    except Exception as exc:
        error = type(exc).__name__
    audit = copy.deepcopy(getattr(planner, 'last_audit', None) or {})
    budget.record(call_id, audit.get('usage'))
    return {'event': 'llm_result', 'robot_id': rid, 'time': now,
            'call_id': call_id, 'decision': decision, 'audit': audit,
            'error_type': error, 'disposition': 'discarded_run_ended'}


def settle_pending(pending, planners, budgets, pool, *, now, emit):
    """Call after stopping all motors; wait only for already submitted requests.

    Successfully cancelled, not-started Futures release their reservation. This
    function never requests an actor action or submits another provider request.
    """
    for item in pending.values():
        item['future'].cancel()
    pool.shutdown(wait=True, cancel_futures=True)
    for rid, item in list(pending.items()):
        row = settle_completed(rid, item, planners[rid], budgets[rid], now=now)
        if row is None:
            raise RuntimeError('FUTURE_UNRESOLVED_AFTER_SHUTDOWN')
        emit(row)
        pending.pop(rid)
