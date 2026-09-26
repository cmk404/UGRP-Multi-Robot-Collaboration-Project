"""SIM-time cost of thinking and talking (package D): formula, grid, sweeps, records.

Every cost here must be a pure function of the attempt content and the versioned
parameters. No wall time, no API latency, no simulator state.
"""
from __future__ import annotations

import json
import time

import pytest

from harness import zone_sim_cost as zc
from harness.zone_study_contract import (CALL_LOG_SCHEMA, CALL_STATUS, MESSAGE_LOG_SCHEMA,
                                         message_envelope, validate_log_record)

#: Minimal package A ``provenance`` object (closed keys, see PROVENANCE_KEYS).
_PROVENANCE = {'registry_sha256': 'a' * 64, 'order_sheet_sha256': 'b' * 64,
               'map_file_sha256': 'c' * 64, 'public_map_sha256': 'd' * 64,
               'code_sha': 'deadbee', 'execution_bundle_id': 'zone_study_offline_v1',
               'model': 'none-fixture', 'provider': None, 'model_settings_sha256': None,
               'prompt_template_sha256': None, 'cost_profile_id': 'zone_sim_cost.v1',
               'input_profile_id': 'zone_study_inputs.v1'}


def _attempt(**kw):
    return zc.Attempt(**kw)


# ---------------------------------------------------------------------------
# Formula and grid

def test_default_params_reproduce_the_design_example_and_are_marked_provisional():
    p = zc.params()
    # Design doc section 5: 120 output tokens and one utterance cost 3.7 SIM s
    # (alpha 1.0 + beta .02*120 + gamma .3), delivery 0.1 s.
    cost = zc.call_cost([_attempt(output_tokens=120, utterances=1)], p)
    assert cost.sim_s == pytest.approx(3.7)
    assert zc.delivery_delay_s(1, p) == pytest.approx(.1)
    # Input tokens are charged on top (user decision 2026-09-26): the 8,000-token
    # input budget adds 1.6 s at 5,000 tokens/s of prefill.
    with_input = zc.call_cost([_attempt(input_tokens=8000, output_tokens=120, utterances=1)], p)
    assert with_input.sim_s == pytest.approx(5.3)
    assert with_input.breakdown['input_s'] == pytest.approx(1.6)
    assert p.provisional and 'provisional' in p.note


def test_breakdown_adds_up_and_is_json_serialisable():
    p = zc.params()
    cost = zc.call_cost([_attempt(input_tokens=500, output_tokens=60, utterances=2)], p)
    b = cost.breakdown
    assert b['overhead_s'] + b['input_s'] + b['output_s'] + b['utterance_s'] == pytest.approx(cost.raw_s)
    assert b['error_s'] == 0. and b['timeout_s'] == 0.
    assert json.loads(json.dumps(cost.to_dict()))['params_version'] == p.version


@pytest.mark.parametrize('quantum,raw_tokens,expected', [
    (.1, 120, 3.7),      # exact multiple must not jump a whole step
    (.5, 120, 4.0),      # raw 3.9 -> next 0.5 grid point
    (1., 120, 4.0),
    (.5, 121, 4.0),
    (.5, 140, 4.5),      # raw 4.1
])
def test_cost_is_rounded_up_onto_the_sim_grid(quantum, raw_tokens, expected):
    p = zc.params().variant(f'q={quantum}', quantum_s=quantum)
    cost = zc.call_cost([_attempt(output_tokens=raw_tokens, utterances=1)], p)
    assert cost.sim_s == pytest.approx(expected)
    assert cost.sim_s / quantum == pytest.approx(round(cost.sim_s / quantum))
    assert cost.sim_s >= cost.raw_s


def test_quantize_edges():
    assert zc.quantize(0., .1) == 0.
    assert zc.quantize(-3., .1) == 0.
    assert zc.quantize(3.7, .1) == pytest.approx(3.7)      # 3.7/.1 is 36.9999... in binary
    assert zc.quantize(3.70001, .1) == pytest.approx(3.8)
    with pytest.raises(ValueError):
        zc.quantize(1., 0.)


def test_zero_scale_is_free_and_reproduces_todays_runner_behaviour():
    free = zc.params('zone_sim_cost.v1_free')
    cost = zc.call_cost([_attempt(input_tokens=8000, output_tokens=768, utterances=3)], free)
    assert cost.sim_s == 0. and cost.raw_s == 0.
    assert zc.delivery_delay_s(3, free) == 0.
    assert free.min_call_s() == 0.
    # and a positive scale on the same attempt is strictly positive
    assert zc.call_cost([_attempt(output_tokens=1)], zc.params()).sim_s > 0.


@pytest.mark.parametrize('field,values', [
    ('input_tokens', (0, 1000, 8000)),
    ('output_tokens', (0, 120, 768)),
    ('utterances', (0, 1, 4)),
])
def test_cost_is_non_decreasing_in_every_charged_quantity(field, values):
    p = zc.params()
    costs = [zc.call_cost([_attempt(**{field: v})], p).raw_s for v in values]
    assert costs == sorted(costs)
    assert costs[-1] > costs[0]


def test_long_utterances_cost_more_than_short_ones():
    p = zc.params()
    short = zc.call_cost([_attempt(output_tokens=40, utterances=1)], p)
    long = zc.call_cost([_attempt(output_tokens=600, utterances=1)], p)
    assert long.sim_s > short.sim_s
    # the difference is exactly the extra output tokens
    assert long.raw_s - short.raw_s == pytest.approx(p.output_token_s * 560)


def test_scale_multiplies_the_whole_formula():
    base = zc.params()
    doubled = base.variant('scale=2', scale=2.)
    a = [_attempt(input_tokens=1000, output_tokens=100, utterances=2)]
    assert zc.call_cost(a, doubled).raw_s == pytest.approx(2 * zc.call_cost(a, base).raw_s)


# ---------------------------------------------------------------------------
# Failed attempts and retries

def test_failed_attempts_pay_and_a_retry_is_charged_as_a_further_attempt():
    p = zc.params()
    invalid = _attempt(outcome='invalid', input_tokens=8000, output_tokens=40)
    ok = _attempt(input_tokens=8000, output_tokens=120, utterances=1)
    one = zc.call_cost([ok], p)
    retried = zc.call_cost([invalid, ok], p)
    assert retried.sim_s > one.sim_s
    assert retried.raw_s == pytest.approx(zc.attempt_cost_s(invalid, p) + zc.attempt_cost_s(ok, p))
    assert retried.outcome == 'ok'                 # the call succeeded, but it paid twice
    assert retried.breakdown['attempts'] == 2
    assert retried.breakdown['overhead_s'] == pytest.approx(2 * p.call_overhead_s)


def test_transport_errors_and_timeouts_use_pre_registered_costs():
    p = zc.params()
    error = zc.call_cost([_attempt(outcome='error')], p)
    timeout = zc.call_cost([_attempt(outcome='timeout')], p)
    assert error.sim_s == pytest.approx(p.error_s) and error.outcome == 'error'
    assert timeout.sim_s == pytest.approx(p.timeout_s) and timeout.outcome == 'timeout'
    # the timeout cost is pre-registered, never the observed wall latency
    assert timeout.breakdown['timeout_s'] == pytest.approx(p.timeout_s)
    # tokens reported alongside a failure are still charged
    partial = zc.call_cost([_attempt(outcome='error', output_tokens=50)], p)
    assert partial.raw_s == pytest.approx(p.error_s + 50 * p.output_token_s)


def test_a_call_needs_at_least_one_attempt():
    with pytest.raises(ValueError):
        zc.call_cost([], zc.params())


@pytest.mark.parametrize('kw', [{'outcome': 'nope'}, {'input_tokens': -1}, {'output_tokens': 1.5},
                                {'utterances': -2}])
def test_attempt_validates_its_fields(kw):
    with pytest.raises(ValueError):
        zc.Attempt(**kw)


@pytest.mark.parametrize('kw', [{'quantum_s': 0.}, {'quantum_s': -1.}, {'scale': -1.},
                                {'call_overhead_s': -1.}, {'timeout_s': -.5}])
def test_params_validate_their_fields(kw):
    with pytest.raises(ValueError):
        zc.CostParams(**kw)


# ---------------------------------------------------------------------------
# Delivery

def test_broadcast_delivery_is_not_serialised_by_default():
    p = zc.params()
    assert zc.delivery_delay_s(1, p) == zc.delivery_delay_s(3, p)
    fan = p.variant('per_recipient_s=.2', per_recipient_s=.2)
    assert zc.delivery_delay_s(3, fan) > zc.delivery_delay_s(1, fan)
    assert zc.delivery_delay_s(3, fan) == pytest.approx(zc.quantize(.1 + .6, p.quantum_s))


# ---------------------------------------------------------------------------
# Versioning and provenance

def test_registry_versions_are_looked_up_and_unknown_ones_fail_loudly():
    assert zc.params().version == 'zone_sim_cost.v1'
    assert zc.params('zone_sim_cost.v1_free').scale == 0.
    with pytest.raises(KeyError):
        zc.params('zone_sim_cost.v9')


def test_params_are_frozen_and_digest_every_field():
    p = zc.params()
    with pytest.raises(Exception):
        p.call_overhead_s = 2.
    assert p.digest() == zc.params().digest()
    for field, value in [('call_overhead_s', 1.5), ('output_token_s', .03), ('utterance_s', .4),
                         ('delivery_s', .2), ('timeout_s', 30.), ('quantum_s', .5), ('scale', 2.)]:
        other = p.variant(f'{field}', **{field: value})
        assert other.digest() != p.digest()
        assert other.version != p.version           # a sweep variant is never mistaken for the base


def test_cost_rows_carry_the_cost_schema_and_re_emit_package_a_records():
    p = zc.params()
    cost = zc.call_cost([_attempt(output_tokens=30, utterances=1)], p)
    call = zc.CallCostRecord(call_id='call-0001-r1', actor='r1', trigger='blockage', started_sim_s=4.,
                             finished_sim_s=4. + cost.sim_s, cost=cost, merged_triggers=('idle',))
    row = json.loads(json.dumps(call.to_dict(), ensure_ascii=False))
    assert row['schema'] == zc.CALL_COST_SCHEMA == 'ugrp.zone_sim_cost.call_cost.v1'
    assert row['sim_cost_s'] == cost.sim_s and row['trigger'] == 'blockage'
    assert row['status'] == 'ok' and row['merged_triggers'] == ['idle']
    assert row['cost']['params_digest'] == p.digest()
    msg = zc.MessageCostRecord(message_id='call-0001-r1-m1', sender='r1', recipient='r2',
                               encoding='free_ko', sent_sim_s=4.7, delivered_sim_s=4.8, broadcast=True,
                               call_id='call-0001-r1')
    assert json.loads(json.dumps(msg.to_dict()))['schema'] == zc.MESSAGE_COST_SCHEMA
    assert json.loads(json.dumps(msg.to_dict()))['delivered_sim_s'] == pytest.approx(4.8)
    # the same numbers under package A's call log schema, validated by A
    record = zc.contract_call_record(call, run_id='run-1', condition_name='peer_ko', seed=13,
                                    request_id='req_1', call_index=0, input_sha256='a' * 64,
                                    provenance=_PROVENANCE)
    validate_log_record(record)
    assert record['schema'] == CALL_LOG_SCHEMA
    assert record['sim_cost_s'] == pytest.approx(cost.sim_s)
    assert record['released_at_sim_s'] - record['requested_at_sim_s'] == pytest.approx(cost.sim_s)
    assert record['trigger'] == 'own_view_change'          # A's enum for a blockage trigger
    assert record['status'] == 'ok' and record['http_attempts'] == 1
    assert record['cost_terms']['params_digest'] == p.digest()
    assert record['output_tokens'] == 30 and record['cost_terms']['utterances'] == 1


def test_every_scheduler_trigger_and_outcome_maps_onto_package_a():
    from harness.zone_event_scheduler import TRIGGERS
    from harness.zone_study_inputs import TRIGGERS as A_TRIGGERS

    assert set(zc.TRIGGER_TO_CONTRACT) == set(TRIGGERS)
    assert set(zc.TRIGGER_TO_CONTRACT.values()) <= set(A_TRIGGERS)
    assert {zc.contract_trigger(t) for t in TRIGGERS} <= set(A_TRIGGERS)
    with pytest.raises(KeyError):
        zc.contract_trigger('peer_job_end')               # not a call trigger of this study
    assert set(zc.OUTCOME_STATUS) == set(zc.OUTCOMES)
    assert set(zc.OUTCOME_STATUS.values()) <= set(CALL_STATUS)
    for outcome, status in zc.OUTCOME_STATUS.items():
        assert zc.call_cost([_attempt(outcome=outcome)]).status == status


def test_broadcast_edges_become_one_package_a_message_record():
    envelope = message_envelope('peer_ko', 'call-0001-r1-m1', 'r1', ['r2', 'r3'],
                                {'text': 'door_narrow가 막혀 있습니다.'}, created_at_sim_s=4.7)
    edges = [zc.MessageCostRecord(message_id='call-0001-r1-m1', sender='r1', recipient=rid,
                                  encoding='free_ko', sent_sim_s=4.7, delivered_sim_s=4.8,
                                  broadcast=True, call_id='call-0001-r1')
             for rid in ('r3', 'r2')]                     # order must not matter
    rows = zc.contract_message_records(edges, run_id='run-1', condition_name='peer_ko', seed=13,
                                       envelopes={'call-0001-r1-m1': envelope})
    assert len(rows) == 1
    validate_log_record(rows[0])
    assert rows[0]['schema'] == MESSAGE_LOG_SCHEMA and rows[0]['encoding'] == 'free_ko'
    assert [d['recipient'] for d in rows[0]['deliveries']] == ['r2', 'r3']
    assert rows[0]['delivered_at_sim_s'] == pytest.approx(4.8)
    assert rows[0]['delivery_delay_s'] == pytest.approx(.1)
    assert rows[0]['korean_ok'] is True


def test_cost_does_not_depend_on_wall_time():
    p = zc.params()
    attempts = [_attempt(input_tokens=8000, output_tokens=120, utterances=1)]
    first = zc.call_cost(attempts, p)
    time.sleep(.05)
    assert zc.call_cost(attempts, p).to_dict() == first.to_dict()
    assert not hasattr(zc, 'time')                  # the module never reads a clock


# ---------------------------------------------------------------------------
# Sensitivity sweep

def test_scale_sweep_covers_the_planned_multipliers_with_distinct_versions():
    variants = zc.sweep()
    assert [v.scale for v in variants] == list(zc.SWEEP_SCALES) == [0., .5, 1., 2., 4.]
    assert [v.version for v in variants] == [f'zone_sim_cost.v1+scale={s:g}' for s in zc.SWEEP_SCALES]
    assert len({v.digest() for v in variants}) == len(variants)


def test_axis_sweep_separates_per_call_and_per_token_cost():
    variants = zc.sweep_axes()
    versions = [v.version for v in variants]
    assert 'zone_sim_cost.v1+call_overhead_s=4' in versions     # rare long utterances
    assert 'zone_sim_cost.v1+output_token_s=0.08' in versions   # frequent short utterances
    assert 'zone_sim_cost.v1+utterance_s=3' in versions
    # one-at-a-time: a variant differs from the base in at most its own axis
    base = zc.params().to_dict()
    for variant in variants:
        axis = variant.version.rsplit('+', 1)[1].split('=')[0]
        changed = [k for k, v in variant.to_dict().items() if k != 'version' and base[k] != v]
        assert changed in ([], [axis]), (variant.version, changed)
    with pytest.raises(KeyError):
        zc.sweep(zc.params(), 'no_such_axis', (1.,))


def test_grid_sweep_is_the_cartesian_product():
    grid = zc.sweep_grid(axes={'call_overhead_s': (.5, 2.), 'output_token_s': (.01, .04)})
    assert len(grid) == 4
    assert {(v.call_overhead_s, v.output_token_s) for v in grid} == {(.5, .01), (.5, .04), (2., .01), (2., .04)}


def test_sensitivity_table_is_labelled_approximate():
    calls = [[_attempt(input_tokens=8000, output_tokens=120, utterances=1)],
             [_attempt(outcome='invalid', input_tokens=8000, output_tokens=40),
              _attempt(input_tokens=8000, output_tokens=200, utterances=2)]]
    rows = zc.sensitivity(calls)
    assert [r['version'] for r in rows] == [v.version for v in zc.sweep()]
    assert rows[0]['total_sim_s'] == 0.                     # scale 0
    totals = [r['total_sim_s'] for r in rows]
    assert totals == sorted(totals) and totals[-1] > totals[2]
    assert all(r['approximate'] and 'not re-run' in r['note'] for r in rows)
    assert all(len(r['per_call_sim_s']) == 2 for r in rows)


def test_recost_of_stored_records_is_labelled_approximate():
    p = zc.params()
    cost = zc.call_cost([_attempt(output_tokens=120, utterances=1)], p)
    record = zc.CallCostRecord(call_id='c1', actor='r2', trigger='report', started_sim_s=1.,
                               finished_sim_s=1. + cost.sim_s, cost=cost)
    rows = zc.recost([record], p.variant('scale=4', scale=4.))
    assert rows[0]['was_sim_s'] == cost.sim_s
    assert rows[0]['now_sim_s'] == pytest.approx(4 * cost.raw_s)
    assert rows[0]['approximate'] is True
