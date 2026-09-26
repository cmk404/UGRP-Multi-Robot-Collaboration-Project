"""Regression tests of Codex's fifth review of PR 194 (``codex-194-r5``).

The review reported one P1 and two P2 findings against ``4c8f908``. Each test
below reproduces the reported counterexample in memory and asserts the fixed
behaviour; the boundary tests next to it pin the neighbouring cases. Offline
only: no simulator, no model call, no network. The pre-fix failures and the
mutation check are recorded in
``experiments/2026-09-26-zone-study-offline-smoke/review-r5/``.

* P1 — a transport that sent a reserved internal retry and then failed with an
  unknown usage was counted as ONE attempt, and ``commit`` refunded the second
  reservation: 3 real sends, 2 in the ledger, 0 violations.
* P2 — a summary-only record (no ``calls``) lost ``tokens_complete=False`` and
  ``usage_unknown_calls``: the known 833 + 40 tokens became a confirmed 873.
* P2 — repeated trials of one condition/scenario/seed shared one TensorBoard run
  name, so the last one written overwrote the other in the viewer.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math

import pytest

from harness import zone_event_scheduler as ds
from harness import zone_sim_cost as zc
from harness import zone_study_contract as c
from harness import zone_study_eval as ev
from scripts import zone_study_report as report

SEED = 601


def _provenance():
    return {'registry_sha256': c.registry_sha256(), 'order_sheet_sha256': 'a' * 64,
            'map_file_sha256': 'b' * 64, 'public_map_sha256': 'c' * 64, 'code_sha': 'deadbeef',
            'execution_bundle_id': 'zone_study_offline_v1', 'model': 'none-fixture-v1',
            'provider': None, 'model_settings_sha256': None, 'prompt_template_sha256': 'd' * 64,
            'cost_profile_id': 'zone_sim_cost.v1', 'input_profile_id': 'zone_study_inputs.v1'}


def _log(sched, condition='peer_ko', seed=SEED):
    return sched.contract_log(run_id='run', condition_name=condition, seed=seed, provenance=_provenance())


# =========================================================================== #
# P1 — attempts that were reserved and sent keep their budget when usage is unknown

class _SendsRetryThenFails:
    """Sends the request and every internal retry it could reserve, then fails.

    ``sent`` counts requests that actually left: the submit, and each retry whose
    reservation the scheduler granted (a compliant transport sends only those).
    ``outcome`` is what ``reply`` finally does: raise an exception, or return a
    :class:`CallReply` built from the number of attempts it sent.
    """

    def __init__(self, outcome, *, retries=1):
        self.outcome = outcome
        self.retries = retries
        self.sent = 0

    def submit(self, call):
        self.sent += 1
        return call

    def reply(self, token):
        sent = 1
        for _ in range(self.retries):
            if not token.reserve(1):
                break
            self.sent += 1
            sent += 1
        result = self.outcome(sent)
        if isinstance(result, BaseException):
            raise result
        return result


FAILURES = {
    'plain_exception': lambda sent: RuntimeError('connection reset'),
    'failure_without_attempts': lambda sent: ds.TransportFailure('no usage report', attempts=()),
    'failure_partly_known': lambda sent: ds.TransportFailure(
        'retry failed without a usage report', usage_known=False,
        attempts=(zc.Attempt(outcome='invalid', input_tokens=833, output_tokens=40),)),
}


def _cap2(**kw):
    """HTTP cap of 2 attempts for r1; the scheduler may retry a failed call once."""
    return ds.CallPolicy(max_http_attempts_per_actor=2, max_calls_per_actor=5, **kw)


def _run(transport, policy, until_s=60.0):
    sched = ds.EventScheduler(transport, policy=policy)
    sched.trigger('r1', 'start')
    sched.run(until_s=until_s)
    return sched


def _ledger_attempts(sched):
    return (sum(len(call.cost.attempts) for call in sched.calls)
            + sum(row['http_attempts'] for row in sched.censored))


@pytest.mark.parametrize('failure', sorted(FAILURES))
def test_r5_p1_an_unknown_usage_failure_keeps_the_attempts_it_sent(failure):
    """The counterexample: the first request and its reserved retry both left,
    then the reply failed without a usage report."""
    transport = _SendsRetryThenFails(FAILURES[failure])
    sched = _run(transport, _cap2())
    first = sched.calls[0]
    assert len(first.cost.attempts) == 2 and first.notes['usage_known'] is False
    # every real send is in the ledger and the budget, and the cap holds
    assert transport.sent == 2 == _ledger_attempts(sched) == sched.budget.used['r1']
    assert sched.budget.used['r1'] <= 2
    # the scheduler's retry is refused instead of being paid from a refund
    assert len(sched.calls) == 1 and sched.metrics['r1']['budget_refused'] == 1
    # both attempts are charged SIM time (the review: 1 s for two calls)
    two = [zc.Attempt(outcome='error'), first.cost.attempts[-1]]
    assert first.cost.sim_s == zc.call_cost(two, sched.params).sim_s
    # the violation — reserved attempts the reply did not account for — is recorded
    assert sched.unreported_attempts == [{'call_id': first.call_id, 'actor': 'r1', 'reserved': 2,
                                          'reported': 0 if failure != 'failure_partly_known' else 1,
                                          'counted': 2}]
    assert [e['kind'] for e in sched.events].count('attempts_unreported') == 1
    log = _log(sched)
    assert log['unreported_attempts'] == sched.unreported_attempts
    record = log['calls'][0]
    c.validate_log_record(record)
    assert record['http_attempts'] == 2 and record['cost_terms']['usage_bound'] == 'lower_bound'
    if failure == 'failure_partly_known':          # the known part stays a lower bound
        assert (record['input_tokens']['text'], record['output_tokens']) == (833, 40)
    assert sched.over_budget_attempts == []


def test_r5_p1_a_reply_that_declares_its_usage_unknown_keeps_its_reserved_attempts():
    """The same rule for a transport that RETURNS (not raises) an unknown usage."""
    transport = _SendsRetryThenFails(
        lambda sent: ds.CallReply(attempts=(zc.Attempt(outcome='error'),), usage_known=False))
    sched = _run(transport, _cap2())
    assert transport.sent == 2 == _ledger_attempts(sched) == sched.budget.used['r1']
    assert sched.unreported_attempts[0]['reserved'] == 2 and sched.unreported_attempts[0]['counted'] == 2


def test_r5_p1_a_call_censored_at_the_horizon_keeps_its_sent_attempts():
    """The censor path fetches the reply at the horizon through the same rule.

    An observation queued at 0.4 s keeps the reply unfetched (its earliest SIM
    completion is 0.5 s) until the 0.3 s horizon closes the call as ``pending``.
    """
    transport = _SendsRetryThenFails(FAILURES['plain_exception'])
    sched = ds.EventScheduler(transport, policy=_cap2())
    sched.trigger('r1', 'start')
    sched.arm_observations(('r1',), period_s=1.0, first_at=0.4)
    sched.run(until_s=0.3)
    row = sched.censored[0]
    assert row['reason'] == 'pending' and row['usage_known'] is False
    assert row['http_attempts'] == 2 == transport.sent == sched.budget.used['r1']
    assert row['charged_sim_s'] == zc.call_cost([zc.Attempt(outcome='error')] * 2, sched.params).sim_s
    assert sched.unreported_attempts[0]['counted'] == 2
    record = _log(sched)['calls'][0]
    c.validate_log_record(record)
    assert record['status'] == 'censored' and record['http_attempts'] == 2


def test_r5_p1_the_team_cap_holds_when_two_actors_fail_after_a_retry():
    """Team budget 4: two actors each send 2 and fail; nothing is refunded, so
    neither scheduler retry can send a fifth request."""
    transport = _SendsRetryThenFails(FAILURES['plain_exception'])
    sched = ds.EventScheduler(transport, policy=ds.CallPolicy(max_attempts_total=4, max_calls_per_actor=5))
    sched.trigger('r1', 'start')
    sched.trigger('r2', 'start')
    sched.run(until_s=60.0)
    assert transport.sent == 4 == sched.budget.used_total() == _ledger_attempts(sched)
    assert len(sched.unreported_attempts) == 2


# --- boundaries: what must NOT change -------------------------------------- #

def test_r5_p1_boundary_a_single_reserved_attempt_is_one_attempt_and_no_violation():
    transport = _SendsRetryThenFails(FAILURES['plain_exception'], retries=0)
    sched = _run(transport, _cap2(max_retries=0))
    assert [len(call.cost.attempts) for call in sched.calls] == [1]
    assert transport.sent == 1 == sched.budget.used['r1'] and sched.unreported_attempts == []


def test_r5_p1_boundary_a_refused_retry_reservation_is_not_counted_as_sent():
    """Cap 1: the retry reservation is refused, so the request never left."""
    transport = _SendsRetryThenFails(FAILURES['plain_exception'])
    sched = _run(transport, ds.CallPolicy(max_http_attempts_per_actor=1, max_calls_per_actor=5))
    assert transport.sent == 1 == sched.budget.used['r1'] == _ledger_attempts(sched)
    assert sched.unreported_attempts == []


def test_r5_p1_boundary_a_known_usage_reply_accounts_for_its_own_attempts():
    """A reply whose usage is KNOWN states how many attempts it sent; an unused
    reservation is refunded, and nothing is padded."""
    transport = _SendsRetryThenFails(lambda sent: ds.CallReply(attempts=(zc.Attempt(),), action='go'))
    sched = _run(transport, _cap2(max_retries=0))
    assert [len(call.cost.attempts) for call in sched.calls] == [1]
    assert sched.budget.used['r1'] == 1 and sched.unreported_attempts == []
    assert sched.calls[0].notes['usage_known'] is True


def test_r5_p1_boundary_more_attempts_than_reserved_stay_a_budget_breach():
    """Reporting MORE attempts than reserved is still the finding 15 breach."""
    three = (zc.Attempt(outcome='error'),) * 3
    transport = _SendsRetryThenFails(lambda sent: ds.TransportFailure('x', attempts=three,
                                                                      usage_known=False), retries=0)
    sched = _run(transport, _cap2(max_retries=0))
    assert sched.unreported_attempts == []
    assert sched.over_budget_attempts[0]['unreserved'] == 2 and sched.discarded[0]['reason'] == 'budget_breach'


# =========================================================================== #
# P2 — a summary-only record keeps the unknown marker and the known lower bound

def _summary_only(trial_id='peer_ko-mixed-s1', seed=1, *, tokens=None, **model):
    """A provisional trial record with a model summary and NO call rows."""
    summary = {'logical_calls': 1, 'http_attempts': 1, 'censored_calls': 0,
               'tokens': tokens if tokens is not None else {'input': 833, 'output': 40,
                                                            'image': 0, 'cached': 0},
               'sim_cost_s': {'call': 2.5, 'think': 2.5, 'talk': 0.0, 'delivery': 0.0}}
    summary.update(model)
    return {'schema': ev.PROVISIONAL_SCHEMA, 'trial_id': trial_id, 'condition': 'peer_ko',
            'scenario': 'mixed', 'seed': seed, 'robots': ['r1', 'r2', 'r3'],
            'end_reason': 'sim_horizon', 't0_sim_s': 0.0, 'end_sim_s': 60.0,
            'budget': {'sim_horizon_s': 60.0}, 'orders': [],
            'referee': {'deliveries': [], 'conflicts': [], 'deadlocks': []}, 'model': summary}


def _metrics(record):
    return ev.efficiency_metrics(ev.parse_trial(copy.deepcopy(record)))


UNKNOWN = {'tokens_complete': False, 'usage_unknown_calls': 1}


def test_r5_p2_a_summary_only_unknown_usage_stays_a_lower_bound(tmp_path):
    """The counterexample, from the metrics through the report and TensorBoard export."""
    record = _summary_only(**UNKNOWN)
    metrics = _metrics(record)
    assert metrics['model_cost_source'] == 'summary'
    assert metrics['tokens_complete'] is False and metrics['usage_unknown_calls'] == 1
    assert metrics['tokens_total'] is None and metrics['tokens_input'] is None
    assert metrics['tokens_output'] is None
    assert (metrics['tokens_input_lower_bound'], metrics['tokens_output_lower_bound'],
            metrics['tokens_total_lower_bound']) == (833, 40, 873)

    row = ev.summarise([ev.parse_trial(copy.deepcopy(record))])['conditions']['peer_ko']
    assert row['usage_unknown_calls'] == 1 and row['tokens_incomplete_trials'] == 1
    assert row['cohort_tokens_total'] is None and row['cohort_tokens_total_lower_bound'] == 873
    assert row['metrics']['tokens_total'] is None and row['metrics']['tokens_total_lower_bound'] == 873

    trials = tmp_path / 'trials'
    trials.mkdir()
    (trials / 'unknown.json').write_text(json.dumps(record, ensure_ascii=False))
    report.build([trials], tmp_path / 'report', resamples=50, now=0.0)
    text = (tmp_path / 'report' / 'summary.md').read_text()
    line = next(l for l in text.splitlines() if l.startswith(f'| {ev.CONDITION_LABELS_KO["peer_ko"]} | 1 |'))
    assert line.endswith('| ≥873 (미상 1) |'), line
    assert '토큰 사용량 미상' in text
    runs = {r['run']: r for r in json.loads((tmp_path / 'report' / 'scalars.json').read_text())['runs']}
    trial_run = runs['peer_ko/peer_ko-mixed-s1']
    assert 'result/tokens_total' not in trial_run['scalars']
    assert trial_run['scalars']['result/usage_unknown_calls'] == 1.0
    assert trial_run['scalars']['result/tokens_total_lower_bound'] == 873.0
    assert trial_run['hparams']['tokens_complete'] is False
    assert runs['cohort/peer_ko']['hparams']['tokens_complete'] is False


def test_r5_p2_a_count_alone_marks_the_summary_incomplete():
    metrics = _metrics(_summary_only(usage_unknown_calls=2))
    assert metrics['tokens_complete'] is False and metrics['usage_unknown_calls'] == 2
    assert metrics['tokens_total'] is None and metrics['tokens_total_lower_bound'] == 873


def test_r5_p2_an_incomplete_summary_must_count_its_unknown_calls():
    """Without the count, the report's ``(미상 n)`` marker would have nothing to show."""
    with pytest.raises(ev.TrialError, match='usage_unknown_calls'):
        _metrics(_summary_only(tokens_complete=False))


@pytest.mark.parametrize('model', [{'tokens_complete': True, 'usage_unknown_calls': 1},
                                   {'tokens_complete': False, 'usage_unknown_calls': 0}])
def test_r5_p2_a_contradictory_summary_marker_is_refused(model):
    with pytest.raises(ev.TrialError, match='contradict'):
        _metrics(_summary_only(**model))


@pytest.mark.parametrize('value', [True, False, '1', 1.5, -1, math.nan, math.inf, [1], {}])
def test_r5_p2_boundary_a_malformed_unknown_count_is_refused(value):
    with pytest.raises(ev.TrialError, match='usage_unknown_calls'):
        _metrics(_summary_only(usage_unknown_calls=value))


@pytest.mark.parametrize('value', ['false', 0, 1, 'True', [], math.nan])
def test_r5_p2_boundary_a_non_bool_complete_flag_is_refused(value):
    with pytest.raises(ev.TrialError, match='tokens_complete'):
        _metrics(_summary_only(tokens_complete=value))


@pytest.mark.parametrize('value', [math.nan, math.inf, -1, '833', True, 833.5])
def test_r5_p2_boundary_a_corrupt_summary_token_count_is_refused(value):
    """A corrupt count cannot become a lower bound (``-1``) or a silent cast (``'833'``)."""
    tokens = {'input': value, 'output': 40, 'image': 0, 'cached': 0}
    with pytest.raises(ev.TrialError, match='tokens.input'):
        _metrics(_summary_only(tokens=tokens, **UNKNOWN))


def test_r5_p2_boundary_controls_complete_zero_and_legacy_summaries():
    complete = _metrics(_summary_only(tokens_complete=True, usage_unknown_calls=0))
    assert complete['tokens_complete'] is True and complete['tokens_total'] == 873
    assert complete['usage_unknown_calls'] == 0
    # a legacy summary without either field reads as before: complete, count not reported
    legacy = _metrics(_summary_only())
    assert legacy['tokens_complete'] is True and legacy['tokens_total'] == 873
    assert legacy['usage_unknown_calls'] is None
    # an integral float is a count (JSON writers may emit 833.0); a missing term stays None
    floats = _metrics(_summary_only(tokens={'input': 833.0, 'output': 40, 'image': None, 'cached': 0}))
    assert floats['tokens_input'] == 833 and floats['tokens_total'] is None


def _known_calls(seed):
    reply = ds.CallReply(attempts=(zc.Attempt(input_tokens=100, output_tokens=20),))
    sched = ds.EventScheduler(ds.ReplayTransport({'r1': [reply]}), policy=ds.CallPolicy(max_calls_per_actor=1))
    sched.trigger('r1', 'start')
    sched.run(until_s=60.0)
    return _log(sched, seed=seed)['calls']


def test_r5_p2_a_summary_next_to_the_call_log_must_agree_on_the_marker():
    """With a call log, a summary that claims unknown usage the log does not show is refused."""
    record = _summary_only(**UNKNOWN)
    record['calls'] = _known_calls(1)
    record['model'].update({'tokens': {'input': 100, 'output': 20, 'image': 0, 'cached': 0}})
    for key in ('logical_calls', 'http_attempts', 'sim_cost_s'):
        record['model'].pop(key)
    with pytest.raises(ev.TrialError, match='usage_unknown_calls'):
        _metrics(record)
    record['model'].update({'tokens_complete': True, 'usage_unknown_calls': 0})
    assert _metrics(record)['tokens_total'] == 120


# =========================================================================== #
# P2 — one TensorBoard run per trial, never shared by repeated seeds

def _unknown_calls(seed):
    sched = ds.EventScheduler(_SendsRetryThenFails(FAILURES['plain_exception'], retries=0),
                              policy=ds.CallPolicy(max_calls_per_actor=1, max_retries=0))
    sched.trigger('r1', 'start')
    sched.run(until_s=60.0)
    return _log(sched, seed=seed)['calls']


def _calls_trial(trial_id, seed, calls):
    record = _summary_only(trial_id=trial_id, seed=seed)
    record.pop('model')
    record['calls'] = calls
    return record


def _repeated_seed_report(tmp_path, *, tb_events=None):
    """The counterexample: the same condition, scenario and seed twice, one trial
    with an unknown usage and one with 120 known tokens."""
    trials = tmp_path / 'trials'
    trials.mkdir()
    for record in (_calls_trial('peer_ko-mixed-s1', 1, _unknown_calls(1)),
                   _calls_trial('peer_ko-mixed-s1-rep', 1, _known_calls(1))):
        (trials / f'{record["trial_id"]}.json').write_text(json.dumps(record, ensure_ascii=False))
    return report.build([trials], tmp_path / 'report', resamples=50, now=0.0, tb_events=tb_events)


def test_r5_p2_repeated_trials_of_one_seed_get_separate_runs(tmp_path):
    _repeated_seed_report(tmp_path)
    exported = json.loads((tmp_path / 'report' / 'scalars.json').read_text())
    names = [r['run'] for r in exported['runs']]
    assert len(names) == len(set(names))
    runs = {r['run']: r for r in exported['runs']}
    unknown, known = runs['peer_ko/peer_ko-mixed-s1'], runs['peer_ko/peer_ko-mixed-s1-rep']
    assert 'result/tokens_total' not in unknown['scalars']
    assert unknown['scalars']['result/usage_unknown_calls'] == 1.0
    assert unknown['hparams']['tokens_complete'] is False
    assert known['scalars']['result/tokens_total'] == 120.0
    assert known['scalars']['result/usage_unknown_calls'] == 0.0
    assert known['hparams']['tokens_complete'] is True
    assert unknown['hparams']['seed'] == known['hparams']['seed'] == 1


def _tb_run(directory, run):
    from tensorboard.backend.event_processing import event_accumulator
    from tensorboard.plugins.hparams import plugin_data_pb2

    acc = event_accumulator.EventAccumulator(str(directory / run))
    acc.Reload()
    scalars = {tag: [e.value for e in acc.Scalars(tag)] for tag in acc.Tags()['scalars']}
    hparams = {}
    for content in acc.PluginTagToContent('hparams').values():
        data = plugin_data_pb2.HParamsPluginData.FromString(content)
        if data.HasField('session_start_info'):
            hparams = {k: v.string_value for k, v in data.session_start_info.hparams.items()}
    return scalars, hparams


def test_r5_p2_repeated_trials_stay_apart_in_the_tensorboard_event_files(tmp_path):
    """Read back from the event files: each run holds ONE trial's values."""
    pytest.importorskip('tensorboard', reason='the offline CI job has no tensorboard; '
                                              'the scalars.json test above is the enforced check')
    _repeated_seed_report(tmp_path, tb_events=tmp_path / 'events')
    scalars, hparams = _tb_run(tmp_path / 'events', 'peer_ko/peer_ko-mixed-s1')
    assert 'result/tokens_total' not in scalars and scalars['result/usage_unknown_calls'] == [1.0]
    assert hparams['tokens_complete'] == 'False'
    scalars, hparams = _tb_run(tmp_path / 'events', 'peer_ko/peer_ko-mixed-s1-rep')
    assert scalars['result/tokens_total'] == [120.0] and scalars['result/usage_unknown_calls'] == [0.0]
    assert hparams['tokens_complete'] == 'True'


def test_r5_p2_a_duplicate_trial_id_is_refused_by_the_export():
    record = _calls_trial('peer_ko-mixed-s1', 1, _known_calls(1))
    trials = [ev.parse_trial(copy.deepcopy(record)), ev.parse_trial(copy.deepcopy(record))]
    with pytest.raises(ev.TrialError, match='duplicate TensorBoard run'):
        ev.scalar_export(ev.summarise(trials))


@pytest.mark.parametrize('trial_id', ['../escape', 'a/b', '.hidden', 'a b', 'x\\y', 'a\x00b',
                                      '/abs', 'peer_ko-미상', 5])
def test_r5_p2_boundary_a_trial_id_that_is_not_one_safe_path_component_is_refused(trial_id):
    record = _calls_trial('peer_ko-mixed-s1', 1, _known_calls(1))
    record['trial_id'] = trial_id
    with pytest.raises(ev.TrialError, match='trial_id'):
        ev.scalar_export(ev.summarise([ev.parse_trial(record)]))


def _payload(run):
    return {'runs': [{'run': run, 'step': 0, 'hparams': {'condition': 'peer_ko'},
                      'scalars': {'result/model_calls': 1.0}}]}


def _tree(directory):
    return {str(p.relative_to(directory)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(directory.rglob('*')) if p.is_file()}


def test_r5_p2_an_existing_snapshot_run_is_never_written_again(tmp_path):
    """An old snapshot (for example v4's ``peer_ko/mixed-s1``) is never rewritten;
    the check runs before anything is written, so tensorboard is not needed."""
    events = tmp_path / 'events'
    (events / 'peer_ko' / 'old-trial').mkdir(parents=True)
    (events / 'peer_ko' / 'old-trial' / 'events.out.tfevents.1.old').write_bytes(b'old snapshot')
    before = _tree(events)
    payload = _payload('peer_ko/new-trial')
    payload['runs'].append(_payload('peer_ko/old-trial')['runs'][0])
    with pytest.raises(FileExistsError, match='peer_ko/old-trial'):
        report.write_events(payload, events, at=0.0)
    assert _tree(events) == before and not (events / 'peer_ko' / 'new-trial').exists()


def test_r5_p2_the_report_refuses_an_existing_run_before_writing_any_file(tmp_path):
    """``build`` checks the logdir first, so no half-written report is left behind."""
    events = tmp_path / 'events'
    (events / 'peer_ko' / 'peer_ko-mixed-s1').mkdir(parents=True)
    with pytest.raises(FileExistsError, match='peer_ko/peer_ko-mixed-s1'):
        _repeated_seed_report(tmp_path, tb_events=events)
    assert not (tmp_path / 'report').exists()
    assert sorted(p.name for p in (events / 'peer_ko').iterdir()) == ['peer_ko-mixed-s1']


@pytest.mark.parametrize('run', ['../outside', 'ABSOLUTE', 'peer_ko/../../outside', '', 'a//b',
                                 './a', 'a\\..\\outside', 'a\x00b', None])
def test_r5_p2_a_run_path_outside_the_logdir_is_refused(tmp_path, run):
    events = tmp_path / 'events'
    events.mkdir()
    if run == 'ABSOLUTE':                     # an absolute path, kept inside tmp_path
        run = str(tmp_path / 'outside')
    with pytest.raises(ValueError, match='run name'):
        report.write_events(_payload(run), events, at=0.0)
    assert not (tmp_path / 'outside').exists() and list(events.iterdir()) == []
