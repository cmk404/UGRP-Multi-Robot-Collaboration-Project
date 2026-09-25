"""Tests for the EVALUATION-ONLY study metrics (synthetic logs only).

No simulation, no model call, no network. The synthetic records follow the
provisional schema in ``docs/zone_study_metrics.md``; Package A
(``kiro/zone-study-contract``) owns the final one.
"""
from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from harness import zone_study_eval as ev
from scripts import zone_study_report as report


ALLOWED_KEYS = ['static_map', 'order_sheet', 'own_rgb', 'own_commands', 'inbox']


def trial(condition='peer_ko', scenario='mixed', seed=101, *, end_reason='orders_complete',
          end_sim_s=900.0, horizon=1800.0, leader_id=None, utterances=(), deliveries=None,
          idle=None, referee_extra=None, requests=None, replans=(), decision_changes=(),
          model=None, orders=None, provenance=None):
    """One synthetic trial record. Only the fields a test needs are filled."""
    orders = orders if orders is not None else [
        {'order_id': 'order-1', 'item_ids': ['long_beam-1'], 'kind': 'long_beam',
         'count': 1, 'required_robots': 2, 'destination_zone': 'A'},
        {'order_id': 'order-2', 'item_ids': ['crate-2'], 'kind': 'crate',
         'count': 1, 'required_robots': 1, 'destination_zone': 'B'},
    ]
    referee = {'deliveries': deliveries if deliveries is not None else [
        {'item_id': 'long_beam-1', 'zone': 'A', 'sim_s': 600.0, 'correct': True},
        {'item_id': 'crate-2', 'zone': 'B', 'sim_s': 850.0, 'correct': True}],
        'conflicts': [], 'deadlocks': []}
    referee.update(referee_extra or {})
    record = {
        'schema': ev.PROVISIONAL_SCHEMA,
        'trial_id': f'{condition}-{scenario}-s{seed}',
        'condition': condition, 'scenario': scenario, 'seed': seed,
        'robots': ['r1', 'r2', 'r3'],
        'provenance': provenance if provenance is not None else {
            'code_sha': 'abc123', 'map_id': 'zone_wide_two_doors',
            'map_sha256': 'f' * 64, 'order_sheet_sha256': 'e' * 64,
            'cost_profile_sha256': 'd' * 64},
        'budget': {'sim_horizon_s': horizon, 'http_attempts': 90, 'max_calls_per_actor': 30},
        't0_sim_s': 0.0, 'end_sim_s': end_sim_s, 'end_reason': end_reason,
        'orders': orders, 'referee': referee,
        'idle': idle if idle is not None else {
            'r1': {'thinking': 12.0, 'door_wait': 3.0},
            'r2': {'thinking': 9.0, 'team_rendezvous': 5.0},
            'r3': {'thinking': 7.0, 'unassigned': 20.0}},
        'replans': list(replans),
        'decision_changes': list(decision_changes),
        'model': model if model is not None else {
            'logical_calls': 24, 'http_attempts': 26,
            'tokens': {'input': 90000, 'output': 4200, 'image': 1800, 'cached': 0},
            'sim_cost_s': {'think': 24.0, 'talk': 3.0, 'delivery': 0.5},
            'wall_latency_ms': [810.0, 930.0]},
        'requests': requests if requests is not None else [
            {'request_id': 'req-1', 'robot': 'r1', 'sim_s': 0.0, 'input_keys': list(ALLOWED_KEYS)},
            {'request_id': 'req-2', 'robot': 'r2', 'sim_s': 1.0, 'input_keys': list(ALLOWED_KEYS)}],
        'utterances': list(utterances),
    }
    if leader_id:
        record['leader_id'] = leader_id
    return record


def utter(message_id='m-1', sender='r1', recipients=('r2',), sim_s=100.0, text='',
          encoding='free_ko', **extra):
    row = {'message_id': message_id, 'sender': sender, 'recipients': list(recipients),
           'sim_s': sim_s, 'delivered_sim_s': sim_s + 0.1, 'encoding': encoding,
           'text': text, 'sim_cost_s': 1.9}
    row.update(extra)
    return row


class ParseTest(unittest.TestCase):
    def test_unknown_schema_is_refused(self):
        bad = trial()
        bad['schema'] = 'ugrp.something.else.v9'
        with self.assertRaises(ev.TrialError):
            ev.parse_trial(bad)

    def test_unknown_condition_and_end_reason_refused(self):
        for key, value in (('condition', 'peer_english'), ('end_reason', 'went_fine')):
            bad = trial()
            bad[key] = value
            with self.assertRaises(ev.TrialError):
                ev.parse_trial(bad)

    def test_leader_id_required_only_for_leader_condition(self):
        with self.assertRaises(ev.TrialError):
            ev.parse_trial(trial(condition='leader_ko'))
        with self.assertRaises(ev.TrialError):
            ev.parse_trial(trial(condition='peer_ko', leader_id='r1'))
        parsed = ev.parse_trial(trial(condition='leader_ko', leader_id='r3'))
        self.assertEqual(parsed['leader_id'], 'r3')

    def test_leader_rotates_across_seeds(self):
        trials = [ev.parse_trial(trial(condition='leader_ko', seed=s, leader_id=l))
                  for s, l in ((101, 'r1'), (102, 'r2'), (103, 'r3'))]
        summary = ev.summarise(trials)
        self.assertEqual(summary['conditions']['leader_ko']['leader_ids'], ['r1', 'r2', 'r3'])

    def test_parse_does_not_mutate_input(self):
        raw = trial()
        before = json.dumps(raw, sort_keys=True)
        ev.parse_trial(raw)
        self.assertEqual(before, json.dumps(raw, sort_keys=True))

    def test_horizon_required(self):
        bad = trial()
        bad['budget'] = {}
        with self.assertRaises(ev.TrialError):
            ev.parse_trial(bad)

    def test_provisional_condition_names_are_normalised_onto_package_a(self):
        """Package A's names are canonical; the earlier spellings still parse."""
        for logged, expected in (('peer_structured', 'structured'),
                                 ('central_rgb_reference', ev.REFERENCE_CONDITION)):
            record = trial()
            record['condition'] = logged
            parsed = ev.parse_trial(record)
            self.assertEqual(parsed['condition'], expected)
            self.assertEqual(parsed['condition_as_logged'], logged)
        # package A's own names need no alias
        for name in ('no_comm', 'peer_ko', 'structured', ev.REFERENCE_CONDITION):
            record = trial()
            record['condition'] = name
            parsed = ev.parse_trial(record)
            self.assertEqual(parsed['condition'], name)
            self.assertNotIn('condition_as_logged', parsed)

    def test_package_c_envelope_fields_are_accepted(self):
        record = trial(condition='peer_ko', utterances=[{
            'message_id': 'm-1', 'from_robot': 'r1', 'recipients': ['r2'],
            'sent_at_sim_s': 700.0, 'delivered_at_sim_s': 700.1, 'encoding': 'ko_free',
            'text': 'long_beam-1을 A에 배달했습니다.', 'sim_cost_s': 1.9}])
        parsed = ev.parse_trial(record)
        self.assertEqual(parsed['utterances'][0]['sender'], 'r1')
        self.assertEqual(parsed['utterances'][0]['sim_s'], 700.0)
        dia = ev.dialogue_metrics(parsed)
        self.assertEqual(dia['messages'][0]['claims'][0]['verdict'], 'true')
        self.assertEqual(ev.channel_compliance(parsed)['violations'], [])

    def test_package_c_structured_payload_key_is_accepted(self):
        record = trial(condition='structured', utterances=[{
            'message_id': 'm-1', 'from_robot': 'r1', 'recipients': ['r2'],
            'sent_at_sim_s': 300.0, 'encoding': 'structured',
            'structured': {'act': 'reject', 'item': 'crate-2'}}])
        parsed = ev.parse_trial(record)
        self.assertEqual(ev.act_types(parsed['utterances'][0])['coarse'], ['objection'])
        self.assertEqual(ev.channel_compliance(parsed)['violations'], [])


class EfficiencyTest(unittest.TestCase):
    def test_success_makespan_and_deliveries(self):
        eff = ev.efficiency_metrics(ev.parse_trial(trial(end_sim_s=900.0)))
        self.assertTrue(eff['success'])
        self.assertFalse(eff['censored'])
        self.assertAlmostEqual(eff['makespan_sim_s'], 900.0)
        self.assertAlmostEqual(eff['par_makespan_sim_s'], 900.0)
        self.assertEqual(eff['delivered_items'], 2)
        self.assertEqual(eff['ordered_items'], 2)
        self.assertEqual(eff['delivery_rate'], 1.0)
        self.assertAlmostEqual(eff['talk_sim_cost_s'], 3.5)

    def test_fast_failure_never_beats_slow_success(self):
        slow_success = ev.efficiency_metrics(ev.parse_trial(
            trial(end_sim_s=1799.0, end_reason='orders_complete')))
        for reason in ev.FAILURE_END_REASONS:
            fast_failure = ev.efficiency_metrics(ev.parse_trial(
                trial(end_sim_s=40.0, end_reason=reason, deliveries=[])))
            self.assertGreater(fast_failure['par_makespan_sim_s'],
                               slow_success['par_makespan_sim_s'], reason)
            self.assertTrue(fast_failure['censored'], reason)
            self.assertIsNone(fast_failure['makespan_success_only_s'], reason)

    def test_failures_stay_in_the_denominators(self):
        trials = [ev.parse_trial(trial(seed=101)),
                  ev.parse_trial(trial(seed=102, end_reason='sim_horizon',
                                       end_sim_s=1800.0, deliveries=[])),
                  ev.parse_trial(trial(seed=103, end_reason='budget_exhausted',
                                       end_sim_s=120.0, deliveries=[]))]
        row = ev.summarise(trials)['conditions']['peer_ko']
        self.assertEqual(row['trials'], 3)
        self.assertEqual(row['successes'], 1)
        self.assertAlmostEqual(row['success_rate'], 1 / 3)
        self.assertEqual(row['cohort_ordered_items'], 6)
        self.assertEqual(row['cohort_delivered_items'], 2)
        self.assertAlmostEqual(row['cohort_delivery_rate'], 2 / 6)
        self.assertEqual(row['end_reasons'],
                         {'orders_complete': 1, 'sim_horizon': 1, 'budget_exhausted': 1})
        # charged = 900 + 2*3600 = 8100 over 2 delivered items
        self.assertAlmostEqual(row['cohort_par_sim_s_per_delivered'], 8100.0 / 2)

    def test_penalty_factor_must_not_reward_failure(self):
        with self.assertRaises(ev.TrialError):
            ev.efficiency_metrics(ev.parse_trial(trial()), penalty_factor=0.5)

    def test_success_after_horizon_is_refused(self):
        with self.assertRaises(ev.TrialError):
            ev.efficiency_metrics(ev.parse_trial(trial(end_sim_s=2000.0, horizon=1800.0)))

    def test_item_counted_once_and_misdelivery_split(self):
        eff = ev.efficiency_metrics(ev.parse_trial(trial(deliveries=[
            {'item_id': 'long_beam-1', 'zone': 'A', 'sim_s': 100.0},
            {'item_id': 'long_beam-1', 'zone': 'A', 'sim_s': 200.0},
            {'item_id': 'crate-2', 'zone': 'C', 'sim_s': 300.0}])))
        self.assertEqual(eff['delivered_items'], 1)
        self.assertEqual(eff['misdelivered_items'], 1)
        self.assertEqual(eff['undelivered_items'], 1)

    def test_idle_conflict_deadlock_replan_counts(self):
        record = trial(
            referee_extra={'conflicts': [{'kind': 'role_contention', 'sim_s': 30.0},
                                         {'kind': 'conflicting_destination', 'sim_s': 60.0}],
                           'deadlocks': [{'sim_s': 90.0, 'duration_s': 25.0}]},
            replans=[{'sim_s': 100.0, 'robot': 'r1', 'kind': 'job_change'},
                     {'sim_s': 200.0, 'robot': 'r2', 'kind': 'passage_change'}])
        eff = ev.efficiency_metrics(ev.parse_trial(record))
        self.assertEqual(eff['conflicts'], 2)
        self.assertEqual(eff['conflicts_by_kind'],
                         {'role_contention': 1, 'conflicting_destination': 1})
        self.assertEqual(eff['deadlocks'], 1)
        self.assertAlmostEqual(eff['deadlock_sim_s'], 25.0)
        self.assertEqual(eff['replans'], 2)
        self.assertAlmostEqual(eff['idle_robot_s'], 56.0)
        self.assertEqual(eff['idle_by_reason']['thinking'], 28.0)

    def test_unknown_idle_reason_folds_into_other(self):
        eff = ev.efficiency_metrics(ev.parse_trial(
            trial(idle={'r1': {'staring_at_wall': 5.0}})))
        self.assertEqual(eff['idle_by_reason'], {'other': 5.0})


class BoundaryTest(unittest.TestCase):
    def test_clean_trial_passes(self):
        audit = ev.audit_input_boundary(ev.parse_trial(trial()))
        self.assertTrue(audit['clean'])
        self.assertEqual(audit['input_leaks'], [])

    def test_referee_and_top_inputs_are_flagged(self):
        for leaked in ('top_rgb', 'referee', 'gt_pose', 'teacher_receipt',
                       'peer_rgb', 'measured_joints', 'nav_cam'):
            record = trial(requests=[{'request_id': 'req-1', 'robot': 'r1', 'sim_s': 0.0,
                                      'input_keys': ALLOWED_KEYS + [leaked]}])
            audit = ev.audit_input_boundary(ev.parse_trial(record))
            self.assertFalse(audit['clean'], leaked)
            self.assertEqual(audit['input_leaks'][0]['forbidden_input_keys'], [leaked])

    def test_unknown_input_key_is_not_silently_allowed(self):
        record = trial(requests=[{'request_id': 'req-1', 'robot': 'r1', 'sim_s': 0.0,
                                  'input_keys': ALLOWED_KEYS + ['mystery_channel']}])
        audit = ev.audit_input_boundary(ev.parse_trial(record))
        self.assertFalse(audit['clean'])
        self.assertEqual(audit['unknown_input_keys'][0]['unknown_input_keys'], ['mystery_channel'])

    def test_forbidden_grounds_citation_is_flagged(self):
        record = trial(utterances=[utter(text='crate-2를 B에 배달했습니다.',
                                         grounds=['referee-deliveries-3'])])
        audit = ev.audit_input_boundary(ev.parse_trial(record))
        self.assertFalse(audit['clean'])
        self.assertEqual(audit['forbidden_grounds'][0]['forbidden_grounds'],
                         ['referee-deliveries-3'])

    def test_reference_condition_is_excluded_from_main_boundary_verdict(self):
        record = trial(condition=ev.REFERENCE_CONDITION,
                       requests=[{'request_id': 'req-1', 'robot': 'commander', 'sim_s': 0.0,
                                  'input_keys': ['static_map', 'order_sheet', 'peer_rgb']}])
        audit = ev.audit_input_boundary(ev.parse_trial(record))
        self.assertFalse(audit['clean'])
        self.assertTrue(audit['input_leaks'][0]['reference_condition'])

    def test_metrics_are_read_only(self):
        record = ev.parse_trial(trial(
            utterances=[utter(text='r2, door_narrow가 막혀 있습니다.')],
            replans=[{'sim_s': 120.0, 'robot': 'r2', 'kind': 'passage_change'}]))
        snapshot = copy.deepcopy(record)
        ev.trial_metrics(record)
        self.assertEqual(snapshot, record)

    def test_evaluation_verdicts_are_not_written_into_robot_inputs(self):
        """Truthfulness verdicts must never appear in a request or an inbox."""
        record = ev.parse_trial(trial(utterances=[
            utter(sim_s=700.0, text='long_beam-1을 A에 배달했습니다.')]))
        out = ev.trial_metrics(record)
        self.assertEqual(out['dialogue']['messages'][0]['claims'][0]['verdict'], 'true')
        blob = json.dumps(record, ensure_ascii=False)
        for token in ('verdict', 'claims_true', 'par_makespan', 'truthful_share'):
            self.assertNotIn(token, blob)
        for request in record['requests']:
            self.assertTrue(set(request['input_keys']) <= ev.ALLOWED_INPUT_KEYS)


class ChannelTest(unittest.TestCase):
    def test_no_comm_must_be_silent(self):
        clean = ev.channel_compliance(ev.parse_trial(trial(condition='no_comm')))
        self.assertEqual(clean['violations'], [])
        noisy = ev.channel_compliance(ev.parse_trial(
            trial(condition='no_comm', utterances=[utter(text='r2, 도와주세요.')])))
        self.assertEqual(noisy['violations'][0]['kind'], 'no_comm_message')

    def test_leader_hub_and_spoke_only(self):
        ok = ev.channel_compliance(ev.parse_trial(trial(
            condition='leader_ko', leader_id='r2',
            utterances=[utter('m-1', 'r2', ('r1',), text='r1, crate-2를 B로 배달하십시오.'),
                        utter('m-2', 'r1', ('r2',), text='알겠습니다.')])))
        self.assertEqual(ok['violations'], [])
        bad = ev.channel_compliance(ev.parse_trial(trial(
            condition='leader_ko', leader_id='r2',
            utterances=[utter('m-3', 'r1', ('r3',), text='r3, 같이 갑시다.')])))
        self.assertEqual(bad['violations'][0]['kind'], 'follower_to_follower')

    def test_leader_broadcast_is_flagged(self):
        bad = ev.channel_compliance(ev.parse_trial(trial(
            condition='leader_ko', leader_id='r1',
            utterances=[utter('m-1', 'r1', ('r2', 'r3'), text='모두 대기하십시오.')])))
        self.assertEqual(bad['violations'][0]['kind'], 'leader_broadcast')

    def test_structured_condition_rejects_free_text(self):
        bad = ev.channel_compliance(ev.parse_trial(trial(
            condition='peer_structured',
            utterances=[utter('m-1', 'r1', ('r2',), text='그냥 말로 할게요',
                              encoding='structured',
                              message={'act': 'inform', 'item': 'crate-2', 'state': 'held'})])))
        self.assertEqual([v['kind'] for v in bad['violations']], ['free_text_in_structured'])

    def test_korean_condition_rejects_structured_encoding(self):
        bad = ev.channel_compliance(ev.parse_trial(trial(
            condition='peer_ko',
            utterances=[utter(encoding='structured', message={'act': 'inform'})])))
        self.assertEqual(bad['violations'][0]['kind'], 'non_korean_encoding')

    def test_unknown_recipient_is_flagged(self):
        bad = ev.channel_compliance(ev.parse_trial(trial(
            utterances=[utter(recipients=('r9',), text='안녕하세요.')])))
        self.assertIn('unknown_recipient', [v['kind'] for v in bad['violations']])


class DialogueTest(unittest.TestCase):
    def test_korean_share_ignores_literal_ids(self):
        record = ev.parse_trial(trial(utterances=[
            utter('m-1', text='r2, long_beam-1을 A 구역으로 함께 옮기겠습니다.'),
            utter('m-2', text='I will take crate-2 to B instead.'),
            utter('m-3', text='   ')]))
        dia = ev.dialogue_metrics(record)
        self.assertEqual(dia['korean_checked'], 2)
        self.assertEqual(dia['korean_ok'], 1)
        self.assertEqual(dia['korean_share'], 0.5)
        self.assertEqual(dia['silent_messages'], 1)
        self.assertEqual(dia['code_switch_messages'], 1)

    def test_silence_is_not_korean_success(self):
        dia = ev.dialogue_metrics(ev.parse_trial(trial(utterances=[utter(text='')])))
        self.assertEqual(dia['korean_checked'], 0)
        self.assertIsNone(dia['korean_share'])
        self.assertEqual(dia['silent_messages'], 1)

    def test_translated_ids_are_reported(self):
        dia = ev.dialogue_metrics(ev.parse_trial(trial(utterances=[
            utter(text='로봇 2, 빨강-1을 에이 구역으로 옮기세요.')])))
        kinds = {issue['kind'] for row in dia['id_issues'] for issue in row['id_issues']}
        self.assertTrue({'robot_id_variant', 'translated_label', 'translated_zone'} <= kinds)

    def test_declared_map_literals_are_not_code_switching(self):
        record = trial(utterances=[
            utter(text='door_narrow가 막혀 있어 door_wide로 우회합니다.')])
        record['literals'] = ['door_narrow', 'door_wide', 'P1', 'P1-2']
        dia = ev.dialogue_metrics(ev.parse_trial(record))
        self.assertEqual(dia['code_switch_messages'], 0)
        self.assertEqual(dia['korean_share'], 1.0)

    def test_undeclared_english_still_counts_as_code_switching(self):
        dia = ev.dialogue_metrics(ev.parse_trial(trial(utterances=[
            utter(text='The narrow door seems blocked, rerouting now.')])))
        self.assertEqual(dia['code_switch_messages'], 1)
        self.assertEqual(dia['korean_ok'], 0)

    def test_referee_named_passages_count_as_literals(self):
        record = trial(
            referee_extra={'blockages': [{'passage': 'corridor_north', 'from_s': 0.0, 'to_s': None}]},
            utterances=[utter(text='corridor_north가 막혀 있습니다.')])
        self.assertIn('corridor_north', ev.literal_tokens(ev.parse_trial(record)))
        dia = ev.dialogue_metrics(ev.parse_trial(record))
        self.assertEqual(dia['code_switch_messages'], 0)

    def test_act_types_cover_the_five_study_kinds(self):
        record = ev.parse_trial(trial(condition='leader_ko', leader_id='r1', utterances=[
            utter('m-1', 'r2', ('r1',), text='long_beam-1을 A에 배달했습니다.'),
            utter('m-2', 'r2', ('r1',), text='crate-2를 대신 가져와 주세요.'),
            utter('m-3', 'r1', ('r2',), text='r2, crate-2를 B로 배달하십시오.'),
            utter('m-4', 'r2', ('r1',), text='그 경로는 막혀 있어서 거절합니다.'),
            utter('m-5', 'r2', ('r1',), text='알겠습니다.')]))
        dia = ev.dialogue_metrics(record)
        for kind in ev.ACT_TYPES:
            self.assertIn(kind, dia['acts_coarse'], kind)

    def test_polite_request_is_not_an_order(self):
        acts = ev.act_types(utter(text='crate-2를 B로 옮겨 주세요.'))
        self.assertIn('request', acts['coarse'])
        self.assertNotIn('order', acts['coarse'])

    def test_structured_act_enum_maps_to_coarse_types(self):
        acts = ev.act_types(utter(encoding='schema',
                                  message={'act': 'reject', 'item': 'crate-2'}))
        self.assertEqual(acts['fine'], ['reject'])
        self.assertEqual(acts['coarse'], ['objection'])

    def test_truthfulness_against_the_referee(self):
        record = ev.parse_trial(trial(utterances=[
            # true: referee delivered long_beam-1 to A at 600s
            utter('m-1', sim_s=700.0, text='long_beam-1을 A에 배달했습니다.'),
            # false: claimed before the referee delivery time
            utter('m-2', sim_s=500.0, text='crate-2를 B에 배달했습니다.'),
            # false: wrong zone
            utter('m-3', sim_s=900.0, text='long_beam-1을 C에 배달했습니다.')]))
        dia = ev.dialogue_metrics(record)
        verdicts = [m['claims'][0]['verdict'] for m in dia['messages']]
        self.assertEqual(verdicts, ['true', 'false', 'false'])
        self.assertEqual(dia['claims_true'], 1)
        self.assertEqual(dia['claims_false'], 2)
        self.assertAlmostEqual(dia['truthful_share'], 1 / 3)

    def test_missing_referee_sublog_is_unverifiable_not_false(self):
        record = ev.parse_trial(trial(utterances=[
            utter(text='door_narrow가 막혀 있습니다.')]))
        dia = ev.dialogue_metrics(record)
        self.assertEqual(dia['messages'][0]['claims'][0]['verdict'], 'unverifiable')
        self.assertEqual(dia['claims_false'], 0)
        self.assertIsNone(dia['truthful_share'])

    def test_blockage_and_hold_claims_use_intervals(self):
        record = ev.parse_trial(trial(
            referee_extra={'blockages': [{'passage': 'door_narrow', 'from_s': 100.0, 'to_s': 400.0}],
                           'holds': [{'item_id': 'crate-2', 'robot': 'r1',
                                      'from_s': 200.0, 'to_s': None}]},
            utterances=[utter('m-1', sim_s=200.0, text='door_narrow가 막혀 있습니다.'),
                        utter('m-2', sim_s=500.0, text='door_narrow가 막혀 있습니다.'),
                        utter('m-3', sim_s=300.0, text='crate-2를 들고 있습니다.')]))
        dia = ev.dialogue_metrics(record)
        self.assertEqual([m['claims'][0]['verdict'] for m in dia['messages']],
                         ['true', 'false', 'true'])

    def test_claims_are_scoped_to_the_cue_sentence(self):
        claims = ev.extract_claims(
            utter(text='door_narrow가 막혀 있습니다. door_wide로 우회하십시오.'))
        self.assertEqual(claims, [{'type': 'blocked', 'passage': 'door_narrow'}])

    def test_delivered_zone_falls_back_across_sentences(self):
        claims = ev.extract_claims(
            utter(text='A 구역에 도착했습니다. long_beam-1을 내려놓았습니다.'),
            labels=('long_beam-1',))
        self.assertIn({'type': 'delivered', 'item_id': 'long_beam-1', 'zone': 'A'}, claims)

    def test_unrelated_item_in_another_sentence_is_not_claimed_delivered(self):
        claims = ev.extract_claims(
            utter(text='long_beam-1을 A에 배달했습니다. crate-2는 아직 찾지 못했습니다.'),
            labels=('long_beam-1', 'crate-2'))
        self.assertEqual([c['item_id'] for c in claims if c['type'] == 'delivered'],
                         ['long_beam-1'])

    def test_explicit_claims_win_over_rules(self):
        record = ev.parse_trial(trial(utterances=[
            utter(text='상황 공유합니다.', sim_s=700.0,
                  claims=[{'type': 'delivered', 'item_id': 'long_beam-1', 'zone': 'A'}])]))
        dia = ev.dialogue_metrics(record)
        self.assertEqual(dia['messages'][0]['claims'][0]['verdict'], 'true')

    def test_grounds_verdicts(self):
        self.assertEqual(ev.grounds_verdict(utter(grounds=['own-r1-0042', 'command-r1-0018'])),
                         'grounded')
        self.assertEqual(ev.grounds_verdict(utter(grounds=['top-0007'])), 'forbidden')
        self.assertEqual(ev.grounds_verdict(utter()), 'unknown')

    def test_preceding_utterances_before_decision_changes(self):
        record = ev.parse_trial(trial(
            utterances=[utter('m-1', 'r1', ('r2',), sim_s=100.0,
                              text='door_narrow가 막혀 있습니다. 우회하십시오.'),
                        utter('m-2', 'r1', ('r3',), sim_s=100.0, text='r3, 대기하십시오.')],
            decision_changes=[{'sim_s': 110.0, 'robot': 'r2', 'kind': 'passage_change',
                               'reason': 'peer report'},
                              {'sim_s': 900.0, 'robot': 'r2', 'kind': 'job_change'}]))
        influence = ev.dialogue_metrics(record)['decision_influence']
        self.assertEqual(influence['decision_changes'], 2)
        self.assertEqual(influence['changes_with_prior_inbound'], 1)
        self.assertEqual(influence['rows'][0]['preceding_inbound'][0]['message_id'], 'm-1')
        self.assertEqual(influence['rows'][1]['preceding_inbound'], [])

    def test_replans_are_used_when_decision_changes_absent(self):
        record = ev.parse_trial(trial(
            utterances=[utter('m-1', 'r1', ('r2',), sim_s=100.0, text='막혀 있습니다.')],
            replans=[{'sim_s': 105.0, 'robot': 'r2', 'kind': 'passage_change'}]))
        influence = ev.dialogue_metrics(record)['decision_influence']
        self.assertEqual(influence['decision_changes'], 1)
        self.assertEqual(influence['changes_with_prior_inbound'], 1)

    def test_no_comm_has_no_preceding_utterances(self):
        record = ev.parse_trial(trial(condition='no_comm',
                                      replans=[{'sim_s': 105.0, 'robot': 'r2', 'kind': 'job_change'}]))
        influence = ev.dialogue_metrics(record)['decision_influence']
        self.assertEqual(influence['changes_with_prior_inbound'], 0)


class ComparisonTest(unittest.TestCase):
    def cohort(self):
        trials = []
        for seed, (base, var) in enumerate([(1000.0, 900.0), (1200.0, 1000.0),
                                            (800.0, 850.0), (1100.0, 950.0)], start=101):
            trials.append(ev.parse_trial(trial('no_comm', seed=seed, end_sim_s=base)))
            trials.append(ev.parse_trial(trial('peer_ko', seed=seed, end_sim_s=var)))
        return trials

    def test_paired_values_match_on_scenario_and_seed(self):
        pairs = ev.paired_values(self.cohort(), 'par_makespan_sim_s', 'no_comm', 'peer_ko')
        self.assertEqual(len(pairs), 4)
        self.assertEqual([p['seed'] for p in pairs], [101, 102, 103, 104])

    def test_unmatched_seeds_are_dropped(self):
        trials = self.cohort() + [ev.parse_trial(trial('peer_ko', seed=999))]
        pairs = ev.paired_values(trials, 'par_makespan_sim_s', 'no_comm', 'peer_ko')
        self.assertEqual(len(pairs), 4)

    def test_comparison_reports_interval_without_p_value(self):
        out = ev.compare_conditions(self.cohort(), 'par_makespan_sim_s',
                                    'no_comm', 'peer_ko', resamples=500, seed=7)
        self.assertEqual(out['n_pairs'], 4)
        self.assertAlmostEqual(out['mean_diff'], -100.0)
        self.assertEqual(out['pairs_variant_lower'], 3)
        self.assertIsNotNone(out['diff_ci']['low'])
        self.assertLessEqual(out['diff_ci']['low'], out['mean_diff'])
        self.assertGreaterEqual(out['diff_ci']['high'], out['mean_diff'])
        self.assertTrue(out['small_sample'])
        self.assertNotIn('p_value', out)
        self.assertNotIn('significant', json.dumps(out))

    def test_bootstrap_is_deterministic_for_a_fixed_seed(self):
        cohort = self.cohort()
        first = ev.compare_conditions(cohort, 'par_makespan_sim_s', 'no_comm', 'peer_ko',
                                      resamples=500, seed=3)['diff_ci']
        second = ev.compare_conditions(cohort, 'par_makespan_sim_s', 'no_comm', 'peer_ko',
                                       resamples=500, seed=3)['diff_ci']
        self.assertEqual(first, second)

    def test_single_pair_reports_no_interval(self):
        trials = [ev.parse_trial(trial('no_comm', seed=1, end_sim_s=1000.0)),
                  ev.parse_trial(trial('peer_ko', seed=1, end_sim_s=900.0))]
        ci = ev.compare_conditions(trials, 'par_makespan_sim_s', 'no_comm', 'peer_ko',
                                   resamples=100)['diff_ci']
        self.assertEqual(ci['n'], 1)
        self.assertIsNone(ci['low'])

    def test_success_metric_is_comparable_with_failures_kept(self):
        trials = [ev.parse_trial(trial('no_comm', seed=1, end_reason='sim_horizon',
                                       end_sim_s=1800.0, deliveries=[])),
                  ev.parse_trial(trial('peer_ko', seed=1, end_sim_s=900.0)),
                  ev.parse_trial(trial('no_comm', seed=2, end_sim_s=1000.0)),
                  ev.parse_trial(trial('peer_ko', seed=2, end_sim_s=950.0))]
        out = ev.compare_conditions(trials, 'success', 'no_comm', 'peer_ko', resamples=200)
        self.assertEqual(out['baseline_mean'], 0.5)
        self.assertEqual(out['variant_mean'], 1.0)
        self.assertAlmostEqual(out['mean_diff'], 0.5)

    def test_reference_condition_excluded_unless_requested(self):
        trials = self.cohort() + [
            ev.parse_trial(trial(ev.REFERENCE_CONDITION, seed=s, end_sim_s=700.0))
            for s in (101, 102, 103, 104)]
        without = ev.compare_all(trials, metrics=('par_makespan_sim_s',), resamples=100)
        self.assertEqual({c['variant'] for c in without}, {'peer_ko'})
        with_ref = ev.compare_all(trials, metrics=('par_makespan_sim_s',),
                                  include_reference=True, resamples=100)
        self.assertIn(ev.REFERENCE_CONDITION, {c['variant'] for c in with_ref})

    def test_repeats_of_a_seed_are_averaged_into_one_pair(self):
        trials = [ev.parse_trial(trial('no_comm', seed=1, end_sim_s=1000.0)),
                  ev.parse_trial(trial('peer_ko', seed=1, end_sim_s=800.0)),
                  ev.parse_trial(trial('peer_ko', seed=1, end_sim_s=1000.0))]
        trials[2]['trial_id'] = 'peer_ko-mixed-s1-rep2'
        pairs = ev.paired_values(trials, 'par_makespan_sim_s', 'no_comm', 'peer_ko')
        self.assertEqual(len(pairs), 1)
        self.assertAlmostEqual(pairs[0]['variant'], 900.0)
        self.assertEqual(pairs[0]['reps'], (1, 2))

    def test_rank_biserial_bounds(self):
        self.assertEqual(ev._rank_biserial([1.0, 2.0, 3.0]), 1.0)
        self.assertEqual(ev._rank_biserial([-1.0, -2.0]), -1.0)
        self.assertIsNone(ev._rank_biserial([0.0, 0.0]))


class SummaryTest(unittest.TestCase):
    def test_mixed_provenance_is_flagged(self):
        trials = [ev.parse_trial(trial(seed=1)),
                  ev.parse_trial(trial(seed=2, provenance={'code_sha': 'zzz999',
                                                           'map_id': 'zone_wide_two_doors'}))]
        prov = ev.summarise(trials)['provenance']
        self.assertIn('code_sha', prov['mixed_fields'])
        self.assertFalse(prov['single_bundle'])

    def test_scalar_export_shape(self):
        trials = [ev.parse_trial(trial('no_comm', seed=1)),
                  ev.parse_trial(trial('peer_ko', seed=1,
                                       utterances=[utter(text='A 구역으로 갑니다.')]))]
        scalars = ev.scalar_export(ev.summarise(trials))
        runs = {r['run'] for r in scalars['runs']}
        self.assertIn('no_comm/mixed-s1', runs)
        self.assertIn('cohort/peer_ko', runs)
        trial_run = next(r for r in scalars['runs'] if r['run'] == 'peer_ko/mixed-s1')
        self.assertIn('result/par_makespan_sim_s', trial_run['scalars'])
        self.assertIn('dialogue/utterances', trial_run['scalars'])
        self.assertEqual(trial_run['hparams']['condition'], 'peer_ko')
        for value in trial_run['scalars'].values():
            self.assertIsInstance(value, float)

    def test_empty_input_yields_no_trials(self):
        self.assertEqual(ev.load_trials([]), [])


class ReportTest(unittest.TestCase):
    def setUp(self):
        import tempfile
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.trials = self.dir / 'trials'
        self.trials.mkdir()
        rows = []
        for seed, (base, var) in enumerate([(1000.0, 900.0), (1200.0, 1000.0),
                                            (900.0, 950.0)], start=101):
            rows.append(trial('no_comm', seed=seed, end_sim_s=base))
            rows.append(trial('peer_ko', seed=seed, end_sim_s=var, utterances=[
                utter('m-1', 'r1', ('r2',), sim_s=700.0,
                      text='long_beam-1을 A에 배달했습니다.', grounds=['own-r1-0042']),
                utter('m-2', 'r2', ('r1',), sim_s=705.0, text='알겠습니다.')],
                decision_changes=[{'sim_s': 710.0, 'robot': 'r2', 'kind': 'job_change'}]))
            leader = ['r1', 'r2', 'r3'][seed % 3]
            follower = next(r for r in ('r1', 'r2', 'r3') if r != leader)
            rows.append(trial('leader_ko', seed=seed, end_sim_s=var + 20.0, leader_id=leader,
                              utterances=[utter('m-1', leader, (follower,),
                                                text='crate-2를 B로 배달하십시오.')]))
            rows.append(trial('peer_structured', seed=seed, end_sim_s=var + 10.0,
                              utterances=[utter('m-1', 'r1', ('r2',), encoding='structured',
                                                message={'act': 'inform', 'item': 'crate-2',
                                                         'state': 'held'})]))
        for row in rows:
            (self.trials / f'{row["trial_id"]}.json').write_text(
                json.dumps(row, ensure_ascii=False))

    def tearDown(self):
        self.tmp.cleanup()

    def test_report_writes_korean_summary_and_scalars(self):
        out = report.build([self.trials], self.dir / 'report', resamples=200, now=0.0)
        self.assertEqual(out['trials'], 12)
        text = (self.dir / 'report' / 'summary.md').read_text()
        self.assertIn('한국어 로봇 대화 효율 연구', text)
        for label in ev.CONDITION_LABELS_KO.values():
            if label.startswith('R '):
                continue
            self.assertIn(label, text)
        self.assertIn('빠른 실패가 느린 성공보다 좋게 보이지 않는다', text)
        metrics = json.loads((self.dir / 'report' / 'metrics.json').read_text())
        self.assertEqual(metrics['summary']['conditions']['peer_ko']['trials'], 3)
        self.assertEqual(len(metrics['sources']), 12)
        scalars = json.loads((self.dir / 'report' / 'scalars.json').read_text())
        self.assertEqual(scalars['schema'], 'ugrp.zone_study_scalars.v1')
        self.assertTrue(any(r['run'].startswith('cohort/') for r in scalars['runs']))

    def test_report_records_source_hashes(self):
        report.build([self.trials], self.dir / 'report', resamples=100, now=0.0)
        metrics = json.loads((self.dir / 'report' / 'metrics.json').read_text())
        for row in metrics['sources']:
            self.assertEqual(len(row['sha256']), 64)
            self.assertEqual(row['sha256'], report.sha256_file(row['path']))

    def test_report_flags_a_leaking_trial(self):
        bad = trial('peer_ko', seed=777, requests=[
            {'request_id': 'req-1', 'robot': 'r1', 'sim_s': 0.0,
             'input_keys': ALLOWED_KEYS + ['referee']}])
        (self.trials / 'leak.json').write_text(json.dumps(bad, ensure_ascii=False))
        out = report.build([self.trials], self.dir / 'report', resamples=100, now=0.0)
        self.assertEqual(out['boundary_clean_trials'], out['trials'] - 1)
        text = (self.dir / 'report' / 'summary.md').read_text()
        self.assertIn('평가 자료가 로봇 입력으로 흘러간 시행이 있다', text)
        self.assertIn('referee', text)

    def test_reference_trials_do_not_raise_the_main_leak_warning(self):
        ref = trial(ev.REFERENCE_CONDITION, seed=888, requests=[
            {'request_id': 'req-1', 'robot': 'commander', 'sim_s': 0.0,
             'input_keys': ['static_map', 'order_sheet', 'peer_rgb']}])
        (self.trials / 'reference.json').write_text(json.dumps(ref, ensure_ascii=False))
        report.build([self.trials], self.dir / 'report', resamples=100, now=0.0)
        text = (self.dir / 'report' / 'summary.md').read_text()
        self.assertIn('주 4조건의 모든 시행에서 금지 입력 key', text)
        self.assertIn('전지적 참조 상한', text)
        self.assertNotIn('평가 자료가 로봇 입력으로 흘러간 시행이 있다', text)

    def test_comparison_table_is_grouped_by_metric(self):
        report.build([self.trials], self.dir / 'report', resamples=100, now=0.0)
        text = (self.dir / 'report' / 'summary.md').read_text()
        self.assertIn('**par_makespan_sim_s**', text)
        self.assertIn('**success**', text)
        self.assertIn('차이 부호는 `비교 − 기준`', text)

    def test_duplicate_trial_ids_across_files_are_refused(self):
        row = trial('peer_ko', seed=101)
        (self.trials / 'copy.json').write_text(json.dumps(row, ensure_ascii=False))
        with self.assertRaises(ev.TrialError):
            report.build([self.trials], self.dir / 'report', resamples=50, now=0.0)

    def test_tensorboard_events_are_written_when_requested(self):
        try:
            import tensorboard  # noqa: F401
        except ImportError:
            self.skipTest('tensorboard not installed')
        out = report.build([self.trials], self.dir / 'report', resamples=100,
                           tb_events=self.dir / 'events', now=0.0)
        self.assertEqual(out['events']['runs'], out['runs'])
        self.assertGreater(out['events']['scalars'], 0)
        files = list((self.dir / 'events').rglob('events.out.tfevents.*'))
        self.assertEqual(len(files), out['runs'])
        self.assertTrue(all(f.stat().st_size > 0 for f in files))

    def test_cli_main_runs(self):
        code = report.main([str(self.trials), '--output', str(self.dir / 'cli'),
                            '--bootstrap-resamples', '50'])
        self.assertEqual(code, 0)
        self.assertTrue((self.dir / 'cli' / 'summary.md').exists())


if __name__ == '__main__':
    unittest.main()
