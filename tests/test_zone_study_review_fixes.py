"""2026-09-26 Codex adversarial review: one regression per finding.

Source: ``docs/design/2026-09-26-zone-study-packages-review-codex.md`` (18
findings against packages A/C/D/E/I and the integration branch). Each test here
reproduces the counterexample the review reported and asserts the fixed
behaviour, so a regression re-opens a named finding instead of a nameless test.

Offline only: no simulator window, no model call, no network. The fixture frames
are the small JPEG bytes of ``harness.zone_study_offline.FrameLibrary``.

Design choices taken where the review left one open (user decision 2026-09-26):

* finding 2  — ONE message bus: package C validates and mints the canonical
  envelope id, the SIM scheduler is the only owner of an inbox;
* finding 5  — a failed reply executes NOTHING and its retry is billed;
* finding 8  — an English utterance is delivered, flagged and costed, never
  blocking the recipient's next call;
* finding 18 — the common guidance blocks are token-identical and only the
  channel section differs; the token counts are measured per condition.
"""
from __future__ import annotations

import copy
import json

import pytest

from harness import zone_event_scheduler as ds
from harness import zone_map_schematic as zms
from harness import zone_sim_cost as zc
from harness import zone_study_contract as c
from harness import zone_study_eval as ev
from harness import zone_study_inputs as si
from harness import zone_study_offline as off
from harness import zone_study_prompts_ko as pk
from harness import zone_study_protocol as zp
from harness import zone_study_scenarios as E

SEED = 601                        # s1_normal_mixed's first seed
KO_TEXT = 'order-1은 제가 end_neg 역할로 맡겠습니다.'
EN_TEXT = 'I will take order-1 as end_neg.'


# --------------------------------------------------------------------------- #
# helpers

def _library():
    return off.FrameLibrary()


def _trial(condition='peer_ko', scenario='s1_normal_mixed', seed=SEED, horizon_s=40.):
    return off.run_trial(scenario, condition, seed, horizon_s=horizon_s, library=_library())


def _fresh(condition='peer_ko', scenario='s1_normal_mixed', seed=SEED, horizon_s=40.):
    """A trial that has NOT run: its command history is still empty."""
    return off.OfflineTrial(E.load(scenario), condition=condition, seed=seed,
                            horizon_s=horizon_s, library=_library())


def _inputs(condition='peer_ko', actor='r1'):
    trial = _fresh(condition)
    return trial, trial.build_inputs(actor, sim_time_s=0.0, request_id='req_probe')


def _other_frame(used: bytes) -> tuple[str, bytes]:
    """Some fixture frame whose bytes differ from ``used``."""
    for name, data in off.frames():
        if data != used:
            return name, data
    raise AssertionError('the frame library has only one distinct frame')


# --------------------------------------------------------------------------- #
# Blocker 1 — images bound to validated references

def test_f01_a_foreign_camera_frame_cannot_be_relabelled_as_the_own_wrist_rgb():
    trial, bundled = _inputs()
    _, other = _other_frame(bundled.wrist_jpeg)
    with pytest.raises(zp.ProtocolError, match='validated reference'):
        pk.StudyInputs(payload=bundled.payload_dict(), wrist_jpeg=other, seed=bundled.seed)
    # the real frame still works and is reported with its ref
    again = pk.StudyInputs(payload=bundled.payload_dict(), wrist_jpeg=bundled.wrist_jpeg,
                           seed=bundled.seed)
    manifest = pk.image_manifest(again)
    assert manifest[0]['label'] == pk.IMAGE_OWN
    assert manifest[0]['bytes_sha256'] == manifest[0]['sha256'] == pk.image_sha256(bundled.wrist_jpeg)
    assert manifest[0]['ref'].startswith('own-r1-')


def _schematic_trial(condition='peer_ko'):
    """A trial whose map bundle was rendered WITH its schematic, plus the real PNG."""
    scenario = E.load('s1_normal_mixed')
    bundle = E.bundle_for(scenario, schematic=True)
    data, _ = zms.load_map(scenario['map_id'])
    png, meta = zms.render_schematic(data, width_px=760,
                                     landmark_detail=scenario.get('landmark_detail', 'full'))
    assert meta['png_sha256'] == bundle['schematic']['png_sha256']     # same frozen artefact
    trial = off.OfflineTrial(scenario, condition=condition, seed=SEED, map_bundle=bundle,
                             library=_library(), horizon_s=40.)
    return trial, png


def test_f01_a_map_figure_must_be_the_pinned_schematic_of_the_frozen_map():
    """Second review: the old positive case used a self-made PNG and its OWN
    declared hash, so any bytes with a matching self-declared ref passed."""
    trial, png = _schematic_trial()
    bundled = trial.build_inputs('r1', sim_time_s=0.0, request_id='req_probe')
    payload = bundled.payload_dict()
    assert payload['static_map']['schematic_ref']['png_sha256'] == trial.source.pinned['schematic_png_sha256']
    ok = pk.StudyInputs(payload=payload, wrist_jpeg=bundled.wrist_jpeg, seed=SEED,
                        pinned=trial.source.pinned, map_figure_jpeg=png)
    assert pk.IMAGE_MAP in [row['label'] for row in pk.image_manifest(ok)]
    # the real figure without the pin is refused: a ref alone proves nothing
    with pytest.raises(zp.ProtocolError, match='pinned'):
        pk.StudyInputs(payload=payload, wrist_jpeg=bundled.wrist_jpeg, seed=SEED, map_figure_jpeg=png)
    # wrong bytes for the pinned ref
    with pytest.raises(zp.ProtocolError, match='validated reference'):
        pk.StudyInputs(payload=payload, wrist_jpeg=bundled.wrist_jpeg, seed=SEED,
                       pinned=trial.source.pinned, map_figure_jpeg=bundled.wrist_jpeg)


def test_f01_a_wrist_photo_with_a_self_declared_schematic_ref_is_refused():
    """The reported counterexample: wrist bytes + an arbitrary schematic_ref."""
    trial, bundled = _inputs()                          # a run WITHOUT a schematic
    assert trial.source.pinned['schematic_png_sha256'] is None
    forged = bundled.payload_dict()
    forged['static_map']['schematic_ref'] = {
        'ref': f'map-{forged["static_map"]["map_id"]}-schematic', 'kind': 'map_schematic',
        'png_sha256': pk.image_sha256(bundled.wrist_jpeg)}
    assert any('pinned map schematic' in p
               for p in c.payload_violations(forged, seed=SEED, pinned=trial.source.pinned))
    for pinned in (trial.source.pinned, None):
        with pytest.raises(zp.ProtocolError):
            pk.StudyInputs(payload=forged, wrist_jpeg=bundled.wrist_jpeg, seed=SEED, pinned=pinned,
                           map_figure_jpeg=bundled.wrist_jpeg)
    # and a real schematic of ANOTHER map cannot be relabelled as this one
    wrong = copy.deepcopy(forged)
    wrong['static_map']['schematic_ref']['ref'] = 'map-some_other_map-schematic'
    assert any('is not the schematic of map' in p for p in c.payload_violations(wrong, seed=SEED))


def test_f01_the_whole_final_request_is_hashed_not_only_the_payload():
    trial, bundled = _inputs()
    window = trial.channel.window_context('r1', now_sim_s=0.0)
    request = pk.build_request(bundled, window=window)
    assert request['request_sha256'] and request['request_sha256'] != request['input_sha256']
    # the digest covers the dialogue window, the system text AND the image bytes
    other = pk.build_request(bundled, window={**window, 'max_utterances': window['max_utterances'] + 1})
    assert other['request_sha256'] != request['request_sha256']
    assert other['input_sha256'] == request['input_sha256']
    _, frame = _other_frame(bundled.wrist_jpeg)
    swapped = pk.request_digest(request['messages'][0]['content'], request['messages'][1]['content'],
                                [{'label': pk.IMAGE_OWN, 'image': pk._uri(frame)}])
    assert swapped != request['request_sha256']


# --------------------------------------------------------------------------- #
# Blocker 2 — one owner of the message bus

def test_f02_the_scheduler_is_the_only_owner_of_an_inbox():
    bus = zp.Transport('peer_ko', seed=SEED, delivery_owner=off.BUS_OWNER)
    bus.open_window('w1', at_sim_s=0.)
    receipt = bus.send('r1', recipients=['r2'], text=KO_TEXT, at_sim_s=1.)
    assert receipt.accepted
    # accepted, costed and logged, but NOT yet readable: the bus owner decides when
    assert bus.inbox('r2', now_sim_s=10_000.) == ()
    assert receipt.deliveries == (('r2', None),)
    with pytest.raises(zp.ProtocolError, match='does not own this message bus'):
        bus.commit_delivery(receipt.envelope.message_id, at_sim_s=2., owner='someone_else')
    bus.commit_delivery(receipt.envelope.message_id, at_sim_s=2., owner=off.BUS_OWNER)
    delivered = bus.inbox('r2', now_sim_s=10.)
    assert [row['message_id'] for row in delivered] == [receipt.envelope.message_id]
    assert set(delivered[0]) == set(c.ENVELOPE_KEYS)          # A's closed envelope, no delivery time
    with pytest.raises(zp.ProtocolError, match='already delivered'):
        bus.commit_delivery(receipt.envelope.message_id, at_sim_s=3., owner=off.BUS_OWNER)


def test_f02_one_canonical_message_id_runs_from_validation_to_the_sim_log():
    trial, result = _trial('peer_ko')
    assert result.messages, 'the peer condition must have delivered something'
    envelope_ids = set(trial.envelopes)
    log_ids = {row['message_id'] for row in result.messages}
    edge_ids = {edge.message_id for edge in trial.scheduler.messages}
    inbox_ids = {row['message_id'] for actor in ('r1', 'r2', 'r3')
                 for row in trial.channel.inbox(actor, now_sim_s=10_000.)}
    assert log_ids <= envelope_ids and edge_ids <= envelope_ids and inbox_ids <= envelope_ids
    assert log_ids == edge_ids
    for row in result.messages:                    # C's window-sender-number shape, not call-id-mN
        assert row['message_id'].startswith('w1-')
    # and the inbox a robot reads never carries a delivery time or a broadcast flag
    for actor in ('r1', 'r2', 'r3'):
        for row in trial.channel.inbox(actor, now_sim_s=10_000.):
            assert set(row) == set(c.ENVELOPE_KEYS)
    assert trial.scheduler.bus is trial.channel


def test_f02_the_scheduler_inbox_is_the_same_canonical_envelope_as_the_bus_inbox():
    """Second review: D's inbox lacked recipients/created_at_sim_s and carried
    the id as the body, and the old check here was ``assert ... or True``."""
    trial, result = _trial('peer_ko')
    delivered = 0
    for actor in ('r1', 'r2', 'r3'):
        mine = list(trial.scheduler.inbox(actor))
        assert mine == list(trial.channel.inbox(actor, now_sim_s=10_000.))
        for row in mine:
            delivered += 1
            assert set(row) == set(c.ENVELOPE_KEYS)
            assert actor in row['recipients'] and isinstance(row['created_at_sim_s'], float)
            assert set(row['body']) == {'text'} and row['body']['text'] != row['message_id']
        if not mine:
            continue
        # the D inbox is directly a valid package A input of the recipient
        bundled = trial.build_inputs(actor, sim_time_s=10_000., request_id='req_d_inbox')
        payload = bundled.payload_dict()
        payload['inbox'] = mine
        assert c.payload_violations(payload, seed=SEED, pinned=trial.source.pinned) == []
    assert delivered, 'the peer trial must have delivered at least one envelope'


def test_f02_the_delivery_callback_receives_the_canonical_envelope():
    bus = zp.Transport('peer_ko', seed=SEED, delivery_owner=off.BUS_OWNER)
    bus.open_window('w1', at_sim_s=0.)
    seen = []

    def reply(call):
        receipt = bus.send('r1', recipients=['r2'], text=KO_TEXT, at_sim_s=call.started_sim_s)
        envelope = receipt.envelope
        return ds.CallReply(attempts=(zc.Attempt(utterances=1),), messages=(ds.Message(
            sender='r1', recipients=tuple(envelope.recipients), body=envelope.body(),
            message_id=envelope.message_id),))

    sched = ds.EventScheduler(ds.ReplayTransport({'r1': reply}), bus=bus, bus_owner=off.BUS_OWNER,
                              on_message=lambda a, m, t: seen.append((a, m)),
                              policy=ds.CallPolicy(trigger_on_message=False))
    sched.trigger('r1', 'start')
    sched.run(until_s=50)
    assert [a for a, _ in seen] == ['r2']
    assert seen[0][1] == bus.inbox('r2', now_sim_s=50.)[0] == sched.inbox('r2')[0]
    assert seen[0][1]['body'] == {'text': KO_TEXT} and seen[0][1]['recipients'] == ['r2']
    assert 'delivered_sim_s' not in seen[0][1]                 # delivery time is evaluation data
    assert sched.delivery_log('r2')[0]['message_id'] == seen[0][1]['message_id']


def test_f02_a_scheduler_with_a_bus_refuses_a_second_id_space():
    bus = zp.Transport('peer_ko', seed=SEED, delivery_owner=off.BUS_OWNER)
    bus.open_window('w1', at_sim_s=0.)
    sched = ds.EventScheduler(ds.ReplayTransport({'r1': [ds.CallReply(
        attempts=(zc.Attempt(utterances=1),),
        messages=(ds.Message(sender='r1', recipients=('r2',), body='x'),))]}),
        bus=bus, bus_owner=off.BUS_OWNER)
    sched.trigger('r1', 'start')
    with pytest.raises(ValueError, match='canonical message_id'):
        sched.run(until_s=50)
    with pytest.raises(ValueError, match='delivery_owner'):
        ds.EventScheduler(ds.ReplayTransport({}), bus=zp.Transport('peer_ko', seed=SEED))


# --------------------------------------------------------------------------- #
# High 3 — the closed schema closes types and the deep structure

def test_f03_a_coordinate_cannot_hide_in_a_closed_schema_value():
    trial, bundled = _inputs()
    payload = bundled.payload_dict()
    payload['order_sheet']['orders'][0]['initial_location']['slot'] = [0.4, -2.45]
    problems = c.payload_violations(payload, seed=SEED)
    assert any('initial_location.slot' in p for p in problems), problems
    with pytest.raises(zp.ProtocolError):
        pk.StudyInputs(payload=payload, wrist_jpeg=bundled.wrist_jpeg, seed=SEED)


def test_f03_the_map_interior_is_closed_even_when_the_hash_is_recomputed():
    trial, bundled = _inputs()
    payload = bundled.payload_dict()
    payload['static_map']['public_map']['walls'][0]['survey_xy_m'] = [1.0, 2.0]
    payload['static_map']['public_map_sha256'] = c.digest(payload['static_map']['public_map'])
    problems = c.payload_violations(payload, seed=SEED)
    assert any('walls[] carries key(s) outside the contract' in p for p in problems), problems


def test_f03_the_payload_is_compared_with_the_frozen_order_sheet():
    trial, bundled = _inputs()
    payload = bundled.payload_dict()
    payload['order_sheet']['orders'][0]['destination_zone'] = 'C'
    assert c.payload_violations(payload, seed=SEED) == []          # self-consistent
    assert any('pinned frozen order sheet' in p
               for p in c.payload_violations(payload, seed=SEED, pinned=trial.source.pinned))
    with pytest.raises(zp.ProtocolError):
        pk.StudyInputs(payload=payload, wrist_jpeg=bundled.wrist_jpeg, seed=SEED,
                       pinned=trial.source.pinned)


# --------------------------------------------------------------------------- #
# High 4 — the validated input is deeply immutable

def test_f04_a_payload_mutated_after_validation_cannot_reach_the_model():
    trial, bundled = _inputs()
    with pytest.raises(TypeError):
        bundled.payload['teacher_receipt'] = {'done': True}
    with pytest.raises(TypeError):                         # nested, not only the top level
        bundled.payload['order_sheet']['orders'][0]['count'] = 99
    window = trial.channel.window_context('r1', now_sim_s=0.0)
    body = pk.build_request(bundled, window=window)['messages'][1]['content']
    assert 'teacher_receipt' not in body
    # a caller that edits the copy it was handed changes nothing
    leaked = bundled.payload_dict()
    leaked['teacher_receipt'] = {'done': True}
    assert 'teacher_receipt' not in pk.build_request(bundled, window=window)['messages'][1]['content']
    assert bundled.payload_sha256 == si.payload_sha256(bundled.payload_dict())


# --------------------------------------------------------------------------- #
# High 5 — a failed reply pays but executes nothing

def test_f05_a_failed_reply_executes_nothing_and_its_retry_is_billed():
    actions, deliveries = [], []
    bad = ds.CallReply(attempts=(zc.Attempt(outcome='invalid', output_tokens=40, utterances=1),),
                       action='go A',
                       messages=(ds.Message(sender='r1', recipients=('r2',), body='보고',
                                            message_id='m-bad'),))
    good = ds.CallReply(attempts=(zc.Attempt(outcome='ok', output_tokens=40),), action='go B')
    sched = ds.EventScheduler(ds.ReplayTransport({'r1': [bad, good]}),
                              on_action=lambda a, action, t: actions.append((a, action)),
                              on_message=lambda a, m, t: deliveries.append(m['message_id']))
    sched.trigger('r1', 'start')
    sched.run(until_s=100)
    assert actions == [('r1', 'go B')]                 # the invalid decision never ran
    assert deliveries == []                            # and its utterance was never delivered
    assert [row['call_id'] for row in sched.discarded] == [sched.calls[0].call_id]
    assert sched.discarded[0]['action'] == 'go A' and sched.discarded[0]['messages'] == 1
    # the failed attempt and the retry are both charged
    assert sched.calls[0].cost.outcome == 'invalid' and sched.calls[0].cost.sim_s > 0
    assert sched.calls[1].retry_of == sched.calls[0].call_id and sched.calls[1].cost.sim_s > 0
    assert sched.metrics['r1']['retries'] == 1
    assert sched.budget.used_total() == 2               # both attempts consumed the HTTP budget


# --------------------------------------------------------------------------- #
# High 6 — billed utterances equal produced utterances

def test_f06_a_message_cannot_be_free_and_a_billed_utterance_cannot_be_silent():
    with pytest.raises(ValueError, match='review finding 6'):
        ds.CallReply(attempts=(zc.Attempt(),),
                     messages=(ds.Message(sender='r1', recipients=('r2',), body='보고'),))
    with pytest.raises(ValueError, match='review finding 6'):
        ds.CallReply(attempts=(zc.Attempt(utterances=1),))
    reply = ds.CallReply(attempts=(zc.Attempt(utterances=1),),
                         messages=(ds.Message(sender='r1', recipients=('r2',), body='보고',
                                              message_id='m-1'),))
    cost = zc.call_cost(reply.attempts)
    assert cost.breakdown['utterances'] == 1 and cost.breakdown['utterance_s'] > 0


def test_f06_a_rejected_utterance_is_still_billed_but_never_delivered():
    rejected = ds.Message(sender='r1', recipients=('r2',), body=None, message_id='m-rej',
                          rejection='window_cap')
    reply = ds.CallReply(attempts=(zc.Attempt(utterances=1),), messages=(rejected,))
    sched = ds.EventScheduler(ds.ReplayTransport({'r1': [reply]}))
    sched.trigger('r1', 'start')
    sched.run(until_s=100)
    assert sched.messages == [] and sched.inbox('r2') == ()
    assert [row['rejection'] for row in sched.rejected_messages] == ['window_cap']
    assert sched.metrics['r1']['utterances'] == 1        # generated, therefore charged
    assert sched.calls[0].cost.breakdown['utterance_s'] > 0


def test_f06_the_offline_loop_bills_exactly_what_the_replies_produced():
    trial, result = _trial('peer_ko')
    checks = off.cost_checks(trial, result)
    assert checks['ok'], checks['problems']
    assert checks['billed_utterances'] == checks['produced_utterances'] == len(result.messages)


# --------------------------------------------------------------------------- #
# High 7 — one item vocabulary for A and C

def test_f07_an_item_kind_a_allows_is_accepted_by_the_structured_validator():
    scenario = E.load('s5_moved_dropped_item')
    bundle = E.bundle_for(scenario)
    source = si.OrderSheetSource(scenario, bundle)
    sheet = source.sheet()
    vocab = si.vocabulary(sheet, bundle['public_map'])
    fungible = next(o for o in sheet['orders'] if o['identity'] == 'kind_fungible')
    kind = fungible['kind']
    assert kind in vocab.items                                      # A allows the kind
    body = {'act': 'inform', 'item': kind, 'zone': fungible['destination_zone'], 'role': None,
            'passage': None, 'location_ref': None, 'state': 'held', 'confidence': 'high',
            'observed_at_sim_s': 1.0, 'reply_to': None}
    c.check_message('structured', 'r1', ['r2'], body, vocabulary=vocab)   # A accepts
    reply = {'request_id': 'req_1', 'action': {'kind': 'continue'},
             'decision_sources': ['own_rgb'],
             'messages': [{'recipients': ['r2'], 'message': body, 'reply_to': None}]}
    # with A's Vocabulary, C accepts the same message
    checked = zp.validate_reply(reply, request_id='req_1', condition='structured', actor='r1',
                                order_ids=[o['order_id'] for o in sheet['orders']],
                                item_ids=[i for o in sheet['orders'] for i in o['item_ids']],
                                vocabulary=vocab)
    assert checked['messages'][0]['message']['item'] == kind
    # without it, the kind is unknown: that was the reported mismatch
    with pytest.raises(zp.ProtocolError, match='run vocabulary'):
        zp.validate_reply(reply, request_id='req_1', condition='structured', actor='r1',
                          order_ids=[o['order_id'] for o in sheet['orders']],
                          item_ids=[i for o in sheet['orders'] for i in o['item_ids']])


def test_f07_every_scenario_order_can_be_named_in_a_structured_message():
    for scenario_id in E.scenario_ids():
        scenario = E.load(scenario_id)
        bundle = E.bundle_for(scenario)
        sheet = si.OrderSheetSource(scenario, bundle).sheet()
        vocab = si.vocabulary(sheet, bundle['public_map'])
        for order in sheet['orders']:
            name = order['item_ids'][0] if order['item_ids'] else order['kind']
            body = {'act': 'inform', 'item': name, 'zone': order['destination_zone'],
                    'role': (order.get('required_robots') or 1) and
                            (sheet['kinds'][order['kind']]['roles'][0]),
                    'passage': None, 'location_ref': None, 'state': 'placed',
                    'confidence': 'high', 'observed_at_sim_s': 2.0, 'reply_to': None}
            c.check_message('structured', 'r1', ['r2'], body, vocabulary=vocab)
            zp.validate_reply({'request_id': 'r', 'action': {'kind': 'continue'},
                               'decision_sources': ['own_rgb'],
                               'messages': [{'recipients': ['r2'], 'message': body,
                                             'reply_to': None}]},
                              request_id='r', condition='structured', actor='r1',
                              vocabulary=vocab)


# --------------------------------------------------------------------------- #
# High 8 — an English utterance is delivered, flagged, non-blocking

def test_f08_an_english_utterance_is_delivered_flagged_and_does_not_block_the_recipient():
    bus = zp.Transport('peer_ko', seed=SEED, delivery_owner=off.BUS_OWNER)
    bus.open_window('w1', at_sim_s=0.)
    receipt = bus.send('r1', recipients=['r2'], text=EN_TEXT, at_sim_s=1.)
    assert receipt.accepted and receipt.language_violation
    assert [row['message_id'] for row in bus.language_flags] == [receipt.envelope.message_id]
    bus.commit_delivery(receipt.envelope.message_id, at_sim_s=2., owner=off.BUS_OWNER)
    inbox = list(bus.inbox('r2', now_sim_s=5.))
    assert [row['body']['text'] for row in inbox] == [EN_TEXT]
    # the RECIPIENT's next payload builds: the sender's slip is not the receiver's failure
    trial, bundled = _inputs()
    payload = bundled.payload_dict()
    payload['inbox'] = inbox
    payload['sim_time_s'] = 5.0
    payload['robot_id'] = 'r2'
    payload['channel'] = c.channel_section('peer_ko', 'r2', SEED)
    payload['own_rgb_refs'] = [dict(row, ref=row['ref'].replace('own-r1-', 'own-r2-'))
                               for row in payload['own_rgb_refs']]
    assert c.payload_violations(payload, seed=SEED) == []
    assert c.language_violations('peer_ko', inbox[0]['body']) == \
        ['free message body has no Korean text']


# --------------------------------------------------------------------------- #
# High 9 — the leader rotation cannot be bypassed

def test_f09_an_explicit_leader_cannot_bypass_the_seed_rotation():
    assert zp.leader_for_seed(11) == 'r3'
    assert zp.leader_of('leader_ko', seed=11) == 'r3'
    with pytest.raises(zp.ProtocolError, match='rotation'):
        zp.leader_of('leader_ko', seed=11, leader='r1')
    with pytest.raises(zp.ProtocolError, match='pass the seed'):
        zp.leader_of('leader_ko', leader='r1')
    # a labelled diagnostic may still do it, and says so
    assert zp.leader_of('leader_ko', seed=11, leader='r1', allow_override=True) == 'r1'
    for helper in (zp.allowed_edges, zp.role_of):
        with pytest.raises(zp.ProtocolError, match='rotation'):
            helper('leader_ko', 'r1', seed=11, leader='r1') if helper is zp.role_of \
                else helper('leader_ko', seed=11, leader='r1')


def test_f09_payload_prompt_and_transport_must_name_the_same_leader():
    trial, bundled = _inputs('leader_ko')
    payload = bundled.payload_dict()
    leader = payload['leader_id']
    assert leader == zp.leader_for_seed(SEED) == trial.channel.leader
    window = trial.channel.window_context(bundled.robot_id, now_sim_s=0.0)
    other = next(r for r in zp.ROBOTS if r != leader)
    with pytest.raises(zp.ProtocolError, match='rotation'):
        pk.build_request(bundled, leader=other, window=window)
    with pytest.raises(zp.ProtocolError, match='payload names leader'):
        pk.build_request(bundled, leader=other, window=window, allow_leader_override=True)
    assert pk.build_request(bundled, leader=leader, window=window)['prompt_role'] in \
        ('leader', 'follower')


# --------------------------------------------------------------------------- #
# High 10 — calls/messages are the only cost aggregation source

def test_f10_a_standard_call_log_is_never_reported_as_zero_cost():
    trial, result = _trial('peer_ko')
    record = trial.trial_record(result)
    assert record['calls'], 'the record must carry package A call rows'
    stripped = copy.deepcopy(record)
    stripped.pop('model')
    metrics = ev.efficiency_metrics(ev.parse_trial(stripped))
    assert metrics['model_calls'] == len(record['calls'])
    assert metrics['think_sim_cost_s'] > 0 and metrics['tokens_input'] > 0
    assert metrics['model_cost_source'] == 'calls'
    # a separate summary must agree with the log
    wrong = copy.deepcopy(record)
    wrong['model']['logical_calls'] = 0
    with pytest.raises(ev.TrialError, match='review finding 10'):
        ev.efficiency_metrics(ev.parse_trial(wrong))
    assert ev.efficiency_metrics(ev.parse_trial(copy.deepcopy(record)))['model_cost_source'] == \
        'calls+summary'


def test_f10_a_missing_cost_source_stays_missing_instead_of_zero():
    assert ev.model_aggregate({'condition': 'no_comm'}) is None


# --------------------------------------------------------------------------- #
# High 11 — delivery aggregation keeps recoveries and fungible orders

def _delivery_trial(**kw):
    """Minimal provisional trial record for the delivery aggregation."""
    base = {'schema': ev.PROVISIONAL_SCHEMA, 'trial_id': 'peer_ko-mixed-s601',
            'condition': 'peer_ko', 'scenario': 'mixed', 'seed': 601,
            'robots': ['r1', 'r2', 'r3'], 'end_reason': 'orders_complete',
            't0_sim_s': 0.0, 'end_sim_s': 500.0,
            'budget': {'sim_horizon_s': 900.0},
            'orders': [{'order_id': 'order-1', 'kind': 'cyan', 'count': 1, 'item_ids': ['cyan_1'],
                        'identity': 'specific_item', 'destination_zone': 'A'},
                       {'order_id': 'order-2', 'kind': 'red', 'count': 2, 'item_ids': [],
                        'identity': 'kind_fungible', 'destination_zone': 'B'}],
            'referee': {'deliveries': [], 'conflicts': [], 'deadlocks': []}}
    base['referee']['deliveries'] = kw.pop('deliveries', [])
    base.update(kw)
    return ev.parse_trial(base)


def test_f11_a_corrected_misdelivery_counts_as_delivered():
    trial = _delivery_trial(deliveries=[
        {'item_id': 'cyan_1', 'zone': 'B', 'sim_s': 100.0},      # wrong zone
        {'item_id': 'cyan_1', 'zone': 'A', 'sim_s': 200.0}])     # corrected
    state = ev.delivery_state(trial)
    assert set(state['delivered']) == {'cyan_1'} and state['misdelivered'] == {}
    metrics = ev.efficiency_metrics(trial)
    assert metrics['delivered_items'] == 1 and metrics['misdelivered_items'] == 0
    assert metrics['misdeliveries_recovered'] == 1          # the mistake stays visible


def test_f11_a_fungible_order_is_matched_by_kind_not_by_a_missing_item_id():
    trial = _delivery_trial(deliveries=[
        {'item_id': 'red_1', 'zone': 'B', 'sim_s': 100.0},
        {'item_id': 'red_2', 'zone': 'B', 'sim_s': 150.0}])
    state = ev.delivery_state(trial)
    assert set(state['delivered']) == {'red_1', 'red_2'}
    assert state['misdelivered'] == {} and state['surplus'] == []
    assert state['by_order']['order-2'] == {'ordered': 2, 'delivered': 2, 'complete': True}
    wrong_zone = _delivery_trial(deliveries=[{'item_id': 'red_1', 'zone': 'C', 'sim_s': 100.0}])
    assert set(ev.delivery_state(wrong_zone)['misdelivered']) == {'red_1'}


def test_f11_an_unordered_item_and_a_delivery_after_the_end_are_not_deliveries():
    trial = _delivery_trial(deliveries=[
        {'item_id': 'tile_9', 'zone': 'A', 'sim_s': 100.0, 'correct': True},   # not ordered
        {'item_id': 'cyan_1', 'zone': 'A', 'sim_s': 600.0, 'correct': True}])  # after end_sim_s
    state = ev.delivery_state(trial)
    assert state['delivered'] == {} and state['surplus'] == ['tile_9']
    assert len(state['outside_window']) == 1
    assert ev.efficiency_metrics(trial)['deliveries_outside_window'] == 1


# --------------------------------------------------------------------------- #
# High 12 — the same proposition gets the same verdict in both encodings

_REFEREE = {'deliveries': [{'item_id': 'cyan_1', 'zone': 'A', 'sim_s': 100.0}],
            'holds': [{'item_id': 'beam_1', 'robot': 'r2', 'from_s': 0.0, 'to_s': 300.0}],
            'conflicts': [], 'deadlocks': []}
_LABELS = ('order-1', 'order-2', 'cyan_1', 'beam_1', 'A', 'B', 'C', 'r1', 'r2', 'r3')


def test_f12_a_korean_delivery_claim_about_a_real_id_is_checkable():
    trial = _delivery_trial(referee=_REFEREE)
    utterance = {'message_id': 'm-1', 'sender': 'r1', 'encoding': 'free_ko',
                 'text': 'cyan_1을 A에 내려놓았습니다.', 'sim_s': 150.0}
    claims = ev.extract_claims(utterance, _LABELS)
    assert claims == [{'type': 'delivered', 'item_id': 'cyan_1', 'zone': 'A'}]
    assert ev.check_claim(claims[0], trial, 150.0) == 'true'
    false_claim = ev.extract_claims({**utterance, 'text': 'cyan_1을 B에 내려놓았습니다.'}, _LABELS)[0]
    assert ev.check_claim(false_claim, trial, 150.0) == 'false'


def test_f12_the_same_claim_is_judged_the_same_in_free_text_and_in_the_schema():
    trial = _delivery_trial(referee=_REFEREE)
    free = {'message_id': 'm-1', 'sender': 'r1', 'encoding': 'free_ko', 'sim_s': 150.0,
            'text': 'cyan_1을 A에 내려놓았습니다.'}
    structured = {'message_id': 'm-2', 'sender': 'r1', 'encoding': 'schema', 'sim_s': 150.0,
                  'message': {'act': 'inform', 'item': 'cyan_1', 'zone': 'A', 'state': 'placed'}}
    verdicts = {row['message_id']: [ev.check_claim(claim, trial, row['sim_s'])
                                    for claim in ev.extract_claims(row, _LABELS)]
                for row in (free, structured)}
    assert verdicts == {'m-1': ['true'], 'm-2': ['true']}


def test_f12_a_structured_hold_claim_uses_the_envelope_sender():
    trial = _delivery_trial(referee=_REFEREE)
    # r1 claims to hold beam_1, but the referee says r2 holds it
    claim = ev.extract_claims({'message_id': 'm-3', 'sender': 'r1', 'encoding': 'schema',
                               'message': {'act': 'inform', 'item': 'beam_1', 'state': 'held'}},
                              _LABELS)[0]
    assert claim == {'type': 'holding', 'item_id': 'beam_1', 'robot': 'r1'}
    assert ev.check_claim(claim, trial, 100.0) == 'false'
    truthful = ev.extract_claims({'message_id': 'm-4', 'sender': 'r2', 'encoding': 'schema',
                                 'message': {'act': 'inform', 'item': 'beam_1', 'state': 'held'}},
                                _LABELS)[0]
    assert ev.check_claim(truthful, trial, 100.0) == 'true'
    # the free-text twin agrees
    free = ev.extract_claims({'message_id': 'm-5', 'sender': 'r1', 'encoding': 'free_ko',
                              'text': 'beam_1을 제가 들고 있습니다.'}, _LABELS)[0]
    assert ev.check_claim(free, trial, 100.0) == 'false'


# --------------------------------------------------------------------------- #
# Medium 13 — an allowed leader broadcast is not a violation

def _leader_trial(seed, leader, utterances):
    return ev.parse_trial({
        'schema': ev.PROVISIONAL_SCHEMA, 'trial_id': f'leader_ko-mixed-s{seed}',
        'condition': 'leader_ko', 'scenario': 'mixed', 'seed': seed, 'leader_id': leader,
        'robots': ['r1', 'r2', 'r3'], 'end_reason': 'orders_complete',
        'end_sim_s': 500.0, 'budget': {'sim_horizon_s': 900.0},
        'orders': [{'order_id': 'order-1', 'kind': 'cyan', 'count': 1, 'item_ids': ['cyan_1'],
                    'identity': 'specific_item', 'destination_zone': 'A'}],
        'referee': {'deliveries': [], 'conflicts': [], 'deadlocks': []},
        'utterances': list(utterances)})


def _utter(message_id, sender, recipients, text):
    return {'message_id': message_id, 'sender': sender, 'recipients': list(recipients),
            'encoding': 'free_ko', 'text': text, 'sim_s': 10.0}


def test_f13_a_leader_broadcast_to_every_follower_is_allowed():
    seed = 12                                   # 12 % 3 == 0 -> r1
    assert zp.leader_for_seed(seed) == 'r1'
    trial = _leader_trial(seed, 'r1', [_utter('m-1', 'r1', ('r2', 'r3'), '모두 대기하십시오.')])
    report = ev.channel_compliance(trial)
    assert report['violations'] == []
    assert ('r1', 'r2') in set(map(tuple, report['edges']))
    # every recipient edge of the broadcast is an edge the contract allows
    edges = c.allowed_edges('leader_ko', seed)
    assert {('r1', 'r2'), ('r1', 'r3')} <= edges
    assert ev.audit_input_boundary(trial)['channel_violations'] == []


def test_f13_a_follower_to_follower_message_is_still_a_violation():
    trial = _leader_trial(12, 'r1', [_utter('m-1', 'r2', ('r3',), 'r3, 같이 갑시다.')])
    assert [v['kind'] for v in ev.channel_compliance(trial)['violations']] == ['follower_to_follower']


# --------------------------------------------------------------------------- #
# Medium 14 — every audit failure is aggregated the same way

def _audited_trial(requests):
    return ev.parse_trial({
        'schema': ev.PROVISIONAL_SCHEMA, 'trial_id': 'peer_ko-mixed-s601',
        'condition': 'peer_ko', 'scenario': 'mixed', 'seed': 601,
        'robots': ['r1', 'r2', 'r3'], 'end_reason': 'orders_complete',
        'end_sim_s': 500.0, 'budget': {'sim_horizon_s': 900.0},
        'orders': [{'order_id': 'order-1', 'kind': 'cyan', 'count': 1, 'item_ids': ['cyan_1'],
                    'identity': 'specific_item', 'destination_zone': 'A'}],
        'referee': {'deliveries': [{'item_id': 'cyan_1', 'zone': 'A', 'sim_s': 100.0}],
                    'conflicts': [], 'deadlocks': []},
        'requests': list(requests), 'utterances': []})


def test_f14_an_unvalidated_payload_is_counted_as_a_violation_everywhere():
    trial = _audited_trial([{'request_id': 'req_1', 'robot': 'r1', 'sim_s': 1.0,
                             'payload_validated': False, 'status': 'input_rejected'}])
    boundary = ev.audit_input_boundary(trial)
    assert boundary['clean'] is False
    assert boundary['unvalidated_payloads']
    assert ev.boundary_status(boundary) == 'violation'
    assert ev.boundary_failures(boundary) == {'unvalidated_payloads': 1}
    summary = ev.summarise([trial])
    row = summary['conditions']['peer_ko']
    assert row['boundary_violation_trials'] == 1 and row['boundary_clean_trials'] == 0
    assert row['boundary_failures'] == {'unvalidated_payloads': 1}
    assert row['boundary_status_counts'] == {'violation': 1}


def test_f14_an_unknown_input_key_is_counted_as_a_violation_everywhere():
    trial = _audited_trial([{'request_id': 'req_1', 'robot': 'r1', 'sim_s': 1.0,
                             'payload_validated': True, 'status': 'ok',
                             'input_keys': ['static_map', 'order_sheet', 'mystery_board']}])
    boundary = ev.audit_input_boundary(trial)
    assert ev.boundary_status(boundary) == 'violation'
    assert ev.boundary_failures(boundary) == {'unknown_input_keys': 1}
    row = ev.summarise([trial])['conditions']['peer_ko']
    assert row['boundary_violation_trials'] == 1


def test_f14_a_clean_trial_is_clean_in_every_consumer():
    trial = _audited_trial([{'request_id': 'req_1', 'robot': 'r1', 'sim_s': 1.0,
                             'payload_validated': True, 'status': 'ok'}])
    boundary = ev.audit_input_boundary(trial)
    assert ev.boundary_status(boundary) == 'clean' and ev.boundary_failures(boundary) == {}
    row = ev.summarise([trial])['conditions']['peer_ko']
    assert row['boundary_clean_trials'] == 1 and row['boundary_violation_trials'] == 0


# --------------------------------------------------------------------------- #
# Medium 15 — the HTTP budget is reserved before the request

def test_f15_three_simultaneous_actors_cannot_share_the_last_attempt():
    policy = ds.CallPolicy(max_attempts_total=1, max_http_attempts_per_actor=30, min_interval_s=0.)
    reply = ds.CallReply(attempts=(zc.Attempt(),))
    sched = ds.EventScheduler(ds.ReplayTransport({a: [reply] for a in ds.DEFAULT_ACTORS}),
                              policy=policy)
    for actor in ds.DEFAULT_ACTORS:
        sched.trigger(actor, 'start')
    sched.run(until_s=100)
    # exactly one call may start, and the other two are refused BEFORE the request
    assert len(sched.calls) == 1
    assert sum(sched.metrics[a]['budget_refused'] for a in ds.DEFAULT_ACTORS) == 2
    assert sched.budget.used_total() == 1
    assert len(sched.budget.refusals) == 2


def test_f15_an_actor_has_its_own_http_attempt_cap():
    policy = ds.CallPolicy(max_http_attempts_per_actor=2, max_attempts_total=90, min_interval_s=0.,
                           max_calls_per_actor=30, idle_reask_s=1.)
    reply = ds.CallReply(attempts=(zc.Attempt(),))
    sched = ds.EventScheduler(ds.ReplayTransport(lambda call: reply), policy=policy)
    for _ in range(5):
        sched.trigger('r1', 'start')
        sched.run(until_s=100)
    assert sched.budget.used['r1'] == 2
    assert sched.metrics['r1']['budget_refused'] >= 1


def test_f15_a_compliant_transport_reserves_every_retry_before_sending_it():
    """Second review: the old test EXPECTED 3 attempts under a cap of 1 and
    froze a wrong ``over=1``. A compliant transport reserves each internal retry
    through ``PendingCall.reserve``; a refused reservation is never sent."""
    greedy = ds.CallReply(attempts=(zc.Attempt(outcome='error'), zc.Attempt(outcome='error'),
                                    zc.Attempt(outcome='ok')), action='go')
    actions = []
    policy = ds.CallPolicy(max_attempts_total=1, max_http_attempts_per_actor=1, max_retries=1)
    sched = ds.EventScheduler(ds.ReplayTransport({'r1': [greedy]}), policy=policy,
                              on_action=lambda a, action, t: actions.append(action))
    sched.trigger('r1', 'start')
    sched.run(until_s=200)
    assert sched.budget.used_total() == 1 and sched.over_budget_attempts == []
    assert [len(call.cost.attempts) for call in sched.calls] == [1]     # the retries never left
    assert sched.calls[0].cost.outcome == 'error' and actions == []
    assert sched.metrics['r1']['budget_refused'] == 1                   # the scheduler retry too
    assert len(sched.budget.refusals) == 2


class _IgnoresTheBudget:
    """A NON-compliant transport: retries internally without reserving."""

    def __init__(self, reply):
        self.reply_value = reply

    def submit(self, call):
        return call

    def reply(self, token):
        return self.reply_value


def test_f15_an_unreserved_retry_is_counted_exactly_and_executes_nothing():
    greedy = ds.CallReply(attempts=(zc.Attempt(outcome='error'), zc.Attempt(outcome='error'),
                                    zc.Attempt(outcome='ok')), action='go')
    actions = []
    policy = ds.CallPolicy(max_attempts_total=1, max_http_attempts_per_actor=1, max_retries=1)
    sched = ds.EventScheduler(_IgnoresTheBudget(greedy), policy=policy,
                              on_action=lambda a, action, t: actions.append(action))
    sched.trigger('r1', 'start')
    sched.run(until_s=200)
    assert sched.over_budget_attempts == [{'call_id': sched.calls[0].call_id, 'actor': 'r1',
                                          'reserved': 1, 'actual': 3, 'unreserved': 2, 'over': 2}]
    assert actions == []                                      # the breach is not rewarded
    assert sched.discarded[0]['reason'] == 'budget_breach'
    assert len(sched.calls) == 1                              # and it is not retried
    budget = ds.AttemptBudget(per_actor=1, total=1)
    assert budget.reserve('r1', 1) is True
    assert budget.commit('r1', reserved=1, actual=3) == 2


def test_f15_the_offline_loop_keeps_the_attempt_budget_inside_its_cap():
    trial, result = _trial('peer_ko')
    budget = result.cost['attempt_budget']
    assert budget['total'] == trial.policy.max_attempts_total
    assert budget['per_actor'] == trial.policy.max_http_attempts_per_actor
    assert budget['used_total'] <= budget['total']
    assert all(count <= budget['per_actor'] for count in budget['used'].values())
    assert off.cost_checks(trial, result)['ok']


# --------------------------------------------------------------------------- #
# Medium 16 — unfinished calls are censored, never dropped

def test_f16_a_call_that_does_not_finish_before_the_horizon_stays_in_the_ledger():
    slow = ds.CallReply(attempts=(zc.Attempt(input_tokens=123, output_tokens=1000),))
    sched = ds.EventScheduler(ds.ReplayTransport({'r1': [slow]}))
    sched.trigger('r1', 'start')
    report = sched.run(until_s=2.0)          # the call needs far more than 2 SIM seconds
    assert sched.calls == [] and sched.censored, 'the call must survive as censored'
    assert report.censored_calls == 1
    row = sched.censored[0]
    assert row['actor'] == 'r1' and row['elapsed_sim_s'] == 2.0
    assert sched.ledger[row['call_id']]['status'] == 'censored'
    log = sched.contract_log(run_id='run', condition_name='peer_ko', seed=SEED,
                             provenance=_provenance())
    assert [record['status'] for record in log['calls']] == ['censored']
    record = log['calls'][0]
    assert record['sim_cost_s'] == 2.0                      # SIM time elapsed, action not released
    # second review: the API call happened, so its KNOWN usage is kept (was 0)
    assert record['input_tokens']['text'] == 123 and record['output_tokens'] == 1000
    assert record['cost_terms']['censored'] is True and record['cost_terms']['usage_known'] is True
    assert record['cost_terms']['would_release_sim_s'] > 2.0
    assert record['http_attempts'] == 1 and sched.budget.used_total() == 1
    c.validate_log_record(record)
    # a censored call can never complete later, even when the loop resumes
    sched.run(until_s=100)
    assert sched.calls == [] and len(sched.censored) == 1


def test_f16_a_reply_not_yet_fetched_at_the_horizon_is_fetched_for_its_usage():
    slow = ds.CallReply(attempts=(zc.Attempt(input_tokens=77, output_tokens=500),), action='go')
    actions = []
    sched = ds.EventScheduler(ds.ReplayTransport({'r1': [slow]}),
                              on_action=lambda a, action, t: actions.append(action))
    sched.trigger('r1', 'start')
    assert sched.params.min_call_s() == .5
    sched.arm_observations(('r1',), period_s=.2)            # keeps the loop short of min_call_s
    sched.run(until_s=.3)                                    # next tick .4 < .5: not fetched yet
    row = sched.censored[0]
    assert row['reason'] == 'pending' and row['usage_known'] is True
    assert (row['input_tokens'], row['output_tokens']) == (77, 500)
    assert actions == []                                     # fetched for accounting, never executed


def _provenance():
    return {'registry_sha256': c.registry_sha256(), 'order_sheet_sha256': 'a' * 64,
            'map_file_sha256': 'b' * 64, 'public_map_sha256': 'c' * 64, 'code_sha': 'deadbeef',
            'execution_bundle_id': 'zone_study_offline_v1', 'model': 'none-fixture-v1',
            'provider': None, 'model_settings_sha256': None, 'prompt_template_sha256': 'd' * 64,
            'cost_profile_id': 'zone_sim_cost.v1', 'input_profile_id': 'zone_study_inputs.v1'}


def test_f16_the_offline_record_separates_elapsed_sim_time_from_api_resources():
    trial, result = _trial('peer_ko', horizon_s=42.)
    censored = [row for row in result.calls if row['status'] == 'censored']
    assert censored, 'a 42 s horizon must leave calls in flight'
    assert result.cost['censored_calls'] == len(censored)
    assert result.cost['censored_elapsed_sim_s'] > 0
    assert all(row['cost_terms']['usage_known'] and row['output_tokens'] > 0
               and row['input_tokens']['text'] > 0 for row in censored)
    # no action row and no command of a censored call
    censored_ids = {row['request_id'] for row in censored}
    assert not censored_ids & {a['request_id'] for a in result.actions}
    record = trial.trial_record(result)
    assert record['model']['censored_calls'] == len(censored)
    derived = ev.model_aggregate(ev.parse_trial(copy.deepcopy(record)))
    assert derived['censored_calls'] == len(censored) and derived['mismatch'] == []
    assert derived['completed_calls'] == len(record['calls']) - len(censored)
    # the API totals include the censored calls and match the attempt budget
    assert derived['http_attempts'] == result.cost['attempt_budget']['used_total']
    assert derived['tokens']['output'] == sum(row['output_tokens'] for row in result.calls)
    assert off.cost_checks(trial, result)['ok']


# --------------------------------------------------------------------------- #
# Medium 17 — the public scenario metadata hides the hidden event

def test_f17_no_public_scenario_field_names_the_hidden_event_or_its_solution():
    for scenario_id in E.scenario_ids():
        scenario = E.load(scenario_id)
        public, private = E.public_part(scenario), E.private_part(scenario)
        assert 'notes' not in scenario and 'notes' not in public, scenario_id
        assert private['design_notes_ko'], scenario_id
        blob = repr(public)
        # the order sheet legitimately names the ordered items; the EVENT KIND,
        # its SIM time and the design note are what used to leak through notes
        for event in private.get('hidden_events') or ():
            assert str(event.get('kind')) not in blob, (scenario_id, event.get('kind'))
            at = ((event.get('trigger') or {}).get('at_sim_s'))
            if at is not None:
                assert str(at) not in blob, (scenario_id, at)
        for sentence in private['design_notes_ko'].split('. '):
            assert sentence.strip() not in blob, (scenario_id, sentence)


def test_f17_the_order_sheet_names_the_scenario_only_by_an_opaque_ref():
    for scenario_id in E.scenario_ids():
        source = si.OrderSheetSource(E.load(scenario_id), E.bundle_for(E.load(scenario_id)))
        sheet = source.sheet()
        assert sheet['scenario_id'] == c.scenario_ref(scenario_id)
        assert c.SCENARIO_REF.match(sheet['scenario_id'])
        blob = repr(sheet)
        suffix = scenario_id.split('_', 1)[1]           # e.g. 'moved_dropped_item'
        assert scenario_id not in blob and suffix not in blob, (scenario_id, suffix)
        assert source.manifest()['scenario_id'] == scenario_id      # evaluation side keeps the name


def test_f17_a_descriptive_scenario_id_in_the_order_sheet_is_refused():
    trial, bundled = _inputs()
    payload = bundled.payload_dict()
    payload['order_sheet']['scenario_id'] = 's5_moved_dropped_item'
    assert any('opaque scenario_ref' in p for p in c.payload_violations(payload, seed=SEED))


# --------------------------------------------------------------------------- #
# Medium 18 — the fixed prompt is symmetric and measured

def test_f18_the_common_guidance_blocks_are_token_identical_across_conditions():
    report = pk.prompt_token_report(seed=12)
    rows = [row for key, row in report['rows'].items()
            if row['condition'] in c.MAIN_CONDITIONS]
    # the head carries the identity/role sentence, which is a ROLE difference,
    # not a guidance difference; every other common block must be identical
    assert len({row['common_tokens'] - row['blocks']['head'] for row in rows}) == 1, \
        {key: row['common_tokens'] for key, row in report['rows'].items()}
    for block in pk.COMMON_BLOCKS:
        if block == 'head':
            continue
        assert len({row['blocks'][block] for row in rows}) == 1, block
    # and the whole difference between two conditions IS the channel section
    for left in rows:
        for right in rows:
            if left['role'] != right['role']:
                continue
            assert (left['tokens'] - right['tokens']
                    == left['channel_tokens'] - right['channel_tokens']), (left, right)


def test_f18_only_the_channel_section_differs_between_conditions():
    for rid in zp.ROBOTS:
        parts = {name: pk.prompt_parts(name, rid, seed=12) for name in c.MAIN_CONDITIONS}
        for block in pk.COMMON_BLOCKS:
            if block == 'head':
                continue
            assert len({p[block] for p in parts.values()}) == 1, block
        assert len({p['channel'] for p in parts.values()}) == len(c.MAIN_CONDITIONS)


def test_f18_the_behaviour_guidance_is_no_longer_follower_only():
    """The stop/refuse instructions used to exist only in the follower block, so
    the leader condition also got extra BEHAVIOUR guidance, not only a channel."""
    for name in c.MAIN_CONDITIONS:
        for rid in zp.ROBOTS:
            text = pk.system_prompt(name, rid, seed=12)
            assert '안전하지 않다고 판단하면' in text, (name, rid)
            assert pk.KO_BEHAVIOUR in text


def test_f18_the_common_guidance_asks_only_for_what_every_output_can_express():
    """Second review: the guidance asked every condition to leave a reason and
    report it, but no_comm/structured have no field to do so. It now names only
    the action and decision_sources, which every condition's schema has."""
    for word in ('이유', '보고', 'reason'):
        assert word not in pk.KO_BEHAVIOUR, word
    assert 'decision_sources' in pk.KO_BEHAVIOUR and '"wait"' in pk.KO_BEHAVIOUR
    for name in c.MAIN_CONDITIONS:
        text = pk.system_prompt(name, 'r1', seed=12)
        assert '최상위 키는 ' + ', '.join(zp.REPLY_FIELDS) + '입니다' in text
    with pytest.raises(zp.ProtocolError):
        zp.validate_reply({'request_id': 'q', 'action': {'kind': 'wait'},
                           'decision_sources': ['own_rgb'], 'messages': [], 'reason': '위험'},
                          request_id='q', condition='structured', actor='r1')


def test_f18_the_fixed_prompt_cost_is_measured_per_condition():
    report = pk.prompt_token_report(seed=12)
    assert report['tokenizer'] == pk.TOKENIZER_VERSION
    for key, row in report['rows'].items():
        assert row['tokens'] == sum(row['blocks'].values()), key
        assert row['tokens'] == row['common_tokens'] + row['channel_tokens'], key
    # the residual fixed-prompt difference is MEASURED and is entirely the
    # channel section (the structured condition must list its fields)
    main = [row for row in report['rows'].values() if row['condition'] in c.MAIN_CONDITIONS]
    spread = max(row['tokens'] for row in main) - min(row['tokens'] for row in main)
    channel_spread = (max(row['channel_tokens'] for row in main)
                      - min(row['channel_tokens'] for row in main))
    assert report['channel_token_spread'] == channel_spread
    assert report['common_token_spread'] == 0
    assert spread == channel_spread and spread > 0, (spread, channel_spread)
    assert pk.count_tokens('r1은 order-1을 A로 옮깁니다') > 0


def test_f18_the_fixed_prompt_is_billed_at_one_size_in_every_main_condition():
    """Second review: measuring the residual difference was not controlling it.
    Under ``FIXED_PROMPT_POLICY`` the system prompt is billed at the same size
    for every main-condition actor; only the variable part is billed as counted."""
    billed, actual = set(), set()
    for name in c.MAIN_CONDITIONS:
        for rid in zp.ROBOTS:
            trial = _fresh(name)
            bundled = trial.build_inputs(rid, sim_time_s=0.0, request_id='req_bill')
            window = trial.channel.window_context(rid, now_sim_s=0.0) if trial.spec.channel_open else None
            request = pk.build_request(bundled, window=window)
            row = request['billed_tokens']
            assert row['policy'] == pk.FIXED_PROMPT_POLICY
            assert row['system_actual'] == request['tokens']['system']
            assert row['total_text_billed'] == row['system_billed'] + request['tokens']['user']
            billed.add(row['system_billed'])
            actual.add(row['system_actual'])
    assert len(billed) == 1 and len(actual) > 1            # equalised, although the texts differ
    assert billed == {max(actual)}
    # the offline loop charges the billed count, not a fixed stand-in
    trial, result = _trial('structured')
    request = result.requests[0]
    assert result.calls[0]['input_tokens']['text'] == request['billed_tokens']['total_text_billed']


def test_f18_every_request_reports_its_token_counts():
    trial, bundled = _inputs()
    window = trial.channel.window_context('r1', now_sim_s=0.0)
    request = pk.build_request(bundled, window=window)
    tokens = request['tokens']
    assert tokens['tokenizer'] == pk.TOKENIZER_VERSION
    assert tokens['system'] > 0 and tokens['user'] > 0
    assert tokens['total_text'] == tokens['system'] + tokens['user']
    assert tokens['images'] == len(request['images'])


# =========================================================================== #
# 2026-09-26 SECOND review (Codex re-review of the first-round fixes): every
# "partially resolved" finding gets a counterexample that fails when the defect
# is reintroduced. The mutation checks are recorded in the PR table.

# --- finding 1: the final request is archived and re-hashes -----------------

def test_r2_f01_every_offline_request_is_archived_and_rehashes_to_its_digest():
    trial, result = _trial('peer_ko')
    assert result.requests and len(result.requests) == len(result.calls)
    checks = off.request_checks(result)
    assert checks['ok'], checks['problems']
    row = result.requests[0]
    assert {'request_sha256', 'system', 'user', 'image_refs'} <= set(row)
    assert row['request_sha256'] == pk.request_digest_from_refs(row['system'], row['user'], row['image_refs'])
    # an archive that edited or dropped part of the request is detected
    for mutate in (lambda r: r.update(user=r['user'].replace('order_sheet', 'order_shee7')),
                   lambda r: r.update(system=r['system'] + ' '),
                   lambda r: r['image_refs'][0].update(bytes_sha256='0' * 64),
                   lambda r: r.pop('request_sha256')):
        broken = copy.deepcopy(row)
        mutate(broken)
        assert pk.verify_archived_request(broken), broken.keys()
    # and the trial record carries the digest next to the payload keys
    record = trial.trial_record(result)
    archive = {r['request_id']: r for r in record['request_archive']}
    assert archive[row['request_id']]['request_sha256'] == row['request_sha256']


def test_r2_f01_robot_views_of_the_commander_are_immutable_and_rebound():
    trial = _fresh('reference_R')
    bundled = trial.build_inputs('commander', sim_time_s=0.0, request_id='req_views')
    with pytest.raises(TypeError):
        bundled.robot_views['r1'] = bundled.robot_views['r2']
    # even a forced swap of the whole mapping cannot reach the model
    views = dict(bundled.robot_views)
    swapped = {'r1': views['r2'], 'r2': views['r1'], 'r3': views['r3']}
    assert swapped['r1'] != views['r1']
    object.__setattr__(bundled, 'robot_views', swapped)
    with pytest.raises(zp.ProtocolError, match='not the frame'):
        pk.build_request(bundled)


# --- finding 3: nested value types and the normal-path pin -------------------

def test_r2_f03_a_nested_object_under_an_allowed_map_key_is_refused():
    trial, bundled = _inputs()
    for mutate, where in (
            (lambda pm: pm['walls'][0].update(center_m={'survey_xy_m': [1.0, 2.0]}), 'walls[].center_m'),
            (lambda pm: pm['passages'][0].update(width_m=[0.5, 0.1]), 'passages[].width_m'),
            (lambda pm: pm['pickup_bays'][0]['slots'][0].update(half_extents_m={'x': 1}),
             'slots[].half_extents_m'),
            (lambda pm: pm['landmarks']['placement'].update(live_offset_m=0.1), 'landmarks.placement'),
            (lambda pm: pm['landmarks']['tags'][0].update(center_m=[1.0, 'x', 2.0]), 'tags[].center_m'),
            (lambda pm: pm.update(bounds_m={'survey': [0, 0, 1, 1]}), 'public_map.bounds_m')):
        payload = bundled.payload_dict()
        mutate(payload['static_map']['public_map'])
        payload['static_map']['public_map_sha256'] = c.digest(payload['static_map']['public_map'])
        problems = c.payload_violations(payload, seed=SEED)          # hash self-consistent
        assert any(where in p for p in problems), (where, problems)


def test_r2_f03_nested_values_of_the_order_sheet_and_belief_are_typed():
    trial, bundled = _inputs()
    payload = bundled.payload_dict()
    kind = next(iter(payload['order_sheet']['kinds']))
    payload['order_sheet']['kinds'][kind]['roles'] = [{'pose_hint': [0.1, 0.2]}]
    assert any('order_sheet.kinds' in p for p in c.payload_violations(payload, seed=SEED))
    payload = bundled.payload_dict()
    payload['self_belief']['region'] = {'x_m': 0.4, 'y_m': -2.45}
    assert any('self_belief.region' in p for p in c.payload_violations(payload, seed=SEED))


def test_r2_f03_the_normal_construction_path_compares_with_the_frozen_map():
    """``build_call_input`` used to validate without ``source.pinned``, so a
    caller-supplied static_map with a rewritten projection AND hash passed."""
    trial = _fresh('peer_ko')
    forged = copy.deepcopy(trial.static_map)
    forged['public_map']['walls'][0]['height_m'] = 9.9
    forged['public_map_sha256'] = c.digest(forged['public_map'])
    kw = dict(robot_id='r1', condition_name='peer_ko', request_id='req_pin', sim_time_s=0.0,
              source=trial.source, seed=SEED, own_rgb_refs=[], own_command_history=[])
    assert c.payload_violations(dict(si.build_call_input(static_map=trial.static_map, **kw)),
                                seed=SEED, pinned=trial.source.pinned) == []
    with pytest.raises(c.ContractViolation, match='pinned map'):
        si.build_call_input(static_map=forged, **kw)


# --- finding 6: a malformed output keeps its utterances and tokens -----------

def test_r2_f06_a_malformed_reply_bills_its_tokens_and_every_utterance(monkeypatch):
    original = off.FixtureActor.respond
    state = {'broken': 0}

    def respond(self, request):
        raw = json.loads(original(self, request))
        if self.actor == 'r1' and state['broken'] == 0:
            state['broken'] += 1
            raw['messages'] = [{'recipients': ['r2'], 'reply_to': None, 'text': '첫 발화입니다.'},
                               {'recipients': ['r3'], 'reply_to': None, 'text': '둘째 발화입니다.'}]
            raw['extra_key'] = True                           # malformed: fails C's reply schema
        return json.dumps(raw, ensure_ascii=False)

    monkeypatch.setattr(off.FixtureActor, 'respond', respond)
    trial, result = _trial('peer_ko')
    bad = next(row for row in result.calls if row['status'] == 'invalid_json')
    assert bad['cost_terms']['utterances'] == 2 and bad['cost_terms']['gamma_s_per_utterance'] > 0
    assert bad['output_tokens'] > 0 and bad['input_tokens']['text'] > 0
    assert bad['action_id'] is None and bad['message_ids'] == []
    assert trial.scheduler.discarded[0]['unparsed_utterances'] == 2
    assert bad['request_id'] not in {a['request_id'] for a in result.actions}
    archived = next(r for r in result.requests if r['request_id'] == bad['request_id'])
    assert archived['status'] == 'invalid_json' and archived['unparsed_utterances'] == 2
    checks = off.cost_checks(trial, result)
    assert checks['ok'], checks['problems']
    assert checks['billed_utterances'] == checks['produced_utterances']


def test_r2_f06_a_transport_failure_keeps_the_usage_it_knows():
    class Failing:
        def submit(self, call):
            return call

        def reply(self, token):
            raise ds.TransportFailure('malformed JSON after the provider billed it',
                                      attempts=(zc.Attempt(outcome='invalid', input_tokens=300,
                                                           output_tokens=90, utterances=2),))

    sched = ds.EventScheduler(Failing(), policy=ds.CallPolicy(max_retries=0))
    sched.trigger('r1', 'start')
    sched.run(until_s=50)
    cost = sched.calls[0].cost
    assert (cost.breakdown['output_tokens'], cost.breakdown['utterances']) == (90, 2)
    assert cost.outcome == 'invalid' and sched.transport_errors[0]['usage_known'] is True
    assert sched.discarded[0]['unparsed_utterances'] == 2
    # an exception without usage stays labelled as unknown, never silently 0
    class Broken(Failing):
        def reply(self, token):
            raise RuntimeError('socket closed')

    other = ds.EventScheduler(Broken(), policy=ds.CallPolicy(max_retries=0))
    other.trigger('r1', 'start')
    other.run(until_s=50)
    assert other.transport_errors[0]['usage_known'] is False
    with pytest.raises(ValueError, match='only a failed reply'):
        ds.CallReply(attempts=(zc.Attempt(utterances=1),), unparsed_utterances=1)


# --- finding 10: None stays None, the summary is fully compared, no overlap --

def test_r2_f10_a_missing_cost_source_stays_none_in_the_final_metrics():
    trial = _delivery_trial()                           # no calls, no model summary
    metrics = ev.efficiency_metrics(trial)
    for key in ('think_sim_cost_s', 'talk_sim_cost_s', 'utterance_sim_cost_s', 'call_sim_cost_s',
                'model_calls', 'tokens_total', 'talk_share_of_makespan'):
        assert metrics[key] is None, key


def test_r2_f10_image_cache_and_delivery_in_the_summary_must_match_the_log():
    trial, result = _trial('peer_ko')
    record = trial.trial_record(result)
    for path, value in ((('tokens', 'image'), 7), (('tokens', 'cached'), 5),
                        (('sim_cost_s', 'delivery'), 99.0), (('sim_cost_s', 'call'), 1.0)):
        wrong = copy.deepcopy(record)
        wrong['model'][path[0]][path[1]] = value
        with pytest.raises(ev.TrialError, match='.'.join(path)):
            ev.efficiency_metrics(ev.parse_trial(wrong))


def test_r2_f10_think_and_talk_do_not_overlap():
    trial, result = _trial('peer_ko')
    record = trial.trial_record(result)
    metrics = ev.efficiency_metrics(ev.parse_trial(copy.deepcopy(record)))
    done = [c for c in record['calls'] if c['status'] != 'censored']
    total = round(sum(c['sim_cost_s'] for c in done), 4)
    utterance = round(sum(c['cost_terms']['gamma_s_per_utterance'] for c in done), 4)
    assert utterance > 0
    assert metrics['call_sim_cost_s'] == pytest.approx(total)
    assert metrics['think_sim_cost_s'] == pytest.approx(total - utterance)
    assert metrics['think_sim_cost_s'] + metrics['utterance_sim_cost_s'] == \
        pytest.approx(metrics['call_sim_cost_s'])
    assert metrics['talk_sim_cost_s'] == pytest.approx(utterance + metrics['delivery_sim_cost_s'])


# --- finding 11: a wrong-zone fungible item never consumes the order ---------

def test_r2_f11_a_misdelivered_fungible_item_does_not_consume_the_quantity():
    trial = _delivery_trial(deliveries=[
        {'item_id': 'red_0', 'zone': 'C', 'sim_s': 100.0},     # wrong zone
        {'item_id': 'red_1', 'zone': 'B', 'sim_s': 110.0},
        {'item_id': 'red_2', 'zone': 'B', 'sim_s': 120.0}])
    state = ev.delivery_state(trial)
    assert set(state['delivered']) == {'red_1', 'red_2'}
    assert set(state['misdelivered']) == {'red_0'} and state['surplus'] == []
    assert state['by_order']['order-2'] == {'ordered': 2, 'delivered': 2, 'complete': True}
    # a third right-zone item beyond the quantity is surplus, not a delivery
    extra = _delivery_trial(deliveries=[{'item_id': f'red_{i}', 'zone': 'B', 'sim_s': 100.0 + i}
                                        for i in range(3)])
    assert len(ev.delivery_state(extra)['delivered']) == 2
    assert ev.delivery_state(extra)['surplus'] == ['red_2']


# --- finding 12: only item/order ids become claim items ----------------------

def test_r2_f12_a_robot_id_or_role_in_the_sentence_is_not_an_item_claim():
    trial = _delivery_trial(referee=_REFEREE)
    for text in ('r1인 제가 cyan_1을 A에 내려놓았습니다.', 'west 역할로 cyan_1을 A에 내려놓았습니다.',
                 'commander 지시대로 cyan_1을 A에 내려놓았습니다.'):
        claims = ev.extract_claims({'message_id': 'm-9', 'sender': 'r1', 'encoding': 'free_ko',
                                    'text': text}, _LABELS + ('west', 'commander'))
        assert claims == [{'type': 'delivered', 'item_id': 'cyan_1', 'zone': 'A'}], (text, claims)
        assert [ev.check_claim(claim, trial, 150.0) for claim in claims] == ['true']


# --- finding 17: public_part() is safe to reuse ------------------------------

def test_r2_f17_public_part_names_the_scenario_only_by_its_opaque_ref():
    for scenario_id in E.scenario_ids():
        scenario = E.load(scenario_id)
        public = E.public_part(scenario)
        assert public['scenario_id'] == c.scenario_ref(scenario_id)
        suffix = scenario_id.split('_', 1)[1]
        assert scenario_id not in repr(public) and suffix not in repr(public), scenario_id
        # the sheet built from the public part is still the run's sheet
        bundle = E.bundle_for(scenario)
        assert si.OrderSheetSource(public, bundle).sha256 == si.OrderSheetSource(scenario, bundle).sha256


# --- extra: a robot-written message naming nav_cam is not a host leak --------

def test_r2_extra_a_delivered_message_that_mentions_nav_cam_does_not_block_the_recipient():
    text = 'nav_cam은 사용하지 말고 자기 카메라만 보세요.'
    bus = zp.Transport('peer_ko', seed=SEED, delivery_owner=off.BUS_OWNER)
    bus.open_window('w1', at_sim_s=0.)
    receipt = bus.send('r1', recipients=['r2'], text=text, at_sim_s=1.)
    assert receipt.accepted                                   # C accepts it
    bus.commit_delivery(receipt.envelope.message_id, at_sim_s=2., owner=off.BUS_OWNER)
    trial = _fresh('peer_ko')
    bundled = trial.build_inputs('r2', sim_time_s=5.0, request_id='req_nav')
    payload = bundled.payload_dict()
    payload['inbox'] = list(bus.inbox('r2', now_sim_s=5.))
    assert c.payload_violations(payload, seed=SEED, pinned=trial.source.pinned) == []   # A too
    # a HOST-built value that names nav_cam is still refused
    payload['self_belief']['notes_ko'] = 'nav_cam 영상에서 본 위치'
    assert any('nav_cam' in p for p in c.payload_violations(payload, seed=SEED))


# --- extra: no audit evidence is ``unverified``, never ``clean`` -------------

def test_r2_extra_a_trial_without_audit_evidence_is_unverified():
    trial, result = _trial('peer_ko')
    record = trial.trial_record(result)
    boundary = ev.audit_input_boundary(ev.parse_trial(copy.deepcopy(record)))
    assert ev.boundary_status(boundary) == 'clean'            # every call row is evidence
    assert all(req.get('input_keys') for req in ev.parse_trial(copy.deepcopy(record))['requests'])
    bare = _audited_trial([])
    assert ev.boundary_status(ev.audit_input_boundary(bare)) == 'unverified'
    silent = _audited_trial([{'request_id': 'req_1', 'robot': 'r1', 'sim_s': 1.0, 'status': 'ok'}])
    audit = ev.audit_input_boundary(silent)
    assert ev.boundary_status(audit) == 'unverified' and audit['unconfirmed_payloads']
    row = ev.summarise([bare])['conditions']['peer_ko']
    assert row['boundary_unverified_trials'] == 1 and row['boundary_clean_trials'] == 0
