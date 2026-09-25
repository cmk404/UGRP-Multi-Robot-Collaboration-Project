"""Package C: prompts and the communication protocol of the four study conditions.

Offline only: no SIM, no model call. Checks channel isolation (no_comm sends and
receives 0, no follower-to-follower, structured refuses free text, explicit
recipients, per-window caps), that a message cannot change a host claim, reply
parsing and schema violations, language drift being flagged and not repaired,
and leader rotation by seed.

Every input bundle here is a REAL package A payload built by
``harness.zone_study_inputs.build_call_input`` from a real static map bundle, so
these tests fail if package C drifts from A's contract.
"""
import copy
import dataclasses
import inspect
import json
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from harness import zone_study_prompts_ko as pk
from harness import zone_study_protocol as zp
from harness.zone_dialogue_metrics import hangul_ratio
from harness.zone_map_schematic import map_bundle, static_map_section
from harness.zone_study_contract import ContractViolation, condition as contract_condition
from harness.zone_study_inputs import (SCENARIO_SCHEMA, OrderSheetSource, build_call_input,
                                       command_entry, own_rgb_ref, vocabulary)

ROBOTS = zp.ROBOTS
MAP_ID = 'zone_wide_two_doors_tags_v1'
SCENARIO = {
    'schema': SCENARIO_SCHEMA, 'scenario_id': 'c_protocol_fixture', 'map_id': MAP_ID,
    'seeds': [3, 4, 5, 13],
    'orders': [
        {'order_id': 'order-1', 'kind': 'long_beam', 'item_ids': ['long_beam-1'], 'count': 1,
         'required_robots': 2, 'destination_zone': 'A', 'identity': 'specific_item',
         'initial_location': {'pickup_bay': 'P1', 'slot': 'P1-2'}},
        {'order_id': 'order-2', 'kind': 'tile', 'item_ids': ['tile-1'], 'count': 1,
         'required_robots': 1, 'destination_zone': 'C', 'identity': 'specific_item',
         'initial_location': {'pickup_bay': 'P2', 'slot': 'P2-1'}},
    ],
}
JPEG = bytes([0xFF, 0xD8, 0xFF, 0xE0]) + b'kiro-test-wrist' * 4
FRAME_SHA = 'd' * 64
KO_TEXT = 'order-1은 제가 end_neg 역할로 맡겠습니다. door_narrow가 좁아 door_wide로 돌아갑니다.'
EN_TEXT = 'I will take order-1 as end_neg and go through the wide door instead.'
_BUNDLE = None


def bundle():
    """Real map bundle (no schematic render: these tests need no PNG)."""
    global _BUNDLE
    if _BUNDLE is None:
        _BUNDLE = map_bundle(MAP_ID, schematic=False)
    return _BUNDLE


def source():
    return OrderSheetSource(SCENARIO, bundle())


def payload(condition='peer_ko', rid='r1', *, seed=None, request_id='req_1', sim_time_s=4.0,
            inbox=None, own_rgb=True, commands=None, belief=None, team_rgb=None, issued_orders=None):
    """One validated package A per-call payload."""
    spec = contract_condition(condition)
    allow = spec.input_allowlist
    kw = {}
    if 'own_rgb_refs' in allow:
        kw['own_rgb_refs'] = [own_rgb_ref(rid, 42, 3.5, FRAME_SHA)] if own_rgb else []
    if 'own_command_history' in allow:
        kw['own_command_history'] = list(commands if commands is not None else
                                        [command_entry('cmd-0001', 1.0, 'goto',
                                                       {'target_zone': 'A'})])
    if 'self_belief' in allow:
        kw['self_belief'] = belief if belief is not None else {'region': 'unknown', 'confidence': 'low'}
    if 'inbox' in allow:
        kw['inbox'] = list(inbox or [])
    if 'team_rgb_refs' in allow:
        kw['team_rgb_refs'] = list(team_rgb if team_rgb is not None else
                                   [own_rgb_ref(r, 42, 3.5, FRAME_SHA) for r in ROBOTS])
    if 'issued_orders' in allow:
        kw['issued_orders'] = list(issued_orders or [])
    return build_call_input(robot_id=rid, condition_name=condition, request_id=request_id,
                            sim_time_s=sim_time_s, static_map=static_map_section(bundle()),
                            source=source(), seed=seed, **kw)


def inputs(condition='peer_ko', rid='r1', *, seed=None, map_figure_jpeg=None, **kw):
    """A :class:`pk.StudyInputs` wrapping a real A payload."""
    data = payload(condition, rid, seed=seed, **kw)
    if rid == zp.COMMANDER:
        return pk.StudyInputs(payload=data, robot_views={r: JPEG for r in ROBOTS},
                              map_figure_jpeg=map_figure_jpeg, seed=seed)
    return pk.StudyInputs(payload=data, wrist_jpeg=JPEG, map_figure_jpeg=map_figure_jpeg, seed=seed)


def transport(condition, **kw):
    args = dict(vocabulary=vocabulary(source().sheet(), bundle()['public_map']))
    args.update(kw)
    t = zp.Transport(condition, **args)
    t.open_window('w1', at_sim_s=10.0)
    return t


def struct(**kw):
    body = {'act': 'inform', 'item': 'order-1', 'zone': 'A', 'role': 'end_neg',
            'passage': 'door_narrow', 'location_ref': 'P1-2', 'state': 'blocked',
            'confidence': 'high', 'observed_at_sim_s': 9.5, 'reply_to': None}
    body.update(kw)
    return body


# --- condition registry and leader rotation -------------------------------

def test_four_main_conditions_and_the_reference_ceiling():
    main = [name for name, s in zp.SPECS.items() if s.main_condition]
    assert main == ['no_comm', 'peer_ko', 'leader_ko', 'structured']
    assert zp.spec('reference_R').main_condition is False
    assert zp.spec('reference_R').robot_llm is False and zp.spec('reference_R').commander_llm is True
    assert zp.actors('peer_ko') == ROBOTS and zp.actors('reference_R') == (zp.COMMANDER,)
    # Only channel fields may differ between the four main conditions. Listing
    # every field means a new ConditionSpec field cannot escape this check.
    fields = tuple(f.name for f in dataclasses.fields(zp.ConditionSpec))
    channel = ('name', 'topology', 'encoding', 'rotating_leader', 'max_window_utterances',
               'max_robot_utterances')
    assert set(fields) == set(channel) | {'robot_llm', 'commander_llm', 'main_condition'}
    same = {tuple(getattr(zp.SPECS[c], f) for f in fields if f not in channel) for c in main}
    assert same == {(True, False, True)}               # robot_llm, commander_llm, main_condition
    assert {zp.SPECS[c].topology for c in main} == {'none', 'mesh', 'star'}
    # package A's encoding literals
    assert {zp.SPECS[c].encoding for c in main} == {'none', 'free_ko', 'schema'}


def test_specs_are_derived_from_the_package_a_registry():
    """A condition, topology or encoding change in A must reach this protocol."""
    from harness.zone_study_contract import CONDITIONS as A_CONDITIONS

    assert tuple(zp.SPECS) == tuple(A_CONDITIONS) == zp.CONDITIONS
    for name, contract in A_CONDITIONS.items():
        s = zp.SPECS[name]
        assert (s.topology, s.encoding) == (contract.topology, contract.encoding)
        assert s.rotating_leader is contract.leader_rotation
        assert s.robot_llm is contract.robot_llm and s.main_condition is contract.is_main
        from harness.zone_study_contract import allowed_edges as a_edges
        if s.channel_open:
            assert zp.allowed_edges(name, seed=13) == a_edges(name, 13)
        else:
            # no message channel here: reference_R orders through an action, so
            # this protocol relays nothing even though A declares a downlink
            assert zp.allowed_edges(name, seed=13) == frozenset()


def test_leader_rotates_r1_r2_r3_by_seed():
    assert [zp.leader_for_seed(s) for s in (0, 1, 2, 3, 4, 5)] == ['r1', 'r2', 'r3', 'r1', 'r2', 'r3']
    assert {zp.leader_for_seed(s) for s in range(11, 14)} == set(ROBOTS)
    for seed, lead in ((11, 'r3'), (12, 'r1'), (13, 'r2')):
        assert zp.leader_of('leader_ko', seed=seed) == lead
        assert zp.role_of('leader_ko', lead, seed=seed) == 'leader'
        assert [zp.role_of('leader_ko', r, seed=seed) for r in ROBOTS].count('follower') == 2
        # the star follows the rotating leader; no follower-to-follower edge exists
        assert zp.allowed_edges('leader_ko', seed=seed) == frozenset(
            [*((lead, b) for b in ROBOTS if b != lead), *((b, lead) for b in ROBOTS if b != lead)])
    with pytest.raises(zp.ProtocolError):
        zp.leader_of('leader_ko')                      # rotation needs a seed
    with pytest.raises(zp.ProtocolError):
        zp.leader_of('peer_ko', leader='r1')           # peers have no leader


def test_topologies_and_unknown_condition():
    assert zp.allowed_edges('no_comm') == frozenset() and zp.allowed_edges('reference_R') == frozenset()
    assert zp.allowed_edges('peer_ko') == zp.allowed_edges('structured')
    assert len(zp.allowed_edges('peer_ko')) == 6 and len(zp.allowed_edges('leader_ko', seed=1)) == 4
    with pytest.raises(zp.ProtocolError):
        zp.spec('leader_en')


# --- channel isolation ----------------------------------------------------

@pytest.mark.parametrize('condition', ['no_comm', 'reference_R'])
def test_closed_channel_sends_and_receives_zero(condition):
    t = transport(condition)
    r = t.send('r1', recipients=['r2'], text=KO_TEXT, at_sim_s=10.5)
    assert (r.accepted, r.rejection) == (False, 'channel_closed')
    assert t.send('r2', recipients=['r1', 'r3'], structured=struct(), at_sim_s=11.).rejection == 'channel_closed'
    assert all(t.inbox(rid, now_sim_s=99.) == () and t.delivered_count(rid) == 0 for rid in ROBOTS)
    assert t.sent_count() == 0 and len(t.rejections) == 2 and t.remaining('r1') == 0


def test_peer_mesh_delivers_to_named_recipients_only():
    t = transport('peer_ko')
    r = t.send('r1', recipients=['r2'], text=KO_TEXT, at_sim_s=10.)
    assert r.accepted and r.deliveries == (('r2', 10.1),)
    assert [m['sender'] for m in t.inbox('r2', now_sim_s=10.1)] == ['r1']
    assert t.inbox('r3', now_sim_s=99.) == ()          # never addressed
    assert t.send('r2', recipients=['r1', 'r3'], text=KO_TEXT, at_sim_s=11.).accepted
    assert len(t.inbox('r3', now_sim_s=99.)) == 1
    assert t.send('r3', recipients=['r3'], text=KO_TEXT, at_sim_s=11.).rejection == 'self_recipient'
    assert t.send('r3', recipients=['r4'], text=KO_TEXT, at_sim_s=11.).rejection == 'unknown_recipient'
    assert t.send('r3', recipients=[], text=KO_TEXT, at_sim_s=11.).rejection == 'no_recipients'


def test_leader_star_refuses_follower_to_follower():
    t = transport('leader_ko', seed=12)               # leader r1
    assert t.leader == 'r1'
    assert t.send('r1', recipients=['r2'], text=KO_TEXT, at_sim_s=10.).accepted        # leader -> follower
    assert t.send('r2', recipients=['r1'], text='거절합니다. door_narrow가 막혀 보입니다.',
                  at_sim_s=10.2).accepted                                             # follower -> leader
    bad = t.send('r2', recipients=['r3'], text=KO_TEXT, at_sim_s=10.4)
    assert (bad.accepted, bad.rejection) == (False, 'no_follower_to_follower')
    mixed = t.send('r3', recipients=['r1', 'r2'], text=KO_TEXT, at_sim_s=10.5)
    assert mixed.rejection == 'no_follower_to_follower'   # rejected whole, not partly relayed
    assert t.inbox('r3', now_sim_s=99.) == ()
    assert t.delivered_count('r2') == 1 and t.sent_count('r2') == 1


def test_structured_channel_refuses_free_text_and_unknown_ids():
    t = transport('structured')
    assert t.send('r1', recipients=['r2'], text=KO_TEXT, at_sim_s=10.).rejection == 'free_text_not_allowed'
    smuggled = t.send('r1', recipients=['r2'], structured={**struct(), 'reason': '좁아서'}, at_sim_s=10.)
    assert smuggled.rejection == 'free_text_not_allowed'
    assert t.send('r1', recipients=['r2'], structured=struct(item='beam-99'),
                  at_sim_s=10.).rejection == 'schema'
    assert t.send('r1', recipients=['r2'], structured=struct(act='chat'), at_sim_s=10.).rejection == 'schema'
    assert t.send('r1', recipients=['r2'], structured=struct(confidence=0.9),
                  at_sim_s=10.).rejection == 'schema'
    ok = t.send('r1', recipients=['r2'], structured=struct(), at_sim_s=10.)
    assert ok.accepted and t.inbox('r2', now_sim_s=10.1)[0]['body'] == struct()
    assert t.sent_count() == 1


def test_free_text_channel_refuses_a_structured_body_and_empty_text():
    t = transport('peer_ko')
    assert t.send('r1', recipients=['r2'], structured=struct(), at_sim_s=10.).rejection == 'structured_not_allowed'
    assert t.send('r1', recipients=['r2'], text='   ', at_sim_s=10.).rejection == 'empty_text'
    assert t.send('r1', recipients=['r2'], text='가' * (zp.MAX_TEXT_CHARS + 1),
                  at_sim_s=10.).rejection == 'text_too_long'


def test_reply_to_only_for_messages_this_robot_received():
    t = transport('leader_ko', seed=12)
    mid = t.send('r1', recipients=['r2'], text=KO_TEXT, at_sim_s=10.).envelope.message_id
    assert t.send('r2', recipients=['r1'], text='확인했습니다.', reply_to=mid, at_sim_s=10.2).accepted
    # r3 never received that message, so it cannot reference the id
    assert t.send('r3', recipients=['r1'], text='확인했습니다.', reply_to=mid,
                  at_sim_s=10.3).rejection == 'unknown_reply_to'


# --- utterance caps -------------------------------------------------------

def test_window_and_per_robot_caps():
    t = transport('peer_ko')
    assert (t.cap_window, t.cap_robot) == (6, 2)
    for rid in ('r1', 'r2'):
        assert t.remaining(rid) == 2
        for _ in range(2):
            assert t.send(rid, recipients=[b for b in ROBOTS if b != rid], text=KO_TEXT, at_sim_s=10.).accepted
        over = t.send(rid, recipients=[b for b in ROBOTS if b != rid], text=KO_TEXT, at_sim_s=10.)
        assert (over.accepted, over.rejection) == (False, 'robot_cap')   # the window still has room
        assert t.remaining(rid) == 0
    assert t.sent_count() == 4 and t.remaining('r3') == 2
    for _ in range(2):
        assert t.send('r3', recipients=['r1'], text=KO_TEXT, at_sim_s=10.).accepted
    assert t.sent_count() == 6 and t.remaining() == 0
    assert t.send('r3', recipients=['r1'], text=KO_TEXT, at_sim_s=10.).rejection == 'window_cap'
    t.open_window('w2', at_sim_s=20.)                 # caps reset per window
    assert t.remaining('r1') == 2 and t.send('r1', recipients=['r2'], text=KO_TEXT, at_sim_s=20.).accepted


def test_window_cap_stops_a_talkative_team():
    t = transport('peer_ko', max_robot_utterances=6)
    for i in range(6):
        assert t.send('r1', recipients=['r2'], text=f'{i}번째 보고입니다.', at_sim_s=10.).accepted
    capped = t.send('r1', recipients=['r2'], text=KO_TEXT, at_sim_s=10.)
    assert (capped.accepted, capped.rejection) == (False, 'window_cap')
    assert t.delivered_count('r2') == 6


def test_sending_without_a_window_is_rejected():
    t = zp.Transport('peer_ko')
    assert t.send('r1', recipients=['r2'], text=KO_TEXT, at_sim_s=1.).rejection == 'no_window'


# --- delivery timing and robot-facing records -----------------------------

def test_delivery_delay_and_inbox_fields():
    t = transport('peer_ko')
    t.send('r1', recipients=['r2'], text=KO_TEXT, at_sim_s=10.)
    assert t.inbox('r2', now_sim_s=10.05) == ()        # not yet delivered
    record, = t.inbox('r2', now_sim_s=10.1)
    assert set(record) <= set(zp.INBOX_FIELDS)
    assert record['body'] == {'text': KO_TEXT} and record['sender'] == 'r1'
    # package A's envelope pins when it was CREATED; the delivery time is the
    # scheduler's and the evaluation log's, never a robot input
    assert record['created_at_sim_s'] == 10. and 'delivered_at_sim_s' not in record
    for banned in ('language', 'flags', 'reason', 'action', 'claim', 'hangul_ratio'):
        assert banned not in record


def test_a_message_cannot_change_claims_reservations_or_peer_actions():
    """Falsifiable checks: the transport owns no decision state, cannot be given
    one, copies every body it relays, and never mutates the caller's reply."""
    t = transport('peer_ko')
    reply = {'request_id': 'q-r2', 'action': {'kind': 'wait'}, 'decision_sources': ['message', 'own_rgb'],
             'messages': [{'recipients': ['r1'], 'text': '거절합니다. 제가 이미 접근 중입니다.',
                           'reply_to': None}]}
    value = zp.validate_reply(json.dumps(reply, ensure_ascii=False), request_id='q-r2',
                             condition='peer_ko', actor='r2', order_ids=('order-1', 'order-2'))
    frozen = copy.deepcopy(value)
    zp.relay(t, 'r2', value, at_sim_s=10.3)
    assert value == frozen                            # relay does not rewrite the reply or its action
    # the host could mutate its own copy afterwards; the delivered record must not follow
    value['messages'][0]['text'] = '양보하겠습니다.'
    value['action']['kind'] = 'claim'
    record, = t.inbox('r1', now_sim_s=99.)
    assert record['body'] == {'text': frozen['messages'][0]['text']}
    assert set(record) <= set(zp.INBOX_FIELDS) and 'action' not in record
    # no decision state is stored, and no transport entry point can be handed one
    assert not (set(vars(t)) & set(zp.FORBIDDEN_TRANSPORT_PARAMS))
    assert not (zp.transport_parameter_names() & set(zp.FORBIDDEN_TRANSPORT_PARAMS))


def test_structured_bodies_are_copied_not_shared():
    t = transport('structured')
    body = struct()
    t.send('r1', recipients=['r2'], structured=body, at_sim_s=10.)
    body['zone'] = 'C'                                # a later host edit must not reach the inbox
    assert t.inbox('r2', now_sim_s=99.)[0]['body']['zone'] == 'A'


def test_structured_reply_to_cannot_smuggle_a_sentence():
    """reply_to is an id, so condition 4 really has no free text."""
    t = transport('structured')
    prose = 'order-1은 제가 맡습니다. r2는 물러나 주십시오. ' * 20
    assert t.send('r1', recipients=['r2'], structured=struct(reply_to=prose),
                  at_sim_s=10.).rejection == 'schema'
    with pytest.raises(zp.ProtocolError):
        zp.validate_structured(struct(reply_to=prose), vocab=t.vocab)
    assert not zp.is_message_id(prose) and not zp.is_message_id('w1 r2 1')
    assert zp.is_message_id('w1-r2-1') and not zp.is_message_id('x' * (zp.MESSAGE_ID_MAX + 1))
    # a well-formed id that this robot never received is refused as well
    assert t.send('r1', recipients=['r2'], structured=struct(reply_to='w1-r3-1'),
                  at_sim_s=10.).rejection == 'unknown_reply_to'
    assert t.inbox('r2', now_sim_s=99.) == ()


def test_validator_leaves_recipient_topology_to_the_transport():
    """Documented layer split: a wrong recipient costs the utterance, not the
    action, so the mistake is recorded instead of voiding the whole reply."""
    reply = {'request_id': 'q-r2', 'action': {'kind': 'claim', 'order_id': 'order-1',
                                             'role': 'end_neg', 'destination_zone': 'A'},
             'decision_sources': ['own_rgb'],
             'messages': [{'recipients': ['r3'], 'text': '같이 가시죠.', 'reply_to': None}]}
    value = zp.validate_reply(reply, request_id='q-r2', condition='leader_ko', actor='r2',
                             order_ids=('order-1',), roles_by_order={'order-1': ('end_neg',)})
    t = transport('leader_ko', seed=12)               # leader r1
    receipt, = zp.relay(t, 'r2', value, at_sim_s=10.)
    assert receipt.rejection == 'no_follower_to_follower'
    assert value['action']['order_id'] == 'order-1'    # the action survives the rejected utterance
    assert t.inbox('r3', now_sim_s=99.) == () and t.rejections[0]['rejection'] == 'no_follower_to_follower'


def test_unknown_sender_and_reopened_windows_are_refused():
    t = transport('peer_ko')
    assert t.send('r9', recipients=['r1'], text=KO_TEXT, at_sim_s=10.).rejection == 'unknown_sender'
    assert t.send(zp.COMMANDER, recipients=['r1'], text=KO_TEXT, at_sim_s=10.).rejection == 'unknown_sender'
    with pytest.raises(zp.ProtocolError):             # re-opening a window would reset the cap
        t.open_window('w1', at_sim_s=20.)
    assert t.windows == ['w1']


def test_optional_run_budget_survives_new_windows():
    t = zp.Transport('peer_ko', max_total_utterances=3)
    for i in range(4):
        t.open_window(f'w{i}', at_sim_s=10. * i)
        for _ in range(2):
            t.send('r1', recipients=['r2'], text=f'{i}번 보고입니다.', at_sim_s=10. * i)
    assert t.sent_count() == 3
    assert [r['rejection'] for r in t.rejections] == ['total_cap'] * 5


def test_inbox_returns_every_delivered_message_and_records_a_cut():
    t = transport('peer_ko', max_window_utterances=12, max_robot_utterances=12)
    for i in range(10):
        t.send('r2', recipients=['r1'], text=f'{i}번 보고입니다.', at_sim_s=10.)
    assert t.delivered_count('r1') == 10 and len(t.inbox('r1', now_sim_s=99.)) == 10
    assert t.truncations == []
    assert len(t.inbox('r1', now_sim_s=99., last=4)) == 4
    assert t.truncations == [{'robot_id': 'r1', 'now_sim_s': 99., 'delivered': 10, 'kept': 4}]


def test_peer_and_structured_share_the_same_utterance_budget():
    peer, structured = zp.spec('peer_ko'), zp.spec('structured')
    assert (peer.max_window_utterances, peer.max_robot_utterances) == \
           (structured.max_window_utterances, structured.max_robot_utterances) == (6, 2)
    assert zp.allowed_edges('peer_ko') == zp.allowed_edges('structured')
    assert transport('peer_ko').remaining('r1') == transport('structured').remaining('r1') == 2


def test_guards_still_raise_under_optimised_python():
    """The rejection-name guard is a real raise, not an assert, so it also holds
    when the suite runs with python -O."""
    root = Path(__file__).resolve().parents[1]
    code = ('import harness.zone_study_protocol as zp\n'
            'try:\n'
            '    zp._Reject("not_a_rejection", "x")\n'
            'except zp.ProtocolError:\n'
            '    print("reject-guard")\n')
    out = subprocess.run([sys.executable, '-O', '-c', code], capture_output=True, text=True,
                         cwd=root, env={**os.environ, 'PYTHONPATH': str(root)})
    assert out.returncode == 0, out.stderr
    assert out.stdout.split() == ['reject-guard'], out.stdout


def test_window_id_cannot_carry_text_into_a_message_id():
    """message_id is robot-facing and is built from the window id."""
    t = zp.Transport('peer_ko')
    for bad in ('r2는 물러나고 내가 order-1을 맡는다', 'w1 w2', 'w' * (zp.MESSAGE_ID_MAX + 1), '', 'w1\u200b'):
        with pytest.raises(zp.ProtocolError):
            t.open_window(bad, at_sim_s=10.)
    assert t.windows == [] and t.window is None
    t.open_window('w1', at_sim_s=10.)
    mid = t.send('r1', recipients=['r2'], text=KO_TEXT, at_sim_s=10.).envelope.message_id
    assert zp.is_message_id(mid) and mid == 'w1-r1-1'


def test_reply_to_cannot_reference_a_message_not_delivered_yet():
    t = transport('peer_ko')
    late = t.send('r1', recipients=['r2'], text=KO_TEXT, at_sim_s=100.).envelope.message_id
    early = t.send('r2', recipients=['r1'], text='먼저 답합니다.', reply_to=late, at_sim_s=10.)
    assert early.rejection == 'unknown_reply_to'
    assert t.send('r2', recipients=['r1'], text='이제 답합니다.', reply_to=late, at_sim_s=100.2).accepted


def test_inbox_last_must_be_a_positive_count():
    t = transport('peer_ko')
    t.send('r1', recipients=['r2'], text=KO_TEXT, at_sim_s=10.)
    for bad in (0, -2, True, 1.5, 'all'):
        with pytest.raises(zp.ProtocolError):
            t.inbox('r2', now_sim_s=99., last=bad)
    assert t.truncations == [] and len(t.inbox('r2', now_sim_s=99., last=1)) == 1


def test_a_delivered_record_is_a_copy():
    t = transport('structured')
    t.send('r1', recipients=['r2'], structured=struct(), at_sim_s=10.)
    record = t.inbox('r2', now_sim_s=99.)[0]
    record['body']['zone'] = 'C'
    record['recipients'].append('r3')
    assert t.inbox('r2', now_sim_s=99.)[0]['body']['zone'] == 'A'
    assert t.inbox('r2', now_sim_s=99.)[0]['recipients'] == ['r2']


def test_validated_action_is_a_copy_of_the_reply():
    raw = _reply()
    value = zp.validate_reply(copy.deepcopy(raw), request_id='q-1', condition='peer_ko', actor='r1',
                             order_ids=('order-1',), roles_by_order={'order-1': ('end_neg',)})
    value['action']['order_id'] = 'order-2'
    assert raw['action']['order_id'] == 'order-1'


def test_the_prompt_states_the_transports_real_budget():
    """A window context from the transport, not the condition default."""
    t = zp.Transport('peer_ko', max_window_utterances=30, max_robot_utterances=9,
                     max_total_utterances=50,
                     vocabulary=vocabulary(source().sheet(), bundle()['public_map']))
    t.open_window('w1', at_sim_s=10.)
    t.send('r1', recipients=['r2'], text=KO_TEXT, at_sim_s=10.)
    context = t.window_context('r1', now_sim_s=99.)
    body = json.loads(pk.build_request(inputs(), window=context)['messages'][1]['content'])
    # ``received`` is not duplicated here: package A's ``inbox`` carries it.
    assert body[pk.WINDOW_KEY] == {'window_id': 'w1', 'max_utterances': 30,
                                   'max_your_utterances': 9, 'your_utterances_left': 8,
                                   'sent': ['w1-r1-1']}
    assert set(body[pk.WINDOW_KEY]) == set(pk.WINDOW_FIELDS)
    with pytest.raises(zp.ProtocolError):             # an open channel needs its window id
        pk.build_request(inputs())
    with pytest.raises(zp.ProtocolError):             # unknown window fields are refused
        pk.build_request(inputs(), window={'window_id': 'w1', 'budget': 99})
    assert zp.Transport('no_comm').window_context('r1', now_sim_s=1.) is None


def test_build_request_requires_the_validated_input_bundle():
    """A look-alike object must not be able to skip package A's boundary."""
    leaky = SimpleNamespace(payload=payload('no_comm'), wrist_jpeg=JPEG, map_figure_jpeg=None,
                            robot_views=None, seed=None, condition='no_comm', robot_id='r1',
                            request_id='q', sim_time_s=1.0, inbox=())
    with pytest.raises(zp.ProtocolError):
        pk.build_request(leaky)
    # and a raw dict payload is not a StudyInputs either
    with pytest.raises(zp.ProtocolError):
        pk.build_request(payload('no_comm'))


def test_study_inputs_refuse_a_payload_outside_the_contract():
    """The boundary is package A's: a tampered payload never reaches a prompt."""
    for mutate in ({'top_rgb': ['top-0001']},                       # evaluation-only camera
                   {'teacher_receipt': {'placed': True}},           # teacher judgement
                   {'peer_commands': []},                           # another robot's commands
                   {'hidden_events': []},                           # private schedule
                   {'zone_counts': {'A': 1}}):                      # host progress
        bad = {**payload('no_comm'), **mutate}
        with pytest.raises(zp.ProtocolError):
            pk.StudyInputs(payload=bad, wrist_jpeg=JPEG)
    # a foreign wrist frame is refused too
    bad = payload('no_comm')
    bad['own_rgb_refs'] = [own_rgb_ref('r2', 42, 3.5, FRAME_SHA)]
    with pytest.raises(zp.ProtocolError):
        pk.StudyInputs(payload=bad, wrist_jpeg=JPEG)
    # the wrong schema id is refused before anything else
    with pytest.raises(zp.ProtocolError):
        pk.StudyInputs(payload={**payload('no_comm'), 'schema': 'ugrp.zone_study_call_input.v0'},
                       wrist_jpeg=JPEG)


def test_package_a_rejects_the_live_state_the_local_validators_used_to_catch():
    """The checks that were local to package C now live in package A's contract."""
    with pytest.raises(ContractViolation):            # a measured joint inside own command arguments
        command_entry('cmd-1', 1., 'goto', {'qpos': [.1, .2]})
    with pytest.raises(ContractViolation):            # peer state inside own command arguments
        inputs('no_comm', commands=[{'command_id': 'cmd-1', 'issued_at_sim_s': 1., 'kind': 'goto',
                                     'arguments': {'held_by': 'r2'}, 'local_state': 'command_issued'}])
    assert command_entry('cmd-1', 1., 'goto', {'speed': .2})['local_state'] == 'command_issued'
    with pytest.raises(ContractViolation):            # an executor success is not a self state
        command_entry('cmd-1', 1., 'goto', {}, local_state='grasp_success')
    for bad in ({'ground_truth_xyz_m': [1, 2, 0]},                 # ground truth
                {'r2_position_m': [1, 2]},                         # peer position
                {'top_rgb_ref': 'top-0001'}):                      # evaluation-only camera
        with pytest.raises(ContractViolation):
            inputs('no_comm', belief=bad)
    allowed = inputs('no_comm', belief={'region': 'pickup', 'confidence': 'low',
                                       'sources': ['own-r1-0042']})
    assert allowed.own_belief['confidence'] == 'low'
    # A checks key names and structure, not the meaning of a value: an off-enum
    # confidence passes the contract, so whoever fills the belief owns that.
    assert inputs('no_comm', belief={'region': 'pickup', 'confidence': 0.9}).own_belief == \
        {'region': 'pickup', 'confidence': 0.9}


def test_order_sheet_comes_from_the_config_and_rejects_live_state():
    sheet = source().sheet()
    assert sheet['schema'] == pk.ORDER_SHEET_SCHEMA
    assert sheet['orders'][0]['initial_location']['slot'] == 'P1-2'
    assert sheet['kinds']['long_beam']['roles'] == ['end_neg', 'end_pos']
    for mutate in (lambda o: o.update(status='delivered'),
                   lambda o: o.update(xyz_m=[1, 2, 0]),
                   lambda o: o.update(required_robots=1),
                   lambda o: o.update(destination_zone='D'),
                   lambda o: o.update(item_ids=[]),
                   lambda o: o.update(initial_location={'slot': 'P1-2'})):
        scenario = copy.deepcopy(SCENARIO)
        mutate(scenario['orders'][0])
        with pytest.raises(ContractViolation):
            OrderSheetSource(scenario, bundle())
    with pytest.raises(ContractViolation):
        OrderSheetSource({**SCENARIO, 'schema': 'ugrp.zone_scenario.v0'}, bundle())


def test_image_bytes_are_copied_at_construction():
    buf = bytearray(JPEG)
    got = pk.StudyInputs(payload=payload('no_comm'), wrist_jpeg=buf)
    buf[0] = 0
    assert got.wrist_jpeg == JPEG


# --- reply parsing and schema violations ----------------------------------

def _reply(**kw):
    value = {'request_id': 'q-1', 'action': {'kind': 'claim', 'order_id': 'order-1', 'role': 'end_neg',
                                            'destination_zone': 'A'},
             'decision_sources': ['static_map', 'order_sheet', 'own_rgb'],
             'messages': [{'recipients': ['r2'], 'text': KO_TEXT, 'reply_to': None}]}
    value.update(kw)
    return value


def _validate(value, condition='peer_ko', actor='r1', **kw):
    args = dict(request_id='q-1', condition=condition, actor=actor,
                order_ids=('order-1', 'order-2'), item_ids=('long_beam-1', 'tile-1'),
                roles_by_order={'order-1': ('end_neg', 'end_pos'), 'order-2': ('west',)},
                passages=('door_narrow', 'door_wide'), location_refs=('P1-2',))
    args.update(kw)
    return zp.validate_reply(value if isinstance(value, str) else json.dumps(value, ensure_ascii=False), **args)


def test_valid_reply_separates_action_from_messages():
    value = _validate(_reply())
    assert value['action'] == {'kind': 'claim', 'order_id': 'order-1', 'role': 'end_neg',
                              'destination_zone': 'A'}
    assert value['messages'][0]['text'] == KO_TEXT
    assert 'text' not in value['action'] and 'kind' not in value['messages'][0]


def test_fenced_json_is_accepted_and_malformed_json_is_not():
    fenced = '```json\n' + json.dumps(_reply(), ensure_ascii=False) + '\n```'
    assert _validate(fenced)['request_id'] == 'q-1'
    for raw in ('{"request_id": "q-1", ', 'not json at all', '[]', '"q-1"'):
        with pytest.raises(zp.ProtocolError):
            _validate(raw)


@pytest.mark.parametrize('mutate', [
    lambda v: v.pop('messages'),
    lambda v: v.update(reason='짧은 근거'),
    lambda v: v.update(request_id='q-2'),
    lambda v: v.update(action={'kind': 'claim', 'order_id': 'order-9', 'role': 'end_neg',
                               'destination_zone': 'A'}),
    lambda v: v.update(action={'kind': 'claim', 'order_id': 'order-1', 'role': 'west',
                               'destination_zone': 'A'}),
    lambda v: v.update(action={'kind': 'claim', 'order_id': 'order-1', 'role': 'end_neg',
                               'destination_zone': 'D'}),
    lambda v: v.update(action={'kind': 'order', 'assignments': {r: None for r in ROBOTS}}),
    lambda v: v.update(action={'kind': 'wait', 'order_id': 'order-1'}),
    lambda v: v.update(decision_sources=[]),
    lambda v: v.update(decision_sources=['top_rgb']),
    lambda v: v.update(decision_sources=['own_rgb', 'own_rgb']),
    lambda v: v.update(messages=[{'recipients': ['r1'], 'text': KO_TEXT, 'reply_to': None}]),
    lambda v: v.update(messages=[{'recipients': [], 'text': KO_TEXT, 'reply_to': None}]),
    lambda v: v.update(messages=[{'recipients': ['r2'], 'text': '', 'reply_to': None}]),
    lambda v: v.update(messages=[{'recipients': ['r2'], 'text': KO_TEXT}]),
    lambda v: v.update(messages=[{'recipients': ['r2'], 'message': struct(), 'reply_to': None}]),
])
def test_schema_violations_are_refused(mutate):
    value = _reply()
    mutate(value)
    with pytest.raises(zp.ProtocolError):
        _validate(value)


def test_no_comm_reply_must_be_silent():
    with pytest.raises(zp.ProtocolError):
        _validate(_reply(), condition='no_comm')                       # messages non-empty
    with pytest.raises(zp.ProtocolError):
        _validate(_reply(messages=[], decision_sources=['message']), condition='no_comm')
    assert _validate(_reply(messages=[], decision_sources=['own_rgb']), condition='no_comm')['messages'] == []


def test_structured_reply_refuses_free_text_and_checks_the_schema():
    ok = _validate(_reply(messages=[{'recipients': ['r2'], 'message': struct(), 'reply_to': None}]),
                   condition='structured')
    assert ok['messages'][0]['message']['act'] == 'inform'
    for bad in ([{'recipients': ['r2'], 'text': KO_TEXT, 'reply_to': None}],
                [{'recipients': ['r2'], 'message': {**struct(), 'text': '좁아요'}, 'reply_to': None}],
                [{'recipients': ['r2'], 'message': struct(state='maybe'), 'reply_to': None}],
                [{'recipients': ['r2'], 'message': struct(passage='door_side'), 'reply_to': None}],
                [{'recipients': ['r2'], 'message': struct(observed_at_sim_s=-1), 'reply_to': None}]):
        with pytest.raises(zp.ProtocolError):
            _validate(_reply(messages=bad), condition='structured')


def test_commander_orders_through_action_and_stays_silent():
    order = {'kind': 'order', 'assignments': {
        'r1': {'order_id': 'order-1', 'role': 'end_neg', 'destination_zone': 'A'},
        'r2': {'order_id': 'order-1', 'role': 'end_pos', 'destination_zone': 'A'},
        'r3': None}}
    value = _validate(_reply(action=order, messages=[], decision_sources=['static_map', 'own_rgb']),
                      condition='reference_R', actor=zp.COMMANDER)
    assert value['action']['assignments']['r3'] is None
    with pytest.raises(zp.ProtocolError):                      # a robot may not order peers
        _validate(_reply(action=order, messages=[]), condition='peer_ko', actor='r1')
    with pytest.raises(zp.ProtocolError):                      # commander must not talk
        _validate(_reply(action=order, messages=[{'recipients': ['r1'], 'text': KO_TEXT,
                                                  'reply_to': None}]),
                  condition='reference_R', actor=zp.COMMANDER)
    with pytest.raises(zp.ProtocolError):                      # missing robot in assignments
        _validate(_reply(action={'kind': 'order', 'assignments': {'r1': None}}, messages=[]),
                  condition='reference_R', actor=zp.COMMANDER)


def test_release_and_wait_actions():
    assert _validate(_reply(action={'kind': 'release', 'order_id': 'order-1'}))['action']['kind'] == 'release'
    assert _validate(_reply(action={'kind': 'wait'}))['action'] == {'kind': 'wait'}
    with pytest.raises(zp.ProtocolError):
        _validate(_reply(action={'kind': 'release', 'order_id': 'order-9'}))


# --- language drift: flagged, never repaired ------------------------------

def test_non_korean_message_is_flagged_and_still_delivered_unchanged():
    t = transport('peer_ko')
    receipt = t.send('r1', recipients=['r2'], text=EN_TEXT, at_sim_s=10.)
    assert receipt.accepted                                   # flagged, not blocked
    assert 'non_korean' in receipt.language['flags'] and receipt.language['korean'] is False
    record, = t.inbox('r2', now_sim_s=10.1)
    assert record['body'] == {'text': EN_TEXT}                # byte-identical, never rewritten
    assert 'language' not in record and 'flags' not in record
    entry, = [e for e in t.log if e['accepted']]
    assert 'non_korean' in entry['language']['flags']          # evaluation log only


def test_korean_text_with_literal_ids_is_not_flagged():
    report = zp.language_report(KO_TEXT, literals=('order-1', 'end_neg', 'door_narrow', 'door_wide'))
    assert report['korean'] and report['flags'] == () and report['hangul_ratio'] > 0.9


def test_code_switch_and_literal_id_drift_are_flagged_separately():
    mixed = zp.language_report('order-1을 wide door로 옮기겠습니다.', literals=('order-1',))
    assert 'code_switch' in mixed['flags'] and mixed['latin_words'] == ['wide', 'door']
    drift = zp.language_report('로봇 2에게 넘깁니다.', literals=())
    assert 'literal_id_issue' in drift['flags']
    assert zp.language_report('', literals=())['flags'] == ('silence',)


def test_validators_never_reject_a_reply_for_language():
    """Language drift is an evaluation flag, not a schema violation."""
    value = _validate(_reply(messages=[{'recipients': ['r2'], 'text': EN_TEXT, 'reply_to': None}]))
    assert value['messages'][0]['text'] == EN_TEXT


# --- Korean prompt skeletons ----------------------------------------------

CASES = [('no_comm', 'r1', None), ('peer_ko', 'r2', None), ('leader_ko', 'r3', 13),
         ('leader_ko', 'r2', 13), ('structured', 'r1', None), ('reference_R', zp.COMMANDER, None)]
LITERALS = ('r1', 'r2', 'r3', 'A', 'B', 'C', 'order_id', 'item_id', 'order-1', 'end_neg', 'door_narrow',
            'request_id', 'action', 'decision_sources', 'messages', 'recipients', 'reply_to',
            'static_map', 'order_sheet', 'own_rgb', 'own_commands', 'own_belief', 'message', 'null',
            'claim', 'continue', 'release', 'wait', 'order', 'destination_zone', 'kind', 'role',
            'assignments', 'unknown', 'text', 'long_beam', 'tile', 'sim_time_s', 'P1-2', 'seed',
            'commander', 'leader', 'follower', 'nav_cam', 'RGB', 'SIM', 'LLM', 'JSON', 'ID', 'act',
            'observed_at_sim_s', 'location_ref', 'passage', 'state', 'confidence', 'item', 'zone',
            'propose', 'request', 'accept', 'reject', 'inform', 'correct', 'yield', 'cancel',
            'suspected', 'clear', 'blocked', 'present', 'absent', 'held', 'placed', 'low', 'medium',
            'high', 'window_id', 'max_utterances', 'issued_orders', 'robot_views', 'wrist', 'note',
            'reason', 'schema', 'true', 'false')


@pytest.mark.parametrize('condition,rid,seed', CASES)
def test_prompt_is_korean_and_names_the_output_schema(condition, rid, seed):
    text = pk.system_prompt(condition, rid, seed=seed)
    assert hangul_ratio(text, LITERALS) > 0.9, condition
    for token in ('request_id', 'action', 'decision_sources', 'messages', 'A', 'B', 'C'):
        assert token in text
    assert 'TOP' in text                                     # explicitly denied, never supplied
    assert 'action과 messages는 분리됩니다' in text


def test_condition_blocks_say_exactly_what_the_channel_allows():
    assert 'messages는 반드시 빈 배열' in pk.system_prompt('no_comm', 'r1')
    assert '한국어로' in pk.system_prompt('peer_ko', 'r1')
    lead = pk.system_prompt('leader_ko', 'r2', seed=13)       # seed 13 -> leader r2
    follow = pk.system_prompt('leader_ko', 'r3', seed=13)
    assert 'leader는 당신입니다' in lead and 'follower끼리는 서로 말할 수 없으므로' in lead
    assert 'leader는 r2입니다' in follow and '다른 follower를 recipients에 넣은 메시지는' in follow
    assert 'r2에게만' in follow
    structured = pk.system_prompt('structured', 'r1')
    assert '자유 문장 없이' in structured and ', '.join(zp.STRUCT_ACTS) in structured
    assert 'text, reason, note' in structured
    commander = pk.system_prompt('reference_R', zp.COMMANDER)
    assert 'kind "order"' in commander and '참고 상한' in commander


def test_goal_and_input_boundary_text_is_identical_across_all_main_conditions():
    """The task and input-boundary text is byte-identical in all four main
    conditions; the role sentence and the channel/language blocks may differ."""
    bodies = set()
    for condition in ('no_comm', 'peer_ko', 'leader_ko', 'structured'):
        for rid in ROBOTS:
            head = pk.system_prompt(condition, rid, seed=12).split('통신:')[0]
            bodies.add(head.split('\n', 1)[1])        # drop only the identity/role sentence
    assert len(bodies) == 1, bodies
    # the language rule differs only between free text and the fixed schema
    assert pk.KO_LANGUAGE.strip() in pk.system_prompt('peer_ko', 'r1')
    assert pk.KO_LANGUAGE.strip() in pk.system_prompt('no_comm', 'r1')
    assert pk.KO_LANGUAGE.strip() in pk.system_prompt('leader_ko', 'r1', seed=12)
    assert pk.KO_LANGUAGE_STRUCT.strip() in pk.system_prompt('structured', 'r1')
    assert pk.KO_LANGUAGE.strip() not in pk.system_prompt('structured', 'r1')


@pytest.mark.parametrize('condition,rid,seed', CASES)
def test_every_call_carries_the_static_map_the_order_sheet_and_own_inputs(condition, rid, seed):
    channel = {'window': {'window_id': 'w1'}} if zp.spec(condition).channel_open else {}
    bundled = inputs(condition, rid, seed=seed)
    req = pk.build_request(bundled, **channel)
    body = json.loads(req['messages'][1]['content'])
    assert body['schema'] == pk.CONTRACT_PAYLOAD_SCHEMA
    assert body['static_map']['map_id'] == MAP_ID
    assert body['static_map']['public_map_sha256'] == bundle()['public_map_sha256']
    assert body['static_map']['map_file_sha256'] == bundle()['map_file_sha256']
    assert [o['order_id'] for o in body['order_sheet']['orders']] == ['order-1', 'order-2']
    assert body['request_id'] == 'req_1' and body['sim_time_s'] == 4.0
    assert req['input_sha256'] == bundled.payload_sha256
    labels = [i['label'] for i in req['images']]
    if rid == zp.COMMANDER:
        assert labels == ['WRIST RGB r1', 'WRIST RGB r2', 'WRIST RGB r3']
        assert 'self_belief' not in body and 'own_rgb_refs' not in body
        assert [r['ref'] for r in body['team_rgb_refs']] == [f'own-{r}-0042' for r in ROBOTS]
    else:
        assert labels == [pk.IMAGE_OWN]
        assert body['own_command_history'] and body['self_belief']['region'] == 'unknown'
        assert body['own_rgb_refs'][0]['ref'] == f'own-{rid}-0042'
    assert not any('TOP' in label for label in labels)
    assert 'top_cameras' not in req['messages'][1]['content']
    assert body['channel']['role'] == req['prompt_role']


def test_dialogue_window_only_exists_where_the_channel_does():
    for condition, rid in (('no_comm', 'r1'), ('reference_R', zp.COMMANDER)):
        body = json.loads(pk.build_request(inputs(condition, rid))['messages'][1]['content'])
        assert pk.WINDOW_KEY not in body
        # package A refuses an inbox for a condition that delivers nothing
        with pytest.raises(ContractViolation):
            build_call_input(robot_id=rid, condition_name=condition, request_id='req_1',
                             sim_time_s=4., static_map=static_map_section(bundle()),
                             source=source(), inbox=[{'message_id': 'm'}])
    t = transport('peer_ko')
    t.send('r2', recipients=['r1'], text=KO_TEXT, at_sim_s=10.)
    inbox = t.inbox('r1', now_sim_s=10.1)
    bundled = inputs('peer_ko', 'r1', inbox=inbox, sim_time_s=11.)
    body = json.loads(pk.build_request(bundled, window={'window_id': 'w1'})['messages'][1]['content'])
    window = body[pk.WINDOW_KEY]
    assert window['window_id'] == 'w1' and 'received' not in window
    assert body['inbox'] == [dict(inbox[0])]           # the delivered message is A's inbox
    assert (window['max_utterances'], window['max_your_utterances']) == (6, 2)
    assert window['your_utterances_left'] == 2


def test_the_inbox_is_package_a_envelopes_without_evaluation_fields():
    """A transport record carries no language flag and no delivery time."""
    t = transport('peer_ko')
    t.send('r2', recipients=['r1'], text=EN_TEXT, at_sim_s=10.)      # flagged, still delivered
    record = t.inbox('r1', now_sim_s=10.1)[0]  # noqa: F841 - checked below
    assert set(record) <= set(zp.INBOX_FIELDS)
    assert record['schema'] == 'ugrp.zone_study_message.v1'
    assert record['encoding'] == 'free_ko' and record['body'] == {'text': EN_TEXT}
    assert 'language' not in record and 'delivered_at_sim_s' not in record
    assert t.log[-1]['language']['flags']              # the flag stays in the evaluation log
    # an extra field cannot ride along: package A closes the envelope
    with pytest.raises(ContractViolation):
        payload('peer_ko', 'r1', sim_time_s=11.,
                inbox=[{**record, 'language': {'flags': ['non_korean']}}])


def test_commander_views_only_in_the_reference_condition():
    with pytest.raises(zp.ProtocolError):
        pk.StudyInputs(payload=payload('peer_ko'), wrist_jpeg=JPEG,
                       robot_views={r: JPEG for r in ROBOTS})
    with pytest.raises(zp.ProtocolError):
        pk.StudyInputs(payload=payload('reference_R', zp.COMMANDER), wrist_jpeg=JPEG)
    # package A drops the own camera for reference_R robots and gives them no payload
    with pytest.raises(ContractViolation):
        payload('reference_R', 'r1')


# --- the package A boundary seen from package C ----------------------------

def test_the_public_map_projection_carries_no_top_camera():
    public = bundle()['public_map']
    text = json.dumps(public, ensure_ascii=False)
    for token in ('top_camera', 'cctv', 'nav_cam', 'box_positions', 'robot_positions'):
        assert token not in text
    assert [p['id'] for p in public['passages']] == ['door_narrow', 'door_wide']
    assert inputs('no_comm').map_sha256 == bundle()['public_map_sha256']


def test_delivered_records_must_match_the_condition_encoding():
    free = transport('peer_ko')
    free.send('r2', recipients=['r1'], text=KO_TEXT, at_sim_s=10.)
    ko_record = free.inbox('r1', now_sim_s=10.1)[0]
    fixed = transport('structured')
    fixed.send('r2', recipients=['r1'], structured=struct(), at_sim_s=10.)
    schema_record = fixed.inbox('r1', now_sim_s=10.1)[0]
    with pytest.raises(ContractViolation):            # free text into the structured condition
        payload('structured', 'r1', sim_time_s=11., inbox=[ko_record])
    with pytest.raises(ContractViolation):            # structured body into the Korean condition
        payload('peer_ko', 'r1', sim_time_s=11., inbox=[schema_record])
    assert pk.build_request(inputs('structured', 'r1', sim_time_s=11., inbox=[schema_record]),
                            window={'window_id': 'w1'})


def test_commander_views_must_be_the_robot_roster():
    data = payload('reference_R', zp.COMMANDER)
    for views in ({'ground_truth_view': JPEG}, {'r1': JPEG, 'r2': JPEG}):
        with pytest.raises(zp.ProtocolError):
            pk.StudyInputs(payload=data, robot_views=views)
    assert pk.StudyInputs(payload=data, robot_views={r: JPEG for r in ROBOTS}).robot_views.keys() \
        == {'r1', 'r2', 'r3'}


def test_module_takes_no_host_decision_state():
    names = set()
    for func in (pk.build_request, pk.system_prompt):
        names.update(inspect.signature(func).parameters)
    assert not (names & set(zp.FORBIDDEN_TRANSPORT_PARAMS))


# --- one window end to end (offline, fixture replies) ---------------------

@pytest.mark.parametrize('condition,seed', [('no_comm', 4), ('peer_ko', 4), ('leader_ko', 13),
                                            ('structured', 4)])
def test_one_window_round_trip_per_condition(condition, seed):
    """request -> fixture reply -> validate -> relay -> inbox -> next request.

    A fixture reply is an explicit offline stand-in, never a fallback for a
    model failure, and the delivered messages only reach the named recipients.
    """
    s = zp.spec(condition)
    vocab = vocabulary(source().sheet(), bundle()['public_map'])
    t = zp.Transport(condition, seed=seed, vocabulary=vocab)
    t.open_window('w1', at_sim_s=10.)
    lead = zp.leader_of(condition, seed=seed)
    sent = 0
    for rid in ROBOTS:
        inbox = t.inbox(rid, now_sim_s=10.) if s.channel_open else None
        channel = dict(window={'window_id': 'w1'}) if s.channel_open else {}
        bundled = inputs(condition, rid, seed=seed, sim_time_s=10.,
                         **({'inbox': inbox} if s.channel_open else {}))
        request = pk.build_request(bundled, **channel)
        assert len(request['messages']) == 2 and len(request['images']) == 1
        target = lead if (lead and rid != lead) else next(b for b in ROBOTS if b != (lead or rid))
        messages = []
        if s.channel_open:
            body = {'recipients': [target], 'reply_to': None}
            body.update({'text': f'{target}에게 보고합니다. order-1을 확인했습니다.'}
                        if s.encoding == 'free_ko' else {'message': struct(act='inform')})
            messages = [body]
        reply = {'request_id': bundled.request_id, 'action': {'kind': 'wait'},
                 'decision_sources': ['static_map', 'own_rgb'], 'messages': messages}
        value = zp.validate_reply(json.dumps(reply, ensure_ascii=False), request_id=bundled.request_id,
                                 condition=condition, actor=rid, order_ids=bundled.order_ids(),
                                 item_ids=bundled.item_ids(),
                                 roles_by_order=bundled.roles_by_order(),
                                 passages=bundled.passages(), location_refs=bundled.location_refs())
        receipts = zp.relay(t, rid, value, at_sim_s=10. + 0.1 * len(ROBOTS))
        sent += sum(r.accepted for r in receipts)
    assert t.sent_count() == sent == (0 if not s.channel_open else 3)
    if not s.channel_open:
        assert all(t.inbox(rid, now_sim_s=99.) == () for rid in ROBOTS)
        return
    # the leader hears both followers; each robot only sees what was addressed to it
    if lead:
        assert len(t.inbox(lead, now_sim_s=99.)) == 2
    delivered = t.inbox(ROBOTS[0], now_sim_s=99.)
    after = pk.build_request(inputs(condition, ROBOTS[0], seed=seed, sim_time_s=99., inbox=delivered),
                             window={'window_id': 'w1'}, sent=t.sent_ids(ROBOTS[0], 'w1'))
    body = json.loads(after['messages'][1]['content'])
    window = body[pk.WINDOW_KEY]
    assert t.sent_ids(ROBOTS[0], 'w1') and window['your_utterances_left'] == 1
    assert all(set(m) <= set(zp.INBOX_FIELDS) for m in body['inbox'])
