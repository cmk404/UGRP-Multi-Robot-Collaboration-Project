"""The solo skill's loop guard is independent of a task's SIM expiry."""

import pytest

from harness.solo_box_transport import SoloBoxTransport


class _WaitingBox:
    phase = 'approach'

    def decide(self, _observation):
        return {'kind': 'wait', 'duration': .2}


def _waiting_skill(*, max_decisions=600):
    skill = SoloBoxTransport(robot_id='r2', navigator=object(),
                             max_decisions=max_decisions)
    skill.initialized = True
    skill.box = _WaitingBox()
    return skill


def test_legacy_solo_decision_limit_is_unchanged():
    skill = _waiting_skill()
    skill.steps = 599
    assert skill.decide({}, b'')[0]['kind'] == 'wait'
    assert skill.steps == 600
    with pytest.raises(RuntimeError, match='decision budget exhausted'):
        skill.decide({}, b'')


def test_explicit_longer_solo_decision_limit_keeps_loop_guard():
    skill = _waiting_skill(max_decisions=1200)
    skill.steps = 599
    assert skill.decide({}, b'')[0]['kind'] == 'wait'
    assert skill.steps == 600
    skill.steps = 1199
    assert skill.decide({}, b'')[0]['kind'] == 'wait'
    assert skill.steps == 1200
    with pytest.raises(RuntimeError, match='decision budget exhausted'):
        skill.decide({}, b'')


@pytest.mark.parametrize('limit', [0, -1, 10001, 1.5, True, '1200'])
def test_solo_decision_limit_must_be_finite_positive_integer(limit):
    with pytest.raises(ValueError, match='solo decision budget'):
        _waiting_skill(max_decisions=limit)
