"""Boundary checks for the fixed three-teacher launch budget."""
import pytest

from scripts.run_act_route_teachers import effective_wall_cap


def test_aggregate_cap_shortens_last_case_without_skipping_it():
    assert effective_wall_cap(4620, 1560) == 1560
    assert effective_wall_cap(1500, 1560) == 1490
    assert effective_wall_cap(11, 1560) == 1
    assert effective_wall_cap(10, 1560) == 0
    assert effective_wall_cap(9, 1560) == 0


def test_invalid_budget_rejected_before_any_teacher_launch():
    with pytest.raises(ValueError, match='invalid teacher wall budget'):
        effective_wall_cap(100, 0)
    with pytest.raises(ValueError, match='invalid teacher wall budget'):
        effective_wall_cap(100, 1560, cleanup_reserve=-1)
