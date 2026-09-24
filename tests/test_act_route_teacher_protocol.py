"""Boundary checks for the fixed three-teacher launch budget."""
import pytest

from scripts.run_act_route_teachers import (effective_wall_cap, managed_caps,
                                            managed_run_command)


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


def test_manager_timeout_precedes_outer_deadline_and_argv_records_it():
    outer, inner = managed_caps(4620, 1560)
    assert (outer, inner) == (1560, 1550)
    command = managed_run_command('/tmp/fresh-record', ['--variant', 'shared_crossing'], inner)
    assert command[:5] == ['bash', 'scripts/open_simulation.command', 'workflow', 'run', 'dispatch-skills']
    assert float(command[command.index('--timeout')+1]) == inner < outer
    assert command[-3:] == ['--', '--variant', 'shared_crossing']


def test_no_teacher_launch_if_manager_has_no_positive_inner_window():
    assert managed_caps(20, 1560) == (10, 0)
    assert managed_caps(21, 1560) == (11, 1)
    with pytest.raises(ValueError, match='nonpositive manager timeout'):
        managed_run_command('/tmp/fresh-record', [], 0)
