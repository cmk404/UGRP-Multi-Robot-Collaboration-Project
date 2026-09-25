"""M1 input contract and outcome fields for the own-camera zone skill (Codex review issue 1).

M1 (user decision 2026-09-25): one robot delivers one cyan box through the door
using ONLY its own ``robot_cam`` RGB, the static map and its own issued command
history. A ground-truth pose stub may drive a *diagnostic* skill-isolation run,
but such a run is never an M1 result.

This module is the single place that decides:

* which pose-source labels an M1 run may use (allow-list: own-camera
  estimators only; everything else, including ``gt_stub_eval_only``, is
  rejected),
* the REQUIRED outcome fields every result and every derived result carries:
  ``mode``, ``counts_as_m1``, ``m1_success``, ``diagnostic_success``,
  ``pose_source``, ``pose_sources_seen`` and ``input_contract``.

The executor (skill), the judge (placement verdict / runner evaluation) and the
exporter (derived TensorBoard results) all call it, so a GT pose cannot turn
into an M1 success anywhere downstream.
"""
from __future__ import annotations

import re
from typing import Any, Iterable, Mapping

MODES = ('diagnostic', 'm1')
GT_POSE_SOURCES = frozenset({'gt_stub_eval_only'})
# Allow-list for M1: estimators that use only own robot_cam RGB + static map + own commands.
# 'owncam_pf' = the #178/#197 wall/door-tag particle filter (PoseReport source 'owncam_pf_v2:<calib sha8>',
# agreed on PR #181 2026-09-25).
M1_POSE_SOURCE_PREFIXES = ('own_rgb_', 'owncam_pf')
_FORBIDDEN_TOKENS = ('gt', 'truth', 'sim_state', 'qpos', 'top', 'nav_cam', 'cctv')
REQUIRED_OUTCOME_KEYS = ('mode', 'counts_as_m1', 'm1_success', 'diagnostic_success', 'pose_source',
                         'pose_sources_seen', 'input_contract', 'success_semantics')
SUCCESS_SEMANTICS = ('m1_success: counts_as_m1 AND evaluation-only GT placement in the ordered slot AND the '
                     'skill own-RGB claim IN_SLOT. diagnostic_success: the same placement test for a run that '
                     'may use a GT pose stub (skill isolation); never an M1 result. Derived results set '
                     '"success" = m1_success.')


class ContractViolation(ValueError):
    """An input or claim that the M1 contract forbids."""


def is_m1_pose_source(label: Any) -> bool:
    if not isinstance(label, str) or label in GT_POSE_SOURCES:
        return False
    low = label.lower()
    if not low.startswith(M1_POSE_SOURCE_PREFIXES):
        return False
    tokens = set(re.split(r'[^a-z0-9]+', low))
    return not any(tok in tokens for tok in _FORBIDDEN_TOKENS) and 'nav_cam' not in low


def require_m1_pose_source(label: Any, where: str) -> None:
    if not is_m1_pose_source(label):
        raise ContractViolation(f'M1 mode rejects pose source {label!r} at {where}')


def check_mode(mode: Any) -> str:
    if mode not in MODES:
        raise ContractViolation(f'mode must be one of {MODES}, got {mode!r}')
    return mode


def outcome_fields(*, mode: str, pose_sources_seen: Iterable[str], diagnostic_success: bool,
                   input_contract: Mapping[str, Any], cameras_seen: Iterable[str] = ('robot_cam',)) -> dict[str, Any]:
    """The required outcome block. ``m1_success`` can only be true for a clean M1 run."""
    check_mode(mode)
    sources = sorted(set(pose_sources_seen))
    cameras = sorted(set(cameras_seen))
    counts = (mode == 'm1' and bool(sources) and all(is_m1_pose_source(s) for s in sources)
              and cameras == ['robot_cam'])
    return {'mode': mode, 'counts_as_m1': bool(counts), 'm1_success': bool(counts and diagnostic_success),
            'diagnostic_success': bool(diagnostic_success),
            'pose_source': sources[0] if len(sources) == 1 else sources,
            'pose_sources_seen': sources, 'cameras_seen': cameras,
            'input_contract': dict(input_contract), 'success_semantics': SUCCESS_SEMANTICS}


def validate_outcome(result: Mapping[str, Any]) -> None:
    """Raise if a (derived) result lacks the outcome fields or claims M1 with a forbidden input."""
    missing = [k for k in REQUIRED_OUTCOME_KEYS if k not in result]
    if missing:
        raise ContractViolation(f'result lacks required outcome fields {missing}')
    check_mode(result['mode'])
    sources = list(result['pose_sources_seen'])
    clean = result['mode'] == 'm1' and bool(sources) and all(is_m1_pose_source(s) for s in sources)
    if result['counts_as_m1'] and not clean:
        raise ContractViolation(f'counts_as_m1 with pose sources {sources} in mode {result["mode"]}')
    if result['m1_success'] and not result['counts_as_m1']:
        raise ContractViolation('m1_success without counts_as_m1')
    if result['m1_success'] and not result['diagnostic_success']:
        raise ContractViolation('m1_success without the placement test passing')
    if 'success' in result and bool(result['success']) != bool(result['m1_success']):
        raise ContractViolation('"success" must equal m1_success in results carrying the M1 contract')
