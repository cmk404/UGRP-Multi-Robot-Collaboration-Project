"""M1 contract: what an M1 run may use and how its success is recorded.

Codex review #1/#2: an M1 executor, judge and exporter must reject any pose
source that is not the own-camera estimator (in particular
``gt_stub_eval_only``), must validate every observation handed to the skill
(own ``robot_cam``, own robot id, fresh frame, hash of the JPEG bytes), and
must keep diagnostic success and M1 success as separate fields that survive
every derived export.
"""
from __future__ import annotations

import base64
import hashlib
from collections.abc import Iterable, Mapping

M1_SOURCE_PREFIX = 'owncam_pf'
FORBIDDEN_SOURCES = ('gt_stub_eval_only', 'gt', 'truth', 'sim')
M1_REQUIRED_FIELDS = ('counts_as_m1', 'm1_success', 'diagnostic_success', 'pose_sources_seen',
                      'input_contract', 'm1_failed_checks')


class M1ContractError(RuntimeError):
    pass


def require_m1_source(label: str) -> str:
    if not isinstance(label, str) or not label.startswith(M1_SOURCE_PREFIX) or \
            any(label == f or label.startswith(f + ':') for f in FORBIDDEN_SOURCES):
        raise M1ContractError(f'M1 rejects pose source {label!r}')
    return label


def validate_observation(obs: Mapping, *, robot_id: str, previous_frame_id: int | None,
                         now: float, max_age_s: float = .25) -> None:
    """Own wrist camera, own robot, a new frame, recent, and the hash matches the bytes."""
    if obs.get('camera') != 'robot_cam':
        raise M1ContractError(f"M1 observation camera {obs.get('camera')!r} is not robot_cam")
    if obs.get('robot_id') != robot_id:
        raise M1ContractError(f"M1 observation robot {obs.get('robot_id')!r} is not {robot_id}")
    frame_id = obs.get('frame_id')
    if previous_frame_id is not None and (not isinstance(frame_id, int) or frame_id <= previous_frame_id):
        raise M1ContractError('M1 observation is not a new frame')
    if now - float(obs.get('sim_time', -1e9)) > max_age_s:
        raise M1ContractError('M1 observation is stale')
    jpeg = base64.b64decode(obs['image'])
    if hashlib.sha256(jpeg).hexdigest() != obs.get('sha256'):
        raise M1ContractError('M1 observation hash mismatch')


def judge(*, pose_sources: Iterable[str], skill_reason: str, skill_claim_in_slot: bool,
          gt_box_in_slot: bool, wall_contacts: int, weld_used: bool, face_fallback_used: bool,
          pickup_source: str, within_limit: bool, extra_checks: Mapping[str, bool] | None = None) -> dict:
    """Separate diagnostic and M1 success; every failed M1 check is named."""
    sources = sorted(set(pose_sources))
    checks = {
        'all_pose_sources_owncam': bool(sources) and all(s.startswith(M1_SOURCE_PREFIX) for s in sources),
        'pickup_from_own_rgb': pickup_source == 'own_rgb_search',
        'skill_claims_in_slot': bool(skill_claim_in_slot),
        'gt_box_in_slot': bool(gt_box_in_slot),
        'no_wall_contact': wall_contacts == 0,
        'weld_off': not weld_used,
        'no_face_normal_map_fallback': not face_fallback_used,
        'within_sim_limit': bool(within_limit),
        **{k: bool(v) for k, v in (extra_checks or {}).items()},
    }
    failed = [k for k, ok in checks.items() if not ok]
    diagnostic = bool(gt_box_in_slot and skill_claim_in_slot)
    return {'counts_as_m1': checks['all_pose_sources_owncam'] and checks['pickup_from_own_rgb'],
            'm1_success': not failed, 'm1_failed_checks': failed, 'm1_checks': checks,
            'diagnostic_success': diagnostic, 'false_success': bool(skill_claim_in_slot and not gt_box_in_slot),
            'pose_sources_seen': sources, 'skill_reason': skill_reason}


def assert_exportable(result: Mapping) -> None:
    """Exporter guard: derived results must keep the M1 fields and never upgrade a non-M1 run."""
    missing = [k for k in M1_REQUIRED_FIELDS if k not in result]
    if missing:
        raise M1ContractError(f'M1 export needs {missing}')
    if result['m1_success'] and not result['counts_as_m1']:
        raise M1ContractError('m1_success without counts_as_m1')
    if result['m1_success']:
        for s in result['pose_sources_seen']:
            require_m1_source(s)
