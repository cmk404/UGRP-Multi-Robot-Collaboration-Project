"""Strict artifact checks for the fixed solo-41 pilot; not a visual-review claim."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

EXPECTED = {'seed': 41, 'robots': 1, 'seconds': 300, 'max_calls': 30,
            'max_input_tokens': 60000, 'model': 'gemini-3.8-flash',
            'communication': 'none', 'impratio': 10, 'noslip_iterations': 3,
            'record': True, 'max_transient_failures': 2}


def _json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def _jsonl(path):
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines() if line.strip()]


def _file(root, relative):
    path = (root / relative).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError('artifact path escapes run directory')
    return path


def _video_ok(path):
    import cv2
    cap = cv2.VideoCapture(str(path))
    try:
        if not cap.isOpened():
            return False
        frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        ok_first, _ = cap.read()
        if frames <= 1 or not ok_first:
            return False
        cap.set(cv2.CAP_PROP_POS_FRAMES, frames - 1)
        ok_last, _ = cap.read()
        return bool(ok_last)
    finally:
        cap.release()


def _positive_int(value):
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError('max input tokens must be a positive integer') from exc
    if isinstance(value, bool) or parsed <= 0 or str(value).strip() != str(parsed):
        raise argparse.ArgumentTypeError('max input tokens must be a positive integer')
    return parsed


def verify_run(root: Path, *, max_input_tokens: int = 60000,
               expected_seed: int = 41) -> dict:
    """Recompute accounting and require original input files, release and finish.

    Passing means only automated artifact consistency. Inspect the actual video
    and owned-camera sequence before claiming physically sensible delivery.
    """
    if (not isinstance(max_input_tokens, int) or isinstance(max_input_tokens, bool)
            or max_input_tokens <= 0):
        raise ValueError('max_input_tokens must be a positive integer')
    if not isinstance(expected_seed, int) or isinstance(expected_seed, bool):
        raise ValueError('expected_seed must be an integer')
    expected = {**EXPECTED, 'max_input_tokens': max_input_tokens, 'seed': expected_seed}
    checks = {}
    details = {}
    try:
        config = _json(root / 'run-config.json')
        result = _json(root / 'result.json')
        source = _json(root / 'source-manifest.json')
        rows = _jsonl(root / 'llm-decisions.jsonl')
        commands = _jsonl(root / 'commands.jsonl')
        checks['fixed_pilot_config'] = all(config.get(k) == v for k, v in expected.items())
        checks['reported_fixed_budgets'] = all(result.get('budgets', {}).get(k) == v for k, v in {
            'sim_seconds': 300, 'max_calls_per_robot': 30,
            'max_input_tokens_per_robot': max_input_tokens}.items())
        checks['single_robot_seed_and_mode'] = (result.get('active_robots') == ['r1']
            and result.get('seed') == expected_seed and result.get('communication') == 'none'
            and result.get('model') == EXPECTED['model'])
        checks['source_manifest_hash'] = (bool(source) and hashlib.sha256(
            json.dumps(source, sort_keys=True).encode()).hexdigest() == result.get('source_hash'))
        requests = [r for r in rows if r.get('event') == 'llm_request']
        results = [r for r in rows if r.get('event') == 'llm_result']
        # Older recordings predate this explicit request field. For newly
        # configured runs, every submitted call must retain the same setting.
        if 'reasoning_effort' in result:
            effort = result['reasoning_effort']
            checks['requested_reasoning_effort_consistent'] = (
                effort in ('none', 'low', 'medium', 'high')
                and bool(requests)
                and all(r.get('requested_reasoning_effort') == effort for r in requests))
        req = {r['call_id']: r for r in requests}
        res = {r['call_id']: r for r in results}
        checks['no_missing_or_duplicate_call_accounting'] = (bool(req)
            and len(req) == len(requests) and len(res) == len(results) and set(req) == set(res)
            and len(req) == result.get('llm_calls', {}).get('r1') and len(req) <= 30)
        total = 0; unknown = 0; models = set(); frames_ok = True; actual_replies = 0
        for call_id, row in res.items():
            if row.get('disposition') == 'cancelled_before_start':
                continue
            audit = row.get('audit') or {}
            usage = audit.get('usage') or {}
            tokens = usage.get('prompt_tokens')
            if isinstance(tokens, int) and not isinstance(tokens, bool) and tokens >= 0:
                total += tokens
            else:
                unknown += 1
            model = audit.get('response_model')
            if model is not None:
                models.add(model)
            if isinstance(audit.get('raw_text'), str) and audit['raw_text'].strip():
                actual_replies += 1
            request = req.get(call_id, {})
            original = set()
            for entry in request.get('images', []):
                raw = _file(root, entry['path']).read_bytes()
                digest = hashlib.sha256(raw).hexdigest()
                frames_ok &= digest == entry['sha256']
                original.add(digest)
            audit_frames = [f for f in audit.get('input_frames', []) if f.get('label') in ('CURRENT_WRIST', 'CURRENT_NAV')]
            frames_ok &= (len(request.get('images', [])) == 2 and len(audit_frames) == 2
                          and {f.get('sha256') for f in audit_frames} == original
                          and all(f.get('transmitted_sha256') == f.get('sha256') for f in audit_frames))
            frames_ok &= request.get('robot_id') == row.get('robot_id') == 'r1'
        reported = result.get('input_usage', {}).get('r1', {})
        checks['actual_model_reply_metadata'] = (actual_replies > 0 and len(models) == 1
            and all(isinstance(m, str) and (m == EXPECTED['model'] or m.startswith(EXPECTED['model'] + '-')) for m in models))
        checks['original_own_rgb_hashes'] = bool(frames_ok)
        # Every historical image must come from an earlier input of this robot.
        # Checking only CURRENT_* would leave the third image unaudited.
        references_ok = True
        prior_nav_hashes = set()
        reference_count = 0
        for request in requests:
            audit = (res.get(request['call_id'], {}).get('audit') or {})
            frames = audit.get('input_frames', [])
            historical = [f for f in frames
                          if f.get('label') not in ('CURRENT_WRIST', 'CURRENT_NAV')]
            references_ok &= len(frames) <= 3 and len(historical) <= 1
            for frame in historical:
                reference_count += 1
                digest = frame.get('sha256')
                references_ok &= (frame.get('label') in ('PREVIOUS_NAV', 'RECENT_BLOCKED_NAV',
                                                        'LAST_TRANSLATION_NAV')
                    and frame.get('camera') == 'nav_cam'
                    and (request.get('robot_id'), digest) in prior_nav_hashes
                    and frame.get('transmitted_sha256') == digest)
                if frame.get('label') in ('RECENT_BLOCKED_NAV', 'LAST_TRANSLATION_NAV'):
                    reference = (audit.get('model_context') or {}).get('navigation_reference') or {}
                    references_ok &= all(reference.get(k) == frame.get(k)
                                         for k in ('frame_id', 'sim_time', 'sha256'))
            prior_nav_hashes.update((request.get('robot_id'), entry.get('sha256'))
                                    for entry in request.get('images', [])
                                    if entry.get('camera') == 'nav')
        checks['historical_rgb_from_prior_own_nav'] = bool(references_ok)
        completed_macros = [c for c in commands if isinstance(c.get('execution'), dict)]
        feedback_matches = bool(completed_macros)
        for request in requests:
            prior = [c for c in completed_macros
                     if c.get('robot_id') == request.get('robot_id')
                     and c.get('time', math.inf) <= request.get('time', -math.inf)]
            if not prior:
                continue
            latest = prior[-1]
            expected_feedback = {'decision_id': latest.get('decision_id'), **latest['execution']}
            reported_feedback = (request.get('execution_feedback') or {}).get('last_macro')
            audit_context = (res.get(request['call_id'], {}).get('audit') or {}).get('model_context') or {}
            transmitted_feedback = (audit_context.get('execution_feedback') or {}).get('last_macro')
            feedback_matches &= reported_feedback == expected_feedback == transmitted_feedback
        checks['execution_results_reach_model_context'] = feedback_matches
        token_check = f'known_reconciled_input_within_{max_input_tokens}'
        checks[token_check] = (unknown == 0 and total <= max_input_tokens
            and reported.get('reported_prompt_tokens') == total
            and reported.get('calls_without_usage') == 0
            and reported.get('unsettled_reservations') == 0)
        accepted = [r for r in results if r.get('disposition') == 'accepted']
        kinds = [(r.get('decision') or {}).get('action', {}).get('kind') for r in accepted]
        approach_index = next((i for i, kind in enumerate(kinds) if kind == 'approach'), None)
        pick_index = next((i for i, kind in enumerate(kinds)
                           if kind == 'pick' and approach_index is not None and i > approach_index), None)
        release_index = next((i for i, kind in enumerate(kinds)
                              if kind == 'release' and pick_index is not None and i > pick_index
                              and any(kinds[j] == 'drive' for j in range(pick_index + 1, i))), None)
        finish_index = next((i for i, kind in enumerate(kinds)
                             if kind == 'finish' and release_index is not None and i > release_index), None)
        checks['model_selected_full_chain'] = finish_index is not None
        outcome = result.get('outcomes', {}).get('r1', {})
        physical_gates = ('lift', 'inside_destination', 'stable', 'no_attachment_constraint', 'visual_release')
        checks['task_and_visual_release_gates'] = (result.get('success') is True and outcome.get('success') is True
            and outcome.get('reason') == 'VISUAL_RELEASE_CONFIRMED'
            and all(outcome.get('gates', {}).get(k) is True for k in physical_gates)
            and not result.get('error'))
        elapsed = result.get('decision_elapsed_sim_s', math.inf)
        passive = result.get('passive_settle_s', math.inf)
        checks['same_active_time_and_existing_passive_settle'] = (
            0 <= elapsed <= 300.002001 and 0 <= passive <= 1.002001)
        stop_time = result.get('episode_stop_sim_time', -1)
        checks['no_motion_command_after_stop'] = (stop_time >= 0 and not any(
            r.get('event') == 'raw_action' and r.get('time', math.inf) > stop_time + 1e-8
            and r.get('raw_action', {}).get('kind') in {'drive', 'arm', 'look'} for r in commands))
        checks['video_decodes_first_and_last_frame'] = _video_ok(root / 'motion-1x.mp4')
        details = {'requests': len(req), 'expected_seed_verified': expected_seed,
                   'historical_reference_images_verified': reference_count,
                   'max_input_tokens_verified': max_input_tokens,
                   'reported_input_tokens_recomputed': total,
                   'calls_without_usage_recomputed': unknown, 'response_models': sorted(models),
                   'decision_elapsed_sim_s': elapsed, 'passive_settle_s': passive}
    except (OSError, ValueError, TypeError, KeyError, ImportError) as exc:
        checks['artifacts_readable'] = False
        details['error_type'] = type(exc).__name__
        details['error'] = str(exc)[:300]
    passed = bool(checks) and all(checks.values())
    return {'status': 'AUTOMATED_CHECKS_PASS_VISUAL_REVIEW_REQUIRED' if passed else 'NOT_VERIFIED',
            'automated_checks_passed': passed, 'visual_review_completed': False,
            'checks': checks, 'details': details,
            'note': 'Tests and artifact consistency are not a substitute for real-run camera/video review.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--max-input-tokens', type=_positive_int, default=60000)
    parser.add_argument('--seed', type=int, default=41)
    args = parser.parse_args()
    report = verify_run(args.run, max_input_tokens=args.max_input_tokens,
                        expected_seed=args.seed)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['automated_checks_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
