#!/usr/bin/env python3
"""2026-09-25 결과 중 TensorBoard 스냅샷이 없는 기록을 파생 뷰로 만든다.

원본 `experiments/2026-09-25-*/results.json`은 읽기만 하고, 각 행을
`scripts/export_tensorboard.py`가 읽을 수 있는 1실행 1폴더 `result.json`
파생 뷰로 복사한다. 수치는 원본에 있는 값만 옮기며 새로 계산하지 않는다.
`derived_view_only`, `offline_source`(경로+SHA-256), `condition`으로 출처와
코호트를 남긴다. 성공 bool은 원본의 심판/수락 판정 필드만 사용하고,
교사 실행기·gt_stub·사후 분석은 조건 문자열에 그대로 표시한다.

사용:
    python3 scripts/build_zone_supplement_views.py --output <파생뷰 루트>
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
EXP = REPO / 'experiments'

TEACHER = 'teacher_executor(ground-truth drive/IK, weld off); not an RGB/own-camera student result'
OFFLINE = 'offline evaluation of recorded frames/messages; no SIM ran in this view'


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path: Path):
    return json.loads(path.read_text())


def text(value) -> str:
    """Join a recorded scope field that may be a string or a list of strings."""
    if isinstance(value, list):
        return '; '.join(str(item) for item in value)
    return str(value)


class Builder:
    def __init__(self, output: Path):
        self.output = output
        self.rows: list[dict] = []

    def add(self, group: str, name: str, origin: Path, result: dict, pointer: str) -> None:
        """Write one derived view; `pointer` names the original JSON location."""
        result = dict(result)
        result['derived_view_only'] = True
        result['offline_source'] = {'path': str(origin), 'sha256': sha256(origin)}
        result['offline_source_pointer'] = pointer
        result['derived_from'] = f'{origin}#{pointer}'
        result['source_result_sha256'] = result['offline_source']['sha256']
        directory = self.output / group / name
        directory.mkdir(parents=True, exist_ok=False)
        (directory / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        self.rows.append({'group': group, 'run': f'{group}-{name}', 'source': str(directory),
                          'condition': result.get('condition'), 'origin': str(origin), 'pointer': pointer})

    # ---------------------------------------------------------------- 경로/문 지도
    def hard_routes(self) -> None:
        origin = EXP / '2026-09-25-zone-hard-routes/results.json'
        data = load(origin)
        short = {'zone_wide_corridor': 'corr', 'zone_wide_two_doors': 'two', 'zone_wide_door': 'door',
                 'zone_wide': 'wide'}
        coord = {'dynamic': 'dyn', 'independent': 'ind', 'plan_first': 'plan'}
        for cohort, key in (('routes', 'runs'), ('baseline', 'baseline_runs')):
            for i, run in enumerate(data[key]):
                variant, mode = run['variant'], run['coordination']
                name = f"{short.get(variant, variant)}-{run['goal']}-{coord.get(mode, mode)}-s{run['seed']}"
                self.add('hr', name, origin, {
                    'case': f"{variant}|{run['goal']}|{mode}",
                    'condition': f'hard-routes {cohort}: {variant} {run["goal"]} {mode} seed {run["seed"]}; '
                                 f'fixture protocol replies (0 LLM calls); {TEACHER}',
                    'policy': f'fixture-{mode}', 'seed': run['seed'], 'goal': run['goal'],
                    'clock': 'sim(synchronous)+wall', 'source_sha': run.get('source_sha'),
                    'success': run['goal_met_referee'], 'stop_reason': run['phase'], 'error': run.get('error'),
                    'sim_s': run['makespan_sim_s'], 'wall_s': run['wall_s'], 'model_calls': run['llm_calls'],
                    'scope': f'{data["claim_scope"]}; makespan_sim_s from first assignment; '
                             'success = referee zone counts',
                    'limits': 'one run per condition; host load recorded in the original record',
                    'evaluation': {k: run[k] for k in (
                        'goal_met_referee', 'goal_met_rgb', 'makespan_sim_s', 'control_end_sim_s', 'jobs_placed',
                        'teacher_path_blocked', 'goal_occupied', 'grasp_failed', 'box_taken_by_peer',
                        'dropped_in_transit', 'generic_yields', 'passage_standoffs', 'passage_fallbacks',
                        'passage_no_spot', 'door_wait_s', 'door_wait_by_passage_s', 'claim_collisions',
                        'invalid_claims', 'contacts', 'drive_wait_s', 'drive_wait_near_passage_s',
                        'load_1min_start', 'load_1min_end') if k in run},
                    'offline_scalar_scope': 'route traffic counters recorded by the evaluation-only referee',
                    'offline_scalars': {
                        'offline/control_end_sim_s': run['control_end_sim_s'],
                        'offline/door_wait_s': run['door_wait_s'],
                        'offline/claim_collisions': run['claim_collisions'],
                        'offline/teacher_path_blocked': run['teacher_path_blocked'],
                        'offline/box_taken_by_peer': run['box_taken_by_peer'],
                        'offline/generic_yields': run['generic_yields'],
                        'offline/passage_standoffs': run['passage_standoffs'],
                        'offline/goal_met_rgb': int(run['goal_met_rgb']),
                    },
                }, f'{key}[{i}]')

    # ------------------------------------------------------- 자기 카메라 위치 추정
    def owncam_loc(self) -> None:
        origin = EXP / '2026-09-25-zone-owncam-loc/results.json'
        data = load(origin)
        for section, cohort in (('preregistered_gates', 'preregistered'),
                                ('posthoc_look_modes_only', 'posthoc-look-only')):
            for split, gates in data[section].items():
                if split not in ('dev', 'test'):
                    continue
                for gate_id, gate in gates.items():
                    prefix = 'ph-' if cohort.startswith('posthoc') else ''
                    name = f'{prefix}{split}-{gate_id.split("_")[0]}'
                    preregistered = data['preregistered_gates'][split][gate_id]
                    scalars = {'offline/pos_err_m': gate['pos_m'], 'offline/yaw_err_deg': gate['yaw_deg'],
                               'offline/frames': gate['frames'],
                               'offline/quantile': preregistered['quantile']}
                    if 'uninitialized' in gate:
                        scalars['offline/uninitialized'] = gate['uninitialized']
                    self.add('loc', name, origin, {
                        'case': gate_id, 'policy': 'wrist_fisheye_pnp+particle_filter',
                        'condition': f'owncam localisation {cohort} {split}: {gate_id}; '
                                     'own-camera AprilTag estimate, evaluation-only GT comparison',
                        'clock': 'offline frames', 'success': bool(gate['pass']),
                        'stop_reason': 'gate_evaluated',
                        'scope': f'{OFFLINE}; {preregistered["meaning"]}; '
                                 + (f'post-hoc look-mode subset, outside the preregistered gate: '
                                    f'{data[section].get("note")}' if prefix
                                    else f'preregistered gate over modes {preregistered["modes"]}'),
                        'limits': 'frames of recorded episodes; localisation error, not transport success',
                        'evaluation': gate,
                        'offline_scalar_scope': 'recorded localisation error quantiles against evaluation-only GT',
                        'offline_scalars': scalars,
                    }, f'{section}.{split}.{gate_id}')

    # ------------------------------------------------------------ 한국어 대화 파일럿
    def dialogue_ko(self) -> None:
        origin = EXP / '2026-09-25-zone-dialogue-ko-pilot/results.json'
        data = load(origin)
        for variant, row in data['variants'].items():
            messages, tokens = row['messages'], row['tokens']
            scalars = {'offline/calls': row['n_calls'], 'offline/transport_errors': row['transport_errors'],
                       'offline/json_ok': row['json_ok'], 'offline/schema_ok': row['schema_ok'],
                       'offline/decision_same_as_original': row['decision_same_as_original'],
                       'offline/decision_same_as_v0': row['decision_same_as_v0'],
                       'offline/silent_messages': messages['silent'],
                       'offline/reason_hangul_ratio_mean': row['reason_hangul_ratio_mean']}
            # V3 is the fixed-schema control: it has no free-text Korean message fields.
            for tag, key in (('hangul_ratio_mean', 'hangul_ratio_mean'), ('korean_ge_0_9', 'korean_ge_0_9'),
                             ('code_switched', 'code_switched'), ('chars_mean', 'chars_mean'),
                             ('over_240_chars', 'over_240_chars'), ('fields_non_null', 'fields_non_null'),
                             ('json_chars_mean', 'json_chars_mean'),
                             ('struct_valid_replies', 'struct_valid_replies')):
                if isinstance(messages.get(key), (int, float)) and not isinstance(messages.get(key), bool):
                    scalars['offline/' + tag] = messages[key]
            self.add('ko', variant, origin, {
                'case': variant, 'policy': 'offline re-ask of recorded ZC2 decision points',
                'condition': f'Korean dialogue pilot {variant}; offline, no SIM, decisions not executed',
                'clock': 'none(offline)', 'stop_reason': 'offline_pilot_complete',
                'model_calls': row['n_calls'],
                'input_tokens': tokens['prompt_tokens']['sum'], 'output_tokens': tokens['completion_tokens']['sum'],
                'scope': data['scope'], 'limits': 'no success field exists; format/language metrics only',
                'evaluation': {k: row[k] for k in (
                    'json_ok', 'schema_ok', 'schema_errors', 'decision_same_as_v0', 'decision_changed_vs_v0',
                    'decision_same_as_original', 'claim_host_valid', 'claim_host_checked',
                    'plan_valid_when_present', 'messages', 'acts', 'reason_hangul_ratio_mean',
                    'unreported_tokens_mean', 'latency_ms_mean_not_a_speed_claim') if k in row},
                'offline_scalar_scope': 'recorded language/format rates of offline model replies; '
                                       'not task efficiency and not a speed claim',
                'offline_scalars': scalars,
            }, f'variants.{variant}')
        labels = data['manual_labels']
        self.add('ko', 'labels', origin, {
            'case': 'manual_act_labels', 'policy': 'rule labels vs one non-blind manual pass',
            'condition': 'Korean dialogue pilot act-label agreement; offline audit of V1/V2 messages',
            'clock': 'none(offline)', 'stop_reason': 'labels_compared',
            'scope': f'{OFFLINE}; labeller: {labels["labeller"]}',
            'limits': 'single non-blind labeller; agreement is not label correctness',
            'evaluation': {k: labels[k] for k in ('labeller', 'n', 'exact_match')},
            'offline_scalar_scope': 'recorded exact-match count between rule labels and the manual pass',
            'offline_scalars': {'offline/labelled': labels['n'], 'offline/exact_match': labels['exact_match']},
        }, 'manual_labels')

    # ------------------------------------------------- 화물 접촉 프로필(무슬립) 수락
    def cargo_noslip(self) -> None:
        origin = EXP / '2026-09-25-zone-cargo-catalogue/results-slip.json'
        data = load(origin)
        for section, label in (('static_hold_60s', 'hold'), ('long_route', 'route'),
                               ('fewer_robots', 'fewer')):
            for key, row in data['acceptance'][section].items():
                profile, item = key.split('-', 1)
                name = f'{label}-{item}-{"noslip" if profile == "cargo_noslip_v1" else "fine"}'
                hold = row.get('hold_slip_mm', {})
                self.add('slip', name, origin, {
                    'case': key, 'policy': profile,
                    'condition': f'cargo acceptance {section}: {item} with contact profile {profile}; {TEACHER}',
                    'clock': 'sim(synchronous)', 'success': row['success'],
                    'stop_reason': row['teacher_outcome'],
                    'scope': text(data['scope']) + f'; success = {section} acceptance criterion',
                    'limits': 'one run per condition; weld off; ground-truth teacher only',
                    'evaluation': row,
                    'offline_scalar_scope': 'recorded grip/placement measurements of the teacher acceptance run; '
                                           'max_hold_slip_mm is the larger of the recorded per-robot values and '
                                           'drop_event_rows counts the recorded drop events',
                    'offline_scalars': {
                        'offline/hold_measured_s': row['hold_measured_s'],
                        'offline/max_hold_slip_mm': max(hold.values()) if hold else 0.0,
                        'offline/placement_pos_err_m': row['placement']['pos_err_m'],
                        'offline/placement_yaw_err_deg': row['placement']['yaw_err_deg'],
                        'offline/max_robot_tilt_deg': row['max_robot_tilt_deg'],
                        'offline/drop_event_rows': len(row['drop_events']),
                        'offline/lifted_clear': int(row['lifted_clear']),
                    },
                }, f'acceptance.{section}.{key}')

    # ------------------------------------------------------------- TOP 화물 인식 v2
    def cargo_perception_v2(self) -> None:
        origin = EXP / '2026-09-25-zone-cargo-perception-v2/results.json'
        data = load(origin)
        for split, profiles in data['results'].items():
            for profile, views in profiles.items():
                merged = views['merged']
                scalars = {'offline/frames': merged['frames'],
                           'offline/false_positives_total': merged['false_positives_total']}
                for cls, row in merged['per_class'].items():
                    rate = row['full']['rate']
                    if isinstance(rate, (int, float)):
                        scalars[f'offline/full_rate/{cls}'] = rate
                targeted = views.get('targeted') or {}
                for key in ('parallel_beam_pairs_both_found_within_3cm', 'parallel_beam_pairs',
                            'extra_beam_reports'):
                    if isinstance(targeted.get(key), (int, float)):
                        scalars[f'offline/{key}'] = targeted[key]
                self.add('perc', f'cargo-{split}-{profile.replace("_", "-")}', origin, {
                    'case': f'{split}|{profile}', 'policy': profile,
                    'condition': f'TOP cargo perception {split} split, profile {profile}; '
                                 'offline evaluation of rendered frames, evaluation-only GT segmentation',
                    'clock': 'none(offline)', 'stop_reason': 'evaluation_complete',
                    'source_sha': data['source'].get('frozen_source_sha'),
                    'scope': f'{OFFLINE}; {text(data["scope"])}',
                    'limits': 'detection rates on rendered frames; no robot ran in this evaluation',
                    'evaluation': views,
                    'offline_scalar_scope': 'recorded per-class detection rates and false positives '
                                            '(merged TOP views); not robot success',
                    'offline_scalars': scalars,
                }, f'results.{split}.{profile}')

    # ------------------------------------------------------------- 구역 상자 4색 RGB
    def rgb_color(self) -> None:
        origin = EXP / '2026-09-25-zone-rgb-color/results.json'
        data = load(origin)
        scene = 'primary_zone_wide'
        for split, profiles in data['results'].items():
            for profile, scenes in profiles.items():
                for view in ('own', 'top_merged'):
                    row = scenes[scene][view]
                    scalars = {'offline/frames': row['frames'],
                               'offline/false_positives_total': row['false_positives_total'],
                               'offline/false_positives_per_frame': row['false_positives_per_frame']}
                    for colour, per in row['per_kind'].items():
                        rate = per['full']['rate']
                        if isinstance(rate, (int, float)):
                            scalars[f'offline/full_rate/{colour}'] = rate
                    self.add('perc', f'colour-{split}-{profile}-{view.replace("_", "-")}', origin, {
                        'case': f'{split}|{profile}|{view}', 'policy': data['profiles'][profile][
                            'own' if view == 'own' else 'top'],
                        'condition': f'zone colour-box detection {split} split, {profile} profile, {view} view '
                                     f'on {scene}; offline evaluation of rendered frames',
                        'clock': 'none(offline)', 'stop_reason': 'evaluation_complete',
                        'source_sha': data['source']['score_source_sha'].get(split),
                        'scope': f'{OFFLINE}; ' + text(data['scope_and_limitations']),
                        'limits': 'detection rates only; the teacher executor was not replaced in this record',
                        'evaluation': row,
                        'offline_scalar_scope': 'recorded per-colour detection rates and false positives; '
                                                'not robot success',
                        'offline_scalars': scalars,
                    }, f'results.{split}.{profile}.{scene}.{view}')


def rename_snapshot(snapshot: Path, index: Path) -> dict:
    """Give the exported runs short names and keep the exporter's name and condition.

    TensorBoard uses the directory name as the run label, so only the label
    changes; each run keeps its own manifest and the original source path.
    """
    rows = {row['source']: row for row in load(index)['rows']}
    collection = load(snapshot / 'collection.json')
    for entry in collection['exported']:
        row = rows.get(entry['source'])
        if row is None:
            raise SystemExit('exported run is not in the derived-view index: ' + entry['source'])
        current, target = snapshot / entry['name'], snapshot / row['run']
        if current.resolve() != target.resolve():
            if target.exists():
                raise SystemExit('short run name already exists: ' + str(target))
            current.rename(target)
        entry['original_name'] = entry.pop('name')
        entry['name'] = row['run']
        entry['condition'] = row['condition']
        entry['origin'] = row['origin']
        entry['pointer'] = row['pointer']
    (snapshot / 'collection.json').write_text(json.dumps(collection, ensure_ascii=False, indent=2) + '\n')
    return {'renamed': len(collection['exported']), 'failed': len(collection.get('failed') or [])}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='new derived-view root (must not exist)')
    parser.add_argument('--rename-snapshot', type=Path,
                        help='exported snapshot directory to relabel with short run names')
    parser.add_argument('--index', type=Path, help='derived-view index.json for --rename-snapshot')
    args = parser.parse_args()
    if args.rename_snapshot:
        if not args.index:
            parser.error('--rename-snapshot needs --index')
        print(json.dumps(rename_snapshot(args.rename_snapshot.resolve(), args.index.resolve()),
                         ensure_ascii=False))
        return 0
    if not args.output:
        parser.error('--output is required')
    output = args.output.resolve()
    if output.exists():
        parser.error('derived-view root must be new; originals and earlier views are never overwritten')
    builder = Builder(output)
    builder.hard_routes()
    builder.owncam_loc()
    builder.dialogue_ko()
    builder.cargo_noslip()
    builder.cargo_perception_v2()
    builder.rgb_color()
    index = {'schema': 'ugrp.zone_supplement_views.v1',
             'purpose': '2026-09-25 결과 중 스냅샷이 없던 기록의 TensorBoard 파생 뷰',
             'rows': builder.rows}
    (output / 'index.json').write_text(json.dumps(index, ensure_ascii=False, indent=1) + '\n')
    print(json.dumps({'views': len(builder.rows), 'root': str(output),
                      'groups': sorted({row['group'] for row in builder.rows})}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
