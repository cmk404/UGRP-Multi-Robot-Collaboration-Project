#!/usr/bin/env python3
"""PR #193 자기 카메라 인식 채점과 PR #189 `cargo_noslip_v1` 부작용 감사를 TensorBoard 파생 뷰로 만든다.

두 실험의 기록(`experiments/...`)과 원본(`outputs/...`)은 **읽기만** 한다. 실행·재채점·재감사를
하지 않고, 원본 JSON에 있는 값만 옮긴다. 각 파생 뷰는 원본 파일의 절대 경로와 SHA-256을 선언하며
`scripts/export_offline_audit.py`가 그 해시를 다시 확인한 뒤에만 이벤트를 만든다.

확인하는 것:

- 채점 원본 `<split>-score*/summary.json`이 커밋된 `experiments/.../<split>-summary.json`과
  **바이트 단위로 같은지**.
- 감사 원본 42개 실행의 `result.json`·`trace.jsonl`·`events.json`·`hashes.json`·`scene.xml`
  210개 파일 해시가 `results.json`의 `raw_files`와 같은지, 실행별 `metrics`가 기록의 사본과 같은지.
- 각 실험 README가 본문에 적은 수치·게이트 판정이 원본 JSON과 같은지(`CROSS_CHECKS`).
  다른 값은 `KNOWN_RECORD_DIFFS`에 적은 것만 허용하고, 그 밖의 불일치는 변환을 중단시킨다.

만들지 않는 것: 보간·평활·재계산, 조건 합산, 임무 성공 판정. 인식 채점은 오프라인 평가이고
접촉 감사는 정답 교사 조건의 물리 측정이다. 둘 다 로봇 임무 성공률이 아니다.

사용:
    python3 scripts/build_perception_noslip_views.py \
        --perception-records <PR193 experiments/2026-09-26-zone-own-perception> \
        --perception-raw <PR193 outputs/2026-09-26-zone-own-perception> \
        --noslip-records <PR189 experiments/2026-09-26-noslip-side-effects> \
        --noslip-raw <PR189 outputs/noslip-audit/5578f09> \
        --output <새 파생 뷰 루트>
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re

SCHEMA = 'ugrp.perception_noslip_views.v1'
VIEW_SCHEMA = 'ugrp.offline_audit_view.v1'

# ---------------------------------------------------------------- 인식 채점 (PR #193)
PERCEPTION_SPLITS = {'dev': 'dev-score-clean', 'test': 'test-score'}
PERCEPTION_FRAMES = {'dev': 'dev-frames-clean', 'test': 'test-frames'}
JUDGMENTS = {'slot_item': 'slot', 'holding_item': 'hold',
             'placed_in_slot': 'placed', 'route_blockage': 'block'}
BELIEFS = {'gt_stub_eval_only': 'gt', 'noise_30mm': 'n30', 'noise_60mm': 'n60'}
PRIMARY_BELIEF = 'gt_stub_eval_only'
CONFIDENT_THRESHOLD = 0.65

# 사전 등록 게이트(PR #193 README "사전 등록한 게이트"). test·primary belief·뷰 단위에만 적용한다.
GATES = {
    'g1_confident_errors_le_1': ('판단별 확신 오류 ≤ 1뷰', 1.0),
    'g2_observable_confident_errors_zero': ('placed_in_slot·holding_item 관측 가능 뷰 확신 오류 0', 0.0),
    'g3_decided_accuracy_ge_0_85': ('판단별 결정한 뷰 정확도 ≥ 0.85', 0.85),
    'g4_observable_accuracy_ge_0_70': ('판단별 관측 가능 뷰 정확도 ≥ 0.70', 0.70),
    'g5_observable_unknown_le_0_35': ('판단별 관측 가능 뷰 unknown 비율 ≤ 0.35', 0.35),
    'g6_placed_zone_b_yes_ge_3': ('placed_zone_B 4뷰 중 ≥ 3뷰 yes (518 거짓 음성 재현 안 됨)', 3.0),
    'g7_not_observable_confident_errors_le_1': ('관측 불가 뷰 전체 확신 오류 ≤ 1', 1.0),
    'g8_unit_tests_pass': ('tests/test_zone_own_perception.py 전체 통과', 1.0),
}
G2_JUDGMENTS = ('placed_in_slot', 'holding_item')
# G8은 PR #193 README가 적은 20/20 통과를 그대로 옮긴 주장이며 이 스냅샷이 재실행하지 않았다.
G8_RECORDED = {'pass': True, 'tests': 20, 'passed': 20,
               'source': 'PR #193 experiments/2026-09-26-zone-own-perception/README.md',
               'reverified_by_this_snapshot': False}

# README 표(뷰 단위·primary belief)와 원본 summary.json이 같은지 확인한다. 값은 소수 3자리 비교다.
PERCEPTION_CROSS_CHECKS = {
    'test': {'slot_item': (24, .458, .542, 1.00, 0, 15, .867, .133, 0),
             'holding_item': (20, .400, .600, 1.00, 0, 16, .750, .250, 0),
             'placed_in_slot': (24, .333, .667, 1.00, 0, 20, .800, .200, 0),
             'route_blockage': (20, .250, .750, 1.00, 0, 20, .750, .250, 0)},
    'dev': {'slot_item': (24, .333, .625, .938, 1, 16, .938, None, 0),
            'holding_item': (20, .300, .700, 1.00, 0, 16, .875, None, 0),
            'placed_in_slot': (24, .208, .792, 1.00, 0, 17, .882, None, 0),
            'route_blockage': (20, .250, .750, 1.00, 0, 20, .750, None, 0)},
}
# README belief 민감도 표: (뷰 단위 전체 정확도, 확신 오류).
BELIEF_CROSS_CHECKS = {
    ('test', 'noise_30mm'): {'slot_item': (.625, 0), 'holding_item': (.600, 0),
                             'placed_in_slot': (.542, 0), 'route_blockage': (.750, 0)},
    ('test', 'noise_60mm'): {'slot_item': (.333, 1), 'holding_item': (.600, 0),
                             'placed_in_slot': (.375, 5), 'route_blockage': (.500, 2)},
    ('dev', 'noise_30mm'): {'slot_item': (.542, 0), 'holding_item': (.700, 0),
                            'placed_in_slot': (.708, 0), 'route_blockage': (.700, 2)},
    ('dev', 'noise_60mm'): {'slot_item': (.292, 2), 'holding_item': (.700, 0),
                            'placed_in_slot': (.375, 4), 'route_blockage': (.450, 1)},
}

# ---------------------------------------------------------------- 접촉 감사 (PR #189)
SCENARIOS = {'drive_commands': 'drive', 'wall_push': 'wall', 'arm_sweep': 'arm',
             'idle_settle': 'idle', 'solo_carry': 'carry', 'solo_hold_load': 'load10x',
             'pair_beam_hold': 'pair'}
PROFILES = {'local_contact_fine': 'base', 'cargo_noslip_v1': 'nos'}
CARGO_SCENARIOS = ('solo_carry', 'solo_hold_load', 'pair_beam_hold')
NOSLIP_TOTALS = {'runs': 42, 'gates': 327, 'failed': 3}

# README 6장이 적은 값과 원본 `results.json`의 seed 11 값을 대조한다.
NOSLIP_CROSS_CHECKS = (
    ('solo_carry', 'local_contact_fine', ('hold_slip_mm', 'r1'), 3.8699),
    ('solo_carry', 'cargo_noslip_v1', ('hold_slip_mm', 'r1'), 0.0064),
    ('solo_carry', 'local_contact_fine', ('carry_slip_mm', 'r1'), 0.6583),
    ('solo_carry', 'cargo_noslip_v1', ('carry_slip_mm', 'r1'), 0.0014),
    ('solo_carry', 'local_contact_fine', ('max_slip_mm', 'r1'), 4.6292),
    ('solo_carry', 'cargo_noslip_v1', ('max_slip_mm', 'r1'), 0.0110),
    ('solo_carry', 'local_contact_fine', ('hold_finger_total_n', 'r1'), 10.38),
    ('solo_carry', 'cargo_noslip_v1', ('hold_finger_total_n', 'r1'), 10.38),
    ('solo_carry', 'local_contact_fine', ('placement_err_mm',), 3.754),
    ('solo_carry', 'cargo_noslip_v1', ('placement_err_mm',), 5.848),
    ('solo_hold_load', 'local_contact_fine', ('hold_slip_mm', 'r1'), 70.0715),
    ('solo_hold_load', 'cargo_noslip_v1', ('hold_slip_mm', 'r1'), 0.1274),
    ('solo_hold_load', 'local_contact_fine', ('hold_finger_total_n', 'r1'), 7.9857),
    ('solo_hold_load', 'cargo_noslip_v1', ('hold_finger_total_n', 'r1'), 10.7228),
    ('solo_hold_load', 'local_contact_fine', ('max_qvel_abs',), 12.46174),
    ('solo_hold_load', 'cargo_noslip_v1', ('max_qvel_abs',), 3.26305),
    ('pair_beam_hold', 'local_contact_fine', ('hold_slip_mm', 'r1'), 16.293),
    ('pair_beam_hold', 'cargo_noslip_v1', ('hold_slip_mm', 'r2'), 0.1661),
    ('pair_beam_hold', 'local_contact_fine', ('carrier_spread_change_mm',), 4.418),
    ('pair_beam_hold', 'cargo_noslip_v1', ('carrier_spread_change_mm',), 5.028),
    ('wall_push', 'local_contact_fine', ('max_penetration_mm',), 0.26284),
    ('wall_push', 'cargo_noslip_v1', ('max_penetration_mm',), 0.26279),
    ('wall_push', 'local_contact_fine', ('along_wall_slide_mm',), 309.391),
    ('wall_push', 'cargo_noslip_v1', ('along_wall_slide_mm',), 309.391),
    ('wall_push', 'local_contact_fine', ('escape_mm',), 121.021),
    ('wall_push', 'cargo_noslip_v1', ('escape_mm',), 121.022),
    ('wall_push', 'local_contact_fine', ('wall_contact_samples',), 235),
)
# 기준 프로필이 짐을 떨어뜨린 시각(README 6.6). hold 시작 기준 상대 SIM 시각이다.
DROP_CROSS_CHECK = {'first_zero_finger_hold_s': 89.45, 'floor_touch_hold_s': 89.5}

# README 본문이 원본 JSON과 다른 값을 적은 곳. 원본 값을 옮기고 차이를 기록에 남긴다.
KNOWN_RECORD_DIFFS = [
    {'where': 'PR #189 README 6.6 표', 'field': 'solo_hold_load/local_contact_fine/hold_creep_mm_per_min (전체 118 s)',
     'readme': 40.7616, 'recorded': 40.76499,
     'note': 'results.json과 원본 result.json은 40.76499다. 이 스냅샷은 원본 값을 옮겼다.'},
    {'where': 'PR #193 README test 총평', 'field': '게이트 범위 뷰 수 / 결정 뷰 수',
     'readme': '84뷰 중 결정 51뷰', 'recorded': '88뷰 중 결정 56뷰',
     'note': 'test-summary.json의 판단별 n 합은 24+20+24+20=88, decided 합은 56이다. 오답 0은 일치한다.'},
]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path: Path):
    return json.loads(path.read_text())


def number(value):
    return type(value) in (int, float)


def near(value, expected, places=3) -> bool:
    """기록 본문이 소수 `places`자리로 적은 값과 원본 값이 같은지 본다.

    허용오차는 그 자리의 반올림 폭(0.5 × 10^-places)뿐이다. 마지막 자리가 1 다르면 불일치다.
    """
    if expected is None or value is None:
        return expected is None and value is None
    if not number(value) or not number(expected):
        return value == expected
    return abs(round(float(value), places) - round(float(expected), places)) <= 0.5 * 10 ** -places


def decimals(expected) -> int:
    """기록 본문이 그 값을 몇 자리까지 적었는지 센다(대조 정밀도)."""
    if isinstance(expected, int):
        return 0
    text = repr(float(expected))
    return len(text.split('.')[1].rstrip('0')) if '.' in text else 0


def exact(value, expected) -> bool:
    if number(value) and number(expected):
        return abs(float(value) - float(expected)) <= 1e-9 + 1e-9 * abs(float(expected))
    return value == expected


# 구역 문자(A/B/C)·로봇 ID를 원본 그대로 쓰므로 대문자를 허용한다.
TAG_KEY = re.compile(r'[A-Za-z0-9_]+')


def leaves(value, prefix: str, out: dict, skipped: list | None = None) -> None:
    """숫자 잎만 태그로 펼친다. bool·None·문자열은 스칼라로 옮기지 않는다.

    태그로 쓸 수 없는 키(대문자·기호)는 옮기지 않고 `skipped`에 남긴다. 원본 값은 그대로 두고
    `evaluation`의 전문에서 읽는다.
    """
    if number(value):
        out[prefix] = value
    elif isinstance(value, dict):
        for key, child in value.items():
            if not TAG_KEY.fullmatch(str(key)):
                if skipped is not None:
                    skipped.append(f'{prefix}/{key}')
                continue
            leaves(child, f'{prefix}/{key}', out, skipped)
    elif isinstance(value, list):
        for index, child in enumerate(value):
            leaves(child, f'{prefix}/{index}', out, skipped)


class Builder:
    def __init__(self, output: Path):
        self.output = output
        self.rows: list[dict] = []
        self.checks: list[dict] = []

    def check(self, name, value, expected, *, places=None):
        ok = near(value, expected, places) if places is not None else exact(value, expected)
        self.checks.append({'check': name, 'recorded': value, 'expected': expected, 'ok': ok})
        if not ok:
            raise SystemExit(f'record cross-check failed: {name}: recorded={value!r} expected={expected!r}')

    def write(self, name: str, view: dict, row: dict) -> None:
        directory = self.output / name
        directory.mkdir(parents=True, exist_ok=False)
        (directory / 'result.json').write_text(json.dumps(view, ensure_ascii=False, indent=1) + '\n')
        self.rows.append({'run': name, 'source': str(directory), **row})


# ------------------------------------------------------------------ 인식 채점 파생 뷰
def perception_scalars(stats: dict, judgment: str) -> dict:
    """채점 요약 한 칸(판단 × 범위)의 기록된 수치를 태그로 옮긴다."""
    out = {}
    plain = ('n', 'unknown', 'decided', 'correct', 'wrong', 'false_confident')
    rates = ('unknown_rate', 'accuracy_of_all', 'accuracy_of_decided', 'false_confident_rate')
    rename = {'n': 'views', 'false_confident': 'confident_errors',
              'false_confident_rate': 'confident_error_rate'}
    for key in plain + rates:
        value = stats.get(key)
        if number(value):
            out['offline/' + rename.get(key, key)] = value
    for group in ('observable', 'not_observable', 'peer_in_lane_reported_separately', 'mapped_wall_view'):
        sub = stats.get(group)
        if not isinstance(sub, dict):
            continue
        label = {'peer_in_lane_reported_separately': 'peer_in_lane',
                 'mapped_wall_view': 'mapped_wall'}.get(group, group)
        for key in plain + rates:
            value = sub.get(key)
            if number(value):
                out[f'offline/{label}/' + rename.get(key, key)] = value
    for case, sub in (stats.get('by_case') or {}).items():
        for key in plain:
            value = sub.get(key)
            if number(value):
                out[f'offline/case/{case}/' + rename.get(key, key)] = value
    return out


def judgment_gates(view: dict, judgment: str) -> dict:
    """사전 등록 게이트 중 판단 단위로 판정할 수 있는 것만 계산한다."""
    observable = view.get('observable') or {}
    gates = {
        'g1_confident_errors_le_1': view.get('false_confident', 0) <= 1,
        'g3_decided_accuracy_ge_0_85': (view.get('accuracy_of_decided') is not None
                                        and view['accuracy_of_decided'] >= 0.85),
        'g4_observable_accuracy_ge_0_70': (observable.get('accuracy_of_all') is not None
                                           and observable['accuracy_of_all'] >= 0.70),
        'g5_observable_unknown_le_0_35': (observable.get('unknown_rate') is not None
                                          and observable['unknown_rate'] <= 0.35),
    }
    if judgment in G2_JUDGMENTS:
        gates['g2_observable_confident_errors_zero'] = observable.get('false_confident', 0) == 0
    return gates


def build_perception(builder: Builder, records: Path, raw: Path) -> dict:
    split_file = records / 'split.json'
    split_sha = sha256(split_file)
    splits = load(split_file)
    summary_index = {}
    for split, folder in PERCEPTION_SPLITS.items():
        original = raw / folder / 'summary.json'
        committed = records / f'{split}-summary.json'
        if not original.is_file():
            raise SystemExit('original score summary missing: ' + str(original))
        digest, committed_digest = sha256(original), sha256(committed)
        if digest != committed_digest:
            raise SystemExit(f'{split}: raw {original} differs from committed {committed}')
        manifest_path = raw / PERCEPTION_FRAMES[split] / 'manifest.json'
        summary_index[split] = {
            'original': original, 'sha256': digest, 'committed': committed,
            'summary': load(original), 'manifest_path': manifest_path,
            'manifest': load(manifest_path), 'manifest_sha256': sha256(manifest_path)}
    for split, entry in summary_index.items():
        summary = entry['summary']
        if summary.get('schema') != 'ugrp.zone_own_perception_eval.v1.score':
            raise SystemExit('unexpected score schema in ' + str(entry['original']))
        builder.check(f'{split}/confident_threshold', summary['confident_threshold'], CONFIDENT_THRESHOLD)
        builder.check(f'{split}/primary_belief', summary['primary_belief_level'], PRIMARY_BELIEF)
        builder.check(f'{split}/split_file_sha256', entry['manifest']['split_file_sha256'], split_sha)
        for judgment, expected in PERCEPTION_CROSS_CHECKS[split].items():
            stats = summary['results'][PRIMARY_BELIEF]['view'][judgment]
            observable = stats['observable']
            found = (stats['n'], stats['unknown_rate'], stats['accuracy_of_all'],
                     stats['accuracy_of_decided'], stats['false_confident'], observable['n'],
                     observable['accuracy_of_all'], observable['unknown_rate'] if expected[7] is not None else None,
                     observable['false_confident'])
            for got, want, label in zip(found, expected, (
                    'views', 'unknown_rate', 'accuracy_of_all', 'accuracy_of_decided',
                    'confident_errors', 'observable_views', 'observable_accuracy',
                    'observable_unknown_rate', 'observable_confident_errors')):
                builder.check(f'{split}/{judgment}/{label}', got, want, places=3)
        for (check_split, belief), table in BELIEF_CROSS_CHECKS.items():
            if check_split != split:
                continue
            for judgment, (accuracy, confident) in table.items():
                stats = summary['results'][belief]['view'][judgment]
                builder.check(f'{split}/{belief}/{judgment}/accuracy_of_all', stats['accuracy_of_all'], accuracy, places=3)
                builder.check(f'{split}/{belief}/{judgment}/confident_errors', stats['false_confident'], confident)

    for split, entry in summary_index.items():
        summary, manifest = entry['summary'], entry['manifest']
        scenes = {f'{s["variant"]}-s{s["seed"]}': s for s in manifest['scenes']}
        base_records = {'experiment_summary': str(entry['committed']),
                        'experiment_summary_sha256': entry['sha256'],
                        'split_file': str(split_file), 'split_file_sha256': split_sha,
                        'frames_manifest': str(entry['manifest_path']),
                        'frames_manifest_sha256': entry['manifest_sha256'],
                        'scene_xml_sha256': {k: v['scene_xml_sha256'] for k, v in scenes.items()}}
        for belief, belief_short in BELIEFS.items():
            for judgment, judgment_short in JUDGMENTS.items():
                view_stats = summary['results'][belief]['view'][judgment]
                tick_stats = summary['results'][belief]['tick'][judgment]
                scalars = perception_scalars(view_stats, judgment)
                for tag, value in perception_scalars(tick_stats, judgment).items():
                    scalars[tag.replace('offline/', 'offline/tick/', 1)] = value
                gate_scalars, gates, success, definition = {}, None, None, None
                if split == 'test' and belief == PRIMARY_BELIEF:
                    gates = judgment_gates(view_stats, judgment)
                    for gate, ok in sorted(gates.items()):
                        gate_scalars[f'gate/{gate}'] = int(ok)
                    gate_scalars['gate/passed'] = sum(gates.values())
                    gate_scalars['gate/total'] = len(gates)
                    success = all(gates.values())
                    definition = ('사전 등록 게이트 중 이 판단에 적용되는 것 전부 통과: '
                                  + '; '.join(GATES[g][0] for g in sorted(gates))
                                  + f'. 확신 오류는 answer≠truth, answer≠unknown, 신뢰도 ≥ {CONFIDENT_THRESHOLD}인 뷰다.')
                scalars.update(gate_scalars)
                name = f'pc-{split}-{judgment_short}-{belief_short}'
                curated = [t for t in ('offline/views', 'offline/unknown_rate', 'offline/accuracy_of_all',
                                       'offline/accuracy_of_decided', 'offline/confident_errors',
                                       'offline/observable/views', 'offline/observable/accuracy_of_all',
                                       'offline/observable/unknown_rate', 'offline/observable/confident_errors',
                                       'gate/passed', 'gate/total') if t in scalars]
                condition = (f'zone-own-perception {split} {judgment} belief={belief}; '
                             f'입력=자기 손목 어안 RGB + 자기 발행 팔 PWM + 정적 지도 + 시나리오 주문서; '
                             f'TOP·시뮬레이터 상태 없음; weld {manifest["weld"]}; '
                             f'render {summary["render_source_sha"][:7]} / score {summary["score_source_sha"][:7]}; '
                             f'정답은 라벨·채점 전용')
                view = {
                    'schema': VIEW_SCHEMA, 'derived_view_only': True,
                    'offline_source': {'path': str(entry['original']), 'sha256': entry['sha256']},
                    'offline_source_pointer': f'results.{belief}.view|tick.{judgment}',
                    'offline_scalar_scope': (
                        f'{split} 분할 {judgment} 판단의 뷰 단위·tick 단위 채점 수치(belief={belief}). '
                        '오프라인 인식 채점이며 로봇 임무 성공률이 아니다.'),
                    'offline_scalars': scalars,
                    'hparam_metrics': curated,
                    'family': 'zone-own-perception', 'policy': 'own_wrist_fisheye_judgments_v1',
                    'case': '|'.join((split, judgment, belief)),
                    'condition': condition, 'split': split, 'judgment': judgment, 'belief': belief,
                    'seed': None, 'source_sha': summary['score_source_sha'],
                    'clock': 'offline scoring (no SIM advance)',
                    'goal': f'{judgment}: yes / no / unknown + 신뢰도',
                    'outcome': ('gates_pass' if success else 'gates_fail') if success is not None else 'no_gate_scope',
                    'success': success, 'success_definition': definition,
                    'stop_reason': 'offline_scored',
                    'scope': (
                        '판단 함수는 자기 손목 RGB 1장·자기 발행 팔 PWM·버전 있는 정적 지도·주문서·주입된 '
                        '자기 위치 belief만 받는다. gt_stub_eval_only는 오차 0 stub이며 위치 추정 결과가 아니다. '
                        'unknown은 1급 답이고 확신 오류만 영수증·메시지 주장이 될 수 있어 따로 센다. '
                        'hold_* 뷰는 자세만 맞춘 인식 fixture이며 파지 성공이 아니다. '
                        'peer_in_lane은 게이트 밖이며 동료/물체 구분은 미구현이다.'),
                    'limits': (
                        '분할당 시드 4개·판단별 사례 4뷰다. 사례 비율을 다른 실험과 합산하지 않는다. '
                        'dev는 임계값 선택에 쓴 분할이므로 test와 합산하지 않는다. '
                        '실제 임무 성공·실시간 실행·통신 조건 비교는 이 기록의 범위가 아니다.'),
                    'evaluation': {
                        'evaluation_only_ground_truth': True,
                        'view_scope': view_stats, 'tick_scope': tick_stats,
                        'gates_applied': gates, 'gate_definitions': GATES if gates else None,
                        'confident_threshold': summary['confident_threshold'],
                        'load_average_1min': summary['load_average_1min'],
                        'render_wall_s': manifest['wall_s'],
                        'render_load_average_1min': manifest['load_average_1min'],
                        'pose_belief_levels': manifest['pose_belief_levels'],
                        'unmapped_block': manifest['unmapped_block'],
                        'hold_views': manifest['hold_views'],
                        'split_scenes': splits,
                    },
                    'records': base_records,
                    'texts': {'evaluation/split_and_scenes': {'split': splits, 'scenes': manifest['scenes']}},
                }
                builder.write(name, view, {
                    'family': 'zone-own-perception', 'split': split, 'judgment': judgment,
                    'belief': belief, 'condition': condition, 'origin': str(entry['original']),
                    'origin_sha256': entry['sha256'], 'success': success,
                    'outcome': view['outcome'], 'scalars': len(scalars)})

    # 코호트 게이트 판정 1개(8개 게이트 전체).
    test = summary_index['test']
    summary = test['summary']
    views = summary['results'][PRIMARY_BELIEF]['view']
    per_judgment = {j: judgment_gates(views[j], j) for j in JUDGMENTS}
    placed_b = views['placed_in_slot']['by_case']['placed_zone_B']
    zone_b_yes = placed_b['correct']
    not_observable_confident = sum((views[j].get('not_observable') or {}).get('false_confident', 0)
                                   for j in JUDGMENTS)
    cohort = {
        'g1_confident_errors_le_1': all(g['g1_confident_errors_le_1'] for g in per_judgment.values()),
        'g2_observable_confident_errors_zero': all(
            per_judgment[j]['g2_observable_confident_errors_zero'] for j in G2_JUDGMENTS),
        'g3_decided_accuracy_ge_0_85': all(g['g3_decided_accuracy_ge_0_85'] for g in per_judgment.values()),
        'g4_observable_accuracy_ge_0_70': all(g['g4_observable_accuracy_ge_0_70'] for g in per_judgment.values()),
        'g5_observable_unknown_le_0_35': all(g['g5_observable_unknown_le_0_35'] for g in per_judgment.values()),
        'g6_placed_zone_b_yes_ge_3': zone_b_yes >= 3,
        'g7_not_observable_confident_errors_le_1': not_observable_confident <= 1,
        'g8_unit_tests_pass': G8_RECORDED['pass'],
    }
    builder.check('test/gates_all_pass', all(cohort.values()), True)
    builder.check('test/placed_zone_B_yes', zone_b_yes, 4)
    builder.check('test/not_observable_confident_errors', not_observable_confident, 0)
    totals = {'views': 0, 'unknown': 0, 'decided': 0, 'correct': 0, 'wrong': 0, 'confident_errors': 0}
    for judgment in JUDGMENTS:
        stats = views[judgment]
        totals['views'] += stats['n']
        totals['unknown'] += stats['unknown']
        totals['decided'] += stats['decided']
        totals['correct'] += stats['correct']
        totals['wrong'] += stats['wrong']
        totals['confident_errors'] += stats['false_confident']
    scalars = {f'gate/{g}': int(ok) for g, ok in sorted(cohort.items())}
    scalars['gate/passed'] = sum(cohort.values())
    scalars['gate/total'] = len(cohort)
    scalars['gate/g6_placed_zone_b_yes'] = zone_b_yes
    scalars['gate/g7_not_observable_confident_errors'] = not_observable_confident
    scalars.update({f'offline/{k}': v for k, v in totals.items()})
    builder.write('pc-test-gates', {
        'schema': VIEW_SCHEMA, 'derived_view_only': True,
        'offline_source': {'path': str(test['original']), 'sha256': test['sha256']},
        'offline_source_pointer': 'results.gt_stub_eval_only.view (판단 4종 합)',
        'offline_scalar_scope': ('test 분할 사전 등록 게이트 8개의 판정과 판단 4종 합계(primary belief). '
                                'G8만 PR #193 README의 기록을 옮긴 주장이며 이 스냅샷이 재실행하지 않았다.'),
        'offline_scalars': scalars,
        'hparam_metrics': ['gate/passed', 'gate/total', 'offline/views', 'offline/decided',
                           'offline/wrong', 'offline/confident_errors'],
        'family': 'zone-own-perception', 'policy': 'own_wrist_fisheye_judgments_v1',
        'case': 'test|preregistered_gates', 'condition': (
            'zone-own-perception test 사전 등록 게이트 8개; 임계값은 dev에서 고르고 test는 1회만 채점'),
        'split': 'test', 'judgment': 'all', 'belief': PRIMARY_BELIEF, 'seed': None,
        'source_sha': summary['score_source_sha'], 'clock': 'offline scoring (no SIM advance)',
        'outcome': 'gates_pass' if all(cohort.values()) else 'gates_fail',
        'success': all(cohort.values()),
        'success_definition': ('사전 등록 게이트 G1–G8 전부 통과. G8은 README가 적은 단위 테스트 '
                               '20/20 통과를 옮긴 주장이며 이 스냅샷이 재실행하지 않았다.'),
        'stop_reason': 'offline_scored',
        'scope': ('뷰 단위·primary belief(gt_stub_eval_only) 기준이다. 결정한 답의 정확도와 unknown 비율을 '
                  '함께 읽는다. 관측 불가 뷰는 전부 unknown이 의도한 결과다.'),
        'limits': ('오프라인 인식 채점이다. 임무 성공·실시간 실행·통신 조건 비교가 아니다. '
                   'dev 수치와 합산하지 않는다.'),
        'evaluation': {'evaluation_only_ground_truth': True, 'gates': cohort,
                       'gate_definitions': GATES, 'per_judgment_gates': per_judgment,
                       'totals_primary_belief_view_scope': totals,
                       'g8_recorded_claim': G8_RECORDED,
                       'placed_zone_B': placed_b,
                       'known_record_differences': KNOWN_RECORD_DIFFS},
        'records': {'experiment_summary': str(test['committed']),
                    'experiment_summary_sha256': test['sha256'],
                    'split_file': str(split_file), 'split_file_sha256': split_sha},
        'texts': {'evaluation/gate_verdicts': cohort,
                  'evaluation/record_differences': KNOWN_RECORD_DIFFS},
    }, {'family': 'zone-own-perception', 'split': 'test', 'judgment': 'all',
        'belief': PRIMARY_BELIEF, 'condition': 'preregistered gates',
        'origin': str(test['original']), 'origin_sha256': test['sha256'],
        'success': all(cohort.values()), 'outcome': 'gates_pass' if all(cohort.values()) else 'gates_fail',
        'scalars': len(scalars)})
    return {'splits': sorted(summary_index), 'runs': len(JUDGMENTS) * len(BELIEFS) * 2 + 1,
            'summary_sha256': {s: e['sha256'] for s, e in summary_index.items()}}


# ------------------------------------------------------------------ 접촉 감사 파생 뷰
def drop_scalars(rows: list[dict], robot: str) -> tuple[dict, dict]:
    """유지 구간에서 접촉이 사라지고 화물이 바닥에 닿은 **기록된 표본**을 찾는다."""
    hold = [r for r in rows if r.get('phase') == 'hold']
    if not hold:
        return {}, {}
    origin = hold[0]['t']
    zero = next((r for r in hold if sum(r['finger_n'][robot]) == 0), None)
    touch = next((r for r in hold if r['cargo_min_z_m'] <= 0.0), None)
    lowest = min(hold, key=lambda r: r['cargo_min_z_m'])
    scalars = {'offline/drop/occurred': int(touch is not None),
               'offline/drop/hold_origin_sim_s': origin,
               'offline/drop/min_cargo_min_z_m': lowest['cargo_min_z_m'],
               'offline/drop/min_cargo_min_z_hold_s': round(lowest['t'] - origin, 6)}
    detail = {'hold_origin_sim_s': origin, 'hold_samples': len(hold),
              'lowest_cargo_min_z_m': lowest['cargo_min_z_m'],
              'lowest_at_sim_s': lowest['t'], 'lowest_at_hold_s': round(lowest['t'] - origin, 6),
              'note': ('원본 trace.jsonl 표본에서 조건을 처음 만족한 시각이다. 표본 간격 밖을 '
                       '보간하지 않았다. hold 상대 시각은 첫 hold 표본을 0으로 둔 값이다.')}
    if zero is not None:
        scalars['offline/drop/first_zero_finger_hold_s'] = round(zero['t'] - origin, 6)
        scalars['offline/drop/first_zero_finger_sim_s'] = zero['t']
        detail['first_zero_finger'] = {'sim_s': zero['t'], 'hold_s': round(zero['t'] - origin, 6),
                                       'cargo_min_z_m': zero['cargo_min_z_m']}
    if touch is not None:
        scalars['offline/drop/floor_touch_hold_s'] = round(touch['t'] - origin, 6)
        scalars['offline/drop/floor_touch_sim_s'] = touch['t']
        scalars['offline/drop/floor_touch_cargo_min_z_m'] = touch['cargo_min_z_m']
        detail['floor_touch'] = {'sim_s': touch['t'], 'hold_s': round(touch['t'] - origin, 6),
                                 'cargo_min_z_m': touch['cargo_min_z_m'],
                                 'cargo_tilt_deg': touch['cargo_tilt_deg']}
    return scalars, detail


def series_spec(trace: Path, digest: str, scenario: str, robots: list[str], origin=None) -> dict:
    tags = [{'tag': 'trace/wall_contacts', 'path': ['wall_contacts']},
            {'tag': 'trace/wall_penetration_mm', 'path': ['wall_penetration_mm']},
            {'tag': 'trace/wall_normal_n', 'path': ['wall_normal_n']}]
    if scenario in CARGO_SCENARIOS:
        tags += [{'tag': 'trace/cargo_min_z_m', 'path': ['cargo_min_z_m']},
                 {'tag': 'trace/cargo_tilt_deg', 'path': ['cargo_tilt_deg']},
                 {'tag': 'trace/cargo_z_m', 'path': ['cargo_xyz', 2]}]
        for robot in robots:
            tags += [{'tag': f'trace/finger_total_n/{robot}', 'path': ['finger_n', robot], 'reduce': 'sum'},
                     {'tag': f'trace/grip_in_cargo_z_mm/{robot}', 'path': ['grip_in_cargo_mm', robot, 2]}]
    spec = {'source': {'path': str(trace), 'sha256': digest}, 'format': 'jsonl',
            'sim_time_field': 't', 'phase_field': 'phase', 'tags': tags}
    if origin is not None:
        spec['relative_time'] = {'tag': 'trace/hold_elapsed_s', 'origin_s': origin, 'phase': 'hold'}
    return spec


def build_noslip(builder: Builder, records: Path, raw: Path) -> dict:
    results_path = records / 'results.json'
    results = load(results_path)
    if results.get('schema') != 'ugrp.contact_profile_side_effect_audit.results.v1':
        raise SystemExit('unexpected audit results schema: ' + str(results.get('schema')))
    results_sha = sha256(results_path)
    recorded_root = Path(results['raw_root_local_only'])
    builder.check('noslip/runs', results['runs'], NOSLIP_TOTALS['runs'])
    builder.check('noslip/side_effect_detected', results['side_effect_detected'], True)
    builder.check('noslip/git_dirty', results['git']['dirty_run_sources'], False)
    # 210개 원본 파일 해시를 전부 확인한다.
    for relative, record in results['raw_files'].items():
        path = raw / relative
        if not path.is_file():
            raise SystemExit('audit original missing: ' + str(path))
        if path.stat().st_size != record['bytes'] or sha256(path) != record['sha256']:
            raise SystemExit('audit original changed since results.json: ' + str(path))
    gates_total = sum(s['seeds'][k]['gates'] for s in results['scenarios'].values() for k in s['seeds'])
    gates_failed = sum(s['seeds'][k]['failed'] for s in results['scenarios'].values() for k in s['seeds'])
    builder.check('noslip/gates_total', gates_total, NOSLIP_TOTALS['gates'])
    builder.check('noslip/gates_failed', gates_failed, NOSLIP_TOTALS['failed'])
    for check in NOSLIP_CROSS_CHECKS:
        scenario, profile, path, expected = check[:4]
        places = check[4] if len(check) > 4 else decimals(expected)
        value = results['scenarios'][scenario]['seeds']['11']['metrics'][profile]
        for key in path:
            value = (value or {}).get(key)
        builder.check(f'noslip/{scenario}/{profile}/{"/".join(path)}', value, expected, places=places)

    drop_checked = False
    for scenario, scenario_short in SCENARIOS.items():
        spec = results['scenarios'][scenario]
        first_metrics = {}
        for seed_key, seed_row in sorted(spec['seeds'].items(), key=lambda kv: int(kv[0])):
            seed = int(seed_key)
            # collect.py는 첫 시드와 지표가 같은 시드의 `metrics` 사본을 생략한다. 그런 시드는 원본
            # 실행의 result.json에서 읽고, 첫 시드와 실제로 같은지 확인한다(시드는 결정성 검사다).
            recorded_metrics = seed_row.get('metrics')
            metrics_by_profile, metrics_source = {}, {}
            for profile in PROFILES:
                run_result = load(raw / f'{scenario}-{profile}-{seed}' / 'result.json')
                if recorded_metrics is not None:
                    if run_result['metrics'] != recorded_metrics[profile]:
                        raise SystemExit(f'recorded metrics differ from the original run: '
                                         f'{scenario}-{profile}-{seed}')
                    metrics_by_profile[profile] = recorded_metrics[profile]
                    metrics_source[profile] = 'results.json seeds.metrics (원본 result.json과 일치 확인)'
                else:
                    if seed_row.get('identical_to_first_seed') is not True:
                        raise SystemExit(f'seed row without metrics is not marked identical: '
                                         f'{scenario} seed {seed}')
                    if run_result['metrics'] != first_metrics[profile]:
                        raise SystemExit(f'seed marked identical to the first seed differs: '
                                         f'{scenario}-{profile}-{seed}')
                    metrics_by_profile[profile] = run_result['metrics']
                    metrics_source[profile] = ('원본 result.json (results.json은 첫 시드와 같아서 사본을 '
                                               '생략했고, 실제로 같은지 확인했다)')
            if recorded_metrics is not None and not first_metrics:
                first_metrics = dict(metrics_by_profile)
            comparison_scalars = {'offline/gates_total': seed_row['gates'],
                                  'offline/gates_failed': seed_row['failed'],
                                  'offline/side_effect_detected': int(bool(seed_row['failed'])),
                                  'gate/hard_gates_ok': int(seed_row['hard_gates_ok'])}
            for failed in seed_row.get('failed_gates') or []:
                tag = failed['gate'].replace('::', '_')
                for field in ('baseline', 'candidate', 'diff', 'limit'):
                    comparison_scalars[f'gate/{tag}/{field}'] = failed[field]
                comparison_scalars[f'gate/{tag}/ok'] = int(failed['ok'])
            for profile, profile_short in PROFILES.items():
                metrics = metrics_by_profile[profile]
                run_dir = raw / f'{scenario}-{profile}-{seed}'
                result_path = run_dir / 'result.json'
                result_rel = f'{scenario}-{profile}-{seed}/result.json'
                trace_rel = f'{scenario}-{profile}-{seed}/trace.jsonl'
                run_result = load(result_path)
                builder.check(f'noslip/{scenario}/{profile}/{seed}/applied',
                              run_result['applied_solver_options'], seed_row['applied'][profile])
                builder.check(f'noslip/{scenario}/{profile}/{seed}/scene_xml',
                              run_result['scene_xml_sha256'], seed_row['scene_xml_sha256'][profile])
                builder.check(f'noslip/{scenario}/{profile}/{seed}/weld', run_result['weld'], 'off')
                builder.check(f'noslip/{scenario}/{profile}/{seed}/source_sha',
                              run_result['git']['sha'], results['git']['sha'])
                scalars = {'offline/sim_time_s': run_result['sim_time_s'],
                           'offline/physics_steps': run_result['physics_steps']}
                for key in ('hold_s', 'limit_s'):
                    if number(run_result.get(key)):
                        scalars['offline/' + key] = run_result[key]
                for key, value in run_result['applied_solver_options'].items():
                    scalars[f'offline/applied/{key}'] = value
                for gate in run_result['hard_gates']:
                    scalars[f'gate/hard/{gate["gate"]}/value'] = gate['value']
                    scalars[f'gate/hard/{gate["gate"]}/limit'] = gate['limit']
                    scalars[f'gate/hard/{gate["gate"]}/ok'] = int(gate['ok'])
                flat, skipped = {}, []
                leaves({k: v for k, v in metrics.items() if k != 'segments'}, 'offline', flat, skipped)
                scalars.update(flat)
                for flag in ('dropped', 'lifted_clear'):
                    if isinstance(metrics.get(flag), bool):
                        scalars['offline/' + flag] = int(metrics[flag])
                for segment in metrics.get('segments') or []:
                    label = segment['label']
                    for key in ('displacement_mm', 'disp_at_command_end_mm', 'yaw_change_deg',
                                'grip_move_mm', 't_start_s', 't_end_s'):
                        if number(segment.get(key)):
                            scalars[f'offline/segment/{label}/{key}'] = segment[key]
                    for joint, angle in (segment.get('joints_deg') or {}).items():
                        if number(angle):
                            scalars[f'offline/segment/{label}/joints_deg/{joint}'] = angle
                for index, value in enumerate(run_result['host']['load_avg_start']):
                    scalars[f'offline/host/load_avg_start/{index}'] = value
                for index, value in enumerate(run_result['host']['load_avg_end']):
                    scalars[f'offline/host/load_avg_end/{index}'] = value
                scalars['offline/host/wall_s_not_a_metric'] = run_result['host']['wall_s_not_a_metric']
                robots = sorted((metrics.get('hold_finger_total_n') or {}) or
                                (metrics.get('hold_slip_mm') or {})) or ['r1']
                trace_path = raw / trace_rel
                trace_sha = results['raw_files'][trace_rel]['sha256']
                origin = None
                detail = {}
                if scenario in CARGO_SCENARIOS:
                    rows = [json.loads(line) for line in trace_path.read_text().splitlines() if line.strip()]
                    drop, detail = drop_scalars(rows, robots[0])
                    scalars.update(drop)
                    origin = drop.get('offline/drop/hold_origin_sim_s')
                    if (scenario == 'solo_hold_load' and profile == 'local_contact_fine'
                            and seed == 11):
                        for key, expected in DROP_CROSS_CHECK.items():
                            builder.check(f'noslip/drop/{key}', drop.get('offline/drop/' + key),
                                          expected, places=decimals(expected))
                        drop_checked = True
                hard_ok = all(gate['ok'] for gate in run_result['hard_gates'])
                name = f'ns-{scenario_short}-{profile_short}-s{seed}'
                condition = (f'noslip-audit {scenario} profile={profile} '
                             f'(noslip_iterations={run_result["applied_solver_options"]["noslip_iterations"]}) '
                             f'seed {seed}; zones/zone_wide_door; 정답 교사·스크립트 tape; weld off; '
                             f'timestep {run_result["applied_solver_options"]["timestep_s"]} s; '
                             f'source {results["git"]["sha"][:7]}')
                curated = [t for t in ('offline/sim_time_s', 'offline/max_slip_mm/r1',
                                       'offline/hold_slip_mm/r1', 'offline/hold_creep_mm_per_min/r1',
                                       'offline/hold_finger_total_n/r1', 'offline/max_cargo_tilt_deg',
                                       'offline/max_penetration_mm', 'offline/along_wall_slide_mm',
                                       'offline/escape_mm', 'offline/max_qvel_abs',
                                       'offline/drop/occurred', 'offline/drop/floor_touch_hold_s',
                                       'offline/applied/noslip_iterations') if t in scalars]
                view = {
                    'schema': VIEW_SCHEMA, 'derived_view_only': True,
                    'offline_source': {'path': str(result_path),
                                       'sha256': results['raw_files'][result_rel]['sha256']},
                    'offline_source_pointer': 'result.json (metrics·hard_gates·applied_solver_options)',
                    'offline_scalar_scope': (
                        f'{scenario} 실행 1개({profile}, seed {seed})의 기록된 물리 지표다. '
                        '정답 교사·스크립트 조건의 SIM 물리 측정이며 RGB 학생 성공률이 아니다.'),
                    'offline_scalars': scalars,
                    'offline_series': series_spec(trace_path, trace_sha, scenario, robots, origin),
                    'hparam_metrics': curated,
                    'family': 'noslip-side-effects', 'policy': 'ground_truth_teacher|scripted_tape',
                    'case': '|'.join((scenario, profile, f'seed{seed}')),
                    'condition': condition, 'scenario': scenario, 'contact_profile': profile,
                    'seed': seed, 'split': None, 'judgment': None, 'belief': None,
                    'source_sha': run_result['git']['sha'], 'clock': 'sim(synchronous)',
                    'sim_s': run_result['sim_time_s'],
                    'goal': spec['question'],
                    'outcome': metrics.get('outcome') or 'measured',
                    'success': hard_ok,
                    'success_definition': ('공통 하드 게이트 통과: eq_active 최댓값 0(weld OFF), '
                                           'max|qvel| ≤ 50, NaN 표본 0. 임무 성공 판정이 아니다.'),
                    'stop_reason': metrics.get('outcome') or 'tape_complete',
                    'scope': (
                        '정답 교사/스크립트 명령 조건의 물리 감사다. RGB·학생 성공률이 아니고 로봇 임무 성공 '
                        '판정도 아니다. weld는 OFF이며 매 step eq_active를 검사했다. wall 시간·sim step 비용은 '
                        '지표가 아니다(호스트를 다른 에이전트와 공유했다). 시계열은 원본 trace.jsonl 표본을 '
                        '그대로 옮긴 것이다.'),
                    'limits': (
                        '조건당 1회다. 시드 11/12/13은 독립 반복이 아니라 결정성 검사다(감사 대상 로봇은 '
                        'fixture로 같은 pose에 고정된다). zone_wide_door 한 변이·동쪽 열린 구역 한정이며 '
                        '실물 MasterPi 마찰 실측과 비교하지 않았다. 자동 게이트 통과는 실제 임무 완주 '
                        '재검증이 아니다.'),
                    'evaluation': {
                        'evaluation_only_ground_truth': True,
                        'metrics': metrics, 'hard_gates': run_result['hard_gates'],
                        'applied_solver_options': run_result['applied_solver_options'],
                        'scene_xml_sha256': run_result['scene_xml_sha256'],
                        'scene_record_sha256': run_result['scene_record_sha256'],
                        'weld': run_result['weld'], 'teacher_condition': run_result['teacher_condition'],
                        'student_result': run_result['student_result'],
                        'host_not_a_metric': run_result['host'],
                        'drop_detail': detail or None,
                        'scalar_keys_not_tag_safe': skipped or None,
                        'metrics_source': metrics_source[profile],
                        'identical_to_first_seed': seed_row.get('identical_to_first_seed'),
                        'tolerance': spec['tolerance'],
                        'question': spec['question'], 'metric': spec['metric'],
                    },
                    'records': {'experiment_results': str(results_path),
                                'experiment_results_sha256': results_sha,
                                'recorded_raw_root': str(recorded_root),
                                'raw_root_used': str(raw),
                                'trace_sha256': trace_sha,
                                'events_sha256': results['raw_files'][f'{scenario}-{profile}-{seed}/events.json']['sha256'],
                                'scene_xml_file_sha256': results['raw_files'][f'{scenario}-{profile}-{seed}/scene.xml']['sha256']},
                    'texts': {'evaluation/events': load(run_dir / 'events.json'),
                              'evaluation/hashes': load(run_dir / 'hashes.json')},
                }
                builder.write(name, view, {
                    'family': 'noslip-side-effects', 'scenario': scenario, 'contact_profile': profile,
                    'seed': seed, 'condition': condition, 'origin': str(result_path),
                    'origin_sha256': view['offline_source']['sha256'], 'success': hard_ok,
                    'outcome': view['outcome'], 'scalars': len(scalars)})
            # 프로필 A/B 비교 1개(시드 안에서만 비교한다).
            name = f'ns-{scenario_short}-cmp-s{seed}'
            condition = (f'noslip-audit {scenario} A/B seed {seed}: '
                         f'{results["profiles"]["baseline"]} vs {results["profiles"]["candidate"]}; '
                         f'같은 시드 안에서만 비교; source {results["git"]["sha"][:7]}')
            for metric_key in ('hold_slip_mm', 'hold_creep_mm_per_min', 'max_slip_mm',
                               'hold_finger_total_n', 'carry_slip_mm'):
                for profile in PROFILES:
                    value = (metrics_by_profile[profile].get(metric_key) or {})
                    if isinstance(value, dict):
                        for robot, number_value in value.items():
                            if number(number_value):
                                comparison_scalars[f'offline/ab/{metric_key}/{robot}/{PROFILES[profile]}'] = number_value
            for metric_key in ('max_cargo_tilt_deg', 'carrier_spread_change_mm', 'placement_err_mm',
                               'max_penetration_mm', 'along_wall_slide_mm', 'escape_mm',
                               'max_qvel_abs', 'wall_contact_samples'):
                for profile in PROFILES:
                    value = metrics_by_profile[profile].get(metric_key)
                    if number(value):
                        comparison_scalars[f'offline/ab/{metric_key}/{PROFILES[profile]}'] = value
            builder.write(name, {
                'schema': VIEW_SCHEMA, 'derived_view_only': True,
                'offline_source': {'path': str(results_path), 'sha256': results_sha},
                'offline_source_pointer': f'scenarios.{scenario}.seeds.{seed}',
                'offline_scalar_scope': (
                    f'{scenario} seed {seed}의 프로필 A/B 게이트 결과와 기록된 양쪽 값이다. '
                    '깨진 게이트의 baseline/candidate/diff/limit는 results.json에 기록된 값이며 '
                    '통과한 게이트의 개별 값은 원본에 기록되지 않았다.'),
                'offline_scalars': comparison_scalars,
                'hparam_metrics': [t for t in ('offline/gates_total', 'offline/gates_failed',
                                               'gate/hard_gates_ok', 'offline/side_effect_detected')
                                   if t in comparison_scalars],
                'family': 'noslip-side-effects', 'policy': 'profile_ab_comparison',
                'case': '|'.join((scenario, 'A/B', f'seed{seed}')),
                'condition': condition, 'scenario': scenario, 'contact_profile': 'A/B',
                'seed': seed, 'split': None, 'judgment': None, 'belief': None,
                'source_sha': results['git']['sha'], 'clock': 'sim(synchronous)',
                'goal': spec['question'],
                'outcome': 'gates_pass' if not seed_row['failed'] else 'gates_fail',
                'success': not seed_row['failed'] and seed_row['hard_gates_ok'],
                'success_definition': ('이 시나리오·시드의 사전 등록 허용오차 게이트 전부 통과 + 공통 하드 게이트 통과. '
                                       '부작용 없음이라는 뜻이며 임무 성공이 아니다.'),
                'stop_reason': seed_row['status'],
                'scope': ('허용오차는 코드의 기존 행동 문턱(교사 정렬 6 mm, 구역 슬롯 ±60 mm, anchor 18 px)보다 '
                          '작게 사전 등록했다. 미끄러짐 감소는 후보의 의도된 효과이므로 게이트 대상이 아니라 '
                          '수치로만 보고한다.'),
                'limits': ('조건당 1회이며 시드는 결정성 검사다. 깨진 게이트의 원인 귀속은 '
                           'evaluation/failed_gates와 원본 README 6.6절을 함께 읽는다.'),
                'evaluation': {'evaluation_only_ground_truth': True,
                               'gates': seed_row['gates'], 'failed': seed_row['failed'],
                               'failed_gates': seed_row.get('failed_gates') or [],
                               'hard_gates_ok': seed_row['hard_gates_ok'],
                               'tolerance': spec['tolerance'],
                               'applied': seed_row['applied'],
                               'scene_xml_sha256': seed_row['scene_xml_sha256'],
                               'metrics_sha256': seed_row['metrics_sha256'],
                               'sim_time_s': seed_row['sim_time_s'],
                               'load_avg_start_not_a_metric': seed_row['load_avg_start'],
                               'load_avg_end_not_a_metric': seed_row['load_avg_end'],
                               'failed_gate_names': spec['failed_gate_names']},
                'records': {'experiment_results': str(results_path),
                            'experiment_results_sha256': results_sha},
                'texts': {'evaluation/failed_gates': seed_row.get('failed_gates') or [],
                          'evaluation/tolerance': spec['tolerance']},
            }, {'family': 'noslip-side-effects', 'scenario': scenario, 'contact_profile': 'A/B',
                'seed': seed, 'condition': condition, 'origin': str(results_path),
                'origin_sha256': results_sha,
                'success': not seed_row['failed'] and seed_row['hard_gates_ok'],
                'outcome': 'gates_pass' if not seed_row['failed'] else 'gates_fail',
                'scalars': len(comparison_scalars)})
    if not drop_checked:
        raise SystemExit('baseline drop cross-check never ran; solo_hold_load seed 11 missing')

    scenario_failed = {s: sum(v['seeds'][k]['failed'] for k in v['seeds'])
                       for s, v in results['scenarios'].items()}
    cohort = {'offline/runs': results['runs'], 'offline/gates_total': gates_total,
              'offline/gates_failed': gates_failed,
              'offline/gates_passed': gates_total - gates_failed,
              'offline/side_effect_detected': int(results['side_effect_detected']),
              'offline/scenarios': len(results['scenarios'])}
    for scenario, failed in scenario_failed.items():
        cohort[f'offline/scenario/{scenario}/gates_failed'] = failed
        cohort[f'offline/scenario/{scenario}/gates_total'] = sum(
            results['scenarios'][scenario]['seeds'][k]['gates']
            for k in results['scenarios'][scenario]['seeds'])
    builder.write('ns-audit', {
        'schema': VIEW_SCHEMA, 'derived_view_only': True,
        'offline_source': {'path': str(results_path), 'sha256': results_sha},
        'offline_source_pointer': 'scenarios (전체 합)',
        'offline_scalar_scope': ('42 실행·327 게이트 코호트 합계다. 시나리오별 깨진 게이트 수를 함께 본다. '
                                 'SIM 물리 감사이며 임무 성공률이 아니다.'),
        'offline_scalars': cohort,
        'hparam_metrics': ['offline/runs', 'offline/gates_total', 'offline/gates_failed',
                           'offline/gates_passed', 'offline/side_effect_detected'],
        'family': 'noslip-side-effects', 'policy': 'profile_ab_comparison',
        'case': 'cohort|42runs', 'condition': (
            'noslip-audit 코호트 전체: 7 시나리오 × 2 프로필 × 3 시드 = 42 실행, 327 사전 등록 게이트'),
        'scenario': 'all', 'contact_profile': 'A/B', 'seed': None,
        'split': None, 'judgment': None, 'belief': None,
        'source_sha': results['git']['sha'], 'clock': 'sim(synchronous)',
        'outcome': 'side_effect_gate_broken' if gates_failed else 'no_measurable_side_effect',
        'success': gates_failed == 0,
        'success_definition': ('327개 사전 등록 게이트 전부 통과. 실제로는 solo_hold_load의 finger_force::r1이 '
                              '시드 3개에서 깨졌고(3/327), 원인은 기준 프로필이 짐을 떨어뜨린 것이다.'),
        'stop_reason': 'cohort_complete',
        'scope': ('구동·벽·팔·쉬는 물체는 측정 한계 안에서 변화가 없었고, 미끄러짐 감소는 후보의 의도된 '
                  '효과로 게이트 없이 수치만 보고했다. 기존 프로필·기본값은 바꾸지 않았다.'),
        'limits': ('조건당 1회, 시드는 결정성 검사, zone_wide_door 한정이다. 실물 실측과 비교하지 않았다. '
                   'RGB 스킬·ACT·구역 실행기를 이 감사에서 돌리지 않았다.'),
        'evaluation': {'evaluation_only_ground_truth': True,
                       'profiles': results['profiles'], 'git': results['git'],
                       'scenario_failed_gates': scenario_failed,
                       'grip_slip_intended_effect': results['grip_slip_intended_effect'],
                       'raw_root_local_only': results['raw_root_local_only'],
                       'raw_files_verified': len(results['raw_files']),
                       'known_record_differences': KNOWN_RECORD_DIFFS},
        'records': {'experiment_results': str(results_path),
                    'experiment_results_sha256': results_sha},
        'texts': {'evaluation/scenarios': {s: {'question': v['question'], 'tolerance': v['tolerance'],
                                               'failed_gate_names': v['failed_gate_names']}
                                           for s, v in results['scenarios'].items()},
                  'evaluation/record_differences': KNOWN_RECORD_DIFFS},
    }, {'family': 'noslip-side-effects', 'scenario': 'all', 'contact_profile': 'A/B', 'seed': None,
        'condition': 'cohort', 'origin': str(results_path), 'origin_sha256': results_sha,
        'success': gates_failed == 0,
        'outcome': 'side_effect_gate_broken' if gates_failed else 'no_measurable_side_effect',
        'scalars': len(cohort)})
    return {'runs': results['runs'], 'gates_total': gates_total, 'gates_failed': gates_failed,
            'results_sha256': results_sha, 'raw_files_verified': len(results['raw_files'])}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--perception-records', type=Path, required=True)
    parser.add_argument('--perception-raw', type=Path, required=True)
    parser.add_argument('--noslip-records', type=Path, required=True)
    parser.add_argument('--noslip-raw', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='새 파생 뷰 루트(이미 있으면 거부)')
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        parser.error('파생 뷰 루트는 새 경로여야 한다; 원본과 기존 뷰는 덮어쓰지 않는다')
    roots = [p.resolve() for p in (args.perception_records, args.perception_raw,
                                   args.noslip_records, args.noslip_raw)]
    if any(output.is_relative_to(root) for root in roots):
        parser.error('파생 뷰 루트는 원본·기록 폴더 밖이어야 한다')
    output.mkdir(parents=True)
    builder = Builder(output)
    perception = build_perception(builder, args.perception_records.resolve(), args.perception_raw.resolve())
    noslip = build_noslip(builder, args.noslip_records.resolve(), args.noslip_raw.resolve())
    index = {
        'schema': SCHEMA,
        'purpose': ('PR #193 자기 카메라 인식 채점과 PR #189 접촉 프로필 부작용 감사의 TensorBoard 파생 뷰'),
        'perception': {**perception, 'records': str(args.perception_records.resolve()),
                       'raw': str(args.perception_raw.resolve())},
        'noslip': {**noslip, 'records': str(args.noslip_records.resolve()),
                   'raw': str(args.noslip_raw.resolve())},
        'known_record_differences': KNOWN_RECORD_DIFFS,
        'cross_checks': builder.checks,
        'rows': builder.rows,
    }
    (output / 'index.json').write_text(json.dumps(index, ensure_ascii=False, indent=1) + '\n')
    print(json.dumps({'views': len(builder.rows), 'cross_checks': len(builder.checks),
                      'root': str(output)}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
