#!/usr/bin/env python3
"""자기 카메라 폐루프 문 통과 코호트(PR #178)를 TensorBoard 파생 뷰로 만든다.

원본(`outputs/owncam-loop-20260925/<시도>/<에피소드>/`의 `result.json`·`manifest.json`)과
실험 기록(`experiments/2026-09-25-zone-owncam-loop/`)은 읽기만 한다. 실행 1개당
`scripts/export_tensorboard.py`가 읽는 1폴더 `result.json` 파생 뷰를 쓰고, 숫자는 원본에
있는 값만 옮긴다(새 계산·보간 없음). 원본 `result.json`의 SHA-256이 기록의
`raw_index.json`과 다르거나 기록 표와 원본 값이 다르면 변환을 거부한다.

- 성공 bool은 원본 `episode_pass`(사전 등록 게이트 R1·R2·R3(+box R4)) 하나만 쓴다.
- 학생 구간 SIM 시간만 `sim_s`로 옮기고 교사 구간은 `evaluation`에 따로 남긴다.
- 도착 선언은 제어기 주장(`claims/protocol_complete`)이며 사후 게이트 통과와 분리한다.
- 정답(GT) 수치는 평가 전용 영역(`evaluation`)에만 둔다.
- dev 수정 시도(a1–a3)·고정 소스 dev(a4)·사전 등록 test를 코호트 문자열로 구분한다.

사용:
    python3 scripts/build_owncam_loop_views.py \
        --records <PR178 experiments/2026-09-25-zone-owncam-loop> \
        --raw <outputs/owncam-loop-20260925> --output <새 파생 뷰 루트>
    python3 scripts/build_owncam_loop_views.py \
        --rename-snapshot <스냅샷 폴더> --index <파생 뷰 index.json>
"""
from __future__ import annotations

import argparse
import collections
import hashlib
import json
from pathlib import Path

SCHEMA = 'ugrp.owncam_loop_views.v1'
RUN_SCHEMA = 'ugrp.owncam_loop_run.v1'
RESULTS_SCHEMA = 'ugrp.owncam_loop_results.v1'

POSE_SOURCE = 'own_wrist_fisheye_tags_v2+particle_filter'
POLICY = 'owncam_drive(particle_filter+map_goto A*)'

# 시도별 코호트 경계. dev는 반복 실행이 허용됐고 test는 고정 소스에서 1회만 돌았다.
COHORTS = {
    'dev-a1': ('dev-amended', 'dev 시도 1: v1 교정. 뒤이어 사전 등록 부속 수정(dev 전용)이 적용돼 보고 dev가 아니다'),
    'dev-a2': ('dev-amended', 'dev 시도 2: 짐/빈 상태 고각 편향 추가. 보고 dev가 아니다'),
    'dev-a3': ('dev-amended', 'dev 시도 3: 드라이버 전용 넓은 둘러보기(±48°). 보고 dev가 아니다'),
    'dev-a4': ('dev-frozen', 'dev 시도 4(고정 소스 9361a8d): 움직임 재적합 + 이동량 기반 둘러보기. 보고 dev'),
    'test': ('test-preregistered', '사전 등록 test 분할: 고정 소스에서 1회만 실행, 재실행 없음'),
}

# 원본 기록 표(results.json)와 원본 실행 JSON이 같은 값을 담고 있는지 확인할 항목.
CROSS_CHECKS = (
    ('pass', lambda raw, man: raw['episode_pass']),
    ('outcome', lambda raw, man: raw['outcome']),
    ('split', lambda raw, man: raw['split']),
    ('condition', lambda raw, man: raw['condition']),
    ('R1_gt_distance_m', lambda raw, man: raw['gates']['R1_exit_waypoint']['gt_distance_m']),
    ('R1_declared', lambda raw, man: raw['gates']['R1_exit_waypoint']['declared_arrival']),
    ('R2_wall_contacts', lambda raw, man: raw['gates']['R2_no_wall_contact']['wall_contacts']),
    ('R3_p90_pos_m', lambda raw, man: raw['gates']['R3_estimate_near_door']['p90_pos_m']),
    ('R3_p90_yaw_deg', lambda raw, man: raw['gates']['R3_estimate_near_door']['p90_yaw_deg']),
    ('R3_frames', lambda raw, man: raw['gates']['R3_estimate_near_door']['frames']),
    ('R3_uninitialized', lambda raw, man: raw['gates']['R3_estimate_near_door']['uninitialized']),
    ('student_sim_s', lambda raw, man: raw['student_sim_s']),
    ('teacher_sim_s', lambda raw, man: raw.get('teacher_sim_s')),
    ('looks', lambda raw, man: raw['looks']),
    ('look_reasons', lambda raw, man: raw['look_reasons']),
    ('student_commands', lambda raw, man: raw['student_commands']),
    ('student_frames', lambda raw, man: raw['student_frames']),
    ('tag_visibility', lambda raw, man: raw['student_tag_visibility']),
    ('contacts_student', lambda raw, man: raw['contacts_student']),
    ('wall_s', lambda raw, man: man['wall_s']),
    ('load_average', lambda raw, man: man['load_average']),
    ('static_map_sha256', lambda raw, man: man['static_map_sha256']),
    ('scene_xml_sha256', lambda raw, man: man['scene_xml_sha256']),
    ('calibration_sha256', lambda raw, man: man['calibration_sha256']),
    ('weld', lambda raw, man: man['weld']),
    ('code_sha', lambda raw, man: man['code']['sha']),
    ('code_dirty', lambda raw, man: man['code']['dirty']),
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path: Path):
    return json.loads(path.read_text())


def same(a, b) -> bool:
    if isinstance(a, float) or isinstance(b, float):
        try:
            return abs(float(a) - float(b)) <= 1e-9 + 1e-9 * abs(float(b))
        except (TypeError, ValueError):
            return False
    return a == b


def short_name(attempt: str, condition: str, seed: int) -> str:
    """짧은 run 이름: dev 시도는 dev-a<N>-, test는 test-."""
    prefix = 'test' if attempt == 'test' else attempt
    return f'{prefix}-{condition}-s{seed}'


class Builder:
    """실행 1개당 파생 뷰 1폴더를 쓰고 index.json 행을 모은다."""

    def __init__(self, records: Path, raw: Path, output: Path):
        self.records = records
        self.raw = raw
        self.output = output
        self.results_path = records / 'results.json'
        self.results = load(self.results_path)
        if self.results.get('schema') != RESULTS_SCHEMA:
            raise SystemExit(f'unexpected results schema: {self.results.get("schema")}')
        self.raw_index = load(records / 'raw_index.json')
        self.prereg = load(records / 'prereg.json')
        self.amendments = load(records / 'prereg_amendments.json')
        self.frozen = load(records / 'frozen_source.json')
        self.episodes = {e['episode_id']: e for e in self.prereg['episodes']}
        self.rows: list[dict] = []

    # ------------------------------------------------------------------ 원본 확인
    def originals(self, row: dict) -> tuple[Path, dict, dict, str]:
        key = f'{row["attempt"]}/{row["episode"]}'
        indexed = self.raw_index['runs'].get(key)
        if indexed is None:
            raise SystemExit('raw_index.json has no entry for ' + key)
        directory = self.raw / row['attempt'] / row['episode']
        result_path, manifest_path = directory / 'result.json', directory / 'manifest.json'
        for path, digest in ((result_path, indexed['result_sha256']),
                             (manifest_path, indexed['manifest_sha256'])):
            if not path.is_file():
                raise SystemExit('original missing: ' + str(path))
            if sha256(path) != digest:
                raise SystemExit('original SHA-256 differs from raw_index.json: ' + str(path))
        raw, manifest = load(result_path), load(manifest_path)
        if raw.get('schema') != RUN_SCHEMA or raw.get('episode') != row['episode']:
            raise SystemExit('unexpected original run record: ' + str(result_path))
        for field, pick in CROSS_CHECKS:
            recorded, original = row.get(field), pick(raw, manifest)
            if not same(recorded, original):
                raise SystemExit(f'{key}: results.json {field}={recorded!r} != original {original!r}')
        return result_path, raw, manifest, indexed['result_sha256']

    # ------------------------------------------------------------------ 파생 뷰 1개
    def add(self, row: dict) -> None:
        result_path, raw, manifest, digest = self.originals(row)
        episode, attempt = row['episode'], row['attempt']
        spec = manifest['spec']
        cohort, cohort_note = COHORTS[attempt]
        carrying = row['condition'] == 'box'
        gates = raw['gates']
        looks = collections.Counter(raw['look_reasons'])
        name = short_name(attempt, row['condition'], spec['seed'])
        frozen_sha = self.frozen['code_commit']
        condition = (
            f'owncam-loop {row["split"]} {attempt} {row["condition"]} seed {spec["seed"]} '
            f'spawn_y {spec["spawn_y"]}; {cohort}; map {spec["map"]} '
            f'(sha {manifest["static_map_sha256"][:8]}); pose_source={POSE_SOURCE}; '
            f'carry={"CARRY_POSTURE(box held)" if carrying else "none(nobox)"}; '
            f'contact {spec["contact_profile"]}; weld {manifest["weld"]}; '
            f'source {manifest["code"]["sha"][:7]} (frozen {frozen_sha[:7]}); '
            'GT는 평가 전용'
        )
        scope = (
            '학생 입력은 자기 손목 fisheye RGB(JPEG q90 640x480)·자기 발행 명령 이력·정적 태그 지도·'
            '고정 교정값·고정 pickup 격자 keep-out뿐이다. nav_cam·TOP·실시간 자세는 쓰지 않는다. '
            f'sim_s={raw["student_sim_s"]} s는 학생 구간 SIM 시간이며 교사 구간'
            f'({raw.get("teacher_sim_s")} s, box 조건만)은 evaluation에 따로 있고 학생 성공에 넣지 않는다. '
            'commands는 학생이 발행한 명령 수이며 실제 관절 상태·이동 성공이 아니다. '
            'model_calls=0은 이 폐루프에 외부 모델 호출이 없다는 뜻이다(결정적 입자 필터 추정 + A* 재계획). '
            'claims/protocol_complete는 학생이 스스로 낸 도착 선언이며 사후 게이트 통과와 분리한다. '
            'evaluation/reported_success는 사전 등록 게이트 R1·R2·R3'
            + ('·R4' if carrying else '') + '의 episode_pass다. '
            'evaluation/robot_robot_contact_samples는 학생 구간의 동료 로봇 접촉 기록 수(contacts_student.peer_robot)다. '
            f'{cohort_note}.'
        )
        limits = (
            '조건·시드당 1회이며 분할·조건을 합산하지 않는다. dev 시도는 모두 보존했고 '
            '보고 dev는 고정 소스 dev-a4, test는 1회 실행이다. 이 스냅샷의 수치는 시뮬레이션 기록이며 '
            '실물 MasterPi 검증이 아니다. 사후 분석(멈춤 관성 재적합 후보)은 이 코호트의 사전 등록 분석이 '
            '아니며 원본 JSON이 없어 수치로 변환하지 않았다.'
        )
        evaluation = {
            'evaluation_only_ground_truth': True,
            'episode_pass': raw['episode_pass'],
            'gates': gates,
            'gate_definitions': self.prereg['gates'],
            'student_sim_s': raw['student_sim_s'],
            'teacher_sim_s': raw.get('teacher_sim_s'),
            'teacher_part': self.prereg['teacher_part'],
            'looks': raw['looks'],
            'look_reason_counts': dict(sorted(looks.items())),
            'student_commands': raw['student_commands'],
            'student_frames': raw['student_frames'],
            'student_tag_visibility': raw['student_tag_visibility'],
            'contacts_student': raw['contacts_student'],
            # 변환기가 스칼라로 내보내는 이름. 학생 구간 동료 로봇 접촉 기록 수다.
            'robot_robot_contact_samples': raw['contacts_student']['peer_robot'],
            'robot_robot_contact_samples_note': (
                'contacts_student.peer_robot 기록 수이며 MuJoCo 접촉 표본 수가 아니다'),
            'final_gt': raw['final_gt'],
            'goal_m': raw['goal_m'],
            'robot_id': raw['robot_id'],
            'run_spec_not_ground_truth': spec,
            'load_average': manifest['load_average'],
            'wall_s': manifest['wall_s'],
            'threads': manifest['env']['threads'],
            'hashes': {k: manifest[k] for k in (
                'static_map_sha256', 'landmarks_sha256', 'base_static_map_sha256',
                'scene_xml_sha256', 'calibration_sha256', 'keepouts_sha256')},
            'timing': {k: manifest[k] for k in ('timestep_s', 'frame_period_s', 'control_period_s', 'sync_sim')},
            'env': {k: manifest['env'][k] for k in ('python', 'platform', 'mujoco', 'opencv', 'numpy')},
            'prereg_sha256': self.amendments['prereg_sha256'],
            'dev_amendments_before_test': [
                {k: a[k] for k in ('commit', 'field', 'why')} for a in self.amendments['amendments']
            ],
        }
        offline_scalars = {
            'offline/r1_gt_distance_m': gates['R1_exit_waypoint']['gt_distance_m'],
            'offline/r2_wall_contacts': gates['R2_no_wall_contact']['wall_contacts'],
            'offline/r3_p90_pos_m': gates['R3_estimate_near_door']['p90_pos_m'],
            'offline/r3_p90_yaw_deg': gates['R3_estimate_near_door']['p90_yaw_deg'],
            'offline/r3_frames': gates['R3_estimate_near_door']['frames'],
            'offline/r3_uninitialized': gates['R3_estimate_near_door']['uninitialized'],
            'offline/looks': raw['looks'],
            'offline/student_frames': raw['student_frames'],
            'offline/tag_visibility': raw['student_tag_visibility'],
            'offline/peer_robot_contacts': raw['contacts_student']['peer_robot'],
            'offline/other_box_contacts': raw['contacts_student']['other_box'],
        }
        if carrying:
            offline_scalars['offline/r4_min_box_z_m'] = gates['R4_box_held']['min_box_z_m']
            offline_scalars['offline/r4_end_box_z_m'] = gates['R4_box_held']['end_box_z_m']
            offline_scalars['offline/teacher_sim_s'] = raw['teacher_sim_s']
        view = {
            'derived_view_only': True,
            'derived_from': f'{result_path}#result.json',
            'source_result_sha256': digest,
            'offline_source': {'path': str(result_path), 'sha256': digest},
            'offline_source_pointer': 'result.json',
            'records': {
                'experiment': str(self.results_path),
                'row': f'runs[{self.results["runs"].index(row)}]',
                'results_sha256': sha256(self.results_path),
                'manifest_sha256': sha256(result_path.parent / 'manifest.json'),
            },
            'condition': condition,
            # split·시도·조건·코호트를 중복 없이 한 칸에 담아 HParams에서 걸러 본다.
            'case': '|'.join(dict.fromkeys([row['split'], attempt, row['condition'], cohort])),
            'policy': f'{POLICY}|pose_source={POSE_SOURCE}',
            'seed': spec['seed'],
            'goal': (f'door-exit waypoint W=({raw["goal_m"][0]:.2f}, {raw["goal_m"][1]:.2f}) '
                     f'through door_1 of {spec["map"]}'),
            'clock': 'sim(synchronous)+wall',
            'source_sha': manifest['code']['sha'],
            'config': {'contact_profile': spec['contact_profile']},
            'success': raw['episode_pass'],
            'success_definition': (
                'preregistered episode_pass = R1(도착 선언 시 GT-W 거리 ≤ 0.10 m) and '
                'R2(학생 구간 벽 접촉 0) and R3(문 영역 추정 오차 p90 < 0.06 m, 미초기화 0)'
                + (' and R4(상자 z > 0.045 m 유지)' if carrying else '')
                + '; 정답은 사후 평가 전용이며 실행 중 학생에게 주지 않았다'
            ),
            'stop_reason': raw['outcome'],
            'protocol_complete': bool(gates['R1_exit_waypoint']['declared_arrival']),
            'sim_s': raw['student_sim_s'],
            'wall_s': manifest['wall_s'],
            'commands': raw['student_commands'],
            'model_calls': 0,
            'scope': scope,
            'limits': limits,
            'evaluation': evaluation,
            'offline_scalar_scope': (
                '원본 실행 기록에 있는 평가 전용 게이트 수치와 둘러보기·태그 가시율·접촉 기록이다. '
                '로봇 임무 성공이나 실물 성능이 아니다.'
            ),
            'offline_scalars': offline_scalars,
        }
        directory = self.output / name
        directory.mkdir(parents=True, exist_ok=False)
        (directory / 'result.json').write_text(json.dumps(view, ensure_ascii=False, indent=1) + '\n')
        self.rows.append({
            'run': name, 'source': str(directory), 'attempt': attempt, 'episode': episode,
            'split': row['split'], 'carry': row['condition'], 'seed': spec['seed'],
            'cohort': cohort, 'condition': condition,
            'origin': str(result_path), 'pointer': 'result.json',
            'origin_sha256': digest, 'success': raw['episode_pass'], 'outcome': raw['outcome'],
        })

    def build(self) -> dict:
        for row in self.results['runs']:
            self.add(row)
        index = {
            'schema': SCHEMA,
            'purpose': '자기 카메라 폐루프 문 통과(PR #178) 22회의 TensorBoard 파생 뷰',
            'records': {
                'root': str(self.records),
                'files': {name: sha256(self.records / name) for name in (
                    'results.json', 'raw_index.json', 'prereg.json', 'prereg_amendments.json',
                    'frozen_source.json')},
            },
            'raw_root': str(self.raw),
            'frozen_source_sha': self.frozen['code_commit'],
            'attempts': self.results['attempts'],
            'summary_k_of_n': self.results['summary_k_of_n'],
            'rows': self.rows,
        }
        (self.output / 'index.json').write_text(json.dumps(index, ensure_ascii=False, indent=1) + '\n')
        return {'views': len(self.rows), 'root': str(self.output),
                'cohorts': sorted({row['cohort'] for row in self.rows})}


def rename_snapshot(snapshot: Path, index: Path) -> dict:
    """내보낸 run 폴더를 짧은 이름으로 바꾸고 원래 이름·조건을 collection.json에 남긴다.

    TensorBoard는 폴더 이름을 run 이름으로 쓰므로 라벨만 바뀌며 각 run의
    manifest.json과 원본 경로는 그대로다.
    """
    rows = {row['source']: row for row in load(index)['rows']}
    collection_path = snapshot / 'collection.json'
    collection = load(collection_path)
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
        for key in ('condition', 'cohort', 'split', 'carry', 'seed', 'attempt', 'episode',
                    'origin', 'pointer', 'origin_sha256'):
            entry[key] = row[key]
    collection_path.write_text(json.dumps(collection, ensure_ascii=False, indent=2) + '\n')
    return {'renamed': len(collection['exported']), 'failed': len(collection.get('failed') or [])}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--records', type=Path, help='PR #178 실험 기록 폴더(읽기 전용)')
    parser.add_argument('--raw', type=Path, help='원본 실행 루트 outputs/owncam-loop-20260925(읽기 전용)')
    parser.add_argument('--output', type=Path, help='새 파생 뷰 루트(이미 있으면 거부)')
    parser.add_argument('--rename-snapshot', type=Path, help='짧은 run 이름으로 바꿀 스냅샷 폴더')
    parser.add_argument('--index', type=Path, help='--rename-snapshot에 쓸 파생 뷰 index.json')
    args = parser.parse_args()
    if args.rename_snapshot:
        if not args.index:
            parser.error('--rename-snapshot needs --index')
        print(json.dumps(rename_snapshot(args.rename_snapshot.resolve(), args.index.resolve()),
                         ensure_ascii=False))
        return 0
    if not (args.records and args.raw and args.output):
        parser.error('--records, --raw, --output are required')
    records, raw, output = args.records.resolve(), args.raw.resolve(), args.output.resolve()
    if output.exists():
        parser.error('파생 뷰 루트는 새 경로여야 한다; 원본과 기존 뷰는 덮어쓰지 않는다')
    if output.is_relative_to(raw) or output.is_relative_to(records):
        parser.error('파생 뷰 루트는 원본·기록 폴더 밖이어야 한다')
    output.mkdir(parents=True)
    print(json.dumps(Builder(records, raw, output).build(), ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
