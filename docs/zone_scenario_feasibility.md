# 구역 시나리오 실현 가능성 오프라인 검사

물리적으로 풀 수 없는 시나리오가 본실험에 들어가지 않게 막는 오프라인 검사다. 시나리오 설정(JSON)과
`maps/zones/`의 정적 지도, `sim.zone_cargo` 화물 목록만 읽는다. MuJoCo·장면 생성·모델 호출·시뮬레이터
상태는 전혀 쓰지 않으며 시나리오 파일을 고치지도 않는다.

- 모듈: `harness/zone_scenario_feasibility.py`
- CLI: `scripts/check_zone_scenarios.py`
- 회귀 검사: `tests/test_zone_scenario_feasibility.py`

## 왜 필요한가 — PR #169의 두 막힘

[2026-09-25 구역 팀 작업 A2](../experiments/2026-09-25-zone-team-a2/README.md) §7의 교사 실현 가능성
스모크에서 두 실행이 **실행기 문제가 아니라 시나리오 자체 때문에** 끝까지 가지 못했다.

| # | 시나리오 | 무엇이 막혔나 | 이 검사가 내는 판정 |
|---|---|---|---|
| B2 | `tri_frame` × `zone_wide_door` | 3인 편대 폭이 0.5 m 문보다 넓어 접촉 전에 332번 거부되고 1800 SIM초를 다 썼다 | `no_carry_route` (실현 불가능). 근거는 편대 최소 폭 0.818 m(맨 발자국) > 개구부 0.50 m |
| B8 | `tri_frame` × `zone_wide_two_doors` | 넓은 문 앞의 색 상자가 3인 편대의 유일한 경로를 막았고, 세 로봇이 모두 팀 물건에 묶여 치울 로봇이 없었다 | `clearing_needs_precedence` (조건부). 동시 필요 로봇 3+1=4대 > 3대이므로 상자를 **먼저** 배달해야 열린다 |

두 판정 모두 설정과 지도만으로 결정되므로 시뮬레이션을 돌리기 전에 알 수 있다.

## 무엇을 검사하는가

1. **운반 경로** (`carry_route`) — 물건의 운반 편대 발자국(물건 + 운반자별 섀시·팔,
   `harness.zone_team_footprint`)이 설정 자세에서 목적 구역 안에 내려놓을 수 있는 자세까지 갈 수 있는가.
   물건 자세 (x, y, yaw) 공간의 너비 우선 탐색이므로 **회전도 경로에 포함**된다. 찾은 경로는
   `harness.static_keepouts.swept_clear`로 구간마다 다시 검사한다.
2. **초기 배치 막힘** (`check_placement_blocking`) — 다른 물건의 초기 배치가 있을 때만 경로가 막히면,
   어떤 배치가 원인인지 하나씩 빼서 찾고, 그 배치를 **치울 로봇이 있는지** 센다.
3. **로봇 수** (`check_robot_count`) — 물건별 필요 인원, 팀 물건 필요 인원 합, 막힘 해소에 동시에
   필요한 최대 인원을 로봇 3대와 비교한다.
4. **숨은 사건** (`check_hidden_events`) — 비공개 `passage_blocked`·`obstruction_added`·`item_moved`가
   일어난 뒤에도 경로가 남는가. 남지 않으면 시나리오가 그것을 **의도된 불가능으로 선언**했는지 본다.
5. **파지 자리** (`check_free_robot_access`) — 빈 로봇 원판(반지름 0.17 m,
   `scripts.zone_teacher.ROBOT_RADIUS_M`)이 물건의 각 파지 자리에 설 수 있는가, 그 여유가 몇 m인가.

## 경로 판정 세 가지

| 판정 | 뜻 | 근거 |
|---|---|---|
| `가능` (feasible) | 계획 여유 0.03 m(`TEAM_MARGIN_M`) 발자국으로 경로가 있고 구간 sweep 검사도 통과했다 | 실제 경로 하나를 제시한다 |
| `여유 부족` (tight) | 맨 발자국으로만 경로가 있거나, 절반 격자에서만 있거나, sweep 검사에서 벽에 닿는다 | 기하학적으로는 가능하지만 실행기의 자기 여유가 거부한다. SIM 확인 대상 |
| `불가능` (infeasible) | 맨 발자국으로도 경로가 없다 | `proof` 필드로 구분: `passage_width`(편대 최소 폭 > 모든 개구부, 해상도와 무관한 하한), `start_pose`(출발 자세부터 막힘), `lattice_search`(절반 격자 재확인까지 실패) |

`min_width`는 볼록 껍질의 성질이므로 **어떤 방향으로 돌려도 필요한 최소 틈**의 하한이다. 0.889 m 편대가
0.50 m 문을 지날 수 없다는 판정은 격자 해상도와 무관하다.

## 시나리오 판정과 종료 코드

- `실현 가능` — `infeasible`·`warning` 발견 사항이 없다.
- `조건부 실현 가능` — `warning`이 있다. 순서 강제(`clearing_needs_precedence`), 여유 부족 경로,
  밀리미터 단위 파지 자리 여유가 여기 들어간다. 실행기·선언 정책이 그 조건을 지켜야 한다.
- `실현 불가능` — `infeasible`이 하나라도 있다. CLI가 종료 코드 1을 낸다(`--strict`는 경고에도 1).

## 쓰는 법

```bash
# 스레드 1개 환경에서 돌린다(다른 SIM 작업과 부하를 나누지 않기 위해).
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1
python3 scripts/ugrp_session.py run scenario-check -- \
  .venv-sim-worker-mac/bin/python scripts/check_zone_scenarios.py \
    --scenario-dir configs/zone_study_scenarios \
    --json outputs/scenario-feasibility.json --markdown outputs/scenario-feasibility.md
```

주요 옵션: `--scenario FILE`(반복 가능), `--margin-m`, `--grid-m`, `--yaw-steps`, `--no-swept`,
`--strict`, `--quiet`. 검사 6종 기준 전체 실행은 Mac에서 40초 내외다(부하 평균 4–6 기록).

파이썬에서 직접:

```python
from harness.zone_scenario_feasibility import evaluate, report_ko
report = evaluate(scenario_dict)          # scenario_dict: 설정 JSON을 읽은 것
print(report.verdict, [f.code for f in report.findings])
print(report_ko(report))                  # 한국어 보고서
report.raise_for_verdict()                # 불가능하면 ScenarioInfeasible
```

## 입력 형식

두 가지를 받는다.

- **연구 시나리오 설정**(패키지 E): 공개부 `orders`(`kind`, `count`, `required_robots`,
  `destination_zone`)와 비공개부 `eval.setup.placements`(`item_id`, `kind`, `order_id`, `pose_m`),
  `eval.hidden_events`.
- **단순 형식**: `{"map_id": ..., "items": [{"item_id", "kind", "pose_m", "destination_zone"}]}`.

`eval` 아래를 읽으므로 **이 검사의 출력은 평가 전용**이다. 보고서·JSON을 로봇 프롬프트에 넣지 않는다.

### 의도된 불가능 선언

사건 뒤 경로가 사라지는 것이 시나리오의 목적이라면 비공개부에 선언한다. 그러면
`no_route_after_event`(불가능)가 `declared_unsolvable_after_event`(정보)로 바뀐다.

```json
"eval": {
  "feasibility": {
    "intentionally_unsolvable": false,
    "items": ["crate_1"],
    "events": ["door_narrow_blocked"],
    "notes_ko": "막힘 발견과 보고만 재는 시나리오이며 배달 완료를 성공 조건으로 쓰지 않는다."
  }
}
```

## 검사하지 않는 것

판정은 **필요조건**이다. 통과가 시행 성공을 뜻하지 않는다.

- 파지·접촉 성공, 화물 미끄러짐, weld 없는 공동 파지의 물리
- 자기 손목 어안 카메라의 위치 추정과 인식
- 실행기의 선언·순서·양보 정책 (B8은 기하학적으로 조건부 가능이지만, 순서를 지키지 못하는 실행기에서는
  여전히 막힌다)
- 격자(기본 0.05 m, yaw 12단계)보다 좁은 틈, 절반 격자 재확인으로도 남는 해상도 한계
- 낙하 지점처럼 실행 시점 물리로 정해지는 위치 (`item_dropped`는 기하 변화가 없다고 기록만 한다)
- 숨은 막힘이 정말 반대편에서 안 보이는지 (자기 카메라 가시성은 별도 검증 대상)

## 2026-09-26 패키지 E 시나리오 6종 검사 결과

`kiro/zone-study-scenarios`(PR #190, `3186100`)의 설정을 읽기만 해서 검사했다. 지도는 main의
`maps/zones/*_tags_v1.json`이다.

| 시나리오 | 판정 | 내용 |
|---|---|---|
| s1_normal_mixed | 실현 가능 | 물건 7개 모두 `door_1` 경로 가능, 막힘·순서 제약 없음 |
| s2_unmapped_blockage | 실현 가능 | `door_narrow_blocked`(45 SIM초) 뒤에도 4개 모두 `door_wide`로 경로 유지 |
| s3_late_rendezvous | 실현 가능 | `r3_hold_late`는 기하 변화가 없어 경로 재검사 대상 아님 |
| s4_narrow_door_standoff | 실현 가능 | 빔 편대(최소 폭 0.27 m)가 0.5 m `corridor_1`을 통과, 구간 sweep 통과 |
| s5_moved_dropped_item | 실현 가능 | `cyan_1_moved` 새 자세(P1-3)에서도 구역 A 경로 유지 |
| s6_novel_relation | **조건부** | `can_1` 파지 자리 여유가 0.0017 m. 설계 의도(빔이 서쪽 접근을 막아 순서가 필요)와 측정 기하가 어긋난다 |

s6의 어긋남: 설정 주석은 섀시 폭 0.39 m로 x구간 [1.10, 1.49]가 막힌다고 보았으나,
`harness.zone_team_footprint`의 측정 섀시는 기준점 앞 0.120 m·뒤 0.100 m다. 그래서 base x=1.295의
섀시는 [1.195, 1.415]이고 `beam_1`의 [1.08, 1.12]와 겹치지 않는다. 교사 계획 원판(반지름 0.17 m)으로도
1.7 mm 남는다. 즉 **접근 불가가 아니라 밀리미터 여유**이며, 0.39 m는 적재 로봇의 회전 외피(문 폭 0.5 m의
근거)여서 정지 상태 발자국으로 쓸 수 없다. 시나리오 의도를 유지하려면 두 물건 간격을 줄이거나
`beam_1`을 동쪽으로 옮겨 여유를 음수로 만들어야 한다. 이 판단은 정적 기하이며 실제 주행 진입은
SIM으로 확인해야 한다.

## 관련 문서

- [구역 팀 작업 A2 실험 기록](../experiments/2026-09-25-zone-team-a2/README.md) — B2·B8 원본
- [실행 버전 관리](execution_versioning.md) — 검사 manifest에 지도 해시·격자·여유를 남기는 이유
- [AGENTS.md](../AGENTS.md) — 로봇 입력 경계(이 검사 출력은 평가 전용)
