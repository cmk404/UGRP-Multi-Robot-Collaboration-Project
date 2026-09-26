# 2026-09-26 구역 교사 막힘 수정: PR #169 B2·B3·B4·B5·B8 (교사 실현 가능성)

**판정 범위(모든 절에 적용):** 이 기록의 모든 이동은 정답 교사(시뮬레이터 자세·접촉력을 읽는 주행·IK·실제 집게, weld OFF)가 한다. 교사는 시연·학습 표적·시나리오 실현 가능성에만 쓴다. **연구 실행기가 아니며, 교사 성공은 로봇·RGB 스킬·학생 성공이 아니다.** 선언은 fixture(LLM 호출 0회)다. 로봇에게 가는 결과는 여전히 교사 영수증이다(`teacher_receipt_L4`, L4 미해결).

- 브랜치 `kiro/zone-teacher-fix`(Kiro 작업), `claude/zone-team-a2`(PR #169, OPEN) 위에 쌓은 PR.
- 막힘 정의: [PR #169 기록 §7](../2026-09-25-zone-team-a2/README.md).
- 접촉 프로필: **`cargo_noslip_v1`(사용자 승인 대기)**. PR #169가 지원하고 A2 스모크가 쓴 프로필이다.
- 실행 번들 ID: 쓰지 않음(`harness/rgb_execution_bundle.py` 변경 없음). 새 실행 경로 `scripts/run_zone_teacher_fix.py`를 `configs/simulation_workflows.json`에 `zone-teacher-fix` 1.0.0으로 등록했다(PR #169 `zone-dispatch` 1.2.0의 교사 진단 어댑터, 목록 개수 테스트 30 → 31).

## 1. 무엇을 바꿨나 (PR #169 소유 파일은 수정하지 않음)

PR #169 소유 파일(`scripts/zone_team_teacher.py`, `scripts/zone_dispatch_v2.py`, `scripts/run_zone_dispatch.py`, `harness/zone_protocol_v2.py`, `harness/zone_outcomes_v2.py`, `tests/test_zone_team_a2.py`)은 한 줄도 바꾸지 않았다. **`CONDITIONS`·`ModeLoop`·랑데부 규칙·선언 검사·프롬프트는 바뀌지 않는다.** 수정은 하위 클래스와 새 실행기에만 있다.

| 막힘 | 파일 | 내용 |
|---|---|---|
| B2·B8 | `harness/zone_teacher_gate.py` | 물리 실행 전 시나리오 설정(설정 전용 자세·정적 지도·착지 배치)만 읽는 팀 경로 검사. 교사 commit과 같은 계획기·발자국·keep-out·장애물 모양·목표 yaw를 쓴다. 판정 `ok` / `order_constrained`(단독 물건을 먼저 치워야 경로가 생김, 최소 원인 목록) / `infeasible`(단독 물건을 모두 빼도 경로 없음) |
| B3·B4 | `scripts/zone_team_teacher_fix.py` | PR #169 교사의 하위 클래스(`FixZoneTeamExecutor`, `FixTeamRobot`, `FixTeamCarry`). 스위치 `b3_claim_yield`, `b4_return_setdown`(기본 켬). 기록은 `result.json`의 `team_executor.teacher_fix` |
| B5 | `harness/zone_label_tracking.py`, `harness/zone_perception_v2.py` | 새 인식 프로필 `top_cargo_v2_track`(선택 시에만). `top_cargo_v1`·`top_cargo_v2`는 바뀌지 않는다 |
| 실행기 | `scripts/run_zone_teacher_fix.py` | PR #169 인자 그대로 + `--teacher-fix`, `--feasibility-gate enforce|report`. `enforce`는 `infeasible`·`order_constrained`를 물리 실행 없이 거부한다(종료 코드 3). 실행기 클래스만 바꾼다 |
| 테스트 | `tests/test_zone_teacher_fix.py` | 15개 (CI 목록에 추가) |

## 2. 사전 등록 (실행 전 커밋)

이 절은 코호트 실행 전에 커밋했다. 실행 뒤 이 절은 고치지 않는다.

**고정 조건:** 소스 = 이 절을 담은 커밋(결과에 `source_sha` 기록), fixture, LLM 0회, 동기 SIM, weld OFF, `cargo_noslip_v1`(사용자 승인 대기), 인식 `top_cargo_v2_track`, `--record-replay`, 교사 수정 스위치 모두 켬(D-B4만 예외), 게이트 `enforce`, SIM 한도 1800초(D-B4만 900초), 스레드 1개 환경, 동시 실행 최대 2개, 실행마다 1분 부하 평균 기록. 실행: `run_cohort.py`. 출력: `outputs/zone-teacher-fix-20260926/`(이 worktree, 로컬만).

**새 seed 코호트(PR #169가 쓰지 않은 seed 21–23):**

| id | 지도 | 목표 | seed | 조건 |
|---|---|---|---|---|
| mix-{dynamic,independent}-s{21,22,23} | `zone_wide_two_doors` | MIXED(A long_beam 1 / B heavy_crate 1 + red 1 / C can 1 + green 1 + tile 1) | 21, 22, 23 | dynamic, independent (6회) |
| tri-dynamic-s{21,22} | `zone_wide_two_doors` | TRI(A tri_frame 1 + red 1 / C green 1) | 21, 22 | dynamic (2회) |

seed 선택 근거(실행 전 오프라인 게이트): MIXED 21–24·26–28 `ok`, 25 설정 실패(can 자리 없음). TRI 21–23·25·27·28 `ok`, 24·26 `order_constrained`. 목록 앞쪽의 `ok` seed를 골랐다. seed 24·26 TRI는 게이트 거부 예시로만 쓴다.

**막힘 재실행(옛 seed, 수정 증거용. 새 seed 코호트에 합산하지 않음):**

| id | 설정 | 목적 |
|---|---|---|
| d-b3-mix-dynamic-s12 | PR #169 C1 `a-two-dynamic-s12`와 같은 지도·목표·seed·조건, 수정 모두 켬 | B3 |
| d-b4-mix-dynamic-s12-b3off | 같은 설정, `b3_claim_yield` 끔, SIM 한도 900초 | B4(B3를 일부러 남겨 `carry_blocked`를 재현) |
| g-b2-door-tri-s11 | `zone_wide_door` TRI s11(C1 b-door-tri) | B2 게이트 거부 |
| g-b8-two-tri-s11 | `zone_wide_two_doors` TRI s11(C2 b-two-tri) | B8 게이트 거부 |

**B5 오프라인 검사:** PR #169 C1 `a-two-dynamic-s12`의 저장된 TOP 영상(빔이 픽업 자리 밖에 내려진 뒤)으로 `top_cargo_v2`와 `top_cargo_v2_track`의 `observe_items`를 두 번씩 돌려 비교한다. 물리 실행 없음.

**판정 기준(교사 실현 가능성):**
- 실행 성공 = `referee_v2.goal_met`, `eq_active_max` = 0, 떨어뜨림 0, 물건당 차감 1회, 이중 소속 0.
- B3 수정됨: d-b3에서 `carry_blocked` 없음, 빔 운반 멈춤 합계 < 60 SIM초, 목표 달성.
- B4 수정됨: d-b4에서 `carry_blocked` 뒤 `return_finished`, 내린 자리가 빔 시작 자세에서 0.10 m 안. (B3를 끈 실행이므로 목표 달성은 기준이 아니다.)
- B2·B8: 게이트가 각각 `infeasible`·`order_constrained`로 거부(물리 실행 없음). **B2·B8의 선언 순서 문제 자체(fixture가 팀 물건을 먼저 고름)는 PR #169 소유 파일(`harness/zone_protocol_v2.py`)에 있어 고치지 않는다.** 교사 수준의 해결책은 거부뿐이다.
- B5 수정됨: 오프라인 검사에서 `long_beam-1`이 `top_cargo_v2_track`에서만 다시 선언 가능 목록에 들어온다.
- 모든 결과(실패 포함)를 보고한다. 이 코호트에는 수정 전 교사 기준선이 없다(새 seed의 수정 전후 비교는 하지 않음).

## 3. 실행 전 진단(기록하지만 결과에 합산하지 않음)

소스 `964ee71`에서 s12 dynamic을 150 SIM초 돌린 디버그 실행(`/tmp`, 삭제)에서 두 문제를 찾아 `b3` 수정을 보완했다.
- PR #169의 비키기 자리는 팀의 다음 12 SIM초 경로만 피한다. 빔 경로가 북쪽 x ≈ 0.95 m 열을 따라가므로 r3는 같은 경로 앞쪽((1.0, −1.0) → (1.0, −0.7) → … → (1.1, 0.7))으로 20초마다 밀려났고, 팀은 150초 동안 12번 멈췄다. PR #169 원 실행의 비키기 자리도 같았다((1.0, −1.0), (1.0, −0.7), (1.1, −0.4)). 수정: 남은 경로 전체를 피하는 자리, 없으면 PR #169 자리.
- 정거장에 선 채 대기(staged)하면 랑데부 규칙이 매 tick 도착을 다시 기록했다. 수정: 비키는 중·대기 중인 로봇은 그 tick의 정거장 점유자에서 뺀다. 이 디버그 뒤에 대기 중 비키기로 넘어갈 때 대기 표시가 바로 풀리던 분기 오류(`elif`)도 고쳤다.

소스 `9fa5e73`의 240 SIM초 디버그(`/tmp`, 삭제)에서는 빔 운반 멈춤이 1번(2.3초)이었고 빔이 190 SIM초에 배달됐다(PR #169 원 실행: 3번 멈춤, 382초 `carry_blocked`). 이 디버그는 사전 등록 재실행이 아니므로 판정에 쓰지 않는다.

## 4. 결과

(실행 뒤 추가)

## 5. 검증 범위

(실행 뒤 추가)

## 6. 남은 일

- workflow 개수 테스트(`tests/test_simulation_workflow_manager.py`, 30 → 31)는 다른 브랜치의 workflow 추가와 병합 충돌할 수 있다. 나중에 병합되는 쪽이 개수를 다시 맞춘다.
