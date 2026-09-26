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

**다시 적는 판정 범위:** 아래 숫자는 모두 정답 교사의 실현 가능성 결과다. 로봇·RGB 스킬·학생 성공이 아니다. 선언은 fixture이고 LLM 호출은 0회다. 모든 실행은 소스 `3885fdf`에서 했다(각 `result.json`의 `source_sha`). 조건은 동기 SIM, weld OFF(`eq_active_max` 0), `cargo_noslip_v1`(사용자 승인 대기), 스레드 1개, 동시 실행 최대 2개다. 원본은 이 worktree의 `outputs/zone-teacher-fix-20260926/`에 있다(로컬만, 원격 백업 아님). 파일 해시는 [results.json](results.json)에 있다.

이 작업은 첫 Kiro 세션(계정 1)이 코호트 실행 중에 멈췄고, 계정 2에서 이어서 마쳤다. 코호트 드라이버(`run_cohort.py`, 세션 `kiro-teacher-fix-cohort`)는 끊기지 않고 끝까지 돌았다. 다시 실행한 것은 없다.

### 4.1 새 seed 코호트(21–23): 교사 실현 가능성 5/8

| 실행 | referee_v2 목표 | 제어 종료 SIM초 | 선언 라운드 | 못 옮긴 물건 | wall초 / 시작 부하(1분) |
|---|---|---|---|---|---|
| mix-dynamic-s21 | 달성 | 477.0 | 10 | – | 1679 / 49.8 |
| mix-independent-s21 | 달성 | 470.0 | 33 | – | 3147 / 23.1 |
| mix-dynamic-s22 | **미달** | 466.0 | 9 | green 상자 `box_01` | 1698 / 118.2 |
| mix-independent-s22 | **미달** | 491.5 | 39 | green 상자 `box_01` | 1015 / 16.6 |
| mix-dynamic-s23 | 달성 | 497.0 | 14 | – | 2820 / 9.6 |
| mix-independent-s23 | 달성 | 519.0 | 45 | – | 2500 / 12.5 |
| tri-dynamic-s21 | 달성 | 377.5 | 4 | – | 975 / 38.4 |
| tri-dynamic-s22 | **미달** | 327.5 | 3 | green 상자 `box_01` | 657 / 14.6 |

- 8회 모두 `FINISHED`로 끝났고 오류는 없다. `eq_active_max` 0, 떨어뜨림 0, 미끄러짐 최대 0.8 mm, 이중 소속 0, 물건당 차감 1회, 과잉 배달 0이다.
- 실패 3회는 모두 seed 22다. 원인은 새 막힘 **N1**이다(§4.3). 교사 이동이나 B2–B8 때문이 아니다. seed 22의 다른 물건은 모두 배달됐다(MIXED 5/6, TRI 2/3 종류).
- 대기 비용: heavy_crate의 `rendezvous_timeout`이 1–3번 있었다(B6과 같은 무통신 팀 형성 비용). s23에서는 red 상자에 `path_blocked`가 dynamic 4번, independent 2번 있었지만 두 실행 모두 목표를 달성했다. 팀 운반 멈춤은 작업당 합계 최대 10.6초다(heavy_crate, s23 dynamic).
- B3 수정 스위치는 새 seed에서 한 번 동작했다(s22 dynamic, `claim_yield` 1회, 운반 멈춤 2.4초). B4 되돌리기는 새 seed에서 쓰이지 않았다.
- 이 코호트에는 수정 전 교사 기준선이 없다(사전 등록 §2). PR #169 seed 11/12 결과와 합산·비교하지 않는다.

### 4.2 막힘별 원인·수정·재실행 증거

| # | 원인 | 수정(파일) | 재실행 증거 | 판정 |
|---|---|---|---|---|
| B2 | tri_frame 발자국(폭 ≥0.889 m)이 `zone_wide_door`의 0.5 m 문을 지날 수 없다. fixture가 같은 팀 물건을 계속 다시 골라 단독 물건이 굶었다 | 물리 실행 전 게이트 `harness/zone_teacher_gate.py:66` `team_route_feasibility`. 교사 commit과 같은 계획기·발자국·keep-out으로 설정 전용 자세에서 경로를 찾는다. `--feasibility-gate enforce`(`scripts/run_zone_teacher_fix.py:46`)면 물리 실행 없이 거부한다 | `g-b2-door-tri-s11`: `infeasible`(tri_frame_0, 단독 물건을 모두 빼도 경로 없음), `run_started` false, 종료 코드 3, wall 16.9초. PR #169 원 실행은 1800 SIM초·wall 6068초를 쓰고 실패했다 | **교사 수준 해결(거부)**. 선언 순서 자체는 open |
| B3 | PR #169의 `carry_blockers`는 선언이 없는 로봇만 비키게 했다. 선언을 가진 r3는 빔 팀 경로 위 heavy_crate 정거장으로 돌아와 기다렸다. 비키기 자리도 다음 12초 경로만 피해 같은 경로 앞쪽으로 밀려났다 | `scripts/zone_team_teacher_fix.py:239` `carry_blockers`(선언 로봇도 비킴), `:222` `request_team_yield`(남은 경로 전체를 피하는 자리), `:210` `station_in_team_path`와 `:91` `FixTeamRobot`(정거장이 팀 경로 안이면 대기, 비키는 중·대기 중에는 정거장 점유자가 아님, 도착 시각 재설정) | `d-b3-mix-dynamic-s12`: `carry_blocked` 없음. 빔 운반 멈춤 1번 2.3초(기준 < 60), 빔 배달 189.8초. 목표 달성, 제어 종료 471.5초. 수정 이벤트는 `claim_yield` 1, `station_staged` 1, `arrival_reset` 1. PR #169 원 실행: 멈춤 6번 합계 150초, 382초 `carry_blocked`, 1800초 미달 | **수정됨**(사전 등록 기준 충족, seed 12 1회) |
| B4 | `hold_lower`가 그 자리에서 내려 다른 물건의 정거장·통로를 막았다(영구 교착) | `scripts/zone_team_teacher_fix.py:146` `FixTeamCarry.fail`, `:157` `_start_return`. `carry_blocked`/`carry_timeout`이고 두 집게가 유지되면(`:142`), 지나온 경로(`:74` `return_poses`)를 되돌아가 시작 자세에 내린다. 그렇지 않으면 PR #169 `hold_lower` | `d-b4-mix-dynamic-s12-b3off`(B3 일부러 끔): 빔이 6번 멈춘 뒤(합계 331초) 383.3초에 `carry_blocked`. 되돌아가 408.9초에 시작 자세에서 7 mm 떨어진 곳에 내렸다(기준 0.10 m). heavy_crate가 440.1초에 commit됐고, 빔을 `long_beam_0@g2`로 다시 선언해 744.2초에 배달했다. 목표 달성, 865.5초(한도 900) | **수정됨**(기준 충족, seed 12 1회) |
| B5 | 라벨은 시작 위치 근처(DEDUPE 반경)에서만 다시 찾는다. 픽업 자리 밖에 내려진 빔은 선언 가능 목록에서 사라졌다 | 선택 프로필 `top_cargo_v2_track`(`harness/zone_perception_v2.py:36`, `harness/zone_label_tracking.py:38` `track_moved_labels`): 구역 밖 같은 종류 검출이 두 번 같은 자리에 있으면 라벨을 옮긴다. TOP RGB와 저작된 TOP 보정만 읽는다. `top_cargo_v1`·`top_cargo_v2`는 그대로다 | 오프라인([b5-offline.json](b5-offline.json)): PR #169 `a-two-dynamic-s12`의 마지막 TOP 3장에서 `top_cargo_v2`는 끝까지 `long_beam-1`이 목록에 없다. `top_cargo_v2_track`은 두 번째 장에서 라벨을 (0.401, −2.150) → (0.948, −1.438)로 옮겨 다시 선언 가능하다. 사후 채점([offline-addendum.json](offline-addendum.json), 사전 등록 아님): 관찰 전용 재생의 빔 자세 (0.950, −1.437)와 2.3 mm 차이(옛 라벨은 0.90 m) | **수정됨(오프라인)**. 물리 실행에서는 쓰이지 않았다(B4 수정으로 빔이 시작 자세로 돌아감) |
| B8 | 넓은 문 앞(1.6, −2.45) red 상자 `box_01`이 tri_frame의 유일한 경로를 막고, 세 로봇이 모두 팀 물건을 골라 상자를 먼저 옮길 로봇이 없었다 | B2와 같은 게이트. 단독 물건을 뺀 경로 검색으로 `order_constrained`와 최소 원인 목록을 낸다 | `g-b8-two-tri-s11`: `order_constrained`, 최소 원인 `[["box_01"]]`, 물리 실행 없음, 종료 코드 3, wall 31.0초 | **교사 수준 해결(거부)**. 선언 순서 자체는 open |

**B2·B8에서 open으로 남는 것:** fixture 선언 규칙이 팀 물건을 먼저 고르는 순서 문제는 `harness/zone_protocol_v2.py`(PR #169 소유, 현재 main)에 있어 고치지 않았다. 게이트는 그런 시나리오를 거부할 뿐 실행하지 않는다. 로봇에게 "불가능" 알림을 주는 경로도 없다(L4 미해결). `CONDITIONS`·`ModeLoop`는 바꾸지 않았다.

### 4.3 새 막힘 N1: seed 22 시작 TOP에서 green 상자 미검출(open)

- 증상: seed 22(MIXED 2회, TRI 1회)의 시작 라벨에 `green-1`이 없다(`labels_vs_setup.match` false). 선언할 수 없어 `box_01`이 픽업 자리에 남았다. 다른 seed(21, 23) 5회는 라벨이 설정과 일치한다.
- 원인(오프라인, [offline-addendum.json](offline-addendum.json) `n1_start_box_detection`): 상자는 (0.40, −0.05)에 있다(설정, 평가 전용). `top-nw` 시작 영상의 (438, 457) px로, 픽업 도색 위 밝은 부분이다. 색상은 H 68–71로 범위 안이다. 그러나 채도 중앙값이 98이라 `top_zone_v2`의 green 하한 S ≥ 100(`harness/zone_color_boxes.py` `TOP_ZONE_HSV`)에 걸린다. 통과한 화소는 39개뿐이고, 열기·닫기 뒤 조각이 10 px로 면적 하한 50 px 아래다. 채도 하한을 낮춘 확인용 마스크(S ≥ 60, 검사 전용)로는 네 TOP 중 `top-nw`에서만 이 상자가 잡혔다.
- 교사 이동과는 무관하다. 로봇에게 가는 TOP 인식(선언 목록)의 한계다. 그 seed의 시나리오는 이 인식으로는 실현할 수 없다.
- 고치지 않았다. 문턱값을 바꾸면 `top_zone_v2`를 쓰는 모든 경로가 바뀐다. 또 dev/test 분할의 재검증(도색·바퀴 오검출)과 새 사전 등록 실행이 필요하다. 인식 담당 작업에 넘긴다. 임시 방안은 게이트에 "시작 TOP 인식이 설정과 맞는지" 검사를 더하는 것이다(아직 없음).

## 5. 검증 범위

- 확인한 것: 새 seed 8회의 교사 실현 가능성(5/8, 실패 3회는 N1), B3·B4 재실행 각 1회, B2·B8 게이트 거부 각 1회, B5 오프라인 재검출 1건. 모든 물리 실행에서 weld OFF, LLM 0회, 동기 SIM이다.
- 확인하지 않은 것: 로봇·RGB 스킬·학생 성공(교사 결과일 뿐), 수정 전후를 같은 새 seed로 비교하기, B3·B4의 여러 seed 반복(각 seed 12 1회), 물리 실행 안에서의 B5 라벨 이동, 복도 지도, `cargo_noslip_v1` 없이 돌린 결과(사용자 승인 대기).
- wall 시간은 공용 호스트 부하(1분 평균 7.7–118)에 따라 657–9773초였다. SIM 결과는 부하와 무관하다(동기 SIM). d-b4의 9773초는 막힌 운반이 반복 재계획된 탓이다.
- 테스트: `tests/test_zone_teacher_fix.py` 15개와 영향 받는 zone·workflow 테스트(PR 본문에 명령·결과).

## 6. 남은 일

- N1: 시작 TOP 인식 누락. 인식 작업에서 문턱값 재검증, 또는 게이트에 인식-설정 일치 검사 추가.
- B2·B8의 선언 순서(fixture가 팀 물건을 먼저 고름)와 로봇에게 가는 불가능 알림(L4)은 `harness/zone_protocol_v2.py` 쪽 작업이다.
- B3·B4는 seed 12 한 번씩만 확인했다. 다른 배치에서 되풀이되는지는 다음 교사 코호트에서 본다.
- workflow 개수 테스트(`tests/test_simulation_workflow_manager.py`)는 다른 브랜치의 workflow 추가와 병합 충돌할 수 있다. 나중에 병합되는 쪽이 개수를 다시 맞춘다.
