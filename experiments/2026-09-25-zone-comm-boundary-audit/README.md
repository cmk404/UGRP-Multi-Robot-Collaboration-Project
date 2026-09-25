# 2026-09-25 구역 벤치마크 정보·통신 경계 감사 (R1)

[연구 TODO](../../docs/research_todo.md)의 "P0 · R1. 로봇별 정보와 통신 경계 감사"를 [구역 통신 비교(ZC1/ZC2)](../2026-09-25-zone-communication/README.md)에 적용했다. 질문은 independent·plan_first·dynamic 비교가 **메시지 유무만 다른 깨끗한 비교인가**이다.

- 감사 기준: main `3cbf4e4`. 구역 코드(`harness/zone_*`, `scripts/run_zone_dispatch.py`, `scripts/zone_teacher.py`, `scripts/three_robot_runtime.py`, `harness/three_robot_plan.py`, `sim/zone_*`, `maps/zones`)는 ZC2 실행 소스 `7ccd6c3`과 바이트가 같다. 아래 `파일:줄`은 모두 `3cbf4e4` 기준이다.
- 원본: 기본 체크아웃 `outputs/zone-communication-20260925/ZC2-*`(로컬에만 있고 원격 백업 아님). 이번 감사는 읽기만 했다.
- 실행: 제어기·LLM 실행은 **없다**. 렌더링만 하는 반사실 확인 1회를 돌렸다. ZC2-s14 장면을 만들고 SIM 0.5초 안정화한 뒤 상자 qpos를 옮기고 `mj_forward` 후 TOP을 다시 렌더링했다(부하 평균 1분 3.44).
- 도구: [audit_requests.py](audit_requests.py)(저장 요청·wire 대조, 채널 계량), [render_counterfactual.py](render_counterfactual.py). 결과·해시는 [results.json](results.json)에 있다. ZC2 요청·wire 파일 818개의 해시 목록은 [cohort-request-manifest.sha256](cohort-request-manifest.sha256)이다.

## 1. 로봇 × 채널 표

세 로봇은 대칭이다. 채널 구성은 로봇마다 같고 내용만 자기 것이다. 표는 조건별 차이를 보인다. "시각"은 해당 필드가 만들어지는 SIM 시각이다.

| 채널 | 출처 | 시각 | independent | plan_first | dynamic |
|---|---|---|---|---|---|
| 자기 RGB `CURRENT OWN RGB` | 로봇 카메라 렌더 (`run_zone_dispatch.py:97-108`) | 질문 직전 | O | 협상 라운드마다 | O |
| TOP RGB 4장 | 고정 TOP 렌더 (같은 곳) | 질문 직전 | O | O | O |
| 임무·목표·지도 설명(system) | `actor_task`, `_COMMON` (`zone_coordination.py:23-32`) | 고정 | O | O | O |
| 조건 블록(system) | `_SOLO` / `_PLAN` / `_CLAIM` | 고정 | 무통신 안내 | 제안·수락 규칙, "balance" | 선언 규칙, **id 동률 규칙** (C2) |
| `box_labels[*].rgb_floor_xy_m` | 첫 TOP 영상 색 검출 (`zone_perception.py:95-105`) | SIM 0.5초 | O | O | O |
| `rgb_view` (남은 상자, 구역 색 개수) | TOP RGB 검출 (`zone_perception.py:108-128`) | 질문 직전 | O | O(협상 시작 시 1회 값) | O |
| `own_jobs` (최근 8개) | 드라이버 발행 기록 + 실행기 영수증 (`run_zone_dispatch.py:198-234`) | 발행·종료 시 | O | 협상 중 비어 있음 | O |
| └ `status` 영수증 | **정답 기반 교사 결과** (L4) | 작업 종료 | O | (사용 안 함) | O |
| └ `slot` | 모든 로봇이 쓰는 칸 풀 (`run_zone_dispatch.py:129-145`) | 발행 | O → **이번 수정으로 제거** (L2) | O → 제거 | O → 제거 |
| `request_id` | 런타임 id (`three_robot_runtime.py:141`) | 질문 | 전체 라운드 번호 → **자기 번호로 수정** (L3) | 협상 턴 | 전체 라운드 번호 |
| `invalid_reason` | 호스트의 자기 선언 검사 | 재질문 | 자기 시야 기준 | — | 동료 선언까지 기준 |
| `conflict` | 호스트 충돌 통지 | 재질문 | — | — | O |
| `team_board.active` | **호스트가 유지하는 동료 선언(의도)** | 매 라운드 | — | 있으나 비어 있음 | O (C1) |
| `team_board.finished/stopped_reports` | 모든 로봇의 교사 영수증 | 매 라운드 | — | 비어 있음 | O (C1, L4) |
| `peer_messages` (최근 8개) | 동료 답의 `message` (`three_robot_runtime.py:164-172`) | 라운드 뒤 | — (`recipients=[]`, 답에 필드 없음) | 협상 발언 | 모든 선언 답을 전원에게 |
| `agreement` (제안·plan_hash) | 합의 객체 | 협상 | — | O | — |
| 행동에 영향(글자 아님): 상자 선점 잠금 | 교사가 동료 작업 상태를 읽음 (`zone_teacher.py:250-263`) | 매 틱 | **실제로 작동** (L1) | 상자가 겹치지 않아 작동 안 함 | 호스트 검사로 거의 작동 안 함 |
| 깨우기 규칙 | 드라이버 | — | 자기 타이머 10초 + L1 멈춤 | 합의 뒤 없음 | 누구든 작업이 끝나면 전원 깨움, 정체 재질문 ≤2 (C3) |
| 모델·설정 | wire 본문 | — | gemini-3.8-flash, max_tokens 1400, temperature 0.2 | 같음 | 같음 |

저장 요청 확인: ZC2의 모델 호출 274건 모두에서 wire로 실제 보낸 본문과 저장 요청이 같았다(system, user JSON, 이미지 이름, 이미지 바이트). independent 142건의 user JSON 키는 `box_labels, own_jobs, request_id, rgb_view, robot_id`뿐이다. dynamic에는 `team_board, peer_messages`(재질문 때 `conflict, invalid_reason`)가, plan_first에는 `agreement, team_board, peer_messages`가 더 있다. system 프롬프트는 조건마다 한 가지이며(로봇 id 제외), 모든 조건이 같은 이미지 5장을 받는다.

## 2. 코디네이터 요청 세 가지 판정

**(1) `rgb_floor_xy_m`·`rgb_view`의 출처 — 깨끗함.**
- 경로: `ZoneRun.tops()`(`run_zone_dispatch.py:93-95`, `render_team_jpeg`) → `detect_all`(`zone_perception.py:47-87`; HSV 색 덩어리 + 작성된 카메라 보정 `pixel_to_floor`) → `label_pickup`(95-105) → `observe`(108-128). 입력은 JPEG, `static_map`, 라벨뿐이다.
- `static_map`의 키는 `bounds_m, top_cameras, obstacles, terrain, regions, zone_slots, box_kinds` 등이다. 상자 자세는 없다. 상자 자세는 `setup_only`에만 있고, 이를 읽는 곳은 교사(`resolve_label`, `zone_teacher`)와 심판뿐이다.
- 반사실 확인: ZC2-s14 장면에서 cargo_box_00을 정답 위치 기준 (+0.037, −0.023) m 옮겼더니 RGB 추정이 (+0.036, −0.025) m 움직였다. cargo_box_03을 (−0.051, +0.029) m 옮기면 추정은 (−0.048, +0.030) m 움직였다. 옮기지 않은 6개는 그대로였다. 추정 오차는 0–9.4 mm다.
- seed 14 렌더링에서 만든 라벨은 ZC2-s14 여섯 실행의 `box-labels.json`과 모두 같았다.
- 값이 격자처럼 보이는 까닭은 배치가 0.6/0.8 m 격자이기 때문이다. 값을 격자에 맞춘 것이 아니다(예: yellow-1 `[-0.195, -0.858]`, 정답 `[-0.2, -0.85]`).

**(2) `own_jobs[*].status`의 결정 — 누설(교사 조건 입력), 중간.**
- 문구 결정: `run_zone_dispatch.py:230-234`. `robot.outcome == 'placed_by_teacher'`이면 "issued sequence finished", 그 밖에는 "executor stopped before finishing"이다.
- outcome은 교사가 시뮬레이터 정답으로 정한다.
  - 들어 올린 뒤 실제 상자 높이가 0.045 m를 넘지 않으면 최대 2회 다시 잡고, 그래도 안 되면 `grasp_failed_by_teacher`(`zone_teacher.py:416-429`).
  - 운반 중 실제 높이가 0.03 m 아래면 `dropped_in_transit`(433-434).
  - 실제 자세로 경로를 계획해 막히면 `teacher_path_blocked`(376-387).
  - 동료 작업 상태를 보고 `box_taken_by_peer`(388-393).
  - `placed_by_teacher`는 이 검사를 모두 지난 뒤 물러나기가 끝나야 나온다(454-458).
- 주입 플래그가 하는 일은 집게를 연 채로 두는 것뿐이다(244-248). 영수증은 물리적으로 실패한 뒤 정답 높이 검사에서 나온다. 곧 접촉 판정이나 주입 플래그를 직접 옮긴 것은 아니다. 그러나 RGB에서 나오지 않은 성공·실패 통보다.
- 이 통보는 세 곳에 쓰인다. 자기 `own_jobs`, dynamic의 `team_board.stopped_reports`(동료에게도 전달), 깨우기다. dynamic은 누구든 작업이 끝나면 전원을 깨우고(227-228), independent는 자기 작업이 멈추면 10초 뒤 다시 묻는다(240-243).
- ZC2-s14-dynamic-graspfail 사례: r1의 세 번 들어올리기가 모두 `box_z_m 0.0159`로 실패해 31.6초에 끝났다. 31.8초 r1 요청의 `rgb_view`에는 green-1이 적재 구역 목록에 **없었다**(r1 몸체가 가렸거나 밀림). 이 시점의 실패 신호는 정답 기반 영수증뿐이었다. 48.3초에 r3가 green-1을 다시 맡았다. 이유는 r1의 메시지("switched to cyan-3")였고, 게시판에는 r1의 멈춤 보고도 있었다.
- 문서 `docs/zone_dispatch.md:29`는 이것을 교사 조건 입력으로 밝혀 두었다. 다만 ZC 기록의 "로봇은 멈춤 영수증만 받는다"는 이 영수증이 정답에서 나온다는 점을 말하지 않는다.

**(3) 프롬프트 발판의 비대칭 — 교란, 중간.**
- 세 조건은 같은 `_COMMON`(`zone_coordination.py:23-32`)을 쓴다. 조건 블록은 다르다.
  - dynamic `_CLAIM`(52-66): "goal minus zone_counts_seen minus active peer claims" 계산법, "avoid peers' current paths", 그리고 모두 양보하면 **"lowest robot_id keeps it"**라는 명시적 대칭 깨기 규칙.
  - independent `_SOLO`(`zone_solo.py:18-28`): 같은 계산법에서 동료 항만 뺐고, "images show where the peers are ... avoid doing the same job"만 있다. 무통신에서도 쓸 수 있는 관례(예: id 순서로 필요 단위를 나눠 맡기)는 주지 않았다. independent 규칙 응답(`zone_solo.py:96-112`)은 바로 그 관례를 쓴다.
  - plan_first `_PLAN`(34-50): 제안자 순환·수락 규칙, "Balance the work".
- 영향: ZC2에서 동률 규칙이 쓰일 수 있었던 충돌은 dynamic s13 graspfail의 1건뿐이다. 그래서 이 규칙이 6/6 대 1/6 차이를 설명하지는 못한다. 그러나 independent에 무통신 관례를 준 조건은 시험하지 않았다.

## 3. 발견 목록

| ID | 종류 | 심각도 | 내용 | 근거 | 상태 |
|---|---|---|---|---|---|
| L1 | 누설 | 높음 | independent에서 교사가 **동료의 의도만으로** 로봇을 멈춘다. 둘 다 상자로 가는 중이면 먼저 배정된 쪽이 상자를 갖는다. 멈춤은 배정한 그 틱에 나고 "stopped" 영수증과 10초 뒤 재질문으로 이어진다 | `zone_teacher.py:250-263, 388-393`; `run_zone_dispatch.py:230-243`. ZC2 `box_taken_by_peer` 23건 모두 배정 뒤 0.0초에 멈췄고, 그때 동료는 22건에서 아직 `to_box`(1건 `grasp`). s14 nominal에서 r2는 "I am right beside red-1"이라고 답했지만 60.8/71.8/82.8초에 연달아 멈췄다. r3는 43.3초에 배정받아 아직 이동 중이었다 | 미수정(SIM 확인 필요). strict xfail 테스트 |
| L2 | 누설 | 낮음 | 칸 id가 공용 풀에서 나와 `own_jobs`에 들어간다. `A3`이면 다른 A구역 작업이 둘 있다는 뜻이다 | `run_zone_dispatch.py:129-145, 203-211`. independent 자기 작업 69개 중 36개의 칸 id가 같은 구역에 진행 중인 동료 작업을 드러냈다. 답에서 칸을 언급한 경우는 0건이다 | **수정**: `zone_coordination.visible_own_jobs`(box, zone, issued_at_sim_s, status), 모든 조건 |
| L3 | 누설 | 낮음 | independent `request_id`에 전체 라운드 번호가 들어간다. 동료가 질문받을 때마다 커진다 | `run_zone_dispatch.py:286-289, 420-421`, `three_robot_runtime.py:141`. independent 6회에서 번호 건너뜀 87회. 옛 코드 반사실: r2가 네 번 더 질문받으면 r1 id가 `solo-2-0-2` → `solo-6-0-6`으로 바뀐다 | **수정**: 로봇별 `own_turns` |
| L4 | 누설(교사 조건) | 중간 | 영수증과 깨우기가 정답 검사로 정해진다 | 위 (2) | 미수정(R2의 RGB 완료 확인 범위) |
| L5 | 잠재 누설 | 낮음 | independent에서 칸 거절 영수증("zone has no free slot")이 동료의 구역 예약을 드러낸다. 구역당 3칸이라 과잉 배달도 최대 3개로 제한된다 | `run_zone_dispatch.py:193-202`, `zone_wide` 칸 3/3/3. ZC2 발생 0건 | 미수정 |
| C1 | 교란 | 높음(해석) | dynamic과 independent의 차이는 메시지만이 아니다. dynamic에는 호스트가 유지하는 **동료 선언 게시판**(의도), 모든 로봇의 영수증, **호스트 중재**(동료 선언을 뺀 필요량으로 거절), 충돌 통지, 사건 기반 깨우기가 함께 있다 | `run_zone_dispatch.py:188-190, 366-402`; `zone_coordination.py:159-227`; 메시지 전원 전달 `three_robot_runtime.py:164-172`. ZC2 dynamic은 충돌 1·무효 0이다. 답은 동료의 현재 작업을 근거로 드는데, 그 정보는 게시판과 메시지 양쪽에 있어 채널을 분리할 수 없다. s14 복구는 메시지를 인용했다 | 설계 사항. 제거 조건 제안 |
| C2 | 교란 | 중간 | 프롬프트 발판 비대칭 | 위 (3) | 미수정 |
| C3 | 교란 | 중간(시간·비용), 낮음(성공) | 깨우기 규칙이 다르다. independent는 null·무효·자기 멈춤 뒤 10초마다 다시 묻고, dynamic은 동료 작업이 끝나면 깨운다 | `run_zone_dispatch.py:34-37, 224-245, 285-329`. independent 답 142개 중 73개가 null 재질문, dynamic은 75개 중 34개 | 미수정 |
| C4 | 교란 | 낮음 | 파지 실패 주입 대상이 "전체에서 두 번째로 발행한 작업"이라 조건마다 로봇·상자가 다르다 | `run_zone_dispatch.py:204-206`. s14: dynamic r1 green-1, independent·plan_first r2 red-1 | 미수정 |
| C5 | 교란 | 낮음 | 재질문 예산이 다르다: independent 라운드당 2회, dynamic 3회, plan_first 협상 8라운드 | `run_zone_dispatch.py:249, 366-434` | 미수정 |
| K1 | 깨끗함 | — | RGB 라벨·시야는 영상에서만 나온다 | 위 (1) | 확인 |
| K2 | 깨끗함 | — | independent 요청에는 자기 채널만 있고, 답은 누구에게도 전달되지 않는다 | `zone_solo.py:33-73`, `run_zone_dispatch.py:420-421`, 저장 요청 142건 | 확인 |
| K3 | 깨끗함 | — | 저장한 것과 보낸 것이 같고, 모델·설정·이미지가 조건 간에 같다 | wire 274건 | 확인 |
| K4 | 깨끗함 | — | 심판(정답 자세)은 제어가 끝난 뒤 출력에만 쓰인다. 종료 조건과 `goal_met_rgb`는 RGB와 영수증만 쓴다 | `zone_coordination.py:234-249`, `run_zone_dispatch.py:276-347` | 코드 확인 |

교사의 이동·회피·비키기(`zone_teacher.py:464-522`)와 라벨→실제 상자 연결(`run_zone_dispatch.py:120-126`)도 정답을 읽는다. 둘은 교사 조건으로 허용되고 모든 조건에 같다. LLM 입력에는 영수증과 시각(L1, L4)을 통해서만 되돌아온다.

## 4. 이번 브랜치의 수정과 테스트

작고 분명한 L2·L3만 고쳤다. 구역 전용 파일이라 실행 번들 폐쇄(`harness/rgb_execution_bundle.py` `source_closure`)에 들어가지 않는다. `three_robot_runtime.py`는 번들 폐쇄에 들어 있으므로 건드리지 않았다.

- `harness/zone_coordination.py`: `OWN_JOB_KEYS`, `visible_own_jobs()`. 모델이 보는 자기 작업 기록에서 칸 id를 뺀다. 결과 파일 `result.json.jobs`에는 칸이 그대로 남는다.
- `harness/zone_solo.py`: 같은 함수를 쓴다.
- `scripts/run_zone_dispatch.py`: `independent_round`의 요청 id를 로봇별 질문 횟수로 만든다. 런타임은 만들어진 요청의 `request_id`로 저장·전송·검증하므로 런타임은 그대로 둔다. 전체 라운드 번호는 RGB 파일 이름과 rounds 기록에만 남는다.
- `tests/test_zone_comm_boundary.py`(CI 목록에 추가):
  - RGB 추정이 영상 이동을 따른다: TOP 영상을 10 px 옮기면 추정이 그 거리만큼 움직인다.
  - `static_map`에 상자 자세가 없다.
  - independent 반사실: 동료의 작업·영수증·메시지와 동료가 추가로 질문받은 횟수를 바꿔도 r1 요청이 바이트 단위로 같다. 같은 라운드에 누가 함께 질문받는지도 영향을 주지 않는다. 전달 대상은 항상 없다.
  - 칸 id를 바꿔도 모델 입력이 같다.
  - 조건별 요청 키와 이미지 집합.
  - L1은 `xfail(strict=True)`로 남겼다. 고치면 이 테스트가 실패해 표시를 지우게 된다.
- 수정 전 코드에서는 L2 테스트가 실패하고, L3는 id가 달라지는 것을 따로 확인했다.
- CI: `scripts/run_ci_tests.py` 2264 passed, 9 skipped, 1 xfailed.

**ZC1/ZC2는 이 수정 전에 실행됐고 다시 검증하지 않았다.** L2·L3 수정으로 independent·dynamic·plan_first 모델 입력이 바뀐다. 칸 id는 세 조건 모두에서 빠지고, request_id는 independent만 바뀐다. 따라서 이후 코호트를 ZC2와 섞지 않는다.

## 5. 제안 (구현하지 않음)

- **L1 최소 수정:** independent에서는 "둘 다 이동 중이면 먼저 배정된 쪽 우선"(`zone_teacher.py:260-262`)을 뺀다. 동료가 이미 상자를 잡는 중이거나(`TAKEN_PHASES`) 상자 앞에서 정렬 중일 때만 멈춘다. 이것은 로봇이 상자 앞에서 볼 수 있는 사건이다. 잡기 전 자리를 두고 다투는 교착은 기존 비키기 규칙과 120초 제한이 처리하는지 규칙 응답 시작 조건(ZC1/ZC2와 같은 fixture 6회)으로 먼저 확인해야 한다.
- **L4:** 교사 조건에서는 지금처럼 밝혀 두고, 결과 해석에 "정답 기반 실패 통보가 있을 때"라는 조건을 붙인다. RGB 스킬로 옮길 때는 TOP·자기 RGB로 완료를 확인하는 것으로 바꾼다(R2).
- **C1 분리 코호트:** `dynamic_board_only`(게시판·호스트 중재 O, 메시지 X)와 `dynamic_messages_only`(메시지 O, 게시판 X, 호스트는 independent처럼 자기 시야만 검사). 이 둘이 있어야 "명시적 LLM 메시지의 효과"를 따로 잴 수 있다.
- **C2:** independent에 무통신 관례(예: id 순서 분담)를 준 조건을 추가하거나, dynamic의 동률 규칙을 조건 차이로 명시해 결과와 함께 보고한다.
- **C3:** 비용·시간 비교에서는 null 재질문과 L1 멈춤 뒤 대기를 따로 센다.

## 6. ZC2 결론은 유지되는가

- **independent 과잉 배달(6회 중 5회):** 경향은 유지된다. L1–L3는 independent에 동료 정보를 **더** 준 쪽이다. 과잉 배달은 서로 다른 같은 색 상자를 고른 데서 나왔고, 이 누설들이 만든 것이 아니다. 다만 independent는 깨끗한 무통신 기준선이 아니다. LLM 호출 수와 SIM 시간에는 L1 멈춤과 C3 재질문이 섞여 있다.
- **dynamic 대 independent:** "dynamic 조정 묶음(동료 선언 게시판 + 호스트 검사 + 메시지 + 사건 깨우기) 대 없음"의 비교로는 유지된다. **명시적 LLM 메시지만의 효과로는 말할 수 없다**(C1).
- **dynamic graspfail 3/3:** 정답 기반 실패 통보(L4)가 있는 교사 조건의 결과로 유지된다. 복구는 s12·s13에서는 실패한 로봇 자신이, s14에서는 동료가 메시지를 근거로 했다.
- **plan_first graspfail 0/3:** 설계대로다(재계획 없음).

## 7. 검증하지 않은 것

- 수정 뒤 코드로 fixture·실LLM 실행은 하지 않았다. 수정이 결과를 바꾸는지는 모른다.
- L1 수정안의 교착 여부(SIM 필요).
- C1 분리 코호트의 효과 크기.
- 모델이 L2·L3 채널을 실제로 썼다는 증거는 없다(답에 칸 언급 0건). 쓰지 않았다는 증명도 아니다.
- ZC1 게이트 실행과 Z1–Z3, ZW 코호트의 요청은 표본으로 보지 않았다. 구역 코드가 같은 경로라서 L1(independent 전용)을 뺀 나머지 판정이 그대로 적용될 것으로 보지만 확인하지 않았다.
