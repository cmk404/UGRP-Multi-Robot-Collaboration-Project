# 2026-09-25 구역 팀 작업 A2: protocol v2 연결과 교사 실현 가능성 스모크

A1 부품을 구역 실행기에 연결해, 섞인 화물 목표(색 상자 + can·tile·long_beam·heavy_crate·tri_frame)를 벽·문 지도에서 끝까지 돌린다. 모든 이동은 정답 교사가 한다. **이 기록의 결과는 교사 실현 가능성(시연을 만들 수 있는가)이며 로봇·학생 성공이 아니다.** 사용자 결정(2026-09-25)에 따라 최종 연구 실행기는 자기 카메라만 쓴다. 연구 조건은 independent / dynamic(분산 한국어 대화) / leader(한 AI가 모든 로봇에 명령)다.

- 브랜치 `claude/zone-team-a2`(draft PR #169). 스택: hard-routes(#173), noslip(#167), perception(#168), perception v2(#174)는 모두 main에 병합됐고, 이 브랜치는 main을 병합했다.
- 스모크 소스: cohort 1 `143360d`(`top_cargo_v1`), cohort 2 `1f25d15`(`top_cargo_v2`). 기록 커밋은 이 README를 담은 커밋이다.
- 실행 흐름: `zone-dispatch` 1.1.0 → 1.2.0. 실행 번들 ID는 사용하지 않았다(`harness/rgb_execution_bundle.py` 변경 없음).
- 결과·해시: [results.json](results.json). 분석: `analyze.py`, `build_results.py`. 실행: `run_smokes.py`.

## 0. 요약

- 섞인 목표(물건 6개, 팀 물건 2개)를 `zone_wide_two_doors`에서 10번 완주했다. cohort 1: dynamic s11, independent s11/s12, plan_first s11/s12. cohort 2: dynamic s11, independent s11/s12, plan_first s11, 파지 실패 주입. referee_v2 목표를 달성했고, 제어 종료는 421–535 SIM초(한도 1800)다. 모든 실행에서 `eq_active` 0, 떨어뜨림 0, 미끄러짐 ≤0.7 mm, 이중 소속 0, 물건당 차감 1회다.
- 막힘·실패 3건: C1 dynamic s12(B3→B4→B5 연쇄 교착), tri_frame × 0.5 m 문(B2), tri_frame × 두 문 지도(B8, 색 상자가 유일한 경로를 막음). tri_frame은 두 지도 모두 교사로 운반하지 못했다. 막힘 목록은 §7에 있다.
- 기록 전 디버그에서 찾은 0.5 m 문 끼임(B1)은 v2 진입 대기로 고쳤다.

## 1. 무엇을 연결했나

A1 부품(`harness/zone_goal_v2.py`, `zone_team_jobs.py`, `zone_team_formation.py`, `zone_team_footprint.py`)을 구역 실행기에 연결했다. 목표에 화물 목록 종류가 있으면 `scripts/run_zone_dispatch.py`가 `scripts/zone_dispatch_v2.py`(protocol v2)로 넘긴다. 색만 있는 목표는 기존 v1 경로 그대로다.

| 부분 | 파일 | 내용 |
|---|---|---|
| 장면 | `harness/zone_mixed_episode.py` | 색 상자는 기존 `episode()`로 놓는다. 화물은 seed마다 정해진 방식으로 놓는다. 팀 발자국·접근 자리가 상자·출발점·벽·통로 앞 차선(길이 1.2 m, 옆 0.2 m)과 겹치지 않아야 한다(설정 전용). `CargoZoneScene.from_cargo_config`와 명시한 접촉 프로필(스모크 `cargo_noslip_v1`)을 쓰고, 벽은 hard-routes 지도에서 온다 |
| 인식 | `harness/zone_perception_v2.py` | TOP RGB만 쓴다. 기본값은 `top_cargo_v2`(PR #174)이고, `--perception-profile top_cargo_v1`로 v1을 고를 수 있다. 여기에 상자 경로(`top_zone_v2`)를 더한다. v2에서 신뢰도 0.5 미만 빔은 다른 TOP 시점이나 1초 뒤 두 번째 촬영으로 확인되어야 물건이 된다. 종류별로 서→동 라벨(`long_beam-1` 등)을 붙이고, RGB 손잡이 위치와 접근 자세를 준다. 모델에는 종류·RGB 위치·방향·손잡이 위치만 보인다(`public_labels`) |
| 과제 글 | `harness/zone_protocol_v2.py` | `task_static_text`: 종류별 필요 인원·역할, 착지 영역(id 없음), 팀 형성 규칙, hard-routes 벽·문 지도. 세 조건에서 바이트가 같다(테스트) |
| 선언 | 같은 파일 | 세 조건 모두 `{item, zone, role}`이다. independent는 `check_independent_claims`, dynamic은 `check_dynamic_claims`, plan_first는 `validate_team_plan`으로 검사한다. 호스트는 팀원을 고르지 않는다 |
| 조건 이음매 | `scripts/zone_dispatch_v2.py` `ModeLoop`·`COORDINATIONS` | 조건마다 `prepare`, `on_claim_end`, `step`을 가진 하위 클래스다(independent, dynamic, plan_first). plan_first는 ZC1/ZC2 재현용으로만 남긴다. **leader(한 모델이 모든 로봇에 명령)는 새 `ModeLoop` 하위 클래스를 `COORDINATIONS`에 등록하고, `harness/zone_protocol_v2.CONDITIONS`에 기본 스위치를, 같은 파일에 메시지 틀을 더해 넣는다.** 실행기·로봇용 결과 장부·심판은 공통이다. 테스트로 표와 스위치 집합이 같은지 확인한다 |
| 조건 스위치 | `CONDITIONS`, `condition_switches` | `peer_board`, `host_arbitration`, `conflict_notices`, `wake_on_peer_job_end`, `peer_messages`. 기제마다 한 곳에서만 읽는다. `--condition-switches`로 하나씩 바꿀 수 있고, 조건의 반복문이 읽지 않는 스위치를 바꾸려 하면 거부한다 |
| 교사 | `scripts/zone_team_teacher.py` | 새 팀 실행기다. 모든 작업이 TeamJob이다(단독 = 1인 팀). 역할 정거장까지 이동 → `RendezvousRule`(모든 정거장이 몸으로 채워지고 spec이 같을 때만 commit, 자기 도착 뒤 최대 60 SIM초) → PREGRASP…RETREAT를 `PhaseBarrier`로 묶는다. 팀 작업에는 12초 강제 파지와 막힘 시 집게 열기를 쓰지 않는다. 다른 팀은 발자국 전체를 장애물로 본다 |
| 팀 경로 | `harness/zone_team_route.py` | 물건 자세 (x, y, yaw) 공간에서 A*를 쓴다(0.05 m, 4방향). 직선 구간은 시작·끝 배치의 볼록 껍질로 정확히 sweep한다. 회전은 발자국 전체 원판이 들어갈 때만 한다. 구간마다 `static_keepouts` swept 검사를 다시 한다. 경로가 없으면 접촉 전에 `no_team_route`(cancel_retreat)로 끝낸다 |
| 착지·심판 | `landing_layout`, `referee_v2` | 슬롯 대신 착지 영역을 쓰고, id는 모델에 보이지 않는다. 심판은 평가 전용 전체 발자국 판정이다. 목표 차감은 `TeamJobLedger`에서 물건당 한 번 한다 |
| 로봇용 결과 | `harness/zone_outcomes_v2.py` | 자기 작업 상태, 게시판 보고, 재질문 시점, 호스트·fixture 배달 목록은 `RobotResults`를 거친다. 교사 운동 장부와 분리되어 있다. 결과 출처는 `TeacherReceiptSource`(`teacher_receipt_L4`) 하나뿐이다 |
| 단일 통로 진입 | `TeamRobot.passage_gate` | v2 로봇만 적용한다. 통로 구역 안에서 자기 경로가 문을 지날 때, 다른 로봇이 문 안에 있거나 문에 더 가까이 움직이는 로봇이 있으면 최대 60 SIM초 기다린다. 몸의 위치·정지 여부·정적 통로·자기 경로만 읽는다 |

실행 흐름(명령): `configs/simulation_workflows.json`의 `zone-dispatch`를 1.1.0 → 1.2.0으로 올렸다. 실행 번들 레지스트리(`harness/rgb_execution_bundle.py`)는 건드리지 않았다.

## 2. L1 수정(의도한 변경)

감사 R1 L1: v1 교사는 동료가 같은 상자로 `to_box`/`align_box` 중이라는 **의도**만으로 로봇을 멈췄다(`_taken_by_peer`의 먼저 배정된 쪽 우선 규칙). `e096324`에서 이 규칙을 지웠다. 이제 멈춤은 몸으로 보이는 두 경우뿐이다.

- 상자가 이미 잡히는 중이거나 배달됐다(`TAKEN_PHASES`, `placed_by_teacher`).
- 정거장에 더 가까운 몸이 서 있다(`RendezvousRule.occupancy` = `station_blocked`, id·동료 선언을 읽지 않음).

`tests/test_zone_comm_boundary.py`의 L1 strict xfail을 지웠고, 이제 통과한다. 새로 추가한 `test_only_a_nearer_body_at_the_station_stops_a_robot`은 가짜 world에서 이 규칙을 확인한다. v1 영수증 목록에는 `station_blocked`가 추가됐다(되돌려 주는 작업).

## 3. 바이트 동일과 의도한 차이(ZC2 재현)

- ZC2 재현 경로는 그대로 남아 있다: `--variant zone_open --allow-retired-variant`, v1 기본값(900 SIM초, `local_contact_fine`), LLM 또는 fixture. v2 옵션(`--extra-cargo`, `--inject-team-grasp-failure`, `--condition-switches`)을 v1에 주면 거부한다.
- 바이트 동일(테스트 `test_v1_maps_plans_and_prompts_are_byte_identical_to_main`, main `3cbf4e4` 기준 golden): `zone_open`·`zone_wide` 지도 파일 해시, 플래너 216건 digest(`plan_path`·`retreat_point`), v1 요청(과제·claim·plan·solo) 바이트. 이 브랜치에서 통과한다.
- 의도한 차이: (1) L1 수정(위). v1 교사의 멈춤 판정이 바뀌었으므로 ZC1/ZC2와 같은 교사 사건열은 보장하지 않는다. (2) hard-routes의 v1 변경(벽이 있는 지도에서만 적용되는 keep-out·통로 규칙, 240초 이동 한도)은 벽이 없는 `zone_open`에서는 동작하지 않는다(위 digest가 확인한다). (3) 단일 통로 진입 대기는 v2 `TeamRobot`에만 있다.

## 4. 리뷰 반영(Codex 검토 #169, 2026-09-25)

- `top_cargo_v1` 틀 안쪽 억제 수정을 되돌렸다(`6f88d7d`). v1은 #168에서 보고된 그대로 둔다. 수정은 인식 에이전트의 `top_cargo_v2`에서 한다. 이 브랜치에서 다시 채점한 결과(test 분할: box_green 병합 271 → 272, 놓침 4 → 3, 로봇 위 바닥 인식 24/25 → 25/25, 나머지 변화 없음)는 **채택하지 않은 진단**으로만 남긴다. 실행마다 `perception_profile`을 기록한다. cohort 1은 `top_cargo_v1`, cohort 2는 PR #174의 `top_cargo_v2`(병합 뒤 기본값)다. 두 프로필 모두 시작 라벨이 설정과 같았다.
- can의 서쪽 치우침(2–2.6 cm)은 기록만 하고 고치지 않았다(#168 기록과 같음).
- 로봇용 결과 장부를 교사 운동 장부와 분리했다(`f881daa`). 교사 영수증은 여전히 로봇에게 가지만(L4 미해결), 모든 결과의 `robot_facing_outcome_source`에 `teacher_receipt_L4`로 표시된다. #170의 RGB 결과는 `outcome()`을 구현한 출처로 끼운다. `confirmed=False`인 결과는 배달 목록에 들어가지 않는다.
- tile 역할 계약: `formations`, `claim_roles`, `fills_formation`을 추가했다. east 선언을 받는다. 역할은 RGB 접근 자세로 묶는다. east tile은 착지 정거장이 같도록 180° 돌려 내려놓는다. 한 tile의 두 쪽을 동시에 선언하면 `formation_conflict`, 두 쪽에 서도 팀이 되지 않는다. **스모크의 fixture는 기본 역할(west)만 쓰므로 east 경로는 단위 테스트로만 확인했다.**
- v2는 `--contact-profile`을 명시해야 한다.
- 조건 스위치를 기제별로 나눴다(C1/C3 교란 분리 준비).

## 5. 대화 재설계를 위한 hook 위치(구현 안 함)

- (a) 메시지 경로: 모든 v2 로봇 글(system·context·템플릿·fixture)은 `harness/zone_protocol_v2.py` 한 곳에 있다. 결과 문구는 `harness/zone_outcomes_v2`의 출처 `status`다. 수신자는 `team.ask(..., recipients=...)`로 이미 조건마다 명시한다(`peer_messages` 꺼짐 = `[]`). 나중에 `recipients` 목록을 선언별로 주면 된다.
- (b) 움직이는 중 질문: `ZoneTeamExecutor.robot_event(rid, kind, now, **detail)`가 그 자리다. 지금은 기록만 한다(`robot_event_hook`는 None). 로봇 자기 관측으로 생기는 사건만 부른다: `station_blocked`, `rendezvous_timeout`, `team_failure`(파지 결과가 예상과 다름). 긴 문 대기(`passage_gate` 사건)와 운반 멈춤도 같은 hook에 연결할 후보다. 호스트 플래그로는 부르지 않는다.
- (c) 게시판·중재·깨우기: `CONDITIONS`의 기제별 스위치와 `--condition-switches`로 조건마다 켜고 끈다.
## 6. 스모크 E2E: 교사 실현 가능성(teacher feasibility)

**판정 범위:** 모든 이동은 정답 교사(주행·IK·실제 집게, weld OFF)가 했고, 선언은 fixture(LLM 0회)다. 이 결과는 "교사가 이 지도·목표에서 시연을 만들 수 있는가"만 판단한다. 로봇·RGB 스킬·학생 성공이 아니다. 사용자 결정(2026-09-25)에 따라 최종 연구 실행기는 자기 카메라만 쓰고, 교사는 시연용이다. 로봇에게 가는 결과는 여전히 교사 영수증이다(`teacher_receipt_L4`).

조건: `zone_wide_two_doors`(b는 `zone_wide_door` 포함), 동기 SIM, `cargo_noslip_v1`, `--record-replay`, 동시 실행 최대 2개. cohort 2는 스레드 1개 환경(`OMP/OPENBLAS/VECLIB/MKL_NUM_THREADS=1`)으로 돌렸다. 부하 평균은 실행마다 기록했다(공용 호스트 부하 4–56). SIM 결과는 부하와 무관하고 wall 시간은 참고용이다.

- 섞인 목표 `MIXED`: A long_beam 1 / B heavy_crate 1 + red 1 / C can 1 + green 1 + tile 1(물건 6개, 팀 물건 2개)
- tri 목표 `TRI`: A tri_frame 1 + red 1 / C green 1
- cohort 1: 소스 `143360d`, `top_cargo_v1`(당시 기본), 출력 `outputs/zone-team-a2-20260925/`(A2 worktree)
- cohort 2: 소스 `1f25d15`, `top_cargo_v2`(PR #174, 약한 빔 확인 규칙), 출력 `outputs/zone-team-a2-v2-20260925/`(dev worktree)

두 cohort의 소스 차이: 인식 프로필(v2 기본값·약한 빔 확인·시작 촬영 1회 추가), 기제별 스위치(동작은 같음), 조건 분배 표(동작 변경 의도 없음), 실행기 스레드 환경. 교사·경로·규칙 코드는 같다. 시작 촬영이 1 SIM초 늘어서 cohort 2의 시각은 대체로 1초 늦다.

| cohort | 실행 | 결말 | referee_v2 목표 | 제어 종료 SIM초 | 팀 형성 대기 최대(s) | 문 대기 / 진입 대기(s) | 떨어뜨림 / 미끄러짐 최대(mm) | eq_active | 선언 결과 | 부하(1분) 시작→끝 | wall(s) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| C1 v1 | a-two-dynamic-s11 | FINISHED | Y | 421.0 | 25.9 | 0 / 0 | 0 / 0.7 | 0 | {'placed_by_teacher': 8, 'rendezvous_timeout': 2} | 8.1→11.06 | 989.9 |
| C1 v1 | a-two-dynamic-s12 | SIM_BUDGET | N | 1800.0 | 1.0 | 0 / 0 | 0 / 0.7 | 0 | {'team_aborted': 2, 'rendezvous_timeout': 26, 'path_blocked': 6, 'placed_by_teacher': 4, 'live': 2} | 40.92→4.8 | 22636.8 |
| C1 v1 | a-two-independent-s11 | FINISHED | Y | 535.0 | 26.4 | 0 / 3.4 | 0 / 0.7 | 0 | {'placed_by_teacher': 8, 'rendezvous_timeout': 1} | 11.06→23.36 | 1319.1 |
| C1 v1 | a-two-independent-s12 | FINISHED | Y | 491.5 | 6.1 | 0 / 6.8 | 0 / 0.7 | 0 | {'placed_by_teacher': 8, 'rendezvous_timeout': 1} | 23.36→7.98 | 1005.1 |
| C1 v1 | a-two-plan_first-s11 | FINISHED | Y | 490.5 | 26.4 | 0 / 3.4 | 0 / 0.7 | 0 | {'placed_by_teacher': 8} | 8.1→56.39 | 1106.9 |
| C1 v1 | a-two-plan_first-s12 | FINISHED | Y | 478.0 | 8.4 | 0 / 0.1 | 0 / 0.6 | 0 | {'placed_by_teacher': 8} | 56.39→40.92 | 1159.3 |
| C1 v1 | b-door-tri-dynamic-s11 | SIM_BUDGET | N | 1800.0 | 12.7 | 0 / 0 | 0 / 0.0 | 0 | {'team_aborted': 993, 'live': 3} | 7.98→19.39 | 6067.6 |
| C1 v1 | b-two-tri-dynamic-s11 | 실행 안 함/중단 | – | – | – | – | – | – | superseded by cohort 2 | – | – |
| C1 v1 | c-two-graspfail-dynamic-s11 | 실행 안 함/중단 | – | – | – | – | – | – | superseded by cohort 2 | – | – |
| C2 v2 | a-two-dynamic-s11 | FINISHED | Y | 422.0 | 26.1 | 0 / 0 | 0 / 0.7 | 0 | {'placed_by_teacher': 8, 'rendezvous_timeout': 2} | 28.35→31.9 | 1145.6 |
| C2 v2 | a-two-dynamic-s12 | 실행 안 함/중단 | – | – | – | – | – | – | stopped at SIM ~731 s by operator (coordinator wrap-up 2026-09-26): deterministic replay (+1 s) of the cohort-1 failure trajectory; partial outputs kept, no res | – | – |
| C2 v2 | a-two-independent-s11 | FINISHED | Y | 525.0 | 26.4 | 0 / 0 | 0 / 0.7 | 0 | {'placed_by_teacher': 8, 'rendezvous_timeout': 1} | 31.9→8.31 | 1122.5 |
| C2 v2 | a-two-independent-s12 | FINISHED | Y | 492.5 | 5.1 | 0 / 9.1 | 0 / 0.6 | 0 | {'placed_by_teacher': 8, 'rendezvous_timeout': 1} | 6.46→7.0 | 781.1 |
| C2 v2 | a-two-plan_first-s11 | FINISHED | Y | 483.5 | 26.0 | 4.2 / 0 | 0 / 0.7 | 0 | {'placed_by_teacher': 8} | 8.31→4.4 | 565.0 |
| C2 v2 | b-door-tri-dynamic-s11 | 실행 안 함/중단 | – | – | – | – | – | – | geometric no_team_route already shown in cohort 1; perception-independent | – | – |
| C2 v2 | b-two-tri-dynamic-s11 | 실행 안 함/중단 | – | – | – | – | – | – | stopped at SIM ~256 s (last claim round) (coordinator wrap-up 2026-09-26) in a claim/abort loop (>=39 claim rounds); cause reproduced offline with the same plan | – | – |
| C2 v2 | c-two-graspfail-dynamic-s11 | FINISHED | Y | 493.5 | 19.5 | 0.9 / 0 | 0 / 0.6 | 0 | {'team_aborted': 2, 'placed_by_teacher': 8, 'rendezvous_timeout': 3} | 4.4→9.68 | 840.5 |


- `eq_active`는 모든 실행의 모든 physics step에서 0이다(`eq_active_max` = 0, weld 없음).
- 이중 소속 0, 물건당 차감 1회(`decrement_once_per_item`), 과잉 배달 0은 모든 실행에서 성립한다.
- 시작 라벨의 종류별 개수는 모든 실행에서 설정과 같다(v1, v2 모두). 끝 view의 확인되지 않은 빔은 0이다. 저장된 start/final TOP 17세트를 오프라인으로 v1·v2 모두 다시 검출한 결과도 같았다.
- 1800 SIM초 한도: 성공한 실행의 제어 종료는 421–535 SIM초로 한도의 30% 이하다. 실패한 실행은 막힘이 풀리지 않아 1800초를 다 썼다. 섞인 목표의 기본 한도 1800초는 성공 경로에 충분하고, 막힘을 끊는 장치는 아니다.

## 7. 막힘·실패 목록(주 산출물)

원인은 `teacher-events.json`, `result.json`, TOP 영상으로 확인했다(관찰 전용). "수정"은 이 작업에서 고친 것, "미수정"은 기록만 한 것이다.

| # | 상태 | 어디서 | 무엇이 막혔나 | 원인(근거) |
|---|---|---|---|---|
| B1 | **수정**(`2e7da48`) | 기록 전 디버그 s12 plan_first(`zone_wide_two_doors`) | 빔 운반을 마친 r1·r2가 함께 heavy_crate로 출발해 0.5 m 좁은 문 안에 나란히 끼었다. 둘 다 240초마다 `path_blocked`를 되풀이했고, 671초에 목표 미달로 끝났다 | 대치 규칙(hard-routes)이 두 로봇 모두에게서 비킬 자리를 찾지 못했다(`passage_yield_end: no_spot` 4634회). 수정: v2 로봇의 단일 통로 진입 대기. 몸 위치·정지·정적 통로·자기 경로만 읽고 최대 60초다. 같은 설정의 재실행은 478초에 성공했다. 기록 cohort의 진입 대기는 0.1–9.1초였고 재발하지 않았다 |
| B2 | 미수정 | C1 `b-door-tri-dynamic-s11`(`zone_wide_door`) | tri_frame이 0.5 m 문을 지날 수 없다. 접촉 전에 `no_team_route`(cancel_retreat)로 332번 거부됐고, 1800 SIM초를 다 썼다(wall 6068초). red·green 상자도 옮기지 못했다 | 발자국 폭 ≥0.889 m라 기하학적으로 불가능하다(설계대로 접촉 전 거부). 막힘의 원인은 로봇에게 가는 결과가 일반 "stopped"뿐이고, fixture가 팀 물건을 먼저 고르기 때문이다. 같은 물건을 바로 다시 선언해 단독 물건이 굶었다. 필요: 되풀이되는 불가능 알림 또는 대화 경로, 선언 선택 규칙 |
| B3 | 미수정 | C1 `a-two-dynamic-s12` | 빔 팀 운반이 r3 때문에 3번(41.9/58.8/49.0초) 멈췄다. 382초에 `carry_blocked`로 함께 내렸다(hold_lower) | r3가 heavy_crate west 정거장(빔 팀 경로 위)을 선언하고, 빔 운반 중인 짝을 기다렸다. 60초가 지나면 20초 비키고, dynamic 조건이 바로 다시 물어 같은 정거장을 선언해 되돌아왔다. 같은 seed의 independent는 다시 묻기 전 10초 지연이 있어 한 번(42.7초) 멈춘 뒤 회복했다. 정거장 대기 자리가 다른 팀 경로와 겹치는지를 선언 단계나 대기 자리에서 고려하지 않는다 |
| B4 | 미수정 | C1 `a-two-dynamic-s12` | 중단된 빔이 heavy_crate의 west 정거장 옆에 내려졌다. 그 뒤 r1은 east 정거장에서 rendezvous timeout을 23번 겪었고, west로 가는 로봇은 `path_blocked`(240초 한도)였다. 영구 교착이며 1800초에 끝났다 | hold_lower는 그 자리에서 내린다. 다른 물건의 정거장·통로를 피해 내릴 곳을 고르지 않는다(끝 TOP 영상으로 확인) |
| B5 | 미수정 | C1 `a-two-dynamic-s12` | 내려진 빔을 아무도 다시 선언할 수 없었다(끝 view의 `pickup_items_still_visible`에 `long_beam-1`이 없음) | 라벨은 시작 위치 근처(DEDUPE 반경)에서만 다시 찾는다. 픽업 자리를 벗어난 물건은 로봇이 선언할 수 있는 목록에서 사라진다 |
| B6 | 예상 비용 | C1/C2 dynamic s11(2회), independent s11/s12(1회), c(3회) | 단독 로봇이 짝보다 먼저 와서 60초 기다린 뒤 멈춘다 | 무통신 팀 형성 규칙의 대기 한도다. 실패가 아니라 시간 비용이고, 모두 목표를 달성했다 |
| B7 | 회복됨 | C2 `c-two-graspfail-dynamic-s11` | 주입된 파지 실패(r1 집게 열림): CLOSE에서 `grasp_contact_missing` → `hold_lower` → ABORTED | 설계대로다. 두 번째 팀 작업 `long_beam_0@g2`가 배달했고, 목표를 493.5초에 달성했다 |
| B8 | 미수정 | C2 `b-two-tri-dynamic-s11`(`zone_wide_two_doors`) | tri_frame 팀이 선언·중단을 되풀이했다(256 SIM초까지 선언 39회). 운영 중단 | 같은 계획기·시작 자세로 오프라인에서 재현했다. 적재 쪽 넓은 문 앞(1.6, −2.45)의 red 상자가 tri_frame의 유일한 경로를 막는다(그 상자를 빼면 경로 있음, 적재→동쪽 가능, 문 너머→구역 A 가능). 세 로봇이 모두 팀 물건에 묶여 상자를 먼저 옮길 로봇이 없다. 배치(색 상자는 통로 앞 차선 제외 규칙 밖)와 순서 문제다 |
| B9 | 참고 | C1 cohort 전체 | wall 시간 565–22637초 | 공용 호스트 부하(1분 평균 최대 56)와 막힌 로봇의 반복 재계획 때문이다. SIM 결과는 부하와 무관하다 |

- hard-routes에서 알려진 "복도 출구 밖 서로 밀기"는 A2 스모크에 복도 지도가 없어 확인하지 않았다. 문 지도의 팀 운반에서는 `passage_standoff`가 한 번도 나오지 않았다.
- 모든 막힘에서 `eq_active` 0, 떨어뜨림 0이다. 미끄러짐은 최대 0.7 mm다(`cargo_noslip_v1`).


## 8. 영상

영상은 `media/`에 있다(관찰 전용 replay 렌더, 제목 줄에 SIM 시각·배속·로봇 단계 표시).

- `media/a-c2-two-mixed-dynamic-s11.mp4` — a: 섞인 목표 dynamic s11(cohort 2) — 성공, 422 SIM초, ×4 (sha256 `12d428542895…`, 0.87 MB)
- `media/b-c1-door-tri-dynamic-s11.mp4` — b: tri_frame × 0.5 m 문(cohort 1) — no_team_route 반복(B2), 1800 SIM초, ×24 (sha256 `0bdac927b29d…`, 0.80 MB)
- `media/blocker-c1-two-mixed-dynamic-s12.mp4` — 막힘 B3–B5: dynamic s12(cohort 1) — carry_blocked 뒤 교착, 1800 SIM초, ×16 (sha256 `0772d54137a5…`, 0.88 MB)
- `media/c-c2-two-graspfail-dynamic-s11.mp4` — c: 빔 팀 파지 실패 주입(cohort 2) — hold_lower 뒤 재시도 성공, ×4 (sha256 `c284ac76dbc5…`, 1.01 MB)
- `b-two-tri-dynamic-s11`(B8)은 중단해서 replay가 없다. 원인은 오프라인 경로 재현으로 확인했다.


## 9. 검증 범위와 확인하지 않은 것

- 확인한 것:
  - `tests/test_zone_team_a2.py` 등 zone 테스트 215개가 main 병합(`bc7b279`) 뒤 통과했다.
  - `scripts/run_ci_tests.py`: 2487 통과, 9 건너뜀, 종료 코드 0. 실행기가 공용 잠금을 잡았다가 풀었다
  - 위 표의 실행: referee_v2, 물건별 결과, 장부 검사, 라벨 대 설정 비교, 부하 평균
  - v1·v2 인식의 오프라인 재검출(저장된 TOP 17세트)
  - B8의 오프라인 경로 재현
- 확인하지 않은 것:
  - 실LLM(`llm` 모드) 실행
  - tile east 역할의 SIM 운반(fixture는 west만 씀, 단위 테스트만)
  - `zone_wide_corridor`의 팀 운반
  - tri_frame 성공 사례(두 시도 모두 막힘)
  - seed 11–12 밖의 배치
  - cohort 2 dynamic s12(1초 늦춘 같은 궤적이라 731초에 중단)
  - 자기 카메라 전용 실행기. 최종 연구 조건이며, 이 기록은 교사 시연 가능성만 다룬다
- 이 결과는 교사 실현 가능성이다. 로봇 성공, RGB 스킬 성공, 학생 성공으로 보고하지 않는다. 로봇에게 가는 결과는 교사 영수증이다(`teacher_receipt_L4`, L4 미해결).
- 원본은 로컬에만 있다(원격 백업 아님): cohort 1은 `/Users/changmin/projects/ugrp-worktrees/zone-team-a2/outputs/zone-team-a2-20260925/`, cohort 2는 `/Users/changmin/projects/ugrp-worktrees/zone-team-a2-dev/outputs/zone-team-a2-v2-20260925/`. 기록되지 않은 디버그 실행(`dbg-dyn`, `dbg-plan`, `dbg-plan2`, `dbg-dump`)은 scratchpad에만 있고 결과에 넣지 않았다.

