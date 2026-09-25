# 벽·문 경로 지도와 교사 통로 규칙 (2026-09-25, zone hard routes)

**범위:** 최종 연구용 환경 구축 단계의 기록이다. 통제 비교가 아니다. 아래 수치는 모두 교사 실행기(정답 좌표 주행·IK, 실제 집게, weld OFF)와 규칙 fixture 응답(LLM 0회)의 결과다. **LLM 협업의 근거가 아니다.** 지도가 교사 조건에서 실행 가능한지와 병목에서 무엇이 일어나는지를 확인한 것이다.

## 요청과 결정

- 요청(2026-09-25): 협업이 필요한 경로를 만든다. 비켜야 하는 병목, 우회로, 그리고 나중에 한 로봇이 발견해 알려야 하는 지도 밖 막힘을 넣는다. 교사 계획기가 벽(회전 가능한 직사각형)과 운반 여유를 알고, 좁은 문에서 마주친 두 로봇이 비켜서 풀어야 한다.
- 추가 요청(같은 날, 사용자 "옛날의 작은 맵은 없애주라"): `zone_open`을 은퇴시켰다. 새 실행의 기본은 `zone_wide`이고, `zone_open`은 `--allow-retired-variant`로 Z1–Z3를 재현할 때만 실행된다. 지도 파일과 배치 코드는 바이트 그대로 남겼다(커밋 `8935ed9`).
- 조정 지시: 통로 규칙은 동료의 작업·의도·선언을 읽지 않는다. 현재 물리 상태와 정적 지도만 쓰고, 세 협업 방식에서 똑같이 동작해야 한다(감사 PR #161 참고). `_taken_by_peer`의 의도 기반 중재(L1)는 후속 작업에서 고치므로 여기서는 손대지 않았다.

## 무엇을 바꿨나

| 커밋 | 내용 |
|---|---|
| `8935ed9` | `zone_open` 은퇴. 기본 `zone_wide`, `--allow-retired-variant`, 장면 목록에서 제외, `DEFAULT_VIEWS`는 기본 지도 기준 |
| `c6cd334` | `harness/static_keepouts.py`, 교사 A*의 벽 직사각형 금지 영역, 운반 여유 훅, 한 차선 통로 대치 규칙, 새 지도 3개, 프롬프트의 정적 지도 문단, workflow `zone-dispatch` 1.1.0 |
| `dc27695` | fixture 행렬 실행기와 관찰 전용 분석·렌더 스크립트(이 폴더) |
| `3183650` | 벽 지도에서만 이동 단계 한도 240초(v1은 120초 그대로). 분석에 재생 기반 대기 시간 추가 |

실행 번들(`harness/rgb_execution_bundle.py`의 source closure 173개 파일)에는 바꾼 파일이 하나도 없다. 그래서 예약한 `rgb-standard-dispatch-v64`는 쓰지 않았다. workflow 등록부에서는 `zone-dispatch`를 1.0.0에서 1.1.0으로 올렸다(기본 지도와 선택지 변경).

## v1 동일성

- 계획기: `zone_open`·`zone_wide`에서 무작위 72개 계획 질의(경로, 운반 경로, 후퇴점)의 digest가 origin/main `3cbf4e4`와 같다(`ee5961d9…`, 테스트).
- 지도 파일 해시, 세 요청 형식(선언·계획·무통신)의 프롬프트 해시가 main과 같다(테스트). 병합 후에는 `test_zone_cargo`의 장면 XML 해시 고정도 통과한다.
- SIM: `zone_wide` G8 dynamic seed 12 fixture를 main(`3cbf4e4`, 별도 worktree)과 이 브랜치(`c6cd334`)에서 돌렸다. `teacher-events.json`, `scene.xml`, `box-labels.json`, 재생 `states.npz`가 바이트 단위로 같다. teacher-events 해시 `ae804c28…`는 ZC2 gate2-dynamic-nominal 기록과도 같다.

## 지도

모두 `zone_wide`(6.45 × 4.6 m, TOP 네 대, 적재 격자, 출발 자리, 구역 A/B/C)를 그대로 두고 안쪽 벽만 더했다. 벽 두께는 0.05 m, 높이는 0.10 m이고 바깥 벽과 같은 색이다. 각 지도는 v1이다.

| 변형 | 벽 | 통로 폭 | 그림 |
|---|---|---|---|
| `zone_wide_door` | x = 2.20 남북 벽 | `door_1` 0.50 m, 중심 (2.20, 0.05), 한 대씩 | `map-zone_wide_door.png`, `render-zone_wide_door.png` |
| `zone_wide_two_doors` | 같은 벽 | `door_narrow` 0.50 m, (2.20, 0.05), 한 대씩 + `door_wide` 1.00 m, y −3.125…−2.125, 두 대 | `map-zone_wide_two_doors.png`, `render-…` |
| `zone_wide_corridor` | 벽이 y 0.90에서 꺾여 x 3.90까지 이어진 복도 벽 | `corridor_1` 0.50 m × 1.725 m(x 2.20…3.925), 한 대씩 + `bay_1` 0.55 × 0.50 m(x 2.825…3.375, y 0.375…0.875) | `map-zone_wide_corridor.png`, `render-…` |

- 문 폭 0.50 m를 고른 이유: 상자를 든 로봇이 도는 데 필요한 지름(앞으로 0.195 m, 약 0.39 m)에 0.11 m를 더했다. 교사는 상자를 들면 반지름 0.21 m로 계획하므로 든 로봇은 중심선 ±0.04 m로만 지나간다. 두 로봇이 나란히 서려면 0.65 m가 필요하다. 실제 차체는 약 0.18 × 0.16 m라, 빈 로봇 두 대라면 물리적으로 비껴 설 여지가 조금 있다. "한 대씩"은 교사 계획 기준이다.
- 넓은 문 1.00 m: 상자를 든 로봇 두 대(0.21 + 0.35 + 0.21 = 0.77 m)에 여유를 더했다. A·C로 가는 길은 좁은 문이 짧고, B로 가는 길은 넓은 문이 짧다. 목적지에 따라 경로가 갈린다.
- 복도: 북쪽 벽을 따라 동쪽으로 한 차선이다. 모든 운반이 이 복도를 지나므로 평균 운반 경로가 7.2 m다(door 4.6 m, two_doors 4.2 m. 교사 경로 계획 3 seed × 12상자 × 9칸 기준). 비켜 서는 자리의 중심은 차선 중심선에서 0.55 m 떨어져 있어 지나가는 로봇과 겹치지 않는다. 비켜 서는 자리는 1곳이다. 복도가 1.7 m라 두 곳을 두면 사이의 한 차선이 0.5 m도 안 되고, 든 로봇(0.42 m)이 들어갈 폭도 안 된다.
- TOP: 벽면에서 0.08 m 이상 떨어진 모든 바닥점(129 × 93 격자)과 문·복도·비켜 서는 자리의 바닥을 한 대 이상의 TOP이 본다(해석적 시선 테스트). 0.10 m 벽은 반대편 TOP 쪽으로 폭 4–6 cm의 바닥 띠를 가린다. 로봇 중심은 벽에서 0.17 m 이상 떨어져 있어 그 띠에 들어가지 않는다. 렌더 그림에서 문과 벽이 TOP 영상에 보인다.
- 로봇 자기 RGB(`robot-rgb-at-door.png`, door-G5-dynamic-s12 r2, SIM 101.5 s, 문 서쪽 0.74 m에서 팔을 접고 동쪽을 봄): 카메라가 0.21 m 높이에서 약 22° 아래를 본다. 문설주가 낮은 어두운 벽으로 보이고, 그 너머의 바닥, 구역 C, 구역 A, 다른 로봇까지 보인다. 즉 벽은 시야를 막지 않는다. 상자를 들고 팔을 내린 상태의 카메라는 바닥만 본다.
- 프롬프트: 이 세 지도에서만 시스템 프롬프트 끝에 `Static map:` 문단을 붙인다. 벽 선분 좌표, 통로 id·종류·폭·범위, 한 대씩인지, 비켜 서는 자리를 적는다. 정적 지도뿐이고 실시간 상태는 없다(`sim.zone_arena.static_map_text`).

## 교사 계획기와 통로 대치 규칙

- 정적 금지 영역 인터페이스(`harness/static_keepouts.py`, `c6cd334` 이후 변경 없음):
  - `keepout_rects(static_map, *, perimeter=False) -> tuple[(cx, cy, hx, hy, yaw), ...]`
  - `rect_distance(point, rect) -> float`, `inside_rect(point, rect, *, grow=0.)`
  - `pose_clear(pose_xy_yaw, footprint, rects, *, bounds=None, margin=0.) -> bool`: footprint는 몸체 좌표의 볼록 다각형 `[(x, y), ...]`, SAT
  - `swept_clear(a, b, footprint, rects, *, bounds=None, margin=0., step_m=.012, step_rad=radians(2)) -> bool`
  - `polygon_at`, `polygons_overlap`, `rect_corners`, `disc_footprint(radius, sides=16)`
  - `passage_zones(static_map, *, side_m=.3) -> [(passage_id, core_rect, zone_rect)]`: 한 차선 통로만. 입구 길이는 `MOUTH_M = {'door': .50, 'corridor': .35}`
- A*(`plan_path(..., rects=())`, `retreat_point(..., rects=())`): 벽에서 로봇 반지름만큼 떨어진다. 이미 더 가까우면 더 가까워지지 않게만 한다. 운반 반지름은 `TeacherRobot.radius(carrying)`이다. 작업에 `carry_radius_m`가 있으면 그 값, 없으면 0.21 m다. 벽이 있는 지도에서만 이동 단계 한도가 240초다(`ROUTE_DRIVE_PHASE_LIMIT_S`. v1은 120초).
- 대치 규칙(`ZoneTeacherExecutor.resolve_passages`): 같은 한 차선 통로 구역에서 2초 이상 멈춘 로봇이 둘 이상이면 우선순위가 가장 낮은 로봇이 비킨다. 우선순위는 (1) 통로 안에 있는지, (2) 로봇 0.30 m 안에 바닥에서 들린 상자가 있는지(상자를 들었는지), (3) 작은 id 순이다. 작업이 없는 로봇은 늘 비킨다. 집거나 내려놓는 중인 로봇은 비키지 않고 고정 장애물로 본다. 6초 넘게 풀리지 않으면 반대쪽 로봇이 비킨다(fallback). 비키는 로봇은 통로 중심선과 상대에게서 0.40 m 떨어진 가장 가까운 자리로 간다. 상대가 통로 구역을 떠나거나 20초가 지나면 끝난다.
- 입력 경계: 이 규칙은 위치, 멈춘 시간, 들린 상자, 로봇 id, 정적 지도만 읽는다. 동료의 작업·목적지·계획 경로·선언은 읽지 않는다. 테스트는 같은 물리 상태에서 작업을 바꿔도 같은 로봇이 비키는지, 해당 소스에 `.job`·`path_goal`·`slot_xy`·`box_body` 읽기가 없는지를 확인한다. 실행기는 협업 방식을 모르므로 plan_first·dynamic·independent에서 똑같이 동작한다. 로봇 자신의 작업 유무와 집기·내려놓기 중인지는 그 로봇 자신의 상태다.
- 남긴 것: 기존 일반 비키기(`resolve_blocks`)는 v1과 같다. 막힌 로봇의 동료 없는 경로를 `avoid`로 넘기므로 요청한 로봇의 경로를 쓴다. 벽 지도에서는 같은 통로 구역 안의 두 로봇에게 적용하지 않는다. `_taken_by_peer`(의도 기반 중재, PR #161 L1)도 그대로다. 둘 다 후속 수정 대상이다.
- 단위 테스트: 좁은 문에서 마주 본 두 로봇(r1 동쪽행, r2 서쪽행). 예전에는 서로 기다리기만 했다. 이제 2초 뒤 r2가 비키고 두 로봇 모두 목적지에 닿으며 `teacher_path_blocked`가 없다. 복도에서 마주 본 두 로봇도 r2가 복도 동쪽 끝 밖으로 물러나 풀린다.

## fixture 결과

조건: `--mode fixture`(규칙 응답, LLM 0회), 동기 SIM, `local_contact_fine`, `--record-replay`, weld OFF. G5 = A 빨강2 · B 청록1 · C 초록1+빨강1. G8 = A 빨강2 · B 청록2 · C 초록1+노랑1, 여분 빨강1·청록1. door와 zone_wide 기준선은 seed 11–13, two_doors와 corridor는 seed 11–12다(아래 "하지 못한 것"). door·two_doors·기준선의 소스는 `dc27695`, corridor는 `3183650`이다. 두 소스의 차이는 벽 지도의 240초 한도뿐이고, door·two_doors 실행은 한 번도 그 한도에 닿지 않았다(`teacher_path_blocked` 0).

집계(목표 달성 = 심판 기준, 교사 조건. makespan은 SIM 초 평균. 대치 = `passage_standoff` 수. 일반 비키기 = `yield` 수. 통로 대기 = 통로 구역에서 막히거나 비키며 기다린 SIM 초의 합. 접촉 = 로봇–로봇 관통 접촉 SIM 초의 합):

| 지도 | 목표 | 방식 | 목표 달성 | makespan | path_blocked | 대치 | 일반 비키기 | 통로 대기 s | 로봇–로봇 접촉 s |
|---|---|---|---|---|---|---|---|---|---|
| zone_wide(기준) | G5 | plan_first | 3/3 | 154.8 | 0 | 0 | 0 | 0 | 1.11 |
| zone_wide(기준) | G5 | dynamic | 3/3 | 150.2 | 0 | 0 | 0 | 0 | 1.11 |
| zone_wide(기준) | G5 | independent | 2/3 | 190.7 | 0 | 0 | 0 | 0 | 1.11 |
| zone_wide(기준) | G8 | plan_first | 3/3 | 156.2 | 0 | 0 | 0 | 0 | 0.0 |
| zone_wide(기준) | G8 | dynamic | 3/3 | 152.5 | 0 | 0 | 0 | 0 | 0.0 |
| zone_wide(기준) | G8 | independent | 0/3 | 258.7 | 0 | 0 | 0 | 0 | 37.18 |
| door | G5 | plan_first | 3/3 | 216.7 | 0 | 0 | 1 | 24.7 | 1.11 |
| door | G5 | dynamic | 3/3 | 220.2 | 0 | 0 | 1 | 21.2 | 1.11 |
| door | G5 | independent | 1/3 | 280.7 | 0 | 0 | 1 | 16.3 | 1.11 |
| door | G8 | plan_first | 3/3 | 230.8 | 0 | 0 | 1 | 3.8 | 0.0 |
| door | G8 | dynamic | 3/3 | 252.0 | 0 | 2 | 3 | 99.9 | 0.0 |
| door | G8 | independent | 0/3 | 335.0 | 0 | 1 | 1 | 28.5 | 0.0 |
| two_doors | G5 | plan_first | 2/2 | 211.5 | 0 | 0 | 0 | 1.8 | 1.11 |
| two_doors | G5 | dynamic | 2/2 | 187.5 | 0 | 0 | 0 | 0 | 1.11 |
| two_doors | G5 | independent | 1/2 | 247.0 | 0 | 0 | 0 | 3.4 | 1.11 |
| two_doors | G8 | plan_first | 2/2 | 194.0 | 0 | 0 | 0 | 0 | 0.0 |
| two_doors | G8 | dynamic | 2/2 | 211.8 | 0 | 0 | 0 | 7.2 | 16.68 |
| two_doors | G8 | independent | 0/2 | 320.5 | 0 | 0 | 0 | 1.5 | 16.98 |
| corridor | G5 | plan_first | 2/2 | 332.8 | 0 | 0 | 2 | 0 | 1.11 |
| corridor | G5 | dynamic | 2/2 | 335.5 | 0 | 1 | 2 | 37.3 | 1.11 |
| corridor | G5 | independent | 1/2 | 494.5 | 2 | 1 | 2 | 24.0 | 148.74 |
| corridor | G8 | plan_first | 2/2 | 368.0 | 0 | 0 | 3 | 0 | 0.2 |
| corridor | G8 | dynamic | 2/2 | 395.5 | 0 | 0 | 5 | 40.6 | 0.2 |
| corridor | G8 | independent | 0/2 | 506.0 | 0 | 1 | 6 | 44.8 | 0.3 |

실행별 표(부하 평균, 대기, 접촉, 해시)는 `results.json`의 `runs`와 `baseline_runs`에 있다. 요약:

- 실행 가능성: plan_first와 dynamic은 세 지도의 G5·G8에서 모든 seed가 목표를 달성했다(door 12/12, two_doors 8/8, corridor 8/8). 이 두 방식에서 `teacher_path_blocked`는 0이다. 로봇–벽 접촉은 42회 중 2회, 각각 한 프레임의 바퀴 스침(0.2–0.3 mm)이다. two_doors `wall_divider_1`과 corridor 출구의 북쪽 바깥 벽이다. 운반 중인 상자와 벽의 접촉은 0회다.
- 경로 비용: 같은 목표에서 makespan 평균(plan_first와 dynamic)이 zone_wide 150–156 s에서 door 217–252 s, two_doors 188–212 s, corridor 333–396 s로 늘었다. 복도는 모든 운반이 한 차선을 지나므로 기준의 약 2.3배다.
- 병목: 통로 대치는 door 3회, corridor 3회, two_doors 0회였다. 모두 우선순위가 낮은 로봇이 비켰고, fallback과 `no_spot`은 0회다. 비키기 6회 중 3회는 상대가 통로 구역을 떠나서 끝났고, 3회는 20초 한도로 끝났다. 한도로 끝난 경우는 상대가 구역 안에서 다른 일로 멈춰 있던 때다. 그 뒤로도 막힘은 생기지 않았다. two_doors에서는 넓은 문이 대부분의 대기를 없앴다(통로 대기 합 13.9 s, door 194.4 s). 로봇이 서 있던 시간은 재생 기준으로 기준선이 50–128 s, 벽 지도가 96–668 s다(`drive_wait_s`).
- 무통신(independent): 기준선과 마찬가지로 대부분 실패했다(`box_taken_by_peer`, 넘치게 채움). 벽 지도 때문에 생긴 실패가 아니다.
- 접촉: seed 11 G5는 모든 지도(기준 포함)에서 9.6–14 s에 적재 구역에서 r1과 r2가 1 mm 미만으로 스친다(기존 현상). two_doors G8 seed 12의 16.7–17.0 s 접촉은 A 접근 자리(3.97–4.16, 0.0)와 적재 구역에서 났다. 기준선에서도 같은 종류가 37 s 있었다(wide-G8-independent-s12).
- 해결하지 못한 사례: corr-G5-independent-s12. r1(상자로 가는 중)과 r2(운반 중)가 복도 동쪽 출구 바로 밖(x 4.15–4.40, y 0.48–0.78)에서 만나 199–400 s 동안 붙어 있었다. r2만 통로 구역 안이어서 대치 규칙이 적용되지 않았다. 계획기의 "동료에게 더 가까워지지 않기" 규칙 때문에 두 로봇 모두 경로가 있다고 판단해 일반 비키기도 일어나지 않았다. 두 작업 모두 `teacher_path_blocked`로 끝났고 목표는 달성하지 못했다. 같은 메커니즘이 벽 없는 기준선에도 있다. 이번에는 고치지 않았다.

## 하지 못한 것 / 검증 범위

- two_doors와 corridor의 seed 13(12회)은 돌리지 않았다. 다른 에이전트의 탐색 실행으로 호스트 1분 부하가 240–490까지 올라 한 회가 15–35분 걸렸기 때문이다. 빈 폴더에 `NOT-RUN.txt`를 두어 러너가 거절하게 했다. 코디네이터 요청 이후 동시 실행은 2개로 제한했다.
- corridor 첫 두 회(`dc27695`, 120초 한도)는 기록으로 남기되 집계에서는 뺐다(superseded). corr-G5-plan_first-s11은 복도 앞에서 42 s를 기다린 뒤 B 칸 1.3 m 앞에서 120초 한도에 걸렸다. 이 결과로 벽 지도의 한도를 240초로 바꿨다(`3183650`). 중간에 멈춘 실행 5개는 결과 없이 `matrix-aborted/`에 있다.
- seed 2–3개, 조건마다 소수다. 교사 조건의 실행 가능성 확인이며 방식 간 비교가 아니다. LLM 협업, RGB 스킬, 실제 로봇 결과가 아니다.
- 통로 대기 시간(`passage_wait`)은 벽 지도 실행기만 기록한다. 기준선과 비교할 때는 재생 기반 `drive_wait_s`(주행 단계에서 멈춰 있던 시간)를 쓴다.
- TensorBoard는 요청에 따라 갱신하지 않았다.

## 원본과 재현

- 원본(로컬 전용, gitignore): `/Users/changmin/projects/ugrp-worktrees/zone-hard-routes/outputs/zone-hard-routes-20260925/` (`matrix/`, `wide-baseline/`, `base-v1-*`, `ident-v1-*`, `matrix-superseded/`, `matrix-aborted/`, 실행마다 `result.json`, `teacher-events.json`, `scene.xml`, `replay/`, `team/`, `rgb/`). 파일 해시는 `results.json`에 있다. 원격 백업은 없다.
- 영상(관찰 전용, 4배속, 960×540): `media/door-G8-dynamic-s13.mp4`(대치 1회, 일반 비키기 3회, 통로 대기 65.5 s), `media/two-G8-dynamic-s12.mp4`, `media/corr-G5-dynamic-s12.mp4`(복도 대치 1회, 대기 30.2 s).
- 재현: `python experiments/2026-09-25-zone-hard-routes/run_matrix.py --output <dir> --jobs 2 [--variants ...]`, `analyze_runs.py --output <dir>/matrix --record <이 폴더> --media <dir>/media --videos <run>...`, `build_results.py`.
