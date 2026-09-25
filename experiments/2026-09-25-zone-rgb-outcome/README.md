# 2026-09-25 구역 작업 결과의 RGB 전용 판정 (감사 L4 대체 후보)

> **2026-09-25 결정으로 로봇 입력용 역할은 폐기, 평가 보조·향후 자기 카메라판 참고용.** 사용자 결정(PR #175 decision_log)으로 TOP 카메라는 로봇 LLM 입력에서 빠지고 평가·채점 전용이 되었다. 로봇 입력은 정적 지도, 설정 작업 순서표, 자기 RGB, 자기 명령 이력, 대화다. 따라서 이 모듈(TOP 전후 비교)은 로봇이 받는 완료 신호가 될 수 없다. 남는 역할은 두 가지다: 평가 쪽 교차 확인, 그리고 앞으로 만들 자기 카메라판의 참고 구현. 아래 "러너 연결 제안"도 이 결정 이전의 기록이다.

[경계 감사](../2026-09-25-zone-comm-boundary-audit/README.md)의 **L4**는 이렇다. 작업 영수증("issued sequence finished" / "executor stopped before finishing")과 깨우기가 교사의 정답 검사(들어 올린 높이, 운반 중 높이, 막힌 경로)에서 나온다. 이번 작업은 이를 대체할 **독립 모듈** `harness/zone_rgb_outcome.py`를 만들고, 기록된 실행으로 오프라인 평가했다.

- 러너(`scripts/run_zone_dispatch.py`), 교사(`scripts/zone_teacher.py`), `harness/zone_coordination.py`는 `claude/zone-team-a2`가 고치는 중이다. 그래서 **건드리지 않았다.** 연결은 아래 6절에 제안만 적었다.
- 실행 번들 레지스트리(`harness/rgb_execution_bundle.py`)도 건드리지 않았다. 새 파일은 번들 source closure 밖이며, 테스트로 확인한다.
- 제어·LLM·물리 스텝 실행은 없다. 기록된 관측 재생(replay) 상태를 **렌더링만** 다시 했다.

## 1. 판정 규칙

입력은 허용된 것뿐이다.
- 작업 배정 때("before")와 작업 뒤("after")의 TOP RGB 4장, 작성된 TOP 보정
- 정적 지도: 구역 사각형
- 자기 작업 지정: 물건 라벨, 종류(색), 출발 위치. 출발 위치는 첫 TOP RGB에서 만든 `box-labels.json` 값이다.
- 목표 영역: 자기 칸(slot) 또는 착지 영역(`zone_goal_v2.landing_layout`)
- (선택) 자기 RGB와 자기 **발행** 팔 펄스

시뮬레이터 자세, 접촉, 측정 관절, 교사 phase·outcome, 주입 플래그는 읽지 않는다. 테스트가 모듈 소스의 이름·호출에 `mujoco/qpos/xpos/setup_only/teacher/inject/contact/referee`가 없는지와 함수 인자 목록을 검사한다.

**발행 명령은 "무엇을 볼지"(종류·출발·목표)만 정한다. 성공 근거로는 쓰지 않는다.** 교사 조건에서는 명령 가지 자체가 정답에 조건화되어 있다. 예를 들어 carry 명령은 정답 높이 검사를 통과해야만 나온다. 그래서 명령 이력을 결과 근거로 쓰면 L4가 다른 모양으로 되살아난다.

검출기:
- 4색 상자: `zone_color_boxes.detect_top`(`top_zone_v2`). dev에서 `zone_perception_v1`과 결과가 같았고, 앞선 색 검출 기록의 test 성능이 더 좋아 기본으로 두었다.
- 화물 종류: `origin/claude/zone-cargo-perception`(PR #168)의 `harness/zone_cargo_perception.detect_all_cargo`를 **import**한다. 복사하지 않는다. 그 모듈이 main에 없으면 화물 종류 판정은 명시적으로 `RuntimeError`를 낸다.

가림 판정 `point_visibility`: 점 주위 바닥 고리(반경 4.5–11 cm)를 before와 after에서 비교한다.
- 고리 픽셀의 30% 넘게 바뀌면(최대 채널 차 > 28) 그 카메라에서는 가려졌다고 본다. 로봇·팔이 덮은 경우다.
- 상자만 빠져나간 경우 고리는 그대로다. 그래서 "보이는데 비었음"과 "가려짐"을 가를 수 있다.
- 고리가 절반 이상 화면 안에 있는 카메라만 쓴다.

| 결과 | 규칙 | 확신도 |
|---|---|---|
| `still_at_source` | 같은 종류가 출발지 6 cm 안에 보임 (tracked) | 0.95 |
| | 12 cm 안에 보임 (잡기 실패로 밀림) | 0.75 |
| | 출발지는 보이는데 목표에도 새로 나타남 (모순) | 0.50 |
| | TOP에서 출발지가 가려짐 + 자기 RGB에 그 색이 손 닿는 거리(≤0.5 m) 또는 화면 가장자리에 걸침 | 0.60 |
| `delivered` | 자기 목표 영역(±6 cm)에 같은 종류가 **새로** 나타남 + 출발지가 **보이면서 비었음** | 0.95 |
| | 위와 같으나 출발지가 가려짐 | **0.50 (확정 안 함)** |
| | 목표 영역 없이 구역만 있음: 구역 안 그 종류 개수 증가 + 출발지가 보이면서 비었음 | 0.60 |
| `seen_elsewhere` | 출발지가 보이면서 비었음 + 목표 밖 어딘가에 같은 종류가 새로 보임 | 0.80 |
| | 출발지가 보이면서 비었음 + TOP엔 없음 + 자기 RGB 손 닿는 곳에 그 색 | 0.55 |
| `not_seen` | 그 밖 (`source_occluded` / `source_empty_item_not_found` / `target_occluded` 표시) | 0 |

- 자기 구역 안, 자기 영역 밖에 같은 종류가 새로 보였는데 자기 영역이 가려져 있으면 증거로 쓰지 않는다. 그 물건이 자기 것인지 동료의 배달인지 알 수 없기 때문이다.
- 확신도는 규칙마다 정한 고정값이다. 보정된 확률이 아니다.
- **확정 정책 `commit`:** 작업 종료 뒤 0, 0.5, 1, 2, 4, 8초(SIM)에 다시 확인한다. `not_seen`이 아니고 확신도가 0.6 이상인 첫 판정을 확정한다. 끝까지 확정되지 않으면 `not_seen`으로 둔다.

## 2. 평가 방법

- 분할([split.json](split.json))은 test 채점 전에 고정했다.
  - dev: 스크립트 fixture 실행 22개(gate·gate2 12, zone-dispatch dev-fixture 9, diag 1)
  - test: LLM 실행 30개(ZC2 18, Z1–Z3 12)
  - 두 분할 모두 파지 실패 주입 실행을 포함한다.
- **입력 영상 재생성:** 각 실행의 `replay/model.mjb` + `states.npz`(SIM 30 fps)에서 qpos를 넣고 `mj_forward`한 뒤 렌더링했다.
  - TOP: 러너 `render_team_jpeg`와 같은 geomgroup, JPEG q95
  - 자기 RGB: 센서 옵션(그룹 4·5 숨김) + `raw_fisheye_remap`, q90
  - 충실도: 실행 중 저장된 TOP 106장(52개 실행 전부)과 비교했다. 평균 절대 픽셀 차는 최대 0.13이고, 20을 넘는 픽셀은 최대 0.21%다. 차이는 캡처와 재생 샘플 사이의 ≤17 ms 시각 차 때문이다.
  - 자기 RGB 표본 1장의 평균 차는 0.006이다.
- 작업 목록과 시각은 `result.json`의 jobs와 `teacher-events.json`(assign/job_end)에서 가져왔다. before는 배정 시각, after는 러너가 관측한 작업 종료 시각(실행기 idle)에서 +0, 0.5, 1, 2, 4, 8초다.
- 자기 발행 팔 펄스: 작업 종료 뒤와 상자로 가는 동안 교사가 큐에 넣은 `FOLDED`를 쓴다. 이것은 명령값이다(측정값이 아님).
- **평가 전용 정답** `eval-labels.json`은 판정이 끝난 뒤에만 읽는다. 기준은 확인 시점의 그 작업 상자의 시뮬레이터 자세다.
  - 바닥 위(z<0.05)이고 작업 구역 안이면 `delivered`
  - 바닥 위이고 배정 때 위치에서 0.15 m 안이면 `still_at_source`
  - 그 밖이면 `elsewhere` (들려 있는 경우 포함)
  - 교사 outcome은 따로 표로 적는다.
- 합성 사례(렌더만): 기록 상태에서 상자·로봇 qpos를 바꾸고 `mj_forward`만 했다. 10종 × 실행당 작업 2개다. dev 기반 실행은 gate-dynamic-nominal·dev-fixture-7, test 기반 실행은 ZC2-s13-dynamic-nominal·Z3-G8-dyn이다.
- 조정: dev에서 규칙 세 가지를 고쳤다. (1) 출발지가 가려졌을 때의 `delivered`를 0.8에서 0.5로 낮춤, (2) 목표가 가려졌을 때 구역 안의 새 같은 종류는 증거에서 뺌, (3) 화면 가장자리의 고리 일부 허용. 그 뒤 `88b033a`에서 모듈을 고정했다(파일 SHA-256 `58126da3…`). test와 synth-test는 한 번 채점했다.
  - test 채점 중 평가 스크립트가 한 번 멈췄다. 재생이 끝난 뒤 종료된 작업 2개(Z1-G8-dyn r2-2, r3-2)에 after 영상이 없었기 때문이다. 이 작업을 건너뛰도록 **스크립트만** 고치고 다시 채점했다. 모듈 규칙은 바꾸지 않았다.

## 3. 결과

행 = 평가 전용 정답, 열 = RGB 판정. "확정"은 8초 안의 commit 결과다.

**test (LLM 30회, 작업 204개; 2개는 after 영상이 없어 제외)**

> 정정(리뷰 뒤): 처음 올린 표는 "종료 시점 / 확정"을 한 칸에 적고 행 합계를 종료 시점 정답으로 적었다. 그런데 확정 판정은 **확정 시점의 정답**과 비교한다. test의 한 작업(Z1-G8-dyn r1-1)은 종료 시점 정답이 elsewhere였고, 확정 시점(+2 s) 정답은 still_at_source였다. 그래서 확정 표의 still_at_source 행은 38이 아니라 **39**이고(`results.json`과 일치), elsewhere 행은 0이다. 두 표로 나눠 적는다.

종료 시점 (+0 s, 정답도 +0 s)

| 정답 \ RGB | delivered | still_at_source | seen_elsewhere | not_seen |
|---|---|---|---|---|
| delivered (165) | 164 | 0 | 1 | 0 |
| still_at_source (38) | **0** | 33 | 0 | 5 |
| elsewhere (1) | **0** | 0 | 0 | 1 |

확정 (8초 안, 정답은 확정 시점)

| 정답 \ RGB | delivered | still_at_source | seen_elsewhere | not_seen |
|---|---|---|---|---|
| delivered (165) | 164 | 0 | 1 | 0 |
| still_at_source (39) | **0** | 39 | 0 | 0 |
| elsewhere (0) | 0 | 0 | 0 | 0 |

- 종료 시점 197/204, 확정 203/204.
- **잘못된 delivered: 모든 확인 시점에서 0건.**
- 교사 outcome별 확정 결과:
  - `placed_by_teacher` 164 → 모두 delivered
  - `grasp_failed_by_teacher` 8 → 모두 still_at_source
  - `box_taken_by_peer` 23 → 모두 still_at_source
  - `teacher_path_blocked` 8 → still_at_source 7, seen_elsewhere 1
  - `unfinished_at_run_end` 1 → still_at_source

**dev (fixture 22회, 작업 151개)**

| 정답 \ RGB (종료 시점 / 확정) | delivered | still_at_source | seen_elsewhere | not_seen |
|---|---|---|---|---|
| delivered (114) | 114 / 114 | 0 / 0 | 0 / 0 | 0 / 0 |
| still_at_source (33) | **0 / 0** | 28 / 32 | 0 / 0 | 5 / 1 |
| elsewhere (4) | **0 / 0** | 0 / 0 | 1 / 3 | 3 / 1 |

- 종료 시점 143/151, 확정 149/151. 잘못된 delivered 0건.
- `dropped_in_transit` 4건: seen_elsewhere 2, still_at_source 1(출발지 근처에서 떨어져 정답도 still), not_seen 1.

**감지까지 걸린 시간** (러너가 작업 종료를 본 시각 기준, SIM 초, 첫 확정 정답까지; 작업은 **종료 시점 정답**으로 묶음 — 그래서 still_at_source가 38이다)

| | delivered | still_at_source | elsewhere |
|---|---|---|---|
| test | 164/165가 0 s | 33/38이 0 s, 38/38이 ≤1.0 s | 1/1이 2.0 s |
| dev | 114/114가 0 s | 28/33이 0 s, 32/33이 ≤2.0 s | 3/4, 최대 4.0 s |

파지 실패 5건(test)은 종료 순간 팔이 아직 접히는 중이다. 상자가 50–160 px만 보여 검출이 안 되고 `not_seen`이 된다. 0.5–1.0초 뒤 `still_at_source`로 확정된다. 감사의 s14 사례(r1 green-1)는 +1.0 s에 확정됐다.

**합성 사례** (확정 결과가 "안전한 결과" 집합 안에 드는지)

| 사례 | dev | test |
|---|---|---|
| 경로 중간 낙하(보임) | 4/4 seen_elsewhere | 4/4 seen_elsewhere |
| 로봇 바로 앞 낙하(몸 아래) | 4/4 (seen_elsewhere 3, not_seen 1) | 4/4 (3, 1) |
| 잡기 실패, 로봇이 잡기 전 위치 | 4/4 still | 4/4 still |
| 잡기 실패, 로봇 몸이 출발지를 덮음 | 4/4 (still 2, not_seen 2) | 4/4 (2, 2) |
| 칸에 놓임, 팔 아래 | 4/4 delivered | 4/4 delivered |
| 칸에 놓임, 로봇 몸이 덮음 | 4/4 (delivered 2, not_seen 2) | **3/4** (delivered 1, not_seen 2, seen_elsewhere 1) |
| 다른 구역에 놓임 | 4/4 seen_elsewhere | 4/4 seen_elsewhere |
| 자기 칸에 다른 색, 자기 상자는 가려진 출발지 | 4/4 (still 2, not_seen 2) | 4/4 |
| 같은 구역 다른 칸에 같은 색, 자기 상자 가려짐 | 2/2 | 3/3 |
| **자기 칸에 같은 색(동료 것), 자기 상자 가려짐** | 2/2 not_seen | 3/3 not_seen |

- 마지막 사례는 색으로는 두 상자를 구분할 수 없는 경우다. 원시 판정은 `delivered` 0.5이지만 확정 기준(0.6) 아래라 확정되지 않는다(dev 1건, test 2건).
- 확정된 잘못된 delivered는 합성 dev·test 모두 0건이다.

## 4. 실패 양상

1. **색만으로는 같은 종류의 개체를 구분할 수 없다.** 출발지가 가려진 동안 같은 색 상자가 자기 영역에 오면 배달처럼 보인다. 그래서 출발지가 보일 때까지 확정을 미룬다. 출발지가 오래 가려지면 `not_seen`에 머문다.
2. **종료 순간의 팔 가림:** 파지 실패 직후 접히는 팔이 상자를 일부 가린다(검출 면적 게이트 미달). 0.5–1 s 재확인이 필요하다. 종료 순간 한 장만 보면 test still 5/38이 `not_seen`이다.
3. **구역 안이지만 칸 밖:** 교사가 경로 막힘으로 멈췄는데 상자는 구역 안에 있는 경우(test Z1-G8-plan r3-1)다. 자기 영역 기준이라 `seen_elsewhere`가 된다. 구역 기준 정답으로는 오답이지만 위험하지 않다.
4. **로봇 몸이 목표 칸을 덮음:** 같은 구역의 다른 새 같은 색이 증거에서 빠지는 조건은 목표가 "가려짐"으로 판정될 때뿐이다. 고리가 덜 바뀌어 "보임"으로 판정되면 `seen_elsewhere`가 된다(합성 test 1건).
5. **들려 있는 동료 상자:** `seen_elsewhere`는 "출발지가 보이면서 비었음"을 요구한다. 그래서 가려진 자기 상자를 동료가 나르는 같은 색으로 착각하지 않는다. 대신 그런 경우는 `not_seen`이 된다.
6. 확신도는 보정되지 않았다. 표본은 상자 4색뿐이다(화물 종류는 평가하지 않음).

## 5. 파일

- `harness/zone_rgb_outcome.py`: `job_outcome`, `sightings`, `point_visibility`, `own_near`, `job_spec`/`slot_target`/`landing_target`, `receipt`, `commit`
- `scripts/eval_zone_rgb_outcome.py`: `render` / `synth` / `score` / `validate`. 작업 흐름 `zone-rgb-outcome-eval`로 등록했다(오프라인, 렌더만).
- `tests/test_zone_rgb_outcome.py` 17개(CI 목록에 추가). 그린 TOP 영상으로 각 규칙, 같은 색 오인 확정 금지, 다른 색 거부, 구역 내 모호성, commit 정책, 영수증 문구, 모듈 입력 순수성(토큰·인자), 정적 지도에 물건 자세 없음, 번들 closure 밖, 화물 detector 부재 시 오류를 확인한다.
- 원본 출력: `/Users/changmin/projects/ugrp/outputs/zone-rgb-outcome-20260925/`(571 MB, 로컬에만 있고 원격 백업 아님)
  - `{dev,test,synth-dev,synth-test}/<run>/<job>/{frames,inputs.json,eval-labels.json}`
  - `score-*.json`, `frames-manifest.json`, `synth-manifest.json`, `replay-validation.json`
  - 해시와 요약은 [results.json](results.json)에 있다.
- 부하 평균(1/5/15분)은 공용 호스트라 기록했다. 동시 실행은 최대 2개였다.
  - dev 렌더: 5.26/8.95/22.23 → 14.05/9.11/17.32
  - test 렌더: 6.61/7.26/17.75 → 28.88/33.06/25.40 (다른 작업 부하가 겹침)
  - synth-dev: 4.93 → 5.75
  - synth-test: 25.22 → 20.89
  - 검증: 6.12/15.18/19.25

## 6. 러너 연결 제안 (통합 담당용, 구현하지 않음)

호출 위치: `scripts/run_zone_dispatch.py`의 `collect_done()`이다. 기준 main `0add360`의 `if rid in active and not robot.busy:` 블록이다.

1. **before 보관:** `assign()`에서 배정 직전 캡처의 TOP(`tops`)와 파일 이름을 `active[rid]['rgb_before']`에 둔다. dynamic과 independent는 배정 직전에 이미 캡처하므로 추가 렌더가 없다.
2. **after 확인:** 실행기가 idle이 되면 다음을 호출한다.
   ```
   frames, tops = zone.capture(f'jobend-{job_id}', robots=[rid])
   res = zro.job_outcome(spec, before, tops, static_map, own_rgb=frames[rid]['own'], own_servo_pose=<그 로봇 포트의 마지막 발행 팔 펄스>)
   ```
   `spec`은 `zro.job_spec(robot_ids=rid, item=job['box'], kind=labels[box]['kind'], source_xy_m=labels[box]['floor_xy_m'], zone=job['zone'], target=zro.slot_target(static, slot))`로 만든다. 팀 작업(A2)은 `robot_ids=[...]`, `target=zro.landing_target(area)`를 쓰고 화물 detector가 필요하다.
3. **영수증 대체:**
   - `own_jobs[rid][-1]['status'] = zro.receipt(res)['status']`
   - 보고에는 `rgb_outcome`·`confidence`·`evidence_images`를 넣고, `executor_receipt` 문구는 뺀다.
   - `finished`에는 **확정된 `delivered`만** 넣는다. `still_at_source`·`seen_elsewhere`는 `stopped`에 넣는다.
   - 확정되지 않은 결과(`not_seen` 또는 확신도 < 0.6)는 `pending_checks`에 넣는다. 이후 매 캡처(다음 claim 라운드, 또는 0.5 s 간격 최대 8 s)마다 같은 before로 `job_outcome`을 다시 부르고, `zro.commit`으로 첫 확정을 쓴다. 8 s 뒤에는 "RGB check: item not seen"으로 남긴다.
   - `delivered` 목록(dynamic의 `remaining_need`)도 RGB 확정 배달에서 만든다.
4. **깨우기:**
   - dynamic: `stats['done_robots'] = []` 초기화를 "실행기 idle"이 아니라 **RGB 판정이 확정될 때**(새 증거)로 옮긴다.
   - independent: `robot.outcome != 'placed_by_teacher'` 대신 **RGB 확정 결과가 `delivered`가 아닐 때** `SOLO_REASK_S` 뒤에 다시 묻는다.
   - 칸 반납(`slots.give_back`)은 실행기 쪽 자원 관리라 모델 입력이 아니다. 그래도 RGB 결과가 `still_at_source`/`seen_elsewhere`일 때 반납하고, `not_seen`이면 확정까지 보류하는 쪽이 일관된다.
5. **남는 교사 의존 (L4b):** 실행기가 **언제** idle이 되는지는 여전히 교사의 정답 검사가 정한다. 파지 실패는 들어 올린 높이로, 낙하는 운반 중 높이로 일찍 끝난다. 영수증 내용은 RGB로 바뀌지만, "일찍 끝났다"는 시각 신호는 남는다. 없애려면 다음 중 하나가 필요하다.
   - 실행기가 결과와 관계없이 고정된 명령 순서·시간 예산을 끝까지 수행
   - 로봇 쪽이 작업 중 캡처마다 RGB 판정으로 스스로 중단을 결정
   이것은 이 PR의 범위 밖이다.

## 7. 검증하지 않은 것

- 러너에 연결한 폐루프 실행(fixture·LLM)은 하지 않았다. 영수증을 바꾸면 조정 결과(LLM 호출 수, makespan, 성공)가 어떻게 달라지는지 모른다.
- 평가 영상은 실행 중 캡처가 아니라 재생 상태에서 다시 렌더링한 것이다(충실도는 위 2절). 실행 중 캡처와 바이트까지 같지는 않다.
- 화물 종류(can·tile·beam·crate·frame)와 팀 작업의 착지 영역 판정은 평가하지 않았다. 해당 detector(PR #168)는 아직 main에 없다.
- 기록된 낙하(`dropped_in_transit`)는 dev 4건, test 0건뿐이다. 낙하 판정 근거는 대부분 합성 사례다.
- 자기 RGB 규칙은 어떤 분할에서도 확정 결과를 만들지 않았다(합성 포함). test 재확인 시점에서 3번 발동했지만 모두 확정 결과가 아니었다. `seen_elsewhere_own_rgb` 2회는 0.55로 확정 기준 아래였고, `still_at_source_own_rgb` 1회는 이미 앞 시점에 확정된 뒤였다. 곧 이 규칙의 효용은 **검증되지 않았다.** 또 작업 종료 순간의 발행 팔 자세를 `FOLDED`로 두었는데, 실제 포트 발행 기록과 대조하지 않았다.
- 확신도 보정, 다른 지도(`zone_wide_door` 등)와 조명 변화는 확인하지 않았다.
- 작업 흐름 등록 때문에 `tests/test_simulation_workflow_manager.py`의 작업 흐름 개수(27 → 28)를 바꿨다. PR #168도 같은 곳을 고치므로 병합할 때 충돌이 난다.

## v2 (2026-09-25 밤 – 09-26): PR #170 리뷰 반영

Codex 적대적 검토(PR #170 코멘트)와 코디네이터 확인에 따라 고쳤다. 위의 v1 결과는 그대로 두었다. 다만 v1 test 표의 행 합계는 정정했다(38 → 39). 커밋 이력은 다시 쓰지 않았다. 원격에서 들어온 main 병합 커밋(`39a4e90`)도 그대로 받았다.

### 무엇이 바뀌었나 (전 → 후)

| 리뷰 항목 | v1 동작 | v2 동작 |
|---|---|---|
| (1) 확정 정책 | `commit()`이 끝까지 저신뢰면 마지막 결과를 돌려줬다. `receipt()`가 "delivered" 문구를 낼 수 있었고, 평가기만 따로 `not_seen`으로 바꿨다 | 모듈 API의 `decide()`/`JobTracker`가 `status: confirmed/unconfirmed`를 낸다. `receipt()`는 미확정이면 항상 "unconfirmed, still observing"이다. 평가기의 별도 바꿈을 없앴고, 영수증·훅·평가가 같은 정책을 쓴다 |
| (2) 가림 | 배정 때와 지금의 고리가 같으면 "보임"으로 봤다. 배정 전부터 서 있는 로봇이 출발지를 가려도 "보이는 빈 출발지"가 되었고, 동료의 같은 색이 목표에 있으면 delivered 0.95 | **출발지가 비었음을 입증해야 한다.** 한 TOP 안에서 넷 모두 성립해야 한다: 라벨 프레임에서 그 카메라가 물건을 봤다 + 그때 고리가 바닥 같았다 + 지금도 고리가 그대로다 + 지금 중심이 맨바닥이다. 입증하지 못하면 `delivered_source_unproven`(0.5)이고 확정되지 않는다. 화물은 전체 발자국이 착지 영역 안, 잘리지 않은 검출, 착지 yaw ±15°(대칭 고려)를 요구한다. 들린 상태는 운반자 발행 집게가 모두 알려져 있고 열려 있어야 한다 |
| (3) 교사 종료와의 결합 | 교사 종료 시각부터 관측했다. 교사 phase로 자기 RGB 사용 여부를 정하고 `FOLDED`를 가정했다 | **배정부터 SIM 1초 고정 주기**로 관측한다. 확정 경로는 셋이다: 안정 2틱 delivered, 발행 명령 기록상 출발지에서 3회 놓음, 교사와 무관한 180초 기한. 같은 로봇이 다음 작업을 받으면 4초 더 보고 확정한다. 자기 RGB 팔 자세와 집게 조건은 **기록된 실제 발행 명령**(새 실행의 `port-commands.jsonl`)에서만 가져온다. 기록이 없는 옛 실행에는 쓰지 않는다 |
| (4) 덮어쓰기 | 같은 출력 경로에 다시 썼다 | 모든 하위 명령이 새 `--out` 디렉터리를 요구하고, 이미 있으면 중단한다 |
| (5) 표 불일치 | README 38 / results 39 | v1 표를 종료 시점과 확정 두 개로 나눠 39로 맞췄다 |

**이 훅만으로 L4는 없어지지 않는다(L4b).** 교사 실행기가 언제 멈추는지, 그래서 로봇이 언제 다음 작업을 받는지는 정답 검사가 정한다. v2의 "다음 작업을 받으면 창 닫기"도 그 시각을 쓴다. 평가에서 정답에서 온 입력은 배정 시각(러너가 발행한 자기 작업)과, 그 발행 시각을 통한 창 닫기뿐이다.

### 평가 방법 (v2)

- 분할 [split-v2.json](split-v2.json)을 녹화와 채점 전에 고정했다.
  - dev_v2 = v1 dev 22회(명령 기록 없음) + 새 fixture 6회(명령 기록 있음)
  - test_v2 = **새 fixture 12회**: zone_wide 41–43 × dynamic/independent/plan_first, zone_open 44–46 dynamic
  - v1 test는 이미 한 번 쓴 분할이라 은퇴시켰다.
- 새 녹화 `record`:
  - 러너(`scripts/run_zone_dispatch.py`)와 교사는 **고치지 않았다.** 포트 `apply`를 감싼 관측 전용 래퍼가 발행 명령만 남긴다.
  - 진단용 운반 중 낙하 주입: N번째 작업에서 운반 3초 뒤 집게를 여는 명령을 발행한다. 이 명령도 기록에 남는다.
  - 파지 실패 주입은 러너의 기존 옵션을 썼다.
  - test 12회 가운데 `test-w43-plan`은 협상 중 `StopIteration`으로 작업 없이 끝났다(fixture 계획 쪽 문제, 이번 범위 밖). `test-o45/o46-dyn`, `test-w42-inde`는 교사의 경로 막힘 재시도가 반복되어 SIM 예산(900 s)까지 갔다.
  - 러너 소스 SHA: dev 6회는 `f32e41c`(2회)/`079ce58`(4회; 문서만 다름), test 12회는 모두 `c30091b`.
- 추적 `track`은 dev에서만 조정했다.
  - 조정 1: 다음 작업을 받으면 창 닫기 (dev 스모크에서 잘못된 delivered 2건: 실패한 작업이 계속 열려 있다가 같은 로봇의 다음 작업이 같은 칸을 채움)
  - 조정 2: 닫은 뒤 4초 유예 (닫는 순간 로봇 몸이 상자를 가림)
  - 그 뒤 `c30091b`(모듈 SHA-256은 `results-v2.json`)로 고정했다. dev 전체를 다시 추적했고, test는 **한 번** 추적·채점했다.
- `label`(평가 전용 정답: 확인 시점의 상자 자세)은 결정 뒤에 따로 썼고, `score`만 읽는다.

### 결과

**test_v2** (새 12회, 작업 269개; 185개는 교사 경로 막힘 재시도의 반복 작업). 행 = 확정 시점 정답, 열 = RGB 결정:

| 정답 \ RGB | delivered | still_at_source | seen_elsewhere | not_seen | unconfirmed |
|---|---|---|---|---|---|
| delivered (57) | 57 | 0 | 0 | 0 | 0 |
| still_at_source (206) | **0** | 196 | 1 | 3 | 6 |
| elsewhere (6) | **0** | 0 | 3 | 2 | 1 |

- 정답 256/269.
- **잘못된 delivered 0건.** 확정 시점 정답 기준으로도, 실행 끝 정답 기준으로도 0이다. 비배달 틱에서 나온 `delivered_source_proven` 관측도 0회다.
- **실행 끝까지 미확정 7/269 (2.6%)**. 확정된 `not_seen`은 5건이다.
- 경로 막힘 반복을 빼면 82개 중 71개가 정답이고 미확정은 5개다.
- 교사 outcome별:
  - `placed_by_teacher` 57 → 모두 delivered
  - `grasp_failed_by_teacher` 11 → still_at_source 10, not_seen 1
  - `dropped_in_transit` 11 → seen_elsewhere 4, not_seen 4, unconfirmed 3
  - `teacher_path_blocked` 187 → still_at_source 185, unconfirmed 2
- 감지 시간(평가 전용 기준):
  - delivered: 정답상 배달된 뒤 중앙 5초, 최대 6초. 2틱 안정 조건 때문이다.
  - 비배달 확정: 교사 종료 뒤 중앙 5초, p90 15초, 최대 170초(기한 경로).

**dev_v2** (28회, 작업 199개): 정답 182/199, **잘못된 delivered 0**, 미확정 4 (2.0%), 확정 not_seen 13.

**결정 동등성:** 실행 중 저장된 TOP(v1 dev+test 52회)과 재렌더 TOP에 같은 `observe`를 돌렸다. 관측 3610건 중 3609건의 규칙이 같았다. 다른 1건(ZC2-s14-independent-graspfail r1-3, 217.8 s)은 저장 영상에서 `not_seen`, 재렌더에서 `still_at_source_tracked`였다.

**합성 사례** (dev·test 각 46개, 적대적 가림 사례 포함):
- 확정된 잘못된 delivered는 0건이다. **다만 평가 도구의 결함이 있었다.** 합성 추적기를 `assigned_at = t0 − 기한`으로 먹여서, 첫 틱에 이미 기한 확정(`deadline_not_seen`)이 났다. 그래서 합성에서는 delivered를 확정할 수 없었다. `placed_clear` 0/4가 그 흔적이다.
- 그래서 **관측 규칙**을 안전 척도로 본다.
  - 적대적 사례 `static_occluder_*`(배정 또는 라벨 프레임부터 로봇이 출발지 위에 서 있음) 12개 중 10개가 `delivered_source_unproven` 또는 `still_at_source_conflict`였다. 옛 방식의 "그대로면 보임" 판정은 이 사례를 막지 못한다(단위 테스트로 확인).
  - 나머지 2개(synth-test, zone_open o44 r1-3)는 `delivered_source_proven`이었다. 0.10 m 서쪽에 둔 로봇이 한 TOP에서는 그 자리를 가리지 못해, 출발지가 실제로 보이면서 비어 있었다. 이것은 가림 오판이 아니라 아래 색 동일성 한계와 같은 경우다. 올바른 도구였다면 확정되었을 것이다.
  - `identity_swap`(자기 상자는 안 보이게 사라졌고, 출발지는 보이면서 비었고, 같은 색 상자가 자기 칸에 있음)은 dev 2/2, test 2/2가 `delivered_source_proven`이었다. **색만으로는 같은 종류의 두 상자를 구분할 수 없다는 한계는 남는다.**
- 합성 결과는 `results-v2.json`의 `synth-*`에 있다.

### 남은 한계와 미검증
- L4b: 실행기 종료 시각은 여전히 정답이 정하고, 창 닫기도 그 시각을 쓴다.
- 색 동일성: 같은 종류의 다른 상자가 자기 영역에 놓이고 자기 상자가 안 보이게 사라지면 delivered가 될 수 있다(합성 관측으로 확인).
- 합성 도구의 기한 결함은 고치지 않았다(범위 확장 금지 지시). 합성 결과는 관측 규칙 기준으로만 읽어야 한다.
- 실제 낙하 11건 중 확정 정답은 4건이다. 7건은 not_seen 또는 미확정이다. 낙하 자리가 로봇 아래라 보이지 않는 경우가 많다.
- 화물(팀 착지)의 발자국·yaw·집게 조건은 단위 테스트로만 확인했다. 화물 실행 기록은 없다.
- 러너 폐루프 연결은 하지 않았다. 2026-09-25 결정으로 로봇 입력용으로는 연결하지 않는다.
- 부하 평균: 공용 호스트라 1분 부하가 최대 90까지 올랐다. 녹화별 부하는 `results-v2.json`의 `recordings`와 `load_log`에 있다. 동시 실행은 2개 이하였고, 코디네이터 요청 뒤 새 실행은 단일 스레드 환경변수로 띄웠다.
- 원본(로컬 전용, 원격 백업 아님):
  - `/Users/changmin/projects/ugrp/outputs/zone-rgb-outcome-runs-20260925/` (녹화)
  - `/Users/changmin/projects/ugrp/outputs/zone-rgb-outcome-v2-20260925/` (추적·정답·점수·합성·동등성, 실패·폐기한 시도 포함)

### 러너 연결 제안 v2 (2026-09-25 결정 이전 작성, 기록용)

**먼저 분명히 해 둔다. 이 훅만으로는 L4가 없어지지 않는다.** 교사 실행기가 언제 멈추는지(파지 실패 뒤, 낙하 뒤, 경로 막힘 뒤)는 여전히 정답 검사가 정한다(L4b). 실행기가 멈춘 로봇에게 다음 작업을 주는 시각도, 그 로봇의 자기 명령 이력이 멈추는 시각도 모두 그 정답에 조건화되어 있다. RGB 판정은 영수증의 **내용**을 정답과 떼어 놓을 뿐, **시각** 신호까지 떼지는 못한다.

1. **배정할 때** (`assign()`): 다음으로 `JobTracker`를 만든다.
   ```
   zro.JobTracker(spec, reference_tops, before_tops, static_map, assigned_at=t)
   ```
   - `reference_tops`: 라벨을 만든 첫 캡처(`zone.capture('start')`의 tops)
   - `before_tops`: 배정 직전 캡처
   - `spec`: `zro.job_spec(...)`, 목표는 `slot_target` 또는 `landing_target`
2. **관측 시계:** `ZoneRun.step` 루프에서 **SIM 1초마다**(`zro.CADENCE_S`) TOP을 한 번 캡처한다. 확정되지 않은 모든 tracker에 `update(t, tops, commands=<각 로봇 포트의 발행 명령 기록>, own_rgb=<그 로봇 RGB>)`를 부른다. 실행기 idle 여부와 관계없이 돈다.
   - 포트에 발행 기록을 남기는 기능이 필요하다. 평가에서는 `record`의 관측 전용 래퍼로 남겼다.
3. **영수증:** `own_jobs[rid][-1]['status'] = zro.receipt(decision)['status']`로 쓴다.
   - 확정 전에는 "RGB check: unconfirmed, still observing"이다.
   - `finished`에는 확정된 `delivered`만, `stopped`에는 확정된 `still_at_source`·`seen_elsewhere`·`not_seen`만 넣는다.
   - dynamic의 `delivered` 목록과 게시판 보고도 같은 확정 결과에서 만든다. 미확정 결과를 따로 해석하는 경로를 두지 않는다.
4. **작업 창 닫기:** 같은 로봇(팀이면 운반자 누구든)에게 다음 작업을 발행하면 `tracker.close(t)`를 부른다. 4초 더 본 뒤 그때까지의 관측으로 확정하고, 그 뒤 영상은 쓰지 않는다. 다음 작업이 같은 영역을 채우는 영상이 앞 작업의 증거가 되지 않게 하기 위해서다.
   - 확정 경로는 셋이다: 안정 2틱 `delivered`, 출발지에서 3회 놓음, 배정 뒤 180초 기한.
5. **깨우기:**
   - dynamic: 동료들을 깨우는 사건은 "실행기 idle"이 아니라 **tracker 확정**이다.
   - independent: 확정 결과가 `delivered`가 아니면 `SOLO_REASK_S` 뒤에 다시 묻는다.
   - 확정 전인 로봇에게 새 작업을 묻는 조건은 여전히 실행기 idle이다(L4b). 이것을 없애려면 실행기가 결과와 관계없이 고정된 명령 순서를 끝까지 수행하거나, 로봇이 RGB 판정으로 스스로 중단을 정해야 한다.
6. **칸 반납:** 확정된 비배달 결과일 때만 반납한다. 미확정이면 보류한다.
