# 손목 카메라 전용 운반 시야와 zone 상자 스킬 (2026-09-25, M1 스킬 측)

**범위:** M1(`zone_wide_door`에서 로봇 1대가 손목 카메라만으로 청록 상자 1개를 문 너머로 배달)의 **시야·스킬 측**을 다룬다. 태그와 위치 추정은 PR #177(`claude/zone-owncam-loc`)이 맡는다.
- 이 기록의 스킬 결과는 모두 `pose_source=gt_stub_eval_only`다. 위치 추정기 자리에 시뮬레이터 정답을 넣은 **스킬 격리 시험**이며, 문을 통과하지 않았다. **M1 성공이 아니다.**

## 요청과 사용자 결정 (2026-09-25)
- 로봇 입력은 자기 `robot_cam`(MasterPi 팔 끝 어안)·정적 지도·자기 발행 명령 이력뿐이다. `nav_cam`은 금지다. TOP은 평가에만 쓴다.
- AprilTag는 벽·문기둥에만 붙인다(화물·로봇 금지). weld는 OFF, 물리는 zone 장면 그대로 `local_contact_fine`이다. 기존 코드를 재사용한다.

## 무엇을 만들었나
| 파일 | 내용 |
|---|---|
| `harness/owncam_view.py` | 발행 PWM → 손목 카메라 광선, 가림 마스크를 반영한 바닥·벽면 가시 범위, 태그 투영 크기. 순수 함수 |
| `scripts/probe_owncam_carry_view.py` | 진단 probe. 교사(GT)가 파지·주행하고, 자세별 렌더·상자 유지·시험 태그 검출을 기록한다. 시험 태그는 probe 장면 전용 시각 평면이며 지도 파일은 바꾸지 않았다 |
| `harness/wrist_zone_skill.py` | **`wrist_zone_skill_v1`**. N7 `VisualBoxSkill`을 서브클래스로 재사용하고, N7 클래스와 기본값은 그대로 둔다. 추가한 것: 주문서 픽업 칸까지 자세 추정 기반 주행, carry 자세, 슬롯 앞 정렬, 놓은 뒤 자기 RGB로 다시 보기 |
| `scripts/run_zone_owncam_skill.py` | 스킬 격리 러너. 사전 등록한 시나리오 501–505와 개발용 401–402가 있다. 로봇 입력은 `robot_cam`뿐이고, 정답은 평가 전용 파일에 따로 쓴다 |
| `tests/test_wrist_zone_skill.py` | 순수 테스트 17개. nav_cam 거부, 모듈의 world/TOP 접근 없음, N7 불변, 폴백 표시 여부 |

- 실행 번들 ID는 쓰지 않았다(dispatch 어댑터와 번들 closure는 바꾸지 않음).
- workflow도 등록하지 않았다. 등록부 `configs/simulation_workflows.json`과 `scripts/run_ci_tests.py`는 PR #169·#173·#177이 함께 바꾸는 파일이라 충돌을 피했다. 새 테스트의 CI 등록은 병합 순서가 정해진 뒤에 한다.

## 1. 상자를 든 상태의 손목 시야 (probe)
| 실행 | 소스 | 순서 | SIM s | 부하 평균(시작) |
|---|---|---|---|---|
| probe1 | `4219506` | 휴식 → 주행 → 문 | 482 | 8.6 |
| probe2 | `a868328` | **문 먼저**(파지 직후, 가림 최악) → 휴식 → 주행 → 구역 C | 559 | 51 |

- **가림:** 카메라가 집게에 고정돼 있어서 들고 있는 상자가 항상 같은 픽셀을 가린다. 파지 직후에는 프레임의 62–64%(유효 픽셀의 약 75%)가 가려지고, 위쪽 약 10° 띠만 보인다. pan은 가림을 풀지 못한다. 자세는 이 띠가 향하는 높이와 거리만 바꾼다.
- **상자 유지:** 모든 자세, 0.4 m 후진·전진, 90° 제자리 회전에서 상자를 놓치지 않았다(weld OFF). 다만 손가락 안에서 누적 미끄러짐이 있었다. probe2 기준으로 파지 후 약 480 SIM s에 1.6–1.8 cm였다.
- **자세 권고** (발행 PWM 3/4/5, 집게 1500, pan 1500):
  - carry_p30 `777/2053/1646`: 광축 -22.6°. 파지 직후 바닥 0.67–3.7 m, 1.0 m 앞 벽 높이 0.005–0.15 m가 보인다. **운반 기본 자세**다.
  - look_p20 `1072/2400/1482`: 광축 -12.5°. 멈춰서 보는 자세이며, 1.0 m 앞 벽 높이 0.15–0.32 m가 보인다.
  - look_p10 `897/1998/1598`: 광축 -2.5°. 높은 곳(0.27–0.54 m)을 본다.
  - 교사 carry(hover, 도구 pitch -90°): 바닥 0.25–0.32 m만 보이고 **벽 태그는 0개**다.
- **구역 칠:** 구역 C 앞에서 보라 칠이 carry_p30에서 화면의 49%, look_p20에서 33%를 차지했다. 교사 hover에서는 0%였다.
- **시험 태그 검출** (파지 직후, OpenCV 36h11):
  - 벽면 0.05 m 높이 태그: carry_p30으로 0.6–1.0 m에서 44–68 px, look_p20으로 1.5 m에서 30 px. **0.3 m에서는 검출되지 않았다.**
  - 문기둥 0.15 m 높이 태그: look_p20으로 1.5→0.3 m 전 구간 31–104 px. 단 0.6 m 이하에서는 pan ±24°가 필요했다.
  - 0.25 m 높이: look_p10에 pan을 더해야 보였다.
  - 태그 크기: 0.08 m 태그는 1.5 m에서 30 px, 0.05 m 태그는 18 px로 둘 다 검출됐다.
- **태그 권고**(PR #177 코멘트 [issuecomment-5833491542](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/177#issuecomment-5833491542)에 게시):
  - 벽면 0.05 m 태그(검은 사각 0.072 m)는 유지한다.
  - 각 문 가장자리 바깥 0.05 m에 문기둥 판(폭 0.09 m, 높이 0.30 m, 양면)을 세우고 0.072 m 태그를 **높이 0.15 m와 0.25 m**에 붙인다.
  - 문 양옆 1 m 안은 0.30 m 간격, 그 밖은 0.5 m 간격으로 붙인다.
  - 정책: 운반은 carry_p30으로 하고, 문 1.5 m·0.6 m 앞에서 멈춰 look_p20으로 pan {1230, 1500, 1770}을 훑는다.
- **한계:** SIM 손목 영상은 핀홀 렌더를 어안으로 재매핑한 것이라 유효 시야가 약 54°×42°다. 실물 어안은 더 넓다.

![wrist views](wrist-views.jpg)

## 2. N7 체인에서 nav_cam·TOP 사용처 감사
| 모듈 | 카메라 | wrist 전용 버전에서의 처리 |
|---|---|---|
| `visual_box_skill.VisualBoxSkill` | `robot_cam`만(`_validate_observation`) | 서브클래스로 그대로 재사용 |
| `markerless_box`, `visual_box_surface`, `visual_attachment`, `markerless_face`, `visual_floor` | 자기 RGB와 발행 PWM | 재사용 |
| `scripts/evaluate_gemini_team.py` (N7 러너) | LLM 주행 입력으로 `nav_cam` 사용(115, 201, 211, 278행) | 쓰지 않음. 주행은 자세 추정 인터페이스로 대체 |
| `visual_drive_guard.validate_visual_drive` | `nav_cam` 전용 | 쓰지 않음. carry_p30의 바닥 띠(0.67–3.7 m)로 대체할 예정(미구현) |
| `visual_navigation.VisualNavigator`, `visual_transport` | `nav_cam` 전용 | 쓰지 않음 |
| `visual_placement.inspect_placement` | wrist와 **nav_cam** 둘 다 필요 | 쓰지 않음. `zone_color_boxes.detect_own(own_zone_v2)`와 자세 추정으로 슬롯을 판정 |
| `solo_box_transport` (dispatch v61) | 조작은 own, 목표 서보는 **TOP** | 파지 설정만 재사용(`previous_endpoint`, 채도 150, `release_refine_ground_fit`) |
| `camera_pixel_grasp` | TOP(공동 빔) | 해당 없음 |
| `zone_color_boxes.detect_own` / `detect_top` | own / TOP | own만 사용 |

## 3. 스킬 격리 결과 (`zone_wide_door` 동쪽 열린 구역, 문 통과 없음)
- 소스는 `d016c04`로 고정했다. 테스트·집계 파일만 더한 `3982bde`에서 이어 돌렸고, 러너 closure는 같다. 501은 `d016c04`에서 실행했다.
- 조건: 시나리오당 1회, SIM 한도 300 s, 동기 SIM, `OMP_NUM_THREADS=1`. 호스트 부하 평균은 20–120이었다(코호트 로그).

| seed | 슬롯 | 결과(러너 판정) | GT 파지 | GT 슬롯 | SIM s | 면 법선 출처 |
|---|---|---|---|---|---|---|
| 501 | B1 | SIM_LIMIT(접근 교착) | ✗ | ✗ | 300.4 | – |
| 502 | A3 | SIM_LIMIT(접근 교착) | ✗ | ✗ | 300.4 | – |
| 503 | A1 | **OWN_RGB_PLACEMENT_IN_SLOT** | ✓ | ✓ (-5.3, -12.6 mm) | 127.3 | 지도 관례 폴백 |
| 504 | C3 | SIM_LIMIT(접근 교착) | ✗ | ✗ | 300.6 | – |
| 505 | B2 | CARRY_VISUAL_GRASP_DRIFT | ✓ | ✗ (들고 있는 채 정지) | 173.3 | 지도 관례 폴백 |

- **사전 등록 코호트 결과:** 파지 2/5, 슬롯 배치 1/5 (pose_source=gt_stub_eval_only). 스킬의 자기 판정과 GT는 5/5 일치했고, 거짓 성공은 0이다. weld eq_active 최댓값 0, 벽 접촉 0.
- **개발 시드(성공률에 합산하지 않음):**
  - 401: 첫 실행은 면 정렬 대기로 실패. 수정 후 성공이며 같은 소스로 재실행해도 SIM 172.854 s로 같았다.
  - 402: 첫 실행은 놓기 확인 실패(`release_refine_ground_fit` 누락). v61 설정을 넣은 뒤 성공했다.

### 실패 원인
1. **접근 교착 (501, 502, 504):**
   - 원인은 zone 천장 조명이다. 상자 앞면 아래쪽이 회색으로 렌더되어, 청록 실루엣의 바닥 직육면체 적합이 흔들린다. 투영 IoU 중앙값은 0.70(0.7 이상인 프레임은 약 절반)이고 가로 추정 표준편차는 6–7 mm였다. 성공 사례는 IoU 0.85–0.93, 표준편차 2.7–3.5 mm였다.
   - N7 접근은 pan(±20 PWM)과 수직 오차 게이트가 **같은 프레임에서** 둘 다 통과해야 전진한다. 두 게이트가 번갈아 걸려 x≈0.25–0.28 m에서 멈췄다. 전진은 7–15회뿐이었고, pan 명령은 1492와 1524 사이를 수백 번 반복했다.
   - 성공 시드(503·401·402)도 같은 0.40 m 대기 거리에서 출발해 전진 22–23회로 0.17 m까지 접근했다.
   - 조정자 가설 중 "대기 거리가 팔 도달 밖이고 가까이 가는 규칙이 없다"는 **틀렸다**. N7 접근에는 반경 0.168 m 초과 시 전진하는 wrist 전용 규칙이 있다.
   - 서보 3의 500 도달도 정상 동작이다(성공한 503도 34프레임). 이후 서보 4로 넘어간다.
   - 가설 중 **맞은 부분:** 진행이 없을 때의 시간 한도나 후퇴가 없다. 실패가 y 위치(조명 각도)와 연관돼 있다.
2. **운반 중 grasp drift (505):**
   - 가장 긴 운반(약 2.5 m, 86 SIM s)에서 상자가 손가락 안에서 천천히 밀렸다.
   - 자기 RGB anchor 중심이 N7 한계 25.6 px를 넘어 정지했다. 상자는 들린 상태(z 0.168)였다. probe에서 본 누적 미끄러짐과 같은 현상이다.
   - 자세를 바꿀 때 보이는 약 32° 기울기는 도구 pitch를 -90°에서 -30°로 바꿨기 때문이고, 모든 실행에서 같았다. 미끄러짐이 아니다.
3. **면 정렬:** 같은 조명 문제 때문에 RGB 면 적합 3회 연속을 얻지 못한 경우가 있었다. 손목 흔들기 8회 뒤 정적 지도 관례(동쪽을 보며 파지)와 자세 추정으로 만든 폴백 법선을 썼다(503, 505, 401). 출처는 결과에 기록했다.

### 다음 버전(v2) 후보: 새 시드 집합으로만 평가하고, v1 결과는 바꾸지 않음
- 접근: pan 재조준에 이력(hysteresis)을 두거나 최근 N개 새 프레임의 평균을 쓴다. 진행이 없으면 후퇴하고 다시 본다.
- 운반: carry 중 anchor를 주기적으로 다시 잡는다(co-motion 확인 후). 또는 경로를 짧게 끊고 정지 점검을 넣는다.
- 파지 깊이(GRASP_Z)가 가림 폭과 미끄러짐에 주는 영향을 측정한다.
- carry_p30 바닥 띠로 wrist 전용 전진 가드를 만든다.

## M1까지 남은 차단 요인
1. 자세 추정기 연결: PR #177의 파티클 필터를 `PoseEstimate` 인터페이스에 붙여야 한다. 지금은 stub이다.
2. 문 통과 정책: carry_p30 주행과 look_p20 정지 관찰, 문기둥 태그(`_tags_v2`).
3. 접근 교착과 운반 drift(v2).
4. 위치 추정 오차가 슬롯 배치(허용 ±6 cm)와 놓은 뒤 확인에 미치는 영향 측정.

## TensorBoard
- 스냅샷: `outputs/tensorboard/0925-zone-owncam-skill`(run 7개: v1-s501…s505, v1-dev401/402).
- 원본 result.json을 바꾸지 않으려고 `outputs/zone-owncam-skill-20260925/tensorboard-view/*/result.json`에 파생 파일을 만들었다. 원본 경로와 해시를 기록했고, `success`는 GT 슬롯 배치다.
- 보기 설정: `outputs/tensorboard-view.json`의 `zone_owncam_skill_v1_20260925` 키. 고정 카드는 reported_success, sim_s, commands, model_calls(0), wall_s다.
- 공용 서버(PID 9291, 다른 작업 소유)에서 run이 나오는 것과 값이 원본과 같은 것을 확인했다. 모델 응답 시간은 LLM이 없어 해당 없음이다.

## 원자료 (로컬 전용, 원격 백업 아님)
- 위치: 기본 체크아웃 `outputs/zone-owncam-skill-20260925/`. probe 2개, `cohort-d016c04/{501..505}`, `dev/*`, `cohort-d016c04/cohort.log`(시드별 부하 평균 포함).
- 경로·SHA-256·부하 평균·단계 시각은 `results.json`에 있다.
- 재생성: `python experiments/2026-09-25-zone-owncam-skill/build_results.py`.
