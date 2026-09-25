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

## v2 사전 등록 (`wrist_zone_skill_v2`, 시험 실행 전 커밋)
v1과 501–505 결과는 위 기록 그대로 둔다. v2는 새 모듈 `harness/wrist_zone_skill_v2.py`다. v1 파일은 바이트 그대로이며 테스트가 d016c04 해시로 확인한다.

**바꾼 것(v1 실패별)**
- 접근 교착(501/502/504):
  - 자기 RGB 추정치를 퓨전한다. 차체가 움직인 뒤의 최근 5프레임 중앙값을 쓰고, IoU 0.75 이상 적합을 우선한다.
  - pan과 손목 보정을 한 동작으로 합쳤다(pan 이력 30 PWM, 근거리 수직 허용 25 px).
  - 보정만 4회 이어지면 짧게 전진한다(0.08, 0.3 s).
  - 한 프레임 적합 실패는 퓨전 추정으로 최대 6프레임 메운다.
  - 40프레임 동안 1.5 cm 진행이 없으면 후퇴(자세 추정 기준 0.15 m)하고 새 스킬로 재접근한다(최대 2회).
- 운반 미끄러짐(505):
  - carry_p30을 유지한다.
  - 자기 RGB anchor 중심 이동 18 px 이상 또는 mask IoU 0.85 미만이면, 또는 N7 drift 정지가 나면 멈춘다. N7 release로 내려놓고 지면 정지를 확인한 뒤, 자세 추정 기준 0.18 m 물러나 wrist 접근으로 다시 잡는다(최대 2회).
  - 짐을 든 상태의 속도 상한을 0.08에서 0.12로 올렸다(빈 이동과 같음).
- 입력 경계는 v1과 같다. `pose_source=gt_stub_eval_only`, 문 통과 없음, **M1 아님**, weld OFF, `local_contact_fine`.

**개발 시드 403–406 (합산 금지)**
- v1 실패와 비슷한 y 위치(403·404)와 긴 운반(405·406)으로 조정했다.
- 조정 이력: fused-only 브리지 추가(404-a 접근 149 s → 404-b 51 s), 운반 속도 상한 추가(405-b 재장착 2회·370 s → 405-c 1회·271 s).
- 최종 조정본에서 403·404·405·406 모두 슬롯 배치했다. 단 403은 브리지 이전 버전에서의 결과다.
- 관찰: 재장착은 운반 속도와 관계없이 운반 시작 약 60 s 뒤에 발동했다. 시간에 따른 grip creep로 본다.

**시험 시드와 판정 (고정)**
- 시드 **511–520**, 각 1회. 시나리오는 `scripts/run_zone_owncam_skill.py::SCENARIOS`에 있으며 모두 분할벽 동쪽이다.
- 주 지표: **GT 슬롯 배치 성공 수/10**. 평가 전용 GT로, 상자가 주문 슬롯 ±0.06 m 안 바닥에 있고 기울기가 15° 미만이어야 한다.
- 보조 지표: GT 파지 성공 수, 스킬 자기 판정과 GT의 일치 수, 거짓 성공 수, SIM 시간, 명령 수, 재장착·후퇴 횟수, weld eq_active 최댓값(0이어야 함), 벽 접촉.
- 한도: SIM 420 s(v1은 300 s. 재장착·재접근 시간을 반영해 늘렸다), 제어 1300단계.

**중단 규칙**
- 시험 중에는 소스를 바꾸지 않는다(시험 커밋 SHA 고정, 깨끗한 트리). 코호트 도중 재조정이나 재실행은 하지 않는다.
- 첫 제어 단계 전 예외 같은 인프라 오류만 1회 재실행하고, 그 사실을 기록한다.
- 결과와 무관하게 10개를 모두 돌린다. 실패도 전부 보고한다.
- 동시 SIM은 2개 이하, 스레드는 1로 제한한다(OMP/OPENBLAS/VECLIB/MKL=1). 시드별 시작 부하 평균을 기록한다.

## v2 결과 (사전 등록 시드 511–520, 소스 `edd075d`, 깨끗한 트리)
- 조건: 동기 SIM, 스레드 1, 레인 2개. 시드별 시작 부하 평균은 12–71이다(`cohort-v2-edd075d/cohort.log`).
- 중간에 세션이 끊겼다가 재개했지만, 10개 모두 이미 끝나 있었다. **재실행은 없다.**

| seed | 슬롯 | 러너 판정 | GT 파지 | GT 슬롯 (오차 mm) | SIM s | 재장착 | 후퇴 |
|---|---|---|---|---|---|---|---|
| 511 | B3 | IN_SLOT | ✓ | ✓ (-15, -3) | 126.6 | 0 | 0 |
| 512 | A1 | IN_SLOT | ✓ | ✓ (-14, +3) | 123.1 | 0 | 0 |
| 513 | A3 | IN_SLOT | ✓ | ✓ (-6, -5) | 232.0 | 1 | 0 |
| 514 | C2 | IN_SLOT | ✓ | ✓ (-5, +3) | 212.5 | 1 | 0 |
| 515 | B1 | IN_SLOT | ✓ | ✓ (-6, -1) | 241.5 | 1 | 0 |
| 516 | C1 | IN_SLOT | ✓ | ✓ (-6, -5) | 202.6 | 1 | 0 |
| 517 | A2 | IN_SLOT | ✓ | ✓ (-3, -9) | 220.0 | 1 | 0 |
| 518 | B2 | NO_UNIQUE_CYAN_BOX | ✓ | ✓ (-4, +10) | 217.6 | 1 | 0 |
| 519 | C3 | CARRY_TOP_GEOMETRY_AMBIGUOUS_FOR_DROP | ✓ | ✗ (들고 있는 채 정지) | 180.3 | 1 | 0 |
| 520 | B3 | IN_SLOT | ✓ | ✓ (-6, +9) | 116.3 | 0 | 0 |

- **주 지표: GT 슬롯 배치 9/10.** (v1은 501–505에서 1/5였다. 시나리오가 달라 직접 비교가 아니라 참고용이다.)
- 보조 지표: GT 파지 10/10. 자기 판정과 GT 일치 9/10, 거짓 성공 0. 재장착은 7/10에서 1회씩, 후퇴는 0. weld eq_active 0, 벽 접촉 0.
- 접근 교착은 사라졌다. 강제 전진은 한 번도 필요 없었고, 퓨전과 합친 보정만으로 풀렸다. v1이 교착했던 y≈-2.7(511)과 y≈+1.1(512)에서도 성공했다.
- 518: 물리 배치는 성공했다. 하지만 구역 B 칠 위에서 놓은 뒤 다시 보기(`detect_own` own_zone_v2)가 두 시점 모두 0개를 검출했다. 보수적 **거짓 음성**이다.
- 519: 재장착 1회 뒤 슬롯 앞에서 회전하던 중, N7 운반 검사가 `TOP_GEOMETRY_AMBIGUOUS_FOR_DROP`로 끝냈다(상자 z 0.170, 들린 상태). v2는 `VISUAL_GRASP_DRIFT`와 anchor 경고에만 재장착하므로 이 보수적 정지는 종료로 처리됐다.
- v3 후보(새 시드 필요):
  - 이 정지 사유에서는 N7 grip 탐침(좌우 pan)을 실행한 뒤 재장착한다.
  - 구역 칠 위 다시 보기에서 청록 검출 범위를 보강한다.
  - grip creep 자체를 줄인다(파지 깊이·운반 자세).
- TensorBoard 새 스냅샷은 `outputs/tensorboard/0925-zone-owncam-skill-v2`(파생 보기 `outputs/zone-owncam-skill-20260925/tensorboard-view-v2/`, 공용 서버에서 run 19개와 값 일치 확인)다. v2 시험 10개, v2 개발 run, v1 기준 5개가 들어 있다. 보기 키는 `zone_owncam_skill_v2_20260926`.

## 운반 미끄러짐 원인 (2026-09-26, v3 이전 진단)
질문: v2에서 상자가 속도와 관계없이 약 60 s마다 손가락 사이로 미끄러졌다. 받아들여야 할 물리인가, 접촉·파지력 인공물인가?

**점검한 설정**
- 파지 명령: PWM 1500이다. `sim/masterpi_dynamics_v2.py`의 `closure=(2000-pwm)/500×0.016 m`에서 완전 닫힘 목표에 해당한다. 턱은 상자에 막혀 11.7 mm에서 멈추고, 목표와의 차이(4.3 mm)×kp 1200으로 턱당 5.19 N을 누른다(forcerange ±18 N 안).
- 손가락–상자 접촉 쌍: mu 3.4, condim 3, solref 0.013, `solreffriction (0, -6000)`(감쇠만 있는 마찰 기준). 상자는 30 g이다.
- 프로필: zone 단독 운반(`run_zone_owncam_skill.py`)은 `local_contact_fine`이다(`noslip_iterations 0`). 화물용 `cargo_noslip_v1`(`sim/zone_cargo_contact.py`, `local_contact_fine`+`noslip_iterations 10`, 해시 `2e003d85…`)은 `CargoZoneScene`에서만 명시적으로 선택된다. zone 교사, RGB 스킬, ACT에는 아직 적용된 적이 없다(zone-cargo-catalogue §8).
- 실물 MasterPi: 파지력·미끄러짐 보정은 `validated:false`이다. 실측 미끄러짐 자료는 없다.

**진단 probe** `scripts/probe_zone_box_grip_hold.py`(소스 `a4225bc`, 깨끗한 트리, GT 교사 파지, weld OFF, 평가 전용)
- 절차: carry 자세(v1/v2 `CARRY_POSTURE`)에서 180 s 정지 유지한 뒤 교사 사각 주행(약 63 s)을 한다.

| 프로필 | noslip | 정지 180 s 끝 변위 | 정지 creep | 주행 중 추가 변위 | 법선력 합 | 접선력 합 | 끝까지 들림 |
|---|---|---|---|---|---|---|---|
| `local_contact_fine` | 0 | 6.35 mm | **2.09 mm/min**(선형) | 2.10 mm | 10.38 N | 0.30 N | ✓ |
| `cargo_noslip_v1` | 10 | 0.05 mm | **0.016 mm/min** | 0.03 mm | 10.38 N | 0.30 N | ✓ |

- **판정: 수치 인공물이다. 받아들일 물리가 아니다.**
  - 무게를 버티는 데 필요한 마찰비는 0.30/10.38 = 0.029로, mu 3.4의 1% 미만이다. Coulomb 마찰이라면 완전히 붙어 있어야 한다.
  - 변위는 정지 중에도 시간에 비례해 선형으로 늘었고(주행 여부와 무관), 파지력과 접촉력이 같은 상태에서 noslip만 켜면 130배 줄었다. soft 마찰 제약의 감쇠-only 기준이 만드는 creep이다(cargo 카탈로그 §8의 0.5 kg 측정과 같은 기전).
  - v2의 "약 60 s마다 재장착"은 이 creep(60 s에 약 2 mm)이 근거리 anchor 경고(18 px)에 닿는 시간과 맞는다. 따라서 **v2의 재장착은 인공물을 가리는 행동이었다.** v2 결과표는 그대로 두되, 재장착 횟수는 물리적 행동 근거가 아니라고 해석을 고친다.
- 조치:
  - 기본값은 바꾸지 않는다. 러너에 `--contact-profile {local_contact_fine, cargo_noslip_v1}`을 추가했다. 기본은 `local_contact_fine`이고, 선택한 프로필·해시·실제 `noslip_iterations`를 result.json에 기록한다.
  - noslip은 전역 solver 옵션이다. 차체 주행과 정지 상자가 변하지 않는다는 근거는 cargo 카탈로그 기록이며, 이 스킬 장면에서의 영향은 v3 두 조건 비교로 확인한다.
- 원자료: `outputs/zone-owncam-skill-20260925/grip-hold/{local_contact_fine,cargo_noslip_v1}/`(samples.jsonl, result.json, hashes.json).

## v3 사전 등록 (`wrist_zone_skill_v3`, 시험 실행 전 커밋)
v1·v2 파일은 바이트 그대로다. 테스트가 d016c04·edd075d 해시로 확인한다. v3는 `harness/wrist_zone_skill_v3.py`다.

**바꾼 것(v2 실패별)**
- 519: N7은 `external_navigation` 과제에서 `TOP_GEOMETRY_AMBIGUOUS_FOR_DROP`를 "외부 계획기가 check_grip을 명시적으로 선택하라"는 인계로 쓴다. v2는 이것을 종료로 처리했다.
  - v3는 차체를 세우고, 현재 프레임을 anchor로 N7의 좌/우/홈 자기 카메라 부착 탐침(`carry_probe_*`)을 실행한다(최대 2회).
  - 통과하면 운반을 재개하고, 실패하면 N7 사유로 정직하게 종료한다.
- 518: 구역 B(파랑) 칠 위에서 조명받은 상자는 어둡고 채도가 낮은 청록이라 `detect_own(own_zone_v2)`가 놓친다.
  - v3는 놓은 뒤 확인에 N7 바닥 직육면체 적합(`observe_ground_box`, 채도 150, 위치 정밀화)을 먼저 쓴다. 바로 앞 release 검사가 쓰는 검출기와 같다. 실패하면 `detect_own`으로 넘어간다. 어느 검출기를 썼는지 기록한다.
  - 오프라인 확인(기록된 v2 프레임): 518 look-back 프레임은 적합 성공 (0.168, -0.009) m였다. 511/513/516에서는 `detect_own`과 1 cm 안에서 일치했다.
- 재장착 로직(v2)은 남겨 두되, 인공물 제거 조건에서는 발동하지 않을 것으로 예상한다. 발동하면 시각 오경보로 따로 보고한다.
- 입력 경계는 v1·v2와 같다. `pose_source=gt_stub_eval_only`, 문 통과 없음, **M1 아님**, weld OFF, #178 위치 추정기 미연결.

**개발 시드 (합산 금지)**: 407(북→구역 B 긴 운반), 518과 519 재실행(v2 시험 시드를 v3 개발용으로만 사용).

**v3 개발 기록** (소스 `5117fa8`; 뒤의 두 실행은 문서만 수정된 dirty 트리, 실행 코드 동일; `outputs/.../dev-v3/`)

| 실행 | 프로필 | 결과 | GT 슬롯 | SIM s | 재장착 | grip check | look-back 검출기 |
|---|---|---|---|---|---|---|---|
| 519-lcf | local_contact_fine | IN_SLOT | ✓ | 212.1 | 1 | 2회 모두 ATTACHED | N7 바닥 적합 |
| 518-ns | cargo_noslip_v1 | IN_SLOT | ✓ | 149.5 | 0 | 0 | N7 바닥 적합 |
| 519-ns | cargo_noslip_v1 | IN_SLOT | ✓ | 149.8 | 0 | 0 | N7 바닥 적합 |
| 407-ns | cargo_noslip_v1 | IN_SLOT | ✓ | 175.6 | 0 | 0 | N7 바닥 적합 |

- v2가 끝내지 못한 두 경우(519 운반 중 정지, 518 거짓 음성)가 v3에서 모두 풀렸다. 개발 중 추가 조정은 없었다.
- noslip 조건에서는 anchor 경고도 재장착도 0이었다. v2의 재장착이 creep 때문이었다는 판정과 맞는다.

**시험 시드와 조건 (고정)**
- 시드 **521–530**, 조건별 각 1회. 시나리오는 `SCENARIOS`에 있다. 모두 분할벽 동쪽이고, 운반 거리는 1.05–3.23 m, 구역 B 목적지는 4개다.
- **주 조건 P**: `--profile v3 --contact-profile cargo_noslip_v1`(명시).
- **보조 조건 S**: `--profile v3 --contact-profile local_contact_fine`. 스킬 수정 효과와 프로필 효과를 분리하고, 인공물이 행동에 주는 비용(재장착 수·시간)을 잰다.
- 주 지표: 조건 P의 **GT 슬롯 배치 수/10**. 평가 전용 GT로, 주문 슬롯 ±0.06 m 안 바닥에 있고 기울기가 15° 미만이어야 한다.

**판정 기준 (조건 P)**
- G1 GT 슬롯 배치 ≥ 9/10.
- G2 자기 판정과 GT 일치 ≥ 9/10, 거짓 IN_SLOT 0.
- G3 weld eq_active 0, 벽 접촉 0, 모든 단계의 `pose_source=gt_stub_eval_only`.
- G4 GT 배치 성공인데 `NO_UNIQUE_CYAN_BOX`로 끝난 경우 0, 들고 있는 채 `CARRY_TOP_GEOMETRY_AMBIGUOUS_FOR_DROP`로 끝난 경우 0.
- 보고 항목: 재장착 수(예상 0), grip check 수와 결과, SIM 시간, 명령 수, 검출기별 look-back 수. 조건 S는 같은 항목으로 보고하고 게이트는 적용하지 않는다(비교용).
- 한도: SIM 420 s, 제어 1300단계(v2와 같음).

**중단 규칙**
- 시험 커밋 SHA 고정, 깨끗한 트리. 코호트 도중 소스 변경·재조정·재실행은 하지 않는다. 첫 제어 단계 전 인프라 예외만 1회 재실행하고 기록한다.
- 조건 P 10개를 먼저 돌린 뒤 조건 S 10개를 돌린다. **거짓 IN_SLOT이 한 번이라도 나오면 즉시 코호트를 멈추고 보고한다.** 그 밖에는 결과와 무관하게 모두 돌린다.
- 동시 SIM은 2개 이하, 스레드는 1(OMP/OPENBLAS/VECLIB/MKL=1)로 제한한다. 시드별 부하 평균을 기록한다.

## v3 결과 (사전 등록 시드 521–530, 소스 `7322a96`, 깨끗한 트리)
- 조건: 동기 SIM, 스레드 1, 동시 SIM 2개 이하. 시드별 시작 부하 평균은 9–37이다(`cohort-v3-7322a96/cohort.log`, sha256 `3ba75790…`).
- **인프라 사건(기록):** 코호트 도중 실행과 무관한 파일(`build_tensorboard_v3.py`)을 worktree에 추가해 트리가 dirty해졌다. 그 결과 조건 P의 523/525/527/529 실행기가 **제어 단계 전에 실행을 거부했다**(출력 없음, 로그는 `P/<seed>.infra-dirty-refusal.log`로 보존).
  - 파일을 밖으로 옮기고, 내 드라이버 셸만 멈췄다(실행 중이던 522는 유지).
  - 같은 인자로 이어 가는 드라이버(`run_v3_cohort_continue.sh`)로 사전 등록 규칙에 따라 1회 재실행했다. 그 밖의 재실행은 없다.

| seed | 슬롯 | P 판정 (cargo_noslip_v1) | P GT·SIM s | P 재장착 | P grip check | S 판정 (local_contact_fine) | S GT·SIM s | S 재장착 | S grip check |
|---|---|---|---|---|---|---|---|---|---|
| 521 | B2 | IN_SLOT | ✓ 114.8 | 0 | 0 | IN_SLOT | ✓ 114.6 | 0 | 0 |
| 522 | B3 | IN_SLOT | ✓ 167.4 | 0 | 0 | IN_SLOT | ✓ 225.5 | 1 | 0 |
| 523 | C1 | IN_SLOT | ✓ 205.5 | 0 | 1 | IN_SLOT | ✓ 265.6 | 1 | 1 |
| 524 | A3 | IN_SLOT | ✓ 169.5 | 0 | 0 | IN_SLOT | ✓ 230.3 | 1 | 0 |
| 525 | C2 | IN_SLOT | ✓ 155.8 | 0 | 0 | IN_SLOT | ✓ 212.4 | 1 | 0 |
| 526 | B1 | IN_SLOT | ✓ 163.6 | 0 | 0 | IN_SLOT | ✓ 261.7 | 1 | 0 |
| 527 | A1 | IN_SLOT | ✓ 152.8 | 0 | 0 | IN_SLOT | ✓ 213.8 | 1 | 0 |
| 528 | A2 | IN_SLOT | ✓ 107.7 | 0 | 0 | IN_SLOT | ✓ 107.6 | 0 | 0 |
| 529 | C3 | CARRY_TOP_GEOMETRY_AMBIGUOUS_FOR_DROP | ✗ 118.0 | 0 | 2 | IN_SLOT | ✓ 217.8 | 1 | 0 |
| 530 | B2 | IN_SLOT | ✓ 162.9 | 0 | 0 | IN_SLOT | ✓ 227.6 | 1 | 0 |

**게이트 판정 (조건 P)**
- G1 GT 슬롯 배치 **9/10** → 통과.
- G2 자기 판정과 GT 일치 **10/10**, 거짓 IN_SLOT 0 → 통과.
- G3 weld 0, 벽 접촉 0, `pose_source`는 모두 `gt_stub_eval_only` → 통과.
- G4 **실패(1건, 529).** 들고 있는 채 `CARRY_TOP_GEOMETRY_AMBIGUOUS_FOR_DROP`로 끝났다(상자 z 0.171 m).
- 다시 보기는 20개 실행 모두 N7 바닥 적합으로 확정됐고, 구역 B 목적지 8회 중 거짓 음성은 0이다(518 유형 해결).

**조건 비교 (보조)**
- S: 10/10 배치, 일치 10/10, 거짓 성공 0. 재장착은 8/10에서 1회씩 일어났다. P는 재장착 0, anchor 경고 0.
- SIM 중앙값: P 159.4 s, S 221.7 s. 제어 단계 중앙값: P 329, S 442.5.
- P가 성공한 9개 시드에서 쌍별 S−P 차이의 중앙값은 **+60.0 s**다. 재장착 1회의 비용이며, 재장착이 없던 521/528은 ±0.2 s다. creep 인공물을 없애면 같은 스킬이 약 60 s 빨라진다.

**529 분석 (v4 후보, 이번 결과에는 반영하지 않음)**
- C3 앞으로 가려고 구역 C 칠 위를 북쪽으로 지날 때, N7의 낮은 표면 높이 인계가 약 4 s 간격으로 세 번 났다(stale surface-height 목표 z<0.035). 앞의 두 grip check는 모두 ATTACHED였고, 세 번째에서 상한 2회에 걸려 종료했다.
- 523(C1)에서도 1회 발생 뒤 통과했다. v2의 519도 같은 C3 목적지에서 났다. 구역 C 칠 위 운반 중의 체계적인 시각 오경보다.
- S의 529는 재장착으로 경로와 시각이 달라져 성공했다(우연적 차이이며 조건 효과로 해석하지 않는다).
- 후보: grip check를 통과한 뒤에는 같은 운반 구간에서 이 인계를 이동 거리 기준으로 억제하거나, 통과 뒤 stale 목표 높이를 갱신한다. 새 버전과 새 시드로만 평가한다.

## 구역 C 운반 중 오경보 원인 (2026-09-26, v4 이전 진단)
대상: v3 P/529(3회, 마지막에 종료), P/523(1회), S/523(1회), v2 519(1회). 모두 상자를 든 채 N7 `TOP_GEOMETRY_AMBIGUOUS_FOR_DROP` 인계가 난 경우다.

- **구역 C 칠 색이 원인이 아니다.** 기록된 오경보 프레임에는 칠이 보이지 않는다. 화면 아래 54%를 든 상자의 면이 채운다.
- 기전(기록 프레임 재생):
  - N7은 운반 중 매 프레임 `visual_box_surface.observe_known_box_top`을 부른다. 윗면 마스크는 명도 152–220이다.
  - 이 검출기는 밝은 **띠**도 윗면으로 받아들이고, 높이는 두 긴 변을 상자의 긴 변 40 mm로 가정해서만 계산한다.
  - 오경보 위치(모두 구역 C 위나 그 옆)에서는 조명 때문에 든 상자 면의 명도가 147–153으로 문턱 152에 걸친다(정상 운반 프레임은 100–141).
  - 그러면 3 px 두께의 띠(62×3 px, 면적 186 px, 최소 180 px 통과)가 생긴다. 이것을 40 mm 변으로 읽으면 먼 바닥 높이(z −0.02…0.01 m)로 계산되고, 신뢰도는 0.93–0.98이다. 실제 상자는 GT z 0.171 m로 들린 상태였다.
- 전수 점검(v3 코호트 20개의 운반 프레임 3179장):
  - 낮은 윗면 검출은 7건이었다.
  - 아래 v4 판정으로 6건은 든 상자 실루엣 안(자기 가림)으로 분류됐다.
  - 1건(P/523 step 309)은 띠가 실루엣 가장자리 5 px 여유 안에 걸쳐 "밖"으로 분류됐다. 이 경우 v4도 v3처럼 grip check를 실행한다(보수적).

## v4 사전 등록 (`wrist_zone_skill_v4`, 시험 실행 전 커밋)
v1·v2·v3 파일은 바이트 그대로다. 테스트가 해시로 고정한다.

**바꾼 것**
- N7의 낮은 윗면 인계에서만 판정을 추가한다. 보고된 윗면 사각형의 네 꼭짓점이 현재 자기 프레임의 **든 상자 실루엣** 안에 있으면 carry로 복귀한다.
  - 실루엣: 청록 색상 성분 중 화면 아래 40 px 안에 닿는 가장 큰 것을 구멍 채움하고 5 px 침식한다.
  - 이때 실루엣 면적은 운반 anchor 실루엣의 0.8–1.25배여야 한다(든 상자가 그대로 있어야 한다).
  - 근거: 바닥 물체는 든 상자를 통과해 보일 수 없다.
- 실루엣이 없거나(실제 낙하), 면적이 달라졌거나, 사각형이 밖에 있으면 v3 경로(정지 후 N7 좌/우/홈 부착 탐침, 최대 2회)를 그대로 쓴다.
- 운반 매 프레임 N7 동시 움직임(comotion) 낙하 검사는 윗면 검사보다 먼저 실행되며 바꾸지 않았다. `VISUAL_GRASP_DRIFT`, anchor 경고 재장착도 그대로다.
- 입력 경계는 이전과 같다. `pose_source=gt_stub_eval_only`, 문 통과 없음, **M1 아님**, weld OFF.

**실제 낙하를 여전히 잡는다는 증거**
- 단위 테스트(`tests/test_wrist_zone_skill_v4.py`, 기록 프레임 fixture):
  - 529·523 오경보 프레임은 자기 가림으로 판정된다.
  - 실제 낙하 프레임(아래 dev 408 fault injection의 첫 프레임, GT z 0.016 m)은 실루엣 소실(`HELD_SILHOUETTE_CHANGED`)로 **거부되지 않는다.**
  - 합성 바닥 상자 프레임, 실루엣 밖 사각형, anchor 없음은 모두 거부되지 않는다.
  - N7 낙하 사유는 그대로 종료된다.
- 러너 `--inject-drop-after-carry-s`: 평가용 고장 주입이다. 운반 시작 N SIM s 뒤 든 상자를 집게 아래 바닥으로 옮기고, 스킬에는 알리지 않는다. 결과에 `fault_injection`과 `counts_for=drop_safety_only`로 기록된다.

**v4 개발 기록** (소스 `a8b9ee2`, `outputs/.../dev-v4/`, 합산 금지)
- 529 재실행(P): 자기 가림 판정 1회, grip check 0회, **IN_SLOT**, GT 배치 ✓, 155.2 s. v3에서는 여기서 종료했다.
- 408 + 낙하 주입(P, 운반 15 s 뒤): 주입 직후 첫 프레임에서 `CARRY_VISUAL_LOAD_DROPPED_OR_OCCLUDED`로 종료했다. IN_SLOT 주장 없음, 상자는 바닥(z 0.016 m).
- 조정: 실루엣이 fisheye 검은 여백 때문에 맨 아래 행에 닿지 않아, "아래 40 px 안"으로 바꿨다(개발 중 1회).

**시험 시드와 조건 (고정)**
- 시드 **531–540**. 이 중 **구역 C 목적지는 531·532·533·534·538·540(6개)**다. 나머지는 A2·B2·A3·B3이다. 운반 거리는 1.39–3.5 m다.
- 조건 **P**: `--profile v4 --contact-profile cargo_noslip_v1`, 시드당 1회.
- 조건 **D (낙하 안전)**: 531과 533에 P와 같은 설정으로 `--inject-drop-after-carry-s 15`를 준다. 배치 지표에는 넣지 않는다.
- 조건 S(`local_contact_fine`)는 이번에 **돌리지 않는다.** 호스트 부하가 높고, v3에서 S−P 비용(+60 s, 재장착)을 이미 측정했다.
- 한도: SIM 420 s, 제어 1300단계.

**판정 기준**
- G1 P GT 슬롯 배치 ≥ 9/10이고, 구역 C 목적지 ≥ 5/6.
- G2 P 자기 판정과 GT 일치 ≥ 9/10, 거짓 IN_SLOT 0.
- G3 weld 0, 벽 접촉 0, 전부 `gt_stub_eval_only`.
- G4 P에서 상자를 든 채(GT z > 0.08) `CARRY_TOP_GEOMETRY_AMBIGUOUS_FOR_DROP` 또는 `GRIP_CHECK_*`로 끝난 경우 0. GT 배치 성공인데 `NO_UNIQUE_CYAN_BOX`인 경우 0.
- G5 D 두 실행 모두 IN_SLOT 주장 없이 낙하·grip 실패 사유로 끝나고, 주입 뒤 5 제어 단계 안에 종료한다.
- 보고 항목: 자기 가림 판정 수, grip check 수와 결과, SIM 시간, 명령 수.

**중단 규칙**
- 시험 커밋 SHA 고정, 깨끗한 트리. 코호트 동안 worktree에 파일을 만들지 않는다(v3 인프라 사건 재발 방지).
- 순서는 D 2개를 먼저 돌린 뒤 P 10개다. **어느 실행이든 거짓 IN_SLOT이 나오면 즉시 멈춘다.**
- 첫 제어 단계 전 인프라 예외만 1회 재실행하고 기록한다.
- 동시 SIM 2개 이하, 스레드 1. 시드별 부하 평균을 기록한다.

## v4 결과 (사전 등록 시드 531–540, 소스 `6664425`, 깨끗한 트리, 실행 코드 = dev `a8b9ee2`)
- 조건: 동기 SIM, 스레드 1, 동시 SIM 2개. 시드별 시작 부하 평균은 7–35다(`cohort-v4-6664425/cohort.log`, sha256 `f86f7633…`). 인프라 사건과 재실행은 없다.

| seed | 목적지 | P 판정 | GT 슬롯 (오차 mm) | SIM s | 자기 가림 판정 | grip check |
|---|---|---|---|---|---|---|
| 531 | **C3** | IN_SLOT | ✓ (-4, +3) | 158.5 | 0 | 0 |
| 532 | **C1** | IN_SLOT | ✓ (-3, -2) | 162.5 | 0 | 0 |
| 533 | **C2** | IN_SLOT | ✓ (-4, 0) | 151.2 | 0 | 0 |
| 534 | **C3** | IN_SLOT | ✓ (-3, +2) | 201.6 | 0 | 0 |
| 535 | A2 | IN_SLOT | ✓ (-4, -1) | 179.8 | 0 | 0 |
| 536 | B2 | IN_SLOT | ✓ (-5, +11) | 160.4 | 0 | 0 |
| 537 | A3 | IN_SLOT | ✓ (-4, -11) | 128.1 | 0 | 0 |
| 538 | **C2** | IN_SLOT | ✓ (-3, +2) | 169.4 | 0 | 0 |
| 539 | B3 | IN_SLOT | ✓ (-5, +11) | 145.2 | 0 | 0 |
| 540 | **C1** | IN_SLOT | ✓ (-3, +1) | 155.6 | 1 | 0 |

**D (낙하 안전, 운반 15 s 뒤 낙하 주입)**
- 531: 주입 80.03 s, 같은 SIM 시각 첫 제어 단계에서 `CARRY_VISUAL_LOAD_DROPPED_OR_OCCLUDED`로 종료했다. IN_SLOT 주장 없음, 상자 z 0.016 m.
- 533: 주입 78.59 s, 첫 제어 단계에서 같은 사유로 종료했다. IN_SLOT 주장 없음, 상자 z 0.016 m.

**게이트 판정: G1–G5 모두 통과**
- G1 P 10/10, 구역 C 목적지 6/6.
- G2 일치 10/10, 거짓 IN_SLOT 0.
- G3 weld 0, 벽 접촉 0, 모두 `gt_stub_eval_only`.
- G4 들고 있는 채 정지 0, 다시 보기 거짓 음성 0.
- G5 D 2/2가 첫 단계에서 낙하를 검출했다.

**보조 지표**
- 재장착 0, grip check 0. SIM 중앙값 159.5 s, 제어 단계 중앙값 337.5.
- **해석 한계:**
  - 시험 코호트에서 이 오경보는 1회(540, 구역 C1 목적지)만 나서 v4 경로가 실제 운반 중 작동한 표본이 적다. dev 529 재실행 1회, 기록 프레임 7건 전수 점검(6건 거부), 단위 테스트가 추가 근거다.
  - 낙하 주입은 순간 이동 방식이며, 검출은 v4 경로가 아니라 N7 동시 움직임 검사가 했다(v4가 그 검사를 약화하지 않았다는 증거).
  - v3 P(521–530)와는 시나리오가 달라 직접 비교가 아니라 참고용이다.
- TensorBoard: 새 run 그룹 `outputs/tensorboard/0926-zone-owncam-skill-v4`(run 24개: v4P 10, v4D 2, v4dev 2, v3P 기준 10). 보기 키는 `zone_owncam_skill_v4_20260926`이고, 생성은 `build_tensorboard_v4.py`로 한다. 공용 서버에서 run 24개와 스칼라 값이 원본과 같은 것을 확인했다.

## Codex 반대 검토 반영 (v5, 2026-09-26)
원문: `docs/design/2026-09-26-owncam-adversarial-review-codex.md`(Kiro draft PR #196). 검토의 줄 번호는 v1/v2 기준이다. v3/v4에 같은 문제가 남았는지 다시 확인했다.

| # | 지적 | v3/v4 상태 | v5 조치 |
|---|---|---|---|
| 1 Blocker | GT pose가 성공 판정까지 간다. 파생 결과에 `success:true`만 남는다. | 남음. 러너가 `GtStubPoseSource`를 쓰고, v3/v4 파생 보기 `success`가 GT 슬롯 배치다. | `harness/m1_contract.py`: `diagnostic_success`와 `m1_success`를 필수 별도 필드로 둔다. `counts_as_m1`·`pose_source`·`input_contract`는 파생 결과까지 필수다. M1 모드는 실행기·판정기·exporter가 GT 출처를 거부한다(허용 목록 `own_rgb_`/`owncam_pf`). 파생 `success`는 `m1_success`다. |
| 2 High | look_back이 camera/robot/frame/hash 검증을 우회한다. | 남음. v3 `confirm_placement`는 검증 없이 `observe_ground_box`를 부른다. | 모든 단계의 관측이 N7 `_validate_observation`(전용 gate 인스턴스)을 먼저 통과해야 한다. 거부되면 `OBSERVATION_REJECTED`로 끝나고 판정은 없다. 음성 테스트: look_back에서 nav_cam·cctv_top·cctv_top_east, 해시 불일치, 다른 로봇, 오래된 프레임. |
| 3 High | 정확한 `pickup_xy`가 주문서에 있다. 면 추정 실패 시 동쪽 관례로 `ready=True`가 된다. | 남음. **v4 P 코호트 10건 중 9건이 지도 fallback 법선을 썼다.** | `CoarseOrderSheet`(0.5 m bay)를 쓰고, 상자 위치와 yaw는 자기 RGB로 얻는다. fallback을 제거하고 inlier 투표 면 추정 + mecanum 면 정렬을 쓴다. 실패 시 재관측 2회 후 중단한다. |
| 4 Medium | `PoseEstimate`에 공분산·신선도가 없다. | 남음. | #181에서 PoseReport·σ 게이트에 합의했다. 추정기가 들어오면 다음 버전에서 구현한다. |
| 5 Medium | 접촉이 macro 단계 표본이다. r1–벽만 센다. | 남음. | 러너 v5가 **모든 physics step**에서 화물–벽, r1–벽, r1–화물, 손가락–화물 접촉을 기록한다(최소 거리, `mj_contactForce` 법선력, 벽 접촉 구간). 평가 전용이다. |
| 6 Medium | `source_sha`가 종료 시점 HEAD다. | 남음. | 시작 시점 HEAD, dirty, 의존 파일 21개의 SHA-256을 기록하고 종료 시 다시 비교한다(`source_changed_during_run`). |
| 7 | 사전 등록 시점 표현이 모호하다. | — | 아래에서 "개발 데이터 수집(dirty, 합산 금지)"과 "시험 고정 커밋"을 구분해 쓴다. |
| 8 Medium | 집계기가 없는 seed를 조용히 뺀다. | 남음. | `build_results.py`가 등록된 seed 목록 전체와 대조한다. 없으면 `missing/infrastructure_failure`로 기록하고 보고서를 막는다(v1–v4 코호트는 누락 0 확인). |
| 9 | loc README의 timestep | 이 PR 파일이 아니다. | 해당 없음. |

**면 법선 원인 (v4 P/531 재검토)**
- N7 규칙은 "IoU ≥ 0.70인 적합이 연속 3개, 서로 10° 이내"다.
- 실제 적합은 대체로 1°/88°(mod 90으로 ±2°)로 일관된다. 그런데 16–22°(IoU 0.73–0.85) 이상치가 한 번 끼면 연속 조건이 깨진다.
- v1은 unready 횟수를 누적으로 세서, 8회를 넘으면 지도 관례로 넘어갔다.

**v5 변경 (`wrist_zone_skill_v5`, 러너 `zone_owncam_runner_v5`, v1–v4 바이트 고정)**
- `RobustFaceAligner`
  - 마지막 섀시 이동 이후 IoU ≥ 0.70 적합 최대 6개를 창에 둔다.
  - 한 적합 기준 7°(mod 90) 안에 ≥ 3개이고 창의 ≥ 60 %이면 준비 완료다. 추정값은 그 적합들의 원형 평균이다.
- 면 정렬(dev 409 근거)
  - N7의 웨이포인트(법선 0.35 m) 접근은 20–30° 면에서 옆으로 약 0.2 m를 가야 한다. 그 전에 v2 무진전 후퇴가 2회 발생했다.
  - v5 순서는 다음과 같다.
    1. 제자리 회전: yaw 오차 ≤ 4°. 회전 뒤마다 창을 비우고 다시 추정한다.
    2. 횡이동: |y| ≤ 1 cm. 이동해도 yaw 창은 유지한다.
    3. N7 직진 접근.
  - 회전 12회 또는 횡이동 40회 안에 수렴하지 않으면 재관측한다.
- bay 접근: bay 서쪽 가장자리에서 0.25 m 떨어진 지점까지 정적 지도 경로로 간다. 이때 bay 전체를 장애물로 둔다. 그 뒤 N7 탐색과 접근을 한다.

## v5 사전 등록 (시험 고정 커밋 전에 작성, 개발 데이터 수집과 분리)
**개발 데이터 수집** (dev 시드 409–411, `outputs/.../dev-v5/`, 합산 금지)
- 409-a: dirty, 첫 v5다. 면 추정 준비는 됐으나 N7 웨이포인트 접근에서 v2 무진전 후퇴 2회 → `GRASP_APPROACH_NO_PROGRESS`. 이것이 mecanum 면 정렬을 도입한 근거다.
- 409-b, 410-a: dirty. 면 정렬 회전은 수렴했다(409-b: 13°→2°, 참값 1°). 횡이동은 0.06 × 0.3 s로 약 3 mm뿐이었고, 매번 yaw 창을 비워서 16회 한도에 걸렸다. 재관측 2회 뒤 `GRASP_FACE_UNOBSERVABLE_AFTER_RELOOKS`로 **정직하게 중단**했다(fallback 없음, IN_SLOT 주장 없음).
- 409-c, 410-b: dirty. 횡이동 뒤 yaw 창 유지, 최대 0.08 × 1.0 s, 한도 회전 12 / 횡이동 40으로 바꿨다. 둘 다 IN_SLOT, GT 슬롯 ✓, 139 / 143 s. 409-c: 회전 6 + 횡이동 8, 정렬 뒤 yaw 0.18°, 슬롯 오차 (-10, -8) mm. 벽 접촉 0.
- 411(깨끗한 dev 커밋 `25af375`, 구역 C3): IN_SLOT, GT 슬롯 ✓, 170.7 s, 벽 접촉 0.

**시험 시드와 조건 (고정)**
- 시드 **541–548**(8개). bay E1–E4에 각 2개다. bay 안 상자 오프셋은 ±0.14 m, **상자 yaw는 −28°…+27°**다. 목적지는 A1·A2·A3·B1·B2·B3·C1·C2를 섞었다(구역 C 2개).
  - 상자 위치와 yaw는 setup 전용이며 스킬에는 주지 않는다. 표는 `scripts/run_zone_owncam_skill_v5.py`의 `SCENARIOS`에 있다.
- 조건 **P**: 러너 v5 `--mode diagnostic --contact-profile cargo_noslip_v1`, 시드당 1회. `cargo_noslip_v1`은 사용자 결정 대기 중인 주 조건이다.
- 한도: SIM 420 s, 제어 900단계. pose는 `gt_stub_eval_only`이므로 **M1 아님**. M1 모드는 추정기가 없어 실행을 거부한다(테스트로 확인).

**판정 기준**
- G1 `diagnostic_success`(GT 슬롯 배치 + 스킬 IN_SLOT 주장) ≥ 6/8. v4 9/10보다 낮춘 이유: bay 탐색과 회전된 상자를 처음 시험한다.
- G2 거짓 IN_SLOT 0, 스킬 판정과 GT 일치 ≥ 7/8.
- G3 모든 파지의 면 법선이 자기 RGB inlier 투표에서 나온다(`face_evidence` 존재). 지도 법선 사용 0(구조상 불가, 결과로 확인). 면 실패는 재관측 또는 중단으로만 끝난다.
- G4 계약: 모든 결과가 `validate_outcome`을 통과한다. `counts_as_m1 = m1_success = false`, `cameras_seen = [robot_cam]`, 관측 거부 0, `source_changed_during_run = false`, 시작 시점 트리가 깨끗하다.
- G5 weld 0. physics step 단위 화물–벽·r1–벽 접촉을 보고한다(목표 0, 발생하면 힘·거리와 함께 적는다).
- G6 등록 시드 8개가 모두 있다. 없으면 `missing/infrastructure_failure`로 보고가 막힌다.
- 보고 항목: 재관측 수, 면 정렬 회전·횡이동 수, SIM 시간, 제어 단계 수, 벽 접촉 힘.

**중단 규칙**
- 시험 커밋 SHA 고정, 깨끗한 트리. 코호트 동안 worktree에 파일을 만들지 않는다.
- 거짓 IN_SLOT이 나오면 즉시 멈춘다. 첫 제어 단계 전 인프라 예외만 1회 재실행하고 기록한다.
- 동시 SIM 2개, 스레드 1. 시드별 부하 평균은 `cohort.log`에 남긴다. 코호트 뒤 조정은 새 버전으로만 한다.

## v5 결과 (사전 등록 시드 541–548, 소스 `4884226`, 깨끗한 트리, 진단 모드 — **M1 아님**)
- 조건: 러너 v5, `cargo_noslip_v1`(사용자 결정 대기), 동기 SIM, 스레드 1, 동시 SIM 2개.
- 시드별 시작 부하 평균은 7.7–21.7이다(`cohort-v5-4884226/cohort.log`, sha256 `79ce4a7b…`). 인프라 사건과 재실행은 없다.

| seed | bay / 상자 yaw | 목적지 | 판정 | GT 슬롯 (오차 mm) | 회전/횡이동 | 재관측 | SIM s |
|---|---|---|---|---|---|---|---|
| 541 | E1 / −18° | B3 | IN_SLOT | ✓ (−13, −4) | 5/8 | 0 | 137.8 |
| 542 | E4 / +27° | A2 | IN_SLOT | ✓ (−17, +6) | 9/9 | 0 | 134.6 |
| 543 | E2 / −8° | **C1** | IN_SLOT | ✓ (−4, −6) | 0/3 | 0 | 137.4 |
| 544 | E3 / +22° | B1 | IN_SLOT | ✓ (−6, −1) | 5/5 | 0 | 190.0 |
| 545 | E1 / +5° | A3 | IN_SLOT | ✓ (−4, +3) | 9/7 | 0 | 192.5 |
| 546 | E4 / −28° | **C2** | IN_SLOT | ✓ (−2, −3) | 10/9 | 0 | 191.2 |
| 547 | E2 / +16° | B2 | IN_SLOT | ✓ (−7, +13) | 5/4 | 0 | 138.7 |
| 548 | E3 / −22° | A1 | IN_SLOT | ✓ (−14, +3) | 1/3 | **1** | 160.1 |

**게이트 판정: G1–G6 모두 통과**
- G1 `diagnostic_success` 8/8(등록 기준 ≥ 6/8).
- G2 거짓 IN_SLOT 0, 판정 일치 8/8.
- G3 8건 모두 `own_rgb_markerless_face_inlier_vote`로 면 법선을 얻었다. 지도 법선 0.
  - 548은 첫 시도에서 회전 12회 뒤에도 준비가 안 돼 `GRASP_BOX_FACE_ALIGNMENT_UNOBSERVABLE` → 재관측 1회 → 성공했다.
  - 정렬 종료 시 자기 추정 yaw 오차는 |≤ 3.2°|, 횡오차 ≤ 9.5 mm다.
- G4 8건 모두 `validate_outcome` 통과. `counts_as_m1 = m1_success = false`, `cameras_seen = [robot_cam]`, 관측 거부 0, 시작 트리 깨끗, 실행 중 소스 변경 없음.
- G5 weld 0. physics step 단위 화물–벽·r1–벽 접촉 0단계. 손가락–화물 최대 법선력 2.2–3.0 N.
- G6 등록 시드 8/8, 누락 0.

**보조 지표와 해석 한계**
- SIM 중앙값 149.4 s, 제어 단계(명령) 중앙값 310.
- v4 P(531–540, 정확한 좌표, yaw 0, 중앙값 159.5 s)와는 조건이 달라 직접 비교가 아니다. 그래도 면 정렬을 추가했는데 시간은 늘지 않았다.
- **이것은 여전히 GT pose stub 진단이다.**
  - 이동 목표·preplace·look-back 지도 변환은 GT pose를 쓴다.
  - 자기 RGB로 바뀐 것은 상자 위치·방향(bay 안 탐색, 면 정렬)과 모든 관측 검증이다.
  - M1은 PoseReport 추정기(#178/#197)를 붙이고 `--mode m1`로 돌려야 한다. 지금은 추정기가 없어 M1 실행이 거부된다.
- 문 통과 없음. 표본은 8개다.

## v6: 정면 상자 면 추정, 공개 re-anchor hook, 정적 keep-out (2026-09-26, M1 #201 요청)
**배경.** M1 dev-a2(#201, `32c7069`)의 s91·s92가 `GRASP_FACE_UNOBSERVABLE_AFTER_RELOOKS`로 끝났다. 축 정렬(yaw 0) 상자를 한 면만 정면으로 보면 N7 바닥 cuboid yaw 적합의 35–53%가 7° 넘게 틀린다. 그래서 v5의 투표(7° 안 60%)가 준비 상태가 되지 않았다. v5 코호트는 yaw 5–28°만 시험했다. s91에서는 v5 bay 접근점이 쉬는 상대 로봇의 spawn 옆에 있어 로봇끼리 226 step 접촉했다.

**변경 (`harness/wrist_zone_skill_v6.py`, 러너 `scripts/run_zone_owncam_skill_v6.py`; v1–v5와 러너 v5는 바이트 그대로 유지, 테스트로 고정)**
- (a) **윗면 먼 모서리 yaw.**
  - 현재 자기 robot_cam 프레임에서 목표 cyan 성분의 열마다 맨 위 픽셀을 모은다.
  - 이 픽셀을 fisheye 보정 → 자기 PWM 카메라 자세 → 윗면 평면(z = 0.032 m)으로 투영한다.
  - RANSAC 직선의 방향을 yaw(mod 90)로 쓴다. 먼 윗모서리는 yaw와 무관하게 바닥 좌표에서 직선이므로 0°에 특수 처리가 없다.
  - 모서리가 영상·렌즈 비네트에 잘리면(열의 25% 초과) 그 프레임은 거부한다.
  - 투표 규칙은 v5와 같다(창 6, ≥ 3개이면서 ≥ 60%, 허용 5°). 면 선택도 v5와 같고 지도 fallback은 없다. N7 cuboid yaw는 증거로만 기록한다.
- (b) **공개 hook** `reanchor_after_probe(home_before, left, right, home_after)`. 박스 스킬과 delivery 둘 다에 있다.
  - 네 프레임 모두 N7 엄격 검사를 통과해야 한다. `home_before`만 직전에 검사한 프레임이어도 된다.
  - pan 차이는 호출자가 아니라 **각 프레임의 자기 PWM**에서 읽는다. 왼쪽 +40…+120, 오른쪽 −40…−120, home ±5이고 팔 서보는 불변이어야 한다.
  - 증거는 N7 lift 뒤 probe와 같은 `previous_endpoint` 방식이다: home→left, home→right, left→right, right→home이 모두 함께 움직여야 한다.
  - 통과할 때만 운반 anchor와 직전 프레임을 `home_after`로 바꾼다.
- (c) **정적 keep-out.** `StaticKeepout`은 정적 배치의 고정 원판(쉬는 로봇 spawn, 주차 자리)만 받는다. 실시간 상대 자세는 받지 않는다.
  - `approach_point()`는 v5 접근점과 bay까지 직선 접근이 모든 원판에서 로봇 반경 0.17 + 여유 0.05 m 떨어지면 v5 점을 그대로 쓴다. 그래서 A2(bay half 0.15)와 호환된다.
  - 아니면 짧은 standoff → 측면 오프셋 → 남/북 방향(로봇이 bay를 바라봄) 순서로 찾는다. 정적 경계 밖은 제외한다. 모두 막히면 `BAY_APPROACH_BLOCKED`.
  - 주행 명령이 원판 안으로 더 들어가면 `STATIC_KEEPOUT_GUARD`로 멈춘다(회전·후퇴는 허용).
  - 러너 v6은 arena spec의 spawn 행 3개 전부를 원판으로 준다(에피소드별 로봇 배정이 아님). `parked_r2` 시나리오는 주차 자리 P1을 추가한다. 평가 전용 r1–상대 로봇 접촉을 physics step마다 센다.

**오프라인 증거 (합산 금지)**
- 렌더 fixture(`tests/fixtures/wrist_zone_skill_v6`, `render_v6_face_fixtures.py`): 상자 yaw −30, −20, −10, −5, **−2, 0, +2**, 5, 10, 20, 30°이고 각 yaw에서 3개 시점이다. 라벨은 평가 전용이다.
  - 받아들인 22프레임은 오차 ≤ 2.7°(중앙값 0.8°)다. 0.30 m 자세 B의 11프레임은 모서리가 잘려 **거부**됐다(틀린 값 0).
  - 같은 프레임에서 N7 cuboid yaw는 0°일 때 최대 30° 틀렸다.
- 기록 프레임 재생(v4 P yaw 0, v5 P 회전 상자, 목표 ≤ 0.405 m):

  | 프레임 | 사용 가능 | 중앙값 | p90 | > 5° |
  |---|---|---|---|---|
  | 정면(\|GT\| ≤ 3°, n = 642) | 97.5% | 0.29° | 1.05° | 0% |
  | 회전(n = 447) | 91.5% | 0.54° | 2.13° | 0.2% |

  같은 정면 프레임에서 v5 근거(N7 cuboid)는 > 7° 비율이 24.7%, p90이 21°였다.
- re-anchor: v5 541의 실제 lift probe 네 프레임은 통과한다. 놓은 뒤 바닥 상자 probe는 거부되고 anchor는 그대로다. 오래된 프레임, 순서·pan 오류, 팔 변화, 해시 변조는 예외로 거부된다.

**개발 실행** (`outputs/.../dev-v6/`, dev 시드, 합산 금지)

| 실행 | 조건 | 결과 |
|---|---|---|
| 412-a (dirty) | yaw 0, 서쪽 | IN_SLOT, GT ✓, 115.7 s. 면 준비 12프레임, 최대 오차 0.81°, 재관측 0 |
| 414-a (dirty) | yaw −30, v5 점에 r2 주차 | **남쪽** 접근. IN_SLOT, GT ✓, 186.5 s. 최대 2.05°. 상대 접촉 0 |
| 413 (`147dcb0`, 깨끗함) | yaw +2, v5 점 남쪽 0.3 m에 r2 주차 | 서쪽 측면 후보(#7). IN_SLOT, GT ✓, 119.7 s. 최대 1.23°. 상대 접촉 0 |

## v6 사전 등록 (시험 고정 커밋 전에 작성)
- 시드 **551–560**(10개). bay E1–E4.
  - **축 정렬 6개**: 551·552·555·560은 yaw 0, 553은 +2, 554는 −2.
  - 회전 4개: 556 +30, 557 −30, 558 +10, 559 −15.
  - r2 주차: 552는 v5 점 북쪽 0.3 m(측면 후보 기대), 557은 v5 점 위(남/북 접근 기대).
  - 표는 러너 v6의 `SCENARIOS`에 있다. 상자 위치·yaw·주차 자리는 setup 값이고, 주차 자리는 정적 keep-out으로만 스킬에 전달된다.
- 조건 **P**: 러너 v6 `--mode diagnostic --contact-profile cargo_noslip_v1`, 시드당 1회. pose는 GT stub이므로 **M1 아님**(`m1_success = false`). SIM 420 s, 900단계.
- 판정 기준
  - G1 `diagnostic_success` ≥ 8/10. **G1a 축 정렬 6개 중 ≥ 5/6**(이번 버전의 주 질문).
  - G2 거짓 IN_SLOT 0. 스킬 판정과 GT 일치 ≥ 9/10.
  - G3 모든 파지의 면 법선이 `own_rgb_markerless_top_edge_vote`에서 나온다. 지도 법선 0.
    - 평가 전용: 면 준비 프레임의 ≥ 95%에서 |추정 − GT 상대 yaw| ≤ 5°.
  - G4 계약: 모든 결과가 `validate_outcome`을 통과한다. `counts_as_m1 = false`, `cameras_seen = [robot_cam]`, 관측 거부 0, 실행 중 소스 변경 없음, 시작 시점 트리가 깨끗하다.
  - G5 weld 0. **r1–상대 로봇 접촉 0 step**(10개 모두). `BAY_APPROACH_BLOCKED` 0. 화물–벽·r1–벽 접촉은 보고한다.
  - G6 등록 시드 10개가 모두 있다. 없으면 `missing/infrastructure_failure`로 보고가 막힌다.
  - 보고 항목: 접근 방향·후보 번호, 재관측, 면 정렬 회전·횡이동, guard 정지, SIM 시간, 단계 수.
- 중단 규칙
  - 시험 커밋 고정, 깨끗한 트리. 거짓 IN_SLOT이 나오면 즉시 멈춘다.
  - 첫 제어 단계 전 인프라 예외만 1회 재실행한다.
  - 동시 SIM 2개, 스레드 1. 시드별 부하 평균을 기록한다. 코호트 뒤 조정은 새 버전으로만 한다.

## M1까지 남은 차단 요인
1. 자세 추정기 연결: PR #178의 폐루프 추정기를 `PoseEstimate` 인터페이스에 붙여야 한다. 지금은 stub이며, #178 보고 이후에 한다.
2. 문 통과 정책: carry_p30 주행과 look_p20 정지 관찰, 문기둥 태그(`_tags_v2`).
3. 접근 교착(v2), 다시 보기 거짓 음성(v3), creep 재장착(v3 P 조건), 운반 중 자기 가림 오경보(v4)는 격리 조건에서 해결됐다. 단 v4 경로의 실제 작동 표본은 적다.
4. 접촉 프로필 결정: zone 스킬을 `cargo_noslip_v1`로 운영할지 사용자 결정이 필요하다. 전역 solver 옵션이며, 팀 운반·교사·ACT 경로에는 아직 적용되지 않았다.
5. 위치 추정 오차가 슬롯 배치(허용 ±6 cm)와 놓은 뒤 확인에 미치는 영향 측정.

## TensorBoard
- 스냅샷: `outputs/tensorboard/0925-zone-owncam-skill`(run 7개: v1-s501…s505, v1-dev401/402).
- 원본 result.json을 바꾸지 않으려고 `outputs/zone-owncam-skill-20260925/tensorboard-view/*/result.json`에 파생 파일을 만들었다. 원본 경로와 해시를 기록했고, `success`는 GT 슬롯 배치다.
- 보기 설정: `outputs/tensorboard-view.json`의 `zone_owncam_skill_v1_20260925` 키. 고정 카드는 reported_success, sim_s, commands, model_calls(0), wall_s다.
- 공용 서버(PID 9291, 다른 작업 소유)에서 run이 나오는 것과 값이 원본과 같은 것을 확인했다. 모델 응답 시간은 LLM이 없어 해당 없음이다.

- v3 스냅샷: `outputs/tensorboard/0926-zone-owncam-skill-v3`(run 34개: v3P-s521…530, v3S-s521…530, v3dev 4개, v2 기준 s511…520). 파생 보기는 `outputs/zone-owncam-skill-20260925/tensorboard-view-v3/`이고, 생성은 `build_tensorboard_v3.py`로 한다.
  - 보기 키는 `zone_owncam_skill_v3_20260926`(HParams 열: policy, case, outcome, 성공, sim_s, commands).
  - 공용 서버(PID 9291, 다른 작업 소유)에서 run 34개가 나오는 것과, 스칼라·HParams 세션 24개(v3) 값이 원본과 같은 것을 확인했다.

- v5 스냅샷: `outputs/tensorboard/0926-zone-owncam-skill-v5`(run 24개: v5P 8, v5dev 6, v4P 기준 10은 진단으로 재표기). 생성은 `build_tensorboard_v5.py`로 한다. 보기 키는 `zone_owncam_skill_v5_20260926`이다.
  - **M1 결과 계약 적용:** 파생 `success` = `m1_success`이므로 모든 run이 0이다(GT stub, `counts_as_m1=false`). `diagnostic_success`는 평가 텍스트와 run 조건에 있다.
  - 공용 서버(PID 9291, 다른 작업 소유)에서 run 24개, 스칼라(v5P-s544: 0 / 190.011 / 372), HParams 세션 8개(v5 P)가 원본과 같은 것을 확인했다.
  - 이전 v1–v4 스냅샷은 덮어쓰지 않았다. 그 `success`는 진단(GT 배치)을 뜻하며 M1이 아니다.

## 원자료 (로컬 전용, 원격 백업 아님)
- v5: `dev-v5/*`(409-a/b/c, 410-a/b, 411-25af375)와 `cohort-v5-4884226/P/<seed>/`(시드별 `contacts-evaluation-only.json`, `provenance-start.json` 포함), `cohort.log`. 경로와 해시는 `results.json`의 `v5_development_runs`, `v5_cohorts`에 있다.
- v4: `dev-v4/*`, `cohort-v4-6664425/{D,P}/<seed>/`와 `cohort.log`. 경로와 해시는 `results.json`의 `v4_development_runs`, `v4_cohorts`에 있다.
- v3: `grip-hold/{local_contact_fine,cargo_noslip_v1}/`, `dev-v3/*`, `cohort-v3-7322a96/{P,S}/<seed>/`와 `cohort.log`. 경로와 해시는 `results.json`의 `grip_hold_probe`, `v3_development_runs`, `v3_cohorts`에 있다.
- 위치: 기본 체크아웃 `outputs/zone-owncam-skill-20260925/`. probe 2개, `cohort-d016c04/{501..505}`, `dev/*`, `cohort-d016c04/cohort.log`(시드별 부하 평균 포함).
- 경로·SHA-256·부하 평균·단계 시각은 `results.json`에 있다.
- 재생성: `python experiments/2026-09-25-zone-owncam-skill/build_results.py`.
