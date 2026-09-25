# 카메라 전용 실행 역량과 구역 연구 격차 분석 (2026-09-25, 읽기 전용)

- 작성: Claude subagent (read-only inventory)
- 날짜: 2026-09-25
- 기준 ref: origin/main `6d80e3e401`, origin/claude/zone-hard-routes `4789d932eb`, origin/claude/zone-team-a2 `143360dcbf`, origin/claude/zone-rgb-outcome `c30091bf72`
- 파일은 수정하지 않았고, 시뮬레이션도 실행하지 않았습니다. 로컬 main은 origin보다 23커밋 뒤처져 있어 모든 인용은 `origin/main`과 표시한 브랜치 기준입니다.

## 핵심 결론
- **자기 RGB와 정적 지도만으로 지도 위 위치를 추정하는 코드와 실험은 저장소에 없습니다.** 지도 기반 주행은 모두 TOP으로 위치를 추정합니다(`harness/known_map_navigation.py:3-6`, `harness/heading_map_navigation.py:146`, `harness/pair_navigation.py:256-262`, `harness/map_goto.py:7`). 오도메트리가 없다는 사실도 코드에 명시되어 있습니다(`harness/real_odometry.py:1-6`).
- 자기 카메라만으로 증명된 것은 **단독 상자 파지·운반·내려놓기**뿐이고, 그것도 벽·문·지도가 없는 장면입니다. `VisualBoxSkill`을 사용한 N7이 5/5입니다(2026-09-09-markerless-n7). 다만 N7의 주행에는 시뮬레이터에만 추가한 `nav_cam`이 쓰였습니다.
- 공동 운반은 **모두 TOP에 의존합니다.** 3대 공동 운반(trio)은 정답 교사로만 성공했습니다.
- **먼저 결정이 필요한 전제**가 하나 있습니다. `nav_cam`은 실제 MasterPi에 없는 추가 카메라입니다. 설치 높이 0.32 m, 25° 아래 방향이고(`sim/navigation_camera_profile.py:1,15-21`), 실물 검증을 하지 않았으며(`docs/camera_team_llm.md:15`), `camera_team` 레이아웃에서만 쓸 수 있습니다(`sim/multi_masterpi_production.py:866-868`). 아래 분석은 "자기 카메라 = 손목 `robot_cam`"을 기본으로 가정했습니다.

## 1. 카메라 전용 실행 역량 목록

| 역량 | 파일 | 입력 카메라 | 장면 | 검증 수치 (실험 ID) | 정답 사용 |
|---|---|---|---|---|---|
| 단독 상자 파지·운반·내려놓기 (`VisualBoxSkill`) | `harness/visual_box_skill.py:30-37,665-697`, `docs/visual_box_skill.md` | 자기 wrist RGB와 자기가 발행한 servo PWM | arena, 단일 상자 | N7 Gemini 5/5 (2026-09-09-markerless-n7) | 없음. PWM은 발행한 setpoint이고 관절은 읽지 않음(`sim/camera_robot_port.py:258-271`) |
| LLM 주행과 조작 선택 | `harness/gemini_transport_policy.py:1,30`, `harness/visual_drive_guard.py:1-5` | wrist와 `nav_cam` | arena / camera_team | N7 5/5, 3로봇 독립 배달 6/9·팀 1/3 (`docs/camera_team_llm.md:50`) | 없음. 단 `nav_cam`은 추가 하드웨어 |
| 색 목표 추종, 주황 장애물·동료 회피 | `harness/visual_navigation.py:78-105` | 자기 RGB 한 대 | 지도 없음, A/B/C 색 구역 | 단독 기록 없음(`tests/test_visual_navigation.py`만) | 없음 |
| 구역 4색 상자 검출 (검출만) | `harness/zone_color_boxes.py` | 자기 RGB 74–80%, 1.2 m 이내 약 97% | zone_wide | 오프라인 (2026-09-25-zone-rgb-color README:17,67) | 정답은 평가에만 사용 |
| 접근·파지 보정 학생 (kernel ridge) | `harness/camera_varied_start_student.py` 등 | **own + TOP** | seed11 고정 장면 | 20/20, 19/20, 29/30 (2026-09-10-grasp-recovery:8, rgb-short-approach, rgb-varied-start:29) | 교사 라벨로 학습(허용 범위) |
| 짧은 운반 20 cm | `harness/camera_short_transport_student.py` | own + TOP | 고정 | 10/10, 다양한 시작 6/10 (2026-09-13-rgb-short-transport:16) | 교사 시연으로 학습 |
| 지도 A*·heading 주행 | `harness/known_map_navigation.py`, `harness/heading_map_navigation.py` | **TOP만**. 자기 RGB는 보관만 함 | maps/navigation | 도착 4/4, 거부 2/2 (2026-09-13-known-map-navigation:55) | 없음 |
| 공동 빔 지형 운반 (`robust`) | `harness/pair_navigation.py`, `harness/pair_transport_vision.py` | TOP으로 바퀴·빔 추적, 자기 RGB는 미끄러짐 감시만 | 좁은 문 포함 지형 5종 | 5/5 (2026-09-14-pair-transport-robustness, `maps/README.md:50-52`) | 없음 |
| dispatch RGB 스킬 (v61) | `harness/rgb_skill_execution.py:283-286,352-478`, `harness/solo_box_transport.py:1,94-161` | 단독: 조작은 own, 목표 servo는 TOP. 공동: TOP 중심 | 공동 출하장 | 6조건 각 1회 성공 (dispatch-adaptive-recovery-20260917) | 없음 |
| ACT 공동 운반 | `configs/model_artifacts.json` seed18/19 | own + TOP + 고정 경로 | open | 교사 3/3, ACT 1/3·0/3 (2026-09-18-act-pair-carry:38) | 교사 시연으로 학습 |
| 구역 교사 | `scripts/zone_teacher.py:1-7,279`, `scripts/zone_team_teacher.py:657-741` (브랜치 `origin/claude/zone-team-a2`) | 정답 `xpos/xquat` 사용 | zone_wide 계열 | 하드 경로 fixture (2026-09-25-zone-hard-routes:69-90) | **정답. 학생이 아님** |
| RGB 결과 판정 | `harness/zone_rgb_outcome.py` (브랜치 `claude/zone-rgb-outcome`) | TOP 4장 | zone | 오프라인 | 새 규칙에서는 **심판 전용으로만** 사용 가능 |

## 2. 질문별 답
- **(a) 자기 카메라로 지도 위 위치 추정: 없음.** 모든 지도 주행은 TOP으로 위치를 추정합니다(위 인용). 실물용 `real_map.py`도 오도메트리가 없어서 로봇 기준 좌표 스냅샷만 저장합니다(`harness/real_map.py:1-5`).
- **(b) 자기 카메라로 벽 우회·문 통과: 없음.** 지도 없는 색 추종과 장애물 회피(`visual_navigation.py`)만 있습니다. N7의 장애물 우회는 `nav_cam`을 썼습니다(N7 `results.json`의 `nav_frames_reviewed`). 구역 하드 경로의 벽은 0.10 m 높이에 바깥 벽과 같은 색이고, **상자를 들고 팔을 내리면 wrist 카메라에는 바닥만 보입니다**(zone-hard-routes README:30,42).
- **(c) 자기 카메라로 상자 파지와 배치: 있음. 단 청록 단일 상자 기준입니다.** `VisualBoxSkill` 기준 N7 5/5입니다. 구역 상자 치수(`sim/zone_arena.py:40`, 34×40×32 mm)는 이 스킬의 상자 가정과 같습니다. `markerless_box`는 청록만 받고, 후보가 둘 이상이거나 1.2 m를 넘으면 거절합니다(zone-rgb-color README:17). 다른 색은 검출만 가능하고, can/tile의 자기 카메라 파지는 없습니다.
- **(d) 카메라 기반 공동 운반: 모두 TOP입니다.** `run_camera_pair_transport.py:88,129-135`는 own + overhead 입력입니다. 성공 사례인 robust 5/5, dispatch, ACT도 TOP으로 바퀴·빔을 추적합니다. 자기 RGB가 맡은 역할은 빔 미끄러짐 감시(2026-09-15-pair-grasp-retention:47)와 파지 보정뿐입니다. 운반 중에는 빔이 자기 카메라 대부분을 가립니다(2026-09-18-act-pair-carry:54). 자기 RGB만 쓴 Gemini 공동 파지는 0회 성공이었습니다(2026-09-09-camera-grasp).
- **(e) 교사 시연으로 학습한 정책**
  - kernel-ridge 접근·파지 학생: 통과했지만 입력이 own + TOP입니다(위 표).
  - ACT seed18/19: `available`이지만 기본 제어기가 아닙니다. 성공률은 1/3, 0/3입니다.
  - `act-action-first-study-seed24`: 오프라인 종료 게이트에 실패했습니다.
  - `act-expanded-20260922`: `unavailable`입니다(`configs/model_artifacts.json`).
  - **자기 RGB만 쓰는 학습 정책과, 구역 교사 시연으로 학습한 정책은 없습니다.**

## 3. 구역 연구 격차 (zone_wide_door / two_doors / corridor)

| 과제 | 지금 있는 것 | 없는 것 | 난이도 |
|---|---|---|---|
| 모든 과제 공통 | 정적 지도 JSON(벽·통로·구역·슬롯, `maps/zones/zone_wide_door.json`), 교사, TOP 심판 | 자기 카메라 위치 추정, 통로 추종, 동료 감지 후 양보, 명령→이동 보정 | 상 |
| 단독 청록 상자 | `VisualBoxSkill` 파지·배치, 청록 검출 | 적재 구역 탐색, 문 통과, 운반 중 시야(바닥만 보임), 구역 슬롯 배치 | 중상 |
| 단독 3색 상자 | 검출만 있음(`own_zone_v2`) | 동일 색 여러 개 식별, 파지 연결 | 중상 |
| can / tile | 교사 물리만 성공(2026-09-25-zone-cargo-catalogue) | 자기 RGB 인식·파지·배치 전부. 종류 인식은 TOP 전용(`top_cargo_v1`) | 상 |
| long_beam 2대 | TOP 기반 빔 운반 5/5, 교사 편대 | 자기 RGB로 빔 끝·자기 정거장 정렬, 가림 상태에서 동기 운반, 문 통과 | 최상 |
| heavy_crate 2대 | 교사만 성공, `cargo_noslip_v1` 필요 | 위 항목 전부와 무게 미끄러짐 | 최상 |
| tri_frame 3대 | 교사만 성공(1/1, noslip 3/3) | 카메라 기반은 전부 없음 | 최상+ |
| 구역 과제 러너 | 교사 실행기만 있음(A2 WIP, PR #169) | 학생 실행기 슬롯. `docs/e2e_status_20260925.md`의 "학생 이동: 미착수" | 중 |

## 4. 해결 방법 후보

**(i) 고전적 영상 파이프라인**
- 방식
  - 위치 추정: 자기 RGB에서 정적 지도 특징(구역·적재 바닥 칠, 벽 하단 경계, 문기둥)을 찾아 입자 필터로 추정합니다.
  - 예측: 발행한 명령 이력으로 dead reckoning합니다(허용). 명령→이동 모델은 교사 로그로 오프라인 보정합니다(교사 예외).
  - 장애물·동료: 자기 RGB에서 검출합니다(`visual_navigation`, `visual_drive_guard` 재사용).
  - 파지: `VisualBoxSkill`의 visual servo를 그대로 씁니다.
- 장점: 해석과 감사가 쉽고, 데이터가 적어도 되며, 기존 코드를 재사용할 수 있습니다.
- 단점
  - 낮은 단색 벽은 대칭이라 위치가 모호합니다.
  - 운반 중에는 시야가 바닥뿐이라 "멈추고 팔을 들어 둘러보기" 매크로가 필요합니다.
  - 명령만으로 추정하면 오차가 쌓입니다. 과거 기록에서 25 cm 명령 적분이 33 mm 짧았습니다(2026-09-10-rgb-short-approach:25). heading 주행도 회전 명령을 신뢰하지 않았습니다(`heading_map_navigation.py:3-6`).
- **필요한 환경 추가:** 벽과 문기둥에 고유 색띠나 ArUco 표식을 붙이는 것입니다.
  - AGENTS.md의 지도 예외는 "고정 장애물·목적 구역 형상"과 "지도·물리 환경의 버전·해시 연결"을 허용합니다. 로봇 외관·카메라·물체를 바꾸지 않으므로 규칙에 걸리지 않습니다.
  - 새 지도 버전(예: `zone_wide_door` v2)으로 등록하고, 현장 일치와 위치 추정 정확도를 따로 검증해야 합니다.
  - 표식 추가는 사용자 확인이 필요한 설계 결정입니다.

**(ii) 교사 시연 모방 학습 (BC/ACT, 자기 RGB와 명령 이력 입력)**
- 장점: 교사가 이미 있어 시연을 대량 생성할 수 있습니다.
- 단점
  - 과거 ACT 결과가 약합니다(1/3, 0/3, 종료 신호 누락: 2026-09-24-action-act).
  - 문을 지나는 긴 과제는 분포 이탈에 약하고, 다중 로봇 동기화를 학습하기 어렵습니다.
  - GPU 예산과 데이터 규모를 먼저 정해야 합니다.

**(iii) 혼합 방식 (권장)**
- 고전적 방식으로 위치를 추정하고, 정적 지도 A*(`map_goto.py`)로 경로를 정합니다. 단 `map_goto`의 시작 위치를 TOP 추정 대신 자기 추정으로 바꿔야 합니다.
- 국소 기술(접근, 문 진입 정렬, 파지 보정)은 교사 라벨로 **자기 RGB 전용 학생을 다시 학습**합니다. 기존 kernel-ridge 파이프라인을 재사용하되 TOP 입력을 제거합니다.
- 교사의 정답 자세는 위치 추정기의 학습 표적과 오프라인 평가에만 씁니다.

**권장 단계**
- **S0 결정:** (1) `nav_cam` 허용 여부. zone 장면에는 현재 없습니다. (2) 벽 표식 추가 여부. (3) TOP은 심판 전용이며, `zone_rgb_outcome`을 평가기로 쓸 수 있습니다.
- **S1 오프라인 위치 추정:** 교사 실행으로 자기 RGB, 발행 명령, 평가 전용 정답을 수집합니다. zone-rgb-color와 같은 render/score 분리 구조를 쓰고, 문 근처 오차 목표를 미리 고정합니다.
- **S2 최소 마일스톤 M1:**
  - 과제: `zone_wide_door`에서 로봇 1대, 동료 없음. 알려진 청록 상자 1개를 적재 구역에서 집어 `door_1`을 지나 구역 슬롯에 놓습니다.
  - 입력: 자기 wrist RGB, 정적 지도, 발행 명령 이력만. TOP은 심판과 영상에만 씁니다.
  - 조건: weld OFF, 벽 접촉 0.
  - 규모: 사전 고정한 시작 5개. 같은 시작의 교사 결과는 기준으로만 비교하고 학생 성공에 합산하지 않습니다.
- **S3:** 4색, can/tile, two_doors/corridor, 자기 RGB로 동료를 감지해 양보하는 다중 단독 운반.
- **S4:** 2대 빔/crate. 자기 RGB 편대 정렬과 명령 기반 동기화를 쓰고, 가림 문제에는 멈추고 둘러보기로 대응합니다.
- **S5:** 3대 frame.

## 확인 범위와 한계
- 문서, 코드, 실험 기록, 브랜치 파일만 읽었습니다. 코드 실행, 테스트, SIM은 하지 않았습니다.
- 로컬 main은 fast-forward하지 않았습니다(읽기 전용 요청).
- `run_camera_pair_transport.py` 코호트의 최종 수치는 2026-09-09 기록 두 건(파지 0회)만 확인했습니다.

result: 저장소에 자기 카메라만으로 지도 위 위치를 추정하거나 문·벽을 통과하는 실행기는 없고(모두 TOP 의존), 자기 카메라 단독 증거는 벽 없는 단독 상자 N7 5/5(nav_cam 사용)뿐입니다. 혼합 방식(표식·지도 입자 필터, 명령 기반 예측, 교사 라벨로 재학습한 자기 RGB 국소 기술)을 권장하며, 첫 마일스톤은 zone_wide_door에서 로봇 1대가 청록 상자를 wrist RGB만으로 문 너머 구역에 배달하는 것입니다.
