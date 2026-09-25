# 03 환경

**기준 커밋: `origin/main` `1cd9ea1` (2026-09-26 확인).**

## 3.1 경기장 — `zone_wide` 계열

기본 지도는 `maps/zones/zone_wide`다. 크기는 **6.45 × 4.6 m, TOP 카메라 4대**이고
TOP은 평가·채점 전용이다([experiments/2026-09-25-zone-wide-arena](../../experiments/2026-09-25-zone-wide-arena/README.md),
PR [#155](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/155)).

이전의 소형 `zone_open`은 새 실행에서 은퇴했다. "파일 자체는 Z1–Z3 재현용으로 남긴다"
([docs/decision_log.md](../decision_log.md) 2026-09-25 지도 항목;
PR [#173](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/173)에서 은퇴 적용).
두 지도 비교 렌더는 `docs/design/media/2026-09-25-zone-open-vs-wide.png`에 있다
(같은 축척, 하향식, 왼쪽 = 은퇴한 `zone_open` 6.45×2.3 m, 오른쪽 = `zone_wide` 6.45×4.6 m,
SHA-256 `d410af31…`).

### 벽·문·복도 지도 3종

PR [#173](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/173)(병합 전 기록,
[experiments/2026-09-25-zone-hard-routes](../../experiments/2026-09-25-zone-hard-routes/README.md))가
`zone_wide` 위에 안쪽 벽을 더해 세 지도를 만들었다.

| 지도 | 구조 | 교사 fixture makespan |
|---|---|---|
| `zone_wide_door` | 0.50 m 문 1개 | 217–252 s |
| `zone_wide_two_doors` | 0.50 m + 1.00 m 문 2개(`door_narrow`, `door_wide`) | 188–212 s |
| `zone_wide_corridor` | 0.50 m 한 차선 1.7 m + 비켜 서는 자리 | 333–396 s |
| (기준) `zone_wide` | 안쪽 벽 없음 | 150–156 s |

교사 A*가 벽 직사각형을 피하고(`harness/static_keepouts.py`), 한 차선 대치는 물리 상태만으로
푼다. fixture(LLM 0회) plan_first·dynamic 28/28 목표 달성, 통로 대치 6회 모두 해결,
복도 출구 밖 정면 밀기 1회(independent)는 미해결이다. **교사 조건이며 LLM 협업 근거가 아니다.**

벽의 성질은 자기 카메라 실행에 직접 영향을 준다. "구역 하드 경로의 벽은 0.10 m 높이에
바깥 벽과 같은 색이고, 상자를 들고 팔을 내리면 wrist 카메라에는 바닥만 보인다"
([own-camera inventory](../design/2026-09-25-own-camera-inventory-claude.md), zone-hard-routes README 인용).

## 3.2 벽 AprilTag 랜드마크

사용자 결정(2026-09-25): "벽과 문기둥에는 AprilTag(`tag36h11`) 표식을 붙여도 된다. 표식은 새로
버전을 매긴 지도에 정적 지도 특징으로 등록한다. **화물과 로봇에는 계속 표식을 붙이지 않는다.**"
([docs/decision_log.md](../decision_log.md)). 화물 표식 금지는 2026-09-09 표식 제거 결정
([docs/cargo_marker_removal_20260909.md](../cargo_marker_removal_20260909.md))을 잇는다.

등록된 태그 지도는 `zones/zone_wide_door_tags_v1.json`,
`zones/zone_wide_two_doors_tags_v1.json`, `zones/zone_wide_corridor_tags_v1.json`이다
([maps/README.md](../../maps/README.md) "구역 벽 AprilTag 지도" 절).

- base 지도 파일은 바꾸지 않고, 태그 지도가 base의 `static_map_sha256`을 기록한다.
- 태그는 벽면에 붙인 **시각 전용 geom**이며 물리는 같다. 테스트가 "같은 명령에서 base 장면과
  태그 장면의 `qpos` 궤적이 비트 단위로 같다"와 "태그 geom은 모두 contype·conaffinity 0"을
  확인한다([experiments/2026-09-25-zone-owncam-loc](../../experiments/2026-09-25-zone-owncam-loc/README.md)).
- `zone_wide_door_tags_v1`: static_map `238edb67…`, base `zone_wide_door` 파일 `a4d2c03d…`,
  **태그 70개**, 기본 배치는 높이 0.05 m·크기 0.072 m다.

`_tags_v2`(PR [#178](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/178),
병합 전)는 가시성 연구 권고를 반영해 문기둥 판(태그 높이 0.15 / 0.25 m, 양면)을 두고 문 1 m
안은 태그 간격 0.30 m로 한다. v1 파일은 해시로 고정하고 물리는 base와 비트 단위로 같다.

권고의 근거는 wrist 가시성 probe다([experiments/2026-09-25-zone-owncam-skill](../../experiments/2026-09-25-zone-owncam-skill/README.md)).
벽면 0.05 m 태그는 carry 자세로 0.6–1.0 m에서 44–68 px지만 **0.3 m에서는 검출되지 않았다.**
문기둥 0.15 m 태그는 look 자세로 1.5→0.3 m 전 구간 31–104 px이고, 0.6 m 이하에서는 pan ±24°가
필요했다.

## 3.3 화물 카탈로그와 접촉 프로필

화물 종류는 필요한 로봇 수별로 정해져 있다(PR
[#164](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/164), 병합;
[experiments/2026-09-25-zone-cargo-catalogue](../../experiments/2026-09-25-zone-cargo-catalogue/README.md)).

| 필요 로봇 | 화물 | 비고 |
|---|---|---|
| 1대 | `can`, `tile` | |
| 2대 | `long_beam`(0.6 m), `heavy_crate`(0.9 kg) | |
| 3대 | `tri_frame`(1.5 kg) | |

**로봇 인원 수는 물리에서 나온다.** 단독 로봇 한계는 0.66 kg이고 0.68 kg에서 한 대가 넘어진다
([docs/decision_log.md](../decision_log.md) 화물 다양성 항목). 정답 교사·weld OFF(`d81514a`)로
solo 3/3·pair 2/2·trio 1/1 운반에 성공했고, **한 대 적은 조건은 3/3 들기 실패**했다.

구역 상자 치수는 34×40×32 mm(`sim/zone_arena.py:40`)이며, 이는 N7 `VisualBoxSkill`의 상자
가정과 같다([own-camera inventory](../design/2026-09-25-own-camera-inventory-claude.md)).

접촉 프로필은 두 가지다.

| 프로필 | 내용 |
|---|---|
| `local_contact_fine` | 기본값. 자기 카메라 실행 기록도 이 프로필을 쓴다 |
| `cargo_noslip_v1` | opt-in. `noslip_iterations 10`. 60초 미끄러짐을 ≤0.51 mm로 줄이고 4 m·두 번 회전 운반을 pair/trio 3/3 성공시켰다(기존 프로필은 crate·frame 실패). 한 대 적은 조건은 여전히 실패. PR [#167](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/167) |

**weld는 전 구간 OFF다**([docs/decision_log.md](../decision_log.md), [AGENTS.md](../../AGENTS.md)).

주의할 검토 지적이 두 건 남아 있다. Codex PR 검토는 `cargo_noslip_v1`이 화물 접촉에 한정하지
않고 장면 전체 solver 옵션을 바꾸므로 "전역 물리 프로필로 명시하고, 하중·장애물 접촉·역할
순열별 대수 부족 조건을 비교해야 한다"고 적었고, PR #169 통합 경로에서 "opt-in이 자동
기본값이 된다"는 지적도 남겼다
([docs/design/2026-09-25-zone-pr-review-codex.md](../design/2026-09-25-zone-pr-review-codex.md)).

## 3.4 로봇 플랫폼 — Hiwonder MasterPi

플랫폼 사실은 오픈소스 조사가 코드에서 확인한 값이다
([own-camera OSS survey](../design/2026-09-25-own-camera-oss-survey-claude.md)).

| 항목 | 값 |
|---|---|
| 차체 | 메카넘 바퀴 4개, 지름 65 mm, 축간 0.12 m (`sim/masterpi_geometry.py`) |
| 팔 | 5자유도 PWM 서보(500–2500 µs). 1번 집게, 3·4·5번 관절, 6번 받침 회전 |
| 탐색 자세 | `SEARCH_POSE={1:2000,3:740,4:2320,5:1320,6:1500}` (`sim/masterpi_production_v2.py:18`) |
| 관절 측정 | **없다.** 실물은 명령 PWM만 안다 (`harness/real_geometry.py`, `real_odometry.py`) |
| 오도메트리 | **없다** (`harness/real_odometry.py:1-6`) |

SO-100/SO-101이나 LeKiwi가 아니며 저장소에 관련 언급이 없다(같은 조사).

### 손목 어안 카메라 (`robot_cam`)

| 항목 | 값 |
|---|---|
| 위치 | 팔 끝(집게)에 달린 icspring 어안 카메라 **1대뿐** |
| 해상도 | 640×480 |
| 내부 파라미터 | fx≈619.5, fy≈622.2, 어안 왜곡 계수 4개 |
| 설치 위치 상태 | "잠정·미검증" (`sim/masterpi_camera_profile.py`) |
| 시뮬레이터 처리 | 핀홀 렌더를 실물 어안 기하로 다시 매핑(`raw_fisheye_remap`). 그래서 `cv2.fisheye` 모델을 시뮬레이터와 실물에 그대로 쓸 수 있다 |
| 유효 시야(SIM) | 약 54°×42°. **실물 어안은 더 넓다**([zone-owncam-skill](../../experiments/2026-09-25-zone-owncam-skill/README.md) 1절 한계) |

**짐을 들면 이 카메라가 가려진다.** 카메라가 집게에 고정돼 있어 들고 있는 상자가 항상 같은
픽셀을 가린다. 파지 직후에는 프레임의 62–64%(유효 픽셀의 약 75%)가 가려지고 위쪽 약 10° 띠만
보이며, pan은 가림을 풀지 못한다([zone-owncam-skill](../../experiments/2026-09-25-zone-owncam-skill/README.md) 1절).

### 사용하지 않는 카메라

- **`nav_cam`(시뮬레이터 전용)** — 640×480 핀홀, 세로 FOV 70°, 차체 기준 (0.05, 0, 0.32) m,
  25° 아래 기울임, `camera_team` 배치에서만 켜진다(`sim/navigation_camera_profile.py`).
  문서에 "이 추가 장치는 실물에서 검증되지 않았다"고 적혀 있다. 사용자 결정으로
  **연구에서 로봇 입력으로 쓰지 않는다**([docs/decision_log.md](../decision_log.md) 2026-09-25 (b)).
  구현 검증기도 `nav_cam` 참조를 거부한다(PR
  [#187](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/187),
  `tests/test_wrist_zone_skill.py`).
- **TOP 카메라 4대** — 평가·채점·영상에만 쓴다([02](02-experiment-design.md)).

## 3.5 시뮬레이터와 호스트

자기 카메라 위치 추정 기록 당시 환경이다
([experiments/2026-09-25-zone-owncam-loc](../../experiments/2026-09-25-zone-owncam-loc/README.md)).

| 항목 | 값 |
|---|---|
| 런타임 | Python 3.12.13, MuJoCo 3.12.0, OpenCV 5.0.0, numpy 2.5.2, macOS 27.2 arm64 |
| 장면 | `TaggedZoneScene`, `local_contact_fine`, weld OFF, timestep 0.002 s, 동기 SIM |
| 호스트 | Apple M3, 메모리 16 GB, CUDA 없음([OSS survey](../design/2026-09-25-own-camera-oss-survey-claude.md)) |
| 환경 이름 | `.venv-sim-worker-mac`(`.venv-sim`은 호환 링크) |

공용 Mac에서 여러 에이전트가 동시에 작업하므로 실행마다 부하 평균을 기록한다. owncam-loc
코호트에서는 "다른 에이전트 SIM 4–5개가 함께 돌아 1분 평균이 12–175"였고, 동기 SIM이라
결과는 부하와 무관하다고 적었다(같은 기록). 스레드는
`OMP/OPENBLAS/VECLIB/MKL_NUM_THREADS=1`로 제한하고 동시 SIM은 2개 이하로 둔다
([AGENTS.md](../../AGENTS.md)).

## 3.6 아직 없는 환경 요소

[docs/e2e_status_20260925.md](../e2e_status_20260925.md)의 "아직 할 일" 표에 있는 항목이다.

- **미등록 장애물/가림** — 시작 시·중간 등장 차단, TOP 비가시·자기 RGB 가시 조건 구현.
  설계 제안은 [2026-09-25-zone-unmapped-blockage-codex.md](../design/2026-09-25-zone-unmapped-blockage-codex.md)이며
  A2 이후 착수로 적혀 있다. TOP이 평가 전용이 된 뒤로는 비공개성의 기준이 바뀌었다.
  "발견 이전에 다른 로봇의 자기 RGB에 단서가 있었는가"가 기준이며 "TOP에 보였다는 이유로
  정보 비대칭 사례에서 제외할 필요는 없다"([설계 v1](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/180) 1절).
- **구역 과제의 time-limit 검사.**
- 설계 v1 3절이 요구하는 **pickup slot 정의**(`P1-2` 등)는 새 지도 버전에 넣어야 한다.
  패키지 A의 도식 렌더에서 `zone_wide_two_doors_tags_v1`의 pickup bay `P1-1..P2-3`을 확인한
  기록이 있다(PR [#187](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/187)).
