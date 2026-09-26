# 2026-09-26 M1 자기 카메라 관측 기억 ON/OFF ("한 번 보고 기억하기")

- 작성: Kiro(`kiro/zone-owncam-memory`, PR #211, base `claude/zone-m1-owncam` #201). `kiro/`는 Kiro 작업 표시다.
- 첫 세션(계정 1)이 dev-a1까지 하고 멈췄다. 계정 2에서 이어서 A1–A3 개정, dev-a2, 동결, test를 했다.
- 설계·재사용 조사: [`docs/design/2026-09-26-owncam-memory-reuse.md`](../../docs/design/2026-09-26-owncam-memory-reuse.md).
- **memory_v2의 모든 결과는 "interim, tag provider"다.** 기억은 표식 비의존이다. 위치 표식 관측만 임시로 AprilTag에서 받았다. 태그는 연구 환경에서 없어질 예정이다.

## 결론

- **사전 등록 주장 규칙 불충족 → "기억 ON이 빠르다"고 주장하지 않는다.** test 161–166, 조건마다 1회: m1_success **OFF 4/6, memory_v2 3/6**, false_success 둘 다 0.
- 둘 다 성공한 3 seed(162·164·165)에서는 memory_v2가 **SIM 161–174 s(41–44%) 짧았다.** 둘러보기 시간은 평균 181.7 → 42.8 s, pan 정지는 134 → 33회, 명령 수는 6,717 → 4,229였다. 평가 전용 위치 오차 p90은 비슷했다(중앙값 0.059 → 0.055 m).
- memory_v2 실패 3건 중 2건은 기억 쪽 결함이다.
  - s161: 도착 확인을 건너뛰어 자세가 0.13 m 틀린 채 파지를 시작했다(A3 문턱이 느슨함).
  - s166: 먼 거리로만 본 상자를 피하지 않고 밀었다. 그 뒤 SIM 한도까지 갇혔다. 거짓 확정 트랙 5개도 여기서 나왔다.
  - 나머지 1건(s163)은 두 조건 모두 같은 탐색 사각지대에서 실패했다(기준선 M1 탐색 설계).
- 지도 v3(`kiro/zone-map-v3`)는 위치 추정 게이트를 통과하지 못했다(v3·A1 모두 실패). 사전 등록 `map_v3_rule`에 따라 **v3는 비교하지 않았다.**

## 조건 (`prereg.json`, `prereg_amendments.json`)

| | off | memory_v2 |
|---|---|---|
| 제어기 | 동결 M1 학생 `harness/m1_owncam_delivery.py`(#201 `ca2fdb8` 해시) | 같은 학생 + `harness/m1_owncam_memory.py` v2 |
| 기억 | 없음 | `harness/owncam_memory.py` v2(표식 비의존), `owncam_landmarks.py`(정적 지도 표식 목록·제공자 인터페이스), `owncam_landmark_tags.py`(**임시 태그 제공자**), `owncam_drive_mem.py` v2 |
| 공통 | 물리 러너 `scripts/run_m1_owncam.run`, 스킬 v9, `calibration_m1_dev.json`, 자기 카메라 PF, M1 게이트·판정, `cargo_noslip_v1`(사용자 승인 2026-09-26), weld OFF, 동기 SIM, robot_cam 5 Hz, SIM 한도 720 s, 지도 `zone_wide_door_tags_v1`, 같은 seed·출발·슬롯·상자 | |

- 입력은 로봇 자신의 `robot_cam` 프레임, 자기 PoseReport, 자기 발행 명령, 정적 지도, 고정 교정뿐이다. GT는 `eval_only/`에만 있다. LLM 호출은 없다.
- 스레드 1(OMP/OPENBLAS/VECLIB/MKL), 동시 2개, 에피소드 전 여유 ≥ 30 GiB(40–113 GiB 기록), 부하 평균은 `launch_load.txt`와 에피소드별 `memory_runner.json`에 있다(15분 평균 38–78). 동기 SIM이라 wall 시간은 결과가 아니다.

### 개정 A1–A3 (dev-a1 뒤, 동결·test 전에 등록)

| ID | 근거 | 변경 |
|---|---|---|
| A1 | 사용자 2026-09-26 "표식은 없애기로 했잖아. 그걸 기억하면 안 되지 않을까?" | memory_v1 → memory_v2. 기억은 정적 지도 기하에서 만든 표식(문기둥 4·벽 모서리 8·문 틈 1·벽면 10) 관측을 저장한다. 어디를 볼지도 이 기하로 정한다. 태그는 교체 가능한 임시 제공자다. 오프라인 확인: 무작위 추정 400개에서 v1과 보기 방식 400/400, 첫 pan 385/392가 같다(`planner_equivalence_check.json`) |
| A2 | dev-a1 memory_v1: 목표 트랙이 GT에서 0.123 m(> 0.10 m) 떨어진 채 σ 0.029 m로 확정 | 자기 RGB 상자 광선을 PF 교정의 카메라 고도 편향(−0.01868 rad, 기존 값)으로 다시 투영. 트랙 σ 하한 = max(0.02 m, 관측 당시 자기 σ). 오프라인 확인(`box_bias_check.json`, 정착 프레임): near 1.0–1.3 m 거리 편향 +0.102 → −0.004 m |
| A3 | dev-a1 memory_v1: 주행 중 자세 오차 0.02 → 0.16 m(σ 0.033 m), 도착 확인 생략 뒤 파지가 0.15 m 어긋남 | 도착 확인은 정지 둘러보기 고정이 신선할 때(이동 ≤ 0.5 m, ≤ 60 s)만 생략. 짧은 보기의 조기 종료는 그 정지 중 둘러보기 고정이 있을 때만 |

## dev (결과가 아님, 전부 기록)

| 시도 | 소스 | 조건 | seed | 결과 | SIM s | 둘러보기 / pan 정지 / 둘러보기 시간 s | 명령 | 위치 오차 p90 m | 거짓 확정 트랙 |
|---|---|---|---|---|---|---|---|---|---|
| dev-a1 | `ad78ef2` | off | 151 | IN_SLOT | 315.7 | 16 / 96 / 130.1 | 5,520 | 0.076 | – |
| dev-a1 | `ad78ef2` | memory_v1 | 151 | IN_SLOT | 252.5 | 12 / 26 / 32.2 | 4,327 | 0.155 | **1**(0.123 m) |
| dev-a2 | `de6b520` | memory_v2 | 151 | IN_SLOT | 251.7 | 13 / 27 / 33.4 | 4,347 | 0.097 | 0 |
| dev-a2 | `de6b520` | memory_v2 | 152 | IN_SLOT | 316.1 | 24 / 41 / 48.8 | 5,365 | 0.196 | 0 |

- dev 점검 규칙(`prereg_amendments.json` `dev_check`): 두 dev 모두 m1_success, false_success 없음, 거짓 확정 트랙 없음, 목표 트랙 GT 오차 ≤ 0.10 m. 결과: **통과**(목표 트랙 오차 0.041 / 0.014 m). 이에 따라 `4716c8e`에서 동결했다(`frozen_source.json`, 파일 40개 해시).
- dev-a2 s152에서 본 위험(동결 전에 규칙대로 통과로 처리했고, 여기 기록한다):
  - 운반 중 자세 오차가 30 s 동안 0.13–0.32 m였는데 PF σ는 0.04 m였다. 짧은 보기 1 pan이 σ 기준으로 조기 종료했다.
  - 먼 거리로만 본 초록 상자(`tentative`)를 바퀴가 64 step 건드렸다. M1 판정은 다른 상자 접촉을 실패로 세지 않는다. M1 test(#201) s106에도 10 step이 있었다.

## test (161–166, 조건마다 1회, 동결 소스 `4716c8e`, 실행 SHA `48d5139`)

| seed | 조건 | 결과 | m1 | SIM s | 둘러보기 / pan 정지 / 시간 s | 탐색 sweep / pan | 명령 | 위치 오차 p50 / p90 m | 게이트 오차 m | 탐색 목표 오차 m | 다른 상자 접촉 step | 거짓 확정 트랙 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 161 | off | SEARCH_NOT_FOUND | ✗ | 200.7 | 6 / 36 / 47.8 | 5 / 30 | 2,536 | 0.039 / 0.166 | – | – | 0 | – |
| 161 | memory_v2 | SKILL_GRASP_TARGET_NOT_VISIBLE | ✗ | 33.8 | 1 / 6 / 7.9 | 0 / 0 | 297 | 0.032 / 0.135 | – | 0.027 | 0 | 0 |
| 162 | off | IN_SLOT | ✓ | 426.5 | 24 / 144 / 195.5 | 2 / 12 | 7,118 | 0.020 / 0.054 | 0.007 | 0.025 | 0 | – |
| 162 | memory_v2 | IN_SLOT | ✓ | **252.3** | 19 / 39 / 49.3 | 1 / 4 | 4,596 | 0.014 / 0.055 | 0.012 | 0.051 | 0 | 0 |
| 163 | off | SEARCH_NOT_FOUND | ✗ | 161.3 | 6 / 36 / 47.7 | 5 / 30 | 2,142 | 0.022 / 0.056 | – | – | 0 | – |
| 163 | memory_v2 | SEARCH_NOT_FOUND | ✗ | 106.6 | 5 / 12 / 16.1 | 5 / 8 | 1,375 | 0.037 / 0.079 | – | – | 0 | 0 |
| 164 | off | IN_SLOT | ✓ | 395.1 | 22 / 132 / 178.9 | 3 / 18 | 6,680 | 0.019 / 0.060 | 0.006 | 0.013 | 0 | – |
| 164 | memory_v2 | IN_SLOT | ✓ | **225.1** | 16 / 31 / 41.3 | 2 / 3 | 4,090 | 0.014 / 0.048 | 0.007 | 0.043 | 0 | 0 |
| 165 | off | IN_SLOT | ✓ | 366.8 | 21 / 126 / 170.8 | 2 / 12 | 6,354 | 0.025 / 0.075 | 0.005 | 0.041 | 0 | – |
| 165 | memory_v2 | IN_SLOT | ✓ | **205.8** | 15 / 30 / 37.8 | 0 / 0 | 4,000 | 0.051 / 0.093 | 0.007 | 0.068 | 0 | 0 |
| 166 | off | IN_SLOT | ✓ | 450.0 | 26 / 156 / 211.9 | 3 / 18 | 7,497 | 0.022 / 0.093 | 0.007 | 0.053 | 242 | – |
| 166 | memory_v2 | SIM_LIMIT | ✗ | 720.5 | 111 / 329 / 382.7 | 0 / 0 | 11,538 | 0.078 / 0.214 | – | 0.080 | **3,609** | **5** |

- 결과의 `IN_SLOT` = `SKILL_OWN_RGB_PLACEMENT_IN_SLOT`. 둘러보기·pan·명령은 로봇 자신의 발행 명령에서 셌다(`build_results.py`). 위치 오차·목표 오차·접촉·거짓 확정은 평가 전용 GT로 채점했다. 거짓 확정 트랙 = 확정 시점에 같은 색 GT 상자(초기 위치)에서 0.10 m보다 먼 트랙이다.
- 주장 규칙(`prereg.json` `claim_rule`, A1로 memory_v1 → memory_v2):
  - (a) memory_v2 성공 수 ≥ OFF: 3 < 4 → **불충족**.
  - (b) memory_v2 false_success 0: 충족.
  - (c) 둘 다 성공한 seed ≥ 3이고 전부 memory_v2가 빠름: 충족(Δ −174.2 / −170.0 / −161.0 s).
  - → `claim_memory_faster = false`.
- 거짓 빈칸(기억이 빈 바닥이라 한 칸 0.05 m 안의 비목표 상자): memory_v2 s166 1칸, 나머지 0.

## 실패 분석 (사후, 평가 전용 기록 사용, 소스 수정 없음)

- **s161 memory_v2: 신선도 문턱이 느슨했다(A3).**
  - 기억은 출발 직후 초기 둘러보기에서 목표를 찾았다(OFF는 끝까지 못 찾음). 트랙 오차는 0.027 m였다.
  - 접근 구간 0.43 m를 SEARCH_POSE로 가는 동안 자세 오차가 0.03 → 0.14 m(yaw 2.6°)로 커졌다. PF σ는 0.034 m였다.
  - 정지 둘러보기 고정에서 0.5 m 안이라 도착 확인을 건너뛰었다. 스킬 v9는 0.13 m 틀린 위치에서 시작해 목표를 놓쳤다.
  - dev-a1과 같은 기제다. A3의 0.5 m는 짐 상태 기준값이다. 빈손 SEARCH_POSE 주행에는 느슨했다.
- **s166 memory_v2: 먼 거리로만 본 상자를 피하지 않았다.**
  - 탐색에서 먼 목표 트랙으로 가던 중(t = 28.7 s), 경로 위 빨간 상자(−0.2, −0.85)를 앞판·바퀴로 밀기 시작했다. 이 상자는 990회 모두 `far_coarse`로만 검출됐다(가까운데도 near가 아닌 이유는 확인하지 않음). near 적중이 없어 `tentative`로 남았고, 기억의 keep-out은 확정 트랙만 쓴다.
  - 상자에 막힌 뒤 PF σ가 0.3–0.6 m로 여러 번 튀었다(추정: 막혀 있는데 명령 기반 운동 모델은 움직였다고 봄). `uncertain` 둘러보기 110회, `refix` 14회를 하다 SIM 한도에 닿았다.
  - 거짓 확정 트랙 5개는 모두 이 구간(자기 위치 오차 0.1–0.34 m)에서 생겼다.
  - OFF도 같은 빨간 상자를 탐색 구간(t = 76.8–158.7 s)에 바퀴로 242 step 건드렸지만 빠져나와 완주했다. OFF의 keep-out은 near 2회 이상 본 군집뿐이라 이 상자는 OFF에서도 keep-out이 아니었다.
- **s163 두 조건, s161 OFF: 같은 자리의 목표를 탐색이 못 찾았다(기준선 탐색).** 목표가 관측점(x = −0.47)과 같은 행의 0.27 m 앞(−0.2, 0.75)에 있었다. 두 조건 모두 탐색 관측점에서 이 상자를 검출하지 못했다(추정: 너무 가까워 탐색 자세의 바닥 시야 아래). memory_v2는 같은 실패까지 SIM 106.6 s(OFF 161.3 s)가 걸렸다. s161 memory_v2는 출발 위치의 초기 둘러보기(0.68 m)에서 이 상자를 찾았다.

## 후속 (이번 비교에 넣지 않음, 새 사전 등록 필요)

1. 빈손 주행의 고정 신선도를 이동 거리가 아니라 추정 오차 증가 모형으로 정한다. 예: 명령 적분 이동 × 운동 잡음, 또는 도착 확인은 항상 1 pan.
2. `far_coarse`만 본 트랙도 σ만큼 부풀린 keep-out에 넣는다. 접촉·정지 감지(명령 대비 영상 변화 없음) 때는 후진하고 전체 둘러보기를 한다.
3. PR #230 권고: `ABSENT_MISSES` 카운터를 존재 확률(persistence filter 수식)로 바꾸고, 트랙에 지도 개체(`location_ref`)를 붙인다.
4. PR #227 비전 검출기가 나오면 `GeometricLandmarkProvider(detector=...)`로 바꿔 태그 없는 지도에서 다시 비교한다.

## 검증

- 테스트: `.venv-sim-worker-mac/bin/python -m pytest tests/test_owncam_memory.py tests/test_simulation_workflow_manager.py tests/test_m1_owncam.py tests/test_owncam_localizer.py` → 107 passed(360 subtests). 기억 핵심 태그 비의존(`test_memory_core_is_tag_free`), 태그 없는 지도에서 동작, 목록 해시가 태그와 무관, 동결 M1 파일 해시 불변을 검사한다.
- test 실행: `test_queue_used.sh`(seed마다 두 조건 동시) → `launch_episode.sh` → `scripts/run_m1_owncam_memory.py --split test --frozen frozen_source.json`. 러너가 깨끗한 트리·동결 SHA 이후 변경 없음·파일 해시 일치를 확인하고 시작했다. 12회 모두 `result.json`이 있다(인프라 실패 0, 재실행 0).
- TensorBoard: 스냅샷 `outputs/tensorboard/0926-zone-owncam-memory`(16 run, 변환 실패 0, `collection.json` sha256 `cd6928e7…`). EventAccumulator 128값이 `results.json`과 같다(`tensorboard_verify.json`). 공용 서버(PID 9291, 재시작 안 함)가 16 run을 읽는다. `outputs/tensorboard-view.json`에 `zone_owncam_memory_20260926` 키만 추가했다.
- 원본: `/Users/changmin/projects/ugrp/outputs/owncam-memory-20260926/{dev-a1,dev-a2,test-a1,logs}`(로컬만, 원격 백업 아님). 파일 해시는 `raw_index.json`에 있다.
- 기록 파일: `results.json`(sha256 `fb1cc073…`), `raw_index.json`(`87360e0c…`), `frozen_source.json`(`277149c9…`).

## 참고 자료

설계 문서 4절과 같은 목록이다. A1–A3 때 추가한 것:

- 논문(PR #230 참고 목록에서 확인 수준과 함께 옮김): Dellaert·Fox·Burgard·Thrun, "Monte Carlo Localization for Mobile Robots," ICRA 1999, https://doi.org/10.1109/ROBOT.1999.772544 / Rosen·Mason·Leonard, "Towards Lifelong Feature-Based Mapping in Semi-Static Environments," ICRA 2016, https://doi.org/10.1109/ICRA.2016.7487237 / Wong·Lozano-Pérez·Kaelbling, "Not seeing is also believing: Combining object and metric spatial information," ICRA 2014, http://hdl.handle.net/1721.1/100724 / Liu·Guo·Warke 외, "DynaMem: Online Dynamic Spatio-Semantic Memory for Open World Mobile Manipulation," 2024, https://arxiv.org/abs/2411.04999 / Burgard·Fox·Thrun, "Active Mobile Robot Localization," IJCAI 1997, http://ijcai.org/Proceedings/97-2/Papers/080.pdf.
- 공개 코드: filterpy 1.4.5(MIT) `predict`/`update` 적응, PythonRobotics `b2020cd`(MIT) EKF-SLAM 대응 적응, OpenCV 5.0.0(Apache-2.0) `cv2.fisheye.distortPoints`. `cv2.findContours`/`approxPolyDP`는 표식 목록에 검토 후 기각.
- 저장소 안: PR #230 `docs/design/2026-09-26-memory-literature.md`, PR #210 `docs/design/2026-09-26-markerless-localization-and-memory.md` 4(b), PR #227 `experiments/2026-09-26-vision-loc/maps/zone_wide_door_walls_v3_notags.json`, PR #208 `experiments/2026-09-26-zone-env-v3/README.md`(v3 게이트 실패)·`maps/zones/zone_wide_door_tags_v3.json`, #201 `harness/m1_owncam_delivery.py`·`scripts/run_m1_owncam.py`·`calibration_m1_dev.json`, `harness/owncam_localizer.py`, `harness/wall_tags.py`, `harness/zone_color_boxes.py`, `harness/visual_arm.py`, `harness/owncam_drive_v2.py`.
- 문서: `docs/tensorboard.md`, `docs/execution_versioning.md`.
