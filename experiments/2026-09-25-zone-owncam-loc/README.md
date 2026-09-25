# 2026-09-25 구역 지도 자기 카메라 위치 추정 (M1 1단계)

- 작업: Claude, 브랜치 `claude/zone-owncam-loc`, PR #177(draft). 기반은 `origin/claude/zone-hard-routes` `4789d93`(PR #173, 미병합)이다.
- 목표: 로봇 1대가 **자기 wrist RGB, 정적 지도, 자기 발행 명령**만으로 zone_wide_door 위 자기 위치를 추정하는 것. M1(청록 상자 1개를 문 너머로 배달)의 1단계다.
- 설계 문서: [docs/zone_owncam_localization.md](../../docs/zone_owncam_localization.md)

## 결론
- 둘러보기 자세(search −21°, 상자 없는 수평 carry +0.5°)에서 **test 문 근처 오차는 중앙값 1.9 cm / 0.19°, p90 5.8 cm / 0.48°**다(사후 look 모드만). 태그 가시율은 92–99%다.
- **사전 등록 게이트는 dev·test 모두 G1·G3 실패, G2만 통과**다. 원인은 상자를 든 carry 자세 두 가지(`carry_level`, 교사 low carry)에서 태그가 **한 번도 보이지 않은 것**(가시율 0%)이다. 이때 위치는 명령 적분만으로 추정되어 문 근처 오차가 16–35 cm까지 커졌다.
- 수평 carry(`CARRY_POSE`)에서는 들고 있는 청록 상자가 영상 아래 약 60%를 가린다. 그런데 0.05 m 높이의 벽 태그는 수평선 아래에 보이므로 전부 가려진다. 좌우 pan을 해도 상자가 카메라와 같이 돌아서 효과가 없다.
- **사후 진단(사전 등록 아님, test 채점 뒤 1회)**: 상자를 든 채 손목(서보 3)을 **20° 숙이면** 가시율이 97%이고 오차는 p50 2.7 cm, p90 5.4 cm다(문 근처 p50 3.0 cm, p90 5.8 cm). 상자는 떨어지지 않았다(z 0.17 m). 30° 숙이면 상자가 다시 시야를 가려 가시율이 7%다. 에피소드 1개(dev 경로)만 본 결과다.
- M1에 필요한 조건: 운반 중 20° 숙인 carry 자세를 쓰거나, 벽 태그를 카메라(0.21–0.23 m)보다 높이 둔다(`_tags_v2`, 가시성 연구의 권장값을 반영).

## 조건
| 항목 | 값 |
|---|---|
| 지도 | `zone_wide_door_tags_v1`. static_map `238edb67…`, base `zone_wide_door` 파일 `a4d2c03d…` 그대로, 태그 70개 |
| 장면 | `TaggedZoneScene`, `local_contact_fine`, weld OFF, timestep 0.00025 s(모든 raw `manifest.json`의 `timestep_s`와 `scene.xml`의 `timestep` 값; 2026-09-26 정정, 이전 표기 0.002 s는 오기), 동기 SIM |
| 카메라 | `robot_cam` 640×480 raw 어안(JPEG q90). TOP·nav_cam은 입력으로 쓰지 않음 |
| 외부 파라미터 | `harness.visual_arm.camera_extrinsics(발행 PWM)`. 측정 관절은 쓰지 않음 |
| 주행 | 정답 교사(`scripts/zone_teacher.py`, 읽기 전용 재사용). 0.25 s마다 촬영, 정지점마다 pan 1500→1900→1100→1500 |
| 기록 소스 | 16 에피소드 모두 `4af81db`의 기록 코드. 기록 관련 파일(`sim/`, `maps/`, 기록기, `zone_teacher`)은 `4f7601e`까지 동일(`git diff` 없음) |
| 환경 | Python 3.12.13, MuJoCo 3.12.0, OpenCV 5.0.0, numpy 2.5.2, macOS 27.2 arm64 |
| 동시 실행 | 기록 프로세스 최대 2개 |

- 기록기 `code.dirty=true`에 대하여: 기록기는 에피소드 **종료 시점**의 HEAD와 git 상태를 적었다. 그 사이 같은 worktree에서 기록기가 import하지 않는 localizer·테스트를 수정·커밋했기 때문에 dirty로 나왔다. 이후 기록기를 고쳐 시작 시점에 적도록 했다. 사후 2개(`558b4c6`)는 clean이다.
- 부하 평균은 에피소드마다 시작·끝 값을 `raw_index.json`과 각 `manifest.json`에 적었다. 공용 Mac에서 다른 에이전트 SIM 4–5개가 함께 돌아 1분 평균이 12–175였다.
- 첫 코호트는 스레드 제한 없이 시작했다. 22:4x 코디네이터 지시 이후의 실행(사후 기록, SIM 테스트, 오프라인 평가)은 `OMP/OPENBLAS/VECLIB/MKL_NUM_THREADS=1`로 했다. 동기 SIM이므로 결과는 부하와 무관하다.

## 분할과 사전 고정
- `episodes.json`: dev는 seed 11·12, test는 seed 21·22다. 각 seed마다 모드 4개(teacher_carry, look_search, look_level, carry_level)이고, 출발 행과 목표 구역이 다르다. 기록 전에 커밋했다(`4af81db`).
- `thresholds.json`: 위치 추정 결과가 나오기 전에 커밋했다(`13c64c4`). 문 영역은 정답 기준 |x−2.2| ≤ 0.6 m, |y−0.05| ≤ 0.5 m이며 평가에서만 쓴다.
  - G1: 문 근처 정지 둘러보기 p90 < 5 cm / 5°
  - G2: 문 근처 전체 p50 < 5 cm / 5°
  - G3: 문 근처 전체 p90 < 10 cm / 10°
  - 모드: look_search, look_level, carry_level
- 보정과 조정은 dev만 사용했다. 약 25개 변형, 필터 seed 2개, look 에피소드 4개로 골랐다. 그 뒤 `calibration_frozen.json`과 코드를 `efdf5f4`로 고정했고, test는 그다음에 한 번만 detect → localize → score했다.
- 고정 후 다시 조정하지 않았다.

## 결과 — 사전 등록 게이트 (`metrics_dev.json`, `metrics_test.json`)
| 게이트 | dev | test |
|---|---|---|
| G1 정지 둘러보기 p90 | 16.0 cm / 3.07° **실패** | 9.1 cm / 2.68° **실패** |
| G2 전체 p50 | 3.2 cm / 0.46° 통과 | 3.2 cm / 0.27° 통과 |
| G3 전체 p90 | 16.5 cm / 4.37° **실패** | 26.2 cm / 3.53° **실패** |
| I1 교사 low carry p50 (참고용) | 20.1 cm / 5.07° | 19.9 cm / 4.64° |

초기화 누락은 0 프레임이다. 모든 선택 프레임에서 필터가 초기화되어 있었다.

## 결과 — 사후 분해 (사전 등록 아님, `posthoc_look_only_and_visibility.json`)
look 모드만 선택(carry_level 제외):

| | G1 p90 | G2 p50 | G3 p90 |
|---|---|---|---|
| dev | 5.4 cm / 0.82° | 2.8 cm / 0.32° | 5.5 cm / 0.88° |
| test | 4.5 cm / 0.43° | 1.9 cm / 0.19° | 5.8 cm / 0.48° |

자세별 태그 가시율(태그 1개 이상 검출된 프레임 비율, 평균 태그 수):

| 자세 | dev | test |
|---|---|---|
| search(−21°, 주행·정지) | 0.93 (2.9) | 0.92 (3.0) |
| 수평, 상자 없음 | 0.98 (3.4) | 0.99 (3.4) |
| **수평 carry, 상자 있음** | **0.00** | **0.00** |
| **교사 low carry(−57°)** | **0.00** | **0.00** |
| 파지 중 | 0.08 | 0.04 |
| 내려놓기 | 0.63 | 0.61 |
| 팔 이동 중 | 0.77 | 0.81 |
| 사후: carry 20° 숙임(상자 있음) | 0.97(주행)·1.0(정지) | – |
| 사후: carry 30° 숙임 | 0.07 | – |

- 마지막 태그 이후 시간별 오차(dev): 2 s 이내 p90 6 cm, 2–10 s p90 5 cm/2°, 10–30 s p90 20 cm, 30 s 이상 p90 28 cm/7°.
- 명령 적분의 한계: 10 s 추측 항법 오차는 look 자세에서 p50 4 cm / 3°, carry에서 p50 10 cm / 15°다(단일 이득 모델 기준).
  - 이 차이 때문에 **자기 명령만으로 판정하는 적재 상태별 운동 모델**을 추가했다. 판정 기준은 파지 높이에서 집게를 닫으라고 명령했는지 여부다. 회전 이득은 적재 0.74, 무적재 1.49다.

## 보정값 (`calibration_frozen.json`, dev 교사 로그로 오프라인 적합)
- 무적재 운동: 이득 대각 (1.33, 1.02, 1.49), τ 0.30 s. 적재: (1.35, 0.91, 0.74), τ 0.44 s. 잡음은 적합값의 2배로 둔다.
- 측정
  - FK 카메라와 실제 렌더 카메라의 차이는 pitch 0.97°, 높이 1.7 mm다. 방위각 오차는 평균 −0.04° ± 0.06°, 고도각 오차는 −1.07° ± 0.03°이며 둘 다 편향으로 흡수했다.
  - 거리 log 편향은 −0.016 + 0.018·r이고, 보정 후 산포는 1.1%다.
  - 채택 σ 하한: 방위각 0.5°, 고도각 2°, 거리 3%, 면 법선 30°. 한 프레임의 태그 수로 우도를 완화한다(`tag_temper` 1). roughening은 (1 cm, 1 cm, 0.3°)다.

## 테스트
- `tests/test_owncam_localizer.py` 12개, 하위 48개.
  - import 허용 목록과 시뮬레이터 상태 이름 금지.
  - `mujoco` import를 막은 subprocess에서의 합성 위치 추정.
  - `eval_only` 열기 차단.
  - PnP·어안 역변환·합성 문 통과 추적 < 3 cm.
  - 기존 지도 SHA 고정, 태그 지도 정의와 base 해시 일치, 태그가 벽면 안·벽 높이 안에 있고 다른 벽과 겹치지 않음, 문기둥 태그가 양면에 있음.
- `tests/test_zone_landmarks_sim.py` 2개.
  - 같은 명령에서 base 장면과 태그 장면의 `qpos` 궤적이 비트 단위로 같다. 태그 geom은 모두 contype·conaffinity 0이다.
  - wrist 렌더에서 태그 2개 이상을 검출하고, 방위각 오차 < 0.5°, 거리 오차 < 10%다.
- 함께 돌린 기존 테스트: `test_simulation_workflow_manager`(카탈로그 29), `test_zone_hard_routes`, `test_zone_dispatch`, `test_visual_arm_sim`. 합계 72 통과.

## 원본 (로컬 전용, gitignore, 원격 백업 없음)
- `/Users/changmin/projects/ugrp/outputs/owncam-loc-20260925/raw/<episode>/` (18개).
  - `inputs/`: 프레임 JPEG, 명령, 프레임 목록.
  - `eval_only/`: 정답 궤적과 프레임 정답.
  - `derived/`: 검출, 추정.
  - `scene.xml`, `manifest.json`.
- 에피소드별 manifest·프레임 묶음·derived 해시, SHA, 부하 평균은 `raw_index.json`에 있다.
- 기록 로그: 같은 폴더의 `rec-dev.log`, `rec-test.log`, `rec-posthoc20.log`, `rec-posthoc30.log`, `launch_load.txt`.
- 커밋한 기록 파일의 SHA-256
  - `thresholds.json` `9ed4211d…`
  - `calibration_frozen.json` `a9df9b7c…`
  - `metrics_dev.json` `217d20a6…`
  - `metrics_test.json` `a4759dba…`
  - `results.json`은 요약본이다.

## 하지 못한 것과 한계
- TensorBoard 스냅샷은 만들지 않았다(최근 구역 기록과 같이 코디네이터 담당). 변환·화면 확인은 미완료다.
- 실행 번들 ID는 쓰지 않았다(dispatch adapter 변경 없음, 오프라인 기록·평가만 함).
- 교사가 주행한 오프라인 평가다. 추정값으로 로봇을 조종한 폐루프 결과가 아니다.
  - 20° 숙임 carry는 에피소드 1개뿐이고 test가 없다.
  - two_doors·corridor 태그 지도는 만들었지만 기록하지 않았다.
- 조명·질감·해상도가 고정된 시뮬레이터 영상이다. 실물 어안 잡음, 모션 블러, 태그 인쇄 오차는 없다.
- 태그 배치는 기본값(0.05 m 높이, 0.072 m)이다. wrist 가시성 연구의 권장값은 아직 반영하지 않았다.

## 재현
```sh
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1
PY=.venv-sim-worker-mac/bin/python; X=experiments/2026-09-25-zone-owncam-loc; D=<raw root>
$PY scripts/record_owncam_localization.py --episodes $X/episodes.json --output $D   # 에피소드별 --only 가능
$PY scripts/eval_owncam_localization.py detect   --data $D --episodes <ids>
$PY scripts/eval_owncam_localization.py localize --data $D --episodes <ids> --calibration $X/calibration_frozen.json
$PY scripts/eval_owncam_localization.py score    --data $D --episodes <ids> --thresholds $X/thresholds.json --output m.json
```
조정 보조 스크립트 `tune_dev.py`, `breakdown.py`, `dead_reckoning_by_posture.py`는 dev 분석용이다.
