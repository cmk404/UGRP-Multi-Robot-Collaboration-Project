# 2026-09-25 자기 카메라 폐루프 문 통과 (M1 2단계, PR #178)

## 질문
로봇이 실행 중에 정답(GT)을 전혀 받지 않고 **자기 손목 fisheye RGB + 정적 태그 지도 + 자기 발행 명령 이력**만으로
`zone_wide_door_tags_v2`의 문(`door_1`, x=2.2)을 지나 출구 지점 W=(2.65, 0.05)에 도착해 스스로 도착을 선언할 수 있는가.
조건은 두 가지다. 상자 없이(nobox), 그리고 공용 자세 `CARRY_POSTURE`로 상자를 든 채(box, weld OFF).

## 설정
- 지도: `maps/zones/zone_wide_door_tags_v2.json`. 파일 sha256은 f9b0ef0d…, 태그 80개, 문기둥 2개이며 태그 높이는 0.15/0.25 m다. v1 파일은 바이트 그대로 유지했고, 물리는 base 지도와 비트 단위로 같다(`tests/test_zone_landmarks_sim.py`).
- 학생 입력: 자기 발행 명령, 자기 `robot_cam` JPEG q90 640x480, 정적 지도, 교정값(`calibration_loop.json`), 고정 pickup 격자 keep-out이다. `nav_cam`·TOP 카메라와 실시간 자세는 쓰지 않는다. 경계 테스트는 `tests/test_owncam_localizer.py`다.
- 학생 동작(`harness/owncam_drive.py`): 입자 필터 추정 → `harness.map_goto.plan_path` A*(추정 시작점, 1 s마다 재계획) → mecanum 추종(동쪽 heading 유지) → 멈춰 둘러보기(`LOOK_P20` + pan sweep).
- 상자 조건: GT 교사가 상자로 가서 집어 들고 carry 단계에 들어가면 학생이 넘겨받는다. 교사 구간 SIM 시간은 따로 적고 학생 성공에 넣지 않는다.
- 공용 자세: `CARRY_POSTURE {1:1500,3:777,4:2053,5:1646,6:1500}`, `LOOK_P20 {3:1072,4:2400,5:1482}`. PR #176이 2026-09-25T16:25Z 코멘트로 확정했다(#176은 7ee43a9로 병합됨).
- 게이트(사전 등록 `prereg.json`, sha256 38873a0d…):
  - R1: 도착 선언 시점에 GT와 W의 거리 ≤ 0.10 m.
  - R2: 학생 구간 벽 접촉 0.
  - R3: 문 영역(|x−2.2|≤0.6, |y−0.05|≤0.5)에서 추정 위치 오차 p90 < 0.06 m이고, 미초기화 프레임이 없다.
  - R4(box): 상자 z > 0.045 m를 끝까지 유지한다.
  - 학생 시간 한도는 SIM 240 s다.
- seed: dev 31/32/33, test 41/42/43, 조건마다 3개. 동기 SIM(timestep 0.25 ms), contact `local_contact_fine`, weld OFF다.
- 환경: macOS arm64, Python 3.12.13, MuJoCo 3.12.0, OpenCV 5.0.0, numpy 2.5.2. `OMP/OPENBLAS/VECLIB/MKL_NUM_THREADS=1`로 두었고 동시 실행은 2개 이하다. 실행별 부하 평균은 `results.json`의 `load_average`와 raw의 `launch_load.txt`에 있다.

## dev 수정 이력 (test 전, `prereg_amendments.json`)
prereg.json은 바꾸지 않았다. 게이트·seed·중단 규칙도 그대로다. dev 시도는 모두 보존했다.

| 시도 | 커밋 | 바꾼 것 | 이유(dev, GT는 오프라인 분석에만 사용) |
|---|---|---|---|
| a1 | 155cb71 | v1 교정 | box R3 7.1 cm로 실패 |
| a2 | a5559e7 | 짐을 든 상태/빈 상태 고각 편향(−2.5°/−1.07°) | 상자 무게로 팔이 FK보다 처짐. 그래도 box R3 7.5 cm |
| a3 | d3466db | 드라이버 전용 넓은 둘러보기 ±48°(`WIDE_LOOK_PANS`, 공용 `LOOK_PANS`는 유지) | 문 안에서는 3.4 m 떨어진 동쪽 벽만 보여 y와 yaw가 서로 바뀜. 북쪽 벽 태그(1.4–2 m)가 y를 고정함. box s33은 240 s 초과 |
| a4 (고정) | 9361a8d | 움직임 이득 재적합(loaded τ 1.0 s), 짐을 든 상태에서는 0.35 m 이동마다 둘러보기 | `CARRY_POSTURE`는 설계상 주행 중 태그를 못 봄. 3 s 무태그 규칙이 약 3 s마다 둘러보기를 걸었음 |

고정 소스: `frozen_source.json`(9361a8d, 실행 파일 11개 sha256). test는 기록만 추가한 HEAD 521ae7b에서 돌았고, 실행 파일 바이트는 같다.

## 결과 (고정 소스 9361a8d)
| 분할/조건 | 성공 | R1 GT 거리 (m) | R3 p90 (m) | 학생 SIM (s) | 둘러보기 | 명령 수 | 태그 가시율 |
|---|---|---|---|---|---|---|---|
| dev nobox | **3/3** | 0.019 / 0.030 / 0.021 | 0.025 / 0.036 / 0.023 | 76.5 / 83.1 / 120.4 | 5 / 5 / 7 | 1065 / 1131 / 1624 | 0.87–0.91 |
| dev box | **3/3** | 0.054 / 0.059 / 0.048 | 0.039 / 0.053 / 0.058 | 75.3 / 104.4 / 187.5 | 6 / 8 / 15 | 1164 / 1589 / 2825 | 0.67–0.73 |
| **test nobox** | **3/3** | 0.022 / 0.023 / 0.031 | 0.048 / 0.029 / 0.024 | 85.1 / 111.1 / 76.3 | 5 / 6 / 5 | 1151 / 1471 / 1063 | 0.88–0.89 |
| **test box** | **1/3** | 0.042 / 0.058 / 0.743 | 0.043 / **0.080** / 0.023 | 74.0 / 102.2 / **240.1** | 6 / 8 / 21 | 1151 / 1565 / 3637 | 0.63–0.72 |

(seed 순서: dev 31/32/33, test 41/42/43)

- 모든 실행에서 벽 접촉, 다른 로봇 접촉, 다른 상자 접촉은 0이다. box 조건의 상자 최저 z는 0.085 m로, 끝까지 들고 있었다(weld OFF).
- 교사 구간 SIM은 dev 20.0–24.6 s, test 23.3–34.1 s이며 학생 성공에 넣지 않았다.
- test box 실패:
  - s42는 R3에서 실패했다(p90 8.0 cm). 도착 자체는 했고 R1은 5.8 cm다.
  - s43은 240 s 한도를 넘었다. 교사가 y≈−2.45에서 상자를 잡아 가장 긴 경로가 됐다. 둘러보기 21회, 그중 refix가 7회다.
- 사전 등록 규칙에 따라 test는 한 번만 실행했고 재실행하지 않았다.

## 사후 분석 (post hoc, 사전 등록 분석 아님)
- `posthoc_stop_coast.py`: dev 재적합(`fit_motion_rowscale.py`)은 주행 시작부터 끝까지의 프레임만 썼다. 그래서 loaded τ=1.0 s 모델은 바퀴가 멈춘 뒤에도 추정을 앞으로 계속 민다. 멈춤 뒤 1.2 s를 포함하면 전방 과대 예측이 구간마다 +3.2 ~ +6.1 cm다(test). dev-a4에서도 +4.1 ~ +4.6 cm였는데, 그때는 둘러보기가 이를 바로잡았다.
  - test s42: `look_arm` 동안 dx가 +4.3 cm에서 +11.4 cm로 커졌다.
- s43: 북쪽으로 옆걸음하는 구간에서 추정 std가 약 0.05 m로 둘러보기 임계값(0.05)과 거의 같았다. 그래서 둘러보기가 "fix 안 됨"(0.050–0.056)으로 끝나 refix가 반복됐다.
- 다음 코호트에서 할 일(새 seed가 필요함):
  1. 멈춤 동역학을 따로 모델링한다(hold면 즉시 감속하거나 가속/감속 τ를 분리). 적합 구간에 멈춤 뒤를 포함한다.
  2. fix 임계값을 정상 상태 std 바닥보다 확실히 위에 두거나 상대 기준으로 바꾼다.
  3. 짐을 든 상태의 옆걸음 속도를 올리거나, 문 앞까지는 대각선 경로를 쓴다.
- #177 v1 녹화의 오프라인 재분석은 이 코호트에 포함하지 않는다(post hoc).

## 파일
- `prereg.json`, `prereg_amendments.json`, `frozen_source.json`, `calibration_loop.json`
- `fit_motion_rowscale.py`(dev 재적합), `posthoc_stop_coast.py`(사후 분석), `build_results.py` → `results.json`(모든 시도 22개, dev a1–a4와 test), `raw_index.json`(실행 디렉터리별 파일 수·바이트·파일 목록 sha256)
- raw: `/Users/changmin/projects/ugrp/outputs/owncam-loop-20260925/{dev-a1,dev-a2,dev-a3,dev-a4,test}/<episode>/` (`inputs/`는 학생 입력, `frames/`는 학생이 본 JPEG, `eval_only/`는 GT·접촉·교사 이벤트로 평가 전용, `manifest.json`, `result.json`). **로컬 보관만 했고 원격 백업은 없다.**
- 실행: `scripts/run_owncam_closed_loop.py --prereg experiments/2026-09-25-zone-owncam-loop/prereg.json --only <ids> --output <dir>`(`ugrp_session.py run`으로 감쌈)
- TensorBoard 변환·표시는 코디네이터가 맡았다.
