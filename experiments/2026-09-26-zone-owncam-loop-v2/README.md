# 2026-09-26 자기 카메라 폐루프 v2: 상자 운반 문 통과 (새 seed 코호트)

## 질문
1차 코호트(`experiments/2026-09-25-zone-owncam-loop`)는 test box가 1/3이었다. 사후 분석에서 원인 두 가지가 나왔다.
- 멈춘 뒤 추정이 3–6 cm 앞으로 미끄러졌다.
- std가 둘러보기 임계값 근처에 머물러 refix가 반복됐다.

이를 dev 자료만으로 고친 뒤, **새 seed**에서 학생이 실행 중 GT 없이 자기 추정만으로 상자를 들고 문을 지나는지 확인한다.

## 사전 등록 (#7 구분)
- **데이터 수집 전 등록**: 커밋 `97ee2b0` 하나에 소스, 교정(`calibration_loop_v2.json`), `prereg.json`이 함께 들어 있다. `prereg.json`에는 dev 31–33과 test 61–66의 seed, 게이트 R1–R4, 코호트 게이트, 중단·재실행·제외 규칙이 있다. v2의 dev·test 실행은 모두 이 커밋 뒤에 시작했다.
- **test 채점 전 동결**: `frozen_source.json`(`97ee2b0`, 파일 12개 sha256)을 커밋 `b3edddf`에 기록했다. 이 커밋은 기록만 추가하며, 그 다음에 test를 1회 실행했다. dev 재실행은 없었다(dev 1회차로 3/3).
- **게이트**(1차와 같음):
  - R1: 도착 선언 시점에 GT와 W=(2.65, 0.05)의 거리 ≤ 0.10 m.
  - R2: 학생 구간(상자 포함) 벽 접촉 0. 매 physics step 검사.
  - R3: 문 영역에서 추정 위치 오차 p90 < 0.06 m.
  - R4: 상자 z > 0.045 m를 끝까지 유지.
- **코호트 게이트**: C1은 test 6개 중 ≥5개 통과, C2는 벽 접촉 0이고 낙하 0. 이 게이트로 M1 통합 진행 여부를 정한다.

## 바꾼 것 (모두 dev 자료로 적합, GT는 오프라인 적합에만 사용)
| 항목 | 내용 | 근거 |
|---|---|---|
| 멈춤 동역학 | `motion_loaded`에 `tau_stop_s` 0.05 s를 더함(바퀴 명령 0/만료 시). 주행 τ 0.8 s, 전진·횡 이득 행 재적합 | `fit_stop_dynamics.py`: 주행 시작부터 다음 주행 시작까지(멈춤과 둘러보기 포함) 20 Hz GT 창 88개. 창 끝 전진 오차 평균 +0.2 cm, p90 0.9 cm(leave-one-run-out p90 0.9 cm). 1차의 τ 1.0 s 단일 모델은 멈춘 뒤 3–6 cm를 더 밀었음 |
| 짐 상태 카메라 보정 | `measurement_loaded.camera_correction`: 카메라 좌표계 δ (1.2, 4.7, −1.6) mm, ω_x 2.62°. 고정 고각 편향 −2.5°를 대체 | `fit_loaded_extrinsic.py`: 검출 7872개 GT 잔차. 고각 −2.74°를 0.00 ± 0.02°로 줄임. run별 제외 적합 ω_x 2.621–2.625° |
| 방위 척도 | `measurement_loaded.azimuth_scale` 1.00284 | 멈춘 둘러보기 프레임에서 관측 방위가 모델보다 약 0.3% 넓음. 빈 손 1.0043도 관측했으나 이번 코호트는 상자 조건만 보므로 적용하지 않음 |
| 둘러보기 정책 (`harness/owncam_drive_v2.py`, 상자 조건만) | fix는 std ≤ 0.06 m. "불확실" 발동은 std > 0.07 m이면서 마지막 둘러보기 뒤 0.10 m 이상 이동했을 때. refix는 연속 1회까지. 이동 기반 둘러보기는 0.5 m마다 | 1차 dev에서 짐 상태 둘러보기 직후 std p90이 0.048 m, 주행 중 p50이 0.039 m여서 0.05 m 단일 임계값에서 반복 발동했음 |

빈 손 조건의 동작·교정은 v1과 같다. `LOOK_PANS`, `CARRY_POSTURE`, `LOOK_P20` 같은 공용 상수도 그대로다. localizer의 새 파라미터는 모두 선택 항목이며, 없으면 v1과 같다(테스트로 확인).

## 결과 (소스 97ee2b0, `local_contact_fine`, weld OFF, 동기 SIM)
| 분할 | 성공 | R1 GT 거리 (cm) | R3 p90 (cm) | 학생 SIM (s) | 둘러보기 | refix | 명령 수 |
|---|---|---|---|---|---|---|---|
| dev 31/32/33 | **3/3** | 3.1 / 4.3 / 3.9 | 2.3 / 3.9 / 4.8 | 64.9 / 93.1 / 150.9 | 5 / 7 / 11 | 0 | 994 / 1410 / 2251 |
| **test 61–66** | **6/6** | 4.0 / 4.1 / 4.5 / 4.6 / 3.9 / 3.7 | 3.1 / 2.8 / 3.1 / 3.6 / 4.4 / 2.7 | 133.8 / 166.7 / 144.5 / 93.6 / 134.8 / 216.1 | 10 / 12 / 11 / 7 / 10 / 16 | 0 | 1421–3241 |

- **코호트 게이트 통과**: C1 6/6, C2 벽 접촉 0이고 상자 최저 z 0.0848 m 이상.
- 다른 로봇·다른 상자 접촉도 0이다. 태그 가시율은 0.59–0.70이다.
- 교사 파지 구간 SIM은 10.5–24.9 s이며, 학생 성공에 넣지 않았다.
- 1차 dev(`dev-a4`, 같은 seed 31–33)와 비교하면 R3 p90은 3.9/5.3/5.8 → 2.3/3.9/4.8 cm, 둘러보기는 6/8/15 → 5/7/11회, SIM은 75/104/188 → 65/93/151 s다.
- 가장 오래 걸린 것은 s66(216 s, 한도 240 s)이다. 긴 옆걸음 경로라 여유가 작다.
- 실행별 부하 평균은 `results.json`과 raw의 `launch_load.txt`에 있다. 스레드 상한은 1이고 동시 실행은 2개다.

## 한계와 범위
- 이 결과는 **교사가 집어 든 뒤의 운반 구간(문 통과 → W 도착 선언)**만 다룬다. 파지·배치·다시 보기를 포함한 M1이 아니다.
- 접촉은 매 physics step에서 로봇과 든 상자 geom과 벽의 쌍을 셌다. 접촉 힘과 관통 깊이는 기록하지 않았다.
- `local_contact_fine`의 grip creep 인공물(PR #181 보고, 2.1 mm/min)은 그대로 두었다. R4는 높이로만 판정한다.
- 빈 손 조건은 이번에 다시 시험하지 않았다(1차 test 3/3). 빈 손 방위 척도 1.0043은 기록만 했다.

## 파일
- `prereg.json`, `frozen_source.json`, `calibration_loop_v2.json`, `stop_fit_dev.json`, `loaded_extrinsic_dev.json`
- `fit_stop_dynamics.py`, `fit_loaded_extrinsic.py`(dev 적합)
- `build_results.py` → `results.json`, `raw_index.json`. 사전 등록 에피소드 전체를 대조하고, 결과가 없으면 `missing`으로 게이트를 막는다.
- raw: `/Users/changmin/projects/ugrp/outputs/owncam-loop-v2-20260926/{dev-a1,test}/<episode>/` — **로컬 보관만 했고 원격 백업은 없다.**
- 실행: 워크플로 `zone-owncam-loop-run`, `--prereg experiments/2026-09-26-zone-owncam-loop-v2/prereg.json`. prereg의 `student` 블록이 driver v2와 교정 파일을 고른다.
