# 2026-09-26 M1 자기 카메라 배달 (pose stub 없음)

**질문.** 로봇 한 대가 실행 중 pose stub 없이 다음을 해낼 수 있는가?
1. 자기 손목 RGB로 픽업 구역에서 cyan 상자를 찾는다.
2. 손목 스킬로 파지한다.
3. 자기 카메라 추정으로 문을 통과해 운반한다.
4. 주문 슬롯에 놓는다.
5. 자기 카메라로 다시 보고 배치를 확인한다.

**입력.** 자기 `robot_cam` JPEG, 자기 발행 명령, 정적 tagged map v2, 고정 교정, 주문서(색·픽업 구역 행·목적 슬롯)만 쓴다. 상자 위치는 주문서에 없다. GT는 `eval_only/`에만 기록한다.

## 상태 (2026-09-26 기준)
- **test(101–106)는 실행하지 않았다.** 파지 스킬 v5가 축 정렬(yaw 0) 상자를 정면에서 볼 때 면 추정이 수렴하지 않는다.
  - dev에서 3/3이 파지 단계에서 실패했다(s91·s92·s93).
  - test 상자도 축 정렬이라, 지금 돌리면 1회용 코호트를 알려진 차단 요인에 소모하게 된다.
  - 스킬 에이전트가 `wrist_zone_skill_v6`(정면 0° 포함 면 추정, 공개 re-anchor hook, peer 회피 접근)를 만들고 있다. v6가 준비되면 amendment로 채택하고, 소스를 동결(`frozen_source.json`)한 뒤 test를 1회 실행한다.
- **dev 진단 성공 1건.** 상자를 +15° 돌린 dev 진단 에피소드 s94(dev-a5, `6352fde`)에서 전 과정을 한 번 완주했다. **M1 코호트 증거가 아니다.**
  - 경로: 탐색(오차 2.3 cm) → 자기 RGB 면 추정 파지 → 운반·문 통과(둘러보기 16회) → 배치(GT 슬롯 오차 2.3 / 1.3 cm) → 다시 보기 IN_SLOT.
  - 기록: 벽·peer·다른 상자 접촉 0, weld OFF, 자세 출처 `owncam_pf_v2:757f7f09`만, SIM 544 s.

## 사전 등록
- `prereg.json`(`969e03e`): seed, 주지표, 주장 규칙, 정지·재시도·제외 규칙을 **데이터 수집 전에 한 커밋으로** 넣었다(Codex #7).
  - dev: 91–93. test: 101–106, 동결 소스로 1회.
  - 주장 규칙: test m1_success ≥ 5/6이면서 false_success 0.
- `prereg_amendments.json`: test 전에만, dev 근거로 student(소스·교정·스킬)를 바꾸고 dev 진단 에피소드를 추가한다. seed·지표·규칙은 바꾸지 않는다. 러너는 적용한 amendment ID와 파일 sha256을 각 manifest에 남긴다.

| amendment | 근거 | 변경 |
|---|---|---|
| A1 | dev-a1 s91: 파지 중 추측 항법 0.95 m 폭주, 단일 태그로는 reset 불가, 엄격 re-anchor 거부, v4 지도 fallback | 스킬 v5(`mode='m1'`, 자기 RGB bay 주문서), `calibration_m1_dev.json`(settled-frame kidnap reset과 'fine' 운동 프로필, s91 dev 적합), 조작 뒤 필수 둘러보기, 자기 pan probe re-anchor |
| A2 | dev-a2 s91·s92: v5 정면 면 추정 실패, s91 peer 접촉 226 step | bay 반폭 x 0.15 m(접근점이 목표 x − 0.40 m), dev 전용 회전 상자 진단 s94(+15°)·s95(−20°) |
| A3 | dev-a3 s93·s94: 러너가 스킬에 robot_id를 넘기지 않음(r2·r3 거부) | 스킬에 에피소드 robot_id 전달 |
| A4 | dev-a4 s94: 문 통과 뒤 스킬 자체 mecanum 주행의 yaw 표류(최대 11°), yaw 게이트에 19회 걸림, SIM_LIMIT | 운반 구간 드라이버(loop v2 plant)가 사전 배치 목표까지 주행. 스킬은 잔차만 보정 |

## dev 결과 (전부 보고, `results.json`)
| 시도 | 에피소드 | 결과 | 도달 단계 |
|---|---|---|---|
| dev-a1 `969e03e` | s91 | CARRY_REANCHOR_UNCONFIRMED | 파지·자세 |
| dev-a2 `32c7069` | s91, s92 | GRASP_FACE_UNOBSERVABLE_AFTER_RELOOKS ×2 | 탐색(오차 3.1 / 2.3 cm) |
| dev-a3 `ce33ffd` | s93, s94 | OBSERVATION_REJECTED(러너 robot_id 버그) ×2 | 탐색 |
| dev-a4 `ea45e3d` | s93 / s94 / s95 | GRASP_TARGET_NOT_VISIBLE / SIM_LIMIT / GRASP_TARGET_NOT_VISIBLE | 탐색 / 운반·문 통과 / 탐색 |
| dev-a5 `6352fde` | s94(+15°, 진단) | **OWN_RGB_PLACEMENT_IN_SLOT, m1 검사 전부 통과** | 전 단계 |

## 관찰 요약 (GT는 오프라인 평가에만 사용)
- **파지 단계 plant.** 팔을 내린 상태에서 전진 반응이 크게 늦다. 0.08×0.3 s 명령이 0.66 cm만 움직였고(명령 2.4 cm), 적합값은 τ ≈ 1.3 s다. 주행 중 입자별 미끄럼 척도(1.27)가 이를 더 부풀린다. A1의 'fine' 프로필을 쓰면 s91 재생에서 파지 구간 오차가 3 cm다(in-sample).
- **kidnap.** 정상 구간에서 태그당 바닥(−9)에 붙은 프레임은 전부 자기 서보 명령 직후(0 s)에 찍힌 것이었다. 명령 0.3 s 이후 프레임 521개에서는 0개였다. 그래서 settled 프레임 2회 연속을 조건으로 쓴다. s91에서 필요한 한 번만 발동했다.
- **v5 정면 면 추정.** 같은 floor-cuboid 적합을 오프라인으로 돌리면, 파지 프레임 중 |yaw 오차| > 7°가 35%(s91), 53%(s92)다. v5의 inlier 규칙은 60%가 필요하다. #181에 보고했다.
- **운반 중 yaw.** 운반 구간 드라이버의 |yaw 오차| p90은 1.0°, err/σ p90은 1.02였다. 스킬 자체 mecanum 주행은 p90 5.4°, err/σ 2.1이었다.
- **re-anchor.** 짐을 든 채 둘러본 뒤 dev-a4에서는 엄격 비교가 매번 실패했고, 자기 pan probe가 매번 통과했다. dev-a5에서는 엄격 비교가 20회 모두 통과했다.

## 검증 범위
- 단위 테스트: `tests/test_m1_owncam.py`(계약, 경계, 자세 한계, 제어기, 교정 추가의 v2 동일성)와 `test_owncam_localizer.py`, `test_wrist_zone_skill_v5.py`가 통과했다.
- M1 성공은 test 코호트에서만 주장한다. dev·진단 결과는 합산하지 않는다. #181 v5 코호트(541–548)와 loop v1/v2 결과도 M1이 아니다.
- 실행 환경: 동기 SIM(timestep 0.00025 s), 스레드 1(`OMP/OPENBLAS/VECLIB/MKL=1`), 동시 실행 최대 2개. 부하 평균은 `outputs/.../launch_load.txt`와 각 manifest에 있다.

## 파일
- 소스: `harness/m1_owncam_delivery.py`, `harness/owncam_pose_source.py`, `harness/m1_owncam_contract.py`, `harness/owncam_localizer.py`(선택 키 추가), `scripts/run_m1_owncam.py`.
- 교정: `calibration_m1_dev.json`(`make_calibration_m1.py`, `fit_fine_motion.py` → `fine_motion_fit_dev.json`).
- 결과: `build_results.py` → `results.json`, `raw_index.json`(파일별 SHA-256).
- 원본: `/Users/changmin/projects/ugrp/outputs/m1-owncam-20260926/`. 로컬에만 있고 원격 백업이 아니다.
- TensorBoard 변환은 별도 작업에서 한다.
