# 2026-09-26 M1 자기 카메라 배달 (pose stub 없음)

**질문.** 로봇 한 대가 실행 중 pose stub 없이 다음을 해낼 수 있는가?
1. 자기 손목 RGB로 픽업 구역에서 cyan 상자를 찾는다.
2. 손목 스킬로 파지한다.
3. 자기 카메라 추정으로 문을 통과해 운반한다.
4. 주문 슬롯에 놓는다.
5. 자기 카메라로 다시 보고 배치를 확인한다.

**입력.** 자기 `robot_cam` JPEG, 자기 발행 명령, 정적 tagged map v2, 고정 교정, 주문서(색·픽업 구역 행·목적 슬롯)만 쓴다. 상자 위치는 주문서에 없다. GT는 `eval_only/`에만 기록한다.

## 상태 (2026-09-26 기준)
- **test(101–106)는 실행하지 않았고 동결도 하지 않았다.** A5(v6, `cargo_noslip_v1`)로 dev를 확인한 결과(dev-a6)는 s93·s95 모두 `GRASP_TARGET_NOT_VISIBLE`이었다.
  - 원인: 스킬 N7 approach 단계의 cyan 검출 임계값(min_saturation 65)이 서쪽 픽업 구역의 밝은 청색 바닥에서 상자를 놓치거나 바닥을 상자로 오검출한다. 기록 프레임을 150으로 재생하면 정상 검출된다.
  - #181에 보고했다. 1회용 test를 쓸지는 코디네이터가 결정한다.
- **dev 진단 성공 2건(같은 s94).** A5 조건(v6·`cargo_noslip_v1`·새 판정식)에서도 dev-a6 s94가 완주했다(아래 표). 첫 완주는 dev-a5 s94(+15°, `6352fde`)였다. 조건은 A5 이전(`local_contact_fine`, v5, 이전 판정식)이다. **M1 코호트 증거가 아니다.**
  - 경로: 탐색(오차 2.3 cm) → 자기 RGB 면 추정 파지 → 운반·문 통과(둘러보기 16회) → 배치(GT 슬롯 오차 2.3 / 1.3 cm) → 다시 보기 IN_SLOT.
  - 기록: 벽·peer·다른 상자 접촉 0, weld OFF, 자세 출처 `owncam_pf_v2:757f7f09`만, SIM 544 s.
- **접촉 프로필.** dev-a1–a5는 전부 `local_contact_fine`로 실행했다(등록 에피소드 기본값). A5 이후(dev-a6부터)는 `cargo_noslip_v1`이다(`noslip_iterations` 10 기록). 사용자 결정은 아직 대기 중이다(#181·#189).

## Codex 사전 검토(#201, `ea45e3d`) 반영: A5, `aa2dced`
| # | 지적 | 조치 |
|---|---|---|
| 1 Blocker | test가 기본 명령으로 실행되고 동결을 강제하지 않음 | 기본은 dev만 실행한다. `--split test`에는 `--frozen`이 필요하고 다음을 모두 요구한다(아니면 시작 전 거부): 깨끗한 트리, 동결 SHA 이후 소스 변경 없음(records-only 제외), 파일별 해시 일치, 같은 skill·교정·접촉 프로필. `make_frozen.py`가 동결 파일을 만든다 |
| 2 Blocker | `cargo_noslip_v1`이 M1 경로에 연결되지 않음 | `zone_cargo_contact.base_profile/apply`를 연결했다. 프로필 기록, 실제 `noslip_iterations`, 최종 XML 해시를 기록하고, 0이면 거부한다. 이전 dev는 `local_contact_fine`이었다고 명시했다 |
| 3 High | look-back 게이트가 실제 확인 프레임에 적용되지 않음 | 모든 확인 단계에서 현재 추정으로 게이트를 검사한다(3 s 이내 둘러보기, σ). 위반하면 다시 둘러보고, 3회 뒤에는 `POSE_UNCERTAIN`으로 끝난다. 판정은 placement frame_id의 게이트를 요구한다 |
| 4 High | 전체 seed 집계·예외 처리 없음 | 물리 시작 전에 `attempt_started.json`을 쓴다. 예외는 실패 결과로 기록한다. 집계기는 101–106마다 채택 시도를 정확히 하나 요구하고, 모두 같은 동결 SHA여야 한다. infrastructure failure만 1회 재실행을 허용하고, 누락·중복이 있으면 판정을 차단한다 |
| 5 High | 운반 중 유지·물리 파지를 검증하지 않음 | 평가 전용으로 physics step마다 검사한다. 양 손가락이 상자에 닿은 step 비율 ≥ 95%, 운반 단계에서 상자 z ≥ 0.04 m(재파지·release는 제외), 벽 접촉 step 수, 최대 관통·법선력. 모두 m1_success에 포함했다 |
| 6 Medium | bay 0.30×0.50 m | 0.5 m 정사각형으로 되돌렸다. 접근점은 v6 `approach_point()`가 정적 spawn keep-out을 피해 고른다 |
| 7 Low | timestep 오기 | #198 `09be45c`를 cherry-pick했다 |

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
| A5 | Codex 사전 검토 1–7, #181 v6(`ac34651`) | 스킬 v6(정적 keep-out, 공개 re-anchor hook), `cargo_noslip_v1` 주 조건(사용자 결정 대기), 0.5 m bay, 확인 프레임별 look-back 게이트, 유지·파지·예외 판정, test 가드와 채택 규칙. 조건과 판정도 바꾼 amendment이므로 `scope_note`에 적었다 |

## dev 결과 (전부 보고, `results.json`)
| 시도 | 에피소드 | 결과 | 도달 단계 |
|---|---|---|---|
| dev-a1 `969e03e` | s91 | CARRY_REANCHOR_UNCONFIRMED | 파지·자세 |
| dev-a2 `32c7069` | s91, s92 | GRASP_FACE_UNOBSERVABLE_AFTER_RELOOKS ×2 | 탐색(오차 3.1 / 2.3 cm) |
| dev-a3 `ce33ffd` | s93, s94 | OBSERVATION_REJECTED(러너 robot_id 버그) ×2 | 탐색 |
| dev-a4 `ea45e3d` | s93 / s94 / s95 | GRASP_TARGET_NOT_VISIBLE / SIM_LIMIT / GRASP_TARGET_NOT_VISIBLE | 탐색 / 운반·문 통과 / 탐색 |
| dev-a5 `6352fde` | s94(+15°, 진단) | **OWN_RGB_PLACEMENT_IN_SLOT, 당시 m1 검사 전부 통과**(A5 이전 판정식) | 전 단계 |
| dev-a6 `aa2dced` | s93 / s95(−20°) | GRASP_TARGET_NOT_VISIBLE ×2(N7 approach 임계값 65가 밝은 청색 바닥에서 실패) | 탐색(오차 8.0 / 7.1 cm) |
| dev-a6 `1ff646e` | s94(+15°, 진단, A5 조건) | **OWN_RGB_PLACEMENT_IN_SLOT, A5 판정식 전부 통과**: noslip 10, 운반 step 913,572에서 양손가락 접촉 100%, 최저 z 0.085 m, 접촉 0, 배치 frame에 게이트, GT 슬롯 오차 2.4 / 1.1 cm, SIM 503 s | 전 단계 |

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
