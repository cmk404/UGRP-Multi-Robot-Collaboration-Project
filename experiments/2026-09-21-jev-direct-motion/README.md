# Jev direct motion comparison — 2026-09-21

Issue #78. Single r2 robot approaches the existing cyan box in the open dispatch arena. Jev/Gemini choose actual 0.2-second mecanum commands from seven discrete candidates. No pretrained approach policy, grasp, carry, or new task negotiation runs. Rule, Gemini and Jev receive the same RGB-derived state and action set. This is a new direct-motion experiment, not a model-only replication of the earlier five-task failure.

## Frozen final protocol

- Final cases: `straight`, `left_offset`, `right_offset`; each policy once per case, 9 episodes. `dev` and `dev_yaw` are separate development poses. The first nine-run comparison was used for observer diagnosis; final-v2 repeats the same cases and is therefore not a fresh holdout. One fixed map, box position, three limited start poses; not broad generalization or statistical superiority.
- Model versions: `jev-1.13.0`, `gemini-3.8-flash`. Policies run rule → Gemini → Jev within each case. Fixed order and single repetitions limit wall-time comparisons.
- Shared setup: six 0.2s forward identity probes, stopped settling. Own identity/front inferred from RGB displacement; no initial true pose in policy input. Full wheel envelope estimates current heading; issued turns never update it. Each command has a 0.2s port lease and 0.05s stopped dwell.
- Shared observation: unchanged own RGB and fixed TOP RGB archived. TOP color/shape detects one cyan box and four-wheel envelope. Nominal camera/feature-plane calibration produces relative range/bearing. Own RGB contributes cyan pixel fraction only; this is classical perception plus a text-state policy, not Jev/Gemini raw image understanding. The observer has a limited nominal ±18-degree wheel heading domain with a 2-degree pixel tolerance and holds on lost/ambiguous evidence.
- Vision goal: range 0.27–0.28m and bearing ±3 degrees, three fresh stopped confirmations. This fixed common completion gate supplies no action direction; models choose all other post-probe motion. Model confidence is recorded, not interpreted as physical safety or used as an actuation threshold. Thus the previous raw pilot's 0.8 confidence filter is not used in any of these matched arms.
- Output-only physical success: terminal stable tail ≥0.35s within 0.26–0.30m and ±6 degrees; RGB completion claim; no cargo/obstacle/peer contacts, weld use or camera/geometry changes. Approach alignment does not certify a grasp-ready arm pose. Evaluator never supplies policy observations, commands or termination.
- Inference pauses SIM, including API latency. No real-time hardware/distributed-control claim. Local command expiry remains active for each SIM step. 70 post-probe steps, 180k input tokens (5k reserved before each call), 360 wall seconds per episode; request timeout 30s, no automatic retries. Raw provider body/request and model version saved. HTTP errors or invalid responses stop that episode and preserve failure; later independent episodes still run.
- Jev's live serialized probabilities can sum to 0.99/1.01. Validator allows only the accumulated half-unit rounding error for seven two-decimal values, while checking bounds, candidate set; raw values stay unchanged.
- Development failures are preserved separately: optical-flow coverage, wheel-corner support, premature RGB completion and weak probe displacement. Development precedes the first comparison; observer diagnosis from the first comparison is disclosed below. Development API attempt stopped on rounded probabilities; Gemini development completed. Source is committed before each run and fixed throughout each cohort.

## Run

Use the existing simulation environment. From this checkout:

```sh
python scripts/ugrp_session.py run jev-direct-motion -- /absolute/path/to/mjpython scripts/run_jev_motion.py --execute --policies rule gemini jev --cases straight left_offset right_offset --prompt-key --output /new/local/output
```

Key comes from hidden input or TYPESAFE_API_KEY; it is never saved. Requests go to the official TypeSafe endpoint with redirects disabled. Raw evidence is local under `/Users/changmin/projects/ugrp/outputs/jev-direct-motion-20260921-*`; Git records are not a video backup. Main is not merged without user approval.

API: https://docs.typesafe.ai/api and https://docs.typesafe.ai/primitives/choice (2026-09-21).

## Amendment after first full comparison

The first cohort at dfca6d7 completed all 9 episodes: rule 2/3, Gemini 2/3, Jev 0/3 under the strict combined completion criterion. A shared negative-yaw observation failed at a wheel aspect ratio of 1.117 vs the old 1.12 cutoff. The final observer now applies a declared two-pixel interval to aspect ratio and a two-degree image-angle tolerance, preserving four-corner visibility gates. A regression includes the original failed RGB.

The Jev response sometimes has a valid `choice` whose serialized probability is not maximal (e.g. backward 0.25 vs stop 0.26). Final-v2 executes the API's actual valid discrete choice and records the mismatch; it does not silently replace it with argmax. Candidate/number/distribution/lease validation remains. Probabilities are diagnostic and never safety evidence. No prompts, goal thresholds, speeds, action candidates or budgets were tuned after this cohort. Final-v2 reruns all policies/cases from the same starts and is kept separate from the first attempt.

## Archived-state representation diagnostic (predeclared)

After final-v2 ends, 12 archived states are compared with `jev-1.13.0`: eight from the separate development poses and four known failure states from final-v2. The exact indices and randomized balanced request order are frozen in `scripts/probe_jev_motion_representation.py` before execution. There are three variants, two repeats each: 72 calls maximum, 150k input-token and 360-second limits; no automatic retries or physical actuation.

1. Original numeric state and instructions.
2. Named range/alignment relations computed from those same RGB values, plus matching literal question wording; unchanged seven motor candidates.
3. Variant 2 plus explicit per-action applicability descriptions. This encodes the reference rule's controller knowledge in the criteria.

The metric is agreement with the existing rule, not optimal-action accuracy or autonomous task success. Several actions may be useful; the rule is only a diagnostic reference. Variant 2 changes representation and wording together, so it cannot isolate a single wording effect. Variant 3 deliberately tests an explicit controller specification and must not be advertised as newly learned planning. Selection includes observed failures; none is a fresh holdout. Simulator truth and referee files are not read by this diagnostic.

## 최종 결과

실행 소스 `49311961d37d6ebea66aceaaa4f5efc2e9175ae3`, raw `/Users/changmin/projects/ugrp/outputs/jev-direct-motion-20260921-final-v2`. 위 프로토콜은 추론 중 SIM을 멈추는 단일 물체 접근·정렬 시험이다.

| 정책 | straight | left_offset | right_offset | 실제 경과 합계 | API 중앙 / p95 | 모델 호출 / 입력 토큰 |
|---|---|---|---|---:|---:|---:|
| 규칙 | 성공 | 성공 | 성공 | 46.49초 | 해당 없음 | 0 / 0 |
| Gemini | 성공 | 성공 | 성공 | 375.80초 | 1.999 / 7.531초 | 116 / 60,821 |
| Jev | 실패 | 실패 | 실패 | 133.69초 | 0.551 / 0.765초 | 138 / 117,419 |

정책별 SIM 시간 합계는 각각 42.65 / 42.65 / 44.40초, 발행 명령 수는 146 / 146 / 157이다. 모델 비용의 실제 청구액은 확인하지 않아 null로 남긴다. 성공 수가 다른 조건의 총 시간을 성능 우위로 해석하지 않는다.

- Jev 직선: 39회 전진 뒤 31회 정지. 후반 RGB 거리는 0.2832m로 목표 상한 바로 밖이다. 사후 물리 위치만 보면 도착 범위에 있었으나 RGB 완료가 없어 `step_budget` 실패다.
- Jev 좌측 시작: 목표가 자기 오른쪽인 장면에서 반대 회전을 반복. 좌회전 23회/우회전 7회 뒤 `four_wheel_envelope_unresolved`로 종료했다. 실제 이동은 거의 없었다.
- Jev 우측 시작: 접근 뒤 주로 반대쪽 정렬을 선택해 방향 오차가 커졌다. 전진 28회/우회전 9회/좌회전 1회 후 같은 관측 오류로 종료했다.
- 9회 모두 화물·장애물·상대 접촉과 weld 사용이 없었고 카메라/형상 불변 검사를 통과했다. 지연은 이 환경에서의 관측값이며 일반적인 서비스 성능 보장이 아니다.

## 입력 표현 진단 결과

소스 `aed309c7e8f8695aaf77e3335180405810581a4f`, raw `/Users/changmin/projects/ugrp/outputs/jev-representation-20260921`. 72회 모두 응답을 기록했으며 입력 63,944토큰, 실제 경과 40.22초였다. 실제 청구액은 미확인이다.

| 구성 | 규칙 일치 | 같은 상태 2회 선택 일치 |
|---|---:|---:|
| 원래 숫자 입력 | 13/24 | 11/12 |
| 의미 상태와 질문 | 24/24 | 12/12 |
| 의미 상태 + 명시적 조건 | 24/24 | 12/12 |

동일 관측에서 입력 구성이 판단을 바꾼다는 증거다. 관측 표현과 질문 문구가 함께 달라졌으며, 알려진 실패 장면을 포함한다. 개선된 하네스로 새로운 주행을 했다는 결과는 아니다. 다음 비교와 외부 코드 검토는 [Jev 제어 설계 검토](../../docs/jev_control_design_review.md)에 정리했다.

## 검증·재현·보관

- 실행 소스 4931196: 전체 오프라인 검사 **997 passed, 1 skipped, 184 subtests passed**. 이후 표현 진단의 경계 검사를 포함한 관련 파일 **10 passed**.
- `scripts/audit_jev_motion.py`로 최종 388개 의사결정을 원본 RGB에서 다시 계산하고 이미지 해시·실제 모델 요청/응답·선택 행동·별도 물리 성공을 검증했다. 정책 입력을 정답으로 보정하지 않았다.
- 9개 원본 주행 영상과 비교 영상 전체를 ffmpeg로 디코딩해 오류 없음을 확인했다. 비교 영상은 결정 단계로 정렬하며 실시간 비교가 아니다. 시작·중간·종료 프레임을 직접 검토했다. 화면 표시용 확대만 적용했고 모델 입력 카메라는 그대로다.
- 비교 영상: `/Users/changmin/projects/ugrp/outputs/jev-motion-review-20260921/comparison.mp4`.
- `report.json`: 최종 9회 결과와 감사. `prior-attempt-results.json`: 첫 비교 9회와 개발 실패 포함, 최종 성공률에 합산하지 않음.
- `protocol.json`, `final-turns.json.gz`: 최종 환경/계약과 388개 의사결정. `representation-*.json`, `representation-api.json.gz`: 진단의 사전 고정 순서·전 상태·72개 실제 요청/응답.
- `raw-manifest.json.gz`: 로컬 원시 자료·검토 자료의 원본 경로, 크기, SHA256. 원본 RGB/영상은 로컬 보관이며 Git 원격 백업이 아니다. `reviewed-source-manifest.json`은 읽기 전용 조사에 사용한 외부 코드 버전을 고정한다.
- 실행한 `jev-motion-final`, `jev-representation` 세션은 완료 후 종료됐다. PR #80은 검토용이며 main에 병합하지 않았다.

```sh
python scripts/audit_jev_motion.py /absolute/raw/final-v2 /new/report-output
python scripts/render_jev_motion.py /absolute/raw/final-v2 /new/video-output
```

감사는 source 4931196 이후 동일한 관측 로직을 사용하는 체크아웃에서 수행해야 한다. 원본 데이터가 없는 새 clone에서는 해시 목록만으로 영상을 재생성할 수 없다.
