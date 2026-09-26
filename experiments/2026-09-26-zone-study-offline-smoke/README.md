# 2026-09-26 구역 대화 연구 no-LLM 오프라인 스모크 (저장 RGB + 가짜 시계)

> **버전 안내.** 이 폴더의 파일은 **v1**(코드 `d87e0f2`) 결과 그대로이며 덮어쓰지 않는다.
> Codex 적대적 검토 18건을 수정한 뒤의 재실행은 [`v2/`](v2/README.md)(코드 `dd497c8`)에 있다.
> 두 버전의 차이·불변 항목은 v2 README의 "v1과 무엇이 다른가"를 본다.
> Codex 3차 재검토 부분 해결 4건(#1·#11·#12·#16)을 고친 뒤의 재실행은 [`v3/`](v3/README.md)(코드 `eb5572b`)에 있다. v1·v2는 그대로 둔다.
> Codex 4차 재검토 #16(사용량 미상 끝단 보존)의 남은 반례 3건을 고친 뒤의 재실행과 첫 보고서·TensorBoard 변환은 [`v4/`](v4/README.md)(코드 `c21a6fe9`)에 있다. v1~v3는 그대로 둔다.
> Codex 5차 검토(P1 1건·P2 2건) 수정 기록은 [`review-r5/`](review-r5/README.md)(코드 `f44ea845`)에 있다. 스모크 출력이 바뀌지 않아 재실행하지 않았고 v5는 없다.
> Codex 6차 검토(P1 1건·P2 1건)와 통합 PR #229가 찾은 2건(재질문 타이머 누적, 계약의 tags_v2 지도 거부) 수정 기록은 [`review-r6/`](review-r6/README.md)(코드 `8ab8f7ed`)에 있다. 재질문 규칙이 채널 조건의 SIM trace를 바꾸므로 새 버전 [`v5/`](v5/README.md)로 재실행했다. v1~v4는 그대로 둔다.

- **상태:** 완료. 30회 전부 통과(`results.json` `ok: true`).
- **실행 소스:** `d87e0f2bc179d00ad4c8de48e0559408d46d6a0c` (브랜치 `kiro/zone-study-core`, worktree `ugrp-wt/kiro-study-core`)
- **실행기:** `scripts/run_zone_study_offline_smoke.py` → `harness/zone_study_offline.py`
- **모델 호출 0회, 물리 0회, 시뮬레이터 import 0회.** 실제 LLM도 MuJoCo도 쓰지 않았다.

## 이 결과가 뜻하는 것과 뜻하지 않는 것

**입증한 것:** 다섯 패키지의 **배선**이다. 시나리오 설정(E) → 호출별 입력(A) → 한국어 프롬프트·통신 프로토콜(C) → SIM 비용·단일 사건 큐(D) → A 로그 → 평가(I)가 실제로 연결되고, 채널 격리·비용 회계·평가 역류 없음이 반증 가능한 검사를 통과한다.

**입증하지 않은 것:** 언어 이해, 통신 효과(조건 간 우열), 물리 운반 성공, 자기 카메라 인식 성능. fixture 응답은 **명시적인 오프라인 대역**이며 모델 실패의 대체가 아니다. 물리가 없으므로 어떤 시행도 성공으로 집계되지 않는다(전 조건 `successes: 0`, `censored_trials`는 시행 수와 같다). 조건별 호출 수·SIM 시간 차이는 fixture의 고정 규칙과 예산 상한이 만든 값이며 **통신 효과가 아니다.**

## 조건과 규모

6 시나리오 × (주 4조건 + 참조 상한 R) × seed 1개 = **30회**. 시나리오마다 선언된 첫 seed 하나만 썼다.

| 조건 | 시행 | 호출 | 발화 | 전달 edge | 자유 문장 | follower↔follower | 사고 SIM s | 발화 SIM s | 종료 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| `no_comm` 무통신 | 6 | 432 | **0** | 0 | 0 | 0 | 1123.2 | **0.0** | sim_horizon 6 |
| `peer_ko` 자유 한국어 | 6 | 540 | 18 | 36 | 18 | 0 | 1420.2 | 5.4 | budget_exhausted 6 |
| `leader_ko` 지휘 겸임 | 6 | 540 | 24 | 24 | 24 | **0** | 1425.6 | 7.2 | budget_exhausted 6 |
| `structured` 정형 | 6 | 540 | 18 | 36 | **0** | 0 | 1420.2 | 5.4 | budget_exhausted 6 |
| `reference_R` 참조 상한 | 6 | 144 | 0 | 0 | 0 | 0 | 374.4 | 0.0 | sim_horizon 6 |

`leader_ko`의 지휘자는 seed마다 순환한다. 이번 여섯 seed의 지휘자는 `r2, r3, r1, r2, r3, r1`이며 **r1/r2/r3를 모두 덮는다**(`cohort_summary.json`의 `leader_ids`).

`leader_ko`의 전달 edge가 발화 수와 같은 것은 허브-스포크이기 때문이다. 지휘자가 follower 두 대에 각각 별도 발화를 보내고 follower는 지휘자에게만 보낸다. `peer_ko`·`structured`는 mesh 방송이라 발화 하나가 edge 두 개다.

## 검사한 것 (모두 통과)

1. **채널 격리** — `results.json` `trials[].channel`
   - `no_comm`: 송신 0, 수신 0, 발화 비용 0. 패키지 A의 입력 allowlist에 `inbox` 키가 아예 없다.
   - `leader_ko`: follower끼리 전달 0건. 테스트에서 follower→follower 시도가 `no_follower_to_follower`로 거절되고 기록되는 것도 확인했다.
   - `structured`: 자유 문장 0건. `text`/`reason`/`note` 밀반입 시도는 거절된다.
   - `peer_ko`: 발화 18건 전부 한국어 본문이고 `korean_share` 1.0, 코드 전환 0건, literal ID 훼손 0건.
   - `reference_R`: LLM actor는 commander 하나뿐이고 메시지 채널이 없다. 지시는 `action`(`kind: order`)으로만 내린다. 로봇은 payload를 받지 못한다.
   - 패키지 C의 inbox 수와 패키지 D 스케줄러의 inbox 수가 30회 모두 일치한다(`inbox_agrees_with_scheduler`).
2. **비용 회계** — `results.json` `trials[].cost`
   - 호출마다 `released_at_sim_s − requested_at_sim_s == sim_cost_s`(패키지 A가 검증).
   - 호출 비용 합계 = 스케줄러의 actor별 사고 시간 합계.
   - 메시지마다 전달 지연이 `zone_sim_cost.delivery_delay_s(수신자 수)`와 같다. 패키지 C 전달 지연을 이 값으로 구성해 두 모델이 같은 수를 쓴다.
   - 채널 없는 조건은 발화 비용 0, 발화한 조건은 발화 비용 > 0.
   - 비용 배율 0(`zone_sim_cost.v1_free`)에서 모든 호출 비용이 0이 되는 진단 조건도 확인했다.
3. **평가 역류 없음** — `results.json` `backflow_probes` 30/30
   - 같은 시행을 시나리오의 **비공개 구역**(숨은 사건 일정 + 가짜 정답 배송 목록)만 바꿔 두 번 돌렸을 때 SIM trace·호출 로그·메시지 로그·행동 로그·호출별 입력 해시가 **모두 동일**했다.
   - fixture actor의 입력은 자기 요청 하나뿐이다(`respond(self, request)`, 보유 속성은 `actor`/`condition`/`seed`/`calls`뿐). `actor_isolation` 통과.
   - 오프라인 시행 기록의 `referee`는 **빈 객체**다. 물리가 없어 대조할 정답이 없고, fixture의 주장을 정답으로 쓰면 순환이 된다.
4. **입력 경계** — 호출 payload 전부가 A의 `validate_robot_payload`를 통과했다(`payload_validated: true`). 프롬프트 전문과 user JSON에 `top_rgb`·`top_camera`·`cctv`·`nav_cam`·`teacher_receipt`·`ground_truth`·`grasp_success`·`qpos`·`hidden_event`·`referee`·`zone_counts` 문자열이 없다. 이미지 라벨에 `TOP`이 없다.

## 입력과 비용 설정 (`config.json`)

- 조건 registry 해시 `f1ff6a49a93124a9…`, 계약 `ugrp.zone_study_contract.v1`, 프롬프트 `ugrp.zone_study_prompts_ko.v1`, 프로토콜 `ugrp.zone_study_protocol.v1`.
- 지도 3종(`zone_wide_two_doors_tags_v1`, `zone_wide_door_tags_v1`, `zone_wide_corridor_tags_v1`)의 파일 해시와 공개 투영 해시를 기록했다. 지도 파일은 수정하지 않았다.
- 시나리오 설정 6개 파일의 해시를 기록했다. 주문서는 설정에서만 만들며 시뮬레이터 상태를 읽지 않는다.
- SIM 비용(잠정, `zone_sim_cost.v1`, 해시 `81b59966e0e23b2e…`): 호출 1.0 s, 입력 토큰 0.0002 s/token, 출력 토큰 0.02 s/token, 발화 0.3 s, 전달 0.1 s, 격자 0.1 s. **실제 모델 처리 속도 측정값이 아니다.**
- fixture 호출의 토큰 수는 고정값(입력 4,000 / 출력 40 + 발화당 30)이다. 실제 토큰 사용량이 아니다.
- **자기 손목 RGB는 저장 프레임 참조**다. `tests/fixtures/markerless_box/blue_floor_release`의 실제 로봇 손목 JPEG 7장을 (로봇, 관측 번호)로 결정적으로 고르고 payload에 `own-<robot>-<번호>`와 바이트 sha256을 넣는다. **이 루프는 인식을 돌리지 않는다.** 참조가 있다는 사실은 로봇이 무엇을 볼 수 있다는 주장이 아니다.
- 시행 horizon 300 SIM s. 호출 예산은 `CallPolicy` 기본값(actor당 30회, 전체 시도 90회, 최소 간격 2 s, 유휴 재검토 10 s).

## 환경

macOS 27.2 arm64, Python 3.12.13, CPU 8. 스레드 환경변수 `OMP_NUM_THREADS=OPENBLAS_NUM_THREADS=VECLIB_MAXIMUM_THREADS=MKL_NUM_THREADS=1`. 실행 시작 시 부하 평균 `11.71 / 13.95 / 15.08`(다른 작업과 공유하는 Mac이다). 이 경로는 물리·학습을 돌리지 않으므로 `scripts/agent_lock.py` 배타 잠금을 잡지 않았고, 대신 `scripts/ugrp_session.py run kiro-zone-study-offline-smoke`로 실행해 종료 시 프로세스 그룹을 정리했다. wall 시간은 결과에 쓰이지 않는다(호출 기록의 `wall_latency_s`는 전부 `null`).

## 알려진 제약과 다음 단계

- 시행 기록 30개 중 서로 다른 SIM trace는 23개다. 시나리오가 다르면 주문서가 달라 발화 본문과 입력 해시가 달라지지만, 호출 수는 예산 상한에 걸려 같아진다. fixture의 규칙이 단순해서 생기는 특성이다.
- `cohort_summary.json`의 `provenance.single_bundle`은 `false`다. 섞인 항목은 `map_file_sha256`·`public_map_sha256`·`order_sheet_sha256`뿐이며, 이는 여섯 시나리오가 지도 3종·주문서 6종을 쓰기 때문에 **당연한 결과**다. `code_sha`·`registry_sha256`·`cost_profile_id`·`input_profile_id`·`model`·`prompt_template_sha256`은 30회 모두 한 값이다. 조건 간 비교는 같은 시나리오·seed 짝 안에서만 한다.
- 발화의 SIM 비용은 그 발화를 만든 **호출**에 부과된다. 그래서 A 메시지 기록에는 발화별 비용 항목이 없고, 대화 비용의 기준은 시행 기록의 `model.sim_cost_s`다.
- **다음 게이트는 물리 no-LLM 스모크**다(6상황 × 주 4조건 × 1 seed = 24회). 자기 카메라 실행기가 준비된 뒤에 수행하며, 교사를 쓴 임시 스모크를 이 게이트 통과로 대체하지 않는다. 이번 오프라인 통과를 물리 통과로 승계하지 않는다.
- LLM 파일럿 예산·표본은 아직 동결하지 않았다. 비용 계수도 잠정이며 본실험 전에 tokenizer·모델 설정과 함께 고정해야 한다.

## 파일

| 파일 | 내용 |
|---|---|
| `results.json` | 30회 시행별 채널·비용 검사, 스케줄러 보고, trace 해시, 역류 probe 30건, actor 격리 |
| `cohort_summary.json` | 패키지 I 조건별 코호트 요약(실패를 분모에 유지) |
| `config.json` | 계약·프롬프트·프로토콜 버전, 조건 registry 해시, 비용 파라미터·해시, seed, 시나리오·지도·저장 프레임 해시 |
| `environment.json` | Python·플랫폼·CPU·부하 평균·스레드 설정 |
| `example_trial_record.json` | `ugrp.zone_study_trial.v1` 형식 예시(`leader_ko`/`s1`, calls·actions를 2개로 줄임) |
| `source.json` | 실행 SHA, 병합한 다섯 패키지 브랜치 SHA, 실행 명령 |
| `raw_index.json` | raw 결과의 **로컬 경로**와 파일별 sha256·크기 |

raw 결과 전체(30개 시행 기록, 6.3 MB)는 로컬에만 있고 Git에서 제외된다. 위치는 기본 체크아웃의 `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke/`다. 처음에는 worktree의 `outputs/`에 썼고, 2026-09-26에 같은 상대 경로로 `mv`했다. 이동 전후 해시가 같았다(`raw_index.json`의 `relocation`). **해시만 저장한 것은 백업이 아니다.**

## 재현

```sh
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1 \
PYTHONPATH=. python3 scripts/ugrp_session.py run kiro-zone-study-offline-smoke -- \
  python3 scripts/run_zone_study_offline_smoke.py --output outputs/zone-study-offline-smoke-NEW-ID
```

자동 회귀검사는 `tests/test_zone_study_offline.py`(36개)이며 `scripts/run_ci_tests.py`의 `TEST_PATTERNS`에 등록했다. 자동 검사 통과와 실제 물리 완주는 구분한다.
