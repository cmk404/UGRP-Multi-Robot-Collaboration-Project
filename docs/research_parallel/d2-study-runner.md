# D2 — 유한 실행·E0 gate·원본 평가·TensorBoard

기준 구현 SHA는 `936bf821b873c9a364a768c4beea7c3debf27dd7`이다. 이 문서는
2차 구현 범위이며 [1차 handoff](d-handoff.md)의 당시 상태를 소급 변경하지 않는다.
후속 PR은 main 대상으로 만들며 사용자 확인 전 병합하지 않는다.

## 구현과 실행의 구분

- `harness/rgb_communication_study.py`: 동결 manifest, fail-closed preflight,
  독립 trial subprocess, 종료 후 evaluator 수집, 맵별 층화 집계.
- `scripts/run_rgb_communication_study.py`: 로컬 `prepare/check`, Colab `run/trial`.
- `scripts/submit_rgb_communication_study.py`: 명시 inventory만 업로드하는
  단일 신규 CPU Colab 제출기. 기존 sparse-source/setup/결과 검증 코드를 재사용한다.
- `harness/rgb_communication_evaluation.py`: runtime 종료·actor 주장·독립 물리
  판정을 분리한다. timeout/오류/중단을 물리 성공으로 덮어쓰지 않는다.
- `scripts/tensorboard_tools/rgb_communication.py`: 원본 해시를 확인하고 다시
  계산한 시행별 HParams/지표. summary의 success를 신뢰하지 않는다.

소스 구현·offline fixture 검사는 실제 물리 실행·모델 호출·영상 확인이 아니다.
합성 fixture는 명시 opt-in과 임시 logdir에서만 변환하며 공용 TensorBoard에 넣지 않는다.

## 승인된 최소 실행과 종료 조건

첫 실제 제출은 deterministic physical replay 단독/공동 각 1회, **LLM 0회**다.
각 시행 SIM ≤180초, wall ≤600초, 원격 준비·실행·회수·세션 정리 전체 ≤1800초다.
제출기는 현 단계에서 `physical_replay`만 받는다. GPU/새 provider/유료 구매/상시
서비스/자동 재시도/추가 학습을 포함하지 않는다. 외부 자원 실패는 기록하고 종료한다.

각 trial은 독립 process group과 wall deadline을 갖는다. runtime에 cleanup 여유
5초를 남긴다. 전체 할당이 부족하면 다음 시행은 `unrun`으로 남고 분모에서 빼지 않는다.
Colab 준비 시간이 길면 실제 job 할당을 줄일 수만 있다. 원격 runtime은 종료 조건에
도달하거나 회수 단계가 실패하면 본 제출기가 만든 유일 session만 stop한다.
stop 실패는 `submission.json`에 별도로 남겨 후속 수동 확인이 필요함을 표시한다.

고정 replay 역할은 명시된 robot별 `replay_actions`이며 협상 결과가 아니다.
자기 RGB 스킬의 `FINISHED_UNVERIFIED/STOPPED`를 받으면 `cannot_continue`로
진단 종료할 수 있다. evaluator는 planner/종료 조건에 전달되지 않는다.
실제 스킬 terminal은 성공 판정이 아니다. `replay_goal_complete`는 해당 시행의
box 또는 beam 물리 판정만 사후 파생하며 전체 `mission_complete`를 바꾸지 않는다.

## Source·environment·split 연결

`rgb-communication-study.v2` manifest는 clean source SHA, 모든 추적된 실행
Python/JSON/XML/requirements의 SHA-256, config hash, 고정 scheduler, 정확한
시행 목록과 분모, 단일 제출자, no-retry를 연결한다. child 진입점도 다시 preflight한다.
backend config와 trial seed, 공개 task, SIM/command 상한이 일치해야 한다.
프로토콜/map/physics/camera/calibration 입력은 상대 경로+실제 SHA로 검증한다.

E0 wrapper `rgb-study-environment.v1`는 다음을 참조한다.

- `source_sha`, `suite:{path,sha256}`, `provenance:{path,sha256}`
- A의 `scenario_assignments`, 환경 `review:{path,sha256}`
- `map:{map_id,map_version,map_instance_sha256,map_group_sha256,stratum}`
- `provenance_artifacts:{data:[ref],checkpoint:[ref],prompt:[ref]}`

기존 22 suite의 `load_suite/validate_splits`와 A의 `audit_exposure`를 호출한다.
map instance/group은 B의 실제 descriptor와 비교한다. 동일 맵 새 배치·Test A·
Test B를 섞지 않는다. unknown foundation pretraining을 무오염으로 선언하지 않는다.
test 노출 뒤 튜닝한 사례의 regression 전환은 A exposure ledger를 따른다.
현재 허용 scope는 `development_connection_smoke`이고 holdout rollout은 차단한다.

A의 `assess_environment_readiness`는 두 수준으로 나뉜다.

1. physical replay 이전: map→scene/goal, fixed camera, reset seed, 단일 clock,
   weld OFF의 소스·원본 hash-bound static 계약 검사. 물리 성공은 아직 미검증.
2. live smoke/pilot 이전: 실제 개발 성공·불가능 안전정지 등 physical controls까지 필요.

기존 `dispatch_open`은 명시 legacy development 예외를 통해서만 첫 진단에 쓰며
새 22 suite 중 하나로 alias하지 않는다. B가 지원하지 않는 맵을 open/north/south로
치환하지 않는다. 현재 기존 suite의 장면 preview와 실제 transport 지원은 별개다.
T1/T2/T3 학습·새 맵 물리 코호트는 본 제출기의 권한 범위 밖이다.

## Live admission은 아직 별도 gate

`llm_smoke/pilot`은 A의 시나리오 readiness + 환경 physical readiness + C의
실제 endpoint/model 비용 강제 근거가 모두 있어야 한다. ProviderSettings를 JSON에서
검증됐다고 선언하거나 실행 가능한 callback으로 우회 등록할 수 없다. 현 D resolver는
승인된 input-token bound가 없으므로 기본 C readiness를 그대로 NO-GO로 유지한다.
모델 선택·system prompt/adapter/이미지 이력 policy hash를 C policy manifest로 고정한다.

LLM pilot는 같은 backend/seed/입력/예산/비동기 scheduler의 2 scenario×3 condition
정확히 6회이다. 통신의 통계적 우열, 미지 맵 일반화, 개별 메시지 인과 효과를 주장하지 않는다.

## 결과·비용의 해석

원본 `runtime.jsonl`, post-close `evaluator.json.source_snapshot`, `plan.json`을
별도 보존한다. `run_finished.outcome=completed`는 actor loop 종료이지 물리 성공이
아니다. 정상 종료+독립 mission true만 system success; timeout/api_error/aborted는
독립 physical true와 공존할 수 있고 두 필드를 모두 남긴다. 잘못된 finish 주장은 별도다.

`planner_decisions`와 실제 `external_model_calls`를 구분한다. 전송 여부 미확인
request가 남으면 actual calls는 null/partially measured이며 admission 상한은 별도다.
provider usage 누락도 토큰 0으로 바꾸지 않는다. 발행 command interval의 겹침은 실제
이동 겹침이 아니다. message→changed action의 시간 연결은 인과효과가 아니다.
환경/제어기/협상 실패 분류는 evaluator가 기록한 값만 사용하며 나머지는 unclassified다.
사건 도달/미측정 수와 모든 배정 시행을 유지하고 성공한 시행끼리의 시간만 따로 요약한다.

## 운영 명령

```sh
python -m scripts.run_rgb_communication_study prepare --config CONFIG.json --output MANIFEST.json
python -m scripts.run_rgb_communication_study check --manifest MANIFEST.json --evidence-root EVIDENCE
python -m scripts.submit_rgb_communication_study --evidence-root EVIDENCE --inventory INVENTORY.json --output NEW_TRANSPORT
```

전송 capsule의 `manifest.json`과 상대 artifact/asset 경로는 원격
`outputs/study-inputs`에 대응해야 한다. inventory는 명시 path→SHA 사전이며 목록 밖
파일은 업로드하지 않는다. 기존 원본은 수정하지 않는다. source와 capsule 각각 업로드
해시 검증 후 실행하고 회수 zip·모든 원본·source SHA를 다시 확인한다.

실제 결과 회수 후에만 [TensorBoard 절차](../tensorboard.md)를 수행한다. 원본 경로/해시
중복 검사를 거쳐 공통 logdir에 새 snapshot을 만들고 readback 및 기본 UI의 표시/고정
지표/열/영상까지 확인한다. `backend/execution.mp4`도 기본 media registry에 등록한다.
fixture 검증만 한 경우 dashboard 재개방·공용 viewer 변경은 하지 않는다.

## 현재 검증 범위

2026-09-22 동결 통합 SHA `bfe478f1209375d774b6dc2881300198f967f5ca`로
승인된 Colab 단독/공동 재생 2회를 모두 실행·회수하고 해당 세션을 종료했다.
두 결과는 runtime terminal .8 / evaluator .75 모순으로 `invalid_artifact`이며,
`replay_goal_complete=null`을 유지한다. LLM 0회, 추가 실행 NO-GO.
전체 offline 1336 passed/7 skipped/198 subtests 및 관련 exporter 34 passed.
원본·모든 실패·진단·TensorBoard 검증 단계는
[별도 실험 기록](../../experiments/2026-09-22-rgb-communication-replay/README.md)에 보존한다.
후속 clock 최소 수정 후보는 이 원본과 분리하며 시간 guard를 완화하지 않는다.

후속 D seam은 B descriptor의 `supervisor_clock_schema`와 C의 `clock_snapshot`
인자를 사전 검사하고, 실제 bundle의 clock-only callback만 runtime supervisor에
연결한다. evaluator로 clock을 구성하거나 actor/common_task에 전달하지 않는다.
새 `rgb-runtime-clock.v1`은 요청/성공 요청/확인 실제/종료 실제 시각을 분리한다.
unknown·잘못된 schema/domain/유한값/순서/종료 event 불일치는 invalid로 거절한다.
확정 partial actual .77과 last-ack .75는 분리 보존하고, 요청 .8을 실제 시각으로
대입하지 않는다. requested/actual의 미세 roundoff 판정은 B가 소유한다.
새 callback 연결·오프라인 회귀는 물리 재실행 권한이나 성공 근거가 아니다.
실제 evaluator clock이 unknown이면 원본 `timestamp_s=null`을 파싱·보존하되
최종 평가는 반드시 invalid로 거절한다. TensorBoard는 이 경우도 진단만 내보내고
미측정 evaluator/final SIM 값을 생성하지 않는다. 숫자로 기록된 구 원본의
시간 선후 guard는 변하지 않는다.

- 새 계약/제출/회수/TensorBoard 테스트는 모두 network-free이며 임시 원본만 사용한다.
- `scripts/run_ci_tests.py`의 기존 `tests/test_rgb_communication*.py` glob으로 자동 포함된다.
- A/B/C의 최종 committed source를 별도 integration branch에 합친 뒤 실제 adapter
  계약과 static gate를 다시 확인해야 한다. 코드 파일의 존재만으로 gate를 열지 않는다.
- 실제 Colab 실행/회수, 물리 성공, live LLM, Test A/B rollout, 실제 dashboard 결과는
  각각 별도 실험 기록에 증거를 남기기 전 완료가 아니다.
