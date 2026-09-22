# 후속 clock 수정 — 오프라인 영향 검증만

실제 실행은 상위 기록의 `bfe478f` 두 시행뿐이다. 이 후속 작업에서 새로운
물리·렌더링·Colab·LLM 실행을 하지 않았고 이전 원본/판정을 바꾸지 않았다.

## 고정 후보와 이전 반례

- 예비 `5e158f970bed3a7d1f64b565b4dcd6a6275b10c4`: A 독립 9개 중 8개 통과, 180초 raw-actual close 반례로 NO-GO. [실패 기록](a2-clock-impact-5e158f.json)을 보존한다.
- 후속 `f139672e130fc921087281c69977f039e09de353`: `codex/advanced-rgb-clock-validation`에 고정·push, 작업 트리 clean에서 검사.
- 구성: A `ca32f280a77b484f299059f381e7ea2c8926850d`, B `da18a7b8984fd1404aa03864ade3b0dfa2cd9694`, C `70243c4e6f376624dc538b3e10db36c501397765`, D `de1f70bf527ba891ab9c7f37e7175cd274fcca4c`.
- 이 보고를 추가한 커밋은 위 검증 후보와 다르며 문서/검증 기록만 추가한다. f139672 소스는 그대로 보존한다.

## 검증 범위

| 검증 | 결과 | 경계 |
|---|---|---|
| D 전체 offline, 127 modules | 1423 passed, 7 skipped, 198 subtests; 268.66초 | `scripts/run_ci_tests.py`, 물리 재실행 아님 |
| D RGB 관련 전체 | 250 passed, 4 skipped, 14 subtests | 현재 B/C/D 고정 조합 |
| D 순수 변경 관련 | 72 passed | callback 연결, 신규 clock 검증, null 원본 진단 표시 |
| A 독립 영향 probe | 10/10 passed | clock 경계·원본 보존·종료 후 writer 동결 |
| A focused 회귀 | 154/154 passed, 실패/skip 0 | 보존된 JUnit과 source hashes 연결 |
| C/D 각각의 180초 종료 연결 | PASS | 실제 C ExecutionClock→B close(raw actual), 산술 fake clock만 |

D와 C가 각각 확인한 종료 값은 요청 180초, 실제 `180.00000000013756`초다.
실제 값을 그대로 close에 전달해 종료했고 추가 fake step은 0이다. 요청 cap을
늘리거나 실제 시간을 요청 180으로 덮어쓰거나 무조건 close(None)로 우회하지 않았다.

새 callback은 clock-only 4필드, C terminal은 8필드이며 actor 입력에 노출하지 않는다.
no-advance/partial/unknown을 분리하고 unknown에 요청/last-ack를 대신 넣지 않는다.
evaluator null은 미측정 원본으로 파싱하지만 최종 평가는 invalid로 유지한다.
TensorBoard fixture는 raw physical true라도 invalid evidence를 성공으로 바꾸지 않으며
미측정 final/evaluator SIM scalar를 만들지 않는다. 실제 공통 대시보드는 재변환하지 않았다.

## 로그 동결 경계와 남은 조건

**bundle.close만으로 worker 로그가 동결되지는 않는다.** 이미 시작된 image-only
worker의 완료 로그는 close 뒤에도 쓸 수 있다. A가 이 late-write를 직접 재현했다.
D의 기존 독립 trial 프로세스 경로에서는 정상 exit/reap, TERM/reap,
SIGTERM 무시 후 5초 grace→KILL/reap까지 마친 뒤 전체 artifact 해시가 고정되는
것을 확인했다. timeout으로 완료가 끊긴 worker의 계산 완료/시간은 unknown이다.
이 보장은 in-process caller가 bundle.close 직후 해시를 만드는 방식으로 확장되지 않는다.

A 최종 [영향 감사 원문](a2-clock-impact-review-f139672.json) SHA256:
`75aa9ef5b67619d508a636eec2aeae7e6e275566f80f3b487c4de7909e3ac7ce`.
원문·참조 probe·두 후보의 structured 결과·JUnit을 원형으로 복사했고 각각 해시를
대조했다. A의 494개 실행 소스 해시도 f139672 checkout과 직접 대조했다.

기존 bfe 69개 원본 파일과 인증서가 변하지 않았고, 구 인증서를 새 SHA에 사용하는
[preflight는 차단](d2-old-certificate-rejection-f139672.json)됐다. A의 PASS는
clock/승인된 최소 계측 **영향 범위만**이며 `execution_admission=false`,
`live_readiness=false`, `physical_pass=false`다. 새 전체 boundary certificate나
승인된 새 replay 설정은 생성하지 않았다.

선행 stale-worker 실패는 여전히 별도 미해결이다. SIM 1초/wall 2초 상한은
그대로이며 새 계측으로 과거 실행의 실제 계산 시간을 소급 추정하지 않는다.
물리 성공·복구·통신 효과·provider input/output hard cap 검증은 미완료다.
추가 실제 시행은 별도 허가 전 NO-GO이며 main 병합도 하지 않았다.

GitHub 원격 CI는 로컬/독립 검사와 별도다. 기록 시점에 D 코드 de1f70b의 PR #101은
11 success/1 in progress였고, [f139672 통합 workflow](https://github.com/cmk404/UGRP-Multi-Robot-Collaboration-Project/actions/runs/35700643872)는 진행 중이었다. 상태를 최종 성공으로 선기록하지 않았다. C PR #103의 70243c4 CI 12/12 성공은 C 담당자의 확인이며 통합 CI를 대신하지 않는다.
