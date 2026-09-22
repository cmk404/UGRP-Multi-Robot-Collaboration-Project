# C2 · 실제 요청 어댑터와 비동기 독립 판단

2026-09-22. 공통 출발 SHA `936bf821b873c9a364a768c4beea7c3debf27dd7`.
후속 브랜치 `codex/advanced-rgb-agents`. 기반 #99가 승인 병합되어 이번 PR base는
`main`이다. 이 변경의 병합 승인은 별도다.

범위: R1/R3 및 E0의 입력·시행 조건 고정 계약. 새 세션·외부 모델·물리 실행·학습·
원격 제출은 하지 않았다. 아래 검사는 오프라인 코드/계약 검사이며 통신 효과나
일반화·운반 성공의 증거가 아니다. synthetic 결과를 기본 TensorBoard에 등록하지 않는다.

## 구현

- `harness/rgb_communication_planner.py`: 기존 `GeminiProxyCompleter`를 재사용한다.
  각 로봇은 서로 다른 planner 객체를 갖는다. 현재 자기 RGB/공용 top RGB 원본과
  최근 서로 다른 관측 1쌍을 실제 image content로 직렬화한다. 첫 요청은 2장,
  이후 기본 최대 4장이다. 과거 frame hash만 보고 이미지를 봤다고 주장하지 않는다.
  `history_image_pairs`는 0–3, 모든 비교군에서 동일하게 사전 고정한다.
- 전송 직전 실제 `Request.data`와 사전 예산 계산 본문이 동일한지 검사한다.
  `.request.json`/`.response.json` 원본 bytes·SHA·크기를 새 파일로 보존한다.
  endpoint URL·헤더·인증정보는 저장하지 않는다. HTTP 오류 본문은 비밀정보 가능성으로
  보존하지 않고 안전한 오류 종류를 기록한다. 정상 completion body는 파싱 실패도 보존한다.
- `harness/rgb_communication_async.py`: 하나의 scheduler가 `tick/observe/status/submit/close`를
  소유한다. 모델 worker에는 포트·evaluator·공유 actor memory 참조가 없다.
  로봇별 최대 1개 unresolved 요청, 전체 최대 3개다. 한 모델 지연/오류가 다른 로봇이나
  SIM clock의 진행을 막지 않는다. none/structured/natural 모두 같은 scheduler다.
- 기존 순차 `run_rgb_communication`은 별도 경로로 남는다. 비동기 비교를 순차 baseline과
  같은 처리 조건이라고 부르지 않는다. 완료는 두 경로 모두 `completed`이며 실제 성공이 아니다.
- `cancel_pending/pause/resume/interrupt/release`를 모델이 직접 선택한다. 새 파트너는
  기존 intent/lease 철회 뒤 새 task_id 요청으로 처리한다. 메시지의 help/accept/reject/
  retract/partner_change는 실행 동의의 대체물이 아니다. 무통신도 같은 행동을 사용할 수 있다.

## D 실행기 API

```python
from harness.rgb_communication_async import (
    AsyncRuntimeLimits, OfflineDecisionPlanner, RuntimeControl,
    run_rgb_communication_async,
)
from harness.rgb_communication_planner import ProviderSettings, make_rgb_planners

# 물리 replay: callable은 자기 request만 읽으며 정답은 전달하지 않는다.
planners = {
    rid: OfflineDecisionPlanner(callbacks[rid],
        evidence_kind="deterministic_physical_replay")
    for rid in ("r1", "r2", "r3")
}
result = run_rgb_communication_async(
    actor_port, planners, condition="none", common_task=public_static_task,
    limits=AsyncRuntimeLimits(
        max_ticks=3601, tick_period_s=.05, poll_period_s=.05,
        decision_period_s=1.0, max_calls_per_robot=181,
        wall_timeout_s=600,
    ),
    trace_path=output / "runtime.jsonl", artifact_dir=output / "runtime-inputs",
    provenance=trace_only_provenance,
)
```

`OfflineDecisionPlanner` callback은 `{action, message}`를 반환한다. 요청 ID/revision echo와
외부 호출/토큰 0은 wrapper가 붙인다. fixture의 완료 주장을 물리 성공으로 바꾸지 않는다.
`harness.rgb_communication_runtime.run_rgb_communication_async`에도 wrapper가 있다.

- `tick_period_s`: SIM advance. 현재 async는 SIM clock만 허용한다.
- `poll_period_s`: 최소 wall pacing. clock catch-up burst를 하지 않는다.
- `decision_period_s`: 로봇별 새 관측/판단의 최소 SIM 간격. physics tick과 별도다.
- `max_observation_age_s`: 답변 적용 시 원본 관측의 나이. 기본 5초이며 B strict 상한도 5초다.
- port의 tick/이미지 캡처와 provider의 로컬 prepare는 유한·비차단 계약이다. 임의로 막히는
  네이티브 포트 호출까지 Python scheduler가 강제 중단한다고 주장하지 않는다.

모델용은 `make_rgb_planners(settings, archive_dir, *, robot_ids, http_open, evidence_kind)`이다.
`ProviderSettings`의 model 기본값은 기존 proxy 코드의 `DEFAULT_MODEL`을 재사용한다.
개발 담당 모델(Astra)이나 문서의 과거 모델명을 실험 모델로 자동 채택하지 않는다.
D는 실제 환경에서 모델을 명시하고 가용성/권한을 별도 확인해야 한다.

## 오래된 판단과 취소

요청에는 관측 ID·원본 image/issued-command 해시·자기 상태·수신 보고·로컬 계획 epoch가
결합된 `evidence_revision`이 있다. 응답은 request_id/revision을 정확히 echo해야 한다.
답변 수락 전에 원본 나이, 시간 상한, 취소, 현재 자기 동의/명령 상태, 수신/만료 보고를
검사한다. 늦은 답·중복/다른 요청의 답·철회나 파트너 변경 보고 전의 답은 적용하지 않는다.
거절된 답의 사용량과 응답 원문도 보존한다. 제출의 observation_id/requested_at_s를
새 관측으로 바꿔 오래된 판단을 신선한 판단으로 위장하지 않는다.

B2와 합의한 strict `own_revision`은 **고수준 task/consent generation**이다.
새 task·manual command·pending cancel·pause/resume·lease expiry/release·STOPPED/
FINISHED_UNVERIFIED는 증가한다. 같은 동의 안에서 예상되는 자동 servo/macro pulse와
RUNNING phase 진척만 freshness 비교에서 제외한다. 이들까지 매 tick 변경으로 처리하면
실행 중 pause/interrupt 판단도 무한히 거절되기 때문이다. 해당 원본 pulse history와
관측은 요청 evidence에 그대로 결합/보존하며 나이 제한은 면제하지 않는다.
revision이 없는 legacy 포트에서는 전체 자기 명령/상태 fingerprint를 비교한다.

`RuntimeControl.cancel_inference(rid)`는 판단만 취소한다. 수락된 lease 철회는 actor의
독립 action이다. `control.stop.set()`은 전체 실행 중단이며 port.close로 소유 작업을
정리한다. 이미 전송한 HTTP 요청을 서버에서 취소했다고 주장하지 않는다. 응답이 배수되는
동안 in-flight slot/예약비용을 계속 점유하며 같은 로봇의 두 번째 호출을 열지 않는다.
종료 시 미회수 요청은 `pending_planner_requests`에 남는다. worker는 이후 동작을 제출할 수
없지만 archive에 늦은 응답을 쓸 수 있으므로 원본 회수 완료/hash 고정은 D가 별도 확인한다.

## 비용·결과 의미

실제 전송 전에 전체 직렬화 입력(과거/현재 image 포함)의 **증명된 최대 입력토큰**과
제공자가 강제하는 출력 상한을 예약한다. `input_token_bound(bytes)` callback과
`input_bound_evidence/output_limit_evidence`가 없으면 `PROVIDER_BOUNDS_UNAVAILABLE`로
외부 호출 전에 차단한다. callback은 검증된 로컬 capability이며 네트워크 호출/추정량이 아니다.
D JSON 설정이 임의 callable/문자열 선언만으로 이 권한을 만들 수 없도록 resolver가 닫혀 있다.

현재 기존 proxy 코드에는 이를 입증하는 token-count/enforcement 계약이 없다.
**기본 readiness=false이며 실제 모델 실행 준비 완료가 아니다.** endpoint 접속·모델 목록도
이번 작업에서 조회하지 않았다. 정상 응답의 token usage는 있을 수도 없을 수도 있다.
usage가 빠지면 0으로 세지 않고 해당 최대 예약량을 유지한다. 검증된 bound를 제공자가
위반하면 수치를 clamp하지 않고 새 admission을 중단하고 모든 실행을 닫는다.

- `planner_decisions`: offline 포함 모든 admitted 판단 수.
- `external_model_calls`: 응답/오류에서 실제 send가 확인된 호출 수.
- `external_model_call_upper_bound`: 아직 회수되지 않은 요청을 포함한 admission 상한.
- `unknown_external_call_requests`: 전송 여부가 아직 확인되지 않은 live 요청.
- `tokens`: measured/unknown-call/charged(알 수 없으면 예약량) 입력·출력 토큰을 따로 기록한다.
- message 비용은 sender/recipient/content/ID/condition/TTL을 포함한 직렬화 bytes, 건수로 제한한다.
  별도의 tokenizer가 없는 message-token 수를 측정했다고 하지 않는다.
- `run_finished.outcome=completed`와 `actor_finish_claims/details`는 actor 정상 종료/주장이다.
  timeout/api_error/aborted와 물리 mission_complete는 분리된다. D가 별도 evaluator로 판정한다.

## E0 · 개발/시험 정보 분리

`provenance`는 run_started에만 저장하며 planner에 전달하지 않는다. 관측/status/static_context는
정확한 허용 필드만 통과한다. split·training seed·시험/사건 정답을 삽입하면 fail closed한다.
사람이 읽을 수 있는 run_id도 시험 분할을 누설할 수 있어 actor 요청에는 opaque hash로 준다.

`ProviderSettings.policy_manifest()`는 model/temperature/reasoning/output/timeout/과거영상 수,
system prompt SHA, adapter SHA, 합성 policy SHA를 준다. D는 여기에 controller/tool/skill hash,
데이터/맵 부모 출처·개발 노출 ledger·budget·scheduler를 묶어 비교군에서 고정해야 한다.
허용된 정적 지도의 ID/기하 설명은 actor에 남는다. 경로/지도 설명에 시험 성적을 섞지 않는다.
이번 변경 자체가 train/dev/Test A/Test B 분할의 적합성이나 foundation model 사전학습의
무오염을 증명하지는 않는다. A/D의 별도 E0 gate를 유지한다.

## 검증 기록과 남은 단계

- 관련 C/B/D/proxy/camera/CoELA 로컬 회귀: **121 passed, 29 subtests passed**.
  외부 요청은 byte-recording HTTP stub으로만 대체했다.
- 이후 취소된 transport의 drain 뒤 새 판단을 허용하는 회귀를 추가했고 최종 C 전용
  세 모듈은 **48 passed**. 전체 공용 CI는 별도 확인 후 PR에 상태를 남긴다.
- 지연 r1을 남겨도 r2/r3 완료·SIM 진행·종료가 유지됨, 최대 동시성·로봇별 1회,
  취소/관측나이/ID/자기 상태/보고 만료/partner retract, 독립 cancel→새 partner 요청 검사.
- 실제 serializer의 두 현재 영상 및 두 과거 영상, wire/body 일치·원본 archive·usage 미측정
  예약·사전 token 부족 차단·잘못된 JSON 보존·direct split injection 거절 검사.
- 작업 중 B2 strict 포트와 endpoint stub 연결: r1의 task_request→pause→resume→interrupt
  4건 ACCEPTED, 원본 관측 correlation과 revisions 0→2→3→5→6 확인. 최초 진단의 임의
  object ID는 static allowlist 밖이라 거절됐고 선언된 box_01로 고쳐 재검사했다.
- 작업 중 D2 evaluator에 C async의 17 event를 직접 검증하고 완료 주장+physical false를
  failure/false_finish_claim, physical true를 success로 분리하는 score 연결을 확인했다.
  B/D 작업 중 소스 검사이며 최종 고정 통합 SHA의 전체 재검증을 대체하지 않는다.
- 실제 모델/물리 결과 없음. D 단일 제출자의 Colab replay/회수, A 독립 최종 감사,
  provider hard-bound 검증과 실제 endpoint readiness, 고정 SHA CI/승인 단계가 남는다.
  기존 ACT/뷰어 프로세스·데이터·다른 작업 worktree는 수정하지 않았다.
