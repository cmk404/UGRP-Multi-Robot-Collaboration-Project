# A2 실행 가능한 시나리오·증거 API

코드 기준 `936bf821b873c9a364a768c4beea7c3debf27dd7`. 실제 물리/LLM 실행 없음.
`harness.rgb_communication_scenarios`는 evaluator-side 전용이다. catalog나 links를 actor에
주입하지 않는다. 별도 성공 oracle, 역할 배정기, 물리 사건 주입기를 만들지 않는다.

## D 연결

```python
from pathlib import Path
from harness.rgb_communication_scenarios import load_scenarios, assess_readiness

specs = load_scenarios("tests/fixtures/rgb_communication_scenarios/catalog.json")
report = assess_readiness(
    spec, evidence, expected_source_sha=execution_sha,
    artifact_root=Path(retrieved_artifacts), source_root=Path(execution_checkout),
)
# llm_smoke/pilot admission은 report['ready']가 True여야 한다.
# physical_replay 진단은 D의 별도 stage·LLM0·2회·상한 계약이다.
```

현재 catalog의 `execution.components/pins`는 null이다. 실제 backend와 환경을 검증하기
전에는 채우지 않는다. 선택 후보는 normal + public contention(6회), recovery는 관측
근거 확보 전 보류다. 공용 top 사건은 사적 정보가 아니다. 정상 후보조차 최종 backend의
실제 영상/스킬 증거를 아직 갖추지 않았으므로 후보 선정은 readiness 통과가 아니다.

evidence의 정확한 필드:

- `schema_version: rgb-scenario-readiness.v1`, `evidence_kind: independent_review`
- `source_sha`, `scenario_sha256` (`canonical_sha256(spec)`)
- `components`: serializer_id/backend_id/scheduler_id; spec과 같아야 함
- `pins`: setup/map/physics/camera_sha256; spec과 같아야 함
- `source_files`: 상대 소스 경로→파일 SHA-256. 최소 B 계약/포트/스킬, C runtime/planner,
  D study/runner 파일. 추가 import/transitive 제어 모듈도 독립 감사에서 포함해야 함.
- `boundary_audit`: `{path, sha256}` JSON 파일 참조
- `physical_replays`: solo/joint 두 `{path, sha256}` JSON 파일 참조
- `visual_witnesses`: event_id/robot_id/observation_id/visibility/observed_at_s/frame/review

`path`는 artifact_root 아래 상대 경로다. 존재·범위·실제 SHA를 확인한다.
소스 파일은 source_root를 기준으로 확인한다. 절대/상위 탈출·symlink 탈출은 거부한다.

모든 review JSON은 공통 `binding`(source_sha/scenario_sha256/components/pins/source_files),
`scope`, `verdict: pass`, `evidence_kind: independent_review`를 가진다.

- boundary scope=offline, independent_reviewer=A2, `cases`는 AUDIT_CASES의 모든 ID→pass,
  `artifacts`는 실제 오프라인 감사 원본 참조 목록이다. characterization green은 pass가 아니다.
- replay scope=physical, kind=solo 또는 joint, model_calls=0, weld_enabled=false,
  replay_goal_complete=true, target_object_ids=solo `["box"]`/joint `["beam"]`.
  raw mission_complete bool도 그대로 보존한다. 단일 목표 replay가 전체 2+1 임무를
  완주했다는 뜻이 아니다. artifacts는 회수한 실행/평가/영상 원본 참조 목록이다.
- image scope=visual, independent_reviewer=A2, criterion_visible=true,
  private_information_claim=false, `witness`는 review를 제외한 정확한 witness 사본이다.

해시는 무결성 검사이지 서명/신원 인증이 아니다. 독립 검토자가 직접 검사한 기록만
사용한다. 테스트는 임시 디렉터리의 가짜 리뷰로 **검사기의 분기**를 확인하며 실제
독립 리뷰나 물리 증거로 저장하지 않는다. 모델/자원/예산 gate는 D/C의 별도 필수 조건이다.

## 사건과 판단 연결

`validate_episode(spec, runtime_events, links, condition=...)`는 원본 event_id를 이용한다.
links schema는 `rgb-scenario-links.v1`: scenario_id/condition, physical_event,
recognitions다. physical_event는 event_id/reached/recovered/evaluator_artifact_sha256.
각 recognition은 robot_id/observation_event_ids/decision_event_id/report_event_id/
revision_event_id/cancel_event_id/prior_command_event_id/command_event_id를 가진다.
optional 참조는 null이고, 보고하지 않은 결정은 silence로 보존한다.

검사는 실제 planner request에 포함된 같은 actor의 image hash, 시간순서, report의
decision_id, revised decision의 action 변경, command_issued의 decision_id/command_id와
실제 이전 명령 대비 변경을 확인한다. before/after stall은 같은 actor의 서로 다른
관측 시간과 자기 명령 증거가 필요하다. 물리 사건에 도달했어도 recognition이 없을 수
있으며 이를 정상적으로 기록한다. annotation의 의미 해석은 별도 영상/원문 리뷰다.
이 연결 검사는 메시지의 인과 효과나 물리적 성공을 증명하지 않는다.

`validate_episode(..., artifact_root=Path(...), require_wire_images=True)`는 C의
planner_responded.payload.artifacts[".request.json"]을 실제로 열고 SHA·request_id·actor와
전송된 image_url의 JPEG 해시를 검사한다. 과거 프레임의 hash/ref만 텍스트에 들어 있으면
그 프레임을 본 것으로 세지 않는다. 기본 metadata 검사는 이 단계와 달라 결과의
`model_image_exposure_verified`가 false다. live 사건 해석에는 strict 옵션이 필수다.
거부된 late/cancelled reply는 planner_responded 기록이 있어도 인식 근거로 인정하지 않는다.

E0의 분할·노출 및 환경 gate는 [a2-environment](a2-environment.md)를 따른다.
`assess_readiness`와 E0, C provider 예산, D admission은 모두 독립 필수 gate다.

```sh
python -m harness.rgb_communication_scenarios \
  --catalog tests/fixtures/rgb_communication_scenarios/catalog.json
python -m unittest discover -s tests -p test_rgb_communication_scenarios.py -v
```

`--evidence FILE --scenario-id ID --expected-source-sha SHA --artifact-root DIR
--source-root DIR`를 추가하면 live 증거 무결성 gate를 실행한다. NO-GO 종료코드=2.
