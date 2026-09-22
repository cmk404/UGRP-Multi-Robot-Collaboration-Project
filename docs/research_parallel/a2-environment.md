# A2 E0 · 맵·출처·환경 정합성

요구 기준: 통합 구현 `936bf821`, 추가 E0 문서 `6588daf` (2026-09-22).
기존 `sim.act_map_suite.validate_splits`, `layout_digest`, 작성 map digest를 그대로 사용한다.
22개 기존 suite를 새로 만들거나 물리 실행하지 않는다. 모든 검사는 actor 밖이다.

## 검사 API

```python
from harness.rgb_communication_scenarios import (
    audit_exposure, validate_training_comparison, assess_environment_readiness,
)
from sim.act_map_suite import load_suite

_, cases = load_suite()  # train6/dev3/TestA3/TestB2/control2 + legacy regression6
exposure = audit_exposure(cases, provenance, scenario_assignments)
validate_training_comparison(training_plan, exposure)  # 학습 설계를 사용할 때만
environment = assess_environment_readiness(
    review, cases, artifact_root=artifact_root,
    expected_source_sha=source_sha, expected_backend_id=backend_id,
    purpose="physical_replay", allow_legacy_dispatch_open=True,
)
```

`purpose=physical_replay`는 물리 검증 증거를 **만들기 전**의 정적/reset 계약 검사다.
ready=true여도 physical_controls_verified=false이며 live 또는 heldout 성공이 아니다.
D의 LLM0·2회·회당180SIM/600wall·전체1800wall 제한을 추가 적용한다.

`purpose=live`는 실제 개발/불가능 대조 검사를 요구한다. B의 현재 지원은 기존
dispatch_open 운반이며 22-suite의 운반은 미지원이라고 B/D가 보고했다. legacy registry는
physical_replay에서 명시적으로 허용할 때만 `authored_map('open')`의 실제 정의로 검증한다.
이를 suite dev-open이나 새 맵으로 alias하지 않는다. live에 legacy 플래그는 거부한다.

## 출처/노출 스키마

provenance schema=`rgb-map-exposure.v1`, fields=records/exposures/freeze_order/final_test_ids.

각 records 항목:

- id, kind: episode/frame/augmentation/dataset/checkpoint/prompt/foundation_model/scenario
- parents: 부모 artifact ID 목록. 전체 조상 맵을 상속하며 순환·없는 부모를 거부한다.
- map_refs: `{map_id, map_sha256, layout_sha256}` 목록. 부모 없는 episode의 실제 맵 출처.
- declared_splits: 조상의 모든 원본 분할을 정확하게 열거한다. frame/증강에서 바꿀 수 없다.
- origin: known 또는 unknown. foundation 사전학습 미상은 unknown으로 기록한다.

각 exposures 항목은 record_id/use/order/artifact_sha256이다. use는 geometry_qa,
training, prompt_tuning, checkpoint_selection, demo, policy_diagnosis, final_evaluation.
order는 감사 가능한 기록 순서이며 벽시계 해석을 대신하지 않는다. 참조 artifact는 D의
pinned input에서 실제 존재/해시를 확인한다. API는 선언된 provenance의 연결을 검사하며
데이터 내용 전체나 알려지지 않은 foundation pretraining의 무오염을 증명하지 않는다.

정적 기하 QA만으로 heldout을 오염으로 분류하지 않는다. 반면 시험에서 진단·시연·튜닝을
했거나 동결 전 시험 평가를 했으면 effective_split=regression이다. 최종 결과를 본 뒤
다른 맵에서라도 정책/프롬프트를 튜닝하면 이미 본 시험은 새 정책의 미사용 시험이 아니다.
exposures의 배열 순서가 바뀌어도 order 비교로 같은 판정을 낸다.

final_test_ids는 원본 A/B 전체를 포함해야 한다. 실패 맵 사후제외는 거부한다. 오염 맵을
그대로 heldout으로 남기지도 않는다. 새 suite/프로토콜의 새 holdout을 사전 동결해야 한다.
scenario_assignments는 `{scenario_id,parent_map_id,map_sha256,layout_sha256,split}`이며
물체·사건·프레임 변형도 effective 부모 split을 상속한다. seed/시작점 변화는 맵 수가 아니다.

## T1/T2/T3

training_plan schema=`rgb-training-comparison.v1`, arms/fixed_test_ids/training_seeds/
selection_rule_sha256를 둔다. 최소 T1/T2, 선택적 T3다.

각 arm은 id/map_ids/episodes/transitions/frames/updates/batch_size/
architecture_sha256/input_sha256/initial_checkpoint_sha256/device/precision을 가진다.
T1/T2의 map_ids만 달라야 한다. T2는 T1의 맵을 포함한 더 넓은 분포다. 데이터량은
episode뿐 아니라 transition/frame 수까지 같아야 하며 update/batch/구조/입력/초기
checkpoint/장치/정밀도를 일치시킨다. 모든 arm은 같은 반복 training seed(최소2),
개발 선택 규칙, 전체 fixed Test A/B를 사용한다.

T3는 T2와 같은 map 집합에서 데이터량을 늘리는 별도 비교다. 추가 update를 기록하되
T1↔T2의 다양성 효과로 합치지 않는다. 실험 실행/표본 수 승인·비용 산정은 별도다.
이 API 통과가 실제 학습이나 성능 비교의 완료를 뜻하지 않는다.

## 환경 증거 스키마

review schema=`rgb-environment-readiness.v1`, source_sha/backend_id/scope/maps/checks.
maps 항목은 map_id/map_sha256/layout_sha256/scene/camera/reset/evaluator_config/capability.
scene/camera/reset/evaluator_config는 `{path,sha256}` 실제 artifact 참조다.
capability는 transport 또는 safe_stop이며 unsupported를 open 경로로 대체할 수 없다.

physical_replay scope=`static_reset_preflight`, checks는 다음 정확한 5개다:
map_scene_goal_correspondence, fixed_camera_configuration, reset_seed_contract,
single_physics_clock, weld_off. 각 check는 실제 정적 계약 근거를 가진다.

live scope=`development_physical_controls`, checks는 정확한 10개다:
map_scene_goal_correspondence, initial_nonpenetration, loaded_clearance,
fixed_camera_visibility, reset_seed_consistency, observation_timestamp_alignment,
easy_development_success, impossible_safe_stop, single_physics_clock, weld_off.
쉬운 개발 맵과 불가능 control 맵을 모두 포함해야 한다. 교사 정답 경로는 actor 성공으로
사용하지 않는다. 초기 관통/하중 폭/고정 카메라 가림/seed reset/관측시각·명령 대응을
원본에서 검토한다. 실제 목표 운반과 안전 정지를 각각 판정한다.

checks 값은 `{path,sha256}` JSON 참조다. JSON fields는 check_id/verdict=pass/
source_sha/backend_id/maps_sha256/evidence_kind/artifacts. maps_sha256는 전체 maps의
canonical_sha256이다. evidence_kind는 정적 진단=static_contract_check, live=
physical_diagnostic. artifacts는 비어 있지 않은 raw 참조 목록이며 실제 해시를 확인한다.
기하/모의 fixture를 physical로 승격하는 기록을 만들면 안 된다. checker는 서명 시스템이
아니므로 담당자 원본과 독립 검토를 반드시 유지한다.

## 현재 ACT 노출 이력의 사용 범위

ACT #92 담당자가 2026-09-22 전달한 **진행 보고**: expanded dataset SHA-256
`83dbd7707aa0fc464a1db70fd1ea87f5603ab37c4346476a299b7386158bfd95`, train8/dev3,
open/seed11/dock_a 출발 교란. 기존 test-minus/plus/yaw는 진단 노출된 회귀 조건이며,
qualification-1도 진단에 노출됐다고 알렸다. 실행 고정 소스 `738c4de`, 설계 PR92
`9a1d2f8`; 진행 중 원본 경로는 research-controller-validation worktree의
outputs/research-repair/qualification-v2다. 이 작업은 그 결과를 회수·재평가하거나
기존 소스를 수정하지 않았다. 이 보고만으로 Test A/B 또는 동일량 다양성 효과를
판정하지 않는다. 향후 실제 dataset/checkpoint 부모 manifest를 가져와 위 schema에
연결해야 하며, 지금 완료 provenance로 채워 넣지 않는다.
