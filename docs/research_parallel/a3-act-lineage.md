# A3 · ACT 노출 계보와 다음 구조적 비교 준비

이 기록은 기존 파일의 출처 감사다. 학습·추론·기존 진행 중 평가를 실행하거나 완료로 표시하지 않는다. 모델 성능·통신 효과 비교도 아니다.

## 직접 확인한 원본

ACT 소유자가 제공한 `act-parallel-review/ugrp/experiments/2026-09-22-research-controller-qualification/`의 acceptance-v3-full-protocol.json, runtime-v3-manifest.json, terminal-candidate-development-audit.json, README.md를 읽었다. 이어 protocol의 자산22개를 원본 경로에서 재해시하고 expanded dataset→training report→retained checkpoint와11개 episode의 metadata66개를 검사했다. 원본 모델/데이터는 복사하거나 변경하지 않았다.

- expanded dataset: `83dbd7707aa0fc464a1db70fd1ea87f5603ab37c4346476a299b7386158bfd95`
- retained checkpoint: `c98a640167110f7ee0b5fc2b1d5edde4a4065464c6c47519b366cc3ff8ee4e96`
- dataset 역할은 train8 / development3이며, 부모 정적 지도는 open/shared_crossing/narrow_south/rough_south의 기존 dispatch4정의다.
- train과 development가 open/shared_crossing 부모를 공유한다. episode 단위 역할 분리는 구조적 지도 holdout이 아니다. 역사적 부모의 effective split은 regression으로 기록한다.
- 현재 qualification3개는 open·seed11·dock_a에서 초기 offset만 달라진 노출된 고정 회귀 사례다. 새로운 부모 지도 또는 Test A/B로 세지 않는다.
- 새 terminal 후보는 개발 단계 채택에 실패했고 protocol은 이전 expanded checkpoint를 유지한다. 이번 감사는 그 선택·해시 연결만 확인한다.

episode의 committed plan 해시도 dataset에 결박되어 있으나 이는 재생된 계획 자산이지 새로운 독립 LLM 판단이 아니다. 모든 과거 프롬프트·foundation pretraining 노출을 완전히 재구성했다고 주장하지 않는다. RGB 이미지의 기록된 해시는 dataset 해시에 포함되지만, 이번 감사에서는 이미지 원본 전체를 재해시/시각 검토하지 않았다.

## 재현 도구와 산출물

`tests/fixtures/rgb_communication_scenarios/audit_a3_metadata.py`는 명시적 qualification/dataset/training-report/checkpoint 경로를 읽어 장면58개 매트릭스와 출처 계보를 하나의 새 JSON에 저장한다. 이미 있는 보고서를 덮어쓰지 않는다. `heavy_runtimes_imported=[]`를 검사하며 MuJoCo/Torch import·world·추론·렌더링이 없다. 원본 절대 경로와 실제/기대 해시·부모 지도/geometry/reset·effective split·미검증 범위를 저장한다.

개발 확인 packet은 로컬 `outputs/a3-metadata-development.json`이다. 최종 커밋 뒤 새 packet을 별도 이름으로 만들어 전달한다. raw assets는 로컬에만 있으며 해시 기록이 원격 백업은 아니다. Google Drive는 사용하지 않는다.

## T1/T2 전제: 아직 입장 불가

기존 `audit_exposure`와 `validate_training_comparison`의 fail-closed 검사를 유지한다. 새 suite는 train6/dev3/TestA3/TestB2/control2/regression6을 구분하고, augmentation/frame/dataset/checkpoint/prompt가 모든 부모 split을 전이 상속해야 한다. 같은 parent의 seed/goal/초기 offset·물체 수 반복은 지도 다양성 증가로 세지 않는다.

T1/T2는 지도 분포만 다르게 하고 episode·transition·frame 수, updates/batch, architecture/input/init checkpoint, device/precision을 맞춘다. T2의 train 부모 집합은 T1의 엄격한 상위집합이어야 한다. 두 개 이상 고정 training seed, 개발 전용 선택 규칙 해시, 전체 Test A/B 고정 평가가 필요하다. 데이터 양도 늘리는 T3는 별도 비교다. 알려지지 않은 출처를 diagnostic-only로 받더라도 heldout/T1T2 입장을 승인하지 않는다.

현재 B가 이 새 지도 운반을 지원하지 않고, 데이터/프롬프트 계보의 미확인 범위도 남아 있으므로 `T1_T2_admission=false`, `heldout_claim_ready=false`, `execution_admission=false`다. 추가 학습·teacher 생성·안전대조 실행은 이번 승인의 범위가 아니다.
