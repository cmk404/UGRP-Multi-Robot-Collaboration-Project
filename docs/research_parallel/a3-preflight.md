# A3 · 새 local source/config 독립 감사 계약

`rgb-offline-boundary-review.v2`, `independent_reviewer=A3`를 사용한다. `assess_offline_boundary(..., required_schema="rgb-offline-boundary-review.v2")`로 v1 downgrade를 거부한다. 기존 v1/A2는 과거 읽기 호환만 유지한다. 최종 source SHA, 전체 관련 파일 해시, offline reference 자신만 제외한 config 해시, serializer/backend/scheduler ID와 원본 감사 artifact 해시가 전부 일치해야 한다. 단순 fixture pass나 이 문서가 인증서는 아니다.

필수18항목은 `A3_AUDIT_CASES`가 정의한다. 기존11항목(허용 입력·평가 비간섭·수신자·TTL·느린 동료·취소·중복 공동 의도·lease·queued expiry·finish 분리·request→command 출처)에 다음7항목을 더한다.

1. `worker_total_wall_and_sim_age_at_consumption`: 제출→소비 total wall2초, capture→소비 SIM1초 유지. 완료<2초/poll>2초, 미완료>2초, 완료>2초를 구분하여 모두 기존 소비 gate대로 판정한다. 완료 시간 계측이 허용 범위 변경은 아니다.
2. `completed_worker_cancel_and_revision`: 완료 뒤 취소/actor revision/consent 변경이 있으면 stale 결과를 적용하지 않는다.
3. `local_child_finalization_before_hashing`: normal/TERM/KILL 후 자체 자식이 실제 종료·회수된 다음 최종 해시. 실행 중 쓰기·unknown actual clock을 완료로 승격하지 않는다.
4. `scene_backend_definition_binding`: native58과 B실제지원 분리, exact dispatch map/reset/goal/camera와 최종 XML·관측크기 차이 명시. unsupported alias 금지.
5. `asset_and_parent_map_exposure_integrity`: read-only 실제 자산 재해시·manifest와 descriptor 결박, ACT 부모 지도·기존 노출·unknown prompt lineage 구분.
6. `observation_cache_capture_identity_and_isolation`: 원래 capture SIM/wall/hash 유지. cache 조회 시각으로 capture timestamp를 새로 찍어 freshness를 우회하지 않는다. own RGB는 로봇 간 공유 금지. physics/명령/reset/episode/camera 변경 경계는 무효화 또는 새 cache로 분리한다.
7. `single_clock_service_fairness`: future 회수 순서를 바꾸더라도 단일 clock·actor별 결정 기회·bounded tick/cancel/consent 검사를 유지한다. 모델 수·실패 분모를 숨기지 않는다.

## 감사 순서와 권한

- D가 B/C/A 및 공통 source-closure 변경을 통합하고 SHA를 고정한다. 기존 모델·환경은 복제하지 않는다.
- D의 새 준비 config/descriptor/local asset catalog/evidence는 최종 SHA로 다시 만든다. A가 정보 경계·실행 경계 전체를 독립 검토하고 반례를 실행한다(가짜 endpoint/원본 파일 읽기만).
- A는 `pass` 또는 blocker를 명시한다. 과거 bfe478f 인증과 f139672 clock10항목 pass는 이전되지 않는다.
- D만 자원 조정 후 승인2회 실제 실행한다. A3 offline pass는 물리 성공·live LLM pilot 승인·추가 실행 예산이 아니다.
- 회수 뒤 A는 영상/요청 이미지/실행/평가 원본을 독립 검토하고 반례만 B/C에 보낸다. D가 성공·실패 모두 TensorBoard 변환·실제 로딩·화면 확인을 담당한다.

현 단계에서는 새 D 최종 실행 SHA/config가 아직 제공되지 않았으므로 A3 실행 전 인증을 발행하지 않았다. 새 물리 결과도 없다.
