# A2 독립 감사 · 기준선과 최종 경로를 구분

## 통합 출발점: NO-GO

감사 대상은 `936bf821b873c9a364a768c4beea7c3debf27dd7`이다. 정확한 네 production
파일 해시와 재현 관측은 tests/fixtures/rgb_communication_scenarios/baseline_audit.json.
로컬 오프라인 fake endpoint/clock만 사용했다. 실제 시뮬레이션·렌더링·외부 LLM 0회.

| ID | 실제 재현 | 담당 수정 계약 |
|---|---|---|
| A2-01 | lease10s 안에서 t0 r1 명령이 t6의 r2 명령과 실행됨. request age 상한5s를 dispatch에서 재확인하지 않음 | B: 큐 실행 직전 원래 발행시각·lease 검사, group hold/release |
| A2-02 | 동의 대기 intent를 interrupt하려면 아직 없는 lease가 필요하여 철회가 거부됨 | B/C: cancel_pending, 새 task ID, 결정 revision, 늦은 이전 답 거부 |
| A2-03 | r1 호출이2wall초 지연되면 r2/r3 최초 요청도wall2에 시작. 늦은 r1 답은 거부됨 | C: 각 actor 단일 in-flight 비동기 판단, clock/다른 actor 진행 |
| A2-04 | LOCAL_COMMAND에 command ID는 있으나 decision/observation ID가 없음 | B/D: 실제 발행 event에 두 원래 ID를 보존 |

위 4개 characterization test의 green은 **결함 재현**이다. source hash가 달라지면
과거 버전 전용 테스트를 skip한다. 이 skip은 최종 경로 pass가 아니며 아래 독립 gate를
별도로 실행해야 한다. 통합 때 고친 코드에 과거 결함을 다시 강요하지 않는다.

같은 기준에서 duplicate joint intent/만료·취소된 lease의 늦은 명령/recipient 격리/
delivered TTL/actor finish와 evaluator 비간섭은 positive regression으로 검사했다.
정적 context·frame·status allowlist 및 queued TTL은 기존 B/C/A 검사도 함께 실행한다.

## 최종 B/C/D SHA에 재적용

최종 source SHA와 전체 제어 경로 파일 해시를 고정한 뒤 아래 AUDIT_CASES를 각각
새 공개 API에 적용한다. 새 async/skill/study 파일 및 가져오는 serializer/control 모듈을
source_files에 추가한다. 일부 담당 test 통과를 전체 독립 compliance로 옮겨 적지 않는다.

1. **actor_source_allowlist / evaluator_noninterference**: 같은 JPEG·자기 명령·수신 메시지를
   고정하고 hidden evaluator/peer state/seed/split만 바꾼 paired run. 실제 provider request
   body, planner 시작 시각, 제출/발행 명령이 같아야 한다. evaluator는 actor 실행 중 호출 금지.
2. **recipient_isolation / queued_and_delivered_ttl**: r1→r2만 전송, r3 request에 payload
   없음. 도착 전 TTL, 이미 받은 뒤 TTL, inference 중 TTL을 각각 검사. expired content가
   active memory/행동 권한으로 남지 않아야 한다.
3. **slow_peer_progress / cancelled_late_reply**: 느린 r1을 event latch로 보류하며 r2/r3와
   단일 clock이 진행하는지 확인. cancel/own revision/파트너 변화/만료 뒤 r1 응답은 기록만
   남기고 메시지·명령을 실행하지 않는다. wall budget에 종료 중 in-flight도 포함한다.
4. **duplicate_joint_intent / lease_expiry_and_reuse / queued_command_expiry**: 새 request ID의
   중복 intent, pending 취소, active pause/interrupt, lease 만료와 폐기 task 재사용. 느린
   동료를 기다린 옛 명령은 합법적인 새 lease로 세탁되지 않아야 한다. 다른 solo는 진행한다.
5. **finish_claim_separation / request_decision_command_provenance**: 거짓 완료 주장을
   evaluator가 모델 입력/종료에 교정하지 않아야 한다. 실제 actor finish, runtime 종료,
   raw mission verdict, replay 목표 verdict를 각각 남긴다. 원래 request/observation/
   decision/command/lease ID와 이미지·wire text hash를 원본 파일까지 확인한다.
6. **scenario/E0**: 실제 JPEG에서 사건을 볼 수 있는지 독립 검토한다. 과거 프레임 해시만
   텍스트에 있다는 사실은 모델이 이미지를 봤다는 뜻이 아니다. actual wire의 image parts까지
   `validate_episode(..., require_wire_images=True)`로 검사한다. E0 split/exposure/환경 대조
   gate 및 C provider bound/D 예산 gate도 모두 필요하다.

## 연결·본실험·인과 주장의 경계

초기 6회는 정상+공용 경쟁 후보 × none/structured/natural, 각각 같은 scene/setup/seed,
RGB/스킬/scheduler/clock/모델/기억/토큰·메시지·명령 상한이다. map/카메라/물리 해시가
없으면 실행 불가다. 경쟁이 자연스럽게 발생하지 않은 trial도 남기며 harness가 로봇 역할을
강제해 사건을 만들어내지 않는다. 복구 후보는 before/after 관측·명령 증거 확보 전 제외하되
결과를 본 뒤 성공한 조건으로 바꾸는 것이 아니라 **실행 전** 후보 선택 사유를 고정한다.

본실험은 맵별 paired block + 반복 training/model/physics seeds, 미리 정한 표본 수·
최소효과·불확실성 분석을 사용한다. 현재 함수는 schedule을 만들 뿐 적절한 표본 수를
자동 승인하지 않는다. 주 통신 효과 뒤 message_delay/message_limit/memory_limit/
scheduler ablation을 별도 block으로 비교한다. 동시에 두 축을 바꾸지 않는다.

분모는 전체 사전 배정 trial이며 unrun/API오류/실패/timeout/중단을 보존한다. 성공한
trial의 시간과 전체 비용을 구분하며 미측정 토큰을0으로 만들지 않는다. 물리 사건 도달,
actor 인식, 보고/침묵, 수정/취소, 실제 명령 변화, 사후 복구 성공은 각각 집계한다.
실패해 협상 지점에 못 간 시행은 통신이 불필요하다는 근거가 아니다.

동일 상태에서의 메시지 전달/차단 인과 비교는 물리 상태·시뮬레이터 clock·각 actor
기억·모델 요청·명령 큐·lease·통신 큐·난수 상태까지 복원 검증이 필요하다. 일부 로그
재생이나 저장된 응답만으로 이 조건을 충족하지 못한다. 현재 analyzer는 언제나
causal_effect_established=false를 반환한다.
