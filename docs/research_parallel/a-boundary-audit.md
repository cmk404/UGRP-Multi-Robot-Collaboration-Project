# A · 현재 RGB/CoELA 정보 경계 감사

2026-09-22, production 기준 `120cc821b6a1d5c104aab8c8ef2260cf7f8c9a7b`.
소스 지문·기존 요청 표본은 [source_manifest.json](../../tests/fixtures/rgb_communication_audit/source_manifest.json),
검사는 [test_rgb_communication_boundary_audit.py](../../tests/test_rgb_communication_boundary_audit.py).
production을 수정하지 않았다. 아래 판정은 이 스냅샷에만 적용한다.

## 실행 경로 분류

| 경로 | 실제 입력/실행 | 이번 비교에 대한 판정 |
|---|---|---|
| `mixed_warehouse_runtime.py` | global available/idle/history가 prompt와 wake에 들어감 | legacy 비교용; RGB 독립 경계로 이식 불가 |
| `coela_runtime.py` → `MixedEngine(strict_local)` | 로컬 메모리/inbox, RGB-D ArUco range/bearing, `ideal_sim_geometry` 실행·완료 | 통신 모듈 참고; RGB 준수/물리 제어 증거가 아님 |
| `camera_runtime.py` → `CameraPlanner`/`CameraRobotPort` | 자기 robot_cam JPEG, 자기 명령, 독립 futures, bounded raw commands, 사후 평가 | 현재 좁은 RGB 경계; 2+1 고수준 공동 스킬 미연결 |
| dispatch `TeamAgreement` → `DispatchExecution`/skills | 자기 RGB+top+지도, full proposal/committed plan, 전원 정확 ACK | RGB 입력과 shared-plan 출하 경로; 독립 A/B/C 효과 분리에는 추가 채널/장벽 제거 필요 |
| 신규 B/C | actor port와 통신 runtime 별도 구현 중 | 계약 초안만 전달받음; 통합 SHA·샘플에 대한 재감사는 별도 gate |

## actor × 정보 출처

R1/R2/R3에 동일한 권한 규칙을 적용한다. i는 수신 actor, j는 다른 actor다.

| 필드/출처 | R1 | R2 | R3 | 추적·검사 경계 |
|---|---|---|---|---|
| `own_rgb` | r1 camera | r2 camera | r3 camera | raw bytes/hash·frame ID·캡처 시각; j의 own RGB 자동 공유 금지 |
| `top_rgb` | 공용 top | 공용 top | 공용 top | 실제 받은 top frame별 기록; 공용에 보이는 사실을 private로 분류하지 않음 |
| 자기 발행 명령 | r1 명령만 | r2 명령만 | r3 명령만 | servo target/모터 명령은 측정 관절·이동 성공과 다름 |
| 과거 영상·자기 기억 | r1 과거만 | r2 과거만 | r3 과거만 | 관측 시각을 기억 갱신 시각으로 덮어쓰지 않음; 같은 보존 정책 |
| 정적 과제/지도/카메라 보정 | 동일 | 동일 | 동일 | 고정 버전·해시·출처; live pose/완료 목록을 static 문자열에 삽입 금지 |
| 실제 peer 메시지 | r1 수신함 | r2 수신함 | r3 수신함 | none은 비어 있음; sender/recipient/ID/송신·수신·만료 시각; 미전달 메시지 열람 금지 |
| 자기 요청·접수/거부·lease | r1 요청 | r2 요청 | r3 요청 | 자기 tuple만 반환; 다른 작업/역할/점유자의 상세는 비공개 |
| 공동 GO/HOLD | 참여 시에만 | 참여 시에만 | 참여 시에만 | 명시한 참여자 간 matching의 최소 bit; 제3자의 wake 금지 |
| 완료 | 자기 RGB claim | 자기 RGB claim | 자기 RGB claim | peer 완료는 수신 주장 또는 직접 영상 해석; evaluator 완료 아님 |
| evaluator 좌표/접촉/성공/측정 관절 | 금지 | 금지 | 금지 | 별도 출력; 기억·깨우기·거부·route 후보·오류 문자열에도 삽입 금지 |
| peer private reason/plan 및 전역 상태 | 금지 | 금지 | 금지 | 허용 메시지로 명시 송신된 내용만 주장으로 수신 가능 |

현재 raw `CameraPlanner`에는 top과 static map이 없다. 신규 경로가 이를 추가하면
세 조건 모두 동일하게 받도록 한다. 정적 지도 좌표와 RGB에서 계산한 image 좌표는
실시간 정답 좌표와 다르므로 단순 키워드 blacklist만으로 허용 여부를 판단하지 않는다.

## 필드별 재현 결과와 수정 요구

### A-01 · 공유 계획을 통한 추가 채널 — 파일럿 차단

`harness/dispatch_plan.py:61`의 `build_dispatch_request`는 `agreement` 전체와
`reply_binding.plan_hash`를 넣는다. `TeamAgreement.context()`는 다른 로봇이 제안한
전체 task/participants/route/after를 전달하고, `harness/dispatch_execution.py:85`의
요청은 `committed_plan`을 포함한다. 빈 inbox여도 전체 제안을 읽는 재현 검사가 있다:
`test_empty_dispatch_inbox_still_receives_peer_authored_whole_plan`.

기존 실제 F1 `r2-plan-3-request.json` 1건의 텍스트/이미지 라벨도 읽고 해시를 확인했다.
`agreement.proposal.plan`에 r1/r3 봉·north, r2 상자·south 및 `after=[beam_job]`가
들어 있다. 이 실제 표본은 자신의 제안도 포함하는 협상 입력이고, synthetic 검사는
비제안 actor r3에게 전달되는 것을 재현한다. 표본은 source manifest의 로컬 원본을
가리키며 전체 코호트/영상/모델 동작을 재평가하지 않았다.

**B/C 요구:** 자기 request만 작성해 참여자 matching을 요청하는 경계로 연결한다.
공동 허가는 자기 tuple의 매칭 결과만 반환한다. all-robot TeamAgreement를 none의
실행 선행 조건으로 두지 않는다. plan hash는 전체 peer plan을 알아야만 제출할 수
있는 채널이므로 새로운 무통신 consent의 필수 입력으로 사용하지 않는다.

### A-02 · blacklist의 nested 평가 필드 방어 부족 — adapter 통합 차단

`harness/camera_policy.py:86,176`는 observation의 바깥 키를 제한하지만
`actuator_state`/memory 내부는 주로 spatial 키 blacklist로 검사한다.
`success`, `contact_count`, `measured_joints`를 actuator_state에 넣으면 실제 모델
호출의 JSON에 그대로 들어간다. `test_current_planner_blacklist_does_not_reject_semantic_oracle_fields`
가 각 payload의 전달을 확인한다. 실제 `CameraRobotPort`는 해당 키를 만들지 않으며
현재 live leak을 발견했다는 뜻은 아니다. 새 producer를 신뢰만 하면 생기는 방어 공백이다.

**B/C 요구:** 관측·자기 명령·lifecycle·error payload까지 source별 중첩 허용 필드를
정한다. static_context는 생성 시 고정하고 라이브 값을 넣지 않는다. 평가 정답만
바꾼 두 실행에서 actor JSON·기억·호출 시점·action이 같아야 한다. 자유문장의 진실성은
schema로 증명할 수 없으므로 생산자·출처 감사가 필요하다.

### A-03 · RGB runtime이 intent TTL을 적용하지 않음 — C 만료 계약 요구

`Communication.send`는 intent에 `intent_ttl_s=12`를 넣지만
`camera_runtime.py:88–96`은 메시지를 list memory에 보관한다. `Memory.active_reports()`를
호출하지 않으며 TTL 판정/expired 표시가 없다. 가짜 시계에서 수신 뒤 13초 이상이
지나도 다음 모델 입력에 같은 intent가 표시 없이 남는 것을 재현한다. 같은 메시지를
CoELA `Memory`의 receiver clock에서 경과시키면 active 목록에서 제외된다.

**C 요구:** clock domain·수신/만료 시각을 명시하고 active inbox와 expired claim을
구분한다. 오래된 내용 보관 자체가 금지인 것은 아니다. 갱신 없는 의도를 reservation,
busy 또는 completed 사실로 승격하지 않는다. 회복 판단 중 새 메시지/만료가 생기면
응답의 근거 버전을 재검사한다. TTL test는 thread 타이밍 대신 명시적 clock으로 작성한다.

### A-04 · 현재 raw RGB 행동은 공동 요청 능력이 없음 — 연결 전제

`CameraPlanner` 행동은 arm/drive/look/wait다. `request_joint`는
`INVALID_ACTION_FIELDS`로 거부되는 characterization test가 있다. 이는 raw API의
지원 범위이며 새 high-level adapter의 action 이름을 강제하지 않는다. 반면 기존
`MixedWarehouseReferee`는 별도 peer 메시지 없이 두 독립 claim을 matching하여
공동 예약할 수 있다. 이 프로토콜 성질을 신규 RGB 포트에서 다시 확인해야 한다.

**B/C 요구:** none에서도 같은 high-level submit과 모든 로봇의 실제 독립 판단을
지원하고, consent 부족은 실패/대기로 남긴다. 검증기가 화물·파트너·역할을 고르거나
누락된 claim을 생성하지 않는다. 등록된 운동 capability가 실제 환경을 지원하는지는
별도 실행 증거가 필요하다.

### A-05 · 관측/완료 ID는 발신자 주장 — evidence 의미 보존

`Communication.send`는 `observed_ids`가 허용 ID인지 검사하며 시각적 사실을 검증하지
않는다. source frame hash도 스스로 붙이지 않는다. 이것은 일반 메시지 전송의 성질이며
그대로 peer claim이면 허용된다. receipt를 RGB 관측 또는 평가 확인으로 해석하는 순간
문제가 된다. 기존 CoELA의 별도 관측 receipt는 RGB-D 관측 계약 안에서만 해석해야 한다.

**C/D 요구:** `claim`/`local_visual_estimate`/`evaluator_verdict`를 분리하고 메시지→
송신 decision→observation ID를 추적한다. false completed report도 허용된 모델 출력
오류로 보존·사후 판정한다. evaluator로 전송 전 진실을 보정하지 않는다.

### A-06 · 깨우기와 예약 응답 — 확인 범위 및 미검증

현행 raw RGB runtime은 자기 command의 ready time에 재판단하며 peer delivery가
즉시 깨우는 별도 경로는 없다. private reason만 바꾼 none의 paired run은 r2/r3의
전체 입력·관측 시각·호출 수를 바꾸지 않았다. 수신자 격리와 private reason 분리는
기존 thread 테스트와 새 deterministic 테스트로 확인했다. 이 결과는 실제 추론 지연의
분포 또는 신규 B/C runtime의 비간섭성을 검증하지 않는다.

legacy `mixed_warehouse_runtime.py:133–172`는 available/idle/history와 recovery를
prompt 및 wake key에 사용한다. strict_local CoELA는 자기 status/history/inbox key로
좁혔지만 `MixedEngine._complete()`는 ideal-SIM 실행의 성공을 own history로 알려 준다.
관측 필드의 `controller_contract=ideal_sim_geometry`와 수치 range/bearing가
`Perception.read()`를 통과하는 재현이 있으므로 RGB 준수로 이름을 바꾸면 안 된다.

**B/C 요구:** clock tick/자기 lifecycle/자기 수신 inbox 외 wake 의존성을 검사한다.
거부 응답에는 상대 선택·점유자·전체 완료 리스트가 없어야 한다. 최소 GO/HOLD bit와
자원 unavailable에 따른 자기 status 변경은 공통 채널로 기록하며, 무관한 peer 변경에
대한 비간섭성과 구분한다. 전체 팀의 물리 완료로 runtime을 조기 종료해 비용/호출 수가
달라지는 경우도 평가 정답의 개입이므로 연결 파일럿에서는 사후 평가를 기본으로 한다.

## 통과한 현행 경계와 테스트의 의미

실제 `CameraRobotPort`를 fake world에 연결해 r2의 숨은 의도·evaluator 성공만 변경한
두 입력이 같음을 확인했다. 허용 외 world/robot accessor를 읽으면 실패하도록 했다.
own robot_cam의 byte/hash는 모델 이미지 part로 전달되고, runtime이 저장한 JPEG의
해시가 planner_input과 일치한다. directed message는 r2에만 도착하고 r3의 memory에
들어가지 않는다. none의 송신은 거부된다.

새 감사 tests의 green은 위 경계와 **차단 사유의 재현 성공**을 뜻한다. 특히 A-01–04가
해결되었다는 인증이 아니다. skip/xfail로 결함을 감추지 않았다. 신규 B/C가 준비되면
확정 SHA와 실제 serializer/port로 재감사하고, 결함 characterization은 수정된 경계의
회귀검사로 전환한다. 현재 main의 보존된 legacy 경로를 무조건 깨뜨릴 필요는 없다.
