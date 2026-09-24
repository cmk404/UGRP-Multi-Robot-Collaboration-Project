# 공동 coarse 접근 동시 전진 후보: 실행 전 설계

상태: 코드 후보 `d2be823` 및 집중 테스트 검증 완료, 물리 실행·성공 판정 없음. 새 번들 ID·최종 실행 소스 해시, host lock, 원본 보존 확인 전에는 실행하지 않는다.

## 관측된 문제와 한 변수

기존 `route-teachers-managed-v3/raw/south-train-a`는 교사 소스 `931d910`의 `dispatch_shared_crossing`/seed 11/`dock_a`/`local_contact_fine`/실시간 RGB/저장 F3 계획 진단이다. `pair-decisions.json` SHA-256 `35487ec2ad36193ffc907dbb6076d9a310b2d7ac6a167352d0efb885eaae94bc`의 coarse 120회에서 r1의 유효한 전진 `.12` 제안이 매번 0으로 바뀌었다. r3는 횡방향 또는 회전을 보정했고, 두 로봇의 전방 영상 간격 차이는 첫/마지막 표본 약 5.23/6.22px로 기존 16px 선행 한도 안이었다. 결과 `result.json` SHA-256 `ae4db1591ebf5f756656a0b63f30b2885ef87ccd2e9af1bb85a3fa89ce5bdd4c`는 상자 대상의 물리 성공과 빔 파지 전 coarse 120회 예산 소진에 따른 전체 실패를 구분한다. 1,500초 wall 제한이나 간섭 중단이 원인이 아니다.

후보 변수는 `--coarse-concurrent-alignment` 하나다(기본 OFF). ON에서는 같은 신선한 공용 TOP RGB·자기 바퀴/빔 추정으로 두 모델이 모두 유효하고, 전진하는 로봇 자신의 yaw/횡오차가 준비 범위이며, 상대보다 16px 이상 앞서지 않을 때 동료의 yaw/횡방향 보정만을 이유로 자기 전진을 0으로 만들지 않는다. 동료의 원래 RGB 보정 명령은 바꾸지 않는다. 영상/수치/역할/지도 범위가 불명확하면 양쪽 HOLD다.

`dispatch_open`과 고정 `dispatch_shared_crossing`의 정확한 authored 지도 버전/해시 범위에서만 opt-in을 검토한다. 같은 사전 지도를 공유하지만 실제 숨은 장애물이 있는 `north_blocked` variant는 적용하지 않는다. 실시간 정답 좌표·관절·접촉·심판 판정은 제어에 넣지 않는다. 기존 coarse 120회, 후속 fine/grasp 모델과 기준, 물리·접촉·weld OFF, 원영상 0.6초 TTL과 0.25초 개별 발행 lease, 명령 갱신 경로는 유지한다. 고정 0.2초 coarse 명령 전체가 원영상 TTL 안에 들어가도록 후보의 판독 나이는 최대 0.4초로 제한하고, 실제 발행 직전에도 판독부터의 경과와 0.2초 발행 기간을 다시 검사한다. 이 후보에서 lease 연장이나 동기 모드 전환은 하지 않는다.

## 후속 검증 순서

1. 저장된 첫/마지막 coarse 판단 fixture와 기존 테스트로 flag OFF의 명령 동일성, ON의 동시 전진, 선행 로봇 정지, 유효하지 않은 수치·관측·역할 거절, runner의 지도/실시간 조건을 확인한다. 정적 검사나 회귀 통과를 물리 성공으로 세지 않는다.
2. root 검토 뒤 새 실행 번들 ID·소스/설정/입력 해시를 고정하고, 다른 에이전트의 CPU 잠금이 해제된 유한 실행 창에서 동일 저장 계획·포즈의 새 결과를 수집한다. 최초 후보는 원 실패 사례 `south-train-a`의 진단이며, 명령 기록의 r1/r3 실제 동시 발행, 각자 RGB 오차 변화, 16px/TTL/lease 위반 여부, coarse 후 fine/파지/전체 물리 결과를 각각 판정한다. 영상과 별도 referee 원본을 확인한다.
3. 한 사례 개선만으로 교사 3/3이나 ACT 학습 적격을 주장하지 않는다. 세 고정 포즈 각각의 새 물리 성공과 입력·행동·split 품질 감사가 모두 통과해야 데이터셋/ACT 학습 gate를 다시 평가한다. 실패와 중단도 분모에 남기며 재시도는 새 사전 기록이 있을 때만 한다.

집중 검증: 신규 테스트와 기존 skill binding/realtime/coarse handoff/approach renewal/simulation CLI 관련 테스트 6파일에서 `166 passed, 2 skipped` (2.90초). 이것은 명령 판단·CLI 회귀이며 물리 결과는 아니다.

과거 F3 `result.json` SHA-256 `abd29504c108c7ee66169545f870d62c549e06c2648c1204ba93a841cb822fa2`의 `e099a4a` 물리 성공은 다른 실행 소스와 조건이다. 이번 후보의 성공이나 속도 개선으로 승계하지 않는다. 호스트 테스트가 겹친 기존 수집 시간도 속도 비교에 사용하지 않는다.

## 사전 고정 A/B 계약 (아직 미실행)

기계 판독용 계약은 `prospective-ab-v1.json`이다. 비교는 **새 후보의 동일한 최종 소스·번들에서 OFF 1회, ON 1회**이며 두 팔 사이의 유일한 실행 옵션 차이는 `--coarse-concurrent-alignment`다. `d2be823a23e90cc9f9c63bb8e6839f2565587c9a`는 검토 대상 구현 커밋일 뿐, 실행 소스 해시·번들 ID는 root가 테스트/리뷰 뒤 별도로 고정한다. 두 팔 모두 `shared_crossing`, seed 11, `dock_a`, 저장된 F3 계획 SHA-256 `398b303eb826d120047deeb03e156c80250d392bb0e8354bec655e4a0175ca57`, `local_contact_fine`, `--realtime-control`, `--efficient-capture`, video 10fps, viewer/realtime factor 1, 같은 grasp/stage 모델·reference TOP, 시작 변위 `(0.006, 0.004, 0.25°)`를 사용한다. 계획 replay는 새 LLM 의사결정이 아니다. 두 팔은 각각 새 출력 디렉터리를 쓰고 원본 v3는 읽기 전용이다. 두 팔 각각 전체 wall 상한 360초, 관리 실행 상한 370초, 총 wall 상한 740초와 청소 여유 20초를 둔다. 팔당 시도는 한 번이며 시간 초과·실패도 분모에 남긴다. 안전 위반, 소스 불일치, 외부 CPU 간섭이 확인되면 후속 실행을 멈추고 이유를 기록한다. 다른 과제의 물리/pytest가 진행 중인 동안에는 시작하지 않는다.

2026-09-24 v3의 `south-train-a`는 실패 원인과 입력 선정 근거다. 그때 소스 `931d910998a94361342c372ec2675c475daa625f`, wall 277.9706초, coarse 120회 소진, 전체 물리 실패였다. A/B의 OFF가 그 결과를 재현했다고 미리 가정하거나 v3 결과를 새 A/B의 세 번째 표본으로 넣지 않는다. 새 후보 두 팔의 소스·설정·모델·지도·계획·카메라 입력과 시뮬레이션 프로필 일치를 실행 전 확인하고, 실행 중 변경은 비교 무효로 남긴다.

## 물리 판정과 감사 인터페이스

각 팔에 원본 `result.json`, `pair-decisions.json`, `issued-commands.json`, `referee-only.jsonl`, `execution.mp4`, RGB 원본/해시, 장면·소스·번들 manifest와 관리 실행 기록을 보존한다. 두 팔 모두 120 coarse 결정 상한, 원래 fine/grasp 및 weld OFF를 유지한다. OFF/ON에서 전체 물리 `physical_success`, `protocol_complete`, 각 cargo의 도착·해제·안정, coarse 종료 여부/결정 수, wall 시간·명령 수·모델 호출/지연을 각각 보고한다. ON의 성공만으로 원인을 확정하지 않는다. 명령 동시성은 같은 SIM 시각에 r1의 **실제로 발행된** 양의 forward와 r3의 turn/left PWM이 겹친 구간으로 세고, 실제 병진/회전은 별도 referee-only 궤적과 영상에서 검사한다. 발행 명령·PWM은 물리 이동이나 임무 성공의 증거가 아니다. 첫/마지막 저장 결정의 정적 재평가도 새로운 물리 증거가 아니다.

감사기는 coarse index별로 같은 `frame_id`/공용 TOP SHA/관측 시각, 두 로봇 자기 RGB SHA, 모델 제안과 조정 후 명령, 물리 robot ID·port, 발행 전 SIM 시각과 **port 적용 후 실제 `issued_at_s`/종료 시각·motor PWM**, 소스 코드/지도 해시를 한 행으로 연결해야 한다. 양의 forward가 나온 모든 행에서 TOP 캡처 나이 ≤0.4초, 실제 `issued_at_s + effective_duration_s ≤ observed_at_s + 0.6초`, 선행 격차 `<16px`를 검증한다. 무효·미확인 입력 및 TTL 만료는 두 로봇 HOLD로 검증하고, ON 미적용 scope는 `requested=true, applied=false, reason`으로 구분한다. 우선 결과 JSON과 port 명령 감사만 읽고, 이후 referee/영상으로 물리 변위를 독립 평가한다. referee 위치·접촉은 학생 제어나 성공 통보에 역주입하지 않는다.

현재 `pair-decisions.json`의 `coarse` 행과 `image_binding` RGB 행, `issued-commands.json`의 개별 `issued_at_s`는 있으나, cluttered-map 명령에는 source frame/own RGB SHA/실제 port PWM의 일대일 receipt가 없다. 따라서 **발행 로그를 순서로 추정하는 것만으로 TTL·동시 PWM 완전 통과를 선언하지 않는다.** 물리 시작 전 최소 감사 인터페이스는 pair-drive 발행 직후 robot별 `coarse_index`, capture frame/시각/TOP·own SHA, requested action, port ack PWM, actual issue/expiry SIM 시각을 불변 receipt로 기록하고 두 robot receipt를 같은 batch ID로 묶는 것이다. 출력 전용 로깅으로 유지하며 제어 입력에는 쓰지 않는다. 이 인터페이스가 구현되지 않으면 물리 실행 결과는 탐색 진단으로 기록하되 명령 안전 감사 통과·동시 물리 효과 판정은 보류한다. 1사례의 성공은 교사 3/3 승인이나 ACT 학습 데이터 허가가 아니다.

## Root 직접 후속 구현 (2026-09-25)

서브에이전트 중단 뒤 root가 출력 전용 port receipt와 무효 ack 회귀 검사를 완성했다. main 위에 병합하고 v48 / workflow 1.48.0으로 등록했다(v45·v46·v47은 별도 후보). source frame/own·TOP 해시, 실제 port ack PWM과 issue/expiry를 `coarse_command_receipt`로 연결한다. physics step으로 port cache가 지워지기 전에 기록을 복사하며 이후 제어에는 사용하지 않는다. 앞의 166 passed는 receipt 변경 전 검사다. 새 전체 검사와 물리 A/B는 아직 완료되지 않았고 Claude의 CPU 잠금 때문에 로컬 실행을 시작하지 않았다.

#142가 먼저 main에 병합되어 최종 후보를 **v49 / workflow 1.49.0 (부모 v47)**로 다시 등록했다. v48은 바이트 그대로 은퇴 보존하며 물리 실행한 적은 없다. 기존 handoff 테스트의 가짜 IO에도 새 출력 기록이 요구하는 issued-port/time 문맥을 명시했다.
