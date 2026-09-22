# A · 6회 연결 파일럿 go/no-go

2026-09-22 기준 **NO-GO**. 문서·합성 fixture·오프라인 검사의 완료와 실제 실행
준비는 별도다. 미충족 gate를 true로 바꾸지 않았으며 외부 실행은 이번 배치 범위 밖이다.

| Gate | 이번 판정 | 통과할 증거/담당 |
|---|---|---|
| R0 연구 질문/주 지표/6회 범위 | 초안 작성 | [A 프로토콜](a-protocol.md)을 D의 단일 manifest와 일치시키기 |
| 관측과 actor serializer | 현행 좁은 RGB만 검사 | B/C 실제 연결에서 own/top·명령·static/inbox 중첩 allowlist 및 해시 확인 |
| peer/evaluator 비간섭 | 현행 fake 기반 검사 | 같은 RGB/자기 이력에서 숨은 peer/evaluator만 변경해 prompt·memory·wake·action 불변 |
| none 공동 요청 | 옛 protocol만 확인 | 새 B/C에서 message=0인 두 독립 actor submit→joint GO, 무관 solo 진행 |
| 최소 동기화 | 의미 계약 전달 | own request 결과만 반환, peer payload·전체 역할표·점유자·미응답자 누출 없음 |
| 메시지/응답 만료 | 현행 gap 재현 | C explicit clock, 수신자/만료/지연 응답·새 evidence로 stale reply 거부 |
| 완료 통보/평가 | 현행 raw 사후 평가 확인 | B/C 로컬 RGB claim와 D evaluator verdict 분리; evaluator로 재판단/종료 성공 유도 없음 |
| 독립 실행·스킬 | 미검증 | 동일 frozen RGB 스킬, 파트너만 장벽, 잘못된 결정 보정 없음; 물리 지원 조건 별도 확인 |
| 정상/복구 시나리오 | 후보만 정의 | 실제 own/top/request에서 정보 차이/복구 도달 근거, camera/FOV 보존 |
| 물리/모델/환경 provenance | 미고정 | integrated SHA, model/weights, map+physical linkage, calibration, camera, weld OFF hash |
| 예산·clock·종료 | 유한 초안 | D 총량과 C/B 실제 강제 일치; retry 포함·unknown usage 처리·owned 종료 확인 |
| 관측→행동→평가 trace | 통합 미검증 | C schema와 D consumer를 확정 SHA로 연결, exact image/text/reply·실패·미실행 보존 |
| 제출·회수·보고 | 미실행 | 한 제출자/유한 Colab CLI job, raw hash/위치, 성공·실패 TensorBoard 실제 로딩·화면 확인 |

새 연결에서 필수인 반례는 다음과 같다.

1. r1의 입력 이미지를 고정한 채 r2의 비공개 계획·완료·관절·평가 성공만 바꾼다.
   r1의 context뿐 아니라 호출 시각·메모리와 action이 같아야 한다. 다만 r1이 실제
   요청한 공동 matching/자원 충돌의 최소 결과 변화는 사전 선언된 공통 채널이다.
2. r1→r2 메시지를 보낸다. r3는 내용·ID·수신 wake를 얻지 않는다. none에서는
   어떤 경로로도 message/공유 plan을 받지 않으며 joint request는 여전히 가능하다.
3. lease/frame/message가 유효할 때 모델 요청 후 만료 또는 자기 상태 변경을 만든다.
   늦은 응답이 command/새 message를 발행하지 않게 한다. 비용과 버려진 reply는 보존한다.
4. 잘못된 로컬 DONE과 거짓 peer 완료를 입력한다. 실행기가 evaluator로 보정하지
   않아야 하며, 사후 평가가 잘못된 완료로 기록해야 한다.
5. 공유 top에 보이는 장애물과 한 actor의 자기 명령·기억을 분리한다. 전자를 숨긴
   비대칭 fixture를 통신 우위 시나리오로 채택하지 않는다.

첫 6회 이후에는 본실험 반복 수·ID/배치 교차·효과 기준·통계 분석을 별도로 고정한다.
계약 테스트가 통과해도 6회 연결·실제 물리 지원·실물 검증의 완료 표시를 하지 않는다.
