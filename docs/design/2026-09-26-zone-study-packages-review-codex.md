# 2026-09-26 구역 대화 연구 패키지 적대적 검토 — Codex 검토

- **작성:** Codex CLI (reasoning xhigh, 읽기 전용). 파일 수정·브랜치 생성·fetch·댓글·모델 호출·시뮬레이션 없이 Git 객체 읽기와 메모리 내 순수 함수 반례만 수행했다.
- **날짜:** 2026-09-26
- **검토 대상 ref와 SHA:** `origin/kiro/zone-study-contract=8587bb9017c0`(A/PR 187), `origin/kiro/zone-study-protocol=e8c80eb093d6`(C/PR 184), `origin/kiro/zone-sim-cost=6be139590336`(D/PR 186), `origin/kiro/zone-study-eval=a68ce05683cd`(I/PR 185), `origin/kiro/zone-study-scenarios=3186100117c4`(E/PR 190), `origin/kiro/zone-study-core=65a96f29d92d`(Core). 본문 행 번호는 검토 시점 기준이며 이후 커밋에서 바뀌었을 수 있다.
- **상태:** 수정 추적: PR 194. 이 문서는 검토 결과 원문이며, 지적이 해결됐다는 뜻이 아니다. 통합 브랜치 `kiro/zone-study-core`에서 finding별 수정·회귀 테스트로 추적한다.
- 아래 본문은 Codex 출력 원문 그대로다. 비밀정보·토큰은 없었다. 끝의 Codex 세션 ID는 인증 정보가 아니라 로컬 재개용 식별자라서 출처로 남겼다.

---

**판정: 현재 상태로 본실험에 투입하면 안 됩니다.** 입력 경계 우회, C–D 전달 인터페이스 불일치, 비용 누락, 조건별 평가 편향이 통합본에도 남아 있습니다.

검토 대상은 아래 **로컬 ref의 SHA**입니다. 이후 `A:path:line` 같은 표기는 이 표의 ref를 뜻합니다.

| 표기 | 로컬 ref | 검토 SHA |
|---|---|---|
| A / PR 187 | `origin/kiro/zone-study-contract` | `8587bb9017c0` |
| C / PR 184 | `origin/kiro/zone-study-protocol` | `e8c80eb093d6` |
| D / PR 186 | `origin/kiro/zone-sim-cost` | `6be139590336` |
| I / PR 185 | `origin/kiro/zone-study-eval` | `a68ce05683cd` |
| E / PR 190 | `origin/kiro/zone-study-scenarios` | `3186100117c4` |
| Core | `origin/kiro/zone-study-core` | `65a96f29d92d` |

**Blocker**

1. **실제 이미지가 검증한 wrist/map 참조에 묶이지 않습니다.**  
   **위치:** `Core:harness/zone_study_prompts_ko.py:89–94`, `:356–368`, `:424–430`.  
   **실패 시나리오:** 정상적인 자기 wrist 참조·해시가 있는 payload에 다른 카메라 바이트를 `wrist_jpeg`로 붙이거나 TOP 영상을 `map_figure_jpeg`로 붙여도 통과합니다. 이미지 검사는 비어 있지 않은 bytes인지뿐이고, 라벨은 함수가 정상 이름으로 생성합니다. 서로 다른 바이트를 넣는 반례를 확인했습니다. 기록되는 `input_sha256`도 실제 이미지와 추가된 `dialogue_window`를 포함한 최종 요청의 해시가 아닙니다.  
   **수정:** 이미지별 참조·촬영 시각·실제 바이트 해시를 대조하고, 지도 그림은 고정 지도에서 생성한 산출물만 허용하십시오. 최종 system/user/image 요청 전체를 별도로 해시·보존해야 합니다.

2. **C와 D가 서로 다른 메시지 ID·inbox·전달 시계를 소유합니다.**  
   **위치:** `Core:harness/zone_study_protocol.py:405–414`, `:449`; `Core:harness/zone_event_scheduler.py:472–490`, `:505–514`. D 원본에도 `harness/zone_event_scheduler.py:440–482`에 같은 문제가 있습니다.  
   **실패 시나리오:** C는 `window-sender-number` ID로 자체 inbox에 전달하고, D는 `call-id-mN` ID를 다시 만듭니다. D의 inbox를 A에 그대로 넣으면 `recipients`, `created_at_sim_s`가 없고 `delivered_sim_s`, `broadcast`가 추가되어 거절됩니다. 반대로 D만 사용하면 조건·허브-스포크·발화 한도 검사가 없습니다. 두 경로를 함께 사용하면 비용 지불 전 C inbox 노출이나 중복 전달을 막을 연결 계약이 없습니다.  
   **수정:** 하나의 canonical envelope/ID를 C 검증부터 D 예약·전달·로그까지 유지하십시오. inbox 변경 권한은 SIM 스케줄러 하나만 가져야 합니다.

**High**

3. **A의 닫힌 schema가 값의 타입과 깊은 하위 구조를 닫지 않습니다.**  
   **위치:** `A:harness/zone_study_contract.py:534–555`, `:558–575`; Core도 동일.  
   **실패 시나리오:** `initial_location.slot=[0.4,-2.45]`로 정확 좌표를 넣어도 `payload_violations()`가 빈 목록을 반환합니다. `public_map.walls[0].survey_xy_m`에 좌표를 추가하고 투영 해시를 다시 계산하는 경우도 통과합니다. 정상 `OrderSheetSource`는 slot을 제한하지만, C가 받는 최종 payload 검증에는 그 보장이 없습니다.  
   **수정:** slot/bay는 지도 어휘에 속하는 문자열로 검사하고, 지도 내부까지 타입·키를 닫으십시오. 제공자가 계산한 해시의 자기 일치만 보지 말고 고정 지도·주문서 원본과 대조해야 합니다.

4. **`frozen=True`인 입력 객체를 생성 후 변경하면 재검증 없이 전송됩니다.**  
   **위치:** `Core:harness/zone_study_prompts_ko.py:68–78`, `:126–127`, `:406–430`.  
   **실패 시나리오:** 검증된 `StudyInputs`의 `payload` dict에 `teacher_receipt={"done": true}`를 추가한 뒤 `build_request()`를 호출하면 그대로 user JSON에 들어갑니다. 이 반례를 확인했습니다. dataclass의 frozen은 내부 dict 변경을 막지 않습니다.  
   **수정:** 내부 데이터를 깊게 불변화하거나 접근 시 복사본만 반환하고, 실제 직렬화 직전에 최종 요청을 다시 검증하십시오.

5. **실패한 응답의 행동·메시지가 실행된 뒤 재시도됩니다.**  
   **위치:** `D:harness/zone_event_scheduler.py:428–434`; `Core:harness/zone_event_scheduler.py:460–466`.  
   **실패 시나리오:** `outcome="invalid"`인데 부분 파싱된 action/messages가 남은 `CallReply`는 생성 가능합니다. 스케줄러는 먼저 action과 메시지를 적용한 뒤 실패 여부를 확인하고 재시도합니다. 잘못된 결정을 실행하면서 추가 판단 기회까지 얻습니다.  
   **수정:** 실패 시도의 비용·원문은 보존하되 실행 효과는 차단하십시오. 부분 수락을 지원하려면 호출 실패와 메시지별 거절을 별도 상태로 정의해야 합니다.

6. **실제 발화 수와 청구 발화 수가 독립적입니다.**  
   **위치:** `D:harness/zone_event_scheduler.py:87–96`, `:440–458`; `Core:harness/zone_event_scheduler.py:92–101`; `Core:harness/zone_sim_cost.py:255–256`.  
   **실패 시나리오:** `CallReply(messages=(메시지,))`의 기본 attempt는 `utterances=0`입니다. 순수 계산에서 메시지 1개가 있어도 발화 비용은 0이고 총비용은 기본 호출비 1초였습니다. `_emit()`은 청구 수와 무관하게 모든 메시지를 전달합니다.  
   **수정:** 발화 수를 실제 모델 출력에서 산출하고 attempt 기록과 일치시켜야 합니다. 전송 거절된 발화도 생성 비용에서 빠지지 않도록 하십시오.

7. **A가 허용한 물건 종류를 C의 정형 응답 검증기가 거절합니다.**  
   **위치:** `A:harness/zone_study_inputs.py:277–287`; `Core:harness/zone_study_protocol.py:653–657`; `E:configs/zone_study_scenarios/s5_moved_dropped_item.json:19–26`.  
   **실패 시나리오:** A vocabulary에는 `red` 같은 kind가 있지만 C의 `validate_reply()`는 item/order ID만 합칩니다. 실제 s5 주문으로 `item="red"`를 보내면 A는 허용하고 C는 거절했습니다. 개체 ID가 없는 fungible 주문에서 정형 조건만 표현이 제한됩니다.  
   **수정:** C도 A의 `Vocabulary` 객체를 직접 사용하십시오. 실제 s1–s6 주문으로 정형 응답을 검증해야 합니다.

8. **C가 수락한 영어 메시지가 A에서 수신자의 다음 호출을 막습니다.**  
   **위치:** `C:harness/zone_study_protocol.py:420–428`; `Core:harness/zone_study_protocol.py:452–460`; `A:harness/zone_study_contract.py:293–309`, `:634–635`.  
   **실패 시나리오:** C는 `"Move to A"`를 수락·전달하지만 A는 “한국어 없음”으로 inbox를 거절합니다. 반례를 확인했습니다. 발신자의 언어 오류가 수신자의 입력 구성 실패로 전파되고, inbox에 남으면 반복됩니다.  
   **수정:** 한국어 수락 규칙을 A/C에서 통일하고, 전달 전에 검사하십시오. 거절 비용과 기록은 남기되 수신자의 정상 호출을 막아서는 안 됩니다.

9. **명시적 `leader` 인자로 seed 순환을 우회할 수 있습니다.**  
   **위치:** `C:harness/zone_study_protocol.py:163–175`; `Core:harness/zone_study_protocol.py:183–195`, `Core:harness/zone_study_prompts_ko.py:395–396`.  
   **실패 시나리오:** seed 11에서 A는 r3을 leader로 정하지만 C에 `leader="r1"`을 주면 r1을 선택합니다. 고정 leader로 모든 시행을 실행하거나, payload와 system prompt/전송 topology의 leader가 달라질 수 있습니다.  
   **수정:** 연구 실행에서는 override를 제거하거나 `leader == leader_for_seed(seed)`를 강제하고, payload·프롬프트·전송기의 일치를 검사하십시오.

10. **표준 호출 로그가 있어도 평가 비용·호출 수가 0으로 보고됩니다.**  
    **위치:** `Core:harness/zone_study_eval.py:211–246`, `:483–488`, `:521–527`.  
    **실패 시나리오:** A 형식의 호출 로그 1개에 1.6 SIM초와 토큰을 기록해도, 별도 `trial.model` 집계가 없으면 호출·HTTP 시도·토큰·사고/발화 비용이 모두 0입니다. 반례를 확인했습니다. 별도 집계가 잘못되어도 원본 로그와 대조하지 않습니다.  
    **수정:** `calls/messages`를 유일한 집계 원본으로 삼으십시오. 별도 요약을 유지한다면 일치 검사를 필수로 하고 누락을 0으로 바꾸지 않아야 합니다.

11. **배송 집계가 복구 성공을 지우고 fungible 주문을 잘못 판정합니다.**  
    **위치:** `I:harness/zone_study_eval.py:366–376`; `Core:harness/zone_study_eval.py:450–463`, `:505–510`.  
    **실패 시나리오:** B 오배송 후 A로 바로잡아도 첫 delivery만 남겨 `delivered=0`입니다. fungible `red_1 → B`는 `correct`가 생략되면 주문에 개체 ID가 없어 오배송 처리됩니다. 반대로 주문 밖 물건이나 종료 후 delivery도 `correct=True`이면 배송률에 포함됩니다. 모두 순수 함수 반례로 확인했습니다.  
    **수정:** 시행 시간 범위 안에서 identity/kind·목적지·주문 수량을 매칭하고, 최종 충족 상태와 오배송 이력을 분리하십시오.

12. **사실성 지표가 한국어와 정형 메시지에서 서로 다른 명제를 평가합니다.**  
    **위치:** `I:harness/zone_study_eval.py:128`, `:604–612`; `Core:harness/zone_study_eval.py:138`, `:648–650`, `:691–699`, `:735–744`.  
    **실패 시나리오:** E의 실제 ID `cyan_1`, `beam_1`은 하이픈 숫자만 찾는 정규식에 잡히지 않아 한국어 배송 주장이 `item_id=null → unverifiable`이 됩니다. 정형 `held` 주장은 본문에 없는 `sender`를 읽어 robot이 null이 되고, r1의 주장인데 r2가 잡고 있어도 참입니다. 두 반례를 확인했습니다.  
    **수정:** 주문서에 선언된 ID를 직접 매칭하고 정형 주장에는 envelope 발신자를 전달하십시오. 같은 참/거짓 명제를 한국어·정형 양쪽에 넣어 판정이 같은지 검사해야 합니다.

**Medium**

13. **허용된 leader broadcast를 채널 위반으로 판정합니다.**  
    **위치:** `I:harness/zone_study_eval.py:294–300`; `Core:harness/zone_study_eval.py:381–387`.  
    **실패 시나리오:** leader가 두 follower에게 같은 메시지를 보내는 것은 A/C의 허브-스포크 간선을 모두 만족하지만 I는 `leader_broadcast` 위반으로 셉니다. leader 조건만 부당하게 불리해집니다.  
    **수정:** 수신자별 간선을 A의 `allowed_edges()`로 검사하고, 별도 금지 합의가 없는 broadcast 제한을 제거하십시오.

14. **입력 감사 실패가 보고서·코호트 판정에서 사라집니다.**  
    **위치:** `Core:harness/zone_study_eval.py:978–981`, `:1218`; `Core:scripts/zone_study_report.py:138–147`.  
    **실패 시나리오:** `payload_validated=False`이면 시행의 `boundary.clean`은 false지만, 집계는 `unvalidated_payloads`와 `unknown_input_keys`를 누락합니다. 반례에서 위반 시행 수는 0이고 보고서는 문제가 없다는 문장을 출력했습니다. 코호트 export도 clean으로 표시합니다.  
    **수정:** 모든 감사 실패 항목을 동일하게 집계하고, clean/violation/unverified 상태를 공통 함수로 산출하십시오.

15. **HTTP 예산을 요청 전에 예약하지 않아 동시 호출·내부 재시도가 상한을 넘습니다.**  
    **위치:** `D:harness/zone_event_scheduler.py:371–379`, `:411–413`; `Core:harness/zone_event_scheduler.py:403–411`, `:443–445`.  
    **실패 시나리오:** 총예산 90 중 완료된 시도가 89개면 세 로봇의 동시 요청이 모두 허용됩니다. 한 `CallReply`에 여러 HTTP attempt가 들어가는 경우도 호출 전에 제한하지 않습니다. E의 `http_attempts_per_actor=30`과 D의 logical-call 상한도 같은 단위가 아닙니다.  
    **수정:** HTTP attempt마다 시작 전에 팀/actor 예산을 예약하고, transport 내부 재시도도 같은 예산 소유자를 거치게 하십시오.

16. **horizon에서 끝나지 않은 호출이 비용 로그에서 누락됩니다.**  
    **위치:** `Core:harness/zone_event_scheduler.py:308–317`, `:454–457`, `:281–294`.  
    **실패 시나리오:** 호출이 horizon 전에 시작했지만 완료 예정 시각이 horizon 뒤이면 `self.calls`에 추가되지 않습니다. `RunReport.calls`와 `contract_log()`는 이 호출을 제외합니다. 긴 추론으로 실패한 조건의 호출·토큰 사용량이 작게 보일 수 있습니다.  
    **수정:** 시작 시점부터 호출 ledger를 만들고 종료 시 outstanding 호출을 censored 상태로 보존하십시오. 종료까지 경과한 SIM 비용과 실제 사용한 API 자원을 구분해 기록해야 합니다.

17. **시나리오의 공개 메타데이터가 숨은 사건의 종류·대상을 알려줍니다.**  
    **위치:** `E:configs/zone_study_scenarios/s3_late_rendezvous.json:3–7`, `s5_moved_dropped_item.json:3–7`, `s6_novel_relation.json:7`; `E:harness/zone_study_scenarios.py:183–185`; `A:harness/zone_study_inputs.py:209–215`.  
    **실패 시나리오:** 공개 `notes`에는 r3 지연, cyan 이동/red 낙하, s6 해법이 들어 있습니다. 현재 A 주문서는 notes를 버리지만 `scenario_id`는 전달하므로 최소한 사건 유형을 관측 전에 알려줍니다. E의 `public_part()`를 안전한 공개 입력으로 재사용하면 대상과 해법까지 노출됩니다.  
    **수정:** 설명적 이름·실험 의도·해법은 평가 전용으로 옮기고 로봇에는 불투명한 ID만 주십시오. **현재 canonical 주문서에서 정확한 사건 시각이 직접 전달되는 경로는 발견하지 못했습니다.**

18. **고정 프롬프트의 길이·행동 지침 차이가 채널 효과와 섞입니다.**  
    **위치:** `C:harness/zone_study_prompts_ko.py:291–320`; `Core:harness/zone_study_prompts_ko.py:227–256`, `:330–353`; `Core:harness/zone_sim_cost.py:255`.  
    **실패 시나리오:** r1 system prompt는 no_comm 1,327자, peer 1,581자, leader 1,689자, structured 2,027자였습니다. 토큰 수는 별도 측정이 필요하지만, 고정 설명량 차이가 매 호출 입력 비용에 포함됩니다. follower에만 추가된 정지·거절 지침도 행동 차이를 만듭니다.  
    **수정:** 공통 작업·안전·행동 지침을 동일하게 만들고, 채널 선택에 필요한 차이를 최소화하십시오. 고정 안내문의 tokenizer 비용을 측정·통제하여 실제 대화량 비용과 분리해야 합니다.

**검증 범위와 미확인 사항**

- 파일 수정·브랜치 생성·fetch·댓글·모델 호출·시뮬레이션은 하지 않았습니다. Git 객체 읽기와 **메모리 내 순수 함수 반례**만 수행했고 작업 트리는 깨끗합니다.
- Core는 A/C의 주문서·envelope, D/I의 일부 schema 명칭을 정렬했습니다. 그 정렬을 완료된 전체 실행 통합으로 볼 수는 없습니다. `Core:tests/test_zone_study_protocol.py:893–941`의 round trip에는 D 스케줄러·E 실제 시나리오·I 평가가 함께 연결되지 않습니다.
- 기본 성공률/PAR 집계는 기록된 실패 시행을 분모에 유지하고, `reference_R`도 기본 조건 비교에서 제외합니다.
- 실제 카메라 출처, 화물 배치·가시성·도달성, s6의 필요한 조작 가능성, 숨은 사건의 실제 적용, 전체 호출 cadence, API 완료 순서를 바꾼 통합 실행은 검증하지 않았습니다. pytest·offline smoke·원격 CI도 실행하지 않았습니다.


Codex session ID: 01a0d9da-9df9-7a22-9aa5-d52fce85aa9a
Resume in Codex: codex resume 01a0d9da-9df9-7a22-9aa5-d52fce85aa9a
