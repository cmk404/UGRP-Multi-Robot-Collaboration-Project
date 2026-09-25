# 2026-09-25 한국어 자연어 대화 vs 정형/무통신 비교 설계 제안 — Codex 분석

- **작성:** Codex CLI (gpt-6-astra, reasoning xhigh). Claude 코디네이터가 2026-09-25에 위임한 읽기 전용 분석이다. 파일 수정·시뮬레이션·LLM 호출은 하지 않았다.
- **기준 SHA:** 로컬 main `ad18b93`(TeamJob 기준 `5a720bc`, hard-routes 기준 `dc27695` 병기). 원격 PR 상태는 연결 실패로 확인하지 못했다.
- **상태:** **제안, 미검증** (proposal, not verified)
- 아래 본문의 로컬 절대경로 링크(`/Users/changmin/projects/ugrp/...`, `/Users/changmin/projects/ugrp-worktrees/...`)는 작성자 기기와 작성 시점 경로를 가리킨다. 이후 커밋에서 줄 번호·구조가 바뀌었을 수 있다.

## 이후 진행 상황 (기록 시점 2026-09-25)

이 보고서가 제안한 A(무통신)/B(정형 통신)/C(한국어 자연어)/D(게시판 제거실험) 비교 구조 중, **작은 한국어 파일럿(3절의 오프라인 저장 RGB 파일럿)이 branch `claude/zone-dialogue-ko-pilot`에서 진행 중**이다. 이 보고서가 지적한 실행기 연결(구역 프롬프트에 한국어 발화 요구 추가, A/B/C/D 조건 배선, `run_zone_dispatch.py`/`zone_coordination.py` 등에 대한 실제 통합)은 **아직 남아 있다**.

---

**핵심 권고는 ‘무통신·정형 통신·자유 한국어 대화’를 같은 실행기와 같은 정보 경계에서 비교하는 것이다.** 기존 `dynamic`의 성능을 한국어 자연어 대화의 효과로 해석할 수는 없다.

읽기 기준은 로컬 main `ad18b93`, TeamJob `5a720bc`, hard-routes `dc27695`다. 파일 수정·시뮬레이션·LLM 호출은 하지 않았다. 원격 PR 상태는 연결 실패로 확인하지 못했다.

현재 상태는 다음과 같다.

- 구역 프롬프트는 영어이며 한국어 발화 요구가 없다. `message`는 **필수 키지만 빈 문자열로 침묵 가능**하다. 240자는 프롬프트 지시이고 실제 검증 상한은 600자다. [프롬프트](/Users/changmin/projects/ugrp/harness/zone_coordination.py:52), [검증](/Users/changmin/projects/ugrp/harness/three_robot_plan.py:58)
- ZC2 18개 로그의 실제 전송 메시지는 129건—dynamic 75건, plan_first 54건—이며 한글 포함 메시지는 0건이다. [영어 발화 예시](/Users/changmin/projects/ugrp/outputs/zone-communication-20260925/ZC2-s14-dynamic-graspfail/team/conversation.jsonl:2)
- 이동 중인 로봇은 구역 판단 호출 대상에서 제외된다. 충돌 시 제한된 재질문은 있지만 자유로운 사건 기반 대화는 아니다. [호출 조건](/Users/changmin/projects/ugrp/scripts/run_zone_dispatch.py:278)

연구 질문은 **C−A: 한국어 대화를 허용한 효과**, **B−A: 정형 정보 교환 효과**, **C−B: 자유로운 설명·협상의 추가 효과**로 나눈다. 이는 [연구 TODO](/Users/changmin/projects/ugrp/docs/research_todo.md:63)의 비교 구조와 일치한다.

| 조건 | 로봇 사이에 전달되는 정보 | 호스트 역할 |
|---|---|---|
| A 무통신 | 고수준 메시지 없음 | 공통 실행·최소 동기화 |
| B 정형 통신 | 고정 스키마의 보고·요청·수락·양보 | 메시지 전달, 형식·예산 검사 |
| C 한국어 자연어 | 로봇이 직접 작성한 자유 한국어 문장 | B와 동일 |
| D 게시판 제거실험 | 명시적 메시지 없이 호스트 게시판 제공 | 게시판 이외에는 A와 동일 |

B는 `act, item, zone, role, passage, observed_at, confidence, reply_to` 같은 필드를 미리 고정하고 자유문자열을 금지한다. 미지 장애물도 관측 위치·막힘 상태·불확실성을 표현할 수 있어야 한다. C에는 같은 수신자 선택권·전달 지연·손실·예산을 제공한다. **B−C는 지정한 표현 체계의 비교**이며, 언어 형식만의 효과를 좁게 보려면 동일한 사실을 두 형식으로 전달하는 보조 실험이 필요하다.

주 비교 A/B/C에서는 다음 경계를 적용한다.

- **게시판·중재 제거:** `team_board`, 동료 영수증, 동료 선언을 차감한 잔여 목표, `conflict`, 동료 예약을 설명하는 거절 사유를 제공하지 않는다. 각 로봇은 자기 관측과 실제 수신 메시지로 동료의 의도·상태를 추정한다. 공유 슬롯 번호·전역 질문 번호도 노출하지 않는다.
- **깨우기 통일:** 같은 자기 관측 주기와 로컬 타이머를 사용한다. 동료 작업 종료로 전원을 깨우지 않는다. B/C에서는 **실제로 받은 메시지**만 수신자의 재판단을 유발할 수 있다. 같은 호출·토큰 상한을 적용하고 추가 호출은 비용으로 센다.
- **최소 물리 동기화 유지:** TeamJob의 동일 spec·상보 역할 확인, 참여자 단계 장벽, 함께 정지·내리기, 만료 처리는 모든 조건에서 동일하게 둔다. A도 독립적으로 맞는 행동을 선택하면 실행 가능해야 한다. 장벽의 누락 참여자·상대 목적지·불일치 원인은 LLM에 반환하지 않는다. 시작/정지로 드러나는 최소 정보는 명시한다.
- **교사의 판단을 대화로 오인하지 않기:** L1의 의도 기반 선점 정지와 L4의 정답 기반 실패 통보를 해결해야 한다. 영수증을 일반적인 “멈춤”으로 바꾸는 것만으로 정보 출처가 깨끗해지지는 않는다. A2의 교사 실행 결과는 별도 조건으로 표시하고, RGB 기반 사건 인식이 연결되기 전에는 카메라 기반 복구를 주장하지 않는다.

D는 유지하되, 기존 `dynamic` 전체는 **게시판·중재·통지·깨우기 묶음**으로 별도 보존한다. 그 묶음의 메시지 ON/OFF 비교와 D의 게시판 단독 효과를 혼동하지 않는다. [C1 감사 근거](/Users/changmin/projects/ugrp/experiments/2026-09-25-zone-comm-boundary-audit/README.md:76)

대화 구현은 다음을 권고한다.

- **한국어 프롬프트를 기본으로 한다.** 과제 설명·출력 설명·자기 판단 요약까지 공통 한국어로 맞춘다. `r1`, 화물·사건·요청 ID, `A/B/C`, `kind`, `role`, JSON 키와 열거값은 원문 그대로 유지한다. 기존 영어 프롬프트+한국어 메시지는 개발 파일럿에서만 비교하고 본실험 전에 하나로 고정한다. 옛 [한국어 요구](/Users/changmin/projects/ugrp/harness/coela_modules.py:182)는 참고하되 그 프롬프트의 협업 전략까지 복사하지 않는다.
- **행동과 송신을 분리한다.** 세 로봇 모두 이동·파지·집결 대기 중 발언할 수 있다. 자기 RGB에서 막힘을 발견하거나, 파지 후 영상 변화가 기대와 다르거나, 집결 대기가 길어지면 로컬 판단을 요청한다. 발언 여부와 내용은 모델이 결정한다. 호스트의 주입 플래그·접촉 정답·동료 상태가 사건 발생을 대신 알려서는 안 된다.
- **방송과 직접 전달을 모두 허용한다.** B/C 모두 `recipients`로 한 명 또는 두 명을 선택한다. 직접 메시지를 제삼자에게 자동 복사하지 않으며, 방송은 수신자별 전달·입력 토큰 비용까지 기록한다.
- **한 결정 안에서 왕복을 허용한다.** 시작안은 협상창당 총 6발화, 로봇당 최대 2발화다. 앞선 발화를 실제 수신한 뒤 응답해야 한다. 침묵·거절·결렬도 허용하고, 한도 도달 시 호스트가 합의를 만들어 주지 않는다. 이동 중 행동 변경은 실행기의 공통 안전 경계에서 적용한다.
- **길이는 토큰과 문자 수로 함께 제한한다.** 파일럿 시작안은 발화당 256 모델 토큰·600자, 에피소드당 로봇별 20발화·통신 출력 4,000토큰이다. 호출·전체 입력 토큰 상한도 별도로 둔다. 수치는 개발 파일럿에서 조정한 뒤 고정한다. 문장 예시, “누가 무엇을 말하라”, 양보 순서 같은 대본은 제공하지 않는다.

현재 SIM 중심 연구 방침에 맞춰 추론 중 SIM 정지 여부와 메시지 전달 시점을 전 조건에서 고정한다. 이동 중 발언은 **작업 진행 중 판단 가능**을 뜻하며 실시간 추론 성능 주장과 구분한다.

대화 평가는 아래처럼 **평가 전용 출력**에 저장한다. 평가 점수·정답·행위 분류가 로봇 기억이나 재판단 조건으로 돌아가면 안 된다.

| 평가 항목 | 측정 방법 |
|---|---|
| 한국어 준수 | 허용된 literal ID·열거값을 제외한 문자에서 `한글/전체 문자 문자수` 비율을 계산한다. 코드 전환 구간, 영어로 바뀐 발화, ID 오기·번역을 별도 집계한다. 침묵은 한국어 성공으로 세지 않는다. |
| 근거와 사실성 | 발화를 명제로 나누고 **발화 당시 자기 RGB·명시적 믿음 기록·수신 보고**와 대조한다. 별도로 같은 시각의 평가 정답과 비교한다. 관측 오류, 근거 없는 주장, 의도와 완료의 혼동, 오래된 보고를 구분한다. |
| 대화 행위 | `claim`, `request`, `inform-obstacle`, `agree`, `yield`를 기본으로 질문·정정·거절·침묵을 추가한다. 사후 다중 라벨과 조건을 가린 사람 표본 검토를 사용한다. |
| 상호작용 | 실제 전달→수신 요청 포함→응답→역할·경로·대기 변경을 ID로 연결한다. 일방 방송과 상대 발화에 반응한 왕복 대화를 구분한다. |
| 인과 효과 | 전달 직전 물리 상태·로봇별 기억·큐·시계·난수 상태에서 메시지 유지/제거 분기를 반복한다. **내용만 제거하고 깨우기는 유지**한 분기와 **전달 자체 제거** 분기를 구분한다. 저장 응답을 그대로 재생하는 것은 인과 검증이 아니다. |
| 비용·효율 | 전체 완료율, 실패 포함 SIM makespan, 집결·통로 대기, 중복 작업·복구, 호출·재시도·입출력 토큰을 집계한다. 메시지가 이후 문맥에서 반복 소비한 토큰도 포함한다. 한국어 토큰 부담은 실제 모델 토크나이저/usage로 측정하며 고정 배수를 가정하지 않는다. |

한국어 능력과 언어 유지 능력은 아직 검증되지 않았다. 한국어 출력에 실패한 응답을 자동 번역하거나 평가에서 제거하지 말고 원문 그대로 남긴다. C2의 `lowest robot_id` 같은 고수준 분담 규칙은 주 비교에서 제거하고, 필요하면 **모든 조건에 같은 관례를 준 별도 제거실험**으로 둔다.

**실행용 claim은 구조화된 필드로 유지하는 편이 타당하다.** 모든 조건에서 같은 `{item, zone, role}` 행동 인터페이스를 사용하되, 이것은 자기 실행기에만 전달한다. 자연어를 호스트가 해석해 예약·배정·동료 기억을 자동 갱신하면 다시 중앙 조정이 된다. 동료는 대화를 해석한 뒤 자신의 claim을 직접 제출해야 한다. 자유문장과 claim이 모순되면 호스트가 고쳐 주지 않고 평가에 남긴다.

미지도 장애물은 정적 지도·공유 경로 캐시에 몰래 추가하지 않는다. 고정 카메라와 공용 TOP을 유지하고 **실제 저장 영상에서 한 로봇만 먼저 알 수 있었는지** 확인한다. 모두에게 보인다면 정보 비대칭 사례가 아니다. hard-routes의 자동 양보·정답 기반 우회도 주의해야 한다. 이를 유지하면 결론은 “공통 자동 주행 위에 대화가 더한 효과”로 제한된다.

구체적인 연결 지점은 다음과 같다. 마지막 두 행은 해당 기존 worktree 기준이다.

| 파일·줄 | 후속 변경 범위 |
|---|---|
| [zone_coordination.py:23](/Users/changmin/projects/ugrp/harness/zone_coordination.py:23), [zone_solo.py:18](/Users/changmin/projects/ugrp/harness/zone_solo.py:18), [zone_arena.py:191](/Users/changmin/projects/ugrp/sim/zone_arena.py:191) | 공통 한국어 지시, 조건별 메시지 형식, C2 제거 |
| [zone_coordination.py:95](/Users/changmin/projects/ugrp/harness/zone_coordination.py:95), [193](/Users/changmin/projects/ugrp/harness/zone_coordination.py:193) | 게시판 입력과 동료 선언 중재 분리 |
| [run_zone_dispatch.py:224](/Users/changmin/projects/ugrp/scripts/run_zone_dispatch.py:224), [367](/Users/changmin/projects/ugrp/scripts/run_zone_dispatch.py:367) | 종료 기반 깨우기·충돌 재질문을 로컬 사건·수신 경로로 분리 |
| [three_robot_runtime.py:128](/Users/changmin/projects/ugrp/scripts/three_robot_runtime.py:128), [164](/Users/changmin/projects/ugrp/scripts/three_robot_runtime.py:164) | 행동 없는 송신, 수신자 선택, 다턴·이동 중 요청 |
| [communication_observer.py:91](/Users/changmin/projects/ugrp/harness/communication_observer.py:91), [rgb_communication_evaluation.py:42](/Users/changmin/projects/ugrp/harness/rgb_communication_evaluation.py:42) | 송수신·행동 연결 및 평가 전용 언어·비용 지표 |
| [test_zone_comm_boundary.py:132](/Users/changmin/projects/ugrp/tests/test_zone_comm_boundary.py:132), [198](/Users/changmin/projects/ugrp/tests/test_zone_comm_boundary.py:198) | 비공개 상태 불변성·깨우기 격리·L1 회귀 |
| [zone_team_jobs.py:91](/Users/changmin/projects/ugrp-worktrees/zone-team-jobs/harness/zone_team_jobs.py:91), [331](/Users/changmin/projects/ugrp-worktrees/zone-team-jobs/harness/zone_team_jobs.py:331) | 공통 장벽·집결 규칙 유지, 내부 상세 정보 차단 |
| [hard-routes/zone_teacher.py:616](/Users/changmin/projects/ugrp-worktrees/zone-hard-routes/scripts/zone_teacher.py:616) | 자동 통로 우선순위·후퇴 개입 감사 |

실행 순서는 **hard-routes → TeamJob A2 → 정보 경계 검사 → 한국어 파일럿 → 비교 실험**으로 잡는다. [A2 계획](/Users/changmin/projects/ugrp-worktrees/zone-team-jobs/experiments/2026-09-25-zone-team-jobs/README.md:151)도 hard-routes 이후 연결을 명시한다.

1. **A2 물리 게이트:** 단독·2대·3대 운반, 문 통과, 늦은 집결, 부분 파지 실패·공동 내리기를 확인한다. weld OFF, 이중 소속 없음, 물건당 완료 1회, 오래된 장벽 거부가 기준이다. 고정 계획 검증과 새 LLM 협상을 구분한다.
2. **정보 경계 게이트:** 동료의 보이지 않는 선언·완료·질문 횟수를 바꿔도 자기 입력·깨우기가 동일해야 한다. A의 송수신 0, B의 자유문자열 차단, C의 본문 변경이 호스트 실행 명령을 직접 바꾸지 않음, 수신자 격리·평가 출력 역류 방지를 검사한다.
3. **작은 한국어 파일럿:** 저장 RGB의 막힘·파지 실패·집결 대기 3상황 × 2반복, 협상당 최대 6발화로 먼저 검사한다. 기준안은 세 로봇 모두 실제 한국어 송신·응답, 상황별 유효 왕복 1회 이상, 발화 95% 이상에서 literal 제외 한글 비율 ≥0.9, literal 보존 100%다. 그다음 같은 기능을 작업 진행 중 확인한다. 이는 언어·연결 검사이며 우열 실험이 아니다.
4. **소규모 비교와 본실험:** 정상·통로 경쟁·검증된 비공개 막힘·복구를 A/B/C의 짝지은 조건으로 실행한다. 로봇 ID·발견자·배치를 교차하고, 실패 주입을 “전체 두 번째 작업” 대신 사전 지정한 물체·사건으로 맞춘다. 파일럿으로 예산·반복 수를 정한 뒤 새 시험 조건과 소스를 고정한다. 실패·언어 이탈도 분모에 포함하고, 새 결과는 로컬 원본 및 평가 전용 TensorBoard 지표로 보존한다.
