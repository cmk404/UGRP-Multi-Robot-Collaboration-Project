# 02 실험 설계

**기준 커밋: `origin/main` `1cd9ea1` (2026-09-26 확인).**
출처가 `docs/design/2026-09-25-zone-dialogue-study-design-codex.md`인 항목은 아직 main에 없고
`origin/claude/records-0926` 브랜치(PR [#180](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/180))에 있다.
설계 문서의 상태는 "설계 제안(검증 전)"이다.

## 2.1 주 4조건과 참조 상한 R

조건 축은 **통신 채널만** 바꾸고 지도·작업·로봇·물리·입력을 모두 같게 둔다
([docs/decision_log.md](../decision_log.md) 2026-09-25 "후속 방향 갱신" 항목).
구현된 조건 registry는 `harness/zone_study_contract.py`(PR
[#187](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/187))에 있다.

| 조건 | 통신 채널 | 근거 |
|---|---|---|
| (1) `no_comm` | 고수준 메시지 송수신 0 | 사용자 결정 2026-09-25 |
| (2) `peer_ko` | 자유 한국어 동료 대화. mesh, 수신자 선택 가능(분산) | 같음 |
| (3) `leader_ko` | 로봇 한 대가 지휘를 겸한다. **허브-스포크**(지휘자↔각 추종자만), 추종자끼리 직접 전달 없음. 추종자는 한국어로 보고·이의 제기 가능 | 사용자 승인 2026-09-25 |
| (4) `peer_structured` | (2)와 같은 정보를 고정 스키마로만. 자유 문장 없음 | 사용자 결정 2026-09-25 |
| 참조 R `central_rgb_reference` | **주 조건 아님.** 전지적 지휘자가 세 로봇의 카메라를 모두 받고, 로봇에는 LLM이 없다 | 같음 |

조건 (4)를 둔 이유는 결정 로그에 있다. "정형 메시지 통제 조건은 같은 정보를 고정 스키마로만
주고받게 해서, 자연어의 효과와 정보 공유의 효과를 분리한다."
R을 주 조건에서 뺀 이유도 같은 절에 있다. "모든 로봇의 카메라를 보는 전지적 지휘자는 주 조건이
아니라 참고용 상한으로 따로 둔다."

Codex 설계는 R을 수학적 최적 상한으로 표현하지 말라고 적는다. "단일 모델의 오류나 병목 때문에
실제 상한보다 낮을 수 있으므로 수학적 최적 상한으로 표현하지 않는다"
([설계 v1](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/180) 2절).
평가 구현에서도 R은 짝 비교에서 기본 제외이고 `--include-reference`로만 포함한다
(PR [#185](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/185)).

### 주장할 수 있는 비교와 할 수 없는 비교

[설계 v1](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/180) 7절 표를 그대로 따른다.

| 비교 | 뒷받침하는 주장 | 단독으로 뒷받침하지 못하는 주장 |
|---|---|---|
| (2)−(1) | 자유 한국어 통신을 허용한 전체 효과 | 자연어가 정형 정보 공유보다 우수함 |
| (4)−(1) | 고정 schema 정보 공유 효과 | 자연어 효과 |
| (2)−(4) | 해당 자유 대화 프로토콜과 schema의 차이 | 내용 선택까지 다른 상태에서 언어 형식만의 효과 |
| (3)−(2) | star 지휘·권한 집중·지휘 부담의 결합 효과 | 중앙화 하나의 순수 효과 |
| 주 조건−R | 정보가 풍부한 중앙 참조와의 격차 | 한국어 대화의 인과 효과 |

## 2.2 지휘자 순환과 허브-스포크

사용자 승인(2026-09-25) 내용은 결정 로그에 항목별로 있다.

- 로봇 한 대가 지휘자를 겸하고, **고정하지 않는다. 시행·시드마다 r1/r2/r3로 돌려서 누가
  지휘자인지와 그 위치의 영향을 상쇄한다.**
- 통신은 지휘자 ↔ 각 추종자만 잇는 허브-스포크다. 추종자끼리는 직접 대화하지 않는다.
- 지휘자는 작업 배정, 팀 운반 역할, 좁은 문 통과 순서를 정한다.
- 추종자는 지시를 따르되 막힘이나 파지 실패 같은 상황은 한국어로 보고하고 이의를 제기할 수 있다.
- 지휘자는 시작 시, 보고를 받았을 때, 누군가 할 일이 없어졌을 때 호출된다.

구현된 순환 규칙은 `robots[seed % 3]`이다(seed 11→r3, 12→r1, 13→r2;
PR [#187](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/187)).
follower↔follower 전달은 거부되고, 평가 쪽에서도 허브-스포크 위반(follower↔follower,
leader 방송)을 채널 위반으로 센다(PR
[#185](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/185)).

Codex 설계 v1 본문은 `r1` 고정 지휘와 별도 commander(③b)를 권고했으나, 이는 사용자 결정으로
대체됐다. 설계 문서 머리말이 그 대체 관계를 적고 있다
([설계 v1 머리말](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/180)).

## 2.3 입력 계약 — 매 호출에 무엇을 주는가

사용자 결정(2026-09-25, [docs/decision_log.md](../decision_log.md))에 따라 로봇 LLM은 매 호출에
다음만 받는다.

1. **정적 지도** — 벽·문·복도·구역 A/B/C·집하 칸(pickup bay/slot), 버전과 해시 포함.
   AprilTag 랜드마크도 정적 지도 특징으로 등록한다([03](03-environment.md)).
2. **작업 지시서(주문서)** — 물품 종류, 개수, 필요한 로봇 수, 목적 구역, 집하 칸 슬롯 수준의
   대략적 초기 위치. **반드시 시나리오 설정에서 만들고 시뮬레이터 상태에서 만들지 않는다.**
3. **자기 손목 RGB**(MasterPi 팔 끝 어안 `robot_cam`)와 **자기 발행 명령 이력**.
4. 해당 조건이 허용한 **실제 수신 메시지**.

금지 입력은 다음과 같다.

- 공용 TOP 카메라 원본과 **TOP에서 만든 좌표·물체 라벨·구역별 개수·완료 판정·요약문**
  ([설계 v1](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/180) 2절).
- 실시간 상태(위치·완료 여부·변화). "로봇은 이를 자기 카메라, 자기 명령 이력, 대화로만
  알아낸다"(결정 로그).
- 다른 로봇의 RGB와 raw 명령 로그, 전역 작업표, 숨은 사건 일정, simulator body ID.
- 시뮬레이터 전용 `nav_cam`([04](04-executor.md)).
- 교사 정답 기반 영수증. 설계 v1은 "문구를 '작업 종료'로 바꾸는 것만으로 L4가 사라지지
  않는다"고 적는다(4절).

구현 쪽 검증은 PR [#187](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/187)에
있다. 네 주 조건의 `static_map`·`order_sheet`·`own_rgb_refs`·`own_command_history`·`self_belief`가
바이트 동일함을 테스트가 확인하고, 검증기가 TOP 프레임·정답 pose·측정 관절·접촉·교사 영수증·
완료/성공 플래그·동료 상태·숨은 사건·평가 지표 키를 중첩 위치까지 거부한다. 주문서는
`sim.*`/`mujoco` 임포트 없는 별도 프로세스에서 같은 해시가 나오는지 확인한다.

### 자기 위치는 정답이 아니라 belief로 유지한다

설계 v1 3절은 `region: "unknown"`, `last_visual_anchor`, `confidence`, `sources`를 담은 belief
구조를 제시하고 "마지막 목적지는 도착 증거가 아니다", "호스트가 현재 pose로 보정하지 않는다"고
적는다. 저장소 규칙도 같다. "발행 명령은 실제 관절 상태·이동 성공이 아니다"
([AGENTS.md](../../AGENTS.md)).

### 리터럴 보존

한국어 프롬프트 안에서도 로봇 ID(`r1`/`r2`/`r3`), 물건 ID, 구역 문자 `A`/`B`/`C`, `role`,
통로 ID, JSON 키와 열거값은 번역하거나 바꾸지 않는다(설계 v1 3절의 공통 프롬프트 골격).
구현에서는 JSON 키를 ASCII만 허용하고, 자유 메시지는 한국어가 없으면 거부하며 라틴 단어를
지표로 기록한다(PR [#187](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/187)).
파일럿에서 실제로 측정한 준수율은 [05](05-results.md)에 있다.

## 2.4 대화·사고 시간의 SIM 비용

사용자 결정: "말하고 생각하는 시간도 SIM 시간을 소모해야 한다"
([docs/decision_log.md](../decision_log.md) 2026-09-25).

문제 진단은 두 곳에 같은 내용으로 있다. 현재 구역 러너는 **LLM 응답을 기다리는 동안 SIM 시간이
전진하지 않는다.** `team.ask()`가 future 결과를 받은 뒤 루프 끝에서만 물리를 전진시키고
makespan은 `motion_started` 이후 SIM 시간이다
([설계 v1](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/180) 5절;
PR [#186](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/186)이
`scripts/three_robot_runtime.py:150`, `scripts/run_zone_dispatch.py:332`, `:275`, `:339`를 인용).
그래서 생각과 대화가 공짜이고, 연구 질문을 물을 수 없다.

구현된 비용식은 `harness/zone_sim_cost.py`(PR
[#186](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/186))에 있다.

```
d_q = quantum * ceil( scale * (alpha + a_in*N_in + beta*N_out + gamma*U) / quantum )
```

- 기본값은 **모두 잠정(`provisional=True`)**: `alpha=1.0`s, `a_in=0.0002`s/tok,
  `beta=0.02`s/tok, `gamma=0.3`s/발화, 전달 `0.1`s, 오류 `0.5`s, timeout `20`s, `quantum=0.1`s.
  "이 프로젝트 모델의 측정값이 아니다. 모델·tokenizer 확정 후 검증·동결이 필요하다."
- `scale` 배율 `0, 0.5, 1, 2, 4`를 스윕하고 `scale=0`(`zone_sim_cost.v1_free`)은 비용 없는
  대화의 진단 조건이다.
- 설계 v1의 예시(출력 120토큰·1발화 → 3.7초)는 `a_in=0`과 같고, 입력 8,000토큰이면 5.3초다.

실행 의미는 설계 v1 5절이 7개 규칙으로 고정했다. 요지는 (a) 호출 시작 시각의 관측에 대한
결정이다, (b) 호출한 로봇은 안전한 hold로 들어가고 **다른 로봇과 물리는 계속 진행한다**,
(c) 응답·메시지는 `t+d_q` 전에 노출되지 않는다, (d) 동시 호출 비용은 겹친다,
(e) **wall 시간·API 완료 순서가 SIM trace를 바꾸지 않는다**.
스케줄러 구현(`harness/zone_event_scheduler.py`, PR
[#186](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/186))은
가짜 시계 단일 SIM 사건 큐이고, 동시 호출 3×5.3초가 makespan 5.3초가 되는 것을 확인했다.
저장 로그 재계산은 모두 `approximate: True`로 표시한다. 비용이 바뀌면 행동·관측 시점도 바뀌므로
최종 비교는 재실행해야 한다.

## 2.5 호출 정책(개발 시작값)

설계 v1 6절과 PR [#186](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/186)의
`CallPolicy`가 같은 값을 쓴다. 모두 잠정이다.

| 항목 | 값 |
|---|---|
| 자기 카메라 관측 주기 | 1 SIM초 |
| 호출 계기 | 시작, 로컬 영상 belief의 의미 있는 변화, 자기 타이머 만료, 실제 메시지 수신 |
| actor당 최소 호출 간격 | 2초 |
| 동시 미완료 호출 | 1개 |
| idle 재검토 / 진행 중 재검토 | 10초 / 60초 |
| 예산 | actor당 30호출, 전체 90시도 |

조건별 차이는 계기뿐이다. (1)은 메시지 계기만 없고, (2)·(4)는 mesh 수신자, (3)은 star 수신자만
계기를 얻는다. "교사 종료·동료의 숨은 완료·전역 진행률은 호출 계기가 아니다"(설계 v1 6절).
공정성의 정의도 명시돼 있다. "호출 수를 강제로 같게 만드는 것이 아니라 판단 자격·처리 규칙·
상한을 같게 하고 실제 추가 호출 비용을 지불하게 하는 것."

## 2.6 지표와 실패 처리

주 지표는 설계 v1 7절에 정의돼 있고, 평가 구현은 PR
[#185](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/185)에 있다.

- **전체 성공률** — 고정 SIM·계산 예산 안에서 주문을 충족한 비율. 정책 실패·deadlock·예산
  소진 포함.
- **SIM makespan** — 추론·발화·전달·물리 대기를 포함한 배송 완료 시각.
- **실패를 분모에 유지한다.** 구현의 주 시간 지표 `par_makespan_sim_s`는 성공에 실제 makespan,
  비성공에 `horizon × 2`를 부과한다. `sim_horizon`·`budget_exhausted`·`deadlock`·`aborted`·
  `api_failure`·`policy_failure`·`orders_incomplete`가 모두 성공률·배송률 분모에 남는다.
- **대화 지표** — 한국어 준수(리터럴 제외 한글 비율, 코드 전환, literal 손상, 침묵),
  사실성, 행위 유형, 결정 연결(`message_id → delivered_at → decision_sources → 행동 변경`),
  정보 이득, 오정보 비용.
- **평가 역류 금지.** 평가자는 정답 상태와 TOP을 쓸 수 있지만 "평가 결과를 로봇의 다음
  입력이나 호출 계기로 되돌리지 않는다"(설계 v1 7절). 구현은 계산 전후 직렬화를 비교해
  기록이 바뀌면 오류를 낸다.
- 파일럿 규모에서 유의성 검정을 만들지 않고, 짝 10개 미만은 `small_sample`로 표시한다
  (PR [#185](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/185)).

"모델이 메시지를 인용했다는 사실은 인과 효과의 증명이 아니다"(설계 v1 7절). 인과 확인은
호출 계기를 유지한 **내용 제거**와 **전달 자체 제거**를 나눠 재추론하는 방식으로 하며,
저장된 응답을 그대로 재생하는 비교로는 주장하지 않는다.

## 2.7 규모(제안값, 실행 승인 아님)

설계 v1 10절의 제안이며 "실행 승인이나 성능 충분성의 판단이 아니다".

| 단계 | 제안 규모 |
|---|---|
| 저장 RGB + fake clock | 모든 조건·③b·R 각각 |
| 물리 no-LLM smoke | 6상황 × 주 4조건 × 1 seed = 24회 |
| 주 LLM 파일럿 | 5상황 × 4조건 × 3 paired seeds = 60회 |
| 새 사건 표현력 | (2)/(4) × 3 seeds = 6회 |
| 시행당 예산 | 1,800 SIM초, HTTP 시도 90회, actor당 30회, 출력 768토큰 |

주 파일럿의 HTTP 시도 상한은 5,400회, 보조까지 포함하면 7,020회다. ZC3 사전 등록 초안은
별도로 예산 3단계(36 / 72 / 216회, 약 1.9 / 3.7 / 10.9 h)를 계산해 두었고 **고정 전 초안**이며
사용자 예산 결정이 필요하다([experiments/2026-09-25-zc3-prereg-draft](../../experiments/2026-09-25-zc3-prereg-draft/README.md)).

## 2.8 은퇴한 조건

`plan_first`(제안자 순환 + 만장일치 ACK)는 주 연구 조건에서 뺀다. 이유는
"협상 프로토콜의 변형일 뿐이고 연구 조건이 아니다"이며, Z1–Z3·ZC1·ZC2 재현을 위해 코드는
남긴다([docs/decision_log.md](../decision_log.md) 2026-09-25 "주 연구 조건 변경" 절).
설계 v1 8절은 `--allow-retired-coordination` 뒤에 보존할 것을 제안하고, 새 비용식을 적용한
실행을 과거 결과의 동일 재현으로 부르지 말라고 적는다.
