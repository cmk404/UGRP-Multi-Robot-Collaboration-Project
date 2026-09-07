# UGRP 실행 로드맵

> 기준일: 2026-08-13
> 상태: 연구 시작 전
> 목표: 자연어 기반 분산 다중로봇 협력의 효과를 재현 가능한 실험으로 검증

## 0. 현재 출발점

현재 `ugrp` 저장소에는 연구 코드가 없다. 기존 자산은 다음과 같이 분리한다.

- 공식 계획: 로봇팔 3대 기반 블록 분류·협력 이송
- 최신 수정안: Isaac Lab 기반 휴머노이드 2–3대 가상 물류 환경
- 기존 구현: 실제 LLM·로봇과 연결되지 않은 브라우저 시연용 `decentral-cobot`

따라서 첫 구현은 기존 브라우저 시연을 확장하는 방식이 아니다. **작은 추상 환경에서 실험 프로토콜을 먼저 검증하고, 같은 인터페이스를 Isaac Lab 환경으로 옮기는 방식**을 기본 경로로 삼는다.

## 1. 최종 실험 목표

### 연구 질문

두 로봇이 큰 물체를 공동 이송하는 상황에서, 동일한 LLM을 사용하더라도 **peer-to-peer 자연어 통신**이 통신 없는 조건보다 장애물·고장·목표 변경 이후의 재계획 성공률을 높이는가?

### 최소 실험 구성

- 로봇: 2대
- 물체: 큰 상자 1개
- 목표: 도착 구역 1개
- 환경: 단순 물류 공간
- 행동: `navigate_to`, `approach_object`, `request_partner`, `wait_for_partner`, `start_transport`, `reroute`, `deliver`
- 예외: 통로 차단 1종부터 시작
- 중앙 계획자: 없음
- 환경 심판: 충돌·행동 유효성·동기화·성공 여부만 판정

### 비교 조건

| 조건 | 설명 | 용도 |
| --- | --- | --- |
| `rule` | 규칙 기반 분산 에이전트 | 환경과 평가 지표 검증 |
| `llm_no_comm` | LLM 에이전트가 메시지를 주고받지 않음 | 통신 없는 LLM 기준선 |
| `llm_peer_comm` | 동등한 LLM 에이전트가 자연어로 협상 | 주 실험 조건 |
| `central` | 중앙 계획자 | 시간과 자원이 허용될 때의 보조 비교군 |

### 핵심 지표

주 지표는 **예외 발생 후 복구 성공률**로 고정한다.

보조 지표:

- 전체 작업 성공률
- 파트너 요청·매칭 성공률
- 완료 step 수 및 wall-clock latency
- deadlock 발생률
- invalid action·파싱 실패 횟수
- 자연어 통신 turn 수
- 입력·출력 토큰 수와 추론 비용

## 2. 전체 순서와 완료 기준

| 단계 | 기간 | 할 일 | 산출물 | 다음 단계 진입 조건 |
| --- | --- | --- | --- | --- |
| 0. 범위 고정 | 8/13–8/15 | 공식안·수정안 승인 상태 확인, 연구 질문과 MVP 확정 | 연구 범위 1장, 결정 로그 | 팀·멘토가 환경과 실험 질문에 동의 |
| 1. 태스크 명세 | 8/15–8/17 | 상태·행동·성공·실패·예외·종료 조건 정의 | `task_spec.md` | 사람이 읽고 episode를 동일하게 재현 가능 |
| 2. 추상 환경 | 8/18–8/24 | 물리엔진 없이 상태 전이와 환경 심판 구현 | `run_episode()`와 rule baseline | 고정 seed에서 rule 조건이 반복 재현됨 |
| 3. 로깅 기반 | 8/21–8/25 | 모든 상태·행동·메시지·비용을 저장 | JSONL episode log, metric script | 로그 하나만으로 episode 재구성 가능 |
| 4. LLM 통합 | 8/25–8/31 | 통신 없음과 peer 통신 두 조건 구현 | agent API, message schema, prompt version | 두 조건이 동일 action space에서 실행됨 |
| 5. 파일럿 | 9/1–9/7 | 조건별 소수 반복, 실패 원인 분류 | pilot report | 주요 실패가 분류되고 재현됨 |
| 6. 본실험 | 9/8–10/4 | seed와 조건을 고정해 반복 실행 | versioned dataset | 누락 episode와 실행 오류가 없음 |
| 7. 분석 | 10/5–10/18 | 지표 계산, 조건 비교, 통계 검정 | tables, plots, analysis report | 결론이 실제 로그와 연결됨 |
| 8. 확장 검증 | 10/19–11/15 | L3 예외, 3번째 로봇 또는 Isaac Lab 이식 | robustness report | 핵심 결과의 적용 범위와 한계가 명시됨 |
| 9. 보고·발표 | 11/16–2027.01 | 보고서, 영상, 코드·데이터 정리 | 최종보고서와 발표 자료 | 재현 절차가 제3자에게 전달됨 |

날짜는 현재 기준의 실행안이다. 공식 UGRP 일정이나 멘토 지시가 다르면 결정 로그에 변경 이유를 남기고 갱신한다.

## 3. 첫 72시간 실행 목록

### Day 1 — 범위와 책임

1. 팀원과 멘토에게 공식 계획과 Isaac Lab 수정안을 비교한 1장 공유
2. Isaac Lab 전환이 공식 변경인지 확인
3. 다음 세 항목을 확정
   - 로봇 수: 우선 2대
   - 첫 환경: 추상 물류 환경
   - 첫 예외: 통로 차단
4. 팀원별 산출물 담당자를 정함
   - 환경·심판
   - 에이전트·통신
   - baseline·실험
   - 로깅·분석
5. 이번 주의 완료 기준을 고정
   - LLM 없이 rule agent가 큰 상자를 협력 이송
   - 장애물 발생 후 reroute 또는 실패를 로그로 기록

### Day 2 — 태스크 명세

`task_spec.md`에 아래를 작성한다.

- 좌표계와 공간 크기
- 로봇 초기 위치
- 상자 초기 위치와 목표 위치
- 로봇별 관측 범위
- 각 skill의 입력·출력·실패 조건
- 협력 이송을 시작할 수 있는 조건
- 두 로봇의 출발 동기화 조건
- 장애물 주입 시점과 위치
- deadlock 정의
- 성공·실패·timeout 정의
- episode 최대 step
- seed와 초기화 규칙

이 문서에 없는 상태나 예외는 첫 실험에 넣지 않는다.

### Day 3 — 환경 골격

처음부터 Isaac Lab이나 실제 로봇을 붙이지 않는다. 다음 순서로 최소 환경을 만든다.

```text
reset(seed)
  -> 초기 상태 생성
step(robot_id, action)
  -> 행동 유효성 검사
  -> 상태 전이
  -> 충돌·동기화·성공 판정
  -> 관측 반환
run_episode(policy, seed, config)
  -> episode 로그 저장
```

구현 완료 기준:

- 같은 seed와 같은 policy가 같은 결과를 냄
- invalid action이 환경을 망가뜨리지 않음
- 예외 이벤트가 지정한 step에 정확히 발생함
- episode가 성공·실패·timeout 중 하나로 종료됨

## 4. 단계별 구현 방법

### Phase 1 — 추상 환경과 심판

환경은 에이전트의 계획을 대신 만들면 안 된다. 환경이 제공하는 것은 다음뿐이다.

- 로봇의 관측
- 물체와 목표의 상태
- 행동의 실행 결과
- 충돌·동기화·성공 여부

환경이 하면 안 되는 것:

- 어떤 로봇이 파트너가 되어야 하는지 결정
- 역할 순서를 자동으로 지정
- 실패 시 최적 경로를 대신 계산
- LLM에게 전체 전역 상태를 몰래 제공

### Phase 2 — 규칙 baseline

규칙 baseline은 연구 결과를 내기 위한 경쟁자가 아니라 환경 검증용으로 먼저 만든다.

최소 규칙:

1. 큰 물체를 감지하면 파트너 요청
2. 파트너가 수락하면 각자 지정된 접근 위치로 이동
3. 둘 다 준비되면 공동 이송
4. 통로가 차단되면 고정 우회 경로 시도
5. 우회 실패 또는 timeout이면 실패 종료

이 baseline이 안정적으로 실행되기 전에는 LLM을 연결하지 않는다.

### Phase 3 — 공통 agent API

모든 조건은 같은 인터페이스를 사용한다.

```python
observation = agent.observe(environment_view, inbox)
action = agent.decide(observation)
result = environment.step(agent_id, action)
agent.receive(result)
```

조건별 차이는 다음으로 제한한다.

- `rule`: 규칙으로 `decide`
- `llm_no_comm`: inbox를 비움
- `llm_peer_comm`: inbox에 다른 로봇의 메시지를 제공

LLM 출력은 자연어 원문과 별도로 구조화된 action을 가져야 한다.

```json
{
  "speech": "I need a partner before transport.",
  "intent": "request_partner",
  "action": "request_partner",
  "object_id": "box_0",
  "target_id": "goal_0",
  "confidence": 0.82
}
```

파싱 실패, 실행 불가능한 action, 환경 거부 action은 성공으로 처리하지 않고 별도 오류로 기록한다.

### Phase 4 — 파일럿

파일럿은 통계 결론을 내기 위한 실험이 아니다. 파이프라인 오류를 찾는 단계다.

권장 실행:

- 조건: `rule`, `llm_no_comm`, `llm_peer_comm`
- 태스크: L1 단독 이송, L2 협력 이송, L3 통로 차단
- 조건별 episode: 5–10회
- 고정 seed와 별도 탐색 seed 모두 사용

분류할 실패:

- 환경 오류
- 관측 누락
- LLM 파싱 실패
- 잘못된 역할 제안
- partner handshake 실패
- 동기화 실패
- 충돌
- deadlock
- timeout
- 예외 복구 실패

실패 유형을 분류하지 못한 상태에서 prompt를 고치지 않는다.

### Phase 5 — 본실험

파일럿에서 환경과 로깅이 안정화되면 조건·seed를 사전에 고정한다.

기본 실행량:

- 3개 조건
- L1·L2·L3 세 단계
- 조건·단계별 20회 이상
- 동일한 초기 상태 seed를 조건 간 공유

총 180 episode가 기본 규모다. 3번째 로봇이나 중앙 baseline을 추가하면 별도 실험군으로 기록하고 기본 결과와 섞지 않는다.

모든 episode에 다음 메타데이터를 저장한다.

```text
experiment_id
condition
scenario
seed
environment_version
agent_version
model_id
prompt_version
config_hash
start_time
end_time
result
failure_reason
```

## 5. 저장소 구조

초기 구현은 다음 구조로 시작한다.

```text
ugrp/
├── README.md
├── ROADMAP.md
├── docs/
│   ├── task_spec.md
│   ├── decision_log.md
│   └── experiment_protocol.md
├── src/
│   ├── env/
│   ├── agents/
│   ├── baselines/
│   ├── logging/
│   └── evaluation/
├── configs/
├── scripts/
├── tests/
└── runs/                 # git에 원시 대용량 결과를 무조건 넣지 않음
```

코드의 책임은 다음처럼 분리한다.

- `env`: 상태, 행동 실행, 심판
- `agents`: 관측을 받아 행동·메시지를 생성
- `baselines`: 규칙·통신 없음·peer 통신 정책
- `logging`: episode와 token·latency 기록
- `evaluation`: 성공률·deadlock·비용 계산

## 6. 팀장 운영 순서

팀장은 모든 모듈을 직접 구현하지 않는다. 다음 네 가지를 소유한다.

1. 연구 범위 변경 승인
2. 인터페이스와 평가 지표 고정
3. 멘토 보고와 위험 제거
4. 조건 간 공정한 비교 보장

각 담당자에게는 기능명이 아니라 산출물을 할당한다.

```text
담당자 / 산출물 / 완료 기준 / 마감일 / 의존성
```

주간 회의는 다음 순서로 진행한다.

1. 지난 주 산출물 시연
2. 실패 로그 3개 검토
3. 지표 변화 확인
4. 막힌 의존성 제거
5. 다음 주 산출물과 완료 기준 확정

코드가 많아졌는지는 진척도가 아니다. 다음 세 가지가 진척도다.

- 같은 조건을 반복 실행할 수 있는가
- baseline과 실험 조건을 공정하게 비교할 수 있는가
- 실패 원인을 로그에서 설명할 수 있는가

## 7. 절대 순서를 바꾸지 않을 것

```text
범위 확정
  -> 태스크 명세
  -> 환경 심판
  -> rule baseline
  -> 로거
  -> 통신 없는 LLM
  -> peer 자연어 통신
  -> 파일럿
  -> 본실험
  -> 통계 분석
  -> Isaac Lab/실물 확장
```

다음 행동은 금지한다.

- 환경 심판보다 먼저 LLM prompt 최적화
- baseline 없이 LLM 성공률 주장
- 고정 seed 없이 조건 비교
- 자연어 원문을 저장하지 않고 parsed action만 저장
- 브라우저 demo를 실제 연구 실험으로 보고
- 예상 수치를 결과표에 입력
- 3대 로봇과 비전·보행을 동시에 시작

## 8. 첫 번째 실제 작업

이 로드맵 이후 가장 먼저 만들 파일은 코드가 아니라 아래 세 개다.

1. `docs/task_spec.md`
2. `docs/decision_log.md`
3. `docs/experiment_protocol.md`

그 다음 첫 코드 목표는 다음 한 문장으로 제한한다.

> 고정 seed의 단순 물류 환경에서 두 규칙 기반 에이전트가 큰 상자를 공동 이송하고, 통로 차단 후 성공 또는 실패를 재현 가능한 JSONL 로그로 남긴다.

이 목표가 통과되면 LLM을 붙인다. 통과하지 않으면 로봇 수·모델·프롬프트를 늘리지 않고 환경과 심판부터 고친다.
