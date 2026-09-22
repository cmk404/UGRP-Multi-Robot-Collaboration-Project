# 통신 연구 병렬 작업 배정

작성: 2026-09-22. [연구 TODO](research_todo.md)의 첫 구현 배치다. 연구 전체 완료나 본실험 실행을 뜻하지 않는다.
사용자 요청에 따라 같은 UGRP 프로젝트에 네 개의 독립 세션과 worktree를 만든다. 출발 main은 `120cc821b6a1d5c104aab8c8ef2260cf7f8c9a7b`이며 연구 TODO는 아직 미병합인 PR #93의 참고 문서다.

담당 네 세션의 지정 모델은 사용자 추가 요청에 따라 `gpt-6-astra`, 추론 수준은 `xhigh`다. 네 세션의 `thread_settings_applied` 기록에서 모두 적용을 확인했다. 첫 조사 턴은 기본값 `gpt-5.6-sol/high`로 시작했으므로 전체 작업이 처음부터 Astra였다고 표현하지 않는다. 모델 확인을 위한 일시 종료 요청은 해제하고 작업을 계속하도록 전달했다.

## 역할과 완료 산출물

| 담당 | 연구 TODO | 이번에 완료할 것 | 다음 담당에게 넘길 것 |
|---|---|---|---|
| A · 연구 설계·정보 경계 감사 | R0/R1 | 주 비교 프로토콜, 로봇별 정보 출처 표, 기존 코드의 누출/공정성 검사, 두 파일럿 시나리오 후보 | B/C에 수정 요구와 go/no-go 기준, D에 비교 조건·분모·지표 |
| B · 공통 RGB 실행 어댑터 | R2 | 현재 RGB 스킬에 연결할 최소 실행 포트, 지원 범위, 격리·동의·만료·거부 테스트 | C에 버전 있는 포트 계약과 fixture, D에 별도 평가 출력 경계 |
| C · 독립 에이전트 통신 런타임 | R3 | 세 독립 agent의 none/structured/natural 전환, 기억/inbox/예산·응답 만료, 추적 가능한 이벤트 | B와의 연결 코드, D에 JSONL 예제와 스키마, A에 감사 표본 |
| D · 평가 도구·파일럿 준비 | R4/R6 | 시행 manifest, 누락·실패를 보존하는 집계, dry-run/fixture 검증, 유한 실행 계획 | 통합 뒤 사용할 6회 파일럿 명령·예산 항목·실행 전 검사 |

실행 포트와 통신 정책을 다른 담당자가 맡아 동시에 개발하되, 처음에 작은 인터페이스부터 합의한다. A/D는 실행기 완성을 기다리지 않고 현재 코드 감사와 합성 fixture로 진행한다. 합성 fixture·저장 응답 재생·실제 모델 호출·물리 실행은 서로 다른 검증 단계로 표시한다.

## 편집 소유권

- A: `docs/research_parallel/a-*`, `tests/test_rgb_communication_boundary_audit.py`, 전용 감사 fixture. production 수정 없이 결함과 재현 근거를 B/C에 전달한다.
- B: 새 `harness/rgb_execution_port.py`, 필요 시 `harness/rgb_execution_contract.py`, 전용 포트 테스트/fixture, `docs/research_parallel/b-*`. 기존 RGB 스킬 파일 수정이 필요하면 기존 ACT 작업과 먼저 범위를 조율한다.
- C: 새 `harness/rgb_communication_runtime.py`, 필요 시 `scripts/run_rgb_communication.py`, 전용 런타임 테스트/fixture, `docs/research_parallel/c-*`. 기존 CoELA 모듈의 변경이 필요하면 C가 단독 담당한다.
- D: 새 `scripts/evaluate_rgb_communication.py`, 필요 시 `harness/rgb_communication_evaluation.py`, 전용 평가 테스트/fixture, `docs/research_parallel/d-*`.
- 조정 세션: 본 문서, 연구 TODO, 공용 README/current_status/실험 인덱스와 최종 CI 등록의 통합. 각 담당자는 공용 파일 수정 요구를 handoff로 남긴다.

파일명은 새 파일의 초기 배정이다. 기존 구현 재사용이 더 작고 안전하면 변경 근거와 새 소유권을 먼저 공유한다. 다른 담당자의 worktree를 직접 편집하지 않는다.

## 인터페이스와 합치는 순서

1. **초기 계약:** B가 actor 관측/자기 상태/행동 제출/진행/종료 포트와 평가 전용 경계를 제안하고 C/D가 확인한다. 공용 타입 파일은 B만 작성한다. A는 허용 정보인지 감사한다.
2. **독립 개발:** C는 fake execution port, B는 fake command/clock, D는 합성 trace, A는 현행 코드 및 격리 fixture로 진행한다. 미완료 의존성을 실제 통합 완료로 표시하지 않는다.
3. **추적 계약:** C가 `schema_version`, `run_id`, `robot_id`, `condition`, 관측/결정/메시지/행동 식별자, SIM/wall 시각을 포함한 최소 JSONL 예제를 D에 제공한다. D는 별도 평가 출력과 결합한다.
4. **1차 납품:** 각자 테스트·커밋·push·PR·CI 상태와 `docs/research_parallel/<담당>-handoff.md`를 제출한다. handoff에는 정확한 SHA, 인터페이스, 지원/미지원, 검증/미검증, 다음 명령과 의존 PR을 쓴다.
5. **통합 검사:** 확정 커밋을 별도 통합 작업에서 연결하고 A가 정보 경계를 다시 감사한다. 관련 테스트를 공용 CI에 등록한다. main 병합은 해당 PR에 대한 사용자 명시 승인 뒤에만 한다.
6. **실제 파일럿:** 통합 SHA·모델/지도/물리/보정·환경·시행/전체 예산·종료 조건을 고정하고 정상+복구/정보 차이 두 조건 × 세 통신 방식의 6회 연결 실험을 한 담당자만 제출한다. 이 6회로 통계적 우열을 선언하지 않는다.

이번 위임 배치의 실행 권한은 문서·소스·로컬 오프라인 단위/계약 테스트와 파일럿 준비까지다. 외부 모델 호출, 물리 시뮬레이션, 렌더링, 학습, 원격 작업 제출, 실물 구동은 시작하지 않는다. 선행 조건이 충족되지 않으면 필요한 후속 실행과 제한을 보고한다.

## 기존 작업·자원과의 경계

- 기존 **Continue ACT map diversity**는 그대로 유지한다. ACT/학습/회수와 #89/#91/#92의 작업을 네 세션에서 중복 수행하지 않는다. 필요한 의존성은 먼저 해당 세션의 최신 상태를 읽고 문의한다.
- 외부 실험을 여러 담당자가 동시에 시작하지 않는다. 다음 실제 실행 단계의 기본 환경은 Colab CLI이며, 유한한 Kaggle 배치는 해당 절차를 따른다. 과거 특정 Mac 실행 허용을 확대 적용하지 않는다.
- 무거운 환경·모델 파일을 worktree마다 복제하거나 새 시뮬레이션 환경을 만들지 않는다. 기존 원본·프로세스·평가 산출물을 삭제·정리하지 않는다.
- 현재 RGB·정적 지도 입력 경계, weld OFF, 카메라/FOV 보존을 모든 담당자에게 동일 적용한다. 평가 정답은 actor 입력/행동 보정/성공 통보로 전달하지 않는다.
- 새 실제 결과의 TensorBoard 처리는 결과 회수 담당자 한 명이 공통 경로에서 수행한다. 합성 fixture는 연구 결과로 등록하지 않는다. UGRP에서는 Drive와 별도 예약 자동화를 사용하지 않는다.

## 세션 식별자

호스트는 모두 `local`, 프로젝트 ID는 `b2b5ee2e-e479-4371-8aa5-5c541d9f7efa`다. 네 세션의 실제 착수와 독립 작업 경로를 확인했다. 아래 상태는 배정 시점의 스냅샷이며 작업 완료를 뜻하지 않는다.

| 담당 | task ID | worktree ID |
|---|---|---|
| A | `01a0c783-4c16-7fa2-a517-57a2060c0a11` | `9208` |
| B | `01a0c783-4c8d-74e3-ac99-297dcb21cd12` | `6f6c` |
| C | `01a0c783-4c1c-7af0-8c4f-af217b4d88cd` | `e282` |
| D | `01a0c783-4cde-74b2-8c66-4f98bad3fd40` | `cf5f` |

조정 세션: `01a0c302-9437-75b2-a3db-908498e80b0b`. 기존 ACT 세션: `01a0c1c2-6acf-7182-b575-1486508963fb`.
