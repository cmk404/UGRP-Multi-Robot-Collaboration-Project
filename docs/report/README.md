# 연구 보고서 초안 (살아 있는 문서)

**기준 커밋: `origin/main` `1cd9ea1` (2026-09-26 확인).**

최종 보고서를 쓸 때 바로 쓸 수 있도록, 저장소에 이미 있는 근거만 모아 장별로 정리한 초안이다.
새 실험·시뮬레이션·모델 호출 없이 기록을 읽어 작성했다.

## 장 구성

| 장 | 내용 |
|---|---|
| [01 연구 질문과 동기](01-research-question.md) | 자연어 대화가 효율에 주는 영향, 저장소가 인용한 선행연구 |
| [02 실험 설계](02-experiment-design.md) | 주 4조건 + 참조 상한 R, 입력 계약, 대화 SIM 비용, 지휘자 순환·허브-스포크 |
| [03 환경](03-environment.md) | `zone_wide` 계열 지도, 문·복도, 벽 AprilTag, 화물 카탈로그, 접촉 프로필, MasterPi와 손목 어안 카메라 |
| [04 실행기](04-executor.md) | 자기 카메라 실행(M1 계획, 위치 추정 결과, wrist 스킬 v1/v2와 `gt_stub` 한계), 교사의 역할 한정 |
| [05 결과 (지금까지)](05-results.md) | 실험별 결과와 검증 범위. 교사 가능성·gt_stub 스킬 격리·오프라인 평가·실제 로봇 성공을 구분 |
| [06 결정 이력](06-decision-history.md) | 날짜별 사용자 방향 변경과 결정 로그 인용 |
| [07 한계와 남은 일](07-limitations.md) | 미검증 항목, 차단 요인, 진행 중 작업 |

## 이 초안을 읽는 규칙

- 각 장 첫머리에 **기준 커밋**을 적었다. 이후 main이 바뀌면 그 커밋부터 diff를 보고 갱신한다.
- 모든 사실 주장에는 출처(파일·실험 ID·PR 번호)를 붙였다. 저장소에서 근거를 찾지 못한 항목은
  `[출처 확인 필요]`로 표시했고, 추측으로 채우지 않았다.
- 날짜가 붙은 기록의 "현재"는 그 기록 당시를 뜻한다([AGENTS.md](../../AGENTS.md)).
  여기서 인용한 수치는 각 기록의 실행 SHA·조건 범위 안에서만 유효하며, 현재 main에서 새로
  실행한 결과가 아니다.
- **"설계 v1"** 은 Codex 통합 연구 설계
  `docs/design/2026-09-25-zone-dialogue-study-design-codex.md`를 뜻한다. 아직 main에 없고
  `origin/claude/records-0926` 브랜치와 PR
  [#180](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/180)에 있으므로
  링크는 그 PR을 가리킨다. 문서 상태는 "설계 제안(검증 전)"이고, 본문의 `r1` 고정 지휘·별도
  commander·교사 실행기 전제는 사용자 결정으로 대체됐다([06](06-decision-history.md)).
  #180이 병합되면 링크를 상대 경로로 바꾼다.
- **아직 이 연구의 로봇 성공은 없다.** 자기 카메라 실행기는 준비 단계이고, 주 4조건으로 실행한
  결과도 없다([05](05-results.md), [07](07-limitations.md)).

## 갱신 절차

1. `git fetch origin && git log --oneline 1cd9ea1..origin/main` 으로 새 병합을 확인한다.
2. 새로 병합된 PR의 `experiments/<ID>/` 기록을 읽고 [05](05-results.md) 표에 행을 추가한다.
   기존 행은 덮어쓰지 않는다.
3. 사용자 결정이 바뀌면 [06](06-decision-history.md)에 날짜·인용과 함께 덧붙이고, 앞선 결정은
   기록 당시 내용대로 남긴다.
4. 각 장의 기준 커밋을 새 SHA로 갱신한다.
5. 결과를 보고할 때는 [docs/tensorboard.md](../tensorboard.md)의 스냅샷 절차를 함께 따른다.
