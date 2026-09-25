# 07 한계와 남은 일

**기준 커밋: `origin/main` `1cd9ea1` (2026-09-26 확인).**

## 7.1 이 연구가 아직 말할 수 없는 것

| 주장 | 왜 아직 못 하는가 |
|---|---|
| 한국어 대화가 작업 효율을 바꾼다 / 바꾸지 않는다 | 주 4조건으로 실행한 결과가 하나도 없다. "아직 이 세 조건으로 실행한 결과는 없다"([docs/decision_log.md](../decision_log.md)) |
| `dynamic`이 무통신보다 낫다 = 대화의 효과 | ZC2의 이득은 게시판·중재·메시지 묶음 전체이고, 메시지는 한국어가 0건이었다([05](05-results.md) ZC2 행) |
| 자기 카메라만으로 로봇이 임무를 완수한다 | 스킬 결과는 모두 `gt_stub_eval_only`이고 문을 통과하지 않았다. 위치 추정은 교사 주행 오프라인 평가다([04](04-executor.md)) |
| wrist 스킬 v2가 9/10으로 잘 된다 = M1 진척 | 같은 gt_stub 조건이며 M1이 아니다. v1의 1/5과 시나리오가 달라 직접 비교도 못 한다(PR [#181](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/181)) |
| 교사가 3대 운반에 성공했으므로 학생도 가능하다 | 교사는 정답 좌표·IK·접촉을 쓴다. "교사 주행은 로봇 성공으로 세지 않는다"([docs/decision_log.md](../decision_log.md)) |
| TOP 인식기 성능이 자기 카메라 성능이다 | `top_cargo_v1/v2`는 TOP 전용이며 "Cargo2의 개선을 자기 RGB 성능으로 승계하지 않음"([설계 v1](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/180) 8절) |
| 시뮬레이션 결과가 실물 MasterPi 성능이다 | 저장소 전체 규칙. 손목 카메라 설치 위치는 "잠정·미검증"이고 SIM 유효 시야는 약 54°×42°로 실물 어안보다 좁다([03](03-environment.md)) |

## 7.2 실행기 쪽 차단 요인

[04](04-executor.md) 마지막 절과 같다.

1. **자세 추정기 연결.** PR #177 파티클 필터를 `PoseEstimate` 인터페이스에 붙여야 한다.
   현재는 stub이다.
2. **문 통과 정책.** `carry_p30` 주행과 `look_p20` 정지 관찰, 문기둥 태그 `_tags_v2`.
   PR [#178](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/178)의
   dev·test 실행이 미완료다.
3. **carry 자세 태그 가시성.** 상자를 든 수평 carry에서 태그 가시율이 0%였다. 20° 숙임
   해법은 **사후 진단 에피소드 1개**뿐이고 test가 없다([05](05-results.md) owncam-loc 행).
4. **위치 추정 오차가 슬롯 배치(±6 cm)와 놓은 뒤 확인에 주는 영향**은 아직 측정하지 않았다.
5. **자기 RGB로 판단해야 하는 네 가지**(올바른 물건을 들었는가 / 앞이 막혔는가 / pickup slot에
   물건이 있는가 / 구역에 내려놓았는가)는 아직 구현·검증되지 않았다
   ([설계 v1](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/180) 4절).
6. **2대·3대 공동 운반의 자기 카메라 경로는 전부 없다.** 역량 목록은 long_beam 2대를
   "최상", tri_frame 3대를 "최상+"로 분류하고 "카메라 기반은 전부 없다"고 적는다
   ([own-camera inventory](../design/2026-09-25-own-camera-inventory-claude.md) 3절).
   운반 중에는 빔이 자기 카메라 대부분을 가리고, 자기 RGB만 쓴 Gemini 공동 파지는 0회
   성공이었다(같은 문서 (d)).

## 7.3 입력 경계에 남은 위반

Codex PR 검토가 확정으로 분류한 것 중 미해결 항목이다
([docs/design/2026-09-25-zone-pr-review-codex.md](../design/2026-09-25-zone-pr-review-codex.md)).

- **PR #169 blocker** — 정답 기반 교사 결과가 모델 입력·깨우기·목표 차감으로 직접 돌아간다.
  "교사 운동 예외는 알림 예외가 아니다."
- **L1 미수정** — independent에서 교사가 동료 의도로 로봇을 멈추는 문제. 경계 감사가
  "심각도 높음"으로 분류했고 수정되지 않았다
  ([zone-comm-boundary-audit](../../experiments/2026-09-25-zone-comm-boundary-audit/README.md)).
  A2는 이동 중 분기를 제거했지만 `_taken_by_peer`가 동료 phase/outcome을 읽는다.
- **L4 잔존** — 교사 종료가 `RobotResults.end()`에 연결된다. PR #170의 RGB 판정만으로는
  제거되지 않는다.
- **호스트 조정 장치** — dynamic의 board·host arbitration·conflict notice·peer-job-end wake
  다섯 스위치가 모두 ON이고, C2 tie-break(최소 robot ID 우선)가 프롬프트에 있다. 주 조건에서는
  모두 OFF/제거해야 한다(설계 v1 6절 호스트 장치 표).
- **통로 자동 양보** — hard-routes 교사가 통로 내 위치·들린 상자·robot ID로 후퇴할 로봇을
  정한다. "정답 기반 중재는 본실험에서 제거"해야 한다(같은 표).
- **미관측 장애물이 경로 선택에 반영되지 않아야 한다.** "아직 관측하지 않은 장애물을 planner에
  넣어 자동 우회시키면 통신의 정보 효과가 사라진다"(설계 v1 6절).

## 7.4 파일럿·지표 쪽 한계

[zone-dialogue-ko-pilot](../../experiments/2026-09-25-zone-dialogue-ko-pilot/README.md) 7절과
PR [#188](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/188)에서.

- 작업 효율·완료율·makespan을 측정하지 않았다(SIM 없음, 바뀐 결정을 실행하지 않음).
- 표본이 작다(단일 턴 변형당 20개, 창 6개, V0 반복 8쌍). plan 결정 변화는 기준 잡음과
  구분할 수 없다.
- 미보고 토큰(total−prompt−completion)이 0이 되는 원인을 확인하지 않았다.
- 사건 기반 창, 이동 중 발화, 미지 장애물, 2로봇 직접 전달 상황은 기록에 없어 검사하지 못했다.
- 수동 행위 라벨은 한 명이 비맹검으로 붙였다. 행위 규칙 v2는 수동 표본(한국어 20·영어 12)
  밖에서 검증되지 않았고 완료 상태 보고 2건을 놓치며, **다음 실행에만 적용된다.**
- SIM 비용 모델 계수는 모두 잠정이고 이 프로젝트 모델의 측정값이 아니다. 모델·tokenizer 확정
  후 검증·동결이 필요하다(PR [#186](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/186)).
- 저장 로그의 비용 재계산은 근사다. "비용이 바뀌면 행동과 관측 시점도 달라지므로 최종 비교는
  다시 실행한다"(설계 v1 5절).

## 7.5 환경·자료 쪽 한계

- **미등록 장애물/가림은 미착수다**([docs/e2e_status_20260925.md](../e2e_status_20260925.md)).
  통신이 정보 이득을 낼 수 있는 핵심 시나리오여서 본실험 전에 필요하다.
- **구역 과제 time-limit 검사 미착수**(같은 문서).
- **pickup slot 정의**가 새 지도 버전에 필요하다(설계 v1 3절).
- raw 영상·로그·원시 wire는 **로컬 보관이고 원격 백업이 아니다.** owncam-loc·owncam-skill·
  ko-pilot 모두 `outputs/` 아래 gitignore 경로에만 있다고 명시한다. 해시만 저장소에 있다.
- 공용 Mac 부하 평균이 12–175까지 변동한다. 동기 SIM이라 결과에는 영향이 없다고 기록했으나,
  wall 시간 비교는 이 조건에서 하지 않는다([03](03-environment.md)).

## 7.6 실행 예산 결정이 필요하다

| 항목 | 제안값 | 상태 |
|---|---|---|
| 주 LLM 파일럿 | 5상황 × 4조건 × 3 paired seeds = 60회, HTTP 시도 상한 5,400회 | 설계 v1 10절 제안. "실행 승인이나 성능 충분성의 판단이 아니다" |
| ZC3 코호트 | 36 / 72 / 216회 (약 1.9 / 3.7 / 10.9 h) | [초안, 고정 전](../../experiments/2026-09-25-zc3-prereg-draft/README.md). 사용자 예산 결정 필요 |
| SIM 비용 배율 스윕 | `0, 0.5, 1, 2, 4` | PR #186에 구현, 실행 없음 |

"대규모 학습·평가는 실행 예산·자원을 먼저 정한다"([AGENTS.md](../../AGENTS.md)).

## 7.7 권장 순서

설계 v1 9·10절과 PR 의존 관계를 합친 것이다. 이 초안의 판단이며 사용자 승인 사항은 아니다.

1. **M1 완주.** PR #178 폐루프 실행 + #177 추정기를 `PoseEstimate`에 연결 + v2 스킬.
   여기까지가 [05](05-results.md)의 "R. 실제 로봇 성공" 첫 행이 된다.
2. **PR #169 병합 게이트.** 설계 v1의 G0. 그 전에는 runner 변경·runner 기반 실험에 착수하지
   않는다. 교사 영수증·호스트 조정 장치를 robot-facing 경로에서 끊는다.
3. **패키지 A→C/D/I 통합.** PR #184·#185·#186·#187을 러너에 연결한다. 현재 셋 다
   "러너 미통합"이다.
4. **정보 경계 게이트.** 동료의 보이지 않는 선언·완료·질문 횟수를 바꿔도 자기 입력·깨우기가
   동일해야 한다. (1) 송수신 0, (4) 자유문자열 차단, 수신자 격리, 평가 역류 방지를 검사한다.
5. **no-LLM 물리 smoke** 24회 → **LLM 파일럿** 60회 → 본실험. 예산을 먼저 동결한다.
6. 새 결과는 실패까지 포함해 `experiments/<ID>/`와 새 TensorBoard 스냅샷에 남기고
   ([docs/tensorboard.md](../tensorboard.md)), 이 보고서의 [05](05-results.md)·[06](06-decision-history.md)를
   갱신한다.
