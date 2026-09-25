# 05 결과 (지금까지)

**기준 커밋: `origin/main` `1cd9ea1` (2026-09-26 확인).**

이 장의 모든 수치는 각 실험 기록의 실행 SHA·조건 범위 안에서만 유효하다. 서로 다른 날짜·
실행 경로의 성공률을 합산하지 않는다([AGENTS.md](../../AGENTS.md)).
대부분 **조건당 1회**라 사례로 읽는다.

## 5.1 주장 종류를 먼저 구분한다

| 종류 | 뜻 | 여기서 해당하는 것 |
|---|---|---|
| **T. 교사 물리 가능성** | 정답 좌표·IK를 쓰는 교사가 그 동작을 물리적으로 해낼 수 있음 | 화물 카탈로그, noslip, 하드 경로, 구역 배송·통신 벤치마크 |
| **S. gt_stub 스킬 격리** | 자기 카메라 스킬만 시험하되 위치 추정 자리에 정답을 넣음 | wrist 스킬 v1·v2 |
| **O. 오프라인 평가** | SIM을 새로 돌리지 않거나 교사 주행 로그를 사후 채점 | 위치 추정, 인식기, 한국어 파일럿, 경계 감사 |
| **R. 실제 로봇(학생) 성공** | 허용 입력만으로 로봇이 임무를 완수 | **이 연구에서는 아직 없다** |

교사 성공을 학생 성공으로 보고하지 않으며, 계산 경로·교사 주행도 학생 성공이 아니다
([AGENTS.md](../../AGENTS.md) 지도 경로 예외).

## 5.2 실험별 결과 표

### T. 교사 물리 가능성

| 실험 ID | 조건·소스 | 결과 | 검증 범위와 한계 |
|---|---|---|---|
| [zone-cargo-catalogue](../../experiments/2026-09-25-zone-cargo-catalogue/README.md) (PR #164) | 정답 교사, weld OFF, `d81514a` | solo 3/3, pair 2/2, trio 1/1 운반 성공. 한 대 적은 조건 3/3 들기 실패. 단독 한계 0.66 kg(0.68 kg 전복) | 조건당 1회, 구역 교사 연동 전. 자기 RGB 인식·파지는 전혀 없음 |
| 같은 기록의 noslip 후속 (PR #167) | opt-in `cargo_noslip_v1`(`noslip_iterations 10`, `8886cf5`) | 60초 미끄러짐 ≤0.51 mm, 4 m·두 번 회전 운반 pair/trio 3/3 (기존 프로필은 crate·frame 실패) | 한 대 적은 조건 여전히 실패. 전역 solver 변경의 부작용 검사는 화물 없는 평지 주행뿐([PR 검토](../design/2026-09-25-zone-pr-review-codex.md)) |
| [zone-hard-routes](../../experiments/2026-09-25-zone-hard-routes/README.md) (PR #173, 병합 전) | fixture, LLM 0회, `dc27695`/`3183650` | plan_first·dynamic 28/28 목표 달성. 통로 대치 6회 모두 해결. makespan: zone_wide 150–156 s / door 217–252 s / two_doors 188–212 s / corridor 333–396 s | 교사 조건이며 **LLM 협업 근거가 아니다**. 복도 출구 밖 정면 밀기 1회(independent) 미해결 |
| [zone-wide-arena](../../experiments/2026-09-25-zone-wide-arena/README.md) (PR #155) | 실LLM, 교사 실행기 | 규칙 응답 3/3(`bc4a784`), ZW1 4/4 목표 달성(`e4ccaf6`), 수정본 ZW2 4/4(`f30c9f2`) | 두 번 세기 결함을 찾아 고쳤으나(`9412326`) ZW2에서 같은 상황이 재현되지 않아 **수정의 실제 LLM 검증은 아니다**. 호출 수 우열은 코호트마다 달랐다 |
| [zone-dispatch](../../experiments/2026-09-25-zone-dispatch/README.md) | 교사 실행기, plan_first vs dynamic × G5/G8 | Z3(`e520a3a`) 4/4 목표 달성. SIM 시간은 한쪽으로 기울지 않고 LLM 호출·토큰은 dynamic이 적음 | 이동·IK가 정답 좌표인 교사 조건. **두 방식 모두 대화하므로 무통신 대비 통신 효과는 측정하지 않았다**([docs/current_status.md](../current_status.md)) |
| [zone-communication](../../experiments/2026-09-25-zone-communication/README.md) (ZC1/ZC2) | 실LLM 18회, `7ccd6c3`, 교사 실행기, zone_wide G8+여분 2, seed 12–14 | 정상/파지실패 주입 각 3회: independent 0/3·1/3(6회 중 5회 과잉 배달), plan_first 3/3·0/3, **dynamic 3/3·3/3** | ZC1은 교사 막힘으로 중단 후 재등록. **dynamic의 이득은 게시판·중재·메시지 묶음 전체**이며 메시지만의 효과가 아니다. 한국어 메시지 0건. 입력 경계 수정 전 실행이고 재검증하지 않았다 |
| [dynamic-team-recovery](../../experiments/2026-09-25-dynamic-team-recovery/README.md) | 실패 주입 11회(v51 E1–E4, v56 V1–V4, v58 W1–W3) | 결정 대화는 모두 선택지 안에서 끝남. 두 화물 전체 성공은 접근 실패 뒤(E1·V1·W1)와 v58 상자 실패 뒤(W3) | **파지 실패 뒤 빔 복구는 학습된 미세 정렬 지원 범위 밖으로 미해결** |
| [plan-guidance](../../experiments/2026-09-25-plan-guidance/README.md) | v45, 같은 장면·지시, C0/C1/C2 각 5회 | 모두 물리 성공. 동시 실행 선택 3/5 → 5/5 → 5/5. SIM 141.26초(직렬 181.46–216.06초) | C1이 천장이라 미리보기(C2)의 추가 효과는 미측정 |
| [map-goto-navigation](../../experiments/2026-09-25-map-goto-navigation/README.md) (PR #148, 기본 OFF v59) | 저장 계획 재생, open/seed11 | A* 2/2, 기존 경로 2/2 성공. 상자 먼저 운반 48.3→26.4 SIM초 | 조건당 1회. **실제 LLM 실행·다른 지도는 미검증**. 시작 위치는 TOP 추정 |

### S. gt_stub 스킬 격리 (M1 아님)

| 실험 ID | 조건·소스 | 결과 | 검증 범위와 한계 |
|---|---|---|---|
| [zone-owncam-skill](../../experiments/2026-09-25-zone-owncam-skill/README.md) v1 (PR #176) | `pose_source=gt_stub_eval_only`, 시드 501–505 각 1회, `d016c04`, SIM 한도 300 s, weld OFF, `local_contact_fine` | GT 파지 2/5, **GT 슬롯 배치 1/5**. 자기 판정과 GT 5/5 일치, 거짓 성공 0, weld eq_active 0, 벽 접촉 0 | 위치 추정 자리에 시뮬레이터 정답을 넣은 스킬 격리. **문을 통과하지 않았고 M1 성공이 아니다.** 실패 원인은 조명으로 인한 접근 교착(3건)과 운반 grasp drift(1건) |
| 같은 기록 v2 (PR #181, 병합 전) | 같은 입력 경계, 시드 511–520 각 1회, `edd075d`, SIM 한도 420 s | **GT 슬롯 배치 9/10**, GT 파지 10/10, 자기 판정 일치 9/10, 거짓 성공 0, 재장착 7/10, 후퇴 0 | 같은 gt_stub 조건, **문 통과 없음, M1 아님**. 개발 시드 403–406 미합산. v1의 1/5와는 시나리오가 달라 직접 비교 불가 |

### O. 오프라인 평가

| 실험 ID | 조건·소스 | 결과 | 검증 범위와 한계 |
|---|---|---|---|
| [zone-owncam-loc](../../experiments/2026-09-25-zone-owncam-loc/README.md) (PR #177) | 교사 주행, 자기 wrist RGB만, `zone_wide_door_tags_v1`, 사전 등록 `13c64c4`, 보정 고정 `efdf5f4` | **사전 등록 게이트 dev·test 모두 G1·G3 실패, G2만 통과.** 사후 look-only 재계산은 test 문 근처 p50 1.9 cm/0.19°, p90 5.8 cm/0.48°. 상자 든 carry 태그 가시율 0% | **교사가 주행한 오프라인 평가이고 폐루프가 아니다.** 20° 숙임 carry(가시율 97%)는 사후 진단 에피소드 1개뿐이고 test가 없다. 시뮬레이터 영상 고정 조명·질감 |
| [zone-dialogue-ko-pilot](../../experiments/2026-09-25-zone-dialogue-ko-pilot/README.md) (PR #172) | **SIM 없음.** ZC2 저장 요청 재질의, HTTP 124회, gemini-3.8-flash temp 0.2 | V2(전체 한국어) 한글 비율 0.992, 한국어≥0.9 **19/20**, 코드 전환 1/20, ID 오류 0. 다턴 창 4개에서 세 로봇 모두 한국어 발화(한글 비율 1.00) | **작업 효율·완료율·makespan은 측정하지 않았다.** V2에서 claim 결정 10개 중 4개가 V0와 달라졌다. "차례대로 말하는 구조(직렬화)의 효과이며 한국어 자연어 자체의 효과로 볼 근거가 없다" |
| [zone-comm-boundary-audit](../../experiments/2026-09-25-zone-comm-boundary-audit/README.md) (PR #161) | ZC2 저장 요청 274건(wire 일치) 감사 | 누설 3건 발견: independent의 교사 상자 선점(L1, 미수정), 칸 id(L2, 수정), 전체 라운드 번호(L3, 수정). RGB 좌표 출처는 렌더 반사실로 깨끗함 확인 | 감사이며 새 실행 아님. **ZC2는 수정 전 실행이며 재검증하지 않았다** |
| [zone-rgb-color](../../experiments/2026-09-25-zone-rgb-color/README.md) (PR #163) | `881b356`, zone_wide dev 6시드/test 10시드(130장면), 정답은 평가 전용 | test에서 `top_zone_v2` TOP 청록 88.9%·초록 98.6%·빨강 100%·노랑 95.0%, 오검출 0/520. `own_zone_v2` 자기 RGB 74.0–80.3%(1.2 m 이내 97%), 오검출 0, 색 혼동 0 | **검출만이고 제어 미연결.** 원거리(>1.2 m)는 방향만 정확하고 거리가 +21% 길다 |
| [zone-cargo-perception](../../experiments/2026-09-25-zone-cargo-perception/README.md) (PR #168) | `top_cargo_v1`, `07a958d`, TOP 입력 | test 병합 TOP 완전 가시 can 216/220·tile 219/219·beam 184/184·crate 209/212·frame 93/93, 종류 혼동 0, 오검출 2 | **TOP 전용이므로 자기 RGB 성능으로 승계할 수 없다.** 제어 미연결. can 서쪽 2–2.6 cm 치우침, 틀 안쪽 상자 1개 소실 |
| [zone-cargo-perception-v2](../../experiments/2026-09-25-zone-cargo-perception-v2/README.md) (PR #174) | `top_cargo_v2`, `330e856`, v1은 바이트 그대로 | v1→v2: 빔 293/297→297/297, 붙은 평행 빔 쌍 5/11→11/11, 시야 경계 쌍 8/12→12/12, 틀 안쪽 상자 0/40→37/40, 오검출 8→7 | 회귀: 빔 중복 0→4(신뢰도 <0.5). TOP 전용, 제어 미연결 |
| [zone-rgb-outcome](../../experiments/2026-09-25-zone-rgb-outcome/README.md) (PR #170) | TOP 전후 비교 모듈 v2, 새 fixture test 12회(작업 269) | 잘못된 delivered 0, 미확정 7(2.6%), 정답 256 | **2026-09-25 결정으로 로봇 입력용 역할은 폐기**, 평가 보조·향후 자기 카메라판 참고용. 색 동일성 한계·L4b 남음. PR 검토는 "이 훅만으로 L4는 제거되지 않는다"고 지적 |
| [zone-team-jobs](../../experiments/2026-09-25-zone-team-jobs/README.md) (PR #165) | 열린 바닥 교사 스모크, `f9446a2`, weld OFF | pair_beam·trio_frame 각 1/1. 목표 v2, TeamJob 원자적 commit, 무통신 팀 형성 규칙(RendezvousRule), 착지 영역·심판 v2 | **구역 교사·실행기 연결(A2)은 계획만.** 스모크이며 협업 성능 비교 아님 |
| [r3-regrasp-pose](../../experiments/2026-09-25-r3-regrasp-pose/README.md) (PR #162) | v61 Y1·Y2 오프라인 재분석(새 SIM 없음) | 실패 슬롯 r3는 물리 r1. 빔 정지 위치 0.59 mm 변화만으로 잔차가 12.11/12.51/15.26 → 6.50/6.15/6.35(한계 11.56)로 복귀 | 수정 미실행. 이 원인은 현재 단계에서 보류 |
| [zc3-prereg-draft](../../experiments/2026-09-25-zc3-prereg-draft/README.md) (PR #160) | ZC2 원본 18회로 비용 재계산 | 6칸 블록당 입력 590k 토큰, 드라이버 wall 913초. 예산 3단계(36/72/216회 ≈ 1.9/3.7/10.9 h) | **초안, 고정 전.** SIM·LLM 실행 없음, 사용자 예산 결정 필요 |

### R. 실제 로봇(학생) 성공

**이 연구 조건에서는 아직 없다.** 근거는 셋이다.

1. 사용자 결정 절 마지막이 "**아직 이 세 조건으로 실행한 결과는 없다**"고 적는다
   ([docs/decision_log.md](../decision_log.md) 2026-09-25 "주 연구 조건 변경").
2. 후속 방향 갱신도 "입력 변경·조건 추가는 아직 구현하거나 실행하지 않았다"고 적는다(같은 절).
3. 자기 카메라 실행기의 두 결과 모두 `gt_stub_eval_only`이고 문을 통과하지 않았다
   ([04](04-executor.md)).

`docs/current_status.md`의 "자기 카메라로 위치를 추정하는 실행기는 없다"도 같은 상태를 적는다.
TensorBoard 보충 스냅샷 기록도 "어떤 run도 자기 카메라 학생 운반 성공이 아니다"라고 명시한다
(PR [#183](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/183) 한계 절).

## 5.3 한국어 대화 파일럿을 더 자세히

이 결과가 연구 질문에 가장 가까우므로 따로 정리한다
([experiments/2026-09-25-zone-dialogue-ko-pilot](../../experiments/2026-09-25-zone-dialogue-ko-pilot/README.md)).

프롬프트 변형 네 가지를 같은 결정 지점 20개에 적용했다. V0는 원래 영어(재생 대조),
V1은 영어 프롬프트 + 언어 규칙 한 문장, V2는 전체 한국어, V3는 message를 정형 객체로 바꾼 것이다.

| 변형 | 한글 비율 | 한국어≥0.9 | 코드 전환 | V0 대비 claim 결정 동일 | 입력/출력 토큰(평균) |
|---|---|---|---|---|---|
| V0 | 0.00 | 0/20 | – | 기준 | 6,599 / 165 |
| V1 | 0.954 | 16/20 | 4/20 | 10/10 | 6,659 / 162 |
| V2 | **0.992** | **19/20** | 1/20 | **6/10** | 6,836 / 200 |
| V3 | 해당 없음 | 해당 없음 | 해당 없음 | 8/10 | 6,756 / 201 |

읽을 때 주의할 점이 기록에 있다.

- **재생 잡음이 크다.** V0 wire는 원본과 20/20 바이트 동일했지만 원본에서 유효했던 결정과 같은
  값은 17/19였고, 원문 텍스트가 완전히 같은 경우는 0/20이었다. 반복 호출 8쌍 중 결정이 같은
  것은 6쌍이다. 불일치는 모두 plan 모드였고 claim 모드는 원본 10/10·반복 4/4로 안정적이었다.
- **한국어 토큰 부담은 작았다.** V2 입력 +236토큰(+3.6%), 출력 +35토큰이며 미보고 토큰이 없어
  total은 오히려 V0보다 작았다. 입력은 이미지가 대부분이다. "고정 배수는 가정하지 않았다."
- **협상은 거의 없었다.** 다턴 창 6개에서 질문·거절·정정은 한 번도 없고 직접 전달(한 명 지정)도
  0건이다. M1 V2만 상대 발화를 받아 말했고 그조차 두 번째 라운드는 합의 내용 되풀이였다.
  창 하나에 약 42k 토큰이 들었고 "메시지 텍스트가 아니라 이미지를 매 턴 다시 보내는 비용이
  대부분"이다.
- **충돌 감소는 직렬화 때문일 수 있다.** 원본에서 cyan-1 충돌이 났던 M1이 V2·V3 모두 충돌 없이
  끝났지만 기록은 이를 "가능한 설명"으로만 두고, "A에도 같은 순서·창 구조를 주거나 동시 응답을
  공통으로 유지해야 C−A가 언어 효과만 반영한다"고 적는다(PR
  [#188](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/188)에서
  인과 문장을 완화).

파일럿의 통합 권고는 C 조건 프롬프트를 V2로 고정하고, **A·B도 같은 한국어 과제 설명을 쓰고
메시지 형식만 바꾸는 것**이다. 그래야 언어 효과와 통신 효과가 섞이지 않는다(같은 기록 6절).

## 5.4 진행 중인 연구 계층 작업

실행 결과는 아니지만 본실험 전제 조건이다. 모두 DRAFT PR이고 병합 전이다.

| 패키지 | PR | 내용 | 검증 |
|---|---|---|---|
| A 연구 계약·입력 | [#187](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/187) | 조건 registry, 입력 allowlist, 시나리오 기반 주문서, 지도 도식 | 70 passed. SIM·모델 호출 없음 |
| C 프롬프트·통신 | [#184](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/184) | 4조건 한국어 프롬프트와 통신 프로토콜 | 오프라인 전용 |
| D SIM 비용·스케줄러 | [#186](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/186) | 비용식·사건 큐. **러너 미통합** | 가짜 시계 테스트 |
| I 평가·결과 | [#185](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/185) | 효율·대화 지표, 짝 비교, 입력 경계 감사 | 합성 로그 64건 |
| 파일럿 리뷰 후속 | [#188](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/188) | #172 지적 3건 수정(원시 wire 덮어쓰기, 인과 문장, 행위 규칙 v2) | 61 passed. 규칙 v2는 **다음 실행에만** 적용 |
| A2 러너 연결 | [#169](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/169) | 혼합 화물·팀 운반·벽을 구역 실행기에 배선 | WIP. 실행기 연결 미완료 |
| 자기 카메라 폐루프 | [#178](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/178) | tags_v2, 추정 기반 A* 주행 | dev·test 실행 미완료 |

## 5.5 TensorBoard

2026-09-25에 병합됐지만 스냅샷이 없던 기록 6개를 `outputs/tensorboard/0925-zone-supplement`
(116 run)으로 변환했다(PR [#183](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/183)).
116/116 run을 EventAccumulator로 다시 읽고 813개 스칼라를 원본 JSON에서 독립 재계산해 대조,
불일치 0이다. 기존 스냅샷(owncam-skill v1/v2, zone-dispatch, ZC2, zone-wide 등)은 source 경로로
확인해 중복 변환을 피했다. 보기 설정은 `outputs/tensorboard-view.json`의 키로 관리한다.

같은 기록의 한계 문장을 그대로 옮긴다. "교사·GT 심판·오프라인·사후 코호트를 합산하지 않으며,
어떤 run도 자기 카메라 학생 운반 성공이 아니다."
