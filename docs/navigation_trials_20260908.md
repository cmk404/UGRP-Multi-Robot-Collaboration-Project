# Gemini 운반 시행착오 결과 — 2026-09-08

관측·판단 메모를 보강한 현재 정책은 입력 상한 120,000토큰에서 같은 seed 41 운반을 2회 연속 완주했다. Gemini가 매번 행동을 직접 선택했고 실제 목적지 내부 배치·놓기·완료를 검증했다.60,000토큰에서는 세 후보 모두 완주하지 못했다.

## 실험 결과

모두 로봇 1대, seed 41, 최대 30호출, 300 SIM초, Gemini 3.8 Flash, impratio 10, noslip 3, 통신 없음이다.

| 실험 | 입력 상한 | 실제 호출 | 실제 입력 토큰 | 운반 |
|---|---:|---:|---:|---|
| C1 관측·메모 | 60,000 | 11 | 55,967 | 실패 |
| C2 긴 행동 안내 | 60,000 | 10 | 52,631 | 실패 |
| C3 메모 회귀 수정 | 60,000 | 11 | 56,703 | 실패 |
| D1 예산 진단 | 120,000 | 18 | 94,065 | 성공 |
| D2 동일 조건 재현 | 120,000 | 17 | 86,818 | 성공 |

C2는 새 메모 검증기가 화면상 x=비율을 숨은 좌표로 오인해 유효한 행동 1회를 버린 회귀가 있었다. C3에서 수정했고, 보조메모 형식 오류도 정상 행동을 버리지 않도록 raw응답과 오류를 기록한다. C3에서는 별도로 오래된 모델응답 1회가 기존 시간검사에서 폐기됐다. 실패를 숨기거나 성공횟수에 포함하지 않았다.

## 무엇이 바뀌었나

- 원본NAV RGB에서 주황/자홍 색 영역의 화면비율, 목표 바닥색 영역, 직전 대비 크기·화면중심 변화를 제공한다. 기존 전진 stop 검사 결과도 현재 관측으로 전달한다. 색영역은 객체추적이나 안전경로 보장이 아니다.
- Gemini가 관측,현재 의도,전진 재개 조건을 navigation_note로 직접 작성하며 다음 호출에 전달된다. raw응답/실제 입력/행동/수락·거부/실제 제어결과는 llm-decisions.jsonl와 commands.jsonl에서 연결된다.
- 충분히 보이는 통로에서는 기존 허용 범위 내 긴 행동을 검토하도록 안내했다. fwd/turn은 정규화된 명령이고 duration만 초 단위다. 경로를 코드가 고르거나 모델명령을 임의 연장하지 않는다. 기존 0.25초 이하 RGB veto 및 파지·배치 기준은 유지했다.
- 검증기에 명시적 입력상한 옵션을 추가했다. 기본 60,000 검증은 그대로 엄격하며 실험파일의 상한을 그대로 믿지 않는다.

## 판단한 내용

초기 가설은 장애물 회피의 진행 근거와 의도 유지 부족이었다. 화면 판단검사3회에서는 이전에 거부된 전진이 회전으로 바뀌었고 C1에서도 제안시점 guard 거부는 0이었다. 그러나 이동 중 안전중단과 재정렬은 여전히 발생한다. 직진으로 바뀌면서 이전 우회방향 의도가 희미해지는 현상도 관찰했다.

현재 정책은 충분한 관측호출을 주면 이 구간을 실제로 통과했다. 첫 성공은94,065토큰/18호출/204.252SIM초, 재현은86,818토큰/17호출/201.766SIM초였다. 따라서 현재 채택한 방법은 관측·기억 보강과120k상한의 조합이다. 예산만의 효과나 각 프롬프트 변경의 독립 효과는 분리 입증하지 않았다. 별도 bypass_side/bypass_until은 다음 비용최적화 후보로 남기고 현재 성공한정책은 유지했다.

## 검증 및 한계

관련 검사 121 passed / 23 subtests. 성공 2회 각각 자동 검사 14개 통과: 원본RGB해시,실제모델응답,회계,실행피드백 전달,입력·시간·호출한도,행동전체체인,최종물리·놓기판정,종료후동작,영상디코딩. 영상1643/1623프레임 전체디코딩 후 각각 시간순12장,NAV18/17장,최종손목을 수동검토했다. 물체의 최종높이는 둘다약0.01589m로 바닥에 놓였고,장애물/동료2mm초과침투 지표는0이다. 전프레임 수동동작감사,다른seed·배치,3대협업,실물은 검증하지 않았다.60k내 성공도 미해결이다.

모든 판단과 가설 수정은 실험 일지에 기록했다. 이번 작업은 고정 장면 3호출과 전체 5실험을 합쳐 70호출,362,703reportedinput을 사용했다. 시작한 프로세스 그룹 6개 모두 종료. UGRP 예외에 따라 로컬에만 저장했다.

## 자료

- [상세 실험 일지](/Users/changmin/projects/ugrp/outputs/navigation-trials-20260908/journal.md)
- [첫 성공 영상](/Users/changmin/projects/ugrp/outputs/navigation-trials-20260908/d1-budget/solo-41/motion-1x.mp4)
- [재현 성공 영상](/Users/changmin/projects/ugrp/outputs/navigation-trials-20260908/d2-repeat/solo-41/motion-1x.mp4)
- [재현 입력·메모 요약](/Users/changmin/projects/ugrp/outputs/navigation-trials-20260908/d2-repeat/solo-41/review-data.json)
- [재현 검증 결과](/Users/changmin/projects/ugrp/outputs/navigation-trials-20260908/d2-repeat/solo-41/budget-verification.json)
- [재현 원본NAV 모음](/Users/changmin/projects/ugrp/outputs/navigation-trials-20260908/d2-repeat/solo-41/nav-qa.jpg)

## 재현 명령

이 명령은 새 실제 Gemini 사용량을 소모한다. output과 세션 이름은 새 이름을 사용한다. 기존 고정60k pilot 기본값은 바꾸지 않았다.

```sh
.venv-sim-worker-mac/bin/python scripts/ugrp_session.py run navigation-repro -- .venv-sim-worker-mac/bin/python -m scripts.evaluate_gemini_team --output outputs/navigation-repro/solo-41 --seed 41 --robots 1 --seconds 300 --model gemini-3.8-flash --max-calls 30 --max-input-tokens 120000 --input-request-estimate 6000 --impratio 10 --noslip-iterations 3 --communication none --request-timeout 30 --max-transient-failures 2 --record
.venv-sim-worker-mac/bin/python -m scripts.verify_gemini_budget_run outputs/navigation-repro/solo-41 --max-input-tokens 120000
```
