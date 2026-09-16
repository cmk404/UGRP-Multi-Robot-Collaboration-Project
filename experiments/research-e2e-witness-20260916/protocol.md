# 부분 완료 교착과 영상 오판 수정 검증

사용자 요청: 이전 E2E에서 확인한 문제를 해결한다. PR #57 후보이며 병합하지 않는다.

## 변경과 완료 기준

- GRASP의 fresh DONE + READY에서는 완료한 포트의 설정을 유지하고 미완료 포트만
  명령을 받는다. 만료/철회/오래된 허가 및 다른 공동 단계의 편측 명령은 차단한다.
- 임시 RGB actor의 PREPARE READY / GRASP DONE은 독립 RGB witness가 재검토한다.
  witness는 actor의 답·명령 이력·peer claim·평가를 받지 않는다. 자기 RGB/공용 TOP의
  현재·이전 원본과 TOP 중앙 절반을 2배 확대한 표시만 본다. 실제 카메라/FOV는 동일하다.
  확대는 새 해상도나 가림 뒤 정보를 만들지 않는다.
- 두 손가락과 그 사이의 물체, 열린/닫힌 상태, 자기 식별과 정지를 모두 확인한
  confidence ≥.8 보고만 기존 claim을 유지한다. 확인 실패는 UNCERTAIN/wait이며
  GRASP 실패는 모두 정지 후 새 PREPARE를 요청한다(최대 3회 GRASP 시도).
  다음 자기 actor 입력에는 자신의 영상 검토 이력만 전달한다.
- 이는 모델의 자기 확증을 줄이는 장치다. 별도 물리 평가를 대체하지 않으며,
  두 모델의 동의도 접촉이나 하중 유지의 참값을 보증하지 않는다.

## 사전 고정 검증

1. 자동 회귀: 두 로봇 각각 선완료, fresh DONE 유지, 남은 명령 발행 후 양쪽 완료,
   만료/철회/오래된 허가/완료 포트 재발행/다른 단계 차단; 영상 근거 누락 시 보류.
2. `tests/fixtures/research_visual_witness/cases.json`: 과거 잘못된 PREPARE 2개와
   GRASP 2개, 교사 초기화 후 실제 파지가 된 자세의 PREPARE 2개를 이미지로 분류한다.
   각 원본과 SHA를 고정한다. label/source는 출력용이며 모델 요청에 포함하지 않는다.
   양성 교사 장면의 분류는 학생 독립 실행 성공이 아니다. 전체 결과를 보존한다.
3. 최종 후보 source commit 고정 후 natural/none 각 1회 실제 E2E:
   seed 11, Gemini 3.8 Flash, temperature .2, max_tokens 900, timeout 30초,
   요청당 최대 3시도, 행동 48턴/협상 6턴, 입력 600,000토큰·900초 soft cap.
   witness 요청·실패·토큰·시간도 동일 예산에 포함한다. usage 없는 비용은 미상이다.
   독립 2개 actor의 원시 명령 시험이며 팀원의 최종 로컬 파지 정책이 아니다.
4. actor/witness 실제 wire, 원본·확대 이미지 파생, 자기 명령·피드백 출처,
   역할 합의·재시도, 실행 명령과 물리 결과를 감사하고 영상을 검토한다.

자기 RGB·공용 RGB·자기 명령 이력·허용 peer claim 경계를 유지한다. 평가 참값은
출력 전용, weld OFF, 물리 성공 기준·초기 위치·외관·카메라 배치 변경 없음.
SIM은 추론 중 멈춘다. 통신 효과·새 조건 일반화는 이 소표본으로 결론내리지 않는다.
실행은 소유 세션 래퍼를 사용하며 완료 시 정리한다. raw outputs는 로컬 보관,
선별 소형 재현 fixture와 요약 기록은 Git에 저장한다.

```sh
python scripts/probe_research_visual_witness.py tests/fixtures/research_visual_witness/cases.json \
  --output outputs/research-visual-witness-probe-20260916
python scripts/ugrp_session.py run e2e-witness-natural -- \
  mjpython scripts/run_research_camera_e2e.py --communication natural --rounds 48 \
  --max-input-tokens 600000 --max-wall-s 900 --output outputs/research-e2e-witness-natural-20260916
```

none도 같은 설정으로 별도 output/세션에서 실행한다. 진단 후 코드 변경이 필요하면
진단을 따로 보존하고 최종 후보의 두 조건 전체를 다시 검증한다.
