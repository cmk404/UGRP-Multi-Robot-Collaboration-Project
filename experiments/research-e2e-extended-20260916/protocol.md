# 확장 예산 E2E 시험 — 사전 계획

사용자 요청: 현재 수정본으로 E2E 테스트를 진행한다.
PR #57 미병합 후보를 사용하며 main은 변경하지 않는다.
기존 18만 입력 토큰 시험은 모두 PREPARE에서 예산을 소진했으므로,
예산을 늘렸을 때 공동 파지·운반·해제까지 도달하는지 확인한다.

- natural/none 각 1회, seed=11, Gemini 3.8 Flash, temperature=.2,
  max_tokens=900, 요청 timeout=30초, 요청당 최대 3시도.
- 동작 48턴, 협상 최대 6턴, 보고 입력 토큰 soft cap 600,000,
  벽시계 soft cap 900초. 진행 중 호출은 경계를 넘을 수 있다.
  usage가 없는 실패 요청의 비용은 알 수 없으며 무료로 간주하지 않는다.
- 역할·행동 프롬프트, 입력 경계, 준비 조건, 물리 성공 기준, 초기 장면,
  카메라 배치/FOV·형상, weld OFF 및 로컬 임대 실행은 이전 후보와 동일하다.
  CLI에 기존 실행 함수의 토큰/시간 예산 선택만 노출한다. 소스 커밋 후
  두 조건을 모두 끝낼 때까지 코드를 고정한다.
- 실제 독립 LLM 2개 + RGB→원시 명령 임시 actor 경로다. 팀원의 최종
  로컬 파지 기술을 새로 구현하거나 교사 자세/시연을 실행에 넣지 않는다.
- 입력은 해당 로봇 RGB·공용 TOP RGB·자기 발행 명령 이력과 모델 peer claim이다.
  none은 자연어 전달만 차단하며 구조적 역할 합의는 공통이다.
  정답 좌표/관절/접촉/평가는 출력 전용이며 행동·단계 전환에 사용하지 않는다.

기록: 역할 합의, 각 단계 도달, 모델 READY/DONE과 별도 물리 판정,
정지 이유, 요청·재시도·명령 수, 시간·토큰·알 수 없는 비용, 전체 평가 시계열,
원본 경로/해시를 보존한다. 실제 입력/전송 본문 및 로컬 RGB 판단을 감사하고
영상을 검토한다. 성공하지 않더라도 최초 미통과 조건을 명시한다.
이 표본으로 통신 효과나 일반화된 성공률을 결론내리지 않는다.

```sh
python scripts/ugrp_session.py run e2e-extended-natural -- \
  mjpython scripts/run_research_camera_e2e.py --communication natural \
  --rounds 48 --max-input-tokens 600000 --max-wall-s 900 \
  --output outputs/research-e2e-extended-natural-20260916
```

none도 같은 설정과 별도 세션·output을 사용한다. raw는 outputs/ 로컬 보관이며
원격 백업이 아니다. 새 지속 서비스를 만들지 않고 종료 시 소유 세션을 정리한다.
