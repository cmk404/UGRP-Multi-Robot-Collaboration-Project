# Jev 실행 감시 API 파일럿 — 2026-09-20

관련 이슈: #75. 사용자 승인: 실제 API 호출까지 수행.

## 사전 고정 범위
- 저장된 두 운반 에피소드의 6개 시점, 각 2회, 최대 12회. 자동 재시도 없음. 첫 HTTP/전송/스키마 오류에서 중단하고 기록 보존.
- 정상 성공 기록 seed18 open-minus의 20/80, seed19 open-plus의 100/300/600/892 시점; 각 시점 직전 4개와 현재 관측, 모델 슬롯 r1만 사용. 선택된 알려진 사례로서 holdout 일반화 평가가 아님.
- 명시적 허용 목록: 자기 ACT RGB 완료 추정, 자기 RGB 부착 연속성 추정, 자기 직전 발행 명령, 자기/공용 RGB의 프레임 평균 절대 변화. 이미지 해시와 원본 JSON 해시 기록. 원본의 물리 로봇 ID와 모델 슬롯을 구분.
- 동적 정답 위치/관절/접촉/평가 결과/상대 비공개 상태는 보내지 않음. 원본 이미지·결과·로컬 경로도 API에는 보내지 않음. 입력은 저장된 영상에서 파생한 텍스트 상태이며 Jev의 직접 영상 인식 시험이 아님.
- 이미지는 매개변수/FOV/외관을 바꾸지 않고 읽기만 함. 시뮬레이터·로봇·서비스 시작 없음, actuation false. 완료나 방출 행동 없음.
- 비교 규칙: 최신 자기 RGB 부착 추정 false이면 hold_and_escalate; 자기 done 추정 또는 5개 모두 직전 명령 0이면 reobserve; 그 외 continue. 이 규칙과의 일치율은 정답률이 아님.
- 기록: 실제 모델 버전, 원본 요청/응답, 선택별 확률, confidence, 토큰, HTTP 왕복 지연. HTTP 지연은 연결 준비 포함/시각 처리 제외. 물리 복구·낙하·충돌 성능이나 LLM 대비 우위를 주장하지 않음.
- 키는 숨김 입력 또는 TYPESAFE_API_KEY로 읽고 저장하지 않음. 공식 endpoint만 사용하고 redirect 금지. 전체 소스는 호출 전에 커밋.

## 실행
```sh
python scripts/jev_execution_shadow.py \
  --source-root /path/to/experiment-v2/final \
  --output outputs/jev-shadow-NEW-ID --execute --prompt-key --repeats 2
```
실행 환경에는 Pillow가 필요. 기존 시뮬레이션 Python을 재사용하며 새로운 GPU 환경을 설치하지 않음.

## 공식 API 근거
2026-09-20 조회: https://docs.typesafe.ai/api , https://docs.typesafe.ai/confidence
