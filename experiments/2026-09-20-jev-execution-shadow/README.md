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

## 실제 API 결과

실행 소스 `cf1330a0c8870da9acbdef6f4c24cf1b4025f4ea`. 실제 모델 `jev-1.13.0`. 6개 상태를 각 2회 호출하여 12/12 HTTP 200 및 응답 스키마 검증 통과.

| 상태 | Jev 1회 / 2회 | 규칙 |
|---|---|---|
| 20260918-open-minus-20-r1 | continue / continue | continue |
| 20260918-open-minus-80-r1 | continue / continue | continue |
| 20260919-open-plus-100-r1 | reobserve / reobserve | continue |
| 20260919-open-plus-300-r1 | reobserve / reobserve | reobserve |
| 20260919-open-plus-600-r1 | reobserve / reobserve | reobserve |
| 20260919-open-plus-892-r1 | hold_and_escalate / hold_and_escalate | hold_and_escalate |

- HTTP 왕복 지연 중앙값 542.4ms, 범위 491.3–672.7ms. 12회뿐인 표본의 nearest-rank p95=672.7ms로 운영 지연 보장은 아니다.
- 입력 19,662 / 출력 668 tokens. 공급사 공개 입력 단가 $0.042/백만 기준 계산값 약 $0.000826; 청구 내역을 확인한 금액은 아니다.
- 두 반복의 최종 선택 일치 6/6. 분포와 confidence는 달라졌으며 결정론이나 광범위한 안정성의 증거가 아니다. confidence 범위 0.43–0.58; 임계값으로 실행을 허가하지 않았다.
- 규칙과 10/12 일치. seed19 open-plus 100에서 규칙은 continue, Jev는 두 번 모두 reobserve. 이는 완료 점수 변화와 영상/명령 이력이 있는 경계 사례지만 Jev가 왜 선택했는지 설명 출력은 없다. 재관측이 실제 성과를 개선하는지는 확인되지 않았다.
- 복구 요청 선택 0건. 이 입력·후보에서 복구 전환 능력은 입증되지 않았다.
- 같은 상태를 LLM에 보내는 비교는 이번 파일럿에서 실행하지 않았다. Jev의 LLM 대비 정확도/지연/비용 우위는 미검증.
- 전체 영상에서 목표/파트너 관계를 재인식하는 대신 기존 ACT/영상 보호 장치 추정과 단순 픽셀 변화만 제공했다. 모델이 기존 추정 오류를 독립적으로 바로잡을 정보는 제한적이다.
- 원 요청·응답·모델·토큰·출처·해시는 records/ 및 summary.json에 보존. 원본 RGB/전체 운반 로그는 기존 로컬 위치에만 있으며 이 PR은 영상 백업이 아니다.
- 입력/출력에서 API 키 형태가 남지 않았음을 검사했다. 키는 getpass의 숨김 입력으로 전달했고 파일·Git에 기록하지 않았다.

## 다음 검증

동일 텍스트 상태의 구조화 출력 LLM 비교, 영상에서 의미 있는 진행/목표 관계를 추출하는 관측 모듈 검증, 이후 같은 실행기로 하는 폐루프 비교가 필요하다. 이번 API 파일럿을 로봇 복구 성공으로 해석하지 않는다.

## 코드 검증

- 관련 입력 격리/이미지 변조/잘못된 API 출력 테스트 2개 통과.
- 프로젝트 전체 offline suite: 990 passed, 1 skipped, 184 subtests passed (57.09s).
- 실제 API 12회와 오프라인 테스트는 별도 증거다.
