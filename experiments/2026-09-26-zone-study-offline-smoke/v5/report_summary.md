# 한국어 로봇 대화 효율 연구 — 평가 요약

- 생성: 2026-09-26T21:14:25+0900 (평가 전용 사후 분석)
- 시행 수: 30, 시나리오: s1_normal_mixed, s2_unmapped_blockage, s3_late_rendezvous, s4_narrow_door_standoff, s5_moved_dropped_item, s6_novel_relation
- 실패 가중치(PAR): 비성공 시행에 horizon × 2.0를 부과한다. 빠른 실패가 느린 성공보다 좋게 보이지 않는다.
- 선행 발화 탐색 창: 30.0초 (연관이며 인과가 아니다)
- 실행 번들 단일 여부: 아니오 — 혼재 필드: map_file_sha256, order_sheet_sha256, public_map_sha256

이 문서는 저장된 평가 로그만 읽는다. 시뮬레이션·모델 호출을 하지 않았고, 정답·TOP·심판 판정은 로봇 입력으로 되돌리지 않는다.

## 1. 조건별 효율

| 조건 | 시행 | 성공 | 성공률 | PAR makespan(SIM초) | 성공 makespan | 배송률 | 발화 비용(초) | idle(로봇초) | 충돌 | 교착 | 재계획 | 호출 | 토큰 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| ① 무통신 | 6 | 0 | 0.000 | 600.0 | — | 0.000 | 0.0 | 232.9 | 0 | 0 | 0 | 68.5 | 545304 |
| ② 자유 한국어 동료 대화 | 6 | 0 | 0.000 | 600.0 | — | 0.000 | 1.2 | 258.2 | 0 | 0 | 0 | 74.5 | 612578 |
| ③ 한국어 지휘 겸임 | 6 | 0 | 0.000 | 600.0 | — | 0.000 | 1.6 | 248.6 | 0 | 0 | 0 | 71.5 | 583785 |
| ④ 정형 메시지 대조 | 6 | 0 | 0.000 | 600.0 | — | 0.000 | 1.2 | 259.6 | 0 | 0 | 0 | 74.5 | 620300 |
| R 전지적 지휘 참조 상한 | 6 | 0 | 0.000 | 600.0 | — | 0.000 | 0.0 | 0.0 | 0 | 0 | 0 | 22.8 | 179913 |

PAR makespan은 실패·중단·시간 초과·예산 소진을 분모에 유지한 값이다. 성공 makespan 열은 성공한 시행만의 평균이므로 단독으로 조건을 비교하지 않는다.

## 2. 조건별 대화 지표

| 조건 | 발화 | 전달 edge | 한국어 준수 | 침묵 | 코드전환 | ID 손상 | 사실 | 거짓 | 확인불가 | 사실성 | 채널 위반 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| ① 무통신 | 0 | 0 | — | 0 | 0 | 0 | 0 | 0 | 0 | — | 0 |
| ② 자유 한국어 동료 대화 | 18 | 36 | 1.000 | 0 | 0 | 0 | 0 | 0 | 0 | — | 0 |
| ③ 한국어 지휘 겸임 | 24 | 24 | 1.000 | 0 | 0 | 0 | 0 | 0 | 0 | — | 0 |
| ④ 정형 메시지 대조 | 18 | 36 | — | 0 | 0 | 0 | 0 | 0 | 0 | — | 0 |
| R 전지적 지휘 참조 상한 | 0 | 0 | — | 0 | 0 | 0 | 0 | 0 | 0 | — | 0 |

한국어 준수는 literal ID·enum을 제외한 한글 비율이 0.9 이상인 발화의 비율이다. 침묵은 준수 성공으로 세지 않는다. 사실성은 발화 시각의 평가 로그와 대조한 결과이며 확인불가는 실패로 바꾸지 않는다.

### 행위 유형

| 조건 | order | report | request |
|---|---:|---:|---:|
| ① 무통신 | 0 | 0 | 0 |
| ② 자유 한국어 동료 대화 | 0 | 18 | 0 |
| ③ 한국어 지휘 겸임 | 12 | 24 | 0 |
| ④ 정형 메시지 대조 | 0 | 0 | 18 |
| R 전지적 지휘 참조 상한 | 0 | 0 | 0 |

PR 172(병합)의 세부 규칙 라벨을 재사용하고 지휘 조건용 `order`만 추가했다. 대응 규칙은 `docs/zone_study_metrics.md`에 있다.

### 결정 변경 직전 발화

| 조건 | 결정 변경 | 직전 수신 발화 있음 | 비율 | 선행 발화 행위 |
|---|---:|---:|---:|---|
| ① 무통신 | 0 | 0 | — | — |
| ② 자유 한국어 동료 대화 | 0 | 0 | — | — |
| ③ 한국어 지휘 겸임 | 0 | 0 | — | — |
| ④ 정형 메시지 대조 | 0 | 0 | — | — |
| R 전지적 지휘 참조 상한 | 0 | 0 | — | — |

## 3. 짝지은 seed 비교


**par_makespan_sim_s**

| 기준 | 비교 | 짝 | 제외 짝 | 기준 평균 | 비교 평균 | 차이 | 95% 부트스트랩 구간 | dz | rank-biserial | 비교>기준 |
|---|---|---:|---:|---:|---:|---:|---|---:|---:|---:|
| ③ 한국어 지휘 겸임 | ④ 정형 메시지 대조 | 6 | 0 | 600.000 | 600.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ① 무통신 | ③ 한국어 지휘 겸임 | 6 | 0 | 600.000 | 600.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ① 무통신 | ② 자유 한국어 동료 대화 | 6 | 0 | 600.000 | 600.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ① 무통신 | ④ 정형 메시지 대조 | 6 | 0 | 600.000 | 600.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ② 자유 한국어 동료 대화 | ③ 한국어 지휘 겸임 | 6 | 0 | 600.000 | 600.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ② 자유 한국어 동료 대화 | ④ 정형 메시지 대조 | 6 | 0 | 600.000 | 600.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |

**success**

| 기준 | 비교 | 짝 | 제외 짝 | 기준 평균 | 비교 평균 | 차이 | 95% 부트스트랩 구간 | dz | rank-biserial | 비교>기준 |
|---|---|---:|---:|---:|---:|---:|---|---:|---:|---:|
| ③ 한국어 지휘 겸임 | ④ 정형 메시지 대조 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ① 무통신 | ③ 한국어 지휘 겸임 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ① 무통신 | ② 자유 한국어 동료 대화 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ① 무통신 | ④ 정형 메시지 대조 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ② 자유 한국어 동료 대화 | ③ 한국어 지휘 겸임 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ② 자유 한국어 동료 대화 | ④ 정형 메시지 대조 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |

**delivery_rate**

| 기준 | 비교 | 짝 | 제외 짝 | 기준 평균 | 비교 평균 | 차이 | 95% 부트스트랩 구간 | dz | rank-biserial | 비교>기준 |
|---|---|---:|---:|---:|---:|---:|---|---:|---:|---:|
| ③ 한국어 지휘 겸임 | ④ 정형 메시지 대조 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ① 무통신 | ③ 한국어 지휘 겸임 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ① 무통신 | ② 자유 한국어 동료 대화 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ① 무통신 | ④ 정형 메시지 대조 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ② 자유 한국어 동료 대화 | ③ 한국어 지휘 겸임 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ② 자유 한국어 동료 대화 | ④ 정형 메시지 대조 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |

**idle_robot_s**

| 기준 | 비교 | 짝 | 제외 짝 | 기준 평균 | 비교 평균 | 차이 | 95% 부트스트랩 구간 | dz | rank-biserial | 비교>기준 |
|---|---|---:|---:|---:|---:|---:|---|---:|---:|---:|
| ③ 한국어 지휘 겸임 | ④ 정형 메시지 대조 | 6 | 0 | 248.600 | 259.600 | 11.000 | [8.283, 13.800] | 2.763 | 1.000 | 6/6 |
| ① 무통신 | ③ 한국어 지휘 겸임 | 6 | 0 | 232.950 | 248.600 | 15.650 | [12.833, 18.467] | 3.825 | 1.000 | 6/6 |
| ① 무통신 | ② 자유 한국어 동료 대화 | 6 | 0 | 232.950 | 258.250 | 25.300 | [22.400, 28.100] | 6.323 | 1.000 | 6/6 |
| ① 무통신 | ④ 정형 메시지 대조 | 6 | 0 | 232.950 | 259.600 | 26.650 | [23.000, 30.300] | 4.889 | 1.000 | 6/6 |
| ② 자유 한국어 동료 대화 | ③ 한국어 지휘 겸임 | 6 | 0 | 258.250 | 248.600 | -9.650 | [-9.900, -9.450] | -30.670 | -1.000 | 0/6 |
| ② 자유 한국어 동료 대화 | ④ 정형 메시지 대조 | 6 | 0 | 258.250 | 259.600 | 1.350 | [-1.200, 4.000] | 0.361 | 0.600 | 3/6 |

**conflicts**

| 기준 | 비교 | 짝 | 제외 짝 | 기준 평균 | 비교 평균 | 차이 | 95% 부트스트랩 구간 | dz | rank-biserial | 비교>기준 |
|---|---|---:|---:|---:|---:|---:|---|---:|---:|---:|
| ③ 한국어 지휘 겸임 | ④ 정형 메시지 대조 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ① 무통신 | ③ 한국어 지휘 겸임 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ① 무통신 | ② 자유 한국어 동료 대화 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ① 무통신 | ④ 정형 메시지 대조 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ② 자유 한국어 동료 대화 | ③ 한국어 지휘 겸임 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ② 자유 한국어 동료 대화 | ④ 정형 메시지 대조 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |

**deadlocks**

| 기준 | 비교 | 짝 | 제외 짝 | 기준 평균 | 비교 평균 | 차이 | 95% 부트스트랩 구간 | dz | rank-biserial | 비교>기준 |
|---|---|---:|---:|---:|---:|---:|---|---:|---:|---:|
| ③ 한국어 지휘 겸임 | ④ 정형 메시지 대조 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ① 무통신 | ③ 한국어 지휘 겸임 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ① 무통신 | ② 자유 한국어 동료 대화 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ① 무통신 | ④ 정형 메시지 대조 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ② 자유 한국어 동료 대화 | ③ 한국어 지휘 겸임 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ② 자유 한국어 동료 대화 | ④ 정형 메시지 대조 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |

**replans**

| 기준 | 비교 | 짝 | 제외 짝 | 기준 평균 | 비교 평균 | 차이 | 95% 부트스트랩 구간 | dz | rank-biserial | 비교>기준 |
|---|---|---:|---:|---:|---:|---:|---|---:|---:|---:|
| ③ 한국어 지휘 겸임 | ④ 정형 메시지 대조 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ① 무통신 | ③ 한국어 지휘 겸임 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ① 무통신 | ② 자유 한국어 동료 대화 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ① 무통신 | ④ 정형 메시지 대조 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ② 자유 한국어 동료 대화 | ③ 한국어 지휘 겸임 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ② 자유 한국어 동료 대화 | ④ 정형 메시지 대조 | 6 | 0 | 0.000 | 0.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |

**model_calls**

| 기준 | 비교 | 짝 | 제외 짝 | 기준 평균 | 비교 평균 | 차이 | 95% 부트스트랩 구간 | dz | rank-biserial | 비교>기준 |
|---|---|---:|---:|---:|---:|---:|---|---:|---:|---:|
| ③ 한국어 지휘 겸임 | ④ 정형 메시지 대조 | 6 | 0 | 71.500 | 74.500 | 3.000 | [3.000, 3.000] | — | 1.000 | 6/6 |
| ① 무통신 | ③ 한국어 지휘 겸임 | 6 | 0 | 68.500 | 71.500 | 3.000 | [3.000, 3.000] | — | 1.000 | 6/6 |
| ① 무통신 | ② 자유 한국어 동료 대화 | 6 | 0 | 68.500 | 74.500 | 6.000 | [6.000, 6.000] | — | 1.000 | 6/6 |
| ① 무통신 | ④ 정형 메시지 대조 | 6 | 0 | 68.500 | 74.500 | 6.000 | [6.000, 6.000] | — | 1.000 | 6/6 |
| ② 자유 한국어 동료 대화 | ③ 한국어 지휘 겸임 | 6 | 0 | 74.500 | 71.500 | -3.000 | [-3.000, -3.000] | — | -1.000 | 0/6 |
| ② 자유 한국어 동료 대화 | ④ 정형 메시지 대조 | 6 | 0 | 74.500 | 74.500 | 0.000 | [0.000, 0.000] | — | — | 0/6 |

**tokens_total**

| 기준 | 비교 | 짝 | 제외 짝 | 기준 평균 | 비교 평균 | 차이 | 95% 부트스트랩 구간 | dz | rank-biserial | 비교>기준 |
|---|---|---:|---:|---:|---:|---:|---|---:|---:|---:|
| ③ 한국어 지휘 겸임 | ④ 정형 메시지 대조 | 6 | 0 | 583785.167 | 620300.500 | 36515.333 | [35400.000, 37865.500] | 21.038 | 1.000 | 6/6 |
| ① 무통신 | ③ 한국어 지휘 겸임 | 6 | 0 | 545303.500 | 583785.167 | 38481.667 | [37386.000, 39792.500] | 22.728 | 1.000 | 6/6 |
| ① 무통신 | ② 자유 한국어 동료 대화 | 6 | 0 | 545303.500 | 612578.500 | 67275.000 | [65010.000, 70044.000] | 18.972 | 1.000 | 6/6 |
| ① 무통신 | ④ 정형 메시지 대조 | 6 | 0 | 545303.500 | 620300.500 | 74997.000 | [72786.000, 77658.000] | 21.873 | 1.000 | 6/6 |
| ② 자유 한국어 동료 대화 | ③ 한국어 지휘 겸임 | 6 | 0 | 612578.500 | 583785.167 | -28793.333 | [-30228.000, -27624.000] | -15.532 | -1.000 | 0/6 |
| ② 자유 한국어 동료 대화 | ④ 정형 메시지 대조 | 6 | 0 | 612578.500 | 620300.500 | 7722.000 | [7614.000, 7776.000] | 58.380 | 1.000 | 6/6 |

**talk_sim_cost_s**

| 기준 | 비교 | 짝 | 제외 짝 | 기준 평균 | 비교 평균 | 차이 | 95% 부트스트랩 구간 | dz | rank-biserial | 비교>기준 |
|---|---|---:|---:|---:|---:|---:|---|---:|---:|---:|
| ③ 한국어 지휘 겸임 | ④ 정형 메시지 대조 | 6 | 0 | 1.600 | 1.200 | -0.400 | [-0.400, -0.400] | — | -1.000 | 0/6 |
| ① 무통신 | ③ 한국어 지휘 겸임 | 6 | 0 | 0.000 | 1.600 | 1.600 | [1.600, 1.600] | — | 1.000 | 6/6 |
| ① 무통신 | ② 자유 한국어 동료 대화 | 6 | 0 | 0.000 | 1.200 | 1.200 | [1.200, 1.200] | — | 1.000 | 6/6 |
| ① 무통신 | ④ 정형 메시지 대조 | 6 | 0 | 0.000 | 1.200 | 1.200 | [1.200, 1.200] | — | 1.000 | 6/6 |
| ② 자유 한국어 동료 대화 | ③ 한국어 지휘 겸임 | 6 | 0 | 1.200 | 1.600 | 0.400 | [0.400, 0.400] | — | 1.000 | 6/6 |
| ② 자유 한국어 동료 대화 | ④ 정형 메시지 대조 | 6 | 0 | 1.200 | 1.200 | 0.000 | [0.000, 0.000] | — | — | 0/6 |

짝 수가 10개 미만인 비교가 있다(짝 [6]). 유의성 검정을 하지 않고 구간과 짝별 표만 읽는다. 차이 부호는 `비교 − 기준`이며 짝별 값은 `metrics.json`에 있다.

## 4. 입력 경계 감사

주 4조건의 모든 시행에서 금지 입력 key·미검증 payload·알 수 없는 입력 key·금지 근거 인용·채널 위반이 없었다.

시행 상태 집계: clean 24, unverified 6

R 조건 6개 시행은 설계상 전지적 참조 상한이며 추가 입력(—)을 받는다. 주 조건 경계 판정에 합산하지 않는다.

## 5. 원본

| 파일 | SHA-256 |
|---|---|
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/leader_ko-s1_normal_mixed-s601.json` | `cedb9f2ad61360412f0c2cc3fe6de5f53c7ee551ac3dfbb38e5a52900f04bfd5` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/leader_ko-s2_unmapped_blockage-s611.json` | `388eb344e8e06b064a00bca0006903cd1d6faa7d7cc2717af178e7d5beb7de42` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/leader_ko-s3_late_rendezvous-s621.json` | `7c0e86d7bf30f7f4efdc2eb49417cb1f650b1759efd527d8b2728116921763f3` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/leader_ko-s4_narrow_door_standoff-s631.json` | `187cd631f4db2f016e46d9bb499c2ce414747bd7636021a1a0c0d17233b8b1e2` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/leader_ko-s5_moved_dropped_item-s641.json` | `b9011b3a51a136544c41e4620efde7d2ceba9b079f84c22808a37ce5d4d52d96` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/leader_ko-s6_novel_relation-s651.json` | `9bfdfe695afa70b64f038fb675d45b1f97862383be05ce9ac259736e1679b771` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/no_comm-s1_normal_mixed-s601.json` | `b1c79f2d3f5dc452560f5d17f940ce167d377bbf6b94eec734f61d618f065243` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/no_comm-s2_unmapped_blockage-s611.json` | `95ff23a1417dded9bb48c2f71c0c55896c20a1c17da40709f8e553fd522aa798` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/no_comm-s3_late_rendezvous-s621.json` | `9b7af0f891c4b90cda5da5818e05589bd88fe423660684480c0c2bdabebb68eb` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/no_comm-s4_narrow_door_standoff-s631.json` | `fe7483576abd2f6a6148e1b8de924228e0bcfe33751b5206dc25836308a2a3c4` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/no_comm-s5_moved_dropped_item-s641.json` | `f9d97c2c06b2374e9490d6dee3042409d7bc6494e1740c8dcc681b6c4e87271b` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/no_comm-s6_novel_relation-s651.json` | `55300f73ad64552474ee16e6080ab2a18c3f8d88a0a2f51893d77dc38478962e` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/peer_ko-s1_normal_mixed-s601.json` | `ecde15964bd035ab0ffaee1cc5fa57efdf5894c9932fa33fc5cc42222493b12a` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/peer_ko-s2_unmapped_blockage-s611.json` | `929eef32dfd205c843d07e9124bc9425c2632144539ba3c77ce2e32db8db5519` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/peer_ko-s3_late_rendezvous-s621.json` | `bd0baef8f66d402585622b8b8739ef1c089fadd33d0a36d8faad1d08ca8c644a` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/peer_ko-s4_narrow_door_standoff-s631.json` | `2a4c9247cc30488dada8f6aa684a714ee0277667af774d3b9963849aa658bca3` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/peer_ko-s5_moved_dropped_item-s641.json` | `842584fb3e0c2b230ac0923db274e486b44c29540b00de54d1c3c39ac0e43004` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/peer_ko-s6_novel_relation-s651.json` | `7dee4073a0fa8fbe061aa53b6782cdfd7fa8c6812f0c7ad7d098b79335130090` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/reference_R-s1_normal_mixed-s601.json` | `79c211df9780cfae803ffe7056e7f4d5d63c15932b1b2bab2f33ff21cc60c716` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/reference_R-s2_unmapped_blockage-s611.json` | `e6610680548bd020aa63235589411d460f65cbadc6b60aa0047733e1f474bd51` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/reference_R-s3_late_rendezvous-s621.json` | `03aabd038aaf96e188a49434cb764687138a75958478c883d4dfc954d7896867` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/reference_R-s4_narrow_door_standoff-s631.json` | `173bd7aa4add788aa84da057cddaee2c1e85c745cbc8ca69f9203432466b8b78` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/reference_R-s5_moved_dropped_item-s641.json` | `67f69b061133816b88e795102046e00a64b2e85a1434009a3542a92eb8331eca` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/reference_R-s6_novel_relation-s651.json` | `9c9a5ef76f336ec10bae10fda48530c93046a9f7b2d05dc18e1f5e91f3ea0476` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/structured-s1_normal_mixed-s601.json` | `74e82b64cb7f31c6db19fbfcf3d6f22fdcb59d834ca57b8fe918d9693c3a28f1` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/structured-s2_unmapped_blockage-s611.json` | `655c71ab200bf660dd1c4512115b968a033aa2fbc9b71777a297cb354f3434a2` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/structured-s3_late_rendezvous-s621.json` | `b616a976f8eca881be0875af79485f936b56c813034c686bbe407fa3c5576730` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/structured-s4_narrow_door_standoff-s631.json` | `26dd98379579cabd34b66cf0bd17a8b81f7ca0f757ff57224a2d3d32cb48d024` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/structured-s5_moved_dropped_item-s641.json` | `f3149e5bab81c3fcfc7efe91772c33465f22829f63d38518fa856b0977f841b2` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/trial_records/structured-s6_novel_relation-s651.json` | `35058f61eb4192ef94b865af411fb28124a8d2b9483abd625243de40dc7fa54e` |

## 6. 남은 검증

- 이 보고서는 지표 계산의 정확성만 확인한다. 조건 간 우열은 사전 고정한 코호트를 실제로 실행한 뒤에만 주장한다.
- 로그 schema는 Package A(`harness/zone_study_contract.py`)가 소유한다. `ugrp.zone_study_trial.v1`는 A의 호출·메시지·행동 기록을 담고 A가 검증한다. 과거 파일럿 로그를 위해 `ugrp.zone_study_trial.provisional.v1`도 계속 읽는다.
