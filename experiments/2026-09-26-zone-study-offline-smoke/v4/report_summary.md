# 한국어 로봇 대화 효율 연구 — 평가 요약

- 생성: 2026-09-26T16:10:33+0900 (평가 전용 사후 분석)
- 시행 수: 30, 시나리오: s1_normal_mixed, s2_unmapped_blockage, s3_late_rendezvous, s4_narrow_door_standoff, s5_moved_dropped_item, s6_novel_relation
- 실패 가중치(PAR): 비성공 시행에 horizon × 2.0를 부과한다. 빠른 실패가 느린 성공보다 좋게 보이지 않는다.
- 선행 발화 탐색 창: 30.0초 (연관이며 인과가 아니다)
- 실행 번들 단일 여부: 아니오 — 혼재 필드: map_file_sha256, order_sheet_sha256, public_map_sha256

이 문서는 저장된 평가 로그만 읽는다. 시뮬레이션·모델 호출을 하지 않았고, 정답·TOP·심판 판정은 로봇 입력으로 되돌리지 않는다.

## 1. 조건별 효율

| 조건 | 시행 | 성공 | 성공률 | PAR makespan(SIM초) | 성공 makespan | 배송률 | 발화 비용(초) | idle(로봇초) | 충돌 | 교착 | 재계획 | 호출 | 토큰 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| ① 무통신 | 6 | 0 | 0.000 | 600.0 | — | 0.000 | 0.0 | 232.9 | 0 | 0 | 0 | 68.5 | 545304 |
| ② 자유 한국어 동료 대화 | 6 | 0 | 0.000 | 600.0 | — | 0.000 | 1.2 | 315.9 | 0 | 0 | 0 | 90 | 741258 |
| ③ 한국어 지휘 겸임 | 6 | 0 | 0.000 | 600.0 | — | 0.000 | 1.6 | 316.6 | 0 | 0 | 0 | 90 | 736158 |
| ④ 정형 메시지 대조 | 6 | 0 | 0.000 | 600.0 | — | 0.000 | 1.2 | 319.8 | 0 | 0 | 0 | 90 | 750654 |
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
| ③ 한국어 지휘 겸임 | ④ 정형 메시지 대조 | 6 | 0 | 316.650 | 319.800 | 3.150 | [0.567, 5.733] | 0.785 | 0.429 | 3/6 |
| ① 무통신 | ③ 한국어 지휘 겸임 | 6 | 0 | 232.950 | 316.650 | 83.700 | [79.533, 88.167] | 13.730 | 1.000 | 6/6 |
| ① 무통신 | ② 자유 한국어 동료 대화 | 6 | 0 | 232.950 | 315.850 | 82.900 | [78.700, 87.300] | 13.676 | 1.000 | 6/6 |
| ① 무통신 | ④ 정형 메시지 대조 | 6 | 0 | 232.950 | 319.800 | 86.850 | [81.350, 92.850] | 10.583 | 1.000 | 6/6 |
| ② 자유 한국어 동료 대화 | ③ 한국어 지휘 겸임 | 6 | 0 | 315.850 | 316.650 | 0.800 | [0.733, 0.867] | 7.303 | 1.000 | 6/6 |
| ② 자유 한국어 동료 대화 | ④ 정형 메시지 대조 | 6 | 0 | 315.850 | 319.800 | 3.950 | [1.300, 6.600] | 0.958 | 1.000 | 4/6 |

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
| ③ 한국어 지휘 겸임 | ④ 정형 메시지 대조 | 6 | 0 | 90.000 | 90.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ① 무통신 | ③ 한국어 지휘 겸임 | 6 | 0 | 68.500 | 90.000 | 21.500 | [21.000, 22.500] | 17.555 | 1.000 | 6/6 |
| ① 무통신 | ② 자유 한국어 동료 대화 | 6 | 0 | 68.500 | 90.000 | 21.500 | [21.000, 22.500] | 17.555 | 1.000 | 6/6 |
| ① 무통신 | ④ 정형 메시지 대조 | 6 | 0 | 68.500 | 90.000 | 21.500 | [21.000, 22.500] | 17.555 | 1.000 | 6/6 |
| ② 자유 한국어 동료 대화 | ③ 한국어 지휘 겸임 | 6 | 0 | 90.000 | 90.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |
| ② 자유 한국어 동료 대화 | ④ 정형 메시지 대조 | 6 | 0 | 90.000 | 90.000 | 0.000 | [0.000, 0.000] | — | — | 0/6 |

**tokens_total**

| 기준 | 비교 | 짝 | 제외 짝 | 기준 평균 | 비교 평균 | 차이 | 95% 부트스트랩 구간 | dz | rank-biserial | 비교>기준 |
|---|---|---:|---:|---:|---:|---:|---|---:|---:|---:|
| ③ 한국어 지휘 겸임 | ④ 정형 메시지 대조 | 6 | 0 | 736158.000 | 750654.000 | 14496.000 | [14496.000, 14496.000] | — | 1.000 | 6/6 |
| ① 무통신 | ③ 한국어 지휘 겸임 | 6 | 0 | 545303.500 | 736158.000 | 190854.500 | [177870.000, 210495.500] | 7.916 | 1.000 | 6/6 |
| ① 무통신 | ② 자유 한국어 동료 대화 | 6 | 0 | 545303.500 | 741258.000 | 195954.500 | [182970.000, 215595.500] | 8.127 | 1.000 | 6/6 |
| ① 무통신 | ④ 정형 메시지 대조 | 6 | 0 | 545303.500 | 750654.000 | 205350.500 | [192366.000, 224991.500] | 8.517 | 1.000 | 6/6 |
| ② 자유 한국어 동료 대화 | ③ 한국어 지휘 겸임 | 6 | 0 | 741258.000 | 736158.000 | -5100.000 | [-5100.000, -5100.000] | — | -1.000 | 0/6 |
| ② 자유 한국어 동료 대화 | ④ 정형 메시지 대조 | 6 | 0 | 741258.000 | 750654.000 | 9396.000 | [9396.000, 9396.000] | — | 1.000 | 6/6 |

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
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/leader_ko-s1_normal_mixed-s601.json` | `52a1bdd6fa0b070390bf961eb0e799cab02a56acc8460cff25ec8a2535348400` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/leader_ko-s2_unmapped_blockage-s611.json` | `986eae3fad6fcfe1bafe721c13d35899f364c16c2b13bfd15d241d6796a40326` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/leader_ko-s3_late_rendezvous-s621.json` | `e8a14f8bff51c03929f1a84725ea2fdfc373cc0c886252e822ca1f87d7709afd` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/leader_ko-s4_narrow_door_standoff-s631.json` | `0474186c84a66f7feeb79c33f8af49140d56bb00d79b1d796527b756c32e89fe` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/leader_ko-s5_moved_dropped_item-s641.json` | `a885b38aae01b875a0e2fd114b57997122397fe43a7c728da0bfa3894d123973` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/leader_ko-s6_novel_relation-s651.json` | `caf243e820da7306425a6b3b963a3abdc797f7b6302a966745237c61be38cb94` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/no_comm-s1_normal_mixed-s601.json` | `8d37c9baef3b59e7575f96f99c6f859e1b7b9d307213e37fecdea37e3281bd5d` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/no_comm-s2_unmapped_blockage-s611.json` | `ad4e1c76e4b692d67413a1c8993eac7bcab6b9eed1a6d92190beadedaa24a6c3` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/no_comm-s3_late_rendezvous-s621.json` | `84fe3342ce09cc7688c68c11be78d6ac434e454bb2da07b91b716a0488d48fa1` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/no_comm-s4_narrow_door_standoff-s631.json` | `a5881940863d623aa53d31924dcabb199cbd3cefa9e29b0970d050d336949dcb` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/no_comm-s5_moved_dropped_item-s641.json` | `49f8f916348beb174d202f7d10e501f8b4ee33bc40ce14684ae9e892056dd57b` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/no_comm-s6_novel_relation-s651.json` | `87324434421b3b845a3eb2d30281d000a13724469a822addbe9612a0d8f4bde7` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/peer_ko-s1_normal_mixed-s601.json` | `aad9cfb9c6cd7db78cf0b90282c57494d3e4eca17ee14d119402adde1b15eb82` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/peer_ko-s2_unmapped_blockage-s611.json` | `97a1812ccdb2d5ed6115fe85d4d0d957019db17c28db6f1ada0ff4da06fd9d79` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/peer_ko-s3_late_rendezvous-s621.json` | `050d11b33511621bac189a0da36a0f0cc34bf0b2be6f4d30378d05051f6c1f71` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/peer_ko-s4_narrow_door_standoff-s631.json` | `5d9144c89c0c9e0745abcffd0d1262ed2ba5ac67eebc474bda48281ff0bdb1e6` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/peer_ko-s5_moved_dropped_item-s641.json` | `688292efc759cbf13bb01eb571cb6717132b3691cbfba963aa85ae618131a830` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/peer_ko-s6_novel_relation-s651.json` | `af24a28c0474c6684da60ac00105dd6f697cc347724040336ad73ec9749d17f5` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/reference_R-s1_normal_mixed-s601.json` | `7092f7b658defc87ca7b805542e23881269f7a5653788d69ca70fb0e7090938a` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/reference_R-s2_unmapped_blockage-s611.json` | `ea2c46c1641193f94faa4e30abb4d2dd5f2e58ce04eb9ae6b85a68805dcaf5dd` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/reference_R-s3_late_rendezvous-s621.json` | `b0a5e45c8fdb3d60182c91a170149ed75e815a6ec4cb12b9344619160400c50d` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/reference_R-s4_narrow_door_standoff-s631.json` | `f6af7165f7b0d8a041b76d1527df7c3a3cd1a2675f0c38a7ecffd56ac4912ea4` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/reference_R-s5_moved_dropped_item-s641.json` | `034b2ea170b652eaeec63d21667ab6260e90637f805f412ad4685ffa03db758c` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/reference_R-s6_novel_relation-s651.json` | `42828b27730d81b6133e2a98cb1f9f187c8a37b16617a76b0f5ec759f21ab135` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/structured-s1_normal_mixed-s601.json` | `653eaef57b0ccc61a36d401136dcdbbc2c0cceea3b2cc84b661e5c85f4b86d25` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/structured-s2_unmapped_blockage-s611.json` | `11398b778a3d7915f0c6d828981409c41021564cffc1669c1fc74a8b84a476ba` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/structured-s3_late_rendezvous-s621.json` | `c96c37ebb71d618837062496d03259a0069c95652add83414b5e8c38fb49ea3f` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/structured-s4_narrow_door_standoff-s631.json` | `5b6121660128b8573e02c4c1eea8c3ffff29e9653c91f69826316755728a69dd` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/structured-s5_moved_dropped_item-s641.json` | `10663dee08c5de71cd77d4759f259e0c762c8b37c03d300855612600e3ff85d0` |
| `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/trial_records/structured-s6_novel_relation-s651.json` | `497eda736bd088818dece056c11e5e50e2463e3f121d1667469d974388eb3523` |

## 6. 남은 검증

- 이 보고서는 지표 계산의 정확성만 확인한다. 조건 간 우열은 사전 고정한 코호트를 실제로 실행한 뒤에만 주장한다.
- 로그 schema는 Package A(`harness/zone_study_contract.py`)가 소유한다. `ugrp.zone_study_trial.v1`는 A의 호출·메시지·행동 기록을 담고 A가 검증한다. 과거 파일럿 로그를 위해 `ugrp.zone_study_trial.provisional.v1`도 계속 읽는다.
