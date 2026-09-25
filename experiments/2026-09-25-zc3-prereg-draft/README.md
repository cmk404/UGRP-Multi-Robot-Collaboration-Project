# ZC3 본비교 사전 등록 초안과 실행 예산 (2026-09-25)

> **초안이며 고정되지 않았다.** 이 문서는 사전 등록이 아니다. 사용자가 예산 단계와 아래 "결정할 것"을 고른 뒤 최종 문서를 커밋해야 사전 등록이 된다. 이번 작업에서는 SIM과 실LLM을 한 번도 실행하지 않았다. 모든 수치는 이미 기록된 원본에서 계산했다.

- 관련 TODO: R0(주 결과와 의미 있는 차이의 수치 고정), R4(파일럿으로 예산 결정), R5(고정한 새 조건에서 본실험), R6(효과와 비용 분석). [연구 TODO](../../docs/research_todo.md)
- 근거 기록: [ZC1/ZC2 구역 통신 비교](../2026-09-25-zone-communication/README.md), [넓은 경기장 ZW1/ZW2](../2026-09-25-zone-wide-arena/README.md), [구역 배송 Z1–Z3](../2026-09-25-zone-dispatch/README.md)
- 분석 스크립트: [`analysis/zc3_prereg_analysis.py`](analysis/zc3_prereg_analysis.py). 산출물은 [`analysis/measured_runs.json`](analysis/measured_runs.json), [`analysis/power.json`](analysis/power.json), [`analysis/seed_plan.json`](analysis/seed_plan.json), [`budget.json`](budget.json)이다.

## 0. 주장 범위

ZC3도 ZC2와 마찬가지로 **교사 실행기 조건**이다. 이동과 IK는 정답 좌표를 쓰고, 잡기는 실제 집게로 한다(weld OFF). 그래서 ZC3의 결론은 "교사가 움직임을 맡았을 때 협업 방식(무통신 / 사전 합의 / 동적 대화)이 목표 달성과 비용에 주는 차이"로 한정된다. RGB 스킬 성공이나 학생 정책 성공, 실물 성능으로 보고하지 않는다.

조건 사이에는 메시지 채널 말고도 차이가 있다. ZC2 구현 기준으로 적는다.
- independent는 팀 게시판과 호스트 중재가 없다. 10 SIM초마다 다시 묻는다.
- dynamic에는 호스트 선언 검사(두 번 세기 수정 `9412326` 포함)와 "겹치면 가장 작은 id가 가져감" 프롬프트 규칙이 있다.
- plan_first는 실패 뒤 다시 계획하지 않는다.

따라서 ZC3가 추정하는 것은 **"통신 방식 묶음(채널 + 호스트 규칙 + 프롬프트 규칙)"의 효과**다. 메시지 하나의 인과 효과가 아니다(R6 마지막 항목의 분기 복원은 범위 밖).

## 1. 측정한 실행 비용 (ZC1/ZC2 원본)

원본은 기본 체크아웃 `outputs/zone-communication-20260925/`에 있다(로컬 전용이며 원격 백업이 아니다). 스크립트가 원본 `result.json`, `team/team.json`, `teacher-events.json`, `loads.jsonl`을 다시 읽어 표를 만든다. 커밋된 ZC2 `results.json`과 성공, 과잉/부족, 완료 SIM초, 호출, 입력/출력 토큰을 대조했고 **불일치는 0건**이다.

용어:
- 제어 종료 SIM초: `sim_end_s`
- 완료 SIM초: 첫 배정부터 마지막 작업까지
- wall초: 에피소드 루프 시간(모델 지연 포함)
- 드라이버 wall초: `loads.jsonl`의 시작→끝(장면 생성과 재생 기록 포함)
- 호출당 지연: `team.json`의 `latency_ms` 평균

### ZC2 실LLM 18회 (`7ccd6c3`, zone_wide, G8 + 여분 빨강1·청록1, `gemini-3.8-flash`)

| 실행 | 성공 | 실패 유형 | 제어 종료 SIM초 | 완료 SIM초 | 호출 | 입력 토큰 | 출력 토큰 | wall초 | 드라이버 wall초 | 호출당 지연 s |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| s12 independent nominal | ✗ | 과잉 배달 | 221.8 | 220.0 | 23 | 146,389 | 1,913 | 221.5 | 222 | 5.19 |
| s12 plan_first nominal | ✓ | — | 164.8 | 163.0 | 13 | 86,420 | 2,864 | 112.7 | 113 | 5.32 |
| s12 dynamic nominal | ✓ | — | 154.8 | 153.0 | 12 | 79,581 | 1,119 | 108.5 | 109 | 4.69 |
| s12 independent graspfail | ✗ | 과잉 배달 | 221.8 | 220.0 | 23 | 146,204 | 1,906 | 230.5 | 231 | 5.28 |
| s12 plan_first graspfail | ✗ | 부족 배달 | 147.8 | 146.0 | 6 | 39,406 | 1,411 | 86.3 | 86 | 5.43 |
| s12 dynamic graspfail | ✓ | — | 165.8 | 164.0 | 13 | 86,901 | 1,304 | 127.0 | 128 | 4.79 |
| s13 independent nominal | ✗ | 과잉 배달 | 257.3 | 255.5 | 27 | 171,656 | 2,116 | 241.0 | 241 | 4.69 |
| s13 plan_first nominal | ✓ | — | 155.3 | 153.5 | 6 | 39,405 | 1,440 | 93.3 | 94 | 4.16 |
| s13 dynamic nominal | ✓ | — | 128.8 | 127.0 | 10 | 65,914 | 989 | 91.8 | 92 | 4.58 |
| s13 independent graspfail | ✓ | — | 156.3 | 154.5 | 14 | 88,073 | 1,090 | 141.6 | 142 | 5.15 |
| s13 plan_first graspfail | ✗ | 부족 배달 | 150.8 | 149.0 | 7 | 45,841 | 1,686 | 96.9 | 97 | 5.40 |
| s13 dynamic graspfail | ✓ | — | 171.8 | 170.0 | 15 | 100,168 | 1,423 | 125.5 | 126 | 5.11 |
| s14 independent nominal | ✗ | 과잉 배달 | 253.8 | 252.0 | 24 | 152,506 | 1,838 | 259.8 | 260 | 5.98 |
| s14 plan_first nominal | ✓ | — | 159.3 | 157.5 | 15 | 100,422 | 2,958 | 110.1 | 111 | 5.31 |
| s14 dynamic nominal | ✓ | — | 139.8 | 138.0 | 13 | 79,514 | 1,129 | 189.5 | 190 | 10.67 |
| s14 independent graspfail | ✗ | 과잉 배달 | 273.8 | 272.0 | 31 | 195,912 | 2,442 | 270.4 | 270 | 4.78 |
| s14 plan_first graspfail | ✗ | 부족 배달 | 133.3 | 131.5 | 9 | 59,408 | 2,013 | 82.8 | 84 | 5.51 |
| s14 dynamic graspfail | ✓ | — | 192.8 | 191.0 | 13 | 86,747 | 1,320 | 142.1 | 142 | 5.59 |

칸별 평균(3회):

| 칸 | 성공 | 입력 토큰 | 호출 | 드라이버 wall초 |
|---|---|---:|---:|---:|
| independent nominal | 0/3 | 156.9k | 24.7 | 241 |
| independent graspfail | 1/3 | 143.4k | 22.7 | 214 |
| plan_first nominal | 3/3 | 75.4k | 11.3 | 106 |
| plan_first graspfail | 0/3 | 48.2k | 7.3 | 89 |
| dynamic nominal | 3/3 | 75.0k | 11.7 | 130 |
| dynamic graspfail | 3/3 | 91.3k | 13.7 | 132 |

- 한 seed에서 6칸을 모두 돌리는 한 블록의 비용: 입력 590k 토큰(최댓값으로 합하면 707k), 출력 10.3k 토큰, 드라이버 wall 913초(최댓값 합 1,072초). 참고로 앞서 쓰던 "실행당 1.5–2.5분, 입력 60k–110k 토큰"은 plan_first와 dynamic에만 맞는다. **independent는 실행당 약 2.4–4.5분, 입력 88k–196k 토큰이다.**
- 호출당 지연은 4.2–6.0초다. s14 dynamic nominal만 평균 10.7초였고, 그래서 이 실행의 wall이 190초로 늘었다.
- 시작 조건 fixture(LLM 0회)의 드라이버 wall 합: ZC2 gate 6회 599초. ZC1 gate의 independent graspfail은 교사 막힘 때문에 393초였다. fixture 행 전체는 `measured_runs.json`의 `gate_fixture_runs`에 있다.
- 보조 참고: ZW1/ZW2(zone_wide)와 Z3(zone_open)의 plan_first/dynamic 실행 12회. 조건마다 1회이고 목표와 소스가 다르므로 합치지 않는다. 입력 47k–111k 토큰, wall 89–158초였다(`supplementary_cost_reference`).
- 비용(USD)은 `team.json`의 `cost_usd`가 null이다(제공자 청구 정보 없음). `budget.json`의 단가 칸을 채우면 환산할 수 있다.

## 2. 정밀도와 검정력

가정은 `power.json`의 `assumptions`에 있다. 요약:
- (seed, 조건, 시나리오)마다 1회만 돌린다. seed가 독립 표본이므로 각 칸의 결과는 seed 모집단 평균 성공률을 갖는 베르누이다. 같은 seed를 반복하지 않으므로 칸 안의 beta-binomial 과산포는 생기지 않는다.
- 6칸이 같은 seed 목록을 쓴다(블록 설계). 아래 계산은 짝을 무시한 비짝 검정(Fisher)과 독립 Beta 사후분포를 쓴다. seed 난이도가 조건 사이에 공유되면 이 계산은 보수적이다.
- 주 대비는 두 개다. 가족 α 0.05를 Holm/Bonferroni로 나눠 각 α 0.025로 둔다.
- ZC2의 3/3과 0/3은 극단값이다. 그래서 계획 성공률은 Jeffreys 사후평균으로 줄였다(3/3→0.875, 0/3→0.125, 6/6→0.929, 1/6→0.214). 보수적(0.8 대 0.3)과 중간(0.8 대 0.5) 가정도 함께 계산했다.

ZC2를 지금 검정하면: P1(dynamic 6/6 대 independent 1/6, 시나리오 합산)은 Fisher p = 0.015, P(p_dyn > p_ind) = 0.998이다. P2(graspfail, dynamic 3/3 대 plan_first 0/3)는 Fisher p = 0.10, 차이의 95% CI는 [0.21, 1.00]이다. n = 3으로는 P2를 확정할 수 없다.

### (a) 95% CI 반폭 목표에 필요한 칸당 실행 수

| 목표 | 한 칸 성공률, 최악(p≈0.5) | p = 0.8일 때 기대 | p = 0.9 | p = 0.95 | 두 칸 차이(Newcombe), 최악 | 0.8 대 0.3 | 0.875 대 0.125 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 반폭 ≤ 0.15 | Wilson 39 / CP 47 | 25 / 31 | 16 / 19 | 12 / 14 | 81 | 60 | 37 |
| 반폭 ≤ 0.20 | Wilson 21 / CP 27 | 13 / 17 | 9 / 12 | 7 / 9 | 44 | 33 | 21 |

### (b) ZC2 크기의 효과를 검출하는 데 필요한 칸당 실행 수

Fisher 양측 α 0.025, 검정력 0.8 / 0.9. Bayes는 Beta(1,1) 사전분포에서 P(p_dyn > p_other) ≥ 0.975에 도달할 확률(assurance)이 0.8 이상이 되는 n이다.

| 대비 | 계획 성공률 | Fisher 0.8 | Fisher 0.9 | Bayes 0.8 |
|---|---|---:|---:|---:|
| P1 dynamic 대 independent (두 시나리오 합산, 팔당 2n) | 0.929 대 0.214 (ZC2 축소) | 5 | 7 | 4 |
| | 0.8 대 0.3 | 10 | 13 | 8 |
| | 0.8 대 0.5 | 26 | 34 | 20 |
| P2 graspfail에서 dynamic 대 plan_first | 0.875 대 0.125 (ZC2 축소) | 9 | 10 | 6 |
| | 0.8 대 0.3 | 20 | 25 | 15 |
| | 0.8 대 0.5 | 52 | 67 | 39 |

- P1의 합산 근사는 시나리오를 층으로 하는 정확 검정을 가정한 크기 계산이다. 두 시나리오의 성공률 차이가 크면 약간 낙관적이다. 시나리오별 비교(S1: nominal에서 dynamic 대 independent)는 P2와 같은 n이 필요하다.
- **보조 결과(기술 통계로만 보고):** 완료 SIM초, 제어 종료 SIM초, 호출, 입력/출력 토큰, 호출당 지연, wall(참고). ZC2의 칸별 SD는 완료 SIM초 4.8–58.9초, 호출 1.2–8.5회, 입력 토큰 7.7k–54k다. 평균 SIM초의 95% 반폭 ±10초는 dynamic 7–8회, independent nominal 15회면 된다. independent graspfail은 SD 58.9초라 134회가 필요하다.

## 3. 설계 제안

### 칸
- **경기장:** `zones/zone_wide` v1이 주 경기장이며 기본값이다(ZC1/ZC2와 같다). `zone_open` v2는 선택 확장 축이다. full 단계에서만 따로 등록하고 **zone_wide 결과와 합치지 않는다.**
  - zone_open을 넣는 이유: TOP이 2대(요청당 이미지 3장)이고 통로가 좁다. 이런 장면에서도 조건의 순서가 유지되는지 보는 복제 확인이다.
  - 선결 조건: independent는 zone_open에서 한 번도 돌지 않았다. 그래서 zone_open 개발 seed로 fixture 시작 조건을 따로 통과해야 한다.
- **조건 3 × 시나리오 2:** independent / plan_first / dynamic × nominal / graspfail(`--inject-grasp-failure 2`: 두 번째로 배정된 작업의 집게가 열린 채 남는다). 목표 G8, 여분 빨강1·청록1.
- **고정 설정:** `gemini-3.8-flash`, 동기 SIM, `local_contact_fine`, weld OFF, `--record-replay`, `--max-sim-s 900`, `--max-wall-s 3600`, `--planning-rounds 8`. 실행기는 등록된 workflow `zone-dispatch`(`scripts/run_zone_dispatch.py`)다. 소스는 최종 사전 등록 커밋으로 고정하고 코호트 동안 바꾸지 않는다.
- **모델:** 하나로 고정한다(R5/S2: 통신 조건과 모델을 동시에 바꾸지 않는다). 모델 비교는 별도 후속 실험이다.

### Seed와 출발 배치 교차
- seed가 상자 배치, 색 순서, **로봇 id ↔ 출발 줄 배정**을 함께 섞는다(`sim.zone_arena.episode`). ZC2의 seed 12–14는 순열이 r1-r2-r3, r3-r2-r1, r2-r3-r1로 치우쳐 있었다. dynamic의 "가장 작은 id가 가져감" 규칙 때문에 r1의 위치가 결과에 영향을 줄 수 있다.
- **개발 seed**(시작 조건과 디버그에만 쓰고 분석하지 않음): zone_wide 101, 102, 103, 104, 106, 107 / zone_open 201, 202, 203, 207, 210, 212
- **보류(시험) seed**(순서대로 앞에서 n개를 쓴다. n = 6, 12, 18, 24의 앞부분이 모두 6개 순열을 같은 횟수로 포함한다):
  - zone_wide: 1001, 1002, 1003, 1004, 1006, 1007 | 1008, 1009, 1010, 1011, 1012, 1014 | 1015, 1016, 1018, 1026, 1028, 1032 | 1033, 1034, 1035, 1036, 1037, 1047
  - zone_open(선택): 2001, 2002, 2003, 2008, 2009, 2016, 2017, 2018, 2019, 2020, 2022, 2026
- 선택 규칙은 설정 난수만 보고 결과는 보지 않는다(`seed_plan.json`의 `rule`). 11–14는 이미 쓴 seed라 뺐다. 최종 목록은 사전 등록 커밋 때 고정하고, 그 뒤에는 바꾸지 않는다.
- 파지 실패가 어느 로봇에게 가는지는 배정 순서로 정해져 통제하지 않는다(ZC2 graspfail 9회 중 r2 7, r3 1, r1 1). 로봇과 출발 줄을 기록하고 기술 통계로 보고한다.

### 실행 순서
- 블록은 seed다. 블록 순서를 섞고, 블록 안의 6칸 순서도 섞는다. 난수 seed는 `20260925`이며 medium 예시 순서가 `seed_plan.json`에 있다. 시간에 따른 프록시·모델 변화가 한 조건에만 몰리지 않게 하려는 것이다.
- 한 번에 1회씩 실행한다. `scripts/agent_lock.py`(owner claude) 잠금 안에서 돌리고 실행마다 부하 평균을 기록한다(ZC2와 같다).

### 성공, 실패, 제외 (사전에 고정하고 사후 제외는 없음)
- **성공(주 결과):** 제어가 끝났을 때 심판의 구역별 색 개수가 목표와 정확히 같다. 동시에 `phase = FINISHED`, `error = null`이고 SIM 900초와 wall 3600초 안에 끝나야 한다. TOP RGB 일치(`goal_met_rgb`)는 따로 적는다.
- **실패 유형:** 과잉 배달 / 부족 배달 / 둘 다 / SIM 시간 초과 / wall 시간 초과 / API·전송 오류 / 교사 결함(`goal_occupied`가 아닌 `teacher_path_blocked`) / 교착·정체. 모두 분모에 넣는다(배정한 전체 실행 기준, ITT). 원인별 표를 함께 낸다. 인프라 오류를 뺀 민감도 분석은 보조로만 쓴다.
- **유일한 재시도 규칙:** 첫 모델 요청 전에 프로세스가 죽으면(프록시 미기동, MuJoCo 초기화 실패 등) 같은 (seed, 칸)을 제자리에서 한 번 다시 돌린다. 두 시도를 모두 기록한다. 두 번째도 실패하면 오류 실패로 센다. 첫 모델 요청 이후의 결과는 어떤 것도 빼지 않는다.

### 중지 규칙
- 효과가 크거나 없다는 이유로 중간에 멈추지 않는다. 예정한 n을 모두 채운다.
- **시작 조건:** 개발 seed 101에서 6칸 fixture(LLM 0회)를 돌린다. 오류가 없고 `goal_occupied`가 아닌 교사 막힘이 0이어야 LLM 실행을 시작한다(ZC2 규칙과 같다).
- **중단(기록하고 새 코호트로 재등록, 이전 실행과 합치지 않음):**
  - 교사 결함이 2회 나온 경우
  - API·전송 오류가 누적 10%를 넘은 경우
  - 응답의 모델 문자열이 바뀐 경우
  - 누적 입력 토큰이 선택한 단계 추정치의 1.3배를 넘은 경우
  - 중단한 경우 완료된 블록까지만 "미완료 코호트"로 보고하고 효과 주장은 하지 않는다.

### 분석 계획
- **P1:** dynamic 대 independent. 시나리오를 층으로 한 정확 검정(Mantel–Haenszel 정확)으로 한다.
- **P2:** graspfail에서 dynamic 대 plan_first. Fisher 정확 검정으로 한다.
- Holm으로 두 대비를 보정하고, 성공률 차이와 Newcombe 95% CI를 함께 보고한다. 같은 seed 짝 분석(정확 McNemar)과 Beta(1,1) 사후확률은 보조로 함께 싣는다.
- P2에서 plan_first가 실패하는 것은 설계상 예상된다(실패 뒤 재계획이 없음). 그래서 P2는 사실상 "dynamic의 복구가 새 seed에서도 유지되는가"를 묻는다. 이 점을 결론에 그대로 적는다.
- **보조:** S1(nominal에서 dynamic 대 independent), plan_first 대 dynamic nominal(동등성 주장은 하지 않음), 과잉/부족 수, `box_taken_by_peer`, 충돌·무효 선언.
- 비용은 칸별 평균/SD/범위와 seed 짝 차이의 bootstrap 95% CI로 보고한다(seed 단위 재표집 10,000회, 난수 seed 고정). 비용 항목은 완료 SIM초, 호출, 토큰, 호출당 지연이다.
- 성공한 실행끼리의 SIM 시간 비교는 따로 표시한다. 실패한 실행의 짧은 시간은 이득으로 세지 않는다.
- 결과는 새 TensorBoard 스냅샷에 넣는다(ZC3 실행 뒤 별도 작업이며 이번 초안에서는 하지 않았다).

## 4. 예산 3단계

추정 근거: ZC2 칸별 평균. Mac, 동기 SIM, 한 번에 1회. 잠금 대기와 기록 작업 등 측정하지 않은 여유로 1.15배를 곱했다. "상한"은 칸별 최댓값을 합한 값이다. zone_open 비용은 zone_wide 값을 보수적 상한으로 썼다(이미지 5장 → 3장).

| 단계 | 칸당 n | 실LLM 실행 | fixture | 입력 토큰(상한) | 출력 토큰 | 모델 호출 | wall 시간(상한) |
|---|---|---:|---:|---:|---:|---:|---:|
| small | zone_wide 6 | 36 | 6 | 3.5M (4.2M) | 62k | ~550 | 1.9 h (2.2 h) |
| medium | zone_wide 12 | 72 | 6 | 7.1M (8.5M) | 124k | ~1,100 | 3.7 h (4.3 h) |
| full | zone_wide 24 + zone_open 12 | 216 | 12 | 21.2M (25.5M) | 372k | ~3,300 | 10.9 h (12.7 h) |

- full에서 zone_open을 빼면 144회, 입력 약 14.2M 토큰, 약 7.2 h다.
- 한 칸의 최악 반폭 ≤ 0.15를 원하면 칸당 39–47회가 필요하다(zone_wide만 234–282회, 입력 23–28M 토큰, 약 11.6–13.9 h).
- 모든 실행이 wall 제한(3600초)에 걸리는 최악의 경우는 실행 수 × 1 h다. 실제로 가장 긴 실행은 270초였다.

| 단계 | 주장할 수 있는 것 | 주장할 수 없는 것 |
|---|---|---|
| small | ZC2 크기의 P1 효과가 새 보류 seed에서 재현되는가(Fisher 검정력 약 0.90). P2는 ZC2 크기일 때 Bayes 진술만 가능(assurance 0.82, Fisher 0.55). 칸별 비용 기술 통계 | 0.8 대 0.3 이하 효과. 칸 성공률의 반폭 ≤ 0.20(최악 0.38, 6/6이어도 0.23). 경기장·목표·모델 일반화 |
| medium | P1은 보수적 효과 0.8 대 0.3에서 검정력 0.88. P2는 ZC2 크기에서 0.93(0.8 대 0.3에서는 Bayes assurance 0.77뿐). 성공률이 0이나 1 근처인 칸의 반폭 ≤ 0.20(12/12이면 0.13). 짝 비용 차이 | P2의 0.8 대 0.3 빈도주의 검정력 0.8. 중간 효과(0.8 대 0.5). 칸 최악 반폭 ≤ 0.20. 경기장 일반화 |
| full | P1·P2 모두 0.8 대 0.3에서 검정력 약 1.0 / 0.88. P1은 0.8 대 0.5에서 0.75(Bayes 0.88). 칸 최악 반폭 ≤ 0.20(Wilson 기준, CP는 0.21). zone_open 별도 복제표 | 칸 반폭 ≤ 0.15. P2의 중간 효과(칸당 약 52회 필요). 모델·목표 일반화. 학생·RGB 스킬 성공. 실시간 동작 |

## 5. 의존성: 동시 진행 중인 작업

- **구역 통신 경계 감사(`claude/zone-comm-audit`):** 감사에서 누출이나 혼입이 나오면 ZC3는 그 수정이 main에 들어간 뒤에 소스를 고정한다. 누출 예시: independent가 동료의 선언·게시판·호스트 판정을 간접적으로 받는 경우, 또는 호스트 검사가 시뮬레이터 상태를 쓰는 경우. 그 경우 이 초안의 비용 추정도 다시 계산해야 할 수 있다. ZC2 결과는 감사 결과가 나올 때까지 개발 증거로만 본다.
- **상자 색 인식(`claude/zone-rgb-color`):** 이 작업이 TOP RGB 상자 추정을 바꾸면 로봇 입력이 바뀐다. ZC3 소스를 그 병합 전으로 고정할지 후로 고정할지 정해야 한다. 코호트 도중에는 바꾸지 않는다. 병합 뒤로 고정하면 새 입력 경로로 시작 조건을 다시 통과해야 한다.
- Codex의 빔 재파지 작업은 dispatch 경로이며 zone 실행기와 겹치지 않는다.

## 6. 결정할 것 (사용자)

1. 예산 단계: small / medium / full, 또는 full에서 zone_open을 뺀 144회
2. 주 대비 두 개(P1 시나리오 합산, P2 graspfail)와 α 0.025씩(Holm)으로 할지. Bayes 기준을 주 판정으로 올릴지
3. "의미 있는 차이"의 수치(R0). 제안은 성공률 차이 ≥ 0.3이다(보수적 계획 0.8 대 0.3을 기준으로 크기 결정).
4. 교사 실행기 조건을 유지할지. RGB 실행기로 옮기는 것은 별도 단계다.
5. zone-comm-audit와 zone-rgb-color 병합을 기다릴지. 기다린다면 어느 커밋에 고정할지
6. 모델 단가(USD 환산용)와 전체 토큰 상한

## 7. 재현과 입력 해시

```bash
/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python \
  experiments/2026-09-25-zc3-prereg-draft/analysis/zc3_prereg_analysis.py \
  --outputs-root /Users/changmin/projects/ugrp/outputs
```

- 두 번 실행해 산출물 네 개가 바이트 단위로 같음을 확인했다(약 8초, SIM/LLM 호출 없음).
- 읽은 입력 118개(원본 JSON/JSONL 117개 + 커밋된 ZC2 `results.json`)의 SHA-256은 `analysis/measured_runs.json`의 `input_sha256`에 있다. 주요 입력:

| 파일 | SHA-256 |
|---|---|
| `outputs/zone-communication-20260925/loads.jsonl` | `2a5708f9f6bc5796c8557a3e5635bde4a7a678d6498f217f5af3b81e549ecf20` |
| `outputs/zone-wide-20260925/loads.jsonl` | `dcb5059d4240af7de1e9e92bcebcef738a38fd0b44e9b84322bfa7890ab9cff1` |
| `outputs/zone-dispatch-20260925/loads.jsonl` | `b5248d100614316f06f5d552d0707f6ee6b2c7890ea3815398dad180e89a3f4c` |
| `experiments/2026-09-25-zone-communication/results.json` | `8e866f419d58f055354ff2cfafd53675e3221eb38a1a2d5eee5bf33ad24b2f0c` |

- seed 계획은 `sim.zone_arena.episode`의 설정 난수만 호출한다(장면 생성이나 SIM 실행 없음). 따라서 이 초안을 넣은 커밋의 `sim/zone_arena.py`에 의존한다.
- 이번 작업은 TensorBoard를 갱신하지 않았다. 새 실험 결과가 없기 때문이다.
