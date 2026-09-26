# 2026-09-26 환경 v3: 벽 0.40 m + 희소 AprilTag (zone 지도)

- 작업: Kiro, 브랜치 `kiro/zone-map-v3`, PR #208(draft). 첫 Kiro 세션(계정 1)이 코호트 실행 중 멈춰, 계정 2가 같은 worktree·브랜치·PR에서 이어받았다. 이어받은 시점에 실행 중이던 test 코호트는 종료하지 않고 끝까지 기다렸다. 이미 실행한 사전 등록 코호트는 다시 돌리지 않았다.
- 사용자 요청(2026-09-26): "벽에 뭐가 저렇게 많아야 하나"(태그를 필요한 곳에만), "벽이 너무 낮은 거 같아, 벽 너머가 보여야 하나"(벽을 카메라보다 높게).
- 조건: 동기 SIM, `cargo_noslip_v1`(사용자 승인), weld OFF, 실행 중 GT 없음(평가 전용), 스레드 1, 동시 SIM ≤ 2, 코호트 전 디스크 여유 ≥ 30 GiB 확인. LLM·외부 API 호출 없음.

## 결론 (요약)
- **환경 v3 자체(정적 검사)는 통과했다.** 지도·장면이 일치한다(ST1). 벽 0.40 m에서 다른 방이 wrist 화면에 0 px로 보인다(ST4, v2는 10k–92k px). 과제 정거장 38/38에서 태그가 1장 이상 보인다(ST3). 문 지도의 TOP 평가 카메라는 관련 바닥을 100% 본다(ST5). 복도 지도에서는 가림이 늘었다(아래).
- **v3 지도(태그 33장)의 자기 카메라 위치 추정 게이트는 실패했다**(새 seed 721–729, 1회). box 3/6, nobox 0/3, 벽 접촉 223회.
- 원인은 둘로 나뉜다.
  - 태그 문제(M1, M2): v2에서는 0.10 m 벽 **너머로** 보이던 태그가 위치 추정을 지탱했다. 벽이 높아지자 그 태그가 사라졌다.
  - 태그와 무관한 물리 문제(M3): 벽이 0.40 m가 되자, 예전에는 벽 위로 지나가던 팔·손가락·든 상자가 벽에 닿았다.
- 수정안 A1(태그 추가만, 새 지도 `*_tags_v3a1`, 새 seed 741–749)을 결과 전에 등록하고 실행했다. **A1도 게이트는 실패했다**: box 4/6, nobox 2/3, 벽 접촉 2.
  - 위치 추정만 보면(R2 제외, 부가 판독) nobox 3/3, box 4/6으로 좋아졌고, 벽 접촉은 223 → 2로 줄었다.
  - 남은 실패는 셋이다: 손가락 접촉(M3), S6가 없는 북쪽 접근, 교사 구간에서 이미 커진 인계 오차.
- M1 dev 점검(3단계)은 위치 추정 게이트 통과가 조건이라 **실행하지 않았다**.
- 다음 단계는 태그가 아니라 학생 쪽이다. 3-D 몸체 envelope, 문 안 둘러보기 pan 제한, 인계 전 fix가 필요하다(새 사전 등록 필요, [한계](#한계와-열린-문제)).

## 환경 v3 정의
### 벽 높이 0.40 m (`walls_v3`)
근거는 [`camera_height.json`](camera_height.json)이다(`camera_height.py`, MuJoCo `mj_forward`, 발행 PWM → 관절). `sim/zone_arena.py`의 `WALL_PROFILES`에 추가만 했다.

| 항목 | 높이 |
|---|---|
| 운용 자세 wrist 카메라 최고 | 0.239 m(`look_p10`). SEARCH 0.209, CARRY_POSTURE 0.208, LOOK_P20 0.204, 생산 CARRY_POSE 0.230 |
| 명령 가능한 전 PWM 범위의 카메라 최고 | **0.321 m**(팔 수직 `{3:1425, 4:1550, 5:1590}`, FK 0.3209 m와 일치) |
| 로봇 geom 최고점 | 0.367 m(손가락 패드, 팔 수직) |

규칙 `ceil_0.05(max(카메라 최대 + 0.05, 로봇 최고 + 0.03))`에 따라 **0.40 m**로 정했다. 벽 footprint·문·통로·구역·TOP 카메라·출발 위치는 바꾸지 않았다. v1/v2 지도·형상 파일은 바이트 그대로다(`tests/test_zone_env_v3.py`가 SHA-256 고정).

### 태그 규칙 `ugrp.zone_tag_rule.v3` (위치 추정 실행 전 커밋 `a82f0926`)
[`tag_rule_v3.md`](tag_rule_v3.md)와 `sim/zone_tag_rule_v3.py`에 있다. 사이트 하나는 벽면 한 곳의 세로 3장(z 0.05/0.15/0.25 m, tag36h11 0.072 m, v1/v2와 같은 크기)이다. 두는 곳은 문틀 양면, 픽업 bay slot, 구역(입구와 slot), 복도 입구·차로 축·대피 bay다.

| 지도 | v3 태그(사이트) | v1 / v2 | v3a1(수정안) | 파일 sha256 (v3 / v3a1) |
|---|---|---|---|---|
| `zone_wide_door` | 33 (11) | 70 / 80 | 45 (15) | `74a3a959…` / `23de1456…` |
| `zone_wide_two_doors` | 36 (12) | 65 / 84 | 54 (18) | `9a139176…` / `6088284c…` |
| `zone_wide_corridor` | 33 (11) | 87 / – | (문 없음, v3 그대로) | `1414781a…` |

- 지도 파일에는 base 지도 해시, 벽 프로필 기록(해시 포함), v3a1은 수정 대상 v3 지도 해시(`amends`)를 넣었다.
- 등록부는 따로 두었다: `sim/zone_landmarks.ENV_V3_TAGGED_MAPS`, `ENV_V3A1_TAGGED_MAPS`. 기존 `TAGGED_MAPS`의 계약은 그대로다.
- `sim/zone_scene.py`는 `cargo_noslip_v1` 이름을 받는 부분만 추가했다(M1 러너와 같은 XML, 테스트로 확인).

## 사전 등록
- `prereg.json`, `loop/prereg.json`, `m1/prereg.json`: 커밋 `251fd0a8`, 모든 v3 실행 전이다.
  - 정적 검사 ST1–ST5.
  - 폐루프 게이트: loop v2 학생(드라이버 v2 + `calibration_loop_v2.json`) 그대로, R1–R4.
    - LG1: box ≥ 5/6.
    - LG2: nobox 3/3.
    - LG3: 벽 접촉 0, 낙하 0.
  - M1 dev: s731–733, 폐루프 통과 시에만.
  - 수정안 정책: 1회, 태그 추가만, 위치·이유 명시, 새 지도 버전, 새 seed.
- 수정안 A1: `prereg_amendments.json`, `loop/prereg_a1.json`, `m1/prereg_a1.json`. 커밋 `23f068a1`, v3 test 실패 뒤, A1 실행 전이다.
- test 소스: `8b361441`(dirty 없음). A1 test 소스: `23f068a1`(dirty 없음). 러너 `scripts/run_owncam_closed_loop.py`는 수정하지 않았다.
- TOP은 평가 전용 래퍼 `observe_top.py`로만 녹화했다. dev s711을 래퍼 없이 다시 돌렸다(`dev-plain`). 입력 프레임·명령·학생 로그·GT 궤적 sha256이 래퍼 실행과 같았다. 즉 래퍼는 물리·입력을 바꾸지 않는다.

## 1단계: 정적 검사 (소스 `f843d392`, 기록 `static_results.json`)
| 검사 | 결과 |
|---|---|
| ST1 지도·장면 일치 | 통과. 세 지도 모두 파일 = 정의. 벽 0.40 m(컴파일 모델 오차 < 1e-6 m), 태그 시각 전용, noslip 10, weld 0 |
| ST2 거리별 판독 | SEARCH: z0.05 태그를 3.0 m까지 읽고, 1.5 m 이내 거리 오차 ≤ 5 mm. z0.15는 0.3 m에서만 읽는다. LOOK_P20(빈손): 세 높이 모두 2.5–3.5 m까지. **상자 든 CARRY 주행 자세: z0.05만 0.6–1.0 m에서 읽는다.** 상자 든 LOOK_P20: z0.15 0.6–3.0 m, z0.25 1.5–3.0 m |
| ST3 정거장 적용 범위 | 통과. 게이트 38/38 정거장에서 태그 ≥ 1장. 최소는 preplace_A1 loaded 4장. 가장 가까운 태그가 가장 먼 곳은 문 출구 W(빈손) 2.72 m |
| ST4 벽 가림 | 통과. v3 wrist 12개 화면(4 자세 × 3 거리)에서 동쪽 방 0 px. v2는 같은 화면에서 10k–92k px([그림](media/st4_wall_occlusion_v2_top_v3_bottom.jpg)) |
| ST5 TOP 가시 범위 (카메라 이동 없음) | door·two_doors: 관련 영역 모두 1.0. 관찰자 바닥 전체는 0.987 → 0.951. **corridor: 차로 0.973 → 0.869, 대피 bay 0.963 → 0.818, 안쪽 벽 0.30 m 이내 0.987 → 0.934**. 복도 지도는 기울어진 TOP 시선이 높은 벽에 가린다. 카메라는 옮기지 않았고, 복도 지도에서 TOP 평가를 쓰려면 사용자 결정이 필요하다 |

그림: [ST3 정거장 화면](media/st3_station_views.jpg), [ST2 거리](media/st2_distance_look_p20_loaded.jpg), [ST5 가시 범위](media/st5_top_coverage.png).

## 2단계: v3 폐루프 위치 추정 test (1회, `loop/results.json`)
문 출구 W=(2.65, 0.05)까지 자기 추정만으로 주행한다. box 조건은 교사가 상자를 들어 준 뒤 학생이 이어받는다. 교사 구간은 성공에 넣지 않는다.

| 에피소드 | 결과 | R1 GT 거리 | R2 벽 접촉 | R3 p90 | 학생 SIM | 둘러보기 | 명령 |
|---|---|---|---|---|---|---|---|
| nobox s721 | 실패(R3) | 5.2 cm | 0 | 8.7 cm | 76.4 s | 5 | 1064 |
| nobox s722 | 실패(R2) | 5.6 cm | 4 | 5.6 cm | 124.9 s | 9 | 1789 |
| nobox s723 | 실패(R2) | 3.4 cm | 3 | 2.3 cm | 108.5 s | 6 | 1445 |
| box s724 | 실패(시간 한도) | – | 0 | 4.9 cm | 240.1 s | 18 | 3607 |
| box s725 | **통과** | 4.3 cm | 0 | 2.4 cm | 115.6 s | 9 | 1768 |
| box s726 | **통과** | 3.0 cm | 0 | 4.4 cm | 235.8 s | 19 | 3635 |
| box s727 | 실패(R2, R3) | 3.2 cm | 110 | 23.5 cm | 150.1 s | 11 | 2243 |
| box s728 | 실패(시간 한도, R2, R3) | – | 106 | 16.0 cm | 240.1 s | 20 | 3672 |
| box s729 | **통과** | 5.9 cm | 0 | 3.8 cm | 101.9 s | 8 | 1536 |

- 코호트 게이트: **LG1 3/6, LG2 0/3, LG3 벽 접촉 223·낙하 0 → 실패.**
- 상자 낙하는 0이다. 태그 가시율은 box 0.22–0.55, nobox 0.71–0.80이다.
- dev s711/s712(게이트 아님)도 R3에서 실패했다.
- 기준선은 loop v2 test(v2 지도, 80태그, 0.10 m 벽)다. box 6/6, 학생 SIM 93.6–216.1 s. **기준선의 접촉 프로필은 `local_contact_fine`**이라 접촉 프로필도 다르다. 아래 기하 분석(M1, M3)은 접촉 프로필과 무관한 사실만 쓴다.

### 실패 원인 (사후, 평가 전용 GT, `analyze_failures.py` → `loop/failure_analysis.json`)
- **M1 짐 상태 접근 띠가 눈먼다.**
  - 위치: 남쪽에서 문으로 가는 경로, divider 서쪽 면을 따라 x 1.80–2.05, y −1.30…−0.30.
  - v2 기준선: 이 띠의 944 프레임 중 559 프레임에서 태그가 보였다. 관측 1257건 중 **1149건은 0.10 m 벽 너머 동쪽 방 태그**였다. 0.40 m 벽이면 가려지고, 규칙으로 뺀 것이 77건, v3에 남는 것은 31건이다.
  - v3: s728은 띠의 570/570 프레임, s727은 115/115 프레임에서 태그가 없었다. 상자 든 odometry가 x를 짧게 추정했다(s728 −0.11 m). 그런데도 std는 0.03 m로 작았다(거짓 확신). 결국 상자가 divider 문설주에 닿았다(y −0.27…−0.16).
- **M2 문 안 y·yaw 혼동.** 문 안에서 보이는 v3 태그는 2.7–3.2 m 떨어진 동쪽 벽 구역 사이트뿐이다. 광축 근처 좁은 방위에 몰려 있고, PnP 거리 오차가 0.2–0.4 m다. v2에서는 북쪽 벽 x 3.3–4.3 태그가 이를 풀었다. v3에서는 s711, s721의 R3가 실패했다.
- **M3 높은 벽에 몸이 닿는다(태그와 무관).**
  - 0.10 m 벽에서는 0.10 m 아래 섀시만 벽에 닿을 수 있었다. 섀시는 기준점에서 x +0.094, |y| 0.081 m까지다.
  - 0.40 m 벽에서는 그 위의 팔·손가락·상자도 닿는다. 상자 든 자세는 x +0.167 m, 빈손 둘러보기 pan은 |y| 0.141 m까지 뻗는다(`mj_forward`).
  - s722/s723은 추정 오차가 2–13 mm였는데도, 문 안 둘러보기 pan 중 손가락이 divider 끝에 닿았다([영상](media/clip_v3test_nobox_s723_finger_contact.mp4)).
  - 계획기(`harness/map_goto.plan_path`, `escape_start_m=0.25`)는 출발점 0.25 m 안에서 막힌 칸을 지나갈 수 있다. 2-D 몸체 envelope ±0.15 m + 여유 0.02 m이면 0.50 m 문에서 한쪽 여유가 약 0.08 m다.
  - 학생은 이번 검증에서 고정이다. 따라서 M3는 **열린 문제**다.
- **M4 픽업 영역 둘러보기 증가.** 'uncertain' 둘러보기가 s724 12회, s728 16회였다. 일부는 M1의 영향이고, 둘러보기 때문에 box 두 에피소드가 240 s 한도에 걸렸다.

영상: [s727 상자·문설주 접촉](media/clip_v3test_box_s727_jamb_contact.mp4), [s725 통과](media/clip_v3test_box_s725_pass.mp4). 전체 영상은 `outputs/`에 있다(아래).

## 수정안 A1 (태그 추가만, 커밋 `23f068a1`)
`prereg_amendments.json`에 위치·이유·예측을 적었다. v3 사이트와 태그 id 0–32는 그대로다. 규칙은 `ugrp.zone_tag_rule.v3a1`이다.
- **S6 문 접근 차로**(M1 대응)
  - 위치: 픽업 영역 중심선을 향한 문 가장자리가 안쪽 벽 끝이면, 그 벽의 픽업 쪽 면에 문틀 사이트부터 0.30 m마다 1.20 m까지 둔다.
  - 문 지도 결과: (2.175, −0.55)·(−1.15)·(−1.45). −0.85는 기존 site_05에 병합됐다.
  - 이유: 벽 바로 옆을 달리는 짐 상태 카메라는 그 면의 태그를 옆으로 0.4–0.9 m 떨어진 곳에서만 읽는다(ST6).
- **S7 문 출구 측면**(M2 대응)
  - 위치: 가까운 옆 바깥 벽의 구역 쪽, 문 중심에서 45° 방향.
  - 문 지도 결과: 북쪽 벽 (3.575, 1.425).
  - 이유: 문 영역에서 1.4–1.9 m에 보이는 축 밖 태그다.
- **ST6 근거**(`static_candidates.py` → `st6_candidates.json`, 사후, `mj_forward`)
  - 방법: 모든 벽면에 0.15 m 간격 후보 태그를 두고 정거장별로 판독했다.
  - 기록: 첫 실행은 렌더 geom 버퍼(10000)를 넘겨 버리고(`st6-overflowed`), 높이별 배치로 다시 돌렸다.
  - 띠 정거장 중 divider 면 사이트가 읽히는 곳: v3 1/10 → v3a1 6/10(x 1.85는 5/5, x 1.95는 1/5).
  - 문 정거장에서 측면 사이트가 읽히는 곳: 빈손 14/15, 짐 상태 5/5.
- 사전 예측: M1, M2는 줄고 M3는 그대로다. 그래서 LG3가 접촉만으로 실패할 수 있다.
- 부가 판독(게이트 아님): R2를 뺀 R1·R3(·R4)를 따로 보고한다.

## 수정안 A1 결과
새 seed 741–749, 1회 실행, 소스 `23f068a1`(dirty 없음), 지도 `zone_wide_door_tags_v3a1`(45태그). 결과는 `loop/results.json`의 `v3a1-test`에 있다.

| 에피소드 | 결과 | R1 GT 거리 | R2 벽 접촉 | R3 p90 | 학생 SIM | 둘러보기 | 명령 |
|---|---|---|---|---|---|---|---|
| nobox s741 | **통과** | 2.5 cm | 0 | 5.6 cm | 98.5 s | 7 | 1405 |
| nobox s742 | 실패(R2) | 1.7 cm | 2 | 2.6 cm | 93.0 s | 6 | 1290 |
| nobox s743 | **통과** | 2.6 cm | 0 | 3.9 cm | 115.4 s | 6 | 1514 |
| box s744 | **통과** | 5.1 cm | 0 | 3.2 cm | 65.7 s | 5 | 1003 |
| box s745 | **통과** | 4.1 cm | 0 | 4.2 cm | 146.1 s | 11 | 2210 |
| box s746 | **통과** | 6.1 cm | 0 | 3.8 cm | 66.9 s | 5 | 1016 |
| box s747 | 실패(R3) | 6.4 cm | 0 | 7.2 cm | 130.5 s | 10 | 1987 |
| box s748 | 실패(시간 한도, R3) | – | 0 | 7.7 cm | 240.1 s | 22 | 3790 |
| box s749 | **통과** | 4.8 cm | 0 | 5.1 cm | 117.2 s | 9 | 1782 |

- 코호트 게이트: **LG1 4/6, LG2 2/3, LG3 벽 접촉 2·낙하 0 → 실패.** 수정안은 1회까지라 더 고치지 않는다.
- v3 → A1 비교:

| 항목 | v3 | A1 |
|---|---|---|
| 게이트 통과 nobox / box | 0/3 / 3/6 | 2/3 / 4/6 |
| 위치 추정만(R1·R3·R4, R2 제외, 부가 판독) nobox / box | 2/3 / 3/6 | **3/3** / 4/6 |
| 벽 접촉 | 223 | 2 |

  seed가 달라 같은 장면의 쌍 비교는 아니다.
- 실패 원인(사후, `loop/failure_analysis.json`의 `v3a1_test`):
  - **s742**(M3): 추정 오차 2.0 cm인데, 문 안 둘러보기 pan 중 오른손가락이 divider 끝에 닿았다. v3의 s722/s723과 같은 기구다. 예측대로 태그로는 줄지 않았다.
  - **s747**: 북쪽 출발에서 divider_2 쪽으로 접근했다. 이 경로는 S6가 없는 쪽이다(규칙상 픽업 중심선 쪽 문 가장자리만). 문 앞 x 2.0–2.2 구간에서 태그 0장, y 오차 −7.4 cm로 R3 7.2 cm가 됐다. 측면 사이트(42–44)는 문 안쪽 x 2.3 이후에야 보였다.
  - **s748**: 학생 인계 시점에 이미 추정 오차가 0.475 m였다. 교사가 픽업 영역 남동쪽 모서리까지 대각선으로 주행하는 동안(교사 구간) 태그가 거의 보이지 않았다. 이후 'uncertain' 둘러보기를 0.1 m마다 반복하며 오차를 0.03 m까지 줄였지만, 240 s 한도에 걸렸다(둘러보기 22회, 태그 가시율 0.109). 교사 주행과 인계 규약의 문제이며, 태그 배치와는 간접적으로만 관련된다.
- 짐 상태 접근 띠(M1): A1에서 띠를 지난 에피소드(s745, s748, s749)의 띠 평균 오차는 2.8–3.5 cm였다. v3의 s727/s728은 7.3–11.1 cm였고, 상자·문설주 접촉은 0으로 줄었다([s745 영상](media/clip_v3a1test_box_s745_approach_lane_pass.mp4), 띠에서 site_12 태그가 보인다).

## 3단계: M1 dev 점검
**실행하지 않았다.** 사전 등록(`m1/prereg.json`, `m1/prereg_a1.json`)은 폐루프 게이트 통과를 조건으로 하는데, v3와 A1 모두 실패했다. seed 731–733과 751–753은 사용하지 않은 채 남아 있다.

## 영상과 TensorBoard
- 전체 영상: `/Users/changmin/projects/ugrp/outputs/zone-env-v3-20260926/videos/*.mp4`. 왼쪽은 wrist(로봇 입력), 오른쪽은 TOP 4대 모자이크(평가 전용), SIM 2배속이다. 목록과 sha256은 `videos.json`에 있다.
- PR 첨부용 짧은 영상(파일당 < 1 MiB, 합계 < 1 MiB): `media/clip_*.mp4`.
- TensorBoard 스냅샷 `outputs/tensorboard/0926-zone-env-v3`(27 run, 기록 `tensorboard_snapshot.json`)
  - run 구성:
    - `v3dev-*`: dev 2.
    - `v3devplain-*`: TOP 녹화 없는 결정성 확인 1.
    - `v3test-*`: v3 test 9.
    - `v3a1test-*`: A1 test 9.
    - `base-loopv2-*`: 기준선 loop v2 test 6. v2 지도, `local_contact_fine`.
  - 영상: 20 run에 `media/execution.mp4`가 있다. 미디어 서버는 127.0.0.1:6009다.
  - 검증:
    - EventAccumulator scalar 162개가 원본과 불일치 0이다.
    - 실행 중인 공용 서버(6006, logdir `outputs/tensorboard`)가 27 run을 모두 나열한다.
    - 서버에서 받은 scalar 108개(성공·SIM·명령·호출)가 `loop/results.json`과 일치한다.
    - 영상 범위 요청이 206 video/mp4로 응답했다.
  - 보기 설정은 `outputs/tensorboard-view.json`의 `zone_env_v3_20260926` 키(고정 카드 6개, run filter `^0926-zone-env-v3/`, HParams 열)다. 헤드리스 Chrome으로 고정 링크를 열어, 고정 카드 6개와 27 run 목록이 뜨는 것을 확인했다.
  - 한계: 공용 logdir에 run이 1000개를 넘어, TensorBoard가 새 run을 기본 선택 해제한다. 곡선을 보려면 run 목록에서 선택해야 한다.
  - 기존 변환기는 R1 거리·R3 p90·벽 접촉을 scalar로 내보내지 않는다. 이 값들은 Text `evaluation/referee_only`에 있다.
  - 모델 호출은 0이고, 모델 응답 시간은 해당 없음이다.
- 링크: http://127.0.0.1:6006/?pinnedCards=%5B%7B%22plugin%22%3A%22scalars%22%2C%22tag%22%3A%22evaluation%2Freported_success%22%7D%2C%7B%22plugin%22%3A%22scalars%22%2C%22tag%22%3A%22result%2Fsim_s%22%7D%2C%7B%22plugin%22%3A%22scalars%22%2C%22tag%22%3A%22result%2Fcommands%22%7D%2C%7B%22plugin%22%3A%22scalars%22%2C%22tag%22%3A%22result%2Fwall_s%22%7D%2C%7B%22plugin%22%3A%22scalars%22%2C%22tag%22%3A%22result%2Fmodel_calls%22%7D%2C%7B%22plugin%22%3A%22scalars%22%2C%22tag%22%3A%22claims%2Fprotocol_complete%22%7D%5D&smoothing=0&runFilter=%5E0926-zone-env-v3%2F#timeseries

## 재현
```sh
# 정적 검사 (raw는 outputs/, 요약은 static_results.json)
.venv-sim-worker-mac/bin/python experiments/2026-09-26-zone-env-v3/static_checks.py --record experiments/2026-09-26-zone-env-v3/static_results.json
# 폐루프 한 에피소드 (스레드 1, 디스크 확인, 소유 세션)
experiments/2026-09-26-zone-env-v3/run_one.sh loop experiments/2026-09-26-zone-env-v3/loop/prereg_a1.json a1test-box-s744 <out>
# 결과·사후 분석·ST6
.venv-sim-worker-mac/bin/python experiments/2026-09-26-zone-env-v3/build_results.py --raw <outputs>/zone-env-v3-20260926/loop --output experiments/2026-09-26-zone-env-v3/loop/results.json
.venv-sim-worker-mac/bin/python experiments/2026-09-26-zone-env-v3/analyze_failures.py --v3 <loop/test> --v2 <owncam-loop-v2-20260926/test> --output experiments/2026-09-26-zone-env-v3/loop/failure_analysis.json
.venv-sim-worker-mac/bin/python experiments/2026-09-26-zone-env-v3/static_candidates.py --library <static/posture_library.json> --output <st6> --record experiments/2026-09-26-zone-env-v3/st6_candidates.json
```

## 원본 위치 (로컬 보관, 원격 백업 아님)
`/Users/changmin/projects/ugrp/outputs/zone-env-v3-20260926/`
- `static/`: 정적 검사 raw, 프레임 78장.
- `loop/dev`, `loop/dev-plain`, `loop/test`, `loop/a1-test`: 에피소드별 `result.json`·`manifest.json`, wrist 입력 프레임, 명령, `eval_only/`(GT·접촉·TOP).
- `st6/`, `st6-overflowed/`(사용하지 않음), `videos/`.
- 에피소드별 결과·manifest·지도 파일 sha256과 파일 목록 해시는 `loop/results.json`의 `raw`에 있다.

## 한계와 열린 문제
- M3(높은 벽에 팔·손가락·상자가 닿음)는 태그로 풀 수 없다. A1에서도 s742에서 남았다. 계획기의 3-D 몸체 envelope(팔·상자 높이 포함), 문 안 둘러보기 pan 제한, `escape_start` 조정은 학생 변경이라 새 사전 등록이 필요하다. 이번 작업 범위 밖이다.
- A1 S6는 픽업 중심선 쪽 문 가장자리에만 태그를 둔다. 북쪽(divider_2 쪽) 접근은 문 앞 0.2 m 구간이 태그 없이 남는다(s747). 규칙을 더 바꾸려면 새 수정안과 새 seed가 필요하다(이번 작업의 수정안 1회는 사용함).
- box 조건에서 교사 구간 동안 학생 추정기가 커진 오차를 그대로 넘겨받는 경우가 있다(A1 s748 인계 오차 0.475 m, std 0.07 m). 인계 규약은 이번에 바꾸지 않았다.
- v3 test와 A1 test는 seed가 달라 쌍 비교가 아니다. 조건당 9회라 비율 차이의 통계적 확신은 작다.
- 복도 지도의 TOP 가시 범위가 줄었다(차로 0.869, 대피 bay 0.818). 카메라 배치 변경은 사용자 결정이다.
- v3 test와 기준선 loop v2 test는 접촉 프로필이 다르다(`cargo_noslip_v1` / `local_contact_fine`).
- 시뮬레이션 결과이며 실물 MasterPi 검증이 아니다.
