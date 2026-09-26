# 2026-09-26 태그 없는 비전 위치 추정 (Kiro)

**질문(사용자, 2026-09-26).** "벽과 문기둥에 표식을 남기지 않고, vision을 이용해서 인식하게 하면 안되나?" 최종 연구 환경은 AprilTag가 전혀 없는 환경 v3(`walls_v3` 0.40 m 벽 + 기본 지도)이다. 손목 어안 RGB의 비전 인식만으로 문 근처(짐 운반 중) 위치를 태그 수준에 가깝게 추정할 수 있는가?

**답(이 오프라인 검증의 범위 안에서).** 아직 문틀·벽 태그를 뺄 근거로는 모자란다. 태그 없는 test 6회를 한 번 채점했다. 짐을 들고 문 근처를 지날 때 p90 **7.0 cm**, 횡방향 p99 **15.3 cm**, yaw p90 2.9°였다. 사전 등록 게이트(횡 p99 ≤ 6 cm, p90 ≤ 5 cm, yaw p90 ≤ 3°)는 **FAIL**이다(G1·G2 실패, G3 통과). 같은 프레임의 PR #210 경계 검출기는 48.8 cm, 명령 적분은 89.9 cm였다. 비전 인식은 교사 라벨을 그대로 넣은 오라클(6.3 cm)과 거의 같다. 남은 오차는 인식이 아니라 필터 쪽이다: 짐 운반 중 운동 모델의 횡 편향, 끼인 로봇에서의 재위치 추정 부재, 긴 벽을 따라갈 때의 발산. 참고로 태그 기준은 다른 환경이다: M1 frames(tags_v2, 0.10 m 벽) 문 근처 p90 3.6 cm, 환경 v3 루프 test(tags_v3) 문 근처 p90 11.5 cm(폐루프).

- 작성: Kiro(`kiro/zone-vision-loc`, `kiro/`는 Kiro 작업 표시). PR #210(마커 없는 탐침, main 병합됨)의 후속. 설계·결과 문서: [docs/design/2026-09-26-vision-localization-tagfree.md](../../docs/design/2026-09-26-vision-localization-tagfree.md).
- 성격: 태그 없는 환경의 **교사 주행 렌더를 오프라인으로 재생**해 위치 추정만 평가했다. 추정으로 로봇을 몬 폐루프 결과가 아니다. 새 LLM/API 호출 없음.

## 입력 경계

| 입력 | 학생(위치 추정) | 비고 |
|---|---|---|
| 자기 손목 프레임 `frames/*.jpg` | 사용 | 원시 어안 JPEG |
| 자기 발행 명령 `inputs/commands.jsonl` | 사용 | 운동 모델, 서보 상태, 짐 상태, 안정 시간 |
| 자기 조작 단계 전환 `inputs/motion_profile.jsonl` | 사용 | M1 PF와 같은 운동 프로필 전환 |
| 정적 지도 `maps/zone_wide_door_walls_v3_notags.json` | 사용 | 벽·문·경계, 태그 0개 |
| 고정 보정 | 사용 | K·D, M1 운동 모델(해시 확인), `calibration_train.json`(train GT로 오프라인 적합) |
| 분할 모델 가중치 | 사용 | train 렌더로 학습, 해시 고정 |
| `teacher/`(교사 자세 보고·사건) | 사용 안 함 | 테스트로 차단 |
| `eval_only/`(GT 궤적, 분할 라벨, 카메라 자세) | 사용 안 함 | 학습 표적·보정 적합(train)·오라클 진단·채점만 |

## 재현

```sh
# 0) v3 소스(렌더 전용, 희소 worktree): git worktree add --detach ../kiro-vision-loc-v3src 7cedb049
# 1) 교사 렌더(최대 2 sim, 스레드 1, df >= 30 GiB, 기계 전체 sim < 6 대기)
experiments/2026-09-26-vision-loc/run_render.sh kiro-vl-renderA <out>/render vl-dev-s909,vl-train-s901,...
# 2) 보정(train), 학습(train, dev 검증), 관측(학생 입력만), 필터, 채점
PY=.venv-sim-worker-mac/bin/python; TPY=/Users/changmin/Project-Runtimes/ugrp/.venv-reference-act/bin/python
$PY  experiments/2026-09-26-vision-loc/vision_loc_cli.py calibrate --episodes vl-train-s901 ... --settled-s 0.3 --output calibration_train.json
$TPY experiments/2026-09-26-vision-loc/vision_loc_cli.py train --output <out>/model/seg-v2 --epochs 4 --every 3
$TPY experiments/2026-09-26-vision-loc/vision_loc_cli.py segment --episodes <eps> --checkpoint <ckpt> --output <out>/obs-v2
$PY  experiments/2026-09-26-vision-loc/vision_loc_cli.py oracle --episodes <eps> --output <out>/oracle-obs     # 진단 전용
$PY  experiments/2026-09-26-vision-loc/vision_loc_cli.py localize --episodes <eps> --calibration calibration_train.json \
       --config selected_config.json --filters vision,boundary,deadreck,oracle --obs <out>/obs-v2 --oracle-obs <out>/oracle-obs --output <est>
$PY  experiments/2026-09-26-vision-loc/vision_loc_cli.py score --episodes <eps> --estimates <est> --obs ... --oracle-obs ... --output metrics.json
```

test 에피소드는 `prereg.json`이 있고 동결 파일·모델·설정·보정 해시가 모두 같을 때만 실행된다(`require_frozen`).

## 데이터: 태그 없는 교사 렌더 17회

M1 실행기를 그대로 쓰고 장면(태그 0)·자세 원천(GT 교사)·분할 라벨 저장만 바꿨다(`run_vl_teacher_render.py`). 소스는 `kiro/zone-map-v3` `7cedb049`(희소 detached worktree `ugrp-wt/kiro-vision-loc-v3src`)에 고정했다. 동시 sim 2개, 스레드 1, 시작 전 기계 전체 sim 수(< 6)·디스크(≥ 30 GiB) 확인, 부하는 `launch_load.txt`와 각 `teacher_manifest.json`에 있다.

| 분할 | 에피소드 | 교사 결과 | SIM s | 프레임 | wall s | 부하(시작 → 끝) |
|---|---|---|---:|---:|---:|---|
| train | vl-train-s901 | 슬롯 배치 성공 | 274.4 | 1458 | 614 | 56.8 → 70.6 |
| train | vl-train-s902 | SIM 한도 | 720.5 | 3596 | 2227 | 14.0 → 86.9 |
| train | vl-train-s903 | 성공 | 296.6 | 1583 | 1077 | 70.6 → 134.1 |
| train | vl-train-s904 | 성공 | 416.8 | 2177 | 1722 | 86.9 → 61.6 |
| train | vl-train-s905 | 탐색 실패 | 201.1 | 999 | 806 | 134.1 → 66.8 |
| train | vl-train-s906 | 성공 | 385.7 | 2029 | 1244 | 61.6 → 24.1 |
| train | vl-train-s907 | 성공 | 372.6 | 1956 | 1185 | 66.8 → 24.1 |
| train | vl-train-s908 | 파지 대상 안 보임 | 205.4 | 1114 | 703 | 24.1 → 28.9 |
| dev | vl-dev-s909 | SIM 한도(접근점에서 끼임) | 720.5 | 3596 | 2314 | 5.2 → 56.8 |
| dev | vl-dev-s910 | 성공 | 405.9 | 2122 | 1052 | 5.1 → 14.0 |
| dev | vl-dev-s911 | 정적 keep-out 보호 정지 | 154.5 | 810 | 401 | 24.1 → 22.1 |
| test | vl-test-s912 | 성공 | 350.8 | 1846 | 1456 | 22.1 → 24.3 |
| test | vl-test-s913 | 성공 | 363.6 | 1912 | 1470 | 28.9 → 105.7 |
| test | vl-test-s914 | 성공 | 378.8 | 2012 | 1185 | 24.3 → 14.5 |
| test | vl-test-s915 | SIM 한도(탐색 중 끼임) | 720.5 | 3596 | 2016 | 105.7 → 73.2 |
| test | vl-test-s916 | 성공 | 388.1 | 2036 | 1121 | 14.5 → 60.8 |
| test | vl-test-s917 | 성공 | 296.6 | 1583 | 835 | 73.2 → 39.5 |

- 교사 성공 11/17. 실패 회차의 프레임도 학습·평가에 쓴다. 문 근처 지표는 문을 지난 회차에서만 나온다(test 5회, dev 1회).
- 첫 렌더 시도 2개는 30초 뒤 멈췄다(실행기 이름을 `run_` 접두어로 바꿔 기계 전체 sim 수에 잡히게 함). 부분 폴더는 `render-aborted-rename/`에 보존했다.

## 개발(dev) 과정 — 전부 기록 (`dev_variants.json`)

| 변형 | 내용 | dev 문 근처·운반 p90 / 횡 p99 / yaw p90 |
|---|---|---|
| 오라클 eff 8 / 12 / 12-disc / 20-disc | 교사 라벨, 유효 열 수와 템퍼링 방식 (s910 + train s903) | 5.4/5.8/1.1, 5.4/6.0/1.0, 7.8/8.5/1.4, 8.2/8.9/1.5 cm·° |
| seg-v1 | 증강 배치의 BN 누적 통계 → 평가 모드에서 벽을 바닥으로 읽음(dev 벽 IoU 0.74) | 폐기, seg-v2(precise BN)로 재학습 |
| vision 320×240 | 학습 크기로 추론: 경계가 약 3 px 아래로 치우침(1/8 해상도 머리) | 24.7 / 11.5 / 16.8 (s910) |
| w1 / w2 | 480×360 추론, eff 8 / 12 | 16.1/5.5/4.8, 14.9/6.8/5.0 |
| w3 / w4 | + 클래스 유도 부분 픽셀 보정 6 px, eff 8 / 12 | 6.9/6.1/4.2, 7.7/7.8/4.2 |
| w5 | w3 + 10 px 미만 바닥 조각 제거 | 7.4/6.3/2.0 |
| **w6 (선택)** | **w3 + 이웃 열 일관성 4 px** | **6.9/5.9/4.5** |
| w7 | w3 + 둘 다 | 7.2/6.9/4.2 |
| 경계(PR #210 검출기) / 명령 적분 / 오라클 | 같은 필터·보정 | 29.2/13.2/7.8, 60.8/17.5/12.4, 6.6/5.9/1.3 |
| 운동 모델 재적합(train) | `fit_motion` | 명령 적분 p90 1.23 → 2.12 m로 악화, 기각 |

- 선택 규칙(변형 결과를 보기 전 기록): dev 문 근처·운반 횡 p99 최소 → p90 → 전체 p90. dev에서 문을 지난 회차는 s910 하나(246 프레임)라 선택이 불안정하다.
- dev에서 찾아 고친 모델 오류: 팬-차체 yaw 결합(짐 든 둘러보기 뒤 약 4° 오차), 문틀 열의 기대 행(최대 400 px), 로봇 뒤 벽을 첫 벽으로 잡는 문제.

## 사전 등록 (`prereg.json`, 커밋 `3e82a564`, test 실행 전 push)

- 학생 동결: 모델 sha256 `34853903…39fd9`, `calibration_train.json`, `selected_config.json`(=w6), 모듈 해시. 운동 모델은 M1 그대로.
- 게이트(door_1 운반): 문 폭 0.50 m, 운반 외형 ±0.15 m → 한쪽 여유 0.10 m. yaw 3°면 외형 앞 모서리(0.20 m 앞)가 1.0 cm 흔들리고, 경로 추종·접촉 여유 약 3 cm를 빼면 횡 위치 오차에 6 cm가 남는다. PASS = 문 근처·운반 프레임(test 풀링)의 횡 p99 ≤ 6 cm **그리고** 위치 p90 ≤ 5 cm **그리고** yaw p90 ≤ 3°, 문 근처·운반 프레임이 20개 이상인 test 회차 ≥ 3.

## test 결과 (6회, 12,985 프레임, 한 번 채점)

| 필터 | 전체 p50 / p90 | 문 근처·운반 (n=1241) p50 / p90 / p99 | 횡 p99 | yaw p90 | 운반 p50 / p90 | 무부하 p50 / p90 |
|---|---|---|---:|---:|---|---|
| **비전(학생)** | 4.1 / 193.7 cm | **4.0 / 7.0 / 15.9 cm** | **15.3 cm** | **2.9°** | 3.2 / 7.5 cm | 6.6 / 197.2 cm |
| 경계(PR #210 검출기) | 20.7 / 90.3 | 31.4 / 48.8 / 59.3 | 30.5 | 25.5 | 17.8 / 37.4 | 26.2 / 93.3 |
| 명령 적분 | 44.3 / 93.4 | 49.2 / 89.9 / 95.0 | 37.3 | 12.8 | 64.1 / 94.8 | 34.8 / 90.7 |
| 오라클(진단, 학생 아님) | 3.6 / 193.1 | 2.8 / 6.3 / 10.9 | 8.8 | 1.2 | 2.2 / 4.9 | 6.9 / 197.2 |

게이트: G1 횡 p99 15.3 cm > 6 cm **실패**, G2 p90 7.0 cm > 5 cm **실패**, G3 yaw p90 2.9° ≤ 3° 통과, G4 문을 지난 회차 5/6(유효). 판정 **FAIL**.

회차별 문 근처·운반(비전 p90 / 횡 p99, 오라클 p90 / 횡 p99, cm): s912 6.6/4.9, 5.0/5.0 · s913 6.7/5.8, 5.9/5.4 · s914 6.6/5.4, 4.8/4.7 · s916 **11.9/17.2**, 9.6/9.3 · s917 6.3/4.1, 5.8/6.1 · s915 문 도달 없음.

- 횡 p99가 6 cm를 넘은 것은 s916 하나 때문이다. 나머지 4회는 모두 6 cm 이하였다.
- 전체 p90 1.9 m는 s915(아래 사후 분석)의 끼임 때문이다. 오라클도 같다.
- **거짓 검출:** 학습한 날카로운 아래모서리 중 교사 라벨 구간에서 5 px 넘게 벗어난 것 6.5%(810,810개 중), 놓친 것 1.2%. 윗모서리는 14.4% / 7.8%.
- **분할 품질(dev, 320×240 입력):** 화소 정확도 0.993, IoU 바닥 0.993·벽 0.954·물체 0.982·배경 0.798.
- **추론 비용**(에이전트 잠금 안, M3, 부하 약 3.5, `results/inference_cost.json`): 480×360 분할 MPS 19.7 ms(CPU 1스레드 86.7 ms), 열 관측 13.9 ms, PF 측정 갱신(입자 2000 × 열 96) 47.8 ms → 프레임당 약 81 ms(MPS). 5 Hz에서 로봇당 CPU 약 0.4코어.

### 태그 기준(다른 환경, 참고)

| 기준 | 환경 | 전체 p50 / p90 | 문 근처 p50 / p90 |
|---|---|---|---|
| M1 tag PF(실행 중 기록, PR #210 채점) | tags_v2, 0.10 m 벽, M1 test 6회 | 2.0 / 7.0 cm | 1.8 / 3.6 cm |
| 환경 v3 루프 test tag PF(원본에서 다시 계산) | tags_v3, 0.40 m 벽, 폐루프 9회 | 2.9 / 11.1 cm | 2.7 / 11.5 cm (운반 3.1 / 12.4) |

궤적·환경·루프 방식이 달라 같은 조건의 비교가 아니다.

### 사후 분석 (test 뒤 작성, 사전 등록 아님: `posthoc.py`, `results/posthoc_*.json`)

- **문 평면(|x−2.2| < 0.15 m, 운반, n=340):** 비전 횡 p90 5.3 / 최대 6.6 cm, 오라클 5.1 / 6.1 cm, 경계 16.1 / 25.5 cm, 명령 적분 27.9 / 30.1 cm. 비전과 오라클 모두 모든 회차에서 약 +3.5 cm의 같은 방향 횡 편향이 있다(카메라 횡 오프셋은 1 mm 미만으로 확인). 짐이 시야 아래를 가린 채 문을 지나는 구간의 운동 모델 편향으로 본다.
- **끼인 로봇:** s915(test)와 s909(dev)에서 교사 로봇이 약 600 SIM s 동안 거의 움직이지 않았는데 자기 명령은 계속 나갔다(마지막 300 s 이동 3.7 cm, 바퀴 명령 2,995개). 모든 필터(오라클 포함)가 명령을 적분해 길을 잃었다. 재위치 추정(센서 재설정, 증강 MCL 등)이 없다. s915를 빼면 비전 전체 p50 / p90은 3.1 / 7.5 cm, 오라클 2.3 / 5.8 cm, 경계 12.8 / 35.9 cm다.
- **s916:** 문 앞 칸막이 벽을 따라 북쪽으로 운반하는 동안 비전만 횡 28–37 cm 틀렸다(오라클 3–5 cm). 문 0.6 m 앞 둘러보기에서 2 cm로 돌아왔다. 문 근처 구역(문 0.6 m 앞부터)의 횡 p99를 이 구간이 만든다.

## 권고

- **벽·문틀 태그를 지금 빼지 않는다.** 태그 없는 비전 추정은 사전 등록한 door_1 운반 게이트를 넘지 못했다(문 근처 p90 7.0 cm·횡 p99 15.3 cm, 기준 5 / 6 cm).
- **다만 방향은 맞다.** 같은 프레임에서 손 검출기(48.8 cm)의 약 1/7이다. 인식은 오라클 수준이다(7.0 대 6.3 cm). 희소 태그 환경 v3의 폐루프 기록(문 근처 p90 11.5 cm)보다도 낮다(조건이 달라 참고만).
- **태그 제거 전에 필요한 것(필터 쪽):** (1) 짐 운반 중 운동 모델의 횡 편향 보정 또는 운반 자세의 측정 추가(문 평면 +3.5 cm), (2) 끼임·미끄러짐에서 되살아나는 재위치 추정, (3) 긴 벽을 따라갈 때의 발산 방지(s916), (4) 태그 없는 추정으로 로봇을 모는 폐루프 M1 검증.
- 게이트를 넘으면 벽·문틀·복도 입구 태그를 모두 뺀 환경(사용자 결정)에서 대화 연구를 진행할 수 있다. 관측 기억·자연어 공유는 위치 추정 방식과 독립이다.

## 파일

| 파일 | 내용 |
|---|---|
| `tagfree_scene.py`, `maps/zone_wide_door_walls_v3_notags.json` | 태그 없는 walls_v3 지도(sha256 `61d35247…`)와 장면 glue |
| `run_vl_teacher_render.py`, `run_render.sh`, `episodes.json` | 교사 렌더(분할 라벨·카메라 GT), 실행기, 사전 고정 분할 |
| `vision_loc.py` | 열 구간 관측, 시선 기반 기대 행, 우도, M1 PF 하위 클래스, 학생 재생 |
| `seg_model.py` | LR-ASPP 학습·precise BN·추론 |
| `vision_loc_cli.py` | calibrate / fit-motion / train / segment / oracle / localize / score / bench, test 동결 검사 |
| `calibration_train.json`, `selected_config.json`, `dev_configs/`, `dev_variants.json`, `summarize_dev.py` | 보정, 선택 설정, dev 변형 전체 |
| `prereg.json`, `make_prereg.py` | 게이트와 동결 학생 |
| `model_manifest.json`, `model_train_info.json` | 모델 해시·출처(Release는 보류) |
| `results/` | test 지표, 결과·게이트, 추론 비용, 사후 분석, TensorBoard 검증 |
| `build_results.py`, `build_tensorboard.py`, `posthoc.py`, `grid.sh` | 결과 모음, 스냅샷, 사후 분석, 오프라인 격자 |
| `references.md` | 참고 자료 |
| `tests/test_vision_loc.py` | 19개(합성 라벨 기하, 문틀 열, 구간, 필터 추적, 안정 게이트, 입력 경계, 분할·렌더 분리, torch 시 분할기) |

- 원본(로컬 전용, 원격 백업 아님): `/Users/changmin/projects/ugrp/outputs/vision-loc-20260926/`(렌더 672 MB, 모델, 관측, 추정, 격자). 모델 `model/seg-v2/seg_lraspp_mbv3.pt`(13.1 MB, sha256 `348539030fda962cc5ba64e21c956619bc4db9321a99cd3ae6746611a9939fd9`). **GitHub Release 등록은 보류**(`docs/model_artifacts.md` 절차 미수행).
- TensorBoard: `outputs/tensorboard/0926-vision-loc`(31 run: 에피소드 6 × 필터 4, 풀링 4, 태그 참조 2, 학습 1). EventAccumulator 재독 684건·공용 서버(127.0.0.1:6006) HTTP 660건 대조, 불일치 0. 보기 설정 `outputs/tensorboard-view.json`의 `vision_loc_20260926`.

## 검토 범위와 한계

- 교사가 몬 궤적의 개루프 재생이다. 태그 없는 추정으로 몰았을 때의 궤적·둘러보기는 다르다.
- 시뮬레이터 렌더(고정 조명·재질)로만 학습·평가했다. 실물 이전은 검증하지 않았다.
- 보정(팔 처짐, 팬-차체 결합)은 결정론적 시뮬레이터라 거의 오차 없이 맞았다. 실물에서는 더 변한다.
- dev에서 문을 지난 회차가 1회뿐이라 설정 선택이 불안정하다.

## 참고 자료

### 논문 (직접 확인한 것만)

- A. Howard, M. Sandler, G. Chu, L.-C. Chen, B. Chen, M. Tan, W. Wang, Y. Zhu, R. Pang, V. Vasudevan, Q. V. Le, H. Adam, "Searching for MobileNetV3", ICCV 2019. https://arxiv.org/abs/1905.02244 — LR-ASPP 분할 머리와 백본(채택).
- L.-C. Chen, G. Papandreou, F. Schroff, H. Adam, "Rethinking Atrous Convolution for Semantic Image Segmentation" (DeepLabv3), 2017. https://arxiv.org/abs/1706.05587 — 비교 후보(기각).
- F. Boniardi, A. Valada, R. Mohan, T. Caselitz, W. Burgard, "Robot Localization in Floor Plans Using a Room Layout Edge Extraction Network", IROS 2019. https://arxiv.org/abs/1903.01804 — 학습한 배치 가장자리 + 평면도 PF(이번 설계의 가장 가까운 선행 연구).
- C. Chen, R. Wang, C. Vogel, M. Pollefeys, "F³Loc: Fusion and Filtering for Floorplan Localization", CVPR 2024. https://arxiv.org/abs/2403.03370 — 평면도 깊이 + 필터(기각: 재학습 필요).
- T. Sattler, Q. Zhou, M. Pollefeys, L. Leal-Taixé, "Understanding the Limitations of CNN-based Absolute Camera Pose Regression", CVPR 2019. https://arxiv.org/abs/1903.07504 — 자세 회귀 기각 근거.
- A. Kendall, M. Grimes, R. Cipolla, "PoseNet: A Convolutional Network for Real-Time 6-DOF Camera Relocalization", ICCV 2015. https://arxiv.org/abs/1505.07427 — 자세 회귀 후보(기각).
- A. Kirillov 외, "Segment Anything", ICCV 2023. https://arxiv.org/abs/2304.02643 — 범용 분할 후보(기각: 비용).
- R. Grompone von Gioi, J. Jakubowicz, J.-M. Morel, G. Randall, "LSD: a Line Segment Detector", IPOL 2012. https://www.ipol.im/pub/art/2012/gjmr-lsd/ — 선분 후보(기각).
- J. Tobin, R. Fong, A. Ray, J. Schneider, W. Zaremba, P. Abbeel, "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World", IROS 2017. https://arxiv.org/abs/1703.06907 — 실물 이전 한계의 근거.
- PR #210 설계 문서의 조사(visual sonar, 평면도 PF, RoboCup 가장자리 MCL, VPR 등)를 그대로 이어받는다: [docs/design/2026-09-26-markerless-localization-and-memory.md](../../docs/design/2026-09-26-markerless-localization-and-memory.md) 2절.

### OSS·라이브러리

| 이름 | 버전 | 라이선스 | 재사용한 것 |
|---|---|---|---|
| torchvision ([github.com/pytorch/vision](https://github.com/pytorch/vision)) | 0.26.0 | BSD-3-Clause | `lraspp_mobilenet_v3_large` 모델 정의, `MobileNet_V3_Large_Weights.IMAGENET1K_V1` 백본 가중치(`mobilenet_v3_large-8738ca79.pth`, sha256 `8738ca79…a04`) |
| PyTorch | 2.11.0 (MPS) | BSD-3-Clause | 학습·추론, DataLoader |
| OpenCV (`opencv-python-headless`) | 4.13.0 (학습 환경) / 5.0.0 (sim 환경) | Apache-2.0 | 어안 복원 `fisheye.initUndistortRectifyMap`, JPEG/PNG 입출력 |
| MuJoCo | 3.12.0 | Apache-2.0 | 교사 렌더의 분할 영상(`Renderer.enable_segmentation_rendering`) |
| PythonRobotics ([github.com/AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)) | b2020cd (M1 PF가 인용) | MIT | M1 PF 구조(예측·가중치·저분산 재표본화) — M1 localizer를 통해 간접 상속 |
| TensorBoard | 2.21.0 (sim 환경) | Apache-2.0 | 스냅샷 보기(저장소 `scripts/tensorboard_tools/export.py` Writer로 이벤트 파일 작성) |
| Ultralytics YOLO | — | AGPL-3.0 ([라이선스](https://www.ultralytics.com/license)) | 사용 안 함(라이선스·출력 형식 이유로 기각) |

### 내부 모듈·PR

- PR #210 `experiments/2026-09-26-markerless-probe/markerless_probe.py`: `ColumnModel`(열 궤적), `MapGeometry`(벽 발자국·슬랩 광선 추적), `undistort`, `load_m1_localizer`/`load_m1_calibration`(해시 확인), 손 검출기 `detect_boundaries`/`boundary_loglik`(기준선).
- M1 PF `harness/owncam_localizer.py` @ `22c84842`(PR #201, sha256 `0304d7c4…`), M1 보정 `experiments/2026-09-26-zone-m1-owncam/calibration_m1_dev.json`.
- M1 실행기·제어기(교사 렌더): `scripts/run_m1_owncam.py`, `harness/m1_owncam_delivery.py`, `harness/wrist_zone_skill_v9.py`(PR #181 고정본) @ `kiro/zone-map-v3` `7cedb049`.
- 환경 v3 벽 프로필 `sim/zone_arena.py` `WALL_PROFILES['walls_v3']`/`apply_wall_profile`, `sim/zone_scene.py` `ZoneScene`(PR #208, `kiro/zone-map-v3`).
- 운동 모델 재적합 변형(기각): `scripts/eval_owncam_localization.py` `fit_motion`.
- TensorBoard: `scripts/tensorboard_tools/export.py` `Writer`; 빌더 구조는 PR #210 `build_tensorboard.py`.
- 태그 참조: PR #210 `results/metrics_test.json`(M1 test, tags_v2), `kiro/zone-map-v3` 환경 v3 루프 test 원본(`outputs/zone-env-v3-20260926/loop/test`, 기록 커밋 `007949bf`).

### 문서·웹 페이지

- torchvision LRASPP 문서: https://pytorch.org/vision/stable/models/lraspp.html
- MuJoCo Python 렌더링(분할 렌더): https://mujoco.readthedocs.io/en/stable/python.html
- Ultralytics 라이선스: https://www.ultralytics.com/license
- 저장소 `docs/model_artifacts.md`(모델 배포 절차), `docs/tensorboard.md`.

