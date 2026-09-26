# 2026-09-26 태그 없는 비전 위치 추정 — 벽·문·문틀 인식(분할) → PF 측정 (Kiro)

- 작성: Kiro(`kiro/zone-vision-loc`, `kiro/`는 Kiro 작업 표시). 2026-09-26. PR #210(마커 없는 탐침)의 후속이다.
- 상태: 설계와 오프라인 검증 결과다. 교사가 몬 렌더 궤적의 개루프 재생이며, 태그 없는 추정으로 로봇을 몬 폐루프 검증이 아니다.
- 실험 기록: [experiments/2026-09-26-vision-loc](../../experiments/2026-09-26-vision-loc/README.md).
- 사용자 결정(2026-09-26): 최종 연구 환경에는 AprilTag가 전혀 없다(벽·문틀·복도 입구·픽업 칸·구역 모두). 환경 v3 벽 프로필 `walls_v3`(0.40 m, `kiro/zone-map-v3`)을 기본 지도(`maps/zones/zone_wide_door.json`)에 적용하고 표식은 0개다. 로봇별 관측 기억과 자연어 공유는 유지한다.

## 0. 결론

> **검토 후 정정(2026-09-26, Codex PR #227 검토; test 채점 뒤 추가).** test s917은 train s903의 재주행이었다(GT 위치 차이 최대 0.01 mm, 같은 JPEG 213장·문 근처 42장). s912·s914의 문 통과 일부도 train과 1 mm 안이었다(`results/overlap_round2_vs_train.json`). 이 test는 오염된 기록으로 남긴다. dev 선택은 기록한 규칙(횡 p99 최소 → w1)과 달리 w6였다(실험 README dev 절). "병목은 인식이 아니라 필터"는 과한 결론이라 고쳤다. "0.4코어"는 근거가 없어 지웠다. 최종 환경은 태그 0개다(사용자 결정). 아래 FAIL은 태그를 다시 넣자는 뜻이 아니다.

1. **태그 없는 비전 추정은 아직 게이트를 넘지 못한다.** 태그 없는 환경의 test 6회를 한 번 채점했다(위 정정: 일부는 train 재주행). 짐을 들고 문 근처를 지날 때 비전 추정은 p90 **7.0 cm**, 횡 p99 **15.3 cm**, yaw p90 2.9°였다. 사전 등록 게이트(door_1 운반 여유에서 유도: 횡 p99 ≤ 6 cm, p90 ≤ 5 cm, yaw p90 ≤ 3°)는 **FAIL**이다.
2. **그래도 큰 진전이다.** 같은 프레임에서 PR #210 손 검출기(벽 높이만 0.40으로)는 48.8 cm, 명령 적분은 89.9 cm였다. 5회 중 4회는 문 근처 횡 p99가 6 cm 이하였다(4.1–5.8 cm).
3. **인식과 필터가 함께 남은 문제다.** 교사 분할 영상을 그대로 넣은 오라클은 p90 6.3 cm였지만 횡 p99는 비전 15.3 cm, 오라클 8.8 cm였고, s916에서는 비전만 횡 28–37 cm 틀렸다(오라클 3–5 cm). 사후 분석(사전 등록 아님)에서 세 가지를 찾았다. (a) 문 평면에서 비전·오라클 모두 같은 방향 약 +3.5 cm 횡 편향(짐이 시야를 가린 채 운동 모델로 지나는 구간), (b) 끼인 로봇(명령은 나가는데 움직이지 않음)에서 재위치 추정이 없어 모든 필터가 길을 잃음(test s915, dev s909), (c) 긴 벽을 따라 운반할 때 비전만 횡 30 cm 발산했다가 문 앞 둘러보기에서 회복(s916).
4. **태그 기준은 다른 환경이다.** M1 tag PF(tags_v2, 0.10 m 벽) 문 근처 p90 3.6 cm, 환경 v3 폐루프 루프 test(tags_v3) 11.5 cm. 비전은 전자에 못 미치고 후자보다 낮다(조건이 달라 참고만).
5. **다음 단계(3차, `kiro/zone-vision-loc-v3`):** 우도의 화면 밖 경계 처리와 방향별 정보량, 증강 MCL 재위치 추정, 짐 운반 운동 모델, train과 독립인 새 test, 태그 없는 추정으로 모는 폐루프 M1. 태그 없는 환경(사용자 결정)에서 대화 연구를 진행한다. 관측 기억·자연어 공유 설계(PR #210 4(b))는 위치 추정 방식과 독립이다.

## 1. 문제와 목표

- 위치 추정은 모든 통신 조건에 같은 "설비"다(PR #210 1절). 태그를 빼면 로봇은 정적 지도의 벽·문 기하와 자기 명령만으로 자세를 추정해야 한다.
- PR #210은 손으로 만든 바닥/벽 경계 검출기로 태그 없이 추정했다. M1 프레임(0.10 m 벽, 태그 있는 영상)에서 문 근처 p90 83 cm였다. 틀린 프레임의 약 70%에서 측정이 틀린 자세를 더 지지했다.
- 이번 목표: 손목 어안 RGB에서 벽·바닥·자기 팔·물체를 **학습한 분할 모델**로 읽고, 그 결과를 지도와 비교하는 측정으로 M1 PF에 넣는다. 문 근처(짐 운반 중) 정확도를 태그 수준에 가깝게 만드는지 본다.

## 2. 환경과 데이터 (태그 없는 렌더)

- **지도:** `zone_wide_door_walls_v3_notags` = 기본 지도 + `walls_v3`(모든 벽 0.40 m) + 빈 태그 목록. 파일은 실험 폴더 `maps/`에 있고 `tagfree_scene.py`가 v3 소스(`kiro/zone-map-v3` `7cedb049`)의 `apply_wall_profile`로 매번 다시 만들어 대조한다. `sim/`·`maps/zones/`는 고치지 않았다(다른 작업 소유).
- **교사 렌더:** M1 실행기(`scripts/run_m1_owncam.py`)를 그대로 쓰고 다음만 바꿨다(교사 예외, `run_vl_teacher_render.py`).
  1. 장면: 태그 없는 장면(`TagFreeZoneScene`).
  2. 자세: 제어기의 자세 원천을 시뮬레이터 정답(`teacher_gt_eval_only`)으로 바꿈. 탐색 시점·접근·운반 구간·문 앞 둘러보기·되보기 등 제어기 논리는 M1 그대로다.
  3. 스킬 v9를 `diagnostic` 모드로 실행(교사 자세 허용).
  4. 자기 손목 카메라를 찍을 때마다 같은 카메라의 MuJoCo 분할 영상(이상적 핀홀)과 참 카메라 자세를 `eval_only/`에 저장.
- **자료 분리:** 학생은 `frames/`, `inputs/commands.jsonl`, `inputs/frames.jsonl`(자세 보고 제거), `inputs/motion_profile.jsonl`(제어기 자신의 조작 단계 전환)만 읽는다. 교사 기록은 `teacher/`, 정답은 `eval_only/`에 둔다. 테스트가 학생 경로의 `eval_only`·`teacher` 접근을 막는다.
- **분할(사전 등록, `episodes.json`):** seed 901–917. train 8회(분할 모델 학습 전용), dev 3회(PF 조정·모델 선택), test 6회(게이트 커밋 뒤 한 번 채점). 배치는 M1 코호트(시작 줄 0.55/−0.85/−2.25, 슬롯 A/B/C, 청록 목표 + 빨강 2 + 초록 1)를 따른다.
- **교사 결과:** 교사 주행도 실패가 있다(SIM 한도, 탐색 실패, 파지 대상 미발견). 실패 회차도 학습·평가 프레임으로 쓰되, 문 근처 지표는 실제로 문을 지난 회차에서만 나온다. 회차별 결과는 실험 README에 있다.

## 3. 방법

### 3.1 인식: 5분류 분할 (재사용)

- `torchvision.models.segmentation.lraspp_mobilenet_v3_large`(LR-ASPP, MobileNetV3-Large; torchvision 0.26.0, BSD-3-Clause), ImageNet 사전학습 백본(`mobilenet_v3_large-8738ca79.pth`). 5분류 머리만 새로 만들고 전체를 미세조정했다.
- 입력: 원시 어안 프레임 → 측정 K·D로 핀홀 복원(PR #210 `undistort`) → 학습 320×240, 추론 480×360(dev 선택, 같은 가중치). 출력 확률을 640×480으로 올린다.
- 분류: 바닥(바닥+구역 도색) / 벽 / 자기 로봇 / 물체(상자·다른 로봇·소품) / 배경. 표적: 교사 렌더의 분할 영상(학습 전용).
- 학습: train 8회에서 3프레임마다(4,973장), AdamW 1e-3 OneCycle, 4 epoch, 좌우 반전·밝기/대비/잡음 증강, MPS.
- **BN 보정:** 증강된 배치의 누적 BN 통계로는 평가 모드에서 벽을 바닥으로 읽었다(dev 벽 IoU 0.74, 같은 프레임을 배치 통계로 돌리면 벽 재현율 0.99). 학습 뒤 증강 없는 train 프레임으로 BN 통계를 다시 모았다("precise BN", fvcore `update_bn_stats`·torch `swa_utils.update_bn`과 같은 절차).

### 3.2 관측: 열별 구간

열 96개(각 5 px 띠)마다, 분할 확률로 **첫 벽의 아래모서리(바닥/벽 경계)가 있는 행의 구간**을 만든다.

| 열에서 본 것 | 관측 |
|---|---|
| 벽 바로 아래 바닥 | 날카로운 경계(부분 픽셀 교차점) |
| 벽과 바닥 사이에 자기 팔·든 상자·다른 로봇·상자 | 구간 [벽 아래끝, 다음 바닥 윗끝] (가림) |
| 벽이 영상 아래까지 | 하한(벽이 아주 가까움) |
| 벽 없이 바닥만 | 상한(보이는 바닥 끝보다 위에 벽) — 빈 공간이라는 음의 증거 |

- 벽 윗모서리(벽 아래, 배경 위)도 같은 방식으로 쓴다. 0.40 m 벽은 카메라(약 0.2 m)보다 높아 대부분 "윗모서리가 시야 위" 상한이 된다.
- 문 개구부·문틀(칸막이 벽 끝)·벽 모서리는 열 사이의 도약과 꺾임으로 나타난다. 별도 "문 검출기"를 두지 않고, 지도 기하가 그 위치를 예측하게 했다.

### 3.3 기대 행: 시선 기반 광선 추적

- PR #210의 열 궤적(`ColumnModel`)과 벽 발자국(`MapGeometry`)을 재사용했다.
- 바꾼 점: PR #210은 바닥 궤적을 **따라** 처음 만나는 벽을 찾았다. 카메라가 숙여져 있어 한 영상 열은 연직면이 아니다. 문틀 옆 열에서는 위쪽 광선이 문틀을 맞히는데 바닥 궤적은 문을 지나간다(합성 렌더에서 기존 식이 최대 400 px 틀림). 벽이 카메라보다 높으면 "카메라에서 바닥점까지의 수평 선분이 벽 발자국을 지나는가"가 정확한 가림 조건이므로, 바닥 궤적 위의 점을 밀어 가며 그 선분이 처음 막히는 지점을 구했다(끝점 진입 + 발자국 꼭짓점 통과 사건, 시야 부채꼴 밖 꼭짓점은 제외).
- 로봇 뒤의 벽은 첫 벽이 되지 않는다(PR #210 식은 0.6 m 뒤에서 시작해 뒤 벽을 잡았다).

### 3.4 우도와 필터

- 입자마다 구간 관측의 확률: 날카로운 경계는 가우스(σ 2.5 px), 구간은 기대 행 + N(0, σ²)가 구간에 들어갈 확률. 이상치 하한(ε), 한 프레임의 열들이 외부 파라미터 오차를 공유하므로 유효 열 수로 누른다(PR #210과 같은 방식).
- 필터는 M1 PF(`owncam_localizer.py`, M1 커밋 `22c84842`, 해시 확인)를 그대로 상속했다. 운동 모델·보정·조작 단계 전환·재표본화도 M1 그대로다. 측정만 바꿨다. 초기값은 자기 도크 ±15 cm/±10°(PR #210과 같음).
- **고정 보정(train에서 GT로 오프라인 적합):**
  - 팔 처짐: 자세·짐별 고도 편향과 카메라 높이(탐색 −0.0170 rad·−1.7 mm, 둘러보기 −0.0231, 운반 −0.0429·−4.4 mm, 짐 든 둘러보기 −0.0448).
  - **짐을 든 채 팬을 돌리면 차체가 반대로 돈다:** −1.44×10⁻⁴ rad/PWM(팬 2030에서 −4.4°), 팬을 되돌리면 복귀한다(잔차 p95 0.002°). 이를 모르면 둘러보기 중 측정이 차체 yaw를 끌어가고 팬 복귀 뒤 약 4° 틀린 채 남는다(dev s910 오라클에서 운반 yaw p90 4.2° → 0.6°).
  - 안정 시간: 자기 팔/팬 명령 뒤 0.2 s 안의 프레임은 쓰지 않는다(첫 0.1 s는 방위 오차 p95 5.9°, 0.2–0.3 s는 0.02°).
- **기준선(같은 필터·같은 보정):** (a) PR #210의 손 검출기(벽 높이만 0.40)와 그 우도, 기하는 3.3절 식, (b) 명령 적분만. **진단(학생 아님):** 교사 분할 영상을 그대로 넣은 오라클.

## 4. 개발(dev) 과정과 선택

- dev는 3회였지만 문을 지난 회차는 s910 하나(문 근처 246 프레임)였다. 설정 선택은 불안정하다. 변형 전체는 `experiments/2026-09-26-vision-loc/dev_variants.json`.
- dev에서 찾은 문제와 수정:
  1. 문틀 열의 기대 행 오류(3.3절) → 시선 기반 광선 추적.
  2. 짐 든 둘러보기 뒤 yaw 약 4° 오차 → 팬-차체 결합 보정.
  3. BN 누적 통계로 평가 모드 분할이 벽을 바닥으로 읽음 → precise BN, seg-v2 재학습.
  4. 학습 크기(320×240) 추론은 경계가 약 3 px 아래로 치우침(1/8 해상도 머리) → 480×360 추론(같은 가중치) + 클래스 유도 부분 픽셀 보정(±6 px) + 이웃 열 일관성 검사(4 px).
- 선택(w6): 480×360, 보정 6 px, 일관성 4 px, 유효 열 8, σ 2.5 px, 안정 0.2 s. dev s910 문 근처·운반 p90 6.9 cm·횡 p99 5.9 cm·yaw p90 4.5°(오라클 6.6 / 5.9 / 1.3).
- 검토 후 정정: 기록한 규칙(횡 p99 최소)대로면 w1(5.53 cm)이었다. 실제로는 문 근처 p90이 두 배 넘게 나쁜 w1·w2(정제 없음)를 미리 기록하지 않은 기준으로 빼고 w3–w7 중 횡 p99 최소인 w6를 골랐다. test 관측 생성 전의 결정이다.

## 5. test 결과 (한 번 채점, `prereg.json` 커밋 `3e82a564` 뒤)

| 필터 | 문 근처·운반 (n=1241) p50 / p90 / p99 | 횡 p99 | yaw p90 | 전체 p50 / p90 | 끼인 s915 제외 전체 p50 / p90 (사후) |
|---|---|---:|---:|---|---|
| **비전(학생)** | **4.0 / 7.0 / 15.9 cm** | **15.3 cm** | **2.9°** | 4.1 / 193.7 cm | 3.1 / 7.5 cm |
| 경계(PR #210 검출기) | 31.4 / 48.8 / 59.3 | 30.5 | 25.5 | 20.7 / 90.3 | 12.8 / 35.9 |
| 명령 적분 | 49.2 / 89.9 / 95.0 | 37.3 | 12.8 | 44.3 / 93.4 | 26.7 / 91.3 |
| 오라클(진단) | 2.8 / 6.3 / 10.9 | 8.8 | 1.2 | 3.6 / 193.1 | 2.3 / 5.8 |

- 게이트: G1 횡 p99 15.3 > 6 cm 실패, G2 p90 7.0 > 5 cm 실패, G3 yaw 2.9 ≤ 3° 통과, G4 문을 지난 회차 5/6 유효 → **FAIL**.
- 회차별 문 근처·운반 비전 p90 / 횡 p99: s912 6.6 / 4.9, s913 6.7 / 5.8, s914 6.6 / 5.4, **s916 11.9 / 17.2**, s917 6.3 / 4.1 cm.
- 거짓 검출(학습한 아래모서리가 교사 라벨 구간에서 5 px 넘게 벗어남) 6.5%, 놓침 1.2%. dev 분할 IoU 바닥 0.993·벽 0.954·물체 0.982.
- 추론 비용(에이전트 잠금 안): 분할 MPS 19.7 ms(CPU 1스레드 86.7 ms) + 열 관측 13.9 ms + PF 측정 47.8 ms. 따로 잰 구성요소 경과 시간 p50의 합(약 81 ms, MPS 대기 포함, 디코딩 제외)이며 전체 경로 지연·CPU 사용량이 아니다.
- 사후(사전 등록 아님): 문 평면(|x−2.2| < 0.15 m) 횡 p90 / 최대 — 비전 5.3 / 6.6 cm, 오라클 5.1 / 6.1 cm, 경계 16.1 / 25.5 cm.
- 표 전체와 태그 기준 표는 실험 README에 있다.

## 6. 재사용·기각한 대안

| 후보 | 판단 |
|---|---|
| torchvision LR-ASPP MobileNetV3 (BSD-3) | 채택. 작은 분할 모델, CPU/MPS 추론 가능, 라이선스 호환 |
| torchvision DeepLabV3 / FCN (BSD-3) | 기각. 같은 역할에 더 무겁다(ResNet 백본) |
| torchvision Keypoint R-CNN(문틀 꼭짓점) | 기각. 문 근처에서만 보이는 점이라 문 밖 구간의 측정이 없다. 분할 + 지도 기하가 문틀·모서리를 함께 다룬다 |
| Ultralytics YOLO (AGPL-3.0) | 기각. AGPL은 네트워크 제공 시 소스 공개 의무가 있어 저장소 라이선스 정책을 정하기 전에 들이지 않는다. 검출 상자는 경계 행을 주지 않는다 |
| Segment Anything (Apache-2.0) | 기각. ViT 인코더가 프레임당 비용이 크고 의미 분류가 없다 |
| OpenCV LSD 선분 | 기각(주 방법으로). 체커 바닥이 선분을 많이 만들고 무늬 없는 벽은 경계 외 선이 없다. PR #210 손 검출기가 같은 한계를 보였다 |
| F3Loc / LaLaLoc 등 평면도 깊이 | 기각. 실내 사진으로 학습된 모델이라 다시 학습해야 하고, 우리 PF·운동 모델과 따로 논다. 분할 + 기하가 같은 1D 거리 정보를 준다 |
| 절대 자세 회귀(PoseNet)·장소 인식 | 기각. 자세 회귀는 영상 검색 수준의 정확도에 머문다는 분석(Sattler 등 2019)이 있고, 두 방이 거의 같아 별칭이 크다 |
| VLM | 기각. 실시간 LLM/API 호출 금지, 프레임당 비용 |
| 운동 모델 재적합(train) | dev 변형으로 시험했으나 dev s910 명령 적분 위치 오차가 커져(p90 1.23 → 2.12 m, yaw p90은 14.2° → 2.6°) 기각. M1 운동 모델 유지 |

## 7. 한계와 위험

- **개루프 재생:** 교사(정답 자세)가 몬 궤적이다. 태그 없는 추정으로 몰면 궤적·둘러보기 빈도가 달라진다. 폐루프 검증이 다음 단계다.
- **시뮬레이터 전용 모델:** 조명·벽색·바닥 무늬가 고정된 렌더로만 학습했다. 실물 카메라에 옮기려면 영역 무작위화(Tobin 등 2017) 또는 실물 라벨이 필요하다.
- **결정론적 시뮬레이터:** 팔 처짐·팬-차체 결합 보정이 거의 오차 없이 맞는 것은 시뮬레이터가 결정론적이기 때문이다. 실물에서는 서보 부하에 따라 더 변한다.
- **교사 실패:** 교사가 문을 지나지 못한 회차는 문 근처 표본을 주지 않는다.
- **대칭:** 두 방이 거의 같아 전역 초기화는 여전히 어렵다(도크 초기값을 썼다).

## 8. 참고 자료

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
- PR #210 설계 문서의 조사(visual sonar, 평면도 PF, RoboCup 가장자리 MCL, VPR 등)를 그대로 이어받는다: [docs/design/2026-09-26-markerless-localization-and-memory.md](2026-09-26-markerless-localization-and-memory.md) 2절.

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

