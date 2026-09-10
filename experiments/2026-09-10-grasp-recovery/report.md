# 비선형 RGB 국소 파지 복구

이전 선형 학생이 실패한 큰 복합 자세 오차를 privileged teacher로 복구시킨 뒤, 자기 카메라와 고정 상단 카메라에서 보정 명령을 학습했다. 사전 선언한 새 자세 **20개 전부 성공**했고, 기존 14개도 전부 성공했다. 이전 선형 모델은 같은 새 자세에서 7/20, 보정 없는 시연 재생은 8/20이다. 접근 범위를 확장하기 위한 gate(새 자세 ≥18/20, 기존 14/14)를 통과했다.

## 범위와 데이터

- 장면/seed 11/단일 직육면체/로봇 위치/카메라 설치 위치 및 FOV는 유지했다. weld assistance는 꺼져 있다.
- 학생 보정 함수는 현재 own RGB와 fixed shared top RGB만 받는다. 실행기는 자기 발행 명령에 한 번 최대 25 pulse 보정량을 더한다. GT, IK, 관절 측정, 접촉 및 성공 판정은 학생 입력이 아니다.
- 이 단계의 초기 접근, 닫기와 들기는 기존 교사 명령 재생이다. 학습된 전체 접근 정책, LLM 협업, 새 배치/물체 일반화 또는 하드웨어 성공으로 해석하지 않는다.
- 학습/개발/검증/유지 조건은 source `523873613dc3a7aab942baf565cc86fb167dfb34` 전에 선언했다. [protocol.json](protocol.json), [training-cases.json](training-cases.json), [validation-cases.json](validation-cases.json), [retention-cases.json](retention-cases.json)을 참조한다.
- Teacher 32/32 physical success, actor RGB/label pair 316개(각 로봇 158개)다. 모든 sample ID, pairing, 원본 RGB SHA-256, 물리 scoring을 검증했다. 목표 명령까지 남은 전체 3관절 pulse 차이를 교사 label로 저장한다. 성공만 골라 실패를 지운 데이터셋이 아니며, 이번 32개 모두 성공해 제외 case가 없다.
- 풀링한 grayscale/Sobel RGB 특징과 목표 영상 차이를 PCA 24차원으로 줄이고 Gaussian kernel ridge inverse model을 학습했다. reference goal 0-label anchor를 추가하고 support/residual 기반 범위 검사를 적용한다. 이 모델은 신경망 또는 LLM 학습이 아니다.
- trajectory case를 묶은 4-fold 선택을 사용했다. 다만 PCA와 bandwidth 후보는 전체 training pool에서 구성했으므로 CV MSE는 완전히 독립된 검증 점수가 아니다. r1/r3 MSE 약 806.2/1230.2는 적합도 진단이며 성공률이 아니다. 학습 wall time은 1.54초다. 인접 RGB와 반복 goal anchor는 상관된 표본이다.
- 개발 pilot 3/3은 goal zero와 과거 실패 두 자세이며 모두 이미 학습에 포함돼 있다. 아래 새로운 20개 rollout과 구분한다. 최종 20개를 본 뒤 모델/하이퍼파라미터를 수정하지 않았다.

## 실험과 결과

최종 비교 source는 `9d5872c35b2e4f5d7ddda1e54e9f2db09e2d8000`, 학습/교사/pilot은 `523873613dc3a7aab942baf565cc86fb167dfb34`이다. 둘 사이의 변경은 비교 검사와 output-only 초기 물리 상태 기록이며 actor/model/control은 바꾸지 않았다. 실험 중 소스는 clean SHA로 고정했다.

새 20개는 기존 실패 부근 대칭 8개, 넓은 대칭 8개, 양쪽 오차가 다른 비대칭 4개다. 새 물체 배치가 아니라 고정 장면의 새 팔 명령 오차다. 난수 seed와 모든 조건은 실행 전에 저장했다.

| 조건 | 새 자세 성공 | 평균 wall s/run | 보정 arm actions 합계 | 판단 수 |
|---|---:|---:|---:|---:|
| 비선형 RGB recovery | 20/20 | 5.49 | 1543 | 640 |
| 이전 선형 RGB student | 7/20 | 5.64 | 1899 | 640 |
| 보정 없는 시연 재생 | 8/20 | 5.54 | 0 | 640 |

매 실행 복구 16회×0.45초, 로봇별 16판단, 동일 닫기/들기/2.2초 유지 예산이다. playback도 공정한 시간 비교를 위해 예측을 계산하고 보정만 적용하지 않는다. API 호출 및 토큰 비용은 0이다. wall 시간은 렌더링/저장/초기화 포함 로컬 관측값이다. 전체 60회 wall은 394.05초였다.

새 모델은 모두 최대 7.5686–7.7226cm 들어 2.2초 연속 유지했다. 판정은 높이 ≥3cm, 두 로봇 모두 bilateral finger contact, weld 없음이 ≥2초 연속 유지될 때만 성공이다. 단순 재생 heldout-15는 1.7초 유지해 실패이며, heldout-20은 일부 상승했지만 양쪽 접촉 유지가 없어 실패다.

기존 6개 작은 오차와 8개 큰 복합 오차의 retention도 14/14다(85.82초). 기존 12개 성공을 보존하고 과거 두 실패 `[90,-90,-90]`, `[90,-90,7]`도 각각 2.2초 유지했다. 20/20 관측값을 일반적인 성공 확률 100%나 다른 물체/배치에 대한 보장으로 표현하지 않는다.

## 비교 검사 수정과 검증

첫 validation v1은 heldout-03 linear에서 byte-exact RGB 비교가 중단됐다. r1 own JPEG의 2,073,600 channel 값 중 111개가 달랐고 최대 차이 2/255, 평균 절대 차이 0.00005498/255였다. top/r3 RGB와 simulation time은 일치했다. 이 중단 실행 8개와 원본은 별도로 보존하며 최종 성공률에 합치지 않는다.

v2는 매 조건의 초기 qpos/qvel/sim time 정확 일치와 동일 시작/종료 시간(5.382..16.472 SIM초)을 확인한다. JPEG는 byte가 다를 때만 decoded channel 최대 차이 ≤3, 평균 차이 ≤0.001의 매우 작은 오차를 허용한다. 최종 60개에서 비정확 pairing은 heldout-05 linear의 r1 own 한 장뿐이며 max1/mean0.00002604였다. source/skill/fixture/perturbation와 output-only 물리 상태는 동일했다. 모델을 재학습하지 않고 20×3 전체를 새로 실행했다.

CI에서 새 pairing 테스트가 Pillow를 요구해 offline dependency 환경에서 import 실패했다. 이미 설치되는 OpenCV로 decode를 바꿨고, 전체 40쌍 비교 결과가 기존 Pillow 계산과 같은지 원본 데이터로 다시 검증해 모두 일치했다. 이 변경은 actor나 물리 동작을 바꾸지 않는다. 로컬 최종 414 tests + 140 subtests 통과(7.13초). 최종 GitHub CI는 별도 확인 기록을 남긴다.

사전 pilot, 중단 v1, 최종 60회, retention 14회 총 85회/2720개 판단을 모두 재검증하고 보존한다. [full-audit.json](full-audit.json)은 저장된 원본 RGB/모델에서 예측 dict와 실제 arm action/자기 명령 이력을 재계산하고, evaluation-only 표본으로 성공 판정을 재계산한 결과다. 대표 영상은 teacher, 개발 실패 복구 2개, heldout-01 새 모델/선형, heldout-20 새 모델/playback의 첫/중간/마지막 프레임을 직접 검토했다. 학생은 물체를 양쪽에서 들어 유지했고 실패 대조군은 바닥에 남거나 한쪽만 올라갔다. 모든 영상 전체 프레임을 전수 수동 검토한 것은 아니다.

[최종 비교 프레임](validation-video-review.jpg) · [과거 실패 복구](pilot-video-review.jpg) · [Teacher QA](teacher-video-review.jpg)

## 저장과 다음 단계

각 `*-raw-manifest.json`에 모든 raw 파일 경로/크기/SHA-256과 ZIP 포함 여부를 기록한다. `*-evidence.zip`에는 모든 결과/판정/모델, teacher RGB 전체, 선택한 대표 실행의 RGB 전체를 보관한다. 원본 mp4와 생략된 회차 RGB는 `/Users/changmin/projects/ugrp/outputs/grasp-recovery-*`에 로컬 보관하며 원격 백업이 아니다. ZIP 자체와 내용의 읽기 검증을 수행한다. 프로젝트 예외에 따라 Drive를 사용하지 않는다.

PR: https://github.com/kcm0127-dotcom/ugrp/pull/31 (명시적 병합 승인 전에는 병합하지 않음). 조건부 승인된 다음 작업은 팔을 접고 20–30cm 실제 wheel approach를 수행한 뒤, 영상으로 정지를 판단하고 이 파지 단계로 연결하는 별도 curriculum이다.
