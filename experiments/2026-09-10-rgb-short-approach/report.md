# RGB 20–30cm 직진 접근과 파지 연결

두 로봇이 각자 RGB로 전진량과 정지를 결정하고, 실제 바퀴로 20–30cm 접근한 뒤 기존 RGB 국소 파지 모델로 물체를 들도록 연결했다. 사전에 저장한 새 거리 20개에서 **학생 19/20, 고정 시간 주행 2/20**이다. 학생의 접근 중 물체 접촉 step은 **0**이며, 사전 기준(≥18/20 및 접촉 0)은 **통과**했다.

## 범위와 제어 경계

같은 장면, seed 11, 같은 물체와 로봇 heading을 유지하고 r1/r3 출발 거리만 각각 20–30cm로 바꿨다. own/top 카메라 위치·FOV·해상도를 유지했다. 초기 fixture 배치 이후 base pose를 직접 쓰거나 GT 접근/IK 정렬을 호출하지 않는다. 접힌 팔로 시작하고 두 CameraRobotPort에 raw wheel 명령을 적용한 뒤 동일 세계의 물리 시간을 한 번만 진행한다.

학생의 모델 입력은 자기 카메라 RGB와 고정 상단 RGB다. 실행기는 자기 명령 이력과 controller phase만 추가로 관리한다. 좌표, 측정 속도, 접촉, 성공 판정은 교사 label 또는 사후 평가 기록으로 분리하며 학생의 전환 판단에 주지 않는다. 매 0.2초 전진/정지를 판단하고, 양쪽 ready 이후 0 속도 명령으로 0.25초 기다린 새 영상 확인을 두 차례 수행한다. `stationary`는 이 명령과 dwell을 뜻하며 측정 qvel 임계값 판정이 아니다. 실패한 확인이나 학습 범위 밖 영상에서는 멈춘다.

확인이 끝나면 팔 전개 명령을 재생하고, 이전 단계의 비선형 RGB 파지 보정을 16회 적용한다. 닫기·들기·hold는 고정 skill이다. weld assistance는 끈다. 이 결과는 일반 경로 탐색, 새 물체/배치, LLM 협업 또는 실물 로봇 성공이 아니다. 이전 파지 단독 결과 20/20+retention14/14는 부모 PR31 기록에 있다.

## 학습과 개발 과정

20, 22.5, 25, 27.5, 30cm의 양쪽 독립 조합 25개에서 privileged teacher가 실제 바퀴로 이동해 접근과 파지에 모두 성공했다. 제외 case는 없다. RGB/label은 총 2872개(로봇별 1436개)이며 ID, 연결, 이미지 해시를 검사했다. 정지 뒤 최대 목표 오차는 3.454mm였다. 모든 34개 교사 실행(초기 실패 파일럿 포함)의 판정을 10Hz 로그에서 재계산했다([teacher-audit.json](teacher-audit.json)).

각 로봇은 400개의 균형 표본을 골라 풀링한 grayscale/Sobel 영상 특징, PCA 24차원, Gaussian kernel ridge로 전진값과 stop label을 학습한다. 모든 stop 표본과 궤적 전체를 덮는 표본을 유지하며, 목표 영상 anchor를 더한 회귀 support는 401개다. 신경망이나 LLM 학습이 아니다. 사례별 4-fold 선택을 쓰지만 PCA / bandwidth 후보를 전체 학습 pool에서 구성하므로 CV MSE는 완전히 독립된 검증 점수가 아니다.

초기 교사 파일럿은 gain 0.8에서 관성으로 지나쳐 0/3, gain 0.2에서는 너무 작은 명령으로 12–13mm 전에 멈춰 0/3이었다. 최소 이동 명령 0.01을 더한 교사 파일럿은 3/3이었다. 이 9개 결과를 모두 보존한다. 교사 gain 변경은 교사 전용이며 학생에 좌표 규칙을 넣지 않았다.

첫 학생 파일럿은 20cm만 성공해 1/3이었다. 30cm와 비대칭의 두 번째 이동 뒤 영상이 PCA residual gate에 걸렸다. 기존 표본 400개만으로 정한 범위가 정상 교사 프레임 20/1436개를 로봇별로 잘못 거부했고, 모두 회귀 표본에서 제외된 프레임이었다. 같은 시점의 교사 영상 residual과 실패 학생 영상 residual은 거의 같았다. 회귀와 PCA는 유지하고 범위 보정에 전체 1436개 학습 RGB를 사용하도록 수정했다. 기존 1.25배 여유, nearest/effective support 검사도 유지했다. 개발/검증 영상으로 범위 임계값을 맞추지 않았다. 수정 뒤 파일럿 3/3이 실제 접근·파지에 성공했고, 그 뒤 새 20개를 처음 실행했다. 학습 wall은 v1 5.01초, v2 17.14초다.

## 고정 시간 비교군과 새 20개 결과

비교군은 forward 0.1을 일정 시간 적용하며 RGB 판단을 계산하되 동작에는 쓰지 않는다. 25cm 학습 궤적의 명령 적분으로 2.2초를 추정했지만 실제 25cm 파일럿은 33.15mm 짧아 파지에 실패했다. 물리 이동량으로 보정한 2.4초 파일럿은 4.905mm 앞에 멈추고 파지·2.7초 유지를 성공했다. **새 20개를 보기 전에 2.4초로 고정**했고, 모든 거리와 양쪽 로봇에 같은 값을 사용했다.

| 조건 | 전체 접근·파지 성공 | 평균 접근 SIM초 | 평균 wall초/실행 | 접근 판단·wheel 명령 수 | 파지 판단 수 | 접근 접촉 step |
|---|---:|---:|---:|---:|---:|---:|
| RGB 학생 | 19/20 | 11.53 | 19.34 | 2324 | 608 | 0 |
| 고정 2.4초 주행 | 2/20 | 2.90 | 10.59 | 600 | 640 | 0 |

판단·명령 수는 두 로봇 합계이며 0 속도 확인 명령도 포함한다. 접근 SIM시간에는 정지 확인 0.5초가 포함된다. wall은 렌더링·초기화·저장 포함 로컬 관측값이다. 학생은 목표를 보며 감속하므로 고정 속도보다 느리다. 전체 40회 wall은 660.21초다. 외부 모델 API 호출 및 추론 토큰 비용은 0이다.

성공은 접근 완료와 접근 중 접촉 0에 더해, 물체가 시작 높이보다≥3cm 높고 두 로봇 모두 양쪽 손가락 접촉을 weld 없이≥2초 연속 유지할 때만 인정한다. 거리 20개는 10개 대칭 및 10개 독립 비대칭이며 원래 [validation-cases.json](validation-cases.json)에 선언했다. 각 조건의 초기 qpos/qvel/sim time은 정확히 일치했다. 초기 RGB는 byte 일치 또는 decoded channel 최대 차≤3/평균 차≤0.001의 좁은 허용 범위로 확인했다. 관측 성공률을 다른 물체/배치에 대한 보장으로 해석하지 않는다.

전체 case 결과는 [results.csv](results.csv), 요약은 [summary.json](summary.json)에 있다. 학생 실패 case: approach-heldout-16. 고정 주행 실패 case: approach-heldout-01, approach-heldout-03, approach-heldout-04, approach-heldout-05, approach-heldout-06, approach-heldout-07, approach-heldout-08, approach-heldout-09, approach-heldout-10, approach-heldout-11, approach-heldout-12, approach-heldout-13, approach-heldout-14, approach-heldout-15, approach-heldout-16, approach-heldout-18, approach-heldout-19, approach-heldout-20.

유일한 학생 실패인 heldout-16은 r1/r3 출발 거리가 29.9/29.12cm였다. 목표 약 2–3mm 앞까지 갔지만 첫 0.25초 정지 확인 뒤 r1의 전진 예측이 0.0030898로 허용값 0.003을 조금 넘었다. stop score는 0.6913으로 통과했어도 두 조건을 모두 만족해야 하므로 파지를 시작하지 않았다. 이 실패를 본 뒤 임계값을 바꾸거나 재시도해 성공률에 합치지 않았다. [실패 판단 기록](heldout-16-failure.json)과 [해당 영상](heldout-16-video-review.jpg)을 보존한다.

## 검증·재현·보관

교사 수집과 첫 모델/pilot/calibration source는 360e747fe8d126d0b42693276f0632b4d8989b3d이다. 최종 모델·pilot·40회 비교 source는 `7701d1d719d6ecce42de2101a38cc47e8591e244`다. 실행 중 clean source SHA와 모든 model artifact hash를 고정했다. 전체 비교 후 모델이나 임계값을 수정하지 않았다. 최종 source의 로컬 436 tests + 140 subtests가 통과했으며, GitHub offline/Ubuntu simulation4개 check도 통과했다. CI는 실제 학생 성공률을 대신하지 않는다.

런타임 감사는 RGB/model/skill 해시, 실제 저장된 이미지에서의 예측, 명령 이력, wheel 실행 trace, 두 차례 확인 전이, nested grasp 명령 및 10Hz 성공 판정을 확인한다. 접근 물체 접촉은 매 물리 step 계측하지만 감사는 event/count의 내부 일관성을 검사하며 MuJoCo 접촉 자체를 독립 재구성하지 않는다.10Hz 기록은 초기 folded setup 뒤부터 시작해 접근·팔 전개·파지·hold를 포함한다. 대표 영상의 시작/중간/끝과 실제 이동·정지·들기 모습을 직접 확인했으며 모든 영상의 전 프레임 수동 검토는 아니다.

최종 모든 40회와 수정 파일럿 3회는 같은 source에서 재감사해 모두 통과했다. 총 48회 기록에서 접근 판단 3,458개와 파지 판단 1,440개를 검증했다. 이전 source의 학생 파일럿 3회와 비교군 보정 2회는 해당 source에서 완료한 감사 기록을 보존한다. [full-audit.json](full-audit.json)에 구분한다. [검증 영상 프레임](validation-video-review.jpg), [개발 단계 영상](development-video-review.jpg), [교사 영상](teacher-video-review.jpg)을 함께 저장한다.

각*-raw-manifest.json은 모든 raw 파일의 위치·크기·SHA256과 어느 ZIP에 포함됐는지 기록한다. ZIP에는 모든 결과/평가/모델, 전체 교사 학습 RGB, 개발 파일럿 RGB, 최종 대표 case RGB를 보관한다. 각 ZIP은 압축 해제 읽기 검증을 했다. 원본 MP4와 생략한 최종 회차 RGB는 `/Users/changmin/projects/ugrp/outputs/rgb-approach-*`에 로컬 보관하며 원격 백업이 아니다. 여러 shard로 나눈 학습 ZIP은 같은 빈 폴더에 모두 풀면 trainer 입력 구조를 복원한다. 최종 모델 ZIP은 별도의 빈 폴더에 푼다. source SHA checkout과 부모 PR31의 grasp model을 사용해 protocol/cases 및 저장된 cohort command대로 재현한다.

PR32: https://github.com/kcm0127-dotcom/ugrp/pull/32 . 부모 PR31 위에 쌓인 작업 브랜치이며 명시적인 병합 승인 전에는 main에 병합하지 않는다. 프로젝트 예외에 따라 Google Drive를 사용하지 않는다.
