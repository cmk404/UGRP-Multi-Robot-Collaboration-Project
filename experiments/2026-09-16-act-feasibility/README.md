# ACT 확대 학습과 실행 가능성 검증

2026-09-16. PR #58의 0/3 파일럿을 후속 학습으로 검토했다. **새 거리쌍20개에서 기존 커널20/20, ACT seed16 20/20, ACT seed17 20/20 접근·파지 성공이다.**

## 무엇을 학습했는가

- 출력은 전진 명령과 정지 준비 점수다. ACT가 팔·집게·운반 전체를 학습한 결과가 아니다.
- ACT가 RGB로 접근/정지를 결정한 뒤 기존 공통 파지 스킬로 넘긴다. 기존 커널도 같은 파지 스킬을 사용한다.
- 각 학생은 자기 RGB와 고정 공용 top RGB만 받는다. 실제 좌표·측정 관절·접촉·평가는 학습 교사/평가 출력에만 있으며 학생 추론에 전달하지 않는다.
- 카메라 pose/FOV, 로봇/물체 외관, 물리 환경을 유지하고 weld를 끈다. 20~30cm의 고정 장면 직선 접근에 대한 결과다.

## 데이터

기존 성공 시연 25개 중 20개를 학습, 이전에 이미 본 5개를 개발용으로 전환했다. 새 교사의 실제 연속 주행 30개를 추가했으며 30/30 접근·파지 성공, 접근 중 물체 접촉 0, 제외 0이었다. 교사의 성공을 학생 성공에 합산하지 않는다.

최종 학습은 50개 궤적이며 로봇별 2,870개 프레임 + 같은 목표 RGB anchor 1개다. 개발은 5개 궤적, 로봇별 290프레임이다. 정지 표적은 학습 원본에서 r1 169개, r3 187개다. 개발 정지 프레임은 두 로봇 합계36개, 이동 프레임은544개다.

30개 수집에 534.17초가 걸렸다. 교사 수집 SHA와 학습/실행 SHA는 `361695c31569fb2004304fbcafe6cf0b5738fc7a`다. 원본 데이터/교사 설정/원본 영상은 로컬 `outputs/teacher-expand30-v1`에 있다.

[protocol.json](protocol.json)의 새 거리쌍20개는 추가 학습 이전에 커밋했다. 최종 시험 영상/정답으로 모델·체크포인트·임계값을 선택하지 않았다. 개발/최종 조건에서 같은 궤적 프레임을 임의 분할하지 않았다. 같은 장면 내 새 거리쌍이므로 새 물체·조명·회전·실물에 대한 일반화로 해석하지 않는다.

## 학습 방법

공식 LeRobot ACT의 고정 커밋 `89236ea0f4f81a81ca566081e20dd1ff5f823cbe`와 앞선 두 줄 RGB-only device 호환 패치를 사용한다. Transformer/VAE 및 masked L1+KL은 upstream 구현이다.

- torchvision 공식 ImageNet ResNet18을 초기화하고 영상 인코더를 고정한다. RGB128×128과 ImageNet mean/std 전처리를 명시한다.
- 학습 동안만 고정 CNN 특징을 재사용한다. 원 영상 경로와 캐시 경로의 출력 및 행동 헤드 기울기가 정확히 같고 예외 시 원래 CNN이 복원되는 것을 테스트했다. 배포 워커에는 캐시/교사/표적을 전달하지 않고 원본 JPEG 두 장을 넣는다.
- 전체 파라미터는 로봇당11,418,898개, 실제 학습 파라미터는251,986개다. Frozen vision 설정이지 전체 모델 fine-tuning/원 논문 설정 재현이 아니다.
- Transformer64, chunk4, 매 새 영상에서 첫 행동1개 실행, batch32, AdamW1e-4→cosine1e-5, KL1, CPU2 threads. 준비 기준 `stop_score>=0.65 AND forward<=0.003`은 변경하지 않았다. stop_score는 보정된 확률이 아니다.
- 균형 샘플링은 정지/느린 접근/나머지 접근의 세 그룹에 같은 추출 질량을 준다. 미래 chunk의 패딩은 기존대로 마스킹한다.
- 개발 점수 `정지 누락률 + 10×잘못된 준비 판정률 + 전진 MAE/0.15`로 로봇별 체크포인트를 선택한다. 마지막 step을 무조건 선택하지 않는다.
- 최종 학습은 seed20260916/20260917 두 번, 각 로봇10,000회 업데이트다. 실행 중 모델을 보정하거나 실패 사례를 제외하지 않는다.

## 원인 진단과 개발 실행

|설정|학습 궤적|업데이트/로봇|개발 정지 누락|개발 잘못된 준비|실제 실행|
|---|---:|---:|---:|---:|---|
|이전 무작위 CNN 파일럿|20|300|36/36|0/544|0/3, 이전 기록|
|고정 ImageNet CNN, 일반 샘플링|20|3,000|26/36|0/544|미실행|
|같은 설정, 균형 샘플링|20|3,000|0/36|1/544|개발 조건3/3|
|확대 균형 학습 seed16|50|10,000|0/36|0/544|최종 결과 별도|
|확대 균형 학습 seed17|50|10,000|0/36|1/544|최종 결과 별도|

일반/균형3000회 대조는 같은 영상 인코더·모델·데이터·학습 횟수로 샘플링을 비교했다. 이전300회 파일럿과 확대 모델의 차이에는 사전학습·해상도·데이터·학습량 등 여러 요인이 섞여 있으므로 이를 특정 요인 하나의 효과로 단정하지 않는다.

개발3회는 이전에 사용한 거리쌍이다. 새 조건 성공률과 합산하지 않는다. 영상3개에서 시작/중간/끝9프레임을 확인했다. 이미지 참조888개가 모두 해시 일치했고 시작/끝 weld OFF였다.

첫 가중치 다운로드는 Python urllib DNS 오류로 학습 전에 중단됐다. 공식 URL에서 curl로 다시 받고 SHA256 `f37072fd47e89c5e827621c5baffa7500819f7896bbacec160b1a16c560e07ec`을 검증한 뒤 동일 소스로 새 출력 폴더에 재실행했다. 실패 로그도 보존한다.

## 학습 비용과 체크포인트

seed16의 전체 학습/커널 적합/특징 계산/평가 wall은17.99분, seed17은17.45분이었다. 일부 병렬 실행되어 이를 경과 시간으로 단순 합산하지 않는다. 순수 업데이트와 중간 평가/읽기 확인 구간은 로봇 합계 seed16약739.81초, seed17약679.57초다. 실제로 관측한 Mac 실행 시간이며 일반적인 학습 시간 보장이 아니다.

선택된 step은 seed16 r1/r3 모두9500, seed17 r1=9500/r3=8500이다. 마지막10000회 모델이 더 나쁜 개발 정지 판정을 내린 경우도 있어 개발 체크포인트 선택이 필요했다. 외부 LLM 호출/토큰은0이며 클라우드 GPU 환경을 사용하지 않았다.

## 최종 물리 시험

|정책|접근·파지 성공|평균 접근 SIM초|평균 접근 명령 수(두 로봇 합계)|평균 전체 wall초|
|---|---:|---:|---:|---:|
|기존 커널, 같은50개 시연으로 재학습|20/20|11.47|115.7|16.82|
|ACT seed20260916|20/20|11.55|116.5|22.40|
|ACT seed20260917|20/20|11.40|115.0|22.62|

60회 모두 런타임 오류·timeout 없이 종료했고 접근 중 물체 접촉0이었다. 두 학습 초기값의 커널 산출물이 byte-identical이므로 기존 방식20회를 공통 기준으로 사용했다. ACT40회는 **같은20개 조건을 두 학습 초기값으로 반복한 것**이며 독립된40개 장면으로 세지 않는다. 첫20쌍은 정책 순서를 번갈아 실행하고, 두 번째 초기값20회는 뒤이어 직렬 실행했다.

[첫 초기값 전체40회](final-seed16-summary.json), [두 번째 초기값20회](final-seed17-summary.json), [전체 감사](input-audit.json).

20개 조건 모두 세 비교군의 초기 qpos/qvel 등 물리 상태와 첫 own/top RGB가 정확히 같았다. 이미지 참조17,728개, 원본 파일13,838개의 해시가 일치했다. 제어기 명령6,944개를 모두 재현했고, 각 로봇/실행의 시작·중간·끝에서 모델 예측360개를 원본 RGB와 저장 체크포인트로 재현했다. 모델의 모든 호출을 재추론한 것은 아니다. 시작/끝 weld OFF와 설정 OFF를 확인했다.

case01/10/20의 세 정책 영상9개에서 시작/중간/끝27프레임을 직접 검토했다. 접근 후 파지·들기가 지표와 일치했다. 전체 프레임을 사람이 검토했다고 주장하지 않는다. [case01](qa/case-01.jpg), [case10](qa/case-10.jpg), [case20](qa/case-20.jpg), [프레임 원본·해시](qa/review-samples.json).

seed17 case10은 r1 준비 완료 후 r3 접근 동안4회 대기했다. 준비 여부는 ACT 예측이지만 대기와 공동 진입은 기존 공통 제어기의 규칙이다. 이를 ACT가 의사소통/협력을 학습했다는 증거로 해석하지 않는다.

시뮬레이션 시간·명령 수는 작업량 비교에 사용한다. 전체 wall은 워커 초기화, 모델 로딩, 렌더링, IPC와 동시 작업 영향을 포함한다. 첫 비교 초반에는 두 번째 모델 학습도 진행 중이었다. 따라서 정밀 추론 속도 벤치마크가 아니다. 관측된 전체 wall에서는 ACT가 더 느렸고 성공률/명령 수 우월성은 확인되지 않았다.

## 재현 및 보관

로컬 raw RGB/영상/체크포인트와 원격 Git의 코드·요약·해시를 구분한다. raw 자료는 현재 원격 백업되지 않았다. Drive는 사용하지 않는다. 현재 기본 커널과 상위 계획/통신은 유지한다. 학습 코드와 선택 가능한 ACT 어댑터를 제공하며 main 병합은 별도 승인 대상이다.


## 판단과 남은 범위

- 좁은 직선 접근에서는 ACT가 영상으로 준비 시점을 학습해 실제 과제를 수행할 수 있다는 근거를 확보했다. 이전0/3으로 학습 가능성을 부정할 수 없었다.
- 균형 샘플링 대조와 개발 체크포인트 선택이 유효했다. 마지막 학습 손실만 보고 모델을 채택하면 정지 판정이 나쁜 모델을 고를 수 있었다.
- 기존 커널도20/20이며 실제 처리 시간이 짧았다. 현재 기본값을 ACT로 교체할 근거는 부족하므로 기본 커널을 유지한다. ACT의 학습 범위 밖 입력 거부 기능도 아직 없다.
- 다음 확장은 각도·옆 방향 보정/복구를 포함한 **실제 연속 교사 궤적**과 그에 맞는 출력 공간이다. 정적 reset 이미지들을 연속 ACT 시연으로 이어 붙이지 않는다. 이후 팔/집게/하중 운반은 각각 별도 학습·실행 시험이 필요하다. 이 보고서는 그 성능을 검증하지 않았다.

## 실행 예시

이 worktree에서 기존 ACT 전용 환경을 사용한다. 원본 시연/파지 모델은 아래 로컬 경로에 있어야 하며 fresh clone만으로 실험 데이터까지 제공되는 것은 아니다. 새 실행마다 출력 경로와 세션 이름을 바꾼다.

```sh
python3 scripts/ugrp_session.py run act-train-NEW -- \
  /Users/changmin/Project-Runtimes/ugrp/.venv-reference-act/bin/python \
  scripts/train_act_feasibility.py \
  --teacher-dirs /Users/changmin/projects/ugrp/outputs/rgb-approach-teacher-20260910-v1 outputs/teacher-expand30-v1 \
  --protocol experiments/2026-09-16-act-feasibility/protocol.json \
  --sampling balanced --steps 10000 --seed 20260916 --out-dir outputs/act-NEW

python3 scripts/ugrp_session.py run act-physics-NEW -- \
  /Users/changmin/Project-Runtimes/ugrp/.venv-reference-act/bin/python \
  scripts/run_reference_approach_pilot.py \
  --mjpython /Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/mjpython \
  --act-python /Users/changmin/Project-Runtimes/ugrp/.venv-reference-act/bin/python \
  --comparison-dir outputs/act-NEW \
  --grasp-model-dir /Users/changmin/projects/ugrp/outputs/grasp-recovery-model-20260910-v1 \
  --protocol experiments/2026-09-16-act-feasibility/protocol.json --out-dir outputs/act-physics-NEW
```

새 환경의 선택 의존성/두 줄 device 패치는 [레퍼런스 문서](../../docs/reference_alignment.md)를 따른다. 사전학습 가중치는 torchvision 공식 URL `https://download.pytorch.org/models/resnet18-f37072fd.pth`이며 위 SHA256과 일치해야 한다. 추론 체크포인트 로드는 가중치를 다운로드하지 않는다.

## 검증

- 실제 upstream backward·캐시 출력/기울기 동등성·예외 복원·체크포인트·RGB 워커 입력 경계 관련10개 테스트 통과.
- 초기 실행 코드의 Ubuntu CI 네 종류 모두 통과. 최종 기록 커밋 CI는 PR checks에서 확인한다.
- 학습·교사·개발/최종 물리·감사 세션은 완료 시 정리한다. 다른 작업 프로세스를 종료하지 않는다.
- [원본 위치와 해시](raw-locations-and-hashes.json), [교사 전체 기록](teacher-report.json), [개발 실행](development-physics-summary.json), [초기 다운로드 실패](development-setup-event.json)를 보존한다.
