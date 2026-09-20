# ACT 해상도 확대와 조건 선택

사용자 요청에 따라 기존 128/256px × 1/4프레임 비교를 512px까지 확장한다.
원본 train/development 표본의 두 영상은 960×720이며, 같은 bilinear 정사각 축소를 유지한다.
카메라 배치/FOV, 이미지 수, 정적 문맥, 자기 발행 명령, 모델 파라미터, 원본 데이터와 분할,
8000 updates·batch32·두 seed, 정상 물리·weld OFF·종료 기준을 바꾸지 않는다.
최근 4프레임은 기존 두 카메라의 시간 이력이며 새 카메라/정답 센서가 아니다.

## 비교 설계

`protocol.json`: 512px × 1/4프레임 × 두 seed의 추가 4개 학습, 기존 네 배치에서
교사 포함 20회 물리 비교. 기존 128/256 모델 학습과 실행은 원래 소스에 고정해 보존한다.
추가 지원에 따른 기존 모델의 출력 일치와 교사 대조군의 물리 결과를 확인하고 비교한다.

`selection.json`: 현재까지 본 결과와 선택 기준을 실행 전에 기록했다. 기존 네 배치는 이제
조건 선택용이며, 여기서 고른 결과를 새로운 환경에 대한 최종 성능으로 표현하지 않는다.
최소 seed 성공 수 → 전체 성공 수 → 봉 성공 수 → 잘못된 완료 → 접촉 실행 수 → 추론 지연 순으로
조건을 비교한다. 모든 학생 조건이 전체 작업 성공 0회면 사용 가능한 최적이 없다고 보고한다.
성공 후보가 있으면 사전 지정한 새로운 네 오프셋에서 교사 및 128px/1프레임과 별도 확인한다.
실물·새 장면 일반화나 전역 최적을 주장하지 않는다.

## 실행 경계

- 학습: kcm0127 계정의 Colab CLI T4. Drive를 사용하지 않는다.
- 먼저 512px/4프레임 2-update 진단으로 full-data cache/native 및 CPU 배포 오차와 자원을 측정한다.
  GPU/RAM/시간 한계가 있으면 실패를 보존하고 변경 사항을 별도로 기록한다. 배치/정밀도를 묵시적으로 낮추지 않는다.
- source와 protocol은 커밋·해시 검증 후 업로드한다. 프로토콜 경로를 명시해 이전 8개 학습을 다시 실행하지 않는다.
- 원본 canonical data SHA는 `3db730eaf9d827a40b355d7b53188993e14b75d10fd10b9b213a67f830fba167`.
- 기존 학습 소스 `9aef6bb0fb7d05e6b419b2f79b361ebd29099948`, 물리 소스 `e00435b82470e722196858728bc22c1123ef730c`.
- 학습 결과 회수·전체 해시·Mac native 검사 후 물리 실행한다. 기존 36회가 끝난 뒤 추가 코호트를 시작해
  로컬 물리 워커 수 2개와 자원 비교 조건을 유지한다.
- 모델·원본·영상은 로컬 outputs에 보관한다. 요약/해시만 Git에 기록하며 원격 raw 백업을 주장하지 않는다.

```sh
python scripts/colab_carry_bundle.py pack --protocol experiments/2026-09-21-carry-resolution-sweep/protocol.json --output outputs/resolution-sweep/carry-512.zip
# Colab bundle source root에서:
python scripts/run_colab_carry_training.py --dataset /content/ugrp-resolution/data/dataset.json --protocol experiments/2026-09-21-carry-resolution-sweep/protocol.json --out /content/resolution-diagnostic --diagnostic
# 진단 통과 뒤 --diagnostic 없이 새로운 출력 폴더에서 동일 명령.
```

아직 512px의 학습 완료·성능 개선·최적 조건을 주장하지 않는다.


## T4의 512px/4프레임 자원 진단

소스 `64c79057c790493d6ef20ec35c43e36e0dd75cc0`의 전체 데이터 진단에서 1 update 및
개발 평가를 완료했으나 2번째 backward가 CUDA OOM으로 중단됐다. 2.01 GiB 추가 할당을
요청할 때 T4 가용 메모리는 약 160 MiB였다. 샘플링한 GPU 사용량 최고치는 14,753 MiB,
프로세스 트리 RSS 최고치는 약 7.67 GiB다. 본 학습 모델이나 성능 비교 완료를 뜻하지 않는다.
실패 로그·report·환경·자원 추적 파일의 ZIP과 내부 해시를 Mac에서 확인했고
`t4-diagnostic.json`에 기록했다. 배치32/float32와 8000-update 조건을 유지하기 위해
더 큰 GPU의 할당 가능 여부를 확인한다. 신규 GPU의 기종과 수치 오차도 별도로 기록한다.


## T4 메모리 절약 실행

A100과 L4 할당은 계정 quota/entitlement 사유로 backend에서 거절됐다. 배치·정밀도·학습량을
바꾸지 않고 `torch.utils.checkpoint`의 non-reentrant encoder 재계산을 추가했다.
학습 중 각 encoder layer의 activation만 재계산하며, module/state_dict·추론 경로·optimizer·
학습/개발 평가 batch32를 유지한다. 활성화 여부는 report와 resume signature에 기록한다.
CPU의 실제 ACT 배치32 두 update에서 loss·모든 gradient·가중치·RNG가 기존 방식과 정확히 일치했다.
CUDA에서도 동일 검사를 한 뒤 큰 조건을 재검증한다. CPU 검증만으로 CUDA 일치를 주장하지 않는다.

최종 CPU 예측 파일 생성은 batch8로 나눠 같은 전체 표본을 처리한다. 학습/개발 선택에는 영향을
주지 않으며, CPU batch32/8 출력 차이를 검사했다. report에 이 실행 설정을 남긴다.
새 프로토콜의 `execution`은 자원 처리 설정이고, 기존 `training`의 batch32·8000 updates는 그대로다.


추가 메모리 진단은 학습 종료 뒤 전체 train/dev CPU 예측 파일 생성까지 실행하는 **별도 2-step 기능 점검**도 포함한다.
이 진단은 scheduler 총 2 step이며 본 8000-step 코호트나 성능 결과에 재사용하지 않는다.
본 코호트는 두 seed 각각 처음부터 8000 update로 실행한다. 첫 update의 모델/optimizer moment/RNG는
기존 T4 batch32 첫 update와 비교하되, 총 scheduler 길이가 다른 진단의 학습률·scheduler 상태는
동일성 대상에서 제외하고 명시한다. 별도 CPU/CUDA 두-update parity 검사는 같은 scheduler 조건을 사용한다.
