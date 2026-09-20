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
