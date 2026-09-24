# 두 번째 ACT 학습의 별도 최종화 복구

원본 관리 실행 `train-seed24-deployed-first-managed`는 같은 데이터·seed로 8,000/8,000 optimizer update를 약 977.9초에 마쳤지만, 선택된 3,000단계 상태의 캐시 검증에서 종료됐다. 원본 manifest는 `process_failed`/exit 1, 원본 학습 report는 `complete=false`이다. `resume.pt`, report, manifest와 원본 console log는 수정하지 않는다. 개발 에피소드 종료 판정은 선택 상태에서도 조기 종료 2/4·종료 누락 2/4여서 선택 적격성이 없다.

읽기 전용 [원인 자료](cache-failure-diagnosis.json)의 여섯 probe 중 실패는 학습 마지막 행 `native-v25-full:r3:113` 한 곳이다. 기존 전체 8행 ACT 출력의 1e-5 검사에서 미사용 chunk 1–6의 done 좌표 6개만 불일치했고 최대 절대차는 3.8743e-5였다. 배포되는 chunk 0은 기존 1e-5 검사 안이며 종료 임계값 0.65의 결정은 모든 chunk에서 동일했다. 고정 CNN을 원래 데이터 캐시처럼 batch 32로 인코딩하면 수치차가 생겼고, 배포와 같은 history-window batch 4로 인코딩한 probe 특징과 출력은 native와 같았다. [PyTorch 수치 정확도 설명](https://docs.pytorch.org/docs/2.11/notes/numerical_accuracy.html#batched-computations-or-slice-computations)은 배치와 개별 계산의 작은 차이가 가능한 일반 원리만 설명한다. 이번 원인 판단의 직접 근거는 저장된 입력·가중치에 대한 여섯 probe이다. 이 probe가 전체 데이터나 물리 성능을 증명하지는 않는다.

새 [여섯 probe 게이트 재실행](recovery-guard-six-probe-replay.json)은 같은 선택 가중치와 실제 RGB에서 기존 전체 chunk 검사 실패 1건, 새 첫 행동·전체 chunk·종료 결정 검사 통과 6건을 남긴다. 별도의 [네 readback 입력](recovery-memo-readback-preflight.json)에서는 `cache_features=True`로 복원한 배포 worker 모델과 캐시 없는 모델의 raw 첫 행동이 모두 같았고, 종료 결정도 4/4 일치했으며 실제 특징 캐시 hit 64건을 확인했다. 이 네 입력만으로 전 행의 memoization 동등성이나 물리 성공을 주장하지 않는다.

복구 소스의 캐시 검증은 배포 첫 행동 전체 네 좌표에 기존 `atol=rtol=1e-5`를 유지한다. 여덟 chunk의 모든 값은 유한하고 `atol=rtol=1e-4` 안이어야 하며, 각 chunk의 raw done 점수가 0.65를 넘는지의 결정은 전부 같아야 한다. 전체 chunk 허용 수준은 기존 CPU 배포 비교의 1e-4와 같다. 기존 **전체 chunk 1e-5 검사 결과도 별도로 기록**하며, 실패를 통과로 바꾸지 않는다. 선택 상태의 세 학습 probe와 세 개발 probe뿐 아니라, 최종화에서 학습·개발 모든 행의 원본 RGB 배포 경로와 batch 32 캐시 경로를 비교해 같은 세 게이트를 적용한다. 어느 행이든 실패하면 새 artifact는 `complete=false`이고 모델 등록 대상이 아니다. 모든 학습·개발 행의 native CPU 추론 지표와 캐시 지표는 별도 보존한다. 새 지표가 나빠져도 임계값이나 체크포인트를 다시 고르지 않는다.

별도 `act-input-finalization` workflow는 원본의 report·manifest·체크포인트와 소스 동결 파일의 명시적 SHA, 데이터 SHA, 8,000단계 상태, seed 20260924, 3,000단계 선택 상태, 입력 adapter, 보조 종료 목적함수와 가중치, 설치된 ACT 환경, v28 추론 소스의 주요 파일 해시를 확인한다. 원본 선택 텐서의 해시를 모델 읽기 전후 확인한다. optimizer를 만들거나 step을 수행하지 않는다. 새 report의 `complete=true`는 **선택된 진단용 가중치의 내보내기·readback·오프라인 게이트 완료**만 뜻한다. `source_sha`는 복구 실행의 깨끗한 Git SHA, `training_source_sha`와 `inference_source_sha`는 원래 v28 `931d910...`이다. 원본 `process_failed`와 `complete=false`, 완료한 8,000단계·원본 학습 시간, 선택 적격성 실패, 이번 optimizer update 0건을 모두 기록한다. 원본 progress는 새 학습 곡선으로 복제하지 않고 해시로 참조한다. 물리 운반 성공이나 제어기 채택을 주장하지 않는다.

코드 검토·커밋 후 물리 실행이 없는 창에서만 아래 경로를 사용한다. `RECORD`는 새 관리 출력으로 정해 덮어쓰지 않는다. 원본 해시는 [실패 감사](train-seed24-deployed-first-failure-audit.json)와 대조한다. `SOURCE_FREEZE_SHA`는 동결 파일을 별도로 확인해 기입한다.

```bash
UGRP_SIM_PYTHON=/Users/changmin/Project-Runtimes/ugrp/.venv-reference-act/bin/python \
OMP_NUM_THREADS=2 MKL_NUM_THREADS=2 \
bash scripts/open_simulation.command workflow run act-input-finalization \
  --record "$RECORD" --timeout 720 -- \
  --failed-run /Users/changmin/projects/ugrp/outputs/act-action-training-20260924/train-seed24-deployed-first-managed \
  --dataset /Users/changmin/projects/ugrp/outputs/act-action-training-20260924/data/combined-dataset.json \
  --source-freeze /Users/changmin/projects/ugrp/outputs/act-action-training-20260924/refinement-source-freeze.json \
  --expected-manifest-sha256 aa90d237e0c772eff7a87797e76542cf92764b929375fa8d0c74f0c6cae0fedb \
  --expected-report-sha256 eb9c083a14aa9bab9d809fbb0935ffb339ac6324efdf7fd8f9d767a6a043a055 \
  --expected-checkpoint-sha256 546a7cf3c3f761eb269b793e85037c83e00c0dba2cc6248a13cc0fa7e908e6f0 \
  --expected-source-freeze-sha256 "$SOURCE_FREEZE_SHA"
```

복구의 추가 wall time 상한은 720초다. 앞선 8,000단계 학습 약 978초와 합쳐 원래 1,800초 자원 한도 안에 둔다. 원인 분리를 위한 여섯 probe의 약 7초 추론은 복구 작업과 별도 진단으로 기록한다. 이 복구가 실패하면 새 managed 실행과 `complete=false` artifact를 보존하고, 원본을 완료로 수정하거나 허용치를 다시 조정하지 않는다.
