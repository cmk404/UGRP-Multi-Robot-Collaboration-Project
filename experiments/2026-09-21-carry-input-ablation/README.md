# 공동 운반 ACT 입력 해상도·시간 이력 비교

128/256px × 현재 1시점/최근 4시점을 동일 데이터·동일 학습량으로 비교한다. 두 학습 seed와 네 개의 새 소규모 출발점 조건, 교사 대조군을 사전에 `protocol.json`에 고정했다. 결과가 기록되기 전의 계획이며 성공을 주장하지 않는다.

기존 운반 궤적 6개와 개발 궤적 2개를 재사용한다. 매 조건 8,000 update, batch32, frozen ImageNet ResNet18, transformer64, action chunk8, 기존 개발 점수로 checkpoint를 고른다. 4시점은 같은 로봇·같은 궤적 안에서 현재와 앞선 세 관측을 시간 순으로 사용하며 시작에는 첫 관측을 반복한다. 학생 입력에 교사 행동·측정 관절·정답 좌표·접촉·성공 판정을 추가하지 않는다.

각 프레임을 원래 CNN으로 독립 인코딩한 후 feature map을 가로로 이어 ACT 위치 인코딩에 전달한다. 이미지 자체를 이어 CNN 경계에 가짜 시각 특징을 만드는 방식은 아니다. 시간 순서는 위치 인코딩으로 구별된다. 두 카메라는 기존처럼 별도 입력이다. 문맥은 과거 자기 발행 명령과 고정 작업 설명의 8차원 벡터 네 개다. 1시점 조건은 현재 벡터를 네 번 반복해 학습 파라미터 수와 초기 가중치를 동일하게 맞춘다. 따라서 과거의 8차원 ACT checkpoint와 byte-identical baseline이 아니라, 네 조건 간 architecture를 맞춘 새로운 기준선이다.

반복 명령의 경과 초를 추가하지 않으므로 4시점은 최근 네 번의 판단이라는 의미다. 과거 시뮬레이션 시각과 RGB 참조는 감사에 보관하지만 모델 wire로 보내지 않는다. 외부 worker에는 각 요청에 명시적인 로봇별 이력을 보내므로 두 로봇이 같은 worker를 사용해도 기억이 섞이지 않는다.

실행은 기록 계획을 재생하고 기존 접근·파지·방출, 물리 `local_contact_fine`, weld OFF, 카메라 배치/FOV, 모터 범위·동기화·종료 기준을 유지한다. 외부 LLM 호출은 없다. 실제 처음부터 집어 들어 운반하는 비교이며, 운반 전 중단은 전체 실패에 남기고 운반 조건부 분모에서 분리한다.

새 작은 출발점도 접근 과정에서 수렴할 수 있으므로 운반 진입의 평가 좌표·RGB 차이는 사후 별도 분석한다. 동일 장면 계열의 입력 ablation이며 새 물체·실물·임의 역할·복구 일반화를 주장하지 않는다. 기본 제어기는 바꾸지 않는다.

실행기는 `scripts/ugrp_session.py run <name> -- python scripts/run_carry_input_ablation.py ...`로 소유 세션 안에서 실행한다. 학습은 직렬, 물리 실행은 최대 두 개다. 실행 전 소스를 커밋하고 cohort 동안 고정한다. raw·가중치·영상은 로컬 outputs, 요약·검증·해시는 이 폴더에 보존한다. Drive는 사용하지 않는다.

## 실행 준비 상태 (2026-09-21)

소스 `ab534347d1813c547f077ddf446ee780d140a31d`의 전체 GitHub CI가 통과했다. 전체 로컬 학습 cohort는 시작하지 않았다. 사용자의 별도 Colab 요청에 따라 GPU 학습 패키지 작업에서 학습을 진행하고, 모델 회수 뒤 이 로컬 물리 평가기를 사용한다. Colab 실행 성공이나 운반 성공은 아직 검증되지 않았다.

로컬 256px/4시점 2-update 진단은 첫 학습 update 전 CNN 특징 비교의 `atol=rtol=1e-5`에서 중단됐다. 배치32로 계산한 캐시와 배치4의 배포 경로 사이 float32 수치 오차였다. 원본 첫 32행과 네 조건의 3개 창을 확인한 결과 feature 최대 절대 차이는 0.000341, 전체 action chunk 차이는 0.00000453이었다. 별도 기록은 `numerical-diagnostic.json`에 있다. 실패 원본과 진단 스크립트/출력은 기록된 로컬 경로에 보존하며 백업을 주장하지 않는다.

검증은 feature `atol=5e-4, rtol=1e-5`와 action chunk `atol=rtol=1e-5`를 함께 적용하며 실제 오차를 기록한다. 초기 모델뿐 아니라 개발 점수로 선택한 최종 모델의 train/development 표본에도 적용한다. 임의의 잘못된 캐시를 통과시키지 않는 회귀검사를 포함한다. 이 수치 허용 범위는 모든 GPU·학습 완료 모델의 동등성을 미리 보장하지 않으므로 Colab에서도 실제 검증을 수행해야 한다. 학습량·데이터·입력·카메라·모델 구조는 바꾸지 않았다.

후속 준비 검증(`f7a5c4c`, `readiness.json`): 첫 train/dev 궤적에서 로봇별 16행만 취한 별도 진단 데이터로 256px/4시점 2-update 학습·개발 선택·모델 저장·재로딩·최종 native/cache 비교를 완료했다. 이 작은 진단 모델로 새 MuJoCo 실행에서 파지 후 운반 단계에 진입해 로봇당 2회 판단했다. 총 4개 실제 요청의 전체 RGB/명령 이력·wire 해시·native 추론 재생·실제 발행 명령을 모두 대조했고 사후 평가도 재계산해 일치했다. 카메라 형상 불변, weld 0회, 외부 모델 호출 0회다. 2회 판단 제한으로 의도한 예산 초과 종료이며 성공률 자료가 아니다. 진단용 영상 마지막 프레임도 확인했고 소유 프로세스는 모두 종료됐다.

Colab 회수 모델은 `model-<arm>-s<seed>/report.json`과 `act/{model.safetensors,config.json,adapter.json}` 구조로 보관한다. 로컬 평가기는 dataset·model hash, arm·seed·8,000 update·batch32, native/cache 검증 존재를 확인한다. 경로 재배치 데이터는 원본 manifest 및 파일 해시와의 일치 증거를 별도로 보존해야 한다. 원본 JSON을 바꿔 놓고 해시만 덮어쓰면 안 된다. 학습 SHA와 물리 실행 SHA는 각각 기록한다.

```sh
# 실제 전체 학습 모델 8개를 회수·검증한 뒤 실행. tiny-smoke 모델은 사용하지 않는다.
python scripts/ugrp_session.py run carry-input-final -- python scripts/run_carry_input_ablation.py \
  --out outputs/ablation-final-NEW \
  --reuse-training /absolute/path/to/verified-training \
  --act-python /Users/changmin/Project-Runtimes/ugrp/.venv-reference-act/bin/python \
  --mjpython /Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/mjpython \
  --grasp /Users/changmin/projects/ugrp-worktrees/dispatch-e2e/outputs/dispatch-transfer-t4/models/grasp \
  --stages /Users/changmin/projects/ugrp-worktrees/dispatch-e2e/outputs/dispatch-transfer-t4/models/varied
python scripts/audit_carry_input_ablation.py --out outputs/ablation-final-NEW \
  --act-python /Users/changmin/Project-Runtimes/ugrp/.venv-reference-act/bin/python
```

위 `python`은 기존 `.venv-sim-worker-mac/bin/python`이다. 실행 전에 작업 트리와 학습 provenance를 확인하고 소스를 커밋한다. 본 결과가 없으므로 고해상도나 이력의 유효성을 아직 판정하지 않는다. 향후 해상도만 개선되면 공간 정보 손실, 이력만 개선되면 현재 프레임의 상태 식별 부족을 지지한다. 둘 다 개선되지 않아도 정보 충분성이 증명되는 것은 아니며 데이터 다양성·frozen encoder·종료/협업 구조 등의 대안 원인이 남는다. 두 seed와 같은 장면 계열 네 조건은 광범위한 일반화나 강한 통계적 결론에 충분하지 않다.

계산 비용 참고: 256px/4시점 tiny 모델의 실제 CPU worker 추론은 로봇별 0.161–0.230초, 두 로봇 합계 0.368/0.419초였다(두 라운드만 측정, worker 초기 시작 시간 제외). 환경의 0.2 SIM초 행동 간격은 wall-clock 실시간 처리 보장이 아니다. 성공률 개선과 더불어 추론 지연·전체 wall 시간의 증가를 비교해야 하며, 이 표본으로 안정적인 p95나 배포 실시간성을 주장하지 않는다.
