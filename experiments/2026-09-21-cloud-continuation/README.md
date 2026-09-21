# ACT·Jev 후속의 Colab/Kaggle 재개

사용자가 두 기존 작업의 후속을 Colab/Kaggle에서 이어 달라고 요청했다. 기존 Mac 원본은 보존하고 새 원격 실행을 별도 코호트로 기록한다. 이 문서는 실행 착수 기록이며 성능 결론이 아니다.

## 재개 시 확인

- 128/256px ACT 학습 모델 8개는 회수되어 있다. 기존 Mac 물리 비교는 36회 중 결과 18개를 남기고 중단됐다. source `e00435b`의 원본 결과를 이번 원격 결과와 한 코호트로 합산하지 않는다.
- 512px 본 학습은 시작되지 않았다. 기존 2-update 기능 진단 중 Colab 세션이 목록에서 사라졌으며 원인은 확정되지 않았다. 캐시 텐서 보유 수정은 현재 main에 포함되어 있다.
- Jev skill 후보 `150554b`의 replacement holdout은 108개 중 결과 7개만 존재했다. 이전 `7dce220`의 32개 진단 결과는 별도 보존한다. 두 결과를 최종 228회 완료로 보고하지 않는다.
- 이 작업 시작 시 Colab 활성 세션 없음, Kaggle GPU 잔여 30h를 조회했다. 이후 상태는 변할 수 있다.

## 실행과 보존

- Colab T4 세션: `ugrp-resume-512-20260921`, ACT 실행 SHA `d13cda20834d039184bf90d5d9c817c1579b494c`.
- 순서: 전체 자료 512px/h4 2-step export 진단 → 512px/h1,h4 × 두 seed 각 8000 updates → 128/256px 기존 8모델+교사 36회 → 512px 4모델+교사 20회. 단계 실패 시 이후 단계는 실행하지 않는다. 전체 원격 작업은 23시간 한도다. 진단은 학습량과 scheduler가 다르므로 본 모델로 재사용하지 않는다.
- Colab의 시뮬레이션과 ACT Python 환경은 분리한다. NumPy/OpenCV 버전 차이를 유지하고 LeRobot 고정 소스 및 기존 device-only patch를 검증했다. batch32, float32, 카메라/FOV, 학생 입력, weld OFF를 유지한다.
- 원본 canonical dataset SHA: `3db730eaf9d827a40b355d7b53188993e14b75d10fd10b9b213a67f830fba167`. 이전 bundle의 자료를 사용하되 실행 소스는 새 Git snapshot이다. 경로 재배치는 canonical/root map/파일 해시를 검증한다.
- Kaggle CPU private/offline: `changmin2026/ugrp-simulation-ed4ad53401e0`, version1, SHA `06c4150364ea14f70cc10c9802446ed278723dde`. 새 holdout의 rule 기준선 12배치 × 3반복=36회, 2 worker. 제공자 종료 한도 1800초. 동일 물리 반복이며 독립 배치 36개가 아니다.
- Jev/Gemini 모델 비교는 인증/원격 endpoint가 미확보라 제출하지 않았다. 로컬 Gemini 주소를 공개하거나 임의 외부 터널을 만들지 않았다.
- 이 작업의 원격 실행에는 외부 모델 호출이 없다. 실제 청구액은 확인 전 null이며 무료 실행 또는 비용 0으로 단정하지 않는다.

로컬 원본/상태 위치:
`/Users/changmin/.codex/worktrees/act-cloud-resume/ugrp/outputs/cloud-resume/`
및 `outputs/kaggle-jev-rule-20260921/`.

`collection-status.json`은 수집 상태, `continuation.json`은 단계 상태다. 완료 ZIP은 분할 회수 후 ZIP/내부 모든 파일의 해시와 source SHA를 확인한다. 회수·해시 확인은 과제 성공이나 모델 성능 감사와 다르다. 새 물리 코호트가 끝난 뒤 엄격 성공·접근 실패·거짓 완료·시간·명령 수·추론 지연과 입력/영상 감사를 수행해야 한다. 선택된 조건의 새 오프셋 확인도 아직 실행하지 않았다.

수집기는 이 작업의 세션만 종료하며 Colab 자동 재접속/재할당·작업 재제출은 하지 않는다. 23.5시간 수집 한도가 있다. Mac에서는 업로드·수집·단위 검사만 수행한다. 자료는 로컬과 비공개 실행 사본으로 보존하며 Drive를 사용하지 않는다. 원본 전체의 영구 원격 백업을 주장하지 않는다.

## 코드와 검사

- 기존 Jev 후보를 별도 작업 브랜치에 통합했다. 충돌은 CI 테스트 목록에서 양쪽 항목을 보존해 해결했다. main은 변경하지 않았다.
- Kaggle 인자에서 원격 Python 경로를 전달하도록 하고 ACT 평가기가 검증된 데이터 경로 재배치를 허용하도록 수정했다.
- `run_cloud_carry_continuation.py`에 진단→학습→물리 평가 및 실패 보존을 연결했다.
- 로컬 오프라인 CI: 1195 passed, 3 skipped, 184 subtests passed. 관련 묶음 51개, 경로/프로토콜 관련 추가 실행 23개와 18개 통과(중복 포함, 합산하지 않음).
- actual GPU forward/export, 모델 학습 완료, 물리 결과 회수와 시각 검토는 각 상태/후속 기록에서 별도로 확인한다.

## 비용 우선순위 변경 — 2026-09-21 11:45 KST 이후

사용자의 512px 계산 비용 우려에 따라 512px 전체 학습과 이후 512px 평가는 보류했다. 앞서 제시한 10~20시간은 실제 512px 본 학습 속도를 측정한 예측이 아니므로 성능/시간 근거로 사용하지 않는다. 진단의 2-update는 완료했지만 export/cache 검증이 끝나지 않은 상태에서 중지했다. 본 8000-update 학습은 시작하지 않았다. 중지 시점 원본 및 체크포인트는 보존하고 별도 interruption manifest와 압축본을 만든다.

Colab에서는 이미 회수·검증한 128/256px 8모델+교사 36회 평가를 먼저 수행하도록 전환한다. 학습·모델·프로토콜은 바꾸지 않고 새 출력 `outputs/cloud-priority/physics128256`을 사용한다. 이 평가의 실행 한도는 4시간, 수집 한도는 4.5시간이다. 이는 완료 예상이 아닌 중단 한도다. Kaggle 규칙 기준선은 그대로 유지한다.

현재 상태는 로컬 `outputs/cloud-resume/priority-collection-status.json`, `physics128256-report.json`, `priority-intervention.json`을 확인한다. 이전 `collection-status.json`과 초기 실행 순서는 과거 기록이다. 512px 재시작은 현재 계획에 포함하지 않는다.
